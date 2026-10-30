#ifndef BUCK_OPTIMAL_H
#define BUCK_OPTIMAL_H
#include <math.h>
#include "buck_config.h"
#include "buck_cm.h"      /* buck_cm_ton_raw: the pulse on-time uses the current loop's inversion */

/* Time-optimal single ON-OFF switching times for a LOAD STEP-UP (Delta I > 0),
 * from the second-order Pade model (TPEL Sec. IV). The two terminal equations
 * (eqs 35-36) reduce to ONE quadratic in the total transition time s = Ton+Toff,
 * i.e. a single square root; Ton/Toff are then linear in s.
 *
 * NOTE: the paper's boxed c0/c1/c2 closed form does not solve eqs (35-36) (it
 * leaves a large current-equation residual); this is the re-derived, verified
 * form (matches the Newton optimum to ~1e-16 in Tests/test_optimal.c).
 *
 * Float. HOST REFERENCE since 2026-09-24: the firmware solves in the ISR with the
 * integer closed form below (buck_opt_solve_i); this form is kept for the tests.
 * Inputs are the shifted state zi = iL - I_load_new,
 * zv = vout - Vref (both SI: A, V) and the load estimate Ihat (A). Outputs Ton,
 * Toff in seconds, clamped to [tmin, tmax]. Returns 1 if a valid solution exists. */
static inline int buck_optimal_onoff(float zi, float zv, float Ihat,
                                     float tmin, float ton_max, float toff_max,
                                     float *Ton, float *Toff)
{
    const float Vin = (float)BUCK_VIN_V, Vref = (float)(BUCK_VOUT_MV * 1e-3);
    const float L = (float)BUCK_L_H, C = (float)BUCK_C_F, r = (float)BUCK_R_OHM;
    const float theta = -r / L, m1 = (Vin - Vref) / L, m2 = Vref / L, msum = m1 + m2;

    const float q  = theta * zi * 0.5f - zv / (2.0f * L) - m2 - theta * Ihat;
    const float g  = 0.25f * (theta * theta - 1.0f / (L * C)) * zi
                   - theta * zv / (4.0f * L) + theta * msum * 0.5f;
    const float b  = theta * zi / (4.0f * C) - zv / (4.0f * L * C) + msum / (2.0f * C);
    const float b1 = zi / (2.0f * C);
    if (b == 0.0f) return 0;

    const float a0 = -(L / Vin) * (zi - g * zv / b);
    const float a1 = -(L / Vin) * (q - g * b1 / b);
    const float A = b * a1 * (1.0f - a1);
    const float B = b * a0 * (1.0f - 2.0f * a1) + b1;
    const float Cc = zv - b * a0 * a0;
    if (A == 0.0f) return 0;

    const float disc = B * B - 4.0f * A * Cc;
    if (disc < 0.0f) return 0;
    const float sq = sqrtf(disc);

    /* Pick the root giving positive Ton and Toff (physical, time-optimal). */
    for (int k = 0; k < 2; ++k) {
        const float s = ((k == 0 ? sq : -sq) - B) / (2.0f * A);
        float ton = a0 + a1 * s;
        float toff = s - ton;
        if (ton > 0.0f && toff > 0.0f) {
            if (ton  < tmin) ton  = tmin;   else if (ton  > ton_max)  ton  = ton_max;
            if (toff < tmin) toff = tmin;   else if (toff > toff_max) toff = toff_max;
            *Ton = ton; *Toff = toff;
            return 1;
        }
    }
    return 0;
}

/* State-anchored tilted-ellipse score S(z_v, d_v) with runtime scales/tilt, both
 * inputs in ADC counts (z_v = v_out - V_ref, d_v = one-period change of v_out).
 * S = (z_v/V_a)^2 + (d_v/dV_a)^2 + 2*rho*(z_v/V_a)*(d_v/dV_a); trigger when S>1.
 * Float reference version (host tests + the debug-panel score in thread mode).
 * Scales are floored to a small positive value so a stray 0 written in Live
 * Expressions can't divide. */
static inline float buck_sa_score_f(float zv_counts, float dv_counts,
                                    float va, float dva, float rho)
{
    if (va  < 1e-3f) va  = 1e-3f;
    if (dva < 1e-3f) dva = 1e-3f;
    const float nv = zv_counts / va, nd = dv_counts / dva;
    return nv * nv + nd * nd + 2.0f * rho * nv * nd;
}

/* ---- Cycle-by-cycle INTEGER trigger for the ISR ------------------------------
 * The ISR must decide S > 1 every switching period without float or divide.
 * Rewrite S by completing the square:
 *     S = (n_v + rho n_d)^2 + (1 - rho^2) n_d^2 = u^2 + w^2,
 *     u = n_v + rho n_d,  w = sqrt(1 - rho^2) n_d,
 * with n_v = z_v / V_a and n_d = d_v / dV_a in Q14. The thread converts the
 * float tunables (V_a, dV_a, rho) once (when they change) into four integer
 * coefficients stored in g_buck.sa; the ISR does 4 SMULL, a few shifts and one
 * 64-bit compare (~18 clocks). The four words are not written atomically:
 * during the single ISR cycle in which a Live Expressions edit lands, the score
 * may mix old and new scales (worst case one spurious trigger while tuning).
 * n = SMULLH(z << 16, inv_q30) (|z| <= 4095 counts; a scale >= 0.5 count before
 * the reciprocal saturates; a saturated reciprocal only makes that axis as
 * sensitive as representable). u^2 + w^2 in Q28 is compared against 2^28. */
typedef struct {
    int32_t inv_va_q30;   /* 2^30 / V_a   (counts^-1) */
    int32_t inv_dva_q30;  /* 2^30 / dV_a  (counts^-1 per period) */
    int32_t rho_q30;      /* rho * 2^30, |rho| < 1   */
    int32_t sq_q30;       /* sqrt(1 - rho^2) * 2^30  */
} buck_sa_coef_t;

static inline void buck_sa_coef_from_float(float va, float dva, float rho, buck_sa_coef_t *c)
{
    if (rho >  0.999f) rho =  0.999f; else if (rho < -0.999f) rho = -0.999f;
    float iv = 1073741824.0f / ((va < 1e-3f) ? 1e-3f : va);
    if (iv > 2147483520.0f) iv = 2147483520.0f;   /* largest float below INT32_MAX */
    float id = 1073741824.0f / ((dva < 1e-3f) ? 1e-3f : dva);
    if (id > 2147483520.0f) id = 2147483520.0f;
    c->inv_va_q30  = (int32_t)(iv + 0.5f);
    c->inv_dva_q30 = (int32_t)(id + 0.5f);
    c->rho_q30     = (int32_t)(rho * 1073741824.0f);
    c->sq_q30      = (int32_t)(sqrtf(1.0f - rho * rho) * 1073741824.0f);
}

/* Returns nonzero when S(z_v, d_v) > 1. Integer only; ISR-safe. */
static inline __attribute__((always_inline)) int
buck_sa_trigger_i(int32_t zv_counts, int32_t dv_counts, const volatile buck_sa_coef_t *c)
{
    const int32_t zv16 = (int32_t)((uint32_t)zv_counts << 16);              /* |z_v| <= 4095 fits */
    const int32_t dv16 = (int32_t)((uint32_t)dv_counts << 16);              /* |d_v| <= 4095 fits */
    const int32_t nv = (int32_t)(((int64_t)zv16 * c->inv_va_q30)  >> 32);   /* Q14 */
    const int32_t nd = (int32_t)(((int64_t)dv16 * c->inv_dva_q30) >> 32);   /* Q14 */
    const int32_t u  = nv + (int32_t)((uint32_t)(int32_t)(((int64_t)nd * c->rho_q30) >> 32) << 2);
    const int32_t w  = (int32_t)((uint32_t)(int32_t)(((int64_t)nd * c->sq_q30) >> 32) << 2);
    const int64_t acc = (int64_t)u * u + (int64_t)w * w;                    /* Q28 */
    return acc > ((int64_t)1 << 28);
}

/* ---- Cycle-resident time-optimal ON-OFF solve (integer, two sqrt, ~130 clocks) --
 * Runs in the ISR on the cycle after the trigger, so the stretched period starts
 * three switching periods after the trigger sample.
 *
 * Charge balance for a load step-up with the series drop r i on BOTH ramps and
 * the PERIODIC steady state as the terminal state. A PWM converter at load
 * I_eq does not sit at i = I_eq at a period start: it sits at the valley
 * I_eq - h, h = ripple/2 = (V_in - V_ref)(V_ref/V_in) T/(2L) (1.74 A here), and
 * the period's own ripple averages to I_eq. The pulse therefore ends the
 * stretched period at (I_eq - h, V_ref) so that the next period is an ordinary
 * one: no ripple/2 T of extra charge in the first period, no valley transient,
 * and the first sample after the pulse sits at the same point of the ripple as
 * every steady-state sample.
 * With a = i_0 - I_eq (< 0: the inductor lags the equilibrium current), dV =
 * V_ref - v_C (> 0: droop), the ON ramp m1 = (V_in - V_ref)/L and the OFF ramp
 * over the fall from I_eq + x down to I_eq - h
 *     m2' = (V_ref + r (I_eq + (x - h)/2)) / L
 * (the trapezoidal average of the drop over the fall), the single ON-OFF pulse
 * that lands on (z_v, z_i) = (0, -h) peaks at I_eq + x with
 *     x^2 = (m2' / (m1 + m2')) [ 2 m1 C dV + a^2 + (m1/m2') h^2 ]
 *         ~= ((V_ref + r i_2) / V_in) [ 2 C (V_in - V_ref)/L dV + a^2 + (V_in - V_ref) h^2 / (V_ref + r i_2) ],
 *     T_on* = (x - a) / m1,   T_off* = (x + h) L / (V_ref + r i_2),   i_2 = I_eq + (x - h)/2.
 * (Charge above I_eq: ON triangle (x^2 - a^2)/(2 m1) + OFF (x^2 - h^2)/(2 m2')
 * = C dV; m1 + m2' ~= V_in/L to 2 %; the h^2 term, ~10 % of the bracket, is
 * taken at the ideal fall voltage so it is a constant.) The ideal-fall x_0
 * (r = 0, h = 0) gives i_2, then one more square root gives x: two table
 * roots. T_on* enters through the trapezoidal inversion buck_cm_ton_raw (one
 * UDIV), T_off* through one more UDIV.
 *
 * Adaptation (TPEL Sec. V): the applied times are t_a = diag(gamma_on, gamma_off)
 * t*, the gains learned by the thread from the measured terminal state
 * (buck_learn_gamma). gamma_on scales the physical on-time (the rise above i_0),
 * gamma_off the off-time (the fall from the peak).
 *
 * Fixed point: currents Q8 A inside the solve (a^2 <= (32 A)^2 fits Q16 A^2),
 * dV clamped to 2047 counts so kv*dV < 2^31, the (V/V_in) factors applied as Q32
 * high-word multiplies (UMULL), and the roots taken by buck_isqrt32 (Q16 A^2 in,
 * Q8 A out). Coefficients that depend on V_ref are converted once by the thread
 * (buck_so_coef_from_float) into g_buck.so, which also carries the learned
 * gains and the equilibrium correction. */
typedef struct {
    uint32_t kv_q16;      /* 2 C (V_in - V_ref)/L * V_per_count: Q16 A^2 per ADC count */
    uint32_t k2_q32;      /* V_ref / V_in, Q32 (ideal-fall root x_0) */
    int32_t  vref_q16;    /* V_ref, Q16 V */
    int32_t  roff_q12;    /* series resistance of the OFF ramp, Q12 ohm (g_buck_opt_r_off_mohm) */
    int32_t  h_q8;        /* half the steady-state ripple at V_ref, Q8 A: the pulse ends at I_eq - h (the steady valley) */
    int32_t  is_q20;      /* inductor-current rise from the valley to the ADC sample instant, Q20 A (m1 (SAMPLE - DT) tick) */
    int32_t  esr_cpa_q12; /* output-capacitor ESR in ADC counts per ampere, Q12 (g_buck_opt_esr_mohm; 0 = no correction): the sample reads v_C + ESR (i_s - I_load) */
    uint32_t h2_q16;      /* (V_in - V_ref) h^2 / V_ref, Q16 A^2: the charge of the fall below I_eq (ideal fall voltage; the r-drop on this ~10 % term is dropped: no divide) */
    int32_t  ndr_q8;      /* r/V_in * PER * 256: extra duty per Q8 A of equilibrium change, Q8 (current-loop reseed) */
    int32_t  dq_q16;      /* 2 T_sw / (C V_per_count): droop (counts) added per ampere of load-inductor gap over the two periods before the pulse, Q16 */
    int32_t  gon_q16;     /* gamma_on,  Q16 (65536 = 1.0): learned, published by the thread */
    int32_t  goff_q16;    /* gamma_off, Q16 */
    int32_t  ieq_q20;     /* learned equilibrium correction b, Q20 A: the solve targets I_hat + b (buck_learn_ieq) */
} buck_so_coef_t;
#define BUCK_OPT_INV_VIN_Q30  ((uint32_t)(1073741824.0 / BUCK_VIN_V + 0.5))              /* 1/V_in, Q30 */
#define BUCK_OPT_CPV_Q32      ((int32_t)(4294967296.0 / (BUCK_UV_PER_COUNT * 1e-6 * 268435456.0) + 0.5))   /* ADC counts per Q28 V, Q32: counts = SMULLH(v_q28, this) */
#define BUCK_OPT_FALL_Q28     ((uint32_t)(BUCK_TICK_S / BUCK_L_H * 268435456.0 + 0.5))     /* tick/L, Q28 A/(V tick) */
#define BUCK_OPT_LT64         ((uint32_t)BUCK_CM_L_OVER_TICK * 64U)                        /* L/tick * 64: Toff = x_q8 * this / (v2_q16 >> 2) */
_Static_assert((uint64_t)BUCK_OPT_LT64 * 40U * 256U < 2147483647ULL, "solve: (x + h) L/tick numerator must fit int32");

static inline void buck_so_coef_from_float(float vref_counts, float r_off_mohm, float esr_mohm, buck_so_coef_t *c)
{
    if (r_off_mohm < 0.0f) r_off_mohm = 0.0f; else if (r_off_mohm > 1000.0f) r_off_mohm = 1000.0f;
    if (esr_mohm < 0.0f) esr_mohm = 0.0f; else if (esr_mohm > 200.0f) esr_mohm = 200.0f;
    c->roff_q12 = (int32_t)(r_off_mohm * 1e-3f * 4096.0f + 0.5f);
    const float vin = (float)BUCK_VIN_V, L = (float)BUCK_L_H, Cf = (float)BUCK_C_F;
    const float vpc = (float)(BUCK_UV_PER_COUNT * 1e-6);
    float vref = vref_counts * vpc;
    if (vref < 0.05f) vref = 0.05f;                      /* stray 0 in Live Expressions */
    if (vref > vin - 0.05f) vref = vin - 0.05f;
    const float tsw = (float)BUCK_TSW_S;
    c->kv_q16   = (uint32_t)(2.0f * Cf * (vin - vref) / L * vpc * 65536.0f + 0.5f);
    c->k2_q32   = (uint32_t)(vref / vin * 4294967296.0f);
    c->vref_q16 = (int32_t)(vref * 65536.0f + 0.5f);
    const float h = 0.5f * (vin - vref) * (vref / vin) * tsw / L;          /* ripple/2 at V_ref, A */
    c->h_q8     = (int32_t)(h * 256.0f + 0.5f);
    c->is_q20   = (int32_t)((vin - vref) / L * (float)(BUCK_SAMPLE_COMPARE - BUCK_DT_FINE_TICKS) * (float)BUCK_TICK_S * 1048576.0f + 0.5f);
    c->esr_cpa_q12 = (int32_t)(esr_mohm * 1e-3f / vpc * 4096.0f + 0.5f);
    c->h2_q16   = (uint32_t)((vin - vref) * h * h / vref * 65536.0f + 0.5f);
    c->ndr_q8   = (int32_t)((float)BUCK_R_OHM / vin * (float)BUCK_HRTIM_PERIOD * 256.0f + 0.5f);
    c->dq_q16   = (int32_t)(2.0f * tsw / (Cf * vpc) * 65536.0f + 0.5f);
    c->gon_q16 = c->goff_q16 = 65536;
    c->ieq_q20 = 0;
}

/* 32-bit integer square root by a 192-entry normalised table with linear
 * interpolation: sqrt(v) = sqrt(v << 2e) >> e with v << 2e in [2^30, 2^32); the
 * table holds sqrt(i * 2^24) and its slope for i = 64..255 (low/high 16 bits of
 * one word). Interpolation error < 1e-5 relative (sqrt is nearly linear over a
 * 1/64 interval). ~18 clocks, no divide, no FPU. Table filled at start-up. */
#define BUCK_SQRT_LUT_N 192U
static uint32_t buck_sqrt_lut[BUCK_SQRT_LUT_N];

static inline void buck_sqrt_lut_init(void)
{
    for (uint32_t i = 0U; i < BUCK_SQRT_LUT_N; ++i) {
        const uint32_t s0 = (uint32_t)(sqrtf((float)(i + 64U) * 16777216.0f) + 0.5f); /* <= 65408 */
        const uint32_t s1 = (uint32_t)(sqrtf((float)(i + 65U) * 16777216.0f) + 0.5f); /* <= 65536 */
        buck_sqrt_lut[i] = s0 | ((s1 - s0) << 16);
    }
}

static inline __attribute__((always_inline)) uint32_t buck_isqrt32(uint32_t v)
{
    if (v == 0U) return 0U;                                        /* CLZ(0) is undefined; 1..3 -> 1 via the table */
    const uint32_t lz = (uint32_t)__builtin_clz(v) & ~1U;         /* even shift: v << lz in [2^30, 2^32) */
    const uint32_t m  = v << lz;
    const uint32_t e  = buck_sqrt_lut[(m >> 24) - 64U];
    const uint32_t f  = (m >> 8) & 0xFFFFU;                          /* position inside the interval, Q16 */
    const uint32_t r  = (e & 0xFFFFU) + ((f * (e >> 16)) >> 16);      /* sqrt(m) */
    return r >> (lz >> 1);
}

typedef struct {
    uint32_t ton, toff;    /* applied CMP1 / off-time, HRTIM ticks; ton == 0: no pulse (peak limit already reached) */
    uint32_t sat;          /* bit0: peak-clamped (gamma_on has no effect), bit1: Ton at TON_MAX */
    int32_t  ipk_q16;      /* applied peak, Q16 A (panel) */
    int32_t  term_q20;     /* inductor current at the end of the applied pulse, Q20 A: the valley of the next period (~ I_eq - h) */
} buck_so_out_t;
#define BUCK_SO_SAT_PEAK 1U
#define BUCK_SO_SAT_TON  2U

/* Post-pulse reseed (precomputed in the write cycle, applied in the PULSE
 * cycle). The inductor current is measured, so the voltage loop's reference is
 * simply the equilibrium current I_eq (clamped to its range), and the current
 * loop's integrator is the duty that holds I_eq: the on-time that produced the
 * measured current at the trigger plus r/V_in * PER per ampere of change (the
 * fitted r IS the DC duty-to-current relation). */
static inline __attribute__((always_inline)) void
buck_opt_reseed_i(const volatile buck_so_coef_t *c, uint32_t duty_trig,
                  int32_t iavg_trig_q20, int32_t ieq_q20, int32_t *new_iref_q16, uint32_t *new_duty)
{
    int32_t iref = ieq_q20 >> 4;                                             /* Q20 -> Q16 A */
    if (iref < BUCK_ACM_IMIN_Q16) iref = BUCK_ACM_IMIN_Q16;
    else if (iref > BUCK_ACM_IMAX_Q16) iref = BUCK_ACM_IMAX_Q16;
    *new_iref_q16 = iref;
    const int32_t di_q8 = (ieq_q20 - iavg_trig_q20) >> 12;                   /* Q8 A */
    int32_t nd = (int32_t)duty_trig + ((di_q8 * c->ndr_q8) >> 16);
    if (nd < (int32_t)BUCK_DUTY_MIN) nd = (int32_t)BUCK_DUTY_MIN;
    else if (nd > (int32_t)BUCK_DUTY_MAX) nd = (int32_t)BUCK_DUTY_MAX;
    *new_duty = (uint32_t)nd;
}

/* i0_q20: inductor current at the START of the pulse (Q20 A: the measured
 * valley extrapolated to that period); iload_q20: equilibrium current I_eq (the
 * measured new load plus the learned correction, Q20 A); vout_q28: V_out;
 * error_counts: V_ref - v_C at the pulse start (droop, counts; the caller has
 * removed the ESR term of the sample). Applies the gains in c (gamma_on on the
 * rise above i0, gamma_off on the off-time), the peak and time clamps and the
 * stretched-period floor, and reports the terminal current of the pulse as
 * applied. */
static inline __attribute__((always_inline)) void
buck_opt_solve_i(const volatile buck_so_coef_t *c, int32_t i0_q20, int32_t iload_q20,
                 int32_t vout_q28, int32_t error_counts, buck_so_out_t *o)
{
    uint32_t dv = (error_counts > 0) ? (uint32_t)error_counts : 0U;
    if (dv > 2047U) dv = 2047U;                               /* kv * dv < 2^31 */
    const int32_t a_q8    = (i0_q20 - iload_q20) >> 12;       /* z_i, Q8 A */
    const int32_t inew_q8 = iload_q20 >> 12;
    const int32_t i0_q8   = i0_q20 >> 12;
    const uint32_t s  = c->kv_q16 * dv + (uint32_t)(a_q8 * a_q8);          /* 2 m1 C dV + a^2, Q16 A^2 (times L) */
    o->ton = 0U; o->toff = 0U; o->sat = 0U; o->ipk_q16 = 0; o->term_q20 = iload_q20;
    const int32_t xmax = BUCK_OPT_IPK_MAX_Q8 - inew_q8;
    if (xmax <= 0) return;                                    /* load already at the peak limit */
    /* Ideal-fall excursion x_0, then the fall's series drop: the OFF ramp from
     * I_eq + x down to the steady valley I_eq - h sees V_ref + r (I_eq + (x - h)/2)
     * on average, and the fall below I_eq adds (V_in - V_ref) h^2 / (V_ref + r i_2)
     * to the charge the pulse must deliver. */
    const int32_t h_q8 = c->h_q8;
    int32_t x0 = (int32_t)buck_isqrt32((uint32_t)(((uint64_t)s * c->k2_q32) >> 32));   /* Q8 A */
    if (x0 > xmax) x0 = xmax;
    const int32_t i2_q8  = inew_q8 + ((x0 - h_q8) >> 1);
    const int32_t v2_q16 = c->vref_q16 + ((i2_q8 * c->roff_q12) >> 4);                /* V_ref + r i_2, Q16 V */
    const uint32_t s_h   = s + c->h2_q16;                                             /* + (V_in - V_ref) h^2 / V_ref, Q16 A^2 */
    const uint32_t k2p   = (uint32_t)(((uint64_t)(uint32_t)v2_q16 * BUCK_OPT_INV_VIN_Q30) >> 14);   /* (V_ref + r i_2)/V_in, Q32 */
    const int32_t x_q8   = (int32_t)buck_isqrt32((uint32_t)(((uint64_t)s_h * k2p) >> 32));  /* Q8 A: nominal excursion above I_eq */
    int32_t xa = x_q8;
    uint32_t sat = 0U;
    if (xa > xmax) { xa = xmax; sat = BUCK_SO_SAT_PEAK; }
    const int32_t vf_q16 = c->vref_q16 + (((inew_q8 + ((xa - h_q8) >> 1)) * c->roff_q12) >> 4);   /* fall voltage at the applied excursion */
    /* Applied rise above i0: gamma_on (I_pk* - i0), re-capped at the peak limit. */
    int32_t rise_q8 = (int32_t)(((int64_t)(inew_q8 + xa - i0_q8) * c->gon_q16) >> 16);
    if (rise_q8 > BUCK_OPT_IPK_MAX_Q8 - i0_q8) { rise_q8 = BUCK_OPT_IPK_MAX_Q8 - i0_q8; sat |= BUCK_SO_SAT_PEAK; }
    const int32_t ipk_q16 = (int32_t)((uint32_t)(i0_q8 + rise_q8) << 8);
    uint32_t ton = buck_cm_ton_raw(ipk_q16, i0_q20, vout_q28) + BUCK_DT_FINE_TICKS;   /* CMP1 = physical + dead time; one UDIV */
    if (ton < BUCK_OPT_TMIN_TICKS) ton = BUCK_OPT_TMIN_TICKS;
    else if (ton >= BUCK_OPT_TON_MAX_TICKS) { ton = BUCK_OPT_TON_MAX_TICKS; sat |= BUCK_SO_SAT_TON; }
    /* Applied off-time: gamma_off * (x + h) L / (V_ref + r i_2) (the nominal fall from
     * the peak down to the steady valley I_eq - h); one UDIV. */
    const uint32_t toff_nom = ((uint32_t)(xa + h_q8) * BUCK_OPT_LT64) / (uint32_t)(vf_q16 >> 2);
    uint32_t toff = (uint32_t)(((uint64_t)toff_nom * (uint32_t)c->goff_q16) >> 16);
    if (toff < BUCK_OPT_TMIN_TICKS) toff = BUCK_OPT_TMIN_TICKS;
    else if (toff > BUCK_OPT_TOFF_MAX_TICKS) toff = BUCK_OPT_TOFF_MAX_TICKS;
    /* The stretched period must not be shorter than the ISR's fixed restore point
     * inside it (BUCK_OPT_PER_MIN_TICKS): extend Toff if needed. */
    if (ton + toff < BUCK_OPT_PER_MIN_TICKS) toff = BUCK_OPT_PER_MIN_TICKS - ton;
    o->ton = ton; o->toff = toff; o->sat = sat; o->ipk_q16 = ipk_q16;
    /* Terminal current of the pulse AS APPLIED: the peak minus what the final
     * off-time removes at the fall slope (V_ref + r i_2)/L, clamps and floor included. */
    const int32_t fall_q8 = (int32_t)(((uint64_t)(toff * (uint32_t)(vf_q16 >> 4)) * BUCK_OPT_FALL_Q28) >> 32);
    o->term_q20 = (int32_t)((uint32_t)(i0_q8 + rise_q8 - fall_q8) << 12);
}

/* ---- Thread-mode adaptation (the dual correction) ------------------------- */

/* Timing adaptation (manuscript eqs 43-48) on the MEASURED terminal state.
 * gon/goff: the gains that produced this pulse (updated in place). ton_a/toff_a:
 * the applied PHYSICAL on/off-times (s). zi0 = i_0 - I_eq at the pulse start
 * (A). zi_f: the measured terminal current minus the target valley I_eq - h
 * (A). zv_f: the measured terminal voltage residual, ESR term removed (V).
 * m1 = (V_in - V_ref)/L, m2 = the fall slope (V_ref + r i_2)/L of this pulse.
 * Terminal map from the pulse start,
 *     z_i,f = zi0 + m1 Ton - m2 Toff,
 *     z_v,f = zv0 + [ zi0 (Ton + Toff) + m1 Ton^2/2 + m1 Ton Toff - m2 Toff^2/2 ] / C,
 * so J = [[m1, -m2], [j21, j22]], j21 = (zi0 + m1 (Ton + Toff))/C, j22 = (zi0 +
 * m1 Ton - m2 Toff)/C, at the applied times. The Newton step
 *     [dTon; dToff] = -mu J^-1 [z_i,f; z_v,f],  J^-1 = [[j22, m2], [-j21, m1]] / det,
 * nulls both residuals to first order; gamma changes by dt / t*, t* = t_a /
 * gamma. Projection: |d gamma| <= dmax per pulse, gamma in [gmin, gmax]. A
 * clamped on-time (sat) cannot change: Toff alone takes the current residual
 * (m2 dToff = mu z_i,f; the voltage residual is unreachable). The determinant
 * is floored at m2 * det_min_a / C. Float, thread. */
static inline void buck_learn_gamma(float *gon, float *goff, float ton_a, float toff_a,
                                    float zi0, float zi_f, float zv_f, float m1, float m2,
                                    float mu, uint32_t sat, float det_min_a, float dmax, float gmin, float gmax)
{
    const float C = (float)BUCK_C_F;
    float dton, dtoff;
    if (m2 < 1e3f) m2 = 1e3f;
    if (sat & (BUCK_SO_SAT_PEAK | BUCK_SO_SAT_TON)) {
        dton  = 0.0f;
        dtoff = mu * zi_f / m2;
    } else {
        const float j21 = (zi0 + m1 * (ton_a + toff_a)) / C;        /* d z_v,f / d T_on  */
        const float j22 = (zi0 + m1 * ton_a - m2 * toff_a) / C;     /* d z_v,f / d T_off */
        float det = m1 * j22 + m2 * j21;
        const float det_min = m2 * det_min_a / C;
        if (det < det_min) det = det_min;
        dton  = -mu * ( j22 * zi_f + m2 * zv_f) / det;
        dtoff = -mu * (-j21 * zi_f + m1 * zv_f) / det;
    }
    if (ton_a > 0.0f && dton != 0.0f) {
        float d = dton * *gon / ton_a;                              /* d gamma = dt / t*,  t* = t_a / gamma */
        if (d > dmax) d = dmax; else if (d < -dmax) d = -dmax;
        float g = *gon + d;
        if (g < gmin) g = gmin; else if (g > gmax) g = gmax;
        *gon = g;
    }
    if (toff_a > 0.0f) {
        float d = dtoff * *goff / toff_a;
        if (d > dmax) d = dmax; else if (d < -dmax) d = -dmax;
        float g = *goff + d;
        if (g < gmin) g = gmin; else if (g > gmax) g = gmax;
        *goff = g;
    }
}

/* Equilibrium correction. The pulse was solved for I_eq = I_hat + b and the
 * loop then settled at I_settled: e_b = I_settled - I_eq is how far the
 * equilibrium was off (a slow load slew that continued after the pulse, a
 * sense-gain error, a biased edge measurement).
 *     b <- b + mu_b e_b,  |b| <= bmax.
 * Float, thread. */
static inline float buck_learn_ieq(float b, float e_b, float mu_b, float bmax)
{
    float n = b + mu_b * e_b;
    if (n > bmax) n = bmax; else if (n < -bmax) n = -bmax;
    return n;
}

#endif
