#ifndef BUCK_CTRL_H
#define BUCK_CTRL_H

/* Per-switching-cycle control algorithm (hardware independent, single
 * precision). The 600 kHz control loop (Src/buck.c) and the host tests
 * (Tests/test_ctrl.c) execute exactly this code.
 *
 * Cycle indexing. Cycle k is the switching period [kT, (k+1)T). The control
 * update that runs early in cycle k+1 receives
 *   v_out[k+1]  the mean of the two A0 samples at the end of cycle k (start of
 *               cycle k+1), and
 *   i_Lm[k]     the measured cycle-average inductor current of cycle k,
 * and performs, every cycle:
 *   hot  (before the CMP1 write, buck_ctrl_fast):
 *        voltage PI -> u[k] = i_ref;  current PI on i_ref - i_Lm[k] -> duty,
 *        written as the CMP1 preload (active from cycle k+2);
 *   after the write (buck_ctrl_slow):
 *        trigger score S[k] and indicator g[k];
 *   then (run_pass: buck_obs_step_ao, the step of buck_iload_obs): the load-current observer, fed v_out[k+1]
 *        and i_Lm[k] (user, 25 Oct: BUCK_OBS_A).
 * Stage 3 (buck_obs_apt, 30 Oct 2026, user decision 1): the Padé current estimator (its per-cycle
 * prediction and measurement correction, the 84 KB coefficient table, alpha_I) is removed; the current
 * PI's feedback is the MEASURED four-sample cycle average i_Lm[k] (the value the load observer uses).
 * The PI gains are unchanged (tools/pi_design.py already designs them for this feedback: the average of
 * cycle k, read early in cycle k+1, acting from cycle k+2). The fields the estimator wrote are kept
 * where g_buck has them (layout unchanged, user decision 3): buck_state_t.iv, iv_pred, avg_pred, kappa
 * are 0; buck_out_t.i_avg = i_meas, eps = 0, i_peak = 0; buck_tune_t.alpha_i is unused. */

#include <stdint.h>
#include "buck_config.h"

#ifndef BUCK_INLINE
#define BUCK_INLINE static inline __attribute__((always_inline))
#endif

/* Products that feed an addition. On the Cortex-M4, VMLA/VMLS/VNMLS take 3
 * cycles and a VMUL followed by a VADD takes 2; the VFPv4 VMLA is not fused, so
 * both give the same rounded result. The empty asm hides the product from the
 * compiler, which otherwise merges it into a VMLA. It has no effect on the host. */
#if defined(__arm__)
#define BUCK_SEPARATE(x) __asm__("" : "+t"(x))
#else
#define BUCK_SEPARATE(x) ((void)0)
#endif

BUCK_INLINE float buck_mul(float a, float b)
{
    float p = a * b;
    BUCK_SEPARATE(p);
    return p;
}

/* a * b with a as the instruction's first operand (Sn) whatever the register allocation (combine,
 * 27 Oct): with both operands NaN the result is a's quieted NaN. Used where the 25 Oct build's
 * register allocation fixed an order that a changed allocation swaps (the NaN payloads of the
 * published values would change). It is also a separate multiply (BUCK_SEPARATE). */
#if defined(__arm__)
BUCK_INLINE float buck_mul_nm(float a, float b)
{
    float r;
    __asm__("vmul.f32 %0, %1, %2" : "=t"(r) : "t"(a), "t"(b));
    return r;
}
BUCK_INLINE float buck_add_nm(float a, float b)
{
    float r;
    __asm__("vadd.f32 %0, %1, %2" : "=t"(r) : "t"(a), "t"(b));
    return r;
}
#else
BUCK_INLINE float buck_mul_nm(float a, float b) { return a * b; }
BUCK_INLINE float buck_add_nm(float a, float b) { return a + b; }
#endif

/* ---- Live tunables (the firmware re-reads them every 64 cycles) ------------ */
typedef struct {
    uint32_t vref_counts;   /* V_ref, A0 counts (the reference moves at a fixed slew) */
    float kp_v, ki_v;       /* voltage PI: A/V, A/(V s) */
    float kp_i, ki_i;       /* current PI: 1/A, 1/(A s) */
    float alpha_i;          /* unused since stage 3 (was the Padé estimator's alpha_I); kept for the layout */
    float s_th;             /* trigger threshold S_th: D4 only (g = S >= S_th) */
    float isense_zero;      /* A1 reading at 0 A, counts */
    float isense_gain;      /* A1 scale, A per count */
    float score_a;          /* trigger score S = a e_v^2 + b e_v + c: a, 1/V^2 (BUCK_SCORE_A_DEFAULT) */
    float score_b;          /* b, 1/V */
    float score_c;          /* c */
    float obs_beta;         /* load-current observer: voltage correction gain beta (BUCK_OBS_BETA_DEFAULT) */
    float obs_gamma;        /* current correction gain gamma, A/V; a pair outside the stable region is not taken up */
} buck_tune_t;

#define BUCK_TUNE_DEFAULTS {                                                   \
    .vref_counts = BUCK_VREF_COUNTS_DEFAULT,                                   \
    .kp_v = BUCK_KP_V, .ki_v = BUCK_KI_V,                                      \
    .kp_i = BUCK_KP_I, .ki_i = BUCK_KI_I,                                      \
    .alpha_i = 0.0f,                                                           \
    .s_th = BUCK_STH_DEFAULT,                                                  \
    .isense_zero = (float)BUCK_ISENSE_ZERO_COUNTS,                             \
    .isense_gain = (float)BUCK_ISENSE_A_PER_COUNT,                             \
    .score_a = BUCK_SCORE_A_DEFAULT, .score_b = BUCK_SCORE_B_DEFAULT,          \
    .score_c = BUCK_SCORE_C_DEFAULT,                                           \
    .obs_beta = BUCK_OBS_BETA_DEFAULT, .obs_gamma = BUCK_OBS_GAMMA_DEFAULT,    \
}

/* ---- Controller state (the firmware keeps it in registers while running) -- */
typedef struct {
    float iv;               /* unused since stage 3 (the Padé estimator's state): 0; kept for the layout */
    float iv_pred;          /* unused since stage 3: 0 */
    float avg_pred;         /* unused since stage 3: 0 */
    float kappa;            /* unused since stage 3: 0 */
    float integ_v;          /* voltage-PI integrator, A */
    float integ_i;          /* current-PI integrator, duty */
    float vref;             /* reference in use, V */
    uint32_t cmp_active;    /* CMP1 of the cycle in progress */
} buck_state_t;

/* ---- Results of the last update (telemetry: written, never read back) ------ */
typedef struct {
    float v;                /* v_out[k+1], V */
    float i_meas;           /* measured cycle average ī_L,m[k], A: the current PI's feedback (stage 3) */
    float i_avg;            /* = i_meas since stage 3 (was the Padé estimate î̄_L[k]); kept for the layout */
    float eps;              /* 0 since stage 3 (was the estimator's residual); kept for the layout */
    float i_load;           /* load current, A: the observer's estimate, the value A2 shows
                               (buck_iload_obs; buck_ctrl_slow() leaves it to the observer;
                               at arming the measured current, the observer's start) */
    float i_ref;            /* u[k] = voltage-PI output, A */
    float i_peak;           /* 0 since stage 3 (was the estimator's î_p); kept for the layout */
    float vref;             /* reference in use, V */
    float score;            /* S[k] */
    uint32_t g;             /* g[k] (also on D4) */
    uint32_t cmp;           /* CMP1 written, ticks (duty = (cmp - DT)/PER) */
} buck_out_t;

/* ---- PI ------------------------------------------------------------------ */
/* u[k] = Kp e[k] + I[k],  I[k] = I[k-1] + Ki T e[k]:  C(z) = Kp + Ki T z/(z-1),
 * the form tools/pi_design.py designs for. Anti-windup by integrator
 * tracking: when u saturates, I is set so that Kp e + I equals the limit. */
BUCK_INLINE float buck_pi_ki_t(float ki)
{
    return ki * (float)BUCK_TSW_S;
}

/* The same step with Ki*T already formed (buck_pi_ki_t): the firmware forms it
 * when it (re)loads the tunables instead of every cycle. */
/* The step in two parts (the firmware writes CMP1 between them): the output
 * u, with the proportional term in *p, and then the integrator update
 * buck_pi_commit(). buck_pi_step_kt() is both, the same operations on the same
 * values. */
BUCK_INLINE float buck_pi_out_kt(float integ, float kp, float ki_t, float lo, float hi, float e, float *p_out)
{
    const float p = buck_mul(kp, e);
    float u = integ + buck_mul(ki_t, e) + p;
    /* max(u, lo) then min(u, hi): for every number the same as clamping hi
     * first, but a NaN (a non-finite tunable written from the debugger) ends
     * at lo instead of passing through, so the duty stays inside its limits. */
    u = (u > lo) ? u : lo;
    u = (u < hi) ? u : hi;
    *p_out = p;
    return u;
}

/* buck_pi_out_kt with the proportional product's operand order fixed (buck_mul_nm): kp first if
 * kp_first, else e first, as the 25 Oct build compiled the current PI in the first pass and in the
 * copies of the pass (28 Oct). With kp and e both NaN the product is the first operand's NaN, and it
 * reaches the integrator (I = u - p with u clamped to a number) and g_buck.st at the next stop; a
 * changed register allocation swapped it in the pass after a tunables pass. */
BUCK_INLINE float buck_pi_out_kt_o(float integ, float kp, float ki_t, float lo, float hi, float e, float *p_out,
                                   int kp_first)
{
    const float p = kp_first ? buck_mul_nm(kp, e) : buck_mul_nm(e, kp);
    float u = integ + buck_mul(ki_t, e) + p;
    u = (u > lo) ? u : lo;                  /* as buck_pi_out_kt */
    u = (u < hi) ? u : hi;
    *p_out = p;
    return u;
}

BUCK_INLINE float buck_pi_commit(float u, float p)
{
    return u - p;
}

BUCK_INLINE float buck_pi_step_kt(float *integ, float kp, float ki_t, float lo, float hi, float e)
{
    float p;
    const float u = buck_pi_out_kt(*integ, kp, ki_t, lo, hi, e, &p);
    *integ = buck_pi_commit(u, p);
    return u;
}

BUCK_INLINE float buck_pi_step(float *integ, float kp, float ki, float lo, float hi, float e)
{
    return buck_pi_step_kt(integ, kp, buck_pi_ki_t(ki), lo, hi, e);
}

/* ---- CMP1 <-> on-time / duty ------------------------------------------------ */
BUCK_INLINE float buck_cmp_to_ton(uint32_t cmp)
{
    return (float)(cmp - BUCK_DT_TICKS) * (float)BUCK_TICK_S;
}

BUCK_INLINE float buck_cmp_to_duty(uint32_t cmp)
{
    return (float)(cmp - BUCK_DT_TICKS) * (float)(1.0 / BUCK_HRTIM_PERIOD);
}

BUCK_INLINE uint32_t buck_duty_to_cmp(float d)
{
    return (uint32_t)(buck_mul(d, (float)BUCK_HRTIM_PERIOD) + 0.5f) + BUCK_DT_TICKS;
}

#if defined(__arm__)
BUCK_INLINE uint32_t buck_usat12(int32_t x)
{
    uint32_t r;
    __asm__("usat %0, #12, %1" : "=r"(r) : "r"(x));
    return r;
}
#else
BUCK_INLINE uint32_t buck_usat12(int32_t x)
{
    return (x < 0) ? 0U : (x > 4095) ? 4095U : (uint32_t)x;
}
#endif

/* The A2 code of a current: 0..BUCK_DAC_FULL_SCALE_A -> 0..4095, truncated
 * (the floor for positive values), negative values 0. */
#define BUCK_DAC_PER_A  ((float)(4095.0 / (double)BUCK_DAC_FULL_SCALE_A))
/* with the scale given (= BUCK_DAC_PER_A; the firmware may hold it in a register) */
BUCK_INLINE uint32_t buck_iload_dac_k(float y, float per_a)
{
    return buck_usat12((int32_t)buck_mul(y, per_a));
}

BUCK_INLINE uint32_t buck_iload_dac(float y)
{
    return buck_iload_dac_k(y, BUCK_DAC_PER_A);
}

/* ---- Load current: fixed-gain observer (user, 25 Oct; BUCK_OBS_A) ------------
 * Predict, then correct, once per cycle:
 *   v^ += a (i_L - i^);  e = v - v^;  v^ += beta e;  i^ -= gamma e
 * with a = T/C, v = v_out[k+1] and i_L = ī_L,m[k] (the raw measured average,
 * buck_step_t.im). Three multiplies and five additions/subtractions (and the A2
 * code's multiply); no NaN guard on the states (user). */
typedef struct { float vh, ih; } buck_obs_t;           /* v^ (V) and the estimate i^ (A) */
typedef struct { float beta, gamma; } buck_obs_k_t;    /* the gains in use (buck_obs_k) */

/* with a given (= BUCK_OBS_A; the firmware may hold it in a register) */
BUCK_INLINE float buck_obs_step_a(buck_obs_t *x, float v, float il, buck_obs_k_t k, float a)
{
    const float vp = x->vh + buck_mul(a, il - x->ih);               /* predict */
    const float e = v - vp;                                         /* correct */
    x->vh = vp + buck_mul(k.beta, e);
    x->ih = x->ih - buck_mul(k.gamma, e);
    return x->ih;
}

/* buck_obs_step_a with the prediction's addition pinned with the product first (buck_add_nm) if
 * pin_prod_first, else exactly buck_obs_step_a (the order left to the compiler). run_pass() pins it
 * in the pass after a tunables pass only (user, 28 Oct, second set): the 25 Oct build compiled the
 * addition with the product first in every pass of its copies, and so did the 28 Oct build before
 * this pin (bin 2f144a53), except in that pass, where its register allocation put v^ first. With v^
 * and the product both NaN the sum is the first operand's NaN, so out.i_load would carry the other
 * NaN's payload (found after the change by the operand-order check that does not depend on the
 * values). The first pass (v^ first in both builds) and the other passes keep their instructions.
 * The same operations on the same values. */
BUCK_INLINE float buck_obs_step_ao(buck_obs_t *x, float v, float il, buck_obs_k_t k, float a, int pin_prod_first)
{
    const float m = buck_mul(a, il - x->ih);
    const float vp = pin_prod_first ? buck_add_nm(m, x->vh) : x->vh + m;   /* predict */
    const float e = v - vp;                                                 /* correct */
    x->vh = vp + buck_mul(k.beta, e);
    x->ih = x->ih - buck_mul(k.gamma, e);
    return x->ih;
}

BUCK_INLINE float buck_obs_step(buck_obs_t *x, float v, float il, buck_obs_k_t k)
{
    return buck_obs_step_a(x, v, il, k, BUCK_OBS_A);
}

/* One cycle as the firmware runs it (the host tests call this; run_pass() runs the same step through
 * buck_obs_step_ao, with the post pass's prediction order pinned, and buck_iload_dac_k, with the A2
 * write as IO_WR): steps *x with this cycle's v and i_L, writes the A2 code to *a2 and returns the estimate. */
BUCK_INLINE float buck_iload_obs(buck_obs_t *x, float v, float il, buck_obs_k_t k, volatile uint32_t *a2)
{
    const float y = buck_obs_step(x, v, il, k);
    *a2 = buck_iload_dac(y);
    return y;
}

/* The start, from the arming pass's results (buck_ctrl_arm; run_loop() seeds
 * it right after arming): i^ = its measured current i_meas, v^ = its v_out. */
BUCK_INLINE buck_obs_t buck_obs_seed(const buck_out_t *arm)
{
    const buck_obs_t x = { arm->v, arm->i_meas };
    return x;
}

/* Float bits through a union (no library call; a VMOV on the M4). */
BUCK_INLINE uint32_t buck_f2u(float f) { const union { float f; uint32_t u; } c = { .f = f }; return c.u; }
BUCK_INLINE float buck_u2f(uint32_t u) { const union { float f; uint32_t u; } c = { .u = u }; return c.f; }

/* The rounding error of t = h gamma, h = a/2 (t the rounded product): h gamma - t
 * exactly, by Dekker's product without FMA. h and gamma are split into a high
 * part (the low 12 of the 23 stored mantissa bits cleared: at most 12
 * significant bits) and the rest (at most 12 bits; h's has 8), so the four
 * partial products are exact, and so is every addition. h's parts are constants
 * (GCC folds them). Exact for gamma >= 2^-96 (the host tests check every
 * mantissa at the extreme exponents); below that a partial product can be
 * subnormal, and buck_obs_gains_ok does not need e. */
/* x with its low 12 bits cleared (one BFC on the target; GCC forms the AND with two BICs). */
#if defined(__arm__)
BUCK_INLINE uint32_t buck_clear12(uint32_t x)
{
    __asm__("bfc %0, #0, #12" : "+r"(x));
    return x;
}
#else
BUCK_INLINE uint32_t buck_clear12(uint32_t x) { return x & 0xFFFFF000U; }
#endif

BUCK_INLINE float buck_obs_prod_err(float gamma, float t)
{
    const float h = 0.5f * BUCK_OBS_A;
    float h2 = h - buck_u2f(buck_f2u(h) & 0xFFFFF000U);
    BUCK_SEPARATE(h2);                      /* h1 formed from h and h2 (exact): one constant less (tscout) */
    const float h1 = h - h2;
    const float g1 = buck_u2f(buck_clear12(buck_f2u(gamma))), g2 = gamma - g1;
    return (((buck_mul(h1, g1) - t) + buck_mul(h1, g2)) + buck_mul(h2, g1)) + buck_mul(h2, g2);
}

/* 1 when (beta, gamma) is inside the observer's stable region (BUCK_OBS_A):
 * gamma > 0 and 0 < beta < 2 - a gamma / 2, i.e. beta + h gamma < 2 with h =
 * a/2, decided exactly for every pair of floats (NaN, infinities, zeros and
 * subnormals included; a point on the boundary is outside). beta, gamma > 0 is
 * tested on the bits (sign clear, not +0; a NaN or +inf that passes fails
 * below). With t = h gamma rounded, e = h gamma - t exactly
 * (buck_obs_prod_err), and (x, y) = (beta, t) when beta >= 1, else (t, beta):
 *   beta + h gamma < 2  <=>  e < (2 - x) - y   (both operations rounded).
 * Proof (round to nearest even; |e| <= ulp(t)/2 <= 2^-24 t):
 *   x in [1, 4]: 2 - x is exact (Sterbenz). If (2 - x) - y is exact too, both
 *     sides compare the same numbers. If not, one of three holds:
 *     - 0 < y < (2 - x)/2: both true. 2 - x is a multiple of 2^-23, so both
 *       differences exceed (2 - x)/2 >= 2^-24, and |e| <= 2^-24 (t < 2);
 *       equality would need t = 2 - 2^-23 rounded from a tie, which goes to 2;
 *     - y > 2 (2 - x) > 0: both false. Both differences are below -y/2, and
 *       |e| < y/2 (y = t: |e| <= 2^-24 t; y = beta > 2 (2 - t) >= 2^-22:
 *       |e| <= 2^-24);
 *     - 2 - x < 0: both false. The rounded difference is at most -t (x = beta)
 *       or 2 - t <= -2^-22 (x = t), and |e| is smaller (2^-24 t; 2^-23, or
 *       2^-22 at t = 4 where 2 - t = -2).
 *   x < 1 (then beta < 1 and t < 1): both true (h gamma < 1; the rounded
 *     difference is at least 2^-24, |e| <= 2^-25).
 *   x > 4: both false, the rounded difference being below -2^-24 t. beta =
 *     +inf or a NaN: false; gamma = +inf or a NaN makes e a NaN: false.
 *   gamma < 2^-96 (e may be inexact, but |e| < 2^-100 and e >= -t, the partial
 *     products being >= 0): beta < 2 is inside by at least 2^-23 and the rounded
 *     difference is at least 2^-24; at beta = 2 it is -t <= e: outside, as it
 *     is.
 * The host tests compare it with an exact reference over random, special and
 * boundary pairs. 10 FP operations for t and e, 2 for the difference, three
 * compares; no division. */
BUCK_INLINE uint32_t buck_obs_gains_ok(float beta, float gamma)
{
    const uint32_t ub = buck_f2u(beta), ug = buck_f2u(gamma);
    const float t = buck_mul(0.5f * BUCK_OBS_A, gamma);
    const float e = buck_obs_prod_err(gamma, t);
    const int one = ub >= 0x3F800000U;      /* beta >= 1 (as bits: also a NaN or a negative beta) */
    /* (2 - x) - y for both orders, then the one of (x, y): the same two roundings of the same
     * operands as selecting x and y first (two FP subtractions more, no conditional moves of
     * x and y; math scout, 27 Oct). */
    const float d_beta = (2.0f - beta) - t;     /* (x, y) = (beta, t) */
    const float d_t = (2.0f - t) - beta;        /* (x, y) = (t, beta) */
    const float d = one ? d_beta : d_t;
    return ((int32_t)ub > 0 && (int32_t)ug > 0 && e < d) ? 1U : 0U;
}

/* The gains when the tunables are (re)loaded (the tunables pass and the loop
 * entry, not every cycle): the tunables' pair if it is inside the stable
 * region, otherwise prev, the gains in use; at the loop entry, which has none,
 * prev = BUCK_OBS_K_DEFAULTS. */
#define BUCK_OBS_K_DEFAULTS  ((buck_obs_k_t){ BUCK_OBS_BETA_DEFAULT, BUCK_OBS_GAMMA_DEFAULT })

BUCK_INLINE buck_obs_k_t buck_obs_k(const buck_tune_t *t, buck_obs_k_t prev)
{
    const buck_obs_k_t k = { t->obs_beta, t->obs_gamma };
    return buck_obs_gains_ok(k.beta, k.gamma) ? k : prev;
}

/* Trigger score (user, 25 Oct; BUCK_SCORE_A_DEFAULT): S = a e_v^2 + b e_v + c,
 * evaluated as (a e_v + b) e_v + c: two multiplies and two adds, no division.
 * Within 4 roundings (2.4e-7) of |a| e_v^2 + |b e_v| + |c| of the exact value.
 * For a finite e_v (always, in the firmware) and no intermediate overflow or
 * underflow, NaN and infinite coefficients give the same NaN and infinities
 * as the expanded form a (e_v e_v) + b e_v + c. Otherwise this form can give
 * +-inf where the expanded one gives NaN (inf - inf, or 0 inf): e.g. +inf for
 * a = 3e38, b = -3e38, e_v = 1.5, for a = 1, b = 0 with an infinite e_v, and
 * for an infinite a with e_v e_v underflowing to 0. At the defaults a = 1,
 * b = c = 0 it is e_v^2 bit for bit: 1 e_v = e_v, e_v + 0 = e_v (or +0 for
 * e_v = -0, with the same square +0), e_v^2 + 0 = e_v^2. */
BUCK_INLINE float buck_trigger_score(float ev, float a, float b, float c)
{
    return buck_mul(buck_mul(a, ev) + b, ev) + c;
}

/* The coefficients, copied when the tunables are (re)loaded (no division:
 * the arming path and the tunables pass have none for the score). */
typedef struct { float a, b, c; } buck_score_k_t;

BUCK_INLINE buck_score_k_t buck_score_k(const buck_tune_t *t)
{
    const buck_score_k_t k = { t->score_a, t->score_b, t->score_c };
    return k;
}

BUCK_INLINE float buck_trigger_score_k(float ev, buck_score_k_t k)
{
    return buck_trigger_score(ev, k.a, k.b, k.c);
}

/* buck_trigger_score_k with the two additions' operand order fixed (buck_add_nm), as the 25 Oct
 * build compiled them: (a e_v) + b and (..) e_v + c in the ordinary passes, b + (a e_v) and
 * c + (..) e_v in the first pass after arming (combine, 27 Oct: with NaN score coefficients the
 * published score's NaN payload depends on it). The same operations on the same values. */
BUCK_INLINE float buck_trigger_score_ko(float ev, buck_score_k_t k, int first_order)
{
    const float m1 = buck_mul(k.a, ev);
    const float s1 = first_order ? buck_add_nm(k.b, m1) : buck_add_nm(m1, k.b);
    const float m2 = buck_mul(s1, ev);
    return first_order ? buck_add_nm(k.c, m2) : buck_add_nm(m2, k.c);
}

/* ---- Controller ------------------------------------------------------------ */
/* ADC1's result is the sum of the cycle's BUCK_VSAMPLE_N A0 samples. */
BUCK_INLINE float buck_adc_to_v(uint32_t adc_v)
{
    return (float)adc_v * (float)BUCK_V_PER_VSUM;
}

/* (float)x * 0.25f. On the M4 one fixed-point VCVT with 2 fraction bits, x / 4 rounded once,
 * instead of a VCVT and a VMUL by 0.25 (bus scout): the same value bit for bit for every x
 * (both are x / 4 rounded once to single; checked for all 2^32 values on the host). */
#if defined(__arm__)
BUCK_INLINE float buck_u32_quarter(uint32_t x)
{
    float f;
    __asm__("vcvt.f32.u32 %0, %0, #2" : "=t"(f) : "0"(x));
    return f;
}
#else
BUCK_INLINE float buck_u32_quarter(uint32_t x) { return (float)x * 0.25f; }
#endif

/* adc_i4 is the SUM of the four A1 samples of the cycle. gain_first: the operand order of the
 * multiply (buck_mul_nm), as the 25 Oct build compiled it per call site (combine, 27 Oct: a
 * changed register allocation swaps it, and with NaN isense_zero and isense_gain the published
 * current then carries the other NaN's payload): the ordinary passes (zero-corrected code,
 * gain), the arming and the first pass (gain, zero-corrected code). */
BUCK_INLINE float buck_adc_to_i_o(const buck_tune_t *t, uint32_t adc_i4, int gain_first)
{
    const float x = buck_u32_quarter(adc_i4) - t->isense_zero;
    return gain_first ? buck_mul_nm(t->isense_gain, x) : buck_mul_nm(x, t->isense_gain);
}

BUCK_INLINE float buck_adc_to_i(const buck_tune_t *t, uint32_t adc_i4)
{
    return buck_adc_to_i_o(t, adc_i4, 0);
}

/* Values handed from buck_ctrl_fast() to buck_ctrl_slow() (registers). */
typedef struct {
    float v;        /* v_out[k+1], V */
    float im;       /* ī_L,m[k], A: the current PI's feedback (stage 3) */
    float iref;     /* u[k], A */
    uint32_t cmp;   /* CMP1 for cycle k+2 */
} buck_step_t;

/* ---- The per-cycle path's constants as values (bus scout, 27 Oct) ---------------
 * The firmware's arming and its first pass after arming take the fields vsum..inv_vin from
 * registers loaded at the loop entry (no literal-pool load between the arming sample and the
 * arming CMP1 write, nor between the first pass's sample and its CMP1 write; Src/buck.c);
 * the other passes pass buck_kc_init() with vsum, d_min, d_max and per taken from registers
 * formed once after the first pass, and the pass after a tunables pass also iref_lo, iref_hi
 * and half from registers formed before its wait (Src/buck.c, run_ctx_t.q; 28 Oct), and
 * every other caller passes buck_kc_init(), whose fields the compiler folds back into the
 * constants. buck_kc_init() gives the very values the constant forms used, so every
 * result is the same bit for bit. */
typedef struct {
    float vsum;                     /* (float)BUCK_V_PER_VSUM */
    float d_min, d_max;             /* BUCK_D_MIN, BUCK_D_MAX */
    float per;                      /* (float)BUCK_HRTIM_PERIOD */
    float inv_vin;                  /* (float)(1.0 / BUCK_VIN_V): the arming's duty */
    float iref_lo, iref_hi, half;   /* BUCK_IREF_MIN_A, BUCK_IREF_MAX_A, 0.5f (the duty's rounding) */
} buck_kc_t;

BUCK_INLINE buck_kc_t buck_kc_init(void)
{
    const buck_kc_t k = { (float)BUCK_V_PER_VSUM, BUCK_D_MIN, BUCK_D_MAX, (float)BUCK_HRTIM_PERIOD,
                          (float)(1.0 / BUCK_VIN_V), BUCK_IREF_MIN_A, BUCK_IREF_MAX_A, 0.5f };
    return k;
}

BUCK_INLINE uint32_t buck_duty_to_cmp_kc(float d, const buck_kc_t *kc)
{
    return (uint32_t)(buck_mul(d, kc->per) + kc->half) + BUCK_DT_TICKS;
}

/* What buck_ctrl_fast_pre() leaves for buck_ctrl_fast_commit(): the two
 * proportional terms and the duty (registers). */
typedef struct { float p_v, p_i, d; } buck_fast_pend_t;

/* Hot path, the part the CMP1 value needs: both PI outputs (the current PI on i_ref - i_Lm[k], the
 * measured average: stage 3), the CMP1 value. The state is not changed; the updates (both
 * integrators) are buck_ctrl_fast_commit(), which the firmware runs after its CMP1 write. gain_first:
 * the operand order of the current's and the current PI's proportional products (buck_adc_to_i_o,
 * buck_pi_out_kt_o), as before. */
BUCK_INLINE uint32_t buck_ctrl_fast_pre_kc(const buck_tune_t *t, float ki_t_v, float ki_t_i, const buck_state_t *s,
                                           uint32_t adc_v, uint32_t adc_i4, buck_step_t *st, buck_fast_pend_t *pd,
                                           const buck_kc_t *kc, int gain_first)
{
    const float v  = (float)adc_v * kc->vsum;                       /* buck_adc_to_v */
    const float im = buck_adc_to_i_o(t, adc_i4, gain_first);
    const float iref = buck_pi_out_kt(s->integ_v, t->kp_v, ki_t_v, kc->iref_lo, kc->iref_hi, s->vref - v, &pd->p_v);
    const float d    = buck_pi_out_kt_o(s->integ_i, t->kp_i, ki_t_i, kc->d_min, kc->d_max, iref - im, &pd->p_i,
                                        gain_first);
    const uint32_t cmp = buck_duty_to_cmp_kc(d, kc);
    pd->d = d;
    st->v = v; st->im = im; st->iref = iref; st->cmp = cmp;
    return cmp;
}

BUCK_INLINE uint32_t buck_ctrl_fast_pre(const buck_tune_t *t, float ki_t_v, float ki_t_i, const buck_state_t *s,
                                        uint32_t adc_v, uint32_t adc_i4, buck_step_t *st, buck_fast_pend_t *pd)
{
    const buck_kc_t kc = buck_kc_init();
    return buck_ctrl_fast_pre_kc(t, ki_t_v, ki_t_i, s, adc_v, adc_i4, st, pd, &kc, 0);
}

/* The state updates of the hot path: voltage and current integrators (buck_pi_step_kt), from the
 * same values. */
BUCK_INLINE void buck_ctrl_fast_commit(buck_state_t *s, const buck_step_t *st, const buck_fast_pend_t *pd)
{
    s->integ_v = buck_pi_commit(st->iref, pd->p_v);
    s->integ_i = buck_pi_commit(pd->d, pd->p_i);
}

/* Hot path: both PIs, with Ki*T of both PIs given
 * (buck_pi_ki_t of t->ki_v and t->ki_i). Returns the CMP1 value. */
BUCK_INLINE uint32_t buck_ctrl_fast_kt(const buck_tune_t *t, float ki_t_v, float ki_t_i, buck_state_t *s,
                                       uint32_t adc_v, uint32_t adc_i4, buck_step_t *st)
{
    buck_fast_pend_t pd;
    const uint32_t cmp = buck_ctrl_fast_pre(t, ki_t_v, ki_t_i, s, adc_v, adc_i4, st, &pd);
    buck_ctrl_fast_commit(s, st, &pd);
    return cmp;
}

/* Hot path: both PIs. Returns the CMP1 value. */
BUCK_INLINE uint32_t buck_ctrl_fast(const buck_tune_t *t, buck_state_t *s,
                                    uint32_t adc_v, uint32_t adc_i4, buck_step_t *st)
{
    return buck_ctrl_fast_kt(t, buck_pi_ki_t(t->ki_v), buck_pi_ki_t(t->ki_i), s, adc_v, adc_i4, st);
}

/* After the compare write: the trigger score and D4, with the score's coefficients given (buck_score_k
 * of *t). Writes the telemetry to *o except o->i_load (the observer's, buck_iload_obs) and returns g[k].
 * first_order: the score additions' operand order (buck_trigger_score_ko). Since stage 3 there is no
 * prediction here (the Padé estimator is removed); cmp_active still follows the CMP1 written (the TOC
 * frame sets it to the held duty; g_buck.st at a stop). */
BUCK_INLINE uint32_t buck_ctrl_slow_ko(const buck_tune_t *t, buck_score_k_t sk, buck_state_t *s,
                                       const buck_step_t *st, buck_out_t *o, int first_order)
{
    const float v = st->v, iref = st->iref;
    /* e_v = v_out - V_ref (eq. trigger_variables), S = a e_v^2 + b e_v + c (user, 25 Oct) */
    const float S = buck_trigger_score_ko(v - s->vref, sk, first_order);
    const uint32_t g = (S >= t->s_th) ? 1U : 0U;                           /* eq. trigger_indicator */
    s->cmp_active = st->cmp;                                               /* active from the next cycle */
    o->v = v; o->i_meas = st->im; o->i_avg = st->im; o->eps = 0.0f;        /* o->i_load: the observer's */
    o->i_ref = iref; o->i_peak = 0.0f; o->vref = s->vref; o->score = S; o->g = g; o->cmp = st->cmp;
    return g;
}

BUCK_INLINE uint32_t buck_ctrl_slow_k(const buck_tune_t *t, buck_score_k_t sk, buck_state_t *s,
                                      const buck_step_t *st, buck_out_t *o)
{
    return buck_ctrl_slow_ko(t, sk, s, st, o, 0);
}

/* After the compare write (the host tests' form: coefficients from *t). */
BUCK_INLINE uint32_t buck_ctrl_slow(const buck_tune_t *t, buck_state_t *s, const buck_step_t *st, buck_out_t *o)
{
    return buck_ctrl_slow_k(t, buck_score_k(t), s, st, o);
}

/* ---- Reference slew ------------------------------------------------------------
 * Soft-start and any later change of vref_counts: the reference moves toward
 * the target by BUCK_SOFTSTART_STEP_V per cycle, and the last step lands
 * exactly on the target:
 *   up:   r = vref + step;  r >= target -> target
 *   down: r = vref - step;  r <= target -> target
 * The target changes only where the firmware re-reads the tunables (every
 * BUCK_TELEMETRY_EVERY cycles), so what the next BUCK_TELEMETRY_EVERY steps
 * need is decided once per reload (buck_ramp_mode; since 28 Oct in the pass
 * after the tunables pass, Src/buck.c), and the control loop runs a
 * copy of itself that has exactly that step built in (buck_ramp_step with a
 * constant mode): no ramp branch in any cycle, and no ramp code at all once the
 * reference is at the target.
 *   BUCK_RAMP_HOLD     vref == target: nothing to do.
 *   BUCK_RAMP_FREE_UP  the target is more than BUCK_RAMP_FAR_V above vref: the
 *                      next BUCK_TELEMETRY_EVERY steps cannot reach it, so the
 *                      limit never applies and each step is vref + step alone.
 *   BUCK_RAMP_UP       the up step with its limit (close to the target).
 *   BUCK_RAMP_DOWN     the down step with its limit.
 * Every cycle of an epoch takes its step, also after the reference has arrived
 * (the limited step then lands on the target again), so vref follows exactly
 * the original per-cycle trajectory. A target changed at a reload, including
 * back to a value reached before, is ramped to from wherever vref is. */
enum { BUCK_RAMP_HOLD = 0U, BUCK_RAMP_UP = 1U, BUCK_RAMP_DOWN = 2U, BUCK_RAMP_FREE_UP = 3U };

/* FREE_UP bound. Below BUCK_RAMP_FREE_MAX_V = 2^BUCK_RAMP_FREE_MAX_LOG2 V each
 * float addition rounds by at most half an ulp, BUCK_RAMP_ROUND_V = 2^(LOG2-24)
 * V, so BUCK_TELEMETRY_EVERY steps add at most
 * BUCK_TELEMETRY_EVERY * (step + BUCK_RAMP_ROUND_V) < (BUCK_TELEMETRY_EVERY + 1)
 * steps; one more step covers the rounding of the test itself. The bound
 * assumes the constant step BUCK_SOFTSTART_STEP_V: a tunable step needs a new
 * proof. */
#define BUCK_RAMP_FAR_V          ((float)(BUCK_TELEMETRY_EVERY + 2U) * BUCK_SOFTSTART_STEP_V)
#define BUCK_RAMP_FREE_MAX_LOG2  4
#define BUCK_RAMP_FREE_MAX_V     ((float)(1UL << BUCK_RAMP_FREE_MAX_LOG2))            /* 16 V */
#define BUCK_RAMP_ROUND_V        (1.0f / (float)(1UL << (24 - BUCK_RAMP_FREE_MAX_LOG2)))
_Static_assert(BUCK_TELEMETRY_EVERY * BUCK_RAMP_ROUND_V < BUCK_SOFTSTART_STEP_V, "FREE_UP rounding bound");

BUCK_INLINE float buck_ramp_target(const buck_tune_t *t)
{
    return (float)t->vref_counts * (float)BUCK_V_PER_COUNT;
}

/* The step for the next epoch, from vref after the latest step (that of the
 * cycle that read the tunables, or of the first pass after arming) and the
 * target of the tunables read last. Since 28 Oct the firmware calls this after
 * a tunables pass in the next pass, before that pass's own step: the same vref. */
BUCK_INLINE uint32_t buck_ramp_mode(float vref, float target)
{
    if (vref < target)
        return (vref + BUCK_RAMP_FAR_V < target && target < BUCK_RAMP_FREE_MAX_V) ? BUCK_RAMP_FREE_UP : BUCK_RAMP_UP;
    return (vref > target) ? BUCK_RAMP_DOWN : BUCK_RAMP_HOLD;
}

/* One cycle's step. step = BUCK_SOFTSTART_STEP_V (a parameter so that the
 * firmware can hold it in a register). */
BUCK_INLINE float buck_ramp_step(uint32_t mode, float vref, float target, float step)
{
    if (mode == BUCK_RAMP_FREE_UP) return vref + step;
    if (mode == BUCK_RAMP_UP)   { const float r = vref + step; return (r >= target) ? target : r; }
    if (mode == BUCK_RAMP_DOWN) { const float r = vref - step; return (r <= target) ? target : r; }
    return vref;
}

/* Arming (gates were off). Starts the reference at the measured output, the
 * current reference at the measured current and the duty at v_out/V_in
 * (since stage 3 no estimator prediction). Returns the first CMP1 value. */
/* The arming in two parts (the firmware writes CMP1 between them):
 * buck_ctrl_arm_cmp() forms the first CMP1 value from v_out alone (the duty
 * v_out/V_in, clamped) and returns it with v and the duty; buck_ctrl_arm_rest()
 * does everything else. buck_ctrl_arm() is both, the same operations on the
 * same values. */
typedef struct { float v, d0; } buck_arm_pend_t;

BUCK_INLINE uint32_t buck_ctrl_arm_cmp_kc(uint32_t adc_v, buck_arm_pend_t *pa, const buck_kc_t *kc)
{
    const float v  = (float)adc_v * kc->vsum;                       /* buck_adc_to_v */
    float d0 = v * kc->inv_vin;
    d0 = (d0 > kc->d_max) ? kc->d_max : d0;
    d0 = (d0 < kc->d_min) ? kc->d_min : d0;
    pa->v = v;
    pa->d0 = d0;
    return buck_duty_to_cmp_kc(d0, kc);
}

BUCK_INLINE uint32_t buck_ctrl_arm_cmp(uint32_t adc_v, buck_arm_pend_t *pa)
{
    const buck_kc_t kc = buck_kc_init();
    return buck_ctrl_arm_cmp_kc(adc_v, pa, &kc);
}

BUCK_INLINE void buck_ctrl_arm_rest_kc(const buck_tune_t *t, buck_state_t *s, const buck_arm_pend_t *pa, uint32_t cmp0,
                                       uint32_t adc_i4, buck_out_t *o)
{
    const float v  = pa->v;
    const float im = buck_adc_to_i_o(t, adc_i4, 1);
    const float target = buck_ramp_target(t);
    s->vref = (v < target) ? v : target;
    float i0 = im;
    i0 = (i0 > BUCK_IREF_MAX_A) ? BUCK_IREF_MAX_A : i0;
    i0 = (i0 < BUCK_IREF_MIN_A) ? BUCK_IREF_MIN_A : i0;
    s->integ_v = i0;
    s->integ_i = pa->d0;
    s->iv = 0.0f; s->iv_pred = 0.0f; s->avg_pred = 0.0f; s->kappa = 0.0f;   /* unused since stage 3 */
    s->cmp_active = cmp0;
    o->v = v; o->i_meas = im; o->i_avg = im; o->eps = 0.0f;
    o->i_load = im;                         /* the observer starts here (buck_obs_seed) */
    o->i_ref = i0; o->i_peak = 0.0f; o->vref = s->vref; o->score = 0.0f; o->g = 0U;
    o->cmp = s->cmp_active;
}

BUCK_INLINE void buck_ctrl_arm_rest(const buck_tune_t *t, buck_state_t *s, const buck_arm_pend_t *pa, uint32_t cmp0,
                                    uint32_t adc_i4, buck_out_t *o)
{
    buck_ctrl_arm_rest_kc(t, s, pa, cmp0, adc_i4, o);
}

BUCK_INLINE uint32_t buck_ctrl_arm(const buck_tune_t *t, buck_state_t *s,
                                   uint32_t adc_v, uint32_t adc_i4, buck_out_t *o)
{
    buck_arm_pend_t pa;
    const uint32_t cmp0 = buck_ctrl_arm_cmp(adc_v, &pa);
    buck_ctrl_arm_rest(t, s, &pa, cmp0, adc_i4, o);
    return cmp0;
}

#endif
