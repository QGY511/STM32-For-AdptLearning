#ifndef BUCK_ESTIMATOR_H
#define BUCK_ESTIMATOR_H
#include <stdint.h>
#include "buck_config.h"

/* Cycle-by-cycle load and valley/peak inductor-current estimator.
 * TPEL adaptive-control paper, Section III-A:
 *   - first-order [1/1] Pade current update (eqs 9-12),
 *   - peak from valley (eq 13), recursive next-valley (eq 14),
 *   - trapezoidal cycle-average (eq 15),
 *   - load = avg inductor current - output-capacitor current (eqs 7-8),
 *   - causal low-pass on the load estimate (eqs 20-21).
 *
 * Fixed-point, DIVIDE-FREE and CALL-FREE. Every runtime product uses the HIGH
 * word of a 32x32 SMULL (SMULLH = (a*b)>>32, one cycle on Cortex-M4).
 * Q-formats: currents Q20 A (+-1024 A, ~1 uA LSB), voltages Q28 V, a Q31,
 * b Q24 A/V, D Q31.
 *
 * The four Pade coefficients a_on(T_on), b_on(T_on), a_off(T_off), b_off(T_off)
 * depend only on the active on-time, so they come from a LOOKUP TABLE indexed by
 * the duty (BUCK_EST_LUT_SHIFT-tick bins, bin-centre value, EXACT (2-x)/(2+x)
 * and 2tau/(L(2+x)) evaluated in double at init). This replaces the per-cycle
 * 1/(2+x) series (two 3-term series + four SMULL rescales, ~35 clocks) with one
 * index computation and two LDRD, and removes the series' 0.5 % coefficient
 * error. Table cost: BUCK_EST_LUT_N * 16 bytes of SRAM (7.2 KB at 8-tick bins).
 * Bin-centre quantisation error at 8 ticks (1.5 ns): |dT_on| <= 0.75 ns ->
 * |d i_peak| <= (Vin-Vout)/L * 0.75 ns ~ 11 mA, comparable to one ADC count of
 * feedback noise (~5 mA in i_avg).
 *
 * Exports a slow display average in integer amps plus per-cycle raw Q20 for
 * validation. */

#define BUCK_SMULLH(a, b) ((int32_t)(((int64_t)(a) * (int32_t)(b)) >> 32))
#define BUCK_SHL(x, n)    ((int32_t)((uint32_t)(x) << (n)))

#ifndef BUCK_EST_LUT_SHIFT
#define BUCK_EST_LUT_SHIFT 3U   /* 8 HRTIM ticks per bin */
#endif
#define BUCK_EST_LUT_N ((((BUCK_DUTY_MAX - BUCK_DUTY_MIN) >> BUCK_EST_LUT_SHIFT)) + 1U)
_Static_assert(BUCK_EST_LUT_N >= 2U && BUCK_EST_LUT_N <= 4096U, "estimator LUT size");

typedef struct {
    int32_t a_on;   /* Q31 */
    int32_t b_on;   /* Q24 A/V */
    int32_t a_off;  /* Q31 */
    int32_t b_off;  /* Q24 A/V */
} buck_est_coef_t;

typedef struct {
    int32_t iv_q20;         /* valley inductor current, Q20 A (state) */
    int32_t vout_prev_q28;  /* previous v_out, Q28 V (state) */
    int32_t iload_q20;      /* filtered load estimate, Q20 A (state) */
    int32_t iavg_q20;       /* last cycle-average inductor current, Q20 A (kept for the capture) */
    /* Slow display averages (leaky integrator), Q20 A. */
    int32_t slow_iv_q20, slow_ip_q20, slow_iavg_q20, slow_iload_q20;
} buck_est_t;

typedef struct {
    int32_t iv_q20, ip_q20, iavg_q20, iload_q20;     /* per-cycle values, Q20 A (iload: filtered state) */
} buck_est_out_t;

/* Unfiltered load for one period: i_avg - C dv/dt (eqs 7-8), Q20 A. Used by
 * buck_est_finish every cycle and by the pulse solve, which recomputes it one
 * period after the trigger: the trigger fires on the load edge itself, where the
 * output capacitor's ESR step makes C dv/dt read amperes for that one sample. */
static inline __attribute__((always_inline)) int32_t buck_est_iload_raw(int32_t iavg_q20, int32_t dv_q28)
{
    return iavg_q20 - BUCK_SHL(BUCK_SMULLH(BUCK_CTS_Q23, dv_q28), 1);
}

/* One table per translation unit that uses the estimator (the ISR file and the
 * host test). Filled by buck_est_lut_init() at start-up (thread mode, double
 * math, once). Kept in SRAM (zero wait state) next to the other hot state. */
static buck_est_coef_t buck_est_lut[BUCK_EST_LUT_N];

/* Slow-average (Q20 A) -> milliamps and rounded integer amperes. */
static inline __attribute__((always_inline)) int32_t buck_q20_to_ma(int32_t i_q20)
{
    return (int32_t)(((int64_t)i_q20 * 1000) >> 20);
}
static inline __attribute__((always_inline)) int32_t buck_q20_to_a(int32_t i_q20)
{
    return (i_q20 + (1 << 19)) >> 20;
}

static inline __attribute__((always_inline)) int32_t buck_v_q28(uint32_t adc)
{
    return (int32_t)adc * BUCK_VCOUNT_ADC_Q28;   /* Q28 volts */
}

/* a-type product: a(Q31) * current(Q20) -> Q20  (SMULLH gives Q19, <<1). */
static inline __attribute__((always_inline)) int32_t buck_mul_a(int32_t a_q31, int32_t i_q20)
{
    return BUCK_SHL(BUCK_SMULLH(a_q31, i_q20), 1);
}

static inline __attribute__((always_inline)) void buck_est_reset(buck_est_t *s, uint32_t adc)
{
    s->iv_q20 = 0;
    s->vout_prev_q28 = buck_v_q28(adc);
    s->iload_q20 = 0;
    s->iavg_q20 = 0;
    s->slow_iv_q20 = s->slow_ip_q20 = s->slow_iavg_q20 = s->slow_iload_q20 = 0;
}

/* Exact [1/1] Pade coefficients for an interval of tau seconds (eqs 11-12):
 * a = (2 - x)/(2 + x), b = 2 tau / (L (2 + x)), x = (r/L) tau. Init-time only,
 * SINGLE precision on purpose: the Cortex-M4 FPU is single precision, so float
 * keeps the soft-float double library out of the image (tools/check_elf.py bans
 * __aeabi_d*). Float rounding (~6e-8 relative) is four orders of magnitude
 * below the bin quantisation, so the table is as exact as the bins allow. */
static inline void buck_est_lut_init(void)
{
    const float theta = (float)(BUCK_R_OHM / BUCK_L_H);          /* 1/s, compile-time double -> float */
    const float tick  = (float)BUCK_TICK_S;
    const float two_over_L = (float)(2.0 / BUCK_L_H);
    for (uint32_t k = 0U; k < BUCK_EST_LUT_N; ++k) {
        /* Bin centre in CMP1 ticks: DUTY_MIN + k*2^SHIFT + 2^(SHIFT-1). The
         * high side conducts from the rising dead time to CMP1, so the inductor
         * sees CMP1 - BUCK_DT_FINE_TICKS of on-time and the rest of the period
         * (low side, or its body diode during the dead times) as off-time. */
        const float ton_ticks = (float)(BUCK_DUTY_MIN + (k << BUCK_EST_LUT_SHIFT)
                                        + (1U << (BUCK_EST_LUT_SHIFT - 1U)))
                              - (float)BUCK_DT_FINE_TICKS;
        const float ton  = ton_ticks * tick;
        const float toff = ((float)BUCK_HRTIM_PERIOD - ton_ticks) * tick;
        const float xon = theta * ton, xoff = theta * toff;
        buck_est_lut[k].a_on  = (int32_t)((2.0f - xon)  / (2.0f + xon)  * 2147483648.0f + 0.5f);
        buck_est_lut[k].a_off = (int32_t)((2.0f - xoff) / (2.0f + xoff) * 2147483648.0f + 0.5f);
        buck_est_lut[k].b_on  = (int32_t)(two_over_L * ton  / (2.0f + xon)  * 16777216.0f + 0.5f);
        buck_est_lut[k].b_off = (int32_t)(two_over_L * toff / (2.0f + xoff) * 16777216.0f + 0.5f);
    }
}

/* The one-cycle update is split in two so the current-mode ISR can run only the
 * part it needs BEFORE the compare write (the next-valley prediction, ~20
 * clocks) and the rest after it:
 *   buck_est_predict : Pade coefficients + peak (eq 13) + next valley (eq 14);
 *                      returns the next valley, stores the peak in *ip_out
 *   buck_est_finish  : cycle average (eq 15), load (eqs 7-8, 20-21), outputs,
 *                      display averages, state update
 * Both take v_out in Q28 V (buck_v_q28(adc)), computed once by the caller.
 * buck_estimate() = predict + finish (voltage-mode ISR and host tests). */
static inline __attribute__((always_inline)) int32_t
buck_est_predict_iv(int32_t iv_q20, int32_t vout, uint32_t duty, int32_t *ip_out)
{
    /* Pade coefficients (eqs 11-12) from the table. duty is always inside
     * [DUTY_MIN, DUTY_MAX] (PI clamp / new_duty clamp); the unsigned compare
     * also folds a duty < DUTY_MIN wrap-around into the last bin, so a stray
     * value can never index outside the table. */
    uint32_t idx = (duty - BUCK_DUTY_MIN) >> BUCK_EST_LUT_SHIFT;
    if (idx >= BUCK_EST_LUT_N) idx = BUCK_EST_LUT_N - 1U;
    const buck_est_coef_t *const c = &buck_est_lut[idx];

    /* Peak (eq 13) and next valley (eq 14). b*v: Q24*Q28 -> Q20 (direct). */
    const int32_t ip = buck_mul_a(c->a_on, iv_q20) + BUCK_SMULLH(c->b_on, BUCK_VIN_Q28 - vout);
    *ip_out = ip;
    return buck_mul_a(c->a_off, ip) - BUCK_SMULLH(c->b_off, vout);
}
static inline __attribute__((always_inline)) int32_t
buck_est_predict(const buck_est_t *s, int32_t vout, uint32_t duty, int32_t *ip_out)
{
    return buck_est_predict_iv(s->iv_q20, vout, duty, ip_out);
}

static inline __attribute__((always_inline)) void
buck_est_finish(buck_est_t *s, int32_t vout, uint32_t duty, int32_t ip, int32_t iv_next,
                buck_est_out_t *out)
{
    const int32_t iv = s->iv_q20;

    /* Trapezoidal average (eq 15): (ip + D iv + (1-D) iv+)/2 = (ip + iv+ + D (iv - iv+))/2. */
    const int32_t d = (int32_t)duty * BUCK_INV_TSW_Q31;          /* Q31 */
    const int32_t iavg = (ip + iv_next + buck_mul_a(d, iv - iv_next)) >> 1;

    /* Load (eqs 7-8): iavg - (C/T_sw)*dv. CTS(Q23)*dv(Q28) -> Q19, <<1 = Q20. */
    const int32_t dv = vout - s->vout_prev_q28;
    const int32_t iload_raw = buck_est_iload_raw(iavg, dv);
    /* Causal low-pass (eqs 20-21). alpha(Q31)*dcurrent(Q20) -> Q19, <<1 = Q20. */
    const int32_t iload = s->iload_q20
                        + BUCK_SHL(BUCK_SMULLH(BUCK_EST_ALPHA_Q31, iload_raw - s->iload_q20), 1);

    /* Per-cycle raw (Q20) for validation. */
    out->iv_q20 = iv;
    out->ip_q20 = ip;
    out->iavg_q20 = iavg;
    out->iload_q20 = iload;

#if BUCK_EST_DISPLAY
    /* Slow display average (leaky integrator); converted to mA/A by the caller. */
    s->slow_iv_q20    += (iv        - s->slow_iv_q20)    >> BUCK_EST_SLOW_SHIFT;
    s->slow_ip_q20    += (ip        - s->slow_ip_q20)    >> BUCK_EST_SLOW_SHIFT;
    s->slow_iavg_q20  += (iavg      - s->slow_iavg_q20)  >> BUCK_EST_SLOW_SHIFT;
    s->slow_iload_q20 += (iload     - s->slow_iload_q20) >> BUCK_EST_SLOW_SHIFT;
#endif

    s->iv_q20 = iv_next;
    s->vout_prev_q28 = vout;
    s->iload_q20 = iload;
    s->iavg_q20 = iavg;
}

/* Measurement-assisted load estimate (the manuscript's "any estimated quantity
 * can be replaced by its measured counterpart"): the average inductor current
 * of the period is MEASURED, so only eqs 7-8 and the low-pass 20-21 remain.
 * iavg_q20: measured average current of this period (sample + ripple
 * correction); vout: this cycle's sample, Q28 V. ~10 clocks. */
static inline __attribute__((always_inline)) void
buck_load_update(buck_est_t *s, int32_t iavg_q20, int32_t vout)
{
    const int32_t iload_raw = buck_est_iload_raw(iavg_q20, vout - s->vout_prev_q28);
    s->iload_q20 += BUCK_SHL(BUCK_SMULLH(BUCK_EST_ALPHA_Q31, iload_raw - s->iload_q20), 1);
    s->vout_prev_q28 = vout;
    s->iavg_q20 = iavg_q20;
#if BUCK_EST_DISPLAY
    s->slow_iload_q20 += (s->iload_q20 - s->slow_iload_q20) >> BUCK_EST_SLOW_SHIFT;
#endif
}

/* One switching-cycle update. adc = v_out counts; duty = active T_on ticks.
 * ~40 clocks on Cortex-M4 (was ~85 with the per-cycle series coefficients). */
static inline __attribute__((always_inline)) void
buck_estimate(buck_est_t *s, uint32_t adc, uint32_t duty, buck_est_out_t *out)
{
    const int32_t vout = buck_v_q28(adc);
    int32_t ip;
    const int32_t iv_next = buck_est_predict(s, vout, duty, &ip);
    buck_est_finish(s, vout, duty, ip, iv_next, out);
}

#endif
