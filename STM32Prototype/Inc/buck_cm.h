#ifndef BUCK_CM_H
#define BUCK_CM_H
#include <stdint.h>
#include "buck_config.h"

/* Predictive (model-based) peak-current-mode inner loop.
 *
 * The Pade [1/1] current update used by the estimator is exactly the
 * trapezoidal integration of  L di/dt = V_L - r i  over the on-time:
 *     i_pk = i_v + (T_on/L) * (V_L - r (i_v + i_pk)/2),   V_L = V_in - V_out.
 * Solving that same relation for the on-time gives the exact inverse of the
 * model, one division, no iteration:
 *     T_on = L (i_pk_ref - i_v) / (V_L - r (i_v + i_pk_ref)/2).
 * The ISR feeds it the valley the estimator predicts for the NEXT period
 * (iv_next, which buck_estimate leaves in s->iv_q20) and writes the result to
 * CMP1 for that period: sample in period n, predict the valley of n+1, program
 * the on-time of n+1 (Chen/Prodic/Erickson/Maksimovic, predictive current-
 * programmed control). Pinning the peak gives the classic valley recursion
 * d iv(n+1) = -(m2/m1) d iv(n): stable for D < 0.5, which BUCK_DUTY_MAX
 * guarantees here (no slope compensation needed at D ~ 0.2).
 *
 * Fixed point: currents Q16 A, voltages Q16 V. T_on[ticks] = (L/tick) * dI / V_L
 * with L/tick = BUCK_CM_L_OVER_TICK (~1774 ticks*V/A). dI is clamped so the
 * product stays in int32; one 32-bit UDIV (2..12 clocks on Cortex-M4, bounded).
 * The result is clamped to the PWM duty range, so a saturated on-time is the
 * only way the loop can "open"; the voltage PI's anti-windup tracks that. */

#define BUCK_CM_VIN_Q16        ((int32_t)(BUCK_VIN_V * 65536.0 + 0.5))
#define BUCK_CM_R_Q12          ((int32_t)(BUCK_R_OHM * 4096.0 + 0.5))                  /* series r, Q12 ohm */
#define BUCK_CM_L_OVER_TICK    ((int32_t)(BUCK_L_H / BUCK_TICK_S + 0.5))               /* ticks*V/A */
#define BUCK_CM_VL_MIN_Q16     ((int32_t)(0.5 * 65536.0))    /* floor on the inductor voltage (V) */
#define BUCK_CM_DI_MAX_Q16     ((int32_t)(16.0 * 65536.0))   /* dI clamp: 16 A * 1774 < 2^31 */
_Static_assert((int64_t)BUCK_CM_DI_MAX_Q16 * BUCK_CM_L_OVER_TICK < 2147483647LL, "cm: dI*L/tick must fit int32");

/* Peak-current-reference range of the voltage PI (Q16 A). The upper bound is the
 * same peak the time-optimal pulse is capped to. */
#define BUCK_CM_IPK_MIN_Q16    ((int32_t)(0.5 * 65536.0))
#define BUCK_CM_IPK_MAX_Q16    ((int32_t)(BUCK_OPT_IPK_MAX_A * 65536.0))
_Static_assert(BUCK_CM_IPK_MAX_Q16 <= BUCK_CM_DI_MAX_Q16 + (int32_t)(2.0 * 65536.0),
               "cm: peak reference range vs dI clamp");

/* Raw PHYSICAL on-time (HRTIM ticks the inductor sees V_in - V_out; NOT the
 * CMP1 value, which is this plus the rising dead time BUCK_DT_FINE_TICKS; NOT
 * clamped to the PWM duty range; 0 when the valley is already at/above the
 * peak) that makes the predicted peak of a period starting at valley iv_next
 * equal ipk_ref. Shared by the every-cycle current loop (clamped below) and the
 * time-optimal pulse solve, which needs on-times beyond DUTY_MAX inside its
 * stretched period. iv_next: Q20 A; vout_q28: this cycle's V_out (Q28 V). */
static inline __attribute__((always_inline)) uint32_t
buck_cm_ton_raw(int32_t ipk_ref_q16, int32_t iv_next_q20, int32_t vout_q28)
{
    const int32_t ivn  = iv_next_q20 >> 4;                                   /* Q20 -> Q16 A */
    int32_t di = ipk_ref_q16 - ivn;
    if (di <= 0) return 0U;                               /* already above the peak */
    if (di > BUCK_CM_DI_MAX_Q16) di = BUCK_CM_DI_MAX_Q16;
    /* Inductor voltage over the ramp with the trapezoidal r drop: r*(iv+ipk)/2.
     * |iv+ipk| < 2^21 (32 A) keeps the Q12 product in int32. */
    int32_t vl = BUCK_CM_VIN_Q16 - (vout_q28 >> 12)
               - (((ivn + ipk_ref_q16) * BUCK_CM_R_Q12) >> 13);
    if (vl < BUCK_CM_VL_MIN_Q16) vl = BUCK_CM_VL_MIN_Q16;
    return (uint32_t)(di * BUCK_CM_L_OVER_TICK) / (uint32_t)vl;             /* one UDIV */
}

/* CMP1 value (HRTIM ticks, clamped to [DUTY_MIN, DUTY_MAX]) that makes the next
 * period's predicted peak equal ipk_ref: the physical on-time plus the rising
 * dead time. iv_next: predicted valley at the start of that period (Q20 A, from
 * the estimator); vout_q28: this cycle's V_out sample as the estimator computed
 * it (Q28 V), so nothing is recomputed. */
static inline __attribute__((always_inline)) uint32_t
buck_cm_ton_ticks(int32_t ipk_ref_q16, int32_t iv_next_q20, int32_t vout_q28)
{
    uint32_t ton = buck_cm_ton_raw(ipk_ref_q16, iv_next_q20, vout_q28) + BUCK_DT_FINE_TICKS;
    if (ton < BUCK_DUTY_MIN) ton = BUCK_DUTY_MIN;         /* includes the di <= 0 case */
    else if (ton > BUCK_DUTY_MAX) ton = BUCK_DUTY_MAX;
    return ton;
}

#endif
