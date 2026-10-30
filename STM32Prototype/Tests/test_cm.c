/* Predictive peak-current-mode inversion (Inc/buck_cm.h): the on-time it
 * computes, pushed forward through the estimator's own Pade/trapezoidal model
 * with EXACT coefficients at that on-time, must reproduce the requested peak.
 * Also checks the clamps and that a whole closed current loop (inversion +
 * estimator recursion) pins the peak from cycle to cycle at D < 0.5. */
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "buck_estimator.h"
#include "buck_cm.h"

static double a_ex(double tau){ double x=(BUCK_R_OHM/BUCK_L_H)*tau; return (2.0-x)/(2.0+x); }
static double b_ex(double tau){ double x=(BUCK_R_OHM/BUCK_L_H)*tau; return 2.0*tau/(BUCK_L_H*(2.0+x)); }

int main(void)
{
    const double tick = BUCK_TICK_S, vcount = BUCK_UV_PER_COUNT * 1e-6;
    printf("L/tick=%d ticks*V/A  r=%d/4096 ohm\n", (int)BUCK_CM_L_OVER_TICK, (int)BUCK_CM_R_Q12);

    /* 1. Inversion accuracy over the operating space (only clamp-free points). */
    double maxerr = 0.0; unsigned n = 0;
    for (int adc = 700; adc <= 1300; adc += 50)
    for (double iv = -3.0; iv <= 8.0; iv += 0.5)
    for (double ipk = 0.5; ipk <= 10.0; ipk += 0.5) {
        const uint32_t ton = buck_cm_ton_ticks((int32_t)(ipk * 65536.0), (int32_t)(iv * 1048576.0), buck_v_q28((uint32_t)adc));
        if (ton <= BUCK_DUTY_MIN || ton >= BUCK_DUTY_MAX) continue;   /* clamped: not a pure inversion */
        const double tau = (ton - BUCK_DT_FINE_TICKS) * tick, vout = adc * vcount;   /* inductor sees CMP1 - dead time */
        const double ipk_model = a_ex(tau) * iv + b_ex(tau) * (BUCK_VIN_V - vout);   /* eq 13 */
        const double e = fabs(ipk_model - ipk);
        if (e > maxerr) maxerr = e;
        ++n;
    }
    printf("inversion: %u points, max |peak error| = %.4f A (1 tick = %.3f A of ramp)\n",
           n, maxerr, (BUCK_VIN_V - 1.0) / BUCK_L_H * tick);
    assert(n > 1000);
    assert(maxerr < 0.02);   /* tick quantisation ~3 mA + Q16 rounding */

    /* 2. Clamps. */
    assert(buck_cm_ton_ticks((int32_t)(1.0 * 65536), (int32_t)(5.0 * 1048576), buck_v_q28(1000)) == BUCK_DUTY_MIN);   /* di < 0 */
    assert(buck_cm_ton_ticks((int32_t)(10.0 * 65536), (int32_t)(-8.0 * 1048576), buck_v_q28(1000)) == BUCK_DUTY_MAX); /* huge dI */
    assert(buck_cm_ton_ticks((int32_t)(10.0 * 65536), (int32_t)(0.0 * 1048576), buck_v_q28(4095)) <= BUCK_DUTY_MAX);  /* V_L floor */
    assert(buck_cm_ton_ticks((int32_t)(0.5 * 65536), 0, buck_v_q28(0)) >= BUCK_DUTY_MIN);
    puts("PASS: on-time inversion reproduces the Pade peak; clamps hold");

    /* 3. Closed inner loop: estimator recursion driven by the inversion pins the
     *    peak within a few cycles and stays there (D ~ 0.2 < 0.5), at fixed V_out. */
    buck_est_lut_init();
    buck_est_t s; buck_est_reset(&s, 1000);
    s.iv_q20 = (int32_t)(-1.0 * 1048576.0);          /* wrong initial valley on purpose */
    const int32_t ipk_ref = (int32_t)(4.0 * 65536.0);
    uint32_t duty = BUCK_DUTY_MIN;
    double last_peak = 0.0;
    for (int k = 0; k < 40; ++k) {
        buck_est_out_t eo;
        buck_estimate(&s, 1000, duty, &eo);                 /* period k with the committed duty */
        duty = buck_cm_ton_ticks(ipk_ref, s.iv_q20, buck_v_q28(1000));  /* on-time for period k+1 */
        last_peak = eo.ip_q20 / 1048576.0;
        if (k >= 8) assert(fabs(last_peak - 4.0) < 0.05);   /* peak pinned after the transient */
    }
    printf("closed loop: peak settles to %.3f A for a 4.0 A reference, duty %u ticks (%.1f%%)\n",
           last_peak, duty, 100.0 * duty / BUCK_HRTIM_PERIOD);
    puts("PASS: predictive current loop pins the peak (stable at D < 0.5)");
    return 0;
}
