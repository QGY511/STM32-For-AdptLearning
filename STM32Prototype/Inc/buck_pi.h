#ifndef BUCK_PI_H
#define BUCK_PI_H
#include "buck_config.h"

/* Saturation tracking anti-windup. The integrator retains fractional units.
 * A negative integrator is valid when the proportional term saturates high. */
/* Clamped control output u (Q16, in the caller's units: duty ticks in voltage
 * mode, peak amperes in current mode) and proportional term p. The caller acts
 * on u immediately, then updates integrator = u - p AFTER the latency-critical
 * compare write (keeps the anti-windup store off the path). */
static inline __attribute__((always_inline)) int32_t
buck_pi_u(int32_t error, int32_t integrator_q16, int32_t kp_q16, int32_t ki_q16,
          int32_t umin_q16, int32_t umax_q16, int32_t *p_out)
{
    const int32_t p = kp_q16 * error;
    int32_t u = integrator_q16 + ki_q16 * error + p;
    if (u > umax_q16) u = umax_q16;
    else if (u < umin_q16) u = umin_q16;
    *p_out = p;
    return u;
}

/* Full voltage-mode step (compute + integrator update), duty-tick clamps. Used
 * by host tests; the ISR uses buck_pi_u so it can defer the integrator store
 * past the duty write. */
static inline __attribute__((always_inline)) uint32_t
buck_pi_step(int32_t error, int32_t *integrator_q16, int32_t kp_q16, int32_t ki_q16)
{
    int32_t p;
    const int32_t u = buck_pi_u(error, *integrator_q16, kp_q16, ki_q16,
                                BUCK_MIN_Q16, BUCK_MAX_Q16, &p);
    *integrator_q16 = u - p;
    return (uint32_t)u >> BUCK_PI_SHIFT;
}

#endif
