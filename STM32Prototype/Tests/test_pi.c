#include <assert.h>
#include <stdio.h>
#include "buck_pi.h"

static uint32_t seed = 0x600474U;
static uint32_t random_u32(void)
{
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return seed;
}

int main(void)
{
    assert(BUCK_HRTIM_PERIOD == 8960U);
    /* BUCK_TARGET_ADC is the legacy sense-network prediction (2266/5490 divider) for BUCK_VOUT_MV;
     * the board senses directly and runs at BUCK_SETPOINT_DEFAULT_ADC. Check the macro arithmetic only. */
    assert(BUCK_TARGET_ADC == (uint32_t)((double)BUCK_VOUT_MV * 2266.0 / 5490.0 * 4096.0 / 3300.0 + 0.5));
    assert(BUCK_DUTY_MIN == 896U);  /* 10% before dead time */
    assert(BUCK_DUTY_MAX == 4480U); /* 50% before dead time */
    assert(BUCK_DT_COUNTS == 68U);
    int32_t i = BUCK_MIN_Q16;
    int64_t oracle_i = BUCK_MIN_Q16;
    /* Wide oracle and UBSan catch overflow and anti-windup divergence under
     * repeated rail reversals, sustained saturation and full-scale noise. */
    for (unsigned n = 0; n < 2000000U; ++n) {
        int32_t e;
        switch (n % 8U) {
        case 0: e = 4095; break;
        case 1: e = -4095; break;
        case 2: e = 0; break;
        case 3: e = 1; break;
        case 4: e = -1; break;
        default: e = (int32_t)(random_u32() % 8191U) - 4095; break;
        }
        const int64_t p = (int64_t)BUCK_KP_Q16 * e;
        int64_t expected = oracle_i + (int64_t)BUCK_KI_Q16 * e + p;
        if (expected < BUCK_MIN_Q16) expected = BUCK_MIN_Q16;
        if (expected > BUCK_MAX_Q16) expected = BUCK_MAX_Q16;
        oracle_i = expected - p;
        const uint32_t duty = buck_pi_step(e, &i, BUCK_KP_Q16, BUCK_KI_Q16);
        assert(duty == (uint32_t)(expected >> 16));
        assert(i == oracle_i);
        assert(duty >= BUCK_DUTY_MIN && duty <= BUCK_DUTY_MAX);
    }
    for (unsigned n = 0; n < 100000U; ++n) buck_pi_step(4095, &i, BUCK_KP_Q16, BUCK_KI_Q16);
    assert(buck_pi_step(4095, &i, BUCK_KP_Q16, BUCK_KI_Q16) == BUCK_DUTY_MAX);
    assert(buck_pi_step(-4095, &i, BUCK_KP_Q16, BUCK_KI_Q16) < BUCK_DUTY_MAX);
    for (unsigned n = 0; n < 100000U; ++n) buck_pi_step(-4095, &i, BUCK_KP_Q16, BUCK_KI_Q16);
    assert(buck_pi_step(-4095, &i, BUCK_KP_Q16, BUCK_KI_Q16) == BUCK_DUTY_MIN);
    assert(buck_pi_step(4095, &i, BUCK_KP_Q16, BUCK_KI_Q16) > BUCK_DUTY_MIN);
    i = BUCK_MIN_Q16;
    for (unsigned n = 0; n < 1000U; ++n) buck_pi_step(1, &i, BUCK_KP_Q16, BUCK_KI_Q16);
    assert(buck_pi_step(1, &i, BUCK_KP_Q16, BUCK_KI_Q16) > BUCK_DUTY_MIN); /* fractional Ki not lost */
    uint32_t r = 0;
    unsigned ramp_cycles = 0;
    while (r < (BUCK_SETPOINT_DEFAULT_ADC << 16)) {
        r += BUCK_REFERENCE_STEP_Q16;
        if (r > (BUCK_SETPOINT_DEFAULT_ADC << 16)) r = BUCK_SETPOINT_DEFAULT_ADC << 16;
        ++ramp_cycles;
    }
    assert(ramp_cycles <= BUCK_SOFTSTART_CYCLES);
    assert(ramp_cycles >= BUCK_SOFTSTART_CYCLES - 2U);
    /* Exhaustive legal duty range: hardware-deadtime waveform never overlaps
     * and both on-times meet the configured minimum. */
    for (unsigned duty = BUCK_DUTY_MIN; duty <= BUCK_DUTY_MAX; ++duty) {
        assert(duty - BUCK_DT_FINE_TICKS >= BUCK_MIN_GATE_TICKS);
        assert(BUCK_HRTIM_PERIOD - duty - BUCK_DT_FINE_TICKS >= BUCK_MIN_GATE_TICKS);
    }
    puts("PASS: 2M wide-oracle/UBSan PI cases, rail recovery, fractional integration, ramp, PWM bounds");
}
