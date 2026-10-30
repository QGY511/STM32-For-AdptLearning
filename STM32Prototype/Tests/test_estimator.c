/* Validates the fixed-point, divide-free estimator (Inc/buck_estimator.h)
 * against a double-precision oracle of TPEL Sec. III-A eqs 9-15, 7-8, 20-21.
 * The fixed path takes its Pade coefficients from the bin-centre lookup table
 * (8-tick bins); the oracle uses the exact T_on, so any gap is table
 * quantisation + Q-format rounding error. */
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "buck_estimator.h"

static uint32_t seed = 0x600474u;
static uint32_t rnd(void){ seed ^= seed<<13; seed ^= seed>>17; seed ^= seed<<5; return seed; }

static double a_exact(double tau){ double x=(BUCK_R_OHM/BUCK_L_H)*tau; return (2.0-x)/(2.0+x); }
static double b_exact(double tau){ double x=(BUCK_R_OHM/BUCK_L_H)*tau; return 2.0*tau/(BUCK_L_H*(2.0+x)); }

int main(void)
{
    const double ttick = BUCK_TICK_S;
    const double vcount = BUCK_UV_PER_COUNT * 1e-6;      /* V per ADC count */
    const double Csw = BUCK_C_F * BUCK_SWITCH_HZ;        /* C / T_sw, A/V */
    const double alpha = BUCK_EST_ALPHA_Q31 / 2147483648.0;

    buck_est_t s;
    double o_iv = 0.0, o_iload = 0.0, o_vprev;
    buck_est_lut_init();
    /* Table sanity: monotone a/b in T_on, exact at a bin centre. */
    for (unsigned k = 1; k < BUCK_EST_LUT_N; ++k) {
        assert(buck_est_lut[k].a_on  < buck_est_lut[k-1].a_on);   /* longer T_on -> smaller a_on */
        assert(buck_est_lut[k].b_on  > buck_est_lut[k-1].b_on);
        assert(buck_est_lut[k].a_off > buck_est_lut[k-1].a_off);  /* shorter T_off -> larger a_off */
        assert(buck_est_lut[k].b_off < buck_est_lut[k-1].b_off);
    }
    {
        const double tc = (BUCK_DUTY_MIN + 8*10 + 4 - BUCK_DT_FINE_TICKS) * ttick;   /* bin centre minus the dead time */
        assert(fabs(buck_est_lut[10].a_on / 2147483648.0 - a_exact(tc)) < 1e-6);
        assert(fabs(buck_est_lut[10].b_on / 16777216.0 - b_exact(tc)) < 1e-5);
    }

    uint32_t adc = 1000;                                 /* ~1 V */
    buck_est_reset(&s, adc);
    o_vprev = adc * vcount;

    double max_err_ip = 0, max_err_iavg = 0, max_err_iload = 0;
    uint32_t duty = 1700;

    for (unsigned n = 0; n < 200000u; ++n) {
        /* Stimulus: ripple + occasional load-step-like vout shifts and duty moves. */
        int ripple = (int)(rnd() % 7) - 3;               /* +-3 counts */
        if (n % 4096u == 0u) adc = 900 + (rnd() % 200);  /* 0.9..1.1 V load steps */
        if (n % 512u == 0u)  duty = 1400u + (rnd() % 1200u); /* ~15..29% operating band */
        uint32_t adc_s = (uint32_t)((int)adc + ripple);
        if (adc_s > 4095u) adc_s = 4095u;

        /* Fixed-point step. */
        buck_est_out_t eo;
        buck_estimate(&s, adc_s, duty, &eo);

        /* Oracle (double), same recursion, exact coefficients. */
        double vout = adc_s * vcount;
        double ton = ((double)duty - BUCK_DT_FINE_TICKS) * ttick, toff = (double)(BUCK_HRTIM_PERIOD - duty + BUCK_DT_FINE_TICKS) * ttick;
        double aon = a_exact(ton), bon = b_exact(ton);
        double aoff = a_exact(toff), boff = b_exact(toff);
        double ip = aon * o_iv + bon * (BUCK_VIN_V - vout);
        double iv_next = aoff * ip - boff * vout;
        double D = (double)duty / BUCK_HRTIM_PERIOD;
        double iavg = 0.5 * (ip + D * o_iv + (1.0 - D) * iv_next);
        double dv = vout - o_vprev;
        double iload_raw = iavg - Csw * dv;
        double iload = o_iload + alpha * (iload_raw - o_iload);

        double f_ip = eo.ip_q20 / 1048576.0, f_iavg = eo.iavg_q20 / 1048576.0, f_iload = eo.iload_q20 / 1048576.0;
        if (n > 100u) { /* skip initial transient of the recursion */
            double e1 = fabs(f_ip - ip), e2 = fabs(f_iavg - iavg), e3 = fabs(f_iload - iload);
            if (e1 > max_err_ip) max_err_ip = e1;
            if (e2 > max_err_iavg) max_err_iavg = e2;
            if (e3 > max_err_iload) max_err_iload = e3;
        }
        o_iv = iv_next; o_iload = iload; o_vprev = vout;

        assert(eo.ip_q20 > -(1<<30) && eo.ip_q20 < (1<<30));     /* no overflow */
        assert(eo.iload_q20 > -(1<<30) && eo.iload_q20 < (1<<30));
    }
    printf("estimator max |err| vs double oracle: ip=%.4f A  iavg=%.4f A  iload=%.4f A\n",
           max_err_ip, max_err_iavg, max_err_iload);
    assert(max_err_ip < 0.10);
    assert(max_err_iavg < 0.10);
    assert(max_err_iload < 0.10);
    puts("PASS: fixed-point estimator matches double oracle within 100 mA over 200k cycles (torture stimulus)");
    return 0;
}
