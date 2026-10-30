/* ISR-side time-optimal pulse solve and its thread-mode adaptation
 * (Inc/buck_optimal.h: buck_isqrt32, buck_so_coef_from_float, buck_opt_solve_i,
 * buck_opt_reseed_i, buck_learn_gamma, buck_learn_ieq).
 *  1. The table square root is within 2 LSB of floor(sqrt) over the whole u32 range.
 *  2. The integer solve reproduces a double reference of the same closed form
 *     (peak excursion, Ton via the trapezoidal inversion, Toff, the gains
 *     gamma_on/gamma_off, clamps, PER floor, terminal current, reseed).
 *  3. Physics: on the ideal LC plant, the unclamped pulse (gamma = 1) returns the
 *     shifted state to (0, 0) -- the charge-balance closed form is correct.
 *  4. The dual correction (2x2 Newton on the MEASURED terminal state for the
 *     timing gains; the settled load for the equilibrium correction b) nulls
 *     the terminal residual and finds the load bias on a plant whose L, C, r
 *     and load differ from the model's. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "buck_estimator.h"
#include "buck_optimal.h"

static const double Vin = BUCK_VIN_V, L = BUCK_L_H, C = BUCK_C_F, r = BUCK_R_OHM;
static const double vpc = BUCK_UV_PER_COUNT * 1e-6, tick = BUCK_TICK_S;
static const double vref = BUCK_SETPOINT_DEFAULT_ADC * (BUCK_UV_PER_COUNT * 1e-6);

/* Double reference of buck_opt_solve_i. Returns the nominal excursion x. */
static double ref_solve(double i0, double inew, double vout_counts, int dv, double gon, double goff,
                        double *ton, double *toff, int *sat, double *ipk, double *term)
{
    const double vout = vout_counts * vpc;
    if (dv < 0) dv = 0; if (dv > 2047) dv = 2047;
    const double a = i0 - inew;
    const double ss = 2.0 * C * (Vin - vref) / L * vpc * dv + a * a;
    const double xmax = BUCK_OPT_IPK_MAX_A - inew;
    *sat = 0; *ton = 0; *toff = 0; *ipk = 0; *term = inew;
    if (xmax <= 0.0) return 0.0;
    const double h = 0.5 * (Vin - vref) * (vref / Vin) * (1.0 / BUCK_SWITCH_HZ) / L;   /* ripple/2 */
    double x0 = sqrt((vref / Vin) * ss); if (x0 > xmax) x0 = xmax;      /* ideal-fall excursion */
    const double v2 = vref + r * (inew + (x0 - h) / 2.0);                  /* fall voltage: V_ref + r i_2 */
    const double x = sqrt((v2 / Vin) * (ss + (Vin - vref) * h * h / vref));   /* h^2 term at the ideal fall voltage, as the ISR */
    double xa = x; if (xa > xmax) { xa = xmax; *sat |= 1; }
    const double vf = vref + r * (inew + (xa - h) / 2.0);
    double rise = gon * (inew + xa - i0);
    if (i0 + rise > BUCK_OPT_IPK_MAX_A) { rise = BUCK_OPT_IPK_MAX_A - i0; *sat |= 1; }
    *ipk = i0 + rise;
    /* trapezoidal inversion, as buck_cm_ton_raw */
    double vl = Vin - vout - r * (i0 + *ipk) / 2.0; if (vl < 0.5) vl = 0.5;
    double di = *ipk - i0; if (di > 16.0) di = 16.0;
    double t = ((di <= 0.0) ? 0.0 : L * di / vl / tick) + BUCK_DT_FINE_TICKS;   /* CMP1 = physical + dead time */
    if (t < BUCK_OPT_TMIN_TICKS) t = BUCK_OPT_TMIN_TICKS;
    else if (t >= BUCK_OPT_TON_MAX_TICKS) { t = BUCK_OPT_TON_MAX_TICKS; *sat |= 2; }
    double f = goff * (xa + h) * L / vf / tick;           /* off-time: gamma_off times the fall down to the steady valley I_eq - h */
    if (f < BUCK_OPT_TMIN_TICKS) f = BUCK_OPT_TMIN_TICKS;
    else if (f > BUCK_OPT_TOFF_MAX_TICKS) f = BUCK_OPT_TOFF_MAX_TICKS;
    if (t + f < BUCK_OPT_PER_MIN_TICKS) f = BUCK_OPT_PER_MIN_TICKS - t;
    *ton = t; *toff = f;
    *term = *ipk - f * tick * vf / L;                     /* what the applied off-time removes */
    return x;
}

/* Plant with the series drop on both ramps, piecewise linear at the average
 * current of each ramp (the trapezoidal model the estimator and the solve use),
 * at the plant's own L_p, C_p and TRUE load. Returns the terminal state. */
static void plant_pulse(double Lp, double Cp, double rp, double Itrue, double i0, double v0, double ton, double toff,
                        double *i_f, double *v_f, double *ipk_out)
{
    const double kk = ton / Lp;
    const double ipk = (i0 + kk * ((Vin - v0) - rp * i0 / 2.0)) / (1.0 + kk * rp / 2.0);      /* ON: V_in - v - r i */
    const double ko = toff / Lp;
    const double i_end = (ipk - ko * (vref + rp * ipk / 2.0)) / (1.0 + ko * rp / 2.0);       /* OFF: -(v + r i) */
    const double q = ((i0 - Itrue) + (ipk - Itrue)) / 2.0 * ton + ((ipk - Itrue) + (i_end - Itrue)) / 2.0 * toff;
    *i_f = i_end; *v_f = v0 + q / Cp; if (ipk_out) *ipk_out = ipk;
}

static void coef(buck_so_coef_t *c, double gon, double goff, double unused)
{
    (void)unused;
    buck_so_coef_from_float((float)BUCK_SETPOINT_DEFAULT_ADC, (float)BUCK_R_MOHM, 0.0f, c);
    c->gon_q16 = (int32_t)(gon * 65536.0 + 0.5); c->goff_q16 = (int32_t)(goff * 65536.0 + 0.5);
}

int main(void)
{
    buck_sqrt_lut_init();
    /* 1. square root */
    uint32_t worst = 0; unsigned n = 0;
    const uint32_t edge[] = {0, 1, 2, 3, 4, 5, 15, 16, 17, 255, 256, 257, 65535, 65536, 65537,
                             0x3FFFFFFF, 0x40000000, 0x40000001, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF};
    for (unsigned i = 0; i < sizeof edge / sizeof edge[0]; ++i) {
        const uint32_t v = edge[i], q = buck_isqrt32(v), e = (uint32_t)floor(sqrt((double)v));
        const uint32_t d = q > e ? q - e : e - q; if (d > worst) worst = d; ++n;
        assert(d <= 2);
    }
    srand(1);
    for (unsigned i = 0; i < 2000000; ++i) {
        const uint32_t v = ((uint32_t)rand() << 17) ^ (uint32_t)rand() ^ ((uint32_t)rand() >> (rand() % 31));
        const uint32_t q = buck_isqrt32(v), e = (uint32_t)floor(sqrt((double)v));
        const uint32_t d = q > e ? q - e : e - q; if (d > worst) worst = d; ++n;
        assert(d <= 2);
    }
    printf("isqrt32: %u values, worst |error| = %u LSB\n", n, worst);
    puts("PASS: table square root within 2 LSB over the u32 range");

    /* 2. integer solve vs double reference of the same closed form, over the gain range */
    const double gains[][2] = {{1.0, 1.0}, {0.5, 0.5}, {2.0, 2.0}, {0.7, 1.4}, {1.5, 0.6}, {1.0, 2.0}};
    double max_ton = 0, max_toff = 0, max_x = 0, max_term = 0; unsigned pts = 0, pulses = 0, sats = 0, sat_mism = 0;
    for (unsigned gi = 0; gi < 6; ++gi) {
        buck_so_coef_t c; coef(&c, gains[gi][0], gains[gi][1], 1.0);
        for (int dv = 0; dv <= 400; dv += 8)
        for (double inew = 0.5; inew <= 11.0; inew += 0.5)
        for (double zi = -8.0; zi <= 1.0; zi += 0.5) {
            const double i0 = inew + zi;
            const uint32_t adc = (uint32_t)(BUCK_SETPOINT_DEFAULT_ADC - dv);
            buck_so_out_t o;
            buck_opt_solve_i(&c, (int32_t)(i0 * 1048576.0), (int32_t)(inew * 1048576.0), buck_v_q28(adc), dv, &o);
            double ton, toff, ipk, term; int sat;
            const double x = ref_solve(i0, inew, (double)adc, dv, gains[gi][0], gains[gi][1], &ton, &toff, &sat, &ipk, &term);
            ++pts;
            if (ton == 0.0) { assert(o.ton == 0); continue; }
            ++pulses; sats += sat != 0;
            if ((o.sat != 0) != (sat != 0)) {
                /* only at an exact clamp boundary: the nominal excursion at the peak
                 * cap (Q8 rounding), the applied peak at the cap, or Ton at TON_MAX */
                assert(fabs(x - (BUCK_OPT_IPK_MAX_A - inew)) < 0.03 || fabs(ipk - BUCK_OPT_IPK_MAX_A) < 0.03
                       || fabs(ton - BUCK_OPT_TON_MAX_TICKS) < 2.0);
                sat_mism++;
            }
            const double eton = fabs((double)o.ton - ton), etoff = fabs((double)o.toff - toff);
            const double ex = fabs(o.ipk_q16 / 65536.0 - ipk), et = fabs(o.term_q20 / 1048576.0 - term);
            if (eton > max_ton) max_ton = eton; if (etoff > max_toff) max_toff = etoff;
            if (ex > max_x) max_x = ex; if (et > max_term) max_term = et;
            /* Q8 currents (4 mA) + Q16 coefficients + tick rounding */
            assert(eton  <= 0.01 * ton  + 12.0);
            assert(etoff <= 0.01 * toff + 16.0);
            assert(ex <= 0.02 + 0.005 * fabs(x));
            assert(et <= 0.03 + 0.005 * fabs(x));
            assert(o.ton >= BUCK_OPT_TMIN_TICKS && o.ton <= BUCK_OPT_TON_MAX_TICKS);
            assert(o.toff >= BUCK_OPT_TMIN_TICKS && o.ton + o.toff >= BUCK_OPT_PER_MIN_TICKS);
            assert(o.ton + o.toff < 65503U);
            /* reseed: the current reference is I_eq itself; the duty is the trigger's
             * on-time plus r/V_in PER per ampere of change */
            int32_t niref; uint32_t nd;
            const double i_trig = 1.0; const uint32_t d_trig = 1500U;
            buck_opt_reseed_i(&c, d_trig, (int32_t)(i_trig * 1048576.0), (int32_t)(inew * 1048576.0), &niref, &nd);
            assert(niref >= BUCK_ACM_IMIN_Q16 && niref <= BUCK_ACM_IMAX_Q16);
            assert(nd >= BUCK_DUTY_MIN && nd <= BUCK_DUTY_MAX);
            if (inew < BUCK_OPT_IPK_MAX_A) assert(fabs(niref / 65536.0 - inew) < 0.001);
            const double nd_want = d_trig + (inew - i_trig) * BUCK_R_OHM / BUCK_VIN_V * BUCK_HRTIM_PERIOD;
            if (nd_want > BUCK_DUTY_MIN + 2 && nd_want < BUCK_DUTY_MAX - 2) assert(fabs((double)nd - nd_want) < 3.0);
        }
    }
    printf("solve: %u points, %u pulses (%u clamped, %u sat-flag boundary cases); max |dTon| %.1f ticks, |dToff| %.1f ticks, |dIpk| %.3f A, |dterm| %.3f A\n",
           pts, pulses, sats, sat_mism, max_ton, max_toff, max_x, max_term);
    puts("PASS: integer ISR solve matches the double closed form (gains, clamps, PER floor, terminal current, reseed)");

    /* 3. physics of the closed form (gamma = 1, unclamped points): on the plant the
     *    model assumes (series drop r on both ramps, trapezoidal average), the
     *    pulse returns the state to (0, 0) up to the i_2 approximation. */
    {
        buck_so_coef_t c; coef(&c, 1.0, 1.0, 1.0);
        double worst_v = 0, worst_i = 0; unsigned k = 0;
        for (int dv = 10; dv <= 120; dv += 10)
        for (double inew = 1.0; inew <= 4.0; inew += 1.0)
        for (double zi = -3.0; zi <= 0.0; zi += 0.5) {
            const double i0 = inew + zi;
            buck_so_out_t o;
            buck_opt_solve_i(&c, (int32_t)(i0 * 1048576.0), (int32_t)(inew * 1048576.0),
                             buck_v_q28((uint32_t)(BUCK_SETPOINT_DEFAULT_ADC - dv)), dv, &o);
            if (o.sat || o.ton == 0 || o.ton + o.toff == BUCK_OPT_PER_MIN_TICKS) continue;   /* clamped or floored: not a pure solution */
            const double ton = (o.ton - BUCK_DT_FINE_TICKS) * tick, toff = o.toff * tick;
            double i_f, v_f, ipk;
            plant_pulse(L, C, r, inew, i0, vref - dv * vpc, ton, toff, &i_f, &v_f, &ipk);
            const double h = 0.5 * (Vin - vref) * (vref / Vin) * (1.0 / BUCK_SWITCH_HZ) / L;
            const double zv_end = v_f - vref, zi_end = i_f - (inew - h);   /* the steady valley, not I_eq */
            if (fabs(zv_end) > worst_v) worst_v = fabs(zv_end);
            if (fabs(zi_end) > worst_i) worst_i = fabs(zi_end);
            assert(fabs(o.ipk_q16 / 65536.0 - ipk) < 0.05);                    /* the peak lands where the inversion says */
            assert(fabs(o.term_q20 / 1048576.0 - i_f) < 0.08);                 /* and so does the terminal current */
            ++k;
        }
        printf("physics: %u unclamped pulses on the r-plant; terminal |z_v| <= %.1f mV, |z_i| <= %.3f A\n", k, worst_v * 1e3, worst_i);
        assert(k > 50);
        assert(worst_v < 3e-3 && worst_i < 0.08);
        puts("PASS: the closed form with the off-ramp drop lands the modelled plant on its periodic steady state within 3 mV / 80 mA");
    }

    /* 4. Dual correction on a MISMATCHED plant. Repeated identical load steps;
     *    each pulse is solved with the model (L, C, r) for the equilibrium
     *    I_hat + b, where I_hat is the estimated load (true load minus a bias),
     *    and applied to a plant with its own L_p, C_p, r_p and the TRUE load.
     *    As the ISR measures it: the terminal current and voltage directly
     *    (z_i,f = valley - target valley, z_v,f), and the settled load
     *    afterwards (e_b = I_true - I_eq). */
    {
        const double mu = BUCK_OPT_GAMMA_MU_Q8 / 256.0, dmax = BUCK_OPT_GAMMA_STEP_Q16 / 65536.0;
        const double gmin = BUCK_OPT_GAMMA_MIN_Q16 / 65536.0, gmax = BUCK_OPT_GAMMA_MAX_Q16 / 65536.0;
        const double mub = BUCK_OPT_IEQ_MU_Q8 / 256.0, bmax = BUCK_OPT_IEQ_MAX_A;
        struct { double Lp, Cp, rp, bias, itrue, i0; int dv; int clamped; } cases[] = {
            { L,        C,        r,        0.0,  3.0, 1.0, 20, 0 },   /* matched: must stay near gamma = 1, b = 0 */
            { 0.8 * L,  1.3 * C,  r,        0.0,  3.0, 1.0, 20, 0 },   /* L 20 % low, C 30 % high */
            { L,        C,        r,        0.6,  3.0, 1.0, 20, 0 },   /* load under-estimated by 0.6 A (slow slew): b must learn it */
            { 1.2 * L,  0.8 * C,  r,       -0.4,  3.6, 1.0, 20, 0 },   /* everything off, load over-estimated */
            { 0.9 * L,  1.1 * C,  0.5 * r,  0.3,  3.3, 2.0, 20, 0 },   /* small droop, half the step in the inductor, r half the model's */
            { L,        C,        0.2 * r,  0.0,  3.0, 1.0, 20, 0 },   /* r a fifth of the model's: fall slower than modelled */
            { 1.2 * L,  0.8 * C,  r,       -0.4,  5.6, 1.0, 90, 1 },   /* peak-clamped pulse: only the current residual can be nulled */
        };
        for (unsigned ci = 0; ci < sizeof cases / sizeof cases[0]; ++ci) {
            const double Lp = cases[ci].Lp, Cp = cases[ci].Cp, rp = cases[ci].rp, Itrue = cases[ci].itrue, ihat = Itrue - cases[ci].bias;
            float gon = 1.0f, goff = 1.0f, bq = 0.0f; double zi_f = 0, zv_f = 0, e_b = 0; int p, converged = -1;
            for (p = 0; p < 24; ++p) {
                buck_so_coef_t c; coef(&c, gon, goff, 1.0);
                const double i0 = cases[ci].i0, v0 = vref - cases[ci].dv * vpc, ieq = ihat + bq;
                const double h = c.h_q8 / 256.0;
                buck_so_out_t o;
                buck_opt_solve_i(&c, (int32_t)(i0 * 1048576.0), (int32_t)(ieq * 1048576.0),
                                 buck_v_q28((uint32_t)(BUCK_SETPOINT_DEFAULT_ADC - cases[ci].dv)), cases[ci].dv, &o);
                assert(o.ton != 0);
                const double ton = (o.ton - BUCK_DT_FINE_TICKS) * tick, toff = o.toff * tick;
                double i_f, v_f;
                plant_pulse(Lp, Cp, rp, Itrue, i0, v0, ton, toff, &i_f, &v_f, NULL);
                zi_f = i_f - (ieq - h);          /* measured terminal current against the target valley */
                zv_f = v_f - vref;               /* measured terminal voltage */
                e_b  = Itrue - ieq;              /* the loop settles at the true load */
                const int clamped = (o.sat & BUCK_SO_SAT_PEAK) != 0;   /* the pulse cannot deliver the charge: only z_i can be nulled */
                if (cases[ci].clamped) assert(clamped);
                if (getenv("DUAL_VERBOSE")) printf("    case %u pulse %2d: gon %.3f goff %.3f b %+.3f ton %u toff %u sat %u ipk %.2f | z_i %+.3f z_v %+.1f mV e_b %+.3f\n",
                                                   ci, p, gon, goff, bq, o.ton, o.toff, o.sat, o.ipk_q16 / 65536.0, zi_f, zv_f * 1e3, e_b);
                if (fabs(zi_f) < 0.06 && (fabs(zv_f) < 3e-3 || clamped) && fabs(e_b) < 0.1) { if (converged < 0) converged = p; if (p >= converged + 2) break; }
                const double m1 = (Vin - vref) / L, i2 = 0.5 * (o.ipk_q16 / 65536.0 + ieq - h), m2 = (vref + r * i2) / L;
                buck_learn_gamma(&gon, &goff, (float)ton, (float)toff, (float)(i0 - ieq),
                                 (float)zi_f, (float)zv_f, (float)m1, (float)m2, (float)mu, o.sat,
                                 (float)BUCK_OPT_GAMMA_DET_MIN_A, (float)dmax, (float)gmin, (float)gmax);
                bq = buck_learn_ieq(bq, (float)e_b, (float)mub, (float)bmax);
                assert(gon >= gmin - 1e-6 && gon <= gmax + 1e-6 && goff >= gmin - 1e-6 && goff <= gmax + 1e-6);
                assert(fabs(bq) <= bmax + 1e-6);
            }
            printf("dual: L %.0f%% C %.0f%% r %.0f%% load bias %+.1f A, step %.1f -> %.1f A, droop %d counts: gamma_on %.3f gamma_off %.3f b %+.3f A, residual z_i %.3f A z_v %.1f mV e_b %.3f A, nulled from pulse %d\n",
                   100.0 * Lp / L, 100.0 * Cp / C, 100.0 * rp / r, cases[ci].bias, cases[ci].i0, Itrue, cases[ci].dv, gon, goff, bq, zi_f, zv_f * 1e3, e_b, converged + 1);
            assert(converged >= 0 && converged <= 20);   /* mu = 0.25 times the plant/model fall-slope ratio per pulse: a 5x r mismatch needs ~18 */
            /* Matched L, C and load: nothing to learn. */
            if (ci == 0) assert(fabs(gon - 1.0) < 0.06 && fabs(goff - 1.0) < 0.06 && fabs(bq) < 0.1);
            if (ci == 2) assert(fabs(bq - cases[ci].bias) < 0.2);    /* the equilibrium correction found the bias */
            if (cases[ci].clamped) assert(gon == 1.0f);              /* a clamped on-time is never adapted */
        }
        puts("PASS: dual correction nulls the measured terminal state on mismatched L, C, r and load within 21 pulses (b finds the load bias; clamped: Toff on the current residual only)");
    }

    return 0;
}
