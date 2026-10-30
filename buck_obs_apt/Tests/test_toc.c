/* Host tests of the feed-forward TOC (stage 4, 30 Oct 2026): Inc/buck_toc.h (the code the firmware runs)
 * and the frame's sequence of Inc/buck_toc_start.h / Src/buck_toc.c on a switched plant.
 *
 * 1. The L-C solve: on the lossless L-C model the pair lands on (I_eq - h, V_ref), checked by the model's
 *    own end map (buck_toc_model_end) and by an independent piecewise integration in double, over random
 *    initial states of both orders; the late first span (buck_toc_t2_for) still lands the current.
 * 2. The Jacobian step (buck_toc_learn): on an L-C plant whose slopes and capacitance differ from the
 *    model's, repeated pulses from the same state drive both residuals to zero; a clamped first span
 *    moves only the second span's gain and nulls the current residual.
 * 3. Closed loop: the switched synchronous buck of Tests/test_ctrl.c (L, r_L, C, dead time with body
 *    diodes, the 650 kHz current-sense filter, 12-bit ADCs, the firmware's sampling instants), the
 *    firmware's per-cycle controller (Inc/buck_ctrl.h) and the TOC frame with the firmware's timing: the
 *    start at the counter the linked code reaches (README "Clock budget"), the first span at least as
 *    long as its solve and guard, the frame's samples, the first period at the held duty, the PIs
 *    restarted from the first period's measured mean current. Load steps with the TOC off and on, and
 *    the learning over repeated steps.
 *
 * Build and run: make toc-test */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "buck_ctrl.h"
#include "buck_toc.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)
static double urand(void) { return rand() / (double)RAND_MAX; }

/* ---- 1. the solve ----------------------------------------------------------------------------- */
/* the pair on an L-C plant with slopes q1, q2 (A/s) and C, in double: end current - I_eq and end v */
static void lc_end(double a, double v0, double q1, double q2, double C, double t1, double t2, double *zi, double *v)
{
    const double i1 = a + q1 * t1;                              /* (i - I_eq) after the first span */
    const double Q = (a + i1) / 2.0 * t1 + (i1 + (i1 + q2 * t2)) / 2.0 * t2;
    *zi = i1 + q2 * t2;
    *v = v0 + Q / C;
}

static void test_solve(void)
{
    buck_toc_k_t k;
    buck_toc_k_init(&k, (float)(BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT));
    const double T = BUCK_TSW_S, tick = BUCK_TICK_S, C = BUCK_C_F, vref = k.vref;
    const double m1 = (BUCK_VIN_V - vref) / BUCK_L_H, m2 = vref / BUCK_L_H;
    const double h = 0.5 * (BUCK_VIN_V - vref) * (vref / BUCK_VIN_V) * T / BUCK_L_H;
    double w_model = 0, w_lc_i = 0, w_lc_v = 0, w_late = 0;
    long n_ok[2] = { 0, 0 }, n_none[2] = { 0, 0 };
    srand(11);
    for (int n = 0; n < 400000; ++n) {
        const uint32_t order = (uint32_t)(n & 1);
        /* a load step of 0.2..8 A seen through one cycle's dv, a droop / hump of 2..120 mV */
        const double step = 0.2 + 7.8 * urand(), dvm = 0.002 + 0.118 * urand();
        const double ieq = 0.5 + 6.0 * urand();
        const double a = (order == BUCK_TOC_ON_OFF) ? -step * urand() : step * urand();
        const double v = (order == BUCK_TOC_ON_OFF) ? vref - dvm : vref + dvm;
        const double im = ieq + a;
        const float v_prev = (float)(v - a * T / C);           /* so that (C/T)(v - v_prev) = a */
        buck_toc_sol_t s;
        const int32_t ok = buck_toc_solve(&k, order, (float)v, v_prev, (float)im, &s);
        if (!ok) { ++n_none[order]; continue; }
        ++n_ok[order];
        const double t1 = s.t1 * tick, t2 = s.t2 * tick;
        float zi, zv;
        buck_toc_model_end(&k, order, s.a, s.dv, (float)t1, (float)t2, &zi, &zv);
        w_model = fmax(w_model, fmax(fabs(zi) / 1.0, fabs(zv) / 1e-3));
        const double q1 = (order == BUCK_TOC_ON_OFF) ? m1 : -m2, q2 = (order == BUCK_TOC_ON_OFF) ? -m2 : m1;
        double zi_d, v_d;
        lc_end(s.a, v, q1, q2, C, t1, t2, &zi_d, &v_d);
        w_lc_i = fmax(w_lc_i, fabs(zi_d + h));                 /* lands on the valley I_eq - h */
        w_lc_v = fmax(w_lc_v, fabs(v_d - vref));
        /* a first span longer than solved (late): the second lands the current all the same */
        const double t1l = t1 * (1.0 + urand());
        const double t2l = buck_toc_t2_for(&k, order, s.a, (float)(t1l / tick)) * tick;
        if (t2l > 0.0) {
            lc_end(s.a, v, q1, q2, C, t1l, t2l, &zi_d, &v_d);
            w_late = fmax(w_late, fabs(zi_d + h));
        }
    }
    printf("solve: ON-OFF %ld pairs (%ld none), OFF-ON %ld (%ld none); model end map within %.2e (of 1 A / 1 mV); "
           "L-C in double: end current %.2e A from I_eq - h, end voltage %.2e V from V_ref; late first span: %.2e A\n",
           n_ok[0], n_none[0], n_ok[1], n_none[1], w_model, w_lc_i, w_lc_v, w_late);
    CHECK(n_ok[0] > 150000 && n_ok[1] > 150000, "solve: pairs found");
    CHECK(w_model < 2e-2 && w_lc_i < 2e-3 && w_lc_v < 3e-5 && w_late < 3e-3, "solve lands on the target");
    /* the held duty: (V_ref + r_L I_eq)/V_in, clamped */
    CHECK(fabs(buck_toc_hold_duty(0.8f, 3.0f) - (0.8 + BUCK_RL_OHM * 3.0) / BUCK_VIN_V) < 1e-6 &&
          buck_toc_hold_duty(0.8f, 1e3f) == BUCK_D_MAX && buck_toc_hold_duty(0.8f, NAN) == BUCK_D_MIN, "held duty");
}

/* ---- 2. the Jacobian step ------------------------------------------------------------------------ */
static void test_learn(void)
{
    buck_toc_k_t k;
    buck_toc_k_init(&k, (float)(BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT));
    const double tick = BUCK_TICK_S, vref = k.vref, h = k.h;
    /* plants: slope and capacitance errors (a steeper fall, as the board's series loss makes it) */
    const struct { double f1, f2, fc; const char *name; } pl[] = {
        { 1.0, 1.0, 1.0, "matched" }, { 1.25, 1.0, 1.0, "L -20 %" }, { 1.0, 1.6, 1.0, "fall 1.6x steeper" },
        { 0.9, 1.3, 1.3, "rise -10 %, fall +30 %, C +30 %" },
    };
    for (unsigned p = 0; p < sizeof pl / sizeof pl[0]; ++p) {
        for (uint32_t order = 0; order < 2; ++order) {
            const double m1 = k.m1 * pl[p].f1, m2 = k.m2 * pl[p].f2, C = BUCK_C_F * pl[p].fc;
            const double q1 = (order == BUCK_TOC_ON_OFF) ? m1 : -m2, q2 = (order == BUCK_TOC_ON_OFF) ? -m2 : m1;
            const double a = (order == BUCK_TOC_ON_OFF) ? -2.0 : 2.0, dvm = 0.02;
            const double v = (order == BUCK_TOC_ON_OFF) ? vref - dvm : vref + dvm, ieq = 2.0;
            float g1 = 1.0f, g2 = 1.0f;
            double z0 = 0, zl = 0;
            int steps = 0;
            for (int it = 0; it < 40; ++it) {
                buck_toc_sol_t s;
                (void)buck_toc_solve(&k, order, (float)v, (float)(v - a * BUCK_TSW_S / BUCK_C_F), (float)(ieq + a), &s);
                const double t1 = s.t1 * g1 * tick, t2 = s.t2 * g2 * tick;
                double zi, ve;
                lc_end(a, v, q1, q2, C, t1, t2, &zi, &ve);
                const double res_i = zi + h, res_v = ve - vref;   /* the measured residuals */
                const double z = fabs(res_i) / 0.1 + fabs(res_v) / 1e-3;
                if (it == 0) z0 = z;
                zl = z;
                if (buck_toc_learn(&k, order, (float)a, (float)t1, (float)t2, 0U, 0U, (float)res_i, (float)res_v, &g1, &g2))
                    ++steps;
            }
            printf("learn %-32s %s: residual %.3g -> %.3g (of 0.1 A + 1 mV) in 40 pulses, gains %.3f / %.3f\n",
                   pl[p].name, order ? "OFF-ON" : "ON-OFF", z0, zl, (double)g1, (double)g2);
            CHECK(steps == 40 && (zl < 0.05 || zl < 1e-3 * z0), "learn %s order %u converges", pl[p].name, (unsigned)order);
        }
    }
    /* a clamped first span: only the second gain moves, and the current residual goes to 0 */
    {
        const double m1 = k.m1, m2 = k.m2 * 1.5, C = BUCK_C_F;
        float g1 = 1.0f, g2 = 1.0f;
        double res_i = 0;
        for (int it = 0; it < 40; ++it) {
            buck_toc_sol_t s;
            (void)buck_toc_solve(&k, BUCK_TOC_ON_OFF, (float)(vref - 0.02), (float)(vref - 0.02 + 2.0 * BUCK_TSW_S / C),
                                 0.0f, &s);
            const double t1 = s.t1 * tick * 1.3, t2 = buck_toc_t2_for(&k, BUCK_TOC_ON_OFF, s.a, (float)(s.t1 * 1.3)) * g2 * tick;
            double zi, ve;
            lc_end(s.a, vref - 0.02, m1, -m2, C, t1, t2, &zi, &ve);
            res_i = zi + k.h;
            (void)buck_toc_learn(&k, BUCK_TOC_ON_OFF, s.a, (float)t1, (float)t2, 1U, 0U, (float)res_i, (float)(ve - vref),
                                 &g1, &g2);
        }
        printf("learn, first span clamped (fall 1.5x): g1 %.3f (kept), g2 %.3f, current residual %.2e A\n", (double)g1,
               (double)g2, res_i);
        CHECK(g1 == 1.0f && fabs(res_i) < 0.02, "clamped first span: g2 alone nulls the current residual");
    }
    /* the gates: a residual beyond SKIP or not finite teaches nothing */
    {
        float g1 = 1.0f, g2 = 1.0f;
        const uint32_t r1 = buck_toc_learn(&k, 0U, -2.0f, 4e-7f, 2e-6f, 0U, 0U, 7.0f, 0.0f, &g1, &g2);
        const uint32_t r2 = buck_toc_learn(&k, 0U, -2.0f, 4e-7f, 2e-6f, 0U, 0U, 0.0f, NAN, &g1, &g2);
        CHECK(r1 == 0U && r2 == 0U && g1 == 1.0f && g2 == 1.0f, "learn gates");
        CHECK(buck_toc_gain_ok(NAN) == 1.0f && buck_toc_gain_ok(7.0f) == 1.0f && buck_toc_gain_ok(1.3f) == 1.3f, "gain check");
    }
}

/* ---- 3. closed loop ----------------------------------------------------------------------------- */
/* The switched plant of Tests/test_ctrl.c. */
typedef struct {
    double L, rl, C, esr, vin, vd;
    double i, vc, y;
    double R, istep;
    int noise;
} plant_t;

static double p_vout(const plant_t *p) { return (p->vc + p->esr * (p->i - p->istep)) / (1.0 + p->esr / p->R); }

/* sw: 1 high side, 0 low side, -1 dead time (body diode) */
static void p_step(plant_t *p, int sw, double dt, double *int_il)
{
    const int n = (int)ceil(dt / 4e-9);
    if (n <= 0) return;
    const double hh = dt / n, ws = 2 * M_PI * 650e3;
    for (int k = 0; k < n; ++k) {
        double vsw;
        if (sw == 1) vsw = p->vin;
        else if (sw == 0) vsw = 0.0;
        else if (p->i > 0) vsw = -p->vd;
        else if (p->i < 0) vsw = p->vin + p->vd;
        else vsw = NAN;
        double x[3] = { p->i, p->vc, p->y }, kk[4][3];
        for (int st = 0; st < 4; ++st) {
            const double c = (st == 0) ? 0 : (st == 3 ? 1.0 : 0.5);
            double xi[3];
            for (int j = 0; j < 3; ++j) xi[j] = x[j] + (st ? c * hh * kk[st - 1][j] : 0);
            plant_t q = *p; q.i = xi[0]; q.vc = xi[1];
            const double vo = p_vout(&q), io = vo / p->R + p->istep;
            kk[st][0] = isnan(vsw) ? 0.0 : (vsw - p->rl * xi[0] - vo) / p->L;
            kk[st][1] = (xi[0] - io) / p->C;
            kk[st][2] = ws * (xi[0] - xi[2]);
        }
        const double i0 = p->i;
        for (int j = 0; j < 3; ++j) x[j] += hh / 6 * (kk[0][j] + 2 * kk[1][j] + 2 * kk[2][j] + kk[3][j]);
        if (sw < 0 && ((i0 > 0 && x[0] < 0) || (i0 < 0 && x[0] > 0))) x[0] = 0.0;
        p->i = x[0]; p->vc = x[1]; p->y = x[2];
        if (int_il) *int_il += 0.5 * (i0 + p->i) * hh;
    }
}

static uint32_t adc12(double volts, int noise)
{
    double c = volts / BUCK_V_PER_COUNT;
    if (noise) c += (rand() % (2 * noise + 1)) - noise;
    c = floor(c + 0.5);
    return (uint32_t)(c < 0 ? 0 : c > 4095 ? 4095 : c);
}

/* One stretch of the timer [0, per) ticks: the output edges and the sample instants, in ticks. */
typedef struct { uint32_t t; int sw; } edge_t;
typedef struct {
    edge_t e[8]; int ne;                /* the switch state from e[j].t on (ascending; e[0].t = 0) */
    uint32_t ti[4], tv[2];              /* ADC2 and ADC1 trigger instants */
    uint32_t per;
} span_plan_t;

typedef struct { uint32_t vsum, isum; double imax, imin; } span_res_t;

static void run_span(plant_t *p, const span_plan_t *sp, span_res_t *r)
{
    const double tick = BUCK_TICK_S, lat = 45e-9;
    r->vsum = r->isum = 0; r->imax = -1e9; r->imin = 1e9;
    double t = 0;
    int e = 0, ki = 0, kv = 0;
    const double tend = sp->per * tick;
    while (t < tend - 1e-15) {
        double next = (e + 1 < sp->ne) ? sp->e[e + 1].t * tick : tend;
        if (ki < 4 && sp->ti[ki] * tick + lat < next) next = sp->ti[ki] * tick + lat;
        if (kv < 2 && sp->tv[kv] * tick + lat < next) next = sp->tv[kv] * tick + lat;
        p_step(p, sp->e[e].sw, next - t, NULL);
        t = next;
        r->imax = fmax(r->imax, p->i); r->imin = fmin(r->imin, p->i);
        if (ki < 4 && fabs(t - (sp->ti[ki] * tick + lat)) < 1e-15) { r->isum += adc12(BUCK_ISENSE_V_PER_A * p->y, p->noise); ++ki; }
        if (kv < 2 && fabs(t - (sp->tv[kv] * tick + lat)) < 1e-15) { r->vsum += adc12(p_vout(p), p->noise); ++kv; }
        if (e + 1 < sp->ne && fabs(t - sp->e[e + 1].t * tick) < 1e-15) ++e;
    }
}

/* a normal period with CMP1 = cmp; hs_at_start: the high side is on across the boundary (no dead time) */
static void plan_normal(span_plan_t *sp, uint32_t cmp, int hs_at_start)
{
    sp->ne = 0;
    if (!hs_at_start) { sp->e[sp->ne++] = (edge_t){ 0, -1 }; sp->e[sp->ne++] = (edge_t){ BUCK_DT_TICKS, 1 }; }
    else sp->e[sp->ne++] = (edge_t){ 0, 1 };
    sp->e[sp->ne++] = (edge_t){ cmp, -1 };
    sp->e[sp->ne++] = (edge_t){ cmp + BUCK_DT_TICKS, 0 };
    for (int j = 0; j < 4; ++j) sp->ti[j] = BUCK_ISAMPLE_CMP((uint32_t)j);
    sp->tv[0] = BUCK_VSAMPLE0_CMP; sp->tv[1] = BUCK_VSAMPLE_CMP;
    sp->per = BUCK_HRTIM_PERIOD;
}

/* Firmware timing of the start (README "Clock budget": the linked code, static count): the SST/SRT at
 * this counter value, and the solve's counter read this many ticks after it (the first span's end then
 * lies at least BUCK_TOC_SWU_GUARD_TICKS beyond that). */
#define SIM_C_S_TICKS     3350U
#define SIM_SOLVE_TICKS   1888U

typedef struct {
    double t_up, t_down, t_end, i_base_R, i_step, L_plant, rl_plant;
    int noise, toc, learn, cycles;      /* cycles: up/down step pairs (period 2 x (t_down - t_up)) */
    const char *name;
} tscen_t;

typedef struct {
    double dip, hump, imax, imin;
    long frames, late, none;
    float g[2][2];
    double zv_first, zv_last, zi_last;
    double zi_first_o[2], zi_last_o[2], zv_last_o[2];   /* by order: the first and the last frame's residuals */
} tres_t;

static void run_loop_sim(const tscen_t *sc, tres_t *res)
{
    const double T = BUCK_TSW_S;
    plant_t p = { .L = sc->L_plant, .rl = sc->rl_plant, .C = BUCK_C_F, .esr = 0.0, .vin = BUCK_VIN_V, .vd = 0.7,
                  .R = sc->i_base_R, .noise = sc->noise };
    buck_tune_t tune = BUCK_TUNE_DEFAULTS;
    tune.score_a = 2500.0f;                     /* S = (e_v / 20 mV)^2, S_th = 1 */
    const float ki_t_v = buck_pi_ki_t(tune.ki_v), ki_t_i = buck_pi_ki_t(tune.ki_i);
    buck_state_t s; memset(&s, 0, sizeof s);
    buck_obs_t obs = { 0, 0 };
    const buck_obs_k_t ok = BUCK_OBS_K_DEFAULTS;
    buck_toc_k_t k;
    const float vref = (float)(BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT);
    buck_toc_k_init(&k, vref);
    float g[2][2] = { { 1.0f, 1.0f }, { 1.0f, 1.0f } };
    memset(res, 0, sizeof *res);
    res->imin = 1e9; res->imax = -1e9; res->zv_first = NAN;
    uint32_t cmp_active = BUCK_CMP_MIN, cmp_next = BUCK_CMP_MIN;
    int running = 0, hs_at_start = 0;
    uint32_t vsum = 0, isum = 0;                /* the samples of the period just run */
    uint32_t trig = 0, holdoff = 0;
    float v_prev = 0.0f;
    /* the episode: 0 none, 1 = the next pass reads the frame's end (T), 2 = it reads after P1 (1) */
    int ep = 0;
    uint32_t ep_order = 0, ep_sat = 0, ep_ok = 0, ep_t1 = 0, ep_t2 = 0, hold_cmp = 0;
    float ep_a = 0, ep_dhold = 0, ep_g1 = 1, ep_g2 = 1, v_end = 0, ep_ieq = 0;
    double t = 0.0;
    const double period2 = 2.0 * (sc->t_down - sc->t_up);
    while (t < sc->t_end) {
        /* the load: steps up at t_up (+ n period2), down at t_down (+ n period2), sc->cycles times */
        double ph = t - sc->t_up;
        int cyc = (ph >= 0) ? (int)(ph / period2) : -1;
        double in_cyc = (ph >= 0) ? ph - cyc * period2 : -1;
        p.istep = (cyc >= 0 && cyc < sc->cycles && in_cyc < sc->t_down - sc->t_up) ? sc->i_step : 0.0;
        /* ---- the pass at the start of this period (the samples of the previous one) */
        span_plan_t sp;
        int frame = 0;
        if (!running) {
            if (t >= 1e-3) {
                cmp_next = buck_ctrl_arm(&tune, &s, vsum, isum, &(buck_out_t){0});
                obs = (buck_obs_t){ (float)vsum * (float)BUCK_V_PER_VSUM, buck_adc_to_i(&tune, isum) };
                v_prev = obs.vh;
                running = 1;
            }
        } else if (ep == 1) {                   /* T: the frame's end; P2 at the held duty too */
            v_end = (float)vsum * (float)BUCK_V_PER_VSUM;
            cmp_next = hold_cmp;
            obs.vh = v_end;
            ep = 2;
        } else if (ep == 2) {                   /* 1: the PIs restart from the first period */
            const float i1 = buck_adc_to_i(&tune, isum);
            s.integ_v = fminf(fmaxf(i1, BUCK_IREF_MIN_A), BUCK_IREF_MAX_A);
            s.integ_i = ep_dhold;
            buck_step_t stp; buck_out_t o;
            cmp_next = buck_ctrl_fast_kt(&tune, ki_t_v, ki_t_i, &s, vsum, isum, &stp);
            const uint32_t gg = buck_ctrl_slow(&tune, &s, &stp, &o);
            (void)buck_obs_step(&obs, stp.v, stp.im, ok);
            const float zv = v_end - vref, zi = k.cdt * (stp.v - v_end);
            if (getenv("TOC_TRACE"))
                printf("    frame: order %u sat %u ok %u t1 %u t2 %u a %.2f I_eq %.2f v_end %.4f v_1 %.4f i1 %.2f "
                       "z_v %.1f mV z_i %.2f A g %.3f/%.3f\n", (unsigned)ep_order, (unsigned)ep_sat, (unsigned)ep_ok,
                       (unsigned)ep_t1, (unsigned)ep_t2, (double)ep_a, (double)(ep_ieq), (double)v_end, (double)stp.v,
                       (double)i1, (double)zv * 1e3, (double)zi, (double)ep_g1, (double)ep_g2);
            if (isnan(res->zv_first)) { res->zv_first = zv; res->zi_first_o[0] = res->zi_first_o[1] = NAN; }
            if (isnan(res->zi_first_o[ep_order])) res->zi_first_o[ep_order] = zi;
            res->zi_last_o[ep_order] = zi; res->zv_last_o[ep_order] = zv;
            res->zv_last = zv; res->zi_last = zi;
            if (sc->learn && ep_ok) {
                const uint32_t sat1 = (ep_sat & 3U) != 0U, sat2 = (ep_sat & 12U) != 0U;
                float g1 = ep_g1, g2 = ep_g2;
                if (buck_toc_learn(&k, ep_order, ep_a, (float)ep_t1 * (float)BUCK_TICK_S, (float)ep_t2 * (float)BUCK_TICK_S,
                                   sat1, sat2, zi, zv, &g1, &g2)) {
                    if (ep_order == 0) { g[0][0] = g1; g[0][1] = g2; } else { g[1][1] = g1; g[1][0] = g2; }
                }
            }
            v_prev = stp.v;
            trig = 0; holdoff = BUCK_TOC_HOLDOFF_PASSES;
            (void)gg;
            ep = 0;
        } else {
            const float v = (float)vsum * (float)BUCK_V_PER_VSUM;
            const float S = buck_trigger_score_k(v - s.vref, buck_score_k(&tune));
            if (sc->toc && trig && s.vref == vref && S >= tune.s_th) {
                /* ---- the start (Inc/buck_toc_start.h, with the firmware's timing) */
                const float im = buck_adc_to_i(&tune, isum);
                const uint32_t order = (v < vref) ? BUCK_TOC_ON_OFF : BUCK_TOC_OFF_ON, on_off = order == BUCK_TOC_ON_OFF;
                const float g1 = on_off ? g[0][0] : g[1][1], g2 = on_off ? g[0][1] : g[1][0];
                const uint32_t lim = on_off ? (uint32_t)fminf(fmaxf((BUCK_TOC_IPK_MAX_A - im) * k.t_m1, 0.0f), (float)BUCK_TOC_TON_MAX_TICKS)
                                            : (uint32_t)fminf(fmaxf((im - BUCK_TOC_IVAL_MIN_A) * k.t_m2, 0.0f), (float)BUCK_TOC_TOFF_MAX_TICKS);
                const uint32_t c_s = SIM_C_S_TICKS;
                buck_toc_sol_t sol;
                buck_toc_solve1(&k, order, v, v_prev, im, &sol);
                uint32_t t1 = (uint32_t)fmaxf(sol.t1 * g1 + 0.5f, 0.0f), sat = 0;
                if (!(sol.t1 * g1 > 0.0f)) t1 = 0;
                if (t1 > lim) { t1 = lim; sat = BUCK_TOC_SAT_LIMIT1; }
                const uint32_t cn = c_s + SIM_SOLVE_TICKS;
                uint32_t cmp2 = on_off ? c_s + BUCK_DT_TICKS + t1 : c_s + t1 - BUCK_DT_TICKS;
                if (cmp2 < cn + BUCK_TOC_SWU_GUARD_TICKS) { cmp2 = cn + BUCK_TOC_SWU_GUARD_TICKS; sat = BUCK_TOC_SAT_LATE1; }
                const int32_t okk = buck_toc_ok(&k, order, &sol);
                buck_toc_solve2(&k, order, &sol);
                const uint32_t t1a = on_off ? cmp2 - c_s - BUCK_DT_TICKS : cmp2 + BUCK_DT_TICKS - c_s;
                if (!okk) sat |= BUCK_TOC_SAT_LATE1;
                float t2f = (sat & 3U) ? buck_toc_t2_for(&k, order, sol.a, (float)t1a) * g2 : sol.t2 * g2;
                uint32_t t2 = (t2f > 0.0f) ? (uint32_t)(t2f + 0.5f) : 0U;
                const uint32_t lo = on_off ? BUCK_TOC_OFF_MIN_TICKS : BUCK_TOC_ON_MIN_TICKS;
                const uint32_t hi = on_off ? BUCK_TOC_TOFF_MAX_TICKS : BUCK_TOC_TON_MAX_TICKS;
                if (t2 < lo) { t2 = lo; sat |= BUCK_TOC_SAT_LIMIT2; }
                if (t2 > hi) { t2 = hi; sat |= BUCK_TOC_SAT_LIMIT2; }
                uint32_t per = on_off ? cmp2 + t2 - BUCK_DT_TICKS : cmp2 + BUCK_DT_TICKS + t2;
                const uint32_t plo = BUCK_TOC_SWU2_FIRST + BUCK_TOC_A0_LEAD_TICKS + BUCK_TOC_SWU_GUARD_TICKS;
                if (per < plo) { per = plo; sat |= BUCK_TOC_SAT_PER; }
                /* the frame: the period's own on-time, then the first span from c_s, the second, the end */
                sp.ne = 0;
                sp.e[sp.ne++] = (edge_t){ 0, hs_at_start ? 1 : -1 };
                if (!hs_at_start) sp.e[sp.ne++] = (edge_t){ BUCK_DT_TICKS, 1 };
                if (cmp_active < c_s) { sp.e[sp.ne++] = (edge_t){ cmp_active, -1 }; sp.e[sp.ne++] = (edge_t){ cmp_active + BUCK_DT_TICKS, 0 }; }
                if (on_off) {
                    sp.e[sp.ne++] = (edge_t){ c_s, -1 };                 /* SST: low side off, high side after DT */
                    sp.e[sp.ne++] = (edge_t){ c_s + BUCK_DT_TICKS, 1 };
                    sp.e[sp.ne++] = (edge_t){ cmp2, -1 };
                    sp.e[sp.ne++] = (edge_t){ cmp2 + BUCK_DT_TICKS, 0 };
                } else {
                    if (cmp_active >= c_s) { sp.e[sp.ne++] = (edge_t){ c_s, -1 }; sp.e[sp.ne++] = (edge_t){ c_s + BUCK_DT_TICKS, 0 }; }
                    sp.e[sp.ne++] = (edge_t){ cmp2, -1 };
                    sp.e[sp.ne++] = (edge_t){ cmp2 + BUCK_DT_TICKS, 1 };
                }
                for (int j = 0; j < 4; ++j) sp.ti[j] = BUCK_ISAMPLE_CMP((uint32_t)j);
                sp.tv[0] = per - BUCK_TOC_A0_LEAD_TICKS; sp.tv[1] = per - BUCK_VSAMPLE_LEAD_TICKS;   /* the frame's end pair */
                sp.per = per;
                frame = 1;
                ep_dhold = buck_toc_hold_duty(vref, sol.ieq);
                hold_cmp = buck_duty_to_cmp(ep_dhold);
                cmp_next = on_off ? hold_cmp : hold_cmp - BUCK_DT_TICKS;      /* P1 (the return words) */
                ep_order = order; ep_sat = sat; ep_ok = (uint32_t)okk; ep_t1 = t1a; ep_t2 = t2; ep_a = sol.a; ep_ieq = sol.ieq;
                ep_g1 = g1; ep_g2 = g2;
                (void)buck_obs_step(&obs, v, im, ok);
                v_prev = v;
                trig = 0;
                ep = 1;
                ++res->frames;
                if (sat & BUCK_TOC_SAT_LATE1) ++res->late;
                if (!okk) ++res->none;
            } else {
                buck_step_t stp; buck_out_t o;
                cmp_next = buck_ctrl_fast_kt(&tune, ki_t_v, ki_t_i, &s, vsum, isum, &stp);
                const uint32_t gg = buck_ctrl_slow(&tune, &s, &stp, &o);
                (void)buck_obs_step(&obs, stp.v, stp.im, ok);
                if (holdoff) --holdoff;
                trig = (gg ^ 1U) & (holdoff == 0U);
                v_prev = stp.v;
                if (s.vref != vref) s.vref = buck_ramp_step(buck_ramp_mode(s.vref, vref), s.vref, vref, BUCK_SOFTSTART_STEP_V);
            }
        }
        /* ---- this period (or the frame) */
        if (!frame) plan_normal(&sp, running ? cmp_active : 0U, hs_at_start);
        if (!running) { sp.ne = 1; sp.e[0] = (edge_t){ 0, -1 }; }
        span_res_t r;
        run_span(&p, &sp, &r);
        vsum = r.vsum; isum = r.isum;
        if (t > 2e-3) { res->imax = fmax(res->imax, r.imax); res->imin = fmin(res->imin, r.imin); }
        const double v = p_vout(&p);
        if (t > sc->t_up && t < sc->t_up + 0.4e-3) res->dip = fmax(res->dip, vref - v);
        if (t > sc->t_down && t < sc->t_down + 0.4e-3) res->hump = fmax(res->hump, v - vref);
        hs_at_start = frame && ep_order == BUCK_TOC_OFF_ON;
        cmp_active = cmp_next;
        t += sp.per * BUCK_TICK_S;
        (void)T;
    }
    memcpy(res->g, g, sizeof g);
}

static void test_closed_loop(void)
{
    const double R1 = BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT;   /* 1 A at V_ref */
    const struct { tscen_t s; } sc[] = {
        { { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 3 -> 1 A, PI alone" } },
        { { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 3 -> 1 A, TOC" } },
        { { 13e-3, 15e-3, 17e-3, R1, 5.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 6 -> 1 A, PI alone" } },
        { { 13e-3, 15e-3, 17e-3, R1, 5.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 6 -> 1 A, TOC" } },
        { { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 2, 1, 0, 1, "1 -> 3 -> 1 A, TOC, ADC noise +-2" } },
    };
    tres_t r[sizeof sc / sizeof sc[0]];
    for (unsigned i = 0; i < sizeof sc / sizeof sc[0]; ++i) {
        srand(21);
        run_loop_sim(&sc[i].s, &r[i]);
        printf("[%s] dip %.1f mV, hump %.1f mV; i_L %.2f .. %.2f A; frames %ld (first span late %ld, no pair %ld)\n",
               sc[i].s.name, r[i].dip * 1e3, r[i].hump * 1e3, r[i].imin, r[i].imax, r[i].frames, r[i].late, r[i].none);
    }
    CHECK(r[0].frames == 0 && r[2].frames == 0, "no frame with the TOC off");
    CHECK(r[1].frames == 2 && r[3].frames == 2, "one frame per step with the TOC on");
    CHECK(r[1].dip < 0.6 * r[0].dip && r[1].hump < 0.6 * r[0].hump, "TOC cuts the 2 A dip and hump");
    CHECK(r[3].dip < 0.6 * r[2].dip && r[3].hump < 0.6 * r[2].hump, "TOC cuts the 5 A dip and hump");
    CHECK(r[1].imax < BUCK_TOC_IPK_MAX_A + 1.0 && r[3].imax < BUCK_TOC_IPK_MAX_A + 1.0, "peak current within the limit");
    CHECK(r[4].frames >= 2 && r[4].dip < 0.7 * r[0].dip, "TOC with ADC noise");
    /* learning over repeated steps on the board-like plant and on a mismatched one */
    const tscen_t lr[] = {
        { 12e-3, 12.6e-3, 12e-3 + 12 * 1.2e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 12, "learning, 12 x (1 -> 3 -> 1 A)" },
        { 12e-3, 12.6e-3, 12e-3 + 12 * 1.2e-3, R1, 2.0, 0.8 * BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 12, "learning, plant L -20 %" },
    };
    for (unsigned i = 0; i < sizeof lr / sizeof lr[0]; ++i) {
        tres_t q, q0;
        tscen_t nolearn = lr[i]; nolearn.learn = 0;
        srand(21); run_loop_sim(&nolearn, &q0);
        srand(21); run_loop_sim(&lr[i], &q);
        printf("[%s] frames %ld; gains ON-OFF %.3f / %.3f, OFF-ON %.3f / %.3f; ON-OFF z_i %.2f -> %.2f A (z_v %.1f mV), "
               "OFF-ON z_i %.2f -> %.2f A (z_v %.1f mV); dip / hump of the first step %.1f / %.1f mV\n", lr[i].name, q.frames,
               (double)q.g[0][0], (double)q.g[0][1], (double)q.g[1][0], (double)q.g[1][1], q.zi_first_o[0], q.zi_last_o[0],
               q.zv_last_o[0] * 1e3, q.zi_first_o[1], q.zi_last_o[1], q.zv_last_o[1] * 1e3, q.dip * 1e3, q.hump * 1e3);
        (void)q0;
        CHECK(q.frames == 24, "%s: one frame per step", lr[i].name);
        /* ON-OFF: the first span is the minimum (the solve's time) on these steps, so the second span's gain
         * takes the current residual (the fall is steeper than the lossless model's); OFF-ON's first span is
         * at its valley limit and the second at its minimum: nothing to learn there (README) */
        CHECK(fabs(q.zi_last_o[0]) < 0.35 * fabs(q.zi_first_o[0]) && q.g[0][1] < 0.9f,
              "%s: ON-OFF current residual learned away", lr[i].name);
        CHECK(fabs(q.zv_last_o[0]) < fabs(q.zv_first) && fabs(q.zv_last_o[0]) < 0.025, "%s: ON-OFF voltage residual smaller",
              lr[i].name);
        CHECK(q.g[0][0] >= BUCK_TOC_G_MIN && q.g[0][1] >= BUCK_TOC_G_MIN && q.g[1][0] >= BUCK_TOC_G_MIN &&
              q.g[1][1] >= BUCK_TOC_G_MIN, "%s: gains in range", lr[i].name);
    }
}

int main(void)
{
    test_solve();
    test_learn();
    test_closed_loop();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "all tests passed", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
