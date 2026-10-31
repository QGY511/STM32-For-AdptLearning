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
        const int32_t ok = buck_toc_solve(&k, order, (float)v, v_prev, (float)im, 0.0f, &s);
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
                (void)buck_toc_solve(&k, order, (float)v, (float)(v - a * BUCK_TSW_S / BUCK_C_F), (float)(ieq + a), 0.0f, &s);
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
                                 0.0f, 0.0f, &s);
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
    /* the Jacobian step in two parts (the firmware's split) gives the same gains bit for bit */
    {
        long bad = 0;
        srand(13);
        for (int n = 0; n < 200000; ++n) {
            const uint32_t order = (uint32_t)(n & 1), sat1 = (uint32_t)(rand() % 4 == 0), sat2 = (uint32_t)(rand() % 4 == 0);
            const float a = (float)(-4.0 + 8.0 * urand()), t1 = (float)(1e-7 + 3e-6 * urand()), t2 = (float)(1e-7 + 5e-6 * urand());
            const float zi = (float)(-8.0 + 16.0 * urand()), zv = (float)(-0.25 + 0.5 * urand());
            float g1 = (float)(0.4 + 2.1 * urand()), g2 = (float)(0.4 + 2.1 * urand()), h1 = g1, h2 = g2;
            const uint32_t r = buck_toc_learn(&k, order, a, t1, t2, sat1, sat2, zi, zv, &g1, &g2);
            buck_toc_dt_t d;
            const uint32_t r2 = buck_toc_learn_dt(&k, order, a, t1, t2, sat1, zi, zv, &d);
            if (r2) buck_toc_learn_g(t1, t2, sat1, sat2, &d, &h1, &h2);
            bad += r != r2 || memcmp(&g1, &h1, 4) != 0 || memcmp(&g2, &h2, 4) != 0;
        }
        CHECK(bad == 0, "learn in two parts = learn (%ld differ)", bad);
    }
    /* the equilibrium correction: b converges on a constant bias, is clipped per step and bounded, gates */
    {
        float b = 0.0f;
        for (int it = 0; it < 20; ++it) { const float e = 0.7f - b; (void)buck_toc_learn_b(e, &b); }
        float bc = 0.0f;
        (void)buck_toc_learn_b(5.0f, &bc);                       /* clipped at 2 A: b = 0.5 x 2 */
        float bm = 2.9f;
        (void)buck_toc_learn_b(1.0f, &bm);                       /* bounded at 3 A */
        float bs = 0.3f;
        const uint32_t s1 = buck_toc_learn_b(7.0f, &bs), s2 = buck_toc_learn_b(NAN, &bs);
        printf("equilibrium correction: b -> %.4f A for a 0.7 A bias in 20 frames; one 5 A residual moves it %.2f A; "
               "bounded at %.2f A\n", (double)b, (double)bc, (double)bm);
        CHECK(fabsf(b - 0.7f) < 1e-4f && bc == BUCK_TOC_B_MU * BUCK_TOC_ZI_CLIP_A && bm == BUCK_TOC_B_MAX_A &&
              s1 == 0U && s2 == 0U && bs == 0.3f, "learn b");
        CHECK(buck_toc_b_ok(NAN) == 0.0f && buck_toc_b_ok(4.0f) == 0.0f && buck_toc_b_ok(-1.5f) == -1.5f, "b check");
        /* b moves I_eq one for one */
        buck_toc_sol_t s0, s1b;
        buck_toc_solve1(&k, BUCK_TOC_ON_OFF, (float)(vref - 0.02), (float)(vref - 0.015), 2.0f, 0.0f, &s0);
        buck_toc_solve1(&k, BUCK_TOC_ON_OFF, (float)(vref - 0.02), (float)(vref - 0.015), 2.0f, 0.5f, &s1b);
        CHECK(fabsf(s1b.ieq - s0.ieq - 0.5f) < 1e-5f && fabsf(s1b.a - s0.a + 0.5f) < 1e-5f, "b in the solve: I_eq + b");
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

/* Firmware timing of the start: the SST/SRT at this counter value, the solve's counter read this many
 * ticks after it (the first span's end then lies at least BUCK_TOC_SWU_GUARD_TICKS beyond that), and the
 * counter read after the second update's wait this many ticks after the SST (or at BUCK_TOC_SWU2_FIRST if
 * later; the frame's end at least BUCK_TOC_END_LEAD_TICKS after it). Two sets: the static count of the
 * linked code (README "Clock budget"), and as measured on the board (31 Oct, one frame: g_buck.toc.c_start,
 * PER' - BUCK_TOC_END_LEAD_TICKS; the solve's read is not measured: 62 clocks x 1.36, the factor measured
 * from the SST to the read after the wait). */
typedef struct { uint32_t c_s, solve, wait, held_learn; const char *name; } sim_timing_t;
/* held_learn: the periods the duty stays held after P1's sample while the Jacobian step runs (Src/buck_toc.c,
 * toc_keep; 31 Oct: the TOC code runs no PI pass): none by the static count (the step ends before the next
 * sample), two with the board's ~2 x */
static const sim_timing_t SIM_STATIC = { 3350U, 1984U, 32U * 184U, 0U, "static count" };
static const sim_timing_t SIM_BOARD = { 4352U, 32U * 84U, 8064U, 2U, "board, measured 31 Oct" };
static sim_timing_t sim_tm = { 3350U, 1984U, 32U * 184U, 0U, "static count" };
#define SIM_C_S_TICKS     (sim_tm.c_s)
#define SIM_SOLVE_TICKS   (sim_tm.solve)
#define SIM_WAIT_TICKS    (sim_tm.wait)

typedef struct {
    double t_up, t_down, t_end, i_base_R, i_step, L_plant, rl_plant;
    int noise, toc, learn, cycles;      /* cycles: up/down step pairs (period 2 x (t_down - t_up)) */
    const char *name;
    double slew;                        /* the load's edges ramp over this, s (0: steps) */
    double esr;                         /* output capacitor ESR, ohm */
    int mix;                            /* 1: every other cycle's step is 2.5 x i_step */
} tscen_t;

typedef struct {
    double dip, hump, imax, imin;
    long frames, late, none;
    float g[2][2];
    double zv_first, zv_last, zi_last;
    double zi_first_o[2], zi_last_o[2], zv_last_o[2];   /* by order: the first and the last frame's residuals */
    double eb_first_o[2], eb_last_o[2];                 /* by order: the equilibrium residual I_load - I_eq */
    float b[2];
    double dip_last, hump_last;                         /* of the last cycle */
    long learn_low;
    long per_sat;                                       /* frames whose end is the minimum (SAT_PER) */
} tres_t;

static void run_loop_sim(const tscen_t *sc, tres_t *res)
{
    const double T = BUCK_TSW_S;
    plant_t p = { .L = sc->L_plant, .rl = sc->rl_plant, .C = BUCK_C_F, .esr = sc->esr, .vin = BUCK_VIN_V, .vd = 0.7,
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
    float bq[2] = { 0.0f, 0.0f };
    memset(res, 0, sizeof *res);
    res->imin = 1e9; res->imax = -1e9; res->zv_first = NAN;
    res->eb_first_o[0] = res->eb_first_o[1] = NAN;
    int last_cyc = -1;
    uint32_t cmp_active = BUCK_CMP_MIN, cmp_next = BUCK_CMP_MIN;
    int running = 0, hs_at_start = 0;
    uint32_t vsum = 0, isum = 0;                /* the samples of the period just run */
    uint32_t trig = 0, holdoff = 0;
    float v_prev = 0.0f;
    /* the episode: 0 none, 1 = the next pass reads the frame's end (T), 2 = it reads after P1 (1), 3 = a sample
     * read with the duty held (ep_held more) */
    int ep = 0;
    uint32_t ep_held = 0;
    uint32_t ep_order = 0, ep_tail = 0, ep_sat = 0, ep_ok = 0, ep_t1 = 0, ep_t2 = 0, hold_cmp = 0;
    float ep_a = 0, ep_dhold = 0, ep_g1 = 1, ep_g2 = 1, v_end = 0, ep_ieq = 0;
    double t = 0.0;
    const double period2 = 2.0 * (sc->t_down - sc->t_up);
    while (t < sc->t_end) {
        /* the load: steps up at t_up (+ n period2), down at t_down (+ n period2), sc->cycles times */
        double ph = t - sc->t_up;
        int cyc = (ph >= 0) ? (int)(ph / period2) : -1;
        double in_cyc = (ph >= 0) ? ph - cyc * period2 : -1;
        {
            const double ist = (sc->mix && (cyc & 1)) ? 2.5 * sc->i_step : sc->i_step, hi_len = sc->t_down - sc->t_up;
            double f = 0.0;
            if (cyc >= 0 && cyc < sc->cycles) {
                if (in_cyc < hi_len) f = (sc->slew > 0.0) ? fmin(1.0, in_cyc / sc->slew) : 1.0;
                else f = (sc->slew > 0.0) ? fmax(0.0, 1.0 - (in_cyc - hi_len) / sc->slew) : 0.0;
            }
            p.istep = f * ist;
            if (cyc != last_cyc && cyc >= 0 && cyc < sc->cycles) { last_cyc = cyc; res->dip_last = res->hump_last = 0.0; }
        }
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
        } else if (ep == 3) {                   /* a sample while the TOC code still works: the duty held */
            const float v = (float)vsum * (float)BUCK_V_PER_VSUM;
            (void)buck_obs_step(&obs, v, buck_adc_to_i(&tune, isum), ok);
            v_prev = v;
            cmp_next = hold_cmp;
            if (--ep_held == 0U) ep = 0;
        } else if (ep == 2) {                   /* 1: the PIs' integrators from the first period, the duty held */
            const float i1 = buck_adc_to_i(&tune, isum);
            s.integ_v = fminf(fmaxf(i1, BUCK_IREF_MIN_A), BUCK_IREF_MAX_A);
            s.integ_i = ep_dhold;
            buck_step_t stp;
            stp.v = (float)vsum * (float)BUCK_V_PER_VSUM;
            cmp_next = hold_cmp;
            (void)buck_obs_step(&obs, stp.v, i1, ok);
            /* the firmware's residuals (Src/buck_toc.c): the landing and the end voltage for the timing, the load
             * over P1 against I_eq for b */
            const float zv = v_end - vref, zi = i1 - ep_ieq;
            const float i_load = i1 - k.cdt * (stp.v - v_end), e_b = i_load - ep_ieq;
            if (isnan(res->eb_first_o[ep_order])) res->eb_first_o[ep_order] = e_b;
            res->eb_last_o[ep_order] = e_b;
            const int low = !(i1 >= BUCK_TOC_LEARN_IMIN_A);
            res->learn_low += sc->learn && low;
            if (sc->learn && !low) (void)buck_toc_learn_b(e_b, &bq[ep_order]);
            if (getenv("TOC_TRACE"))
                printf("    frame: order %u sat %u ok %u t1 %u t2 %u a %.2f I_eq %.2f v_end %.4f v_1 %.4f i1 %.2f "
                       "z_v %.1f mV z_i %.2f A g %.3f/%.3f\n", (unsigned)ep_order, (unsigned)ep_sat, (unsigned)ep_ok,
                       (unsigned)ep_t1, (unsigned)ep_t2, (double)ep_a, (double)(ep_ieq), (double)v_end, (double)stp.v,
                       (double)i1, (double)zv * 1e3, (double)zi, (double)ep_g1, (double)ep_g2);
            if (isnan(res->zv_first)) { res->zv_first = zv; res->zi_first_o[0] = res->zi_first_o[1] = NAN; }
            if (isnan(res->zi_first_o[ep_order])) res->zi_first_o[ep_order] = zi;
            res->zi_last_o[ep_order] = zi; res->zv_last_o[ep_order] = zv;
            res->zv_last = zv; res->zi_last = zi;
            if (sc->learn && ep_ok && !low) {
                const uint32_t sat1 = (ep_sat & 3U) != 0U, sat2 = (ep_sat & 12U) != 0U;
                float g1 = ep_g1, g2 = ep_g2;
                if (buck_toc_learn(&k, ep_order, ep_a, (float)ep_t1 * (float)BUCK_TICK_S, (float)ep_t2 * (float)BUCK_TICK_S,
                                   sat1, sat2, zi, zv, &g1, &g2)) {
                    if (ep_order == 0) { g[0][0] = g1; g[0][1] = g2; } else { g[1][1] = g1; g[1][0] = g2; }
                }
            }
            v_prev = stp.v;
            trig = 0; holdoff = BUCK_TOC_HOLDOFF_PASSES;
            ep_held = (sc->learn && ep_ok && !low) ? sim_tm.held_learn : 0U;
            ep = ep_held ? 3 : 0;
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
                buck_toc_solve1(&k, order, v, v_prev, im, bq[order], &sol);
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
                const uint32_t c2 = (c_s + SIM_WAIT_TICKS > BUCK_TOC_SWU2_FIRST) ? c_s + SIM_WAIT_TICKS : BUCK_TOC_SWU2_FIRST;
                const uint32_t plo = (c2 + BUCK_TOC_END_LEAD_TICKS > BUCK_TOC_PER_MIN_TICKS) ? c2 + BUCK_TOC_END_LEAD_TICKS
                                                                                            : BUCK_TOC_PER_MIN_TICKS;
                uint32_t t_end = 0U;            /* OFF-ON stretched: the ON span's end (Timer A CMP1), else 0 */
                if (per < plo) {
                    per = plo; sat |= BUCK_TOC_SAT_PER;
                    if (!on_off) {
                        /* the solved end by the compare if far enough ahead, else ended by SRT at the read
                         * (~10 clocks later on the board); never later than either */
                        t_end = cmp2 + BUCK_DT_TICKS + t2;
                        sat |= BUCK_TOC_SAT_TAIL;
                        if (t_end < c2 + BUCK_TOC_TAIL_LEAD_TICKS) { t_end = c2 + 32U * 10U; sat |= BUCK_TOC_SAT_TAIL_NOW; }
                        t2 = (t_end > cmp2 + BUCK_DT_TICKS) ? t_end - cmp2 - BUCK_DT_TICKS : 0U;
                    }
                }
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
                    if (t_end) {                /* the tail: low side on to the frame's end */
                        sp.e[sp.ne++] = (edge_t){ t_end, -1 };
                        sp.e[sp.ne++] = (edge_t){ t_end + BUCK_DT_TICKS, 0 };
                    }
                }
                for (int j = 0; j < 4; ++j) sp.ti[j] = BUCK_ISAMPLE_CMP((uint32_t)j);
                sp.tv[0] = per - BUCK_TOC_A0_LEAD_TICKS; sp.tv[1] = per - BUCK_VSAMPLE_LEAD_TICKS;   /* the frame's end pair */
                sp.per = per;
                frame = 1;
                ep_dhold = buck_toc_hold_duty(vref, sol.ieq);
                hold_cmp = buck_duty_to_cmp(ep_dhold);
                cmp_next = (on_off || t_end) ? hold_cmp : hold_cmp - BUCK_DT_TICKS;      /* P1 (the return words) */
                ep_tail = t_end != 0U;
                ep_order = order; ep_sat = sat; ep_ok = (uint32_t)okk; ep_t1 = t1a; ep_t2 = t2; ep_a = sol.a; ep_ieq = sol.ieq;
                ep_g1 = g1; ep_g2 = g2;
                (void)buck_obs_step(&obs, v, im, ok);
                v_prev = v;
                trig = 0;
                ep = 1;
                ++res->frames;
                if (sat & BUCK_TOC_SAT_LATE1) ++res->late;
                if (sat & BUCK_TOC_SAT_PER) ++res->per_sat;
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
        if (cyc == sc->cycles - 1 && in_cyc >= 0) {
            const double hi_len = sc->t_down - sc->t_up;
            if (in_cyc < 0.4e-3) res->dip_last = fmax(res->dip_last, vref - v);
            if (in_cyc >= hi_len && in_cyc < hi_len + 0.4e-3) res->hump_last = fmax(res->hump_last, v - vref);
        }
        hs_at_start = frame && ep_order == BUCK_TOC_OFF_ON && !ep_tail;
        cmp_active = cmp_next;
        t += sp.per * BUCK_TICK_S;
        (void)T;
    }
    memcpy(res->g, g, sizeof g);
    memcpy(res->b, bq, sizeof bq);
}

static void test_closed_loop_board(void);

static void test_closed_loop(void)
{
    const double R1 = BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT;   /* 1 A at V_ref */
    sim_tm = SIM_STATIC;
    printf("closed loop, the start's timing: %s\n", sim_tm.name);
    const struct { tscen_t s; } sc[] = {
        { { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 3 -> 1 A, PI alone", 0, 0, 0 } },
        { { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 3 -> 1 A, TOC", 0, 0, 0 } },
        { { 13e-3, 15e-3, 17e-3, R1, 5.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 6 -> 1 A, PI alone", 0, 0, 0 } },
        { { 13e-3, 15e-3, 17e-3, R1, 5.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 6 -> 1 A, TOC", 0, 0, 0 } },
        { { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 2, 1, 0, 1, "1 -> 3 -> 1 A, TOC, ADC noise +-2", 0, 0, 0 } },
        { { 13e-3, 15e-3, 17e-3, R1, 1.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 2 -> 1 A, PI alone", 0, 0, 0 } },
        { { 13e-3, 15e-3, 17e-3, R1, 1.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 2 -> 1 A, TOC", 0, 0, 0 } },
    };
    tres_t r[sizeof sc / sizeof sc[0]];
    for (unsigned i = 0; i < sizeof sc / sizeof sc[0]; ++i) {
        srand(21);
        run_loop_sim(&sc[i].s, &r[i]);
        printf("[%s] dip %.1f mV, hump %.1f mV; i_L %.2f .. %.2f A; frames %ld (first span late %ld, no pair %ld, "
               "end at its minimum %ld)\n", sc[i].s.name, r[i].dip * 1e3, r[i].hump * 1e3, r[i].imin, r[i].imax,
               r[i].frames, r[i].late, r[i].none, r[i].per_sat);
    }
    CHECK(r[0].frames == 0 && r[2].frames == 0, "no frame with the TOC off");
    CHECK(r[1].frames == 2 && r[3].frames == 2, "one frame per step with the TOC on");
    /* the 2 A step's solved on-time is below the minimum first span (LATE): the second span then removes the
     * extra current by the lossless model, which the plant's steeper fall overdoes until g_off is learned */
    CHECK(r[1].dip < 0.65 * r[0].dip && r[1].hump < 0.6 * r[0].hump, "TOC cuts the 2 A dip and hump");
    CHECK(r[3].dip < 0.6 * r[2].dip && r[3].hump < 0.6 * r[2].hump, "TOC cuts the 5 A dip and hump");
    CHECK(r[1].imax < BUCK_TOC_IPK_MAX_A + 1.0 && r[3].imax < BUCK_TOC_IPK_MAX_A + 1.0, "peak current within the limit");
    CHECK(r[4].frames >= 2 && r[4].dip < 0.7 * r[0].dip, "TOC with ADC noise");
    /* the 1 A down step's frame ends at its minimum (the valley-limited OFF span is long, the ON span short):
     * the ON span ends as solved, the rest with the low side on (31 Oct; with the high side on to the moved
     * end the hump was 248 mV, 3.5 x the PI's) */
    CHECK(r[6].frames == 2 && r[6].per_sat >= 1, "the 1 A step's OFF-ON frame ends at its minimum");
    CHECK(r[6].hump < 0.7 * r[5].hump && r[6].dip < 1.05 * r[5].dip, "TOC on the 1 A step: the tail keeps the hump down");
    /* learning over repeated steps: the board-like plant, a mismatched one, a slewing load (the edge comes
     * mid-slew: the edge estimate is short of the final load, b's case), mixed step sizes, ESR */
    const tscen_t lr[] = {
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, 16 x (1 -> 3 -> 1 A)", 0, 0, 0 },
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 2.0, 0.8 * BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, plant L -20 %", 0, 0, 0 },
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, 10 us slewing edges", 10e-6, 0, 0 },
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, 2 A / 5 A alternating", 0, 0, 1 },
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, 5 mOhm ESR", 0, 5e-3, 0 },
    };
    for (unsigned i = 0; i < sizeof lr / sizeof lr[0]; ++i) {
        tres_t q, q0;
        tscen_t nolearn = lr[i]; nolearn.learn = 0;
        srand(21); run_loop_sim(&nolearn, &q0);
        srand(21); run_loop_sim(&lr[i], &q);
        printf("[%s] frames %ld; g ON-OFF %.3f / %.3f, OFF-ON %.3f / %.3f; b %+.2f / %+.2f A; ON-OFF z_i %.2f -> %.2f A, "
               "e_b %.2f -> %.2f A, z_v %.1f mV; OFF-ON z_i %.2f -> %.2f A, e_b %.2f -> %.2f A, z_v %.1f mV; last step dip / hump "
               "%.1f / %.1f mV (no learning %.1f / %.1f mV); %ld frames below LEARN_IMIN\n", lr[i].name, q.frames,
               (double)q.g[0][0], (double)q.g[0][1], (double)q.g[1][0], (double)q.g[1][1], (double)q.b[0], (double)q.b[1],
               q.zi_first_o[0], q.zi_last_o[0], q.eb_first_o[0], q.eb_last_o[0], q.zv_last_o[0] * 1e3, q.zi_first_o[1],
               q.zi_last_o[1], q.eb_first_o[1], q.eb_last_o[1], q.zv_last_o[1] * 1e3, q.dip_last * 1e3, q.hump_last * 1e3,
               q0.dip_last * 1e3, q0.hump_last * 1e3, q.learn_low);
        CHECK(q.frames == 2 * lr[i].cycles, "%s: one frame per step", lr[i].name);
        CHECK(q.g[0][0] >= BUCK_TOC_G_MIN && q.g[0][1] >= BUCK_TOC_G_MIN && q.g[1][0] >= BUCK_TOC_G_MIN &&
              q.g[1][1] >= BUCK_TOC_G_MIN && fabsf(q.b[0]) <= BUCK_TOC_B_MAX_A && fabsf(q.b[1]) <= BUCK_TOC_B_MAX_A,
              "%s: gains and b in range", lr[i].name);
        /* the hump within 10 %: the 2 A down step's OFF-ON frame is valley-limited and (since 31 Oct) ends with
         * the tail, so it has no free span to learn (z_v stays ~+27 mV, README Known limits); b alone moves,
         * and its correct I_eq removes a small bias that had covered part of that shortfall (L -20 %: 47.6 ->
         * 50.6 mV), as with ESR below */
        if (!lr[i].mix && lr[i].esr == 0.0)
            CHECK(q.dip_last <= q0.dip_last * 1.05 && q.hump_last <= q0.hump_last * 1.10, "%s: learning does not make the "
                  "last step worse", lr[i].name);
        /* with ESR the edge cycle's dv carries ESR x the load step, so the unlearned I_eq is ~2 A high and the
         * pulse oversized (which hides the lossless model's charge shortfall); b takes the bias out (its job), the
         * dip then is the model's (README, Known limits) */
        if (lr[i].esr > 0.0)
            CHECK(fabs(q.eb_last_o[0]) < 0.3 * fabs(q.eb_first_o[0]) && fabs(q.eb_last_o[1]) < 0.3 * fabs(q.eb_first_o[1]),
                  "%s: b takes the ESR bias of the edge estimate out", lr[i].name);
        if (i <= 1)
            CHECK(fabs(q.zi_last_o[0]) < 0.35 * fabs(q.zi_first_o[0]) && fabs(q.eb_last_o[0]) < 0.3,
                  "%s: ON-OFF landing learned away, equilibrium residual small", lr[i].name);
        if (i == 2)
            CHECK(q.b[0] > 0.1f && fabs(q.eb_last_o[0]) < 0.5 * fabs(q.eb_first_o[0]),
                  "%s: b takes the slewing edge's shortfall", lr[i].name);
    }
}

/* The same steps with the start's timing measured on the board (31 Oct): the first span's minimum is ~0.64 us
 * (the current to the 10 A limit on every step up, then the second span takes it back), the frame's end is
 * later. Checked: the peak current, the tail on the down steps, learning's effect on the dip; the dips of the
 * unlearned small steps up are reported, not checked (worse than the PI's at 1 A: README Known limits). */
static void test_closed_loop_board(void)
{
    const double R1 = BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT;
    sim_tm = SIM_BOARD;
    printf("closed loop, the start's timing: %s\n", sim_tm.name);
    const tscen_t sc[] = {
        { 13e-3, 15e-3, 17e-3, R1, 1.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 2 -> 1 A, PI alone", 0, 0, 0 },
        { 13e-3, 15e-3, 17e-3, R1, 1.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 2 -> 1 A, TOC", 0, 0, 0 },
        { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 3 -> 1 A, PI alone", 0, 0, 0 },
        { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 3 -> 1 A, TOC", 0, 0, 0 },
        { 13e-3, 15e-3, 17e-3, R1, 5.0, BUCK_L_H, BUCK_RL_OHM, 0, 0, 0, 1, "1 -> 6 -> 1 A, PI alone", 0, 0, 0 },
        { 13e-3, 15e-3, 17e-3, R1, 5.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 0, 1, "1 -> 6 -> 1 A, TOC", 0, 0, 0 },
        { 13e-3, 15e-3, 17e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 2, 1, 0, 1, "1 -> 3 -> 1 A, TOC, ADC noise +-2", 0, 0, 0 },
    };
    tres_t r[sizeof sc / sizeof sc[0]];
    for (unsigned i = 0; i < sizeof sc / sizeof sc[0]; ++i) {
        srand(21);
        run_loop_sim(&sc[i], &r[i]);
        printf("[board: %s] dip %.1f mV, hump %.1f mV; i_L %.2f .. %.2f A; frames %ld (first span late %ld, end at its "
               "minimum %ld)\n", sc[i].name, r[i].dip * 1e3, r[i].hump * 1e3, r[i].imin, r[i].imax, r[i].frames,
               r[i].late, r[i].per_sat);
        if (sc[i].toc)
            CHECK(r[i].frames >= 2 && r[i].imax < BUCK_TOC_IPK_MAX_A + 1.0, "board: %s: frames, peak within the limit",
                  sc[i].name);
    }
    CHECK(r[1].hump < 0.7 * r[0].hump && r[3].hump < 0.7 * r[2].hump && r[5].hump < 0.7 * r[4].hump,
          "board: the down steps' humps below the PI's (the tail)");
    CHECK(r[5].dip < 0.7 * r[4].dip, "board: TOC cuts the 5 A dip");
    const tscen_t lr[] = {
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, 16 x (1 -> 3 -> 1 A)", 0, 0, 0 },
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 1.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, 16 x (1 -> 2 -> 1 A)", 0, 0, 0 },
        { 12e-3, 12.6e-3, 12e-3 + 16 * 1.2e-3, R1, 2.0, BUCK_L_H, BUCK_RL_OHM, 0, 1, 1, 16, "learning, 10 us slewing edges", 10e-6, 0, 0 },
    };
    for (unsigned i = 0; i < sizeof lr / sizeof lr[0]; ++i) {
        tres_t q, q0;
        tscen_t nolearn = lr[i]; nolearn.learn = 0;
        srand(21); run_loop_sim(&nolearn, &q0);
        srand(21); run_loop_sim(&lr[i], &q);
        printf("[board: %s] g ON-OFF %.3f / %.3f; b %+.2f / %+.2f A; last step dip / hump %.1f / %.1f mV (no learning "
               "%.1f / %.1f mV)\n", lr[i].name, (double)q.g[0][0], (double)q.g[0][1], (double)q.b[0], (double)q.b[1],
               q.dip_last * 1e3, q.hump_last * 1e3, q0.dip_last * 1e3, q0.hump_last * 1e3);
        /* the 1 A step up lands P1's mean current below BUCK_TOC_LEARN_IMIN_A (the 10 A peak, then the long fall):
         * the gate keeps it from learning (A1 reads a negative valley as 0); reported, not checked */
        if (lr[i].i_step >= 2.0)
            CHECK(q.dip_last < 0.5 * q0.dip_last, "board: %s: learning cuts the dip", lr[i].name);
        else
            CHECK(q.learn_low > 0, "board: %s: the up step's landing below the learning gate", lr[i].name);
    }
    sim_tm = SIM_STATIC;
}

int main(void)
{
    test_solve();
    test_learn();
    test_closed_loop();
    test_closed_loop_board();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "all tests passed", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
