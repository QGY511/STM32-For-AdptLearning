/* Host tests for Inc/buck_toc_plan.h and Inc/buck_learn.h (the code the
 * firmware runs), against the double-precision originals in
 * Tests/reference_toc.h (the PLECS planner core toc_plan, the span map and
 * endpoint Jacobian tm_*, the G solve tm_gain_step, the q Jacobian
 * ap_phase_voltage_jacobian; copied unchanged from the backup of 29 Oct) and
 * against the formulas of control_pseudocode.txt transcribed in double here.
 *
 * 1. Span map, P_x, P_h, endpoint Jacobian (both orders).
 * 2. Planner: prediction to S0, target construction, order and durations
 *    against toc_plan, the endpoint of the chosen pair, both orders.
 * 3. Frame limits (default H): ticks, PER' of both orders, the limits.
 * 4. The stepped planner against the straight-line one, bit for bit; two
 *    planners interleaved; steps per plan.
 * 5. Learning: scalar step, DeltaI, q (Jq and the update), G (Jacobian at S0,
 *    the update), kappa, the handover clamps, the observer step at D, the
 *    frame prediction.
 * 6. NaN and infinite inputs: never a plan or a coefficient update from them.
 * 7. Stage 2 (29 Oct 2026): the residual verification against the forward maps
 *    (Q11), the frame encoder (Inc/buck_toc_frame.h), the poll hook, the D-pass
 *    helpers (ADC2 model value, deadbeat gain Q5, observer current Q6, OFF-ON
 *    tail valley Q7), and the stage-1 verifier's gaps F05-F07, F09, F12-F14.
 * 8. Stage 2b (30 Oct 2026): the cheaper planner (user decision 1: one order by the
 *    sign of the voltage error, the larger root, the residual check) against the full
 *    planner (the SPEC's shortest feasible pair): how often the full planner chooses
 *    the other order and how much longer/shorter the cheaper plan is then; equality
 *    when both choose the same order; its six parts = the polled form; the frame's
 *    model in parts = stepping; the cheap ADC2 model value (the encoder's split).
 *
 * Build and run: make toc-test */

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "buck_learn.h"
#include "reference_toc.h"

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { if (++failures <= 30) { printf("FAIL: " __VA_ARGS__); printf("\n"); } } } while (0)

static uint32_t rng = 0x73491abcU;
static double rand01(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return (double)rng / 4294967295.0; }
static float randf(double a, double b) { return (float)(a + (b - a) * rand01()); }
static double scaled(double x, double y) { return fabs(x - y) / fmax(1.0, fabs(y)); }

#define L_ BUCK_L_H
#define C_ BUCK_C_F
#define R_ BUCK_RL_OHM
#define VIN_ BUCK_VIN_V
#define T_ BUCK_TSW_S
#include "buck_toc_fw.h"            /* BUCK_TOC_PLAN_PERIODS (the planning periods of the load-step requests) */
#define BUCK_TOC_PLAN_PERIODS_TEST BUCK_TOC_PLAN_PERIODS

static TM_Model tm_model(double ieq) { const TM_Model m = { L_, C_, R_, VIN_, ieq }; return m; }

/* The two spans of an order in double (toc_advance). */
static TM_State ref_pair(TM_State x, double on, double off, int order_pm, double ieq)
{
    double i = x.i, v = x.v;
    if (order_pm > 0) { toc_advance(&i, &v, on, 1, L_, C_, R_, VIN_, ieq); toc_advance(&i, &v, off, 0, L_, C_, R_, VIN_, ieq); }
    else { toc_advance(&i, &v, off, 0, L_, C_, R_, VIN_, ieq); toc_advance(&i, &v, on, 1, L_, C_, R_, VIN_, ieq); }
    const TM_State y = { i, v };
    return y;
}

static int pm(uint32_t order) { return order == BUCK_TOC_ON_OFF ? 1 : -1; }

/* ---- 1. span map ------------------------------------------------------------------ */
static void test_span(void)
{
    double wx = 0, wp = 0, wd = 0, wj = 0;
    for (int k = 0; k < 20000; ++k) {
        const buck_xy_t x = { randf(-5, 20), randf(0.2, 3.5) };
        const float h = randf(0, 12e-6), ieq = randf(-2, 12);
        const uint32_t gate = (uint32_t)(k & 1);
        buck_toc_span_t f;
        buck_toc_span(&f, h);
        const buck_xy_t z = buck_toc_map_f(&f, x, h, gate, ieq);
        const buck_m2_t p = buck_toc_phi(&f);
        const buck_xy_t q = buck_toc_dh(&f, x, z, gate, ieq);
        const TM_Model m = tm_model(ieq);
        TM_State zd, qd; TM_Mat pd;
        CHECK(tm_interval(&m, (TM_State){ x.i, x.v }, h, (int)gate, &zd, &pd, &qd) == TM_OK, "oracle span");
        const double ex = fmax(scaled(z.i, zd.i), scaled(z.v, zd.v));
        const double ep = fmax(fmax(scaled(p.a, pd.m00), scaled(p.b, pd.m01)), fmax(scaled(p.c, pd.m10), scaled(p.d, pd.m11)));
        /* P_h: the backup test_toc.c measure and limit (5e-4, floors 1e4 A/s and 100 V/s) */
        const double ed = fmax(fabs(q.i - qd.i) / fmax(1e4, fabs(qd.i)), fabs(q.v - qd.v) / fmax(100, fabs(qd.v)));
        wx = fmax(wx, ex); wp = fmax(wp, ep); wd = fmax(wd, ed);
        CHECK(ex < 2e-6 && ep < 2e-6 && ed < 5e-4, "span errors %.3g %.3g %.3g", ex, ep, ed);
        /* endpoint and its Jacobian, both orders */
        const uint32_t o = (uint32_t)((k >> 1) & 1);
        const float on = randf(0, 6e-6), off = randf(0, 6e-6);
        buck_m2_t j;
        const buck_xy_t y = buck_toc_endpoint(x, on, off, o, ieq, &j);
        TM_State yd; TM_Mat jd;
        CHECK(tm_endpoint(&m, (TM_State){ x.i, x.v }, on, off, pm(o), &yd, &jd) == TM_OK, "oracle endpoint");
        const double ej = fmax(fmax(fabs(j.a - jd.m00) / fmax(1e5, fabs(jd.m00)), fabs(j.b - jd.m01) / fmax(1e5, fabs(jd.m01))),
                               fmax(fabs(j.c - jd.m10) / fmax(1e3, fabs(jd.m10)), fabs(j.d - jd.m11) / fmax(1e3, fabs(jd.m11))));
        wj = fmax(wj, fmax(ej, fmax(scaled(y.i, yd.i), scaled(y.v, yd.v))));
        CHECK(ej < 5e-5 && scaled(y.i, yd.i) < 4e-6 && scaled(y.v, yd.v) < 4e-6, "endpoint/Jacobian %.3g", ej);
    }
    printf("span map vs double: state %.2g, P_x %.2g, P_h %.2g, endpoint+Jacobian %.2g (scaled)\n", wx, wp, wd, wj);
}

/* ---- request generator --------------------------------------------------------------- */
static buck_toc_req_t rand_req(uint32_t predict)
{
    buck_toc_req_t r;
    memset(&r, 0, sizeof r);
    r.vref = (rand01() < 0.6) ? 0.8057f : randf(0.5, 3.0);
    r.i_r = randf(-1, 12);
    r.v_r = r.vref + randf(-0.3, 0.3);
    r.ton_n = randf(BUCK_D_MIN, BUCK_D_MAX) * (float)T_;
    r.predict = predict;
    r.m_r = randf(0, 10);
    r.raw_before = r.m_r - randf(-3, 3);
    r.delta_i = randf(-0.5, 0.5);
    r.kappa = randf(-0.2, 0.6);
    r.q[0] = randf(-0.3, 0.3); r.q[1] = randf(-0.3, 0.3);
    r.g[0].i = randf(0.8, 1.25); r.g[0].v = randf(0.8, 1.25);
    r.g[1].i = randf(0.8, 1.25); r.g[1].v = randf(0.8, 1.25);
    r.resume_cmp = BUCK_CMP_MIN + (uint32_t)(rand01() * (BUCK_CMP_MAX - BUCK_CMP_MIN));
    return r;
}

/* Runs the stepped planner to the end; returns the number of steps. */
static int run_steps(buck_toc_planner_t *p, const buck_toc_req_t *rq)
{
    memset(p, 0xA5, sizeof *p);
    buck_toc_plan_start(p, rq);
    int n = 0;
    while (buck_toc_plan_step(p) == BUCK_TOC_BUSY && n < 100) ++n;
    return n + 1;
}

/* ---- 2. planner ------------------------------------------------------------------------- */
static void test_planner(void)
{
    int ref_ok = 0, cnt[2] = { 0, 0 }, ready = 0, limit = 0, none = 0, mismatch = 0, already = 0;
    double worst = 0, wend = 0, wpred = 0, wtarget = 0;
    for (int k = 0; k < 60000; ++k) {
        const buck_toc_req_t rq = rand_req((uint32_t)(k & 1));
        buck_toc_planner_t p;
        run_steps(&p, &rq);
        const buck_toc_plan_t *r = &p.r;
        if (r->status == BUCK_TOC_INVALID) { CHECK(0, "planner invalid on finite input %d", k); continue; }
        /* Ieq, the prediction to S0 and the targets in double */
        const double z = (double)rq.m_r - rq.raw_before, ieq = (double)r->ieq;
        CHECK(fabs(ieq - (rq.m_r + rq.delta_i + rq.kappa * z)) <= 2e-6 * (1 + fabs(ieq)), "Ieq_plan");
        TM_State x0d = { rq.i_r, rq.v_r };
        if (rq.predict) x0d = ref_pair(x0d, rq.ton_n, T_ - rq.ton_n, 1, ieq);
        wpred = fmax(wpred, fmax(scaled(r->x0.i, x0d.i), scaled(r->x0.v, x0d.v)));
        CHECK(scaled(r->x0.i, x0d.i) < 4e-6 && scaled(r->x0.v, x0d.v) < 4e-6, "prediction to S0");
        const double vr = (double)r->x0.v + R_ * ieq;
        const double rip = 0.5 * ((VIN_ - vr) * rq.ton_n + vr * (T_ - rq.ton_n)) / L_;
        const double t0 = ieq - 0.5 * rip + rq.q[0], t1 = ieq + 0.5 * rip + rq.q[1];
        wtarget = fmax(wtarget, fmax(fabs(p.target[0] - t0), fabs(p.target[1] - t1)));
        CHECK(fabs(p.target[0] - t0) < 2e-5 && fabs(p.target[1] - t1) < 2e-5, "target construction");
        /* the core against the original double planner, on the same float inputs */
        const TOC_Result d = toc_plan(r->x0.i, r->x0.v, ieq, rq.vref, VIN_, L_, C_, R_, p.target[0], p.target[1]);
        if (r->status == BUCK_TOC_ALREADY) { ++already; CHECK(d.status == TOC_ALREADY_TARGET, "already"); continue; }
        if (d.valid) {
            ++ref_ok;
            if (r->status == BUCK_TOC_NONE) { ++mismatch; CHECK(0, "reference-feasible plan not found %d", k); continue; }
            const double tot = (double)r->ton0 + r->toff0, er = fabs(tot - (d.ton + d.toff)) / fmax(1e-9, d.ton + d.toff);
            worst = fmax(worst, er);
            if (pm(r->order) != d.order || er > 3e-4) {
                ++mismatch;
                CHECK(0, "planner order/time %d/%d %.3g (%d)", pm(r->order), d.order, er, k);
            }
            const TM_State e = ref_pair((TM_State){ r->x0.i, r->x0.v }, r->ton0, r->toff0, pm(r->order), ieq);
            wend = fmax(wend, fmax(scaled(e.i, r->target_i), scaled(e.v, r->vref)));
            CHECK(scaled(e.i, r->target_i) < 2e-5 && scaled(e.v, r->vref) < 2e-5, "endpoint of the plan");
            ++cnt[r->order];
            if (r->status == BUCK_TOC_READY) ++ready; else if (r->status == BUCK_TOC_LIMIT) ++limit;
        } else {
            CHECK(r->status == BUCK_TOC_NONE, "float plan where the reference has none (%d)", k);
            ++none;
        }
    }
    CHECK(ref_ok > 30000 && cnt[0] > 3000 && cnt[1] > 3000 && ready > 3000, "coverage %d %d %d %d", ref_ok, cnt[0], cnt[1], ready);
    CHECK(mismatch == 0, "planner mismatches %d", mismatch);
    printf("planner vs toc_plan (double): %d feasible (ON-OFF %d, OFF-ON %d; within limits %d, outside %d), %d none, %d already;\n"
           "  mismatches %d, worst total-duration error %.2g, endpoint %.2g; prediction %.2g, targets %.2g A\n",
           ref_ok, cnt[0], cnt[1], ready, limit, none, already, mismatch, worst, wend, wpred, wtarget);
}

/* ---- 3. frame limits ------------------------------------------------------------------------ */
static void test_limits(void)
{
    int checked = 0, lim_span = 0, lim_per = 0;
    for (int k = 0; k < 60000; ++k) {
        const buck_toc_req_t rq = rand_req(1U);
        buck_toc_planner_t p;
        run_steps(&p, &rq);
        const buck_toc_plan_t *r = &p.r;
        if (r->status != BUCK_TOC_READY && r->status != BUCK_TOC_LIMIT) continue;
        const double fon = (double)r->g.i * r->ton0 / BUCK_TICK_S, foff = (double)r->g.v * r->toff0 / BUCK_TICK_S;
        if (fon > BUCK_TOC_PER_MAX_TICKS || foff > BUCK_TOC_PER_MAX_TICKS) { CHECK(r->status == BUCK_TOC_LIMIT, "huge span"); ++lim_per; continue; }
        CHECK(fabs((double)r->on_ticks - fon) <= 0.5 + 1e-2 && fabs((double)r->off_ticks - foff) <= 0.5 + 1e-2, "tick rounding");
        CHECK(r->ton_x == (float)r->on_ticks * (float)BUCK_TICK_S && r->toff_x == (float)r->off_ticks * (float)BUCK_TICK_S, "executed spans");
        const uint32_t per = (r->order == BUCK_TOC_ON_OFF) ? r->on_ticks + r->off_ticks
            : BUCK_DT_TICKS + r->off_ticks + r->on_ticks + BUCK_HRTIM_PERIOD - rq.resume_cmp;
        buck_toc_frame_words_t fw;
        if (r->on_ticks >= BUCK_TOC_ON_MIN_TICKS && r->off_ticks >= BUCK_TOC_OFF_MIN_TICKS)
            CHECK(r->per_ticks == per, "PER'");             /* (the encoder stops before PER' at a span minimum) */
        const int in = buck_toc_frame_encode(r->order, r->on_ticks, r->off_ticks, rq.resume_cmp, &fw) == BUCK_TOC_FRAME_OK;
        CHECK(in == (r->status == BUCK_TOC_READY), "limit decision");
        if (in) CHECK(memcmp(&fw, &r->fw, sizeof fw) == 0, "frame words of the plan");
        if (!in) { if (per > BUCK_TOC_PER_MAX_TICKS) ++lim_per; else ++lim_span; }
        ++checked;
    }
    CHECK(lim_span > 100 && lim_per > 100, "both limits exercised: %d %d", lim_span, lim_per);
    buck_toc_req_t rq = rand_req(0U);
    buck_toc_planner_t p;
    for (int tries = 0;; ++tries) {                      /* a request that plans, then a bad resume CMP1 */
        rq = rand_req(0U);
        run_steps(&p, &rq);
        if (p.r.status == BUCK_TOC_READY) break;
        if (tries > 100000) { CHECK(0, "no READY plan in 100000 requests"); return; }
    }
    rq.resume_cmp = BUCK_CMP_MAX + 1U; run_steps(&p, &rq);
    CHECK(p.r.status == BUCK_TOC_INVALID, "resume CMP outside the PI range");
    printf("frame limits: %d plans checked (ticks, PER' of both orders, the encoder's decision and words); outside: %d short span or ADC placement, %d long frame\n",
           checked, lim_span, lim_per);
}

/* ---- 4. stepped = straight-line ------------------------------------------------------------- */
static void test_chunks(void)
{
    int maxsteps = 0, diff = 0;
    long hist[32] = { 0 };
    for (int k = 0; k < 60000; ++k) {
        const buck_toc_req_t rq = rand_req((uint32_t)(k % 3 != 0));
        buck_toc_planner_t p;
        const int n = run_steps(&p, &rq);
        buck_toc_plan_t d;
        memset(&d, 0xA5, sizeof d);
        buck_toc_plan_direct(&rq, &d);
        if (memcmp(&d, &p.r, sizeof d) != 0) { ++diff; CHECK(0, "stepped != direct (%d)", k); }
        maxsteps = n > maxsteps ? n : maxsteps;
        ++hist[n < 31 ? n : 31];
    }
    /* two planners interleaved, step by step */
    int idiff = 0;
    for (int k = 0; k < 5000; ++k) {
        const buck_toc_req_t ra = rand_req(1U), rb = rand_req(0U);
        buck_toc_planner_t a, b, a1, b1;
        run_steps(&a1, &ra); run_steps(&b1, &rb);
        memset(&a, 0xA5, sizeof a); memset(&b, 0xA5, sizeof b);   /* as run_steps: unset fields compare equal */
        buck_toc_plan_start(&a, &ra); buck_toc_plan_start(&b, &rb);
        int ba = 1, bb = 1;
        while (ba || bb) {
            if (ba) ba = buck_toc_plan_step(&a) == BUCK_TOC_BUSY;
            if (bb) bb = buck_toc_plan_step(&b) == BUCK_TOC_BUSY;
        }
        idiff += memcmp(&a.r, &a1.r, sizeof a.r) != 0 || memcmp(&b.r, &b1.r, sizeof b.r) != 0;
        CHECK(buck_toc_plan_step(&a) == a.r.status, "a finished planner returns its status");
    }
    CHECK(diff == 0 && idiff == 0, "chunking changed results: %d, interleaved %d", diff, idiff);
    /* buck_toc_plan_run() with a clock that advances 7..70 per read, deadlines
     * every 60..400 clocks (several calls per plan): the same plan */
    int rdiff = 0, calls_max = 0;
    for (int k = 0; k < 20000; ++k) {
        const buck_toc_req_t rq = rand_req((uint32_t)(k & 1));
        buck_toc_planner_t ref, p;
        run_steps(&ref, &rq);
        memset(&p, 0xA5, sizeof p);
        buck_toc_plan_start(&p, &rq);
        static volatile uint32_t clk;
        int calls = 0;
        int32_t st;
        do {
            clk = (uint32_t)(rand01() * 4e9);
            const uint32_t deadline = clk + 60U + (uint32_t)(rand01() * 340);
            st = buck_toc_plan_run(&p, &clk, deadline);   /* clk advanced below by the "work" */
            clk += 7U + (uint32_t)(rand01() * 63);
            ++calls;
        } while (st == BUCK_TOC_BUSY && calls < 1000);
        rdiff += memcmp(&p.r, &ref.r, sizeof p.r) != 0;
        calls_max = calls > calls_max ? calls : calls_max;
    }
    CHECK(rdiff == 0, "run() != step(): %d", rdiff);
    /* Equal-length candidates (F06): the first in the candidate order is kept. Injected at SEL
     * (the candidates are complete there; they were verified at PUSH). */
    int ties = 0;
    for (int k = 0; k < 2000; ++k) {
        const buck_toc_req_t rq = rand_req(1U);
        buck_toc_planner_t p;
        memset(&p, 0xA5, sizeof p);
        buck_toc_plan_start(&p, &rq);
        while (p.stage != BUCK_TOC_ST_SEL) buck_toc_plan_step(&p);
        if (p.c.n == 0U) continue;
        buck_toc_cand_t *c = &p.c;
        c->n = 2U; c->u[1] = c->u[0]; c->w[1] = c->w[0]; c->o[1] = c->o[0] ^ 1U;
        while (buck_toc_plan_step(&p) == BUCK_TOC_BUSY) {}
        CHECK(p.r.order == c->o[0] || p.r.status == BUCK_TOC_INVALID, "tie: the first candidate");
        ++ties;
    }
    const int skipped = ties;
    printf("stepped vs straight-line planner: %d differences in 60000 (bitwise), %d interleaved, %d run() (up to %d calls),\n"
           "  %d ties kept the first candidate; steps per plan max %d (", diff, idiff, rdiff, calls_max, skipped, maxsteps);
    for (int n = 0; n < 32; ++n) if (hist[n]) printf(" %d:%ld", n, hist[n]);
    printf(" )\n");
}

/* ---- a READY plan of the given order --------------------------------------------------------- */
static buck_toc_plan_t ready_plan(uint32_t order)
{
    buck_toc_planner_t p;
    for (int tries = 0; tries < 100000; ++tries) {      /* bounded (stage-1 verifier F07) */
        buck_toc_req_t rq = rand_req(1U);
        rq.vref = 0.8057f;
        run_steps(&p, &rq);
        if (p.r.status == BUCK_TOC_READY && p.r.order == order) return p.r;
    }
    CHECK(0, "no READY plan of order %u in 100000 requests", (unsigned)order);
    exit(1);
}

static double ref_scalar(double J, double res, double eta, double lambda, double sigma, double cap)
{
    const double j = J / sigma, r = res / sigma, s = eta * j * r / (j * j + lambda);
    return fmax(-cap, fmin(cap, s));
}

/* OFF-ON q at S = B (BUCK_LEARN_QPEAK_AT_B, stage2 S2-Q3) in double: ap_phase_voltage_jacobian's
 * construction (reference_toc.h) with the post-D spans tail (off), on1 (on), per1 - on1 (off). */
static int ref_q_offon_at_b(const AP_Model *m, const buck_toc_plan_t *ep, double iE, double vE, double tail,
                            double per1, double on1, double *Jv)
{
    AP_State initial = { ep->x0.i, ep->x0.v };
    AP_Mat j;
    int rc = ap_endpoint_j(m, initial, ep->ton0, ep->toff0, -1, &j);
    if (rc < 0) return rc;
    const double det = j.a * j.d - j.b * j.c;
    const double st_on = (ep->ton_x / ep->ton0) * (j.d / det), st_off = (ep->toff_x / ep->toff0) * (-j.c / det);
    AP_Tan x = { 0 };
    x.i = ep->x0.i; x.v = ep->x0.v;
    if ((rc = ap_span(m, &x, ep->toff_x, 0, st_off)) < 0) return rc;
    if ((rc = ap_span(m, &x, ep->ton_x, 1, st_on)) < 0) return rc;
    x.i = iE; x.v = vE; x.q = 0; x.sq = 0;
    if ((rc = ap_span(m, &x, tail, 0, 0)) < 0) return rc;
    if ((rc = ap_span(m, &x, on1, 1, 0)) < 0) return rc;
    if ((rc = ap_span(m, &x, per1 - on1, 0, 0)) < 0) return rc;
    *Jv = x.sq / m->C;
    return AP_OK;
}

/* ---- 5. learning ----------------------------------------------------------------------------- */
static void test_learning(void)
{
    /* scalar step */
    double ws = 0;
    for (int k = 0; k < 20000; ++k) {
        const float J = randf(-1, 1), res = randf(-0.1, 0.1), sg = randf(0.001, 0.1);
        const float s = buck_learn_scalar_step(J, res, 0.1f, 0.1f, 1.0f / sg, 0.05f);
        const double e = fabs(s - ref_scalar(J, res, 0.1, 0.1, sg, 0.05));
        ws = fmax(ws, e);
        CHECK(e < 1e-7 + 1e-6 * fabs(s), "scalar step");
    }
    /* DeltaI: the inline constant-gain form against the general step with T = t - a.t */
    double wdi = 0;
    int gated = 0;
    for (int k = 0; k < 20000; ++k) {
        buck_learn_anchor_t a = { randf(0.7, 0.9), randf(0, 10), 0, 1U };
        a.ic = a.m + randf(-0.5, 0.5);
        const float v = a.v + randf(-0.06, 0.06), iavg = randf(0, 10), m = a.m + randf(-1.3, 1.3), di = randf(-1, 1);
        const float out = buck_learn_delta_i(&a, v, iavg, m, di);
        const double nu = v - (a.v + T_ / C_ * ((double)iavg - a.ic));
        const double exp_ = (fabs((double)m - a.m) <= 1.0)
            ? di + ref_scalar(-T_ / C_, nu, 0.25, 0.001, 0.02, 1.0) : (double)di;
        gated += fabs((double)m - a.m) > 1.0;
        wdi = fmax(wdi, fabs(out - exp_));
        CHECK(fabs(out - exp_) < 3e-6 * (1 + fabs(exp_)), "DeltaI step %g %g", out, exp_);
    }
    {
        buck_learn_anchor_t a = { 0.8f, 3, 3, 0U };
        CHECK(buck_learn_delta_i(&a, 0.9f, 3, 3, 0.25f) == 0.25f, "DeltaI without an anchor");
        a.valid = 1U;
        CHECK(buck_learn_delta_i(&a, NAN, 3, 3, 0.25f) == 0.25f && buck_learn_delta_i(&a, 0.8f, INFINITY, 3, 0.25f) == 0.25f &&
              buck_learn_delta_i(&a, 0.8f, 3, NAN, 0.25f) == 0.25f, "DeltaI non-finite data");
        buck_learn_anchor_set(&a, 0.81f, 2.0f, 0.5f);
        CHECK(a.valid && a.v == 0.81f && a.m == 2.0f && a.ic == 2.5f, "anchor = {V, m, m + future DeltaI}");
        printf("learning: scalar step %.2g; DeltaI vs the spec step %.2g (KD %.4f A/V, %d gated by the 1 A raw limit)\n",
               ws, wdi, (double)BUCK_LEARN_KD, gated);
    }

    /* q */
    double wq = 0, wqn = 0;
    int qok = 0, qrej = 0, qord[2] = { 0, 0 };
    for (int k = 0; k < 4000; ++k) {
        const uint32_t o = (uint32_t)(k & 1);
        const buck_toc_plan_t ep = ready_plan(o);
        const float i_d = ep.target_i + randf(-0.7, 0.7), v_d = ep.vref + randf(-0.02, 0.02);
        const float tail = randf(0.8e-6, 1.6e-6), on1 = randf(0.1e-6, 0.8e-6), per1 = (float)T_;
        const float h = (o == BUCK_TOC_OFF_ON) ? (BUCK_LEARN_QPEAK_AT_B ? tail + per1 : tail) : per1;
        const float v_s = v_d + randf(-0.01, 0.01);
        buck_learn_q_t j;
        memset(&j, 0xA5, sizeof j);
        buck_learn_q_start(&j, &ep, i_d, v_d, tail, on1, per1, v_s, 1U);
        CHECK(j.h == h, "q: h derived from the spans (F14)");
        int n = 0;
        while (buck_learn_q_step(&j) == BUCK_LEARN_BUSY && n < 50) ++n;
        const AP_Model am = { L_, C_, R_, VIN_, ep.ieq, 15, 1, .013, .013, .1, .99 };
        double jd, si, sv;
        int rd;
        if (o == BUCK_TOC_OFF_ON && BUCK_LEARN_QPEAK_AT_B)
            rd = ref_q_offon_at_b(&am, &ep, i_d, v_d, tail, per1, on1, &jd);
        else
            rd = ap_phase_voltage_jacobian(&am, pm(o), ep.x0.i, ep.x0.v, ep.ton0, ep.toff0, ep.ton_x, ep.toff_x,
                                           i_d, v_d, o == BUCK_TOC_OFF_ON ? tail : 0.0, per1, on1, i_d, v_d, &jd, &si, &sv);
        CHECK(rd == AP_OK, "q reference status %d", rd);
        if (j.status == BUCK_LEARN_SINGULAR) continue;
        const double ej = fabs(j.jq - jd) / fmax(1e-3, fabs(jd));
        wq = fmax(wq, ej);
        CHECK(ej < 1e-3, "Jq %g vs %g (order %u)", (double)j.jq, jd, (unsigned)o);
        const double hc = h / C_;
        if (fabs(fabs(jd) - hc * 0.05) < 0.01 * hc * 0.05) continue;     /* at the floor: either outcome */
        if (fabs(jd) < hc * 0.05) { CHECK(j.status == BUCK_LEARN_REJECT && j.q_new == ep.q_used, "Jq floor"); ++qrej; continue; }
        CHECK(j.status == BUCK_LEARN_OK, "q status %d", (int)j.status);
        const double qn = ep.q_used - ref_scalar(jd, (double)v_s - v_d, 0.10, 0.10, hc * 0.20, 0.05);
        wqn = fmax(wqn, fabs(j.q_new - qn));
        CHECK(fabs(j.q_new - qn) < 2e-5, "q update %g vs %g", (double)j.q_new, qn);
        ++qok; ++qord[o];
    }
    CHECK(qok > 1500 && qord[0] > 500 && qord[1] > 500, "q coverage %d", qok);
    {   /* q gates */
        const buck_toc_plan_t ep = ready_plan(BUCK_TOC_OFF_ON);
        buck_learn_q_t j;
        buck_learn_q_start(&j, &ep, ep.target_i + 0.76f, ep.vref, 1e-6f, 0.5e-6f, (float)T_, ep.vref, 1U);
        CHECK(j.status == BUCK_LEARN_REJECT && buck_learn_q_step(&j) == BUCK_LEARN_REJECT && j.q_new == ep.q_used, "q endpoint i gate");
        buck_learn_q_start(&j, &ep, ep.target_i, ep.vref + 0.026f, 1e-6f, 0.5e-6f, (float)T_, ep.vref, 1U);
        CHECK(j.status == BUCK_LEARN_REJECT, "q endpoint v gate");
        buck_learn_q_start(&j, &ep, ep.target_i, ep.vref, 1e-6f, 0.5e-6f, (float)T_, ep.vref, 0U);
        CHECK(j.status == BUCK_LEARN_REJECT, "q raw-load gate");
        buck_learn_q_start(&j, &ep, ep.target_i, ep.vref, 0, 0.5e-6f, (float)T_, ep.vref, 1U);
        CHECK(j.status == BUCK_LEARN_REJECT, "q zero tail / h");
        buck_learn_q_start(&j, &ep, NAN, ep.vref, 1e-6f, 0.5e-6f, (float)T_, ep.vref, 1U);
        CHECK(j.status == BUCK_LEARN_REJECT, "q NaN endpoint");
        buck_learn_q_start(&j, &ep, ep.target_i, ep.vref, 1e-6f, 0.5e-6f, (float)T_, NAN, 1U);
        while (buck_learn_q_step(&j) == BUCK_LEARN_BUSY) {}
        CHECK(j.status == BUCK_LEARN_NONFINITE && j.q_new == ep.q_used, "q NaN V_S");
        const buck_toc_plan_t e2 = ready_plan(BUCK_TOC_ON_OFF);
        buck_learn_q_start(&j, &e2, e2.target_i, e2.vref, 0, 2e-6f, 1e-6f, e2.vref, 1U);
        CHECK(j.status == BUCK_LEARN_REJECT, "q on1 > period");
        buck_toc_plan_t e3 = ready_plan(BUCK_TOC_OFF_ON);
        e3.ton_x = e3.toff_x = 0.0f;                           /* nothing executed: Jq = 0, below the floor */
        buck_learn_q_start(&j, &e3, e3.target_i, e3.vref, 1e-6f, 0.5e-6f, (float)T_, e3.vref + 0.01f, 1U);
        while (buck_learn_q_step(&j) == BUCK_LEARN_BUSY) {}
        CHECK(j.status == BUCK_LEARN_REJECT && j.jq == 0.0f && j.q_new == e3.q_used, "q Jq floor");
        /* F05: |Jq| between 0.1x and 1x the floor must be rejected, just above it accepted: scale the
         * executed spans down until Jq lands there (bisection on the scale) */
        int f05_rej = 0, f05_ok = 0;
        for (int k = 0; k < 400 && (f05_rej < 20 || f05_ok < 20); ++k) {
            buck_toc_plan_t e4 = ready_plan((uint32_t)(k & 1));
            const float tl = 1e-6f, o1 = 0.5e-6f, p1 = (float)T_;
            const double hcf = ((e4.order == BUCK_TOC_OFF_ON) ? (BUCK_LEARN_QPEAK_AT_B ? tl + p1 : tl) : p1) / C_ * 0.05;
            double lo = 0.0, hi = 1.0;
            for (int it = 0; it < 40; ++it) {          /* find the scale where |Jq| = floor */
                const double mid = 0.5 * (lo + hi);
                buck_toc_plan_t e5 = e4;
                e5.ton_x = (float)(e4.ton_x * mid); e5.toff_x = (float)(e4.toff_x * mid);
                buck_learn_q_start(&j, &e5, e5.target_i, e5.vref, tl, o1, p1, e5.vref, 1U);
                while (buck_learn_q_step(&j) == BUCK_LEARN_BUSY) {}
                if (fabs(j.jq) < hcf) lo = mid; else hi = mid;
            }
            if (!(hi < 0.999)) continue;               /* Jq never reaches the floor from above: skip */
            for (int side = 0; side < 2; ++side) {
                const double sc = side ? hi * 1.02 : lo * 0.9;   /* just above / ~below the floor */
                buck_toc_plan_t e5 = e4;
                e5.ton_x = (float)(e4.ton_x * sc); e5.toff_x = (float)(e4.toff_x * sc);
                buck_learn_q_start(&j, &e5, e5.target_i, e5.vref, tl, o1, p1, e5.vref + 0.001f, 1U);
                while (buck_learn_q_step(&j) == BUCK_LEARN_BUSY) {}
                const double r = fabs(j.jq) / hcf;
                if (r > 0.1 && r < 0.98) { CHECK(j.status == BUCK_LEARN_REJECT, "F05 Jq below the floor (%.3f)", r); ++f05_rej; }
                if (r > 1.0 && r < 1.2) { CHECK(j.status == BUCK_LEARN_OK, "F05 Jq above the floor (%.3f)", r); ++f05_ok; }
            }
        }
        CHECK(f05_rej >= 20 && f05_ok >= 20, "F05 coverage %d %d", f05_rej, f05_ok);
        /* F12: the job copies the plan at start; changing the plan afterwards changes nothing */
        buck_toc_plan_t e6 = ready_plan(BUCK_TOC_ON_OFF);
        buck_learn_q_t j1, j2;
        buck_learn_q_start(&j1, &e6, e6.target_i, e6.vref, 0, 0.5e-6f, (float)T_, e6.vref + 0.003f, 1U);
        buck_learn_q_start(&j2, &e6, e6.target_i, e6.vref, 0, 0.5e-6f, (float)T_, e6.vref + 0.003f, 1U);
        memset(&e6, 0x5A, sizeof e6);
        while (buck_learn_q_step(&j1) == BUCK_LEARN_BUSY) {}
        buck_toc_plan_t e7 = ready_plan(BUCK_TOC_OFF_ON);
        (void)e7;
        while (buck_learn_q_step(&j2) == BUCK_LEARN_BUSY) {}
        CHECK(j1.status == j2.status && j1.q_new == j2.q_new, "F12 the q job does not read the plan after its start");
        printf("q floor (F05): %d rejected below, %d accepted above; F12 copy checked\n", f05_rej, f05_ok);
    }
    printf("q: Jq vs ap_phase_voltage_jacobian worst %.2g, update vs spec %.2g A (%d updates: ON-OFF %d, OFF-ON %d; %d below the Jq floor)\n",
           wq, wqn, qok, qord[0], qord[1], qrej);

    /* G */
    double wjg = 0, wg = 0;
    int gok = 0, gsing = 0;
    for (int k = 0; k < 4000; ++k) {
        const uint32_t o = (uint32_t)(k & 1);
        const buck_toc_plan_t ep = ready_plan(o);
        const float v0 = ep.x0.v + randf(-0.01, 0.01);
        buck_learn_g_t g;
        memset(&g, 0xA5, sizeof g);
        buck_learn_g_start(&g, &ep, v0);
        int n = 0;
        while (buck_learn_g_step(&g) == BUCK_LEARN_BUSY && n < 10) ++n;
        CHECK(g.status == BUCK_LEARN_OK && n == 4, "G Jacobian status %d steps %d", (int)g.status, n + 1);
        const TM_Model m = tm_model(ep.ieq);
        TM_State yd; TM_Mat jt, jg;
        tm_endpoint(&m, (TM_State){ ep.x0.i, v0 }, ep.ton, ep.toff, pm(o), &yd, &jt);
        tm_gain_jacobian(jt, ep.ton0, ep.toff0, &jg);
        const double si = fmax(fabs(jg.m00), fabs(jg.m01)), sv = fmax(fabs(jg.m10), fabs(jg.m11));
        const double e = fmax(fmax(fabs(g.jg.a - jg.m00), fabs(g.jg.b - jg.m01)) / si, fmax(fabs(g.jg.c - jg.m10), fabs(g.jg.d - jg.m11)) / sv);
        wjg = fmax(wjg, e);
        CHECK(e < 2e-5, "Jg %g", e);
        const float i_d = ep.target_i + randf(-1, 1), v_d = ep.vref + randf(-0.03, 0.03);
        buck_learn_g_teach_start(&g, i_d, v_d);
        while (buck_learn_g_step(&g) == BUCK_LEARN_BUSY) {}
        TM_State dg;
        const TM_Mat jf = { g.jg.a, g.jg.b, g.jg.c, g.jg.d };
        const int rd = tm_gain_step(jf, (TM_State){ (double)i_d - ep.target_i, (double)v_d - ep.vref }, .2, .02, .01, .1, &dg);
        if (g.status == BUCK_LEARN_SINGULAR) { ++gsing; continue; }
        CHECK(rd == TM_OK && g.status == BUCK_LEARN_OK, "G update status %d %d", rd, (int)g.status);
        double alpha = 1;
        if (dg.i != 0) alpha = fmin(alpha, .02 * ep.g.i / fabs(dg.i));
        if (dg.v != 0) alpha = fmin(alpha, .02 * ep.g.v / fabs(dg.v));
        const double gi = ep.g.i + alpha * dg.i, gv = ep.g.v + alpha * dg.v;
        wg = fmax(wg, fmax(fabs(g.g_new.i - gi), fabs(g.g_new.v - gv)));
        CHECK(fabs(g.g_new.i - gi) < 1e-5 * gi && fabs(g.g_new.v - gv) < 1e-5 * gv, "G update");
        CHECK(fabs(g.g_new.i - ep.g.i) <= 0.020001 * ep.g.i && fabs(g.g_new.v - ep.g.v) <= 0.020001 * ep.g.v, "G 2 %% cap");
        ++gok;
    }
    CHECK(gok > 3800, "G coverage %d", gok);
    {
        buck_learn_g_t g;
        const buck_toc_plan_t ep = ready_plan(BUCK_TOC_ON_OFF);
        buck_learn_g_start(&g, &ep, ep.x0.v);
        buck_learn_g_teach_start(&g, ep.target_i, ep.vref);         /* before the Jacobian */
        CHECK(g.status == BUCK_LEARN_REJECT && g.g_new.i == ep.g.i && g.g_new.v == ep.g.v, "G teach without Jacobian");
        buck_learn_g_start(&g, &ep, ep.x0.v);
        while (buck_learn_g_step(&g) == BUCK_LEARN_BUSY) {}
        buck_learn_g_teach_start(&g, NAN, ep.vref);
        while (buck_learn_g_step(&g) == BUCK_LEARN_BUSY) {}
        CHECK(g.status == BUCK_LEARN_NONFINITE && g.g_new.i == ep.g.i, "G NaN endpoint");
        buck_learn_g_start(&g, &ep, INFINITY);
        while (buck_learn_g_step(&g) == BUCK_LEARN_BUSY) {}
        CHECK(g.status == BUCK_LEARN_NONFINITE, "G infinite V(S0)");
    }
    printf("G: Jg vs tm_endpoint/tm_gain_jacobian worst %.2g, update vs tm_gain_step + common alpha %.2g (%d updates, %d singular)\n",
           wjg, wg, gok, gsing);

    /* kappa */
    double wk = 0;
    for (int k = 0; k < 20000; ++k) {
        const float ku = randf(-0.5, 1.5), mr = randf(0, 10), z = randf(-4, 4), me = randf(0, 10);
        float out = 7;
        CHECK(buck_learn_kappa(ku, mr, z, me, &out) == 1U, "kappa status");
        const double e = (double)ku + .05 * z * ((double)me - (mr + (double)ku * z)) / (.04 + (double)z * z);
        wk = fmax(wk, fabs(out - e));
        CHECK(fabs(out - e) < 2e-6, "kappa");
    }
    float kk = 7;
    CHECK(buck_learn_kappa(0.3f, 2, NAN, 3, &kk) == 0U && kk == 7, "kappa NaN");

    /* handover */
    int hc = 0;
    for (int k = 0; k < 20000; ++k) {
        const float m = randf(-4, 14), di = randf(-1, 1), vref = randf(0.5, 3.3);
        buck_toc_handover_t h;
        buck_learn_handover(m, di, vref, &h);
        const double ieq = (double)m + di, dut = (vref + R_ * ieq) / VIN_;
        CHECK(h.ieq_end == m + di, "Ieq_end");
        CHECK(h.integ_v == fmin(fmax(m + di, BUCK_IREF_MIN_A), BUCK_IREF_MAX_A), "integ_v clamp");
        CHECK(h.integ_i >= BUCK_D_MIN && h.integ_i <= BUCK_D_MAX && fabs(h.integ_i - fmin(fmax(dut, BUCK_D_MIN), BUCK_D_MAX)) < 1e-6,
              "integ_i clamp");
        CHECK(h.cmp == buck_duty_to_cmp(h.integ_i) && h.cmp >= BUCK_CMP_MIN && h.cmp <= BUCK_CMP_MAX, "restart CMP1");
        hc += (dut < BUCK_D_MIN || dut > BUCK_D_MAX || ieq < BUCK_IREF_MIN_A || ieq > BUCK_IREF_MAX_A);
    }
    buck_toc_handover_t hn;
    buck_learn_handover(NAN, 0, 0.8f, &hn);
    CHECK(hn.integ_v == BUCK_IREF_MIN_A && hn.integ_i == BUCK_D_MIN && hn.cmp == buck_duty_to_cmp(BUCK_D_MIN), "handover NaN -> low limits");

    /* observer step at D */
    const buck_obs_k_t ok = BUCK_OBS_K_DEFAULTS;
    int obs_diff = 0;
    for (int k = 0; k < 2000; ++k) {
        buck_obs_t x = { randf(0.7, 0.9), randf(0, 10) }, y = x;
        const float v = randf(0.7, 0.9), il = randf(0, 10), h = randf(0.2e-6, 12e-6);
        const float r1 = buck_toc_obs_step_d(&x, v, il, ok, h);
        const float r2 = buck_obs_step_a(&y, v, il, ok, h * (float)(1.0 / BUCK_C_F));
        obs_diff += memcmp(&x, &y, sizeof x) != 0 || r1 != r2;
    }
    CHECK(obs_diff == 0, "observer at D");
    printf("kappa vs spec %.2g; handover clamps (%d of 20000 at a limit); observer at D = buck_obs_step_a(a = h/C) "
           "(at h = T: a = %.9g, BUCK_OBS_A = %.9g)\n", wk, hc, (double)((float)T_ * (float)(1.0 / C_)), (double)BUCK_OBS_A);

    /* the executed frame */
    double wf = 0, wmeas = 0;
    for (int k = 0; k < 4000; ++k) {
        const buck_toc_plan_t ep = ready_plan((uint32_t)(k & 1));
        const buck_xy_t x0 = { ep.x0.i + randf(-0.1, 0.1), ep.x0.v + randf(-0.005, 0.005) };
        buck_toc_frame_t fr;
        buck_toc_frame_start(&fr, &ep, x0);
        CHECK(buck_toc_frame_step(&fr) == BUCK_LEARN_BUSY && buck_toc_frame_step(&fr) == BUCK_LEARN_BUSY &&
              buck_toc_frame_step(&fr) == BUCK_LEARN_BUSY && buck_toc_frame_step(&fr) == BUCK_LEARN_BUSY &&
              buck_toc_frame_step(&fr) == BUCK_LEARN_OK && buck_toc_frame_step(&fr) == BUCK_LEARN_OK, "frame steps");
        buck_toc_frame_t fr2;
        buck_toc_frame_start(&fr2, &ep, x0);
        CHECK(buck_toc_frame_run(&fr2) == BUCK_LEARN_OK && memcmp(&fr2.xd, &fr.xd, sizeof fr.xd) == 0 &&
              fr2.q1 == fr.q1 && fr2.q2 == fr.q2 && fr2.inv_h == fr.inv_h, "frame_run = the steps");
        double i = x0.i, v = x0.v;
        const int g1 = ep.order == BUCK_TOC_ON_OFF;
        const double h1 = g1 ? ep.ton_x : ep.toff_x, h2 = g1 ? ep.toff_x : ep.ton_x;
        toc_advance(&i, &v, h1, g1, L_, C_, R_, VIN_, ep.ieq);
        const double im = i, q1 = 0.5 * h1 * ((double)x0.i + i);
        toc_advance(&i, &v, h2, !g1, L_, C_, R_, VIN_, ep.ieq);
        const double q2 = 0.5 * h2 * (im + i);
        const double e = fmax(fmax(scaled(fr.xd.i, i), scaled(fr.xd.v, v)), fmax(fabs(fr.q1 - q1) / fmax(1e-6, fabs(q1)), fabs(fr.q2 - q2) / fmax(1e-6, fabs(q2))));
        wf = fmax(wf, e);
        CHECK(e < 1e-5, "frame prediction %g", e);
        const float ia = randf(0, 10), kc = randf(0, 1);
        /* the model's ADC2 value: the mean of the piecewise-linear model current at the 4 instants */
        double pm_ = 0;
        for (int jj = 0; jj < 4; ++jj) {
            const double tau = ((double)ep.fw.mcmp[jj] - BUCK_DT_TICKS) * BUCK_TICK_S;
            pm_ += (tau <= h1) ? x0.i + (im - x0.i) * tau / h1 : im + (i - im) * (tau - h1) / h2;
        }
        pm_ *= 0.25;
        const float pred = buck_toc_frame_meas(&fr, ep.fw.mcmp);
        wmeas = fmax(wmeas, fabs(pred - pm_));
        CHECK(fabs(pred - pm_) < 1e-4 * (1 + fabs(pm_)), "ADC2 model value %g %g", (double)pred, pm_);
        const double rec = fr.xd.i + kc * (ia - (double)pred);
        CHECK(fabs(buck_toc_frame_correct(&fr, ia, pred, kc) - rec) < 1e-5, "i(D) correction");
        const double il = (fr.q1 + fr.q2 + h2 * (ia - (double)pred)) / (h1 + h2);
        CHECK(fabs(buck_toc_frame_il(&fr, ia, pred) - il) < 1e-4 * (1 + fabs(il)), "observer current at D (Q6)");
        /* Q5: k = 2a/(1+a) of the last span, a = (2 - theta h)/(2 + theta h) */
        const double a_ = (2 - BUCK_THETA * h2) / (2 + BUCK_THETA * h2);
        CHECK(fabs(buck_toc_deadbeat_k(fr.h2) - 2 * a_ / (1 + a_)) < 1e-6, "deadbeat k");
        /* Q7: the OFF tail with Buck_disturb_obs's current-only form (buck_pade_coeffs' a_off/b_off) */
        const float tl = randf(0.4e-6, 1.3e-6), id_ = randf(0, 10), vd_ = randf(0.7, 0.9);
        const double xo = BUCK_THETA * tl;
        const double ia_ = (2 - xo) / (2 + xo) * id_ - 2 * tl / (L_ * (2 + xo)) * vd_;
        CHECK(fabs(buck_toc_tail_valley(id_, vd_, tl) - ia_) < 2e-5 * (1 + fabs(ia_)), "tail valley (Q7)");
    }
    printf("frame prediction (D state, span charges) vs double %.2g; ADC2 model value %.2g A; i(D), observer current,\n"
           "  deadbeat k, tail valley checked\n", wf, wmeas);
}

/* ---- 6. non-finite inputs ------------------------------------------------------------------ */
static void test_nonfinite(void)
{
    const float bad[3] = { NAN, INFINITY, -INFINITY };
    int ready_bad = 0, total = 0;
    for (int k = 0; k < 3000; ++k) {
        buck_toc_req_t rq = rand_req((uint32_t)(k & 1));
        float *fields[] = { &rq.i_r, &rq.v_r, &rq.ton_n, &rq.m_r, &rq.raw_before, &rq.delta_i, &rq.kappa,
                            &rq.q[0], &rq.q[1], &rq.g[0].i, &rq.g[0].v, &rq.g[1].i, &rq.g[1].v, &rq.vref };
        const int nf = (int)(sizeof fields / sizeof fields[0]);
        const int f = k % nf;
        *fields[f] = bad[(k / nf) % 3];
        buck_toc_planner_t p;
        const int n = run_steps(&p, &rq);
        buck_toc_plan_t d;
        memset(&d, 0xA5, sizeof d);
        buck_toc_plan_direct(&rq, &d);
        CHECK(memcmp(&d, &p.r, sizeof d) == 0 && n < 30, "non-finite: stepped = direct");
        /* a bad gain of the order NOT selected does not matter */
        const int unused_gain = f >= 9 && f <= 12 && (uint32_t)((f - 9) / 2) != p.r.order;
        if (p.r.status == BUCK_TOC_READY && !unused_gain) ++ready_bad;
        if (p.r.status == BUCK_TOC_READY)
            CHECK(buck_toc_fin(p.r.ton) && buck_toc_fin(p.r.toff) && p.r.per_ticks <= BUCK_TOC_PER_MAX_TICKS, "finite plan");
        ++total;
    }
    CHECK(ready_bad == 0, "a plan from a non-finite input: %d", ready_bad);
    /* zero / negative gains, garbage magnitudes */
    buck_toc_req_t rq = rand_req(1U);
    rq.g[0].i = rq.g[1].i = 0.0f; rq.g[0].v = rq.g[1].v = -1.0f;
    buck_toc_planner_t p;
    run_steps(&p, &rq);
    CHECK(p.r.status != BUCK_TOC_READY, "non-positive gains");
    rq = rand_req(1U); rq.i_r = 3e37f; rq.m_r = -3e37f;
    run_steps(&p, &rq);
    CHECK(p.r.status != BUCK_TOC_READY, "huge finite inputs");
    printf("non-finite inputs: %d requests with a NaN/inf field, no plan from them (stepped = direct)\n", total);
}

static uint32_t svc_count(void *c)
{
    struct ctx { int n, stop_at; } *x = c;
    return (++x->n >= x->stop_at) ? 1U : 0U;
}

/* ---- 7. stage 2 -------------------------------------------------------------------------------- */
/* The forward-map decision of the stage-1 planner (two span maps, tolerance 64 eps (1 + |x0| + |target|)). */
static int fwd_ok(const buck_toc_planner_t *p, float u, float w, uint32_t o)
{
    const uint32_t on_first = (o == BUCK_TOC_ON_OFF);
    const float h1 = (on_first ? u : w) * BUCK_TOC_UF, h2 = (on_first ? w : u) * BUCK_TOC_UF;
    buck_toc_span_t f;
    buck_toc_span(&f, h1);
    const buck_xy_t xm = buck_toc_map_f(&f, p->r.x0, h1, on_first, p->r.ieq);
    buck_toc_span(&f, h2);
    const buck_xy_t y = buck_toc_map_f(&f, xm, h2, on_first ^ 1U, p->r.ieq);
    return fabsf(y.i - p->target[o]) <= p->tol_i[o] && fabsf(y.v - p->rq.vref) <= p->tol_v;
}

static void test_stage2(void)
{
    /* Q11: the residual of the eliminated row against the forward maps, on every root the planner
     * finds (true roots) and on perturbed candidates (false roots): the same decision */
    long agree = 0, disagree = 0, trues = 0, falses = 0, true_fwd_rej = 0, false_fwd_acc = 0, res_only = 0;
    double worst_true = 0, worst_fwd = 0;
    for (int k = 0; k < 30000; ++k) {
        const buck_toc_req_t rq = rand_req((uint32_t)(k & 1));
        buck_toc_planner_t p;
        memset(&p, 0, sizeof p);
        buck_toc_plan_start(&p, &rq);
        while (p.stage < BUCK_TOC_ST_DONE) {
            const uint32_t st = p.stage;
            if (st == BUCK_TOC_ST_PUSH || st == BUCK_TOC_ST_PUSH + BUCK_TOC_ST_NORD) {
                const uint32_t o = (st == BUCK_TOC_ST_PUSH) ? BUCK_TOC_ON_OFF : BUCK_TOC_OFF_ON;
                for (uint32_t r = 0; r < p.nr; ++r) {
                    if (!(p.wmask & (1U << r)) || !(p.ru[r] >= 0 && p.rw[r] >= 0) || !(p.ru[r] + p.rw[r] > 0)) continue;
                    const uint32_t row0 = (p.rows >> r) & 1U;
                    for (int pert = 0; pert < 4; ++pert) {
                        const float fu = pert == 0 ? 1.0f : pert == 1 ? 1.001f : pert == 2 ? 0.999f : 1.0f;
                        const float fw = pert == 3 ? 1.001f : 1.0f;
                        const float u = p.ru[r] * fu, w = p.rw[r] * fw;
                        const int a = (int)buck_toc_res_ok(&p.bf, p.d1, u, w, row0), b = fwd_ok(&p, u, w, o);
                        if (a == b) ++agree; else { ++disagree; if (pert == 0) ++true_fwd_rej; else if (b) ++false_fwd_acc; else ++res_only; }
                        if (pert == 0) { ++trues; CHECK(a, "Q11: a true root fails the residual test"); }
                        else ++falses;
                        if (b && !a) {              /* the forward maps' worst accepted endpoint, for comparison */
                            const TM_State e = ref_pair((TM_State){ p.r.x0.i, p.r.x0.v }, u * 1e-6, w * 1e-6, pm(o), p.r.ieq);
                            worst_fwd = fmax(worst_fwd, fmax(fabs(e.i - p.target[o]) / (1 + fabs(p.target[o])),
                                                             fabs(e.v - p.rq.vref) / (1 + fabs(p.rq.vref))));
                        }
                        if (a) {                    /* accepted: its endpoint in double must be close */
                            const TM_State e = ref_pair((TM_State){ p.r.x0.i, p.r.x0.v }, u * 1e-6, w * 1e-6, pm(o), p.r.ieq);
                            const double err = fmax(fabs(e.i - p.target[o]) / (1 + fabs(p.target[o])),
                                                    fabs(e.v - p.rq.vref) / (1 + fabs(p.rq.vref)));
                            worst_true = fmax(worst_true, err);
                            CHECK(err < 1e-2, "Q11: an accepted candidate misses the target by %.2g", err);
                        }
                    }
                }
            }
            if (buck_toc_plan_step(&p) != BUCK_TOC_BUSY) break;
        }
    }
    printf("Q11 residual vs forward maps: %ld agree, %ld disagree (%ld true roots: all pass the residual, %ld of them fail the\n"
           "  forward maps' 64-eps tolerance; %ld candidates perturbed by 1e-3: %ld pass only the forward maps, %ld only the\n"
           "  residual); worst relative endpoint error in double of a candidate accepted by the residual %.2g (only by the\n"
           "  forward maps: %.2g)\n",
           agree, disagree, trues, true_fwd_rej, falses, false_fwd_acc, res_only, worst_true, worst_fwd);
    /* F06: the residual tolerance at 2x and 0.5x (a scaled residual on a constructed form) */
    {
        buck_toc_bf_t bf = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0, 0, 0, 0 };
        /* row 1 (tested when w came from row 0): d1 + p1 u + p1 w + k1 u w with d1 chosen so that the
         * residual is +-x times the tolerance */
        const float u = 0.5f, w = 0.25f, scale0 = (0.5f + 0.25f) + 0.125f;
        for (int side = 0; side < 2; ++side) {
            const float x = side ? 2.0f : 0.5f;
            const float target = x * BUCK_TOC_RES_EPS * (fabsf(-scale0) + scale0);
            const float d1 = -scale0 + target;
            const uint32_t ok = buck_toc_res_ok(&bf, d1, u, w, 1U);
            CHECK(ok == (side ? 0U : 1U), "F06: residual at %.1fx the tolerance", (double)x);
        }
    }
    /* F06: ALREADY at 2x / 0.5x of its tolerance; F09: ton_n outside [0, T] */
    {
        int done = 0;
        for (int k = 0; k < 2000 && done < 50; ++k) {
            buck_toc_req_t rq = rand_req(0U);
            buck_toc_planner_t p;
            run_steps(&p, &rq);
            if (p.r.status == BUCK_TOC_INVALID) continue;
            /* move x0 onto the valley target +- f x the allowance, v exactly at Vref */
            const float t0 = p.target[0], tol = p.tol_i[0] * (BUCK_TOC_SAME_EPS / BUCK_TOC_VERIFY_EPS);
            for (int side = 0; side < 2; ++side) {
                buck_toc_req_t r2 = rq;
                r2.i_r = t0 + (side ? 2.0f : 0.5f) * tol;
                r2.v_r = rq.vref;
                buck_toc_planner_t p2;
                run_steps(&p2, &r2);
                if (fabsf(p2.target[0] - t0) > 0) continue;  /* the ripple moved with x0.v: skip */
                CHECK((p2.r.status == BUCK_TOC_ALREADY) == (side == 0), "F06 already-at-target at %dx/2", side ? 4 : 1);
            }
            ++done;
        }
        buck_toc_req_t rq = rand_req(0U);
        buck_toc_plan_t d;
        rq.ton_n = -1e-9f; buck_toc_plan_direct(&rq, &d);
        CHECK(d.status == BUCK_TOC_INVALID, "F09 ton_n < 0");
        rq.ton_n = (float)T_ * 1.001f; buck_toc_plan_direct(&rq, &d);
        CHECK(d.status == BUCK_TOC_INVALID, "F09 ton_n > T");
        rq.ton_n = NAN; buck_toc_plan_direct(&rq, &d);
        CHECK(d.status == BUCK_TOC_INVALID, "F09 ton_n NaN");
    }
    /* the frame encoder: its constraints over all orders and a grid of spans and resume CMP1s;
     * the boundaries PER' = max and the span minima (F06 M15) */
    long enc_ok = 0, enc_lim = 0;
    for (uint32_t o = 0; o < 2; ++o)
        for (uint32_t on = 300; on < 30000; on += 97)
            for (uint32_t off = 600; off < 40000; off += 211)
                for (uint32_t res = BUCK_CMP_MIN; res <= BUCK_CMP_MAX; res += 1792) {
                    buck_toc_frame_words_t w;
                    if (buck_toc_frame_encode(o, on, off, res, &w) != BUCK_TOC_FRAME_OK) { ++enc_lim; continue; }
                    ++enc_ok;
                    int bad = w.per < BUCK_TOC_PER_MIN_TICKS || w.per > BUCK_TOC_PER_MAX_TICKS || on < BUCK_TOC_ON_MIN_TICKS ||
                              off < BUCK_TOC_OFF_MIN_TICKS || w.cmp4 < BUCK_TOC_ADC1_FIRST || w.cmp3 != w.cmp4 + BUCK_VSAMPLE_GAP_TICKS ||
                              w.cmp3 + BUCK_VSAMPLE_LEAD_TICKS != w.d_ref || w.mcmp[0] < BUCK_TOC_ADC2_FIRST ||
                              w.mcmp[3] + BUCK_TOC_ADC2_END_GUARD > w.per || w.cmp1 >= w.per || w.cmp3 >= w.per;
                    for (int j = 0; j < 3; ++j) bad |= w.mcmp[j + 1] < w.mcmp[j] + BUCK_TOC_ADC2_GAP_TICKS;
                    if (o == 0) bad |= w.per != on + off || w.cmp1 != BUCK_DT_TICKS + on || w.set_cmp2;
                    else bad |= w.cmp2 != off || w.cmp1 != BUCK_DT_TICKS + off + on || w.per != w.cmp1 + BUCK_HRTIM_PERIOD - res ||
                                !w.set_cmp2 || w.cmp2 >= w.cmp1;
                    CHECK(!bad, "frame words o%u on %u off %u res %u", (unsigned)o, (unsigned)on, (unsigned)off, (unsigned)res);
                }
    {
        buck_toc_frame_words_t w;
        const uint32_t on = 20000U;
        CHECK(buck_toc_frame_encode(0, on, BUCK_TOC_PER_MAX_TICKS - on, 1500, &w) == BUCK_TOC_FRAME_OK, "PER' = max is inside");
        CHECK(buck_toc_frame_encode(0, on, BUCK_TOC_PER_MAX_TICKS - on + 1, 1500, &w) == BUCK_TOC_FRAME_LIMIT, "PER' = max + 1 is out");
        CHECK(buck_toc_frame_encode(0, BUCK_TOC_ON_MIN_TICKS, 8000, 1500, &w) == BUCK_TOC_FRAME_OK, "on = min is inside");
        CHECK(buck_toc_frame_encode(0, BUCK_TOC_ON_MIN_TICKS - 1, 8000, 1500, &w) == BUCK_TOC_FRAME_LIMIT, "on = min - 1 is out");
        CHECK(buck_toc_frame_encode(1, 8000, BUCK_TOC_OFF_MIN_TICKS, 1500, &w) == BUCK_TOC_FRAME_OK, "off = min is inside");
        CHECK(buck_toc_frame_encode(1, 8000, BUCK_TOC_OFF_MIN_TICKS - 1, 1500, &w) == BUCK_TOC_FRAME_LIMIT, "off = min - 1 is out");
    }
    printf("frame encoder: %ld frames checked (limits, ADC placement, words of both orders), %ld outside the limits\n",
           enc_ok, enc_lim);
    /* the poll hook: the same plan with a hook that is called and never aborts; an abort */
    {
        static volatile uint32_t flag;
        static int calls;
        struct ctx { int n, stop_at; } cx;
        int diffs = 0, aborts = 0;
        for (int k = 0; k < 5000; ++k) {
            const buck_toc_req_t rq = rand_req((uint32_t)(k & 1));
            buck_toc_plan_t a, b2;
            memset(&a, 0xA5, sizeof a); memset(&b2, 0x5A, sizeof b2);
            buck_toc_plan_direct(&rq, &a);
            flag = 1U; cx.n = 0; cx.stop_at = 1 << 30;
            const buck_toc_poll_t poll = { &flag, 1U, svc_count, &cx };
            buck_toc_plan_direct_poll(&rq, &b2, &poll);
            diffs += memcmp(&a, &b2, sizeof a) != 0;
            cx.n = 0; cx.stop_at = 2;
            buck_toc_plan_direct_poll(&rq, &b2, &poll);
            aborts += b2.status == BUCK_TOC_ABORT || a.status == BUCK_TOC_INVALID || a.status == BUCK_TOC_ALREADY;
            calls = cx.n;
        }
        (void)calls;
        CHECK(diffs == 0 && aborts == 5000, "poll hook: %d differences, %d aborts", diffs, aborts);
        printf("poll hook: the same plan with the hook called at every poll point (5000), abort honoured\n");
    }
}

/* ---- 8. stage 2b ------------------------------------------------------------------------------------ */
/* a load step seen BUCK_TOC_PLAN_PERIODS + 1 periods later with the duty held (the firmware's situation) */
static buck_toc_req_t step_req(void)
{
    buck_toc_req_t r;
    memset(&r, 0, sizeof r);
    r.vref = (rand01() < 0.8) ? 0.8057f : randf(0.5, 3.0);
    const float i1 = randf(0, 10);
    float st = randf(0.3, 8);
    if (rand01() < 0.5) st = -st;
    float i2 = i1 + st;
    if (i2 < -1) i2 = -1;
    if (i2 > 12) i2 = 12;
    const float duty = (r.vref + (float)R_ * i1) / (float)VIN_;
    r.ton_n = duty * (float)T_;
    const float rip = (float)((VIN_ - r.vref) * duty * T_ / L_);
    r.i_r = i1 - 0.5f * rip + randf(-0.3, 0.3);
    r.v_r = r.vref - (i2 - i1) * (float)(T_ / C_) * (float)(BUCK_TOC_PLAN_PERIODS_TEST + 1) * randf(0.3, 1.2) + randf(-0.01, 0.01);
    r.m_r = i2; r.raw_before = i2;
    r.q[0] = randf(-0.05, 0.05); r.q[1] = randf(-0.05, 0.05);
    r.g[0].i = randf(0.95, 1.05); r.g[0].v = randf(0.95, 1.05);
    r.g[1].i = randf(0.95, 1.05); r.g[1].v = randf(0.95, 1.05);
    r.resume_cmp = buck_toc_resume_cmp(r.vref, i2);
    return r;
}

static void test_stage2b(void)
{
    /* the cheaper planner against the full one, on the random requests of section 2 and on load steps */
    for (int dist = 0; dist < 2; ++dist) {
        long n = 0, same = 0, same_diff = 0, other = 0, other_none = 0, other_limit = 0, other_ready = 0, full_ok = 0;
        long cheap_extra = 0;
        double rsum = 0, rmin = 1e30, rmax = 0, wsame = 0;
        for (int k = 0; k < 100000; ++k) {
            const buck_toc_req_t rq = dist ? step_req() : rand_req(0U);
            buck_toc_plan_t f, c;
            buck_toc_plan_direct(&rq, &f);
            buck_toc_plan_fast(&rq, &c);
            ++n;
            if (f.status == BUCK_TOC_INVALID || f.status == BUCK_TOC_ALREADY) {
                CHECK(c.status == f.status, "2b: INVALID/ALREADY differ (%d %d)", (int)f.status, (int)c.status);
                continue;
            }
            if (f.status == BUCK_TOC_NONE) { cheap_extra += (c.status == BUCK_TOC_READY || c.status == BUCK_TOC_LIMIT); continue; }
            ++full_ok;                          /* READY or LIMIT: a verified pair exists */
            const int rule_onoff = (rq.vref - c.x0.v > 0.0f);
            const uint32_t ro = rule_onoff ? BUCK_TOC_ON_OFF : BUCK_TOC_OFF_ON;
            if (ro == f.order) {
                ++same;
                const double e = fmax(fabs((double)c.ton0 - f.ton0) / fmax(1e-9, f.ton0 + f.toff0),
                                      fabs((double)c.toff0 - f.toff0) / fmax(1e-9, f.ton0 + f.toff0));
                wsame = fmax(wsame, e);
                if (c.status != f.status || c.order != f.order || e > 1e-5) ++same_diff;
            } else {
                ++other;
                if (c.status == BUCK_TOC_NONE) ++other_none;
                else {
                    const double q = ((double)c.ton0 + c.toff0) / ((double)f.ton0 + f.toff0);
                    rsum += q; rmin = fmin(rmin, q); rmax = fmax(rmax, q);
                    if (c.status == BUCK_TOC_READY) ++other_ready; else ++other_limit;
                }
            }
        }
        CHECK(same_diff == 0, "2b: the cheaper plan differs from the full one in the same order (%ld)", same_diff);
        CHECK(cheap_extra == 0, "2b: a cheaper plan where the full planner has none (%ld)", cheap_extra);
        const long other_pair = other_ready + other_limit;
        printf("cheaper planner (%s, %ld requests; full planner: a pair in %ld): the same order in %ld (%.2f %%; identical\n"
               "  plan, worst span difference %.1g of the frame); the other order in %ld (%.2f %%): there no pair in %ld (NONE),\n"
               "  a pair in %ld (READY %ld, LIMIT %ld), length / the full plan's mean %.3f, %.3f .. %.3f\n",
               dist ? "load steps" : "random", n, full_ok, same, 100.0 * same / full_ok, wsame, other, 100.0 * other / full_ok,
               other_none, other_pair, other_ready, other_limit, other_pair ? rsum / other_pair : 0.0,
               other_pair ? rmin : 0.0, rmax);
    }
    /* the six parts = the polled form, bit for bit; an abort at every poll point */
    {
        int diffs = 0, aborts = 0;
        for (int k = 0; k < 20000; ++k) {
            const buck_toc_req_t rq = (k & 1) ? step_req() : rand_req(0U);
            buck_toc_plan_t a, b2;
            buck_toc_planner_t pw;
            memset(&a, 0xA5, sizeof a); memset(&b2, 0x5A, sizeof b2); memset(&pw, 0x3C, sizeof pw);
            buck_toc_plan_fast(&rq, &a);
            int32_t st = buck_toc_fast_1(&pw, &rq, &b2);
            if (st == BUCK_TOC_BUSY) st = buck_toc_fast_2(&pw, &rq, &b2);
            if (st == BUCK_TOC_BUSY) st = buck_toc_fast_3(&pw, &b2);
            if (st == BUCK_TOC_BUSY) st = buck_toc_fast_4(&pw, &b2);
            if (st == BUCK_TOC_BUSY) st = buck_toc_fast_5(&pw, &rq, &b2);
            if (st == BUCK_TOC_BUSY) st = buck_toc_fast_6(&rq, &b2);
            diffs += (memcmp(&a, &b2, sizeof a) != 0) || st != a.status;
            static volatile uint32_t flag = 1U;
            struct { int n, stop_at; } cx = { 0, 1 + k % 6 };
            const buck_toc_poll_t poll = { &flag, 1U, svc_count, &cx };
            buck_toc_plan_fast_poll(&rq, &b2, &poll);
            aborts += (b2.status == BUCK_TOC_ABORT) || (cx.n < cx.stop_at);
        }
        CHECK(diffs == 0 && aborts == 20000, "2b: parts vs polled form: %d differences, %d aborts", diffs, aborts);
        printf("cheaper planner in six parts = the polled form (20000, bitwise); an abort at each poll point honoured\n");
    }
    /* the frame's model in parts (run_a1, run_a2, run_b) = stepping; the cheap ADC2 model value */
    {
        double wm = 0;
        long n1s[5] = { 0, 0, 0, 0, 0 };
        for (int k = 0; k < 4000; ++k) {
            const buck_toc_plan_t ep = ready_plan((uint32_t)(k & 1));
            const buck_xy_t x0 = { ep.x0.i + randf(-0.1, 0.1), ep.x0.v + randf(-0.005, 0.005) };
            buck_toc_frame_t fs, fp;
            buck_toc_frame_start(&fs, &ep, x0);
            for (int j = 0; j < 6; ++j) (void)buck_toc_frame_step(&fs);
            buck_toc_frame_start(&fp, &ep, x0);
            buck_toc_frame_run_a1(&fp);
            buck_toc_frame_run_a2(&fp);
            CHECK(buck_toc_frame_run_b(&fp) == BUCK_LEARN_OK, "2b: frame parts status");
            CHECK(memcmp(&fp.mid, &fs.mid, sizeof fs.mid) == 0 && memcmp(&fp.xd, &fs.xd, sizeof fs.xd) == 0 &&
                  fp.q1 == fs.q1 && fp.q2 == fs.q2 && fp.inv_h1 == fs.inv_h1 && fp.inv_h2 == fs.inv_h2 && fp.inv_h == fs.inv_h,
                  "2b: frame parts = stepping");
            /* the encoder's split of the four instants */
            uint32_t n1 = 0, s1 = 0, s2 = 0;
            const uint32_t first = (ep.order == BUCK_TOC_ON_OFF) ? ep.on_ticks : ep.off_ticks;
            for (int j = 0; j < 4; ++j) {
                const uint32_t tm = ep.fw.mcmp[j] - BUCK_DT_TICKS;
                if (tm <= first) { ++n1; s1 += tm; } else s2 += tm;
            }
            CHECK(ep.fw.n1 == n1 && ep.fw.sum1 == s1 && ep.fw.sum2 == s2, "2b: encoder split");
            ++n1s[n1 <= 4 ? n1 : 4];
            const float a = buck_toc_frame_meas(&fs, ep.fw.mcmp), b = buck_toc_frame_meas_w(&fp, &ep.fw);
            const double e = fabs((double)a - b) / (1.0 + fabs((double)a));
            wm = fmax(wm, e);
            CHECK(e < 2e-5, "2b: cheap ADC2 model value %g vs %g", (double)b, (double)a);
        }
        printf("frame model in parts = stepping (4000, bitwise); cheap ADC2 model value vs buck_toc_frame_meas worst %.2g\n"
               "  (instants in the first span: 0:%ld 1:%ld 2:%ld 3:%ld 4:%ld)\n", wm, n1s[0], n1s[1], n1s[2], n1s[3], n1s[4]);
    }
}

int main(void)
{
    test_span();
    test_planner();
    test_limits();
    test_chunks();
    test_learning();
    test_nonfinite();
    test_stage2();
    test_stage2b();
    printf("\n%s (%d failures)\n", failures ? "TOC TESTS FAILED" : "all TOC tests passed", failures);
    return failures ? 1 : 0;
}
