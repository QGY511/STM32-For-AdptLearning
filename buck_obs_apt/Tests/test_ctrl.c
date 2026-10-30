/* Host tests for Inc/buck_ctrl.h (the exact code the firmware runs).
 *
 * Stage 4 (30 Oct 2026): the Padé estimator is gone (stage 3); its sections
 * (the coefficient table, prediction and correction against the specification,
 * the valley-error dynamics, the closed loop's direct form) went with it.
 * 1. The trigger score a e_v^2 + b e_v + c (user, 25 Oct) against the formula
 *    in double; the reference ramp.
 * 2. The load-current observer (user, 25 Oct): its step against the recursion
 *    in double and the two first-order stages it equals; the reload check of
 *    its gains against an exact reference of the stable region; its seed.
 * 3. Closed loop: a switched synchronous buck (L, r_L, C, dead time with
 *    body-diode conduction, 650 kHz current-sense filter, 12-bit ADCs, the
 *    firmware's sampling instants and 2-cycle pipeline) driven by the same
 *    controller code: soft-start, 1 A -> 3 A -> 1 A load steps, the observer
 *    driving A2.
 *
 * Build and run: make test */

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "buck_ctrl.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

static int same_bits(float a, float b) { uint32_t x, y; memcpy(&x, &a, 4); memcpy(&y, &b, 4); return x == y; }

/* ---- 2b. reference ramp --------------------------------------------------------
 * The references are the two forms the firmware ran before, with their
 * vref_reached gate (the ramp code ran only while vref_counts != vref_reached,
 * and vref_reached was cleared while the reference moved, so a target changed
 * back to its previous value mid-ramp is still reached):
 *   ramp_ifelse  the original if/else form,
 *   ramp_clamp   the clamped-difference form that replaced it (same values).
 * The firmware now picks one step per epoch of BUCK_TELEMETRY_EVERY cycles
 * (buck_ramp_mode at each reload of the tunables, buck_ramp_step every cycle).
 * Checked cycle by cycle, bit for bit: up, down, a target changed at a reload
 * in the middle of a ramp (also back to the value reached before), from
 * arm-like and arbitrary starts to every V_ref, at every epoch phase; each ramp
 * must arrive exactly on the target within |dv|/step + 3 steps of the change
 * and be in HOLD from the next reload on. */
typedef struct { float vref; uint32_t reached; } ramp_ref_t;

static void ramp_ifelse(uint32_t counts, ramp_ref_t *s)
{
    if (counts == s->reached) return;
    const float target = (float)counts * (float)BUCK_V_PER_COUNT;
    float r = s->vref;
    if (r < target) { r += BUCK_SOFTSTART_STEP_V; if (r >= target) r = target; }
    else if (r > target) { r -= BUCK_SOFTSTART_STEP_V; if (r <= target) r = target; }
    s->vref = r;
    s->reached = (r == target) ? counts : ~0U;
}

static void ramp_clamp(uint32_t counts, ramp_ref_t *s)
{
    if (counts == s->reached) return;
    const float target = (float)counts * (float)BUCK_V_PER_COUNT;
    float d = target - s->vref;
    d = (d > BUCK_SOFTSTART_STEP_V) ? BUCK_SOFTSTART_STEP_V : d;
    d = (d < -BUCK_SOFTSTART_STEP_V) ? -BUCK_SOFTSTART_STEP_V : d;
    const float r = s->vref + d;
    s->vref = r;
    s->reached = (r == target) ? counts : ~0U;
}

/* Steps allowed for a ramp over dv: |dv|/step, plus float rounding of the
 * accumulated steps (well under 1 %), plus the landing step. */
static uint32_t ramp_limit(float dv)
{
    return (uint32_t)(fabsf(dv) / (float)BUCK_SOFTSTART_STEP_V * 1.01f) + 3U;
}

static long ramp_cycles, ramp_free_cycles;

/* The firmware's loop from its entry (loops = phase) with the tunables' target
 * counts0; at reload number switch_at the tunables read counts1, at reload
 * number back_at counts2. Returns 0 when every cycle matched both references
 * and the ramp ended on the last target (in HOLD) within the step limit. */
static int ramp_run(float vref0, uint32_t reached0, uint32_t phase, uint32_t counts0,
                    uint32_t switch_at, uint32_t counts1, uint32_t back_at, uint32_t counts2)
{
    ramp_ref_t a = { vref0, reached0 }, c = a;
    uint32_t counts = counts0, loops = phase, reloads = 0;
    float vref = vref0, target = (float)counts * (float)BUCK_V_PER_COUNT;
    buck_tune_t t = BUCK_TUNE_DEFAULTS;
    t.vref_counts = counts;
    CHECK(buck_ramp_target(&t) == target, "ramp target");
    uint32_t mode = buck_ramp_mode(vref, target);
    uint32_t limit = ramp_limit(target - vref);
    const uint32_t last = (back_at != ~0U) ? back_at : (switch_at != ~0U) ? switch_at : 0U;   /* last target change */
    for (uint32_t n = 0; ; ++n) {
        /* one cycle: the step, then (every BUCK_TELEMETRY_EVERY) the reload */
        ramp_ifelse(counts, &a);
        ramp_clamp(counts, &c);
        vref = buck_ramp_step(mode, vref, target, BUCK_SOFTSTART_STEP_V);
        ++ramp_cycles; ramp_free_cycles += mode == BUCK_RAMP_FREE_UP;
        if (vref != a.vref || vref != c.vref || a.reached != c.reached) return 1;
        if (((++loops) & (BUCK_TELEMETRY_EVERY - 1U)) == 0U) {
            ++reloads;
            if (reloads == switch_at || reloads == back_at) {
                counts = (reloads == switch_at) ? counts1 : counts2;
                limit = n + ramp_limit((float)counts * (float)BUCK_V_PER_COUNT - vref);
            }
            t.vref_counts = counts;
            target = buck_ramp_target(&t);
            mode = buck_ramp_mode(vref, target);
            /* finished: on the target, in HOLD, and no reload still to come */
            if (mode == BUCK_RAMP_HOLD && reloads >= last) break;
            if (mode == BUCK_RAMP_HOLD && vref != target) return 4;
        }
        if (n > limit && vref != target) return 2;           /* not there in time */
        if (n > limit + 2U * BUCK_TELEMETRY_EVERY && reloads >= last) return 5;   /* there, but not in HOLD */
    }
    return (vref == (float)counts * (float)BUCK_V_PER_COUNT && a.reached == counts) ? 0 : 3;
}

/* One FREE_UP epoch against the limited up step, for (vref, target) pairs at
 * the edge of the FREE_UP condition and up to BUCK_RAMP_FREE_MAX_V. */
static long ramp_free_edge(void)
{
    long bad = 0, n = 0;
    srand(11);
    for (long i = 0; i < 400000; ++i) {
        const float target = BUCK_RAMP_FREE_MAX_V * (float)rand() / (float)RAND_MAX;
        /* vref from just inside the FREE_UP bound to a few steps below it */
        float vref = target - BUCK_RAMP_FAR_V - (float)(rand() % 64) * 1e-7f * (float)(1 + (i & 7));
        if (i & 1) vref = nextafterf(target - BUCK_RAMP_FAR_V, 0.0f);
        if (vref < 0.0f) continue;
        for (int k = 0; k < 64 && buck_ramp_mode(vref, target) != BUCK_RAMP_FREE_UP; ++k) vref = nextafterf(vref, 0.0f);
        if (buck_ramp_mode(vref, target) != BUCK_RAMP_FREE_UP) continue;
        ++n;
        float x = vref, y = vref;
        for (uint32_t k = 0; k < BUCK_TELEMETRY_EVERY; ++k) {
            x = buck_ramp_step(BUCK_RAMP_FREE_UP, x, target, BUCK_SOFTSTART_STEP_V);
            y = buck_ramp_step(BUCK_RAMP_UP, y, target, BUCK_SOFTSTART_STEP_V);
            if (x != y || x >= target) { ++bad; break; }
        }
    }
    /* The bound holds only where one float addition rounds by at most 2^-20 V: FREE_UP must be off
     * for targets at and above BUCK_RAMP_FREE_MAX_V, however far below them vref is. */
    long above = 0;
    for (uint32_t k = 0; k < 4000U; ++k) {
        const float target = BUCK_RAMP_FREE_MAX_V * (1.0f + (float)k * 0.25f);
        above += buck_ramp_mode(0.0f, target) == BUCK_RAMP_FREE_UP;
        above += buck_ramp_mode(0.5f * target, target) == BUCK_RAMP_FREE_UP;
    }
    printf("FREE_UP epochs at the edge of the bound (targets 0..%.0f V): %ld, %ld differ from the limited step; "
           "%ld FREE_UP choices at or above %.0f V\n", (double)BUCK_RAMP_FREE_MAX_V, n, bad, above, (double)BUCK_RAMP_FREE_MAX_V);
    return n > 100000 ? bad + above : bad + above + 1;
}

static void test_ramp(void)
{
    int bad = 0, runs = 0;
    for (uint32_t start = 0; start < 4096U; start += 7U) {
        const float v = (float)start * (float)BUCK_V_PER_COUNT;
        for (uint32_t target = 0; target < 4096U; target += 211U) {
            const uint32_t ph = (start * 13U + target) & (BUCK_TELEMETRY_EVERY - 1U);
            const float tv = (float)target * (float)BUCK_V_PER_COUNT;
            /* arm-like start (vref = min(v_out, target)), as buck_ctrl_arm() */
            const float va = (v < tv) ? v : tv;
            bad += ramp_run(va, (va == tv) ? target : ~0U, ph, target, ~0U, 0U, ~0U, 0U) != 0; ++runs;
            /* from v_out, up or down, to the target */
            bad += ramp_run(v, ~0U, ph, target, ~0U, 0U, ~0U, 0U) != 0; ++runs;
            /* away, and retargeted in the middle to a third value */
            bad += ramp_run(tv, target, ph, 4095U - target, 1U + start % 7U, (target * 3U) % 4096U, ~0U, 0U) != 0; ++runs;
            /* away, and back to the value reached before, in the middle of the ramp */
            bad += ramp_run(tv, target, ph ^ 21U, 4095U - target, 1U + start % 5U, target, ~0U, 0U) != 0; ++runs;
            /* at the target, away at a reload, back before arriving */
            bad += ramp_run(tv, target, ph, target, 1U, (target + 1500U) % 4096U, 2U + start % 11U, target) != 0; ++runs;
        }
    }
    /* targets far outside the ADC range: FREE_UP is off above BUCK_RAMP_FREE_MAX_V */
    for (uint32_t k = 0; k < 40; ++k) {
        const uint32_t target = 4096U + k * 997U;          /* 3.3 .. 35 V */
        bad += ramp_run((float)(k * 83U) * (float)BUCK_V_PER_COUNT, ~0U, k, target, ~0U, 0U, ~0U, 0U) != 0; ++runs;
        bad += ramp_run((float)target * (float)BUCK_V_PER_COUNT, target, k, k * 50U, ~0U, 0U, ~0U, 0U) != 0; ++runs;
    }
    printf("reference ramp (firmware epochs) vs the original if/else and clamp forms: %d runs, %ld cycles (%.0f %% FREE_UP), "
           "%d mismatches or unfinished\n", runs, ramp_cycles, 100.0 * (double)ramp_free_cycles / (double)ramp_cycles, bad);
    CHECK(bad == 0, "reference ramp");
    CHECK(ramp_free_edge() == 0, "reference ramp: FREE_UP bound");
}

/* ---- 2d. trigger score and indicator (user, 25 Oct) ----------------------------
 * S = a e_v^2 + b e_v + c, e_v = v_out - V_ref:
 * - buck_trigger_score_k() against the formula in double over random a, b, c
 *   and e_v of both signs and many magnitudes (zeros included, S < 0 among
 *   them), relative to |a| e_v^2 + |b e_v| + |c| (the size of the terms: S
 *   itself can cancel to 0);
 * - at the default a, b, c: bit-identical to e_v * e_v for every sign and
 *   magnitude (zeros, subnormals, overflow to +inf);
 * - NaN, infinite and negative scores through buck_ctrl_slow(): NaN -> D4 low;
 *   +inf -> D4 high; negative (finite or -inf) -> D4 low while S_th > S;
 * - through buck_ctrl_slow() with random tunables: g = (S >= S_th) with the
 *   published S, which matches the formula; g, S and the controller state the
 *   same bits with other observer gains (the score and D4 do not read them). */
static double urand(void) { return rand() / (double)RAND_MAX; }
static float logu(double lo, double hi) { return (float)exp(log(lo) + (log(hi) - log(lo)) * urand()); }
static float slogu(double lo, double hi) { const float x = logu(lo, hi); return (rand() & 1) ? -x : x; }

/* one buck_ctrl_slow() with v, V_ref and the tunables: returns g, the published S in *S */
static uint32_t slow_once(const buck_tune_t *t, float v, float vref, float *S, buck_state_t *so)
{
    buck_state_t s;
    memset(&s, 0, sizeof s);
    s.vref = vref; s.cmp_active = BUCK_CMP_MIN + 700U;
    const buck_step_t st = { .v = v, .im = 1.0f, .iref = 1.0f, .cmp = BUCK_CMP_MIN + 900U };
    buck_out_t o;
    const uint32_t g = buck_ctrl_slow(t, &s, &st, &o);
    *S = o.score;
    if (so) *so = s;
    return g;
}

static void test_score(void)
{
    /* the formula, in double */
    double worst = 0.0;
    long neg = 0;
    srand(3);
    for (int n = 0; n < 2000000; ++n) {
        buck_tune_t t = BUCK_TUNE_DEFAULTS;
        const int ka = rand() % 8;
        t.score_a = (ka == 0) ? 0.0f : (ka == 1) ? 1.0f : slogu(1e-3, 1e4);
        t.score_b = (rand() % 4 == 0) ? 0.0f : slogu(1e-3, 1e3);
        t.score_c = (rand() % 4 == 0) ? 0.0f : slogu(1e-4, 1e2);
        const int ke = rand() % 16;
        const float ev = (ke == 0) ? 0.0f : (ke == 1) ? -0.0f : slogu(1e-6, 3.3);
        const float S = buck_trigger_score_k(ev, buck_score_k(&t));
        const double a = t.score_a, b = t.score_b, c = t.score_c, e = ev;
        const double ref = a * e * e + b * e + c, scale = fabs(a) * e * e + fabs(b * e) + fabs(c);
        worst = fmax(worst, (scale > 0.0) ? fabs((double)S - ref) / scale : fabs((double)S) * 1e30);
        neg += S < 0.0f;
    }
    /* the defaults: e_v^2 bit for bit */
    long def_bad = 0, def_n = 0;
    {
        const buck_tune_t td = BUCK_TUNE_DEFAULTS;
        const buck_score_k_t kd = buck_score_k(&td);
        static const float evs[] = { 0.0f, -0.0f, 1e-45f, -1e-45f, FLT_MIN, -FLT_MIN, 1e-20f, 0.1f, -0.1f, 1.0f, -1.0f,
                                     3.3f, -3.3f, 1.8e19f, -1.8e19f, 2e19f, -2e19f, FLT_MAX, -FLT_MAX, INFINITY, -INFINITY };
        for (unsigned i = 0; i < sizeof evs / sizeof evs[0]; ++i, ++def_n)
            def_bad += !same_bits(buck_trigger_score_k(evs[i], kd), evs[i] * evs[i]);
        for (int n = 0; n < 1000000; ++n, ++def_n) {
            const float ev = slogu(1e-30, 1e19);
            def_bad += !same_bits(buck_trigger_score_k(ev, kd), ev * ev);
        }
        def_bad += !isnan(buck_trigger_score_k(NAN, kd)); ++def_n;
    }
    /* NaN, infinities, negative S: D4 (V_ref = 1 V; e_v exact) */
    const float I = INFINITY;
    const struct { float a, b, c, ev, sth; int cls; uint32_t g; } sp[] = {
        /* cls: 0 NaN, 1 +inf, 2 -inf, 3 finite < 0, 4 finite >= 0 */
        { NAN, 0, 0, 0.125f, 1, 0, 0 }, { 1, NAN, 0, 0.125f, 1, 0, 0 }, { 1, 0, NAN, 0.125f, 1, 0, 0 },
        { I, 0, 0, 0.125f, 1, 1, 1 }, { I, 0, 0, -0.125f, 1, 1, 1 }, { I, 0, 0, 0.0f, 1, 0, 0 },
        { -I, 0, 0, 0.125f, 1, 2, 0 }, { 1, I, 0, 0.125f, 1, 1, 1 }, { 1, I, 0, -0.125f, 1, 2, 0 },
        { 1, I, 0, 0.0f, 1, 0, 0 }, { 1, 0, I, 0.125f, 1, 1, 1 }, { 1, 0, -I, 0.125f, 1, 2, 0 },
        { 3e38f, 0, 0, 2.0f, 1, 1, 1 },                             /* overflow */
        { 1, 0, -0.5f, 0.125f, 1, 3, 0 }, { 0, 8, 0, -0.125f, 1, 3, 0 }, { -1, 0, 0, 0.5f, 1, 3, 0 },
        { 1, 0, -0.5f, 0.125f, -1, 3, 1 },                          /* S_th < S < 0: D4 high (g = S >= S_th) */
        { 0, 8, 0, 0.125f, 1, 4, 1 }, { 1, 0, 2, 0.0f, 1, 4, 1 }, { 1, 0, 0.6f, 0.0f, 1, 4, 0 },
        { 1, 0, 0, 1.0f, 1, 4, 1 }, { 1, 0, 0, -1.0f, 1, 4, 1 },
        { 1, 0, 0, 0.5f, 1, 4, 0 },
    };
    int sp_bad = 0;
    for (unsigned i = 0; i < sizeof sp / sizeof sp[0]; ++i) {
        buck_tune_t t = BUCK_TUNE_DEFAULTS;
        t.score_a = sp[i].a; t.score_b = sp[i].b; t.score_c = sp[i].c; t.s_th = sp[i].sth;
        float S;
        const uint32_t g = slow_once(&t, 1.0f + sp[i].ev, 1.0f, &S, NULL);
        const int cls = isnan(S) ? 0 : (S == INFINITY) ? 1 : (S == -INFINITY) ? 2 : (S < 0.0f) ? 3 : 4;
        if (cls != sp[i].cls || g != sp[i].g) {
            ++sp_bad;
            printf("  score a = %g, b = %g, c = %g, e_v = %g, S_th = %g: S %g, g %u (expected class %d, g %u)\n",
                   (double)sp[i].a, (double)sp[i].b, (double)sp[i].c, (double)sp[i].ev, (double)sp[i].sth,
                   (double)S, (unsigned)g, sp[i].cls, (unsigned)sp[i].g);
        }
    }
    /* D4 follows S_th; the observer's gains change nothing here */
    long g_bad = 0, g_n = 0, g_on = 0, sep_bad = 0;
    double worst_g = 0.0;
    srand(5);
    for (int n = 0; n < 300000; ++n) {
        buck_tune_t t = BUCK_TUNE_DEFAULTS;
        t.score_a = (rand() % 4 == 0) ? 1.0f : slogu(1e-2, 1e3);
        t.score_b = (rand() % 2 == 0) ? 0.0f : slogu(1e-3, 1e2);
        t.score_c = (rand() % 2 == 0) ? 0.0f : slogu(1e-4, 1e1);
        t.s_th = slogu(1e-4, 1e2);
        const float v = (float)(3.3 * urand()), vref = (float)(3.3 * urand());
        float S, S2;
        buck_state_t s1, s2;
        const uint32_t g = slow_once(&t, v, vref, &S, &s1);
        g_bad += g != ((S >= t.s_th) ? 1U : 0U);
        g_on += g; ++g_n;
        const double e = (double)v - (double)vref, a = t.score_a, b = t.score_b, c = t.score_c;
        worst_g = fmax(worst_g, fabs((double)S - (a * e * e + b * e + c)) / (fabs(a) * e * e + fabs(b * e) + fabs(c)));
        buck_tune_t t2 = t;
        t2.obs_beta = slogu(1e-3, 3.0); t2.obs_gamma = slogu(1e-3, 1e3);   /* other observer gains, inside or not */
        const uint32_t g2 = slow_once(&t2, v, vref, &S2, &s2);
        sep_bad += g2 != g || !same_bits(S2, S) || memcmp(&s1, &s2, sizeof s1) != 0;
    }
    printf("trigger score a e_v^2 + b e_v + c: within %.1e of the double reference relative to |a| e_v^2 + |b e_v| + |c| "
           "(2000000 random a, b, c, e_v; %ld with S < 0); %ld of %ld not bit-identical to e_v^2 at the defaults; "
           "%d of %u NaN/inf/negative cases wrong\n", worst, neg, def_bad, def_n, sp_bad, (unsigned)(sizeof sp / sizeof sp[0]));
    printf("D4 threshold: g = (S >= S_th) wrong in %ld of %ld passes (%ld with g = 1; S within %.1e); "
           "%ld changed by other observer gains\n", g_bad, g_n, g_on, worst_g, sep_bad);
    CHECK(worst < 3e-7 && neg > 100000 && def_bad == 0 && sp_bad == 0, "trigger score");
    /* (S within 5 roundings here: e_v = v - V_ref is rounded too) */
    CHECK(g_bad == 0 && g_on > 1000 && g_on < g_n - 1000 && worst_g < 4e-7 && sep_bad == 0,
          "D4 follows S_th; the observer's gains do not reach the score, D4 or the controller");
}

/* ---- 2e. load current: the observer (user, 25 Oct) ------------------------------
 * - buck_obs_step() one step at a time against the same step in double from
 *   the same state (a, beta, gamma: the firmware's floats), over random states,
 *   inputs and gains inside the stable region (double and complex poles);
 * - the identity: from the seed (v^ = v[0], i^ = i_L[0]) the recursion with
 *   beta = 1 - p^2, gamma = (1 - p)^2 / a equals two first-order stages
 *   y += (1 - p)(Î - y) on Î[k] = i_L[k] - (v[k] - v[k-1]) / a, both in double,
 *   for several p; the firmware's float observer with the default gains against
 *   those stages (p = exp(-T/9.2 us)) over random input walks;
 * - a against T/C (1/162 V/A) and the default gains against 1 - p^2 and
 *   (1 - p)^2 C/T with p from libm's exp (rounded to float: the same bits);
 * - buck_iload_obs(): the estimate and the A2 code floor(y DAC_PER_A) clipped
 *   to 0..4095;
 * - buck_obs_seed() after buck_ctrl_arm(): v^ = the arming pass's v, i^ = its
 *   i_meas (= out.i_load), both against the ADC sums converted in double. */
typedef struct { double vh, ih; } obs_d_t;

static double obs_d_step(obs_d_t *x, double v, double il, double a, double beta, double gamma)
{
    const double vp = x->vh + a * (il - x->ih);     /* predict */
    const double e = v - vp;                        /* correct */
    x->vh = vp + beta * e;
    x->ih = x->ih - gamma * e;
    return x->ih;
}

/* one cycle of a random input walk: v near 0.8 V with +-2 counts of noise and
 * rare 0.2 V jumps, i_L in 0..9 A with 50 mA of noise and rare jumps */
static void obs_walk(double *vm, double *im, float *v, float *il)
{
    if (rand() % 400 == 0) *vm = 0.6 + 0.4 * urand();
    if (rand() % 300 == 0) *im = 9.0 * urand();
    *v = (float)(*vm + BUCK_V_PER_COUNT * ((rand() % 5) - 2));
    *il = (float)(*im + 0.05 * (2.0 * urand() - 1.0));
}

static void test_obs(void)
{
    const double T = BUCK_TSW_S, a = (double)BUCK_OBS_A;
    /* the defaults, from the user's tau = 9.2 us written here (not BUCK_OBS_TAU_S), and against the decided
     * digits: beta = 0.3039396; gamma = 4.447826 to 7 digits (4.4478256; the float is 4.4478254) */
    const double p = exp(-T / 9.2e-6);
    const float beta_ref = (float)(1.0 - p * p), gamma_ref = (float)((1.0 - p) * (1.0 - p) * (BUCK_C_F / BUCK_TSW_S));
    const int def_bad = !same_bits(BUCK_OBS_BETA_DEFAULT, beta_ref) || !same_bits(BUCK_OBS_GAMMA_DEFAULT, gamma_ref) ||
                        fabs((double)BUCK_OBS_BETA_DEFAULT - 0.3039396) > 1e-7 ||
                        fabs((double)BUCK_OBS_GAMMA_DEFAULT - 4.447826) > 1e-6 ||
                        !same_bits(BUCK_OBS_A, (float)(1.0 / 162.0)) || !same_bits(BUCK_OBS_A, (float)(T / BUCK_C_F));
    /* one step against double */
    srand(41);
    double step_worst = 0.0;
    long step_n = 0, out_of_region = 0;
    for (int n = 0; n < 2000000; ++n) {
        const double r = 0.3 + 0.69 * urand(), th = (rand() % 3 == 0) ? 0.0 : M_PI * urand();
        const float fb = (float)(1.0 - r * r), fg = (float)((1.0 + r * r - 2.0 * r * cos(th)) / a);
        out_of_region += !buck_obs_gains_ok(fb, fg);
        const float vh = (float)(0.2 + 1.2 * urand()), ih = (float)(-2.0 + 12.0 * urand());
        const float v = (float)(0.2 + 1.2 * urand()), il = (float)(-2.0 + 12.0 * urand());
        buck_obs_t x = { vh, ih };
        obs_d_t xd = { vh, ih };
        const buck_obs_k_t k = { fb, fg };
        const float y = buck_obs_step(&x, v, il, k);
        const double yd = obs_d_step(&xd, v, il, a, fb, fg);
        /* a few roundings of |i^|, |i_L| and gamma |v| (e = v - v^ cancels) */
        const double scale = 1e-6 * (1.0 + fabs(yd) + (double)fg * 2.0);
        step_worst = fmax(step_worst, fmax(fabs((double)y - yd), fabs((double)x.vh - xd.vh)) / scale);
        ++step_n;
    }
    /* the identity, in double, and the firmware's observer against it */
    static const double taus[] = { 4e-6, BUCK_OBS_TAU_S, 16e-6, 33.32e-6 };
    double id_worst = 0.0, fw_worst = 0.0;
    long a2_bad = 0;
    srand(43);
    for (unsigned it = 0; it < sizeof taus / sizeof taus[0]; ++it) {
        const double pp = exp(-T / taus[it]), bd = 1.0 - pp * pp, gd = (1.0 - pp) * (1.0 - pp) / a;
        for (int run = 0; run < 50; ++run) {
            double vm = 0.8, im = 9.0 * urand();
            float v, il;
            obs_walk(&vm, &im, &v, &il);
            obs_d_t xd = { v, il };
            buck_obs_t x = { v, il };
            double y1 = il, y2 = il, vprev = v;
            for (int n = 0; n < 4000; ++n) {
                obs_walk(&vm, &im, &v, &il);
                const double yd = obs_d_step(&xd, v, il, a, bd, gd);
                const double Ihat = (double)il - ((double)v - vprev) / a;
                vprev = v;
                y1 += (1.0 - pp) * (Ihat - y1);
                y2 += (1.0 - pp) * (y1 - y2);
                id_worst = fmax(id_worst, fabs(yd - y2));
                if (taus[it] == BUCK_OBS_TAU_S) {
                    volatile uint32_t a2 = ~0U;
                    const float y = buck_iload_obs(&x, v, il, BUCK_OBS_K_DEFAULTS, &a2);
                    fw_worst = fmax(fw_worst, fabs((double)y - y2));
                    const double pr = (double)y * (4095.0 / BUCK_DAC_FULL_SCALE_A), ref = fmin(4095.0, fmax(0.0, floor(pr)));
                    a2_bad += !((double)a2 == ref || (fabs(pr - floor(pr + 0.5)) < 1e-6 * fmax(1.0, pr) && fabs((double)a2 - ref) <= 1.0));
                }
            }
        }
    }
    /* the A2 code over the whole range and beyond (negative: 0, above 10 A: 4095) */
    srand(47);
    for (int n = 0; n < 1000000; ++n) {
        buck_obs_t x = { 0.8f, (float)(-3.0 + 16.0 * urand()) };
        volatile uint32_t a2 = ~0U;
        const float y = buck_iload_obs(&x, 0.8f, x.ih, BUCK_OBS_K_DEFAULTS, &a2);   /* e = 0: y = i^ */
        const double pr = (double)y * (4095.0 / BUCK_DAC_FULL_SCALE_A), ref = fmin(4095.0, fmax(0.0, floor(pr)));
        a2_bad += !((double)a2 == ref || (fabs(pr - floor(pr + 0.5)) < 1e-6 * fmax(1.0, pr) && fabs((double)a2 - ref) <= 1.0));
    }
    /* the seed: from the arming pass */
    long seed_bad = 0;
    srand(53);
    for (int n = 0; n < 200000; ++n) {
        buck_tune_t t = BUCK_TUNE_DEFAULTS;
        if (n & 1) { t.isense_zero = (float)(20.0 * urand()); t.isense_gain = (float)(0.015 + 0.01 * urand()); }
        const uint32_t adc_v = (uint32_t)rand() % 8191U, adc_i4 = (uint32_t)rand() % 16381U;
        buck_state_t s;
        buck_out_t o;
        memset(&s, 0, sizeof s);
        (void)buck_ctrl_arm(&t, &s, adc_v, adc_i4, &o);
        const buck_obs_t x = buck_obs_seed(&o);
        const double v = adc_v * BUCK_V_PER_VSUM, im = (adc_i4 / 4.0 - (double)t.isense_zero) * (double)t.isense_gain;
        seed_bad += !same_bits(x.vh, o.v) || !same_bits(x.ih, o.i_meas) || !same_bits(o.i_load, o.i_meas) ||
                    fabs((double)x.vh - v) > 1e-6 * fmax(1.0, v) || fabs((double)x.ih - im) > 1e-6 * fmax(1.0, fabs(im));
    }
    printf("load-current observer: a %.7f V/A, defaults beta %.7f, gamma %.7f A/V (%s T/C, 1 - p^2, (1 - p)^2 C/T, tau 9.2 us); one step within %.2f x "
           "1e-6 (1 + |i^| + 2 gamma) of double over %ld random states and stable gains (%ld outside the region); "
           "recursion = two first-order stages within %.1e A (double, 4 poles), firmware within %.1e A of the stages; "
           "%ld A2 codes wrong; %ld seeds wrong of 200000\n", (double)BUCK_OBS_A, (double)BUCK_OBS_BETA_DEFAULT, (double)BUCK_OBS_GAMMA_DEFAULT,
           def_bad ? "NOT" : "=", step_worst, step_n, out_of_region, id_worst, fw_worst, a2_bad, seed_bad);
    CHECK(def_bad == 0 && step_worst < 1.0 && out_of_region == 0 && id_worst < 1e-9 && fw_worst < 1e-4 && a2_bad == 0 && seed_bad == 0,
          "load-current observer");
}

/* ---- 2f. the observer's gains: reload check (user, 25 Oct) -----------------------
 * Stable region (predict-then-correct, Inc/buck_config.h): gamma > 0 and
 * 0 < beta < 2 - a gamma / 2, open. buck_obs_gains_ok() against an exact
 * reference in double (h gamma, h = a/2, is exact in double: 21 + 24 bits;
 * 2 - beta or 2 - h gamma is exact where it matters, and the rounded
 * difference keeps its sign), over: every pair of special values (NaN of both
 * signs, infinities, zeros, subnormals, FLT_MAX, 2 and its neighbours, ...);
 * random bit patterns; random positive pairs of every size; the floats next to
 * the boundary line on both sides (from random gamma and from random beta);
 * and the points exactly on it (gamma = m 2^k, m odd < 2^18, with 2 - h gamma
 * a float: rejected; the float below accepted, the one above rejected). Also:
 * buck_obs_prod_err() against the exact rounding error for every mantissa at
 * the extreme exponents; the region against the spectral radius of the error
 * dynamics in double (away from the boundary); the counterexample to
 * 1 - a gamma > -1; buck_obs_k(): the tunables' pair when inside, otherwise the
 * previous gains bit for bit, the compiled defaults at the loop entry, over
 * random reload sequences. */
static int ref_inside(float beta, float gamma)
{
    if (!(beta > 0.0f && gamma > 0.0f) || isinf(beta) || isinf(gamma)) return 0;     /* also a NaN */
    const double b = beta, q = (double)(0.5f * BUCK_OBS_A) * (double)gamma;          /* exact */
    if (b >= 4.0 || q >= 4.0) return 0;
    if (b >= 1.0) return (2.0 - b) - q > 0.0;       /* 2 - b exact (Sterbenz); a rounded difference keeps its sign */
    if (q >= 1.0) return (2.0 - q) - b > 0.0;       /* 2 - q exact */
    return 1;                                       /* b < 1, q < 1 */
}

static float bits_float(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }
static uint32_t rand32(void) { return ((uint32_t)rand() << 17) ^ ((uint32_t)rand() << 3) ^ (uint32_t)rand(); }

static long gains_n, gains_bad, gains_in;
static void gains_pair(float beta, float gamma)
{
    const uint32_t got = buck_obs_gains_ok(beta, gamma);
    const int want = ref_inside(beta, gamma);
    ++gains_n; gains_in += want;
    if (got != (uint32_t)want) {
        if (++gains_bad <= 10) printf("  gains beta = %.9g, gamma = %.9g: %u, expected %d\n", (double)beta, (double)gamma, (unsigned)got, want);
    }
}

static void test_obs_gains(void)
{
    const float h = 0.5f * BUCK_OBS_A;
    /* the rounding error of h gamma, every mantissa at the extreme exponents */
    long err_bad = 0, err_n = 0;
    static const int exps[] = { -96, 0, 9, 127 };
    for (unsigned k = 0; k < sizeof exps / sizeof exps[0]; ++k)
        for (uint32_t m = 0; m < (1U << 23); ++m, ++err_n) {
            const float g = bits_float(((uint32_t)(exps[k] + 127) << 23) | m);
            const float t = h * g;
            err_bad += (double)buck_obs_prod_err(g, t) != (double)h * (double)g - (double)t;
        }
    /* special values, every pair */
    static const float sv[] = { NAN, -NAN, INFINITY, -INFINITY, 0.0f, -0.0f, 1e-45f, -1e-45f, FLT_MIN, -FLT_MIN, 1e-30f,
                                0.25f, 0.5f, 1.0f, 2.0f, -2.0f, 0x1.fffffep0f, 0x1.000002p1f, 3.0f, 4.0f, 0.30393961f,
                                4.4478254f, 324.0f, 648.0f, 0x1.43fffep9f, 1e30f, FLT_MAX, -FLT_MAX, -1.0f, 1e-38f };
    gains_n = gains_bad = gains_in = 0;
    for (unsigned i = 0; i < sizeof sv / sizeof sv[0]; ++i)
        for (unsigned j = 0; j < sizeof sv / sizeof sv[0]; ++j) gains_pair(sv[i], sv[j]);
    const long n_special = gains_n;
    /* random bit patterns, and random positive pairs of every size */
    srand(61);
    for (int n = 0; n < 4000000; ++n) gains_pair(bits_float(rand32()), bits_float(rand32()));
    for (int n = 0; n < 8000000; ++n) {
        const float beta = (rand() % 8 == 0) ? bits_float(rand32() & 0x3FFFFFFFU) : logu(1e-6, 4.0);
        const float gamma = (rand() % 8 == 0) ? bits_float(rand32() & 0x447FFFFFU) : logu(1e-6, 2000.0);
        gains_pair(beta, gamma);
    }
    /* next to the boundary beta = 2 - h gamma: from gamma, and from beta */
    for (int n = 0; n < 4000000; ++n) {
        const float gamma = (rand() & 1) ? (float)(648.0 * urand()) : logu(1e-12, 648.0);
        const float b0 = (float)(2.0 - (double)h * (double)gamma);
        float b = b0;
        for (int k = 0; k < 4; ++k) b = nextafterf(b, 0.0f);
        for (int k = 0; k < 9; ++k, b = nextafterf(b, 4.0f)) gains_pair(b, gamma);
    }
    for (int n = 0; n < 4000000; ++n) {
        const float beta = (rand() & 1) ? (float)(2.0 * urand()) : (float)(2.0 - exp(log(1e-7) + (log(2.0) - log(1e-7)) * urand()));
        const float g0 = (float)((2.0 - (double)beta) / (double)h);
        float g = g0;
        for (int k = 0; k < 4; ++k) g = nextafterf(g, 0.0f);
        for (int k = 0; k < 9; ++k, g = nextafterf(g, 1e4f)) gains_pair(beta, g);
    }
    /* exactly on the boundary: gamma = m 2^k (m odd, small) with 2 - h gamma a float; and its float neighbours */
    long on_line = 0, on_bad = 0;
    for (uint32_t m = 1; m < (1U << 18); m += 2U)
        for (int k = -40; k < 12; ++k) {
            const float gamma = ldexpf((float)m, k);
            const double q = (double)h * (double)gamma;
            if (!(q > 0.0 && q < 2.0)) continue;
            const float b = (float)(2.0 - q);
            if ((double)b != 2.0 - q || !(b > 0.0f)) continue;        /* 2 - h gamma not a float */
            ++on_line;
            on_bad += buck_obs_gains_ok(b, gamma) != 0U;                         /* on the line: outside */
            on_bad += buck_obs_gains_ok(nextafterf(b, 0.0f), gamma) != 1U;       /* below it: inside */
            on_bad += buck_obs_gains_ok(nextafterf(b, 4.0f), gamma) != 0U;       /* above it: outside */
            gains_pair(b, gamma); gains_pair(nextafterf(b, 0.0f), gamma); gains_pair(nextafterf(b, 4.0f), gamma);
        }
    /* the region against the spectral radius of the error dynamics (away from the boundary) */
    long jury_bad = 0, jury_n = 0;
    const double a = (double)BUCK_OBS_A;
    srand(67);
    for (int n = 0; n < 2000000; ++n) {
        const float beta = (float)(-0.5 + 3.0 * urand()), gamma = (float)(-100.0 + 800.0 * urand());
        const double b = beta, g = gamma, tr = 2.0 - b - a * g, det = 1.0 - b, disc = tr * tr - 4.0 * det;
        const double rad = (disc >= 0.0) ? fmax(fabs(0.5 * (tr + sqrt(disc))), fabs(0.5 * (tr - sqrt(disc)))) : sqrt(fmax(det, 0.0));
        if (fabs(rad - 1.0) < 1e-9) continue;
        ++jury_n;
        jury_bad += (rad < 1.0) != (buck_obs_gains_ok(beta, gamma) != 0U);
    }
    /* not needed: 1 - a gamma > -1 (a gamma = 3, beta = 0.25: |z| = 0.866) */
    const int counter_ok = buck_obs_gains_ok(0.25f, (float)(3.0 / a)) == 1U;
    /* the reload: the tunables' pair when inside, the gains in use otherwise, the defaults at the entry */
    long k_bad = 0;
    srand(71);
    for (int run = 0; run < 20000; ++run) {
        buck_tune_t t = BUCK_TUNE_DEFAULTS;
        float rb = BUCK_OBS_BETA_DEFAULT, rg = BUCK_OBS_GAMMA_DEFAULT;     /* the reference: the last pair inside */
        buck_obs_k_t k = BUCK_OBS_K_DEFAULTS;
        for (int n = 0; n < 40; ++n) {
            const int kind = rand() % 6;
            t.obs_beta = (kind == 0) ? NAN : (kind == 1) ? bits_float(rand32()) : (float)(2.2 * urand() - 0.1);
            t.obs_gamma = (kind == 2) ? INFINITY : (kind == 3) ? -(float)(10.0 * urand()) : (float)(700.0 * urand());
            k = (n == 0) ? buck_obs_k(&t, BUCK_OBS_K_DEFAULTS) : buck_obs_k(&t, k);     /* n = 0: the loop entry */
            if (ref_inside(t.obs_beta, t.obs_gamma)) { rb = t.obs_beta; rg = t.obs_gamma; }
            else if (n == 0) { rb = BUCK_OBS_BETA_DEFAULT; rg = BUCK_OBS_GAMMA_DEFAULT; }
            k_bad += !same_bits(k.beta, rb) || !same_bits(k.gamma, rg);
        }
    }
    const buck_tune_t td = BUCK_TUNE_DEFAULTS;
    const int def_inside = buck_obs_gains_ok(td.obs_beta, td.obs_gamma) == 1U;
    printf("observer gain check: rounding error of h gamma exact for %ld of %ld mantissas; %ld pairs (%ld special, %ld inside, "
           "%ld exactly on the boundary line), %ld differ from the exact region; %ld of %ld boundary points or neighbours wrong; "
           "spectral radius < 1 disagrees in %ld of %ld; a gamma = 3, beta = 0.25 %s; %ld reloads wrong; defaults %s\n",
           err_n - err_bad, err_n, gains_n, n_special, gains_in, on_line, gains_bad, on_bad, 3 * on_line, jury_bad, jury_n,
           counter_ok ? "inside" : "OUTSIDE", k_bad, def_inside ? "inside" : "OUTSIDE");
    CHECK(err_bad == 0 && gains_bad == 0 && on_line >= 50 && on_bad == 0 && jury_bad == 0 && counter_ok && k_bad == 0 && def_inside,
          "observer gain check");
}

/* ---- 3. closed loop ---------------------------------------------------------- */
typedef struct {
    double L, rl, C, esr, vin, vd;      /* plant */
    double i, vc, y;                    /* inductor current, capacitor voltage, sensor filter state */
    double R, istep;                    /* load: resistor + current sink */
    int noise;                          /* ADC noise, LSB (uniform +-) */
    int ideal_dt;                       /* 1: switch node at 0 V in the dead time (what the model assumes) */
} plant_t;

static double p_vout(const plant_t *p)
{
    /* v_out = v_c + ESR (i_L - i_o),  i_o = v_out/R + I_step */
    return (p->vc + p->esr * (p->i - p->istep)) / (1.0 + p->esr / p->R);
}

/* Integrate over dt with the switch-node state sw: 1 = high side on,
 * 0 = low side on, -1 = both off (dead time: body diode), -2 = gates
 * disabled. RK4 in substeps of <= 4 ns. Returns the integral of i_L and of
 * the load current over the interval. */
static void p_step(plant_t *p, int sw, double dt, double *int_il, double *int_io)
{
    const int n = (int)ceil(dt / 4e-9);
    const double h = dt / n, ws = 2 * M_PI * 650e3;
    for (int k = 0; k < n; ++k) {
        double vsw;
        if (sw == 1) vsw = p->vin;
        else if (sw == 0 || (sw == -1 && p->ideal_dt)) vsw = 0.0;
        else if (p->i > 0) vsw = -p->vd;
        else if (p->i < 0) vsw = p->vin + p->vd;
        else vsw = NAN;   /* no conduction path: i stays 0 */
        double x[3] = { p->i, p->vc, p->y }, kk[4][3];
        for (int st = 0; st < 4; ++st) {
            const double c = (st == 0) ? 0 : (st == 3 ? 1.0 : 0.5);
            double xi[3];
            for (int j = 0; j < 3; ++j) xi[j] = x[j] + (st ? c * h * kk[st - 1][j] : 0);
            plant_t q = *p; q.i = xi[0]; q.vc = xi[1];
            const double vo = p_vout(&q), io = vo / p->R + p->istep;
            kk[st][0] = isnan(vsw) ? 0.0 : (vsw - p->rl * xi[0] - vo) / p->L;
            kk[st][1] = (xi[0] - io) / p->C;
            kk[st][2] = ws * (xi[0] - xi[2]);
        }
        const double i0 = p->i;
        const double io0 = p_vout(p) / p->R + p->istep;
        for (int j = 0; j < 3; ++j) x[j] += h / 6 * (kk[0][j] + 2 * kk[1][j] + 2 * kk[2][j] + kk[3][j]);
        /* diode conduction ends at zero current (DCM) */
        if (sw < 0 && ((i0 > 0 && x[0] < 0) || (i0 < 0 && x[0] > 0))) x[0] = 0.0;
        p->i = x[0]; p->vc = x[1]; p->y = x[2];
        const double io1 = p_vout(p) / p->R + p->istep;
        *int_il += 0.5 * (i0 + p->i) * h;
        *int_io += 0.5 * (io0 + io1) * h;
    }
}

static uint32_t adc12(double volts, int noise)
{
    double c = volts / BUCK_V_PER_COUNT;
    if (noise) c += (rand() % (2 * noise + 1)) - noise;
    c = floor(c + 0.5);
    return (uint32_t)(c < 0 ? 0 : c > 4095 ? 4095 : c);
}

typedef struct {
    double t_step_up, t_step_down, t_end;
    double i_base_R, i_step;       /* base resistor, step current */
    double L_plant, vin_plant, esr;
    int noise;
    const char *name;
    int ideal_dt;
    float s_th;                    /* 0: the configured S_th */
    int score_set;                 /* 1: the score coefficients below; 0: the configured a, b, c */
    float score_a, score_b, score_c;
    float obs_beta;                /* 0: the configured gains of the load-current observer */
    float obs_gamma;
    int trig_up, trig_down;        /* D4 on the step up / down: 1 must fire, 0 must stay low, -1 not checked */
} scen_t;

typedef struct { double sum, sum2, mn, mx; long n; } stat_t;
static void st_add(stat_t *s, double x) { if (!s->n) { s->mn = s->mx = x; } s->sum += x; s->sum2 += x * x; s->mn = fmin(s->mn, x); s->mx = fmax(s->mx, x); ++s->n; }
static double st_mean(const stat_t *s) { return s->sum / s->n; }
static double st_rms_dev(const stat_t *s) { const double m = st_mean(s); return sqrt(fmax(0, s->sum2 / s->n - m * m)); }

/* The A2 / out.i_load bounds of the closed loop (user, 25 Oct: observer on the
 * raw measured average, double pole at 9.2 us). The observer study (25 Oct,
 * the same plant, 30 ms windows) gives, for this observer fed î̄_L: 5.09 mA rms
 * at 3 A with +-2 counts of ADC noise (within 0.35 mA at 1 A); fed i_meas, the
 * noise rises by 14-15 % (tau 8 us: 7.03 against 6.18 mA, tau 16 us: 3.22
 * against 2.81 mA), so about 5.9 mA here. Bias with i_meas: +0.027 / -0.018 A
 * (body diodes), -0.016 / -0.016 A (ideal dead time) at 1 / 3 A, against
 * -1.36 / +0.34 A fed î̄_L. Steps (tau 8 us, i_meas, body diodes): 50 % after
 * 11.7 us up and down, 90 % after 86.7 us up (the true load itself dips with
 * v_out after the step) and 35.0 us down; the double pole alone reaches 50 /
 * 90 % of a step of Î after 1.68 / 3.89 tau (15.4 / 35.8 us at 9.2 us). */
#define OBS_BIAS_MAX_A      0.05    /* the measured average's accuracy from 1 A up (README: +-0.04 A) */
#define OBS_NOISE_MAX_A     0.008   /* 5.9 mA expected, 2 ms windows here */
#define OBS_T50_MAX_S       20e-6   /* up and down: the double pole's own 15.4 us plus the input's rise */
#define OBS_T90_UP_MAX_S    110e-6  /* the study's 86.7 us at tau 8 us, plus the slower pole */
#define OBS_T90_DOWN_MAX_S  50e-6   /* the study's 35.0 us at tau 8 us, plus the slower pole */

static void run_closed_loop(const scen_t *sc, int strict)
{
    const double T = BUCK_TSW_S, tick = BUCK_TICK_S, lat = 45e-9;   /* ADC trigger-to-hold latency */
    plant_t p = { .L = sc->L_plant, .rl = BUCK_RL_OHM, .C = BUCK_C_F, .esr = sc->esr, .vin = sc->vin_plant,
                  .vd = 0.7, .R = sc->i_base_R, .noise = sc->noise, .ideal_dt = sc->ideal_dt };
    buck_tune_t tune = BUCK_TUNE_DEFAULTS;
    if (sc->s_th > 0.0f) tune.s_th = sc->s_th;
    if (sc->score_set) { tune.score_a = sc->score_a; tune.score_b = sc->score_b; tune.score_c = sc->score_c; }
    if (sc->obs_beta != 0.0f) tune.obs_beta = sc->obs_beta;
    if (sc->obs_gamma != 0.0f) tune.obs_gamma = sc->obs_gamma;
    /* the same controller beside it with other observer gains (D4, the score and the
     * whole controller must not notice) */
    buck_tune_t tune_ref = tune;
    tune_ref.obs_beta = 0.5f; tune_ref.obs_gamma = 1.0f;
    buck_state_t s;
    buck_out_t o;
    memset(&s, 0, sizeof s);
    memset(&o, 0, sizeof o);
    buck_state_t s_ref;                 /* the controller with other observer gains and the per-cycle ramp, side by side */
    buck_out_t o_ref;
    memset(&o_ref, 0xA5, sizeof o_ref);     /* every field must be written: the cycle compare reads them all */
    ramp_ref_t ramp_ref = { 0.0f, ~0U };
    long ident_bad = 0;
    const long periods = (long)(sc->t_end / T);
    uint32_t pend1 = 0, pend2 = 0;      /* CMP1 for the next period and the one after */
    int running = 0, armed_now = 0;
    const double vref = BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT;
    stat_t v_lo = {0}, v_hi = {0}, dc_lo = {0}, dc_hi = {0}, e_iavg = {0}, cmp_lo = {0};
    uint32_t kw = 0;
    float rtarget = 0.0f;               /* reference slew, as the firmware runs it */
    uint32_t rmode = BUCK_RAMP_HOLD;
    /* the load current and A2 from the firmware's observer (buck_iload_obs, as in
     * run_pass, fed v_out[k+1] and the raw measured average), seeded at arming
     * (buck_obs_seed, the loop entry) and its gains checked at the entry and at
     * every reload (buck_obs_k); beside it, in double: the same recursion with
     * the same gains, the two first-order stages on Î[k] = i_meas[k] -
     * (v[k] - v[k-1])/a with the double pole of the defaults and the recursion
     * with that pole's exact gains (both only with the default gains in use),
     * the observer's response to the A0 part -(v[k] - v[k-1])/a alone, and the
     * observer with the compiled defaults (when the tunables' pair is rejected,
     * A2 must be that one, bit for bit) */
    const double a_d = (double)BUCK_OBS_A, pp = exp(-T / BUCK_OBS_TAU_S);
    const double bd = 1.0 - pp * pp, gd = (1.0 - pp) * (1.0 - pp) / a_d;
    buck_obs_t x = { 0.0f, 0.0f }, xdef = x;
    buck_obs_k_t ok = BUCK_OBS_K_DEFAULTS;
    obs_d_t xd = { 0, 0 }, xe = xd, xv = xd;
    double y1d = 0.0, y2d = 0.0, vprev_d = 0.0, rec_worst = 0.0, id_worst = 0.0, iddd_worst = 0.0;
    long a2_bad = 0, seed_bad = 0, def_bad = 0;
    int is_def = 0;
    stat_t e_ob_lo = {0}, e_ob_hi = {0}, ob_pre = {0}, ob_hi = {0}, ob_end = {0}, tr_pre = {0}, tr_hi = {0}, tr_end = {0};
    stat_t a0_lo = {0}, a0_hi = {0}, yv_lo = {0}, yv_hi = {0};
    enum { STEP_LOG = 1024 };
    static float ob_up[STEP_LOG], ob_dn[STEP_LOG];
    static double tr_up[STEP_LOG], tr_dn[STEP_LOG];
    int nup = 0, ndn = 0;
    stat_t e_meas_lo = {0}, e_meas_hi = {0}, e_avg_lo = {0}, e_avg_hi = {0};
    long trig_steady = 0, trig_up = 0, trig_down = 0, trig_logic_bad = 0;
    double s_steady_max = -INFINITY, s_up_max = -INFINITY, s_down_max = -INFINITY;   /* peak trigger score S (can be < 0) */
    double v_min_up = 9, v_max_down = 0, t_last_out_up = 0;
    /* the published score against a e_v^2 + b e_v + c in double, from the published
     * v and V_ref, independent of buck_trigger_score_k(): with b != 0 it catches a
     * wrong sign of e_v */
    double score_worst = 0.0;
    srand(7);
    for (long n = 0; n < periods; ++n) {
        const double t0 = n * T;
        p.istep = (t0 >= sc->t_step_up && t0 < sc->t_step_down) ? sc->i_step : 0.0;
        /* arm 1 ms after start, in the middle of a period (low side for the rest) */
        const uint32_t cmp = running ? pend1 : 0;
        pend1 = pend2;
        /* events of this period, in ticks */
        double il_int = 0, io_int = 0;
        uint32_t isum = 0, vsum = 0; double vs[2] = { 0, 0 };
        const double t_samp_i[4] = { T / 8 + lat, 3 * T / 8 + lat, 5 * T / 8 + lat, 7 * T / 8 + lat };
        const double t_samp_v[2] = { BUCK_VSAMPLE0_CMP * tick + lat, BUCK_VSAMPLE_CMP * tick + lat };
        double tev[16]; int sw[16]; int ne = 0;
        if (running && !armed_now) {
            tev[ne] = BUCK_DT_TICKS * tick; sw[ne++] = -1;                  /* [0, DT): both off */
            tev[ne] = cmp * tick; sw[ne++] = 1;                             /* [DT, CMP1): HS */
            tev[ne] = (cmp + BUCK_DT_TICKS) * tick; sw[ne++] = -1;          /* dead time */
            tev[ne] = T; sw[ne++] = 0;                                      /* LS */
        } else if (armed_now) {
            tev[ne] = 0.4e-6; sw[ne++] = -2;                                /* disabled until the arm write */
            tev[ne] = T; sw[ne++] = 0;                                      /* then LS for the rest */
        } else {
            tev[ne] = T; sw[ne++] = -2;
        }
        /* integrate through switching events and sample instants */
        double t = 0; int e = 0, ki = 0, kv = 0;
        while (t < T - 1e-15) {
            double next = tev[e];
            if (ki < 4 && t_samp_i[ki] < next) next = t_samp_i[ki];
            if (kv < 2 && t_samp_v[kv] < next) next = t_samp_v[kv];
            p_step(&p, sw[e], next - t, &il_int, &io_int);
            t = next;
            if (ki < 4 && fabs(t - t_samp_i[ki]) < 1e-15) { isum += adc12(BUCK_ISENSE_ZERO_V + BUCK_ISENSE_V_PER_A * p.y, p.noise); ++ki; }
            if (kv < 2 && fabs(t - t_samp_v[kv]) < 1e-15) { vs[kv] = p_vout(&p); vsum += adc12(vs[kv], p.noise); ++kv; }
            if (fabs(t - tev[e]) < 1e-15) ++e;
        }
        armed_now = 0;
        const double iavg_true = il_int / T, iload_true = io_int / T;
        const double vdc = (iload_true - p.istep) * p.R;
        const uint32_t adc_v = vsum;         /* ADC1: the two A0 samples summed */
        /* the voltage the controller measures (the mean of its two samples, before
         * quantization), and the cycle-average output a meter would read */
        const double vsample = 0.5 * (vs[0] + vs[1]);
        /* the control update (runs early in period n+1) */
        if (!running) {
            if (t0 + T >= 1e-3) {
                pend2 = buck_ctrl_arm(&tune, &s, adc_v, isum, &o);
                s_ref = s;
                ramp_ref = (ramp_ref_t){ s.vref, (s.vref == buck_ramp_target(&tune)) ? tune.vref_counts : ~0U };
                /* the run_loop() entry: the observer from the arming pass, its gains checked against the defaults */
                ok = buck_obs_k(&tune, BUCK_OBS_K_DEFAULTS);
                x = xdef = buck_obs_seed(&o);
                seed_bad += !same_bits(x.vh, o.v) || !same_bits(x.ih, o.i_meas) || !same_bits(o.i_load, o.i_meas) ||
                            fabs((double)x.vh - adc_v * BUCK_V_PER_VSUM) > 1e-6 ||
                            fabs((double)x.ih - (isum / 4.0 - (double)tune.isense_zero) * (double)tune.isense_gain) > 1e-6;
                is_def = same_bits(ok.beta, BUCK_OBS_BETA_DEFAULT) && same_bits(ok.gamma, BUCK_OBS_GAMMA_DEFAULT);
                xd = xe = (obs_d_t){ x.vh, x.ih };
                xv = (obs_d_t){ x.vh, 0.0 };
                y1d = y2d = x.ih;
                vprev_d = x.vh;
                rtarget = buck_ramp_target(&tune);   /* run_loop() entry */
                rmode = buck_ramp_mode(s.vref, rtarget);
                pend1 = pend2;   /* the arm write is active from the next period too */
                running = 1; armed_now = 1;
            }
            continue;
        }
        buck_step_t stp;
        pend2 = buck_ctrl_fast(&tune, &s, adc_v, isum, &stp);
        const uint32_t g = buck_ctrl_slow(&tune, &s, &stp, &o);
        /* the load current and A2 from the firmware's observer (buck_iload_obs, as run_pass) */
        uint32_t a2_now = ~0U;
        const float y = buck_iload_obs(&x, stp.v, stp.im, ok, &a2_now);
        o.i_load = y;                                    /* what A2 and out.i_load show */
        {
            /* A2 = floor(y DAC_PER_A) clipped (exact but within 1e-6 of an integer) */
            const double pr = (double)y * (4095.0 / BUCK_DAC_FULL_SCALE_A), ref = fmin(4095.0, fmax(0.0, floor(pr)));
            a2_bad += !((double)a2_now == ref || (fabs(pr - floor(pr + 0.5)) < 1e-6 * fmax(1.0, pr) && fabs((double)a2_now - ref) <= 1.0));
            /* the recursion in double, the two stages and the A0 part */
            rec_worst = fmax(rec_worst, fabs((double)y - obs_d_step(&xd, o.v, o.i_meas, a_d, ok.beta, ok.gamma)));
            const double dv = (double)o.v - vprev_d, Ihat = (double)o.i_meas - dv / a_d;
            vprev_d = o.v;
            y1d += (1.0 - pp) * (Ihat - y1d);
            y2d += (1.0 - pp) * (y1d - y2d);
            if (is_def) {
                id_worst = fmax(id_worst, fabs((double)y - y2d));
                iddd_worst = fmax(iddd_worst, fabs(obs_d_step(&xe, o.v, o.i_meas, a_d, bd, gd) - y2d));
                def_bad += !same_bits(y, buck_obs_step(&xdef, stp.v, stp.im, BUCK_OBS_K_DEFAULTS));
            }
            const double yv = obs_d_step(&xv, o.v, 0.0, a_d, ok.beta, ok.gamma), a0 = -dv / a_d;
            const int lo = (t0 > sc->t_step_up - 2e-3 && t0 < sc->t_step_up) || (t0 > sc->t_end - 2e-3);
            const int hi = (t0 > sc->t_step_down - 2e-3 && t0 < sc->t_step_down);
            if (lo) { st_add(&a0_lo, a0); st_add(&yv_lo, yv); }
            if (hi) { st_add(&a0_hi, a0); st_add(&yv_hi, yv); }
        }
        ++kw;
        /* the reference step, chosen at every reload of the tunables as in the firmware, and the observer's gains
         * checked there (the pass's observer step came first: a reload takes effect from the next pass) */
        s.vref = buck_ramp_step(rmode, s.vref, rtarget, BUCK_SOFTSTART_STEP_V);
        if ((kw & (BUCK_TELEMETRY_EVERY - 1U)) == 0U) {
            rtarget = buck_ramp_target(&tune); rmode = buck_ramp_mode(s.vref, rtarget);
            ok = buck_obs_k(&tune, ok);
        }
        {   /* the same samples through the controller with other observer gains and the per-cycle
             * ramp behind its vref_reached gate: every bit must match */
            buck_step_t stp_ref;
            const uint32_t cmp_ref = buck_ctrl_fast(&tune_ref, &s_ref, adc_v, isum, &stp_ref);
            const uint32_t g_ref = buck_ctrl_slow(&tune_ref, &s_ref, &stp_ref, &o_ref);
            o_ref.i_load = o.i_load;                     /* the observer's (not the direct form's) */
            ramp_ref.vref = s_ref.vref;
            ramp_clamp(tune.vref_counts, &ramp_ref);      /* base2's form */
            s_ref.vref = ramp_ref.vref;
            ident_bad += cmp_ref != pend2 || g_ref != g || memcmp(&o_ref, &o, sizeof o) != 0 ||
                         memcmp(&s_ref, &s, sizeof s) != 0;
        }
        /* o.* refers to cycle n (the one just simulated) */
        const double tn = t0;
        const int steady_lo = (tn > sc->t_step_up - 2e-3 && tn < sc->t_step_up) || (tn > sc->t_end - 2e-3);
        const int steady_hi = (tn > sc->t_step_down - 2e-3 && tn < sc->t_step_down);
        trig_logic_bad += g != (o.score >= tune.s_th ? 1U : 0U);
        {
            const double e = (double)o.v - (double)o.vref, a = tune.score_a, b = tune.score_b, c = tune.score_c;
            const double scale = fabs(a) * e * e + fabs(b * e) + fabs(c);
            score_worst = fmax(score_worst, fabs(o.score - (a * e * e + b * e + c)) / fmax(1e-30, scale));
        }
        if (steady_lo || steady_hi) s_steady_max = fmax(s_steady_max, o.score);
        if (steady_lo) { st_add(&e_ob_lo, y - iload_true);
                         if (tn < sc->t_step_up) { st_add(&ob_pre, y); st_add(&tr_pre, iload_true); }
                         else { st_add(&ob_end, y); st_add(&tr_end, iload_true); } }
        if (steady_hi) { st_add(&e_ob_hi, y - iload_true); st_add(&ob_hi, y); st_add(&tr_hi, iload_true); }
        if (tn >= sc->t_step_up && nup < STEP_LOG) { ob_up[nup] = y; tr_up[nup] = iload_true; ++nup; }
        if (tn >= sc->t_step_down && ndn < STEP_LOG) { ob_dn[ndn] = y; tr_dn[ndn] = iload_true; ++ndn; }
        if (steady_lo) st_add(&dc_lo, vdc);
        if (steady_hi) st_add(&dc_hi, vdc);
        if (steady_lo) { st_add(&v_lo, vsample); st_add(&cmp_lo, o.cmp); trig_steady += g;
                         st_add(&e_meas_lo, o.i_meas - iavg_true); st_add(&e_avg_lo, o.i_avg - iavg_true); }
        if (steady_hi) { st_add(&v_hi, vsample); trig_steady += g;
                         st_add(&e_meas_hi, o.i_meas - iavg_true); st_add(&e_avg_hi, o.i_avg - iavg_true); }
        if (tn > 12e-3) st_add(&e_iavg, o.i_avg - iavg_true);
        if (tn >= sc->t_step_up && tn < sc->t_step_up + 0.5e-3) { trig_up += g; v_min_up = fmin(v_min_up, vsample); s_up_max = fmax(s_up_max, o.score); }
        if (tn >= sc->t_step_down && tn < sc->t_step_down + 0.5e-3) { trig_down += g; v_max_down = fmax(v_max_down, vsample); s_down_max = fmax(s_down_max, o.score); }
        if (tn >= sc->t_step_up && tn < sc->t_step_down && fabs(vsample - vref) > 0.005) t_last_out_up = tn - sc->t_step_up;
    }
    CHECK(s.vref == buck_ramp_target(&tune) && rmode == BUCK_RAMP_HOLD, "%s: soft-start ramp finished", sc->name);
    CHECK(ident_bad == 0, "%s: %ld cycles differ with other observer gains (%g, %g)", sc->name, ident_bad,
          (double)tune_ref.obs_beta, (double)tune_ref.obs_gamma);
    printf("\n[%s] L=%.0f nH, Vin=%.2f V, ESR=%.0f mOhm, ADC noise +-%d LSB\n", sc->name, sc->L_plant * 1e9,
           sc->vin_plant, sc->esr * 1e3, sc->noise);
    printf("  v_out as measured (mean of the two samples)  %.1f A: mean %.4f V (V_ref %.4f), p-p %.1f mV | %.1f A: mean %.4f V, p-p %.1f mV\n",
           vref / sc->i_base_R, st_mean(&v_lo), vref, (v_lo.mx - v_lo.mn) * 1e3,
           vref / sc->i_base_R + sc->i_step, st_mean(&v_hi), (v_hi.mx - v_hi.mn) * 1e3);
    printf("  v_out DC (cycle average): %+.1f mV from V_ref at %.1f A, %+.1f mV at %.1f A\n",
           (st_mean(&dc_lo) - vref) * 1e3, vref / sc->i_base_R, (st_mean(&dc_hi) - vref) * 1e3, vref / sc->i_base_R + sc->i_step);
    printf("  step up: min %.4f V (dip %.1f mV), inside +-5 mV for good after %.1f us; step down: max %.4f V (+%.1f mV)\n",
           v_min_up, (vref - v_min_up) * 1e3, t_last_out_up * 1e6, v_max_down, (v_max_down - vref) * 1e3);
    /* step response of A2 and of the true load: first time past 50 % / 90 % of the step, each normalised to its
     * own levels before and after (one cycle of pipeline latency not counted, as before) */
    double u50 = -1, u90 = -1, d50 = -1, d90 = -1, tu_min = 9, td_min = 9;
    for (int j = 0; j < nup; ++j) {
        const double f = (ob_up[j] - st_mean(&ob_pre)) / (st_mean(&ob_hi) - st_mean(&ob_pre));
        if (u50 < 0 && f >= 0.5) u50 = j * T;
        if (u90 < 0 && f >= 0.9) u90 = j * T;
        if (j * T < 0.5e-3) tu_min = fmin(tu_min, (tr_up[j] - st_mean(&tr_pre)) / (st_mean(&tr_hi) - st_mean(&tr_pre)));
    }
    for (int j = 0; j < ndn; ++j) {
        const double f = (ob_dn[j] - st_mean(&ob_hi)) / (st_mean(&ob_end) - st_mean(&ob_hi));
        if (d50 < 0 && f >= 0.5) d50 = j * T;
        if (d90 < 0 && f >= 0.9) d90 = j * T;
        if (j * T < 0.5e-3) td_min = fmin(td_min, (tr_dn[j] - st_mean(&tr_hi)) / (st_mean(&tr_end) - st_mean(&tr_hi)));
    }
    /* the A0 part -(v[k] - v[k-1])/a, differenced white noise when v carries white noise, through the
     * observer: its rms falls to sqrt(sum g[n]^2 / 2) of the per-cycle value, g the impulse response of
     * H(z)(1 - z^-1), i.e. a times the observer's response to a unit impulse in v alone, for the gains in use */
    double ng = 0.0;
    {
        obs_d_t xi = { 0.0, 0.0 };
        for (int j = 0; j < 20000; ++j) {
            const double gj = obs_d_step(&xi, (j == 0) ? 1.0 : 0.0, 0.0, a_d, ok.beta, ok.gamma) * a_d;
            ng += gj * gj;
        }
    }
    const double nf = sqrt(ng / 2.0);
    printf("  A2 / out.i_load (observer: beta %.7g, gamma %.8g A/V%s, fed v_out and i_meas): error %.3f A mean / %.4f A rms "
           "(low load), %.3f / %.4f A (high load); step up 50 %% / 90 %% after %.1f / %.1f us, step down %.1f / %.1f us "
           "(the true load falls back to %.3f / %.3f of its steps); A0 part %.4f / %.4f A rms (per cycle %.3f / %.3f; white "
           "noise: x %.4f)\n", (double)ok.beta, (double)ok.gamma, is_def ? ", the defaults" : "", st_mean(&e_ob_lo),
           st_rms_dev(&e_ob_lo), st_mean(&e_ob_hi), st_rms_dev(&e_ob_hi), u50 * 1e6, u90 * 1e6, d50 * 1e6, d90 * 1e6, tu_min,
           td_min, st_rms_dev(&yv_lo), st_rms_dev(&yv_hi), st_rms_dev(&a0_lo), st_rms_dev(&a0_hi), nf);
    printf("  observer: within %.1e A of its recursion in double; two first-order stages: %.1e A (double: %.1e A)\n",
           rec_worst, is_def ? id_worst : NAN, is_def ? iddd_worst : NAN);
    printf("  steady state, low / high load: measured average error %.3f / %.3f A, corrected estimate error %.3f / %.3f A\n",
           st_mean(&e_meas_lo), st_mean(&e_meas_hi), st_mean(&e_avg_lo), st_mean(&e_avg_hi));
    printf("  measured average-current error after soft-start (incl. steps): %.3f A mean / %.3f A rms\n", st_mean(&e_iavg), st_rms_dev(&e_iavg));
    printf("  duty compare at low load: %.1f ticks, rms jitter %.2f ticks\n", st_mean(&cmp_lo), st_rms_dev(&cmp_lo));
    printf("  trigger (S_th = %g): %ld cycles in steady state, %ld after the step up, %ld after the step down; "
           "peak S %.2g / %.2g / %.2g\n", (double)tune.s_th, trig_steady, trig_up, trig_down, s_steady_max, s_up_max, s_down_max);
    /* strict >= 1: regulation, stability, trigger, the observer on A2; strict = 2 also
     * the estimator accuracy (only meaningful when the plant matches the model). */
    if (strict >= 1) {
        CHECK(fabs(st_mean(&v_lo) - vref) < 0.003 && fabs(st_mean(&v_hi) - vref) < 0.003, "%s: regulation", sc->name);
        /* the DC output a meter reads: the two-sample mean sits near the ripple's average (without ESR) */
        if (sc->esr == 0.0)
            CHECK(fabs(st_mean(&dc_lo) - vref) < 0.0006 && fabs(st_mean(&dc_hi) - vref) < 0.0006,
                  "%s: DC output within 0.6 mV of V_ref", sc->name);
        CHECK(v_lo.mx - v_lo.mn < 0.010 && v_hi.mx - v_hi.mn < 0.010, "%s: steady-state oscillation", sc->name);
        CHECK(trig_logic_bad == 0, "%s: g[k] = (S[k] >= S_th)", sc->name);
        /* 4 roundings, 5 where v - V_ref rounds (v and V_ref more than a factor 2 apart: the soft-start's begin) */
        CHECK(score_worst < 4e-7, "%s: S[k] against a e_v^2 + b e_v + c (worst error %.1e of |a| e_v^2 + |b e_v| + |c|)",
              sc->name, score_worst);
        CHECK((trig_up > 0) == (s_up_max >= tune.s_th) && (trig_down > 0) == (s_down_max >= tune.s_th),
              "%s: trigger on the load steps exactly when S reaches S_th", sc->name);
        if (sc->noise == 0) CHECK(trig_steady == 0, "%s: no trigger in steady state", sc->name);
        /* where the scenario says so, D4 must actually fire (or stay low) on each step */
        if (sc->trig_up >= 0)
            CHECK((trig_up > 0) == (sc->trig_up > 0), "%s: D4 %s on the step up (S_th = %g)", sc->name,
                  sc->trig_up ? "fires" : "stays low", (double)tune.s_th);
        if (sc->trig_down >= 0)
            CHECK((trig_down > 0) == (sc->trig_down > 0), "%s: D4 %s on the step down (S_th = %g)", sc->name,
                  sc->trig_down ? "fires" : "stays low", (double)tune.s_th);
        /* the observer: A2 = floor(y DAC_PER_A); the seed; the recursion in double (float roundings through
         * poles at 0.834: well below 1e-4 A); with the defaults the two stages (the gains rounded to float:
         * 4e-8 relative) and, in double, the identity itself; rejected tunables give the defaults' A2 bit for
         * bit; then the bounds above (OBS_*) */
        CHECK(a2_bad == 0, "%s: A2 code of the observer's estimate (%ld wrong)", sc->name, a2_bad);
        CHECK(seed_bad == 0, "%s: observer seeded from the arming pass's v and i_meas", sc->name);
        CHECK(rec_worst < 1e-4, "%s: observer against its recursion in double (worst %.1e A)", sc->name, rec_worst);
        if (is_def)
            CHECK(id_worst < 1e-4 && iddd_worst < 1e-9 && def_bad == 0,
                  "%s: observer = two first-order stages (worst %.1e A; double %.1e A; %ld differ from the defaults)",
                  sc->name, id_worst, iddd_worst, def_bad);
        CHECK(fabs(st_mean(&e_ob_lo)) < OBS_BIAS_MAX_A && fabs(st_mean(&e_ob_hi)) < OBS_BIAS_MAX_A,
              "%s: A2 bias %.3f / %.3f A (bound %.2f A)", sc->name, st_mean(&e_ob_lo), st_mean(&e_ob_hi), OBS_BIAS_MAX_A);
        if (sc->noise > 0) {
            CHECK(st_rms_dev(&e_ob_lo) < OBS_NOISE_MAX_A && st_rms_dev(&e_ob_hi) < OBS_NOISE_MAX_A,
                  "%s: A2 noise %.4f / %.4f A rms (bound %.3f A)", sc->name, st_rms_dev(&e_ob_lo), st_rms_dev(&e_ob_hi),
                  OBS_NOISE_MAX_A);
            CHECK(st_rms_dev(&yv_lo) < 1.5 * nf * st_rms_dev(&a0_lo) && st_rms_dev(&yv_hi) < 1.5 * nf * st_rms_dev(&a0_hi),
                  "%s: the observer cuts the A0 noise of the load estimate (x %.4f / %.4f, white noise x %.4f)", sc->name,
                  st_rms_dev(&yv_lo) / st_rms_dev(&a0_lo), st_rms_dev(&yv_hi) / st_rms_dev(&a0_hi), nf);
        } else if (sc->esr == 0.0) {    /* with ESR, 162 A/V x ESR = 0.81 of the capacitor current enters Î: the model has none */
            CHECK(u50 > 0 && d50 > 0 && u50 <= OBS_T50_MAX_S && d50 <= OBS_T50_MAX_S && u90 > 0 && u90 <= OBS_T90_UP_MAX_S &&
                  d90 > 0 && d90 <= OBS_T90_DOWN_MAX_S, "%s: A2 step times %.1f / %.1f us up, %.1f / %.1f us down",
                  sc->name, u50 * 1e6, u90 * 1e6, d50 * 1e6, d90 * 1e6);
        }
    }
    if (strict >= 2)
        CHECK(fabs(st_mean(&e_avg_lo)) < 0.05 && fabs(st_mean(&e_avg_hi)) < 0.05, "%s: average-current estimate bias", sc->name);
}

int main(void)
{
    test_score();
    test_ramp();
    test_obs();
    test_obs_gains();
    /* Plant with the switch node at 0 V in the dead time (the ideal-switch model):
     * checks the implementation end to end. */
    scen_t ideal = { 15e-3, 20e-3, 25e-3, 0.8057, 2.0, BUCK_L_H, BUCK_VIN_V, 0.0, 0, "switch node 0 V in dead time (= model)", 1,
                     0.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1, -1 };
    run_closed_loop(&ideal, 2);
    /* the same with the trigger threshold low enough for the 2 A steps (D4 only:
     * the score does not reach A2) */
    scen_t ideal_trig = ideal; ideal_trig.s_th = 0.01f; ideal_trig.name = "switch node 0 V in dead time, S_th = 0.01";
    ideal_trig.trig_up = ideal_trig.trig_down = 1;
    run_closed_loop(&ideal_trig, 1);
    /* the D4 threshold of S_th = 0.01 through the coefficient a: 100 e_v^2 >= 1 */
    scen_t ideal_a = ideal; ideal_a.s_th = 1.0f; ideal_a.score_set = 1; ideal_a.score_a = 100.0f;
    ideal_a.name = "switch node 0 V in dead time, S_th = 1, a = 100";
    ideal_a.trig_up = ideal_a.trig_down = 1;
    run_closed_loop(&ideal_a, 1);
    /* all three coefficients: the linear term makes S depend on the sign of e_v
     * (D4 on the overshoot of the step down only), and c < 0 makes S negative at
     * rest (D4 low) */
    scen_t ideal_abc = ideal; ideal_abc.score_set = 1; ideal_abc.score_a = 100.0f; ideal_abc.score_b = 5.0f;
    ideal_abc.score_c = -0.25f; ideal_abc.name = "switch node 0 V in dead time, a = 100, b = 5, c = -0.25 (S < 0 at rest)";
    ideal_abc.trig_up = 0; ideal_abc.trig_down = 1;
    run_closed_loop(&ideal_abc, 1);
    /* Realistic dead time: body-diode conduction (V_in + 0.7 V for negative,
     * -0.7 V for positive current): regulation must still hold; A2 (the observer on
     * the measured average) must not be biased. */
    const scen_t nominal = { 15e-3, 20e-3, 25e-3, 0.8057, 2.0, BUCK_L_H, BUCK_VIN_V, 0.0, 0, "body diodes in dead time", 0,
                             0.0f, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1, -1 };
    run_closed_loop(&nominal, 1);
    scen_t noisy = nominal; noisy.noise = 2; noisy.name = "ADC noise";
    run_closed_loop(&noisy, 1);
    scen_t esr = nominal; esr.esr = 5e-3; esr.name = "5 mOhm ESR";
    run_closed_loop(&esr, 1);
    /* observer gains outside the stable region in the tunables (beta = 2 > 2 - a gamma/2): rejected at the loop
     * entry and at every reload, so the compiled defaults run, A2 bit for bit as with them */
    scen_t badgain = noisy; badgain.obs_beta = 2.0f; badgain.name = "ADC noise, tune.obs_beta = 2 (outside: the defaults run)";
    run_closed_loop(&badgain, 1);
    /* Model mismatch (not pass/fail: shows the sensitivity) */
    scen_t lmis = nominal; lmis.L_plant = 0.8 * BUCK_L_H; lmis.name = "plant L -20 %";
    run_closed_loop(&lmis, 0);
    scen_t vmis = nominal; vmis.vin_plant = 6.5; vmis.name = "plant V_in 6.5 V (model 6 V)";
    run_closed_loop(&vmis, 0);
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "all tests passed", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
