/* Learning jobs and the TOC completion math on the L-C model (Inc/buck_learn.h; stage 3). */

#include "buck_learn.h"

#define LEARN_EPS64      (64.0f * 1.1920929e-7f)    /* float conditioning floor (backup buck_toc.c:176, 222) */
#define LEARN_END_I      ((float)BUCK_LEARN_END_I_A)
#define LEARN_END_V      ((float)BUCK_LEARN_END_V_V)
#define LEARN_JQ_MIN     ((float)BUCK_LEARN_JQ_MIN)
#define LEARN_SCALE_Q    ((float)BUCK_LEARN_SCALE_Q_A)
#define LEARN_ETA_Q      ((float)BUCK_LEARN_ETA_Q)
#define LEARN_LAMBDA_Q   ((float)BUCK_LEARN_LAMBDA_Q)
#define LEARN_CAP_Q      ((float)BUCK_LEARN_CAP_Q_A)
#define LEARN_ISI        ((float)(1.0 / BUCK_LEARN_SIGMA_G_I_A))   /* S = diag(1/.2 A, 1/.02 V) */
#define LEARN_ISV        ((float)(1.0 / BUCK_LEARN_SIGMA_G_V_V))
#define LEARN_LAMBDA_G   ((float)BUCK_LEARN_LAMBDA_G)
#define LEARN_ETA_G      ((float)BUCK_LEARN_ETA_G)
#define LEARN_REL_G      ((float)BUCK_LEARN_G_REL_MAX)
#define LEARN_ETA_K      ((float)BUCK_LEARN_ETA_KAPPA)
#define LEARN_EPS_K      ((float)BUCK_LEARN_EPS_KAPPA_A2)

/* ---- q ------------------------------------------------------------------------------- */
enum { Q_J0 = 0U, Q_SENS, Q_JX, Q_FIN, Q_DONE };

BUCK_TOC_FAST(buck_learn_q_start) void buck_learn_q_start(buck_learn_q_t *j, const buck_toc_plan_t *ep, float i_d, float v_d, float tail,
                        float on1, float per1, float v_s, uint32_t raw_ok)
{
    /* the episode (F12: copied; the plan may be overwritten by a new request) */
    j->order = ep->order; j->x0 = ep->x0; j->ieq = ep->ieq; j->vs = ep->vs; j->ton0 = ep->ton0; j->toff0 = ep->toff0;
    j->ton_x = ep->ton_x; j->toff_x = ep->toff_x; j->target_i = ep->target_i; j->vref = ep->vref;
    j->q_used = ep->q_used;
    j->i_d = i_d; j->v_d = v_d; j->v_s = v_s; j->raw_ok = raw_ok;
    j->jq = 0.0f; j->q_new = ep->q_used;
    j->stage = Q_J0;
    j->status = BUCK_LEARN_BUSY;
    /* post-D spans and h (F14: derived from them) */
    const uint32_t cyc_ok = (per1 > 0.0f && per1 <= BUCK_TOC_FLT_MAX && on1 >= 0.0f && on1 <= per1);
    uint32_t post_ok;
    if (ep->order == BUCK_TOC_OFF_ON) {
#if BUCK_LEARN_QPEAK_AT_B
        j->h = tail + per1;
        post_ok = (tail > 0.0f && tail <= BUCK_TOC_FLT_MAX) && cyc_ok;
#else
        j->h = tail;
        post_ok = (tail > 0.0f && tail <= BUCK_TOC_FLT_MAX);
#endif
    } else {
        j->h = per1;
        post_ok = cyc_ok;
    }
    /* teaching validity (SPEC 223-224, 231-233, 279-281); also rejects NaNs */
    if (ep->status != BUCK_TOC_READY || !raw_ok || !post_ok ||
        !(ep->ton0 > 0.0f && ep->toff0 > 0.0f) || !(j->h > 0.0f && j->h <= BUCK_TOC_FLT_MAX) ||
        !(buck_toc_abs(i_d - ep->target_i) <= LEARN_END_I) || !(buck_toc_abs(v_d - ep->vref) <= LEARN_END_V)) {
        j->status = BUCK_LEARN_REJECT;
        j->stage = Q_DONE;
    }
}

BUCK_INLINE int32_t q_stage(buck_learn_q_t *j)
{
    const uint32_t on_first = (j->order == BUCK_TOC_ON_OFF);
    uint32_t next = j->stage + 1U;
    int32_t st = BUCK_LEARN_BUSY;
    switch (j->stage) {
    case Q_J0:                                 /* J_tau0 at the nominal plan from x0 */
        (void)buck_lc_pair(j->x0, on_first ? j->ton0 : j->toff0, on_first ? j->toff0 : j->ton0, j->order, j->ieq, j->vs,
                           (buck_xy_t *)0, &j->jt);
        break;
    case Q_SENS: {                             /* d tau_x/dq = diag(tau_x/tau0) J_tau0^-1 [1, 0] */
        const buck_m2_t m = j->jt;
        const float ad = m.a * m.d, bc = m.b * m.c;
        const float det = ad - bc, scale = buck_toc_abs(ad) + buck_toc_abs(bc);
        if (!(buck_toc_fin(det) & buck_toc_fin(scale))) { st = BUCK_LEARN_NONFINITE; break; }
        if (!(buck_toc_abs(det) > LEARN_EPS64 * scale)) { st = BUCK_LEARN_SINGULAR; break; }
        const float t = 1.0f / ((j->ton0 * j->toff0) * det);
        j->son = ((j->ton_x * j->toff0) * m.d) * t;
        j->soff = -((j->toff_x * j->ton0) * m.c) * t;
        if (!(buck_toc_fin(j->son) & buck_toc_fin(j->soff))) st = BUCK_LEARN_NONFINITE;
        break;
    }
    case Q_JX: {                               /* the executed pair: dx_D = J_tau_x [son, soff] */
        buck_m2_t m;
        (void)buck_lc_pair(j->x0, on_first ? j->ton_x : j->toff_x, on_first ? j->toff_x : j->ton_x, j->order, j->ieq,
                           j->vs, (buck_xy_t *)0, &m);
        j->dx.i = m.a * j->son + m.b * j->soff;
        j->dx.v = m.c * j->son + m.d * j->soff;
        break;
    }
    case Q_FIN: {                              /* the post-D spans, Jq, the floor, the step */
        const buck_xy_t s = buck_lc_phi(buck_lc_sincos(j->h * BUCK_LC_OMEGA), j->dx);
        const float jq = s.v - j->dx.v, hc = j->h * BUCK_TOC_INV_C;
        j->jq = jq;
        if (!buck_toc_fin(jq)) { st = BUCK_LEARN_NONFINITE; break; }
        if (!(buck_toc_abs(jq) >= hc * LEARN_JQ_MIN)) { st = BUCK_LEARN_REJECT; break; }   /* scaled Jq floor */
        const float inv_sigma = 1.0f / (hc * LEARN_SCALE_Q);                               /* sigmaV = (h/C) .2 A */
        const float step = -buck_learn_scalar_step(jq, j->v_s - j->v_d, LEARN_ETA_Q, LEARN_LAMBDA_Q, inv_sigma, LEARN_CAP_Q);
        const float q = j->q_used + step;
        if (!buck_toc_fin(q)) { st = BUCK_LEARN_NONFINITE; break; }
        j->q_new = q;
        st = BUCK_LEARN_OK;
        break;
    }
    default:
        return j->status;
    }
    if (st != BUCK_LEARN_BUSY) {
        j->status = st;
        next = Q_DONE;
    }
    j->stage = next;
    return st;
}

BUCK_TOC_FAST(buck_learn_q_step) int32_t buck_learn_q_step(buck_learn_q_t *j)
{
    return q_stage(j);
}

/* ---- G ------------------------------------------------------------------------------------ */
enum { G_J = BUCK_LEARN_G_J, G_WAIT = BUCK_LEARN_G_WAIT, G_SOLVE1, G_SOLVE2, G_UPDATE, G_DONE };

BUCK_TOC_FAST(buck_learn_g_start) void buck_learn_g_start(buck_learn_g_t *g, const buck_toc_plan_t *ep, float v_s0)
{
    const uint32_t g1 = (ep->order == BUCK_TOC_ON_OFF) ? 1U : 0U;
    g->g_used = ep->g;
    g->target_i = ep->target_i;
    g->vref = ep->vref;
    g->order = ep->order;
    g->x0.i = ep->x0.i;                         /* held request i_L ... */
    g->x0.v = v_s0;                             /* ... live raw V(S0) (SPEC 91, 285-286) */
    g->h1 = g1 ? ep->ton : ep->toff;            /* tau_command = diag(g) tau0 */
    g->h2 = g1 ? ep->toff : ep->ton;
    g->ton0 = ep->ton0; g->toff0 = ep->toff0; g->ieq = ep->ieq; g->vs = ep->vs;
    g->g_new = ep->g;
    g->stage = G_J;
    g->status = (ep->status == BUCK_TOC_READY) ? BUCK_LEARN_BUSY : BUCK_LEARN_REJECT;
    if (g->status != BUCK_LEARN_BUSY) g->stage = G_DONE;
}

BUCK_TOC_FAST(buck_learn_g_teach_start) void buck_learn_g_teach_start(buck_learn_g_t *g, float i_d, float v_d)
{
    if (g->stage != G_WAIT) {                   /* no valid Jacobian (SPEC 137): one opportunity, none taken */
        g->status = BUCK_LEARN_REJECT;
        g->stage = G_DONE;
        return;
    }
    g->end.i = i_d - g->target_i;               /* residual r = [i_D - target, V_D - Vref] */
    g->end.v = v_d - g->vref;
    g->status = BUCK_LEARN_BUSY;
    g->stage = G_SOLVE1;
}

BUCK_INLINE int32_t g_stage(buck_learn_g_t *g)
{
    uint32_t next = g->stage + 1U;
    int32_t st = BUCK_LEARN_BUSY;
    switch (g->stage) {
    case G_J: {                                 /* Jtau at the commanded pair from [i0, V(S0)], then Jtau diag(tau0) */
        buck_m2_t jt;
        (void)buck_lc_pair(g->x0, g->h1, g->h2, g->order, g->ieq, g->vs, (buck_xy_t *)0, &jt);
        const buck_m2_t jg = { jt.a * g->ton0, jt.b * g->toff0, jt.c * g->ton0, jt.d * g->toff0 };
        g->jg = jg;
        if (!(buck_toc_fin(jg.a) & buck_toc_fin(jg.b) & buck_toc_fin(jg.c) & buck_toc_fin(jg.d))) {
            st = BUCK_LEARN_NONFINITE;
            break;
        }
        g->status = BUCK_LEARN_OK;              /* the Jacobian is cached; wait for the teach */
        g->stage = G_WAIT;
        return BUCK_LEARN_OK;
    }
    case G_SOLVE1: {                            /* dg = -eta (J'J + lambda I)^-1 J' res, J = S Jg, res = S r */
        const float a = g->jg.a * LEARN_ISI, b = g->jg.b * LEARN_ISI;
        const float c = g->jg.c * LEARN_ISV, d = g->jg.d * LEARN_ISV;
        const float e = g->end.i * LEARN_ISI, f = g->end.v * LEARN_ISV;
        const float aa = a * a, bb = b * b, cc = c * c, dd = d * d;
        const float h00 = (aa + cc) + LEARN_LAMBDA_G, h01 = a * b + c * d, h11 = (bb + dd) + LEARN_LAMBDA_G;
        const float cross = a * d - b * c;
        /* expanded determinant (no cancellation of the Gram products; backup buck_toc.c:173) */
        const float det = (cross * cross + LEARN_LAMBDA_G * ((aa + bb) + (cc + dd))) + LEARN_LAMBDA_G * LEARN_LAMBDA_G;
        const float scale = h00 * h11 + h01 * h01;
        g->h00 = h00; g->h01 = h01; g->h11 = h11;
        g->z0 = a * e + c * f; g->z1 = b * e + d * f;
        g->det = det;
        if (!(buck_toc_fin(det) & buck_toc_fin(scale) & buck_toc_fin(g->z0) & buck_toc_fin(g->z1))) {
            st = BUCK_LEARN_NONFINITE;
            break;
        }
        if (!(det > LEARN_EPS64 * scale)) st = BUCK_LEARN_SINGULAR;
        break;
    }
    case G_SOLVE2: {
        const float t = LEARN_ETA_G / g->det;
        g->dg.i = -(g->h11 * g->z0 - g->h01 * g->z1) * t;
        g->dg.v = -(g->h00 * g->z1 - g->h01 * g->z0) * t;
        if (!(buck_toc_fin(g->dg.i) & buck_toc_fin(g->dg.v))) st = BUCK_LEARN_NONFINITE;
        break;
    }
    case G_UPDATE: {                            /* one common alpha: |alpha dg_j| <= 2 % g_j */
        const float li = LEARN_REL_G * g->g_used.i, lv = LEARN_REL_G * g->g_used.v;
        const float ai = buck_toc_abs(g->dg.i), av = buck_toc_abs(g->dg.v);
        float alpha = 1.0f;
        if (ai > li) alpha = li / ai;
        if (av > lv) { const float x = lv / av; alpha = (x < alpha) ? x : alpha; }
        const buck_xy_t gn = { g->g_used.i + alpha * g->dg.i, g->g_used.v + alpha * g->dg.v };
        if (!(gn.i > 0.0f && gn.i <= BUCK_TOC_FLT_MAX && gn.v > 0.0f && gn.v <= BUCK_TOC_FLT_MAX)) {
            st = BUCK_LEARN_NONFINITE;
            break;
        }
        g->g_new = gn;
        st = BUCK_LEARN_OK;
        break;
    }
    default:
        return g->status;
    }
    if (st != BUCK_LEARN_BUSY) {
        g->status = st;
        next = G_DONE;
    }
    g->stage = next;
    return st;
}

BUCK_TOC_FAST(buck_learn_g_step) int32_t buck_learn_g_step(buck_learn_g_t *g)
{
    return g_stage(g);
}

/* ---- kappa ------------------------------------------------------------------------------------ */
BUCK_TOC_FAST(buck_learn_kappa) uint32_t buck_learn_kappa(float kappa_used, float m_r, float z, float m_e, float *kappa)
{
    const float pred = m_r + kappa_used * z;
    const float k = kappa_used + (LEARN_ETA_K * z * (m_e - pred)) / (LEARN_EPS_K + z * z);
    if (!buck_toc_fin(k)) return 0U;
    *kappa = k;
    return 1U;
}

/* ---- The executed frame ------------------------------------------------------------------------- */
BUCK_INLINE void frame_start(buck_toc_frame_t *fr, const buck_toc_plan_t *ep, buck_xy_t x0)
{
    fr->g1 = (ep->order == BUCK_TOC_ON_OFF) ? 1U : 0U;
    fr->x0 = x0;
    fr->h1 = fr->g1 ? ep->ton_x : ep->toff_x;
    fr->h2 = fr->g1 ? ep->toff_x : ep->ton_x;
    fr->ieq = ep->ieq;
    fr->vs = ep->vs;
    fr->stage = 0U;
}

BUCK_TOC_FAST(buck_toc_frame_start) void buck_toc_frame_start(buck_toc_frame_t *fr, const buck_toc_plan_t *ep, buck_xy_t x0)
{
    frame_start(fr, ep, x0);
}

/* a1: the first span (the split). */
BUCK_TOC_FAST(buck_toc_frame_run_a1) void buck_toc_frame_run_a1(buck_toc_frame_t *fr)
{
    fr->cs1 = buck_lc_sincos(fr->h1 * BUCK_LC_OMEGA);
    fr->mid = buck_lc_map(fr->cs1, fr->x0, fr->g1, fr->ieq, fr->vs);
    fr->stage = 1U;
}

/* The start and a1 in one call (the firmware). */
BUCK_TOC_FAST(buck_toc_frame_start_a1) void buck_toc_frame_start_a1(buck_toc_frame_t *fr, const buck_toc_plan_t *ep, buck_xy_t x0)
{
    frame_start(fr, ep, x0);
    buck_toc_frame_run_a1(fr);
}

/* a2: the second span (the D state) and the charge over the frame. */
BUCK_TOC_FAST(buck_toc_frame_run_a2) void buck_toc_frame_run_a2(buck_toc_frame_t *fr)
{
    fr->cs2 = buck_lc_sincos(fr->h2 * BUCK_LC_OMEGA);
    fr->xd = buck_lc_map(fr->cs2, fr->mid, fr->g1 ^ 1U, fr->ieq, fr->vs);
    fr->qsum = fr->ieq * (fr->h1 + fr->h2) + BUCK_TOC_C * (fr->xd.v - fr->x0.v);
    fr->stage = 2U;
}

/* b1: the rotation of the first ADC2 instant from S0 and the instants' step (they are equally
 * spaced, Inc/buck_toc_frame.h); the deadbeat gain k = th cot th of the last span and 1/h with one
 * division. */
BUCK_TOC_FAST(buck_toc_frame_run_b1) void buck_toc_frame_run_b1(buck_toc_frame_t *fr, const buck_toc_frame_words_t *w)
{
    fr->cs_t0 = buck_lc_sincos((float)(w->mcmp[0] - BUCK_DT_TICKS) * BUCK_LC_TICK_RAD);
    fr->cs_dt = buck_lc_sincos((float)(w->mcmp[1] - w->mcmp[0]) * BUCK_LC_TICK_RAD);
    const float h = fr->h1 + fr->h2, th = fr->h2 * BUCK_LC_OMEGA;
    const float rc = 1.0f / (fr->cs2.s * h);
    fr->k = ((th * fr->cs2.c) * h) * rc;
    fr->inv_h = fr->cs2.s * rc;
    fr->stage = 3U;
}

/* b2: the model current at the four instants (an instant in the first span (n1 of them): on the
 * arc from x0; in the second: on the arc from the split, its angle from the split = its angle from
 * S0 minus the first span's, formed once and then stepped), their mean (pred):
 * i = Ieq + c di - (s/Z) dy (buck_lc_map's current). OK, or NONFINITE. */
BUCK_TOC_FAST(buck_toc_frame_run_b2) int32_t buck_toc_frame_run_b2(buck_toc_frame_t *fr, const buck_toc_frame_words_t *w)
{
    buck_cs_t c = fr->cs_t0;
    const buck_cs_t d = fr->cs_dt, c1 = fr->cs1;
    const float src1 = (fr->g1 ? BUCK_TOC_VIN : 0.0f) - fr->vs, src2 = (fr->g1 ? 0.0f : BUCK_TOC_VIN) - fr->vs;
    const float di1 = fr->x0.i - fr->ieq, ky1 = (fr->x0.v - src1) * BUCK_LC_INV_Z;
    const float di2 = fr->mid.i - fr->ieq, ky2 = (fr->mid.v - src2) * BUCK_LC_INV_Z;
    const uint32_t n1 = w->n1;
    float sum = 0.0f;
    uint32_t j = 0U;
    for (; j < n1; ++j) {                       /* the first span's instants */
        sum += c.c * di1 - c.s * ky1;
        const buck_cs_t nx = { c.c * d.c - c.s * d.s, c.s * d.c + c.c * d.s };
        c = nx;
    }
    if (j < 4U) {                               /* the second span's: their angle from the split */
        const buck_cs_t r0 = { c.c * c1.c + c.s * c1.s, c.s * c1.c - c.c * c1.s };
        c = r0;
        for (;;) {
            sum += c.c * di2 - c.s * ky2;
            if (++j >= 4U) break;
            const buck_cs_t nx = { c.c * d.c - c.s * d.s, c.s * d.c + c.c * d.s };
            c = nx;
        }
    }
    fr->pred = fr->ieq + 0.25f * sum;
    fr->stage = 4U;
    /* one test for all five: a NaN or an infinity in any of them makes the sum non-finite (their
     * finite values are far from overflowing) */
    return buck_toc_fin(((fr->xd.i + fr->xd.v) + (fr->pred + fr->k)) + fr->inv_h) ? BUCK_LEARN_OK : BUCK_LEARN_NONFINITE;
}

BUCK_TOC_FAST(buck_toc_frame_run) int32_t buck_toc_frame_run(buck_toc_frame_t *fr, const buck_toc_frame_words_t *w)
{
    buck_toc_frame_run_a1(fr);
    buck_toc_frame_run_a2(fr);
    buck_toc_frame_run_b1(fr, w);
    return buck_toc_frame_run_b2(fr, w);
}
