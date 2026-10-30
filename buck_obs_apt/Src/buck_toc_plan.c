/* TOC planner on the lossless L-C model (Inc/buck_toc_plan.h; stage 3, 30 Oct 2026): the circle
 * geometry in four parts and their polled form. In the firmware this file is placed in CCM
 * (stage-1 verifier F08). The Padé planner of stages 1-2b (stage machine, full and cheaper
 * planners, residual verification) is removed (user decision 1). */

#include <stddef.h>
#include "buck_toc_plan.h"

BUCK_TOC_FAST(buck_lc_pair) buck_xy_t buck_lc_pair(buck_xy_t x, float h1, float h2, uint32_t order, float ieq, float vs,
                                                   buck_xy_t *mid, buck_m2_t *jt)
{
    const uint32_t g1 = (order == BUCK_TOC_ON_OFF) ? 1U : 0U;
    const buck_xy_t m = buck_lc_map(buck_lc_sincos(h1 * BUCK_LC_OMEGA), x, g1, ieq, vs);
    const buck_cs_t cs2 = buck_lc_sincos(h2 * BUCK_LC_OMEGA);
    const buck_xy_t end = buck_lc_map(cs2, m, g1 ^ 1U, ieq, vs);
    if (mid) *mid = m;
    if (jt) *jt = buck_lc_jt(order, buck_lc_f(m, g1, ieq, vs), cs2, buck_lc_f(end, g1 ^ 1U, ieq, vs));
    return end;
}

#define SAME  (BUCK_TOC_SAME_EPS / BUCK_TOC_VERIFY_EPS)        /* already-at-target: 1/16 of the tolerance */

/* ---- helpers ---------------------------------------------------------------------------- */
/* Ieq_plan = m_R + DeltaI + kappa z, z = m_R - raw_before (SPEC 157-159). */
BUCK_INLINE float toc_ieq(const buck_toc_req_t *rq, buck_toc_plan_t *r)
{
    const float z = rq->m_r - rq->raw_before;
    const float ieq = (rq->m_r + rq->delta_i) + rq->kappa * z;
    r->z = z; r->ieq = ieq; r->m_r = rq->m_r; r->kappa_used = rq->kappa; r->vref = rq->vref;
    r->resume_cmp = rq->resume_cmp;
    r->vs = buck_lc_vs(ieq);
    return ieq;
}

/* ton_n in [0, T] (F09; also rejects a NaN). */
BUCK_INLINE uint32_t toc_ton_ok(const buck_toc_req_t *rq)
{
    return (rq->ton_n >= 0.0f && rq->ton_n <= BUCK_TOC_TSW) ? 1U : 0U;
}

/* Targets (SPEC 162-164, unchanged): ON-OFF Ieq - ripple/2 + q_valley, OFF-ON Ieq + ripple/2 + q_peak,
 * ripple = (Vin ton + vr (T - 2 ton))/(2L), vr = x0.v + R Ieq, ton the held duty. Then the
 * tolerances 64 eps (1 + |x0| + |target|) per component, whose sum is also the finite check of
 * every input (a NaN or an infinity in x0, Ieq, q, Vref or ton reaches a tolerance), and the
 * already-at-target test. BUSY, ALREADY or INVALID. */
BUCK_INLINE int32_t toc_targets(buck_toc_planner_t *p, const buck_toc_req_t *rq, buck_toc_plan_t *r)
{
    const float ieq = r->ieq, ton = rq->ton_n;
    const buck_xy_t x0 = r->x0;
    const float vr = x0.v + BUCK_TOC_R * ieq;
    const float ripple = (BUCK_TOC_VIN * ton + vr * (BUCK_TOC_TSW - (ton + ton))) * BUCK_TOC_HALF_INV_L;
    r->ripple = ripple;
    const float t0 = (ieq - 0.5f * ripple) + rq->q[BUCK_TOC_ON_OFF];
    const float t1 = (ieq + 0.5f * ripple) + rq->q[BUCK_TOC_OFF_ON];
    p->target[0] = t0; p->target[1] = t1;
    const float vref = rq->vref;
    const float d1 = x0.v - vref;
    const float ai = 1.0f + buck_toc_abs(x0.i);
    const float tol_v = BUCK_TOC_VERIFY_EPS * ((1.0f + buck_toc_abs(x0.v)) + buck_toc_abs(vref));
    const float tol_i0 = BUCK_TOC_VERIFY_EPS * (ai + buck_toc_abs(t0)), tol_i1 = BUCK_TOC_VERIFY_EPS * (ai + buck_toc_abs(t1));
    p->tol_i[0] = tol_i0; p->tol_i[1] = tol_i1; p->tol_v = tol_v;
    p->d1 = d1;
    if (!((tol_i0 + tol_i1) + tol_v <= BUCK_TOC_FLT_MAX))
        return BUCK_TOC_INVALID;
    if (buck_toc_abs(x0.i - t0) <= SAME * tol_i0 && buck_toc_abs(d1) <= SAME * tol_v) {
        r->order = BUCK_TOC_ON_OFF; r->target_i = t0;
        return BUCK_TOC_ALREADY;
    }
    return BUCK_TOC_BUSY;
}

/* The order the rule picks (BUCK_TOC_ORDER_RULE) from the planning state and the targets. */
BUCK_INLINE uint32_t toc_order_rule(const buck_toc_planner_t *p, const buck_toc_plan_t *r)
{
    const float v0 = r->x0.v, vref = p->vref;
#if BUCK_TOC_ORDER_RULE == 2
    /* the ideal-LC switching curve through the targets, second order in di = x0.i - Ieq:
     * di <= 0: the ON arc into the peak target, v = Vref + (L/C)(di^2 - rp^2) / (2 (Vin - Vref));
     * di > 0:  the OFF arc into the valley target, v = Vref - (L/C)(di^2 - rv^2) / (2 Vref) */
    const float ieq = r->ieq, di = r->x0.i - ieq;
    const float rv = p->target[BUCK_TOC_ON_OFF] - ieq, rp = p->target[BUCK_TOC_OFF_ON] - ieq;
    const float z2 = (float)(0.5 * BUCK_L_H / BUCK_C_F);
    const float gamma = (di <= 0.0f) ? vref + z2 * (di * di - rp * rp) / (BUCK_TOC_VIN - vref)
                                     : vref - z2 * (di * di - rv * rv) / vref;
    return (v0 < gamma) ? BUCK_TOC_ON_OFF : BUCK_TOC_OFF_ON;
#else
    return (vref - v0 > 0.0f) ? BUCK_TOC_ON_OFF : BUCK_TOC_OFF_ON;
#endif
}

/* ---- Structure clear / copy (Inc/buck_toc_plan.h) -------------------------------- */
BUCK_TOC_NOLIBCALL(buck_toc_zero) void buck_toc_zero(void *p, uint32_t bytes)
{
    uint32_t *w = (uint32_t *)p;
    uint32_t k = 0U;
    for (; k + 4U <= bytes / 4U; k += 4U) { w[k] = 0U; w[k + 1U] = 0U; w[k + 2U] = 0U; w[k + 3U] = 0U; }
    for (; k < bytes / 4U; ++k) w[k] = 0U;
}

BUCK_TOC_NOLIBCALL(buck_toc_copy) void buck_toc_copy(void *d, const void *s, uint32_t bytes)
{
    uint32_t *w = (uint32_t *)d;
    const uint32_t *r = (const uint32_t *)s;
    for (uint32_t k = 0U; k < bytes / 4U; ++k) w[k] = r[k];
}

/* ---- The six parts ------------------------------------------------------------------------- */
/* Part 1: the request part (the plan is cleared first, F13; Ieq_plan, z, vs, x0). Only the request
 * fields the later parts read are kept (vref, q, g). */
BUCK_TOC_FAST(buck_toc_fast_1) int32_t buck_toc_fast_1(buck_toc_planner_t *p, const buck_toc_req_t *rq, buck_toc_plan_t *r)
{
    p->vref = rq->vref;
    p->q[0] = rq->q[0]; p->q[1] = rq->q[1];
    p->g[0] = rq->g[0]; p->g[1] = rq->g[1];
    buck_toc_zero(r, sizeof *r);
    (void)toc_ieq(rq, r);
    if (!toc_ton_ok(rq)) return r->status = BUCK_TOC_INVALID;
    r->x0.i = rq->i_r; r->x0.v = rq->v_r;
    return BUCK_TOC_BUSY;
}

/* Part 2: the ripple, the targets, the tolerances, ALREADY; the order (BUCK_TOC_ORDER_RULE). */
BUCK_TOC_FAST(buck_toc_fast_2) int32_t buck_toc_fast_2(buck_toc_planner_t *p, const buck_toc_req_t *rq, buck_toc_plan_t *r)
{
    const int32_t st = toc_targets(p, rq, r);
    if (st != BUCK_TOC_BUSY) return r->status = st;
    p->order = toc_order_rule(p, r);
    return BUCK_TOC_BUSY;
}

/* Part 3: the circle geometry (Inc/buck_toc_plan.h): the switch point (one square root) and each
 * span's tan(theta/2) = (p x q) / (|p|^2 + p.q) as numerator/denominator. NONE: the circles do not
 * meet. */
BUCK_TOC_FAST(buck_toc_fast_3) int32_t buck_toc_fast_3(buck_toc_planner_t *p, buck_toc_plan_t *r)
{
    const uint32_t o = p->order;
    const float ieq = r->ieq, vs = r->vs;
    const float a0 = (r->x0.i - ieq) * BUCK_LC_Z, at = (p->target[o] - ieq) * BUCK_LC_Z;
    const float du = p->d1;                             /* v0 - Vref = u0 - ut */
    const float u0 = r->x0.v + vs, ut = p->vref + vs;
    const float su = u0 + ut;
    float w, as2;
    if (o == BUCK_TOC_ON_OFF) {
        w = du * (1.0f - su * BUCK_LC_HALF_INV_VIN) + ((at - a0) * (at + a0)) * BUCK_LC_HALF_INV_VIN;
        as2 = at * at - w * (w + (ut + ut));
    } else {
        w = (((a0 - at) * (a0 + at)) + du * su) * BUCK_LC_HALF_INV_VIN;
        as2 = a0 * a0 + (du - w) * (su + w);
    }
    if (!(as2 >= 0.0f)) return r->status = BUCK_TOC_NONE;           /* also a NaN */
    const float sq = buck_toc_sqrt(as2);
    const float as = (o == BUCK_TOC_ON_OFF) ? sq : -sq;
    const float us = ut + w;
    /* span 1 about its centre c1, span 2 about c2: ON-OFF c1 = (0, Vin), c2 = 0; OFF-ON c1 = 0, c2 = (0, Vin) */
    const float c1 = (o == BUCK_TOC_ON_OFF) ? BUCK_TOC_VIN : 0.0f, c2 = BUCK_TOC_VIN - c1;
    const float p1y = u0 - c1, q1y = us - c1;           /* span 1: p = (a0, p1y), q = (as, q1y) */
    const float p2y = us - c2, q2y = ut - c2;           /* span 2: p = (as, p2y), q = (at, q2y) */
    p->n1 = a0 * q1y - p1y * as;
    p->d1n = (a0 * a0 + p1y * p1y) + (a0 * as + p1y * q1y);
    p->n2 = as * q2y - p2y * at;
    p->d2n = (at * at + q2y * q2y) + (as * at + p2y * q2y);
    return BUCK_TOC_BUSY;
}

/* Part 4: one reciprocal for both tan(theta/2), the angles (2 atan) and the nominal spans
 * (theta/omega). NONE: an angle negative or the arc not shorter than half a turn; LIMIT: an angle
 * beyond 2 atan(BUCK_LC_TMAX) (longer than any frame). */
BUCK_TOC_FAST(buck_toc_fast_4) int32_t buck_toc_fast_4(buck_toc_planner_t *p, buck_toc_plan_t *r)
{
    const float d1 = p->d1n, d2 = p->d2n;
    if (!(d1 > 0.0f && d2 > 0.0f)) return r->status = BUCK_TOC_NONE;   /* theta >= pi (or NaN) */
    const float rc = 1.0f / (d1 * d2);
    const float t1 = (p->n1 * d2) * rc, t2 = (p->n2 * d1) * rc;
    if (!(t1 >= 0.0f && t2 >= 0.0f)) return r->status = BUCK_TOC_NONE;
    if (!(t1 <= BUCK_LC_TMAX && t2 <= BUCK_LC_TMAX)) return r->status = BUCK_TOC_LIMIT;
    const float h1 = buck_lc_2atan(t1) * BUCK_LC_INV_OMEGA, h2 = buck_lc_2atan(t2) * BUCK_LC_INV_OMEGA;
    const uint32_t o = p->order;
    r->ton0 = (o == BUCK_TOC_ON_OFF) ? h1 : h2;
    r->toff0 = (o == BUCK_TOC_ON_OFF) ? h2 : h1;
    return BUCK_TOC_BUSY;
}

/* Part 5: the order's data, the commanded spans g ton0, g toff0 (default F: the bank held at R) and
 * their ticks. INVALID: a gain not positive and finite, or the resume CMP1 outside its range;
 * LIMIT: a span beyond the frame limit. */
BUCK_TOC_FAST(buck_toc_fast_5) int32_t buck_toc_fast_5(buck_toc_planner_t *p, const buck_toc_req_t *rq, buck_toc_plan_t *r)
{
    const uint32_t o = p->order;
    const buck_xy_t g = p->g[o];
    r->order = o;
    r->target_i = p->target[o];
    r->q_used = p->q[o];
    r->g = g;
    r->ton = g.i * r->ton0;
    r->toff = g.v * r->toff0;
    if (!(g.i > 0.0f && g.i <= BUCK_TOC_FLT_MAX && g.v > 0.0f && g.v <= BUCK_TOC_FLT_MAX))
        return r->status = BUCK_TOC_INVALID;
    if (!(rq->resume_cmp >= BUCK_CMP_MIN && rq->resume_cmp <= BUCK_CMP_MAX))
        return r->status = BUCK_TOC_INVALID;
    const float fon = r->ton * BUCK_TOC_INV_TICK, foff = r->toff * BUCK_TOC_INV_TICK;
    if (!(fon <= (float)BUCK_TOC_PER_MAX_TICKS && foff <= (float)BUCK_TOC_PER_MAX_TICKS))
        return r->status = BUCK_TOC_LIMIT;
    const uint32_t on = (uint32_t)(fon + 0.5f), off = (uint32_t)(foff + 0.5f);
    r->on_ticks = on; r->off_ticks = off;
    r->ton_x = (float)on * BUCK_TOC_TICK;
    r->toff_x = (float)off * BUCK_TOC_TICK;
    return BUCK_TOC_BUSY;
}

/* Part 6: the frame (PER' and every limit). The final status. */
BUCK_TOC_FAST(buck_toc_fast_6) int32_t buck_toc_fast_6(const buck_toc_req_t *rq, buck_toc_plan_t *r)
{
    const uint32_t ok = buck_toc_frame_encode(r->order, r->on_ticks, r->off_ticks, rq->resume_cmp, &r->fw);
    r->per_ticks = r->fw.per;
    return r->status = ok ? BUCK_TOC_READY : BUCK_TOC_LIMIT;
}

/* A poll point: the sample service when the flag is set; 1 = abort. */
#define POLL() do { if (poll && (*poll->flag & poll->mask) && poll->svc(poll->ctx)) goto abort; } while (0)

BUCK_TOC_FAST(buck_toc_plan_poll) void buck_toc_plan_poll(const buck_toc_req_t *rq, buck_toc_plan_t *r, const buck_toc_poll_t *poll)
{
    buck_toc_planner_t p;                      /* working values only */
    POLL();
    if (buck_toc_fast_1(&p, rq, r) != BUCK_TOC_BUSY) return;
    POLL();
    if (buck_toc_fast_2(&p, rq, r) != BUCK_TOC_BUSY) return;
    POLL();
    if (buck_toc_fast_3(&p, r) != BUCK_TOC_BUSY) return;
    POLL();
    if (buck_toc_fast_4(&p, r) != BUCK_TOC_BUSY) return;
    POLL();
    if (buck_toc_fast_5(&p, rq, r) != BUCK_TOC_BUSY) return;
    POLL();
    (void)buck_toc_fast_6(rq, r);
    return;
abort:
    r->status = BUCK_TOC_ABORT;
}

BUCK_TOC_FAST(buck_toc_plan) void buck_toc_plan(const buck_toc_req_t *rq, buck_toc_plan_t *r)
{
    buck_toc_plan_poll(rq, r, NULL);
}
