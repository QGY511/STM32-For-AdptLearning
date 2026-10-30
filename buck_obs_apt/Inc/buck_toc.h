#ifndef BUCK_TOC_H
#define BUCK_TOC_H

/* Feed-forward time-optimal control (TOC), the math (stage 4, 30 Oct 2026; hardware independent,
 * single precision). The firmware (Src/buck_toc.c) and the host tests (Tests/test_toc.c) run
 * exactly this code.
 *
 * Model: the lossless L-C stage, V_in constant, the output at V_ref during the pulse:
 *   ON  di/dt = +m1 = (V_in - V_ref)/L,   OFF  di/dt = -m2 = -V_ref/L,   C dv/dt = i - I_eq.
 * Initial condition (user, stage 4): the measured cycle average i0 = i_avg and the sampled v0 =
 * v_out of the trigger pass, at the start of the first span. Equilibrium: the per-cycle load
 * estimate i_avg - (C/T)(v_out - v_out,prev) (eq. 7 on one cycle) plus the learned correction b of
 * the order (user, stage 4b), I_eq = that + b, so a = i0 - I_eq = (C/T)(v_out - v_out,prev) - b.
 * Target: the periodic steady state at I_eq, where a period starts at the valley I_eq - h (h = half
 * the ripple at the duty V_ref/V_in) with v = V_ref: the frame ends at a period boundary, so the
 * first normal period after it is an ordinary one.
 * Charge balance (the area of i - I_eq is C (V_ref - v0)), one pulse pair:
 *   ON-OFF (step-up, v0 < V_ref): peak I_eq + x,
 *     x^2 = k_dv (V_ref - v0) + (V_ref/V_in) a^2 + ((V_in - V_ref)/V_in) h^2,
 *     t_on = (x - a)/m1,  t_off = (x + h)/m2;
 *   OFF-ON (step-down, v0 > V_ref): valley I_eq - y,
 *     y^2 = ((V_in - V_ref)/V_in) a^2 + (V_ref/V_in) h^2 - k_dv (V_ref - v0),
 *     t_off = (a + y)/m2,  t_on = (y - h)/m1,
 * with k_dv = 2 C m1 m2/(m1 + m2) = 2 C (V_in - V_ref) V_ref/(L V_in). One square root (VSQRT, 14
 * clocks on the M4), every reciprocal precomputed per V_ref (buck_toc_k_init): no division and no
 * table in the solve (a table would not be faster than VSQRT). */

#include <stdint.h>
#include "buck_config.h"

#ifndef BUCK_INLINE
#define BUCK_INLINE static inline __attribute__((always_inline))
#endif

enum { BUCK_TOC_ON_OFF = 0U, BUCK_TOC_OFF_ON = 1U };
enum { BUCK_TOC_NONE = 0, BUCK_TOC_OK = 1 };
enum {                          /* what clamped a frame (g_buck.toc.sat) */
    BUCK_TOC_SAT_LIMIT1 = 1U,   /* first span at its limit (peak / valley current, or the span maximum) */
    BUCK_TOC_SAT_LATE1 = 2U,    /* first span longer than solved: the solve finished after the model's end */
    BUCK_TOC_SAT_LIMIT2 = 4U,   /* second span at its limit (minimum, maximum, the frame's range) */
    BUCK_TOC_SAT_PER = 8U       /* the frame's end moved later (PER' minimum: master CMP4 + 600, the A0 pair) */
};

/* VSQRT on the target (sqrtf would be a library call that sets errno for x < 0). */
#if defined(__arm__)
BUCK_INLINE float buck_toc_sqrt(float x)
{
    float r;
    __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
    return r;
}
#else
#include <math.h>
BUCK_INLINE float buck_toc_sqrt(float x) { return sqrtf(x); }
#endif

/* Constants of the solve and the learning for one V_ref (divisions here, once per V_ref). */
typedef struct {
    float vref;             /* the reference they are for, V */
    float h;                /* half the steady ripple at the duty V_ref/V_in, A */
    float k_dv;             /* 2 C m1 m2/(m1 + m2), A^2 per V */
    float k_on_a, k_on_h;   /* ON-OFF: V_ref/V_in, ((V_in - V_ref)/V_in) h^2 */
    float k_off_a, k_off_h; /* OFF-ON: (V_in - V_ref)/V_in, (V_ref/V_in) h^2 */
    float t_m1, t_m2;       /* ticks per A of rise (ON) and of fall (OFF): 1/(m1 tick), 1/(m2 tick) */
    float r12, r21;         /* t_m2/t_m1 = m1/m2 and its inverse */
    float m1, m2;           /* A/s */
    float cdt;              /* C/T, A per V */
} buck_toc_k_t;

BUCK_INLINE void buck_toc_k_init(buck_toc_k_t *k, float vref)
{
    const float vin = (float)BUCK_VIN_V, L = (float)BUCK_L_H, C = (float)BUCK_C_F, T = (float)BUCK_TSW_S;
    float vr = vref;
    vr = (vr > 0.05f) ? vr : 0.05f;                 /* (a NaN ends at the lower bound) */
    vr = (vr < vin - 0.05f) ? vr : vin - 0.05f;
    const float m1 = (vin - vr) / L, m2 = vr / L;
    const float h = 0.5f * (vin - vr) * (vr / vin) * T / L;
    k->vref = vref;
    k->h = h;
    k->k_dv = 2.0f * C * (vin - vr) * vr / (L * vin);
    k->k_on_a = vr / vin;
    k->k_on_h = (vin - vr) / vin * h * h;
    k->k_off_a = (vin - vr) / vin;
    k->k_off_h = vr / vin * h * h;
    k->t_m1 = (float)(1.0 / BUCK_TICK_S) / m1;
    k->t_m2 = (float)(1.0 / BUCK_TICK_S) / m2;
    k->r12 = m1 / m2;
    k->r21 = m2 / m1;
    k->m1 = m1;
    k->m2 = m2;
    k->cdt = (float)(BUCK_C_F / BUCK_TSW_S);
}

/* The solve, in two parts so that the firmware can program the first span's end before it forms the
 * second span (Src/buck_toc.c): buck_toc_solve1() gives the first span, buck_toc_solve2() the second,
 * buck_toc_ok() whether the pair is a solution. order: BUCK_TOC_ON_OFF / BUCK_TOC_OFF_ON (the
 * firmware: from the sign of v - V_ref). v, v_prev: this and the previous pass's v_out (V); im: the
 * measured average (A); b: the order's equilibrium correction (A). The spans are model ticks, before
 * the gains. No branch: a radicand below 0
 * gives a NaN span (VSQRT), which the firmware's tick conversion takes as 0 (the span then ends as
 * soon as it can; buck_toc_ok() says 0). With v < V_ref (ON-OFF) or v > V_ref (OFF-ON) the radicand
 * is positive. */
typedef struct {
    float ieq;              /* I_eq, A (not clamped: the spans are, and the held duty) */
    float a;                /* i0 - I_eq, A */
    float dv;               /* V_ref - v0, V */
    float x;                /* ON-OFF: the peak above I_eq; OFF-ON: the valley below it (A) */
    float t1, t2;           /* model spans, ticks: ON-OFF {on, off}, OFF-ON {off, on} */
} buck_toc_sol_t;

BUCK_INLINE void buck_toc_solve1(const buck_toc_k_t *k, uint32_t order, float v, float v_prev, float im, float b,
                                 buck_toc_sol_t *s)
{
    const float a = k->cdt * (v - v_prev) - b, dv = k->vref - v;
    s->ieq = im - a; s->a = a; s->dv = dv;
    if (order == BUCK_TOC_ON_OFF) {
        const float x = buck_toc_sqrt(k->k_dv * dv + k->k_on_a * (a * a) + k->k_on_h);
        s->x = x;
        s->t1 = (x - a) * k->t_m1;
    } else {
        const float y = buck_toc_sqrt(k->k_off_a * (a * a) + k->k_off_h - k->k_dv * dv);
        s->x = y;
        s->t1 = (a + y) * k->t_m2;
    }
}

BUCK_INLINE void buck_toc_solve2(const buck_toc_k_t *k, uint32_t order, buck_toc_sol_t *s)
{
    s->t2 = (order == BUCK_TOC_ON_OFF) ? (s->x + k->h) * k->t_m2 : (s->x - k->h) * k->t_m1;
}

BUCK_INLINE int32_t buck_toc_ok(const buck_toc_k_t *k, uint32_t order, const buck_toc_sol_t *s)
{
    if (order == BUCK_TOC_ON_OFF) return (s->x > s->a) ? BUCK_TOC_OK : BUCK_TOC_NONE;        /* also a NaN */
    return (s->x > k->h && s->x > -s->a) ? BUCK_TOC_OK : BUCK_TOC_NONE;
}

/* Both parts. Returns BUCK_TOC_OK, or BUCK_TOC_NONE when the pair has no positive solution. */
BUCK_INLINE int32_t buck_toc_solve(const buck_toc_k_t *k, uint32_t order, float v, float v_prev, float im, float b,
                                   buck_toc_sol_t *s)
{
    buck_toc_solve1(k, order, v, v_prev, im, b, s);
    buck_toc_solve2(k, order, s);
    return buck_toc_ok(k, order, s);
}

/* The second span after a first span that came out at t1 ticks (the solve finished after the
 * model's end, or the limit), so that the current still lands on the target valley I_eq - h:
 *   ON-OFF:  peak a + t1/t_m1 above I_eq, t2 = (a + t1/t_m1 + h) t_m2 = (a + h) t_m2 + t1 r12;
 *   OFF-ON:  valley y = t1/t_m2 - a below I_eq, t2 = (y - h) t_m1 = t1 r21 - (a + h) t_m1 (<= 0: no ON
 *            needed).
 * In ticks, before the gain; no division. */
BUCK_INLINE float buck_toc_t2_for(const buck_toc_k_t *k, uint32_t order, float a, float t1)
{
    if (order == BUCK_TOC_ON_OFF) return (a + k->h) * k->t_m2 + t1 * k->r12;
    return t1 * k->r21 - (a + k->h) * k->t_m1;
}

/* The two residuals as functions of the applied spans (seconds), the L-C model the Jacobian is
 * taken from. q1, q2: the spans' slopes (A/s, signed): ON-OFF {+m1, -m2}, OFF-ON {-m2, +m1}.
 *   z_i = a + h + q1 t1 + q2 t2                                     (end current - (I_eq - h))
 *   z_v = -dv + (a (t1 + t2) + q1 t1^2/2 + q1 t1 t2 + q2 t2^2/2)/C  (end voltage - V_ref) */
BUCK_INLINE void buck_toc_model_end(const buck_toc_k_t *k, uint32_t order, float a, float dv, float t1, float t2,
                                    float *zi, float *zv)
{
    const float q1 = (order == BUCK_TOC_ON_OFF) ? k->m1 : -k->m2, q2 = (order == BUCK_TOC_ON_OFF) ? -k->m2 : k->m1;
    *zi = a + k->h + q1 * t1 + q2 * t2;
    *zv = -dv + (a * (t1 + t2) + 0.5f * q1 * t1 * t1 + q1 * t1 * t2 + 0.5f * q2 * t2 * t2) / (float)BUCK_C_F;
}

/* The Jacobian step (after the first period). g1, g2: the gains of the first and second span of this
 * order (updated in place). t1, t2: the applied PHYSICAL spans (s); a: i0 - I_eq of the solve. sat1 /
 * sat2: the span was clamped (it cannot follow its gain). zi, zv: the measured residuals (A, V): zi the
 * landing, the first period's measured mean current minus I_eq (stage 4b: not the voltage change,
 * which also carries the equilibrium error, buck_toc_learn_b's), zv the voltage at the frame's end
 * minus V_ref. Returns 1 when the gains were updated, 0 when the residuals were not finite or beyond
 * the SKIP bounds (nothing learned). Two divisions (1/det, 1/(t1 t2)); the rest by the constants of k. */
/* (buck_toc_gclip: one gain's step, clipped to G_STEP, the gain to [G_MIN, G_MAX].) */
BUCK_INLINE float buck_toc_gclip(float g, float d)
{
    d = (d < BUCK_TOC_G_STEP) ? d : BUCK_TOC_G_STEP;
    d = (d > -BUCK_TOC_G_STEP) ? d : -BUCK_TOC_G_STEP;
    float n = g + d;
    n = (n > BUCK_TOC_G_MIN) ? n : BUCK_TOC_G_MIN;
    n = (n < BUCK_TOC_G_MAX) ? n : BUCK_TOC_G_MAX;
    return n;
}

/* In two parts (the firmware runs them in the spare tails of two passes, Src/buck_toc.c):
 * buck_toc_learn_dt(): the gates, the clips, the Jacobian and the span steps dt1, dt2 (one division);
 * buck_toc_learn_g(): the gains from them (one division). buck_toc_learn() is both. */
typedef struct { float d1, d2; } buck_toc_dt_t;

BUCK_INLINE uint32_t buck_toc_learn_dt(const buck_toc_k_t *k, uint32_t order, float a, float t1, float t2,
                                       uint32_t sat1, float zi, float zv, buck_toc_dt_t *d)
{
    if (!(zi < BUCK_TOC_ZI_SKIP_A && zi > -BUCK_TOC_ZI_SKIP_A && zv < BUCK_TOC_ZV_SKIP_V && zv > -BUCK_TOC_ZV_SKIP_V))
        return 0U;                              /* also a NaN */
    zi = (zi < BUCK_TOC_ZI_CLIP_A) ? zi : BUCK_TOC_ZI_CLIP_A;
    zi = (zi > -BUCK_TOC_ZI_CLIP_A) ? zi : -BUCK_TOC_ZI_CLIP_A;
    zv = (zv < BUCK_TOC_ZV_CLIP_V) ? zv : BUCK_TOC_ZV_CLIP_V;
    zv = (zv > -BUCK_TOC_ZV_CLIP_V) ? zv : -BUCK_TOC_ZV_CLIP_V;
    const float inv_c = (float)(1.0 / BUCK_C_F), mu = BUCK_TOC_MU, tick = (float)BUCK_TICK_S;
    const uint32_t on_off = (order == BUCK_TOC_ON_OFF);
    const float q1 = on_off ? k->m1 : -k->m2, q2 = on_off ? -k->m2 : k->m1;
    if (sat1) {
        d->d1 = 0.0f;                           /* the first span cannot move: the second takes the current */
        d->d2 = -mu * zi * (on_off ? -k->t_m2 * tick : k->t_m1 * tick);    /* -mu zi / q2 */
        return 1U;
    }
    const float j21 = (a + q1 * (t1 + t2)) * inv_c;     /* d z_v / d t1 */
    const float j22 = (a + q1 * t1 + q2 * t2) * inv_c;  /* d z_v / d t2 */
    float det = q1 * j22 - q2 * j21;
    const float dmin = k->m2 * BUCK_TOC_DET_MIN_A * inv_c;
    if (det < dmin && det > -dmin) det = (det < 0.0f) ? -dmin : dmin;
    const float r = -mu / det;
    d->d1 = r * (j22 * zi - q2 * zv);
    d->d2 = r * (-j21 * zi + q1 * zv);
    return 1U;
}

BUCK_INLINE void buck_toc_learn_g(float t1, float t2, uint32_t sat1, uint32_t sat2, const buck_toc_dt_t *d,
                                  float *g1, float *g2)
{
    if (!(t1 > 0.0f && t2 > 0.0f)) return;
    const float r12 = 1.0f / (t1 * t2);         /* 1/t1 = t2 r12, 1/t2 = t1 r12 */
    if (!sat1) *g1 = buck_toc_gclip(*g1, d->d1 * *g1 * (t2 * r12));
    if (!sat2) *g2 = buck_toc_gclip(*g2, d->d2 * *g2 * (t1 * r12));
}

BUCK_INLINE uint32_t buck_toc_learn(const buck_toc_k_t *k, uint32_t order, float a, float t1, float t2,
                                    uint32_t sat1, uint32_t sat2, float zi, float zv, float *g1, float *g2)
{
    buck_toc_dt_t d;
    if (!buck_toc_learn_dt(k, order, a, t1, t2, sat1, zi, zv, &d)) return 0U;
    buck_toc_learn_g(t1, t2, sat1, sat2, &d, g1, g2);
    return 1U;
}

/* The equilibrium correction (stage 4b, user): I_eq = the edge's per-cycle estimate + b[order], and
 * after the first period at the held duty
 *     e = I_load - I_eq,   I_load = i_P1 - (C/T)(v_1 - v_end),
 * the load measured over that period (its mean current minus its capacitor current) against the
 * equilibrium the frame was solved for;  b += mu_b e, |b| <= B_MAX. It takes the systematic part of
 * the edge estimate's error (a load still slewing at the edge, the ESR term of the edge cycle's dv, the
 * edge's place in its period); a current-sense offset or gain is in both I_load and I_eq and cancels.
 * Returns 1 when b was updated (e finite and within the SKIP bound; clipped at the CLIP bound). */
BUCK_INLINE uint32_t buck_toc_learn_b(float e, float *b)
{
    if (!(e < BUCK_TOC_ZI_SKIP_A && e > -BUCK_TOC_ZI_SKIP_A)) return 0U;   /* also a NaN */
    e = (e < BUCK_TOC_ZI_CLIP_A) ? e : BUCK_TOC_ZI_CLIP_A;
    e = (e > -BUCK_TOC_ZI_CLIP_A) ? e : -BUCK_TOC_ZI_CLIP_A;
    float n = *b + BUCK_TOC_B_MU * e;
    n = (n < BUCK_TOC_B_MAX_A) ? n : BUCK_TOC_B_MAX_A;
    n = (n > -BUCK_TOC_B_MAX_A) ? n : -BUCK_TOC_B_MAX_A;
    *b = n;
    return 1U;
}

/* b from g_buck.toc.b (writable from the debugger): outside [-B_MAX, B_MAX] or not finite -> 0. */
BUCK_INLINE float buck_toc_b_ok(float b)
{
    return (b >= -BUCK_TOC_B_MAX_A && b <= BUCK_TOC_B_MAX_A) ? b : 0.0f;
}

/* A gain from g_buck.toc.g (writable from the debugger): outside [G_MIN, G_MAX] or not finite -> 1. */
BUCK_INLINE float buck_toc_gain_ok(float g)
{
    return (g >= BUCK_TOC_G_MIN && g <= BUCK_TOC_G_MAX) ? g : 1.0f;
}

/* The held duty after the frame: the duty that holds I_eq, (V_ref + r_L I_eq)/V_in, clamped to the
 * duty range; as a CMP1 value (+ DT; the firmware takes DT off again when the frame ends with the
 * high side on, OFF-ON, where the first period's on-time has no rising dead time). */
BUCK_INLINE float buck_toc_hold_duty(float vref, float ieq)
{
    float d = (vref + (float)BUCK_RL_OHM * ieq) * (float)(1.0 / BUCK_VIN_V);
    d = (d > BUCK_D_MIN) ? d : BUCK_D_MIN;
    d = (d < BUCK_D_MAX) ? d : BUCK_D_MAX;
    return d;
}

#endif
