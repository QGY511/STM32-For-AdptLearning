#ifndef BUCK_TOC_PLAN_H
#define BUCK_TOC_PLAN_H

/* Time-optimal control (TOC) pulse pair on the LOSSLESS L-C model (stage 3, 30 Oct 2026, user
 * decision 1c: the coupled [1/1] Padé (trapezoidal) model of stages 1-2b is removed, with its
 * planner, its span maps and its derivatives; they are in the backups). Single precision, no heap,
 * no library call (VSQRT inline on the target, sqrtf on the host). Hardware independent: the host
 * tests (Tests/test_toc.c) and the firmware execute this code.
 *
 * Model (configured, not identified; buck_config.h): x = [i_L (A), v (V)], gate s (1: high side on),
 * a constant load current Ieq (the episode's), ideal L and C, Vin:
 *   L di/dt = s Vin - v - vs,   C dv/dt = i - Ieq,
 * vs = 0 (BUCK_TOC_LC_RDROP = 0, the default: what decision 1c names). BUCK_TOC_LC_RDROP = 1 (a
 * question, stage3 S3-Q2; not the decision) puts the resistor's DC drop vs = R Ieq into both
 * sources: still the lossless circle geometry, used the same way everywhere the model is used.
 * With omega = 1/sqrt(LC), Z = sqrt(L/C) and the scaled coordinates a = Z (i - Ieq) (V),
 * y = v + vs - s Vin (V), a span of length h is the exact rotation by theta = omega h about the
 * span's centre (counter-clockwise in the (a, y) plane):
 *   a' = cos(theta) a - sin(theta) y,   y' = sin(theta) a + cos(theta) y.
 * In (i, v): i' = Ieq + c di - (s/Z) dy, v' = s Vin - vs + Z s di + c dy, di = i - Ieq,
 * dy = v + vs - s Vin (buck_lc_map). Its state Jacobian is the same rotation for every centre:
 * Phi(h) = [[c, -s/Z], [Z s, c]] (buck_lc_phi), so the tangent through any sequence of spans is
 * the rotation by omega times their total length. The vector field (the derivative of a span's end
 * by its length): f = ((s Vin - v - vs)/L, (i - Ieq)/C) (buck_lc_f). The charge of a span:
 * integral i dt = Ieq h + C (v_end - v_start) (exact for this model).
 * cos/sin by a Taylor polynomial (buck_lc_sincos: 0 <= theta <= 1.35 rad, error < 6e-7; a frame
 * span is at most omega * BUCK_TOC_PER_MAX_TICKS ticks = 1.25 rad).
 *
 * The planner (closed form, "circle geometry"; buck_toc_plan / the parts buck_toc_fast_1..6):
 * the ON arcs are circles about (0, Vin), the OFF arcs about (0, 0) in the (a, u) plane, u = v + vs.
 * The pair ends on the target T = (a_t, u_t) = (Z (target_i - Ieq), Vref + vs):
 *   ON-OFF: the ON circle through x0 meets the OFF circle through T at
 *             u_s = u_t + w,  w = du (1 - (u0 + u_t)/(2 Vin)) + (a_t - a0)(a_t + a0)/(2 Vin),
 *             a_s = +sqrt(a_t^2 - w (w + 2 u_t))           (the switch with i above Ieq);
 *   OFF-ON: the OFF circle through x0 meets the ON circle through T at
 *             u_s = u_t + w,  w = ((a0 - a_t)(a0 + a_t) + du (u0 + u_t))/(2 Vin),
 *             a_s = -sqrt(a0^2 + (du - w)(u0 + u_t + w))   (the switch with i below Ieq);
 *   du = v0 - Vref (no cancellation: every term is a small difference). One square root, no
 *   division (1/(2 Vin) is a constant). The sign of a_s is the stage-2b choice "the larger u" (the
 *   longer on-time; stage2b S2B-Q3); the other intersection is not considered (stage3 S3-Q4).
 *   Each span's angle from its start vector p to its end vector q about its centre:
 *   tan(theta/2) = (p x q) / (|p|^2 + p.q) (|p| = |q| on one circle), one reciprocal for both
 *   spans, theta = 2 atan(t) (odd polynomial, |error| < 2.3e-7 rad for t <= BUCK_LC_TMAX), span =
 *   theta/omega. No candidate (NONE): the circles do not meet, or an angle is negative (the arc
 *   would be longer than half a turn). An angle above 2 atan(BUCK_LC_TMAX) = 1.32 rad (longer than
 *   any frame): LIMIT.
 * The order (stage 2b decision S2-Q5, BUCK_TOC_ORDER_RULE), the targets (SPEC 162-164: ON-OFF Ieq
 * - ripple/2 + q_valley, OFF-ON Ieq + ripple/2 + q_peak, the existing ripple formula), the
 * ALREADY test, the gains (default F), the ticks and the frame (Inc/buck_toc_frame.h) are the
 * stage-2b ones. */

#include <stdint.h>
#include "buck_ctrl.h"
#include "buck_toc_frame.h"

/* The planner and the learning jobs run from CCM in the firmware (zero wait states, as
 * the loop; stage-1 verifier F08: the clock counts assume it). */
#if defined(__arm__)
#define BUCK_TOC_FAST(fn) __attribute__((section(".ccmtext.toc." #fn)))   /* one section each: --gc-sections */
#else
#define BUCK_TOC_FAST(fn)
#endif

/* Word copy / clear of a structure without a library call (-fno-builtin: GCC would call
 * memset/memcpy from flash for a large struct or a recognised loop). Out of line, in CCM
 * (a function with its own optimize attribute is not inlined). */
#if defined(__GNUC__) && !defined(__clang__)
#define BUCK_TOC_NOLIBCALL(fn) __attribute__((noinline, optimize("no-tree-loop-distribute-patterns"))) BUCK_TOC_FAST(fn)
#else
#define BUCK_TOC_NOLIBCALL(fn)
#endif
void buck_toc_zero(void *p, uint32_t bytes);
void buck_toc_copy(void *d, const void *s, uint32_t bytes);

/* 0 (default, user decision 1c): the lossless model, vs = 0. 1: vs = R Ieq (the DC drop of the
 * resistor in both sources; stage3 questions S3-Q2). Every user of the model takes vs from
 * buck_lc_vs(). */
#ifndef BUCK_TOC_LC_RDROP
#define BUCK_TOC_LC_RDROP 0
#endif

/* ---- Model constants (compile time, rounded once to float) ---------------- */
#define BUCK_LC_OMEGA_D  (1.0 / __builtin_sqrt(BUCK_L_H * BUCK_C_F))   /* 105940 rad/s */
#define BUCK_LC_Z_D      (__builtin_sqrt(BUCK_L_H / BUCK_C_F))          /* 34.96 mOhm */
#define BUCK_LC_OMEGA    ((float)BUCK_LC_OMEGA_D)
#define BUCK_LC_INV_OMEGA ((float)(1.0 / BUCK_LC_OMEGA_D))
#define BUCK_LC_Z        ((float)BUCK_LC_Z_D)
#define BUCK_LC_INV_Z    ((float)(1.0 / BUCK_LC_Z_D))
#define BUCK_LC_TICK_RAD ((float)(BUCK_LC_OMEGA_D * BUCK_TICK_S))      /* omega per HRTIM tick */
#define BUCK_LC_HALF_INV_VIN ((float)(0.5 / BUCK_VIN_V))
#define BUCK_LC_TMAX     0.78f       /* tan(theta/2) above this: theta > 1.32 rad, longer than any frame */
#define BUCK_TOC_INV_L  ((float)(1.0 / BUCK_L_H))
#define BUCK_TOC_INV_C  ((float)(1.0 / BUCK_C_F))
#define BUCK_TOC_C      ((float)BUCK_C_F)
#define BUCK_TOC_R      ((float)BUCK_RL_OHM)
#define BUCK_TOC_VIN    ((float)BUCK_VIN_V)
#define BUCK_TOC_TSW    ((float)BUCK_TSW_S)
#define BUCK_TOC_INV_TICK ((float)(1.0 / BUCK_TICK_S))            /* ticks per second */
#define BUCK_TOC_TICK   ((float)BUCK_TICK_S)
/* Ripple of the target construction (unchanged since stage 1; backup buck_event_learning.c:173-176):
 * ripple = ((Vin - vr) ton + vr toff) / (2L), vr = v + R Ieq. */
#define BUCK_TOC_HALF_INV_L ((float)(0.5 / BUCK_L_H))
/* Roundoff allowances (not control limits): the already-at-target test 4 eps of the tolerance
 * 64 eps (1 + |x0| + |target|) per component (backup buck_toc.c:89-91, 100-101). */
#define BUCK_TOC_VERIFY_EPS   (64.0f * 1.1920929e-7f)
#define BUCK_TOC_SAME_EPS     (4.0f * 1.1920929e-7f)
#define BUCK_TOC_FLT_MAX      3.40282347e38f

enum { BUCK_TOC_ON_OFF = 0U, BUCK_TOC_OFF_ON = 1U };

typedef struct { float i, v; } buck_xy_t;           /* state [A, V], or a gain pair {on, off} */
typedef struct { float a, b, c, d; } buck_m2_t;     /* [[a, b], [c, d]] */

/* |x| as one VABS (__builtin_fabsf: a builtin also under -fno-builtin); 1 if x
 * is finite (NaN and +-inf give 0): one compare, no library call. */
BUCK_INLINE float buck_toc_abs(float x) { return __builtin_fabsf(x); }
BUCK_INLINE uint32_t buck_toc_fin(float x) { return (__builtin_fabsf(x) <= BUCK_TOC_FLT_MAX) ? 1U : 0U; }
BUCK_INLINE float buck_toc_max(float x, float y) { return (x > y) ? x : y; }

#if defined(__arm__)
BUCK_INLINE float buck_toc_sqrt(float x)
{
    float y;
    __asm__("vsqrt.f32 %0, %1" : "=t"(y) : "t"(x));
    return y;
}
#else
float sqrtf(float);
BUCK_INLINE float buck_toc_sqrt(float x) { return sqrtf(x); }
#endif

/* ---- The L-C model -------------------------------------------------------------------- */
/* vs of the episode's load (0 unless BUCK_TOC_LC_RDROP). */
BUCK_INLINE float buck_lc_vs(float ieq)
{
#if BUCK_TOC_LC_RDROP
    return BUCK_TOC_R * ieq;
#else
    (void)ieq;
    return 0.0f;
#endif
}

/* cos and sin of theta, 0 <= theta <= 1.35 rad (the longest span of a frame is 1.25 rad, of q's
 * post-D rotation 0.36 rad): Taylor to theta^10 / theta^9, Horner in theta^2 (truncation < 6e-8 /
 * 6e-7 at 1.35 rad; with the scaled radius of the ON circle (~5.2 V) that is < 0.1 mA of current).
 * 11 multiplies, 9 constants. */
typedef struct { float c, s; } buck_cs_t;
BUCK_INLINE buck_cs_t buck_lc_sincos(float th)
{
    const float z = th * th;
    float c = -2.75573192e-7f;                         /* -1/10! */
    c = c * z + 2.48015873e-5f;                        /* 1/8! */
    c = c * z - 1.38888889e-3f;                        /* -1/6! */
    c = c * z + 4.16666667e-2f;                        /* 1/4! */
    c = c * z - 0.5f;
    c = c * z + 1.0f;
    float s = 2.75573192e-6f;                          /* 1/9! */
    s = s * z - 1.98412698e-4f;                        /* -1/7! */
    s = s * z + 8.33333333e-3f;                        /* 1/5! */
    s = s * z - 0.166666667f;
    s = (s * z) * th + th;
    const buck_cs_t r = { c, s };
    return r;
}

/* 2 atan(t), 0 <= t <= BUCK_LC_TMAX: an odd degree-11 polynomial (fitted, |atan error| < 2.3e-7). */
BUCK_INLINE float buck_lc_2atan(float t)
{
    const float z = t * t;
    float p = -2.300727554e-02f;
    p = p * z + 7.429070771e-02f;
    p = p * z - 1.316201538e-01f;
    p = p * z + 1.982192099e-01f;
    p = p * z - 3.332087100e-01f;
    p = p * z + 9.999974966e-01f;
    return (p + p) * t;
}

/* One span: x after h seconds with gate g, its rotation cs = buck_lc_sincos(omega h). */
BUCK_INLINE buck_xy_t buck_lc_map(buck_cs_t cs, buck_xy_t x, uint32_t g, float ieq, float vs)
{
    const float src = (g ? BUCK_TOC_VIN : 0.0f) - vs;
    const float di = x.i - ieq, dy = x.v - src;
    const buck_xy_t z = { ieq + (cs.c * di - (cs.s * BUCK_LC_INV_Z) * dy), src + ((cs.s * BUCK_LC_Z) * di + cs.c * dy) };
    return z;
}

/* The vector field (d/dt of a span's end by its length) at x with gate g. */
BUCK_INLINE buck_xy_t buck_lc_f(buck_xy_t x, uint32_t g, float ieq, float vs)
{
    const buck_xy_t f = { (((g ? BUCK_TOC_VIN : 0.0f) - vs) - x.v) * BUCK_TOC_INV_L, (x.i - ieq) * BUCK_TOC_INV_C };
    return f;
}

/* The tangent through a span (any centre): Phi(h) y. */
BUCK_INLINE buck_xy_t buck_lc_phi(buck_cs_t cs, buck_xy_t y)
{
    const buck_xy_t z = { cs.c * y.i - (cs.s * BUCK_LC_INV_Z) * y.v, (cs.s * BUCK_LC_Z) * y.i + cs.c * y.v };
    return z;
}

/* Endpoint Jacobian d[i, v]_end / d[ton, toff] of a pair from the first span's field at the split
 * (f1), the second span's rotation (cs2) and its field at the end (f2): columns [ton, toff] for
 * both orders. */
BUCK_INLINE buck_m2_t buck_lc_jt(uint32_t order, buck_xy_t f1, buck_cs_t cs2, buck_xy_t f2)
{
    const buck_xy_t q = buck_lc_phi(cs2, f1);
    const buck_m2_t on_off = { q.i, f2.i, q.v, f2.v }, off_on = { f2.i, q.i, f2.v, q.v };
    return (order == BUCK_TOC_ON_OFF) ? on_off : off_on;
}

/* A pair from x (h1, h2: the spans in time order; order: the first gate is ON for ON-OFF): the end,
 * and (if mid/jt are not NULL) the split and the endpoint Jacobian. The host tests' and the G and q
 * jobs' common form. Two sin/cos. */
buck_xy_t buck_lc_pair(buck_xy_t x, float h1, float h2, uint32_t order, float ieq, float vs, buck_xy_t *mid,
                       buck_m2_t *jt);

/* ---- The planning state x0 from the measured average (stage 3, user decision 1c) -------------
 * No estimator: x0.i is the inductor current at the start of a period of the held duty, from the
 * measured four-sample average of a period with that duty and the model's piecewise-linear ripple
 * (the slopes of the L-C model with the sampled v constant over the period):
 *   s_on = (Vin - v - vs)/L, s_off = -(v + vs)/L, t_on = (CMP1 - DT) tick,
 *   i(tau) = i0 + s_on min(tau, t_on) + s_off max(tau - t_on, 0)  (model time tau from the period's
 *   start, where the high side turns on: register time - DT),
 *   i_meas = (1/4) sum_j i(tau_j), tau_j = (2j+1) T/8 - DT tick (the ADC2 instants),
 * so  x0.i = i_meas - (1/4) sum_j [s_on min(tau_j, t_on) + s_off max(tau_j - t_on, 0)]   (the valley),
 *     x0.v = v (the ADC1 sample: the mean of the pair before the period boundary).
 * cmp: the held CMP1 (the request pass's). */
BUCK_INLINE float buck_lc_valley(float i_meas, float v, uint32_t cmp, float vs)
{
    const float ton = (float)(cmp - BUCK_DT_TICKS) * BUCK_TOC_TICK;
    const float s_on = ((BUCK_TOC_VIN - vs) - v) * BUCK_TOC_INV_L, s_off = -(v + vs) * BUCK_TOC_INV_L;
    float sum = 0.0f;
    for (uint32_t j = 0U; j < 4U; ++j) {
        const float tau = (float)(BUCK_ISAMPLE_CMP(j) - BUCK_DT_TICKS) * BUCK_TOC_TICK;
        sum += (tau <= ton) ? buck_mul(s_on, tau) : (buck_mul(s_on, ton) + buck_mul(s_off, tau - ton));   /* no VMLA */
    }
    return i_meas - 0.25f * sum;
}

/* ---- Request inputs (held at R; SPEC 152-175) ------------------------------- */
typedef struct {
    float i_r, v_r;        /* the planning state x0 (the firmware: buck_lc_valley of R's sample, v_R) */
    float ton_n;           /* on-time of the held duty, s, in [0, T] (else INVALID): the ripple */
    float m_r;             /* raw load-observer output at R, A (obs.ih) */
    float raw_before;      /* raw pair 'before' (the previous normal rise's m), A */
    float delta_i;         /* published DeltaI, A */
    float kappa;           /* published kappa (jump coefficient), A/A */
    float q[2];            /* published q[order], A */
    buck_xy_t g[2];        /* published gain banks {g_on, g_off}, frozen here (default F) */
    float vref;            /* reference in use, V */
    uint32_t resume_cmp;   /* CMP1 of the first normal period after the frame (OFF-ON frame length, Q4) */
} buck_toc_req_t;

enum {
    BUCK_TOC_BUSY = 0,         /* parts remain */
    BUCK_TOC_READY = 1,        /* a pair within the frame limits, its frame encoded */
    BUCK_TOC_ALREADY = 2,      /* x0 is the valley target: no pulse (backup buck_toc.c:100) */
    BUCK_TOC_NONE = -1,        /* no pair of the order (the circles do not meet, or an arc over half a turn) */
    BUCK_TOC_LIMIT = -2,       /* the pair violates the frame limits (default H) */
    BUCK_TOC_INVALID = -3,     /* a non-finite input or intermediate, or ton_n outside [0, T] */
    BUCK_TOC_ABORT = -4        /* the poll hook asked to stop (firmware: STOP or a bad sample) */
};

/* The plan (= the episode saved at R, SPEC 170-172, plus the frame). It is
 * cleared when a plan starts (stage-1 verifier F13: nothing of a previous plan
 * remains); afterwards only the fields of the parts that ran are set, so only a
 * READY plan is complete. Users check status first. */
typedef struct {
    int32_t status;
    uint32_t order;            /* BUCK_TOC_ON_OFF / BUCK_TOC_OFF_ON */
    buck_xy_t x0;              /* planning state */
    float ieq;                 /* Ieq_plan = m_R + DeltaI + kappa z, A */
    float vs;                  /* the model's source shift (buck_lc_vs(ieq)) */
    float z;                   /* raw-load jump m_R - raw_before, A */
    float m_r, kappa_used;
    float target_i, vref;      /* the selected order's target */
    float ripple;              /* nominal ripple of the target construction, A */
    float ton0, toff0;         /* planner (nominal) durations, s */
    buck_xy_t g;               /* g_used {on, off} (the order's bank) */
    float q_used;              /* q[order] at R */
    float ton, toff;           /* commanded g ton0, g toff0, s */
    uint32_t on_ticks, off_ticks, per_ticks;   /* executed spans and the frame period PER', ticks */
    float ton_x, toff_x;       /* executed spans (the ticks), s */
    uint32_t resume_cmp;       /* the request's resume CMP1 */
    buck_toc_frame_words_t fw; /* the frame (Inc/buck_toc_frame.h) */
} buck_toc_plan_t;

/* The planner's working values between its parts. */
typedef struct {
    float target[2], tol_i[2], tol_v;
    float d1;                      /* x0.v - Vref */
    float vref, q[2];
    buck_xy_t g[2];
    uint32_t order;
    float n1, d1n, n2, d2n;        /* the two spans' tan(theta/2) as num/den */
} buck_toc_planner_t;

/* Poll hook of the polled form: between its parts it tests *flag & mask and, if set, calls
 * svc(ctx); a nonzero return ends the plan with BUCK_TOC_ABORT. NULL: no polling. */
typedef struct {
    const volatile uint32_t *flag;
    uint32_t mask;
    uint32_t (*svc)(void *ctx);
    void *ctx;
} buck_toc_poll_t;

/* The order rule (stage 2b, user decision S2-Q5 "plan only the pulse order the sign of the error
 * calls for"):
 *   BUCK_TOC_ORDER_RULE 1 (default): Vref - x0.v > 0 -> ON-OFF, otherwise OFF-ON;
 *   BUCK_TOC_ORDER_RULE 2: the side of the ideal-LC (R = 0) switching curve through the two targets,
 *       to second order in the current error (stage2b S2B-Q1; one division). */
#ifndef BUCK_TOC_ORDER_RULE
#define BUCK_TOC_ORDER_RULE 1
#endif

/* The planner in six parts (the firmware polls its samples between them; each part <= ~150 clocks
 * from CCM, emulator):
 *   1 the request part (the plan cleared, Ieq_plan, z, x0, vs);
 *   2 the ripple, the targets, the tolerances, ALREADY, the order;
 *   3 the circle geometry (one square root): both spans' tan(theta/2) as num/den;
 *   4 one reciprocal, the two angles (2 atan), the nominal spans;
 *   5 the gains and the ticks;
 *   6 the frame (PER' and every limit).
 * Parts 1-5 return BUSY to go on, else the final status (also in out->status); part 6 the final
 * status. p is working storage (any contents); rq must stay unchanged until part 6. */
int32_t buck_toc_fast_1(buck_toc_planner_t *p, const buck_toc_req_t *rq, buck_toc_plan_t *out);
int32_t buck_toc_fast_2(buck_toc_planner_t *p, const buck_toc_req_t *rq, buck_toc_plan_t *out);
int32_t buck_toc_fast_3(buck_toc_planner_t *p, buck_toc_plan_t *out);
int32_t buck_toc_fast_4(buck_toc_planner_t *p, buck_toc_plan_t *out);
int32_t buck_toc_fast_5(buck_toc_planner_t *p, const buck_toc_req_t *rq, buck_toc_plan_t *out);
int32_t buck_toc_fast_6(const buck_toc_req_t *rq, buck_toc_plan_t *out);
/* The six parts with a poll between them (the host tests; the firmware calls the parts itself). */
void buck_toc_plan_poll(const buck_toc_req_t *rq, buck_toc_plan_t *out, const buck_toc_poll_t *poll);
void buck_toc_plan(const buck_toc_req_t *rq, buck_toc_plan_t *out);

/* Resume CMP1 for a load Ieq: duty (Vref + R Ieq)/Vin clamped to
 * [BUCK_D_MIN, BUCK_D_MAX] (default E; a NaN ends at BUCK_D_MIN as in
 * buck_pi_out_kt). Also the duty preload of the handover (buck_learn.h). The SPEC's formula with
 * R0 (unchanged; not the L-C model). */
BUCK_INLINE float buck_toc_eq_duty(float vref, float ieq)
{
    float d = (vref + BUCK_TOC_R * ieq) * (float)(1.0 / BUCK_VIN_V);
    d = (d > BUCK_D_MIN) ? d : BUCK_D_MIN;
    d = (d < BUCK_D_MAX) ? d : BUCK_D_MAX;
    return d;
}

BUCK_INLINE uint32_t buck_toc_resume_cmp(float vref, float ieq)
{
    return buck_duty_to_cmp(buck_toc_eq_duty(vref, ieq));
}

#endif
