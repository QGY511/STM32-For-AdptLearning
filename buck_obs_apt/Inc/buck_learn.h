#ifndef BUCK_LEARN_H
#define BUCK_LEARN_H

/* The four learning paths of control_pseudocode.txt (SPEC) and the TOC completion math, on the
 * lossless L-C model of Inc/buck_toc_plan.h (stage 3, 30 Oct 2026, user decision 1c: the same model
 * as the planner, everywhere; the Padé forms of stages 1-2b are removed). Single precision, no
 * heap, no library call. The jobs run in the learning periods after D (Src/buck_toc.c toc_learn,
 * stage 2b decision S2-Q6), one part between two polls.
 *   DeltaI   every normal period, inline (buck_learn_delta_i): the scalar step with T = the nominal
 *            period, a constant gain (no division). No model (unchanged).
 *   q        at S = B (both orders, BUCK_LEARN_QPEAK_AT_B): buck_learn_q_* (Jq through the executed
 *            pair and the post-D spans), straight-line in 4 parts (stage 3, user decision 2).
 *   G        Jacobian at S0 with the bank frozen at R (default F), one part, and the update at B:
 *            buck_learn_g_*.
 *   kappa    at E (= D here): buck_learn_kappa. No model (unchanged).
 * At D: the handover/preload (default E, buck_learn_handover; the SPEC's R0 formula), the L-C
 * prediction of the executed frame and the correction of i(D) (buck_toc_frame_*), the extra observer
 * step over the frame (decision 5, buck_toc_obs_step_d). */

#include <stdint.h>
#include "buck_toc_plan.h"

/* ---- Scalar step (SPEC 212-218), Iscale = 1 A ---------------------------------
 * step = eta j r / (j^2 + lambda), j = J/sigma, r = residual/sigma, clamped to
 * [-cap, cap]. inv_sigma = 1/sigma. A NaN step stays NaN (the callers reject
 * it). One division. */
BUCK_INLINE float buck_learn_scalar_step(float J, float res, float eta, float lambda, float inv_sigma, float cap)
{
    const float j = J * inv_sigma, r = res * inv_sigma;
    float s = (eta * j * r) / (j * j + lambda);
    s = (s > cap) ? cap : s;
    s = (s < -cap) ? -cap : s;
    return s;
}

/* ---- DeltaI, once per normal period (SPEC 177-187, 204) -----------------------
 * Vprediction = a.V + T/C (Iavg - a.Ic_used), nu = V - Vprediction, J = -T/C,
 * future.DeltaI = DeltaI + scalar_step(J, nu, .25, .001, .02 V, 1 A). With T the
 * nominal period (every normal period is exactly T here; the anchor is
 * invalidated across a TOC, a request or a missed pass) the step is
 * clamp(KD nu, +-1 A) with KD = eta j / (sigma (j^2 + lambda)), j = -T/(C sigma)
 * = -0.30864: KD = -40.08 A/V, folded at compile time. */
#define BUCK_LEARN_KTC   ((float)(BUCK_TSW_S / BUCK_C_F))                     /* T/C, V/A */
#define BUCK_LEARN_JL    (-(BUCK_TSW_S / BUCK_C_F) / BUCK_LEARN_SIGMA_LOAD_V)  /* j of the load step */
#define BUCK_LEARN_KD    ((float)(BUCK_LEARN_ETA_LOAD * BUCK_LEARN_JL / \
                          (BUCK_LEARN_SIGMA_LOAD_V * (BUCK_LEARN_JL * BUCK_LEARN_JL + BUCK_LEARN_LAMBDA_LOAD))))
#define BUCK_LEARN_CAPL  ((float)BUCK_LEARN_CAP_LOAD_A)
#define BUCK_LEARN_RAWL  ((float)BUCK_LEARN_RAW_LIMIT_A)

typedef struct { float v, m, ic; uint32_t valid; } buck_learn_anchor_t;   /* normal_anchor {V, m, Ic_used} */

/* future.DeltaI from this rise's V, the interval average current Iavg (the raw
 * cycle average i_meas) and m (obs.ih). Without a valid anchor, with a raw-load
 * change above 1 A, or with a non-finite step: DeltaI unchanged. The caller then
 * sets the anchor (buck_learn_anchor_set) with the returned value. */
BUCK_INLINE float buck_learn_delta_i(const buck_learn_anchor_t *a, float v, float iavg, float m, float delta_i)
{
    const float dm = m - a->m;
    if (!a->valid || !(dm <= BUCK_LEARN_RAWL && dm >= -BUCK_LEARN_RAWL)) return delta_i;
    const float nu = v - (a->v + BUCK_LEARN_KTC * (iavg - a->ic));
    float s = BUCK_LEARN_KD * nu;
    if (!buck_toc_fin(s)) return delta_i;
    s = (s > BUCK_LEARN_CAPL) ? BUCK_LEARN_CAPL : s;
    s = (s < -BUCK_LEARN_CAPL) ? -BUCK_LEARN_CAPL : s;
    return delta_i + s;
}

BUCK_INLINE void buck_learn_anchor_set(buck_learn_anchor_t *a, float v, float m, float delta_i_future)
{
    a->v = v; a->m = m; a->ic = m + delta_i_future; a->valid = 1U;
}

/* ---- Job status ------------------------------------------------------------------ */
enum {
    BUCK_LEARN_BUSY = 0,
    BUCK_LEARN_OK = 1,          /* a result */
    BUCK_LEARN_REJECT = -1,     /* not valid for teaching (SPEC 273-286): coefficient unchanged */
    BUCK_LEARN_SINGULAR = -2,   /* Jacobian / normal equations singular in float */
    BUCK_LEARN_NONFINITE = -3   /* a non-finite intermediate */
};

/* ---- q (SPEC 220-236, 264-271) -----------------------------------------------------
 * Jq = d(V_S - V_D)/dq = (1/C) d(integral_D^S i_L dt)/dq (equal for the model with the episode's load
 * fixed) through the executed pair (tangent d_tau_applied/dq = diag(tau_x/tau0) J_tau0^-1 [1, 0],
 * J_tau0 the endpoint Jacobian at the nominal plan from the plan's x0) and the observed post-D
 * spans (durations fixed: the tangent turns by Phi of their total length, any centres):
 *   ON-OFF (S = B): A = D (the frame ends on the rise), then the A->B cycle: h = per1.
 *   OFF-ON: BUCK_LEARN_QPEAK_AT_B = 1 (default, stage2 S2-Q3): S = B, post-D spans tail + per1;
 *           0: S = A, the tail alone (the SPEC; the firmware then has no V_A and does not start
 *           the job).
 *   Jq = (Phi(h) dx_D).v - dx_D.v, dx_D = J_tau_x [d ton/dq, d toff/dq].
 * h is derived from the spans (stage-1 verifier F14). The job copies the plan fields it needs at
 * start (F12): a new plan does not change a running job. Parts (stage 3, user decision 2: q's
 * learning straight-line): J0 (J_tau0, two sin/cos), SENS (the duration tangent, one division), JX
 * (the executed pair's Jacobian and dx_D, two sin/cos), FIN (the post-D rotation, Jq, the floor,
 * 1/sigmaV, the step: two divisions). */
#ifndef BUCK_LEARN_QPEAK_AT_B
#define BUCK_LEARN_QPEAK_AT_B 1
#endif

typedef struct {
    /* the episode (copied from the plan at start) */
    uint32_t order;
    buck_xy_t x0;
    float ieq, vs, ton0, toff0, ton_x, toff_x, target_i, vref, q_used;
    /* inputs (buck_learn_q_start) */
    float i_d, v_d;            /* corrected model i at D (i_D_rec) and raw V at D */
    float h;                   /* t_S - t_D, s (derived from the post-D spans) */
    float v_s;                 /* measured V at S */
    uint32_t raw_ok;           /* |m_S - m_D| <= 1 A and the A/B observation is the correct one (caller) */
    /* result */
    int32_t status;
    float jq;                  /* V/A */
    float q_new;               /* q_used + step (status OK) */
    /* working */
    uint32_t stage;
    buck_m2_t jt;
    buck_xy_t dx;
    float son, soff;
} buck_learn_q_t;

/* tail: OFF-ON D->A (s); on1, per1: the A->B cycle (s). */
void buck_learn_q_start(buck_learn_q_t *j, const buck_toc_plan_t *ep, float i_d, float v_d, float tail,
                        float on1, float per1, float v_s, uint32_t raw_ok);
int32_t buck_learn_q_step(buck_learn_q_t *j);
#define BUCK_LEARN_Q_PARTS 4U

/* ---- G (SPEC 89-98, 238-251) ----------------------------------------------------------
 * At S0: Jg = J_tau(x0 = [plan i0, live V(S0)], tau_command = diag(g) tau0) diag(tau0), cached
 * once (one part, J: the L-C pair and its endpoint Jacobian). Then, at the gate's return (B here),
 * buck_learn_g_teach_start() with the D endpoint and the parts SOLVE1 (the scaled 2x2 regularised
 * Gauss-Newton system), SOLVE2 (one reciprocal: dg) and UPDATE (common alpha <= 2 % relative, up to
 * two divisions): g_new = g_used + alpha dg, for the next plan of that bank. */
typedef struct {
    /* the G episode (buck_learn_g_start) */
    buck_m2_t jg;              /* Jg_saved */
    buck_xy_t g_used;          /* {g_on, g_off} */
    float target_i, vref;
    uint32_t order;
    /* result */
    int32_t status;            /* Jacobian: BUSY / OK; teach: BUSY / OK / SINGULAR / NONFINITE */
    buck_xy_t g_new;
    /* working */
    uint32_t stage;
    buck_xy_t x0, end, dg;
    float h1, h2, ton0, toff0, ieq, vs;
    float h00, h01, h11, z0, z1, det;
} buck_learn_g_t;

enum { BUCK_LEARN_G_J = 0U, BUCK_LEARN_G_WAIT };   /* the stages a caller tests (the Jacobian's, then the wait) */
void buck_learn_g_start(buck_learn_g_t *g, const buck_toc_plan_t *ep, float v_s0);
void buck_learn_g_teach_start(buck_learn_g_t *g, float i_d, float v_d);
int32_t buck_learn_g_step(buck_learn_g_t *g);

/* ---- kappa at E (SPEC 80-84) -----------------------------------------------------------
 * kappa = kappa_used + eta z (m_E - (m_R + kappa_used z)) / (eps + z^2).
 * Returns 1 and writes *kappa, or 0 (non-finite: kappa unchanged). One division. */
uint32_t buck_learn_kappa(float kappa_used, float m_r, float z, float m_e, float *kappa);

/* ---- Handover at D (SPEC 111-117, default E) ---------------------------------------------
 * Ieq_end = m (latest, after the D observer step) + DeltaI (old);
 * integ_v = Ieq_end clamped to [BUCK_IREF_MIN_A, BUCK_IREF_MAX_A];
 * integ_i = (Vref + R Ieq_end)/Vin clamped to [BUCK_D_MIN, BUCK_D_MAX];
 * the restart CMP1 from integ_i. A NaN Ieq_end ends at the low limits (as
 * buck_pi_out_kt). No division. */
typedef struct { float ieq_end, integ_v, integ_i; uint32_t cmp; } buck_toc_handover_t;

BUCK_INLINE void buck_learn_handover(float m, float delta_i, float vref, buck_toc_handover_t *h)
{
    const float ieq = m + delta_i;
    float iv = (ieq > BUCK_IREF_MIN_A) ? ieq : BUCK_IREF_MIN_A;
    iv = (iv < BUCK_IREF_MAX_A) ? iv : BUCK_IREF_MAX_A;
    h->ieq_end = ieq;
    h->integ_v = iv;
    h->integ_i = buck_toc_eq_duty(vref, ieq);
    h->cmp = buck_duty_to_cmp(h->integ_i);
}

/* ---- The executed frame (SPEC 65-72, 104; decision 4) -------------------------------------
 * The L-C prediction of the executed pair from the S0 state x0 with the episode's Ieq (and vs): mid
 * (the split) and the D state; the model's value of the frame's ADC2 measurement (the mean of the
 * model current at the four ADC2 instants of the frame words, model time = register tick - DT: exact
 * points on the arcs, buck_toc_frame_meas); the charge over the frame (Ieq h + C (v_D - v_0), exact
 * for the model). Then buck_toc_frame_correct(): i_D_rec = i_D,pred + k (Iavg_last - pred), k the
 * deadbeat gain of the last span (Q5's form on this model: a current error at the last span's start
 * turns by cos(omega t), so its mean over the span is sin(th)/th of it and its value at D cos(th):
 * k = th cot th, th = omega h_last; Padé: 2a/(1+a)). Parts (the firmware polls between them):
 * a1 the first span, a2 the second, b1 the ADC2 instants' rotations, b2 the model value, k and 1/h. */
typedef struct {
    buck_xy_t x0, mid, xd;
    float h1, h2, ieq, vs;
    uint32_t g1;               /* gate of the first span */
    buck_cs_t cs1, cs2;        /* the spans' rotations */
    buck_cs_t cs_t0, cs_dt;    /* the first ADC2 instant's rotation from S0, the instants' step */
    float pred;                /* the model's value of the ADC2 measurement, A */
    float k;                   /* deadbeat gain of the last span */
    float qsum;                /* charge over the frame, A s */
    float inv_h;               /* 1/(h1 + h2) */
    uint32_t stage;
} buck_toc_frame_t;

void buck_toc_frame_start(buck_toc_frame_t *fr, const buck_toc_plan_t *ep, buck_xy_t x0);
void buck_toc_frame_run_a1(buck_toc_frame_t *fr);
void buck_toc_frame_start_a1(buck_toc_frame_t *fr, const buck_toc_plan_t *ep, buck_xy_t x0);   /* start + a1 */
void buck_toc_frame_run_a2(buck_toc_frame_t *fr);
void buck_toc_frame_run_b1(buck_toc_frame_t *fr, const buck_toc_frame_words_t *w);
int32_t buck_toc_frame_run_b2(buck_toc_frame_t *fr, const buck_toc_frame_words_t *w);
int32_t buck_toc_frame_run(buck_toc_frame_t *fr, const buck_toc_frame_words_t *w);   /* all parts in one call */

/* i_D_rec = i_D,pred + k (Iavg_last - pred) (SPEC 104). */
BUCK_INLINE float buck_toc_frame_correct(const buck_toc_frame_t *fr, float iavg_last)
{
    return fr->xd.i + fr->k * (iavg_last - fr->pred);
}

/* The current of the observer's step at D (user, Q6): (q1 + q2 + h_last (Iavg_last - pred)) / h,
 * q1 + q2 the model's charge over the frame. */
BUCK_INLINE float buck_toc_frame_il(const buck_toc_frame_t *fr, float iavg_last)
{
    return (fr->qsum + fr->h2 * (iavg_last - fr->pred)) * fr->inv_h;
}

/* The observer's step at D (decision 5): buck_obs_step_a with a = h/C, h the
 * time since the last normal observer step, the same beta and gamma. il: the
 * average current over h (Q6: buck_toc_frame_il). */
BUCK_INLINE float buck_toc_obs_step_d(buck_obs_t *x, float v, float il, buck_obs_k_t k, float h)
{
    return buck_obs_step_a(x, v, il, k, h * BUCK_TOC_INV_C);
}

#endif
