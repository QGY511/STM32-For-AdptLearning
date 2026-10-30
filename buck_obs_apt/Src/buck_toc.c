/* TOC episode and learning, firmware side (stage 2, 29 Oct 2026; stage 2b, 30 Oct 2026; stage 3,
 * 30 Oct 2026: the L-C model everywhere, no estimator).
 * README "TOC and learning" describes the sequence; Inc/buck_toc_frame.h the
 * frame encoding; Inc/buck_toc_fw.h the interface to the control loop.
 *
 * Sequence (n = the request pass R, a normal HOLD pass whose score rose):
 *   R      the normal pass n ran as always (its CMP1 for period n+1 written, the PIs
 *          committed); run_loop calls buck_toc_event() (accepts the request) and
 *          buck_toc_episode() with copies of the controller state.
 *   plan   request: m_R, z, Ieq_plan, x0 from R's sample (stage 3, user decision 1c: no
 *          estimator; x0.i = the valley of the held duty from R's measured average with the
 *          L-C ripple model, buck_lc_valley; x0.v = R's v_out), then the L-C planner (four
 *          parts), the frame's L-C model and the frame words in a MUDIS|TAUDIS bracket.
 *          Planning periods n+1 .. n+N: CMP1 not written (its preload repeats the R value:
 *          the duty is held), PIs frozen, the observer held (S2-Q4), every sample read by
 *          the poll service between parts of at most ~150 clocks. Not ready when sample n+N
 *          is read, or a sample read late (counter check): LATE reject, no fault (stage 2b,
 *          user decision 3).
 *   arm    right after sample n+N: the frame CMP1 and the release through the counter
 *          guard; too late (counter check, or a missed guard): LATE reject.
 *   F      sample n+N+1 (EOC ~36 clocks into the frame): checks that the frame
 *          committed (MUPD, UPD) and the first span's output (O1CPY, SET1/RST1
 *          flags), then the return words (a normal period, CMP1 = resume) in a
 *          bracket, CMP1 and the release through the frame's guard; observer step.
 *   D      the frame's own sample (v_out pair before D, i_L in the last span):
 *          corrected i(D) (Q5), observer step over the frame (decision 5, Q6),
 *          handover (default E), CMP1 = the restart duty through the guard (ON-OFF:
 *          the normal window, A1 running; OFF-ON: before the frame's end), the
 *          return-transfer and grid checks (stage 3: no estimator reseed).
 *   learn  (learning on; stage 2b, user decision 2) the learning periods: the duty held
 *          (the restart CMP1 repeats), PIs frozen, the observer held (one step at B, the
 *          first sample after D; the catch-up at the end); kappa, G and, after B, q; at
 *          most BUCK_TOC_LEARN_PERIODS samples.
 *   then   normal passes. */

#include "stm32g4xx.h"
#include <stddef.h>
#include "buck.h"
#include "buck_learn.h"
#include "buck_toc_fw.h"

#define TOC_CODE  __attribute__((section(".ccmtext.tocfw"), noinline))
#define TOC_COLD  __attribute__((section(".ccmtext.tocfw"), noinline, cold))
#define TA        (HRTIM1->sTimerxRegs[0])
#define HM        (HRTIM1->sMasterRegs)
#define HC        (HRTIM1->sCommonRegs)
#define D4_PIN    (1U << 5)
#define CSR_BAD   (ADC_CSR_OVR_MST | ADC_CSR_OVR_SLV | ADC_CSR_EOC_SLV)
#define VSUM      ((float)BUCK_V_PER_VSUM)
#define UDIS      (HRTIM_CR1_MUDIS | HRTIM_CR1_TAUDIS)
#define NP        BUCK_TOC_PLAN_PERIODS
#define NL        BUCK_TOC_LEARN_PERIODS

buck_toc_mb_t g_toc_mb;

static buck_toc_plan_t toc_plan;        /* the episode (SPEC 170-172) */
static buck_toc_frame_t toc_fr;         /* the executed frame's prediction */
static buck_toc_planner_t toc_pw;       /* the planner's working values */
static uint32_t toc_bracket;            /* 1: frame words held in the preloads by MUDIS|TAUDIS (not armed) */
static buck_learn_q_t toc_q;
static buck_learn_g_t toc_g;
static struct {
    uint32_t run;                       /* g_toc_mb.run of the run this state belongs to */
} L;

/* What the learning periods need from D (stage 2b). */
typedef struct {
    float i_d, v_d, m_e, tail, on1;     /* corrected i(D), raw V(D), m_E, OFF-ON tail D->A, A1's on-time */
    uint32_t d_cmp;                     /* the restart CMP1 (held through the learning periods) */
} toc_dres_t;

/* The planning and learning periods' sample service (the poll hook) */
typedef struct {
    uint32_t nserv;                     /* samples read (since R / since D) */
    uint32_t late_at;                   /* nonzero: stop once this many samples are read */
    uint32_t exit;                      /* a STOP or a bad sample ended the run */
    uint32_t late;                      /* a sample was read late in its period (counter check) */
    uint32_t adc_v;                     /* ADC1 sum of the last sample read */
    uint32_t adc_i;                     /* ADC2 sum of the last sample read */
    uint32_t isum;                      /* ADC2 sums of the samples read, added */
} toc_ectx_t;

/* field by field (an aggregate initializer of this size becomes a memset call in flash) */
#define ECTX_INIT(e, at) do { (e).nserv = 0U; (e).late_at = (at); (e).exit = 0U; (e).late = 0U; (e).adc_v = 0U; \
                              (e).adc_i = 0U; (e).isum = 0U; } while (0)

/* ---- helpers ------------------------------------------------------------------------ */
/* Guarded store (the same instructions as DEADLINE_ASM in Src/buck.c, with the window in
 * registers): the Timer A counter is read and the store happens only if
 * early <= cnt < early + win; otherwise branch to missed. */
#define TOC_GUARD(val, addr, early, win, cnt, missed)                                      \
    do {                                                                                   \
        uint32_t late_;                                                                    \
        __asm__ goto("ldr    %[c], [%[hr], #%c[oc]]\n\t"                                   \
                     "sub    %[l], %[c], %[e]\n\t"                                         \
                     "cmp    %[l], %[w]\n\t"                                               \
                     "it     lo\n\t"                                                       \
                     "strlo  %[v], [%[a]]\n\t"                                             \
                     "bhs    %l[" #missed "]"                                              \
                     : [c] "=&r"(cnt), [l] "=&r"(late_)                                    \
                     : [hr] "r"(HRTIM1), [oc] "i"(offsetof(HRTIM_TypeDef, sTimerxRegs[0].CNTxR)), \
                       [e] "r"(early), [w] "r"(win), [v] "r"(val), [a] "r"(addr)             \
                     : "memory", "cc" : missed);                                           \
        (void)late_;                                                                       \
    } while (0)

static TOC_COLD void toc_outputs_off(buck_t *b)
{
    HC.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
    GPIOB->BSRR = D4_PIN << 16;
    DAC1->DHR12R1 = 0U;
    b->out.g = 0U;
}

static TOC_COLD void toc_trip(buck_t *b, uint32_t code)
{
    toc_outputs_off(b);
    b->fault = code;
    b->state = BUCK_FAULTED;
}

/* Leaving the run from the TOC code: as run_exit() (Src/buck.c) after the state is
 * written back. bad: fault 4; else a STOP (gates off, CMP1 = BUCK_CMP_MIN as run_exit). */
static TOC_CODE void toc_unbracket(void);

static TOC_COLD void toc_exit(buck_t *b, uint32_t bad, uint32_t adc_v, uint32_t adc_i4)
{
    buck_toc_mb_t *m = &g_toc_mb;
    toc_unbracket();                            /* a later RUN must find the normal preloads released */
    b->adc_v_raw = adc_v; b->adc_i4_raw = adc_i4; b->loops = m->loops;
    b->st = m->s;
    if (bad) {
        ADC1->ISR = ADC_ISR_OVR;
        ADC2->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
        ++b->adc_miss;
        toc_trip(b, BUCK_FAULT_ADC);
    } else {
        toc_outputs_off(b);
        TA.CMP1xR = BUCK_CMP_MIN;
        b->state = BUCK_STOPPED;
    }
    m->ret = BUCK_TOC_RET_EXIT;
    b->toc.state = BUCK_TOCS_IDLE;
}

/* A failed guard (fault 5, as in the loop) or hardware check (fault 9). */
static TOC_COLD void toc_fail(buck_t *b, uint32_t code, uint32_t cnt)
{
    buck_toc_mb_t *m = &g_toc_mb;
    const uint32_t cyc = DWT->CYCCNT;
    b->loops = m->loops;
    b->st = m->s;
    if (code == BUCK_FAULT_DEADLINE) { b->trip_cnt = cnt; b->trip_cyc = cyc; }
    toc_trip(b, code);
    m->ret = BUCK_TOC_RET_EXIT;
    b->toc.state = BUCK_TOCS_IDLE;
}

/* The arming came too late (stage 2b, user decision 3): a LATE reject, no fault. restore: the
 * frame CMP1' reached the preload before the release was missed (MUDIS|TAUDIS still set, so
 * nothing has transferred): the held duty goes back into it before the bracket is released. */
static TOC_COLD void toc_arm_late(buck_toc_t *T, uint32_t restore, uint32_t cmp_held)
{
    if (restore) TA.CMP1xR = cmp_held;
    ++T->late_arm;
    ++T->rej[BUCK_TOCR_LATE];
}

/* Waits for ADC1's end of conversion (and ADC2's if both): the loop's 840 not-ready
 * reads (~15 periods), then fault 4 for good as wait_sample() (Src/buck.c). */
static TOC_CODE uint32_t toc_wait(uint32_t both)
{
    const uint32_t need = both ? (ADC_CSR_EOC_MST | ADC_CSR_EOC_SLV) : ADC_CSR_EOC_MST;
    uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD, st;
    while (((st = ADC12_COMMON->CSR) & need) != need) {
        if (--n == 0U) buck_fatal(BUCK_FAULT_ADC);
    }
    return st;
}

/* (Timer A counter) - (master counter), read back to back: a constant while the grid is locked. */
static TOC_CODE int32_t toc_align(void)
{
    const uint32_t mc = HM.MCNTR;
    const uint32_t tc = TA.CNTxR;
    int32_t d = (int32_t)tc - (int32_t)mc;
    /* modulo the (normal) period: the two reads may straddle a roll-over (stage 2b: the reference is
     * now taken anywhere in a planning period) */
    if (d > (int32_t)(BUCK_HRTIM_PERIOD / 2U)) d -= (int32_t)BUCK_HRTIM_PERIOD;
    else if (d < -(int32_t)(BUCK_HRTIM_PERIOD / 2U)) d += (int32_t)BUCK_HRTIM_PERIOD;
    return d;
}

/* One planning/learning-period sample: read and checked; the observer and A2 are held (user Q1:
 * frozen "as appropriate"; stage2 S2-Q4) and the current is summed for the observer's catch-up step
 * (toc_obs_catchup). st: ADC12_CSR as the caller's poll read it (EOC_MST set). Returns 1 when the
 * episode must stop: the run ended (e->exit), the sample was read late in its period (stage 2b:
 * the counter outside [BUCK_TOC_SVC_EARLY, BUCK_TOC_SVC_LATE), e->late), or the late_at-th sample
 * was read. Inline in the episode's and the learning periods' polls (stage 2b: ~25 clocks instead of
 * ~70 through the planner's hook); toc_service() is the hook form. */
static inline __attribute__((always_inline)) uint32_t toc_svc(toc_ectx_t *e, uint32_t st)
{
    buck_t *b = &g_buck;
    /* ADC1, ADC2 and their common registers from one base register (no literal per access) */
    uintptr_t ab = ADC1_BASE;
    __asm__("" : "+r"(ab));
    const uint32_t bad = (st & CSR_BAD) ^ ADC_CSR_EOC_SLV;
    const uint32_t adc_v = ((ADC_TypeDef *)ab)->DR;
    const uint32_t adc_i4 = ((ADC_TypeDef *)(ab + (ADC2_BASE - ADC1_BASE)))->DR;
    const uint32_t cnt = TA.CNTxR;
    const uint32_t cmd = *(volatile const uint32_t *)&b->command;
    if (__builtin_expect((bad | (cmd ^ BUCK_RUN)) != 0U, 0)) {
        toc_exit(b, bad, adc_v, adc_i4);
        e->exit = 1U;
        return 1U;
    }
    e->adc_v = adc_v;
    e->adc_i = adc_i4;
    e->isum += adc_i4;
    const uint32_t n = e->nserv + 1U;
    e->nserv = n;
    if (__builtin_expect(cnt - BUCK_TOC_SVC_EARLY >= BUCK_TOC_SVC_LATE - BUCK_TOC_SVC_EARLY, 0)) {
        e->late = 1U; ++b->toc.late_svc; return 1U;
    }
    return (e->late_at != 0U && n >= e->late_at) ? 1U : 0U;
}

static TOC_CODE uint32_t toc_service(void *ctx)
{
    const uint32_t st = ADC12_COMMON->CSR;
    if (!(st & ADC_CSR_EOC_MST)) return 0U;
    return toc_svc((toc_ectx_t *)ctx, st);
}

/* A poll point: the sample service if an end of conversion is pending. */
#define TOC_POLL(e, stopped) do { const uint32_t st_ = ADC12_COMMON->CSR;                       \
                                  if ((st_ & ADC_CSR_EOC_MST) && toc_svc(&(e), st_)) stopped; } while (0)

/* The observer over the periods whose samples were only read (decision 5's form: one step
 * with a = h/C): h = nper T, il = their mean measured current, v = the last v_out. */
static TOC_CODE void toc_obs_catchup(uint32_t isum, uint32_t nper, uint32_t adc_v)
{
    buck_toc_mb_t *m = &g_toc_mb;
    const float il = ((float)isum * (0.25f / (float)nper) - m->t.isense_zero) * m->t.isense_gain;
    const float v = (float)adc_v * VSUM;
    DAC1->DHR12R1 = buck_iload_dac(buck_obs_step_a(&m->obs, v, il, m->ok, (float)nper * BUCK_OBS_A));
}

/* Back to the normal passes between a sample and the next (the reject path and the end of the
 * learning periods): right after the next sample (read here: it comes ~36 clocks into its period),
 * so that the state for the next normal pass (<= ~150 clocks) and the return (~60) end before
 * that pass's sample at all clock scales up to 1.2 (stage 2b; stage 2 also returned at once when
 * the counter allowed, which made the first normal pass late at 1.1). Returns 1 if the run ended
 * (STOP or a bad sample). */
static TOC_CODE uint32_t toc_to_normal(toc_ectx_t *e)
{
    e->late_at = 0U;
    if (ADC12_COMMON->CSR & ADC_CSR_EOC_MST) { (void)toc_service(e); if (e->exit) return 1U; }
    (void)toc_wait(0U);
    (void)toc_service(e);
    return e->exit;
}

/* ---- learning periods (stage 2b, user decision 2) ------------------------------------------- */
/* B, the first sample after D (both orders): V_B, and the observer's step for it as the normal B
 * pass made it (a = T/C, its measured current), for m_B (q's raw-load gate). */
static TOC_CODE void toc_learn_b(buck_toc_t *T, const toc_ectx_t *e)
{
    buck_toc_mb_t *m = &g_toc_mb;
    const float v = (float)e->adc_v * VSUM;
    const float il = buck_adc_to_i_o(&m->t, e->adc_i, 0);
    const float mb = buck_obs_step_a(&m->obs, v, il, m->ok, BUCK_OBS_A);
    DAC1->DHR12R1 = buck_iload_dac(mb);
    T->v_b = v; T->m_b = mb;
}

/* The learning periods. Jobs in this order, a poll between their stages: kappa (at E = D, SPEC
 * 80-84), G's Jacobian (cached at S0 with the commanded pair, SPEC 89-98; started at F), then,
 * once B's sample is read, G's teach (SPEC 238-251: the gate has returned) and q (SPEC 220-236,
 * S = B for both orders: BUCK_LEARN_QPEAK_AT_B). They stop after BUCK_TOC_LEARN_PERIODS samples,
 * or when a sample is read late; a job not done then is dropped (learn_late). Then the state
 * for the next normal pass: the observer's catch-up over the samples after B (stage 3: no
 * estimator to carry). */
static TOC_CODE void toc_learn(buck_toc_t *T, const buck_toc_plan_t *P, const toc_dres_t *dr, uint32_t jobs)
{
    toc_ectx_t e;
    ECTX_INIT(e, NL);
    const uint32_t want = jobs ? (3U | ((BUCK_LEARN_QPEAK_AT_B || P->order == BUCK_TOC_ON_OFF) ? 4U : 0U)) : 0U;
    uint32_t done = 0U, seen = 0U, isum_b = 0U;
    T->state = BUCK_TOCS_LEARN;
    /* after the first new sample (B): its observer step (stage 3: nothing else is tracked) */
#define LP_TRACK() do { if (e.nserv != seen) {                                                    \
                            if (seen == 0U) { isum_b = e.isum; toc_learn_b(T, &e); }               \
                            seen = e.nserv; } } while (0)
#define LP_POLL() do { TOC_POLL(e, goto end); LP_TRACK(); } while (0)
    LP_POLL();
    if (!want) {                                /* no jobs (a late D pass, stage 2b): B, then back */
        uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD;
        while (seen == 0U) {
            LP_POLL();
            if (--n == 0U) buck_fatal(BUCK_FAULT_ADC);
        }
        goto end;
    }
    {                                           /* kappa */
        float kn;
        if (buck_learn_kappa(P->kappa_used, P->m_r, P->z, dr->m_e, &kn)) { T->kappa = kn; ++T->kappa_n; }
        else ++T->kappa_rej;
        done |= 1U;
    }
    LP_POLL();
    while (toc_g.stage == BUCK_LEARN_G_J && toc_g.status == BUCK_LEARN_BUSY) {   /* G's Jacobian (one part) */
        (void)buck_learn_g_step(&toc_g);
        LP_POLL();
    }
    {                                           /* B */
        uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD;
        while (seen == 0U) {
            LP_POLL();
            if (--n == 0U) buck_fatal(BUCK_FAULT_ADC);
        }
    }
    buck_learn_g_teach_start(&toc_g, dr->i_d, dr->v_d);   /* no valid Jacobian: REJECT */
    for (;;) {
        const int32_t st = buck_learn_g_step(&toc_g);
        if (st == BUCK_LEARN_OK) {
            T->g[toc_g.order][0] = toc_g.g_new.i; T->g[toc_g.order][1] = toc_g.g_new.v;
            ++T->g_n;
            break;
        }
        if (st != BUCK_LEARN_BUSY) { ++T->g_rej; break; }
        LP_POLL();
    }
    done |= 2U;
    LP_POLL();
    if (want & 4U) {
        const float dm = T->m_b - dr->m_e;
        const uint32_t raw_ok = (dm <= (float)BUCK_LEARN_RAW_LIMIT_A && dm >= -(float)BUCK_LEARN_RAW_LIMIT_A);
        buck_learn_q_start(&toc_q, P, dr->i_d, dr->v_d, dr->tail, dr->on1, (float)BUCK_TSW_S, T->v_b, raw_ok);
        for (;;) {
            LP_POLL();
            const int32_t st = buck_learn_q_step(&toc_q);
            if (st == BUCK_LEARN_OK) { T->q[toc_q.order] = toc_q.q_new; ++T->q_n; break; }
            if (st != BUCK_LEARN_BUSY) { ++T->q_rej; break; }
        }
        done |= 4U;
    }
end:
#undef LP_POLL
    if (e.exit) return;
    {
        const uint32_t left = want & ~done;
        T->learn_late += (left & 1U) + ((left >> 1) & 1U) + ((left >> 2) & 1U);
        T->learn_done = done;
    }
    if (toc_to_normal(&e)) return;
    LP_TRACK();
#undef LP_TRACK
    T->learn_samples = e.nserv;
    if (e.nserv >= 2U) toc_obs_catchup(e.isum - isum_b, e.nserv - 1U, e.adc_v);
    T->state = BUCK_TOCS_IDLE;
}

static void toc_episode(void);

/* ---- entry points ------------------------------------------------------------------------- */
/* A new run (run_loop entry). */
static TOC_CODE void toc_new_run(void)
{
    L.run = g_toc_mb.run;
    g_toc_mb.forced = 0U; g_toc_mb.cap_b = 0U;
    g_buck.toc.an_valid = 0U;
    g_buck.toc.state = BUCK_TOCS_IDLE;
}

/* After a normal HOLD pass whose g rose (run_loop's trigger word; no forced passes since stage 2b). */
TOC_CODE void buck_toc_event(void)
{
    buck_toc_mb_t *m = &g_toc_mb;
    buck_t *b = &g_buck;
    buck_toc_t *T = &b->toc;
    m->req = 0U;
    if (L.run != m->run) toc_new_run();
    const uint32_t g = (m->d4 >> 5) & 1U;
    m->g_last = g;
    ++T->edges;
    if (T->toc_enable) {
        if (m->loops - m->loops0 < BUCK_TOC_QUIET_PASSES) ++T->rej[BUCK_TOCR_EARLY];
        else if ((ADC12_COMMON->CSR & ADC_CSR_EOC_MST) && TA.CNTxR >= BUCK_TOC_R_LATE_TICKS)
            ++T->rej[BUCK_TOCR_LATE];           /* R itself late: the next sample waits too long already */
        else { m->req = 1U; toc_episode(); return; }
    }
    m->forced = 0U;
    m->trig = (g ? 0U : D4_PIN) | (m->trig & BUCK_TOC_LEARN_BIT);
}

/* The frame's end: return words for a normal period (RM0440: written after the frame has
 * committed, transferred at its end). */
static TOC_CODE void toc_return_words(void)
{
    TA.PERxR = BUCK_HRTIM_PERIOD;
    TA.CMP2xR = BUCK_TOC_CMP2_IDLE;
    TA.CMP3xR = BUCK_VSAMPLE_CMP;
    TA.CMP4xR = BUCK_VSAMPLE0_CMP;
    TA.SETx1R = HRTIM_SET1R_PER;
    TA.RSTx1R = HRTIM_RST1R_CMP1;
    HM.MPER = BUCK_HRTIM_PERIOD;
    HM.MCMP1R = BUCK_ISAMPLE_CMP(0U);
    HM.MCMP2R = BUCK_ISAMPLE_CMP(1U);
    HM.MCMP3R = BUCK_ISAMPLE_CMP(2U);
    HM.MCMP4R = BUCK_ISAMPLE_CMP(3U);
}

/* The frame words but CMP1, bracketed (toc_bracket = 1 until the release). */
static TOC_CODE void toc_frame_words(const buck_toc_frame_words_t *w)
{
    HC.CR1 = UDIS;
    toc_bracket = 1U;
    TA.PERxR = w->per;
    TA.CMP2xR = w->cmp2;
    TA.CMP3xR = w->cmp3;
    TA.CMP4xR = w->cmp4;
    TA.SETx1R = w->set_cmp2 ? HRTIM_SET1R_CMP2 : HRTIM_SET1R_PER;
    TA.RSTx1R = HRTIM_RST1R_CMP1;
    HM.MPER = w->per;
    HM.MCMP1R = w->mcmp[0];
    HM.MCMP2R = w->mcmp[1];
    HM.MCMP3R = w->mcmp[2];
    HM.MCMP4R = w->mcmp[3];
}

/* No frame after all: the normal words back into the preloads (the active registers are still
 * those of a normal period, CMP1 the held duty), then the release. */
static TOC_CODE void toc_unbracket(void)
{
    if (!toc_bracket) return;
    toc_return_words();
    HC.CR1 = 0U;
    toc_bracket = 0U;
}

static TOC_CODE void toc_episode(void)
{
    buck_toc_mb_t *m = &g_toc_mb;
    buck_t *b = &g_buck;
    buck_toc_t *T = &b->toc;
    buck_toc_plan_t *P = &toc_plan;
    const uint32_t learn = T->learn_enable;
    m->ret = BUCK_TOC_RET_STATE;
    m->trig = learn ? BUCK_TOC_LEARN_BIT : 0U;  /* after the episode: a new request needs g to fall first */
    m->forced = 0U; m->cap_b = 0U; m->g_last = 1U;
    ++T->requests;
    T->state = BUCK_TOCS_PLAN;
    T->plan_ready = 0U;
    toc_ectx_t e;
    ECTX_INIT(e, NP);
    /* sample n+1 may already be waiting (the request pass ends late in its period) */
#define EP_POLL() TOC_POLL(e, goto stop)
    EP_POLL();

    /* ---- request (SPEC 152-175) */
    const uint32_t cmp_r = m->s.cmp_active;     /* CMP1 written by R: held through the planning periods */
    buck_toc_req_t rq;
    rq.m_r = m->ih;
    rq.raw_before = (T->an_valid && T->an_loops + 1U == m->loops) ? T->an_m : rq.m_r;   /* z = 0 without a pair */
    T->an_valid = 0U;                           /* normal_anchor = INVALID */
    rq.delta_i = T->delta_i;
    rq.kappa = T->kappa;
    rq.q[0] = T->q[0]; rq.q[1] = T->q[1];
    rq.g[0].i = T->g[0][0]; rq.g[0].v = T->g[0][1];
    rq.g[1].i = T->g[1][0]; rq.g[1].v = T->g[1][1];
    rq.vref = m->s.vref;
    const float ieq = (rq.m_r + rq.delta_i) + rq.kappa * (rq.m_r - rq.raw_before);   /* = toc_ieq */
    /* x0 (stage 3, user decision 1c: no estimator): the valley of a period of the held duty cmp_r
     * from R's measured average with the L-C ripple model (buck_lc_valley, Inc/buck_toc_plan.h),
     * v = R's v_out sample. */
    rq.i_r = buck_lc_valley(m->im, m->v, cmp_r, buck_lc_vs(ieq));
    rq.v_r = m->v;
    rq.ton_n = (float)(cmp_r - BUCK_DT_TICKS) * BUCK_TOC_TICK;
    rq.resume_cmp = buck_toc_resume_cmp(rq.vref, ieq);     /* Q4 */

    /* ---- plan, reading the samples of the planning periods */
    EP_POLL();
    {                                           /* the L-C planner in its six parts (stage 3) */
        int32_t st = buck_toc_fast_1(&toc_pw, &rq, P);
        if (st == BUCK_TOC_BUSY) { EP_POLL(); st = buck_toc_fast_2(&toc_pw, &rq, P); }
        if (st == BUCK_TOC_BUSY) { EP_POLL(); st = buck_toc_fast_3(&toc_pw, P); }
        if (st == BUCK_TOC_BUSY) { EP_POLL(); st = buck_toc_fast_4(&toc_pw, P); }
        if (st == BUCK_TOC_BUSY) { EP_POLL(); st = buck_toc_fast_5(&toc_pw, &rq, P); }
        if (st == BUCK_TOC_BUSY) { EP_POLL(); (void)buck_toc_fast_6(&rq, P); }
    }
    T->plan_status = P->status;
    T->plan_samples = e.nserv;
    T->order = P->order; T->on_ticks = P->on_ticks; T->off_ticks = P->off_ticks; T->per_ticks = P->per_ticks;
    T->resume_cmp = rq.resume_cmp; T->x0_i = P->x0.i; T->x0_v = P->x0.v; T->ieq = P->ieq; T->target_i = P->target_i;
    if (P->status == BUCK_TOC_ABORT) goto stop;
    if (P->status != BUCK_TOC_READY) {
        const uint32_t why = (P->status == BUCK_TOC_NONE) ? BUCK_TOCR_NONE
                           : (P->status == BUCK_TOC_LIMIT) ? BUCK_TOCR_LIMIT : (P->status == BUCK_TOC_ALREADY) ? BUCK_TOCR_ALREADY
                           : BUCK_TOCR_INVALID;
        ++T->rej[why];
        goto reject;
    }
    /* the executed frame's L-C model (Q5, Q6 inputs): the two spans, the model value of the ADC2
     * measurement at its four instants, the deadbeat gain and 1/h (stage 2b: all before the arming
     * sample, a poll between the parts; nothing but the arming is left for the arming period) */
    const buck_toc_frame_words_t *w = &P->fw;
    EP_POLL();
    buck_toc_frame_start_a1(&toc_fr, P, P->x0);
    EP_POLL();
    buck_toc_frame_run_a2(&toc_fr);
    EP_POLL();
    buck_toc_frame_run_b1(&toc_fr, w);
    EP_POLL();
    (void)buck_toc_frame_run_b2(&toc_fr, w);
    const float tail = (float)(BUCK_HRTIM_PERIOD - (P->resume_cmp - BUCK_DT_TICKS)) * BUCK_TOC_TICK;
    /* the grid reference and the frame words (all but CMP1) into the preloads, held by MUDIS|TAUDIS
     * until the arming's release (no update transfer meanwhile, which changes nothing: the duty is
     * held and the other active registers are those of a normal period) */
    EP_POLL();
    T->align_ref = toc_align();
    toc_frame_words(w);
    T->plan_ready = e.nserv;
    T->state = BUCK_TOCS_WAIT;
    /* wait for the arming sample n+NP */
    e.late_at = 0U;
    {
        uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD;
        while (e.nserv < NP) {
            const uint32_t st = ADC12_COMMON->CSR;
            if (st & ADC_CSR_EOC_MST) {
                if (toc_svc(&e, st)) goto stop;
                n = 3U * BUCK_CYCLES_PER_PERIOD;
            } else if (--n == 0U) {
                buck_fatal(BUCK_FAULT_ADC);
            }
        }
    }

    /* ---- arm (sample n+NP just read): the frame CMP1 and the release of the bracket that holds
     * the other frame words. Not if the next sample is already there (the pass is a period late: the
     * frame would start a period after x0), nor if the counter is past BUCK_TOC_ARM_LAST (stage 2b:
     * LATE reject instead of a missed guard). Nothing runs between the arming and the frame's start
     * (stage 2b), so pass F starts at once. */
    uint32_t cnt;
    if (ADC12_COMMON->CSR & ADC_CSR_EOC_MST) goto late_arm0;
    cnt = TA.CNTxR;
    if (cnt < BUCK_TOC_F_EARLIEST || cnt >= BUCK_TOC_ARM_LAST) goto late_arm0;
    HM.MICR = HRTIM_MICR_MUPD;
    TA.TIMxICR = HRTIM_TIMICR_UPDC | HRTIM_TIMICR_SET1C | HRTIM_TIMICR_RST1C;
    {
        /* the window starts at BUCK_TOC_F_EARLIEST: this pass reads the counter soon after its end of
         * conversion (1152 ticks into the period) */
        const uint32_t early = BUCK_TOC_F_EARLIEST, win = BUCK_WRITE_DEADLINE - BUCK_TOC_F_EARLIEST, zero = 0U;
        TOC_GUARD(w->cmp1, &TA.CMP1xR, early, win, cnt, late_arm0);
        TOC_GUARD(zero, &HC.CR1, early, win, cnt, late_arm1);
    }
    toc_bracket = 0U;
    T->arm_cnt = cnt;
    T->state = BUCK_TOCS_FRAME;

    /* ---- the frame's start: wait for its transfer (MUPD), then at once the return words for
     * the period after it (hw_map 3.5b: they must be written after the frame has committed and
     * before its end - the shortest frames leave no time to wait for the next sample first) */
    uint32_t stop_pending, isr0, c0, commit;
    {
        const uint32_t t0 = DWT->CYCCNT;
        while (!((commit = HM.MISR) & HRTIM_MISR_MUPD) && (uint32_t)(DWT->CYCCNT - t0) < 2U * BUCK_CYCLES_PER_PERIOD) {}
        isr0 = TA.TIMxISR;                      /* flags first, then the counter (stage 2b): if the counter */
        c0 = TA.CNTxR;                          /* is before the split, the flags are too */
        HC.CR1 = UDIS;
        toc_return_words();
        HM.MICR = HRTIM_MICR_MUPD;
        TA.TIMxICR = HRTIM_TIMICR_UPDC;
        const uint32_t early = 0U, win = w->per - BUCK_TOC_RET_GUARD_TICKS, zero = 0U, resume = P->resume_cmp;
        TOC_GUARD(resume, &TA.CMP1xR, early, win, cnt, late_ret);
        TOC_GUARD(zero, &HC.CR1, early, win, cnt, late_ret);
        T->f_cnt = cnt;
    }
    /* ---- F: the last normal period's sample, ~36 clocks into the frame */
    {
        const uint32_t st = toc_wait(0U);
        const uint32_t bad = (st & CSR_BAD) ^ ADC_CSR_EOC_SLV;
        const uint32_t adc_v = ADC1->DR, adc_i4 = ADC2->DR;
        stop_pending = *(volatile const uint32_t *)&b->command ^ BUCK_RUN;
        if (bad) { toc_exit(b, bad, adc_v, adc_i4); return; }
        /* the frame committed at its start (RM0440 item 5: MUPD/UPD set only by a real transfer) */
        if (!(commit & HRTIM_MISR_MUPD) || !(isr0 & HRTIM_TIMISR_UPD)) {
            ++T->chk_commit;
            if (BUCK_TOC_CHECK_TRIP) { toc_fail(b, BUCK_FAULT_TOC, 0U); return; }
        }
        /* the first span's output (stage2 S2-Q2: SET1R taken at the boundary). c0 is read just
         * after TIMxISR, so the flags are not newer than the counter (stage 2b: the MUPD wait may now
         * start late in the first span, after the ADC2 model value). */
        const uint32_t o1 = (isr0 & HRTIM_TIMISR_O1CPY) != 0U, set1 = (isr0 & HRTIM_TIMISR_SET1) != 0U;
        uint32_t out_bad = 0U;
        if (c0 + 64U < w->split_ref)
            out_bad = w->set_cmp2 ? (o1 | set1)                /* OFF-ON: low, no set before CMP2' */
                                  : ((o1 ^ 1U) | (set1 ^ 1U)); /* ON-OFF: set at the start, high */
        if (out_bad) {
            ++T->chk_out;
            if (BUCK_TOC_CHECK_TRIP) { toc_fail(b, BUCK_FAULT_TOC, 0U); return; }
        }
        /* the observer over the planning periods and the last normal one (its sample is this one);
         * V(S0) for G (SPEC 91) */
        toc_obs_catchup(e.isum + adc_i4, e.nserv + 1U, adc_v);
        const float v = (float)adc_v * VSUM;
        T->v_s0 = v;
        if (learn) buck_learn_g_start(&toc_g, P, v);
    }
    T->state = BUCK_TOCS_D;

    /* ---- D: the frame's sample (v_out before D, i_L in the last span) */
    uint32_t d_adc_v, d_adc_i;
    toc_dres_t dr;
    {
        const uint32_t st = toc_wait(1U);
        const uint32_t bad = st & (ADC_CSR_OVR_MST | ADC_CSR_OVR_SLV);
        const uint32_t adc_v = ADC1->DR, adc_i4 = ADC2->DR;
        d_adc_v = adc_v; d_adc_i = adc_i4;
        stop_pending |= *(volatile const uint32_t *)&b->command ^ BUCK_RUN;
        if (bad) { toc_exit(b, bad, adc_v, adc_i4); return; }
        const float v_d = (float)adc_v * VSUM;
        const float ia = buck_adc_to_i_o(&m->t, adc_i4, 0);
        const float i_rec = buck_toc_frame_correct(&toc_fr, ia);                        /* Q5 */
        const float il = buck_toc_frame_il(&toc_fr, ia);                                /* Q6 */
        buck_toc_obs_step_d(&m->obs, v_d, il, m->ok, toc_fr.h1 + toc_fr.h2);           /* decision 5 */
        const float m_e = m->obs.ih;
        buck_toc_handover_t ho;
        buck_learn_handover(m_e, T->delta_i, m->s.vref, &ho);                          /* default E */
        /* the restart CMP1 (Ieq_end) for A2: ON-OFF now (A1 is running, ~1300 ticks in; A1's own sample
         * must not be there yet, else a period late); OFF-ON after the frame's end below (stage2 S2-Q7:
         * the frame's tail may be too short for the D pass to write before it ends). A1 runs with the
         * resume CMP1 of the return words (Ieq_plan, Q4) in both orders. */
        if (!w->set_cmp2) {
            if (ADC12_COMMON->CSR & ADC_CSR_EOC_MST) { cnt = TA.CNTxR; goto late_d; }
            const uint32_t early = BUCK_TOC_F_EARLIEST, win = BUCK_WRITE_DEADLINE - BUCK_TOC_F_EARLIEST;
            TOC_GUARD(ho.cmp, &TA.CMP1xR, early, win, cnt, late_d);
            T->d_cnt = cnt;
        }
        dr.d_cmp = ho.cmp;
        DAC1->DHR12R1 = buck_iload_dac(m_e);
        const uint32_t cmp_a1 = P->resume_cmp;
        m->s.cmp_active = ho.cmp;
        m->s.integ_v = ho.integ_v;
        m->s.integ_i = ho.integ_i;
        /* what the learning periods need */
        dr.i_d = i_rec; dr.v_d = v_d; dr.m_e = m_e; dr.tail = tail;
        dr.on1 = (float)(cmp_a1 - BUCK_DT_TICKS) * BUCK_TOC_TICK;
        if (learn && w->set_cmp2 == 0U) {       /* ON-OFF: A = D, the DeltaI anchor (SPEC 204) */
            T->an_v = v_d; T->an_m = m_e; T->an_ic = m_e + T->delta_i; T->an_loops = m->loops; T->an_valid = 1U;
        }
        T->i_d = i_rec; T->v_d = v_d; T->i_meas_d = ia; T->i_pred_d = toc_fr.pred; T->m_e = m_e; T->end_cmp = ho.cmp;
    }
    /* ---- the return transfer and the grid */
    {
        if (w->set_cmp2) {                      /* OFF-ON: wait for the frame's end */
            const uint32_t c1 = TA.CNTxR;
            const uint32_t t0 = DWT->CYCCNT;
            const uint32_t lim = (c1 < w->per ? (w->per - c1) / 32U : 0U) + 64U;
            while (!(HM.MISR & HRTIM_MISR_MUPD) && (uint32_t)(DWT->CYCCNT - t0) < lim) {}
            /* A1 has just started (MUPD): the restart CMP1 for A2, guarded with the normal deadline */
            if (ADC12_COMMON->CSR & ADC_CSR_EOC_MST) { cnt = TA.CNTxR; goto late_d; }
            const uint32_t early = 0U, win = BUCK_WRITE_DEADLINE;
            TOC_GUARD(dr.d_cmp, &TA.CMP1xR, early, win, cnt, late_d);
            T->d_cnt = cnt;
        }
        const uint32_t misr = HM.MISR;
        const uint32_t isr = TA.TIMxISR;
        if (!(misr & HRTIM_MISR_MUPD) || !(isr & HRTIM_TIMISR_UPD)) {
            ++T->chk_commit;
            if (BUCK_TOC_CHECK_TRIP) { toc_fail(b, BUCK_FAULT_TOC, 0U); return; }
        }
        T->align_last = toc_align();
        const int32_t da = T->align_last - T->align_ref;
        if (da > BUCK_TOC_ALIGN_TOL || da < -BUCK_TOC_ALIGN_TOL) {
            ++T->chk_align;
            if (BUCK_TOC_CHECK_TRIP) { toc_fail(b, BUCK_FAULT_TOC, 0U); return; }
        }
    }
    ++T->frames;
    T->state = BUCK_TOCS_IDLE;
    if (stop_pending) { toc_exit(b, 0U, d_adc_v, d_adc_i); return; }   /* default I: after the frame */
    /* DeltaI pauses until the next telemetry pass sets the learning bit again (<= 64 passes): the first
     * passes after D start late (the D pass is long) and catch up only with the ~130 spare clocks of a
     * pass without DeltaI (README "Clock budget"; stage2 questions S2-Q11) */
    m->di_hold = m->loops + BUCK_TOC_DI_HOLD;
    if (learn) toc_learn(T, P, &dr, 1U);        /* the learning periods (stage 2b) */
    else if ((ADC12_COMMON->CSR & ADC_CSR_EOC_MST) && TA.CNTxR >= BUCK_TOC_D_HOLD_TICKS) {
        /* the D pass ended so late that B's sample is waiting and the first normal pass could not
         * write its CMP1 in time (stage 2b, seen at 1.05 x the clocks): B read here and the duty held
         * one more period (as the learning periods without jobs), then back */
        ++T->d_hold;
        toc_learn(T, P, &dr, 0U);
    }
    return;

stop:                                           /* a STOP / bad sample (exit), or late */
    if (e.exit) return;
    ++T->rej[BUCK_TOCR_LATE];
    goto reject;
late_arm1:
    toc_arm_late(T, 1U, cmp_r);
    goto reject;
late_arm0:
    toc_arm_late(T, 0U, cmp_r);
reject:
    toc_unbracket();
    /* no frame. Back to the normal passes between a sample and the next, with the time to form
     * their state and return before the next end of conversion: the next pass then waits for its
     * sample and writes CMP1 in time. The observer over the periods read since R (stage 3: no
     * estimator to carry). */
    if (toc_to_normal(&e)) return;
    if (e.nserv != 0U) toc_obs_catchup(e.isum, e.nserv, e.adc_v);
    T->state = BUCK_TOCS_IDLE;
    return;
late_ret:
    toc_fail(b, BUCK_FAULT_DEADLINE, cnt);
    return;
late_d:
    toc_fail(b, BUCK_FAULT_DEADLINE, cnt);
    return;
#undef EP_POLL
}
