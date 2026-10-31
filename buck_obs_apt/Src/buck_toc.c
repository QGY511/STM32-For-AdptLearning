/* Feed-forward TOC, firmware side after the start (stage 4, 30 Oct 2026). Inc/buck_toc_fw.h describes
 * the sequence and the interface to the control loop, Inc/buck_toc_start.h the start (inline in the
 * pass: the three software updates), Inc/buck_toc.h the math, README "Feed-forward TOC" the HRTIM
 * mechanism and the bench tests it needs. */

#include "stm32g4xx.h"
#include <stddef.h>
#include "buck.h"
#include "buck_toc.h"
#include "buck_toc_fw.h"

#define TOC_CODE  __attribute__((section(".ccmtext.tocfw"), noinline))
#define TOC_COLD  __attribute__((section(".ccmtext.tocfw"), noinline, cold))
#define TA        (HRTIM1->sTimerxRegs[0])
#define HM        (HRTIM1->sMasterRegs)
#define HC        (HRTIM1->sCommonRegs)
#define D4_PIN    (1U << 5)
#define CSR_BAD   (ADC_CSR_OVR_MST | ADC_CSR_OVR_SLV | ADC_CSR_EOC_SLV)
#define VSUM      ((float)BUCK_V_PER_VSUM)

buck_toc_mb_t g_toc_mb;

/* ---- helpers ------------------------------------------------------------------------------- */
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

/* Leaving the run from the TOC code: as run_exit() (Src/buck.c) after the state is written back.
 * bad: fault 4 (site: g_buck.toc.f4_site); else a STOP (gates off, CMP1 = BUCK_CMP_MIN as run_exit). */
static TOC_COLD void toc_exit(buck_t *b, uint32_t bad, uint32_t adc_v, uint32_t adc_i4, uint32_t site)
{
    buck_toc_mb_t *m = &g_toc_mb;
    b->adc_v_raw = adc_v; b->adc_i4_raw = adc_i4; b->loops = m->loops;
    b->st = m->s;
    if (bad) {
        b->toc.f4_site = site; b->toc.f4_bad = bad;              /* diagnostics (31 Oct) */
        b->toc.f4_csr = ADC12_COMMON->CSR; b->toc.f4_cnt = TA.CNTxR;
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

/* A missed guard (fault 5 for a CMP1 write, as in the loop; fault 9 for a frame write) or a failed
 * hardware check (fault 9). The gates go off at once. */
static TOC_COLD void toc_fail(buck_t *b, uint32_t code, uint32_t cnt)
{
    buck_toc_mb_t *m = &g_toc_mb;
    const uint32_t cyc = DWT->CYCCNT;
    toc_trip(b, code);
    b->loops = m->loops;
    b->st = m->s;
    if (code == BUCK_FAULT_DEADLINE) { b->trip_cnt = cnt; b->trip_cyc = cyc; }
    m->ret = BUCK_TOC_RET_EXIT;
    b->toc.state = BUCK_TOCS_IDLE;
}

/* ---- the event log of the last frame (31 Oct diagnostics: g_buck.toc.ev, README "Enabling" 8): the waits after
 * the frame and what became of their samples, in CPU cycles from the return words' guard on. Integer only
 * (buck_toc_ret logs: BUCK_TOC_CALL_CORE) */
_Static_assert(sizeof(g_buck.toc.ev) / sizeof(g_buck.toc.ev[0]) == BUCK_TOC_EVENTS, "event log size");
#define EV_TLOST  0x008U    /* T's sample overwritten (ADC1 overrun): no learning from this frame */
#define EV_TSKIP  0x010U    /* T's CMP1 write outside its window: skipped */
#define EV_DROP   0x080U    /* a pass's sample too late for its CMP1 write, or incomplete: dropped */
#define EV_START  0x100U    /* a wait began */

/* ADC12_COMMON->CSR as the log's state: ADC1's result 0x01, ADC2's 0x02, ADC1 overrun 0x20, ADC2 overrun 0x40 */
static inline uint32_t toc_ev_state(uint32_t st)
{
    return ((st >> 2) & 0x01U) | ((st >> 17) & 0x02U) | ((st << 1) & 0x20U) | ((st >> 14) & 0x40U);
}

static TOC_CODE void toc_ev(uint32_t code)
{
    buck_toc_t *const T = &g_buck.toc;
    const uint32_t k = T->ev_n;
    if (k < BUCK_TOC_EVENTS) {
        T->ev[k] = (code << 23) | (((DWT->CYCCNT - T->ev_cyc0) & 0xFFFU) << 11) | ((TA.CNTxR >> 5) & 0x7FFU);
        T->ev_n = k + 1U;
    }
}

/* Waits for ADC1's end of conversion: the loop's 840 not-ready reads, then fault 4 for good as
 * wait_sample() (Src/buck.c). A frame is at most ~11 us (~1900 clocks): within the timeout. *cnt: the Timer A
 * counter after it (the probe's pr_c1). */
static TOC_CODE uint32_t toc_wait(uint32_t *cnt)
{
    buck_toc_t *const T = &g_buck.toc;
    const uint32_t c0 = TA.CNTxR;               /* diagnostics (31 Oct): the samples' arrival, g_buck.toc.pr_* */
    uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD, st = ADC12_COMMON->CSR;
    uint32_t c2 = (st & ADC_CSR_EOC_SLV) ? 0xFFFFU : 0U;
    toc_ev(EV_START | toc_ev_state(st));
    while ((st & ADC_CSR_EOC_MST) == 0U) {
        if (c2 == 0U && (st & ADC_CSR_EOC_SLV)) c2 = TA.CNTxR;
        if (--n == 0U) {
            T->f4_site = 4U; T->f4_csr = st; T->f4_cnt = TA.CNTxR;
            buck_fatal(BUCK_FAULT_ADC);
        }
        st = ADC12_COMMON->CSR;
    }
    const uint32_t c1 = TA.CNTxR;
    const uint32_t k = T->pr_n;
    if (k < BUCK_TOC_PROBES) {
        T->pr_c0[k] = c0; T->pr_c2[k] = c2; T->pr_c1[k] = c1; T->pr_st[k] = st;
        T->pr_n = k + 1U;
    }
    *cnt = c1;
    return st;
}

/* The sample of a pass the TOC code runs (31 Oct): used only if the wait returned with the Timer A counter in
 * [BUCK_TOC_F_EARLIEST, BUCK_TOC_PASS_LAST) (the pass's CMP1 write then meets its deadline) and both results are
 * there without an overrun. Otherwise dropped, and the next one taken: the CMP1 in force (the held duty after a
 * frame) stays one period more. On the board the code after a frame runs ~1.5 x slower than counted and reached
 * its samples up to a period late: T's ADC2 read raced the next group (fault 4), a CMP1 write missed its window
 * (fault 5). A drop late in the period takes both results (ADC2's group of this period goes with ADC1's pair);
 * early in the period (before F_EARLIEST, ADC1's result is the last period's) ADC2's is kept when it overran (its
 * data register then holds this period's group). Returns 0 with the data registers read, else the bad bits
 * (ADC12_COMMON->CSR; 0x1: still late after BUCK_TOC_DROP_MAX drops). */
static TOC_CODE uint32_t toc_sample(buck_toc_t *T, uint32_t *adc_v, uint32_t *adc_i4)
{
    for (uint32_t k = 0U; ; ++k) {
        uint32_t c;
        const uint32_t st = toc_wait(&c);
        const uint32_t bad = (st & CSR_BAD) ^ ADC_CSR_EOC_SLV;
        const uint32_t on_time = c - BUCK_TOC_F_EARLIEST < BUCK_TOC_PASS_LAST - BUCK_TOC_F_EARLIEST;
        if ((bad == 0U && on_time) || k == BUCK_TOC_DROP_MAX) {
            *adc_v = ADC1->DR; *adc_i4 = ADC2->DR;
            return (bad == 0U && on_time) ? 0U : (bad | ADC_CSR_ADRDY_MST);
        }
        (void)ADC1->DR;
        if (!(c < BUCK_TOC_F_EARLIEST && (st & ADC_CSR_OVR_SLV))) (void)ADC2->DR;
        ADC1->ISR = ADC_ISR_OVR;
        ADC2->ISR = ADC_ISR_OVR;
        toc_ev(EV_DROP | toc_ev_state(st));
        ++T->drop_n;
    }
}

/* (Timer A counter) - (master counter), read back to back: a constant while the grid is locked. */
static TOC_CODE int32_t toc_align(void)
{
    const uint32_t mc = HM.MCNTR;
    const uint32_t tc = TA.CNTxR;
    int32_t d = (int32_t)tc - (int32_t)mc;
    if (d > (int32_t)(BUCK_HRTIM_PERIOD / 2U)) d -= (int32_t)BUCK_HRTIM_PERIOD;
    else if (d < -(int32_t)(BUCK_HRTIM_PERIOD / 2U)) d += (int32_t)BUCK_HRTIM_PERIOD;
    return d;
}

/* The gains in use, by span (g1 the first span's, g2 the second's) and order, and the equilibrium
 * corrections: g_buck.toc.g and .b checked (buck_toc_gain_ok, buck_toc_b_ok) after every edge and at the
 * run's entry, so that the start only loads them. */
static TOC_CODE void toc_gains_take(buck_toc_mb_t *m, const buck_toc_t *T)
{
    m->b[BUCK_TOC_ON_OFF] = buck_toc_b_ok(T->b[BUCK_TOC_ON_OFF]);
    m->b[BUCK_TOC_OFF_ON] = buck_toc_b_ok(T->b[BUCK_TOC_OFF_ON]);
    m->g1[BUCK_TOC_ON_OFF] = buck_toc_gain_ok(T->g[BUCK_TOC_ON_OFF][0]);    /* g_on */
    m->g2[BUCK_TOC_ON_OFF] = buck_toc_gain_ok(T->g[BUCK_TOC_ON_OFF][1]);    /* g_off */
    m->g1[BUCK_TOC_OFF_ON] = buck_toc_gain_ok(T->g[BUCK_TOC_OFF_ON][1]);    /* g_off */
    m->g2[BUCK_TOC_OFF_ON] = buck_toc_gain_ok(T->g[BUCK_TOC_OFF_ON][0]);    /* g_on */
}

/* An ordinary pass run by the TOC code (the passes after a frame): the next sample in time (toc_sample), both
 * PIs, CMP1 through the guard (a miss: fault 5, as in the loop), the score, the observer, A2, D4. reseed (the
 * restart pass, user): the voltage PI's integrator := the measured mean current (clamped), the current
 * PI's := d_hold, before the PIs. *v_out, *i_out: the sample's v_out and measured mean current.
 * Returns 1 when the run ended (a STOP, a bad sample, a missed guard: m->ret = EXIT), else 0. */
static TOC_CODE uint32_t toc_pass(buck_toc_mb_t *m, buck_t *b, const buck_tune_t *t, uint32_t reseed, float d_hold,
                                  float *v_out, float *i_out)
{
    const buck_kc_t kc = buck_kc_init();
    uint32_t cnt, adc_v, adc_i4;
    const uint32_t bad = toc_sample(&b->toc, &adc_v, &adc_i4);
    if (reseed) b->toc.c_rw = b->toc.pr_c1[b->toc.pr_n - 1U];  /* diagnostics: the restart pass's sample (its wait) */
    const uint32_t cmd = *(volatile const uint32_t *)&b->command;
    if (bad || cmd != BUCK_RUN) { toc_exit(b, bad, adc_v, adc_i4, 3U); return 1U; }
    if (reseed) {
        float iref0 = buck_adc_to_i_o(t, adc_i4, 0);
        iref0 = (iref0 > BUCK_IREF_MIN_A) ? iref0 : BUCK_IREF_MIN_A;   /* (a NaN ends at the lower bound) */
        iref0 = (iref0 < BUCK_IREF_MAX_A) ? iref0 : BUCK_IREF_MAX_A;
        m->s.integ_v = iref0;
        m->s.integ_i = d_hold;
        b->toc.i_first = iref0;
    }
    buck_step_t stp;
    buck_fast_pend_t pd;
    const uint32_t cmp = buck_ctrl_fast_pre_kc(t, m->ki_t_v, m->ki_t_i, &m->s, adc_v, adc_i4, &stp, &pd, &kc, 0);
    {
        const uint32_t early = BUCK_TOC_F_EARLIEST, win = BUCK_WRITE_DEADLINE - BUCK_TOC_F_EARLIEST;
        TOC_GUARD(cmp, &TA.CMP1xR, early, win, cnt, late);
    }
    b->toc.c_pass = cnt;                        /* board timing: this pass's CMP1 write */
    buck_ctrl_fast_commit(&m->s, &stp, &pd);
    buck_out_t o;
    const uint32_t g = buck_ctrl_slow_ko(t, m->sk, &m->s, &stp, &o, 0);
    DAC1->DHR12R1 = buck_iload_dac(buck_obs_step_a(&m->obs, stp.v, stp.im, m->ok, BUCK_OBS_A));
    GPIOB->BSRR = g ? D4_PIN : D4_PIN << 16;
    m->loops += 1U;
    m->v_prev = stp.v;
    b->adc_v_raw = adc_v; b->adc_i4_raw = adc_i4;
    *v_out = stp.v; *i_out = stp.im;
    return 0U;
late:
    toc_fail(b, BUCK_FAULT_DEADLINE, cnt);
    return 1U;
}

/* buck_toc_ret() after its guard (off the return words' path: its own registers, 31 Oct): the alignment reference
 * for T's check, the event log's start, then the period's own A0 pair (its EOC ~1400 ticks after its second
 * trigger) and ADC2's group of the frame, read and dropped. Integer only (BUCK_TOC_CALL_CORE). */
static TOC_CODE void toc_ret_tail(buck_toc_frame_t *f)
{
    buck_toc_t *const T = &g_buck.toc;
    f->align = toc_align();
    T->ev_cyc0 = DWT->CYCCNT;
    T->ev_n = 0U;
    uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD;
    uint32_t st = ADC12_COMMON->CSR;
    toc_ev(EV_START | toc_ev_state(st));
    while ((st & ADC_CSR_EOC_MST) == 0U) {
        if (--n == 0U) {
            T->f4_site = 5U; T->f4_csr = st; T->f4_cnt = TA.CNTxR;
            buck_fatal(BUCK_FAULT_ADC);
        }
        st = ADC12_COMMON->CSR;
    }
    f->v_mid_code = ADC1->DR;
    if (st & ADC_CSR_EOC_SLV) (void)ADC2->DR;  /* ADC2's group of the frame (unused), here and not in T: the next
                                                  group ends 570 ticks into P2, T may come later (31 Oct) */
}

/* ---- after the start -------------------------------------------------------------------------- */
/* Right after the second update (Inc/buck_toc_start.h, through BUCK_TOC_CALL_CORE: no floating point here):
 * the return words for the first normal period P1 into the preloads (they transfer at PER'): the held duty,
 * the duty that holds I_eq, formed by the start before its wait (without DT after OFF-ON: the high side is on
 * across the frame's end; with the OFF-ON tail with DT; the counter read after that wait leaves
 * BUCK_TOC_END_LEAD_TICKS to PER'); then the period's own A0
 * pair, converted inside the frame, is read and dropped (before the frame's end pair converts: else an
 * overrun). A missed guard sets req = FAIL (buck_toc_finish trips fault 9). */
TOC_CODE void buck_toc_ret(void)
{
    buck_toc_mb_t *const m = &g_toc_mb;
    buck_toc_frame_t *const f = &m->f;
    uint32_t cnt;
    const uint32_t ret_cmp1 = f->ret_cmp1, end = f->per - BUCK_TOC_RET_GUARD_TICKS;   /* the start's */
    HM.MICR = HRTIM_MICR_MUPD;                  /* the frame's end transfer sets them again (finish, T) */
    TA.TIMxICR = HRTIM_TIMICR_UPDC;
    TA.CMP2xR = BUCK_TOC_CMP2_BACKSTOP;
    TA.CMP3xR = BUCK_VSAMPLE_CMP;
    TA.CMP4xR = BUCK_VSAMPLE0_CMP;
    TA.SETx1R = HRTIM_SET1R_PER;
    TA.RSTx1R = HRTIM_RST1R_CMP1 | HRTIM_RST1R_CMP2;
    TA.CMP1xR = ret_cmp1;
    TA.PERxR = BUCK_HRTIM_PERIOD;
    {
        const uint32_t early = 0U, mper = BUCK_HRTIM_PERIOD;
        TOC_GUARD(mper, &HM.MPER, early, end, cnt, late);
    }
    f->c_ret = cnt;                             /* board timing (31 Oct) */
    toc_ret_tail(f);
    return;
late:
    f->fail_cnt = cnt;
    m->req = BUCK_TOC_REQ_FAIL;
}

/* After the pass that started the frame (its return words and its observer step are in: Src/buck.c,
 * Inc/buck_toc_start.h): the frame's record, the pass's bookkeeping and, with a frame, its two samples
 * (Inc/buck_toc_fw.h). run_loop has copied its state into the mailbox; it takes s, obs and loops back
 * unless ret = EXIT. */
TOC_CODE void buck_toc_finish(void)
{
    buck_toc_mb_t *const m = &g_toc_mb;
    buck_t *const b = &g_buck;
    buck_toc_t *const T = &b->toc;
    const buck_toc_frame_t *const f = &m->f;
    uint32_t cnt;
    m->ret = BUCK_TOC_RET_STATE;
    const uint32_t hold_cmp = f->hold_cmp;
    const float d_hold = f->d_hold;
    if (m->req == BUCK_TOC_REQ_FAIL) { toc_fail(b, BUCK_FAULT_TOC, f->fail_cnt); return; }
    if (m->req == BUCK_TOC_REQ_FRAME) {
        T->order = f->order; T->sat = f->sat; T->c_start = f->c_s; T->t1_ticks = f->t1;
        /* the second span applied: with the tail (OFF-ON) from the ON span's start, c_s + t1, to its end */
        T->t2_ticks = (f->sat & BUCK_TOC_SAT_TAIL) ? f->t_end - f->c_s - f->t1 : f->t2;
        T->per_ticks = f->per; T->hold_cmp = hold_cmp; T->v0 = m->v; T->i0 = m->im; T->ieq = f->sol.ieq;
        T->a = f->sol.a; T->t1_model = f->sol.t1; T->t2_model = f->sol.t2; T->v_mid = (float)f->v_mid_code * VSUM;
        T->c_swu1 = f->c_swu1; T->c2 = f->c2; T->c_swu2 = f->c_swu2; T->c_ret = f->c_ret;   /* board timing */
        T->c_t = T->c_first = T->c_back = T->csr_back = T->c_tw = T->c_rw = 0U;
        T->pr_n = 0U;
        if (!f->ok) ++T->rej[BUCK_TOCR_NONE];
        T->state = BUCK_TOCS_FRAME;
        T->align_ref = f->align;                /* buck_toc_ret's, inside the frame */
    }
    /* the pass that started it (its observer step, A2 and D4 are its own, Src/buck.c run_pass) */
    m->loops += 1U;
    m->v_prev = m->v;
    m->trig = 0U;                               /* a new request needs g to fall first */
    if (m->req != BUCK_TOC_REQ_FRAME) {
        if (buck_f2u(m->k.vref) != buck_f2u(m->vref)) buck_toc_k_init(&m->k, m->vref);
        toc_gains_take(m, T);
        T->state = BUCK_TOCS_IDLE;
        return;
    }

    /* ---- T: the sample at the frame's end (read in P1, which runs at the held duty; on the board, with the code
     * after the frame ~1.5 x slower than counted, up to a period later, 31 Oct). ADC1's only: ADC2's group of the
     * frame was dropped in buck_toc_ret, its next one is the restart pass's. Overwritten (ADC1 overrun: T came
     * after P1's pair): the newest is used for the observer, no learning from this frame. */
    float v_end;
    uint32_t lost = 0U;
    {
        uint32_t c;
        const uint32_t st = toc_wait(&c);
        T->c_tw = c;                            /* diagnostics (31 Oct): T's sample taken (P1's counter) */
        const uint32_t adc_v = ADC1->DR;
        const uint32_t cmd = *(volatile const uint32_t *)&b->command;
        if (st & ADC_CSR_OVR_MST) {
            ADC1->ISR = ADC_ISR_OVR;
            lost = 1U;
            ++T->t_lost;
            toc_ev(EV_TLOST | toc_ev_state(st));
        }
        /* the frame's end transfer happened (MUPD / UPD, cleared after the updates) */
        if (!(HM.MISR & HRTIM_MISR_MUPD) || !(TA.TIMxISR & HRTIM_TIMISR_UPD)) {
            ++T->chk_commit;
            if (BUCK_TOC_CHECK_TRIP) { toc_fail(b, BUCK_FAULT_TOC, 0U); return; }
        }
        T->align_last = toc_align();
        const int32_t da = T->align_last - T->align_ref;
        if (da > BUCK_TOC_ALIGN_TOL || da < -BUCK_TOC_ALIGN_TOL) {
            ++T->chk_align;
            if (BUCK_TOC_CHECK_TRIP) { toc_fail(b, BUCK_FAULT_TOC, 0U); return; }
        }
        /* P2 at the held duty too (the sample-to-duty pipeline: the PIs restart at sample 1). Outside its window
         * (31 Oct) the write is skipped: P1's return words (the held duty; after OFF-ON without the tail DT
         * shorter) stay until the restart pass's CMP1 */
        uint32_t wrote = 0U;
        {
            const uint32_t early = BUCK_TOC_F_EARLIEST, win = BUCK_WRITE_DEADLINE - BUCK_TOC_F_EARLIEST;
            TOC_GUARD(hold_cmp, &TA.CMP1xR, early, win, cnt, t_skip);
        }
        wrote = 1U;
    t_skip:
        T->c_t = cnt;                           /* board timing: T's CMP1 write (or its miss), P1's counter */
        if (!wrote) { ++T->t_skip; toc_ev(EV_TSKIP); }
        v_end = (float)adc_v * VSUM;
        m->obs.vh = v_end;                      /* the observer's v^ at the frame's end; i^ kept */
        DAC1->DHR12R1 = buck_iload_dac(m->obs.ih);
        m->s.cmp_active = wrote ? hold_cmp : f->ret_cmp1;
        m->loops += 1U;
        T->v_end = v_end;
        T->z_v = v_end - m->vref;
        T->state = BUCK_TOCS_END;
        b->adc_v_raw = adc_v;
        if (cmd != BUCK_RUN) { toc_exit(b, 0U, adc_v, b->adc_i4_raw, 2U); return; }
    }

    /* ---- 1: the sample after the first period: the PIs restart from it (user: the voltage PI's
     * integrator = the first period's measured mean current, no reference learner; the current PI's
     * = the held duty), an ordinary pass otherwise */
    T->state = BUCK_TOCS_FIRST;
    buck_tune_t t;                              /* the loop's held tunables, the fields the pass uses */
    t.kp_v = m->kp_v; t.kp_i = m->kp_i; t.isense_zero = m->isense_zero; t.isense_gain = m->isense_gain;
    t.s_th = m->s_th;
    float v1, i1;
    {
        const uint32_t d0 = T->drop_n;
        if (toc_pass(m, b, &t, 1U, d_hold, &v1, &i1)) return;
        if (T->drop_n != d0) lost = 1U;         /* not P1's sample: the residuals would not be the frame's */
    }
    T->c_first = T->c_pass;                     /* board timing: the restart pass's CMP1 write, P2's counter */
    /* the residuals (Inc/buck_toc.h): the landing and the end voltage for the timing, the load over P1
     * against the equilibrium the frame was solved for for b (stage 4b) */
    const float ieq = f->sol.ieq;
    const float zv = v_end - m->vref, zi = i1 - ieq;
    const float i_load = i1 - m->k.cdt * (v1 - v_end), e_b = i_load - ieq;
    T->v_1 = v1; T->z_i = zi; T->i_load = i_load; T->e_b = e_b;
    const uint32_t learn = T->learn_enable;
    const uint32_t low = !(i1 >= BUCK_TOC_LEARN_IMIN_A);    /* A1 reads a negative valley as 0 (also a NaN) */
    if (learn && low) ++T->learn_low;
    /* the equilibrium correction at once (a few clocks) */
    if (learn && !low && !lost && buck_toc_learn_b(e_b, &T->b[f->order])) { m->b[f->order] = T->b[f->order]; ++T->b_n; }
    /* the Jacobian step in two parts, each in the spare tail of a pass this code runs (the restart pass's
     * tail is too short for it: README "Clock budget"), only while the counter leaves a part's time before
     * the next sample; otherwise the next sample's ordinary pass first (at most BUCK_TOC_LEARN_PASSES) */
    uint32_t part = (learn && f->ok && !low && !lost) ? 1U : 3U;    /* 1: the span steps, 2: the gains, 3: done */
    buck_toc_dt_t d;
    const float t1s = (float)f->t1 * (float)BUCK_TICK_S, t2s = (float)f->t2 * (float)BUCK_TICK_S;
    const uint32_t sat1 = (f->sat & (BUCK_TOC_SAT_LIMIT1 | BUCK_TOC_SAT_LATE1)) != 0U;
    const uint32_t sat2 = (f->sat & (BUCK_TOC_SAT_LIMIT2 | BUCK_TOC_SAT_PER)) != 0U;
    for (uint32_t n = 0U; part < 3U; ) {
        const uint32_t c = TA.CNTxR;
        if (c >= BUCK_TOC_LEARN_PART_FIRST && c < BUCK_TOC_LEARN_PART_LAST && !(ADC12_COMMON->CSR & ADC_CSR_EOC_MST)) {
            if (part == 1U) {
                part = buck_toc_learn_dt(&m->k, f->order, f->sol.a, t1s, t2s, sat1, zi, zv, &d) ? 2U : 4U;
            } else {
                float g1 = f->g1, g2 = f->g2;
                buck_toc_learn_g(t1s, t2s, sat1, sat2, &d, &g1, &g2);
                if (f->order == BUCK_TOC_ON_OFF) { T->g[0][0] = g1; T->g[0][1] = g2; }
                else                             { T->g[1][1] = g1; T->g[1][0] = g2; }
                m->g1[f->order] = g1; m->g2[f->order] = g2;     /* in range by construction (buck_toc_gclip) */
                ++T->learn_n;
                part = 3U;
            }
        } else if (n < BUCK_TOC_LEARN_PASSES) {
            if (toc_pass(m, b, &t, 0U, 0.0f, &v1, &i1)) return;
            ++n;
        } else {
            ++T->learn_late;                    /* not done within the passes: dropped */
            break;
        }
    }
    if (part != 3U) ++T->learn_skip;            /* off, a low current, no pair, beyond SKIP, or late */
    /* the extra passes (BUCK_TOC_PROBE_PASSES): hand back where the loop waits for its sample */
    for (uint32_t n = 0U; n < BUCK_TOC_PROBE_PASSES; ++n)
        if (toc_pass(m, b, &t, 0U, 0.0f, &v1, &i1)) return;
    /* debugger writes of g / b, for the next frame (the learning above writes its results directly): only with
     * a part's time to spare, as a learning part (31 Oct: in P1 after T, ~100 clocks static and more on the
     * board, it delayed every pass after it); else at the next edge's end or the run's entry */
    {
        const uint32_t c = TA.CNTxR;
        if (c >= BUCK_TOC_LEARN_PART_FIRST && c < BUCK_TOC_LEARN_PART_LAST && !(ADC12_COMMON->CSR & ADC_CSR_EOC_MST))
            toc_gains_take(m, T);
    }
    /* board timing: the counter when the loop takes over, and whether its sample is already waiting */
    T->c_back = TA.CNTxR;
    T->csr_back = ADC12_COMMON->CSR;
    ++T->frames;
    T->state = BUCK_TOCS_IDLE;
    m->en = 0U;                                 /* the hold-off: the telemetry pass re-enables (run_pass) */
    m->hold_until = m->loops + BUCK_TOC_HOLDOFF_PASSES;
}

/* The constants for V_ref (run_loop's entry, with the target the ramp ends at) and the gains. */
TOC_COLD void buck_toc_prime(void)
{
    buck_toc_mb_t *const m = &g_toc_mb;
    if (buck_f2u(m->k.vref) != buck_f2u(m->vref)) buck_toc_k_init(&m->k, m->vref);
    toc_gains_take(m, &g_buck.toc);
}
