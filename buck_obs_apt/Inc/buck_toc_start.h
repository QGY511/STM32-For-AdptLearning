#ifndef BUCK_TOC_START_H
#define BUCK_TOC_START_H

/* The start of a feed-forward TOC frame (stage 4, 30 Oct 2026), inline in the HOLD pass whose score
 * rose, before its PIs (Src/buck.c run_pass; Inc/buck_toc_fw.h has the sequence, Src/buck_toc.c the
 * rest). Inline because a call (BUCK_TOC_CALL's save and restore and the callee's prologue, ~50
 * clocks) would delay the start by that much (README "Clock budget"). No CMP1 write here (make check
 * rule 3): the return words, CMP1 of the held duty with them, are buck_toc_ret()'s (BUCK_TOC_CALL).
 *
 * The frame is the period in progress, stretched. The master and Timer A counters run on from where
 * the pass found them (they stay locked: both periods change in the same software updates), so every
 * compare of the frame is a counter value of that period:
 *   ON-OFF  (v < V_ref)  SET1R = PER, RST1R = CMP2:  SST at c_s (high side on after DT),
 *                        CMP2' = c_s + DT + t1 (off), PER' = CMP2' + t2 - DT (the next period's
 *                        high side after DT: the fall lasts t2);
 *   OFF-ON  (v > V_ref)  SET1R = CMP2, RST1R = none: SRT at c_s (high side off, low side on),
 *                        CMP2' = c_s + t1 - DT (on after DT), PER' = CMP2' + DT + t2; the high side
 *                        stays on into the first period (no dead time there: its CMP1 is the held
 *                        duty without DT).
 * The pass has not written CMP1 (the frame starts before its PIs, Src/buck.c): CMP1's active value is
 * the period's own, and no reset source of the frame uses it. If the frame starts while the period's
 * own on-time still runs (the counter below its CMP1), ON-OFF's first span simply goes on from it, and
 * OFF-ON's SRT ends it. The period's own A0 pair (6502 and 8422 ticks) still converts inside the frame
 * (buck_toc_finish() reads and drops that result); a second pair is set at the frame's end (CMP3' =
 * PER' - 538, CMP4' = PER' - 2458). The master's four current compares stay (all four fire once in the
 * stretched period, so ADC2's group stays four samples; its result for the frame is not used).
 *
 * The first span starts with the SST / SRT, before anything is computed (user: no delay, feed-forward).
 * Until the first update its end is the backstop every normal period carries (CMP2 =
 * BUCK_TOC_CMP2_BACKSTOP in RST1R, a no-op in a normal period). Two software updates (CR2 = MSWU |
 * TASWU), each guarded on the counter:
 *   1. the first span's end from the L-C solve (everything from the SST to this update is on the span's
 *      time: the margin it lasts at least, user), clamped to the limit (the peak / valley current),
 *      SET1R/RST1R of the order, a provisional end PER' far enough away; a miss (the span's end or the
 *      backstop would come first) ends the forced span at once: no frame (LATE);
 *   2. after the period's own A0 pair has been triggered: the frame's end PER' (and MPER'), the A0 pair
 *      before it. Then buck_toc_ret(): the return words for the first normal period. */

#include "stm32g4xx.h"
#include <stddef.h>
#include "buck.h"
#include "buck_toc_fw.h"

#define BUCK_TOC_SWU (HRTIM_CR2_MSWU | HRTIM_CR2_TASWU)

/* f + 0.5 converted to ticks, saturating: VCVT.U32 gives 0 for a negative value or a NaN and
 * 0xFFFFFFFF above the range (defined on the target; C's conversion would not be). */
BUCK_INLINE uint32_t buck_toc_ticks(float f)
{
    float r = f + 0.5f;
    __asm__("vcvt.u32.f32 %0, %0" : "+t"(r));
    uint32_t u;
    __asm__("vmov %0, %1" : "=r"(u) : "t"(r));
    return u;
}
BUCK_INLINE uint32_t buck_toc_umax(uint32_t a, uint32_t b) { return (a > b) ? a : b; }
BUCK_INLINE uint32_t buck_toc_umin(uint32_t a, uint32_t b) { return (a < b) ? a : b; }

/* One order (a constant in each of the two copies: nothing order-dependent is decided at run time).
 * Sets m->req: FRAME, or FAIL (the second update's or the return words' guard missed: buck_toc_finish()
 * trips fault 9); it stays NONE for a LATE reject (the forced span ended at once). */
BUCK_INLINE void buck_toc_go(const uint32_t order, HRTIM_TypeDef *const hr, buck_toc_mb_t *m, float v, float im)
{
    const uint32_t on_off = (order == BUCK_TOC_ON_OFF);
    /* the first span's limit: the peak (ON) / valley (OFF) current from i0, and the span maximum (here,
     * before the span: a few clocks of delay, off the span's time) */
    const uint32_t lim = buck_toc_umin(on_off ? buck_toc_ticks((BUCK_TOC_IPK_MAX_A - im) * m->k.t_m1)
                                              : buck_toc_ticks((im - BUCK_TOC_IVAL_MIN_A) * m->k.t_m2),
                                       on_off ? BUCK_TOC_TON_MAX_TICKS : BUCK_TOC_TOFF_MAX_TICKS);
    /* the first span, now (user: no delay, the feed-forward): the high side on (ON-OFF) / off (OFF-ON).
     * Until the first update its end is the normal periods' backstop, CMP2 = BUCK_TOC_CMP2_BACKSTOP */
    if (on_off) hr->sTimerxRegs[0].SETx1R = HRTIM_SET1R_PER | HRTIM_SET1R_SST;
    else        hr->sTimerxRegs[0].RSTx1R = HRTIM_RST1R_CMP1 | HRTIM_RST1R_CMP2 | HRTIM_RST1R_SRT;
    const uint32_t c_s = hr->sTimerxRegs[0].CNTxR;
    uint32_t cnt;
    /* the solve after the SST (its inputs depend on c_s, read after the SST: GCC would otherwise schedule
     * the arithmetic, the square root included, in front of the SST's store) */
    __asm__("" : "+t"(v), "+t"(im) : "r"(c_s));

    /* the L-C solve's first span (Inc/buck_toc.h) from the pass's measured average and v_out, while the
     * span runs (in registers: into the record only at the end), clamped to the limit */
    buck_toc_sol_t sol;
    buck_toc_solve1(&m->k, order, v, m->v_prev, im, m->b[order], &sol);
    const float g1 = m->g1[order];
    uint32_t t1 = buck_toc_ticks(sol.t1 * g1), sat = 0U;
    if (__builtin_expect(t1 > lim, 0)) { t1 = lim; sat = BUCK_TOC_SAT_LIMIT1; }
    /* the provisional frame (the order's sources, an end far enough away), then the counter after the
     * span exists and these stores are done (the barrier: GCC would read it before the solve), so that
     * only the first span's end and its guard come after it (the stores' bus time is not in the guard's
     * budget) */
    const uint32_t per0 = c_s + BUCK_TOC_PER0_REL_TICKS;
    hr->sTimerxRegs[0].SETx1R = on_off ? HRTIM_SET1R_PER : HRTIM_SET1R_CMP2;
    hr->sTimerxRegs[0].RSTx1R = on_off ? HRTIM_RST1R_CMP2 : 0U;     /* OFF-ON: no reset in the frame */
    hr->sTimerxRegs[0].PERxR = per0;
    hr->sMasterRegs.MPER = per0;
    __asm__ volatile("" : : "r"(t1) : "memory");
    const uint32_t cn = hr->sTimerxRegs[0].CNTxR;
    uint32_t cmp2 = on_off ? c_s + BUCK_DT_TICKS + t1 : c_s + t1 - BUCK_DT_TICKS;
    {
        const uint32_t e1 = cn + BUCK_TOC_SWU_GUARD_TICKS;
        if (__builtin_expect(cmp2 < e1, 0)) { cmp2 = e1; sat = BUCK_TOC_SAT_LATE1; }
    }
    hr->sTimerxRegs[0].CMP2xR = cmp2;
    {
        /* 1. the first update: before the first span's end and before the backstop */
        const uint32_t early = 0U, end = buck_toc_umin(cmp2, BUCK_TOC_CMP2_BACKSTOP) - 64U, swu = BUCK_TOC_SWU;
        TOC_GUARDB(hr, swu, &hr->sCommonRegs.CR2, early, end, cnt, missed_start);
    }
    m->f.c_swu1 = cnt;                          /* board timing (31 Oct) */
    /* 2. off the first span's time from here: the second span and the frame's end */
    const int32_t ok = buck_toc_ok(&m->k, order, &sol);
    buck_toc_solve2(&m->k, order, &sol);
    const float g2 = m->g2[order];
    const uint32_t t1a = on_off ? cmp2 - c_s - BUCK_DT_TICKS : cmp2 + BUCK_DT_TICKS - c_s;
    if (!ok) sat |= BUCK_TOC_SAT_LATE1;         /* no solution: the second span as after a late first */
    /* the second span: as solved, or (the first span not as solved) the one that lands the current on
     * the target valley from where the first ended */
    uint32_t t2 = (sat & (BUCK_TOC_SAT_LATE1 | BUCK_TOC_SAT_LIMIT1))
                  ? buck_toc_ticks(buck_toc_t2_for(&m->k, order, sol.a, (float)t1a) * g2)
                  : buck_toc_ticks(sol.t2 * g2);
    {
        const uint32_t lo = on_off ? BUCK_TOC_OFF_MIN_TICKS : BUCK_TOC_ON_MIN_TICKS;
        const uint32_t hi = on_off ? BUCK_TOC_TOFF_MAX_TICKS : BUCK_TOC_TON_MAX_TICKS;
        if (t2 < lo) { t2 = lo; sat |= BUCK_TOC_SAT_LIMIT2; }
        if (t2 > hi) { t2 = hi; sat |= BUCK_TOC_SAT_LIMIT2; }
    }
    uint32_t per = on_off ? cmp2 + t2 - BUCK_DT_TICKS : cmp2 + BUCK_DT_TICKS + t2;
    /* the held duty of P1 and P2, for buck_toc_ret() and buck_toc_finish(): here, before the wait, off the
     * path from the counter read below to the return words (31 Oct) */
    {
        buck_toc_frame_t *const f = &m->f;
        const float d_hold = buck_toc_hold_duty(m->vref, sol.ieq);
        const uint32_t hold_cmp = buck_duty_to_cmp(d_hold);
        f->d_hold = d_hold;
        f->hold_cmp = hold_cmp;
        f->ret_cmp1 = on_off ? hold_cmp : hold_cmp - BUCK_DT_TICKS;   /* P1 after OFF-ON: no dead time */
        f->t_chk = 0U;
    }
    /* the second update comes after the period's own A0 pair has been triggered (the pair stays: moving
     * its second trigger would split it) */
    while (hr->sTimerxRegs[0].CNTxR < BUCK_TOC_SWU2_FIRST) {}
    {
        /* the frame's end far enough after this read for the second update and the return words
         * (BUCK_TOC_END_LEAD_TICKS) */
        const uint32_t c2 = hr->sTimerxRegs[0].CNTxR;
        m->f.c2 = c2;                           /* board timing (31 Oct) */
        const uint32_t lo = buck_toc_umax(c2 + BUCK_TOC_END_LEAD_TICKS, BUCK_TOC_PER_MIN_TICKS);
        if (__builtin_expect(per < lo, 0)) {
            per = lo; sat |= on_off ? BUCK_TOC_SAT_PER : BUCK_TOC_SAT_PER | BUCK_TOC_SAT_TAIL;
            if (!on_off) {
                /* OFF-ON (31 Oct): the high side would stay on to the moved end (the current up by
                 * (V_in - V_ref)/L for every tick of it). The ON span ends as solved, through Timer A CMP1
                 * (no source of the frame so far) as the reset from this update, if that end lies
                 * BUCK_TOC_TAIL_LEAD_TICKS beyond this read; checked after the update (below: SRT if the
                 * counter is near or past it). Otherwise it ends now, by software (SRT): on the board
                 * (31 Oct, one frame) this read came ~1.4 x later than counted, the update missed a compare
                 * set 2048 ticks ahead, and the high side stayed on into P1 (+27 A). The rest of the frame
                 * with the low side on, P1 as after ON-OFF (its CMP1 with DT). make check rule 3 takes the
                 * CMP1 store for the tail's. The ON span applied is the mailbox's (t_end). */
                const uint32_t t_on = cmp2 + BUCK_DT_TICKS + t2;
                if (t_on >= c2 + BUCK_TOC_TAIL_LEAD_TICKS) {
                    hr->sTimerxRegs[0].CMP1xR = t_on;
                    hr->sTimerxRegs[0].RSTx1R = HRTIM_RST1R_CMP1;
                    m->f.t_end = t_on;
                    m->f.t_chk = t_on;
                } else {
                    hr->sTimerxRegs[0].RSTx1R = HRTIM_RST1R_SRT;    /* no reset source in the frame after it */
                    m->f.t_end = c2;
                    sat |= BUCK_TOC_SAT_TAIL_NOW;
                }
                m->f.ret_cmp1 = m->f.hold_cmp;
            }
        }
        if (per > BUCK_TOC_PER_MAX_TICKS) { per = BUCK_TOC_PER_MAX_TICKS; sat |= BUCK_TOC_SAT_LIMIT2; }
    }
    hr->sTimerxRegs[0].PERxR = per;
    hr->sTimerxRegs[0].CMP3xR = per - BUCK_VSAMPLE_LEAD_TICKS;
    hr->sTimerxRegs[0].CMP4xR = per - BUCK_TOC_A0_LEAD_TICKS;
    hr->sMasterRegs.MPER = per;
    {
        /* after the period's own pair, before the provisional frame's end */
        const uint32_t early = BUCK_TOC_SWU2_FIRST, win = per0 - BUCK_TOC_RET_GUARD_TICKS - BUCK_TOC_SWU2_FIRST;
        const uint32_t swu = BUCK_TOC_SWU;
        TOC_GUARDB(hr, swu, &hr->sCommonRegs.CR2, early, win, cnt, missed_frame);
    }
    /* the tail's compare not passed before the update took it (it would pass unseen, the high side on into P1):
     * SRT when the counter is within 64 ticks of it or past it (harmless if it fired) */
    {
        const uint32_t t_chk = m->f.t_chk;
        if (__builtin_expect(t_chk != 0U, 0) && hr->sTimerxRegs[0].CNTxR + 64U >= t_chk) {
            hr->sTimerxRegs[0].RSTx1R = HRTIM_RST1R_CMP1 | HRTIM_RST1R_SRT;
            sat |= BUCK_TOC_SAT_TAIL_SRT;
        }
    }
    /* the return words at once (buck_toc_ret: the held duty above, guarded against PER'), the record after
     * them (make check rule 5d: the call follows the update) */
    m->f.c_swu2 = cnt;                          /* board timing (31 Oct) */
    m->f.per = per;
    m->req = BUCK_TOC_REQ_FRAME;
    BUCK_TOC_CALL_CORE(buck_toc_ret);           /* sets req = FAIL if its guard misses */
    {
        buck_toc_frame_t *const f = &m->f;
        f->order = order; f->sat = sat; f->ok = (uint32_t)ok; f->c_s = c_s; f->t1 = t1a; f->t2 = t2;
        f->sol = sol;
        f->g1 = m->g1[order]; f->g2 = m->g2[order];     /* g1, g2 again: kept across the call they cost a
                                                           spill inside the first span's solve */
    }
    return;
missed_start:
    /* the first update would come too late (the span's end or the backstop first; nothing transferred):
     * the forced span ends now (ON-OFF; OFF-ON's high side stays off until the next period, as normal), and
     * the normal words go back here and are taken up at once by a software update (31 Oct: through
     * buck_toc_start_late() alone, ~105 clocks on the board, a miss near the backstop restored them after the
     * period's end, whose update took the provisional words: a period of PER0' with the order's set/reset,
     * a long ON span). CMP1, CMP3 and CMP4 the start never wrote: the update transfers them unchanged. The
     * counter is below 8960 then (or, past the roll-over, low): PER 8960 lies ahead either way. A LATE reject */
    if (on_off) hr->sTimerxRegs[0].RSTx1R = HRTIM_RST1R_CMP1 | HRTIM_RST1R_CMP2 | HRTIM_RST1R_SRT;
    hr->sTimerxRegs[0].CMP2xR = BUCK_TOC_CMP2_BACKSTOP;
    hr->sTimerxRegs[0].SETx1R = HRTIM_SET1R_PER;
    hr->sTimerxRegs[0].RSTx1R = HRTIM_RST1R_CMP1 | HRTIM_RST1R_CMP2;
    hr->sTimerxRegs[0].PERxR = BUCK_HRTIM_PERIOD;
    hr->sMasterRegs.MPER = BUCK_HRTIM_PERIOD;
    hr->sCommonRegs.CR2 = BUCK_TOC_SWU;
    BUCK_TOC_CALL(buck_toc_start_late);         /* the count (and the same words again, harmless) */
    return;
missed_frame:
    m->f.fail_cnt = cnt;
    m->req = BUCK_TOC_REQ_FAIL;
}

/* The start: v, im, vref of the pass (also into the mailbox for buck_toc_finish). */
BUCK_INLINE void buck_toc_start_inline(buck_t *b, HRTIM_TypeDef *const hr, float v, float im, float vref)
{
    buck_toc_mb_t *const m = &g_toc_mb;
    m->v = v; m->im = im; m->vref = vref;
    m->req = BUCK_TOC_REQ_NONE;
    if (__builtin_expect(buck_f2u(m->k.vref) != buck_f2u(vref), 0)) {   /* a new V_ref: finish forms them */
        ++b->toc.rej[BUCK_TOCR_K];
        return;
    }
    if (__builtin_expect(hr->sTimerxRegs[0].CNTxR > BUCK_TOC_START_LAST, 0)) { ++b->toc.rej[BUCK_TOCR_LATE]; return; }
    if (v < vref) buck_toc_go(BUCK_TOC_ON_OFF, hr, m, v, im);
    else          buck_toc_go(BUCK_TOC_OFF_ON, hr, m, v, im);
}

#endif
