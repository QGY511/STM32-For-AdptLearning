#ifndef BUCK_TOC_FRAME_H
#define BUCK_TOC_FRAME_H

/* The TOC frame: how a planned pulse pair becomes ONE long HRTIM period
 * (stage 2, 29 Oct 2026; user decisions 4 and 6, stage1/questions.txt Q2, Q3,
 * Q13). Integers only, hardware independent: the planner's FINAL stage calls
 * buck_toc_frame_encode() (a plan is READY only if its frame encodes), the
 * firmware writes the words, the host tests check them.
 *
 * Ticks are HRTIM fine ticks from the frame's start (the roll-over at which the
 * frame's preloads take effect). DT = BUCK_DT_TICKS. Model time (the planner's
 * spans, as in Buck_disturb_obs: dead time counted as off-time, the cycle starts
 * where the high side turns on) is register time minus DT.
 *
 *   ON-OFF  SET1R' = PER, RST1R' = CMP1 (as a normal period).
 *           CMP1' = DT + on, PER' = MPER' = on + off.
 *           HS on [DT, DT + on], off until PER' + DT = D = A (the next period starts HIGH).
 *   OFF-ON  SET1R' = CMP2, RST1R' = CMP1 (no PER set: the frame starts LOW without the
 *           CMP1 = 0 null-duty exception, and no reset on CMP3, which is ADC1's trigger:
 *           Q13). CMP2' = off, CMP1' = DT + off + on = D, PER' = CMP1' + T - resume:
 *           after D the normal OFF tail of the resume duty, then A at PER' + DT.
 *   Both    ADC1 (v_out, 2x TROVS): CMP3' = Dref - BUCK_VSAMPLE_LEAD_TICKS, CMP4' = CMP3' -
 *           BUCK_VSAMPLE_GAP_TICKS (Dref = PER' for ON-OFF, CMP1' for OFF-ON): the pair sits
 *           before D as the normal pair sits before the boundary (decision 4).
 *           ADC2 (i_L, 4x TROVS): MCMP1..4' in the last span (decision 4: the average over
 *           the last span), evenly at (2j+1)/8 of it when the span is at least 4 x
 *           BUCK_TOC_ADC2_GAP_TICKS, otherwise BUCK_TOC_ADC2_GAP_TICKS apart and ending
 *           half a gap before the span's end (stage2/questions.txt S2-Q1).
 *           ADC1R/ADC3R keep their sources (TAC3|TAC4, MC1..4): only compares move, so every
 *           frame still delivers exactly 2 + 4 triggers (TROVS pairing intact).
 * Constraints (all checked; a violation makes the plan BUCK_TOC_LIMIT, default H):
 *   on >= BUCK_TOC_ON_MIN_TICKS, off >= BUCK_TOC_OFF_MIN_TICKS (Q2: HRTIM minima),
 *   BUCK_TOC_PER_MIN_TICKS <= PER' <= BUCK_TOC_PER_MAX_TICKS (Q2, Q3),
 *   CMP4' >= BUCK_TOC_ADC1_FIRST (after the previous period's ADC1 conversion),
 *   MCMP1' >= BUCK_TOC_ADC2_FIRST (after the previous period's ADC2 conversion),
 *   MCMP4' <= PER' - BUCK_TOC_ADC2_END_GUARD, every compare in [0x60, PER').
 * The return to normal (all words of a normal period, resume CMP1) is written after the
 * frame has committed (Src/buck_toc.c). */

#include <stdint.h>
#include "buck_config.h"

enum { BUCK_TOC_FRAME_OK = 1, BUCK_TOC_FRAME_LIMIT = 0 };

typedef struct {
    uint32_t order;            /* BUCK_TOC_ON_OFF (0) / BUCK_TOC_OFF_ON (1) */
    uint32_t per;              /* PER' = MPER' */
    uint32_t cmp1, cmp2, cmp3, cmp4;
    uint32_t mcmp[4];          /* ADC2 triggers */
    uint32_t set_cmp2;         /* 0: SET1R' = PER; 1: SET1R' = CMP2 */
    uint32_t d_ref;            /* reference edge at D: ON-OFF PER' (HS rises DT later), OFF-ON CMP1' */
    uint32_t split_ref;        /* reference edge at the split: ON-OFF CMP1', OFF-ON CMP2' (HS on DT later) */
    uint32_t last_a, last_b;   /* the last span in register ticks (physical HS edges) */
    /* stage 2b: the ADC2 instants in model time (register tick - DT) split by span, for the cheap
     * model value of the measurement (buck_toc_frame_meas_w): n1 of them in the first span (model
     * time <= its length), the sums of their model ticks in the first and in the second span */
    uint32_t n1, sum1, sum2;
} buck_toc_frame_words_t;

#ifndef BUCK_INLINE
#define BUCK_INLINE static inline __attribute__((always_inline))
#endif

/* Frame of a pair (on, off ticks) of the given order and the resume CMP1 (OFF-ON tail). */
BUCK_INLINE uint32_t buck_toc_frame_encode(uint32_t order, uint32_t on, uint32_t off, uint32_t resume,
                                           buck_toc_frame_words_t *w)
{
    const uint32_t dt = BUCK_DT_TICKS;
    w->order = order;
    if (on < BUCK_TOC_ON_MIN_TICKS || off < BUCK_TOC_OFF_MIN_TICKS || on > BUCK_TOC_PER_MAX_TICKS ||
        off > BUCK_TOC_PER_MAX_TICKS || resume < BUCK_CMP_MIN || resume > BUCK_CMP_MAX)
        return BUCK_TOC_FRAME_LIMIT;
    if (order == 0U) {                                  /* ON-OFF */
        w->per = on + off;
        w->cmp1 = dt + on;
        w->cmp2 = BUCK_TOC_CMP2_IDLE;                   /* unused (SET1R' = PER) */
        w->set_cmp2 = 0U;
        w->d_ref = w->per;
        w->split_ref = w->cmp1;
        w->last_a = dt + on;
        w->last_b = dt + on + off;
    } else {                                            /* OFF-ON */
        w->cmp2 = off;
        w->cmp1 = dt + off + on;
        w->per = w->cmp1 + BUCK_HRTIM_PERIOD - resume;
        w->set_cmp2 = 1U;
        w->d_ref = w->cmp1;
        w->split_ref = w->cmp2;
        w->last_a = off + dt;
        w->last_b = w->cmp1;
    }
    if (w->per < BUCK_TOC_PER_MIN_TICKS || w->per > BUCK_TOC_PER_MAX_TICKS) return BUCK_TOC_FRAME_LIMIT;
    if (w->d_ref < BUCK_TOC_ADC1_FIRST + BUCK_VSAMPLE_GAP_TICKS + BUCK_VSAMPLE_LEAD_TICKS) return BUCK_TOC_FRAME_LIMIT;
    w->cmp3 = w->d_ref - BUCK_VSAMPLE_LEAD_TICKS;
    w->cmp4 = w->cmp3 - BUCK_VSAMPLE_GAP_TICKS;
    /* ADC2 in the last span */
    const uint32_t span = w->last_b - w->last_a;
    uint32_t sp = span / 4U, t3;
    if (sp >= BUCK_TOC_ADC2_GAP_TICKS) {
        t3 = w->last_a + sp / 2U + 3U * sp;
    } else {
        sp = BUCK_TOC_ADC2_GAP_TICKS;
        t3 = w->last_b - sp / 2U;
    }
    const uint32_t t3max = w->per - BUCK_TOC_ADC2_END_GUARD;
    if (t3 > t3max) t3 = t3max;
    if (t3 < 3U * sp + BUCK_TOC_ADC2_FIRST) return BUCK_TOC_FRAME_LIMIT;
    for (uint32_t j = 0U; j < 4U; ++j) w->mcmp[j] = t3 - (3U - j) * sp;
    {
        const uint32_t first = (order == 0U) ? on : off;
        uint32_t n1 = 0U, s1 = 0U, s2 = 0U;
        for (uint32_t j = 0U; j < 4U; ++j) {
            const uint32_t tm = w->mcmp[j] - dt;       /* mcmp >= 1120 > DT */
            if (tm <= first) { ++n1; s1 += tm; } else { s2 += tm; }
        }
        w->n1 = n1; w->sum1 = s1; w->sum2 = s2;
    }
    /* every compare in [0x60, PER') (cmp2 of ON-OFF is the idle value beyond PER') */
    if (w->cmp1 < 0x60U || w->cmp1 >= w->per || w->cmp3 >= w->per || w->cmp4 < 0x60U ||
        (w->set_cmp2 && (w->cmp2 < 0x60U || w->cmp2 >= w->per)))
        return BUCK_TOC_FRAME_LIMIT;
    return BUCK_TOC_FRAME_OK;
}

#endif
