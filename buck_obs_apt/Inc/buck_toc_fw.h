#ifndef BUCK_TOC_FW_H
#define BUCK_TOC_FW_H

/* Firmware side of the TOC and the learning (stage 2, 29 Oct 2026; stage 2b, 30 Oct 2026; stage 3, 30 Oct 2026;
 * Src/buck_toc.c).
 * The control loop (Src/buck.c run_loop) keeps its state in registers. It talks to
 * the TOC code only through the mailbox g_toc_mb (SRAM1) and calls it with
 * BUCK_TOC_CALL, an inline-asm call that saves and restores every caller-saved
 * register itself (r0-r3, r12, lr, s0-s15), so GCC sees no clobbered register and
 * the loop's register allocation does not have to change around the call (no
 * address of the loop context escapes either). */

#include <stdint.h>
#include "buck_ctrl.h"

/* Dedicated planning periods (user Q1): the frame starts at the boundary after the
 * BUCK_TOC_PLAN_PERIODS-th sample read after the request pass R, i.e. S0 is
 * BUCK_TOC_PLAN_PERIODS + 1 periods after R's period; x0 is predicted over those
 * periods with the held duty. Stage 2b (30 Oct 2026, user decisions 1 and 3): with the
 * cheaper planner the work from R's sample to "ready to arm" (the request pass and its call,
 * x0, the planner, the frame's model, the frame words, the samples' service) measures ~7.9
 * periods in the emulator (1.0 x the clocks: ready after the 7th sample in every TOC scenario,
 * plan_ready = 7), plus one spare period (decision 3) -> 9 (stage 2: 10). 8 leaves ~0.1 period
 * (all TOC scenarios still pass at 1.0 and 1.05; stage2b report). A plan not ready when the last
 * of them is read is rejected LATE (no fault). README "Clock budget"; stage2b questions S2B-Q2. */
#ifndef BUCK_TOC_PLAN_PERIODS
#define BUCK_TOC_PLAN_PERIODS   9U
#endif
/* Dedicated learning periods (stage 2b, user decision 2): after D (and only with learning on)
 * the duty stays held (the D pass's restart CMP1 repeats), the PIs and the estimator stay
 * frozen and the observer is held as in the planning periods (one observer step at B, the
 * first sample after D, for m_B; the catch-up step at the end); kappa, G and, after B, q are
 * computed there, one stage between two polls. At most this many samples; the block ends as
 * soon as the jobs are done (emulator, 1.0: kappa and G ~5 periods; q ~8 more, so a frame whose
 * q passes its gates holds the duty ~14 periods, one whose q is rejected ~6). A job not done by
 * then is dropped (g_buck.toc.learn_late). README "Clock budget"; stage2b questions S2B-Q4. */
#ifndef BUCK_TOC_LEARN_PERIODS
#define BUCK_TOC_LEARN_PERIODS  16U
#endif
#define BUCK_TOC_QUIET_PASSES   24U
#ifndef BUCK_TOC_ENABLE_DEFAULT
#define BUCK_TOC_ENABLE_DEFAULT   0U    /* default J: g_buck.toc.toc_enable at reset (tests build with 1) */
#endif
#ifndef BUCK_LEARN_ENABLE_DEFAULT
#define BUCK_LEARN_ENABLE_DEFAULT 0U    /* default J: g_buck.toc.learn_enable at reset */
#endif     /* default A: no TOC in the first 24 passes after arming */
/* run_loop's trigger word: a normal HOLD pass returns PASS_TOC when (D4 word & trig) != 0.
 * D4 word = D4_PIN (g = 1) or D4_PIN << 16 (g = 0); trig = d4 >> 16 after every such pass,
 * i.e. D4_PIN after g = 0 (the next g = 1 is a rising edge) and 0 after g = 1;
 * BUCK_TOC_FORCE makes the next normal HOLD pass call the handler whatever g is (learning). */
#define BUCK_TOC_FORCE          ((1U << 5) | (1U << 21))
#define BUCK_TOC_FORCED_BIT     (1U << 21)      /* in the word before the pass: it was forced */
/* bit 0 of the trigger word: learning on (DeltaI in the pass; set from g_buck.toc.learn_enable by the
 * TOC code at each call and at the run_loop entry, so it takes effect at the next score edge/handler) */
#define BUCK_TOC_LEARN_BIT      1U
/* run_loop's forced-pass path (stage 2 learning slices; Src/buck.c RUN_TOC) is no longer used
 * since stage 2b (the jobs run in the learning periods): g_toc_mb.forced stays 0. The path and
 * this constant stay so that run_loop's code is unchanged (stage2b questions S2B-Q6). */
#define BUCK_TOC_SLICE_LAST     (BUCK_HRTIM_PERIOD + 32U * 36U - 32U * 130U)
/* Sample service (planning and learning periods): a sample read with the Timer A counter
 * outside [BUCK_TOC_SVC_EARLY, BUCK_TOC_SVC_LATE) was read late (its end of conversion is
 * ~1152 ticks into its period; below EARLY the counter has passed the next boundary, at or
 * above LATE less than ~50 clocks remain before the next current sample's conversion ends,
 * 1056 ticks into the next period: an overrun). The episode then stops: planning -> LATE
 * reject, learning -> the remaining jobs dropped (stage 2b, user decision 3: an overrun becomes
 * a reject, not fault 4). Every stretch of code between two polls is at most ~180 clocks
 * (emulator; the planner's and the learning jobs' parts, README "Clock budget"). */
#define BUCK_TOC_SVC_EARLY      1024U
#define BUCK_TOC_SVC_LATE       8448U
/* Arming: the two guarded writes start only if the counter is below this (with the flag clears
 * they take ~20 clocks, emulator); else LATE reject. A missed arming guard is a LATE reject as well
 * (toc_arm_late). */
#define BUCK_TOC_ARM_LAST       (BUCK_WRITE_DEADLINE - 32U * 24U)
/* Hardware checks of the RM0440 items the manual does not confirm: 1 = trip (fault 9, gates off)
 * on a failed check, 0 = count only (g_buck.toc.chk_*). */
#ifndef BUCK_TOC_CHECK_TRIP
#define BUCK_TOC_CHECK_TRIP     1
#endif
#define BUCK_TOC_ALIGN_TOL      64      /* ticks: allowed change of (Timer A - master) counter read back to back */
#define BUCK_TOC_R_LATE_TICKS   4800U   /* a request is refused if the next sample has waited past this counter */
#define BUCK_TOC_DI_HOLD        96U     /* passes without DeltaI after a frame (the late passes catch up) */
#define BUCK_TOC_D_HOLD_TICKS   2048U   /* after D (no learning): B's sample waiting and the counter past this ->
                                           B read by the TOC code, the duty held one more period (stage 2b) */
#define BUCK_TOC_F_EARLIEST     1024U   /* pass F's guard: a counter below this means the frame has already ended */
#define BUCK_KAPPA_INIT         0.0f    /* SPEC "existing_initial_value": none in the project (stage2 S2-Q9) */

enum { BUCK_TOC_RET_STATE = 0U, BUCK_TOC_RET_EXIT = 1U };

typedef struct {
    /* from run_loop at every PASS_TOC (event) */
    float v, im, ih;            /* this pass's v_out[k+1], raw measured current, observer i^ (m) */
    uint32_t d4;                /* its D4 word (g) */
    uint32_t loops;
    /* from run_loop when g = 1 (a request is possible) */
    buck_state_t s;
    buck_obs_t obs;
    buck_obs_k_t ok;
    buck_tune_t t;              /* only isense_zero and isense_gain are copied (and used) */
    /* results */
    uint32_t req;               /* event: 1 = request accepted, run the episode */
    uint32_t ret;               /* episode: BUCK_TOC_RET_STATE (copy s, obs back) / BUCK_TOC_RET_EXIT */
    uint32_t trig;              /* the trigger word of the normal HOLD passes (Src/buck.c run_pass) */
    uint32_t loops0;            /* loops at the run_loop entry (quiet passes) */
    uint32_t forced;            /* the TOC code forces the normal HOLD passes (learning pending) */
    uint32_t cap_b;             /* run_loop keeps the next forced pass's v, m and loops (B) */
    uint32_t di_hold;           /* DeltaI stays off until loops reaches this (after a frame) */
    uint32_t g_last;            /* g at the handler's last call (a forced pass with g = 1 is an edge only if 0) */
    float v_b, m_b;
    uint32_t loops_b;
    uint32_t run;               /* incremented at each run_loop entry */
} buck_toc_mb_t;

extern buck_toc_mb_t g_toc_mb;

void buck_toc_event(void);      /* after a normal HOLD pass that returned PASS_TOC; runs the episode
                                   (Src/buck_toc.c) when it accepts a request (then req = 1) */

/* The call: every caller-saved register saved and restored here (8-byte aligned: 6 + 16 words). */
#define BUCK_TOC_CALL(fn) \
    __asm__ volatile("push {r0-r3, r12, lr}\n\tvpush {s0-s15}\n\tbl " #fn "\n\tvpop {s0-s15}\n\tpop {r0-r3, r12, lr}" \
                     ::: "memory", "cc")

#endif
