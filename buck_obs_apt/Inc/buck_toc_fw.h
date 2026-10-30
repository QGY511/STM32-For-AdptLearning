#ifndef BUCK_TOC_FW_H
#define BUCK_TOC_FW_H

/* Firmware side of the feed-forward TOC (stage 4, 30 Oct 2026; Inc/buck_toc_start.h, Src/buck_toc.c).
 * The control loop (Src/buck.c run_loop) keeps its state in registers. It talks to the TOC code
 * through the mailbox g_toc_mb (SRAM1) and calls it with BUCK_TOC_CALL, an inline-asm call that saves
 * and restores every caller-saved register itself (r0-r3, r12, lr, s0-s15), so GCC sees no clobbered
 * register and the loop's register allocation does not change around the call (no address of the
 * loop context escapes either).
 *
 * The sequence (one score edge; pass n reads the sample at the end of period n-1):
 *   n    run_pass: the PIs, CMP1 for n+1, the score. g rose and trig is set: the start
 *        (buck_toc_start_inline, Inc/buck_toc_start.h, inline in the pass: the save and restore of a
 *        call alone would take ~50 clocks of the period, README "Clock budget"): the first software
 *        update stretches period n into the frame (the A0 pair moved to its end, the first span's
 *        backstop), SST (ON-OFF) / SRT (OFF-ON): the first span runs; the L-C solve's first span;
 *        the second update (its end); the second span; the third update (the frame's end PER').
 *        The pass returns PASS_TOC (its observer, A2 and D4 are left to the TOC code).
 *        run_loop copies its state into the mailbox; buck_toc_finish(): the return words for the
 *        first normal period P1 (the held duty) into the preloads, the rest of pass n (observer,
 *        A2, D4), then
 *   T    the sample at the frame's end (read in P1): z_v; CMP1 for P2 = the held duty again; the
 *        observer's v^ set to it;
 *   1    the sample after P1 (read in P2): the voltage PI's integrator = P1's measured mean current
 *        (user: no reference learner), the current PI's = the held duty, then an ordinary pass (CMP1
 *        for P3, score, observer, A2, D4); z_i = (C/T) (v_1 - v_T); the Jacobian step on the gains;
 *   then the normal passes, with run_loop's state taken back from the mailbox. */

#include <stdint.h>
#include "buck_ctrl.h"
#include "buck_toc.h"

#ifndef BUCK_TOC_ENABLE_DEFAULT
#define BUCK_TOC_ENABLE_DEFAULT   0U    /* g_buck.toc.toc_enable at reset */
#endif
#ifndef BUCK_LEARN_ENABLE_DEFAULT
#define BUCK_LEARN_ENABLE_DEFAULT 0U    /* g_buck.toc.learn_enable at reset */
#endif
/* Hardware checks of the RM0440 items the manual does not confirm (README, bench tests): 1 = trip
 * (fault 9, gates off) on a failed check, 0 = count only (g_buck.toc.chk_*). */
#ifndef BUCK_TOC_CHECK_TRIP
#define BUCK_TOC_CHECK_TRIP     1
#endif
#define BUCK_TOC_ALIGN_TOL      64      /* ticks: allowed change of (Timer A - master) counter read back to back */
#define BUCK_TOC_F_EARLIEST     1024U   /* a sample after the frame: a counter below this means a period was lost */

enum { BUCK_TOC_RET_STATE = 0U, BUCK_TOC_RET_EXIT = 1U };
enum { BUCK_TOC_REQ_NONE = 0U, BUCK_TOC_REQ_FRAME = 1U, BUCK_TOC_REQ_FAIL = 2U };

/* The frame in flight, from the start to buck_toc_finish(). */
typedef struct {
    uint32_t order, sat, ok;
    uint32_t c_s;               /* counter at the SST/SRT */
    uint32_t per;               /* PER' */
    uint32_t t1, t2;            /* applied physical spans, ticks */
    uint32_t fail_cnt;          /* REQ_FAIL: the counter at the missed guard */
    uint32_t hold_cmp;          /* CMP1 of the held duty (with DT; P1 after OFF-ON without) */
    float d_hold;               /* the held duty */
    float v_mid;                /* the period's own A0 pair, converted inside the frame (dropped; telemetry) */
    buck_toc_sol_t sol;
    float g1, g2;               /* the gains of the spans (in use for this frame) */
} buck_toc_frame_t;

typedef struct {
    /* written by run_pass */
    uint32_t trig;              /* every pass: (g ^ 1) & en. A HOLD pass with g & trig starts a frame */
    uint32_t en;                /* telemetry pass: g_buck.toc.toc_enable and the hold-off over */
    float v_prev;               /* every pass: its v_out (the next pass's v_out,prev) */
    float v, im, vref;          /* the dispatching pass: v_out, measured average, V_ref */
    uint32_t req;               /* the start: BUCK_TOC_REQ_* */
    /* written by run_loop (RUN_TOC), read back after buck_toc_finish */
    buck_state_t s;
    buck_obs_t obs;
    buck_obs_k_t ok;
    float kp_v, kp_i, ki_t_v, ki_t_i, isense_zero, isense_gain, s_th;
    buck_score_k_t sk;
    uint32_t loops;
    uint32_t ret;               /* buck_toc_finish: BUCK_TOC_RET_STATE (take s, obs, loops back) / BUCK_TOC_RET_EXIT */
    uint32_t hold_until;        /* en stays 0 until loops reaches this */
    /* the TOC code's own (the start reads them: one base register for all) */
    buck_toc_k_t k;             /* the solve's constants, for V_ref = k.vref (buck_toc_prime, buck_toc_finish) */
    float g1[2], g2[2];         /* the gains in use by order and span (g_buck.toc.g checked) */
    buck_toc_frame_t f;
} buck_toc_mb_t;

extern buck_toc_mb_t g_toc_mb;

void buck_toc_finish(void);     /* after the start: the return words, the rest of that pass and, with a
                                   frame, its two samples */
void buck_toc_prime(void);      /* run_loop's entry: the constants for g_toc_mb.vref, the gains */
void buck_toc_start_late(void); /* the first update's guard missed: the normal words back, a LATE reject */
void buck_toc_ret(void);        /* after the third update: the return words, the period's own A0 result read */

/* The call: every caller-saved register saved and restored here (8-byte aligned: 6 + 16 words). */
#define BUCK_TOC_CALL(fn) \
    __asm__ volatile("push {r0-r3, r12, lr}\n\tvpush {s0-s15}\n\tbl " #fn "\n\tvpop {s0-s15}\n\tpop {r0-r3, r12, lr}" \
                     ::: "memory", "cc")

/* Guarded store: the Timer A counter (HRTIM base hr) is read and the store happens only if early <= cnt <
 * early + win; otherwise branch to missed (make check, rule 5c: this exact form). */
#define TOC_GUARDB(hr_, val, addr, early, win, cnt, missed)                                \
    do {                                                                                   \
        uint32_t late_;                                                                    \
        __asm__ goto("ldr    %[c], [%[hr], #%c[oc]]\n\t"                                   \
                     "sub    %[l], %[c], %[e]\n\t"                                         \
                     "cmp    %[l], %[w]\n\t"                                               \
                     "it     lo\n\t"                                                       \
                     "strlo  %[v], [%[a]]\n\t"                                             \
                     "bhs    %l[" #missed "]"                                              \
                     : [c] "=&r"(cnt), [l] "=&r"(late_)                                    \
                     : [hr] "r"(hr_), [oc] "i"(offsetof(HRTIM_TypeDef, sTimerxRegs[0].CNTxR)), \
                       [e] "r"(early), [w] "r"(win), [v] "r"(val), [a] "r"(addr)             \
                     : "memory", "cc" : missed);                                           \
        (void)late_;                                                                       \
    } while (0)
#define TOC_GUARD(val, addr, early, win, cnt, missed) TOC_GUARDB(HRTIM1, val, addr, early, win, cnt, missed)

#endif
