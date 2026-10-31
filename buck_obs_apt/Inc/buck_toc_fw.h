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
 *        call alone would take ~50 clocks of the period, README "Clock budget"): SST (ON-OFF) / SRT
 *        (OFF-ON): the first span runs; the L-C solve's first span; the first software update (the
 *        span's end, period n stretched into the frame); the second span and the held duty; the
 *        second update (the frame's end PER', the A0 pair moved to it; OFF-ON stretched to its
 *        minimum: the tail), and right after it buck_toc_ret() (BUCK_TOC_CALL_CORE): the return words
 *        for the first normal period P1 (the held duty) into the preloads. Then the pass's own
 *        observer step, A2 and D4 (31 Oct: in the edge pass itself, Src/buck.c), and it returns
 *        PASS_TOC. run_loop copies its state into the mailbox; buck_toc_finish():
 *   T    the sample at the frame's end (read in P1): z_v; CMP1 for P2 = the held duty again; the
 *        observer's v^ set to it; the checks of debugger writes to g and b;
 *   1    the sample after P1 (read in P2): the voltage PI's integrator = P1's measured mean current
 *        (user: no reference learner), the current PI's = the held duty, then an ordinary pass (CMP1
 *        for P3, score, observer, A2, D4); the load over P1, I_load = i_P1 - (C/T)(v_1 - v_T): the
 *        equilibrium correction b at once; the landing z_i = i_P1 - I_eq and z_v: the Jacobian step on
 *        the gains in two parts, in the spare tails of the next one or two ordinary passes, which the TOC
 *        code runs itself (toc_pass; the restart pass's tail is too short for the step);
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
/* The learning after a frame (stage 4b, Src/buck_toc.c): a part of the Jacobian step (~110 clocks static) starts
 * only with the counter below this, so that it ends by ~2048 ticks into the next period; the next pass (the TOC
 * code's own, toc_pass: 90 clocks from its sample to its CMP1 write) then still writes CMP1 ~100 clocks before
 * the deadline. Otherwise the next ordinary pass first, at most LEARN_PASSES of them. */
#define BUCK_TOC_LEARN_PART_LAST (BUCK_HRTIM_PERIOD + 2048U - 32U * 110U)
/* ... and not before this: this period's own sample (its end of conversion ~1150 ticks in) comes first, so a part
 * never delays the pass of a sample that is about to arrive. */
#define BUCK_TOC_LEARN_PART_FIRST 2048U
#define BUCK_TOC_LEARN_PASSES   4U

enum { BUCK_TOC_RET_STATE = 0U, BUCK_TOC_RET_EXIT = 1U };
enum { BUCK_TOC_REQ_NONE = 0U, BUCK_TOC_REQ_FRAME = 1U, BUCK_TOC_REQ_FAIL = 2U };

/* The frame in flight, from the start to buck_toc_finish(). */
typedef struct {
    uint32_t order, sat, ok;
    uint32_t c_s;               /* counter at the SST/SRT */
    uint32_t per;               /* PER' */
    uint32_t t1, t2;            /* applied physical spans, ticks */
    uint32_t fail_cnt;          /* REQ_FAIL: the counter at the missed guard */
    uint32_t hold_cmp;          /* CMP1 of the held duty (with DT: P2) */
    uint32_t ret_cmp1;          /* P1's CMP1 (the return words): hold_cmp, after OFF-ON without DT (with the
                                   tail with it) */
    uint32_t t_end;             /* SAT_TAIL: the ON span's end (Timer A CMP1, the frame's counter; with TAIL_NOW
                                   the counter read at which it was ended) */
    uint32_t t_chk;             /* the tail's compare to check after the second update (0: none) */
    uint32_t c_swu1, c2, c_swu2, c_ret;   /* Timer A counter at the first update's guard, the read after the
                                   wait, the second update's guard, the return words' guard (board timing) */
    float d_hold;               /* the held duty */
    uint32_t v_mid_code;        /* the period's own A0 pair, converted inside the frame (dropped; telemetry): its
                                   ADC1 code (buck_toc_ret has no floating point: BUCK_TOC_CALL_CORE) */
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
    float b[2];                 /* the equilibrium corrections in use by order (g_buck.toc.b checked) */
    buck_toc_frame_t f;
} buck_toc_mb_t;

extern buck_toc_mb_t g_toc_mb;

void buck_toc_finish(void);     /* after the start: the record, that pass's bookkeeping and, with a frame,
                                   its two samples */
void buck_toc_prime(void);      /* run_loop's entry: the constants for g_toc_mb.vref, the gains */
void buck_toc_start_late(void); /* the first update's guard missed: the normal words back, a LATE reject */
void buck_toc_ret(void);        /* after the second update: the return words, the period's own A0 result read
                                   (BUCK_TOC_CALL_CORE: no floating-point register, make check rule 5d) */

/* The call: every caller-saved register saved and restored here (8-byte aligned: 6 + 16 words). */
#define BUCK_TOC_CALL(fn) \
    __asm__ volatile("push {r0-r3, r12, lr}\n\tvpush {s0-s15}\n\tbl " #fn "\n\tvpop {s0-s15}\n\tpop {r0-r3, r12, lr}" \
                     ::: "memory", "cc")

/* The call of a function that uses no floating-point register (buck_toc_ret; make check rule 5d checks it):
 * the caller-saved core registers only, 17 clocks less than BUCK_TOC_CALL before the call (31 Oct: on the
 * path from the second update to the return words' guard). */
#define BUCK_TOC_CALL_CORE(fn) \
    __asm__ volatile("push {r0-r3, r12, lr}\n\tbl " #fn "\n\tpop {r0-r3, r12, lr}" ::: "memory", "cc")

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
