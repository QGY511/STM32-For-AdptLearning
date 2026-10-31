#ifndef BUCK_H
#define BUCK_H

#include <stdint.h>
#include "buck_ctrl.h"

enum { BUCK_STOP = 0U, BUCK_RUN = 1U };

/* Boot-time bus benchmark (buck_bench(), Src/buck.c): g_buck.bench[i] = extra
 * CPU clocks per access over the same access pattern on SRAM1, measured with
 * DWT once at boot, before the loop starts (minimum of 8 runs of 16 accesses). */
enum {
    BUCK_BENCH_RD_ADC_CSR = 0,  /* read ADC12_CSR (the pacing poll)                     */
    BUCK_BENCH_RD_ADC1_DR,      /* read ADC1_DR                                         */
    BUCK_BENCH_RD_ADC2_DR,      /* read ADC2_DR                                         */
    BUCK_BENCH_RD_DWT_CYCCNT,   /* read DWT_CYCCNT                                      */
    BUCK_BENCH_RD_HRTIM_CNT,    /* read HRTIM Timer A CNTR (APB2)                       */
    BUCK_BENCH_WR_HRTIM_CMP1,   /* back-to-back writes to Timer A CMP1R: bus occupancy  */
    BUCK_BENCH_WR_DAC_DHR,      /* back-to-back writes to DAC1_DHR12R1                  */
    BUCK_BENCH_WR_GPIOB_BSRR,   /* back-to-back writes to GPIOB_BSRR                    */
    BUCK_BENCH_WRLD_HRTIM_CMP1, /* write CMP1R, then load SRAM at once: the load's stall */
    BUCK_BENCH_WRLD_DAC_DHR,    /* write DHR12R1, then load SRAM at once                */
    BUCK_BENCH_WRLD_GPIOB_BSRR, /* write BSRR, then load SRAM at once                   */
    BUCK_BENCH_WRLIT_HRTIM_CMP1,/* write CMP1R, then load CCM (a literal-pool load)     */
    BUCK_BENCH_RD_CCM,          /* load CCM SRAM through D-code while executing from CCM */
    BUCK_BENCH_N
};
enum { BUCK_STOPPED = 0U, BUCK_RUNNING = 1U, BUCK_FAULTED = 2U };
enum {
    BUCK_FAULT_CLOCK = 1U,      /* HSI/PLL/voltage-scaling start-up timeout */
    BUCK_FAULT_ADC_START = 2U,  /* ADC calibration/enable timeout */
    BUCK_FAULT_DLL = 3U,        /* HRTIM DLL calibration timeout */
    BUCK_FAULT_ADC = 4U,        /* running: ADC overrun or no current result this cycle; or no sample at all */
    BUCK_FAULT_DEADLINE = 5U,   /* running: the CMP1 write would miss the period boundary */
    BUCK_FAULT_DAC = 6U,        /* DAC enable timeout */
    BUCK_FAULT_CPU = 8U,        /* NMI / HardFault / MemManage / BusFault / UsageFault */
    BUCK_FAULT_TOC = 9U         /* TOC frame: a hardware check or a frame write window failed (g_buck.toc.chk_*;
                                   Src/buck_toc.c) */
};

/* ---- Feed-forward TOC (stage 4, 30 Oct 2026; Src/buck_toc.c, Inc/buck_toc.h): appended to g_buck,
 * every field before g_buck.toc keeps its offset. All 0 at reset except the defaults below. */
enum { BUCK_TOCS_IDLE = 0U, BUCK_TOCS_FRAME = 1U, BUCK_TOCS_END = 2U, BUCK_TOCS_FIRST = 3U };
enum {                          /* g_buck.toc.rej[] index: a score edge without a (solved) frame */
    BUCK_TOCR_LATE = 0U,        /* the start came after BUCK_TOC_START_LAST or its update's guard missed (the
                                   period's first A0 trigger): no frame */
    BUCK_TOCR_K,                /* V_ref changed since the constants were formed (they are formed then: the next
                                   edge can start) */
    BUCK_TOCR_NONE,             /* the frame started (feed-forward) but the L-C pair has no solution: the
                                   first span ended as soon as possible, the second lands the current */
    BUCK_TOCR_N
};
typedef struct {
    /* tunables, written by the debugger */
    uint32_t toc_enable;        /* 1: a rising score edge (g 0 -> 1) in a HOLD pass starts a TOC (taken up at the
                                   next telemetry pass, every 64 passes) */
    uint32_t learn_enable;      /* 1: the Jacobian step on g and the equilibrium correction b after each frame */
    float g[2][2];              /* timing gains, g[order] = {g_on, g_off} (order 0 ON-OFF, 1 OFF-ON); writable,
                                   a value outside [BUCK_TOC_G_MIN, BUCK_TOC_G_MAX] is used as 1 */
    float b[2];                 /* equilibrium correction by order, A (stage 4b): I_eq = the edge's estimate + b;
                                   writable, outside [-BUCK_TOC_B_MAX_A, BUCK_TOC_B_MAX_A] used as 0 */
    /* counts */
    uint32_t state;             /* BUCK_TOCS_*: where the last episode is / ended */
    uint32_t edges;             /* score edges that reached the TOC code */
    uint32_t frames;            /* frames run to their end */
    uint32_t rej[BUCK_TOCR_N];
    uint32_t learn_n;           /* Jacobian steps taken */
    uint32_t learn_skip;        /* frames whose residuals were beyond the SKIP bounds (or learning off) */
    uint32_t b_n;               /* equilibrium corrections taken */
    uint32_t learn_low;         /* frames not learned from: the first period's mean current below LEARN_IMIN */
    uint32_t learn_late;        /* Jacobian steps dropped for lack of time (none since 31 Oct) */
    /* the last frame */
    uint32_t order;             /* 0 ON-OFF, 1 OFF-ON */
    uint32_t sat;               /* BUCK_TOC_SAT_* */
    uint32_t c_start;           /* Timer A counter at the first software update (the start), ticks */
    uint32_t c_swu2;            /* ... at the second (the solved frame), ticks */
    uint32_t t1_ticks, t2_ticks;   /* the spans as applied (first, second; physical), ticks */
    uint32_t per_ticks;         /* PER' of the frame */
    uint32_t hold_cmp;          /* CMP1 of the held duty (P1 and P2) */
    float v0, i0, ieq, a;       /* solve: v_out, measured average, I_eq, i0 - I_eq */
    float t1_model, t2_model;   /* the solve's spans before gains and clamps, ticks */
    float v_mid;                /* the period's own A0 pair, converted inside the frame (dropped), V */
    float v_end, z_v;           /* the sample at the frame's end and z_v = v_end - V_ref (V) */
    float v_1;                  /* the sample after the first period, V */
    float i_first;              /* the first period's measured mean current i_P1: the voltage PI's new integrator (A) */
    float z_i;                  /* the landing i_P1 - I_eq (A): the timing's current residual */
    float i_load;               /* the load over the first period, i_P1 - (C/T)(v_1 - v_end) (A) */
    float e_b;                  /* the equilibrium residual I_load - I_eq (A): b's */
    /* hardware checks (RM0440 items not confirmed by the manual; README bench tests) */
    uint32_t chk_commit;        /* the frame's end transfer not seen (MUPD / UPD) */
    uint32_t chk_align;         /* master and Timer A counters drifted apart */
    int32_t align_ref, align_last;
    /* Fault 4 (31 Oct, diagnostics; written on the fault paths only): where the last one came from, frame or
     * not, and the ADC status there */
    uint32_t f4_site;           /* 0: none, or the loop's own sample timeout (buck_fatal from a pass or the arming
                                   wait: the CPU then spins in buck_fatal); 1 a pass of the loop: a bad sample
                                   (run_exit); 2 T, the frame-end sample, bad; 3 a pass the TOC code runs (the
                                   restart pass, a learning pass) bad (31 Oct: those passes went, site 3 is a
                                   STOP after P1's sample); 4 the TOC code's sample timeout (toc_wait); 5 the
                                   period's own A0 result not converted inside the frame (buck_toc_ret's wait);
                                   (31 Oct: T no longer faults on its sample, site 2 is a STOP there) */
    uint32_t f4_bad;            /* sites 1-3, the check's bad bits: 0x10 ADC1 overrun, 0x100000 ADC2 overrun,
                                   0x40000 ADC2 without a result (ADC12_COMMON->CSR OVR_MST, OVR_SLV, EOC_SLV
                                   missing) */
    uint32_t f4_csr;            /* ADC12_COMMON->CSR there (sites 1-3 after the data reads: EOC bits cleared) */
    uint32_t f4_cnt;            /* Timer A counter there */
    /* Board timing of the last frame (31 Oct; Timer A counter, ticks, 8960 per period; the frame's own counter
     * runs on past 8960 to PER'): the first update's guard, the read after the second update's wait, the second
     * update's guard, the return words' guard (the frame's counter); T's CMP1 write (P1's counter); c_first and
     * c_pass unused since 31 Oct (no PI pass in the TOC code); the counter when the loop takes over (c_back, the
     * period unknown) and ADC12_COMMON->CSR there (EOC_MST 0x4 set: its sample is already waiting) */
    uint32_t c_swu1, c2, c_ret, c_t, c_first, c_pass, c_back, csr_back;
    uint32_t c_tw, c_rw;        /* the Timer A counter when T's sample (P1) and P1's sample (P2) were taken
                                   (their waits returned) */
    /* buck_fatal (any fault through it, e.g. a sample timeout, fault 4 with f4_site 0; 31 Oct diagnostics): its
     * caller (return address: map it to a function with the .map / disassembly), the fault code before it (a
     * trip the stopped loop then timed out after), and the registers when it stopped: Timer A PER and counter,
     * the master's PER and counter, ADC1 CR (ADSTART 0x4) and ISR, ADC2 ISR */
    uint32_t fatal_lr, fatal_prev, fatal_ta_per, fatal_ta_cnt, fatal_mper, fatal_mcnt;
    uint32_t fatal_adc1_cr, fatal_adc1_isr, fatal_adc2_isr;
    /* The samples' arrival in the passes the TOC code runs after a frame (31 Oct diagnostics; index 0 is T, 1 the
     * restart pass, then the learning and the extra passes; pr_n of them): the Timer A counter when the wait began,
     * when ADC2's result was first seen during it (0xFFFF: already there at the start), when ADC1's came, and
     * ADC12_COMMON->CSR then (OVR_MST 0x10, OVR_SLV 0x100000, EOC_SLV 0x40000) */
    uint32_t pr_n;
    uint32_t pr_c0[6], pr_c2[6], pr_c1[6], pr_st[6];
    /* The event log of the last frame (31 Oct diagnostics; README "Enabling" 8, tools/decode_events.py): from the
     * return words' guard (inside the frame) through the passes the TOC code runs, ev_n entries, each
     * code << 23 | (CPU cycles since the guard, 12 bits) << 11 | Timer A counter / 32. code: 0x100 a wait began,
     * 0x80 a sample read with the duty held (toc_keep), 0x10 T's CMP1 write skipped (late), 0x08 T's
     * sample overwritten; with the ADC state there: 0x01 ADC1's result, 0x02 ADC2's, 0x20 / 0x40 their overruns.
     * ev_cyc0: CYCCNT at the guard */
    uint32_t ev_n, ev_cyc0;
    uint32_t ev[32];
    /* After frames (31 Oct; cumulative): samples read with the held duty kept beyond P1's (toc_keep: the TOC code
     * still worked; the loop's PIs restart a period later for each), T's CMP1 writes skipped (outside their
     * window), T's samples overwritten (no learning from those frames) */
    uint32_t drop_n, t_skip, t_lost;
} buck_toc_t;

/* Everything the 600 kHz control loop touches is in this one object.
 * Live Expressions:
 *   g_buck.command                  write 1 to start, 0 to stop
 *   g_buck.state, g_buck.fault
 *   g_buck.tune.*                   V_ref (counts), PI gains, alpha_I, S_th, A1 calibration, score a/b/c,
 *                                   load-current observer gains beta, gamma
 *   g_buck.out.*                    v, i_meas, i_avg, eps, i_load, i_ref, i_peak, vref, score, g, cmp
 *   g_buck.busy_max, g_buck.loops, g_buck.adc_miss
 *   g_buck.bench                    boot-time bus benchmark (BUCK_BENCH_* above)
 *   g_buck.trip_cnt                 fault 5: Timer A counter at the missed CMP1 write
 *   g_buck.arm_cnt, arm_cyc, first_cnt, first_cyc, trip_cyc   timing of the last start (below) */
typedef struct {
    uint32_t command;          /* BUCK_RUN / BUCK_STOP, written by the debugger */
    uint32_t state;            /* BUCK_STOPPED / BUCK_RUNNING / BUCK_FAULTED */
    uint32_t fault;            /* latched fault code (reset to clear) */
    buck_tune_t tune;          /* live tunables: re-read every 64 cycles */
    buck_out_t out;            /* results of a recent cycle: published every 64 cycles */
    buck_state_t st;           /* controller state: held in registers while running,
                                  written back here when the converter stops */
    uint32_t adc_v_raw;        /* last ADC1 result: sum of the cycle's two A0 samples (x 3.3/8192 = V) */
    uint32_t adc_i4_raw;       /* last A1 result: sum of the four samples of the cycle */
    uint32_t loops;            /* control cycles while running (600 000 per second; skips ahead by up to 31 at each start), every 64 */
    uint32_t busy_max;         /* worst CPU clocks of control work (from reading the samples to
                                  the telemetry writes), sampled every 64th pass while running;
                                  statically 137-142 since 28 Oct (both clock counters; 144-149
                                  with the boot benchmark's wait states). The window's second
                                  CYCCNT read now goes through the DWT base held in a register
                                  (no literal-pool load inside the window), so the board reads a
                                  few clocks less for the same work. 143-150 in the 25 Oct evening
                                  build (141-151 before the arming moved into run_loop(), 25 Oct;
                                  BUCK_Adpt_Learning: 151-160). The telemetry pass does about 35
                                  more (the writes), and every pass must average < 280 clocks:
                                  keep it below ~240 (write 0 to restart) */
    uint32_t adc_miss;         /* ADC overrun / missing-current-result events */
    uint32_t warmup;           /* conversions still to discard after start-up */
    uint32_t rst_cause;        /* RCC->CSR at boot */
    float bench[BUCK_BENCH_N]; /* boot-time bus benchmark, extra clocks per access (see above) */
    uint32_t trip_cnt;         /* fault 5: Timer A counter at the missed CMP1 write, ticks (8960 per
                                  period, 32 per CPU clock); below 2048 = the pass was more than a
                                  period late. g_buck.loops % 64 then says which pass it was. */
    /* Timing of the last start (user, 25 Oct, after the board's fault 5 on the first pass
     * after arming). All four are stored together, once per start, by the first pass after
     * arming: after its CMP1 write, or on its fault-5 trip, or on a STOP or bad sample before
     * its check. Not stored when that pass ends in buck_fatal() (fault 4 by the sample
     * timeout: no end of conversion for 840 polls; or fault 8), nor by a sample timeout while
     * waiting to arm: they then still hold the previous start's values, and nothing in g_buck
     * tells them apart. 0 until the first start. CYCCNT: CPU clocks (168 per us, wraps after
     * 25.6 s); each CYCCNT is read by the instruction just before its counter read. */
    uint32_t arm_cnt;          /* Timer A counter read by the arming check (accepted: 897..4095), ticks:
                                  where in the period arming happened (about 1800-2350 expected since
                                  28 Oct; 2100-2650 in the 25 Oct evening build) */
    uint32_t arm_cyc;          /* DWT CYCCNT at that read */
    uint32_t first_cnt;        /* Timer A counter read by the first pass's guarded CMP1 check, ticks.
                                  When the check fell in the period after arming (as it normally does),
                                  8448 - first_cnt = the first write's margin (negative: fault 5).
                                  Below 2048: the counter had wrapped, the check came more than a period
                                  late (fault 5, = trip_cnt); 8448 - first_cnt is then no margin.
                                  0xFFFFFFFF: that pass stopped (STOP or bad sample) before its check */
    uint32_t first_cyc;        /* DWT CYCCNT at that read (0xFFFFFFFF with first_cnt; first_cyc - arm_cyc
                                  then means nothing): first_cyc - arm_cyc = CPU clocks from the arming
                                  check to the first CMP1 check, = (first_cnt + n x 8960 - arm_cnt) / 32
                                  within a clock or so, n = the period starts between the two reads
                                  (1 when the check fell in the period after arming, as it normally does) */
    uint32_t trip_cyc;         /* fault 5: DWT CYCCNT read at the start of the trip branch (in every
                                  pass, the first pass after arming included, right after a literal
                                  load of DWT's base from CCM), 11 clocks static after the counter read
                                  in trip_cnt in the ordinary copies, 10 in the first pass (its counter
                                  read pipelines behind its CYCCNT read); 3 more with the counter read's
                                  wait, about 1 more for the literal's (since 28 Oct; 11-16 static
                                  before; in a BUCK_ADC_CSR_POLL = 0 build one branch has a register
                                  move between the literal load and the read); 0 until a fault 5 (it
                                  latches until reset, so the value stays) */
    buck_toc_t toc;            /* feed-forward TOC (stage 4, appended: see buck_toc_t) */
} buck_t;

extern buck_t g_buck;

void buck_init(void);
void buck_start(void);
void buck_run(void) __attribute__((noreturn));
void buck_fatal(uint32_t reason) __attribute__((noreturn));

#endif
