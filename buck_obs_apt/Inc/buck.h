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
    uint32_t learn_enable;      /* 1: the Jacobian step on g after each frame */
    float g[2][2];              /* timing gains, g[order] = {g_on, g_off} (order 0 ON-OFF, 1 OFF-ON); writable,
                                   a value outside [BUCK_TOC_G_MIN, BUCK_TOC_G_MAX] is used as 1 */
    /* counts */
    uint32_t state;             /* BUCK_TOCS_*: where the last episode is / ended */
    uint32_t edges;             /* score edges that reached the TOC code */
    uint32_t frames;            /* frames run to their end */
    uint32_t rej[BUCK_TOCR_N];
    uint32_t learn_n;           /* Jacobian steps taken */
    uint32_t learn_skip;        /* frames whose residuals were beyond the SKIP bounds (or learning off) */
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
    float v_1, z_i;             /* the sample after the first period and z_i = (C/T)(v_1 - v_end) (A) */
    float i_first;              /* the first period's measured mean current: the voltage PI's new integrator (A) */
    /* hardware checks (RM0440 items not confirmed by the manual; README bench tests) */
    uint32_t chk_commit;        /* the frame's end transfer not seen (MUPD / UPD) */
    uint32_t chk_align;         /* master and Timer A counters drifted apart */
    int32_t align_ref, align_last;
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
