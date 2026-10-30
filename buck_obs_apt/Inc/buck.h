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
    BUCK_FAULT_TOC = 9U         /* TOC frame: a hardware check failed (g_buck.toc.chk_*; Src/buck_toc.c) */
};

/* ---- TOC and learning (buck_obs_apt stage 2, 29 Oct 2026): APPENDED to g_buck
 * (default L): every field before g_buck.toc keeps its offset. Src/buck_toc.c,
 * README "TOC and learning". All 0 at reset except the coefficients' defaults. */
enum {
    BUCK_TOCS_IDLE = 0U, BUCK_TOCS_PLAN = 1U, BUCK_TOCS_WAIT = 2U, BUCK_TOCS_FRAME = 3U, BUCK_TOCS_D = 4U,
    BUCK_TOCS_LEARN = 5U        /* stage 2b: the learning periods after D */
};
enum {                          /* g_buck.toc.rej[] index: why a request made no frame */
    BUCK_TOCR_NONE = 0U,        /* no verified pair */
    BUCK_TOCR_LIMIT,            /* outside the frame limits (default H) */
    BUCK_TOCR_ALREADY,          /* already at the target */
    BUCK_TOCR_INVALID,          /* non-finite input or intermediate */
    BUCK_TOCR_LATE,             /* the plan was not ready by the arming sample, a sample was read late
                                   (counter check), or the arming came too late (stage 2b: no fault) */
    BUCK_TOCR_EARLY,            /* edge in the first BUCK_TOC_QUIET_PASSES passes after arming */
    BUCK_TOCR_N
};
typedef struct {
    /* tunables, written by the debugger (read at each score edge / each normal HOLD pass) */
    uint32_t toc_enable;        /* 1: a rising score edge (g 0 -> 1) in a normal HOLD pass requests a TOC */
    uint32_t learn_enable;      /* 1: DeltaI every normal HOLD pass; kappa, q and G after each TOC */
    /* learned coefficients: used by the next request (persist over STOP/RUN; writable) */
    float delta_i;              /* DeltaI, A */
    float q[2];                 /* q[ON_OFF] = q_valley, q[OFF_ON] = q_peak, A */
    float g[2][2];              /* timing gains g[order] = {g_on, g_off} */
    float kappa;                /* raw-load jump coefficient, A/A */
    /* DeltaI anchor (SPEC normal_anchor): V, m, Ic = m + DeltaI of the last normal HOLD pass */
    float an_v, an_m, an_ic;
    uint32_t an_loops;          /* loops of that pass; valid only if the next pass has loops + 1 */
    uint32_t an_valid;
    /* episode state and counts */
    uint32_t state;             /* BUCK_TOCS_*: where the last episode is / ended */
    uint32_t edges;             /* score edges seen by the TOC handler (rising g in normal HOLD passes) */
    uint32_t requests;          /* accepted requests (planning started) */
    uint32_t frames;            /* frames executed to D */
    uint32_t rej[BUCK_TOCR_N];  /* requests without a frame, by reason */
    /* the last plan / frame */
    int32_t plan_status;        /* BUCK_TOC_READY 1, ALREADY 2, NONE -1, LIMIT -2, INVALID -3, ABORT -4 */
    uint32_t order;             /* 0 ON-OFF, 1 OFF-ON */
    uint32_t on_ticks, off_ticks, per_ticks, resume_cmp, end_cmp;
    float x0_i, x0_v, ieq, target_i;
    uint32_t plan_samples;      /* samples read while planning (the frame starts after BUCK_TOC_PLAN_PERIODS) */
    float i_d, v_d, i_meas_d, i_pred_d, m_e, v_s0;   /* at D: corrected i(D), V(D), measured/model ADC2 mean, m_E; V(S0) */
    uint32_t arm_cnt, f_cnt, d_cnt;                  /* Timer A counter at the guarded writes: arming, return, D */
    /* hardware checks (RM0440 items not confirmed by the manual; README bench tests) */
    uint32_t chk_commit;        /* frame or return transfer not seen (MUPD/UPD) */
    uint32_t chk_out;           /* CHA1 state/flags not as encoded in the first span (SET1R at the boundary) */
    uint32_t chk_align;         /* master and Timer A counters drifted apart */
    int32_t align_ref, align_last;   /* Timer A minus master counter, read back to back: before / after the frame */
    /* learning */
    uint32_t kappa_n, q_n, g_n, q_rej, g_rej, di_n;  /* updates published / rejected; DeltaI steps */
    uint32_t slice_max;         /* stage 2 learning slices: unused since stage 2b (always 0) */
    /* stage 2b (30 Oct 2026): planning and learning periods, late checks (appended) */
    uint32_t plan_ready;        /* samples read since R when the last frame was ready to arm (spare periods:
                                   BUCK_TOC_PLAN_PERIODS - plan_ready; 0: not ready) */
    uint32_t late_svc;          /* samples read late in their period (planning/learning; counter check) */
    uint32_t late_arm;          /* armings refused late (counter before the guarded writes, or a missed guard) */
    uint32_t learn_samples;     /* samples read in the last learning periods (<= BUCK_TOC_LEARN_PERIODS) */
    uint32_t learn_done;        /* jobs done in the last learning periods: 1 kappa, 2 G, 4 q (published or
                                   rejected by their gates) */
    uint32_t learn_late;        /* learning jobs dropped: not done by the end of the learning periods */
    uint32_t kappa_rej;         /* kappa updates rejected (non-finite) */
    uint32_t d_hold;            /* D passes that ended late: B read by the TOC code, the duty held one more period */
    float v_b, m_b;             /* B (the first sample after D): v_out and the observer's m after its step */
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
    buck_toc_t toc;            /* TOC and learning (stage 2, appended: see buck_toc_t) */
} buck_t;

extern buck_t g_buck;

void buck_init(void);
void buck_start(void);
void buck_run(void) __attribute__((noreturn));
void buck_fatal(uint32_t reason) __attribute__((noreturn));

#endif
