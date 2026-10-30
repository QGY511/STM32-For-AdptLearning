#ifndef BUCK_CONTROL_H
#define BUCK_CONTROL_H
#include <stdint.h>
#include "buck_config.h"
#include "buck_estimator.h"
#if BUCK_OPTIMAL
#include "buck_optimal.h"   /* buck_sa_coef_t / buck_so_coef_t: integer trigger and solve coefficients */
#endif

enum { BUCK_STOP = 0U, BUCK_RUN = 1U };
enum { BUCK_DISARMED = 0U, BUCK_RUNNING = 1U, BUCK_FAULTED = 2U };
enum {
    BUCK_FAULT_CLOCK = 1U, BUCK_FAULT_ADC_START = 2U,
    BUCK_FAULT_DLL = 3U, BUCK_FAULT_ADC_OVERRUN = 4U,
    BUCK_FAULT_DEADLINE = 5U,
    BUCK_FAULT_SENSE = 6U,  /* feedback lost: A0 below a quarter of the reference for BUCK_SENSE_GUARD_US (thread guard) */
    BUCK_FAULT_CPU = 8U
};

/* Everything the 600 kHz ISR touches lives in ONE object, g_buck, so the ISR
 * loads a single base address (each separate global costs a literal-pool load
 * and a register) and addresses every field as base + offset. In Live
 * Expressions use g_buck.tune.<f>, g_buck.status.<f>, g_buck.opt.<f>. */

/* User-written inputs, read by the ISR every cycle. */
typedef struct {
    int32_t  kp_q16;        /* voltage loop: Q16 A/count (current mode) or Q16 duty ticks/count (voltage mode) */
    int32_t  ki_q16;        /* same units, per sample */
    uint32_t setpoint_adc;  /* target, ADC counts */
    uint32_t command;       /* BUCK_RUN / BUCK_STOP (polled by the thread, and by the ISR while disarmed) */
    uint32_t optimal_en;    /* 1 arms the time-optimal response */
    int32_t  kpi_q16;       /* current loop: Q16 duty ticks per Q8 A of current error */
    int32_t  kii_q16;       /* same, per sample */
    int32_t  isense_zero;   /* current-sense amplifier output at zero current, ADC counts */
    int32_t  isense_gain_q16; /* current-sense scale, Q16 A per count (1 mOhm x 40 x 3.3 V/4096: 1321) */
} buck_tune_t;

/* Debugger-readable control-loop telemetry. Signals are stored after the
 * compare write, so they add no pre-write delay. */
typedef struct {
    uint32_t rst_cause;  /* RCC->CSR reset flags at boot (bit27 BOR, 26 PIN, 28 SFT, 29 IWDG) */
    uint32_t state, fault;
    uint32_t setpoint;   /* ramped reference, ADC counts */
    uint32_t adc_raw;    /* A0 feedback sample as converted, ADC counts */
    uint32_t adc_glitch; /* feedback samples held off by the validity test (BUCK_ADC_JUMP_COUNTS): a count that keeps rising means A0 is glitching */
    int32_t  error;      /* setpoint - feedback (accepted sample), ADC counts */
    int32_t  i_term;     /* voltage-loop integrator, Q16 (A in current mode, duty ticks in voltage mode) */
    int32_t  d_term;     /* current-loop integrator, Q16 duty ticks (current mode) */
    uint32_t duty_ticks; /* CMP1 written, duty ticks */
    int32_t  i_avg_q16;  /* measured average inductor current this period (sample + ripple correction), Q16 A */
    int32_t  est_iload_a;/* load estimate (eq. 7 on the measured current, low-passed), integer amperes */
    int32_t  est_iload_ma; /* the same in mA, slow-averaged (BUCK_EST_DISPLAY=1 only) */
    uint32_t thread_loops;    /* main-loop heartbeat: must keep counting while running (thread not starved) */
    /* Time-optimal load-step response: counters (ISR) and the last completed
     * transient in panel units (thread, from the per-transient record). */
    uint32_t opt_trig_count;  /* trigger events */
    uint32_t opt_count;       /* pulses written */
    uint32_t opt_reject_count;/* triggers whose solve found no admissible pulse (load at the peak limit) or that were disarmed meanwhile */
    uint32_t opt_write_phase; /* HRTIM count (ticks) at the last pulse PER/CMP1 write; deadline is BUCK_WRITE_DEADLINE */
    int32_t  opt_score;       /* state-anchored trigger score S of the last sample, Q10 (1024 = boundary S=1); thread-computed display */
    int32_t  opt_zv0, opt_dv0;/* at the trigger: v_out - V_ref and its one-period change, ADC counts */
    int32_t  opt_i0_ma;       /* inductor current at the pulse start (measured valley extrapolated), mA */
    int32_t  opt_ihat_ma;     /* equilibrium current the pulse was solved for (measured load + b), mA */
    int32_t  opt_ipk_ma;      /* applied peak, mA */
    uint32_t opt_ton, opt_toff; /* applied CMP1 / off-time, HRTIM ticks */
    int32_t  opt_zvf;         /* terminal voltage residual z_v,f (raw sample - V_ref), ADC counts (<0: the pulse left the output low) */
    int32_t  opt_zif_ma;      /* terminal current residual z_i,f: measured valley after the pulse - target valley (I_eq - h), mA (>0: landed high) */
    int32_t  opt_zib_ma;      /* equilibrium residual e_b: settled load - I_eq, mA (>0: the pulse aimed below the load) */
} buck_status_t;

#if BUCK_OPTIMAL
/* One transient, trigger to settled: written by the ISR as the state machine
 * advances, read by the thread once complete (learn_req). Two records
 * alternate, so a thread stalled for a whole transition (320 us) still reads
 * a consistent set (an ISR->thread record must never be a live field). */
typedef struct {
    /* trigger (period n) */
    volatile int32_t  zv0, dv0;      /* v_out - V_ref and its one-period change, counts */
    volatile uint32_t adc0;          /* v_out at the trigger, counts */
    volatile int32_t  iavg0_q20;     /* measured average inductor current at the trigger, Q20 A */
    volatile uint32_t duty0;         /* on-time that produced that current, ticks (reseed anchor of the current loop) */
    /* solve (n+1) */
    volatile int32_t  i0_q20;        /* inductor current at the pulse start (measured valley extrapolated to period n+3), Q20 A */
    volatile int32_t  iload_q20;     /* load measured over the period after the edge, Q20 A (no b) */
    volatile int32_t  ieq_q20;       /* equilibrium current the pulse was solved for: iload + b, Q20 A */
    volatile int32_t  gon_q16, goff_q16; /* gains that produced this pulse */
    volatile uint32_t ton, toff;     /* applied CMP1 (physical + dead time) / off-time, ticks; ton == 0: rejected */
    volatile uint32_t sat;           /* BUCK_SO_SAT_* clamp flags */
    volatile int32_t  ipk_q16;       /* applied peak, Q16 A */
    volatile int32_t  term_q20;      /* model terminal current of the applied pulse, Q20 A */
    /* write (n+2): the reseed the PULSE cycle applies */
    volatile uint32_t new_duty;      /* on-time of the first normal period after the pulse (the duty that holds I_eq) */
    volatile int32_t  new_iref_q16;  /* voltage-loop reference after the pulse: I_eq, Q16 A */
    /* settle (n+4): the first sample after the stretched period */
    volatile int32_t  zv_f;          /* v_out - V_ref, counts (raw) */
    volatile int32_t  is_f_q20;      /* MEASURED sample current at that instant, Q20 A */
    volatile int32_t  iavg_f_q20;    /* that period's measured average current, Q20 A */
    /* end of the linear-controller transition (n+4+BUCK_OPT_SETTLE_CYCLES) */
    volatile int32_t  iload_s_q20;   /* settled load estimate, Q20 A */
    volatile int32_t  zv_s;          /* v_out - V_ref then, counts */
} buck_opt_rec_t;
#endif

/* ISR<->thread data for the time-optimal pulse. */
typedef struct {
#if BUCK_OPTIMAL
    buck_opt_rec_t    rec[2];       /* fields are volatile */
#endif
    volatile uint32_t learn_req;    /* ISR -> thread: 0 none, else 1 + index of the completed record; the thread clears it */
    volatile int32_t  sa_zv;        /* ISR -> thread: z_v of the last sample (ADC counts), debug-panel score */
    volatile int32_t  sa_dv;        /* ISR -> thread: d_v of the last sample (ADC counts per period) */
    volatile uint32_t trig;         /* ISR telemetry: 1 from the trigger until the first sample after the pulse */
} buck_opt_io_t;

/* ISR-private control state (visible for the debugger and the host simulator,
 * written only by the ISR except where noted). */
typedef struct {
    uint32_t warmup, running, reference_q16;
    uint32_t ovr_run;      /* consecutive ADC-overrun cycles (debug-halt tolerance) */
    int32_t  integrator_q16;   /* voltage loop */
    int32_t  integrator_i_q16; /* current loop (Q16 duty ticks) */
    uint32_t active_duty;  /* on-time (ticks) currently driving the outputs */
    uint32_t adc_prev;     /* last ACCEPTED feedback sample, counts (sample validity, d_v) */
    uint32_t adc_raw_prev; /* last feedback sample as converted (two consecutive out-of-range samples are accepted) */
#if BUCK_ESTIMATOR
    buck_est_t est;        /* load estimate state: previous v_out, filtered load, previous average current */
#endif
#if BUCK_OPTIMAL
    uint32_t opt_state;    /* OPT_IDLE / WAIT / WRITE / PULSE / SETTLE */
    uint32_t opt_cooldown; /* trigger mask, cycles: soft-start (re-armed by the ramp), the linear-controller transition after a pulse, the short hold-off after a rejected solve */
    uint32_t rec_idx;      /* record being written (0/1) */
    uint32_t settle_pending; /* a pulse's transition is running: sample the settled load when the mask ends */
#endif
} buck_ctl_t;

typedef struct {
    volatile buck_tune_t   tune;
    volatile buck_status_t status;
    buck_opt_io_t          opt;    /* fields are volatile */
#if BUCK_OPTIMAL
    volatile buck_sa_coef_t sa;    /* integer trigger coefficients, written by the thread */
    volatile buck_so_coef_t so;    /* integer pulse-solve coefficients (V_ref, r, ESR) and the learned gains, written by the thread */
#endif
    buck_ctl_t             ctl;
} buck_t;
extern buck_t g_buck;

/* Source-compatibility names. */
#define g_buck_tune          (g_buck.tune)
#define g_buck_status        (g_buck.status)
#define g_buck_opt           (g_buck.opt)
#define g_buck_kp_q16        (g_buck.tune.kp_q16)
#define g_buck_ki_q16        (g_buck.tune.ki_q16)
#define g_buck_setpoint_adc  (g_buck.tune.setpoint_adc)
#define g_buck_command       (g_buck.tune.command)
#define g_buck_optimal_en    (g_buck.tune.optimal_en)

/* Thread-only tunables (not read by the ISR). */
extern volatile float g_buck_sa_va;   /* voltage scale V_a, ADC counts */
extern volatile float g_buck_sa_dv;   /* slope scale dV_a, ADC counts per period */
extern volatile float g_buck_sa_rho;  /* tilt rho, |rho| < 1 */
extern volatile int32_t g_buck_opt_gamma_on_q16;  /* learned timing gains gamma_on / gamma_off, Q16 (65536 = 1.0): T_applied = gamma T* */
extern volatile int32_t g_buck_opt_gamma_off_q16;
extern volatile int32_t g_buck_opt_ieq_q20;       /* learned equilibrium correction b, Q20 A (solve target = measured load + b) */
extern volatile float   g_buck_opt_r_off_mohm;    /* series resistance of the pulse's OFF ramp, mOhm (live tunable) */
extern volatile float   g_buck_opt_esr_mohm;      /* output-capacitor ESR, mOhm (live tunable, 0 = off): solve droop and residual corrections */

/* Per-pulse adaptation log, written by the thread at each completed transient
 * (no ISR cost): the last BUCK_LOG_N pulses' inputs, results and residuals in
 * panel units, for reading through the debugger (Live Expressions: expand
 * g_buck_log, g_buck_log_n counts events; entry (g_buck_log_n - 1) % BUCK_LOG_N
 * is the newest). */
#define BUCK_LOG_N 16U
typedef struct {
    uint32_t n;            /* event index */
    int32_t  zv0, dv0;     /* at the trigger: droop and its one-period change, counts */
    int32_t  ihat_ma;      /* equilibrium current the pulse was solved for (measured load + b), mA */
    int32_t  iload_ma;     /* load measured over the period after the edge, mA */
    int32_t  i0_ma;        /* inductor current at the pulse start, mA */
    uint32_t ton, toff;    /* applied CMP1 / off-time, ticks */
    int32_t  ipk_ma;       /* applied peak, mA */
    int32_t  term_ma;      /* model terminal current of the applied pulse, mA */
    uint32_t sat;          /* clamp flags */
    int32_t  zv_f;         /* terminal voltage residual, counts (raw sample) */
    int32_t  zif_ma;       /* terminal current residual (measured valley - target valley), mA */
    int32_t  zib_ma;       /* equilibrium residual (settled load - I_eq), mA */
    int32_t  gon_q16, goff_q16, ieq_ma;   /* gains and b AFTER this event's update */
    int32_t  reseed_ma;    /* current reference the loop restarted from (I_eq), mA */
    uint32_t flags;        /* bit0: timing update skipped (residual beyond the SKIP bound or clamped pulse), bit1: b update skipped */
} buck_log_t;
extern volatile buck_log_t g_buck_log[BUCK_LOG_N];
extern volatile uint32_t   g_buck_log_n;

void buck_init(void);
void buck_start_sampling(void);
/* Thread-mode poll of g_buck.tune.command (the running ISR no longer reads it
 * every cycle): a STOP disables the gate outputs and drops the loop to the
 * disarmed cold path within one main-loop iteration. */
void buck_command_service(void);
/* One pass of every thread-mode service (main loop body; the host simulator
 * calls it too). */
void buck_thread_service(void);
void buck_fatal(uint32_t reason) __attribute__((noreturn));

#endif
