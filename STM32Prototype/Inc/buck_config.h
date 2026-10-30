#ifndef BUCK_CONFIG_H
#define BUCK_CONFIG_H

#include <stdint.h>
#include <limits.h>

/* Board: NUCLEO-G474RE, active-high gate inputs, VDDA/VREF+ = 3.3 V.
 * HSI works without changes to board solder bridges. Select HSE only after
 * checking the fitted crystal/clock and board clock routing. */
#ifndef BUCK_USE_HSE
#define BUCK_USE_HSE             0
#endif
#define BUCK_HSE_HZ              24000000U
#define BUCK_HSE_BYPASS          0
#define BUCK_CPU_HZ              168000000U
#define BUCK_ADC_HZ              56000000U
#define BUCK_SWITCH_HZ           600000U
#define BUCK_CYCLES_PER_PERIOD   (BUCK_CPU_HZ / BUCK_SWITCH_HZ)
#define BUCK_HRTIM_PERIOD        (BUCK_CYCLES_PER_PERIOD * 32U)

/* A late, fixed sampling phase minimizes the wait for the next safe preload
 * transfer. Reduce this lead only after measuring the worst-case write phase.
 * Units are CPU clocks; HRTIM has 32 fine ticks per CPU clock. */
#ifndef BUCK_SAMPLE_LEAD_CYCLES
#define BUCK_SAMPLE_LEAD_CYCLES   180U
#endif
#define BUCK_WRITE_GUARD_CYCLES   16U
/* Duty-INDEPENDENT ADC trigger. A fixed phase inside the high-side on-time,
 * after the rising-edge + dead-time settle and before the MINIMUM falling edge
 * (DUTY_MIN), so the sample always lands in the same settled interval whatever
 * the duty is. This removes the "sample must clear DUTY_MAX" constraint that
 * capped duty at ~30%: DUTY_MAX is now free, and the compare write lands with
 * large margin before the boundary. Trade-off: sampled in HS-on (verify A0
 * switching noise on the bench) and ~0.5 us more sample-to-actuation latency. */
#ifndef BUCK_SAMPLE_COMPARE
#define BUCK_SAMPLE_COMPARE       600U
#endif
#define BUCK_WRITE_DEADLINE      (BUCK_HRTIM_PERIOD - 32U * BUCK_WRITE_GUARD_CYCLES)

#define BUCK_DEADTIME_NS          50U
/* DTPRSC=0: deadtime resolution is 1/(8*fHRTIM), NOT 1/(32*fHRTIM).
 * Round UP: 68 counts -> 50.595 ns at 168 MHz. */
#define BUCK_DT_COUNTS           ((BUCK_DEADTIME_NS * 1344U + 999U) / 1000U)
#define BUCK_DT_FINE_TICKS       (BUCK_DT_COUNTS * 4U)
#define BUCK_MIN_GATE_PULSE_NS    50U
#define BUCK_MIN_GATE_TICKS      ((BUCK_MIN_GATE_PULSE_NS * 5376U + 999U) / 1000U)
/* Duty command = CMP1 / PER, before hardware dead time. Every armed duty
 * write is saturated to 10..50%; startup uses the same lower bound. */
#define BUCK_DUTY_MIN_PERCENT    10U
#define BUCK_DUTY_MAX_PERCENT    50U
#define BUCK_DUTY_MIN            ((BUCK_HRTIM_PERIOD * BUCK_DUTY_MIN_PERCENT + 99U) / 100U)
#define BUCK_DUTY_MAX            ((BUCK_HRTIM_PERIOD * BUCK_DUTY_MAX_PERCENT) / 100U)

#define BUCK_VIN_MV              6000U
/* Regulated output at the default setpoint (BUCK_SETPOINT_DEFAULT_ADC counts),
 * MEASURED: scope mean 792.6 mV at 1000 counts (12-bit ADC, 3.3 V reference,
 * direct sense: 806 uV/count nominal). It sets BUCK_UV_PER_COUNT, i.e. every
 * voltage the model, the current loop and the pulse solve derive from the ADC.
 * It was 1000 (an assumed 1 mV/count): every model voltage 26 % too large, the
 * pulse refilled a 26 % larger deficit with a 26 % too steep off-ramp and left
 * ~1.3 A above the load after every pulse (opt_verr -95 counts on the bench). */
#define BUCK_VOUT_MV             793U
#define BUCK_VREF_MV             3300U
/* (2.2/12.2)*(1+46.4/36) = 2266/5490. Counts use 4096 quantizer levels. */
#define BUCK_SENSE_NUM           2266U
#define BUCK_SENSE_DEN           5490U
#define BUCK_MV_TO_ADC(mv)       ((uint32_t)(((uint64_t)(mv) * BUCK_SENSE_NUM * 4096U + \
                                            (uint64_t)BUCK_VREF_MV * BUCK_SENSE_DEN / 2U) / \
                                           ((uint64_t)BUCK_VREF_MV * BUCK_SENSE_DEN)))
#define BUCK_TARGET_ADC          BUCK_MV_TO_ADC(BUCK_VOUT_MV)
/* Operational default target in ADC counts (measured correct on this board;
 * differs from BUCK_TARGET_ADC if the fitted feedback scale differs from the
 * assumed sense network). Runtime-tunable via g_buck_setpoint_adc. */
#define BUCK_SETPOINT_DEFAULT_ADC 1000U
/* A0 is PI feedback only: no voltage-threshold or prebias shutdown.
 * Duty saturation does not detect overvoltage or excess current. */

/* ---- Load / valley-peak estimator (TPEL Sec. III-A, eqs 7-15) ------------
 * Physical stage parameters. SET BUCK_R_MOHM from your inductor DCR + FET/path
 * resistance; theta = r/L and the Pade coefficients depend on it. The Q-format
 * constants below are folded at COMPILE time (no runtime float is emitted). */
#ifndef BUCK_ESTIMATOR
#define BUCK_ESTIMATOR           1
#endif
/* Display layer (slow average + mA/A + round-robin panel conversion). Set to 0
 * for the leanest estimator: core recursion only, load exported raw in amps. */
#ifndef BUCK_EST_DISPLAY
#define BUCK_EST_DISPLAY         0
#endif
/* Time-optimal load-step-up response (TPEL Sec. IV-V). On a detected load
 * step-up it inserts one large-signal ON(Ton)-OFF(Toff) pulse (a single
 * stretched HRTIM period) then hands back to the PI; the pulse timing is
 * adapted from the measured terminal residual (Jacobian update, below). Solve,
 * apply and residual measurement all run in the ISR (integer); the thread does
 * the float adaptation once per pulse. Requires the estimator. Bring up
 * current-limited before enabling. */
#ifndef BUCK_OPTIMAL
#define BUCK_OPTIMAL             1
#endif
/* State-anchored trigger on the OUTPUT-CAPACITOR state. The conditions for a
 * pulse are: the response is enabled, soft-start is over, no pulse is in
 * flight, the linear-controller transition after the previous pulse is over
 * (BUCK_OPT_SETTLE_CYCLES), the sample is below V_ref and not rising, the score
 * below exceeds its boundary, and the solve finds an admissible pulse. No
 * arming counter, no recovery test, no learner gates: whatever the pulse
 * leaves behind is measured and fed to the adaptation.
 * At each sample the two inputs are, both in ADC counts,
 *   z_v  = v_out - V_ref                    (memoryless, = -error), and
 *   d_v  = v_out(n) - v_out(n-1)            (one-period change: C dv/dt is the
 *          capacitor current i_L - I_load, the load step itself, BEFORE the
 *          current loop has moved the inductor; 1 A of capacitor current is
 *          6.4 mV = 8 counts per period).
 * With the inductor current MEASURED and regulated, the old current-displacement
 * axis (d_iL) only reported what the current loop was doing; d_v reports the
 * load. A fast 3 A step shows -24 counts in its first period (S = 2.25 at the
 * defaults: fires at once); a slow slew (15-40 us) shows ~1 count per period
 * and fires on the level z_v, i.e. two thirds of the way down the dip that
 * the current-mode PI leaves on its own (~30 mV on the bench). Fire when
 *   S(z) = (z_v/V_a)^2 + (d_v/dV_a)^2 + 2*rho*(z_v d_v)/(V_a dV_a) > 1
 * on the droop side (z_v < 0, d_v <= 0: the solver is the step-up ON-OFF
 * order; a recovering voltage never fires). rho > 0 emphasises the step-up
 * signature (below V_ref AND falling). The score is evaluated EVERY cycle in
 * the ISR in integer arithmetic (buck_sa_trigger_i, ~18 clocks, only when the
 * two sign tests pass); V_a, dV_a and rho stay RUNTIME-tunable floats
 * (g_buck_sa_va/dv/rho, Live Expressions) which the thread converts to the
 * ISR's integer coefficients whenever they change. Noise at the defaults:
 * +-6 counts on the samples (+-12 of d_v) score 0.6; simulation with +-8
 * counts of noise false-triggers on the slope axis, so watch opt_score (the
 * score of the last candidate sample) at steady state and raise dV_a if it
 * sits above ~0.5. */
#define BUCK_SA_VA_COUNTS   25.0f    /* default voltage scale V_a, ADC counts (20 mV) */
#define BUCK_SA_DVA_COUNTS  16.0f    /* default slope scale dV_a, ADC counts per period (2 A of capacitor current) */
#define BUCK_SA_RHO         0.0f     /* default tilt rho, |rho| < 1 */
#define BUCK_SA_STHRESH     1.0f     /* trigger when S > this (ellipse boundary) */
/* Linear-controller transition (manuscript Sec. VI): after a pulse the trigger
 * is masked for this many cycles while the voltage PI owns the recovery from
 * whatever residual the pulse left. The PI reference where it has settled when
 * the mask ends is the reseed learner's sample (below). Without this, a wrong
 * reseed rings the loop, the ring's undershoot re-triggers, every re-trigger
 * discards the settle sample, and the loop limit-cycles (host simulation:
 * 19 pulses and a 1.2 V overshoot on a 1 -> 6 A step with a 15 mOhm plant).
 * 192 cycles = 320 us (bench: the ring is gone by ~250 us). */
#define BUCK_OPT_SETTLE_CYCLES   192U
/* Trigger hold-off after a REJECTED solve (no admissible pulse: the load is
 * already at the peak limit, or the on-time inverts to nothing), cycles. The
 * solve cycle runs no PI, so a trigger that stays true every cycle would
 * otherwise starve the voltage loop of every other sample. */
#define BUCK_OPT_REJECT_HOLD_CYCLES 6U
/* Start-up mask: cycles after arming and after the soft-start ramp completes
 * during which the trigger is held off (the first regulated cycle sees a large
 * one-period current displacement from the un-seeded reference). 3.3 ms. */
#define BUCK_OPT_COOLDOWN_START  2000U
/* ---- Dual correction after the pulse (TPEL manuscript Sec. V + equilibrium) --
 * The applied pulse is t_a = diag(gamma_on, gamma_off) t*, t* the closed-form
 * solution (Sec. IV) for the periodic steady state at (I_eq, V_ref), I_eq =
 * I_hat + b: the load measured one period after the edge plus a learned
 * correction b. Everything the pulse leaves behind is MEASURED (the inductor
 * current is sensed), so the terminal state is observed directly, at the
 * first sample after the stretched period (SETTLE, 600 ticks into the first
 * normal period, before the PI has changed anything):
 *   z_v,f = v_out - V_ref                                   (terminal voltage)
 *   z_i,f = (i_s - RISE) - (I_eq - h)                       (terminal current:
 *           the measured sample minus the 61 ns of on-ramp above the valley,
 *           against the target valley I_eq - h; h = ripple/2)
 * and the equilibrium itself is checked once the linear controller has
 * settled, at the end of the BUCK_OPT_SETTLE_CYCLES mask (~320 us):
 *   e_b   = I_settled - I_eq                                (the load the loop
 *           actually settled at, from the low-passed eq. 7 estimate, against
 *           the equilibrium the pulse was solved for).
 * Three residuals, three parameters, one thread event per pulse:
 *   TIMING       gamma <- Pi[ gamma - mu diag(t*)^-1 J^-1 [z_i,f; z_v,f] ]
 *                (eqs 43-48: the full Newton step on the measured terminal
 *                state; J is the LC Jacobian at the applied times with the
 *                fall slope (V_ref + r i_2)/L). A peak- or Ton-clamped pulse
 *                cannot move its charge: Toff alone takes mu z_i,f / m2.
 *   EQUILIBRIUM  b <- b + mu_b e_b  (bounded).
 * The residuals are separable: a wrong equilibrium moves e_b and not the
 * terminal residuals (the pulse still lands on its target), a wrong L, C or r
 * moves the terminal residuals. The determinant floor (DET_MIN) keeps a pulse
 * with no modelled excursion from an unbounded step.
 * VALIDITY: a residual beyond the CLIP is clipped (it still carries the sign),
 * one beyond the SKIP bound is not information (a corrupted sample, a pulse
 * fired on garbage) and that pulse teaches nothing. On the bench (2026-09-25)
 * a voltage sample of zero counts fired the trigger at every unmasked cycle
 * and the previous clip-only rule ran both learners to their bounds. */
#define BUCK_OPT_GAMMA_MU_Q8     64      /* mu = 0.25 (manuscript Fig. 6 uses 0.2) */
#define BUCK_OPT_GAMMA_MIN_Q16   32768   /* gamma >= 0.5 */
#define BUCK_OPT_GAMMA_MAX_Q16   131072  /* gamma <= 2.0 */
#define BUCK_OPT_GAMMA_STEP_Q16  16384   /* max |delta gamma| per pulse, 0.25 */
#define BUCK_OPT_GAMMA_DET_MIN_A 0.5     /* det J floored at m2 * this / C (A: minimum modelled excursion) */
/* Series resistance used for the pulse's OFF ramp (V_ref + r i)/L: the drop
 * in the inductor path while the low side conducts. Power-on default of the
 * live tunable g_buck_opt_r_off_mohm; BUCK_R_MOHM is a DC fit that may include
 * losses that are not in the fall (dead-time conduction, input sag), so a
 * pulse that lands ABOVE the load with gamma = 1 and b = 0 says this is too
 * high, one that lands below says too low (see the per-pulse log). */
#define BUCK_OPT_R_OFF_MOHM      BUCK_R_MOHM
/* Output-capacitor ESR, mOhm (power-on default of the live tunable
 * g_buck_opt_esr_mohm; 0 = no correction). The ADC sees v_C + ESR (i_L -
 * I_load) at the sample instant. Two places use it, both with the MEASURED
 * sample current: the solve subtracts ESR (i_s - I_hat) from the droop it
 * refills (during a droop the inductor lags the load, so the sample reads
 * lower than the capacitor by ESR times that lag: ~24 mV = 30 counts for a
 * 2 A lag at 12 mOhm, more than half the droop of a 3 A step), and the
 * terminal-voltage residual is corrected the same way against the settled
 * load. Measure it on the scope: the spike during the pulse divided by
 * (I_pk - I_load). No in-loop identification (the previous one read 60-75 %
 * of the truth and needed a residual window the ISR no longer has). */
#define BUCK_OPT_ESR_MOHM        0.0
/* Residual clips (the value still teaches, with its sign) and skip bounds
 * (the pulse teaches nothing): 2 A / 40 counts (32 mV), 6 A / 200 counts. */
#define BUCK_OPT_ZI_CLIP_A       2.0
#define BUCK_OPT_ZV_CLIP_COUNTS  40
#define BUCK_OPT_ZI_SKIP_A       6.0
#define BUCK_OPT_ZV_SKIP_COUNTS  200
#define BUCK_OPT_IEQ_MU_Q8       128     /* mu_b = 0.5 */
#define BUCK_OPT_IEQ_MAX_A       3.0     /* |b| bound, A */
/* Post-pulse operating point: with the inductor current MEASURED the voltage
 * PI's reference is in real amperes, so after the pulse it is reseeded to the
 * equilibrium current I_eq itself, and the current PI to the duty that holds it,
 * duty_trig + r/V_in PER (I_eq - i_avg,trig) (the fitted r is the DC relation
 * between duty and current, which is exactly what this needs). No learned
 * sensitivity, no rest-state tracking. */
#define BUCK_OPT_TMIN_S          100e-9  /* Ton/Toff lower clamp */
/* Peak-current-mode limit for the large-signal pulse (CONTROL, not protection:
 * it only shortens Ton; over-current PROTECTION is external, no in-loop trip).
 * Ton is capped so the predicted peak inductor current I_peak = I_start (Pade
 * valley/avg estimate at the step) + (Vin-Vout)/L * Ton stays <= this. SET THIS
 * to your power stage's safe peak current. This, not TON_MAX, is normally the
 * binding limit; TON_MAX remains the hard <1-period ceiling for CMP1 validity. */
#define BUCK_OPT_IPK_MAX_A       10.0
#define BUCK_OPT_TON_MAX_S       1.5e-6  /* Ton upper clamp: MUST be < one switching
                                          * period. CMP1=Ton is written while the active
                                          * period is still BUCK_HRTIM_PERIOD; a compare
                                          * larger than the active period is out of range
                                          * and is dropped (HS then runs at the old duty),
                                          * so Ton>Tsw silently fails to actuate. Also caps
                                          * the peak-current excursion (dI=(Vin-Vout)Ton/L). */
#define BUCK_OPT_TOFF_MAX_S      6e-6    /* Toff upper clamp (Ton+Toff must fit HRTIM PER) */
/* Minimum length of the stretched period, ticks. The ISR that restores the
 * normal PWM fires at a FIXED point of the stretched period (ADC trigger at
 * CMP3 + conversion + entry, ~4500-5300 ticks in). A stretched period shorter
 * than that would end before the restore is written: the short pulse repeats
 * and the deadline check trips. Holding PER >= one normal period keeps every
 * ISR timing assumption identical to a normal cycle; Toff is extended by the
 * difference (only binds when the learned gain k shrinks Toff/k). */
#define BUCK_OPT_PER_MIN_TICKS   BUCK_HRTIM_PERIOD
#define BUCK_OPT_TMIN_TICKS      ((uint32_t)(BUCK_OPT_TMIN_S  * BUCK_SWITCH_HZ * BUCK_HRTIM_PERIOD + 0.5))
#define BUCK_OPT_TON_MAX_TICKS   ((uint32_t)(BUCK_OPT_TON_MAX_S  * BUCK_SWITCH_HZ * BUCK_HRTIM_PERIOD + 0.5))
#define BUCK_OPT_TOFF_MAX_TICKS  ((uint32_t)(BUCK_OPT_TOFF_MAX_S * BUCK_SWITCH_HZ * BUCK_HRTIM_PERIOD + 0.5))
#define BUCK_OPT_IPK_MAX_Q8      ((int32_t)(BUCK_OPT_IPK_MAX_A * 256.0 + 0.5))
_Static_assert(!BUCK_OPTIMAL || BUCK_ESTIMATOR, "BUCK_OPTIMAL needs the estimator");
_Static_assert((BUCK_OPT_TON_MAX_S + BUCK_OPT_TOFF_MAX_S) * BUCK_SWITCH_HZ * BUCK_HRTIM_PERIOD < 65503.0, "optimal pulse exceeds HRTIM period range");
_Static_assert(BUCK_OPT_TON_MAX_S * BUCK_SWITCH_HZ < 1.0, "Ton clamp must be < one switching period or CMP1=Ton is out of range at write time");
#define BUCK_L_H                 330e-9
#define BUCK_C_F                 260e-6
/* Series resistance sets theta=r/L; the estimator's DC current ~= (D*Vin-Vout)/r,
 * i.e. inversely proportional to r. Calibrated OFFLINE against ground truth:
 * a raw slope of 3800 mA_est per 1 A_true at r=30 mOhm implies r_true ~= 30*3.8. */
#define BUCK_R_MOHM              114.0           /* series resistance, mOhm (offline-calibrated) */
#define BUCK_R_OHM               (BUCK_R_MOHM * 1e-3)
#define BUCK_VIN_V               (BUCK_VIN_MV * 1e-3)
#define BUCK_TSW_S               (1.0 / BUCK_SWITCH_HZ)
#define BUCK_TICK_S              (BUCK_TSW_S / BUCK_HRTIM_PERIOD)
#define BUCK_UV_PER_COUNT        (BUCK_VOUT_MV * 1000.0 / BUCK_SETPOINT_DEFAULT_ADC)
/* Fixed-point Q-formats chosen so every runtime product uses the HIGH word of
 * a 32x32 SMULL (shift == 32, single cycle): currents Q24 A, voltages Q28 V,
 * a coeff Q31, b coeff Q28 A/V, D Q31. Constants folded at compile time. */
#define BUCK_VCOUNT_ADC_Q28      ((int32_t)((BUCK_UV_PER_COUNT * 1e-6) * 268435456.0 + 0.5)) /* V/count, multiply adc -> Q28 V */
#define BUCK_VIN_Q28             ((int32_t)(BUCK_VIN_V * 268435456.0 + 0.5))
#define BUCK_THETA_TICK_Q28      ((int32_t)((BUCK_R_OHM / BUCK_L_H) * BUCK_TICK_S * 268435456.0 + 0.5)) /* x=theta*tau per tick, Q28 */
#define BUCK_BTAU_TICK_Q24       ((int32_t)((2.0 * BUCK_TICK_S / BUCK_L_H) * 16777216.0 + 0.5))         /* 2*tau/L per tick, Q24 A/V */
#define BUCK_INV_TSW_Q31         ((int32_t)(2147483648.0 / BUCK_HRTIM_PERIOD + 0.5))                    /* D = T_on * this, Q31 */
#define BUCK_CTS_Q23             ((int32_t)((BUCK_C_F * BUCK_SWITCH_HZ) * 8388608.0 + 0.5))             /* C/T_sw, A/V, Q23 */
/* Load-estimate low-pass (eq 20-21); ~0.15 per cycle. Raise for less lag. */
#ifndef BUCK_EST_ALPHA_Q31
#define BUCK_EST_ALPHA_Q31       ((int32_t)(0.15 * 2147483648.0 + 0.5))
#endif
/* Extra slow display average for the debug panel: time constant 2^SHIFT cycles
 * (8 -> ~0.43 ms at 600 kHz). Raise for a steadier reading, lower for less lag. */
#ifndef BUCK_EST_SLOW_SHIFT
#define BUCK_EST_SLOW_SHIFT      8
#endif

/* ---- Inner loop: AVERAGE CURRENT MODE with the sensed inductor current ---------
 * 1: the inductor current is measured (shunt x amplifier on A1 = PA1 = ADC2_IN2,
 * converted by ADC2 on the same HRTIM trigger and at the same instant as the
 * voltage on A0), the voltage PI outputs an average-current reference (Q16 A
 * per ADC count) and a current PI turns the current error into the duty. No
 * model current in the loop. The one sample per period is taken a fixed
 * (SAMPLE - DT) ticks into the on-time, so the average over the period is the
 * sample plus the ripple correction (V_in - V_ref)/L * (T_on/2 - (SAMPLE - DT)):
 * one multiply, and the only place L enters the loop. 0: the PI outputs duty
 * ticks directly (voltage mode); the sensed current still feeds the load
 * estimate. The load estimate is the manuscript's eq. 7 on the measured average
 * current: I_load = i_avg - C dv/dt, low-passed (eqs 20-21); the Pade recursion
 * (Inc/buck_estimator.h) is no longer run in the ISR. */
#ifndef BUCK_CURRENT_MODE
#define BUCK_CURRENT_MODE        1
#endif
#define BUCK_ISENSE_MOHM         1.0     /* shunt, mOhm */
#define BUCK_ISENSE_GAIN         40.0    /* sense amplifier gain (V/V) */
#define BUCK_ISENSE_ZERO_COUNTS  0       /* amplifier output at zero current, ADC counts (power-on default of g_buck.tune.isense_zero) */
#define BUCK_ISENSE_A_PER_COUNT  ((BUCK_VREF_MV * 1e-3 / 4096.0) / (BUCK_ISENSE_MOHM * 1e-3 * BUCK_ISENSE_GAIN))   /* 20.1 mA per count */
#define BUCK_ISENSE_GAIN_Q16     ((int32_t)(BUCK_ISENSE_A_PER_COUNT * 65536.0 + 0.5))          /* Q16 A per count: default of g_buck.tune.isense_gain_q16 */
/* Ripple correction, Q16 A per tick of on-time: (V_in - V_ref)/L * tick / 2. The
 * sample sits (SAMPLE - DT) ticks up the on-ramp, so
 *   i_avg = i_sample + RIPPLE * (CMP1 - DT - 2 (SAMPLE - DT)) = i_sample + RIPPLE * (CMP1 - RIPPLE_OFF). */
#define BUCK_ISENSE_RIPPLE_Q16   ((int32_t)((BUCK_VIN_V - BUCK_VOUT_MV * 1e-3) / BUCK_L_H * BUCK_TICK_S / 2.0 * 65536.0 + 0.5))
#define BUCK_ISENSE_RIPPLE_OFF   ((int32_t)(2U * BUCK_SAMPLE_COMPARE - BUCK_DT_FINE_TICKS))
#define BUCK_ISENSE_RISE_Q20     ((int32_t)((BUCK_VIN_V - BUCK_VOUT_MV * 1e-3) / BUCK_L_H * (BUCK_SAMPLE_COMPARE - BUCK_DT_FINE_TICKS) * BUCK_TICK_S * 1048576.0 + 0.5))   /* sample minus valley, Q20 A */
/* Voltage loop, Q16 A per ADC count (1 count ~ 0.79 mV). The bench-tuned
 * 800 / 60 of the previous inner loop rang less but recovered slower; with the
 * average-current inner loop below, a closed-loop sweep (host simulator, PI
 * alone, 1 -> 3 A) gave the best dip / hump at 1600 / 100 (0.73 / 0.80 V against
 * 0.69 / 0.83 V at 800 / 60). Live-tunable via g_buck.tune.kp_q16 / ki_q16: fall
 * back to 800 / 60 if the bench rings. */
#define BUCK_CM_KP_Q16           1600
#define BUCK_CM_KI_Q16           100
/* Current loop, Q16 duty ticks per Q8 A of current error. One period of duty
 * moves the average current by V_in T/(L PER) = 3.4 mA per tick, so 296 ticks/A
 * would be dead-beat; 250 ticks/A (0.977 ticks per Q8 A: 64000) keeps a margin
 * for L and V_in being off, with the PI zero near 15 kHz (Ki 6000). The same
 * sweep preferred this over 150 ticks/A / 2000. Live-tunable via
 * g_buck.tune.kpi_q16 / kii_q16. */
#define BUCK_ACM_KP_Q16          64000
#define BUCK_ACM_KI_Q16          6000
#define BUCK_ACM_IMIN_Q16        ((int32_t)(-2.0 * 65536.0))                     /* current-reference range, Q16 A */
#define BUCK_ACM_IMAX_Q16        ((int32_t)(BUCK_OPT_IPK_MAX_A * 65536.0))
/* Power-on defaults of the live-tunable gains, per mode. */
#if BUCK_CURRENT_MODE
#define BUCK_KP_DEFAULT_Q16      BUCK_CM_KP_Q16
#define BUCK_KI_DEFAULT_Q16      BUCK_CM_KI_Q16
#else
#define BUCK_KP_DEFAULT_Q16      BUCK_KP_Q16
#define BUCK_KI_DEFAULT_Q16      BUCK_KI_Q16
#endif

/* Voltage-mode PI output unit: HRTIM fine ticks; gains: Q16 ticks / ADC count.
 * Provisional bring-up values, NOT a validated compensation design:
 * Kp = 0.02 duty/V, Ki ~= 104.7 duty/(V*s), Ts = 1/600000 s.
 * Kp_q16 = round(Kp * PERIOD / counts_per_volt * 65536)
 * Ki_q16 = round(Ki * Ts * PERIOD / counts_per_volt * 65536)
 * These gains must be assessed with load, ESR, DCR and feedback dynamics. */
#define BUCK_PI_SHIFT            16U
#define BUCK_KP_Q16              22924
#define BUCK_KI_Q16              200
#define BUCK_MIN_Q16             ((int32_t)(BUCK_DUTY_MIN << BUCK_PI_SHIFT))
#define BUCK_MAX_Q16             ((int32_t)(BUCK_DUTY_MAX << BUCK_PI_SHIFT))
#define BUCK_SOFTSTART_MS        10U
#define BUCK_SOFTSTART_CYCLES    (BUCK_SWITCH_HZ / 1000U * BUCK_SOFTSTART_MS)
#define BUCK_REFERENCE_STEP_Q16  (((BUCK_SETPOINT_DEFAULT_ADC << 16) + BUCK_SOFTSTART_CYCLES - 1U) / BUCK_SOFTSTART_CYCLES)
#define BUCK_WARMUP_CYCLES       8U
/* Consecutive ADC-overrun cycles tolerated before faulting. A debugger halt lets
 * the ADC free-run and flag OVR once on resume (harmless -> skip); a genuine
 * ISR-too-slow overruns every cycle and trips the fault within OVR_MAX cycles. */
#define BUCK_ADC_OVR_MAX         8U
/* Sample validity. A feedback sample that differs from the last ACCEPTED one
 * by more than this (256 counts = 200 mV in one period: 30 A of capacitor
 * current, not physics) is held off for that cycle: the PI, the load estimate
 * and the trigger use the previous sample. Two consecutive out-of-range
 * samples are accepted (a real collapse). The cycles right after the pulse
 * are exempt (the pulse legitimately moves the voltage by more). Bench
 * 2026-09-25: single zero-count samples drove the voltage PI to its current
 * limit for a period (+500 mV spikes) and fired the trigger on every one. */
#define BUCK_ADC_JUMP_COUNTS     256U
/* Feedback-lost guard (thread): with the loop running and the reference above
 * 200 counts, a feedback sample below a quarter of the reference for longer
 * than this trips BUCK_FAULT_SENSE (outputs off). An open sense wire reads
 * ~0 counts and would otherwise be regulated to the 10 A current limit. */
#define BUCK_SENSE_GUARD_US      500U

/* Timing firmware runs immediately; gate outputs are armed explicitly.
 * Set this to 1 for autonomous startup after bench validation, or write
 * g_buck_command=1 in the debugger while running (0 disables both gates). */
#ifndef BUCK_AUTO_START
#define BUCK_AUTO_START          1U
#endif
#ifndef BUCK_DIAGNOSTICS
#define BUCK_DIAGNOSTICS         0
#endif
/* Write-phase instrumentation (DSB + a second HRTIM CNT read after the compare
 * write, ~12 clocks) is part of DIAGNOSTICS only in voltage mode: in current
 * mode the ISR is within ~10 clocks of the period and the extra instrumentation
 * would itself trip the deadline. Force it with -DBUCK_DIAG_WRITE_PHASE=1. */
#ifndef BUCK_DIAG_WRITE_PHASE
#define BUCK_DIAG_WRITE_PHASE    (BUCK_DIAGNOSTICS && !BUCK_CURRENT_MODE)
#endif
/* Arm the time-optimal response at boot? Voltage mode: yes (with AUTO_START).
 * Current mode: NO by default. The current-mode inner loop already recovers a
 * 1->6 A step to the same minimum as the pulse does (closed-loop simulation),
 * and the every-cycle trigger block costs ~40 clocks the current-mode ISR can
 * barely spare (~243 of 280 estimated with it, ~210 without). Re-arm at run time
 * with g_buck_tune.optimal_en = 1 after checking g_buck_isr_cycles_max. */
#ifndef BUCK_OPTIMAL_ARM_AT_BOOT
#define BUCK_OPTIMAL_ARM_AT_BOOT (BUCK_AUTO_START && !BUCK_CURRENT_MODE)
#endif
/* Optional D4/PB5 pulse: high during ISR, low after the compare write. */
#ifndef BUCK_SCOPE_PIN
#define BUCK_SCOPE_PIN           0
#endif

_Static_assert(BUCK_CPU_HZ % BUCK_SWITCH_HZ == 0U, "PWM period must be exact");
_Static_assert(BUCK_HRTIM_PERIOD >= 96U && BUCK_HRTIM_PERIOD <= 0xFFDFU, "HRTIM x32 limits");
_Static_assert(BUCK_SAMPLE_COMPARE >= 96U && BUCK_SAMPLE_COMPARE < BUCK_WRITE_DEADLINE, "sampling phase");
_Static_assert(BUCK_SAMPLE_COMPARE > BUCK_DT_FINE_TICKS + 64U && BUCK_SAMPLE_COMPARE + 96U < BUCK_DUTY_MIN, "sample in settled HS-on before minimum duty");
_Static_assert(BUCK_DT_COUNTS > 0U && BUCK_DT_COUNTS <= 511U, "deadtime range");
_Static_assert(BUCK_DUTY_MIN_PERCENT > 0U && BUCK_DUTY_MIN_PERCENT < BUCK_DUTY_MAX_PERCENT && BUCK_DUTY_MAX_PERCENT < 100U, "duty percentage range");
_Static_assert(BUCK_DUTY_MIN >= 96U && BUCK_DUTY_MIN < BUCK_DUTY_MAX, "duty range");
_Static_assert(BUCK_DUTY_MIN >= BUCK_DT_FINE_TICKS + BUCK_MIN_GATE_TICKS, "high-side pulse width");
_Static_assert(BUCK_DUTY_MAX + BUCK_DT_FINE_TICKS + BUCK_MIN_GATE_TICKS <= BUCK_HRTIM_PERIOD, "low-side pulse width");
_Static_assert(BUCK_TARGET_ADC > 0U && BUCK_TARGET_ADC < 4095U, "ADC scaling");
_Static_assert(BUCK_SETPOINT_DEFAULT_ADC > 0U && BUCK_SETPOINT_DEFAULT_ADC < 4095U, "setpoint range");
_Static_assert(BUCK_KP_Q16 >= 0 && BUCK_KI_Q16 > 0, "negative feedback PI gains");
/* Includes worst-case previous saturated integrator and a full-scale reversal.
 * Keeps EVERY hot-path operation inside signed 32-bit range. */
_Static_assert((int64_t)BUCK_MAX_Q16 + (int64_t)(2LL * BUCK_KP_Q16 + BUCK_KI_Q16) * 4095LL < INT32_MAX, "PI overflow: reduce gains/Q format");
_Static_assert((int64_t)(BUCK_OPT_IPK_MAX_A * 65536.0) + (int64_t)(2LL * BUCK_CM_KP_Q16 + BUCK_CM_KI_Q16) * 4095LL < INT32_MAX, "voltage-loop PI overflow: reduce gains");
_Static_assert((int64_t)BUCK_MAX_Q16 + (int64_t)(2LL * BUCK_ACM_KP_Q16 + BUCK_ACM_KI_Q16) * 4096LL < INT32_MAX, "current-loop PI overflow: reduce gains (error is Q8 A, |e| <= 16 A)");
_Static_assert(BUCK_ISENSE_GAIN_Q16 * 4095LL < INT32_MAX, "current sense scale");

#endif
