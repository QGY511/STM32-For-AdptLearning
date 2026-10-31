#ifndef BUCK_CONFIG_H
#define BUCK_CONFIG_H

/* NUCLEO-G474RE buck converter, average-current-mode control at 600 kHz.
 *
 * Pins (Arduino names on the Nucleo header):
 *   D7 = PA8  HRTIM1_CHA1 (AF13)  high-side gate, active high
 *   D8 = PA9  HRTIM1_CHA2 (AF13)  low-side gate, complementary with dead time
 *   D4 = PB5  GPIO output         trigger indicator g[k]
 *   A0 = PA0  ADC1_IN1            v_out (direct, 1 V/V), two samples per cycle summed by ADC1
 *   A1 = PA1  ADC2_IN2            inductor current, 40 mV/A, 0 V at 0 A (negative current reads 0)
 *   A2 = PA4  DAC1_OUT1           load-current estimate, 0..3.3 V = 0..10 A
 *
 * Every value below is either from the specification or was chosen by the
 * user; values carried over from the sibling Buck_Voltage_Mode project are
 * marked [BVM]. */

#include <stdint.h>

/* ---- Clock and PWM timing ------------------------------------------------
 * HSI16 /4 x84 = 336 MHz VCO; R/2 = 168 MHz CPU/HRTIM; P/6 = 56 MHz ADC.
 * 168 MHz x 32 / 600 kHz = 8960 HRTIM fine ticks: an exact 600.000 kHz
 * (nominal; the HSI tolerance applies). */
#define BUCK_CPU_HZ              168000000U
#define BUCK_SWITCH_HZ           600000U
#define BUCK_HRTIM_PERIOD        (BUCK_CPU_HZ / BUCK_SWITCH_HZ * 32U)   /* 8960 ticks */
#define BUCK_CYCLES_PER_PERIOD   (BUCK_CPU_HZ / BUCK_SWITCH_HZ)         /* 280 CPU clocks */
#define BUCK_TSW_S               (1.0 / BUCK_SWITCH_HZ)
#define BUCK_TICK_S              (BUCK_TSW_S / BUCK_HRTIM_PERIOD)       /* 186.0 ps */
#define BUCK_ADC_HZ              (BUCK_CPU_HZ / 3U)                     /* PLL P: 56 MHz */
#define BUCK_ADC_CLK_TICKS       (BUCK_CPU_HZ / BUCK_ADC_HZ * 32U)      /* one ADC clock: 96 ticks */

/* Dead time: 50 ns on both edges. DTPRSC = 0 gives 1/(8 x 168 MHz) = 0.744 ns
 * per count; rounded UP so the gap is never shorter than 50 ns:
 * 68 counts = 50.595 ns = 272 fine ticks. */
#define BUCK_DEADTIME_NS         50U
#define BUCK_DT_COUNTS           ((BUCK_DEADTIME_NS * (BUCK_CPU_HZ / 1000000U) * 8U + 999U) / 1000U)
#define BUCK_DT_TICKS            (BUCK_DT_COUNTS * 4U)

/* Duty. The controller's duty D is the PHYSICAL high-side on-time fraction:
 * both dead times count as off-time, so the compare is CMP1 = D*PER + DT and
 * the estimator uses T_on = D*T_sw, T_off = T_sw - T_on.
 * Compare limits [BVM]: CMP1 in 10..50 % of the period, i.e.
 * D in 6.96..46.96 % of physical on-time. */
#define BUCK_CMP_MIN             ((BUCK_HRTIM_PERIOD * 10U + 99U) / 100U)   /* 896 */
#define BUCK_CMP_MAX             ((BUCK_HRTIM_PERIOD * 50U) / 100U)         /* 4480 */
#define BUCK_D_MIN               ((float)(BUCK_CMP_MIN - BUCK_DT_TICKS) / (float)BUCK_HRTIM_PERIOD)
#define BUCK_D_MAX               ((float)(BUCK_CMP_MAX - BUCK_DT_TICKS) / (float)BUCK_HRTIM_PERIOD)

/* ---- ADC acquisition schedule (one switching cycle = [0, T_sw)) -----------
 * v_out: ADC1, HRTIM ADC trigger 1 = Timer A CMP4 and CMP3: two samples per
 *        cycle (user, 27 Sep). Triggered 2x oversampling (TROVS) adds the
 *        two conversions in hardware and raises one EOC per cycle, after the
 *        second: the result is the sum of the two samples. CMP3, the second,
 *        is placed ~100 ns BEFORE the period boundary so the hold falls in
 *        the quiet low-side interval, ahead of the high-side turn-on DT after
 *        the boundary; CMP4 is BUCK_VSAMPLE_GAP_TICKS earlier, still in the
 *        low-side interval at the largest duty. The sum is 2 x v_out[k+1]
 *        (their mean is v_out[k+1], the end of cycle k = the start of cycle
 *        k+1).
 *        In bulb mode the input is tracked from the end of the previous
 *        conversion and held 2.5 ADC clocks (45 ns, the SMP setting) after the
 *        trigger (RM0440 Figure 93), plus the trigger's resynchronisation to
 *        the asynchronous ADC clock.
 * i_L:   ADC2, HRTIM ADC trigger 3 = Master CMP1..CMP4 at T/8, 3T/8, 5T/8,
 *        7T/8. Triggered 4x oversampling (TROVS) accumulates the four
 *        conversions in hardware and raises one EOC per cycle: the result is
 *        the sum of four equally spaced samples, i.e. 4 x the measured
 *        cycle-average current of cycle k. */
#define BUCK_VSAMPLE_N           2U     /* A0 samples per cycle (user, 27 Sep) */
/* Between the two A0 triggers. A conversion takes 15 ADC clocks (2.5 sampling
 * + 12.5), and a trigger that arrives during a conversion is ignored
 * (RM0440), which would pair the samples across cycles from then on. The ADC
 * clock (PLL P = VCO/6) and the HRTIM clock (VCO/2) are locked 1:3, so both
 * triggers of a pair meet the same resynchronisation delay and the ADC sees
 * them exactly 20 clocks apart: 5 clocks (89 ns) idle after the first
 * conversion. 20 ADC clocks = 357 ns. */
#define BUCK_VSAMPLE_GAP_TICKS   (20U * BUCK_ADC_CLK_TICKS)
#define BUCK_VSAMPLE_LEAD_TICKS  538U                                        /* ~100 ns */
#define BUCK_VSAMPLE_CMP         (BUCK_HRTIM_PERIOD - BUCK_VSAMPLE_LEAD_TICKS)  /* 2nd A0 trigger: Timer A CMP3 */
#define BUCK_VSAMPLE0_CMP        (BUCK_VSAMPLE_CMP - BUCK_VSAMPLE_GAP_TICKS)   /* 1st A0 trigger: Timer A CMP4 */
#define BUCK_ISAMPLE_CMP(n)      ((BUCK_HRTIM_PERIOD * (2U * (n) + 1U)) / 8U) /* n = 0..3 */
/* The CMP1 preload must be written before the period rolls over; the write
 * is checked against this counter value (16 CPU clocks of guard). */
#define BUCK_WRITE_DEADLINE      (BUCK_HRTIM_PERIOD - 32U * 16U)
/* An on-time pass reaches the deadline check no earlier than ~90 clocks
 * (~2900 ticks) into the period (ADC1 EOC ~0.16 us, the loop head, then the
 * reads and the PIs: the counter is read 56-57 clocks into the pass by the
 * static count of the 28 Oct build, 58 in the first pass after arming; ~100
 * clocks, ~3200 ticks, before: 67-68 in the 25 Oct build with the arming in
 * run_loop(), 71 in its first pass after arming; 68-70 before that). A lower
 * count means the counter has wrapped: the pass is more than a period late,
 * which the check also treats as a missed deadline. */
#define BUCK_CHECK_EARLIEST      2048U
#define BUCK_WARMUP_CYCLES       8U     /* conversions discarded after start-up */
#define BUCK_TELEMETRY_EVERY     64U    /* g_buck.out, busy_max and the tunables: every 64 cycles (107 us); the stop command is read every cycle */
#define BUCK_RUN_QUIET_PASSES    24U    /* the first telemetry or tunables pass is this pass after arming (passes 1..23 are ordinary) */
/* Arming (run_loop(); arm() until 25 Oct) enables the gates only when its
 * sample is read early in the period (Timer A counter above BUCK_CMP_MIN and
 * below this; normally about 2100-2650 ticks by the 25 Oct count, about
 * 1800-2350 since 28 Oct: the read comes about 9 clocks earlier), so the
 * first control pass still follows within one period; a late arming sample
 * (e.g. after a debugger halt) retries on the next one. The first pass's CMP1
 * write must land before one period plus 264 clocks after the start of the
 * arming period: from the arming check's counter read, 544 clocks minus the
 * read's position, 416 clocks at 4096 ticks (128 clocks).
 * Since 25 Oct (user, after the board's fault 5 on the first pass after
 * arming) the loop's entry runs in the period before the arming sample, and
 * only the arming lies between the two counter reads: 172-195 core clocks to
 * the first pass's guarded counter read (static count of the linked ELF, the
 * first pass starting late), 196-221 with the boot benchmark's wait states,
 * 213-240 pessimistic. At 4096 that path could grow by 176-220 clocks (about
 * 150 with the board's unexplained excess) before the first write missed:
 * the arming path's slack, not the first write's margin. The first pass waits
 * for its own sample at every accepted arming point (its head comes at most
 * 273 clocks into the arming period, pessimistic, before the next end of
 * conversion at about 313) and checks at the normal place in the next period,
 * 126-144 clocks before the deadline (nominal), at 4096 as at the usual
 * arming point (README, CPU budget).
 * The tighter limit at 4096 is the arming CMP1 write itself (no deadline
 * check by design, so no fault 5): 77-88 / 91-103 / 99-112 clocks after the
 * arming check's counter read (static / nominal / pessimistic; arm(): 83-90 /
 * 99-107 / 109-118), so at 4096 it lands 24-37 clocks before 264 into the
 * period (pessimistic), about 13-31 with the board's excess as a proportion.
 * A later write would leave the first enabled period at the previous CMP1
 * (BUCK_CMP_MIN) instead of buck_ctrl_arm()'s value. Whether to lower this
 * limit is open (user, 25 Oct review).
 * Since 28 Oct (scouted 27 Oct; the arming writes CMP1 first, its constants in registers;
 * the same static count): the arming CMP1 write 29 / 33 / 33 clocks after the
 * counter read (static / nominal / pessimistic, both clock counters), so at
 * 4096 it lands 161 clocks into the period (pessimistic), 103 before 264. The
 * path to the first pass's guarded counter read: 138-141 / 147.12-150.12 /
 * 149-152 by counter 2 (a walk of every path of the linked ELF); counter 1
 * (the emulator's executed paths) counts it 3 clocks longer by its convention,
 * 144 / 153.12 / 155 at the longest (README, CPU budget). The first pass then
 * checks about 20 clocks earlier after its sample than in the 25 Oct build,
 * about 146-164 clocks before the deadline (nominal). The limit is unchanged.
 * Until 25 Oct arm(), its return and the loop entry lay between them: 334-365
 * static (Buck_disturb_obs, 25 Oct; 307-333 in BUCK_Adpt_Learning, 24 Oct
 * 334-357, 27 Sep 310-345, 28 Sep 319-340 in Buck_Current_Mode as counted
 * then), 375-408 with the wait states, and on the board about 460-485: it
 * tripped fault 5 on the first pass after arming at a normal arming point
 * (trip_cnt 8608 against 8448). Was 5120 until the trigger denominators added
 * a division to run_loop()'s entry; kept at 4096 when they went and with the
 * 25 Oct fix (user). */
#define BUCK_ARM_LATEST          4096U

/* ---- Power stage (specification) ----------------------------------------- */
#define BUCK_L_H                 330e-9
#define BUCK_C_F                 270e-6
#define BUCK_VIN_V               6.0                  /* constant, no V_in sense pin (user) */
#define BUCK_RL_OHM              0.114                /* r_L (user; [BVM] DC fit): the PI design (tools/pi_design.py),
                                                         the TOC's held duty (V_ref + r_L I_eq)/V_in; the TOC's L-C
                                                         model is lossless (stage 4) */

/* ---- Sensing --------------------------------------------------------------- */
#define BUCK_ADC_VREF_V          3.3
#define BUCK_V_PER_COUNT         (BUCK_ADC_VREF_V / 4096.0)      /* A0 direct: 805.7 uV per count (user) */
#define BUCK_V_PER_VSUM          (BUCK_V_PER_COUNT / BUCK_VSAMPLE_N)   /* per count of ADC1's result (the sum) */
#define BUCK_VREF_COUNTS_DEFAULT 1000U                           /* V_ref = 1000 counts = 0.8057 V (user) */
#define BUCK_ISENSE_V_PER_A      0.040                           /* 40 mV/A (specification) */
#define BUCK_ISENSE_ZERO_V       0.0                             /* A1 at 0 A (user, corrected: not 1.65 V) */
#define BUCK_ISENSE_ZERO_COUNTS  (BUCK_ISENSE_ZERO_V / BUCK_V_PER_COUNT)       /* 0 */
#define BUCK_ISENSE_A_PER_COUNT  (BUCK_V_PER_COUNT / BUCK_ISENSE_V_PER_A)      /* 20.14 mA per count */

/* ---- Measurement correction: removed in stage 3 (30 Oct 2026, user decision 1a) together with the
 * Padé current estimator (alpha_I, tau_c = 50 us); the current PI's feedback is the measured average. */

/* ---- PI gains: textbook 45 deg phase margin (user) --------------------------
 * Discrete-time design, tools/pi_design.py: sampled averaged plant with
 * r_L = 114 mOhm, the 2-cycle sample-to-duty pipeline, and the firmware PI form
 * C(z) = Kp + Ki T z/(z-1), at no load (worst case for the voltage loop). The design's feedback is
 * the measured cycle average of cycle k (read early in cycle k+1): exactly what the current PI is fed
 * since stage 3 (the Padé estimate before); stage 3 checked it on the switched plant as well
 * (pi_design.py --switched, README).
 *   current loop f_ci = 60 kHz (f_sw/10): PM 45.0 deg, GM 3.2 dB
 *   voltage loop f_cv =  6 kHz          : PM 45.0 deg (no load) .. 61.8 deg
 *                                         (3 A), GM 15.6 dB
 * The current loop's 3.2 dB gain margin (f_sw/10 crossover, chosen by the
 * user) is thin; all four gains are live-tunable. */
#define BUCK_KP_I                0.0237243f           /* duty per A */
#define BUCK_KI_I                3338.07f             /* duty per (A s) */
#define BUCK_KP_V                9.3448f              /* A per V */
#define BUCK_KI_V                285353.0f            /* A per (V s) */
#define BUCK_IREF_MIN_A          (-2.0f)              /* current-reference clamp [BVM] */
#define BUCK_IREF_MAX_A          10.0f

/* ---- Trigger score: S = a e_v^2 + b e_v + c (user, 25 Oct) -----------------
 * e_v = v_out - V_ref in V, signed (as before). S rises quadratically with the
 * voltage error in a transient. a, b and c are live-tunable as
 * g_buck.tune.score_a, score_b and score_c; the defaults a = 1, b = c = 0 give
 * S = e_v^2 exactly. Until 25 Oct: S = (e_v/ev_norm)^2 + (dd/du_norm)^2
 * + rho (e_v/ev_norm)(dd/du_norm) with dd = d[k] - d[k-1], the duty change
 * (24 Oct; du = u[k] - u[k-1] before); rho, the two denominators and the dd
 * term are gone. S drives D4 (g = S >= S_th) and nothing else (the load
 * current no longer uses it: see the observer below). a, b and c affect only
 * D4 and out.score, never the control:
 *   - b != 0 makes S depend on the sign of e_v (b > 0: overvoltage raises S),
 *     and with a < 0, b != 0 or c < 0 S can be negative: D4 then stays low
 *     while S_th > S (with any positive S_th);
 *   - c is a constant offset. With b = 0 and a >= 0, S >= c for every e_v, so
 *     c >= S_th keeps D4 high. Otherwise S can fall below c: with a > 0 and
 *     b != 0 it dips to c - b^2/(4a) at e_v = -b/(2a), and with a < 0 it is
 *     below c for any e_v != 0;
 *   - a NaN S (a NaN coefficient, or infinities that cancel, e.g. an infinite
 *     a or b with e_v = 0) keeps D4 low; S = +inf (an infinite coefficient or
 *     an overflow) sets D4.
 * The PLECS score is (e/0.02)^2 + (dd/2)^2 + p3 e dd with e = V_ref - v: its
 * first term is a = 2500 here; there is no dd term, and a linear term k e in
 * PLECS's e would be b = -k. */
#define BUCK_SCORE_A_DEFAULT     1.0f     /* 1/V^2 */
#define BUCK_SCORE_B_DEFAULT     0.0f     /* 1/V */
#define BUCK_SCORE_C_DEFAULT     0.0f
#define BUCK_STH_DEFAULT         1.0f     /* user, 27 Sep (was 0.01): D4 fires at |e_v| >= 1 V with the default
                                             a, b, c; D4 is all it sets */

/* ---- DAC on A2: 0..3.3 V = 0..10 A (user); negative estimates clip at 0 V -- */
#define BUCK_DAC_FULL_SCALE_A    10.0f

/* ---- Load current: fixed-gain observer (user, 25 Oct) ------------------------
 * A2 and g_buck.out.i_load carry the load current from the user's observer, in
 * predict-then-correct order, once per cycle after the CMP1 write:
 *   v^ += a (i_L - i^);  e = v - v^;  v^ += beta e;  i^ -= gamma e
 * with a = T/C (the model constant BUCK_OBS_A), v = v_out[k+1] (this pass's
 * two-sample mean, the end of cycle k) and i_L = ī_L,m[k], the RAW mean of the
 * cycle's four A1 samples (buck_adc_to_i), not the estimator's î̄_L[k], whose
 * dead-time bias (-1.36 / +0.34 A at 1 / 3 A with body diodes, simulated) the
 * observer would pass on. It is exactly two identical first-order stages
 * y += (1 - p)(Î - y) on Î[k] = ī_L,m[k] - (C/T)(v_out[k+1] - v_out[k]) when
 * beta = 1 - p^2 and a gamma = (1 - p)^2 (a double pole at p). It replaces the
 * per-cycle Î and the first-order filter whose step followed the score (24-25
 * Oct), and its tunables tau0 and iload_s_th; the score no longer touches A2.
 * The estimate drives only A2 and out.i_load, never the control.
 * Gains: live as g_buck.tune.obs_beta and obs_gamma (A/V). Their defaults put a
 * double pole at tau = 9.2 us (user, 25 Oct): p = exp(-T/tau) = 0.83430,
 *   beta = 1 - p^2 = q (2 - q) = 0.3039396,  gamma = (1 - p)^2 C/T = q^2 C/T = 4.4478256 A/V
 * (4.447826 to 7 digits; the float default is 4.4478254),
 * q = 1 - p = 1 - exp(-T/tau), evaluated at compile time with a 7-term
 * series (relative error 1.7e-10 here; both defaults round to
 * the same floats as with the exact exponential).
 * Stable region (predict-then-correct). With the model v[k+1] = v[k] + a (i_L[k]
 * - I) and a constant load I, the voltage error x1 = v - v^ and the current
 * error x2 = I - i^ after a pass move as
 *   x1' = (1 - beta)(x1 - a x2),  x2' = gamma x1 + (1 - a gamma) x2,
 * and the same matrix moves the observer's own state (v^, i^), so its poles are
 * the roots of P(z) = z^2 - (2 - beta - a gamma) z + (1 - beta). Jury: P(1) =
 * a gamma > 0, P(-1) = 4 - 2 beta - a gamma > 0 and |P(0)| = |1 - beta| < 1,
 * i.e. with a > 0
 *   gamma > 0  and  0 < beta < 2 - a gamma / 2
 * (which implies beta < 2 and a gamma < 4: gamma < 648 A/V). Nothing more is
 * needed; in particular not 1 - a gamma > -1 (a gamma = 3, beta = 0.25: P(z) =
 * z^2 + 1.25 z + 0.75, both poles at |z| = 0.866). On the boundary (gamma = 0:
 * a pole at z = 1; beta = 0: |z| = 1; beta = 2 - a gamma/2: z = -1) the
 * observer is not stable, so the region is open and the boundary is rejected.
 * The gains are checked where they are (re)loaded, in the tunables pass and at
 * the loop entry, not every cycle (buck_obs_k): a pair with a NaN, or outside
 * the region, is not taken up and the gains in use stay; at the loop entry,
 * which has no previous gains, those are the compiled defaults (since 25 Oct
 * the entry runs in the stopped cycle that sees the RUN command, a period or
 * more before arming). The check is exact in float (buck_obs_gains_ok). The
 * states have no NaN guard (user, 25 Oct). At arming the observer starts from
 * the arming sample: i^ = its i_meas, v^ = its v (buck_obs_seed; arming
 * publishes out.i_load = i_meas). */
#define BUCK_OBS_A               ((float)(BUCK_TSW_S / BUCK_C_F))    /* a = T/C = 1/162 V/A (0.0061728) */
#define BUCK_OBS_TAU_S           9.2e-6                              /* s: the double pole of the default gains */
#define BUCK_OBS_X               (BUCK_TSW_S / BUCK_OBS_TAU_S)       /* T/tau = 0.18116 */
#define BUCK_OBS_Q               (BUCK_OBS_X * (1.0 - BUCK_OBS_X / 2.0 * (1.0 - BUCK_OBS_X / 3.0 * \
                                  (1.0 - BUCK_OBS_X / 4.0 * (1.0 - BUCK_OBS_X / 5.0 * \
                                  (1.0 - BUCK_OBS_X / 6.0 * (1.0 - BUCK_OBS_X / 7.0)))))))   /* 1 - p = 0.16570 */
#define BUCK_OBS_BETA_DEFAULT    ((float)(BUCK_OBS_Q * (2.0 - BUCK_OBS_Q)))                  /* 1 - p^2 = 0.3039396 */
#define BUCK_OBS_GAMMA_DEFAULT   ((float)(BUCK_OBS_Q * BUCK_OBS_Q * (BUCK_C_F / BUCK_TSW_S)))   /* (1 - p)^2 C/T = 4.4478256 A/V */
_Static_assert(BUCK_OBS_X > 0.0 && BUCK_OBS_X <= 0.5, "observer tau out of the series range");
_Static_assert(BUCK_OBS_BETA_DEFAULT > 0.0f && BUCK_OBS_GAMMA_DEFAULT > 0.0f &&
               BUCK_OBS_BETA_DEFAULT < 2.0f - 0.5f * BUCK_OBS_A * BUCK_OBS_GAMMA_DEFAULT, "default observer gains stable");

/* ---- Start-up (user): gates stay off until g_buck.command = 1 is written;
 * then the reference ramps from the measured v_out to V_ref in 10 ms. ------- */
#define BUCK_SOFTSTART_S         10e-3
#define BUCK_SOFTSTART_STEP_V    ((float)(BUCK_VREF_COUNTS_DEFAULT * BUCK_V_PER_COUNT * BUCK_TSW_S / BUCK_SOFTSTART_S))

_Static_assert(BUCK_CPU_HZ % BUCK_SWITCH_HZ == 0U, "PWM period must be exact");
_Static_assert((BUCK_TELEMETRY_EVERY & (BUCK_TELEMETRY_EVERY - 1U)) == 0U, "telemetry interval: power of two");
_Static_assert(BUCK_RUN_QUIET_PASSES >= 2U && BUCK_RUN_QUIET_PASSES < BUCK_TELEMETRY_EVERY / 2U, "quiet passes after arming");
_Static_assert(BUCK_CHECK_EARLIEST < BUCK_WRITE_DEADLINE, "deadline window");
_Static_assert(BUCK_CMP_MIN < BUCK_ARM_LATEST && BUCK_ARM_LATEST < BUCK_WRITE_DEADLINE, "arming window");
_Static_assert(BUCK_HRTIM_PERIOD >= 96U && BUCK_HRTIM_PERIOD <= 0xFFDFU, "HRTIM x32 limits");
_Static_assert(BUCK_DT_COUNTS > 0U && BUCK_DT_COUNTS <= 511U, "dead-time range");
_Static_assert(BUCK_CMP_MIN >= BUCK_DT_TICKS + 96U && BUCK_CMP_MIN < BUCK_CMP_MAX, "duty range");
_Static_assert(BUCK_CMP_MAX + BUCK_DT_TICKS + 96U <= BUCK_HRTIM_PERIOD, "low-side minimum pulse");
_Static_assert(BUCK_ADC_CLK_TICKS == 96U && (unsigned long long)BUCK_ADC_HZ * BUCK_ADC_CLK_TICKS == BUCK_CPU_HZ * 32ULL, "ADC clock in HRTIM ticks");
_Static_assert(BUCK_VSAMPLE_N == 2U, "A0 samples: Timer A CMP4 and CMP3");
_Static_assert(BUCK_VSAMPLE0_CMP > BUCK_CMP_MAX + BUCK_DT_TICKS + 96U, "both v_out samples inside the low-side interval");
_Static_assert(BUCK_VSAMPLE_LEAD_TICKS > 5U * BUCK_ADC_CLK_TICKS, "2nd v_out hold (SMP + up to 2.5 clocks of trigger latency) before the period end");
_Static_assert(BUCK_ISAMPLE_CMP(0) >= 96U && BUCK_ISAMPLE_CMP(3) < BUCK_HRTIM_PERIOD - 96U, "current sample compares");

/* ---- Feed-forward TOC (stage 4, 30 Oct 2026; Inc/buck_toc.h, Inc/buck_toc_start.h, Src/buck_toc.c) ----
 * The pass whose trigger score rises (g 0 -> 1) starts the TOC at once, in the period of its own
 * sample and before its PIs: the high side is forced on (SST; load step-up, e_v < 0: ON-OFF) or off
 * (SRT; step-down, e_v > 0: OFF-ON) right away, the L-C pair is solved while that first span runs, and
 * a software update of the master and Timer A preloads (CR2 MSWU | TASWU) sets the span's end and
 * stretches the period in progress into the TOC frame; a second sets the frame's end, a normal period
 * boundary followed by the first normal period at the held duty. Times in HRTIM ticks of the counter
 * of the period in progress (the frame shares it). */
#define BUCK_TOC_ON_MIN_TICKS    (BUCK_DT_TICKS + 96U)          /* 368: shorter than the dead time is skipped (RM0440 Fig. 227) */
#define BUCK_TOC_OFF_MIN_TICKS   (2U * BUCK_DT_TICKS + 96U)     /* 640: both dead times + the HRTIM minimum */
#define BUCK_TOC_PER_MARGIN_TICKS 2048U
#define BUCK_TOC_PER_MAX_TICKS   (0xFFDFU - BUCK_TOC_PER_MARGIN_TICKS)   /* the frame's last counter value (RM0440 Table 232) */
#define BUCK_TOC_TON_MAX_S       1.5e-6                         /* ON span upper clamp */
#define BUCK_TOC_TOFF_MAX_S      6.0e-6                         /* OFF span upper clamp */
#define BUCK_TOC_TON_MAX_TICKS   ((uint32_t)(BUCK_TOC_TON_MAX_S / BUCK_TICK_S + 0.5))
#define BUCK_TOC_TOFF_MAX_TICKS  ((uint32_t)(BUCK_TOC_TOFF_MAX_S / BUCK_TICK_S + 0.5))
#define BUCK_TOC_IPK_MAX_A       10.0f      /* ON-OFF: the first span ends no later than i0 + (V_in - V_ref)/L t = this */
#define BUCK_TOC_IVAL_MIN_A      (-2.0f)    /* OFF-ON: the first span ends no later than i0 - V_ref/L t = this */
/* The start (its counter read, before the SST) comes no later than START_LAST, so that the first
 * software update (~100 clocks later: the SST, the solve's first span, the words, the guard) lands
 * before the normal periods' backstop CMP2 (below); a score edge after it is not taken
 * (g_buck.toc.rej[LATE]). README "Clock budget". */
#define BUCK_TOC_START_LAST      (BUCK_TOC_CMP2_BACKSTOP - 64U - 32U * 112U)
/* The second update (the frame's end, the A0 pair moved to it) comes after the period's own A0 pair has
 * been triggered (its second trigger, Timer A CMP3, plus the trigger's resynchronisation and margin):
 * moving the pair's second trigger before it fires would split the TROVS group. */
#define BUCK_TOC_SWU2_FIRST      (BUCK_VSAMPLE_CMP + 256U)
/* A compare written by a software update lies at least this far beyond the counter read before the
 * update (the read, the arithmetic, one HRTIM store and the guard's own read: ~20 clocks static, ~26
 * with the HRTIM accesses' wait states, README "Clock budget"; a miss is a LATE reject). */
#define BUCK_TOC_SWU_GUARD_TICKS 1024U
/* The frame's end: after the period's fourth current trigger (master CMP4, unchanged: the ADC2
 * group of the frame stays four samples) and after the moved A0 pair (2458 ticks before PER'). */
#define BUCK_TOC_PER_MIN_TICKS   (BUCK_ISAMPLE_CMP(3U) + 600U)
#define BUCK_TOC_A0_LEAD_TICKS   (BUCK_VSAMPLE_LEAD_TICKS + BUCK_VSAMPLE_GAP_TICKS)   /* 2458: CMP4' = PER' - this */
/* The provisional frame of the first update ends this far after the start: the second update (after
 * BUCK_TOC_SWU2_FIRST and its path, at the latest ~231 clocks after the SST static, ~290 with wait states;
 * README "Clock budget") must come before it (its guard: a miss is fault 9). 31 Oct: 320 -> 336 clocks, the
 * held duty before the wait and the pessimistic path from it. */
#define BUCK_TOC_PER0_REL_TICKS  (32U * 336U)
/* Timer A CMP2 in normal periods, a reset source of the high side (RST1R = CMP1 | CMP2): a no-op there
 * (the high side is off from CMP1 <= BUCK_CMP_MAX + DT on), and the end of a forced on-time if the
 * first update does not come (only a stalled CPU: every guard miss ends the span at once; do not halt
 * with the stage energized). Late in the period so that it never binds a running start. */
#define BUCK_TOC_CMP2_BACKSTOP   (BUCK_HRTIM_PERIOD - 640U)
#define BUCK_TOC_RET_GUARD_TICKS 512U       /* the return words' last store at least 16 clocks before PER' */
/* After the counter read that follows the second update's wait (31 Oct), README "Clock budget":
 *   SWU2_PATH: to the second update: 29-34 clocks static (46 with the OFF-ON tail), up to 9 HRTIM
 *              accesses: 73 with 3 wait clocks each; the moved A0 pair's first trigger lies at least
 *              this far after the read;
 *   RET_PATH:  to buck_toc_ret()'s guard read: 77-82 clocks static (94 with the tail), up to 18 HRTIM
 *              accesses: 148 with 3 wait clocks each.
 * The frame's end PER' at least END_LEAD after the read (a miss of the return words' guard is fault 9). Until 31 Oct the bound was the A0 lead + one guard (3482 ticks, ~93 clocks)
 * against ~158 clocks static to the return words' guard: a frame ending there tripped fault 9. */
#define BUCK_TOC_SWU2_PATH_TICKS (32U * 80U)
/* The OFF-ON tail's earliest end (Inc/buck_toc_start.h), after the same read: the static path (46 clocks)
 * and 18 clocks of wait states, not the pessimistic budget above. A later update only lets the compare
 * pass unseen: the high side then stays on to the frame's end (the frame as before the tail), no fault. */
#define BUCK_TOC_TAIL_LEAD_TICKS (32U * 64U)
#define BUCK_TOC_RET_PATH_TICKS  (32U * 160U)
#define BUCK_TOC_END_LEAD_TICKS  (BUCK_TOC_RET_PATH_TICKS + BUCK_TOC_RET_GUARD_TICKS)
_Static_assert(BUCK_TOC_END_LEAD_TICKS >= BUCK_TOC_A0_LEAD_TICKS + BUCK_TOC_SWU2_PATH_TICKS,
               "the moved A0 pair's first trigger (PER' - A0 lead) comes after the second update");
#define BUCK_TOC_HOLDOFF_PASSES  128U       /* no new request for this many passes after a frame (213 us) */
_Static_assert(BUCK_TOC_PER_MIN_TICKS < BUCK_HRTIM_PERIOD + BUCK_TOC_A0_LEAD_TICKS, "TOC frame minimum");
_Static_assert(BUCK_TOC_START_LAST + 32U * 64U + BUCK_TOC_TON_MAX_TICKS + BUCK_TOC_TOFF_MAX_TICKS < BUCK_TOC_PER_MAX_TICKS &&
               BUCK_TOC_START_LAST + BUCK_TOC_PER0_REL_TICKS < BUCK_TOC_PER_MAX_TICKS, "TOC frame range");
_Static_assert(BUCK_ISAMPLE_CMP(0U) + BUCK_TOC_PER0_REL_TICKS > BUCK_TOC_SWU2_FIRST + BUCK_TOC_SWU2_PATH_TICKS + BUCK_TOC_RET_GUARD_TICKS &&
               BUCK_TOC_PER0_REL_TICKS > 32U * 290U + BUCK_TOC_RET_GUARD_TICKS,
               "the provisional frame outlasts the second update (earliest start: its wait and path; latest: its path)");
_Static_assert(BUCK_TOC_CMP2_BACKSTOP > BUCK_CMP_MAX + BUCK_DT_TICKS + 96U && BUCK_TOC_CMP2_BACKSTOP < BUCK_HRTIM_PERIOD,
               "the CMP2 backstop is a no-op in a normal period");

/* ---- TOC learning: the Jacobian step on the measured end state (stage 4) ------------------------
 * After a frame the first normal period runs at the held duty (the duty that holds I_eq). Two
 * residuals: z_v = v_out at the frame's end - V_ref, and z_i = the first period's measured mean
 * current - I_eq, the landing (stage 4b: the voltage change over the period, (C/T)(v_1 - v_end), is the
 * landing plus the equilibrium error I_eq - I_load; the latter is the equilibrium correction's, below).
 * The gains g = {g_on, g_off} of the order take the Newton step
 *   [dt1; dt2] = -mu J^-1 [z_i; z_v],   g += g dt / t_applied
 * on the L-C Jacobian at the applied spans, |dg| <= STEP, g in [G_MIN, G_MAX]. A first span that was
 * clamped (the backstop limit, or the solve finished after its end) cannot move: the second span
 * alone takes the current residual. Residuals beyond CLIP are clipped, beyond SKIP the frame
 * teaches nothing. */
#define BUCK_TOC_MU              0.25f
#define BUCK_TOC_G_MIN           0.4f
#define BUCK_TOC_G_MAX           2.5f       /* wide: the L-C model has no series loss (the fall is steeper on the board) */
#define BUCK_TOC_G_STEP          0.25f
#define BUCK_TOC_DET_MIN_A       0.5f       /* det J floored at m2 * this / C */
#define BUCK_TOC_ZI_CLIP_A       2.0f
#define BUCK_TOC_ZV_CLIP_V       0.04f
#define BUCK_TOC_ZI_SKIP_A       6.0f
#define BUCK_TOC_ZV_SKIP_V       0.2f
/* The equilibrium correction b (stage 4b, user; Inc/buck_toc.h buck_toc_learn_b): I_eq = the edge's
 * per-cycle estimate + b[order], b += mu_b (I_load - I_eq) after each frame, the same CLIP/SKIP bounds as
 * the current residual. Neither learner runs when the first period's measured mean current is below
 * LEARN_IMIN: A1 reads a negative current as 0, so the four-sample mean reads high when the valley goes
 * below zero (the A1 bias is +0.02 A at 1 A, Part 2's closed loop). */
#define BUCK_TOC_B_MU            0.5f
#define BUCK_TOC_B_MAX_A         3.0f
#define BUCK_TOC_LEARN_IMIN_A    0.5f

#endif
