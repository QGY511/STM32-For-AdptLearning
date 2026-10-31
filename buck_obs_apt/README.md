NUCLEO-G474RE buck converter: PI + load observer + score at 600 kHz, feed-forward time-optimal control
=========================================================================================================

**buck_obs_apt, stage 4 (30 Oct 2026; 4b: the equilibrium correction b; 31 Oct: the edge pass's observer step,
the frame-end timing fix, the OFF-ON tail).** Every switching cycle (600 kHz, no decimation): the cascaded PI
(voltage PI -> average-current reference, current PI on the measured cycle average -> duty), the load-current
observer (A2) and the trigger score S = a e_v^2 + b e_v + c (D4). No Padé estimator (removed in stage 3). On a
rising score edge a **feed-forward TOC**: the high side is forced on (load step-up) or off (step-down) at once,
in the period of the sample that saw the edge, and the time-optimal ON-OFF / OFF-ON pair is solved on the
lossless L-C model while that first span runs. **The TOC is off by default** (`g_buck.toc.toc_enable = 0`): the
firmware then runs the base loop of Part 2. **Nothing has run on the board; nothing was flashed.** The HRTIM
mechanism (software set/reset of the output and software updates of the preloads in the middle of a period)
is not confirmed by RM0440 for every detail: see *Bench tests* before enabling it with the stage powered.

What the user asked for (30 Oct) and where it is
------------------------------------------------
| # | request | implementation |
|---|---|---|
| - | PI, observer and score every cycle at 600 kHz; no Padé estimator | `Inc/buck_ctrl.h`, `Src/buck.c` (unchanged loop; Padé gone since stage 3) |
| - | a look-up table to speed up the TOC if needed | not needed: the solve is one VSQRT (14 clocks) and products of constants formed once per V_ref (`buck_toc_k_init`); a table would not be faster |
| - | simple L-C model / charge balance for t_on, t_off | `Inc/buck_toc.h` (`buck_toc_solve1/2`, formulas below) |
| 1 | no delay: at the trigger cycle turn the top side on at once (step-up) or the bottom side (step-down) for at least the time the solve needs, feed-forward | `Inc/buck_toc_start.h`: the edge test runs before the pass's PIs, the SST/SRT is the first thing after it; the span's end comes from the solve (its minimum is the solve's time, ~0.51 us) |
| 3 | the solve's initial condition: the measured i_L average and v_out | x0 = (i_avg, v_out) of the pass that saw the edge; I_eq = i_avg - (C/T)(v_out - v_out,prev) + b[order] |
| 4 | the Jacobian is updated from the voltage error at the TOC's end and the difference over the first cycle after it; the first duty held | z_v = v_out(end) - V_ref; P1 (and P2, the pipeline) at the held duty (V_ref + r_L I_eq)/V_in. Since 4b the "difference" is split with P1's measured mean current i_P1: the landing z_i = i_P1 - I_eq for the timing (Newton step on g = {g_on, g_off}, `buck_toc_learn`), the equilibrium e = I_load - I_eq, I_load = i_P1 - (C/T)(v_1 - v_end), for b |
| 4b | bring back the additive equilibrium correction: I_eq = estimate + b, b += mu_b (I_load - I_eq); split the work over cycles if needed | `buck_toc_learn_b` (mu_b 0.5, b within +-3 A, one per order); b at once (~10-15 clocks), the Jacobian step in two parts in the spare tails of the next ordinary passes, which the TOC code runs itself (`toc_pass`) |
| 5 | i_ref = the first cycle's average current; no i_ref learner | the voltage PI's integrator := P1's measured mean current, the current PI's := the held duty, at the sample after P1; DeltaI, kappa, q and the old G bank are removed |
| 31 Oct | the edge pass's observer step in the edge cycle itself (not deferred to the TOC code) | `Src/buck.c` run_pass: after the start (the frame's words and the return words are in), the observer step on the edge's sample, A2 and D4, as an ordinary pass makes them (~21 clocks, inside the frame, on every path of the start); `buck_toc_finish` no longer steps it |

The sequence (Inc/buck_toc_fw.h)
--------------------------------
Pass n reads the sample at the end of period n-1 (EOC ~1150 ticks into period n).
1. **Edge (inline in the HOLD pass, before its PIs).** trig (the previous pass had g = 0, the TOC is enabled,
   the hold-off is over) and S(v_out) >= S_th: the start (`buck_toc_start_inline`). The pass writes no CMP1
   (the frame replaces the period it would be for; the PIs restart after the frame). Not in the first pass
   after arming, the pass after a tunables pass, or during a reference ramp (those copies have no edge test).
2. **The first span, now.** The peak (ON) / valley (OFF) limit from i0 (a few clocks), then ON-OFF (v < V_ref):
   SST, the high side on after DT; OFF-ON: SRT, the high side off. Until the first update the span's end is the
   backstop every normal period carries: Timer A CMP2 = 8320 ticks in RST1R (a no-op in a normal period; it only
   matters if the CPU stalls).
3. **The solve's first span** (one VSQRT), clamped to the limit; the provisional frame (the order's SET1R/RST1R,
   PER' and MPER' far away); the counter; the span's end CMP2 = c_s + DT + t1 (ON-OFF) / c_s + t1 - DT (OFF-ON),
   no earlier than the counter + 1024 ticks; **first software update** (CR2 = MSWU | TASWU), guarded: before the
   span's end and before the backstop. A miss ends the forced span at once (SRT) and restores the normal words:
   a LATE reject, no fault.
4. **The second span** (as solved, or, when the first span came out longer than solved or at its limit, the one
   that lands the current on the target valley from where the first ended), the frame's end PER' (ON-OFF:
   CMP2 + t2 - DT; OFF-ON: CMP2 + DT + t2, the high side then stays on into P1), the held duty for P1 and P2;
   then the counter (`c2`, after the period's own A0 pair has been triggered: counter >= 8678, moving its second
   trigger first would split the TROVS group). PER' no earlier than c2 + `BUCK_TOC_END_LEAD_TICKS` (5632 ticks:
   the second update's and the return words' path, below); the A0 pair moved to PER' - 2458 / PER' - 538;
   **second software update**, before the provisional end. **The OFF-ON tail** (31 Oct): when PER' had to move
   later in an OFF-ON frame, the high side would stay on to the moved end; instead the ON span ends as solved
   (no earlier than c2 + 2048 ticks), through Timer A CMP1 as its reset (RST1R = CMP1 from this update), and the
   rest of the frame runs with the low side on, P1 as after ON-OFF (`sat` 16).
5. `buck_toc_ret()` right after the update (BUCK_TOC_CALL_CORE: no floating point in it, so no FP save): the
   return words for P1 (normal period, CMP1 = the held duty; after OFF-ON without DT: the high side is on across
   the end), guarded against PER' - 512 (miss: fault 9); then the period's own A0 result, converted inside the
   frame, is read and dropped (`toc.v_mid`). Then, still in the edge pass (`run_pass`): the frame's record, and
   the pass's observer step on the edge's sample, A2 and D4 (g = 1), as in an ordinary pass.
6. `buck_toc_finish()` (after run_loop copied its state into the mailbox): **T** (the frame's end sample, read in
   P1): the frame-end transfer seen (MUPD, UPD), the grid locked (Timer A - master counter), CMP1 for P2 = the
   held duty (guarded, fault 5 on a miss), z_v, the observer's v^ set to it; then, in P1's idle time, the checks of
   debugger writes to `g` and `b` (`toc_gains_take`);
   **1** (the sample after P1, read in P2, `toc_pass`): integ_v := P1's measured mean current, integ_i := the held
   duty, an ordinary pass (both PIs, CMP1 for P3 guarded, score, observer, A2, D4); the residuals z_v, z_i, e and
   the equilibrium correction b at once. **Learning in parts** (with `learn_enable`, P1's mean current >=
   `BUCK_TOC_LEARN_IMIN_A` 0.5 A: A1 reads a negative valley as 0): the Jacobian step's span steps (part 1) and its
   gains (part 2), each started only with the counter in [2048, 7488) and no sample pending; otherwise the TOC code
   runs the next sample's ordinary pass first (`toc_pass`, at most 4). Typically: P3's pass, part 1 in its tail,
   P4's pass, part 2 in its tail, then back to run_loop before P5's sample. The new gains and b take effect at the
   next edge. A new request needs g to fall and BUCK_TOC_HOLDOFF_PASSES (128) passes (the telemetry pass re-enables).
The master's four current compares are untouched: they all fire once in the stretched period (ADC2's group stays
four samples; its frame result is not used, and A1 cannot read negative current anyway).

The solve and the learning (Inc/buck_toc.h)
-------------------------------------------
Lossless L-C, the output at V_ref during the pulse: ON di/dt = m1 = (V_in - V_ref)/L, OFF -m2 = -V_ref/L. a =
i0 - I_eq = (C/T)(v_out - v_out,prev) - b, dv = V_ref - v0, h = half the ripple at V_ref/V_in. Target: the periodic
steady state at a period start, (I_eq - h, V_ref), so P1 is an ordinary period.
- ON-OFF: x^2 = k_dv dv + (V_ref/V_in) a^2 + ((V_in - V_ref)/V_in) h^2, t_on = (x - a)/m1, t_off = (x + h)/m2;
- OFF-ON: y^2 = ((V_in - V_ref)/V_in) a^2 + (V_ref/V_in) h^2 - k_dv dv, t_off = (a + y)/m2, t_on = (y - h)/m1;
k_dv = 2 C (V_in - V_ref) V_ref/(L V_in). Applied: t = g t*, g[order] = {g_on, g_off}.
Learning (with `learn_enable`), three measured residuals after P1 (held duty), three parameters:

| residual | measured | moves |
|---|---|---|
| z_v (V) | v_end - V_ref | timing gains g |
| z_i (A), the landing | i_P1 - I_eq (P1's measured mean current against the planned equilibrium) | timing gains g |
| e (A), the equilibrium | I_load - I_eq, I_load = i_P1 - (C/T)(v_1 - v_end) (P1's mean current minus its capacitor current) | b |

The voltage change over P1 alone, (C/T)(v_1 - v_end) = z_i - e, is both errors in one number; with i_P1 they
separate: a wrong I_eq lands as planned (z_i, z_v ~ 0) but shows in e; a timing error moves z_i and z_v, not the
measured load. Timing: the L-C end map z_i = a + h + q1 t1 + q2 t2, z_v = -dv + (a (t1 + t2) + q1 t1^2/2 + q1 t1 t2
+ q2 t2^2/2)/C (q = the spans' slopes); Newton step [dt1; dt2] = -mu J^-1 [z_i; z_v] at the applied spans, g += g
dt/t, mu 0.25, |dg| <= 0.25 per frame, g in [0.4, 2.5] (wide: the board's fall is steeper than V_ref/L). A first
span that was clamped (its limit, or longer than solved) cannot move: the second span alone takes z_i. Two
divisions (1/det, 1/(t1 t2)). Equilibrium: b[order] += 0.5 e, |b| <= 3 A. What b takes: the systematic error of the
edge estimate (a load still slewing when the edge fires, the ESR term of the edge cycle's dv, the edge's place in
its period). Not a current-sense offset or gain: those are in I_eq and I_load alike and cancel (and the pulse
depends on current differences only). Gates for both: |residual| >= 6 A or |z_v| >= 0.2 V (or not finite):
nothing learned; clips 2 A / 40 mV; P1's mean current below 0.5 A: nothing learned (`learn_low`).

Clock budget (static count of the linked ELF; 1 clock per instruction, loads 2, VSQRT 14, taken branch 3, no
peripheral wait states: add ~3 per HRTIM read, the bench of Part 2 measures them)
---------------------------------------------------------------------------------------------------------------
| path | clocks | where in the period |
|---|---|---|
| normal pass, head -> CMP1 write: HOLD copy (with the edge test, TOC enabled) / other copies | 74 / 57-61 | (stage 3: 62 / 57-61) |
| edge pass: head -> the start's counter read | 48 | ~2700 ticks |
| -> SST (ON-OFF) / SRT (OFF-ON) | 19 / 21 | **~3350 ticks (0.62 us into the period, ~0.4 us after the sample's EOC)** |
| SST -> the solve's counter read (solve with b, provisional words) | 62 | |
| -> the first update's guard read | 20 | first update ~6100 ticks (limit 8256) |
| minimum first span (solve + guard 1024 ticks - DT) | | **~2740 ticks = 0.51 us** |
| first update -> the wait for counter 8678 (second span, the held duty), ON-OFF / OFF-ON | 87 / 94 | the wait does not wait: c2 ~9000 / ~9150 ticks |
| c2 -> second update (ON-OFF / OFF-ON / OFF-ON with the tail) | 34 / 29 / 46 | budget `BUCK_TOC_SWU2_PATH_TICKS` 80 clocks (with ~3 wait clocks per HRTIM access) |
| c2 -> the return words' guard read (`buck_toc_ret`) | 82 / 77 / 94 | budget `BUCK_TOC_RET_PATH_TICKS` 160 clocks, + the guard's 512 ticks: PER' >= c2 + 5632 ticks (~14700) |
| after the return words: the record, the edge pass's observer step, A2, D4 | 65 (observer ~21) | ~13900 ticks, inside the frame |
| T: sample read -> CMP1 (P2) write | ~50 | ~2900 ticks into P1 (deadline 8448) |
| 1 (restart pass, `toc_pass`): sample read -> CMP1 (P3) write / -> its end | 90 / +71 | CMP1 ~4100, end ~6650 ticks into P2 |
| residuals, telemetry, b | ~106 | ends ~1050 ticks into P3 |
| ordinary pass run by the TOC code (`toc_pass`) | 90 / +71 | CMP1 ~4100, end ~6650 |
| learning part 1: gates, clips, Jacobian, 1/det, the span steps | ~96 | in P3's tail (start window 2048..7488) |
| learning part 2: 1/(t1 t2), the gains, stores | ~90 | in P4's tail; back to run_loop before P5's sample |

Without the split the Jacobian step (~190 clocks) after the restart pass would end ~170 clocks into P3 and push
P3's CMP1 write to within a few clocks of its deadline (more with wait states). The checks of debugger writes to
`g` and `b` (`toc_gains_take`, ~100 clocks) run in P1 after T's CMP1 write (31 Oct; before the frame's end they
could push a short frame's T pass toward its deadline).

**31 Oct, on the board: the start runs ~1.4 x slower than counted, and a tail compare was missed (+27 A).** The
first frame on the board (an OFF-ON, `score_a` 100, `sat` 29) had its SST at 4352 ticks (counted: ~3350) and its
read after the wait at 12416 (`per_ticks` 18048 - 5632; counted: ~10200): 252 clocks from the SST against 185. The
second update then came at the tail's compare (12416 + 2048) and missed it; the high side stayed on from ~10130
ticks to P1's CMP1, ~1.7 us, and P1's current was ~26 A (`z_i` 27 A, `i_first` clamped at 10); a fault 4 followed in
the loop (not yet located: `toc.f4_*`). Since then: the ON span is ended at once by software (SRT, `sat` 32) when
its solved end is not at least 80 clocks (`BUCK_TOC_TAIL_LEAD_TICKS`) beyond the read; else by the compare, and the
counter is checked right after the update: near or past the compare, SRT (`sat` 64). The budgets are the static
paths x ~1.5: the return words 192 clocks (`BUCK_TOC_END_LEAD_TICKS` 6656), the provisional frame 384 clocks after
the SST (at 336 that frame's update would have come after it with the check), the tail's lead 80. `toc_gains_take`
runs only with a part's time to spare at the episode's end (in P1 it delayed every pass after T). The Timer A
counter at each step is now recorded (`toc.c_swu1`, `c2`, `c_swu2`, `c_ret`, `c_t`, `c_first`, `c_pass`, `c_back`,
`csr_back`): the next frame on the board measures the path instead of the count.

**31 Oct: the frame's end was too early for the return words.** Until 31 Oct PER' had to be only the A0 lead + one
guard (3482 ticks, ~93 clocks) after c2, but from c2 to the return words' guard read were ~158 clocks static (the
second update, the record stores, BUCK_TOC_CALL's FP save, the held duty inside `buck_toc_ret`), and the second
update's own path (35 clocks) had no margin against the moved A0 trigger: a frame ending within ~0.4 us of that
bound (every OFF-ON frame of a down step up to ~2 A in the simulation, which does not model this timing) would
have tripped fault 9. Now the held duty is formed before the wait, `buck_toc_ret` comes right after the update
without an FP save (82 clocks instead of 158), and the bound is sized to the path (`make check` rule 5d: the call
within 12 instructions of the update, no FP register in `buck_toc_ret`). The frame's minimum moved later (c2 +
5632 instead of + 3482), which made the OFF-ON frames' stretched ON span visible: on a 1 A down step the hump was
248 mV against the PI's 70 mV (the high side on to the moved end), hence the tail.
`BUCK_TOC_START_LAST` = 4672 (a later edge is LATE), the guard 1024 ticks (`BUCK_TOC_SWU_GUARD_TICKS`).
`make check` (rule 5d) fails if the forced output store does not precede the solve's square root, if
`buck_toc_ret` is not called through BUCK_TOC_CALL_CORE within 12 instructions after a guarded second update or
uses a floating-point register, or if the OFF-ON tail compare (CMP1, then RST1R = CMP1; exempt from rule 3's
deadline form) is missing or not followed by a guarded update. Each of these was broken once on purpose and fails
it (31 Oct; also the simulation without the tail fails its 1 A check).

Host simulation (`make toc-test`, the switched plant of Part 2 with r_L = 114 mOhm and body diodes, the
firmware's frame timing; S = (e_v/20 mV)^2)
---------------------------------------------------------------------------------------------------------
| case | dip / hump, PI alone | dip / hump, TOC |
|---|---|---|
| 1 -> 3 -> 1 A | 132 / 132 mV | 80 / 51 mV (up: LATE, the solved on-time is below the minimum first span; down: the tail) |
| 1 -> 6 -> 1 A | 304 / 307 mV | 166 / 74 mV (peak i_L 9.6 A) |
| 1 -> 3 -> 1 A, ADC noise +-2 LSB | | 88 / 59 mV |
| 1 -> 2 -> 1 A | 72 / 70 mV | 72 / 41 mV (down: the tail; with the ON span to the moved end 248 mV) |
| 1 -> 2.5 / 1 -> 1.5 A (not in `make toc-test`) | 104 / 102, 36 / 34 mV | 79 / 43, **63** / 39 mV |

The 0.5 A step up is worse with the TOC than without: its ideal on-time is far below the minimum first span
(0.51 us), and the second span cannot take the excess out in time (Known limits).

With the start's timing measured on the board (31 Oct: SST at 4352, the read after the wait 8064 ticks later, the
solve's read 84 clocks after the SST; `make toc-test` runs both timings):

| case | PI alone | TOC (board timing) | learned (16 steps) |
|---|---|---|---|
| 1 -> 2 -> 1 A | 72 / 70 mV | **96** / 22 mV | not learned: P1's current 0.35 A, below the gate |
| 1 -> 3 -> 1 A | 132 / 132 mV | 114 / 34 mV | 26 / 34 mV |
| 1 -> 6 -> 1 A | 304 / 307 mV | 170 / 65 mV | |
| 10 us slewing (2 A) | | 137 / 44 mV | 31 / 35 mV |

The first span cannot be shorter than ~0.64 us there: every step up drives the current to the 10 A limit before
the second span takes it back. The down steps gain from the tail (humps 22-34 mV).

Learning (timing and b) over 16 load cycles, the last step against the same step without learning (dip / hump):

| case | without learning | learned | g_off (ON-OFF) | b up / down |
|---|---|---|---|---|
| 1 -> 3 -> 1 A | 80 / 51 mV | **25 / 49 mV** | 0.64 | -0.00 / -0.02 A (nothing to correct) |
| plant L -20 % | 80 / 48 mV | 26 / 51 mV | 0.67 | -0.09 / +0.12 A |
| 10 us slewing edges | 98 / 75 mV | **26 / 49 mV** | 0.64 | +0.39 / -0.36 A (e 0.42 -> -0.03 A) |
| 2 A / 5 A alternating (the 5 A step) | 166 / 74 mV | 54 / 63 mV | 0.61 | +0.41 / -0.39 A |
| 5 mOhm ESR | 31 / 20 mV | 69 / 29 mV | 0.74 | -2.14 / +1.03 A (e -2.88 -> 0.07 A) |

The 2 A down step's frame is valley-limited and now ends with the tail, so its timing has nothing to learn
(z_v ~+27 mV stays). b alone moves, and with L -20 % its correct I_eq removes a small bias that had covered part
of that shortfall: the hump 48 -> 51 mV (the check allows 10 %). These results depend on the frame's minimum, and
not monotonically: with a 144-clock budget instead of 160 the 2 A down frames end without the tail and the slewing
case learns to 43 / 63 mV.

The unlearned 2 A step is LATE: the first span is the solve's minimum, and the second span removes the extra
current by the lossless model's fall, which the plant's (steeper, with r_L) overdoes: P1 lands 1.4 A below I_eq
and the voltage falls further. Learning g_off removes that.

With ESR the edge cycle's dv carries ESR x the load step, so the unlearned I_eq is ~2-3 A high and the pulse is
oversized, which happens to cover the lossless model's charge shortfall; b removes the bias (its job: I_eq ends at
the measured load) and the dip goes back to the model's. An ESR term in the solve (the droop and z_v corrected by
ESR (i_s - I_load), as STM32Prototype has) would be the next step. On a 2 A step the solved on-time is shorter
than the minimum first span (LATE), so g_on is not learned there. OFF-ON: the first span ends at the valley limit
(-2 A, `BUCK_TOC_IVAL_MIN_A`) and the second at its minimum: z_v stays ~+30 mV; a lower valley limit removes more
charge (the stage's reverse current is the user's call). (The simulation learns at the restart pass; the firmware's
parts finish two periods later, still long before the next edge, 128 passes on.)

Known limits
------------
- The minimum first span is the solve's time (~0.51 us): smaller ideal on-times overshoot (then the second span
  lands the current; z_v carries the excess). A 0.5 A step up comes out worse than with the PI alone (63 against
  36 mV in simulation).
- The frame lasts at least until c2 + 6656 ticks (c2 ~12400 on the board): a shorter solved frame is stretched,
  ON-OFF with a longer fall, OFF-ON with the tail (the ON span as solved or ended at once, then the low side on).
- On the board the start barely fits its period: SST ~4352 (`BUCK_TOC_START_LAST` 4672), the first update ~8190
  (its limit 8256 before the backstop): expect LATE rejects (`rej[0]`). The minimum first span is ~0.64 us: a small
  step up overshoots to the 10 A limit (the 1 A step up worse than the PI alone, and not learnable).
- The edge estimate can be far off on the board: that frame had I_eq = -0.88 A (a negative load) from one
  cycle's dv after a late trigger (`score_a` 100: |e_v| >= 100 mV, when the PI had already cut the current).
- x0 is the sample at the period start (~0.6 us before the SST): the droop in between is not in the model (the
  learner takes the systematic part).
- The lossless model's fall is too shallow on the board (r_L i): g_off learns it (~0.7 in simulation).
- The edge pass runs no PI; a telemetry or tunables pass that falls in an episode is skipped (the next comes 64
  passes later).
- `g_buck.toc.g` and `.b` written from the debugger are taken up after the next edge (checked: g outside [0.4, 2.5]
  -> 1, b outside [-3, 3] A -> 0).
- No ESR term in the model: with a few mOhm of ESR, b removes the edge estimate's ESR bias, which an unlearned frame
  happens to profit from (table above).
- The TOC code runs two to three ordinary passes itself after each learned frame (the same PI code; a telemetry
  or tunables pass that falls there is skipped).

Enabling (Live Expressions, after RUN)
-------------------------------------
1. Make the score fire on the transients you want: e.g. `g_buck.tune.score_a = 2500` (S = (e_v/20 mV)^2,
   S_th = 1); the default a = 1 means |e_v| >= 1 V.
2. `g_buck.toc.toc_enable = 1` (taken up at the next telemetry pass); `g_buck.toc.learn_enable = 1` for the
   Jacobian step and b.
3. Watch `g_buck.toc.edges`, `frames`, `rej[0..2]` (LATE, K: constants for a new V_ref being formed, NONE: no
   pair), `sat` (1 first span at its limit, 2 longer than solved, 4 second span at a limit, 8 frame end moved
   later, 16 with it the OFF-ON tail), `c_start`, `t1_ticks`, `t2_ticks` (with the tail: the ON span applied), `t1_model`, `t2_model`, `z_v`, `z_i` (landing), `i_load`, `e_b`, `b`,
   `i_first`, `g`, `learn_n`, `b_n`, `learn_low`, `learn_late`, `learn_skip`, `chk_commit`, `chk_align`. A fault 9 (`BUCK_FAULT_TOC`) means a frame write window or a hardware check failed.
4. A fault 4 (`BUCK_FAULT_ADC`, 31 Oct diagnostics): `toc.f4_site` says where (0 the loop's own sample timeout, the
   CPU then spins in `buck_fatal`; 1 a pass of the loop, bad sample; 2 T, the frame-end sample, bad; 3 the restart
   or a learning pass, bad; 4 the TOC code's sample timeout; 5 the period's own A0 result not converted inside the
   frame), `toc.f4_bad` which check (0x10 ADC1 overrun, 0x100000 ADC2 overrun, 0x40000 ADC2 without a result),
   `toc.f4_csr` and `toc.f4_cnt` the ADC status and the Timer A counter there; with `toc.state`, `edges`,
   `frames`, `sat`, `c_start`, `per_ticks` and `adc_miss` (counted for sites 1-3 only).
5. The frame's timing on the board (Timer A counter, ticks): `toc.c_start` (SST), `c_swu1`, `c2`, `c_swu2`,
   `c_ret` (the frame's counter), `c_t` (T's CMP1, in P1), `c_first` (the restart pass's, in P2), `c_pass`, `c_back`
   and `csr_back` (where the loop takes over; 0x4 set: its sample already waiting).
`tools/live_expressions.txt` has the rows.

HRTIM configuration (all builds)
--------------------------------
The master is preloaded with update at its roll-over (PREEN | MREPU) and Timer A updates on the master update
(MSTU); RST1R = CMP1 | CMP2 with CMP2 = 8320 (the TOC's backstop; a no-op in a normal period). With equal periods
(always) the roll-over is the same as in Buck_disturb_obs.

Bench tests (RM0440 items the manual does not confirm; gates first at low bus voltage or disconnected)
-----------------------------------------------------------------------------------------------------
- B1 SST / SRT in the middle of a period act at once and go through the dead-time unit (CHA2 off before CHA1
  on after DT; CHA1 off before CHA2 on): scope PA8/PA9 on a forced frame.
- B2 a software update (CR2 TASWU | MSWU) mid-period transfers PER, CMP2..4, SET1R/RST1R at once in both
  timers, keeps the counters locked (`chk_align = 0` over 10^5 frames), and a CMP2 written ahead of the counter
  fires; a PER' beyond the old PER stretches the running period (no roll-over at 8960).
- B3 the frame's end transfers the return words (`chk_commit = 0`), and the period's own A0 pair converts once
  inside the frame (`v_mid`), the frame-end pair once after it; count ADC1/ADC2 EOCs per frame, no OVR.
- B4 the normal-period CMP2 reset at 8320 never shows on CHA1 (the high side is already off).
- B5 (31 Oct) a small load step down (~1 A, `sat` 25 = 16 + 8 + 1): CHA1 goes off at the tail compare (Timer A
  CMP1 written in the second update with RST1R = CMP1, `t2_ticks` after the ON span's start), CHA2 on from there
  to PER', and P1 starts as a normal period (CHA1 on after DT at its start).
- No DBGMCU freeze of HRTIM: a debugger halt inside a frame leaves the forced span to the backstop (gates on).
  Halt only with `toc_enable = 0`.

Verification
------------
- `make` (0 warnings), `make check` (rules 1, 2, 4 unchanged; rule 3 exempts the OFF-ON tail compare since
  31 Oct; rule 5 rewritten for stage 4, self-tested by building the forced output after the solve: it fails;
  the 31 Oct additions likewise, see *Clock budget*), `make test` (the base loop's suite, the Padé sections removed:
  passes), `make toc-test` (the solve against an L-C integration in double, the Jacobian step on mismatched
  L-C plants and in two parts = in one bit for bit, the equilibrium correction, the closed loop above: passes).
- Not done: the differential emulator of stages 2/2b (not in this repository), the board.

Files (stage 4)
---------------
New: `Inc/buck_toc.h` (solve, learning, held duty), `Inc/buck_toc_start.h` (the inline start), `Tests/test_toc.c`
(rewritten). Rewritten: `Src/buck_toc.c` (return words, finish, prime), `Inc/buck_toc_fw.h` (mailbox, guards).
Changed: `Src/buck.c` (edge test before the PIs, the trigger word, RUN_TOC, CMP2/RST1R), `Inc/buck.h` (the TOC
block), `Inc/buck_config.h` (TOC constants, the old frame and learning settings removed), `Tests/test_ctrl.c`
(Padé sections removed), `tools/check_loop.py` (rule 5), `tools/live_expressions.txt`, `Makefile`. Removed:
`Src/buck_toc_plan.c`, `Inc/buck_toc_plan.h`, `Src/buck_learn.c`, `Inc/buck_learn.h`, `Inc/buck_toc_frame.h`,
`Tests/reference_toc.h` (the stage-2/3 planner, the DeltaI/kappa/q/G learners).
31 Oct: `Src/buck.c` (the edge pass's observer step, A2, D4), `Inc/buck_toc_start.h` (the held duty before the
wait, the frame's minimum from `BUCK_TOC_END_LEAD_TICKS`, the OFF-ON tail, the return words right after the second
update, the record after them), `Src/buck_toc.c` (`buck_toc_ret` without floating point, the observer step and
`toc_gains_take` out of the frame's end), `Inc/buck_toc_fw.h` (BUCK_TOC_CALL_CORE, `ret_cmp1`, `t_end`,
`v_mid_code`), `Inc/buck_toc.h` (`BUCK_TOC_SAT_TAIL`), `Inc/buck_config.h` (`SWU2_PATH`, `RET_PATH`,
`END_LEAD`, `TAIL_LEAD`, `PER0_REL` 320 -> 336 clocks), `tools/check_loop.py` (rule 3: the tail compare;
rule 5d: the core-only call, its place, no FP in `buck_toc_ret`, the tail), `Tests/test_toc.c` (the frame's
minimum and the tail as the firmware has them, the 1 A step).

---------------------------------------------------------------------------------------------

Part 2: the base controller (Buck_disturb_obs's README, as of 29 Oct; unchanged code paths)
==========================================================================================
Everything below describes Buck_disturb_obs; it applies to buck_obs_apt with the TOC off, except: the
project/binary names, `g_buck` is 400 B (the TOC block appended), the HRTIM update configuration (Part 1),
Timer A CMP2 = 8320 in RST1R (the TOC's backstop, a no-op in a normal period), the HOLD pass's edge test
before its PIs (Part 1, "Clock budget"), `make check`'s rule 5 and, since stage 3, **no Padé estimator**:
the current PI's feedback is the measured four-sample cycle average ī_L,m[k] (the observer's input), the
84 KB coefficient table and alpha_I are gone (the fields `tune.alpha_i`, `st.iv/iv_pred/avg_pred/kappa`,
`out.eps/i_peak` stay for the layout and are unused; `out.i_avg = out.i_meas`). Where the text below
describes the estimator, it is history.

NUCLEO-G474RE buck converter — average-current-mode control at 600 kHz, load-current observer
==============================================================================================

**Buck_disturb_obs** is a copy of BUCK_Adpt_Learning made on 25 October, after that day's score change and its "fix base" stage. One change (user, 25 Oct):
- **Load current on A2:** the user's fixed-gain observer, in predict-then-correct order, fed v_out[k+1] and the raw measured cycle average of the inductor current, replaces the per-cycle estimate Î[k] and the first-order filter whose step followed the score. Its two gains are live (`tune.obs_beta`, `tune.obs_gamma`), default to a double pole at 9.2 µs, and are checked against the observer's stable region whenever they are reloaded. `tune.iload_tau0`, `tune.iload_s_th` and `out.i_load_alpha` are gone, and so are the per-cycle Î (`buck_load_estimate`) and the controller state's previous v_out (`buck_state_t.v`, i.e. `g_buck.st.v`; not the `st.v` of `run_pass()`, this pass's v_out, which feeds the observer), which only Î used. See *Load current: fixed-gain observer*.

Everything else is BUCK_Adpt_Learning as of 25 October: the Padé estimator, both PIs, the trigger score S = a·e_v² + b·e_v + c (`tune.score_a/b/c`, defaults 1, 0, 0), which now drives D4 only (g = S ≥ S_th), and the loop structure. The two projects are independent. BUCK_Adpt_Learning itself is a copy of Buck_Current_Mode as of 28 September, made on 24 October; its changes (user, 24–25 Oct, from the PLECS model `pade_test_preload_learning_adaptive_duty_endpoint_learner_trim_decoupled_sth4_epload_reversal_eventbased_score_damped_learning_no_replan.plecs`) were the score (until 25 Oct with a duty-change term Δd, ρ and two denominators) and the score-damped filter on A2, which this project replaces.

Its CubeIDE settings (`.cproject`, `.settings`, the debug launch: BUCK_Adpt_Learning's, with the project name replaced) were installed on 25 October with CubeIDE closed. On the first board run that evening (CubeIDE Debug build) it tripped fault 5 on the first pass after arming. The fix of the same evening (user: the loop's entry before the arming sample, the arming inside the loop, timing stamps in `g_buck`; see *CPU budget*, 25 Oct evening; bin e5c7798f…) ran on the board on 27 Oct: the user reported that it works (no data from that run were read). The optimisation of 28 Oct (fewer clocks, the same results bit for bit but for one exception, 5 debugger-stall scenarios in which the 25 Oct evening build trips its own fault 5, see *Status* and *Verification*; see *CPU budget*, 28 Oct; bin 5b99a0d5…: the applied 2f144a53… with the observer's prediction pinned in the post pass) has not run on the board yet. The Makefile builds it as described below. The rest of this file is the Buck_Current_Mode / BUCK_Adpt_Learning description; where the observer makes it out of date it has been updated, and history is kept where it explains the numbers.

Register-level firmware (CMSIS only, no HAL) that performs the following every switching cycle, at 600 kHz, with no decimation:

1. the PI update and its output u[k];
2. the Padé-based inductor-current estimator with measurement correction, giving the cycle-average estimate î̄_L[k], and the load-current observer (on A2; see *Load current*);
3. the trigger score S[k] and the indicator g[k] (on D4).

Status (Buck_disturb_obs, 28 Oct): the code compiles without warnings under the Makefile; `make check` and the host tests with the simulated closed loop pass, and the 28 Oct build gives the same results as the 25 Oct evening build bit for bit in the differential emulator, with one exception (see *Verification*): 5 debugger-stall scenarios in which the slower 25 Oct evening build trips its own fault 5. A second difference, the NaN payload of `out.i_load` in one narrow case (the measured current overflowing in a tunables pass, then NaN sensing tunables), is gone: the user had the observer's prediction pinned in the post pass (28 Oct, second set; bin 5b99a0d5…, 0 clocks). On the board (25 Oct, CubeIDE Debug build of the code before the fix) it tripped fault 5 on the first pass after arming. The fix (25 Oct evening, with the timing stamps; bin e5c7798f…) ran on the board on 27 Oct: the user reported that it works; no data from that run were read. The 28 Oct optimisation (bin 5b99a0d5…; 2f144a53… before the pin) has not run on the board yet. Before running it, rebuild the Debug configuration in CubeIDE (or relaunch with build-before-launch) and flash it; restarting inside a debug session started with an older ELF runs that code again. CubeIDE rebuilt `Debug/` at 21:33 on 28 Oct, and its ELF gives bin 5b99a0d5… (checked 29 Oct); no board run with it has been reported. `g_buck` is unchanged (248 B, every field at the same offset), so the Live Expressions rows are the same for both builds. The seven stamp rows of 25 Oct evening are in `tools/live_expressions.txt` but not in the workspace's Live Expressions list: the user chose not to add them (28 Oct). (Buck_Current_Mode history: it ran on the board from 27 September, no faults, `busy_max` about 220–240 clocks. A capture of its 128 × 128 build with the ladder starting at S0 = 0.002 showed A2 rising most of the way on a 1 → 3 A step, dipping, then settling over about 0.4 ms; the dip came from the ladder's longer averages reaching back across the step. The observer here has no such mechanism.)

Pins
----

| Pin | MCU | Function |
|---|---|---|
| D7 | PA8, HRTIM1_CHA1 (AF13) | high-side gate, active high |
| D8 | PA9, HRTIM1_CHA2 (AF13) | low-side gate, complementary, 50 ns dead time on both edges (68 × 0.744 ns = 50.6 ns, rounded up) |
| D4 | PB5, GPIO | g[k]: high when S[k] ≥ S_th |
| A0 | PA0, ADC1_IN1 | v_out, direct (805.7 µV per count, 3.3 V reference); two samples per cycle, summed by ADC1 |
| A1 | PA1, ADC2_IN2 | inductor current, 40 mV/A, 0 V at 0 A (20.14 mA per count); negative current reads 0 |
| A2 | PA4, DAC1_OUT1 | load current from the observer (by default a double pole at 9.2 µs on the measured current and v_out): 0–3.3 V = 0–10 A; negative values clip at 0 V |

Parameters
----------

All parameters are in `Inc/buck_config.h`. Each one comes from the specification or from an explicit choice made while the code was written:

| Quantity | Value | Source |
|---|---|---|
| f_sw, T_sw | 600 kHz, 1.667 µs; HRTIM period 8960 ticks at 168 MHz × 32 (exact) | spec; 168 MHz HSI chosen for the exact period |
| L, C | 330 nH, 270 µF | spec |
| r_L (θ = r_L/L) | 114 mΩ | chosen (the Buck_Voltage_Mode DC fit) |
| V_in | 6 V constant (no sense pin) | chosen |
| V_ref | 1000 counts = 0.8057 V | chosen |
| Dead time in the estimator | off-time: T_on = CMP1 − DT, T_off = T_sw − T_on | chosen |
| α_I | 1 − exp(−T_sw/τ_c), τ_c = 50 µs → 0.03278 | chosen |
| Trigger score S, S_th | S = a·e_v² + b·e_v + c, e_v = v_out − V_ref in V (signed); a = 1, b = 0, c = 0, i.e. S = e_v² (live: `tune.score_a`, `score_b`, `score_c`); D4 = (S ≥ S_th), S_th = 1 (`tune.s_th`; 0.01 until 27 Sep); S drives nothing else | chosen (25 Oct; 27 Sep–25 Oct: S = (e_v/ev_norm)² + (Δd/du_norm)² + ρ·(e_v/ev_norm)·(Δd/du_norm), ρ = 0, both denominators 1; Δd since 24 Oct, Δu before) |
| PI gains | textbook 45° phase margin; f_ci = 60 kHz (f_sw/10), f_cv = 6 kHz | chosen; derivation below |
| Limits | duty CMP1 10–50 % of the period (physical D 6.96–46.96 %), i_ref −2…10 A | carried over from Buck_Voltage_Mode |
| Load current on A2 | observer, predict then correct: v̂ += a·(i_L − î); e = v − v̂; v̂ += β·e; î −= γ·e, with a = T/C = 1/162 V/A, v = v_out[k+1], i_L = the raw measured cycle average of cycle k; β = 0.3039396, γ = 4.447826 A/V (4.4478256; the float default is 4.4478254) (a double pole at τ = 9.2 µs; live: `tune.obs_beta`, `tune.obs_gamma`, taken up only inside the stable region γ > 0, 0 < β < 2 − a·γ/2) | chosen (25 Oct; 24–25 Oct in BUCK_Adpt_Learning: Î through a first-order filter whose step followed S; until then 128 × 128 cycles in two stages and a ladder of shorter averages) |
| v_out sampling | two A0 samples per cycle, 357 ns apart, the second about 100 ns before the period end; ADC1 sums them (triggered 2× oversampling) and the controller uses their mean | chosen (27 Sep) |
| Start-up | gates off until `g_buck.command = 1`; reference ramps from the measured v_out to V_ref in 10 ms | chosen |

Control structure
-----------------

The control is a cascade. The voltage PI maps (V_ref − v_out) to the average-current reference u[k] = i_ref. The current PI maps (i_ref − î̄_L[k]) to the duty. The inner loop is closed on the **measurement-corrected estimate** î̄_L[k] (chosen). The duty is the physical high-side on-time fraction, and the dead time is added in the compare: CMP1 = D·8960 + 272.

Both PIs have the form u = Kp·e + I with I += Ki·T·e, i.e. C(z) = Kp + Ki·T·z/(z−1). The integrator tracks the output limit for anti-windup.

`tools/pi_design.py` derives the gains. The model is the sampled averaged plant with r_L, the exact 2-cycle sample-to-duty pipeline, and the firmware PI form, evaluated at no load, which is the worst case for the voltage loop:

| Loop | Kp | Ki | Margins |
|---|---|---|---|
| current, f_ci = 60 kHz | 0.0237243 /A | 3338.07 /(A·s) | PM 45.0°, **GM 3.2 dB** |
| voltage, f_cv = 6 kHz | 9.3448 A/V | 285 353 A/(V·s) | PM 45.0° (no load) … 61.8° (3 A), GM 15.6 dB |

All closed-loop poles lie inside the unit circle (|z|max 0.966–0.976 over 0–3 A). Run `python3 tools/pi_design.py <f_ci> <f_cv>` to redesign for other crossovers. All gains are live-tunable (`g_buck.tune.kp_v/ki_v/kp_i/ki_i`).

Acquisition and computation schedule (one period, 280 CPU clocks)
------------------------------------------------------------------

```
cycle k:    0      T/8     3T/8     5T/8     7T/8   T-457ns  T-100ns | cycle k+1: ~0.2 us ........ ~1.4 us
HRTIM:      HS on (CMP1 = D*PER + DT) ... LS on                      |
ADC2 (A1):          x       x        x        x                      |  4 samples, hardware-summed (triggered 4x oversampling)
ADC1 (A0):                                             x        x    |  2 samples, hardware-summed (triggered 2x): 2 x v_out[k+1], EOC ~+0.16 us
CPU:                                                                 |  correction, PIs -> CMP1 preload (~0.9 us) | S, prediction of k+1, observer -> A2, D4
```

- **i_Lm[k].** HRTIM Master CMP1–4 trigger ADC2 at T/8, 3T/8, 5T/8 and 7T/8. Triggered oversampling (TROVS, 4×, no shift) returns the sum of the four samples as one result per cycle. The sampling window is aligned with the switching cycle, and the 650 kHz sensor bandwidth attenuates the harmonics that equal spacing would alias.
- **v_out.** Timer A CMP4 and CMP3, at 8960 − 538 − 1920 and 8960 − 538 ticks (357 ns apart; the second about 100 ns before the boundary), trigger ADC1. Triggered oversampling (TROVS, 2×, no shift) returns the sum of the two samples as one result per cycle, 2 × v_out[k+1] (end of cycle k, also the start value of cycle k+1); the firmware scales it by half an A0 count per count.
  - In bulb mode the input is tracked from the end of the previous conversion and held 2.5 ADC clocks (45 ns) after each trigger (RM0440 Figure 93), plus the trigger's resynchronisation to the ADC clock (a few tens of ns; its value, t_LATR, is in the datasheet and was not checked). The samples are therefore taken at most about 412 ns and 55 ns before the boundary. Both lie in the low-side interval even at the 50 % duty limit (the first at least 370 ns after the low-side turn-on). The second precedes the low-side turn-off at the boundary by only a few tens of ns; at light load, where the valley current is negative, the switch node jumps at that turn-off, not only at the high-side turn-on one dead time later. That sample's position is unchanged from the single-sample design.
  - The spacing is 20 ADC clocks. A conversion takes 15, and RM0440 ignores a trigger that arrives during a conversion, which would pair the samples across cycles from then on. The ADC clock (PLL P = VCO/6) and the HRTIM clock (VCO/2) are locked 1:3, so both triggers of a pair meet the same resynchronisation delay: the ADC always has 5 idle clocks (89 ns) after the first conversion.
  - Each sample has its own trigger (TROVS). RM0440 excludes bulb mode from continuous conversion mode and injected conversions and does not describe bulb timing for back-to-back oversampled conversions; bulb + TROVS is what ADC2 has run on the board.
  - Averaging the two samples lowers uncorrelated ADC noise by √2; noise picked up at a fixed phase of the switching cycle does not average. It also moves the regulated point up the output ripple (3.0–3.6 mV peak-to-peak from the capacitor alone at 1–3 A, falling through the second half of the low-side interval). The single end-of-cycle sample sat 0.7–1.1 mV below the cycle average, so the DC output sat 0.5–1.3 mV above V_ref in simulation; now it is within 0.4 mV (checked by `make test`). With 5 mΩ ESR, +6.6 to +6.8 mV instead of +9.7 to +10.3 mV.
- **Loop pacing.** ADC1's end of conversion paces the control loop, which runs in thread mode from CCM SRAM and polls the flag. This is the same hardware event and still one update per cycle, but it avoids about 50 clocks of exception entry/exit and register save/restore per period. The controller state stays in FPU registers across cycles.
- **Every 64th cycle** the loop also publishes the Live Expressions telemetry (`g_buck.out`, the raw ADC codes, `loops`, `busy_max`), and 32 cycles later re-reads the tunables from `g_buck.tune`, which it otherwise holds in registers. The tunables pass also checks the observer's gains against its stable region (no division). Since 28 Oct the target of the new tunables and the reference step for the next 64 passes are formed in the pass after the tunables pass (the *post pass*, in the same copy, after its CMP1 write); the gains' check stays in the tunables pass. Telemetry and the tunables reload are in different passes so that no pass carries both (27 Sep: a pass that did both, with its divisions, is a likely cause of a fault 5 on the board). The first telemetry or tunables pass comes 24 passes after arming: the first pass after arming used to start late (until 25 Oct arm() and the loop entry ran in the period before it; since then only the arming does, and the first pass normally waits for its sample like any other), and a long pass there would also delay the next pass's CMP1 write (a possible fault 5 on a restart). `loops` therefore skips ahead by up to 31 at each start. D4, A2 and CMP1 are still written every cycle. The command is re-read every cycle.
- **CMP1.** The new duty is written as a preload that becomes active at the next period boundary. The sample-to-duty pipeline is therefore 2 cycles, and the PI design includes it. A write that would miss the deadline trips fault 5.
- **After the write.** S[k] and g[k] are computed and the estimator predicts cycle k+1: the cycle in progress, whose on-time was written by the previous update. Then the observer takes v_out[k+1] and the measured average of cycle k and writes A2, and g[k] goes to D4.

**CPU budget.** A pass must fit one period *on average*. A longer pass delays the next one, and the delay builds up from pass to pass until the CMP1 write misses its deadline, which trips fault 5.

The first version tripped fault 5 on the board. It needed 229–235 core clocks per pass (247–253 during the soft-start ramp), plus the wait states of ten peripheral accesses. The loop was restructured without changing the algorithm or the per-cycle outputs:
- multiply-adds are compiled as VMUL + VADD (2 clocks) instead of VMLA (3 clocks), with the same result;
- telemetry is written, and the tunables re-read, every 64th pass only (in different passes, 32 apart); in between, the tunables stay in registers;
- the peripheral base addresses stay in registers;
- the stop and bad-sample tests share one branch;
- the timer counter is read right before the CMP1 write, so the deadline check covers the write itself. It also catches a counter that has wrapped, i.e. a pass more than one period late;
- `run_loop()` is marked `hot` (GCC had optimised it for size because it was reached only through the cold `arm()`; buck_run() calls it directly since 25 Oct, and `hot` is kept), and Ki·T is formed when the tunables are loaded;
- the reference step is chosen once per 64 passes, when the tunables are re-read (the target can change only then; since 28 Oct by the pass after the tunables pass), and the loop runs one of four compiled copies of the pass: HOLD (at the target: no ramp code), FREE_UP (target more than 66 steps away and below 16 V: the unclamped step, provably exact), UP and DOWN (the step with its limit). The reference follows the original per-cycle trajectory bit for bit (host tests and emulator);
- the Padé table rows are pre-scaled by powers of two (see *Estimator implementation*), and D4's BSRR word is a select.

Safety details added with the last round (the per-cycle outputs are unchanged for all finite values):
- the PI limits are applied as max(u, lo) then min(u, hi), so a NaN from a non-finite tunable written in Live Expressions ends at the lower limit instead of passing through (before, CMP1 became 272 and the Padé row index went out of the table);
- arming (since 25 Oct inside `run_loop()`, before in `arm()`) enables the gates only when its sample is read early in the period (counter above `BUCK_CMP_MIN` and below `BUCK_ARM_LATEST` = 4096 ticks; 5120 before the trigger denominators added a division to the arming path, kept at 4096 when they went and with the 25 Oct fix), otherwise it retries on the next sample, so the first control pass reliably follows within one period. Since 28 Sep it also requires that ADC1 has not converted again since the samples were read (checked after the counter read): a RUN within about two periods of a STOP, or a debugger halt at the wrong moment, could otherwise arm in the next period with that period's sample already waiting, and the first pass tripped fault 4. Since 25 Oct (user, after the board's fault 5 on the first pass after arming) `buck_run()` calls `run_loop()` in the stopped cycle that sees `command = 1`; the loop's entry (prologue, the observer gains' check, the tunables, Ki·T, the score coefficients, the target, the constants) runs in the rest of that period, and the loop then waits for the next sample, arms on it with the same two conditions and goes straight to the first pass. From the arming check's counter read to the first pass's guarded counter read it was 172–195 clocks static, 196–221 with the measured wait states in the 25 Oct evening build (before: 334–365 and 375–408; since 28 Oct 138–141 / 147.12–150.12 by counter 2, 144 / 153.12 at the longest by counter 1, see *CPU budget*, 28 Oct), against 416 at 4096 ticks (the arming path's slack, not the first write's margin: the first pass waits for its own sample at every accepted arming point and keeps 126–144 clocks, about 146–164 since 28 Oct; the arming check normally reads the counter at about 2100–2650 ticks, about 1800–2350 since 28 Oct; the arming CMP1 write itself is the tighter limit at 4096, see *CPU budget*). While it waits to arm, the loop does per sample what the stopped loop does: the raw codes are published, a bad sample is counted in `adc_miss` (its flags cleared) and ends the wait (the converter stays stopped), and a STOP ends it; `state` stays 0 until the arming. The arming writes the controller state straight into the loop's registers (`g_buck.st` is written back at a stop or a trip, as before, but no longer at the arming: during a run it holds the state of the previous stop, zeros before the first), publishes `out.*` and starts the observer from the arming sample, as `arm()` did (until 28 Oct GCC stored 10 of the 11 `out.*` fields before the arming CMP1 write, where `arm()` stored 10 of them after the OENR write and `out.g` between the CMP1 and OENR writes; since 28 Oct the arming writes CMP1 and OENR first and everything else, `out.*` included, after them; see *CPU budget*); it uses the tunables copied at the entry (a period earlier, more if arming retries), not `g_buck.tune` at that moment (accepted by the user);
- the stop command is read with a volatile access every pass;
- `make check` inspects the linked ELF (tools/check_loop.py): the loop must lie in CCM with no cold part, contain no fused multiply-add, and guard every CMP1 write with the deadline check. The read, the check and the write are one asm block (`ldr`, `sub`, `cmp`, `it cc`, `strcc`, then `bcs` to the trip: the write happens only inside the window; since 28 Oct the `bcs` is the asm's own last instruction, an `asm goto`), so GCC's register allocation cannot put anything between them; a plain `bcs` before an unconditional store is accepted too. The checker follows the data (counter → difference → compare, none of them the timer base register) and follows the `bcs` to the fault-5 trip (`r0 = 5` within 40 instructions). It tests its own rule at every run against broken sequences (28 Sep: the version of 27 Sep accepted a build with the trip removed, the condition inverted, or the duty or the raw counter compared). A store to the same offset is exempted only when its base register holds `g_buck`'s address throughout the loop (a `g_buck` field can sit at that offset); since 26 Oct a write of that register whose value is never used as an address (every path from it returns, branches out or calls `buck_fatal()` before a load or store through the register) does not count against it: in a mutant GCC used `g_buck`'s register as a scratch on such a block, and its stores to `busy_max` (offset 156, CMP1's offset) were taken for unguarded CMP1 writes and the tunables rule found no register holding `g_buck`'s address. Deliberately broken builds fail it: VMLA allowed, an instruction or a timer-base reload between the check and the write, the write through a register that also held `g_buck`, the trip removed, the condition inverted, the duty or the raw counter compared. Not analysed: an `strd` or register-offset store that reaches CMP1, a timer base formed another way, a branch into the guarded write, or a base writeback (other than an `ldm`'s) or a call that changes the register holding `g_buck`'s address (not taken as a write of it). Since 25 Oct it also checks the tunables reload: every `g_buck.tune` field must be read from `g_buck` at the loop entry and in the passes, at least once in each (field offsets from the ELF's debug info; `obs_beta` and `obs_gamma` must be among them), and `obs_beta` and `obs_gamma` must not be read on the path of an ordinary pass (from a pass head, without the branch of the `loops & 31` test into the telemetry or tunables pass), since the gains are checked only where they are reloaded (user, 25 Oct). Its parts are self-tested on short sequences at every run. The entry is what `run_loop` executes from its first instruction up to its first ADC12_CSR read (since 25 Oct evening the arming wait's, see below), following its branches. GCC keeps such a load only where the value is used, so a tunables pass that no longer copies a, b, c into the score coefficients or no longer takes up (checks) the observer's gains, or an entry that does not take them from the tunables, fails it. In BUCK_Adpt_Learning the rule counted two reads anywhere in the loop; in this build GCC repeats the gains' check in each of the four copies of the tunables pass, so an entry that took the compiled default gains without reading the tunables still had four reads and passed. Counted separately, it fails (tried, 25 Oct), and so does a tunables pass without the gains' reload, and the reload line (`c->ok = buck_obs_k(&b->tune, c->ok)`) moved into every pass (which the rule's first version of 25 Oct passed). The rule only sees reads of `g_buck.tune`: a check run every pass on the loop's own copy of the tunables (`c->t`) reads nothing from `g_buck` there and passes. Since 25 Oct evening (the arming in `run_loop()`): the arming CMP1 write is exempt from the deadline guard, and only it: the one CMP1 write followed within three instructions, with no store between, by the outputs-enable write (HRTIM OENR) through the same base, and not on the path of an ordinary pass (more than one is an error); its window is not checked by the script (review only). A literal pool between the conditional store and its `bcs` (`strcc`, `b`, `bcs`: GCC did this in the first pass after arming in the 25 Oct evening build; the 28 Oct build has none) is accepted, nothing else there. The loop entry for the tunables rule ends at the first ADC12_CSR read, which is now the arming wait's, so "read at the loop entry" means read before the arming wait; GCC reads it through a register holding its address (`ldr r5, [ip]`), so the script tracks the entry's register values (immediates, literal loads, add/sub of an immediate; at a join only what both paths agree on). Each change is self-tested. Broken versions of the checker (26 Oct, two samples: 26 of the 25 Oct changes, 24 of the 26 Oct one) all fail its self-tests, except two that cannot behave differently on `run_loop` (a branch-range test that the instruction list already implies, and a store test that the next test repeats). Until 26 Oct 13 of the first 21 passed, those two and eleven others (the join with the paths in the other order; a sub, an mvn, a movt or a literal load; a register write or a call not making a register unknown; the entry continued past a call that does not return; two arming writes; one on the path of an ordinary pass; a branch between the CMP1 and OENR writes). Of the second 22, 19 passed the self-tests as first extended on 26 Oct: three of the register copy (not tracked, tracked wrong, or kept when copied from an unknown register) and 16 of the 26 Oct change, 12 that made it accept more (for example a conditional branch's target or fall-through not followed, a conditional `ldm` return taken as the end, an `stm` base or an index register not taken as a use, a literal of another address or any load taken for `g_buck`'s) and 4 that made it reject more (false messages, as for the scratch write above). The self-tests now cover them. The rest of how `run_pass()` wires the results is covered neither by it nor by any test (review only). For example, each of these passes `make`, `make check` and `make test` (tried 25 Oct): the gains' check run every pass on `c->t`, or in the telemetry pass instead of the tunables pass; the observer stepped only in the tunables pass, or twice per pass; î seeded from `out.i_avg` instead of i_meas; the observer fed the estimator's î̄_L instead of the measured average, the check's result ignored (the tunables' gains taken as they are), v̂ started at 0 instead of the arming pass's v_out, the observer moved before the CMP1 write or fed V_ref instead of v_out, A2 written from v̂, `out.i_load` not updated in the telemetry pass (an uninitialised value published, without a warning) or published as v̂ or as i_meas, the entry falling back to 0 instead of the compiled defaults, the reload checking the previous tunables' pair instead of the new one; and, from BUCK_Adpt_Learning, the score coefficients formed from the previous tunables (the order inside the reload) or D4 driven from another value than g.

**24 Oct (BUCK_Adpt_Learning).** Static count of the linked loop of both projects with one timing model (TRM cycle rules: taken branch 3, load 2 or 1 when pipelined after a memory access, but 2 when its address comes from the load just before it, register-offset store 2, STRD 2, VDIV 14, others 1). Each pass is counted from the loop body (the target of the EOC branch) to the next ADC status read; the head itself (`lsls`, the taken `bmi`, and on a late pass three `add`) adds 4 clocks per pass in steady state, 7 on a pass that starts late. That boundary, not the model, is the difference to the table below (28 Sep quiet HOLD: 167 here, 174 there). The rule for a load whose address comes from the load before it was added on 25 Oct; until then such a load was counted as pipelined. It changes no 24 Oct figure, because that build has no such load on a counted path (arm()'s own counter read is one, but the count starts after it). In the 28 Sep build the `busy_max` CYCCNT read is one (after the literal load of the DWT base), which adds 1 clock to each of its telemetry passes. The 28 Sep column also counts a load right after a register-offset store as pipelined (one such pair on every path of each pass, after the CMP1 write and inside the `busy_max` window), and its arming figure a load right after an STRD. The TRM says no instruction can be pipelined after an STR with register offset. Counted that way, every 28 Sep pass and `busy_max` figure is 1 clock more (normal 168 / 170 / 174 / 174 and 203 / 205 / 210 / 210; telemetry 221–242 / 223–245 / 227–249 / 227–249; tunables 241–259 / 246–264 / 250–269 / 249–268; `busy_max` window 163–167 / 164–168 / 168–173 / 168–173; longest pass 269), with the CMP1 write unchanged; the arming figure is 320–341 if, in addition, the load after the STRD is not pipelined (as after a multiple store). The column keeps the figures as first counted; whether to restate it is open (user). No figure of the 24 Oct, 25 Oct or Buck_disturb_obs builds changes under that rule: none has such a pair on a counted path.

| Pass (HOLD / FREE_UP / UP / DOWN) | Buck_Current_Mode, 28 Sep | BUCK_Adpt_Learning, 24 Oct | BUCK_Adpt_Learning, 25 Oct (score a·e_v² + b·e_v + c) | Buck_disturb_obs, 25 Oct (observer) | Buck_disturb_obs, 25 Oct evening (arming in the loop) |
|---|---|---|---|---|---|
| normal, quiet / with the faster averages | 167 / 169 / 173 / 173; 202 / 204 / 209 / 209 | 156 / 161 / 169 / 169, whatever S | 150 / 154 / 160 / 160, whatever S | 140 / 142 / 148 / 148 | 140 / 143 / 147 / 147 |
| telemetry | 220–241 / 222–244 / 226–248 / 226–248 (219–240 / 221–243 / 225–247 / 225–247 with the CYCCNT read counted as pipelined) | 243–246 / 244–257 / 257–260 / 257–260 (the filter coefficients: one VDIV, after the `busy_max` read) | 240–243 / 239–252 / 250–253 / 250–253 | 180 / 182 / 190 / 188 (no division, no coefficients) | 182 / 185 / 189 / 189 |
| tunables, with the copy selection | 240–258 / 245–263 / 249–268 / 248–267 | 232–247 / 241–256 / 249–264 / 249–264 | 211–228 / 219–236 / 226–243 / 226–243 (no VDIV) | 241–262 / 237–264 / 241–262 / 246–267 (the gains' check) | 235–262 / 238–259 / 240–267 / 237–264 |
| `busy_max` window (pass start to the telemetry read) | 162–166 / 163–167 / 167–172 / 167–172 | 156 / 160 / 168 / 168 | 151 / 154 / 160 / 160 | 141 / 143 / 151 / 149 | 143 / 146 / 150 / 150 |
| CMP1 write, clocks into the pass | | 77 / 77 / 77 / 77 | 74 / 74 / 76 / 76 | 74 / 73 / 75 / 75 | 72 / 72 / 73 / 73 (the first pass after arming: 75) |
| first pass after arming, from arm()'s counter read to the CMP1 write (limit 416) | 319–340 | 334–357 (334–356 by the 25 Oct count) | 310–336 | 337–368 | from the arming check's counter read: 175–198 (176–199 from the read's issue; 339–370 that way before) |

In that build the normal pass is 4–11 clocks shorter than the 28 Sep quiet pass and 40–46 shorter than its pass with the faster averages. The longest pass is the tunables pass: 264 in that build (268 with the head in steady state, 271 if it starts late) of the 280-clock period, against 268 (272, 275) on 28 Sep. So on the board `busy_max` should read about the same as Buck_Current_Mode's or a few clocks less, and the margin for wait states before the longest pass fills the period is about 4 clocks larger. All 24 orders of the four copies were re-counted for this body (verification, 24 Oct): none is better on every measure, so the order is unchanged. That still holds with the dependent-load rule, which leaves this build's figures as they are and can only lengthen the other orders. Its `run_loop` was 4482 B (4654 B on 28 Sep).

**25 Oct (score a·e_v² + b·e_v + c).** The score lost its Δd term, ρ, its two denominators and their division: (a·e_v + b)·e_v + c is two multiplies and two adds, and the coefficients are copied when the tunables are loaded. Same model and boundaries as above: the normal pass is 6–9 clocks shorter, the telemetry pass 3–7, the tunables pass 19–23 (the VDIV is gone), the first pass after arming 20–24 (counted with multi-register pushes as 1 + N clocks, which reproduces the 24 Oct figure to within 1 clock; 284–310 with them as 1 clock), and the CMP1 write comes 1–3 clocks earlier.

The HOLD and FREE_UP copies each have two loads whose address comes from the load just before it:
- the counter read before the CMP1 write, after the literal load of the Timer A base;
- in the telemetry pass, the `busy_max` CYCCNT read, after the literal load of the DWT base.

The TRM does not pipeline such a load. That costs those two copies 1 clock on the normal and tunables passes, on the `busy_max` window and before the CMP1 write, and 2 on the telemetry pass. Counted as pipelined, as until 25 Oct, they give 149 / 153 (normal), 238–241 / 237–250 (telemetry), 210–227 / 218–235 (tunables), 150 / 153 (`busy_max` window), the CMP1 write at 73 / 73, and 309–336 (283–310) for the first pass after arming.

The longest pass is now the telemetry pass: 253 (257 with the head in steady state, 260 if it starts late) of 280, 11 clocks less than the 24 Oct tunables pass. `BUCK_ARM_LATEST` stays at 4096 ticks (user): about 80 clocks of margin for wait states on the first pass (60 before by the same count, 59 by the 24 Oct figure).

All 24 copy orders were re-counted again, under both rules. With the dependent loads counted as pipelined, none is better on every measure (the best normal passes, 147 / 152 / 159 / 160, put the CMP1 write a clock later). With the TRM rule, two orders are better on most measures and no worse on any of the normal, telemetry and tunables passes, the CMP1 write and the longest pass:

| Copy order (today: UP, FREE_UP, HOLD, DOWN) | normal | telemetry, longest | tunables, longest | CMP1 write | first pass after arming |
|---|---|---|---|---|---|
| today | 150 / 154 / 160 / 160 | 243 / 252 / 253 / 253 | 228 / 236 / 243 / 243 | 74 / 74 / 76 / 76 | 310–336 |
| HOLD, UP, DOWN, FREE_UP | 148 / 152 / 160 / 159 | 242 / 241 / 252 / 251 | 217 / 228 / 235 / 234 | 74 / 74 / 76 / 75 | 312–335 |
| HOLD, DOWN, UP, FREE_UP | 148 / 152 / 159 / 160 | 242 / 241 / 251 / 252 | 217 / 228 / 234 / 235 | 74 / 74 / 75 / 76 | 312–334 |

The figures are per copy, HOLD / FREE_UP / UP / DOWN. In each of the two orders, the first pass after arming is 2 clocks later at its earliest and 1–2 clocks earlier at its latest, and one CYCCNT read still depends on the load before it. Reordering the copies is a code change and has not been decided, so the order is unchanged. `run_loop` is 4412 B (4404 B in either of the two orders).

**25 Oct, Buck_disturb_obs (the observer).** Same model and pass boundaries, counted on the linked ELF by `pcv`/`busy`/`armpath` and, independently, by the final checker's own path enumerator (identical figures). Against BUCK_Adpt_Learning's 25 Oct build:
- **Normal pass 10–12 clocks shorter** (140 / 142 / 148 / 148). The observer is 8 FP operations where the per-cycle Î (3) and the filter (13, with its two compare-and-select clamps) were, and the HRTIM base now stays in a register, so the counter read before the CMP1 write no longer follows a literal load. The CMP1 write comes 0–1 clock earlier (74 / 73 / 75 / 75); the counter is read 68–70 clocks into the pass.
- **Telemetry pass 63–70 clocks shorter** at its longest (180 / 182 / 190 / 188): the filter's coefficients, with their VDIV, are gone.
- **Tunables pass 19–34 clocks longer** at its longest (262 / 264 / 262 / 267): the observer gains' check (the exact rounding error of a·γ/2 by Dekker's product, 4 multiplies and 4 additions, the selection of the exact difference and three compares; see *Load current*). It is now the longest pass: 267 (271 with the head in steady state, 274 if it starts late) of 280, against 253 in BUCK_Adpt_Learning and 264 on 24 Oct.
- **First pass after arming 27–32 clocks longer** (337–368 of its 416 clocks, about 48 clocks of margin for wait states instead of 80): the same check at the loop entry. 311–342 with multi-register pushes counted as 1 clock.
- What the check costs, from two variants of this build (counted, not kept): with a trivial test in its place that still keeps the previous gains, the tunables pass is 224 / 230 / 234 / 235 at its longest and the arming path 313–334; with no check at all (the tunables' gains taken as they are), 215 / 222 / 226 / 227 and 300–321. So the exact check costs 28–38 clocks in the tunables pass and 24–34 on the arming path, and keeping the previous gains about 8 and 13 more.
- Averaged over 64 passes in steady state (HOLD, with the tunables pass returning to HOLD: 249 clocks here, 211 in BUCK_Adpt_Learning): 142.3 clocks, against 152.4 (142.5 and 152.7 with the longest tunables path from HOLD, which leaves for another copy).
- Dependent loads: only the `busy_max` CYCCNT read in the HOLD and FREE_UP telemetry passes (1 clock each). Counted as pipelined, those two telemetry passes are 179 / 181; nothing else changes.
- All 24 orders of the four copies were counted under both rules (each order builds and passes `make check`). None is at least as good as the current order on every one of the per-copy normal, telemetry and tunables passes, the CMP1 write, the longest pass and the arming path. Over the 24 orders the longest pass ranges 265–274, the HOLD normal pass 139–142 and the arming path's latest 365–369. The order is unchanged. Two orders are better on the limiting figures. DOWN, FREE_UP, UP, HOLD has the same figures as the one below except the first pass after arming, 335–366 (309–340 with pushes as 1 clock), the tunables passes 234–261 / 237–264 / 244–265 / 244–265 (UP 3 clocks longer, still within the longest pass 265) and `run_loop` 4352 B. UP, FREE_UP, DOWN, HOLD gives a longest pass of 265 instead of 267 (269 / 272 with the head instead of 271 / 274), the first pass after arming 336–367 instead of 337–368 (310–341 with pushes as 1 clock), the CMP1 write 1 clock earlier in every copy (73 / 72 / 74 / 74) and normal passes 139 / 142 / 148 / 148 (telemetry 179 / 182 / 190 / 190, tunables 240–261 / 237–264 / 241–262 / 244–265, `busy_max` window 140 / 143 / 151 / 151, `run_loop` 4336 B). It costs 2 clocks on the DOWN telemetry pass (190 against 188) and on the DOWN `busy_max` window (151 against 149). The current order is kept (user, 25 Oct).
- `run_loop` is 4308 B. RAM 89 112 B (16 B less: no filter coefficients, and `out.i_load_alpha` and `buck_state_t.v` are gone), FLASH 8504 B.

**25 Oct evening, Buck_disturb_obs: fault 5 on the board, and the arming moved into the loop.** The first board run (CubeIDE Debug build, the same CCM code as the Makefile build) tripped fault 5 on the first pass after arming. RAM: `loops` = 8 (its start value), `trip_cnt` = 8608 ticks against the deadline 8448 (5 clocks late), v_out 0 V, `adc_miss` 0, `warmup` 0, `busy_max` 0 (no telemetry pass yet). `g_buck.bench`, extra clocks per access over the same access to SRAM1: a CCM literal or data load while executing from CCM +1.06; the HRTIM Timer A counter read +3; ADC12_CSR, ADC1_DR and ADC2_DR reads +1; DWT_CYCCNT 0; a peripheral write followed at once by an SRAM or peripheral load +2 (by a CCM literal +1); back-to-back CMP1 writes +1.06; DAC and BSRR writes about 0.
- **The wait-state model.** The static counts above have no wait states. Counted again on the linked code with each memory access classified by its target (register values tracked along every path) and charged the bench value (nominal), or with CCM literals at +2 and no decay of the write penalties (pessimistic), the old arming path, from `arm()`'s counter read to the first pass's guarded counter read, was 334–365 clocks static, 375–408 nominal and 405–440 pessimistic; the board's path (v_out 0 V, default gains, FREE_UP) 361 / 402 / 432. 30–32 of its loads were CCM literals (31.9 of the 40.9 nominal wait clocks). A normal pass gains only 6–8 (three ADC reads, the counter read, one literal).
- **What the board says.** The deadline is absolute: the first pass's CMP1 write must land before one period plus 264 clocks after the start of the arming period, so the path from the arming counter read must stay below 544 − (that read)/32 clocks. With `trip_cnt` = 8608 the board took about 460–485 clocks at the usual arming point (the counter at about 2000–2900 ticks), 60–80 clocks (16–20 %) more than the wait-state model: the nominal model cannot produce the trip at any accepted arming point, the pessimistic one only with arming at 3520–4096 ticks, later than the end of conversion allows. The excess is unexplained. The best hypothesis, unproven: VLDR and VSTR take 2 clocks and do not pipeline (the path had 37 VLDRs counted at 1 clock and 26 VSTRs: +63). Steady state is not affected by the arming: its CMP1 write lands 100–150 clocks before the deadline.
- **The fix (user, 25 Oct, option f).** `buck_run()` calls `run_loop()` in the stopped cycle that sees `command = 1` (where it called `arm()`), and the loop's entry runs there: the prologue, the exact gains' check (still the loop entry's first statement), the copy of the tunables, Ki·T, the score coefficients, the target and the constants. An empty `asm volatile` with the entry's results as inputs and a memory clobber ends the entry; without it GCC moves Ki·T, the target and `loops` past the wait into the arming path (the 25 Oct prototype; as a mutant of this build 17 clocks more there). The loop then waits for the next sample, arms on it exactly as `arm()` did (window 897…4095 ticks, then the 28 Sep ADC1 check; a refusal waits for the next sample), writes CMP1 and enables the outputs, publishes `out.*`, sets `state` = 1, seeds the observer from the arming sample and goes straight to the first pass. The first pass is a fifth copy of the pass (its reference step chosen at run time; the copy for the next passes is chosen after it, with the same reference values): it alone carries the timing stamps (`g_buck.arm_cnt`, `arm_cyc`, `first_cnt`, `first_cyc`, stored after its CMP1 write; see *Diagnostics*), and a trip stores `trip_cyc`. In the disassembly, the entry ends at 0x100004e2, before the arming wait's first status read (`ldr r5, [ip]`, 0x100004e6), with every tunable and `loops` read and the gains' check, Ki·T (two `vmul` by T), the target and `loops` formed there; between the wait and the first pass there is no read of `g_buck.tune` and no product of the entry.
- **Figures** (linked ELF, from the issue of the arming check's counter read; nominal and pessimistic with the bench's wait states; before → now):

| | before (`arm()`, then the entry) | now (the entry first, the arming in the loop) |
|---|---|---|
| arming check to the first pass's guarded counter read, all paths, first pass starting late (static / nominal / pessimistic) | 334–365 / 375–408 / 405–440 | 172–195 / 196–221 / 213–240 |
| the board's path (v_out 0 V: FREE_UP, the gains accepted) | 361 / 402 / 432; on the board about 460–485 | 195 / 221 / 240; board estimate 231–266 (236 with VLDR/VSTR at 2 clocks, 231–256 with the old residual; 257–265 with a flat 16–20 %); up to about 306 if the old excess (56–85 clocks over the nominal count) is one fixed cost that falls on this path too |
| … to its CMP1 write (`strcc`) | 339–370 / 383–416 / 413–448 | 176–199 / 203–228 / 220–247 |
| status read that finds EOC to the arming check's counter read | 35 / 39 / 40 (stopped loop, then `arm()`) | 28 / 32 / 33 from the wait's poll loop, 22 / 26 / 27 from its first read |
| stopped cycle that sees RUN, from that status read to the arming wait | — (`arm()` ran in it) | 126–136 / 140–150 / 149–159 (to the call 33 / 36 / 36; the entry 93–103 / 104–114 / 113–123); 136 / 148 / 155 on the board's path |
| first pass, from the status read that finds its sample (poll head 6 static, 7 with the read's wait), body to its counter read; to its `strcc` | as any pass of its copy (counter read 68–70 / 70–72 / 70–72, `strcc` 73–75 / 78–80 / 78–80) | 71 / 80 / 87; `strcc` 75 / 87 / 94 |
| first pass, whole: body to the next copy's status read, that read included (the convention of the pass table above); by the copy it hands over to | as any normal pass of its copy (140 / 142 / 148 / 148 static) | 193–217 / 221–246 / 240–266 (191–215 / 218–243 / 237–263 without that read; an independent counter of the 26 Oct review, which lets the A2 write's penalty on the literal load right after it decay over the write's own clock, gives 1 less nominal); to HOLD 193–205, FREE_UP 202–214, UP 201–217, DOWN 195–207 static. The longest pass type after the tunables pass: with its poll head (from the status read that finds its sample) 197–221 static, about 250 nominal, 270 pessimistic at its longest. With the board's 16–20 % that is up to about 20 clocks past 280, which only delays the second pass (it starts up to about 20 clocks later after its sample; its CMP1 write still lands about 90–120 clocks before 264); nothing accumulates, a normal pass being about 150 |
| `run_loop` (CCM), `buck_run` | 4308 B + `arm()` 432 B, 148 B | 5620 B, 136 B |

  **The first CMP1 write's margin** is 126–144 clocks (nominal; about 108–130 with the board's 16–20 % on that pass) at every accepted arming point, 4096 ticks included. The first pass's head comes 94–117 / 108–133 / 118–145 clocks after the arming check's counter read, so at most 128 + 145 = 273 clocks into the arming period (about 290 with the board's excess as a proportion), before the next end of conversion at about 313: the first pass waits for its own sample and reaches its check at EOC + 0–9 + 87 clocks nominal into the next period (77 / 87 / 94 from the status read that finds its sample), 120–138 clocks. **The arming path's slack:** the first write must land within 544 clocks of the arming check's counter read minus the read's position, 416 at 4096 ticks, and the path is 172–195 / 196–221 / 213–240, so at 4096 it could grow by 176–220 clocks (pessimistic to nominal; about 150 with the board's excess as a proportion, 110 or more if the old excess is a fixed 56–85 clocks that also falls here) before the first write would miss; the first pass would first stop waiting for its sample. These 176–220 clocks are not the first write's margin. **The arming CMP1 write** (no deadline check by design, so no fault 5) is the tighter limit on `BUCK_ARM_LATEST`: it comes 77–88 / 91–103 / 99–112 clocks after the arming check's counter read (OENR 2 later; `arm()`: 83–90 / 99–107 / 109–118), so at 4096 ticks it lands 205–216 / 219–231 / 227–240 clocks into the period, 48–59 / 33–45 / 24–37 before 264 (about 13–31 before with the board's 16–20 % on it; at 275–316, past 264, if the old excess were a fixed cost inside this stretch). A late arming write trips nothing: the first enabled period then runs with the previous CMP1 (`BUCK_CMP_MIN`, written at boot and at every stop) instead of `buck_ctrl_arm()`'s value. GCC stores 10 of the 11 `out.*` fields before this write; with them after the OENR write, as `arm()` had 10 of them (it stored `out.g` between the CMP1 and OENR writes; here a memory barrier after OENR; built and counted, not kept), the write comes at 70–82 / 84–97 / 92–106, 6–7 clocks earlier, with the copies' passes unchanged. A third way: hold `buck_ctrl_arm()`'s float constants across the arming wait (load them in the entry). On the board's path 10 CCM literal loads lie between the arming counter read and this write (nine of those constants and a reload of the HRTIM base, whose register is reused), 16 / 27 / 36 clocks together; without the nine constants' loads that path is 13 / 23 / 31 clocks shorter (88 / 103 / 112 to 75 / 80 / 81, counted with them removed, the HRTIM base reload kept); holding the constants in registers may cost the passes' register allocation, as the entry's values taken as `"+t"` outputs did (1–3 clocks per pass); not built. Whether to lower `BUCK_ARM_LATEST`, move those stores or hold those constants is open (user). (28 Oct: the optimisation stores `out.*` after the OENR write and holds those constants in registers from the entry; the write now comes 29 / 33 / 33 clocks after the counter read, 103 clocks before 264 at 4096 ticks, pessimistic. `BUCK_ARM_LATEST` is unchanged.) **The stamps' reads** are on these paths. The arming CYCCNT read (a literal load and a dependent load before the counter read, a stack store before the CMP1 write) puts the arming counter read 6–10 clocks later after its sample (a build without it, counted, not kept: 16 / 18 instead of 22 / 28 clocks from the status read that finds the sample), but not the arming write later relative to that read (78–89 in that build; its first pass came out 2 clocks later, GCC's allocation). The first pass's CYCCNT read costs 1 clock on its `strcc` (74 instead of 75 with the read after the `strcc`, a mutant counted on 25 Oct); the 126–144 above include it. Reading them after the writes instead (for the first pass right after the `strcc`, 5–6 clocks after the counter read; `make check` would then need its rule that the `bcs` follows the `strcc` widened) is open (user). The entry fits the stopped cycle: it reaches the arming wait at most 210 clocks into the period (pessimistic, with the latest EOC and poll phase; about 240 with the board's 20 %), before the next end of conversion at about 313.
- **The other passes** are not instruction-identical to the build before the fix (the request was to keep them unchanged; whether this difference is acceptable is open, user; the 28 Oct optimisation changes every pass's instructions by design, with the results of this build bit for bit but for one exception, see below). Each normal pass has the same number of instructions as before (131 / 133 / 138 / 138 for HOLD / FREE_UP / UP / DOWN, head to loop-back), with GCC's register allocation and stack slots changed. Changed instructions: in all four copies the `loops & 31` test is `lsls #27` instead of `tst #31`; in FREE_UP the ADC2 code, now the value spilled to the stack (`str r6, [sp, #40]`; before, the ADC1 code was), is reloaded with `vldr s14, [sp, #40]` before the CMP1 write where it was moved with `vmov s14, fp`, a value held in `s31` before is reloaded from the stack after the write (`vldr s25, [sp, #12]`), and one 16-bit `ldr` is now 32-bit. Instructions in another place (longest common subsequence of the normal path, with registers, stack slots and branch targets abstracted): HOLD 7, UP 3, DOWN 3, FREE_UP 11. By the static model FREE_UP's CMP1 write is 1 clock earlier (73 → 72) and its normal pass 1 clock longer (142 → 143): the extra reload after the write takes a place in a group of pipelined loads and pushes another load out of it. Static, in the table's convention (before → now): normal 140 / 142 / 148 / 148 → 140 / 143 / 147 / 147; telemetry 180 / 182 / 190 / 188 → 182 / 185 / 189 / 189; tunables 241–262 / 237–264 / 241–262 / 246–267 → 235–262 / 238–259 / 240–267 / 237–264 (HOLD returning to HOLD 241–249 → 235–250); `busy_max` window 141 / 143 / 151 / 149 → 143 / 146 / 150 / 150; CMP1 write 74 / 73 / 75 / 75 → 72 / 72 / 73 / 73 clocks into the pass (counter read 69 / 68 / 70 / 70 → 67 / 67 / 68 / 68). Longest pass still 267 (now UP's tunables pass; 294 nominal and 309 pessimistic with the poll head, as before). Nominal with the bench, in the table's convention (without the head): normal 147.1 / 150.1 / 154.1 / 154.1 (before 147.1 / 149.1 / 155.1 / 155.1), 7.06 wait clocks per pass (three ADC reads, the counter read, one literal; 144.1 / 147.1 / 151.1 / 151.1 without the closing status read); CMP1 77 / 77 / 78 / 78 clocks into the pass (79 / 78 / 80 / 80). Average over 64 passes in steady state: 142.4 (142.3). The `asm` ending the entry takes its values as inputs: as outputs (`"+t"`) they cost the copies 1–3 clocks per pass, and four first-pass copies with the step built in (the dispatch's form) put the copies' CMP1 writes 2–3 clocks later, cost up to 3 clocks per pass and 1.4 KB (both built and counted; not kept).
- RAM 89 112 B as before (`g_buck` is 20 B larger, inside the alignment slack), FLASH 9396 B (8504), CCM 6744 B (5872).

**28 Oct, Buck_disturb_obs: fewer clocks, the same results (not yet run on the board).** The user asked for the loop to use fewer clocks while keeping the trigger score, the PI calculation, the observer and the sensing, with bit-identical results only (NaN payloads included; FMA stays off). Two scouting rounds ran in scratch copies of the project; on 28 Oct the user chose the "recommended merged" build and it was applied in place: bin 2f144a53… (12 540 B, `.ccmtext` 9888 B, `run_loop` 8764 B; the 25 Oct evening build e5c7798f…: 9396 B, 6744 B, 5620 B). The same day the verification after the change found the observer's prediction compiled with its operands swapped in the post pass, and the user had it pinned there (second set of decisions, below): bin 5b99a0d5…, with the same sizes and the same clock counts in both counters. The code changes are in `Src/buck.c`, `Inc/buck_ctrl.h` and `Inc/buck_config.h` (a comment); `Inc/buck.h` changed in comments only, and `g_buck` is unchanged (248 B, `busy_max` and the stamps at offsets 156 and 228–244).
- **What changed.** Each item does the same operations on the same values:
  - *Arming:* CMP1 is formed from v_out alone and written first, then OENR; the controller state, the T_on = 0 prediction and `out.*` follow after a memory barrier. The arming's and the first pass's constants (V per count, the duty limits, 8960, 1/V_in, the T_on = 0 Padé coefficients, the HRTIM and DWT bases, the CSR mask) are loaded into registers at the loop entry. The first pass always takes the UP step (the same bits: after arming the mode is HOLD, UP or FREE_UP, never DOWN).
  - *The pass:* the deadline check is an `asm goto` (the same `ldr`, `sub`, `cmp`, `it`, `strlo`, with the branch to the trip as the asm's last instruction); the corrected valley and both integrator updates run after the CMP1 write; the pass's I/O (ADC1/ADC2 DR, the ADC12_CSR poll, the DAC write) are `ldr`/`str` at immediate offsets from ADC1's base; ADC2 × 0.25 is one `vcvt` with 2 fraction bits; GPIOB's BSRR address comes from one MOV and the telemetry pass takes `out.g` from the BSRR word; the Padé table base is pre-biased; the gains' check forms both difference orders and selects one.
  - *The copies:* their constants are held in registers formed once after the first pass (`run_ctx_t.q`), and the first pass goes straight into the copy of its mode. The telemetry pass is out of line (the tunables pass, the longer, falls through).
  - *The post pass:* after a tunables pass the next pass runs in the same copy and, after its CMP1 write, forms the target of the new tunables, picks the reference step for the next 64 passes (`buck_ramp_mode`) and takes that step itself. Until 28 Oct the tunables pass did this; the values are the same and are used from the same passes. **The observer gains' exact check stays in the tunables pass** (and at the loop entry): user, 28 Oct (option P3, which moved it into the next pass, was rejected).
  - *busy_max:* the window's second CYCCNT read goes through the DWT base the pass holds in a register (no literal-pool load inside the window).
  - *NaN payloads:* where the new register allocation would swap the operands of an FP operation that can have two NaN operands, the order is pinned to the 25 Oct build's (`buck_mul_nm` / `buck_add_nm`): the current scaling, α·e, avg_pred + α·e, iv_pred + κ·e, the score's two additions, the current PI's kp·e (kp first in the first pass, e first in the copies) and, in the post pass only, the observer's prediction v̂ + a·(i_L − î) (the product first, `buck_obs_step_ao`; user, 28 Oct, second set). Everywhere else the prediction keeps the compiler's order, which is the 25 Oct build's: the product first in the copies' ordinary, telemetry and tunables passes, v̂ first in the first pass. In the post pass of every copy GCC had put v̂ first; the check after the change found it (*Verification*), where it changed `out.i_load`'s NaN payload in one narrow case. The pin costs 0 clocks in both counters (FP2 included). The clamped PI sums are not pinned (user, 28 Oct): the sum I + Ki·T·e + Kp·e of the voltage PI and of the current PI, and the current PI's product Ki·T·e, which feeds only its sum, can meet two NaNs with their operands swapped, but the clamp turns a NaN sum into the lower limit, so their differing payloads cannot reach any output.
  - *Tunables:* a memory barrier right after `c->t = b->tune` (GCC may otherwise re-load the tunables from `g_buck.tune` in the post pass, as a `BUCK_ADC_CSR_POLL = 0` build did, which would take a debugger write one pass early). With `BUCK_ADC_CSR_POLL = 0` the passes use `wait_sample()`.
  - *Flags and order:* `RUN_CODE` is `hot, optimize("O3", "no-if-conversion", "no-crossjumping", "no-schedule-insns")` and the copies come in the order HOLD, UP, DOWN, FREE_UP (25–28 Oct, the build that ran on the board on 27 Oct: `"no-tree-coalesce-vars", "no-tree-ter"`, order UP, FREE_UP, HOLD, DOWN). Chosen from 35 flag sets × all 24 orders (840 builds): no worse than the 25 Oct evening build on any target below in either counter, and the best on the most of them.
- **Counters.** Two clock counters, both on the linked ELF with the static model of this section and the bench's wait states (nominal; pessimistic with CCM literals at +2 and no decay of the write penalties). Counter 1 costs the paths the differential emulator executes (26 scenarios); counter 2 walks every path of the disassembly. Both are scratch tools, not in the project. "Table" is the convention of the tables above (body plus the closing status read, 2 / 3 / 3 clocks); "CMP1 after EOC" runs from the status read that finds EOC to the `strlo`; "with head" from that read to the next pass's first status read. Static / nominal, the 25 Oct evening build in brackets; where the two counters agree they share a row.

| figure | 28 Oct (bin 5b99a0d5; 2f144a53 the same) | [25 Oct evening (bin e5c7798f)] |
|---|---|---|
| CMP1 after EOC, every pass but the first, worst (c1 = c2) | 68 / 74 | [c1 82 / 88; c2 79 / 85 with a poll head, 82 / 88 with a late head] |
| CMP1 after EOC, normal pass, HOLD / FREE_UP / UP / DOWN (c1 = c2) | 68 / 67 / 68 / 68; 74 / 73 / 74 / 74 | [78 / 78 / 79 / 79; 84 / 84 / 85 / 85] |
| normal pass (table), HOLD / FREE_UP / UP / DOWN (c1 = c2) | 137 / 138 / 143 / 143; 145 / 146 / 151 / 151 | [140 / 143 / 147 / 147; 147.06 / 150.06 / 154.06 / 154.06] |
| normal pass, mean of the four copies (c1 = c2) | 140.25 / 148.25 | [144.25 / 151.31] |
| `busy_max` window, HOLD / FREE_UP / UP / DOWN (c1 = c2) | 137 / 137 / 142 / 142; 144 / 144 / 149 / 149 | [143 / 146 / 150 / 150; 150.12 / 153.12 / 157.12 / 157.12] |
| telemetry pass (table) (c1 = c2) | 171 / 172 / 178 / 178; 179 / 180 / 186 / 186 | [182 / 185 / 189 / 189; 190.12 / 193.12 / 197.12 / 197.12] |
| tunables pass with head, longest per copy (c1 = c2) | 205 / 205 / 210 / 210; 215.12 / 215.12 / 220.12 / 220.12 | [c1 263 / 263 / 271 / 265, 286 / 286 / 294 / 288; c2 266 / 263 / 271 / 268, 289 / 286 / 294 / 291] |
| longest pass with head: the UP and DOWN tunables pass (c1 = c2; pessimistic) | 210 / 220.12 (222) | [271 / 294 (309)] |
| pass after a tunables pass, with head (not a target) | c1 175 / 187.25; c2 177 / 189.25 | [an ordinary pass: 154 / 161.06] |
| first pass after arming, EOC to the next head (c1 = c2; pessimistic) | 188 / 204.5 (212) | [c1 219 / 248.31 (268); c2 221 / 250.31 (270)] |
| first pass, CMP1 after EOC (c1 = c2) | 68 / 74 | [c1 82 / 95.44; c2 81 / 94.44] |
| arming check's counter read to the first pass's guarded counter read (pessimistic) | c1 144 / 153.12 (155); c2 141 / 150.12 (152) | [c1 194 / 220.19 (239); c2 195 / 221.19 (240)] |
| arming check's counter read to the arming CMP1 write (c1 = c2; pessimistic) | 29 / 33 (33) | [88 / 102.62 (112)] |
| loop entry, in the stopped period (not a target; pessimistic) | c1 120 / 139.12 (156); c2 124 / 145.25 (164) | [c1 103 / 111.5 (119); c2 103 / 113.62 (123)] |
| stopped cycle that sees RUN, its poll read to the arming wait (c2; pessimistic) | 157 / 181.25 (200) | [136 / 149.62 (159)] |
| worst normal pass if VLDR/VSTR take 2 clocks (FP2 hypothesis) | c1 151 / 159; c2 150 / 158 | [c1 156 / 163.06; c2 154 / 161.06] |

  The two counters count the arming path by different conventions (3 clocks apart at its longest): compare it within one counter. The longest pass now ends 70 clocks before the end of the 280-clock period static, 60 nominal and 58 pessimistic (25 Oct evening: 9 static, and 14 nominal and 29 pessimistic past the period, which only delays the next pass), and every pass's CMP1 write comes 10–14 clocks earlier after EOC, static and nominal (the first pass's 14 static, 21 nominal).
- **Against the 25 Oct evening build** the 28 Oct build is no worse on any of these targets, static, nominal, pessimistic or under FP2, in either counter: CMP1 after EOC, the normal pass (worst copy and mean), the `busy_max` window, the longest pass, the first pass and its CMP1 write, the arming path and the arming write. Two figures are higher, neither a target: the pass after a tunables pass (175 / 187.25 by counter 1 against an ordinary pass's 154 / 161.06: it now carries the target and ramp-step work, writes CMP1 at 68 / 74 after EOC and is 33–35 clocks shorter than the longest pass), and the loop entry in the stopped period (120 / 139.12 against 103 / 111.5: it loads the arming's and the first pass's constants; the stopped cycle that sees RUN still reaches the arming wait at most 200 clocks after the status read that finds its EOC, pessimistic, about 250 into the period, before the next end of conversion at about 313). Against the first scouting round's best set (SET I, a scratch build, never in the project) it is no worse on these targets static and nominal, but **not under FP2**: there every normal pass is 1 clock longer (151 against 150 by counter 1), and so is FREE_UP's `busy_max` window, static (144 against 143 by counter 1, 143 against 142 by counter 2; nominal 151 against 151.06 by counter 1); HOLD's window is 1 clock shorter and UP's and DOWN's are equal, so the worst copy's window is equal.
- **The arming.** The arming CMP1 write comes 29 / 33 / 33 clocks after the arming check's counter read, so at 4096 ticks it lands 161 clocks into the period (pessimistic), 103 before 264 (in the 25 Oct evening build 24–37). `BUCK_ARM_LATEST` is unchanged. The first pass's head comes at most 88 clocks (pessimistic) after that read, at most 216 clocks into the arming period, so the first pass still waits for its own sample at every accepted arming point.
- **`busy_max` on the board:** expect about 144–149 if the model holds (about 167–179 with 16–20 % more). Part of the drop from the 25 Oct evening build is the measurement itself: the window's second CYCCNT read no longer follows a literal-pool load of the DWT base inside the window (2 clocks static, about 3 nominal in that build), so the board reads a few clocks less for the same work. The window figures above include this.
- **The stamps.** The arming check now reads its counter about 9 clocks earlier after its sample (23 clocks nominal from the poll read that finds EOC, counter 2; 32.06 before), and the first pass reads its counter about 20 earlier after its own (67 nominal from its status read; 87.44 before). Expected values therefore move (the 25 Oct evening figures in brackets): `arm_cnt` about 1800–2350 ticks [2100–2650]; `8448 − first_cnt` about 4700–5250 ticks, 146–164 clocks [4000–4600, 126–144]; `first_cyc − arm_cyc` about 315–334 [326–345]. `trip_cyc` is read 11 clocks static after the counter read in every ordinary copy and 10 in the first pass, whose counter read pipelines behind its own CYCCNT read: each trip branch starts with a literal load of the DWT base from CCM and then the read (14 / 13 with the counter read's wait, about 1 more nominal for the literal's; before, 11–16 static; built with `BUCK_ADC_CSR_POLL = 0`, one branch has a register move between the two loads).
- **`make check`** passes with the checker unchanged: 9 guarded CMP1 writes (the first pass, and two per copy: its own passes and its post pass) and the arming write. `make test` prints the same output as before, line for line (134 lines: the compile command, the test program's name and its 132 lines of output). A `BUCK_ADC_CSR_POLL = 0` build fails `make check` ("the path of an ordinary pass not found: no pass head (ADC12_CSR read) found": its rules find the passes by that read); so does the 25 Oct evening build built that way, and the user left the checker as it is (28 Oct).
- **Decisions (user, 28 Oct, first set):** apply the recommended merged build; the gains' check stays in the tunables pass (P3 rejected); no further NaN pins (the observer's prediction and the clamped PI sum unpinned, the latter being three two-NaN sites whose NaN payloads cannot reach an output: both clamped PI sums and the current PI's product Ki·T·e, which feeds only its sum; the prediction was pinned in the post pass later that day, second set below); `trip_cyc`: comments only, no code change; `busy_max` and all five stamps are kept; `make check` under `BUCK_ADC_CSR_POLL = 0` is left failing; the stamp rows are not added to the workspace's Live Expressions. Rejected in the scouting rounds (user): S3 (the first pass as an ordinary pass of its copy: the first pass 43 clocks shorter, but the arming path back at the 25 Oct length and two stamps lost), C10 (the arming CMP1 from a 16 KB CCM table), B3 / B8 (the tunables pre-written into an appended, debugger-writable `g_buck` field).
- **Decisions (user, 28 Oct, second set),** after the verification of the applied build (2f144a53…) found, and reproduced twice, that the post pass of every copy adds the observer's prediction with v̂ first, so that `out.i_load` could publish another NaN payload than the 25 Oct evening build (BITID-1):
  1. Pin that addition with the product first in the post pass only; the ordinary passes and the first pass keep their instructions (`buck_obs_step_ao` in `Inc/buck_ctrl.h`, called with `epoch != 0` in `run_pass()`): bin 5b99a0d5…, the same size, both counters' outputs identical to 2f144a53's (FP2 included), and only the post pass's instructions changed (the four `vadd`s swap their operands, and the 4 to 11 instructions after each are reordered: most move by one place; in the UP, DOWN and FREE_UP copies one multiplication moves two places and one subtraction five, three in FREE_UP; no store or peripheral access moves). The first set's "no further NaN pins" rested on a wrong premise, given to the user with the options: that the instruction-level NaN check guards the unpinned sites. That check sees an operand swap only where two NaNs of different payloads meet in one of its scenarios; the operand-order check (`opsorder.py`) records the operand order of every FP addition and multiplication the scenarios execute, NaN or not, and compares it with the 25 Oct evening build's pass by pass, so it sees a swap at every such instruction that both builds execute in the same pass with two different operands, and it showed this one. It never compares eight post-pass instructions, which run one pass later than in that build (it formed them in the tunables pass): the ramp target's multiplication by `BUCK_V_PER_COUNT` and `buck_ramp_mode`'s addition of `BUCK_RAMP_FAR_V`, in each of the four copies. Each has a constant operand, so a swap there cannot change a NaN payload, and the conclusion holds.
  2. FPSCR (its cumulative exception flags IOC, OFC, … and the NZCV flags) is not part of the results: no code reads the flags (the only FPSCR access is `vmrs APSR_nzcv, fpscr` right after an FP compare, which reads the NZCV that compare has just set; *Verification*).
  3. A debugger write to `g_buck.tune` must be taken in the same pass as in the 25 Oct evening build; the instants of the reads inside the tunables pass may move (the third set, below, accepts the measured instants, the loop entry's included). The figure given to the user with this decision, "2–37 clocks earlier", was wrong. Measured over the 901 scenarios (the first load of each field, in clocks after the pass's EOC, at 1.0×; the 25 Oct evening build against 5b99a0d5…): in the tunables pass the 12 fields other than the observer gains are read 0–47 clocks earlier (each field's 1st to 99th percentile within 3–45 earlier) and the gains from 13 earlier to 9 later. At the loop entry (the stopped cycle that sees RUN, which a debugger halt can delay, as in the scenario `halt_in_entry`) `vref_counts` is read 35–54 clocks later, the gains 22–38 later, `kp_v` from 5 earlier to 14 later and the other ten from 28 earlier to 9 later. `g_buck.tune` is read in the same passes in both builds (13 285 passes, no scenario differs). A write that lands between the two builds' reads of a field is taken one reload apart, by the build that reads that field first only at its next reload: in the tunables pass that is mostly the 28 Oct build (it takes the write a reload later), at the loop entry, for `vref_counts` and the gains, the 25 Oct evening build (the 28 Oct build takes it a reload earlier). A write that spans several fields can be split between old and new values differently.
  4. The RAM-execution linker script's overflow (P8: 3096 B, and it no longer fits with `-DBUCK_BENCH=0`) is left as it is (*Findings to decide on*).
- **Decisions (user, 28 Oct, third set),** after the check of the pin found that the figure for decision 3 given to the user, "2–37 clocks earlier" for the 12 fields other than the observer gains in the tunables pass, was wrong (it was measured on one scenario, `plain`, and does not reproduce there: in `plain`'s ten tunables passes those fields moved 5–42 clocks earlier, 5–31 in pass 87) and that at the loop entry `vref_counts` and the gains are read later (35–54 and 22–38 clocks), which the user had not been shown:
  1. The measured instants of the reads of `g_buck.tune` are accepted, the loop entry's included (figures under decision 3 above); documents only, no code change. Not taken: reading the tunables earlier at the loop entry (a code change, not explored).
  2. The comment-only edits beyond the pin are kept: in `Src/buck.c` "(27 Oct)" became "(scouted 27 Oct, applied 28 Oct)" at five places; in `Inc/buck_ctrl.h` the comments on `buck_iload_obs()` no longer say that `run_pass()` calls it (the host tests do; `run_pass()` runs the same step through `buck_obs_step_ao` and `buck_iload_dac_k`), and the pin's comment says "so did the 28 Oct build before this pin (bin 2f144a53)". They change no instruction (bin 5b99a0d5…).
  - Corrected at the same time, documents only: decision 1's description of the operand-order check (the eight post-pass instructions it never compares), the line count of `make test` (134, not 133) and, in the memo, the unpinned clamped PI sums (three sites, as under *NaN payloads*).
- **Still open (unanswered, kept as they are):** the wiring checks (V3-1: wiring mistakes in `Src/buck.c` that pass `make`, `make check` and `make test`, listed under *Verification*).

The rest of this section describes Buck_Current_Mode.

Static count of the linked code, in core clocks (nominal, pessimistic in brackets), without peripheral wait states. "Quiet" is S ≤ S0/128, the two-stage load-current average; "fast averages" is S > S0/128 and up to 264 passes after it (66 with 32 × 32) (Buck_Current_Mode's ladder, replaced here on 24 Oct; see the Buck_Current_Mode README). The four copies of the pass are HOLD (steady state), FREE_UP (soft-start ramp, all but the last 66–130 steps), UP (the last steps of a ramp up, and ramps to 16 V or more) and DOWN:

| Pass | Original | 27 Sep | Now, quiet | Now, fast averages |
|---|---|---|---|---|
| normal, HOLD / FREE_UP / UP / DOWN | 229–235 / 247–253 (265–293) | 167 / 170 / 177 / 177 (179–190) | 174 / 177 / 183 / 182 (187–197) | 208 / 211 / 217 / 216 (226–236) |
| every 64th pass: telemetry | — | 213–223 (243–254) | 226–235 (258–268) | 230–256 (263–292) |
| every 64th pass: tunables, with the copy selection | — | 227–255 (257–290) | 238–264 (269–300) | 242–268 (274–305) |

These counts take a register-offset store (one per pass) as 1 clock; the Cortex-M4 TRM gives it 2, which adds 1 to every pass (the 27 Sep verification quotes that rule). They also take VDIV and multi-register pushes as 1 clock.

Averaged over 64 passes: 175.8 in steady state and 179.1 during the soft-start (27 Sep: 168.7 and 171.9; the build before the optimisation round: 166.9 and 184.0). The faster load-current averages cost 7 clocks on every pass for their gate (the quiet column) and about 35 on the passes where they run; on the telemetry and tunables passes they did not run (A2 kept its value there; in BUCK_Adpt_Learning the filter, and in this project the observer, runs on every pass): the tunables pass stays within 4 clocks of its quiet length, the telemetry pass within 21 (it still reports the shorter average in `out.i_load`). Over 64 passes of a load step in steady state the average is about 210. The only division in a pass is one VDIV in the tunables pass (one reciprocal for both trigger denominators, 14 clocks), so its longest path is about 281 core clocks nominally (UP, the last steps of a ramp; 269 in HOLD); with bus wait states it can run past the period and delay the next pass by a few tens of clocks, which leaves that pass clear of its deadline (the 27 Sep verification found 66 clocks of margin with 45 wait states, nominal count, when the tunables pass was 13 clocks shorter). The load-current window has no division at all (shifts, a count of leading zeros and constants). Before 27 Sep one pass did both the telemetry and the tunables with two VDIVs and a UDIV, about 300–325 core clocks; on the board a build with it tripped fault 5 (27 Sep). That pass is the leading suspect, together with a restart whose first pass was that pass (see *Every 64th cycle*); the board data (`trip_cnt`, `loops`, `busy_max`, `bench`) have not been read yet.

The ten peripheral accesses per pass add an estimated 9–25 wait states. `g_buck.bench` measures them on the board: a boot-time benchmark (see *Diagnostics* below).

On the board, `g_buck.busy_max` records the control work of every 64th pass, from reading the samples to just before the telemetry writes, about 60 clocks less than the telemetry pass. The board read about 220–240 on the build before the 27 Sep round. Statically that window is now about 166 / 168 / 174 / 173 clocks (HOLD / FREE_UP / UP / DOWN, TRM rule; 159 / 161 / 168 / 168 on 27 Sep), up to 4 more while the fast averages run, so expect roughly 205–230. Fault 5 would come at about 240–250 (the pass after the tunables pass is the limit), so keep it below about 240. Write 0 after start-up to see the steady-state value.

Estimator implementation
------------------------

`Inc/buck_ctrl.h` maps one-to-one onto the specification's equations; the comments name each one. a_on, b_on, a_off, b_off and A_I/H_I depend only on the on-time. The on-time can take only the 3585 CMP1 values between 896 and 4480. `buck_pade_rows()` therefore evaluates `buck_pade_coeffs()`, a direct transcription of the specification, once per CMP1 value at start-up (84 KB of SRAM, 24 B per row). Each cycle does one lookup, which is identical to computing the coefficients online but costs a table index instead of about 30 FP operations and a divide. A row holds a_on/2, b_on/2, 2·a_off, b_off, A_I/H_I and d/2, so the prediction needs no ×0.5 and no CMP1-to-duty conversion per cycle. Power-of-two scaling is exact, so the results are bit-identical to the direct form (`buck_est_predict()`), which the host tests check.

- The prediction uses the corrected valley î_v, v_out at the start of the cycle and the constant V_in.
- The correction applies κ_I = α_I·A_I/H_I to the valley and α_I to the average.

**Load current: fixed-gain observer** (user, 25 Oct). The specification's per-cycle estimate Î[k] = î̄_L[k] − (C/T)·(v[k+1] − v[k]) multiplies the difference of two A0 samples by C/T = 162 A/V; one A0 count (0.806 mV) is 0.13 A, so ADC noise makes it jump by amperes, and BUCK_Adpt_Learning filtered it. Here A2 and `out.i_load` come from the user's observer instead, once per pass, after the CMP1 write and the prediction (`buck_iload_obs()`, which the host tests call; `run_pass()` runs the same step through `buck_obs_step_ao()`, with the post pass's prediction order pinned, and writes A2 as an I/O store):

```
v̂ += a·(i_L − î);   e = v − v̂;   v̂ += β·e;   î −= γ·e        (predict, then correct)
```

- **Inputs.** a = T/C = 1/162 V/A, the model constant (`BUCK_OBS_A`). v = v_out[k+1], this pass's two-sample mean (the end of cycle k). i_L = the raw mean of cycle k's four A1 samples (`buck_adc_to_i`), not the estimator's î̄_L[k]: î̄_L carries the dead-time bias (−1.36 / +0.34 A at 1 / 3 A with body diodes, simulated), which the observer would pass on, while the measured average is accurate to a few tens of mA from about 1 A up. The estimate drives only A2 and `out.i_load`; the current loop still uses î̄_L.
- **What it is.** With β = 1 − p² and a·γ = (1 − p)², the observer is exactly two identical first-order stages y ← y + (1 − p)·(Î − y) on Î[k] = i_L[k] − (C/T)·(v_out[k+1] − v_out[k]): its transfer function is (1 − p)²/(1 − p·z⁻¹)² (proved in the observer study; checked by `make test` within 1e−9 A in double and 1e−4 A for the float firmware, observed 1.2e−14 and 4.8e−6 A). Its DC gain is 1, so the estimate's mean is the measured average's.
- **Gains.** Live as `tune.obs_beta` and `tune.obs_gamma` (A/V). The defaults put a double pole at τ = 9.2 µs (user, 25 Oct): p = exp(−T/τ) = 0.83430, β = 1 − p² = 0.3039396, γ = (1 − p)²·C/T = 4.447826 A/V (4.4478256; the float default is 4.4478254; the decision's 4.447825 is that value truncated, not rounded). They are evaluated at compile time from τ with BUCK_ALPHA_I's 7-term series for 1 − p (relative error 1.7e−10; the floats are the same as with libm's exp, which `make test` checks from τ = 9.2 µs written in the test and against the digits 0.3039396 and 4.447826). Other double poles from the observer study: τ = 4 µs: β = 0.5654018, γ = 18.81095; 7.9 µs: 0.3442272, 5.860643; 12 µs: 0.2425349, 2.724140; 16 µs: 0.1880637, 1.585354; 33.32 µs: 0.0951988, 0.385629.
- **Stable region and the check.** With the model v[k+1] = v[k] + a·(i_L[k] − I) and a constant load I, the errors x₁ = v − v̂ and x₂ = I − î after a pass move as x₁′ = (1 − β)·(x₁ − a·x₂), x₂′ = γ·x₁ + (1 − a·γ)·x₂, and the same matrix moves the observer's own state, so its poles are the roots of P(z) = z² − (2 − β − a·γ)·z + (1 − β). The Jury conditions P(1) = a·γ > 0, P(−1) = 4 − 2β − a·γ > 0 and |1 − β| < 1 give **γ > 0 and 0 < β < 2 − a·γ/2** (so β < 2 and γ < 648 A/V). Nothing further is needed: 1 − a·γ > −1 is not (a·γ = 3, β = 0.25: both poles at |z| = 0.866). On the boundary a pole is on the unit circle, so it is rejected. When the tunables are reloaded (the tunables pass, every 64 passes, and the loop entry; not every pass), a pair outside the region or with a NaN is not taken up and the gains in use stay; at the loop entry, which has no gains in use, the compiled defaults run instead. The test is on the pair: a new β is judged with the new γ.
- **The check is exact in float** (`buck_obs_gains_ok()`): β + (a/2)·γ < 2 is decided without FMA for every pair of floats (NaN, ±inf, ±0, subnormals). The rounding error of (a/2)·γ is formed exactly by Dekker's product (both factors split into halves of at most 12 significant bits), and 2 − x is formed with x = β when β ≥ 1, else x = the rounded product, so that it is exact; the comment in `Inc/buck_ctrl.h` gives the proof. `make test` compares it with an exact reference over 84 million pairs: special values, random bit patterns, random positive pairs, the floats next to the boundary line on both sides, and the 80 pairs exactly on it. A plain float test, β < 2 − (a/2)·γ rounded, accepts 32 of those 80 points on the line, rejects 38 floats just inside it and accepts 29 just outside (a mutant of that kind fails `make test`).
- **No NaN guard on the states** (user, 25 Oct): a NaN or ±inf that reached v̂ or î would stay until the next start (v comes from an integer ADC reading and i_L from the A1 calibration tunables).
- **Start.** At arming (inside `run_loop()` since 25 Oct, right after `buck_ctrl_arm()`; before, at the loop entry right after `arm()`) the observer starts from the arming sample: î = its measured current `i_meas` (which arming also publishes as `out.i_load`), v̂ = its v_out (`buck_obs_seed()`). The gates were off, so this is about 0 A, and A2 rises to the load with the double pole after arming.
- **Per pass:** 3 multiplies, 5 additions/subtractions and the A2 code (î·409.5 truncated, USAT 0…4095). No division anywhere, also not in the telemetry pass or the stopped loop (BUCK_Adpt_Learning formed its filter coefficients there with a VDIV).
- **Noise and speed** (simulation, default gains, 2 A steps): ±2 counts of noise on every A0 and A1 sample leave 5.7 / 5.7 mA rms on A2 at 1 / 3 A (BUCK_Adpt_Learning's filter on î̄_L: 6.6 / 6.3 mA; the observer study predicted about 5.9 mA for this observer on the measured average). Of that, the A0 part of Î (0.187 A rms per cycle) falls to 5.1 mA, the factor 0.027 the double pole gives differenced white noise. The bias is +0.027 / −0.018 A with body diodes and −0.015 / −0.017 A with ideal dead time (BUCK_Adpt_Learning: −1.36 / +0.34 A and −0.003 / −0.001 A). A load step reaches 50 % / 90 % on A2 after 15.0 / 88.3 µs up and 13.3 / 41.7 µs down with body diodes, 15.0 / 45.0 µs both ways with ideal dead time (normalised to A2's own levels, as before). The slow 90 % up is the load itself: v_out dips about 0.2 V after the step, so the resistive part of the true load falls to 0.875 of the step and recovers with v_out. BUCK_Adpt_Learning's filter: 27 / 60 µs up with noise (its score speed-up), 73 µs down (observer study). Overshoot after the steps 1.6 / 1.7 % (the filter: 0.5 / 2.8 %).
- **Profile tracking** (the observer study's harness with these settings, not among the host tests): on a 2 kHz square load between 1 and 3 A with 1 µs edges, A2's rms error is 380 mA with body diodes (371 mA after removing the static bias) and 380 mA with ideal dead time, against 1087 (956) and 445 mA for BUCK_Adpt_Learning's filter. The same harness gives 5.90 / 5.98 mA rms of noise over 30 ms windows (the host test's 2 ms windows: 5.7 / 5.7) and the same bias and step times as `make test`.
- **What to expect on A2** (0.330 V per A): about 1.9 mV rms of the simulated noise; slower content passes. At 1 A A2 now reads about 1.02 A, where BUCK_Adpt_Learning's estimate was negative (A2 at 0 V).
- **ESR.** The model's capacitor has no ESR. With an ESR, v_out also moves by ESR·(i_L − i_o), and 162 A/V × ESR of the capacitor current enters Î (0.81 of it at 5 mΩ): the simulated 5 mΩ plant keeps the bias (+0.022 / −0.018 A) but the step down reaches 90 % after 130 µs instead of 42 µs.
- **If A2 looks noisy:** take a few dozen Live Expression snapshots of `out.i_load`, which is exactly the value A2 gets on that pass. If it is steady but the scope is not, the noise is in the analog path (probe ground, 600 kHz switching pickup, the DAC's 600 kHz staircase: use a short ground spring at A2, the scope's 20 MHz limit, or averaging triggered on D7). Faster gains (e.g. the 4 µs pair above) show more of the per-cycle content.
- **Telemetry.** `out.i_load` is î of the telemetry pass. Which gains are in use is not published: a rejected write leaves `tune.obs_beta`/`obs_gamma` showing the written values while the previous ones run (left so, user, 25 Oct).


Build
-----

- **STM32CubeIDE:** the settings are installed (25 Oct, with CubeIDE closed): BUCK_Adpt_Learning's `.cproject`, `.settings/*` and `BUCK_Adpt_Learning Debug.launch` with every `BUCK_Adpt_Learning` replaced by `Buck_disturb_obs` (the launch renamed `Buck_disturb_obs Debug.launch`). Build Debug or Release: the CMSIS include paths, `STM32G474xx`, `-O3` and `-ffp-contract=off` are set in both configurations. If a CubeIDE build lacks the CMSIS paths or uses `-Os`, CubeIDE has overwritten them.
- **Command line** (uses CubeIDE's GCC):

```bash
make -j4
```

```bash
make test
```

```bash
make check
```

The linker scripts place the control loop in CCM SRAM (`.ccmtext`, copied at start-up) and limit RAM to the physical 96 KB. The CCM alias starts above that. RAM use is 89 112 B of 96 KB (90.6 %) with the 84 KB Padé table (FLASH script, which both CubeIDE configurations and the Makefile use; BUCK_Adpt_Learning: 89 128 B; Buck_Current_Mode: 91 176 B with its 2 KB load-current rings). The RAM-execution script `STM32G474RETX_RAM.ld` also loads code and data into RAM: since 28 Oct 101 400 B of 98 304 B, 3096 B too many, and 2072 B too many with `-DBUCK_BENCH=0` (no configuration uses it; 25 Oct evening: 98 328 B, 24 B too many, 97 304 B with `-DBUCK_BENCH=0`; before the fix 97 304 B and 96 280 B). (Buck_Current_Mode overflowed it by 40 B before the 128 × 128 change and by 1576 B after.)

Bring-up and Live Expressions
-----------------------------

1. With the power stage unpowered, flash and run. Check that `g_buck.state = 0` (stopped) and `g_buck.fault = 0`, and that `g_buck.adc_i4_raw / 4` reads close to 0 at 0 A. Set `g_buck.tune.isense_zero` to the measured value.
2. Scope D7/D8 after writing `g_buck.command = 1`. You should see 600 kHz, 50 ns gaps on both edges, and D4 low. Write `0` to stop.
3. With the stage powered and current limited, start again. Watch `g_buck.out.{v, i_meas, i_avg, eps, i_load, i_ref, i_peak, vref, score, g, cmp}`, `g_buck.busy_max` and `g_buck.loops` (600 000 per second).
4. Faults latch until reset, with both gates off:
   - 4: ADC overrun or missing sample
   - 5: CMP1 deadline missed (the loop is too slow on average; note `busy_max`)
   - 1/2/3/6: start-up timeouts
   - 8: CPU fault

   Do not halt the core with the stage energised: the timer keeps running, but the loop stops updating.

The Live Expressions list is in `tools/live_expressions.txt`, 48 rows (BUCK_Adpt_Learning's 42 with `tune.iload_tau0`, `tune.iload_s_th` and `out.i_load_alpha` replaced by `tune.obs_beta` and `tune.obs_gamma`: 41; since 25 Oct evening the start's timing stamps `arm_cnt`, `arm_cyc`, `first_cnt`, `first_cyc`, `trip_cyc` and two rows derived from them). Every row is checked with gdb against the Makefile's ELF (25 Oct evening; the 28 Oct build has the same `g_buck`). The seven stamp rows are not in the workspace's Live Expressions list: the user chose not to add them (28 Oct).

**Controls** (writable):

| Row | Meaning |
|---|---|
| `command` | start / stop |
| `state` | 0 stopped, 1 running, 2 faulted |
| `fault` | latched fault code |
| `tune.*` | V_ref (in counts, and in V), both PI gains, α_I, S_th (D4), the score coefficients `score_a`, `score_b`, `score_c` (S = a·e_v² + b·e_v + c), the load-current observer's gains `obs_beta` and `obs_gamma` (A/V), A1 calibration; a write takes effect within 64 cycles (107 µs). An `obs_beta`, `obs_gamma` pair outside the stable region (γ > 0, 0 < β < 2 − γ/324) or with a NaN is not taken up and the gains in use stay; nothing shows that, so write a pair that stays inside at every step |

**Live in every state**, for the pre-bias check and the A1 zero calibration:

| Row | Meaning |
|---|---|
| `adc_v_raw*3.3/8192.0` | v_out, V (`adc_v_raw` is the sum of the cycle's two A0 samples) |
| `adc_i4_raw/4.0` | A1 in counts: write its settled value into `tune.isense_zero` |
| `(adc_i4_raw/4.0-tune.isense_zero)*tune.isense_gain` | A1 current, A; about 0 once calibrated |

**Live only while running**, updated every 64 cycles. They keep the last published values after a stop (at most 64 cycles before it):

| Row | Meaning |
|---|---|
| `out.v`, `out.vref` | v_out and the reference in use |
| `out.v-out.vref` | e_v |
| `out.i_meas`, `out.i_avg` | measured and corrected-estimate cycle-average current |
| `out.eps`, `out.i_peak` | residual and peak estimate |
| `out.i_load` | the load current as A2 shows it: the observer's estimate î (at arming the measured current, where the observer starts) |
| `out.i_ref` | u[k] |
| `out.score`, `out.g` | S = a·e_v² + b·e_v + c (can be negative with a < 0, b ≠ 0 or c < 0) and g (the same g drives D4) |
| `out.cmp` | CMP1, ticks |
| `((int)out.cmp-272)/8960.0` | physical duty D |

**Diagnostics:**

| Row | Meaning |
|---|---|
| `busy_max` | worst control work per pass in clocks, sampled every 64th pass (from the pass start to the telemetry read: statically 137 / 137 / 142 / 142 for HOLD / FREE_UP / UP / DOWN since 28 Oct, 144 / 144 / 149 / 149 with the bench's wait states, both clock counters; the window's second CYCCNT read no longer follows a literal-pool load, so the board reads a few clocks less for the same work. 25 Oct evening: 143 / 146 / 150 / 150, about 150–157; before that 141 / 143 / 151 / 149; BUCK_Adpt_Learning 151 / 154 / 160 / 160; the telemetry pass does about 35 more after it, 40 before 28 Oct); 25–30 clocks less than Buck_Current_Mode's 162–172 (16–25 in the 25 Oct evening build), whose limit was about 240; write 0 to restart |
| `trip_cnt` | after a fault 5: the Timer A counter at the missed CMP1 write, in ticks (8960 per period, 32 per CPU clock). Below 2048: the pass was more than a period late. With `loops % 64` (0: the pass after the tunables pass, 32: after the telemetry pass, 8 or 40: most likely the first pass after arming, where `loops` starts) it says which pass missed. A pass more than one period and 64 clocks late reads a wrapped counter inside the window: it writes CMP1 a period late, and the next pass trips fault 4 instead |
| `trip_cyc` | after a fault 5: CYCCNT (CPU clocks, 168 per µs) read at the start of the trip branch (since 28 Oct in every pass, the first pass after arming included, right after a literal load of the DWT base from CCM; in a `BUCK_ADC_CSR_POLL = 0` build one branch has a register move between them), 11 clocks in the ordinary copies and 10 in the first pass, whose counter read pipelines behind its own CYCCNT read (static; 14 / 13 with the counter read's wait, about 1 more nominal for the literal's) after the issue of the counter read in `trip_cnt` (25 Oct evening: 11–16, 14–19 with the counter read's wait: its first instruction in the first pass; in the ordinary copies after two register moves and a stack load, and in FREE_UP, UP and DOWN a taken branch); 0 until a fault 5 (it latches until reset). `trip_cyc − arm_cyc` = clocks since the arming check: about 280 per pass, so it says which pass tripped (within 25.6 s of the start: CYCCNT wraps) |
| `arm_cnt`, `arm_cyc` | the last start's arming check: the Timer A counter it read (accepted 897…4095 ticks: where in the period arming happened; about 1800–2350 expected since 28 Oct: the end of conversion, 0–9 clocks of polling and 23 clocks to the read, nominal; 2100–2650 with 32 clocks in the 25 Oct evening build) and CYCCNT read by the instruction just before it. 0 until the first start. Written by the first pass after arming together with `first_*` (once per start), but not when that pass ends in `buck_fatal()` (fault 4 by the sample timeout, i.e. no end of conversion for 840 polls, or fault 8), nor by a sample timeout while waiting to arm: the four stamps then still hold the previous start's values, and nothing in `g_buck` tells them apart (a fault 4 from a bad sample in the first pass does write them, with `first_*` = `0xFFFFFFFF`) |
| `first_cnt`, `first_cyc` | the last start's first pass: the counter its guarded CMP1 check read, and CYCCNT read by the instruction just before it (the arming check reads its pair the same way). `0xFFFFFFFF` if that pass ended (STOP or bad sample) before its check; on its fault 5 they are written too (`first_cnt` = `trip_cnt`). `first_cnt` below 2048: the counter had wrapped, the check came more than a period late (fault 5) |
| `8448-(int)first_cnt` | the first CMP1 write's margin in ticks (32 per clock; negative: fault 5) when its check fell in the period after arming, as it normally does. About 4700–5250 (146–164 clocks) expected since 28 Oct: the first pass normally waits for its sample and checks at EOC + 0–9 + 67 clocks, nominal (about 4250–4900 with the board's 16–20 % on that pass; 25 Oct evening build: 4000–4600, 125–144 clocks, EOC + 0–9 + 87, 3450–4150). No margin when `first_cnt` is below 2048: the check came more than a period late (fault 5), and the row reads 6401–8448. 8449 when `first_cnt` = `0xFFFFFFFF`; 8448 (`first_cnt` 0) before the first start |
| `first_cyc-arm_cyc` | CPU clocks from the arming check to the first pass's check. As the first pass normally waits for its sample, about 280 + (its check's place in its period − the arming check's place): 315–334 expected since 28 Oct (about 322–343 with the board's 16–20 %; 25 Oct evening build: 326–345, 335–356). It equals `(first_cnt + n·8960 − arm_cnt)/32` within a clock or so, n being the period starts between the two reads: n = 1 when the check fell in the period after arming, as it normally does (the formula with 8960); each period later adds 280. Much more than 334 (345 in the 25 Oct evening build) with `first_cnt` late (n = 1) means the first pass started late (it did not wait), i.e. the arming path itself was slow. Meaningless when `first_*` = `0xFFFFFFFF` (the row then shows `0xFFFFFFFF − arm_cyc`); 0 before the first start |
| `loops` | control cycles run (600 000 per second while running; it skips ahead by up to 31 at each start) |
| `adc_miss` | ADC overruns or missing results (while stopped harmless, for example after a debugger halt; a stop no longer adds to it) |
| `warmup` | must reach 0 before a start is possible |
| `rst_cause>>25` | reset cause: 1 option-byte load, 2 NRST pin, 4 brown-out, 8 software, 16 IWDG, 32 WWDG, 64 low-power |
| `bench` | bus benchmark, measured once at boot before the loop starts: extra CPU clocks per access over the same access to SRAM1 (13 values, order in `Inc/buck.h`). Reads: [0] ADC12_CSR, [1] ADC1_DR, [2] ADC2_DR, [3] DWT_CYCCNT, [4] HRTIM Timer A CNTR. Back-to-back writes: [5] CMP1, [6] DAC1, [7] GPIOB_BSRR. Each write followed at once by an SRAM load: [8]–[10]. [11] CMP1 write then a CCM load. [12] a CCM literal load while running from CCM. Build with `-DBUCK_BENCH=0` to leave it out |

Live Expressions polls only a few times per second, so `out.g` and `out.score` are occasional snapshots; scope D4 and A2 for every cycle. `g_buck.st.*` is intentionally not in the list: the controller state lives in CPU registers while running and is written back only on a stop.

CubeIDE reads the workspace copy of the list when the Live Expressions view opens, and overwrites it when the view or the IDE closes. To restore this list, close CubeIDE, then run from this folder:

```bash
cp tools/live_expressions.txt ../.metadata/.plugins/com.st.stm32cube.ide.mcu.livewatch/saved_expr.dat
```

Verification
------------

`make test` runs the exact controller code on the host:

- **Against the specification.** All 3585 table entries match a double-precision transcription of the equations within 2.2e-7 relative error. Prediction and correction match within 3.6e-7 (relative to 10 A) over 200 000 random states. The observer error ratio equals A_I(1 − α_I) within 1.1e-5.
- **Closed loop.** The switched converter is integrated with RK4 at ≤ 4 ns steps. The model includes dead time with body-diode conduction, a first-order 650 kHz current sensor, 12-bit ADCs at the firmware's sampling instants, and the 2-cycle pipeline. The run arms at 1 ms, soft-starts, then steps the load 1 A → 3 A at 15 ms and back at 20 ms. The same samples also go, every cycle, through the direct forms the firmware no longer runs (the unscaled table row with the CMP1 duty, and the per-cycle ramp behind its `vref_reached` gate); every output and state bit must match.
- **Trigger score.** The firmware's S against a·e_v² + b·e_v + c in double over 2 million random a, b, c and e_v of both signs (47 % of the scores negative): within 2.0e-7 of |a|·e_v² + |b·e_v| + |c|; bit-identical to e_v² at the defaults over a million values of e_v (zeros, subnormals, overflow); NaN, ±inf and negative scores through `buck_ctrl_slow()` (NaN: D4 low; +inf: D4 high; negative: D4 low while S_th > S). Over 300 000 random tunables D4 = (S ≥ S_th) exactly, and S, g and the controller state are the same bits with other observer gains.
- **Load-current observer.** One step of `buck_obs_step()` against the same step in double from the same state, over 2 million random states, inputs and gains inside the stable region (double and complex poles): within 0.09 × 1e−6·(1 + |î| + 2γ). The identity: from the seed, the recursion with β = 1 − p², γ = (1 − p)²/a equals the two first-order stages on Î[k] = i_L[k] − (C/T)·(v_out[k+1] − v_out[k]) within 1e−9 A in double (observed 1.2e−14 A; τ = 4, 9.2, 16, 33.32 µs, random input walks), and the float firmware with the default gains is within 1e−4 A of them (observed 4.8e−6 A). a against T/C, and the default gains against 1 − p² and (1 − p)²·C/T with libm's exp from τ = 9.2 µs written in the test, not `BUCK_OBS_TAU_S` (the same floats), and against the digits 0.3039396 and 4.447826; the A2 code against floor(î·409.5) clipped to 0…4095; the seed after `buck_ctrl_arm()` (v̂ = its v_out, î = its i_meas = `out.i_load`, both against the ADC sums converted in double) over 200 000 random readings and calibrations.
- **The gains' check.** The rounding error of (a/2)·γ against the exact one for every mantissa of γ at the exponents −96, 0, 9 and 127 (33.5 million, all exact). `buck_obs_gains_ok()` against an exact reference of the open region in double over 84 million pairs: every pair of 30 special values (NaN of both signs, ±inf, ±0, subnormals, FLT_MAX, 2 and its neighbours, 648 and the float below it), 4 million random bit patterns, 8 million random positive pairs, 72 million floats next to the boundary line on both sides, and the 80 points exactly on it with their neighbours (on the line: rejected; below: accepted; above: rejected); no difference. The region itself against the spectral radius of the error dynamics in double over 2 million pairs away from the boundary (no difference), and a·γ = 3, β = 0.25 inside. `buck_obs_k()` over 20 000 random reload sequences (NaN, ±inf, negative values, random bits): the tunables' pair when inside, otherwise the previous gains bit for bit, the compiled defaults at the loop entry.
- **Closed loop with the observer on A2.** Every cycle runs the firmware's own `buck_iload_obs()` with `buck_obs_seed()` at arming and `buck_obs_k()` at the entry and every 64 cycles, as `run_loop()` does, and checks: the A2 code; the seed; the recursion in double (within 1e−4 A; observed ≤ 5e−6 A); with the default gains the two stages (within 1e−4 A; observed ≤ 5e−6 A) and, in double, the identity (within 1e−9 A; observed about 1e−14 A); the bias at 1 / 3 A below 0.05 A (the measured average's accuracy); with ADC noise the A2 noise below 8 mA rms (the observer study: 5.09 mA fed î̄_L, 14–15 % more fed the measured average, about 5.9 mA) and the A0 part reduced to at most 1.5 times the double pole's factor for white noise; without noise and ESR, the 50 % times below 20 µs and the 90 % times below 110 µs up and 50 µs down (the study: 11.7 / 86.7 µs up and 11.7 / 35.0 µs down at τ = 8 µs). With `tune.obs_beta` = 2 (outside: 2 > 2 − γ/324) the defaults run, and A2 is bit for bit the defaults' observer. The direct form of the controller runs with other observer gains and must match bit for bit (the controller does not read them).
- **Firmware wiring.** `make test` compiles `Inc/buck_ctrl.h` and the tests only (its output is the same, line for line, before and after the 25 Oct evening fix, which changes no line of `Inc/buck_ctrl.h` code, and after the 28 Oct optimisation, whose new forms in `Inc/buck_ctrl.h` are the same operations on the same values). It does not see how `Src/buck.c` compiles or wires them: the 28 Oct build's identity with the 25 Oct evening build rests on the differential emulator below. How `Src/buck.c` uses it is checked by `make check`'s tunables-reload rule (every tunable read at the loop entry, i.e. before the arming wait, and in the passes, the observer gains not on the path of an ordinary pass; see *CPU budget*) and otherwise by review. Of the arming (25 Oct evening), `make check` catches the CMP1 write after the outputs-enable write or without it, the first pass's write unguarded, the first pass's CYCCNT read moved after its `strcc` (in the same `asm` or in another after it: the load then lies between the `strcc` and the branch to the trip, and with another `asm` GCC also replaces the `bcs` by a flag held in a register), and the gains' check, the copy of the tunables or the score coefficients moved past the wait (each tried as a mutant); review only (each passes `make`, `make check` and `make test`, tried): arming without the window check, without the ADC1 re-check, with 5120, or with the window's upper bound off by one (4096 accepted); a refused arm that does not retry (the window's or the ADC1 re-check's refusal ending the wait); `buck_ctrl_arm()` run on `g_buck.tune` instead of the entry's copy; the entry's `asm` removed or Ki·T moved past the wait (only the clock count shows it); a bad sample while waiting not counted (until 26 Oct `make check` failed it on false messages: GCC used `g_buck`'s register as a scratch on that return path, see *CPU budget*), or not ending the wait; a STOP ignored while waiting; the raw codes not published while waiting, or the memory barrier after the wait for the sample removed; `out.*` not published, the state written to `g_buck.st` instead of the loop's registers, v̂ seeded at 0, `state` not set; the arming check's CYCCNT read after its counter read, the stamps stored before the first CMP1 write, the first pass's stamps not stored after its CMP1 write, on its trip or on a stop, `trip_cyc` not stored. No check covers the rest of that wiring, for example the order inside the reload, which values `run_pass()` feeds the observer (v_out and the measured average), where A2 is written (after the CMP1 write), what `out.i_load` publishes, whether the gains' check decides which gains run, the entry's fallback to the compiled defaults, how the observer starts at the entry, or which value drives D4.
- **Reference ramp.** The firmware's epoch scheme (one step mode per 64 cycles) is checked cycle by cycle against the original per-cycle ramp over 58 680 runs (up, down, retargets mid-ramp and back, all epoch phases, targets up to 35 V), and the FREE_UP bound over 400 000 epochs at its edge.
- **The 28 Oct optimisation, bit for bit** (scratch tools, not in the project; 28 Oct, on the final ELF of this project, bin 5b99a0d5…). A differential emulator runs the linked code of both builds (the 25 Oct evening build, bin e5c7798f…, rebuilt from a backup of the project, and the 28 Oct build) instruction by instruction on the same ADC sample streams and debugger writes, and compares every peripheral write and its order inside each pass (`--strict-order`), every `g_buck` field a debugger can see, pass by pass (`busy_max`, `bench`, `rst_cause` and the start stamps, which measure time, are reported, not compared), and every fault and its moment. Suites: 901 scenarios (the 887 of the standard suite: arming, late passes, every copy and transition, ADC faults, clamps, NaN and ±inf tunables and NaN pairs set before RUN, random tunables, the earlier verifiers' groups; and 14 more around the tunables and post passes), 1641 (the 887, 46 at the post pass, and 708 with debugger writes at the post pass's sample, gain sequences and NaN pairs per transition), 1069 (the 887 and 182 with NaN tunables taken up at a tunables pass and a STOP right after the post pass), and the `vbit` group of the verification after the change, 352 scenarios aimed at the post pass (random and special tunables, NaN, ±inf, subnormal sensing and score values, every copy transition, stops and restarts). Each runs with the 28 Oct build emulated at 1.0×, 1.3× and 0.6× speed: every scenario the same, except 5 of the 708 (`vt_tr_stall_*`), where after a debugger stall the 25 Oct evening build trips its own fault 5 and the 28 Oct build does not (with that build at 0.9× speed they are the same too). Before the pin (bin 2f144a53…) 14 of the 352 differed, at all three speeds: when the measured current overflows to ±inf in a tunables pass (only with extreme sensing tunables written by the debugger, for example a current gain of the order of 1e35 or more with a full-scale sample: v̂ becomes the default NaN and î ±inf) and the sensing tunables that pass takes are NaN, that build published the default NaN in `out.i_load` from the next telemetry pass on where the 25 Oct evening build publishes the tunable's NaN (the A2 code is 0 in both). Two instruction-level checks look at the operand order. The NaN check (`nanops.py`) records every FP operation with two NaN operands, in the 611 NaN scenarios of the standard suite and in the 352: at the pinned sites (current scaling, average, valley, score, the current PI's product kp·e, the observer's prediction) the 28 Oct build gives the 25 Oct build's result in every case there; it differs only at the clamped PI sums (the voltage and the current PI's sum, and the current PI's product Ki·T·e, which feeds only its sum: 116 and 276 cases of pass, operation and operands), whose NaN the clamp turns into the lower limit, so no output, telemetry value or `g_buck` field can carry it. That check sees an operand-order difference only where two NaNs of different payloads meet. The operand-order check (`opsorder.py`, added by the verification after the change) records the order of the operands of every executed FP addition and multiplication whatever their values, in the 901 and the 352 scenarios: every pinned site keeps the 25 Oct build's order, the observer's prediction now included (before the pin it was swapped in the post pass of every copy: v̂ first, the product first in the 25 Oct build). The other operations whose operand order changed cannot meet two NaN operands, or their result cannot reach an output. A load check: `g_buck.tune` is read in exactly the same passes in both builds. With `BUCK_ADC_CSR_POLL = 0` both build without warnings, `run_loop` has no ADC12_CSR read, and the 901 scenarios are the same at all three speeds against the 25 Oct evening build built the same way (`make check` fails for both, see *CPU budget*, 28 Oct). Both clock counters (*CPU budget*) reproduce the figures of the scouting rounds exactly; their outputs are byte for byte the same for 2f144a53… and 5b99a0d5….
  *Scope* (user, 28 Oct, second and third sets). The results compared are the peripheral writes, the `g_buck` fields and the faults above, in the emulator's scenarios and with its peripheral model; timing on the board is not modelled beyond the clock counters. FPSCR is not part of the results: its cumulative exception flags (IOC, OFC, …) and its NZCV flags can differ from the 25 Oct evening build's (the gains' check now forms both difference orders unconditionally and can set IOC or OFC where that build sets none), and no code reads them (the code's only FPSCR access is `vmrs APSR_nzcv, fpscr` right after an FP compare, which reads the NZCV that compare has just set; there is no `vmsr`). A debugger write to `g_buck.tune` is taken in the same pass as in the 25 Oct evening build, which is the requirement (the emulator injects debugger writes at pass boundaries); the instants of the reads moved (decision 3: in the tunables pass from 47 clocks earlier to 9 later, at the loop entry from 28 earlier to 54 later), which the user accepted, the loop entry's included (third set); a write that lands between the two builds' reads of a field is taken one reload apart. The "2–37 clocks earlier" given here before was wrong.

| Plant | v_out error (as measured) | DC output − V_ref at 1 A / 3 A | Dip / overshoot (2 A step) | A2 (observer) error at 1 A / 3 A | î̄_L error at 1 A / 3 A (the current loop's feedback) | Peak S on the steps |
|---|---|---|---|---|---|---|
| dead time exactly as modelled (node 0 V) | < 0.3 mV | −0.2 / 0.0 mV | 117 / 117 mV | −15 / −17 mA | −3 / −1 mA | 0.014 (e_v²; the same with Δd, 0.018 with Δu) |
| body diodes in dead time | < 0.1 mV | +0.3 / −0.2 mV | 202 / 174 mV | +27 / −18 mA | **−1.36 / +0.34 A** | 0.041 / 0.030 (e_v²; the same with Δd, 0.054 / 0.039 with Δu) |
| + ADC noise ±2 LSB | < 0.1 mV | +0.3 / −0.1 mV | 203 / 174 mV | +24 / −20 mA (5.7 / 5.7 mA rms; BUCK_Adpt_Learning's filter on Î: −1.36 / +0.34 A, 6.6 / 6.3 mA rms) | −1.36 / +0.34 A | 0.041 / 0.030 (e_v²; the same with Δd, 0.062 / 0.040 with Δu) |
| + 5 mΩ ESR | < 0.1 mV | +6.8 / +6.6 mV | 198 / 170 mV | +22 / −18 mA | −1.30 / +0.40 A | 0.039 / 0.029 (e_v²; the same with Δd, 0.052 / 0.040 with Δu) |

"As measured" is the mean of the two A0 samples, which the loop regulates; the DC output is the cycle average a meter would read. The difference is the output ripple between the samples and the cycle average, mostly the ESR's.

A 2 A step reaches 50 % / 90 % on A2 after 15.0 / 85.0 µs up and 13.3 / 41.7 µs down with ADC noise (15.0 / 88.3 and 13.3 / 41.7 µs without, 15.0 / 45.0 µs both ways with the plant matching the model; the slow 90 % up is the true load's own dip with v_out, see *Load current*). Every figure of the table outside the A2 column is unchanged, bit for bit (the observer drives only A2).

D4 fires when S ≥ S_th. With the default S_th = 1 it therefore stays low for these 2 A steps; the tests also run the model-matched plant with S_th = 0.01, where D4 fires for 21 / 20 cycles on the steps (as with Δd; 35 with Δu) and never in steady state; with a = 100 the same D4 counts at S_th = 1; and with a = 100, b = 5, c = −0.25, S is −0.25 at rest, 0.53 at the dip and 1.7 at the overshoot, so D4 fires only on the step down (27 cycles). A2 is the same in all of them (the score does not reach it).

The loop is stable and regulates in every case. The dip matches what a 6 kHz voltage crossover with 270 µF gives, ΔI/(2π·f_cv·C) ≈ 0.2 V.

Findings to decide on
---------------------

- **Estimator bias from the dead-time model.** With real body-diode conduction, the "dead time is off-time" assumption misses up to V_in·t_dt/T_sw of volt-seconds. At light load the valley current is negative, so the high-side diode conducts at the rising edge. The Padé model contracts fast with r_L = 114 mΩ (A_I ≈ 0.56 per cycle), so α_I = 0.033 cannot remove the offset. î̄_L then reads −1.36 A off at 1 A and +0.34 A at 3 A. A2 no longer shows it: the observer takes the measured average (user, 25 Oct), so A2 is +27 / −18 mA off.

  The measured average itself is accurate to ±0.04 A from about 1 A up, and regulation is not affected, because the voltage PI absorbs the offset. Below that, A1 (0 V at 0 A) clips the negative part of the ripple and the measured average reads high: about +0.1 A at 0.5 A and +0.2 A at 0.1 A load, and A2 with it (with L −20 % the simulated A2 is +64 mA off at 1 A). A larger α_I passes more of the measured average into î̄_L. With α_I = 0.3 the bias halves (−0.73 / +0.18 A). Removing it fully would need a sign-dependent dead-time term in the model, which the specification does not have.
- **The observer gains' check costs clocks** (25 Oct): 28–38 clocks in the tunables pass, then the longest pass (267 of 280 in the table convention, 271 with its head). Since 25 Oct evening it is no longer on the arming path (it runs in the loop's entry, a period before arming). It stays in the tunables pass in the 28 Oct build (user), whose tunables pass, still the longest, is 206 in the table convention, 210 with its head (220.12 nominal with the head), of 280. See *CPU budget*.
- **The board's first-pass fault 5 (25 Oct)** came from about 60–80 clocks (16–20 %) on the old arming path that neither the static count nor the bench's wait states explain (hypothesis: VLDR/VSTR at 2 clocks, unproven). The fix takes about 180 clocks off that path, enough even if the excess is real and general. It ran on the board on 27 Oct: the user reported that it works; no data from that run were read. On a board run, `arm_cnt`, `first_cnt`, `first_cyc − arm_cyc` and `8448 − first_cnt` after a start (see *Diagnostics*) measure the arming point and the first write's margin directly, and with `busy_max` they show whether the excess applies to the ordinary passes too (`busy_max` about 144–149 in the 28 Oct build if the model holds, about 167–179 with 16–20 % more; 150–157 and 175–190 in the 25 Oct evening build).
- **The arming window is review only.** `make check` finds the arming CMP1 write by the outputs-enable write after it and exempts it from the deadline guard, but does not check that the window test (897…4095 ticks) and the ADC1 re-check come before it (mutants without them pass every check). A rule for it would need the counter read and the two tests located on every path to that write.
- **The RAM-execution linker script overflows by 3096 B** since 28 Oct (101 400 of 98 304 B; 2072 B with `-DBUCK_BENCH=0`; no configuration uses it). From 25 Oct evening to 28 Oct it overflowed by 24 B and fitted with `-DBUCK_BENCH=0`. Left as it is (user, 28 Oct, second set).
- **No sign of a rejected gain pair.** A write of `tune.obs_beta`/`obs_gamma` outside the stable region (or a NaN) leaves the gains in use as they were, and Live Expressions still shows the written values. The pair is judged as a pair, so writing β and γ one at a time can pass through a rejected or an unintended intermediate pair.
- **ESR is not in the observer's model.** 162 A/V × ESR of the capacitor current enters the estimate (simulated 5 mΩ: the step down's 90 % after 130 µs instead of 42 µs; the bias unchanged).
- **Current-loop gain margin is only 3.2 dB** at the f_sw/10 crossover. The 30 kHz / 6 kHz alternative gives 7.7 dB (see `tools/pi_design.py`).
- **Arming** enables the gates mid-period, so the low side conducts for the rest of that period. The estimator models that period with T_on = 0. With a pre-biased output this gives a brief negative current.
- **Bulb sampling plus triggered oversampling** has run on the board on ADC2 (4 triggers 23.3 ADC clocks apart). ADC1 uses it too since the two-sample v_out change (2 triggers 20 ADC clocks apart), which ran on the board in the build the user ran on 27 Oct (reported working; no data such as `adc_miss` were read). `-DBUCK_ADC_BULB=0` switches both ADCs to standard sampling.
- **ROVSM.** RM0440's Table 176 lists triggered oversampling with ROVSM = 0, which both ADCs use, as "not supported"; its text says ROVSM is ignored under TROVS (resumed mode is forced) and Figure 133 allows it. ROVSM only matters with injected conversions, which the firmware does not use. Setting ROVSM = 1 on both ADCs would match the table and, per the text, change nothing.
- **ADC1 pairing is not monitored.** If an A0 trigger were ever lost, ADC1 would pair samples across cycles from then on and its end of conversion would come about 360 ns before the boundary. Nothing checks for this: while stopped, `command = 1` would be ignored without a fault (the arming check never sees its counter window); while running, fault 5 trips only if the pass reaches the deadline check early enough. By design it should not happen (locked clocks, 5 idle ADC clocks, no ADC stop anywhere). A counter check at ADC1's end of conversion would detect it.
- **Clock accuracy.** HSI tolerance applies to the 600 kHz.
- **V_in is a constant.** Any difference between the real V_in and 6 V shows up as estimator bias (simulated at 6.5 V: −1.85 / −0.40 A).

Files
-----

| File | Contents |
|---|---|
| `Inc/buck_config.h` | pins, timing, parameters, gains, compile-time checks |
| `Inc/buck_ctrl.h` | per-cycle algorithm (PI, Padé estimator, correction, load-current observer and its gains' check, trigger), hardware-independent |
| `Inc/buck.h`, `Src/buck.c` | clock, GPIO, HRTIM, ADC, DAC, the paced control loop, faults |
| `Src/main.c` | init, start, run |
| `Tests/test_ctrl.c` | host tests and closed-loop simulation |
| `tools/pi_design.py` | discrete-time 45°-PM PI design |
| `Drivers/CMSIS` | vendored CMSIS headers (as in Buck_Voltage_Mode) |
