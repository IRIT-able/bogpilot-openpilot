# Pre-AP: wind-up rate matched to NAP, and the hands-on hand-back

Entries in the `docs/tesla/DIVERGENCES.md` style, kept in their own file so they can be folded into `DIVERGENCES.md`.

Neither change touches the panda, safety, the DBC or any cereal schema. Neither has been driven.

### Pre-AP wind-up rate: NAP's lateral-jerk envelope instead of the 0.9.7 table

- Platforms: Pre-AP Model S. AP1 is untouched and keeps `CarControllerParams`.
- Before: `ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])`, the 0.9.7 limiter
  kept verbatim. Above 15 m/s that is 0.30 deg per command, and Pre-AP sends 0x488 at 50 Hz, so **15 deg/s** of
  wind-up authority at any speed from 34 mph up.
- Evidence: a drive on `7a683247` sat at a median 15.0 m/s (p90 18.3, max 19.4). Its measured wheel movement passed
  that 0.30 ceiling in 1.8% of engaged command windows, with a p99 demand of 0.80 deg per command — so in the
  sharpest part of a curve the limiter, not the car, was what ran out. The debug pack carries no commanded angle
  (`carControl.actuators` holds gas/brake/steer/accel only), so this is measured wheel movement while `latActive`,
  not the command itself.
- What NAP actually does, and the correction it forces: NAP does **not** have a comparable table, and its
  `MAX_ANGLE_RATE = 5` (`tesla/values.py`) is **not** its driving limit — it is the low-speed cap, commented
  "prevent a fault and for low speed comfort". The limit that binds is a lateral-jerk budget
  (`opendbc lateral.py get_max_angle_delta_vm` / `apply_steer_angle_limits_vm`):

      max_curvature_rate = MAX_LATERAL_JERK / v**2                      # (1/m)/s, ~3.59 m/s^3
      max_angle_delta    = deg(VM.get_steer_from_curvature(...)) * STEER_STEP * DT_CTRL
      max_angle_delta    = min(max_angle_delta, MAX_ANGLE_RATE)

  That falls as 1/v**2 and drops below the 5 cap above ~5.5 m/s. Evaluated with NAP's own Tesla vehicle model
  (`tesla.h TESLA_STEERING_PARAMS`: slip_factor -0.000580374383851451, steer_ratio 12.0, wheelbase 2.89) at a
  50 Hz command, NAP allows **0.72** deg per command at 15 m/s, 0.58 at 17, 0.44 at 20 and 0.31 at 25 — not the
  5.00 a flat reading of `MAX_ANGLE_RATE` suggests. At 30 m/s NAP is *below* what this port already allowed
  (0.24 against 0.30).
- Consequence worth stating plainly: **NAP's envelope is inside this port's existing panda ceiling at every
  speed** (`tesla_preap.h angle_rate_up_lookup`: 2.80 per command at 15 m/s against NAP's 0.72). Matching NAP
  needs no safety change at all. Raising the panda to a flat 5 deg per command "to match NAP" would instead make
  this port *more* permissive than NAP above ~5.5 m/s, which is the opposite of parity.
- Our behavior: `PreApCarControllerParams` carries NAP's budget written out in this port's table form, selected
  for Pre-AP in `CarController` via the plan's new `angle_limits`. Per command: 1.60 -> 5.00 at 5 m/s,
  1.34 -> 2.99 at 7, 0.95 -> 1.51 at 10, 0.30 -> 0.74 at 15 (2.5x), 0.30 -> 0.59 at 17, 0.30 -> 0.44 at 20.
- Two deliberate departures from NAP, both so the change only ever adds authority:
  - below ~3 m/s NAP's cap (5.0) is under the old table (10.0 at a standstill) and the cap is used. The old value
    was itself above what this port's panda allows there (8.0), so no reachable command is lost.
  - at the 32 m/s breakpoint the old 0.30 is kept where NAP's budget gives 0.24.
  Everything between is NAP's number, within 0.01 deg at the breakpoints and under 0.15 deg of overshoot in
  between (linear interpolation of a convex curve runs above it; the breakpoints are placed to bound that).
- Unwinding is not touched. `ANGLE_RATE_LIMIT_DOWN` is already at or above NAP's budget at every speed (0.80
  against 0.72 at 15 m/s, 5.76 against 2.99 at 7), so matching NAP there would only slow a return to centre.
- The +/- 20 deg clip around the measured wheel (`_ap1_limited_angle`) is unchanged and still applies after the
  rate step.
- Safety-relevant: it raises how fast openpilot winds the wheel on Pre-AP, by up to 2.5x at normal speeds. Every
  value stays inside the panda's own per-command ceiling, so nothing here can raise a violation, and the ceiling
  that actually bounds the car is unchanged. It has not been driven.
- Tests: `opendbc/car/tesla_ap1/tests/test_preap_angle_rate_nap_parity.py` — recomputes NAP's budget from NAP's
  constants and vehicle model and checks the table against it at the breakpoints and in between, checks the two
  departures, checks authority never drops at or above 3 m/s, checks every wind-up value is inside the panda
  ceiling, checks unwinding and AP1 are untouched, and checks the limiter and the plan helper actually use the
  table they are given.

### Hands-on hand-back: wait for hands below the pause level, not for an absolute 0

- Platforms: AP1 and Pre-AP (the hands pause is shared, `hso.py` / `override.py`).
- Before: the pause fires at `AP1_HANDS_ON_LEVEL` (2), but the release test demanded `EPAS_handsOnLevel == 0` —
  in `DriverOverrideDetector.quiet` (`hands == 0 and torque < quiet_torque and rate < quiet_rate`) and in
  `Ap1DriverYield`'s fallback (`not ap1_driver_input(hands_on_level)`, which is level >= 1). That is stricter
  than the pause: level 1 is any driver torque the EPAS reports at all, it does not pause lateral, and openpilot
  steers through it for most of a normal drive (`ap1_lat_active`). On logged AP1 EPAS the levels seen are 0, 1
  and 3 only, and this file already records that 9 of 14 level 3 overrides dropped to 1 first and reached 0 up to
  2.9 s later. So a brief nudge could cost seconds of no steering with the driver already off the wheel — the
  same latency the resume hold was shortened for (0.8 -> 0.5 -> 0.3 s); this is the other half of it.
- Our behavior: the release test asks for hands at or below `override.QUIET_HANDS_MAX_LEVEL`
  (= `AP1_HANDS_ON_LEVEL - 1`), with the torque and wheel-rate checks unchanged and still required. Resuming at
  level 1 with no torque and a still wheel is the same condition openpilot already steers in continuously, so
  this is not a new state for the EPAS.
- Not changed: the pause itself (the driver still wins on the frame they act), the 0.3 s hold and its
  continuity, `ap1_lat_active`, the quiet torque / rate thresholds and their per-sensitivity scaling, the EPAS
  gate, and the measured-angle soft-start the resume still goes through.
- **This overrides behavior the port asserted on purpose.** Six existing tests asserted "hands flag at any level
  keeps it not quiet"; they are updated here to the new rule. That is a deliberate behavior change, not an
  oversight being corrected, so it is a separate commit from the rate-limit change and can be dropped on its own
  if the dev wants the absolute 0 kept. Nothing was recorded about *why* 0 was required, and the docstrings
  argue the other way, but the dev has the car and the EPAS logs.
- Safety-relevant: it hands lateral back sooner after a hands-on pause, in a state (level 1, quiet torque, still
  wheel) that is already a normal steering state for this port. It has not been driven.
- Tests: `opendbc/car/tesla_ap1/tests/test_preap_steer_quiet_hands.py` — the two constants are checked against
  each other (override cannot import hso), level 1 can be quiet, the pause level never is, torque and wheel rate
  still block below the pause level, the pause threshold and `ap1_lat_active` are unchanged, the driver still
  wins on the first frame, resume from level 1 works with and without a detector, a detector saying not-quiet
  still blocks at level 1, and the hold length and continuity are unchanged.

### Found while checking the above, not fixed here: unwinding above the panda ceiling below 8.3 m/s

`ANGLE_RATE_LIMIT_DOWN` (shared, and untouched by this change) sits above what `tesla_preap.h` allows per command
from a standstill to about 8.3 m/s: 10.0 against 9.0 at 0, 7.60 against 7.40 at 4, 7.00 against 6.60 at 5, 5.76
against 5.00 at 7. In that band a fast unwind can be rejected by the panda as a violation rather than clipped,
which is a plausible source of an unexplained disengage while unwinding out of something at low speed. The old
wind-up table had the same shape (10.0 against 8.0 at a standstill) and this change removed that half of it.
Pinned by a test so it is noticed. Whether the table comes down or the panda goes up is a call for the dev, and it
is deliberately not bundled into a change about wind-up authority.

### An EPAS limit code must not release the wheel (the mid-corner straighten)

- Platforms: Pre-AP worst, AP1 too. Pre-AP was the stricter of the two branches.
- Reported on this build, not reproducible on NAP: halfway through a left turn the EPAS hit a torque limit, all
  lateral control stopped, and the wheel straightened on its own — the car carried straight toward the curb. The
  EPAS had clamped the request and was still steering. openpilot is what let go.
- Two gates did it, both keyed on `EPAS_eacErrorCode` **regardless of `EPAS_eacStatus`**:
  - `epas_gate.epas_not_ready`: any code outside `EAC_OK_ERRORS` (= IDLE, HANDS_ON, HIGH_ANGLE_REQ) blocks the
    gate, and a blocked gate means `CarController` sends steering type NONE — no angle control at all, so the
    EPAS releases and caster straightens the wheel.
  - `steer_fault.steer_fault_temporary`: any code outside the allowed set sets `CarState.steerFaultTemporary`,
    which is `steerTempUnavailable` and drops lateral. Pre-AP takes the `_BENIGN` branch, because
    `is_ap1_family("TESLA_PREAP_MODELS")` is False, so on Pre-AP **every** code except IDLE and HANDS_ON did
    this — code 6 included, stricter than AP1. The docstring attributes the rule to Tinkla
    ("steerWarning unless the name is EAC_ERROR_IDLE"), so it predates the AP1 exceptions.
- The codes in question are limits, not faults. `EAC_ERROR_HIGH_MMOT_SAFETY` (10) is the assist motor's torque
  limit and `EAC_ERROR_HIGH_TORSION_SAFETY` (11) the torsion bar's: the request was larger than the EPAS would
  act on, so it acted on as much as it would and said so. It is still steering, and holding the command finishes
  the turn at whatever it will give.
- NAP's rule, which this adopts: NAP Pre-AP (`opendbc tesla/preap/carstate.py:62-70`) only calls the EPAS a
  rejection when the **status** has gone `EAC_INHIBITED` as well, and then only for the four angle/rate
  validators (6, 7, 8, 9). The torque codes and 12 LOW_ASSIST are not in NAP's set at all. An error code alone
  never stops NAP from commanding — the status does.
- Our behavior: `steer_fault.EAC_LIMIT_ERRORS` collects the **request-origin** codes only — 6, 7, 8, 10, 12 — and
  both gates treat those as clean. The status check is untouched in both: `EAC_INHIBITED` / `EAC_FAULT` /
  `EAC_SNA` still block for every code, so the genuine-rejection case is unchanged and is exactly NAP's condition.
- **The driver-origin codes stay blocking, and the first cut of this change got that wrong.** Blocking is what
  sets `EpasGate.signal = SIGNAL_EPAS` and `steerFaultTemporary`, and that pair *is* the wheel-turn override. A
  first version put codes 5, 9 and 11 in the limit list; driven on this build, turning the wheel past the limit
  then no longer disengaged. They are now their own list, `EAC_DRIVER_ORIGIN_ERRORS`, with module-level asserts
  that they can never appear in either tolerated set:
  - **11 `HIGH_TORSION_SAFETY`** — the torsion bar measures *driver* torque, not the assist motor's. This is the
    driver fighting the wheel, and it is the code the regression was reported against. It is not a sibling of 10
    despite both being "torque".
  - **9 `HIGH_ANGLE_RATE_SAFETY`** — the wheel moved faster than commanded, so something else moved it. NAP uses
    exactly this as its fast override: "FSD disengages using union of handsOnLevel (slow overrides) and high angle
    rate faults (fast overrides, high speed)" (`opendbc tesla/carstate.py`). NAP pairs it with `EAC_INHIBITED`;
    here the code alone keeps blocking, which is the more cautious of the two.
  - **5 `MAX_STEER_DELTA`** — `_ap1_limited_angle` already clips the command to the measured wheel +/- 20 deg, so
    openpilot's own request cannot open a gap that large: the wheel was moved.
- Deliberately still faults, because they are not "I clamped your request": 4 TMP_FAULT (latched, with its own
  recovery in `EpasTmpFaultRecovery`, which must keep seeing it), 13 PINION_VEL_DIFF (the two angle sensors
  disagree — an integrity fault), 14 EPB_INHIBIT, 15 SNA, and 1 MIN_SPEED / 2 MAX_SPEED (outside the EPAS's
  speed window, where commanding achieves nothing). An unrecognised name still fails closed.
- Does **not** supersede the wind-up entry's residual risk: code 9 keeps blocking, so a command fast enough to
  trip the EPAS's own rate check still releases for 0.3 s. That is the right trade — 9 cannot be told apart from a
  driver yank, and for a steering system the cautious reading wins — but it means the faster table's one
  on-car risk stands, and the first drive should still watch for it.
- Safety-relevant: yes, and in both directions. It keeps lateral through an EPAS limit instead of dropping it,
  which is the point; it also means openpilot keeps commanding an EPAS that is reporting a limit. The status
  check is what still stops it, and that is the same condition NAP uses. Not driven.
- Tests: `opendbc/car/tesla_ap1/tests/test_preap_epas_limit_not_fault.py` — the request-origin torque codes block
  neither gate while the EPAS is steering, a sustained limit never starts blocking, the whole limit family is
  clean on both gates and both branches, a refusing status still blocks every code, the real faults and an
  unknown name still block, and TMP_FAULT recovery still sees its code. Plus the override half: the two lists are
  disjoint, every driver-origin code still blocks, still raises `SIGNAL_EPAS` and is still a temporary steer
  fault, turning the wheel past the torsion limit disengages, and a motor-torque limit raises no override signal.
