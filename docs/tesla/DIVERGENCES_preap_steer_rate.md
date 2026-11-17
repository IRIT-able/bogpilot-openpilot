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

### Found while checking the above, not fixed here: unwinding above the panda ceiling below 8.3 m/s

`ANGLE_RATE_LIMIT_DOWN` (shared, and untouched by this change) sits above what `tesla_preap.h` allows per command
from a standstill to about 8.3 m/s: 10.0 against 9.0 at 0, 7.60 against 7.40 at 4, 7.00 against 6.60 at 5, 5.76
against 5.00 at 7. In that band a fast unwind can be rejected by the panda as a violation rather than clipped,
which is a plausible source of an unexplained disengage while unwinding out of something at low speed. The old
wind-up table had the same shape (10.0 against 8.0 at a standstill) and this change removed that half of it.
Pinned by a test so it is noticed. Whether the table comes down or the panda goes up is a call for the dev, and it
is deliberately not bundled into a change about wind-up authority.
