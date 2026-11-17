"""Pre-AP wind-up rate matches NAP's lateral-jerk envelope, and stays inside this port's panda ceiling.

The table this replaces bottomed out at 0.30 deg per command above 15 m/s, which is 15 deg/s at the Pre-AP 50 Hz
0x488 rate. NAP's limit at the same speed is 0.72 deg per command. NAP's `MAX_ANGLE_RATE = 5` is only the
low-speed cap; the limit that binds while driving is a lateral-jerk budget that falls as 1/v**2
(opendbc lateral.py get_max_angle_delta_vm). This recomputes that budget here from NAP's own constants and
vehicle model and checks the table against it, so the two cannot drift apart silently.

AP1 is untouched: it keeps CarControllerParams, and its 0x488 goes out at ~20 Hz.
"""
import math

import numpy as np
import pytest

from opendbc.car.tesla_ap1.actuator_plan import _ap1_limited_angle, apply_std_steer_angle_limits
from opendbc.car.tesla_ap1.values import CarControllerParams, PreApCarControllerParams

# --- NAP's own numbers (opendbc lateral.py, tesla/values.py, safety/modes/tesla.h) ---------------
ACCELERATION_DUE_TO_GRAVITY = 9.81
AVERAGE_ROAD_ROLL = 0.06
NAP_MAX_LATERAL_JERK = 3.0 + ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL   # ~3.59 m/s^3
NAP_MAX_ANGLE_RATE = 5.0           # deg per command, the low-speed cap
NAP_SLIP_FACTOR = -0.000580374383851451
NAP_STEER_RATIO = 12.0
NAP_WHEELBASE = 2.89
PREAP_FRAME_S = 0.02               # 0x488 at 50 Hz on Pre-AP

# This port's panda for Pre-AP (opendbc/safety/modes/tesla_preap.h), deg per command
PANDA_UP_BP, PANDA_UP_V = [2., 7., 17.], [8., 4., 2.5]
PANDA_DOWN_BP, PANDA_DOWN_V = [2., 7., 17.], [9., 5., 4.5]

OLD_UP_BP, OLD_UP_V = [0., 5., 15.], [10., 1.6, .3]


def nap_max_angle_delta(v):
  """NAP's max angle step per command at speed v, deg (lateral.py get_max_angle_delta_vm, roll 0)."""
  v = max(v, 1.0)
  max_curvature_rate = NAP_MAX_LATERAL_JERK / (v * v)                      # (1/m)/s
  curvature_factor = 1.0 / (1.0 - NAP_SLIP_FACTOR * v * v) / NAP_WHEELBASE  # chi = 0
  deg_per_s = math.degrees(max_curvature_rate * NAP_STEER_RATIO / curvature_factor)
  return min(deg_per_s * PREAP_FRAME_S, NAP_MAX_ANGLE_RATE)


def up(v, limits=PreApCarControllerParams):
  lim = limits.ANGLE_RATE_LIMIT_UP
  return float(np.interp(v, lim.speed_bp, lim.angle_v))


def down(v, limits=PreApCarControllerParams):
  lim = limits.ANGLE_RATE_LIMIT_DOWN
  return float(np.interp(v, lim.speed_bp, lim.angle_v))


SPEEDS = np.arange(0.0, 45.01, 0.05)


# --- the parity claim ----------------------------------------------------------------------------

@pytest.mark.parametrize("v", [5., 6., 7., 8.5, 10., 13., 16., 20., 25.])
def test_breakpoints_are_naps_budget(v):
  """Every breakpoint from 5 m/s up is NAP's number (0.01 deg rounding)."""
  assert up(v) == pytest.approx(nap_max_angle_delta(v), abs=0.01), v


def test_between_breakpoints_it_never_runs_far_above_naps_curve():
  """Linear interpolation of a convex curve overshoots; keep that small."""
  over = np.array([up(v) - nap_max_angle_delta(v) for v in SPEEDS])
  assert over.max() < 0.15, f"worst overshoot {over.max():.3f} at {SPEEDS[over.argmax()]:.1f} m/s"


def test_the_two_documented_departures_from_nap():
  # below ~3 m/s NAP's cap is under the old table, and the cap is used
  assert up(0.0) == pytest.approx(NAP_MAX_ANGLE_RATE)
  assert up(2.0) == pytest.approx(NAP_MAX_ANGLE_RATE)
  # at the top breakpoint the old 0.30 is kept rather than NAP's smaller budget
  assert up(32.0) == pytest.approx(0.30)
  assert nap_max_angle_delta(32.0) < 0.30


# --- it must be a real increase where it matters -------------------------------------------------

def test_authority_is_up_at_every_speed_the_car_drives():
  """At and above 3 m/s the new table is never below the old one."""
  for v in SPEEDS[SPEEDS >= 3.0]:
    assert up(v) >= float(np.interp(v, OLD_UP_BP, OLD_UP_V)) - 1e-9, v


def test_the_speed_the_logged_drive_sat_at():
  """Median 15.0 m/s in the saved drive; p99 demand there was 0.80 deg per command."""
  assert up(15.0) == pytest.approx(0.74, abs=0.02)
  assert up(15.0) / float(np.interp(15.0, OLD_UP_BP, OLD_UP_V)) > 2.0


def test_only_creep_speeds_lose_anything():
  lower = SPEEDS[[up(v) < float(np.interp(v, OLD_UP_BP, OLD_UP_V)) - 1e-9 for v in SPEEDS]]
  assert lower.size and lower.max() < 3.0, lower.max() if lower.size else None


# --- the safety envelope -------------------------------------------------------------------------

def test_nothing_in_the_wind_up_table_can_raise_a_panda_violation():
  """Every wind-up value stays under tesla_preap.h's own per-command ceiling, so the panda cannot reject it."""
  for v in SPEEDS:
    assert up(v) <= float(np.interp(v, PANDA_UP_BP, PANDA_UP_V)) + 1e-9, v


def test_unwinding_above_the_panda_ceiling_below_8_3_ms_is_pre_existing_and_unchanged():
  """Found while checking this change; NOT introduced by it and NOT fixed by it.

  ANGLE_RATE_LIMIT_DOWN (shared, untouched here) sits above what tesla_preap.h allows per command from a
  standstill up to about 8.3 m/s: 10.0 vs 9.0 at 0, 7.60 vs 7.40 at 4, 7.00 vs 6.60 at 5, 5.76 vs 5.00 at 7.
  In that band a fast unwind can be rejected by the panda as a violation rather than merely clipped, which is a
  plausible source of an unexplained disengage while turning out of something at low speed. The same shape was
  in the old wind-up table (10.0 against the panda's 8.0 at a standstill) and this change removed that half.
  Pinned here so it is noticed; whether the table comes down or the panda goes up is the dev's call, and it is
  deliberately not bundled into a change about wind-up authority.
  """
  over = SPEEDS[[down(v) > float(np.interp(v, PANDA_DOWN_BP, PANDA_DOWN_V)) + 1e-9 for v in SPEEDS]]
  assert over.size, "if this is empty the unwind table was fixed; drop this test"
  assert over.min() == pytest.approx(0.0)
  assert over.max() == pytest.approx(8.3, abs=0.1)
  for v in SPEEDS[SPEEDS > 8.4]:
    assert down(v) <= float(np.interp(v, PANDA_DOWN_BP, PANDA_DOWN_V)) + 1e-9, v


def test_the_old_table_had_a_value_above_that_ceiling():
  """Context for the low-speed departure: 10.0 at a standstill was already above the panda's 8.0."""
  assert float(np.interp(0.0, OLD_UP_BP, OLD_UP_V)) > float(np.interp(0.0, PANDA_UP_BP, PANDA_UP_V))
  assert up(0.0) <= float(np.interp(0.0, PANDA_UP_BP, PANDA_UP_V))


def test_unwinding_is_untouched_and_already_at_or_above_nap():
  assert PreApCarControllerParams.ANGLE_RATE_LIMIT_DOWN is CarControllerParams.ANGLE_RATE_LIMIT_DOWN
  for v in SPEEDS[SPEEDS >= 1.0]:
    assert down(v) >= nap_max_angle_delta(v) - 1e-9, v


# --- AP1 untouched -------------------------------------------------------------------------------

def test_ap1_keeps_its_own_table():
  assert CarControllerParams.ANGLE_RATE_LIMIT_UP.speed_bp == OLD_UP_BP
  assert CarControllerParams.ANGLE_RATE_LIMIT_UP.angle_v == OLD_UP_V


def test_ap1_inherits_nothing_from_the_preap_table():
  assert PreApCarControllerParams.ANGLE_RATE_LIMIT_UP is not CarControllerParams.ANGLE_RATE_LIMIT_UP
  for attr in ("JERK_LIMIT_MAX", "JERK_LIMIT_MIN", "ACCEL_TO_SPEED_MULTIPLIER"):
    assert getattr(PreApCarControllerParams, attr) == getattr(CarControllerParams, attr), attr


# --- it is actually applied -----------------------------------------------------------------------

def test_the_limiter_uses_the_table_it_is_given():
  """A big step at 15 m/s is clipped to the table's value, Pre-AP's and AP1's alike."""
  for limits, expected in ((PreApCarControllerParams, up(15.0)), (CarControllerParams, 0.30)):
    out = apply_std_steer_angle_limits(50.0, 0.0, 15.0, limits)
    assert out == pytest.approx(expected, abs=1e-6), limits.__name__


def test_plan_helper_defaults_to_ap1_and_takes_preap_limits():
  assert _ap1_limited_angle(50.0, 0.0, 0.0, 15.0) == pytest.approx(0.30, abs=1e-6)
  assert _ap1_limited_angle(50.0, 0.0, 0.0, 15.0, PreApCarControllerParams) == pytest.approx(up(15.0), abs=1e-6)


def test_the_measured_angle_clip_still_applies_on_top():
  """The +/- 20 deg window around the measured wheel is unchanged by the faster table."""
  out = _ap1_limited_angle(400.0, 300.0, 0.0, 5.0, PreApCarControllerParams)
  assert out == pytest.approx(20.0), out


def test_unwinding_direction_is_unchanged_for_preap():
  """A step toward centre uses ANGLE_RATE_LIMIT_DOWN, which this change does not touch."""
  out = apply_std_steer_angle_limits(0.0, 30.0, 15.0, PreApCarControllerParams)
  assert out == pytest.approx(30.0 - down(15.0), abs=1e-6)
