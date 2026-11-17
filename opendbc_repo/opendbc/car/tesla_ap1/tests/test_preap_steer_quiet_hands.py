"""The hands-on hand-back waits for hands below the pause level, not for an absolute level 0.

The pause fires at hso.AP1_HANDS_ON_LEVEL (2). The release test used to demand EPAS_handsOnLevel == 0, which is
stricter than the pause itself: level 1 is any driver torque the EPAS reports at all, it does not pause lateral, and
openpilot steers through it for most of a normal drive (hso.ap1_lat_active). On logged AP1 EPAS the levels seen are
0, 1 and 3 only, and 9 of 14 level 3 overrides dropped to 1 first and reached 0 up to 2.9 s later, so a brief nudge
cost seconds of no steering while the driver was already off the wheel.

What is NOT changed, and these tests hold it down: the pause itself (the driver still wins on the frame they act),
and the torque and wheel-rate checks that decide whether the driver has actually let go.
"""
import pytest

from opendbc.car.tesla_ap1 import override as ov
from opendbc.car.tesla_ap1.hso import (AP1_HANDS_ON_LEVEL, AP1_RESUME_HOLD_FRAMES, Ap1DriverYield,
                                       ap1_lat_active, ap1_steering_pressed)


def test_the_release_level_is_exactly_one_below_the_pause_level():
  """override cannot import hso (hso imports override), so the two constants are checked against each other."""
  assert ov.QUIET_HANDS_MAX_LEVEL == AP1_HANDS_ON_LEVEL - 1


# ---- the detector's quiet flag ------------------------------------------------------------------

def _det(level=ov.SENSITIVITY_BASE_LEVEL):
  return ov.DriverOverrideDetector(level)


def test_hands_below_the_pause_level_can_be_quiet():
  det = _det()
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL, 0.0, 0.0, 0.0)
  assert det.quiet


def test_the_pause_level_is_never_quiet():
  det = _det()
  for hands in (AP1_HANDS_ON_LEVEL, AP1_HANDS_ON_LEVEL + 1, 3):
    det.update(True, hands, 0.0, 0.0, 0.0)
    assert not det.quiet, hands


def test_torque_still_blocks_the_hand_back_below_the_pause_level():
  det = _det()
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL, det.thresholds.quiet_torque_nm, 0.0, 0.0)
  assert not det.quiet


def test_wheel_rate_still_blocks_the_hand_back_below_the_pause_level():
  det = _det()
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL, 0.0, det.thresholds.quiet_rate_dps, 0.0)
  assert not det.quiet


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_the_checks_are_on_magnitude(sign):
  det = _det()
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL, sign * det.thresholds.quiet_torque_nm, 0.0, 0.0)
  assert not det.quiet


# ---- the pause is untouched ---------------------------------------------------------------------

def test_the_pause_threshold_is_unchanged():
  assert not ap1_steering_pressed(0)
  assert not ap1_steering_pressed(1)
  assert ap1_steering_pressed(AP1_HANDS_ON_LEVEL)
  assert ap1_steering_pressed(3)


def test_level_one_still_steers_and_the_pause_level_still_does_not():
  assert ap1_lat_active(True, 1) is True
  assert ap1_lat_active(True, AP1_HANDS_ON_LEVEL) is False


def test_the_driver_still_wins_on_the_frame_they_act():
  y = Ap1DriverYield()
  assert y.update(True, 0) is False
  assert y.update(True, 3) is True, "pause on the first frame at the pause level"


# ---- the yield state machine, with and without a detector ---------------------------------------

def _hold(y, hands, quiet=None, frames=AP1_RESUME_HOLD_FRAMES):
  for _ in range(frames):
    y.update(True, hands, ov.SIGNAL_NONE, quiet)


def test_resume_from_level_one_without_a_detector():
  """The fallback quiet test is the hands flag alone; it now clears at level 1."""
  y = Ap1DriverYield()
  y.update(True, 3)
  assert y.active
  _hold(y, 1)
  assert not y.active, "released at level 1 after the hold"
  assert y.resumed, "resumed is true on the step the hold ends, for the soft-start"


def test_no_resume_while_the_hands_level_still_pauses():
  y = Ap1DriverYield()
  y.update(True, 3)
  _hold(y, AP1_HANDS_ON_LEVEL, frames=AP1_RESUME_HOLD_FRAMES + 50)
  assert y.active


def test_resume_from_level_one_with_a_detector_that_says_quiet():
  y = Ap1DriverYield()
  y.update(True, 3, ov.SIGNAL_HANDS, False)
  assert y.active
  _hold(y, 1, quiet=True)
  assert not y.active


def test_a_detector_that_says_not_quiet_still_blocks_at_level_one():
  """Driver leaning on the wheel with the hands flag down: no hand-back."""
  y = Ap1DriverYield()
  y.update(True, 3, ov.SIGNAL_HANDS, False)
  _hold(y, 1, quiet=False, frames=AP1_RESUME_HOLD_FRAMES + 50)
  assert y.active


def test_the_hold_still_has_to_be_continuous():
  y = Ap1DriverYield()
  y.update(True, 3)
  for _ in range(AP1_RESUME_HOLD_FRAMES - 1):
    y.update(True, 1)
  assert y.active
  y.update(True, 3)                    # a second nudge restarts the count
  for _ in range(AP1_RESUME_HOLD_FRAMES - 1):
    y.update(True, 1)
  assert y.active, "the hold restarted"
  y.update(True, 1)
  assert not y.active


def test_the_hold_length_is_unchanged():
  y = Ap1DriverYield()
  y.update(True, 3)
  for _ in range(AP1_RESUME_HOLD_FRAMES - 1):
    y.update(True, 0)
  assert y.active, "not yet"
  y.update(True, 0)
  assert not y.active


def test_a_disengage_still_clears_at_once():
  y = Ap1DriverYield()
  y.update(True, 3)
  assert y.update(False, 3) is False
  assert not y.active


def test_an_override_signal_still_pauses_regardless_of_the_hands_level():
  y = Ap1DriverYield()
  assert y.update(True, 0, ov.SIGNAL_TORQUE, False) is True
  assert y.signal == ov.SIGNAL_TORQUE
