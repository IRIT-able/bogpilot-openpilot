"""Independent driver-override detector (override.py) and how each legacy Tesla controller uses it.

AP1 Model S, AP1 Model X and Pre-AP Model S read the same EPAS_sysStatus torque, hands level and
wheel rate and send the same DAS_steeringControl, so the detector and its numbers are shared.
Every platform case below drives the real controller with synthetic inputs.
"""
from collections import deque
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.tesla_ap1 import override as ov
from opendbc.car.tesla_ap1.hso import AP1_RESUME_HOLD_FRAMES, Ap1DriverYield
from opendbc.car.tesla_ap1.preap.carcontroller import PreApCarController
from opendbc.car.tesla_ap1.tests.ap1_harness import make_controller
from opendbc.car.tesla_ap1.tests.test_preap_carstate_controller import _params_dir  # noqa: F401  (autouse fixture)
from opendbc.car.tesla_ap1.values import CAR, DBC

S, X, P = CAR.TESLA_AP1_MODELS, CAR.TESLA_AP1_MODELX, CAR.TESLA_PREAP_MODELS
PLATFORMS = [S, X, P]


def run(det, n, engaged=True, hands=0, torque=0.0, rate=0.0, angle=0.0, cmd=None):
  out = []
  for _ in range(n):
    out.append(det.update(engaged, hands, torque, rate, angle, cmd))
  return out


# ---- detector ----------------------------------------------------------------------------------

def test_quiet_driving_never_fires():
  det = ov.DriverOverrideDetector(ov.SENSITIVITY_BASE_LEVEL)   # the module constants (level 3), not the default
  # Normal engaged driving seen in logs: torque to 1.3 Nm, rate to 25 deg/s, command tracking within 3 deg.
  sig = run(det, 500, torque=1.3, rate=25.0, angle=5.0, cmd=7.5)
  assert set(sig) == {ov.SIGNAL_NONE}


def test_torque_fires_on_the_third_frame_not_before():
  det = ov.DriverOverrideDetector(ov.SENSITIVITY_BASE_LEVEL)   # the module constants (level 3), not the default
  sig = run(det, 3, torque=-ov.OVERRIDE_TORQUE_NM)
  assert sig == [ov.SIGNAL_NONE, ov.SIGNAL_NONE, ov.SIGNAL_TORQUE]
  det.reset()
  sig = run(det, 2, torque=3.0) + run(det, 1, torque=0.5) + run(det, 2, torque=3.0)
  assert set(sig) == {ov.SIGNAL_NONE}, "a break in the push restarts the debounce"


def test_rate_needs_torque_as_well():
  det = ov.DriverOverrideDetector(ov.SENSITIVITY_BASE_LEVEL)   # the module constants (level 3), not the default
  assert set(run(det, 50, rate=130.0, torque=0.4)) == {ov.SIGNAL_NONE}, "fast wheel with no driver torque is not an override"
  det.reset()
  sig = run(det, 2, rate=-ov.OVERRIDE_RATE_DPS, torque=ov.OVERRIDE_RATE_TORQUE_NM)
  assert sig == [ov.SIGNAL_NONE, ov.SIGNAL_RATE]


def test_angle_error_needs_a_command_and_motion_away_from_it():
  det = ov.DriverOverrideDetector(ov.SENSITIVITY_BASE_LEVEL)   # the module constants (level 3), not the default
  # command ahead by 6 deg, wheel moving away from it
  sig = run(det, 2, angle=0.0, cmd=6.0, rate=-ov.OVERRIDE_ERROR_RATE_DPS)
  assert sig == [ov.SIGNAL_NONE, ov.SIGNAL_ANGLE_ERROR]
  det.reset()
  assert set(run(det, 20, angle=0.0, cmd=6.0, rate=+40.0)) == {ov.SIGNAL_NONE}, "wheel catching up to the command"
  det.reset()
  assert set(run(det, 20, angle=0.0, cmd=None, rate=-40.0)) == {ov.SIGNAL_NONE}, "no active command, nothing to oppose"
  det.reset()
  assert set(run(det, 20, angle=0.0, cmd=3.0, rate=-40.0)) == {ov.SIGNAL_NONE}, "error below the threshold"


def test_hands_flag_is_reported_first_and_instantly():
  det = ov.DriverOverrideDetector(ov.SENSITIVITY_BASE_LEVEL)   # the module constants (level 3), not the default
  assert run(det, 1, hands=2, torque=5.0) == [ov.SIGNAL_HANDS]
  assert run(det, 1, hands=3) == [ov.SIGNAL_HANDS]


def test_not_engaged_clears_everything():
  det = ov.DriverOverrideDetector(ov.SENSITIVITY_BASE_LEVEL)   # the module constants (level 3), not the default
  run(det, 2, torque=5.0)
  assert det.update(False, 0, 5.0, 0.0, 0.0) == ov.SIGNAL_NONE
  assert run(det, 1, torque=5.0) == [ov.SIGNAL_NONE], "debounce restarts after a gap"


def test_quiet_flag():
  det = ov.DriverOverrideDetector(ov.SENSITIVITY_BASE_LEVEL)   # the module constants (level 3), not the default
  det.update(True, 0, 0.3, 4.0, 0.0)
  assert det.quiet
  # Hands at QUIET_HANDS_MAX_LEVEL (1) with no torque and a still wheel counts as released: level 1 never pauses
  # lateral, so the release test is not stricter than the pause test.
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL, 0.0, 0.0, 0.0)
  assert det.quiet
  for kw in (dict(hands=ov.QUIET_HANDS_MAX_LEVEL + 1), dict(torque=1.2), dict(rate=45.0)):
    det.update(True, kw.get("hands", 0), kw.get("torque", 0.0), kw.get("rate", 0.0), 0.0)
    assert not det.quiet, kw


# ---- yield: first frame, first signal, hold -----------------------------------------------------

def test_yield_takes_the_first_signal_and_waits_for_quiet():
  y = Ap1DriverYield()
  assert y.update(True, 0, ov.SIGNAL_NONE, True) is False
  assert y.update(True, 0, ov.SIGNAL_TORQUE, False) is True and y.signal == ov.SIGNAL_TORQUE
  # the hands flag arrives later: the pause keeps the signal that started it
  assert y.update(True, 3, ov.SIGNAL_HANDS, False) is True and y.signal == ov.SIGNAL_TORQUE
  for _ in range(AP1_RESUME_HOLD_FRAMES + 20):
    y.update(True, 0, ov.SIGNAL_NONE, False)
  assert y.active, "torque still on the wheel: no resume"
  for i in range(AP1_RESUME_HOLD_FRAMES):
    y.update(True, 0, ov.SIGNAL_NONE, True)
  assert not y.active and y.resumed and y.signal == ov.SIGNAL_NONE


def test_yield_without_detector_arguments_is_the_hands_pause():
  y = Ap1DriverYield()
  assert y.update(True, 3) is True and y.signal == ov.SIGNAL_HANDS
  for _ in range(AP1_RESUME_HOLD_FRAMES):
    y.update(True, 0)
  assert not y.active
  y.update(True, 3)
  assert y.update(False, 3) is False and y.signal == ov.SIGNAL_NONE


# ---- controllers: AP1 Model S / Model X / Pre-AP -----------------------------------------------

class Car:
  """One controller plus a scripted EPAS/wheel state, 100 Hz."""

  def __init__(self, platform):
    self.platform = platform
    self.t = 0
    self.angle = 0.0
    self.torque = 0.0
    self.rate = 0.0
    self.hands = 0
    self.status = "EAC_ACTIVE"
    self.error = "EAC_ERROR_IDLE"
    self.planned = 0.0
    self.v_ego = 15.0
    self.enabled = True
    self.lat = True
    if platform == P:
      self.ctl = PreApCarController(DBC[P], SimpleNamespace(carFingerprint=P, openpilotLongitudinalControl=False))
    else:
      self.ctl = make_controller(long=False, fingerprint=platform)

  def cs(self):
    out = SimpleNamespace(steeringAngleDeg=self.angle, steeringTorque=self.torque, steeringRateDeg=self.rate,
                          vEgo=self.v_ego, gasPressed=False, standstill=False)
    if self.platform == P:
      return SimpleNamespace(out=out, hands_on_level=self.hands, di_gas=0.0, preap_pedal_detected=False,
                             preap_pedal_raw=None, preap_cc_cancel_needed=False, steer_warning=self.error,
                             eac_status=self.status, eac_fault=False)
    return SimpleNamespace(out=out, steer_warning=self.error, hands_on_level=self.hands, eac_fault=False,
                           eac_status=self.status, acc_state=4, das_control_counters=deque(), msg_stw_actn_req={},
                           cluster_stock={}, stock_steer_counters=[(self.t // 2) % 16], stock_ap_active=False)

  def step(self):
    """Return the 0x488 control type sent this frame (None when no 0x488 went out) and its angle."""
    CC = structs.CarControl()
    CC.enabled = self.enabled
    CC.latActive = self.lat
    CC.actuators.steeringAngleDeg = self.planned
    _out, can = self.ctl.update(CC.as_reader(), self.cs(), self.t * 10_000_000)
    self.t += 1
    for f in can:
      addr, dat = (f.address, f.dat) if hasattr(f, "address") else (f[0], f[1])
      if addr == 0x488:
        return dat[2] >> 6, -(((dat[0] & 0x7F) << 8 | dat[1]) - 16384) * 0.1
    return None

  def run(self, n):
    return [self.step() for _ in range(n)]

  def settle(self, n=200):
    """Engaged, planner angle 0, wheel at 0: soft-start and ramp finished."""
    return [x for x in self.run(n) if x is not None]


def frames_until_none(sent):
  for i, x in enumerate(sent):
    if x[0] == 0:
      return i
  return None


@pytest.mark.parametrize("platform", PLATFORMS)
def test_quiet_engaged_driving_is_never_released(platform):
  car = Car(platform)
  sent = car.settle(300)
  assert all(t == 1 for t, _a in sent[-50:])
  car.torque, car.rate = 1.2, 20.0
  sent = [x for x in car.run(200) if x is not None]
  assert all(t == 1 for t, _a in sent)
  assert car.ctl.ap1_yield.active is False


@pytest.mark.parametrize("platform", PLATFORMS)
def test_hard_push_with_hands_flag_zero_releases_within_the_debounce(platform):
  car = Car(platform)
  car.settle()
  car.torque, car.rate = -5.0, -80.0     # hands flag still 0: the case the EPAS flag missed
  sent = [x for x in car.run(12) if x is not None]
  n = frames_until_none(sent)
  assert n is not None and n <= 2, sent        # 0x488 goes out every other frame: <= 3 control frames
  assert car.ctl.ap1_yield.active and car.ctl.ap1_yield.signal in (ov.SIGNAL_TORQUE, ov.SIGNAL_RATE)
  # while the push goes on, every frame is NONE at the measured wheel
  car.angle = -12.0
  sent = [x for x in car.run(60) if x is not None]
  assert all(t == 0 and abs(a - car.angle) < 0.2 for t, a in sent)


@pytest.mark.parametrize("platform", PLATFORMS)
def test_commanded_angle_opposing_a_swerve_stops_at_once(platform):
  car = Car(platform)
  car.planned = 6.0
  car.settle(300)
  car.angle = 6.0
  car.settle(100)
  car.rate = -40.0
  car.angle = 0.0                          # wheel dragged away from the 6 deg command, no torque reading
  sent = [x for x in car.run(8) if x is not None]
  assert frames_until_none(sent) is not None
  assert car.ctl.ap1_yield.signal == ov.SIGNAL_ANGLE_ERROR


@pytest.mark.parametrize("platform", PLATFORMS)
def test_resume_only_after_the_driver_lets_go(platform):
  car = Car(platform)
  car.settle()
  car.torque = 4.0
  car.run(10)
  assert car.ctl.ap1_yield.active
  car.run(100)
  assert car.ctl.ap1_yield.active, "torque still on the wheel"
  car.torque, car.rate = 0.0, 0.0
  sent = [x for x in car.run(AP1_RESUME_HOLD_FRAMES + 10) if x is not None]
  assert not car.ctl.ap1_yield.active
  assert sent[-1][0] == 1, "lateral is back after the hold"


@pytest.mark.parametrize("platform", PLATFORMS)
def test_override_signal_is_published_by_the_controller(platform):
  car = Car(platform)
  car.settle()
  assert car.ctl.ap1_yield.signal == ov.SIGNAL_NONE
  car.torque = -3.0
  car.run(6)
  assert car.ctl.ap1_yield.signal == ov.SIGNAL_TORQUE


def test_car_state_carries_the_signal_field():
  cs = structs.CarState()
  assert cs.bogpilot.ap1OverrideSignal == 0
  cs.bogpilot.ap1OverrideSignal = ov.SIGNAL_RATE
  assert cs.bogpilot.ap1OverrideSignal == 3
