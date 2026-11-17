"""Override Sensitivity: five levels, level 3 = the module constants, default 2, hard clamps, a bad or missing value is 2.

Scales the driver-override torque and wheel-rate thresholds (and the matching release numbers). It never changes the hands flag, the angle
error signal, the debounce frame counts, the hand-back hold, the EPAS gate or the panda limits, and there is no "off".
"""
import pytest

from opendbc.car.tesla_ap1 import override as ov
from opendbc.car.tesla_ap1.preap import params
from opendbc.car.tesla_ap1.tests.test_ap1_override_detector import Car, PLATFORMS, run
from opendbc.car.tesla_ap1.tests.test_preap_carstate_controller import _params_dir  # noqa: F401  (autouse fixture)

KEY = params.PreApParamKeys.OVERRIDE_SENSITIVITY

# level: (fire torque Nm, torque needed with a fast wheel Nm, fire rate deg/s, release torque Nm)
TABLE = {1: (2.5, 1.25, 33.75, 1.25), 2: (2.25, 1.125, 31.875, 1.125), 3: (2.0, 1.0, 30.0, 1.0),
         4: (1.75, 0.875, 28.125, 0.875), 5: (1.5, 0.75, 26.25, 0.75)}


def save(tmp, text):
  (tmp / KEY).write_text(text)


# ---- the level table ---------------------------------------------------------------------------

def test_default_is_two_and_three_is_todays_numbers():
  assert ov.SENSITIVITY_DEFAULT == 2
  assert ov.SENSITIVITY_BASE_LEVEL == 3
  t = ov.override_thresholds(3)
  assert t.as_tuple() == (ov.OVERRIDE_TORQUE_NM, ov.OVERRIDE_RATE_TORQUE_NM, ov.OVERRIDE_RATE_DPS, ov.QUIET_TORQUE_NM, ov.QUIET_RATE_DPS)
  assert t.as_tuple() == (2.0, 1.0, 30.0, 1.0, 30.0)
  assert ov.DriverOverrideDetector().level == 2
  d = ov.override_thresholds()
  assert d.as_tuple() == ov.override_thresholds(2).as_tuple() == (2.25, 1.125, 31.875, 1.125, 31.875)
  assert d.torque_nm > t.torque_nm and d.rate_torque_nm > t.rate_torque_nm and d.rate_dps > t.rate_dps   # 2 = a firmer push than 3


@pytest.mark.parametrize("level", sorted(TABLE))
def test_each_level_scales_as_designed(level):
  torque, rate_torque, rate, quiet = TABLE[level]
  t = ov.override_thresholds(level)
  assert t.torque_nm == pytest.approx(torque)
  assert t.rate_torque_nm == pytest.approx(rate_torque)
  assert t.rate_dps == pytest.approx(rate)
  assert t.quiet_torque_nm == pytest.approx(quiet)
  assert t.quiet_rate_dps == pytest.approx(rate)
  assert ov.TORQUE_MIN_NM <= t.torque_nm <= ov.TORQUE_MAX_NM
  assert ov.RATE_MIN_DPS <= t.rate_dps <= ov.RATE_MAX_DPS
  assert t.quiet_torque_nm <= 0.5 * t.torque_nm + 1e-9, "release always sits below fire"


def test_levels_run_from_hard_push_to_light_push():
  rows = [ov.override_thresholds(n) for n in (1, 2, 3, 4, 5)]
  for a, b in zip(rows, rows[1:]):
    assert a.torque_nm > b.torque_nm and a.rate_dps > b.rate_dps and a.rate_torque_nm > b.rate_torque_nm


def test_range_matches_the_published_range():
  assert ov.override_thresholds(1).torque_nm == 2.5 and ov.override_thresholds(5).torque_nm == 1.5
  assert 26.0 < ov.override_thresholds(5).rate_dps < 26.5 and 33.5 < ov.override_thresholds(1).rate_dps < 34.0


# ---- clamps and fallback -----------------------------------------------------------------------

@pytest.mark.parametrize("scale", [0.0, 0.01, 0.5, 2.0, 10.0, 1e9])
def test_extreme_scales_are_clamped(monkeypatch, scale):
  monkeypatch.setitem(ov.SENSITIVITY_SCALE, 3, scale)
  t = ov.override_thresholds(3)
  assert ov.TORQUE_MIN_NM <= t.torque_nm <= ov.TORQUE_MAX_NM
  assert ov.RATE_TORQUE_MIN_NM <= t.rate_torque_nm <= ov.RATE_TORQUE_MAX_NM
  assert ov.RATE_MIN_DPS <= t.rate_dps <= ov.RATE_MAX_DPS
  assert ov.QUIET_TORQUE_MIN_NM <= t.quiet_torque_nm <= 0.5 * t.torque_nm and ov.RATE_MIN_DPS <= t.quiet_rate_dps <= t.rate_dps


def test_edited_base_numbers_cannot_escape_the_clamps(monkeypatch):
  monkeypatch.setattr(ov, "OVERRIDE_TORQUE_NM", 3.0)
  monkeypatch.setattr(ov, "OVERRIDE_RATE_DPS", 80.0)
  t = ov.override_thresholds(3)
  assert t.torque_nm == 2.5 and t.rate_dps == 35.0
  monkeypatch.setattr(ov, "OVERRIDE_TORQUE_NM", 0.1)
  monkeypatch.setattr(ov, "OVERRIDE_RATE_DPS", 1.0)
  t = ov.override_thresholds(3)
  assert t.torque_nm == 1.5 and t.rate_dps == 25.0


@pytest.mark.parametrize("bad", [None, "", " ", "abc", "0", "6", "-1", "2.5", "3.0", "1e0", "٣", "0x3", "99999999999999999999", True, False,
                                 0, 6, -3, 2.5, float("nan"), float("inf"), [3], {"a": 1}, b"3"])
def test_bad_values_fall_back_to_the_default(bad):
  assert ov.sanitize_level(bad) == 2
  assert ov.override_thresholds(bad).as_tuple() == ov.override_thresholds(2).as_tuple()
  assert ov.DriverOverrideDetector(bad).level == 2


@pytest.mark.parametrize("good,level", [(1, 1), (5, 5), ("2", 2), (" 4\n", 4), ("05", 5), (3.0, 3)])
def test_good_values_pass(good, level):
  assert ov.sanitize_level(good) == level


def test_saved_value_is_read_from_the_file(_params_dir):
  assert ov.read_sensitivity_level() == 2, "missing file"
  for text, want in (("1", 1), ("3", 3), ("5", 5), ("5\n", 5), ("0", 2), ("7", 2), ("junk", 2), ("", 2), ("2.5", 2)):
    save(_params_dir, text)
    assert ov.read_sensitivity_level() == want, text
  (_params_dir / KEY).unlink()
  (_params_dir / KEY).mkdir()
  assert ov.read_sensitivity_level() == 2, "unreadable"


def test_not_a_compiled_param_key():
  # Car code cannot read Params; the value lives in a file the Steering Tuning page writes. The key must stay out of the compiled table.
  assert KEY == "OverrideSensitivity" and params.DEFAULTS[KEY] == 2 == ov.SENSITIVITY_DEFAULT


# ---- the detector at each level ----------------------------------------------------------------

@pytest.mark.parametrize("level", sorted(TABLE))
def test_torque_threshold_per_level(level):
  fire = TABLE[level][0]
  det = ov.DriverOverrideDetector(level)
  assert set(run(det, 50, torque=fire - 0.01)) == {ov.SIGNAL_NONE}
  det.reset()
  assert run(det, 3, torque=-fire) == [ov.SIGNAL_NONE, ov.SIGNAL_NONE, ov.SIGNAL_TORQUE], "debounce is 3 frames at every level"


@pytest.mark.parametrize("level", sorted(TABLE))
def test_rate_threshold_per_level(level):
  _, rate_torque, rate, _ = TABLE[level]
  det = ov.DriverOverrideDetector(level)
  assert set(run(det, 20, rate=rate - 0.1, torque=rate_torque + 0.1)) == {ov.SIGNAL_NONE}
  det.reset()
  assert set(run(det, 20, rate=rate + 5, torque=rate_torque - 0.05)) == {ov.SIGNAL_NONE}, "a fast wheel with too little torque is no override"
  det.reset()
  assert run(det, 2, rate=-rate, torque=rate_torque) == [ov.SIGNAL_NONE, ov.SIGNAL_RATE], "2 frames at every level"
  det.reset()
  assert set(run(det, 50, rate=200.0, torque=0.0)) == {ov.SIGNAL_NONE}, "never from rate alone"


@pytest.mark.parametrize("level", sorted(TABLE))
def test_quiet_threshold_per_level(level):
  _, _, rate, quiet = TABLE[level]
  det = ov.DriverOverrideDetector(level)
  det.update(True, 0, quiet - 0.01, rate - 0.1, 0.0)
  assert det.quiet
  det.update(True, 0, quiet, 0.0, 0.0)
  assert not det.quiet
  det.update(True, 0, 0.0, rate, 0.0)
  assert not det.quiet
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL, 0.0, 0.0, 0.0)
  assert det.quiet, "hands below the pause level, no torque, still wheel: released"
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL + 1, 0.0, 0.0, 0.0)
  assert not det.quiet, "the hands level that pauses keeps it not quiet"
  det.update(True, ov.QUIET_HANDS_MAX_LEVEL, quiet, 0.0, 0.0)
  assert not det.quiet, "the torque and rate checks still apply below the pause level"


@pytest.mark.parametrize("level", sorted(TABLE))
def test_hands_and_angle_error_do_not_scale(level):
  det = ov.DriverOverrideDetector(level)
  assert run(det, 1, hands=2, torque=0.0) == [ov.SIGNAL_HANDS]
  det.reset()
  assert run(det, 2, angle=0.0, cmd=6.0, rate=-ov.OVERRIDE_ERROR_RATE_DPS) == [ov.SIGNAL_NONE, ov.SIGNAL_ANGLE_ERROR]
  det.reset()
  assert set(run(det, 20, angle=0.0, cmd=ov.OVERRIDE_ERROR_DEG - 0.1, rate=-40.0)) == {ov.SIGNAL_NONE}


@pytest.mark.parametrize("level", sorted(TABLE))
def test_normal_driving_never_fires_at_any_level(level):
  det = ov.DriverOverrideDetector(level)
  # logged engaged driving: torque to 1.3 Nm, rate to 25 deg/s, command tracking within 3 deg
  assert set(run(det, 500, torque=1.3, rate=25.0, angle=5.0, cmd=7.5)) == {ov.SIGNAL_NONE}
  assert set(run(det, 500, torque=-1.45, rate=-20.0, angle=5.0, cmd=7.5)) == {ov.SIGNAL_NONE}


def test_the_unleveled_detector_is_the_default_level_frame_for_frame():
  import random
  rnd = random.Random(7)
  a, b = ov.DriverOverrideDetector(), ov.DriverOverrideDetector(2)
  for _ in range(5000):
    args = (True, rnd.choice((0, 0, 0, 1, 3)), rnd.uniform(-4, 4), rnd.uniform(-90, 90), rnd.uniform(-30, 30),
            rnd.choice((None, rnd.uniform(-30, 30))))
    assert a.update(*args) == b.update(*args) and a.quiet == b.quiet


# ---- the controllers read it once --------------------------------------------------------------

@pytest.mark.parametrize("platform", PLATFORMS)
def test_controllers_use_the_saved_level(platform, _params_dir):
  assert Car(platform).ctl.ap1_override.level == 2, "nothing saved"
  for text, want in (("1", 1), ("2", 2), ("3", 3), ("4", 4), ("5", 5), ("bogus", 2), ("9", 2)):
    save(_params_dir, text)
    assert Car(platform).ctl.ap1_override.level == want, text


@pytest.mark.parametrize("platform", PLATFORMS)
def test_saved_level_changes_when_a_steady_push_releases_steering(platform, _params_dir):
  def releases(level):
    save(_params_dir, str(level))
    car = Car(platform)
    car.settle()
    car.torque = 1.7                      # between the level 5 (1.5) and level 4 (1.75) fire torque, hands flag 0
    sent = [x for x in car.run(20) if x is not None]
    return any(t == 0 for t, _a in sent)
  assert releases(5) and not releases(4) and not releases(3) and not releases(1)


@pytest.mark.parametrize("platform", PLATFORMS)
def test_level_is_read_once_per_controller(platform, _params_dir):
  save(_params_dir, "5")
  car = Car(platform)
  save(_params_dir, "1")
  car.settle()
  car.torque = 1.7
  assert any(x[0] == 0 for x in car.run(20) if x is not None), "the file changed mid-drive: this controller keeps its level"
  assert car.ctl.ap1_override.level == 5
