"""Yield on an EPAS that stops following commands (INHIBITED, FAULT, SNA or an error code), on every legacy Tesla.

The measured case: the EPAS went INHIBITED with HIGH_ANGLE_RATE_SAFETY (9) while openpilot kept
commanding ANGLE at the old target, then more ANGLE at the measured wheel, and the EPB revoked
EAC. A second drive started the same way from INHIBITED with HANDS_ON (3).
"""
import pytest

from opendbc.car.tesla_ap1.actuator_plan import build_actuator_plan
from opendbc.car.tesla_ap1.epas_gate import EAC_OK_ERRORS, EPAS_CLEAN_FRAMES, EpasGate, epas_not_ready
from opendbc.car.tesla_ap1.override import SIGNAL_EPAS, SIGNAL_NONE
from opendbc.car.tesla_ap1.tests.test_ap1_override_detector import PLATFORMS, Car, P, S, X  # noqa: F401  (autouse fixture below)
from opendbc.car.tesla_ap1.tests.test_preap_carstate_controller import _params_dir  # noqa: F401

ERRORS = ["EAC_ERROR_IDLE", "EAC_ERROR_MIN_SPEED", "EAC_ERROR_MAX_SPEED", "EAC_ERROR_HANDS_ON", "EAC_ERROR_TMP_FAULT",
          "EAC_ERROR_MAX_STEER_DELTA", "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HIGH_ANGLE_RATE_REQ",
          "EAC_ERROR_HIGH_ANGLE_SAFETY", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY", "EAC_ERROR_HIGH_MMOT_SAFETY",
          "EAC_ERROR_HIGH_TORSION_SAFETY", "EAC_ERROR_LOW_ASSIST", "EAC_ERROR_PINION_VEL_DIFF",
          "EAC_ERROR_EPB_INHIBIT", "EAC_ERROR_SNA"]


# ---- gate -----------------------------------------------------------------------------------------

@pytest.mark.parametrize("status", ["EAC_INHIBITED", "EAC_FAULT", "EAC_SNA"])
def test_refusing_status_blocks_on_the_first_frame_whatever_the_error(status):
  for err in ERRORS:
    g = EpasGate()
    assert g.update(True, status, err) is True, (status, err)


@pytest.mark.parametrize("status", ["EAC_AVAILABLE", "EAC_ACTIVE"])
def test_only_idle_hands_on_and_the_limit_codes_pass_when_not_refusing(status):
  for err in ERRORS:
    g = EpasGate()
    assert g.update(True, status, err) is (err not in EAC_OK_ERRORS), (status, err)
  # Pinned so the set cannot drift unnoticed. The limit codes are the ones where the EPAS clamped the request and
  # kept steering; blocking on them released the wheel mid-corner. See steer_fault.EAC_LIMIT_ERRORS.
  assert set(EAC_OK_ERRORS) == {"EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON"} | {
    "EAR_ERROR_MAX_STEER_DELTA", "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HIGH_ANGLE_RATE_REQ",
    "EAC_ERROR_HIGH_ANGLE_SAFETY", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY", "EAC_ERROR_HIGH_MMOT_SAFETY",
    "EAC_ERROR_HIGH_TORSION_SAFETY", "EAC_ERROR_LOW_ASSIST",
  }
  # and the codes that are still faults
  for still_faulting in ("EAC_ERROR_TMP_FAULT", "EAC_ERROR_PINION_VEL_DIFF", "EAC_ERROR_EPB_INHIBIT", "SNA",
                         "EAC_ERROR_MIN_SPEED", "EAC_ERROR_MAX_SPEED"):
    assert still_faulting not in EAC_OK_ERRORS, still_faulting


def test_unknown_state_is_not_ready():
  assert epas_not_ready(None, "EAC_ERROR_IDLE") and epas_not_ready("EAC_ACTIVE", None)
  assert epas_not_ready("EAC_ACTIVE", "EAC_ERROR_IDLE", eac_fault=True)
  assert not epas_not_ready("EAC_ACTIVE", "EAC_ERROR_IDLE")


def test_recovery_needs_a_clean_run_and_a_flap_restarts_it():
  g = EpasGate()
  assert not g.update(True, "EAC_ACTIVE", "EAC_ERROR_IDLE")
  assert g.update(True, "EAC_INHIBITED", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
  for _ in range(EPAS_CLEAN_FRAMES - 1):
    assert g.update(True, "EAC_AVAILABLE", "EAC_ERROR_IDLE")
  assert g.update(True, "EAC_INHIBITED", "EAC_ERROR_IDLE"), "one bad frame restarts the wait"
  for _ in range(EPAS_CLEAN_FRAMES - 1):
    assert g.update(True, "EAC_AVAILABLE", "EAC_ERROR_HANDS_ON")
  assert g.signal == SIGNAL_EPAS and not g.resumed
  assert g.update(True, "EAC_AVAILABLE", "EAC_ERROR_HANDS_ON") is False and g.resumed
  assert g.signal == SIGNAL_NONE
  g.update(True, "EAC_AVAILABLE", "EAC_ERROR_IDLE")
  assert not g.resumed


def test_a_lateral_session_that_ends_starts_clean_and_rereads_the_epas():
  g = EpasGate()
  g.update(True, "EAC_ACTIVE", "EAC_ERROR_IDLE")
  assert g.update(True, "EAC_INHIBITED", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
  assert g.update(False, "EAC_INHIBITED", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY") is False
  assert g.update(True, "EAC_INHIBITED", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY") is True, "next engagement reads the EPAS again"
  assert g.update(False, "EAC_AVAILABLE", "EAC_ERROR_IDLE") is False
  assert g.update(True, "EAC_AVAILABLE", "EAC_ERROR_IDLE") is False


def test_arm_on_active_ignores_an_epas_that_never_reached_active():
  g = EpasGate(arm_on_active=True)
  for _ in range(100):
    assert g.update(True, "EAC_INHIBITED", "EAC_ERROR_TMP_FAULT") is False
  assert g.update(True, "EAC_ACTIVE", "EAC_ERROR_IDLE") is False
  assert g.update(True, "EAC_INHIBITED", "EAC_ERROR_TMP_FAULT") is True
  g.update(False, "EAC_INHIBITED", "EAC_ERROR_TMP_FAULT")
  assert g.update(True, "EAC_INHIBITED", "EAC_ERROR_TMP_FAULT") is False, "disarmed again for the next session"


# ---- planner (AP1 Model S and Model X share it) ----------------------------------------------------

def _plan(**kw):
  args = dict(frame=0, lat_active=True, hands_on_fault=False, openpilot_longitudinal_control=False, enabled=True, long_active=False,
              measured_angle_deg=5.0, requested_angle_deg=12.0, last_angle_deg=11.0, v_ego=15.0, accel=0.0, acc_state=4,
              das_counters=__import__("collections").deque(), pcm_cancel=False, chassis_das_only=True, epas_error="EAC_ERROR_IDLE",
              eac_status="EAC_ACTIVE")
  args.update(kw)
  return build_actuator_plan(args.pop("frame"), args.pop("lat_active"), args.pop("hands_on_fault"), args.pop("openpilot_longitudinal_control"),
                             args.pop("enabled"), args.pop("long_active"), args.pop("measured_angle_deg"), args.pop("requested_angle_deg"),
                             args.pop("last_angle_deg"), args.pop("v_ego"), args.pop("accel"), args.pop("acc_state"),
                             args.pop("das_counters"), args.pop("pcm_cancel"), **args)


def test_planner_active_epas_still_gets_the_planned_angle():
  p = _plan()
  assert p.steer.enabled and p.steer.angle_deg > 5.0


@pytest.mark.parametrize("status", ["EAC_INHIBITED", "EAC_SNA"])
@pytest.mark.parametrize("err", ["EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON", "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY"])
def test_planner_never_commands_angle_to_a_refusing_epas(status, err):
  for lat in (True, False):
    p = _plan(eac_status=status, epas_error=err, lat_active=lat)
    assert p.steer is not None and not p.steer.enabled and p.steer.control_type == 0
    assert p.steer.angle_deg == 5.0 and p.apply_angle_last == 5.0, "measured wheel, not the old 11-12 deg target"


def test_planner_epas_yield_flag_alone_blocks_an_available_epas():
  p = _plan(eac_status="EAC_AVAILABLE", epas_yield=True)
  assert not p.steer.enabled and p.steer.angle_deg == 5.0
  p = _plan(eac_status="EAC_AVAILABLE", epas_yield=False, lat_session=True)
  assert p.steer.enabled, "AVAILABLE with nothing wrong still takes the measured-angle hold"


# ---- controllers: Model S, Model X, Pre-AP -------------------------------------------------------

def engaged(platform, planned=0.0, angle=0.0):
  car = Car(platform)
  car.planned = planned
  car.angle = angle
  car.settle(300)
  return car


@pytest.mark.parametrize("platform", PLATFORMS)
@pytest.mark.parametrize("status,err,hands", [("EAC_INHIBITED", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY", 0),   # the logged override event
                                              ("EAC_INHIBITED", "EAC_ERROR_HANDS_ON", 3),                 # the other drive
                                              ("EAC_INHIBITED", "EAC_ERROR_IDLE", 0),
                                              ("EAC_FAULT", "EAC_ERROR_HIGH_MMOT_SAFETY", 0)])
def test_epas_refusal_stops_commands_on_the_first_frame(platform, status, err, hands):
  car = engaged(platform, planned=6.0, angle=6.0)
  assert all(t == 1 for t, _a in [x for x in car.run(20) if x is not None])
  car.status, car.error, car.hands = status, err, hands
  car.angle = 3.0                                            # wheel moved since
  sent = [x for x in car.run(40) if x is not None]
  assert sent and all(t == 0 for t, _a in sent), "no ANGLE frame from the first frame on"
  assert all(abs(a - 3.0) < 0.2 for _t, a in sent), "measured wheel, never the old 6 deg target"


@pytest.mark.parametrize("platform", PLATFORMS)
def test_recovery_waits_for_a_clean_epas_then_goes_through_the_soft_start(platform):
  car = engaged(platform, planned=6.0, angle=6.0)
  car.status, car.error = "EAC_INHIBITED", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY"
  car.run(20)
  car.status, car.error = "EAC_AVAILABLE", "EAC_ERROR_IDLE"
  early = [x for x in car.run(EPAS_CLEAN_FRAMES - 6) if x is not None]
  assert all(t == 0 for t, _a in early), "EPAS clean for less than the wait: still NONE"
  late = [x for x in car.run(60) if x is not None]
  assert late[-1][0] == 1, "lateral is back"
  assert abs(late[-1][1] - 6.0) < 6.0
  # the first ANGLE frame after the block starts at the measured wheel, not at the old target
  first = next(a for t, a in late if t == 1)
  assert abs(first - 6.0) < 1.0


@pytest.mark.parametrize("platform", PLATFORMS)
def test_a_new_engagement_on_an_inhibited_epas_stays_silent(platform):
  car = Car(platform)
  car.status, car.error = "EAC_INHIBITED", "EAC_ERROR_IDLE"
  if platform == P:
    car.status = "EAC_ACTIVE"       # Pre-AP is mid-session only: it must have been ACTIVE first
    car.run(5)
    car.status = "EAC_INHIBITED"
  sent = [x for x in car.run(100) if x is not None]
  assert all(t == 0 for t, _a in sent[-30:])


@pytest.mark.parametrize("platform", PLATFORMS)
def test_the_published_signal_names_the_epas_when_it_is_the_reason(platform):
  car = engaged(platform)
  car.status, car.error = "EAC_INHIBITED", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY"
  car.run(5)
  assert car.ctl.ap1_epas_gate.signal == SIGNAL_EPAS
