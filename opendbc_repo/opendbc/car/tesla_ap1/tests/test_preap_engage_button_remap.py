"""Pre-AP pedal long: the stalk pull leaves carstate as setCruise, so selfdrived never raises resumeBlocked.

selfdrived.py:267-268 raises resumeBlocked ("Press Set to Engage") when `not CP.pcmCruise and CS.vCruise > 250` and a
buttonEvent of type accelCruise or resumeCruise is seen. On Pre-AP with the pedal, pcmCruise is False and vCruise is
V_CRUISE_UNSET (255) until the first engage, so every drive's first pull showed that no-entry alert. The pull leaves
carstate as setCruise now (preap/cruise.py remap_preap_pedal_button_type), which that rule does not match.

Pre-AP reads the pull off button_states and the edges computed before the remap, so nothing on the car side changes.
"""
from opendbc.car import structs
from opendbc.car.tesla_ap1.preap.engagement import Mode
from opendbc.car.tesla_ap1.preap.cruise import remap_preap_pedal_button_type
from opendbc.car.tesla_ap1.tests.test_preap_carstate_controller import (CANCEL, PULL, _params_dir, _ready_rig,  # noqa: F401
                                                                        params)

ButtonType = structs.CarState.ButtonEvent.Type
# selfdrived.py:267: the two types that raise resumeBlocked.
RESUME_BLOCKED_TYPES = (ButtonType.accelCruise, ButtonType.resumeCruise)
V_CRUISE_UNSET = 255          # selfdrive/car/cruise.py:13


def _pull_events(rig):
  """buttonEvents published for one stalk pull edge."""
  rig.frame(stalk=0)
  return list(rig.frame(stalk=PULL).buttonEvents)


def test_the_pull_is_published_as_setcruise():
  rig, _ = _ready_rig(double=False)
  types = [e.type for e in _pull_events(rig) if e.pressed]
  assert ButtonType.setCruise in types, types
  assert ButtonType.resumeCruise not in types, types


def test_the_pull_no_longer_matches_the_resume_blocked_rule():
  """The whole point: with vCruise unset, selfdrived's rule must not fire on a pull."""
  rig, _ = _ready_rig(double=False)
  events = _pull_events(rig)
  resume_pressed = any(e.type in RESUME_BLOCKED_TYPES for e in events)
  assert not (V_CRUISE_UNSET > 250 and resume_pressed), [e.type for e in events]


def test_the_pull_still_engages_and_still_only_steers_on_one_pull():
  """The remap is display/selfdrived only: the FSM still engages on the pull and stays steering-only."""
  rig, _ = _ready_rig(double=False)
  rig.frame(stalk=0)
  out = rig.frame(stalk=PULL)
  assert rig.cs.update_button_enable(out.buttonEvents), "the pull must still request enable"
  assert out.cruiseState.enabled, "the pull must still engage openpilot"
  assert out.bogpilot.preapLatOnly, "one pull is steering only"
  # On the pull frame itself the FSM is LAT_PENDING: the 400 ms second-pull window is still open.
  assert Mode(out.bogpilot.preapFsmMode) in (Mode.LAT_PENDING, Mode.LAT_ONLY), Mode(out.bogpilot.preapFsmMode)


def test_resume_pressed_level_is_unchanged():
  """fp_ret.resumePressed reads button_states, not the published types, so the stalk level still reports a pull."""
  rig, _ = _ready_rig(double=False)
  rig.frame(stalk=0)
  out = rig.frame(stalk=PULL)
  assert out.bogpilot.resumePressed


def test_cancel_and_the_tips_are_untouched():
  rig, _ = _ready_rig(double=False)
  rig.frame(stalk=0)
  cancel = [e.type for e in rig.frame(stalk=CANCEL).buttonEvents if e.pressed]
  assert ButtonType.cancel in cancel, cancel
  assert ButtonType.setCruise not in cancel, cancel


def test_remap_helper_maps_only_resume():
  assert remap_preap_pedal_button_type(ButtonType.resumeCruise) == ButtonType.setCruise
  for other in (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.cancel,
                ButtonType.setCruise, ButtonType.unknown, ButtonType.leftBlinker):
    assert remap_preap_pedal_button_type(other) == other, other
