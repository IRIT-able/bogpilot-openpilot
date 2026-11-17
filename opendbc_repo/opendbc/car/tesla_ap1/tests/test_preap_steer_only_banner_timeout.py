"""The "Steering only / Double pull for speed control" banner stops after PREAP_STEER_ONLY_SHOW_S.

preapSteerOnly is an ET.PERMANENT alert (selfdrived/events.py:1009-1011), so while the driver stayed on steering only it
sat on the screen for the rest of the drive. It is informational and the same every time: the state itself stays visible without it, because the
lateral-only path colour reads carState.bogpilot.preapLatOnly (ui/onroad/model_renderer.py preap_steer_only), not this
event, and LONG_OVERRIDE still keeps carControl.longActive off. The actionable warnings are never timed out.
"""
from types import SimpleNamespace

from opendbc.car.tesla_ap1.ap1_events import (LONG_OVERRIDE, PREAP_CC_NOT_ARMED, PREAP_SPEED_BLOCKED,
                                              PREAP_STEER_ONLY, PREAP_STEER_ONLY_FRAMES,
                                              PREAP_STEER_ONLY_SHOW_S, Ap1Events)


def _bog(**kw):
  base = dict(ap1DriverYield=False, preapLatOnly=False, preapSpeedBlocked=False, preapPedalDetected=True,
              preapPedalNotResponding=False, preapPedalUnavailable=False, preapRadarNotReady=False,
              ap1SpeedLimitUnknown=False, ap1HandsOnLevel=0, ap1EacInhibited=False)
  base.update(kw)
  return SimpleNamespace(**base)


def _run(ev, frames, **kw):
  """Run `frames` steps and return the list of outputs."""
  return [ev.update([], enabled=True, lat_active=True, bogpilot=_bog(**kw)) for _ in range(frames)]


def test_the_banner_shows_at_first():
  ev = Ap1Events()
  assert PREAP_STEER_ONLY in _run(ev, 1, preapLatOnly=True)[0]


def test_the_banner_stops_after_the_timeout_and_nothing_else_does():
  ev = Ap1Events()
  outs = _run(ev, PREAP_STEER_ONLY_FRAMES + 50, preapLatOnly=True)
  assert PREAP_STEER_ONLY in outs[PREAP_STEER_ONLY_FRAMES - 1], "still up on the last frame of the window"
  assert PREAP_STEER_ONLY not in outs[PREAP_STEER_ONLY_FRAMES], "dropped on the frame after it"
  assert PREAP_STEER_ONLY not in outs[-1]
  # The functional part of lateral-only is untouched: longActive stays off for the whole stretch.
  assert all(LONG_OVERRIDE in o for o in outs)


def test_the_timer_restarts_when_lateral_only_ends():
  ev = Ap1Events()
  _run(ev, PREAP_STEER_ONLY_FRAMES + 10, preapLatOnly=True)
  assert PREAP_STEER_ONLY not in _run(ev, 1, preapLatOnly=True)[0]
  _run(ev, 1, preapLatOnly=False)                                  # FULL, or a disengage
  assert PREAP_STEER_ONLY in _run(ev, 1, preapLatOnly=True)[0], "shown again on the next first pull"


def test_a_disengage_restarts_the_timer_too():
  ev = Ap1Events()
  _run(ev, PREAP_STEER_ONLY_FRAMES + 10, preapLatOnly=True)
  ev.update([], enabled=False, lat_active=False, bogpilot=_bog(preapLatOnly=True))
  assert PREAP_STEER_ONLY in _run(ev, 1, preapLatOnly=True)[0]


def test_the_speed_blocked_warning_is_never_timed_out():
  ev = Ap1Events()
  outs = _run(ev, PREAP_STEER_ONLY_FRAMES + 500, preapLatOnly=True, preapSpeedBlocked=True)
  assert all(PREAP_SPEED_BLOCKED in o for o in outs)
  assert all(PREAP_STEER_ONLY not in o for o in outs), "the blocked warning replaces the banner"


def test_the_cruise_not_armed_warning_is_never_timed_out():
  ev = Ap1Events()
  outs = _run(ev, PREAP_STEER_ONLY_FRAMES + 500, preapLatOnly=True, preapSpeedBlocked=True,
              preapPedalDetected=False)
  assert all(PREAP_CC_NOT_ARMED in o for o in outs)


def test_the_blocked_warning_does_not_spend_the_banner_window():
  """A stretch that shows the blocked warning and then clears must still have banner time left."""
  ev = Ap1Events()
  _run(ev, PREAP_STEER_ONLY_FRAMES + 500, preapLatOnly=True, preapSpeedBlocked=True)
  assert PREAP_STEER_ONLY in _run(ev, 1, preapLatOnly=True)[0]


def test_the_window_is_long_enough_to_read_once_and_no_longer():
  assert PREAP_STEER_ONLY_SHOW_S == 3.0
  assert PREAP_STEER_ONLY_FRAMES == 300, PREAP_STEER_ONLY_FRAMES
