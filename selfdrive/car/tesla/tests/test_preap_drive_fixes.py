"""Pedal feedback layout, pull latching and brake behaviour from route 00000018."""
from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.tesla.preap.engagement import DoublePullEngagement, preap_steer_drop

T = car.CarState.ButtonEvent.Type


def _pull(e, t, brake=False):
  ev = car.CarState.ButtonEvent.new_message(type=T.setCruise, pressed=True)
  out = e.filter_button_events([ev], enabled=True, now_ms=t, brake_pressed=brake, cancel=False)
  ev2 = car.CarState.ButtonEvent.new_message(type=T.setCruise, pressed=False)
  return out + e.filter_button_events([ev2], enabled=True, now_ms=t + 50, brake_pressed=brake, cancel=False)


def test_gas_sensor_byte4_is_state_then_counter():
  p = CANParser("tesla_can", [("GAS_SENSOR", 0)], 2)
  from cereal import messaging
  m = messaging.new_message("can", 1)
  m.can[0].address, m.can[0].src, m.can[0].dat = 0x552, 2, bytes([0, 0, 0, 0, 0x53, 0])
  p.update_strings([m.to_bytes()], sendcan=False)
  assert p.vl["GAS_SENSOR"]["STATE"] == 5 and p.vl["GAS_SENSOR"]["IDX"] == 3


def test_single_pull_latches_lateral_without_enable():
  e = DoublePullEngagement()
  assert _pull(e, 1000) == [] and e.lat_only


def test_double_pull_requests_long():
  e = DoublePullEngagement()
  _pull(e, 1000)
  assert any(b.type == T.setCruise for b in _pull(e, 1200)) and not e.lat_only and e.long_latched


def test_brake_after_double_pull_keeps_lateral():
  e = DoublePullEngagement()
  _pull(e, 1000); _pull(e, 1200)
  e.filter_button_events([], enabled=True, now_ms=5000, brake_pressed=True, cancel=False)
  assert e.lat_only and not e.long_latched


def test_brake_after_single_pull_keeps_lateral():
  e = DoublePullEngagement()
  _pull(e, 1000)
  e.filter_button_events([], enabled=True, now_ms=5000, brake_pressed=True, cancel=False)
  assert e.lat_only


def test_brake_without_a_pull_latches_nothing():
  e = DoublePullEngagement()
  e.filter_button_events([], enabled=True, now_ms=5000, brake_pressed=True, cancel=False)
  assert not e.lat_only


def test_cancel_clears_latch():
  e = DoublePullEngagement()
  _pull(e, 1000)
  e.filter_button_events([], enabled=True, now_ms=2000, brake_pressed=False, cancel=True)
  assert not e.lat_only


def test_steer_drop_matches_panda_conditions():
  assert preap_steer_drop(3, 1, 0) and preap_steer_drop(0, 0, 9)
  assert not preap_steer_drop(2, 1, 0) and not preap_steer_drop(0, 1, 9) and not preap_steer_drop(0, 0, 5)


def _chime(lat, long, prev):
  from openpilot.selfdrive.car.tesla.preap.hud import update_preap_chimes
  return update_preap_chimes(lat_engaged=lat, long_engaged=long, prev=prev)


def test_single_pull_then_cancel_gives_lat_edges_only():
  from openpilot.selfdrive.car.tesla.preap.hud import PreAPChimeState
  c, p = _chime(True, False, PreAPChimeState())
  assert c.lat_engage and not c.long_engage
  c, p = _chime(True, False, p)
  assert not any(c)
  c, p = _chime(False, False, p)
  assert c.lat_disengage and not c.long_disengage


def test_second_pull_and_brake_only_change_long():
  from openpilot.selfdrive.car.tesla.preap.hud import PreAPChimeState
  _, p = _chime(True, False, PreAPChimeState())
  c, p = _chime(True, True, p)
  assert c.long_engage and not c.lat_engage
  c, p = _chime(True, False, p)
  assert c.long_disengage and not c.lat_disengage


def test_lat_chime_events_are_defined_with_sounds():
  from openpilot.selfdrive.controls.lib.events import AudibleAlert, ET, FROGPILOT_EVENTS, FrogPilotEventName
  on = FROGPILOT_EVENTS[FrogPilotEventName.pedalLatEnabled][ET.PERMANENT]
  off = FROGPILOT_EVENTS[FrogPilotEventName.pedalLatDisabled][ET.PERMANENT]
  assert on.audible_alert == AudibleAlert.engage and off.audible_alert == AudibleAlert.disengage
