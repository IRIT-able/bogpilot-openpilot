"""AP1 car events, as event names (no cereal in opendbc).

0.9.7 added these in tesla/interface.py _update. In 0.11 car events are
built in openpilot selfdrive/car/car_events.py; for brand "tesla_ap1" it calls
Ap1Events.update and adds each returned name as log.OnroadEvent.EventName.

- Resume hold (bogpilot.ap1DriverYield): steerOverride, so the state stays
  overriding (grey border) until lateral resumes.
- Tinkla HSO: steerTempUnavailable becomes steerTempUnavailableSilent
  (steerUnavailable from EAC_FAULT is not rewritten).
- EPAS INHIBITED for ~1 s while lateral is wanted (and the driver is not
  steering): steerTempUnavailableSilent.
"""
from opendbc.car import DT_CTRL
from opendbc.car.tesla_ap1.hso import ap1_hso_event_names, ap1_steering_pressed

STEER_OVERRIDE = "steerOverride"
STEER_TEMP_SILENT = "steerTempUnavailableSilent"
# Pre-AP lateral-only engagement (double-pull first pull, re-pull demote): the silent long-override event, the same one a gas
# press raises, so controlsd drops carControl.longActive while cruiseState.enabled stays on. No new event name.
LONG_OVERRIDE = "gasPressedOverride"
# Pre-AP: pedal longitudinal is on and the Comma Pedal stream is missing or stale (carState.bogpilot.preapPedalNotResponding).
PEDAL_NOT_RESPONDING = "pedalNotResponding"
# Pre-AP: the pedal did not take authority (carState.bogpilot.preapPedalUnavailable) while openpilot is engaged: retry with back-off,
# the alert detail comes from carState.bogpilot.preapPedalFailCause. Does not disengage.
PEDAL_RETRYING = "pedalRetrying"
# AP1 openpilot long: standstill, no dashboard speed limit, no stalk tap set (carState.bogpilot.ap1SpeedLimitUnknown): NO_ENTRY.
SPEED_LIMIT_UNKNOWN = "ap1SpeedLimitUnknown"
WRONG_CAR_MODE = "wrongCarMode"
# Pre-AP pull FSM: steering-only banner, speed control blocked (pedal) / stock cruise not armed (no pedal).
PREAP_STEER_ONLY = "preapSteerOnly"
PREAP_SPEED_BLOCKED = "preapSpeedBlocked"
PREAP_CC_NOT_ARMED = "preapCruiseNotArmed"
# Pre-AP pedal: radar not ready (carState.bogpilot.preapRadarNotReady, card.py): its points are kept out of radard and speed control
# uses camera leads. The one radar warning; advisory, as NAP / Tinkla (no no-entry, soft disable or disengage).
PREAP_RADAR_NOT_READY = "preapRadarNotReady"
INHIBIT_ALERT_FRAMES = int(1.0 / DT_CTRL)  # ~1 s at 100 Hz
# How long the "Steering only / Double pull for speed control" banner stays up in one lateral-only stretch.
# It is a PERMANENT alert (events.py preapSteerOnly), so without this it sat on the screen for the whole drive
# when the driver chose to stay on steering only. Long enough to read once: it says what mode you are in and what
# the second pull does, and neither changes while you stay in this mode. After the timeout the banner is dropped,
# but nothing about the state is hidden: the lateral-only path colour stays (ui/onroad/model_renderer.py
# preap_steer_only reads carState.bogpilot.preapLatOnly, not this event), LONG_OVERRIDE below still keeps
# carControl.longActive off, and the actionable warnings (PREAP_SPEED_BLOCKED / PREAP_CC_NOT_ARMED) are never
# timed out. The timer restarts whenever lateral-only ends (a disengage, or FULL), so the banner is shown again
# on the next first pull.
PREAP_STEER_ONLY_SHOW_S = 3.0
PREAP_STEER_ONLY_FRAMES = int(PREAP_STEER_ONLY_SHOW_S / DT_CTRL)


class Ap1Events:
  def __init__(self):
    self.inhibit_alert_frames = 0
    self.steer_only_frames = 0        # frames the steering-only banner has been shown in this lateral-only stretch

  def update(self, names, enabled: bool, lat_active: bool, bogpilot, steer_fault_permanent: bool = False) -> list[str]:
    """names: event names openpilot built so far. Returns the AP1 rewritten list."""
    yielding = bool(bogpilot.ap1DriverYield)
    out = list(names)
    if bool(getattr(bogpilot, "preapLatOnly", False)) and LONG_OVERRIDE not in out:
      out.append(LONG_OVERRIDE)
    if yielding:
      out.append(STEER_OVERRIDE)
    out = ap1_hso_event_names(out)

    # The hands pause sends NONE on purpose (EPAS goes INHIBITED at level 3);
    # that is the driver steering, not EPAS refusing control.
    want_lat = bool(enabled) and bool(lat_active)
    want_lat = want_lat and not ap1_steering_pressed(bogpilot.ap1HandsOnLevel) and not yielding
    if want_lat and bool(bogpilot.ap1EacInhibited) and not steer_fault_permanent:
      self.inhibit_alert_frames += 1
    else:
      self.inhibit_alert_frames = 0
    if self.inhibit_alert_frames >= INHIBIT_ALERT_FRAMES and STEER_TEMP_SILENT not in out:
      out.append(STEER_TEMP_SILENT)

    # Pre-AP: say why when the Comma Pedal goes quiet. It replaces "Cruise Mode Disabled" (cruiseState.available drops with the pedal)
    # because this is the specific cause; while engaged the event disengages immediately (the panda stops allowing controls about a
    # second after the stream stops anyway).
    if bool(getattr(bogpilot, "preapPedalNotResponding", False)):
      out = [name for name in out if name != WRONG_CAR_MODE]
      if PEDAL_NOT_RESPONDING not in out:
        out.append(PEDAL_NOT_RESPONDING)
    elif bool(enabled) and bool(getattr(bogpilot, "preapPedalUnavailable", False)) and PEDAL_RETRYING not in out:
      out.append(PEDAL_RETRYING)
    # AP1 openpilot long: no set speed for a standstill engage (op_long_set_speed.py). NO_ENTRY only, so it never disengages.
    if bool(getattr(bogpilot, "ap1SpeedLimitUnknown", False)) and SPEED_LIMIT_UNKNOWN not in out:
      out.append(SPEED_LIMIT_UNKNOWN)

    if bool(enabled) and bool(getattr(bogpilot, "preapRadarNotReady", False)) and PREAP_RADAR_NOT_READY not in out:
      out.append(PREAP_RADAR_NOT_READY)

    if bool(enabled) and bool(getattr(bogpilot, "preapLatOnly", False)):
      if bool(getattr(bogpilot, "preapSpeedBlocked", False)):
        # Actionable: speed control was refused or dropped. Never timed out, and it does not advance the
        # banner timer (it replaces the banner while it shows).
        out.append(PREAP_SPEED_BLOCKED if bool(getattr(bogpilot, "preapPedalDetected", False)) else PREAP_CC_NOT_ARMED)
      elif self.steer_only_frames < PREAP_STEER_ONLY_FRAMES:
        self.steer_only_frames += 1
        out.append(PREAP_STEER_ONLY)
    else:
      self.steer_only_frames = 0      # lateral-only ended: the banner is shown again on the next first pull
    return out
