import time
#!/usr/bin/env python3
from cereal import car, custom
from openpilot.selfdrive.controls.lib.drive_helpers import CRUISE_LONG_PRESS
from openpilot.selfdrive.controls.lib.events import ET

from openpilot.frogpilot.common.frogpilot_variables import ERROR_LOGS_PATH, GearShifter, NON_DRIVING_GEARS, params, params_memory
from openpilot.selfdrive.car.tesla.stalk_follow import ap1_stalk_commands, crossed_personality
from openpilot.selfdrive.car.tesla.values import CAR

ButtonType = car.CarState.ButtonEvent.Type

AP1_PERSONALITY_RETRY_FRAMES = 10  # card runs at 100 Hz: re-check the stalk personality write every 0.1 s
AP1_PERSONALITY_MAX_FRAMES = 300   # stop retrying after 3 s
FrogPilotButtonType = custom.FrogPilotCarState.ButtonEvent.Type

def handle_experimental_mode(conditional_experimental_mode):
  if conditional_experimental_mode:
    conditional_status = params_memory.get_int("CEStatus")

    override_value = 0 if conditional_status in {1, 2} else 1 if conditional_status >= 3 else 2

    params_memory.put_int("CEStatus", override_value)
  else:
    params.put_bool_nonblocking("ExperimentalMode", not params.get_bool("ExperimentalMode"))

class FrogPilotCard:
  def __init__(self, Car):
    self.car = Car

    self.long_press_threshold = CRUISE_LONG_PRESS * (1.5 if self.car.CP.carName == "gm" else 1)
    self.very_long_press_threshold = CRUISE_LONG_PRESS * 5

    self.accel_pressed = False
    self.decel_pressed = False
    self.resume_pressed = False
    self.spd_ctrl_lvr = 0
    self.force_coast = False
    self.pause_lateral = False
    self.pause_longitudinal = False
    self.prev_distance_button = False
    self.traffic_mode_enabled = False
    self._ap1_stalk_personality = None
    self._ap1_stalk_detent = None
    self._ap1_personality_target = None
    self._ap1_personality_frames = 0

    self.gap_counter = 0
    self.onroad_press_start = None
    self.onroad_traffic_fired = False

    self.error_log = ERROR_LOGS_PATH / "error.txt"

  def update_distance_button(self, sm):
    if sm["carControl"].longActive and self.car.frogpilot_toggles.experimental_mode_via_distance:
      handle_experimental_mode(self.car.frogpilot_toggles.conditional_experimental_mode)
    elif self.car.frogpilot_toggles.force_coast_via_distance:
      self.force_coast = not self.force_coast
    elif self.car.frogpilot_toggles.pause_lateral_via_distance:
      self.pause_lateral = not self.pause_lateral
    elif self.car.frogpilot_toggles.pause_longitudinal_via_distance:
      self.pause_longitudinal = not self.pause_longitudinal
    elif sm["carControl"].longActive and self.car.frogpilot_toggles.traffic_mode_via_distance:
      self.traffic_mode_enabled = not self.traffic_mode_enabled

  def update_distance_button_long(self, sm):
    if sm["carControl"].longActive and self.car.frogpilot_toggles.experimental_mode_via_distance_long:
      handle_experimental_mode(self.car.frogpilot_toggles.conditional_experimental_mode)
    elif self.car.frogpilot_toggles.force_coast_via_distance_long:
      self.force_coast = not self.force_coast
    elif self.car.frogpilot_toggles.pause_lateral_via_distance_long:
      self.pause_lateral = not self.pause_lateral
    elif self.car.frogpilot_toggles.pause_longitudinal_via_distance_long:
      self.pause_longitudinal = not self.pause_longitudinal
    elif sm["carControl"].longActive and self.car.frogpilot_toggles.traffic_mode_via_distance_long:
      self.traffic_mode_enabled = not self.traffic_mode_enabled

  def update_distance_button_very_long(self, sm):
    self.update_distance_button_long(sm)

    if sm["carControl"].longActive and self.car.frogpilot_toggles.experimental_mode_via_distance_very_long:
      handle_experimental_mode(self.car.frogpilot_toggles.conditional_experimental_mode)
    elif self.car.frogpilot_toggles.force_coast_via_distance_very_long:
      self.force_coast = not self.force_coast
    elif self.car.frogpilot_toggles.pause_lateral_via_distance_very_long:
      self.pause_lateral = not self.pause_lateral
    elif self.car.frogpilot_toggles.pause_longitudinal_via_distance_very_long:
      self.pause_longitudinal = not self.pause_longitudinal
    elif sm["carControl"].longActive and self.car.frogpilot_toggles.traffic_mode_via_distance_very_long:
      self.traffic_mode_enabled = not self.traffic_mode_enabled

  def update_lkas_button(self, sm):
    if sm["carControl"].longActive and self.car.frogpilot_toggles.experimental_mode_via_lkas:
      handle_experimental_mode(self.car.frogpilot_toggles.conditional_experimental_mode)
    elif self.car.frogpilot_toggles.force_coast_via_lkas:
      self.force_coast = not self.force_coast
    elif self.car.frogpilot_toggles.pause_lateral_via_lkas:
      self.pause_lateral = not self.pause_lateral
    elif self.car.frogpilot_toggles.pause_longitudinal_via_lkas:
      self.pause_longitudinal = not self.pause_longitudinal
    elif sm["carControl"].longActive and self.car.frogpilot_toggles.traffic_mode_via_lkas:
      self.traffic_mode_enabled = not self.traffic_mode_enabled

  def _apply_ap1_stalk_levels(self):
    """AP1: publish continuous UP/DN/RWD stalk levels onto frogpilotCarState.

    Tesla carstate.button_states tracks SpdCtrlLvr_Stat while held; buttonEvents
    only fire on transitions (~10 ms). Planner samples frogpilotCarState at
    DT_MDL, so levels are required for sticky engage latch, tip hold timing,
    and engaged RWD pull (re-latch SLC+offset). accelPressed is UP-only (not
    OR resume) so tip-up does not fire on RWD; resumePressed carries pull.
    """
    if self.car.CP.carFingerprint != CAR.TESLA_AP1_MODELS:
      return
    cs = getattr(getattr(self.car, "CI", None), "CS", None)
    bs = getattr(cs, "button_states", None) if cs is not None else None
    if not isinstance(bs, dict):
      return
    self.accel_pressed = bool(bs.get(ButtonType.accelCruise, False))
    self.decel_pressed = bool(bs.get(ButtonType.decelCruise, False))
    self.resume_pressed = bool(bs.get(ButtonType.resumeCruise, False))
    self.spd_ctrl_lvr = int(getattr(cs, "spd_ctrl_lvr", 0) or 0)

  def _apply_ap1_stalk(self):
    """AP1 distance stalk drives the existing personality icon and traffic mode.

    Follow seconds are published separately on carState.cruiseState.speedOffset.
    Named detents write LongitudinalPersonality, which controlsd already reads.
    No new param key.
    """
    if self.car.CP.carFingerprint != CAR.TESLA_AP1_MODELS:
      return
    cs = getattr(getattr(self.car, "CI", None), "CS", None)
    decision = getattr(cs, "stalk_follow", None)
    traffic, personality = ap1_stalk_commands(decision)
    if traffic is None:
      return
    self.traffic_mode_enabled = traffic
    # A move onto a named detent sets the target, even if it matches the cached personality: the cache goes
    # stale after a screen tap or Settings change, and a quick flick (e.g. via an in-between detent) would be skipped.
    detent = getattr(decision, "detent", None)
    if personality is None:
      # Landing on 1, 2, 4 or 6 after a quick twist that skipped a named detent on CAN (e.g. 7 -> 2 reported as
      # 200, 166, 100, 33): apply the last named detent passed over, same as a slow twist through it.
      personality = crossed_personality(self._ap1_stalk_detent, detent)
    if personality is not None and (personality != self._ap1_stalk_personality or detent != self._ap1_stalk_detent):
      self._ap1_personality_target = personality
      self._ap1_personality_frames = 0
    self._ap1_stalk_detent = detent
    self._sync_ap1_personality()

  def _sync_ap1_personality(self):
    """Write the stalk's LongitudinalPersonality and re-check the stored value until it matches.

    A put_nonblocking can be lost (Params async writer race) or overtaken, so the param is read back every
    AP1_PERSONALITY_RETRY_FRAMES and rewritten while it differs. Gives up after AP1_PERSONALITY_MAX_FRAMES so a
    later screen tap is never fought. Only the personality param is touched; follow time comes from the stalk.
    """
    target = self._ap1_personality_target
    if target is None:
      return
    frames = self._ap1_personality_frames
    self._ap1_personality_frames += 1
    if frames == 0:
      params.put_nonblocking("LongitudinalPersonality", str(target))
      self._ap1_stalk_personality = target
      return
    if frames % AP1_PERSONALITY_RETRY_FRAMES:
      return
    try:
      stored = int(params.get("LongitudinalPersonality"))
    except (TypeError, ValueError):
      stored = None
    if stored == target or frames >= AP1_PERSONALITY_MAX_FRAMES:
      self._ap1_personality_target = None
      return
    params.put_nonblocking("LongitudinalPersonality", str(target))

  def _apply_ap1_pull_hold(self):
    """AP1 long RWD stalk pull (~2 s) toggles Experimental Mode once.

    Uses CarState.stalk_pull_toggle from stalk_pull_hold. Existing
    handle_experimental_mode hook; no new Params keys. Short pulls and the
    engage pull itself do not toggle.
    """
    if self.car.CP.carFingerprint != CAR.TESLA_AP1_MODELS:
      return
    cs = getattr(getattr(self.car, "CI", None), "CS", None)
    if not getattr(cs, "stalk_pull_toggle", False):
      return
    handle_experimental_mode(self.car.frogpilot_toggles.conditional_experimental_mode)

  def _apply_ap1_fwd_hold(self):
    """AP1 long FWD stalk (~2 s) while disengaged toggles Experimental Mode once.

    Uses CarState.stalk_fwd_toggle from stalk_fwd_hold. Same
    handle_experimental_mode hook as RWD pull — no longActive gate (distance /
    LKAS experimental paths require longActive; this path must work while
    disengaged). Cancel-while-engaged and short FWD do not toggle.
    """
    if self.car.CP.carFingerprint != CAR.TESLA_AP1_MODELS:
      return
    cs = getattr(getattr(self.car, "CI", None), "CS", None)
    if not getattr(cs, "stalk_fwd_toggle", False):
      return
    handle_experimental_mode(self.car.frogpilot_toggles.conditional_experimental_mode)

  def update(self, carState, frogpilotCarState, sm):
    # Upstream FrogPilot-Staging d83b149b: &= (was |=). With |=, AOL turned on whenever cruise was
    # engaged even with the Always On Lateral toggle off, and ignored the LKAS/main condition when on.
    self.always_on_lateral_enabled = self.car.frogpilot_toggles.always_on_lateral_set

    if self.car.frogpilot_toggles.use_lkas_for_aol:
      self.always_on_lateral_enabled &= self.car.frogpilot_toggles.always_on_lateral_lkas or carState.cruiseState.enabled
      self.always_on_lateral_enabled &= frogpilotCarState.alwaysOnLateralAllowed
    else:
      self.always_on_lateral_enabled &= self.car.frogpilot_toggles.always_on_lateral_main or carState.cruiseState.enabled or frogpilotCarState.pedalCruiseLatOnly
      self.always_on_lateral_enabled &= carState.cruiseState.available

    self.always_on_lateral_enabled &= carState.gearShifter not in NON_DRIVING_GEARS
    self.always_on_lateral_enabled &= sm["frogpilotPlan"].lateralCheck
    self.always_on_lateral_enabled &= sm["liveCalibration"].calPerc >= 1
    self.always_on_lateral_enabled &= (ET.IMMEDIATE_DISABLE not in sm["controlsState"].alertType + sm["frogpilotControlsState"].alertType) or self.car.frogpilot_toggles.frogs_go_moo
    self.always_on_lateral_enabled &= not (carState.brakePressed and carState.vEgo < self.car.frogpilot_toggles.always_on_lateral_pause_speed) or carState.standstill
    self.always_on_lateral_enabled &= not self.error_log.is_file() or self.car.frogpilot_toggles.frogs_go_moo

    if sm.updated["frogpilotPlan"] or any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in carState.buttonEvents):
      self.accel_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in carState.buttonEvents)

    if sm.updated["frogpilotPlan"] or any(be.type == ButtonType.decelCruise for be in carState.buttonEvents):
      self.decel_pressed = any(be.type == ButtonType.decelCruise for be in carState.buttonEvents)

    # AP1: continuous UP/DN/RWD levels from carstate.button_states (buttonEvents
    # are ~10 ms edges; frogpilot_vcruise at 20 Hz misses short holds).
    self._apply_ap1_stalk_levels()

    self.force_coast &= not (carState.brakePressed or carState.gasPressed)

    # BogPilot: the on-screen personality button no longer feeds the distance-button counter (that would run the
    # user's Long Distance Button Control, e.g. Experimental Mode). A 2.5 s hold only toggles Traffic Mode; taps
    # are handled in controlsd. Another 2.5 s hold exits Traffic Mode.
    onroad_pressed = params_memory.get_bool("OnroadDistanceButtonPressed")
    if onroad_pressed:
      if self.onroad_press_start is None:
        self.onroad_press_start = time.monotonic()
      if not self.onroad_traffic_fired and time.monotonic() - self.onroad_press_start >= 2.5:
        self.onroad_traffic_fired = True
        if sm["carControl"].longActive or self.traffic_mode_enabled:
          self.traffic_mode_enabled = not self.traffic_mode_enabled
    else:
      self.onroad_press_start = None
      self.onroad_traffic_fired = False

    if frogpilotCarState.distancePressed:
      self.gap_counter += 1
    elif not self.prev_distance_button:
      self.gap_counter = 0

    if not frogpilotCarState.distancePressed and 1 <= self.gap_counter < self.long_press_threshold:
      self.update_distance_button(sm)
    elif self.gap_counter == self.long_press_threshold:
      self.update_distance_button_long(sm)
    elif self.gap_counter == self.very_long_press_threshold:
      self.update_distance_button_very_long(sm)

    lkas_button = any(be.pressed and be.type == FrogPilotButtonType.lkas for be in carState.buttonEvents)

    if lkas_button:
      self.update_lkas_button(sm)

    self.prev_distance_button = frogpilotCarState.distancePressed

    frogpilotCarState.accelPressed = self.accel_pressed
    frogpilotCarState.alwaysOnLateralEnabled = self.always_on_lateral_enabled
    frogpilotCarState.decelPressed = self.decel_pressed
    frogpilotCarState.resumePressed = self.resume_pressed
    frogpilotCarState.spdCtrlLvr = int(getattr(self, 'spd_ctrl_lvr', 0) or 0)
    frogpilotCarState.distanceLongPressed = self.very_long_press_threshold > self.gap_counter >= self.long_press_threshold
    frogpilotCarState.distanceVeryLongPressed = self.gap_counter >= self.very_long_press_threshold
    frogpilotCarState.forceCoast = self.force_coast
    frogpilotCarState.isParked = carState.gearShifter == GearShifter.park
    frogpilotCarState.pauseLateral = self.pause_lateral
    frogpilotCarState.pauseLongitudinal = self.pause_longitudinal
    self._apply_ap1_stalk()
    self._apply_ap1_pull_hold()
    self._apply_ap1_fwd_hold()
    frogpilotCarState.trafficModeEnabled = self.traffic_mode_enabled

    return frogpilotCarState
