"""Pre-AP Model S CarState.

Pre-AP has no Autopilot ECU: bus 2 carries no Mobileye frames (only the optional Comma Pedal), there is no
DAS_control to read, and the EPAS idles in INHIBITED / TMP_FAULT until we send EPB_epasControl. Differences from
the AP1 CarState (cited in docs/tesla/PREAP_PORT.md, issue list in the port notes):

- cruiseState: with the Comma Pedal doing longitudinal (CP.openpilotLongitudinalControl, pcmCruise False) openpilot
  owns "enabled" through PreAPEngagement (NAP-style double pull: first pull steering only, second pull speed control), and `available` means pedal detected + calibrated + healthy.
  Without the pedal, stock TACC drives (pcmCruise True) exactly like AP1.
- standstill is the DI STANDSTILL state, not vEgo < 0.1 (the pedal holds 0 mph; a vEgo standstill plus a brake press
  raised preEnableStandstill and blocked engage).
- steerFaultTemporary is always False (EAC_INHIBITED + TMP_FAULT is the normal idle without an AP ECU).
- brake = BrakeMessage OR DI_brakePedal. gasPressed comes from the interceptor when the pedal is the gas source.
- stockAeb is False, acc_state 0, no cluster/BSM/steer-counter (no stock DAS to follow).
- preapPedalNotResponding: pedal longitudinal on and the Comma Pedal counter has not moved for 0.5 s (missing or stale stream).
"""
from __future__ import annotations

import copy
import time

from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla_ap1.hso import ap1_driver_input
from opendbc.car.tesla_ap1.pedal_signals import pedal_position
from opendbc.car.tesla_ap1.speed_limit import dashboard_speed_limit_ms, frame_fresh
from opendbc.car.tesla_ap1.values import BUTTONS, CANBUS, DBC, DOORS, GEAR_MAP
from opendbc.car.tesla_ap1.preap.calib import is_pedal_calibrated, pedal_can_bus
# Panda gas threshold in GAS_SENSOR.INTERCEPTOR_GAS units: raw 650 (tesla_preap.h:135) with the DBC scale (tesla_ap1_can.dbc:938).
PANDA_GAS_PRESSED_DBC = 650 * 0.0507968128 - 22.85856576
# tesla_preap.h:254 blocks ENABLE=1 from the first INTERCEPTOR_GAS > 650 (gas_pressed, :135) and while the brake is pressed
# (get_longitudinal_allowed / brake_pressed). Ignore a TX-block delta in this window after a gas or brake press edge:
# that block is the expected panda refusal, not a reason to drop speed control. Blocks outside the window still demote.
TX_BLOCK_PRESS_IGNORE_MS = 300
from opendbc.car.tesla_ap1.preap.cruise import (SPD_CTRL_CANCEL, SPD_CTRL_MAIN, is_preap_cancel_echo,
                                                pedal_cruise_available, remap_preap_pedal_button_type)
from opendbc.car.tesla_ap1.preap import ibooster as IBST
from opendbc.car.tesla_ap1.preap.engagement import Decision, PreAPEngagement, preap_epas_refusing, preap_steer_drop
from opendbc.car.tesla_ap1.preap.params import PreApParamKeys, get_bool, set_bool, set_float  # noqa: F401
from opendbc.car.tesla_ap1.preap.pedal_feedback import PedalFeedback

OPTIONAL = float('nan')   # 0.9.7 frequency 0 (never marks the message missing)
ButtonType = structs.CarState.ButtonEvent.Type


class PreApCarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.pedal_long = bool(CP.openpilotLongitudinalControl)
    self.pedal_bus = pedal_can_bus()
    # iBooster swap toggle, read once when the car starts (get_can_parsers reads the same key for the 0x554 subscription).
    self.has_ibooster = bool(get_bool(PreApParamKeys.HAS_IBOOSTER))
    # tesla_preap.h:20, :276: tesla_preap_pedal_bus starts at -1 and is latched by the first 0x552 (:128-135).
    self._preap_pedal_bus_latched = False
    self.preap_pedal_bus_mismatch = False
    self.preap_pedal_bus_ok = False

    # Read by CarController / long controller (names kept from the Pre-AP lane).
    self.msg_stw_actn_req = None
    self.hands_on_level = 0
    self.steer_warning = None
    self.eac_status = None
    self.eac_fault = False
    self.acc_state = 0
    self.spd_ctrl_lvr = 0
    self.cruise_buttons = 0
    self.prev_cruise_buttons = 0
    self.preap_cc_cancel_needed = False
    self.preap_cc_cancel_drop = False   # no pedal: forget a CANCEL not yet sent (StockCCSpoofer.drop_pending)
    self.preap_cc_cancel_tx_ms = 0      # no pedal: monotonic ms of the last CANCEL put on the bus
    self.preap_last_cc_spoof_ms = 0
    self.preap_pedal_detected = False
    self.preap_pedal_raw = None
    self._preap_pedal_last_idx = None
    self._pedal_seen_write = float('-inf')   # time.monotonic() of the last heartbeat write (1 s throttle)
    self.pedal = PedalFeedback()
    self.pedal_interceptor_value = 0.0
    self.di_gas = 0.0
    self.pedal_timeout = True
    self.real_brake_pressed = False
    self.pedal_unavailable = False
    self.pedal_fail_cause = 0
    self.pedal_authority_requested = False
    self.pedal_authority_active = False
    self.pedal_command_counter = 0
    self.pedal_first_enabled_mono_time = 0
    self.vdas_limited_accel = 0.0
    self.pedal_brake_required = False
    # Engagement (stalk pull) and the panda-drop latch.
    self.engagement = PreAPEngagement()
    self.decision = Decision()
    self.preap_fsm_mode = int(self.decision.mode)   # read by the pedal guard (long_controller._append_pedal_command)
    self.preap_pedal_guard_blocks = 0               # pedal ENABLE frames the userspace guard turned into enable=0
    self._preap_guard_blocks_seen = 0
    self.preap_panda_tx_blocked = 0                 # refused own 0x551 / 0x488 frames this step, fed by card.py (preap/panda_link.py)
    self.preap_link = None                          # panda_link.LinkInput, fed by card.py each step (read by the 0x560 stream)
    self._tx_block_press_ms = 0                     # last gas or brake press edge; TX blocks inside the window are ignored
    self._prev_tx_block_gas = False
    self._prev_tx_block_brake = False
    self.preap_panda_gas_pressed = False            # the panda's own gas test on GAS_SENSOR (see below)
    self.preap_gas_clear_updates = 0                # updates since both gas tests read released
    self.preap_standstill_hold = False              # Q1 (b) hold, read by the pedal guard
    self.preap_capture_edge = False                 # one update: the card captures the FULL set speed (selfdrive/car/card.py)
    self.op_enabled = False          # CarController feeds CC.enabled here every frame
    self.preap_panda_drop = False
    self.preap_epas_refusing = False
    self.preap_fresh_pull_pending = False   # mirror of the panda's tesla_preap_need_fresh_pull
    self.preap_steer_armed = False          # mirror of tesla_preap_steer_armed
    self.preap_steer_brake_seen = False     # mirror of tesla_preap_steer_brake_seen
    self.preap_last_pull_ms = -1_000_000
    self._enable_request = False
    self.stock_ap_active = False
    # Fresh posted limit for the SetSpeedLimit engage set (m/s, 0 = unknown). Read by the card on the enable edge.
    self.engage_limit_ms = 0.0

  # openpilot raises buttonEnable only from the pull decision (up/down tips must not engage: the panda arms on a pull).
  def update_button_enable(self, buttonEvents):
    return bool(self.pedal_long and self._enable_request)

  def _mirror_pedal_status(self) -> None:
    """Display only: the Vehicle Controls "Comma Pedal" row reads these files (calib.pedal_live). They are written here,
    from the CAN stream, so the row is right even when the CarController never runs (openpilot toggle off, passive or
    dashcam, controlsd not up). Calibration stays in the controller. Nothing here feeds control, the panda or any TX."""
    detected = bool(self.preap_pedal_detected)
    try:
      set_bool(PreApParamKeys.PEDAL_DETECTED, detected)
      if detected:
        # Heartbeat for the UI: PedalDetected is a latched file and stays True after the car stops. The throttle runs on
        # time.monotonic(), so a wall-clock jump (GPS/NTP time set) cannot stop the writes.
        mono = time.monotonic()
        if mono - self._pedal_seen_write >= 1.0:
          set_float(PreApParamKeys.PEDAL_SEEN_TS, time.time())
          set_float(PreApParamKeys.PEDAL_SEEN_MONO, mono)
          self._pedal_seen_write = mono
    except OSError:
      pass   # fresh device / read-only /data must not stop card

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.chassis]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    fp_ret = ret.bogpilot
    now_ms = int(time.monotonic() * 1000)

    # Speed
    ret.vEgoRaw = cp.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    # Comma Pedal presence. Latch the interceptor bus on the first GAS_SENSOR (0x552), mirroring
    # tesla_preap.h:128-135 (tesla_preap_pedal_bus; init/reset :20, :276). NAP @ e99f4dc75 tesla_preap.h:349-355
    # and BogPilot na-preap @ 568fea947 safety_tesla_preap.h:302-307 also record the first bus, but still drive
    # gas_pressed from every 0x552; the panda here (and this userspace) only trusts the latched bus. A later
    # 0x552 on the other bus is a mismatch: no FULL, never ENABLE=1. Stale IDX timeout is unchanged.
    chassis_idx = cp.vl_all.get("GAS_SENSOR", {}).get("IDX", [])
    cam_idx = cp_cam.vl_all.get("GAS_SENSOR", {}).get("IDX", [])
    seen = []
    if len(chassis_idx):
      seen.append(CANBUS.chassis)
    if len(cam_idx):
      seen.append(CANBUS.autopilot_chassis)
    if not self._preap_pedal_bus_latched and seen:
      self.pedal_bus = seen[0]
      self._preap_pedal_bus_latched = True
    if self._preap_pedal_bus_latched and any(b != self.pedal_bus for b in seen):
      self.preap_pedal_bus_mismatch = True
    self.preap_pedal_bus_ok = bool(self._preap_pedal_bus_latched) and not self.preap_pedal_bus_mismatch

    pedal_cp = cp if self.pedal_bus == CANBUS.chassis else cp_cam
    idx_all = pedal_cp.vl_all.get("GAS_SENSOR", {}).get("IDX", [])
    gas_all = pedal_cp.vl_all.get("GAS_SENSOR", {}).get("INTERCEPTOR_GAS", [])
    if len(idx_all):
      last = int(idx_all[-1])
      if self._preap_pedal_last_idx is None or last != self._preap_pedal_last_idx:
        self.preap_pedal_detected = True
      self._preap_pedal_last_idx = last
    if len(gas_all):
      self.preap_pedal_raw = float(gas_all[-1])
      # The panda's gas test: raw INTERCEPTOR_GAS > 650 (tesla_preap.h:134-135), i.e. > 650 * 0.0507968128 - 22.85856576 in DBC
      # units (tesla_ap1_can.dbc:938). Any frame of this update counts, as each one updates the panda's gas_pressed.
      self.preap_panda_gas_pressed = max(float(g) for g in gas_all) > PANDA_GAS_PRESSED_DBC
    gas_sensor = pedal_cp.vl.get("GAS_SENSOR", {}) or {}
    self.pedal.update(gas_sensor, now_ms)
    self.pedal.update_torque(cp.vl.get("DI_torque1", {}) or {})
    self.pedal_interceptor_value = float(self.pedal.interceptor_value)
    self.pedal_timeout = bool(self.pedal.timeout)
    # A pedal that stops talking is no longer "detected" (UI status and cruise availability follow).
    if self.preap_pedal_detected and self.pedal_timeout and not len(idx_all):
      self.preap_pedal_detected = False
    self._mirror_pedal_status()

    # Gas / brake
    self.di_gas = cp.vl["DI_torque1"]["DI_pedalPos"] / 100.0   # CarState.gas is gone in 0.11.2; the calibration reads this
    # Stock (no interceptor) threshold is 2% so DI_pedalPos noise does not look like a press.
    # Matches NAP nap_conf.py:43 (PEDAL_DI_PRESSED = 2) and NAP preap/carstate.py:36-37
    # (gasPressed = DI_pedalPos > PEDAL_DI_PRESSED). di_gas itself is unchanged; calibration reads it.
    ret.gasPressed = self.di_gas > 0.02
    # Pedal position 0..1 for the onroad pedal gauge (disengaged right bar), in the deprecated CarState.gas slot (no schema
    # change; see DIVERGENCES). DI_pedalPos, not the Comma Pedal GAS_SENSOR. SNA / out of range is -1 (missing).
    ret.deprecated.gas = pedal_position(cp.vl["DI_torque1"]["DI_pedalPos"])
    brake_message_pressed = bool(cp.vl["BrakeMessage"]["driverBrakeStatus"] != 1)
    di_brake_pressed = int(cp.vl["DI_torque2"].get("DI_brakePedal", 0) or 0) == 1
    self.real_brake_pressed = bool(di_brake_pressed or brake_message_pressed)
    ret.brakePressed = self.real_brake_pressed
    # Brake level 0..1 for the onroad pedal gauge, in the deprecated CarState.brake slot (no schema change; display only):
    # binary from the press above, or with the iBooster swap toggle the 0x554 pedal position (missing / stale / fault -> -1).
    # brakePressed itself never changes with the toggle.
    if self.has_ibooster:
      ret.deprecated.brake = IBST.ibooster_brake_level(cp.vl.get(IBST.MSG), frame_fresh(cp, IBST.MSG, IBST.IBOOSTER_MAX_AGE_S))
    else:
      ret.deprecated.brake = IBST.binary_brake_level(self.real_brake_pressed)
    if self.pedal_long and self.preap_pedal_detected:
      ret.gasPressed = bool(self.pedal.gas_pressed)

    # Steering
    epas_status = cp.vl["EPAS_sysStatus"]
    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    self.steer_warning = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    self.eac_status = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]
    ret.steeringPressed = ap1_driver_input(self.hands_on_level)
    self.eac_fault = self.eac_status == "EAC_FAULT"
    ret.steerFaultPermanent = self.eac_fault
    ret.steerFaultTemporary = False   # INHIBITED + TMP_FAULT is the idle state with no Autopilot ECU

    # Cruise (DI) and gear
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp.vl["DI_state"]["DI_speedUnits"]), None)
    acc_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    ret.accFaulted = cruise_state == "FAULT"
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_torque2"]["DI_gear"].get(int(cp.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]
    if speed_units == "KPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_cruiseSet"] * CV.KPH_TO_MS
    elif speed_units == "MPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_cruiseSet"] * CV.MPH_TO_MS
    ret.cruiseState.standstill = False
    ret.standstill = cruise_state == "STANDSTILL"   # DI standstill, not vEgo (pedal holds 0 mph)

    # Buttons
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = int(cp.vl["STW_ACTN_RQ"].get("SpdCtrlLvr_Stat") or 0)
    self.spd_ctrl_lvr = self.cruise_buttons
    button_events = []
    for button in BUTTONS:
      state = (cp.vl[button.can_addr][button.can_msg] in button.values)
      if self.button_states[button.event_type] != state:
        button_events.append(structs.CarState.ButtonEvent(type=button.event_type, pressed=state))
      self.button_states[button.event_type] = state

    pull_edge = any(e.type == ButtonType.resumeCruise and e.pressed for e in button_events)
    if self.pedal_long:
      # Our own STW CANCEL spoof (StockCCSpoofer) must not read as a driver FWD disengage.
      # A cancel is real once more than 600 ms have passed since the last counted pull
      # (tesla_preap.h:114-120); an up or down tip does not open this window.
      for e in button_events:
        if e.type == ButtonType.cancel and is_preap_cancel_echo(
            now_ms=now_ms, last_pull_ms=int(self.preap_last_pull_ms or 0),
            last_cc_spoof_ms=int(self.preap_last_cc_spoof_ms or 0), cruise_enabled=bool(self.op_enabled)):
          e.type = ButtonType.unknown
    # No pedal: no echo filter. A CANCEL that reads back only ends the session (OFF), the stricter outcome; a driver cancel
    # right after a pull is never swallowed while stock cruise is cancelled by the DI.
    cancel_edge = any(e.type == ButtonType.cancel and e.pressed for e in button_events)
    if self.pedal_long:
      # Pedal long (pcmCruise False): the stalk pull leaves carstate as setCruise, not resumeCruise
      # (preap/cruise.py remap_preap_pedal_button_type, written for this and until now never called).
      # selfdrived raises resumeBlocked, "Press Set to Engage", for a resume with no set yet
      # (selfdrived.py:267-268: not pcmCruise and vCruise > 250 and an accel/resume edge; vCruise is
      # V_CRUISE_UNSET 255 until the first engage, so the first pull of every drive showed it). Pre-AP
      # pedal long has no stock set to resume to: the set is captured at engage
      # (card.py initialize_v_cruise / capture_preap_full), as NAP does.
      # Everything Pre-AP reads itself is taken above or off button_states, never off these types:
      # pull_edge / cancel_edge (:255, here), the pull FSM, update_button_enable and fp_ret.resumePressed.
      # Only what leaves carstate changes. Side effect, intended and NAP-parity: cruise.py:144 restores
      # v_cruise_kph_last on an accelCruise / resumeCruise edge, so a re-engage now re-captures the set
      # from ego (or the posted limit) instead of restoring the previous set.
      for e in button_events:
        e.type = remap_preap_pedal_button_type(e.type)
    ret.buttonEvents = button_events

    # The panda drops controls_allowed and the cruise latch on a permanent EPAS fault, hands on at level 3 or an EPAS refusal
    # (tesla_preap.h tesla_preap_epas_state, as in NAP and BogPilot na-preap). The pull FSM is reset on exactly the same EPAS_sysStatus
    # inputs, so openpilot never shows engaged while the panda blocks steering, and re-engaging needs a new pull edge.
    eac_status = int(epas_status["EPAS_eacStatus"])
    eac_error_code = int(epas_status["EPAS_eacErrorCode"])
    self.preap_epas_refusing = preap_epas_refusing(int(self.hands_on_level), eac_status, eac_error_code)
    self.preap_panda_drop = preap_steer_drop(eac_status, int(self.hands_on_level), eac_error_code)
    # Hands on at level 3 or an EPAS refusal: the same inputs as the panda's tesla_preap_epas_blocking drop. steeringDisengage makes
    # openpilot disengage on that frame (steerDisengage, rising edge), with or without the pedal, as NAP does
    # (NAP opendbc preap/carstate.py:60-68 @ cc39c7fa). EAC_FAULT stays steerFaultPermanent (steerUnavailable).
    ret.steeringDisengage = bool(self.preap_epas_refusing)
    # Doors (DOORS includes frunk and trunk, like the panda's tesla_preap_door_open). Read before the pull FSM, which needs them.
    ret.doorOpen = any((self.can_define.dv["GTW_carState"][door].get(int(cp.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)
    if self.pedal_long:
      # Accelerator, by either test. The panda refuses pedal ENABLE while gas_pressed_prev (get_longitudinal_allowed), so the guard
      # sends the first ENABLE=1 only one update after both read released.
      gas_any = bool(ret.gasPressed) or bool(self.preap_panda_gas_pressed)
      if (gas_any and not self._prev_tx_block_gas) or (bool(ret.brakePressed) and not self._prev_tx_block_brake):
        self._tx_block_press_ms = now_ms
      self._prev_tx_block_gas = bool(gas_any)
      self._prev_tx_block_brake = bool(ret.brakePressed)
      # A userspace-guard block or a panda refusal of our own 0x551 / 0x488 frame (their rejected echoes, card.py; plus blocked
      # frames whose echo was lost, fail-safe) drops FULL to steering only with the speed-blocked warning. A TX-block delta inside
      # TX_BLOCK_PRESS_IGNORE_MS of a gas or brake press edge is the expected panda refusal of ENABLE=1
      # (tesla_preap.h:254) and is ignored; a guard block or bus mismatch still demotes, and any TX block
      # outside the window still demotes.
      tx_blocked = int(self.preap_panda_tx_blocked) > 0
      press_tx = bool(self._tx_block_press_ms) and 0 <= (now_ms - self._tx_block_press_ms) <= TX_BLOCK_PRESS_IGNORE_MS
      # The radar is advisory (preap/radar_valid.py, NAP / Tinkla): its state is not an input here. A radar that is not ready only
      # keeps its points out of radard (camera leads) and raises one warning (card.py, preapRadarNotReady).
      speed_block = ((self.preap_pedal_guard_blocks != self._preap_guard_blocks_seen) or (tx_blocked and not press_tx)
                     or bool(self.preap_pedal_bus_mismatch))
      self.preap_gas_clear_updates = 0 if gas_any else min(self.preap_gas_clear_updates + 1, 1000)
      self._preap_guard_blocks_seen = self.preap_pedal_guard_blocks
      self.preap_panda_tx_blocked = 0
      # NAP-style double pull (preap/engagement.py). Unknown DI state / gear are passed as None and read as unsafe by the FSM.
      self.decision = self.engagement.update(
        now_ms, pull_edge=pull_edge, cancel_edge=cancel_edge, brake_pressed=ret.brakePressed,
        op_enabled=bool(self.op_enabled), pedal=True,
        gear_d=(ret.gearShifter == structs.CarState.GearShifter.drive), door_open=bool(ret.doorOpen),
        epas_drop=bool(self.preap_panda_drop), stale=not bool(getattr(cp, "can_valid", True)),
        di_engaged=(acc_enabled if cruise_state is not None else None), v_ego=float(ret.vEgo),
        long_allowed=bool(self.preap_pedal_detected and is_pedal_calibrated() and self.preap_pedal_bus_ok),
        speed_block=speed_block, gas_pressed=gas_any, pedal_bus_ok=bool(self.preap_pedal_bus_ok))
      self.preap_fsm_mode = int(self.decision.mode)
      self.preap_standstill_hold = bool(self.decision.standstill_hold)
      self.preap_capture_edge = bool(self.decision.capture_edge)
      self._enable_request = bool(self.decision.enable_request)
      ret.cruiseState.enabled = bool(self.decision.engaged)
      ret.cruiseState.available = pedal_cruise_available(
        detected=bool(self.preap_pedal_detected), calibrated=is_pedal_calibrated(),
        # a failed pedal acquisition (pedal_unavailable) is retried with back-off and reported by its own alert. It no longer
        # makes cruise "unavailable": that was a latch (wrongCarMode blocked the engage that would have cleared it).
        faulted=bool(ret.accFaulted))
      fp_ret.preapLatOnly = bool(self.decision.lat_only)
      fp_ret.preapAolLat = bool(self.decision.aol_lat)
    else:
      # No pedal: the same pull FSM (NAP runs one FSM for both, NAP preap/engagement.py:126-173 @ e99f4dc75). openpilot still engages
      # from cruiseState.enabled rising (pcmCruise). The first pull is steering only: if it armed stock cruise we CANCEL it (0x45 is
      # cancel-only in the panda); the driver's real second pull re-arms stock cruise and that is FULL.
      self.decision = self.engagement.update(
        now_ms, pull_edge=pull_edge, cancel_edge=cancel_edge, brake_pressed=ret.brakePressed,
        op_enabled=bool(self.op_enabled), pedal=False,
        gear_d=(ret.gearShifter == structs.CarState.GearShifter.drive), door_open=bool(ret.doorOpen),
        epas_drop=bool(self.preap_panda_drop), stale=not bool(getattr(cp, "can_valid", True)),
        di_engaged=(acc_enabled if cruise_state is not None else None), v_ego=float(ret.vEgo),
        long_allowed=False, cancel_tx_ms=int(self.preap_cc_cancel_tx_ms or 0))
      self.preap_fsm_mode = int(self.decision.mode)
      if self.decision.drop_cancel:
        self.preap_cc_cancel_drop = True
        self.preap_cc_cancel_needed = False
      if self.decision.cancel_cc:
        self.preap_cc_cancel_needed = True
      ret.cruiseState.enabled = bool(self.decision.engaged)
      ret.cruiseState.available = (cruise_state == "STANDBY") or acc_enabled
      fp_ret.preapLatOnly = bool(self.decision.lat_only)
      self._enable_request = False

    # Blinkers, seatbelt (doors are read above, before the pull FSM)

    # Mirror of the panda's tesla_preap_need_fresh_pull (tesla_preap.h): set by every EPAS drop (tesla_preap_epas_state), cleared by a
    # stalk pull (SpdCtrlLvr_Stat RWD) in D with the doors closed, except a pull held through the drop (0x45). If a drop and a clearing
    # pull land in the same update the drop wins, which is the stricter of the two frame orders the panda can see.
    lever_pull = self.cruise_buttons == SPD_CTRL_MAIN
    in_d_closed = ret.gearShifter == structs.CarState.GearShifter.drive and not ret.doorOpen
    counted_pull = lever_pull and not (self.preap_fresh_pull_pending and self.prev_cruise_buttons == SPD_CTRL_MAIN) and in_d_closed
    if counted_pull:
      self.preap_fresh_pull_pending = False
    if self.preap_panda_drop:
      self.preap_fresh_pull_pending = True
    # Mirror of the panda's steer-through-brake latch: armed by a counted pull (tesla_preap.h:98-110), disarmed by a cancel outside the
    # 600 ms echo window (:111-120), an EPAS drop or fault (:52-59), leaving D, a door or stale data (:78-88); brake_seen is set by a brake
    # while armed (:89-91). preapSteerLatch = armed && brake_seen (:92): the panda keeps steering through that brake press, so openpilot
    # lateral-only after the brake is on purpose (selfdrived preap_brake). Same drop-wins order as the fresh-pull mirror.
    if counted_pull:
      self.preap_steer_armed, self.preap_steer_brake_seen, self.preap_last_pull_ms = True, False, now_ms
    elif self.cruise_buttons == SPD_CTRL_CANCEL and (now_ms - self.preap_last_pull_ms) > 600:
      self.preap_steer_armed = self.preap_steer_brake_seen = False
    if self.preap_panda_drop or not in_d_closed or not bool(getattr(cp, "can_valid", True)):
      self.preap_steer_armed = self.preap_steer_brake_seen = False
    if self.preap_steer_armed and ret.brakePressed:
      self.preap_steer_brake_seen = True
    fp_ret.preapSteerLatch = bool(self.preap_steer_armed and self.preap_steer_brake_seen)
    fp_ret.preapFsmMode = int(self.decision.mode)
    fp_ret.preapSpeedBlocked = bool(self.decision.speed_blocked or self.decision.standstill_refused or self.decision.cc_not_armed)
    # Stock cruise: while the panda waits for a fresh pull it keeps controls off, so stock TACC coming on without that pull must not
    # engage openpilot. blockPcmEnable stops only the pcmEnable rising edge (selfdrive/car/car_events.py); cruiseState.enabled, pcmDisable
    # and every disengage path are unchanged, and no alert is added. NAP gets the same result by setting cruiseState.enabled only from a
    # stalk pull (NAP opendbc preap/carstate.py:139, preap/engagement.py:67-78 @ cc39c7fa). The pedal mode engages from buttonEnable.
    if not self.pedal_long:
      ret.blockPcmEnable = bool(self.preap_fresh_pull_pending)
    ret.leftBlinker = cp.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
    ret.rightBlinker = cp.vl["GTW_carState"]["BC_indicatorRStatus"] == 1
    ret.seatbeltUnlatched = cp.vl["SDM1"]["SDM_bcklDrivStatus"] != 1
    ret.stockAeb = False

    # Posted limit: GTW UI map / mpp only (no Mobileye fused limit on Pre-AP).
    ui_map = cp.vl.get("UI_driverAssistMapData", {}).get("UI_mapSpeedLimit")
    gps = cp.vl.get("UI_gpsVehicleSpeed", {})
    fp_ret.dashboardSpeedLimit = dashboard_speed_limit_ms(None, ui_map, gps.get("UI_mppSpeedLimit"), gps.get("UI_mapSpeedLimitUnits"))
    # Engage set (SetSpeedLimit): fresh frames only, and the units frame (UI_mapSpeedLimitUnits, 0x2F8) must be
    # fresh too, so a limit in unknown units never latches. Stale or unknown = 0 = normal engage set.
    if frame_fresh(cp, "UI_gpsVehicleSpeed"):
      self.engage_limit_ms = dashboard_speed_limit_ms(
        None, ui_map if frame_fresh(cp, "UI_driverAssistMapData") else None,
        gps.get("UI_mppSpeedLimit"), gps.get("UI_mapSpeedLimitUnits"))
    else:
      self.engage_limit_ms = 0.0

    self.msg_stw_actn_req = copy.copy(cp.vl["STW_ACTN_RQ"])

    # Continuous stalk levels for the SLC hand-over (same carState fields AP1 publishes).
    fp_ret.spdCtrlLvr = max(0, min(255, int(self.spd_ctrl_lvr)))
    fp_ret.accelPressed = bool(self.button_states.get(ButtonType.accelCruise, False))
    fp_ret.decelPressed = bool(self.button_states.get(ButtonType.decelCruise, False))
    fp_ret.resumePressed = bool(self.button_states.get(ButtonType.resumeCruise, False))
    fp_ret.ap1HandsOnLevel = max(0, min(255, int(self.hands_on_level or 0)))
    fp_ret.preapPedalDetected = bool(self.preap_pedal_detected)
    fp_ret.preapPedalCalibrated = bool(is_pedal_calibrated())
    # 0x108 motor torque (Nm) and RPM for the onroad pedal gauge's regen bar, raw decoded (SNA stays -1024 Nm / -32768 rpm).
    # Same DI_torque1 frame and signals as AP1 (tesla_ap1_can.dbc BO_ 264); the field names keep their AP1 prefix (no
    # schema change). Nothing in Pre-AP controls reads them (regen_aware stop shaping is AP1 Model S only).
    di_torque1 = cp.vl["DI_torque1"]
    fp_ret.ap1MotorTorqueNm = float(di_torque1["DI_torqueMotor"])
    fp_ret.ap1MotorRpm = float(di_torque1["DI_motorRPM"])
    fp_ret.preapPedalUnavailable = bool(self.pedal_unavailable)
    fp_ret.preapPedalFailCause = int(self.pedal_fail_cause) if self.pedal_unavailable else 0
    # Pedal longitudinal wanted but the Comma Pedal stream is missing, or its counter has not moved for PEDAL_TIMEOUT_MS (stale).
    # The panda requires the 0x552 stream (steering stops with it), so openpilot says "Pedal Not Responding" (ap1_events.py).
    fp_ret.preapPedalNotResponding = bool(self.pedal_long and self.pedal_timeout)
    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Pre-AP chassis rates under a loaded bus: halve the high-rate expectations so canValid holds.
    messages = [
      ("ESP_B", 50),
      ("DI_torque1", 50),
      ("DI_torque2", 50),
      ("STW_ANGLHP_STAT", 50),
      ("EPAS_sysStatus", 25),
      ("DI_state", 5),
      ("STW_ACTN_RQ", 10),
      ("GTW_carState", 10),
      ("BrakeMessage", 25),
      ("SDM1", 10),
      ("UI_driverAssistMapData", OPTIONAL),
      ("UI_gpsVehicleSpeed", OPTIONAL),
    ]
    if get_bool(PreApParamKeys.HAS_IBOOSTER):
      # iBooster swap: 0x554 on bus 0, frequency 0 like Tinkla (carstate.py:653-662@501c7de), so canValid never depends on it.
      messages.append((IBST.MSG, OPTIONAL))
    cam_messages = []
    # Listen on bus 0 and bus 2 so the first 0x552 can latch, as tesla_preap.h:297-298 does.
    messages.append(("GAS_SENSOR", OPTIONAL))
    cam_messages.append(("GAS_SENSOR", OPTIONAL))
    return {
      Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis], messages, CANBUS.chassis),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.chassis], cam_messages, CANBUS.autopilot_chassis),
    }
