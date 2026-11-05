import copy
import time
from collections import deque
from cereal import car, custom
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.tesla.blindspot import Ap1Blindspot, frames_from_vl_all
from openpilot.selfdrive.car.tesla.cluster import CLUSTER_ADDRS, COUNTER_SIGNALS, MSG_NAMES
from openpilot.selfdrive.car.tesla.hso import ap1_driver_input
from openpilot.selfdrive.car.tesla.stalk_follow import dtr_sample, follow_seconds, parse_stalk_raw
from openpilot.selfdrive.car.tesla.stalk_pull_hold import PULL_HOLD_ENABLED, StalkPullHold
from openpilot.selfdrive.car.tesla.stalk_fwd_hold import FWD_HOLD_ENABLED, StalkFwdHold
from openpilot.selfdrive.car.tesla.speed_limit import dashboard_speed_limit_ms
from openpilot.selfdrive.car.tesla.steer_fault import steer_fault_temporary
from openpilot.selfdrive.car.tesla.values import CAR, DBC, CANBUS, GEAR_MAP, DOORS, BUTTONS
from openpilot.selfdrive.car.tesla.preap.calib import is_pedal_calibrated
from openpilot.selfdrive.car.tesla.preap.pedal_feedback import PedalFeedback
from openpilot.selfdrive.car.tesla.preap.cruise import (
  pedal_cruise_available, remap_preap_pedal_button_type, is_preap_cancel_echo,
)
from openpilot.selfdrive.car.tesla.preap.engagement import DoublePullEngagement, preap_steer_drop
from openpilot.selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine

class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)
    self.button_states = {button.event_type: False for button in BUTTONS}
    # Pre-AP pedal: stalk SpdCtrl + stock-CC cancel echo filter (NAP)
    self.cruise_buttons = 0
    self.prev_cruise_buttons = 0
    self.preap_cc_cancel_needed = False
    self.preap_last_cc_spoof_ms = 0
    self.last_stalk_non_cancel_ms = 0
    self.can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])

    # Needed by carcontroller
    self.msg_stw_actn_req = None
    self.hands_on_level = 0
    self.steer_warning = None
    self.eac_status = None
    self.eac_fault = False
    self.acc_state = 0
    self.das_control_counters = deque(maxlen=32)
    # DTR_Dist_Rq decision. Python only; cereal was not extended.
    self.spd_ctrl_lvr = 0
    self.stalk_follow = None
    # AP1 long RWD pull → Experimental Mode (see stalk_pull_hold.py).
    self.stalk_pull_hold = StalkPullHold()
    self.stalk_pull_toggle = False
    # AP1 long FWD hold while disengaged → Experimental Mode (stalk_fwd_hold.py).
    self.stalk_fwd_hold = StalkFwdHold()
    self.stalk_fwd_toggle = False
    # AP1 only. {addr: decoded values} for stock cluster frames (bus 2) that
    # arrived this step. Read by CarController for the cluster frames.
    self.cluster_stock = {}
    # AP1 only. Stock DAS_steeringControlCounter values (bus 2) received this
    # step, oldest first. CarController follows them (steer_counter.py).
    self.stock_steer_counters = []
    # AP1 only. Last EPB_epasEACAllow seen. None until the first 0x214 frame
    # (vl defaults to 0 before that, which would read as revoked).
    self.epb_eac_allow = None
    self.epb_eac_revoked = False
    # AP1 only. Stock 0x399 blind-spot fields with a per-side hold (blindspot.py).
    self.ap1_bsm = Ap1Blindspot()
    # Pre-AP comma pedal (file-backed status). Detected from GAS_SENSOR IDX updates.
    self.preap_pedal_detected = False
    self.preap_pedal_raw = None
    self._preap_pedal_last_idx = None
    # NAP-style PedalFeedback (GAS_SENSOR STATE/IDX + DI_torqueMotor)
    self.pedal = PedalFeedback()
    self.pedal_interceptor_value = 0.0
    self.pedal_timeout = True
    self.real_brake_pressed = False
    self.pedal_unavailable = False
    self.pedal_authority_requested = False
    self.pedal_authority_active = False
    self.pedal_authority_state = 0
    self.pedal_authority_action = 0
    self.pedal_command_counter = 0
    self.pedal_command_di = 0.0
    self.pedal_brake_required = False
    self.vdas_limited_accel = 0.0
    self.pedal_first_enabled_mono_time = 0
    self.double_pull = DoublePullEngagement()
    self.pedal_cruise_lat_only = False

  def update(self, cp, cp_cam, frogpilot_toggles):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # Default: vEgo-based. Pre-AP overrides below to DI STANDSTILL only (NAP).
    ret.standstill = (ret.vEgo < 0.1)

    # Pre-AP: comma pedal detection from GAS_SENSOR IDX changes on cam/chassis bus.
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      idx_all = cp_cam.vl_all.get("GAS_SENSOR", {}).get("IDX", [])
      gas_all = cp_cam.vl_all.get("GAS_SENSOR", {}).get("INTERCEPTOR_GAS", [])
      if idx_all:
        last = int(idx_all[-1])
        if self._preap_pedal_last_idx is None or last != self._preap_pedal_last_idx:
          self.preap_pedal_detected = True
        self._preap_pedal_last_idx = last
      if gas_all:
        self.preap_pedal_raw = float(gas_all[-1])

    # Gas pedal (DI). Pre-AP pedal-long overrides gasPressed from interceptor below.
    ret.gas = cp.vl["DI_torque1"]["DI_pedalPos"] / 100.0
    ret.gasPressed = (ret.gas > 0)

    # Brake pedal — NAP: BrakeMessage OR DI_brakePedal
    ret.brake = 0
    brake_message_pressed = bool(cp.vl["BrakeMessage"]["driverBrakeStatus"] != 1)
    try:
      di_brake_pressed = int(cp.vl["DI_torque2"].get("DI_brakePedal", 0) or 0) == 1
    except Exception:
      di_brake_pressed = False
    self.real_brake_pressed = bool(di_brake_pressed or brake_message_pressed)
    ret.brakePressed = self.real_brake_pressed

    # Pre-AP: update PedalFeedback from GAS_SENSOR + DI_torqueMotor (NAP-style)
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      now_ms = int(time.monotonic() * 1000)
      gas_sensor = cp_cam.vl.get("GAS_SENSOR", {}) or {}
      self.pedal.update(gas_sensor, now_ms)
      self.pedal.update_torque(cp.vl.get("DI_torque1", {}) or {})
      self.pedal_interceptor_value = float(self.pedal.interceptor_value)
      self.pedal_timeout = bool(self.pedal.timeout)
      if gas_sensor and "INTERCEPTOR_GAS" in gas_sensor:
        self.preap_pedal_raw = float(gas_sensor["INTERCEPTOR_GAS"])

    # Steering wheel
    epas_status = cp_cam.vl["EPAS3P_sysStatus"] if self.CP.carFingerprint == CAR.TESLA_MODELS_RAVEN else cp.vl["EPAS_sysStatus"]

    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    self.steer_warning = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    steer_status = self.can_define.dv["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    self.eac_status = steer_status

    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"] # This is from a different angle sensor, and at different rate
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]
    # Any non-zero hands level is driver input (override, grey border), as in
    # frog_ap1. AP1 EPAS reports 0, 1, 3. The AP1 lateral pause stays at
    # TinklaHandsOnLevel 2 in CarController (hso.ap1_steering_pressed).
    ap1 = self.CP.carFingerprint == CAR.TESLA_AP1_MODELS
    if ap1:
      ret.steeringPressed = ap1_driver_input(self.hands_on_level)
    else:
      ret.steeringPressed = (self.hands_on_level > 0)
    self.eac_fault = steer_status == "EAC_FAULT"
    ret.steerFaultPermanent = self.eac_fault
    if ap1:
      # EPB_epasEACAllow 0: the EPB revoked EAC. EPAS stays INHIBITED until a
      # car power cycle (the comma reboot does not clear it). Report it as a
      # permanent steer fault ("LKAS Fault: Restart the Car") instead of only
      # the silent steer-unavailable warning.
      epb = cp.vl_all.get("EPB_epasControl", {}).get("EPB_epasEACAllow", [])
      if len(epb):
        self.epb_eac_allow = int(epb[-1])
      self.epb_eac_revoked = self.epb_eac_allow == 0
      ret.steerFaultPermanent = ret.steerFaultPermanent or self.epb_eac_revoked
    # AP1: idle, code 6, and latched HANDS_ON (3) are not temporary faults.
    # Both codes stay set after hands return below 2, so they must not keep
    # latActive false. The hands pause is hands_on_level >= 2, not this flag.
    # Other non-idle names still warn. Model 3/Y still faults on code 6 and
    # still does not fault on HANDS_ON.
    # Pre-AP: EAC_INHIBITED + EAC_ERROR_TMP_FAULT is the normal idle (no AP
    # ECU). NAP forces steerFaultTemporary False; mapping INHIBITED/TMP_FAULT
    # to a temporary fault deadlocks latActive and matches the tester's
    # steerTempUnavailableSilent while stopped and not engaged.
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      ret.steerFaultTemporary = False
    else:
      ret.steerFaultTemporary = steer_fault_temporary(self.steer_warning, ap1)

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp.vl["DI_state"]["DI_speedUnits"]), None)

    acc_enabled = (cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL"))

    ret.accFaulted = cruise_state == "FAULT"

    # Pre-AP + comma pedal (pcmCruise=False): available must not require stock
    # DI STANDBY/ENABLED — that surfaces wrongCarMode ("Enable Adaptive Cruise
    # to Engage"). Match NAP/Tinkla: available when pedal is ready; OP owns
    # enabled via stalk buttonEnable (panda arms on SpdCtrl RWD=2).
    preap_pedal_long = (
      self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS
      and bool(self.CP.openpilotLongitudinalControl)
    )
    if preap_pedal_long:
      ret.cruiseState.enabled = False
      ret.cruiseState.available = pedal_cruise_available(
        detected=bool(self.preap_pedal_detected),
        calibrated=is_pedal_calibrated(),
        faulted=bool(ret.accFaulted),
      )
    else:
      ret.cruiseState.enabled = acc_enabled
      ret.cruiseState.available = ((cruise_state == "STANDBY") or ret.cruiseState.enabled)

    # NAP: when pedal long is on, gasPressed comes from interceptor DI, not DI_pedalPos
    if preap_pedal_long and self.preap_pedal_detected:
      ret.gasPressed = bool(self.pedal.gas_pressed)

    # Tinkla-aligned: cruise set is DI_cruiseSet (ACC set), not DI_digitalSpeed
    # (≈ ego). With pcmCruise, VCruiseHelper copies this into controlsState.vCruise
    # so openpilot's set tracks the stalk set. Display/ego stay on ESP vEgo.
    if speed_units == "KPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_cruiseSet"] * CV.KPH_TO_MS
    elif speed_units == "MPH":
      ret.cruiseState.speed = cp.vl["DI_state"]["DI_cruiseSet"] * CV.MPH_TO_MS
    ret.cruiseState.standstill = False # This needs to be false, since we can resume from stop without sending anything special
    # Pre-AP (NAP): standstill is DI cruise STANDSTILL only — not vEgo<0.1.
    # Pedal holds speed at 0 mph; vEgo-based standstill + brake fired
    # preEnableStandstill ("Release Brake to Engage") and blocked engage.
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      ret.standstill = (cruise_state == "STANDSTILL")

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_torque2"]["DI_gear"].get(int(cp.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Buttons
    self.prev_cruise_buttons = self.cruise_buttons
    try:
      self.cruise_buttons = int(cp.vl["STW_ACTN_RQ"].get("SpdCtrlLvr_Stat") or 0)
    except Exception:
      self.cruise_buttons = 0
    buttonEvents = []
    for button in BUTTONS:
      state = (cp.vl[button.can_addr][button.can_msg] in button.values)
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        buttonEvents.append(event)
      self.button_states[button.event_type] = state
    if preap_pedal_long:
      now_ms = int(time.monotonic() * 1000)
      for event in buttonEvents:
        event.type = remap_preap_pedal_button_type(event.type)
        # Track non-cancel stalk edges for CANCEL_ECHO_WINDOW (NAP).
        if event.pressed and event.type in (
            car.CarState.ButtonEvent.Type.setCruise,
            car.CarState.ButtonEvent.Type.accelCruise,
            car.CarState.ButtonEvent.Type.decelCruise,
            car.CarState.ButtonEvent.Type.resumeCruise,
        ):
          self.last_stalk_non_cancel_ms = now_ms
      # Suppress our STW CANCEL spoof so it is not a driver FWD disengage.
      was_enabled = bool(getattr(self, "out", None) and self.out.cruiseState.enabled)
      for event in buttonEvents:
        if event.type == car.CarState.ButtonEvent.Type.cancel:
          if is_preap_cancel_echo(
              now_ms=now_ms,
              last_stalk_non_cancel_ms=self.last_stalk_non_cancel_ms,
              last_cc_spoof_ms=int(getattr(self, "preap_last_cc_spoof_ms", 0) or 0),
              cruise_enabled=was_enabled,
          ):
            event.type = car.CarState.ButtonEvent.Type.unknown
      # Optional NAP-style double-pull under Always On Lateral.
      dp_on = bool(getattr(frogpilot_toggles, "tesla_preap_double_pull", False))
      cancel_pressed = any(
        e.type == car.CarState.ButtonEvent.Type.cancel and e.pressed for e in buttonEvents
      )
      buttonEvents = self.double_pull.filter_button_events(
        buttonEvents,
        enabled=dp_on,
        now_ms=now_ms,
        brake_pressed=bool(ret.brakePressed),
        cancel=cancel_pressed,
      )
      # The panda drops controls on these, so the latch must not outlive them.
      self.preap_panda_drop = preap_steer_drop(self.hands_on_level, int(epas_status["EPAS_eacStatus"]),
                                               int(epas_status["EPAS_eacErrorCode"]))
      if self.preap_panda_drop or ret.gearShifter != car.CarState.GearShifter.drive:
        self.double_pull.reset()
      self.pedal_cruise_lat_only = bool(dp_on and self.double_pull.lat_only)
    ret.buttonEvents = buttonEvents

    # Doors
    ret.doorOpen = any((self.can_define.dv["GTW_carState"][door].get(int(cp.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    # Blinkers
    ret.leftBlinker = (cp.vl["GTW_carState"]["BC_indicatorLStatus"] == 1)
    ret.rightBlinker = (cp.vl["GTW_carState"]["BC_indicatorRStatus"] == 1)

    # Seatbelt
    if self.CP.carFingerprint == CAR.TESLA_MODELS_RAVEN:
      ret.seatbeltUnlatched = (cp.vl["DriverSeat"]["buckleStatus"] != 1)
    else:
      ret.seatbeltUnlatched = (cp.vl["SDM1"]["SDM_bcklDrivStatus"] != 1)

    # Blindspot. AP1 only: stock DAS_blindSpotRearLeft/Right from 0x399 on
    # cp_cam (1/2 = warning, 3 = SNA ignored, bad checksum skipped), held
    # BSM_HOLD_S per side because 0x399 is ~2.4 Hz. Other Teslas: no BSM.
    if ap1:
      ret.leftBlindspot, ret.rightBlindspot = self.ap1_bsm.update(frames_from_vl_all(cp_cam), DT_CTRL)

    # AEB (AP ECU). Pre-AP has no DAS_control.
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      ret.stockAeb = False
    else:
      ret.stockAeb = (cp_cam.vl["DAS_control"]["DAS_aebEvent"] == 1)

    # AP1 tip detent: raw SpdCtrlLvr_Stat (UP_2ND=4 / DN_2ND=8 vs 1ST).
    try:
      self.spd_ctrl_lvr = int(cp.vl["STW_ACTN_RQ"].get("SpdCtrlLvr_Stat") or 0)
    except Exception:
      self.spd_ctrl_lvr = 0

    # Stalk follow detent. STW_ACTN_RQ is already subscribed. No cereal field.
    # A zero timestamp is the parser default, not ACC_DIST_1.
    stw = cp.vl.get("STW_ACTN_RQ")
    ts_map = getattr(cp, "ts_nanos", {}).get("STW_ACTN_RQ", {})
    if not isinstance(stw, dict) or not isinstance(ts_map, dict):
      raw = None
    else:
      raw = dtr_sample(stw.get("DTR_Dist_Rq"), ts_map.get("DTR_Dist_Rq", 0))
    # No debounce: fast intentional spins apply at once (user request, 10/13).
    self.stalk_follow = parse_stalk_raw(raw, self.stalk_follow)
    # Existing cruiseState.speedOffset is unused on Tesla. AP1 publishes the
    # stalk follow seconds there so FrogPilotFollowing can set tFollow.
    # 0 means no ready detent. 255 holds the last seconds via parse_stalk_raw.
    if self.CP.carFingerprint == CAR.TESLA_AP1_MODELS:
      ret.cruiseState.speedOffset = follow_seconds(self.stalk_follow)

    # AP1: long RWD pull while already engaged → one Experimental Mode toggle.
    # Uses prior cruise enabled so the engage pull itself does not fire.
    # DI may bump set speed mid-hold (resume); accepted — see stalk_pull_hold.py.
    # AP1: long FWD hold while already disengaged → one Experimental Mode toggle.
    # Cancel-while-engaged is disarmed; engage mid-hold aborts (stalk_fwd_hold.py).
    self.stalk_pull_toggle = False
    self.stalk_fwd_toggle = False
    if self.CP.carFingerprint == CAR.TESLA_AP1_MODELS:
      prev = getattr(self, "out", None)
      was_engaged = bool(prev.cruiseState.enabled) if prev is not None else False
      spd = cp.vl["STW_ACTN_RQ"].get("SpdCtrlLvr_Stat")
      if PULL_HOLD_ENABLED:
        self.stalk_pull_toggle = self.stalk_pull_hold.update(spd, was_engaged, DT_CTRL)
      if FWD_HOLD_ENABLED:
        self.stalk_fwd_toggle = self.stalk_fwd_hold.update(spd, was_engaged, DT_CTRL)

    # Messages needed by carcontroller
    self.msg_stw_actn_req = copy.copy(cp.vl["STW_ACTN_RQ"])
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      self.acc_state = 0
    else:
      self.acc_state = cp_cam.vl["DAS_control"]["DAS_accState"]
      self.das_control_counters.extend(cp_cam.vl_all["DAS_control"]["DAS_controlCounter"])
    self.cluster_stock = self.new_cluster_frames(cp_cam) if ap1 else {}
    if ap1:
      steer_ctrs = cp_cam.vl_all.get("DAS_steeringControl", {}).get("DAS_steeringControlCounter", [])
      self.stock_steer_counters = [int(c) for c in steer_ctrs]

    # AP1: Mobileye fused (stock 0x399 on cp_cam / bus 2) then GTW UI map/mpp
    # into FrogPilot dashboardSpeedLimit (m/s). Cluster TX is on bus 0 and
    # never reaches cp_cam. Freq-0 UI msgs on chassis; missing → 0.
    # AP1: stock Autopilot active (stock 0x399 on bus 2, state 3/4/5). Gates the
    # standstill-experiment standby DAS_control (Tinkla: not while stock AP).
    if ap1:
      st = cp_cam.vl.get("AutopilotStatus", {}).get("autopilotStatus", 0)
      self.stock_ap_active = 3 <= int(st or 0) <= 5
    # AP1: Mobileye fused (stock 0x399) then GTW UI map/mpp.
    # Pre-AP: no Mobileye — fused=None; map/mpp only (absent msgs → 0).
    if ap1 or self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      if ap1:
        fused = cp_cam.vl.get("AutopilotStatus", {}).get("DAS_fusedSpeedLimit")
      else:
        fused = None
      ui_map = cp.vl.get("UI_driverAssistMapData", {}).get("UI_mapSpeedLimit")
      gps = cp.vl.get("UI_gpsVehicleSpeed", {})
      fp_ret.dashboardSpeedLimit = dashboard_speed_limit_ms(
        fused, ui_map, gps.get("UI_mppSpeedLimit"), gps.get("UI_mapSpeedLimitUnits"),
      )

    if ap1:
      # Regen-aware stop braking inputs (regen_aware.py).
      fp_ret.ap1MotorTorqueNm = float(cp.vl["DI_torque1"]["DI_torqueMotor"])
      fp_ret.ap1MotorRpm = float(cp.vl["DI_torque1"]["DI_motorRPM"])
      amb = cp.vl.get("GTW_ESP1", {})
      fp_ret.ap1AmbientTempC = float(amb["GTW_ambientTemperature"]) if getattr(cp, "ts_nanos", {}).get("GTW_ESP1", {}).get("GTW_ambientTemperature", 0) else float("nan")

    # Pre-AP NAP-style pedal HUD telemetry on FrogPilotCarState (not stock CarState).
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      fp_ret.pedalMaxRegen = bool(self.pedal_brake_required)
      fp_ret.pedalUnavailable = bool(self.pedal_unavailable)
      fp_ret.pedalLongActive = bool(self.pedal_authority_active)
      # First-pull lat-only is filled by double-pull engagement when enabled.
      fp_ret.pedalCruiseLatOnly = bool(getattr(self, "pedal_cruise_lat_only", False))

    return ret, fp_ret

  @staticmethod
  def new_cluster_frames(cp_cam):
    """Stock AutopilotStatus / DAS_status2 / DAS_lanes received this step.

    A frame counts as new when its counter signal has a value in vl_all for
    this update. vl holds the newest decoded values of that frame.
    """
    out = {}
    for addr in CLUSTER_ADDRS:
      name = MSG_NAMES[addr]
      if len(cp_cam.vl_all[name].get(COUNTER_SIGNALS[addr], [])):
        out[addr] = dict(cp_cam.vl[name])
    return out

  @staticmethod
  def get_can_parser(CP, FPCP):
    # Pre-AP + radar GTW emulation forwards chassis frames onto CAN2 at high
    # rate (interruptRateCan2). Under that load the chassis parser saw TIMED OUT
    # on 0x108/0x118/0x20A/0x368 at stock freqs. Halve the high-rate expectations
    # for Pre-AP only so canValid stays true when the car is still sending; AP1
    # freqs unchanged. GTW addrs/rates themselves already match NAP.
    preap = CP.carFingerprint == CAR.TESLA_PREAP_MODELS
    messages = [
      # sig_address, frequency
      ("ESP_B", 50),
      ("DI_torque1", 50 if preap else 100),
      ("DI_torque2", 50 if preap else 100),
      ("STW_ANGLHP_STAT", 50 if preap else 100),
      ("EPAS_sysStatus", 25),
      ("DI_state", 5 if preap else 10),
      ("STW_ACTN_RQ", 10),
      ("GTW_carState", 10),
      ("BrakeMessage", 25 if preap else 50),
    ]

    if CP.carFingerprint == CAR.TESLA_MODELS_RAVEN:
      messages.append(("DriverSeat", 20))
    else:
      messages.append(("SDM1", 10))

    # GTW map / mpp speed limits for dashboardSpeedLimit. Frequency 0 so a
    # car without them does not trip canValid (same pattern as cluster RX).
    # Pre-AP has no Mobileye; same chassis UI msgs still carry map/mpp limits.
    if CP.carFingerprint in (CAR.TESLA_AP1_MODELS, CAR.TESLA_PREAP_MODELS):
      messages.append(("UI_driverAssistMapData", 0))
      messages.append(("UI_gpsVehicleSpeed", 0))

    if CP.carFingerprint == CAR.TESLA_AP1_MODELS:
      # EPB EAC allow (0x214) for the EPB revoke fault. Frequency 0, same reason.
      messages.append(("EPB_epasControl", 0))
      # 0x208 outside air temperature for regen_aware.py. Frequency 0 (no canValid effect).
      messages.append(("GTW_ESP1", 0))

    return CANParser(DBC[CP.carFingerprint]['chassis'], messages, CANBUS.chassis)

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    messages = []

    # Pre-AP has no Autopilot ECU — do not require DAS_control (would keep canValid false).
    if CP.carFingerprint != CAR.TESLA_PREAP_MODELS:
      messages.append(("DAS_control", 40))

    if CP.carFingerprint == CAR.TESLA_MODELS_RAVEN:
      messages.append(("EPAS3P_sysStatus", 100))

    if CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      # Comma pedal on cam/chassis bus. Frequency 0: missing pedal does not trip canValid.
      messages.append(("GAS_SENSOR", 0))

    if CP.carFingerprint == CAR.TESLA_AP1_MODELS:
      # Stock cluster frames for the AP1 cluster substitution. Frequency 0:
      # the parser never marks them missing or timed out, so they cannot
      # change canValid. A car without them just gets no cluster frames.
      for addr in CLUSTER_ADDRS:
        messages.append((MSG_NAMES[addr], 0))
      # Stock 0x488 counter for handover continuity (steer_counter.py).
      # Frequency 0: missing stock frames fall back to the old cadence.
      messages.append(("DAS_steeringControl", 0))

    return CANParser(DBC[CP.carFingerprint]['chassis'], messages, CANBUS.autopilot_chassis)
