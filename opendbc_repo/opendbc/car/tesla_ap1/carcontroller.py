from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla_ap1.actuator_plan import (
  ap1_standby_allowed,
  AP1_ENGAGE_SOFT_START_FRAMES,
  DAS_CONTROL_POWERTRAIN,
  ap1_gas_neutral,
  ap1_hold_measured_angle,
  ap1_should_send_hold_clear,
  build_actuator_plan,
  longitudinal_command_allowed,
)
from opendbc.car.tesla_ap1.cluster import (
  CLUSTER_BUS, ClusterController, HudInputs, path_from_model_v2,
)
from opendbc.car.tesla_ap1.engage_ramp import Ap1EngageSteerRamp
from opendbc.car.tesla_ap1.epas_gate import EAC_BLOCKING_STATUS, ERR4_EPB_FRESH_FRAMES, EpasGate, EpasTmpFaultRecovery
from opendbc.car.tesla_ap1.ic_lanes import IcLanes
from opendbc.car.tesla_ap1.hso import AP1_HANDS_ON_LEVEL, Ap1DriverYield, ap1_lat_active, ap1_steering_pressed
from opendbc.car.tesla_ap1.override import DriverOverrideDetector, read_sensitivity_level
from opendbc.car.tesla_ap1.long_smooth import Ap1AccelSmoother, Ap1JerkLimit, ap1_brake_urgent, ap1_fcw
from opendbc.car.tesla_ap1.steer_counter import Ap1SteerCounterSync
from opendbc.car.tesla_ap1.teslacan import TeslaCAN
from opendbc.car.tesla_ap1.slc_raise import cruise_set_mph
from opendbc.car.tesla_ap1.toggles import enable_ic_integration
from opendbc.car.tesla_ap1.values import DBC, CANBUS, CAR, TeslaAp1SafetyFlags, is_ap1_family, PreApCarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert
GearShifter = structs.CarState.GearShifter


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.chassis])
    self.pt_packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self.tesla_can = TeslaCAN(self.packer, self.pt_packer)
    # Model S only. Dash cluster substitution is not used on Model X.
    self.cluster = ClusterController() if CP.carFingerprint == CAR.TESLA_AP1_MODELS else None
    # modelV2 path for the cluster lanes. opendbc cannot subscribe to messaging,
    # so openpilot's card sets this to a callable returning the latest modelV2
    # (or None). Unset or failing: build_das_lanes falls back to actuator
    # curvature + 50 m, exactly like the 0.9.7 messaging-unavailable path.
    self.model_v2_provider = None
    # Model S only. ICLanes (ic_lanes.py): engaged 0x239 path from the near lane
    # lines in cluster units. Default on; /data/params_bogpilot/ICLanes = 0/false/off
    # restores the shipped mapping. None after any failure (shipped mapping again).
    self.ic_lanes = IcLanes(CP.carFingerprint) if CP.carFingerprint == CAR.TESLA_AP1_MODELS else None
    # AP1 engage soft-start: hold measured ANGLE for ~300 ms after enable.
    self.ap1_prev_enabled = False
    self.ap1_engage_frame = None
    # AP1 hands pause + resume hold (hso.Ap1DriverYield). Read by the
    # interface for the steerOverride event so the border stays grey.
    self.ap1_yield = Ap1DriverYield()
    # Independent override detector (override.py) and the angle of the last ANGLE
    # frame sent (None when the last frame was not an active command).
    self.ap1_override = DriverOverrideDetector(read_sensitivity_level())   # Override Sensitivity, read once per drive
    self.ap1_cmd_deg = None
    self.ap1_epas_gate = EpasGate()
    # Latched EAC_ERROR_TMP_FAULT: bounded measured-wheel ANGLE request while engaged (epas_gate.py).
    self.ap1_tmp_fault = EpasTmpFaultRecovery()
    self.ap1_prev_lat_session = False
    # Structural AP1 (Model S and Model X): engage blend from the measured wheel.
    self.ap1_steer_ramp = Ap1EngageSteerRamp() if is_ap1_family(CP.carFingerprint) else None
    # Structural AP1: 0x488 phase and counter follow the stock DAS (steer_counter.py).
    self.ap1_steer_sync = Ap1SteerCounterSync() if is_ap1_family(CP.carFingerprint) else None
    # Model S only. Accel slew is longitudinal tuning and stays off on Model X.
    self.ap1_accel_smoother = Ap1AccelSmoother() if CP.carFingerprint == CAR.TESLA_AP1_MODELS else None
    # Model S only. Jerk limits are longitudinal tuning and stay off on Model X.
    self.ap1_jerk_limit = Ap1JerkLimit() if CP.carFingerprint == CAR.TESLA_AP1_MODELS else None
    # Model S only. Standstill experiment follows the Model S safety param.
    configs = list(getattr(CP, "safetyConfigs", []) or [])
    self.ap1_standstill = (CP.carFingerprint == CAR.TESLA_AP1_MODELS and len(configs) > 0 and
                           bool(int(getattr(configs[0], "safetyParam", 0)) & TeslaAp1SafetyFlags.STANDSTILL))

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators

    # Structural AP1 (Model S and Model X): hands pause, no hands-on cancel,
    # chassis DAS only, and the lateral session. Longitudinal messages still
    # follow openpilotLongitudinalControl, which stays false on Model X.
    ap1 = is_ap1_family(self.CP.carFingerprint)
    hands_on_fault = (not ap1) and CS.steer_warning == "EAC_ERROR_HANDS_ON" and CS.hands_on_level >= 3
    # AP1 longitudinal is chassis 0x2b9. Do not also plan powertrain 0x2bf.
    chassis_das_only = ap1
    soft_start = False
    driver_yield = False
    epas_yield = False
    tmp_fault_probe = False
    lat_session = None
    # AP1 lateral session: engaged, or Always On Lateral steering while not engaged. controlsd only
    # sets latActive without enabled when bogpilotCarState.alwaysOnLateralEnabled is true, so with
    # AOL off steer_session == CC.enabled and nothing below changes.
    steer_session = bool(CC.enabled) or (ap1 and bool(CC.latActive))
    if ap1:
      enabled = steer_session
      # Hands >= 2 starts it; level 0 for AP1_RESUME_HOLD_S ends it. Only
      # during a lateral session, so it never holds off disengage or a new engage.
      override_signal = self.ap1_override.update(
        enabled, CS.hands_on_level, getattr(CS.out, "steeringTorque", 0.0), getattr(CS.out, "steeringRateDeg", 0.0),
        CS.out.steeringAngleDeg, self.ap1_cmd_deg)
      driver_yield = self.ap1_yield.update(enabled, CS.hands_on_level, override_signal, self.ap1_override.quiet)
      if enabled and not self.ap1_prev_enabled:
        self.ap1_engage_frame = self.frame
      # EPAS refusal gate (epas_gate.py): INHIBITED, FAULT, SNA or a blocking error code stops commanding on
      # this frame. Both the end of that block and a new latActive start the measured-angle soft-start.
      lat_session = bool(CC.latActive)
      epas_yield = self.ap1_epas_gate.update(lat_session, CS.eac_status, CS.steer_warning, bool(CS.eac_fault))
      # Latched code 4 recovery: every rule is in epas_gate.EpasTmpFaultRecovery. Engaged only (CC.enabled,
      # not Always On Lateral). Like the panda it needs a recent 0x214: missing or stale is passed as None.
      was_probe, probe_n = self.ap1_tmp_fault.active, self.ap1_tmp_fault.probes
      epb_age = getattr(CS, "epb_eac_allow_age", None)
      epb_fresh = epb_age is not None and epb_age <= ERR4_EPB_FRESH_FRAMES
      tmp_fault_probe = self.ap1_tmp_fault.update(bool(CC.enabled), lat_session, CS.eac_status, CS.steer_warning,
                                                  CS.hands_on_level, getattr(CS, "epb_eac_allow", None) if epb_fresh else None,
                                                  bool(CS.eac_fault), driver_yield, v_ego=float(CS.out.vEgo))
      if tmp_fault_probe and not was_probe:
        carlog.warning(f"tesla ap1 epas code 4: measured-angle request {self.ap1_tmp_fault.probes}")
      elif was_probe and not tmp_fault_probe:
        carlog.warning(f"tesla ap1 epas code 4: request {probe_n} ended {self.ap1_tmp_fault.last_result}")
      if self.ap1_yield.resumed or self.ap1_epas_gate.resumed or (lat_session and not self.ap1_prev_lat_session):
        # Resume from the measured wheel through the engage soft-start.
        self.ap1_engage_frame = self.frame
      self.ap1_prev_lat_session = lat_session
      if not enabled:
        self.ap1_engage_frame = None
      self.ap1_prev_enabled = enabled
      if self.ap1_engage_frame is not None:
        soft_start = (self.frame - self.ap1_engage_frame) < AP1_ENGAGE_SOFT_START_FRAMES
    if ap1:
      lat_active = ap1_lat_active(CC.latActive, CS.hands_on_level) and not driver_yield
    else:
      lat_active = CC.latActive
    requested_angle = actuators.steeringAngleDeg
    if self.ap1_steer_ramp is not None:
      # Planner path this step: same gate as build_actuator_plan's lkas branch
      # (hands pause / yield, EAC fault and soft-start hold take other branches).
      epas_blocked = epas_yield or CS.eac_status in EAC_BLOCKING_STATUS
      hands_pause = CS.hands_on_level >= AP1_HANDS_ON_LEVEL or driver_yield or epas_blocked
      held = ap1_hold_measured_angle(True, steer_session, bool(CS.eac_fault), hands_pause,
                                     CS.eac_status, CS.steer_warning, soft_start,
                                     lat_session=lat_session, epas_blocked=epas_blocked,
                                     tmp_fault_probe=tmp_fault_probe)
      path_active = (steer_session and lat_active and not hands_on_fault and not hands_pause and
                     not bool(CS.eac_fault) and not held)
      requested_angle = self.ap1_steer_ramp.update(path_active, CS.out.steeringAngleDeg, actuators.steeringAngleDeg)
    accel = actuators.accel
    jerk_limits = {}
    if self.ap1_accel_smoother is not None:
      gas_pressed = bool(getattr(CS.out, "gasPressed", False))
      long_allowed_now = longitudinal_command_allowed(self.CP.openpilotLongitudinalControl, CC.enabled, CC.longActive)
      gas_neutral = ap1_gas_neutral(chassis_das_only, self.CP.openpilotLongitudinalControl, CC.enabled, gas_pressed)
      # FCW or the LongControl stopping state: braking passes through unramped.
      urgent = ap1_brake_urgent(CC)
      fcw = ap1_fcw(CC)
      accel = self.ap1_accel_smoother.update(actuators.accel, long_allowed_now, CS.out.vEgo, gas_neutral=gas_neutral,
                                             urgent=urgent, fcw=fcw,
                                             a_ego=getattr(CS.out, "aEgo", None), gas_pressed=gas_pressed)
      jerk_min, jerk_max = self.ap1_jerk_limit.update(accel, long_allowed_now, CS.out.vEgo, gas_neutral=gas_neutral,
                                                      urgent=urgent, fcw=fcw)
      jerk_limits = {"jerk_min": jerk_min, "jerk_max": jerk_max}
    steer_tick = None
    if self.ap1_steer_sync is not None:
      steer_tick = self.ap1_steer_sync.update(getattr(CS, "stock_steer_counters", ()), self.frame)
    plan = build_actuator_plan(
      self.frame,
      lat_active,
      hands_on_fault,
      self.CP.openpilotLongitudinalControl,
      CC.enabled,
      CC.longActive,
      CS.out.steeringAngleDeg,
      requested_angle,
      self.apply_angle_last,
      CS.out.vEgo,
      accel,
      CS.acc_state,
      CS.das_control_counters,
      CC.cruiseControl.cancel,
      chassis_das_only=chassis_das_only,
      # Code 6 is AP1-only. Model 3/Y still treats it as a temporary fault
      # and does not take the measured-angle ANGLE fallback.
      epas_error=CS.steer_warning if ap1 else None,
      eac_fault=bool(CS.eac_fault) if ap1 else False,
      hands_on_level=CS.hands_on_level if ap1 else 0,
      eac_status=CS.eac_status if ap1 else None,
      soft_start=soft_start,
      driver_yield=driver_yield,
      epas_yield=epas_yield,
      tmp_fault_probe=tmp_fault_probe,
      # Pre-AP winds up on NAP's lateral-jerk envelope (values.PreApCarControllerParams); AP1 keeps its own table.
      angle_limits=None if ap1 else PreApCarControllerParams,
      lat_session=lat_session if ap1 else None,
      # AP1: neutral DAS_control while the driver presses the accelerator.
      gas_pressed=bool(getattr(CS.out, "gasPressed", False)) if ap1 else False,
      steer_tick=steer_tick,
      steer_session=steer_session if ap1 else None,
      standby=ap1_standby_allowed(self.ap1_standstill, self.CP.openpilotLongitudinalControl, CC.enabled,
                                  CS.out.gearShifter == GearShifter.drive,
                                  bool(getattr(CS, "stock_ap_active", False))) if (ap1 and self.ap1_standstill) else False,
    )
    self.apply_angle_last = plan.apply_angle_last
    if plan.steer is not None:
      self.ap1_cmd_deg = plan.steer.angle_deg if plan.steer.enabled else None
    elif not steer_session:
      self.ap1_cmd_deg = None
    if plan.steer is not None and self.ap1_steer_sync is not None:
      self.ap1_steer_sync.commit(plan.steer.counter, self.frame)

    can_sends = []

    if plan.steer is not None:
      can_sends.append(self.tesla_can.create_steering_control(plan.steer.angle_deg, plan.steer.enabled, plan.steer.counter))

    # Longitudinal control (in sync with stock message, about 40Hz)
    chassis_only = DAS_CONTROL_POWERTRAIN not in plan.longitudinal_addrs
    for cmd in plan.longitudinal:
      jl = jerk_limits if cmd.jerk_min is None else {"jerk_min": cmd.jerk_min, "jerk_max": cmd.jerk_max}
      can_sends.extend(self.tesla_can.create_longitudinal_commands(cmd.acc_state, cmd.target_speed, cmd.min_accel, cmd.max_accel,
                                                                   cmd.counter, chassis_only=chassis_only, **jl))

    # Cancel on user steering override, since there is no steering torque blending
    if plan.cancel:
      # Spam every possible counter value, otherwise it might not be accepted
      for counter in range(16):
        can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, True, CANBUS.chassis, counter))
        can_sends.append(self.tesla_can.create_action_request(CS.msg_stw_actn_req, True, CANBUS.autopilot_chassis, counter))

    # AP1 Hold clear. Tinkla HUD_module zeros DAS_gas_to_resume (bit 1 of
    # 0x349 byte 0) and sends the frame. Not sent unless longitudinal is allowed.
    long_allowed = longitudinal_command_allowed(
      self.CP.openpilotLongitudinalControl, CC.enabled, CC.longActive)
    if ap1_should_send_hold_clear(chassis_das_only, long_allowed, CS.acc_state, self.frame):
      can_sends.append(self.tesla_can.create_ap1_hold_clear())

    # AP1 cluster frames. Display only. Added after every actuator frame and
    # never read by the actuator plan above.
    if self.cluster is not None:
      can_sends.extend(self.cluster_frames(CC, CS, now_nanos))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)
    if self.ap1_accel_smoother is not None:
      new_actuators.accel = float(accel)

    self.frame += 1
    return new_actuators, can_sends

  def _model_path_for_cluster(self, enabled, ic_on):
    """modelV2 path only while the toggle is on and engaged.

    Stale/absent modelV2 returns None so build_das_lanes falls back to
    actuator curvature + 50 m. Substitution itself is not stopped.
    """
    if not ic_on or not enabled or self.model_v2_provider is None:
      return None
    try:
      model = self.model_v2_provider()
      return None if model is None else path_from_model_v2(model)
    except Exception:
      # A bad model message: fallback, and stop asking.
      self.model_v2_provider = None
      return None

  def _ic_lanes_poly(self, enabled, ic_on, curvature):
    """Lane polynomial while engaged and ICLanes is on, else None.

    modelV2 comes from the same provider as the shipped path. A failure turns
    ICLanes off for the drive; the shipped mapping then goes on. Display only.
    """
    if self.ic_lanes is None:
      return None
    try:
      self.ic_lanes.poll()
      if not (ic_on and enabled and self.ic_lanes.on):
        return None
      model = self.model_v2_provider() if self.model_v2_provider is not None else None
      return self.ic_lanes.poly(model, curvature)[0]
    except Exception:
      carlog.exception("tesla ap1 ic lanes disabled")
      self.ic_lanes = None
      return None

  def cluster_frames(self, CC, CS, now_nanos):
    """Cluster frames for this step. A failure here turns them off for the
    rest of the drive (stock frames then flow through the panda). It must not
    stop the steering and longitudinal frames already built."""
    try:
      ic_on = enable_ic_integration(self.CP.carFingerprint)
      enabled = bool(CC.enabled)
      model_path = self._model_path_for_cluster(enabled, ic_on)
      lane_poly = self._ic_lanes_poly(enabled, ic_on, float(CC.actuators.curvature))
      hud = CC.hudControl
      h = HudInputs(
        enabled=enabled,
        fcw=hud.visualAlert == VisualAlert.fcw,
        steer_required=hud.visualAlert == VisualAlert.steerRequired,
        audible=hud.audibleAlert != AudibleAlert.none,
        human_steering=ap1_steering_pressed(CS.hands_on_level),
        left_lane_depart=bool(hud.leftLaneDepart),
        right_lane_depart=bool(hud.rightLaneDepart),
        left_blinker=bool(CC.leftBlinker),
        right_blinker=bool(CC.rightBlinker),
        curvature=float(CC.actuators.curvature),
        ic_integration=ic_on,
        model_path=model_path,
        lane_poly=lane_poly,
        # hudControl.setSpeed is the OP cruise target (m/s); may already include
        # the AP1 SLC raise via controlsd's cluster_display_kph lift. Packed
        # with DBC factor 0.4 so IC dig matches (not 2× from legacy 0.2).
        cruise_set_mph=cruise_set_mph(float(hud.setSpeed)) if enabled else None,
      )
      frames = self.cluster.update(h, CS.cluster_stock, now_nanos)
      return [CanData(addr, dat, CLUSTER_BUS) for addr, dat in frames]
    except Exception:
      carlog.exception("tesla ap1 cluster frames disabled")
      self.cluster = None
      return []
