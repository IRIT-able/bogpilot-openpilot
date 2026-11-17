"""Actuator decision for Tesla CarController.

CarController.update packs exactly the messages this plan names. No CAN
packing happens here. While disengaged, the plan has no steering frame so
stock Mobileye 0x488 can keep flowing (AP1 interceptor). While AP1 cruise is
enabled but path lateral is not, 0x488 is still sent: the measured angle and
control type NONE. That is Tinkla's human-control and non-idle EPAS path,
except EAC_ERROR_HIGH_ANGLE_REQ (6) and a latched EAC_ERROR_HANDS_ON (3).
Those two are not path blocks once hands are below the HSO threshold. While
enabled, hands are below that threshold, and EPAS is not EAC_FAULT, 0x488 is
ANGLE. If EPAS is not yet EAC_ACTIVE, a latched code 6/3 is set, or we are
inside the engage soft-start window, the angle is the measured wheel so EPAS
can accept control. Otherwise the planned angle is used when lat is active.
Always rate limited and clipped to measured +/- 20 deg. DAS_control is planned
only when the car params allow openpilot
longitudinal control and CarControl.enabled and CarControl.longActive are
both true, or (AP1 only) while the driver presses the accelerator during
engagement, when the plan carries the neutral frame (ap1_gas_neutral).
Otherwise the plan has no longitudinal command.
"""

from dataclasses import dataclass

import numpy as np
clip = np.clip
from opendbc.car.tesla_ap1.epas_gate import EAC_BLOCKING_STATUS
from opendbc.car.tesla_ap1.hso import AP1_HANDS_ON_LEVEL
from opendbc.car.tesla_ap1.steer_fault import AP1_LATCHED_ANGLE_ERRORS
from opendbc.car.tesla_ap1.values import CarControllerParams


def apply_std_steer_angle_limits(apply_angle, apply_angle_last, v_ego, LIMITS):
  """0.9.7 openpilot angle rate limiter (speed-interpolated up/down tables), kept verbatim for AP1."""
  steer_up = apply_angle_last * apply_angle >= 0. and abs(apply_angle) > abs(apply_angle_last)
  rate_limits = LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else LIMITS.ANGLE_RATE_LIMIT_DOWN
  angle_rate_lim = np.interp(v_ego, rate_limits.speed_bp, rate_limits.angle_v)
  return float(np.clip(apply_angle, apply_angle_last - angle_rate_lim, apply_angle_last + angle_rate_lim))

# safety_tesla.h tesla_tx_hook: type 0 is NONE and type 3 is DISABLED.
# Neither one sets steer_control_enabled.
STEERING_CONTROL_NONE = 0
STEERING_CONTROL_ANGLE = 1

# 11-bit DAS_control. Chassis is 0x2b9 (tesla_can.dbc). Powertrain is 0x2bf
# (tesla_powertrain.dbc). AP1 plans the chassis id only. tesla_tx_hook uses
# 0x2b9 when TESLA_FLAG_POWERTRAIN is unset.
DAS_CONTROL_CHASSIS = 0x2B9
DAS_CONTROL_POWERTRAIN = 0x2BF

# tesla_can.dbc VAL_ 697 DAS_accState. Tinkla create_ap1_long_control uses
# 4 (ACC_ON) in drive and 3 (ACC_HOLD) only for static cruise.
ACC_HOLD = 3
ACC_ON = 4
ACC_CANCEL_GENERIC_SILENT = 13

# AP1 standstill-engage experiment (toggle AP1StandstillEngage + panda
# TESLA_FLAG_AP1_STANDSTILL). Tinkla tesla_unity_dev LONG_module.py 246-255 /
# 355-367 sends, while disengaged and in drive, DAS_control with accState 4,
# setSpeed 0, accel [-1.4, +1.8], jerk [-0.46, +0.476] "so we can enable at
# 0 km/h". BogPilot sends the same but with accelMax 0 (no positive request
# while not engaged); the panda rejects any other shape while long is not allowed.
AP1_STANDBY_ACCEL_MIN = -1.4
AP1_STANDBY_ACCEL_MAX = 0.0
AP1_STANDBY_JERK_MIN = -0.46
AP1_STANDBY_JERK_MAX = 0.476


def ap1_standby_allowed(standstill_flag, openpilot_longitudinal_control, enabled, in_drive, stock_ap_active):
  """True when AP1 sends the standby DAS_control (experiment; AP1StandstillEngage is default ON on Model S AP1, toggles.py)."""
  return (bool(standstill_flag) and bool(openpilot_longitudinal_control) and not bool(enabled)
          and bool(in_drive) and not bool(stock_ap_active))

# ~300 ms at 100 Hz CarController. Rising edge of enabled holds measured
# ANGLE before the planner angle so EPAS can leave AVAILABLE/INHIBITED.
AP1_ENGAGE_SOFT_START_FRAMES = 30

# tesla_can.dbc VAL_ 880 EPAS_eacStatus.
EAC_ACTIVE = "EAC_ACTIVE"


@dataclass(frozen=True)
class SteerCommand:
  angle_deg: float
  enabled: bool
  counter: int

  @property
  def control_type(self) -> int:
    if self.enabled:
      return STEERING_CONTROL_ANGLE
    return STEERING_CONTROL_NONE


@dataclass(frozen=True)
class LongCommand:
  acc_state: int
  target_speed: float
  min_accel: float
  max_accel: float
  counter: int
  # AP1 standby frame only: fixed jerk fields. None = CarController's jerk limits.
  jerk_min: float | None = None
  jerk_max: float | None = None


@dataclass(frozen=True)
class ActuatorPlan:
  steer: SteerCommand | None
  longitudinal: tuple[LongCommand, ...]
  cancel: bool
  apply_angle_last: float
  # Empty unless a DAS_control command is planned. AP1 is (0x2b9,) only.
  longitudinal_addrs: tuple[int, ...] = ()


def steering_control_type(enabled: bool) -> int:
  if enabled:
    return STEERING_CONTROL_ANGLE
  return STEERING_CONTROL_NONE


def ap1_long_acc_state(camera_acc_state, chassis_das_only):
  """ACC state openpilot may put in chassis DAS_control.

  AP1 openpilot longitudinal is adaptive cruise in drive, not Tinkla static
  cruise. Tinkla selfdrive/car/tesla/teslacan.py create_ap1_long_control sets
  DAS_accState to 4 while in drive, and to 3 only when static_cruise and
  cruise are both on. Copying the camera ACC_HOLD (3) made the car wait for
  the accelerator at a stop. Non-AP1 still passes the camera state through.
  """
  # AP1: stock DAS reports accState 0 below ~18 mph with no lead (stock TACC
  # minimum speed). Forwarding that 0 made the DI drop cruise ~100 ms after
  # every sub-18 mph engage (routes 53/54). Tinkla always sends 4 in drive.
  # Stock DAS also sends 13 (ACC_CANCEL_GENERIC_SILENT) on some low-speed
  # roads (route 57: dirt track / unmarked roads, 16:53:37, 17:03:09, 17:03:23).
  # Forwarding it cancelled DI cruise within ~20 ms with no driver input. The
  # stock DAS is not the cruise owner while openpilot is engaged; driver cancel
  # (stalk, brake, gas) uses its own path. Tinkla always sends 4 in drive.
  if chassis_das_only and camera_acc_state in (0, ACC_HOLD, ACC_CANCEL_GENERIC_SILENT):
    return ACC_ON
  return camera_acc_state


def ap1_should_send_hold_clear(ap1, long_allowed, camera_acc_state, frame):
  """When to TX the all-zero 0x349 clear.

  Tinkla HUD_module sends warning matrix 3 at 1 Hz with DAS_gas_to_resume
  forced to 0. Also send while the camera is still reporting ACC_HOLD so the
  clear is on the bus for that stop, not only on the 1 Hz tick.
  Disengaged longitudinal does not send it.
  """
  if not ap1 or not long_allowed:
    return False
  return camera_acc_state == ACC_HOLD or (frame % 100) == 0


def longitudinal_command_allowed(openpilot_longitudinal_control, enabled, long_active):
  """True only when a DAS_control command may be planned.

  CarControl.enabled must be true for actuator commands. longActive is the
  longitudinal engage bit from controlsd. The openpilot-long car param is
  required as well. Any false input means no DAS_control frame.
  """
  return bool(openpilot_longitudinal_control) and bool(enabled) and bool(long_active)


def ap1_gas_neutral(chassis_das_only, openpilot_longitudinal_control, enabled, gas_pressed):
  """True when AP1 sends the neutral DAS_control frame for a driver gas press.

  safety_tesla.h tesla_tx_hook checks DAS_accelMin and DAS_accelMax with
  longitudinal_accel_checks. While the panda has seen the pedal pressed
  (0x108 DI_pedalPos byte 6 != 0, the same test as carstate gasPressed),
  get_longitudinal_allowed() is false and only inactive_accel (raw 375,
  0.00 m/s^2) passes for both. Any other accel request is dropped.

  controlsd drops longActive for the press, but carcontroller runs with the
  new carState and the previous step's CarControl. On the first pressed step
  that stale longActive=True frame was the one the panda rejected (seen twice
  in one AP1 drive log). Gas wins over longActive here.

  Ported from BogPilot 0.9.7 (bogpilot-tesla 026cfb3
  selfdrive/car/tesla/actuator_plan.py:158 ap1_gas_neutral and :233-234), which
  keeps sending DAS_control while long is inactive with accel 0 and
  DAS_setSpeed = vEgo. Tinkla LONG_module also zeroes target_accel when the pedal is pressed
  (CS.realPedalValue > 0). The driver pedal goes to the DI directly and is not
  touched; releasing it returns to the normal path on the next step.
  """
  return bool(chassis_das_only) and bool(openpilot_longitudinal_control) and bool(enabled) and bool(gas_pressed)


def _ap1_limited_angle(requested_angle_deg, last_angle_deg, measured_angle_deg, v_ego, limits=None):
  """Rate limit, then Tinkla's +/- 20 deg clip around the measured wheel.

  The clip is the "to not fault the EPS" bound. It is applied after the
  speed-based rate limit. A requested angle inside that window is unchanged
  aside from the rate step.

  limits: the CarControllerParams to read the rate tables from. None keeps AP1's
  (CarControllerParams); Pre-AP passes PreApCarControllerParams, whose wind-up table is NAP's
  lateral-jerk envelope. The +/- 20 deg clip is the same for both.
  """
  apply_angle = apply_std_steer_angle_limits(requested_angle_deg, last_angle_deg, v_ego,
                                             limits if limits is not None else CarControllerParams)
  return clip(apply_angle, measured_angle_deg - 20, measured_angle_deg + 20)


def ap1_hold_measured_angle(chassis_das_only, enabled, eac_fault, hands_pause,
                           eac_status, epas_error, soft_start, lat_session=None, epas_blocked=False,
                           tmp_fault_probe=False):
  """True when AP1 must command ANGLE at the measured wheel, not the planner.

  EPAS on route 0000000e stayed EAC_INHIBITED with code 6 while openpilot kept
  sending planner ANGLE; the wheel never moved and there was no UI fault.
  Hold measured until EAC_ACTIVE (and after soft-start / latched 6 or 3).
  Only while openpilot wants lateral (lat_session, CarControl.latActive) and
  EPAS is not refusing (epas_blocked): the hold is an ANGLE frame, and ANGLE is
  never sent for a session that is not active or to an EPAS that is INHIBITED
  or in error. None for lat_session means `enabled`.

  tmp_fault_probe (epas_gate.EpasTmpFaultRecovery): the one exception to the lat_session rule. A latched
  EAC_ERROR_TMP_FAULT keeps latActive false and clears only on a new ANGLE request, so the recovery
  asks for a short measured-wheel hold while engaged with an AVAILABLE EPAS. Hands pause, EAC_FAULT and
  a refusing EPAS still win.
  """
  if tmp_fault_probe and chassis_das_only and enabled and not eac_fault and not hands_pause and not epas_blocked:
    return True
  if lat_session is None:
    lat_session = enabled
  if not chassis_das_only or not enabled or not lat_session or epas_blocked or eac_fault or hands_pause:
    return False
  if soft_start:
    return True
  # None means the caller did not pass status (non-AP1 tests). Only a known
  # non-ACTIVE status forces the measured hold.
  if eac_status is not None and eac_status != EAC_ACTIVE:
    return True
  if epas_error in AP1_LATCHED_ANGLE_ERRORS:
    return True
  return False


def build_actuator_plan(frame, lat_active, hands_on_fault, openpilot_longitudinal_control,
                        enabled, long_active,
                        measured_angle_deg, requested_angle_deg, last_angle_deg, v_ego, accel,
                        acc_state, das_counters, pcm_cancel, chassis_das_only=False,
                        epas_error=None, eac_fault=False, hands_on_level=0,
                        eac_status=None, soft_start=False, driver_yield=False, gas_pressed=False,
                        steer_tick=None, standby=False, steer_session=None, lat_session=None, epas_yield=False,
                        tmp_fault_probe=False, angle_limits=None):
  """Same steering branches CarController.update had before this function existed.

  Disengaged lateral: no steering frame. Sending type-NONE 0x488 while
  disengaged replaced stock Mobileye commands on AP1. AP1 with cruise enabled
  and lat inactive still sends 0x488 at the measured angle with type NONE,
  except when ap1_hold_measured_angle is true (not ACTIVE, latched 6/3, or
  soft-start): then type is ANGLE at the measured wheel. While EPAS is
  EAC_ACTIVE, hands are below 2, and there is no latch/soft-start, lat active
  sends the planned angle. Other non-idle codes with lat inactive still take
  the NONE branch. Hands at or above 2 also take NONE and do not cancel.
  driver_yield (AP1 resume hold, hso.Ap1DriverYield) is treated exactly like
  hands at or above 2. apply_angle_last tracks the commanded angle so the next
  path command starts from the wheel. Longitudinal messages are built only when
  longitudinal_command_allowed is true, or as the neutral frame (accelMin =
  accelMax = 0, set speed = v_ego) when ap1_gas_neutral is true. Otherwise an
  inactive long plan is empty.

  steer_tick (AP1, steer_counter.Ap1SteerCounterSync) replaces the frame % 2
  cadence and (frame // 2) % 16 counter so 0x488 follows the stock DAS phase
  and counter. None keeps the old cadence and counter.
  """

  lkas_enabled = lat_active and not hands_on_fault
  apply_angle_last = last_angle_deg
  # AP1 Always On Lateral: the steering branches below use the lateral session (engaged, or AOL
  # lateral while not engaged) instead of CarControl.enabled, so an AOL driver override keeps
  # 0x488 owned by openpilot (type NONE at the measured wheel) and the resume goes through the
  # same measured hold as engaged. None (default, and always with AOL off) means `enabled`.
  # Longitudinal below still uses `enabled` only.
  steer_enabled = enabled if steer_session is None else bool(steer_session)
  steer = None
  # AP1 only. Hands pause and EAC_FAULT still win over a latched code 6 or 3.
  # Code 3 takes this path only after hands drop; at or above 2, hands_pause
  # keeps type NONE even though code 3 is not itself a temporary fault.
  # An EPAS that is INHIBITED, FAULT or SNA (or the gate in epas_gate.py, which also covers error
  # codes and the clean-state wait) is not following ANGLE: stop commanding it, same NONE-at-measured
  # branch as the hands pause, no held target.
  epas_blocked = bool(chassis_das_only) and (bool(epas_yield) or eac_status in EAC_BLOCKING_STATUS)
  hands_pause = bool(chassis_das_only) and (hands_on_level >= AP1_HANDS_ON_LEVEL or bool(driver_yield) or epas_blocked)
  # lat_session: openpilot wants lateral now (CarControl.latActive). Without it nothing here may be an
  # ANGLE frame, including the measured hold. None (default) keeps `steer_enabled`.
  hold_measured = ap1_hold_measured_angle(
    chassis_das_only, steer_enabled, eac_fault, hands_pause, eac_status, epas_error, soft_start,
    lat_session=lat_session, epas_blocked=epas_blocked, tmp_fault_probe=tmp_fault_probe)
  if chassis_das_only and eac_fault:
    lkas_enabled = False

  if steer_tick is None:
    steer_send, steer_counter = frame % 2 == 0, (frame // 2) % 16
  else:
    steer_send, steer_counter = bool(steer_tick.send), int(steer_tick.counter) % 16

  if steer_send:
    if hold_measured:
      # Measured ANGLE: EPAS inhibit recovery, latched 6/3, a latched code 4 request, or engage soft-start.
      apply_angle = _ap1_limited_angle(measured_angle_deg, last_angle_deg, measured_angle_deg, v_ego, angle_limits)
      apply_angle_last = apply_angle
      steer = SteerCommand(apply_angle, True, steer_counter)
    elif lkas_enabled and not hands_pause:
      # Angular rate limit based on speed, then the EPS clip.
      apply_angle = _ap1_limited_angle(requested_angle_deg, last_angle_deg, measured_angle_deg, v_ego, angle_limits)
      apply_angle_last = apply_angle
      steer = SteerCommand(apply_angle, True, steer_counter)
    elif chassis_das_only and steer_enabled and not hands_on_fault:
      # Cruise stays up. Do not send the planned path angle.
      apply_angle_last = measured_angle_deg
      steer = SteerCommand(measured_angle_deg, False, steer_counter)
    else:
      # Interceptor: do not TX 0x488 while disengaged; stock DAS must pass.
      apply_angle_last = measured_angle_deg

  longitudinal = []
  longitudinal_addrs = ()
  gas_neutral = ap1_gas_neutral(chassis_das_only, openpilot_longitudinal_control, enabled, gas_pressed)
  if gas_neutral or longitudinal_command_allowed(openpilot_longitudinal_control, enabled, long_active):
    # Neutral: no accel request (both limits at 0.00 m/s^2, panda inactive_accel).
    target_accel = 0.0 if gas_neutral else accel
    target_speed = max(v_ego + (target_accel * CarControllerParams.ACCEL_TO_SPEED_MULTIPLIER), 0)
    max_accel = 0 if target_accel < 0 else target_accel
    min_accel = 0 if target_accel > 0 else target_accel

    acc_state = ap1_long_acc_state(acc_state, chassis_das_only)
    while len(das_counters) > 0:
      longitudinal.append(LongCommand(acc_state, target_speed, min_accel, max_accel, das_counters.popleft()))
    if longitudinal:
      if chassis_das_only:
        longitudinal_addrs = (DAS_CONTROL_CHASSIS,)
      else:
        longitudinal_addrs = (DAS_CONTROL_CHASSIS, DAS_CONTROL_POWERTRAIN)
  elif standby and chassis_das_only:
    # AP1 standstill experiment: standby DAS_control on the stock counters.
    while len(das_counters) > 0:
      longitudinal.append(LongCommand(ACC_ON, 0.0, AP1_STANDBY_ACCEL_MIN, AP1_STANDBY_ACCEL_MAX, das_counters.popleft(),
                                      AP1_STANDBY_JERK_MIN, AP1_STANDBY_JERK_MAX))
    if longitudinal:
      longitudinal_addrs = (DAS_CONTROL_CHASSIS,)

  # Cancel on user steering override, since there is no steering torque blending
  if hands_on_fault:
    pcm_cancel = True

  cancel = frame % 10 == 0 and pcm_cancel
  return ActuatorPlan(steer, tuple(longitudinal), bool(cancel), apply_angle_last, longitudinal_addrs)
