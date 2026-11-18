"""Tesla Autopilot 1 (HW1, Mobileye) Model S and Model X.

Legacy chassis-bus port (BogPilot). Separate brand from opendbc.car.tesla so
the Model 3/Y port stays untouched. Panda safety: SafetyModel.tesla with the AP1 flag.
Model X is lateral-only. AP2 is not a platform in this tree.
"""
from collections import namedtuple
from dataclasses import dataclass, field
from enum import IntFlag, StrEnum

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig
from opendbc.car.structs import CarParams, CarState
from opendbc.car.tesla.values import TeslaSafetyFlags

Ecu = CarParams.Ecu
ButtonType = CarState.ButtonEvent.Type

Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

# 0.9.7 style angle rate limit table: (speed_bp, angle_v), deg per 50 Hz frame.
AngleRateLimit = namedtuple('AngleRateLimit', ['speed_bp', 'angle_v'])


@dataclass
class TeslaAp1PlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.chassis: 'tesla_ap1_can', Bus.pt: 'tesla_powertrain',
                                                     Bus.radar: 'tesla_radar_bosch_generated'})


class CAR(Platforms):
  TESLA_AP1_MODELS = TeslaAp1PlatformConfig(
    [CarDocs("Tesla Model S (Autopilot 1) 2014-16", "Autopilot", support_type=SupportType.COMMUNITY, support_link="#community")],
    CarSpecs(wheelbase=2.959, steerRatio=15.0),
  )
  # Pre-AP Model S (no Autopilot ECU, no Mobileye). Same chassis DBC and steering envelope as the AP1 Model S.
  # Selected by CarModel / ForceFingerprint, or by detect_preap_candidate (Comma Pedal + no Mobileye on bus 2).
  # Lateral through the Pre-AP safety flag; longitudinal only through the Comma Pedal (see preap/).
  TESLA_PREAP_MODELS = TeslaAp1PlatformConfig(
    [CarDocs("Tesla Model S (Pre-Autopilot) 2012-14", "Pre-Autopilot", support_type=SupportType.COMMUNITY, support_link="#community")],
    CarSpecs(wheelbase=2.959, steerRatio=15.0),
  )
  # Tinkla mass is 2560 + 136 kg. CarSpecs in 0.11.2 has no mass field.
  # centerToFront is 0.5 * wheelbase via the CarSpecs default centerToFrontRatio.
  TESLA_AP1_MODELX = TeslaAp1PlatformConfig(
    [CarDocs("Tesla Model X (Autopilot 1) 2016", "Autopilot", support_type=SupportType.COMMUNITY, support_link="#community")],
    CarSpecs(wheelbase=2.964, steerRatio=15.0),
  )


class TeslaPlatform(StrEnum):
  """Early-Tesla platforms. Recognition only; actuation is a later layer."""
  preap = "preap"
  ap1_s = "ap1_s"
  ap1_x = "ap1_x"
  ap2 = "ap2"


# AP1 is selected by its chassis CAN signature (match_ap1_chassis), not FW.
# No AP1-specific UDS requests: the upstream tesla queries already log these
# ECUs, and duplicate requests would only lengthen fingerprinting for all cars.
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],
  fw_version_regex=rb'.*',
)


class CANBUS:
  # Lateral harness
  chassis = 0
  radar = 1
  autopilot_chassis = 2

  # Longitudinal harness (second panda; not used by AP1)
  powertrain = 4
  private = 5
  autopilot_powertrain = 6


GEAR_MAP = {
  "DI_GEAR_INVALID": CarState.GearShifter.unknown,
  "DI_GEAR_P": CarState.GearShifter.park,
  "DI_GEAR_R": CarState.GearShifter.reverse,
  "DI_GEAR_N": CarState.GearShifter.neutral,
  "DI_GEAR_D": CarState.GearShifter.drive,
  "DI_GEAR_SNA": CarState.GearShifter.unknown,
}

DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]

# Make sure the message and addr is also in the CAN parser!
BUTTONS = [
  Button(ButtonType.leftBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [1]),
  Button(ButtonType.rightBlinker, "STW_ACTN_RQ", "TurnIndLvr_Stat", [2]),
  Button(ButtonType.accelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [4, 16]),
  Button(ButtonType.decelCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [8, 32]),
  Button(ButtonType.cancel, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [1]),
  Button(ButtonType.resumeCruise, "STW_ACTN_RQ", "SpdCtrlLvr_Stat", [2]),
]


class CarControllerParams:
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 1.6, .3])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[10., 7.0, 0.8])
  JERK_LIMIT_MAX = 8
  JERK_LIMIT_MIN = -8
  ACCEL_TO_SPEED_MULTIPLIER = 3

  def __init__(self, CP):
    pass


class PreApCarControllerParams(CarControllerParams):
  """Pre-AP wind-up rate: NAP's lateral-jerk envelope, written out in this port's table form.

  NAP limits the angle step by a lateral-jerk budget rather than a speed table
  (opendbc lateral.py get_max_angle_delta_vm / apply_steer_angle_limits_vm):

      max_curvature_rate = MAX_LATERAL_JERK / v**2            # (1/m)/s, MAX_LATERAL_JERK ~3.59
      max_angle_delta    = deg(VM.get_steer_from_curvature(max_curvature_rate, v, 0)) * STEER_STEP * DT_CTRL
      max_angle_delta    = min(max_angle_delta, MAX_ANGLE_RATE)

  MAX_ANGLE_RATE = 5 deg per command is only the low-speed cap ("prevent a fault and for low speed comfort"),
  not the limit that binds while driving: the jerk budget falls as 1/v**2 and is below the cap above ~5.5 m/s.
  The values here are that expression evaluated at each breakpoint with NAP's own Tesla vehicle model
  (slip_factor -0.000580374383851451, steer_ratio 12.0, wheelbase 2.89 -- tesla.h TESLA_STEERING_PARAMS,
  "based off TESLA_MODEL_Y to match openpilot") and a 50 Hz command, which is Pre-AP's 0x488 rate.

  Against the table this replaces, per command: 1.60 -> 5.00 at 5 m/s, 1.34 -> 2.99 at 7, 0.95 -> 1.51 at 10,
  and 0.30 -> 0.74 at 15 m/s (34 mph), where the old table had already bottomed out. A drive on this build sat at
  a median 15.0 m/s and its measured wheel movement passed the old limit in 1.8% of engaged command windows,
  with a p99 demand of 0.80 deg per command against that 0.30 ceiling: the wind-up limit, not the car, was what
  ran out in the sharpest part of a curve.

  Two deliberate departures from NAP, both to avoid taking away authority the port has today:
    - below ~3 m/s NAP's cap (5.0) is under the old table (10.0 at a standstill), and 5.0 is used. The old value
      also sat above what this port's own panda allows there (8.0 per command, tesla_preap.h), so no reachable
      command is lost.
    - at 32 m/s the old 0.30 is kept where NAP's budget would give 0.24.
  Everything between is NAP's number. Linear interpolation of a convex curve overshoots between breakpoints;
  the breakpoints are placed so the worst overshoot against the continuous NAP budget is under 0.15 deg per
  command, and the whole table stays inside this port's panda ceiling (angle_rate_up_lookup, min headroom
  0.60 deg per command) so nothing here can raise a safety violation.

  Unwinding is unchanged: ANGLE_RATE_LIMIT_DOWN is already at or above NAP's budget at every speed
  (0.80 vs 0.72 at 15 m/s, 5.76 vs 2.99 at 7), so matching NAP there would only slow a return to centre.
  AP1 keeps CarControllerParams: its 0x488 goes out at ~20 Hz on the stock DAS phase (steer_counter.py), so a
  per-command table means a different deg/s there, and there is no NAP AP1 reference to match it to.
  """
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 6., 7., 8.5, 10., 13., 16., 20., 25., 32.],
                                       angle_v=[5., 5., 4.04, 2.99, 2.06, 1.51, .93, .64, .44, .31, .3])


class TeslaAp1SafetyFlags(IntFlag):
  # opendbc/safety/modes/tesla_ap1.h
  LONG_CONTROL = TeslaSafetyFlags.LONG_CONTROL.value
  AP1 = 8
  STANDSTILL = 16
  # Pre-AP (opendbc/safety/modes/tesla_preap.h). PREAP selects the Pre-AP hooks; they never run with AP1 (8) set.
  PREAP = 32
  PREAP_PEDAL = 64
  # Bosch radar GTW on Pre-AP only. Matches NAP TeslaSafetyFlags on SafetyModel.tesla
  # (opendbc cc39c7fa values.py FLAG_RADAR_BEHIND_NOSECONE / FLAG_RADAR_EMULATION).
  PREAP_RADAR_BEHIND_NOSECONE = 128
  PREAP_RADAR_EMULATION = 256


# Chassis-only recognition is insufficient: Pre-AP shares these messages.
# Require stock Mobileye traffic on bus 2 as well. This fallback is considered
# only after upstream selection failed and cannot replace a stock Model 3/Y match.
AP1_CHASSIS_SIGNATURE = {0x45: 8, 0x118: 6, 0x370: 8, 0x20A: 8}


class ForcedFingerprint(dict):
  """Live CAN fingerprint of a car whose model was fixed by the user (CarModel with ForceFingerprint, or FINGERPRINT).
  A fixed model is taken as given and never re-checked against the live bus, as in 0.9.7."""


def match_ap1_chassis(fingerprint) -> bool:
  """AP1 chassis AND stock Mobileye bus-2 signature (0x2B9, 0x488). Pre-AP chassis alone is insufficient.
  0x399 (about 2 Hz) is not required: it can arrive after the fingerprint window closes."""
  bus0 = fingerprint.get(CANBUS.chassis, {}) if isinstance(fingerprint, dict) else {}
  camera = fingerprint.get(CANBUS.autopilot_chassis, {}) if isinstance(fingerprint, dict) else {}
  mobileye = {0x2B9: 8, 0x488: 4}
  return (all(bus0.get(addr) == length for addr, length in AP1_CHASSIS_SIGNATURE.items()) and
          all(camera.get(addr) == length for addr, length in mobileye.items()))


def is_preap(candidate) -> bool:
  """True for the Pre-AP Model S platform. Accepts a CAR member or its name."""
  if candidate is None or isinstance(candidate, bool):
    return False
  try:
    return str(candidate) == str(CAR.TESLA_PREAP_MODELS)
  except Exception:
    return False


_AP1_FAMILY = frozenset((str(CAR.TESLA_AP1_MODELS), str(CAR.TESLA_AP1_MODELX)))


def is_ap1_family(candidate) -> bool:
  """True for Model S AP1 and Model X AP1. Accepts a CAR member or its name."""
  if candidate is None or isinstance(candidate, bool):
    return False
  try:
    return str(candidate) in _AP1_FAMILY
  except Exception:
    return False


# AP1 Bosch radarTracks publish at the radar's own cycle, measured 7.4-12.6 Hz
# depending on track count (about 12 Hz below 10 tracks, 7.4-7.8 Hz at 28-30
# tracks). The slowest 20-sample FrequencyTracker average measured on 14 days of
# AP1 drives was 6.996 Hz, about 0.31 Hz above 6.69 Hz. FrequencyTracker stays
# valid while either its 200-sample or its 20-sample average is in range, and the
# higher of the two, at its worst while engaged, leaves about 1.25 Hz. The stock
# FrequencyTracker floor is 8.0 Hz (0.8 x half of the 20 Hz service). radard
# applies this lower-only override on AP1 Model S and Model X.
AP1_RADARTRACKS_MIN_FREQ_HZ = 6.69


DBC = CAR.create_dbc_map()
