"""Pre-AP CarController — NotAutopilot-inspired, adapted to BogPilot 0.9.7.

Lateral: DAS_steeringControl via existing TeslaCAN (same angle API as AP1 packer).
Longitudinal: NAP-style VirtualDAS + PedalAuthority → Comma Pedal 0x551 when
user wants long AND pedal detected AND calibrated.
Radar: optional donor-VIN / GTW helper TX when TeslaRadarEnabled (safety flag).

Credit: NAP-style. User-facing product name: BogPilot.
"""
from __future__ import annotations

import time
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.tesla.preap.calib import (
  is_pedal_calibrated,
  pedal_can_bus,
  pedal_long_allowed,
)
from openpilot.selfdrive.car.tesla.preap.long_controller import PreAPLongController
from openpilot.selfdrive.car.tesla.preap.params import PreApParamKeys, get_bool, get_str, set_bool, set_str
from openpilot.selfdrive.car.tesla.preap.pedal_can import PedalCommandCounter
from openpilot.selfdrive.car.tesla.preap.radar_donor_vin import (
  RadarDonorVinCommissioner,
  drain_rx_packets,
  normalize_radar_donor_vin,
)
from openpilot.selfdrive.car.tesla.preap.radar_tx import radar_tx_frames
from openpilot.selfdrive.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
from openpilot.selfdrive.car.tesla.teslacan import TeslaCAN
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.tesla.values import CANBUS, CarControllerParams, DBC


class PreApCarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)
    self.pt_packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.tesla_can = TeslaCAN(self.packer, self.pt_packer)
    self.pedal_counter = PedalCommandCounter()
    self.pedal_bus = pedal_can_bus()
    self.stock_cc = StockCCSpoofer()
    self.long_ctrl = PreAPLongController(self.pedal_counter, pedal_can_bus)

    def _store_donor_vin(vin: str) -> None:
      # Never log the VIN value.
      set_str(PreApParamKeys.RADAR_DONOR_VIN, vin)
    self._vin_commissioner = RadarDonorVinCommissioner(_store_donor_vin, now=time.monotonic)
    self._read_vin_was_on = False

  def _update_detection_and_calib(self, CS) -> bool:
    """Mirror PedalDetected."""
    detected = bool(getattr(CS, "preap_pedal_detected", False))
    set_bool(PreApParamKeys.PEDAL_DETECTED, detected)

    # Calibration lives in preap/calibrate_pedal.py; a UI arm no longer captures anything.
    if get_bool(PreApParamKeys.PEDAL_CALIB_ARMED):
      set_bool(PreApParamKeys.PEDAL_CALIB_ARMED, False)

    return detected

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators
    can_sends = []
    self.pedal_bus = pedal_can_bus()
    detected = self._update_detection_and_calib(CS)
    calibrated = is_pedal_calibrated()

    # NAP: lat_active = CC.latActive and hands < 3 (no CC.enabled).
    hands = int(getattr(CS, 'hands_on_level', 0) or 0)
    lat_active = bool(CC.latActive) and hands < 3
    if lat_active:
      angle = apply_std_steer_angle_limits(
        float(actuators.steeringAngleDeg), self.apply_angle_last, CS.out.vEgo, CarControllerParams)
    else:
      angle = float(CS.out.steeringAngleDeg)
    if self.frame % 2 == 0:
      cntr = (self.frame // 2) % 16
      can_sends.append(self.tesla_can.create_steering_control(angle, lat_active, cntr))
      can_sends.append(self.tesla_can.create_epas_control(cntr, 1))
      self.apply_angle_last = angle

    user_wants_long = bool(self.CP.openpilotLongitudinalControl)
    long_ok = pedal_long_allowed(user_wants_long=user_wants_long,
                                 detected=detected, calibrated=calibrated)

    # NAP-style VirtualDAS + PedalAuthority (50 Hz). Sets stock-CC cancel flags.
    if user_wants_long:
      can_sends.extend(self.long_ctrl.update(
        CC, CS, self.frame, long_ok=long_ok, now_nanos=int(now_nanos or 0)))
      if getattr(CS, "preap_cc_cancel_needed", False):
        CS.preap_last_cc_spoof_ms = int(time.monotonic() * 1000)

    can_sends.extend(self.stock_cc.update(CS, self.frame, self.tesla_can, CANBUS.chassis))

    # Phase A donor VIN F190 read — offroad / pre-engage only (from radarvin).
    radar_on = get_bool(PreApParamKeys.RADAR_ENABLED)
    read_vin_on = get_bool(PreApParamKeys.RADAR_READ_VIN, False)
    if radar_on and read_vin_on:
      force_read = read_vin_on and not self._read_vin_was_on
      self._read_vin_was_on = True
      stored = normalize_radar_donor_vin(get_str(PreApParamKeys.RADAR_DONOR_VIN, ""))
      engaged = bool(CC.enabled) or bool(getattr(CS.out, "cruiseState", None) and CS.out.cruiseState.enabled)
      can_sends.extend(self._vin_commissioner.update(
        drain_rx_packets(),
        radar_enabled=True,
        stored_vin=stored,
        engaged=engaged,
        force_read=force_read,
      ))
    else:
      self._read_vin_was_on = False

    # Radar helper TX (donor VIN when stored, else dummy) — low rate.
    if radar_on and (self.frame % 100 == 0):
      can_sends.extend(radar_tx_frames(enabled=True, bus=CANBUS.radar))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
