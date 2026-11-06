#!/usr/bin/env python3
import time

from cereal import car
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.tesla.values import CAR, DBC, CANBUS
from openpilot.selfdrive.car.tesla.preap.params import PreApParamKeys, get_bool, get_float
from openpilot.selfdrive.car.tesla.preap.radar_filter import FrozenTrackFilter
from openpilot.selfdrive.car.tesla.preap.radar_donor_vin import feed_can_strings as _feed_radar_vin_can
from openpilot.selfdrive.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    self.updated_messages = set()
    self.track_id = 0
    self._preap_radar_offset_m = 0.0
    self._preap_frozen = FrozenTrackFilter()
    self._preap_radar_upside_down = False
    self.rcp = None
    self.num_points = 0
    self.trigger_msg = 0

    # Pre-AP default: radar off — skip Bosch parser (radarUnavailable).
    if CP.carFingerprint == CAR.TESLA_PREAP_MODELS and not get_bool(PreApParamKeys.RADAR_ENABLED, False):
      return

    if CP.carFingerprint == CAR.TESLA_MODELS_RAVEN:
      messages = [('RadarStatus', 16)]
      self.num_points = 40
      self.trigger_msg = 1119
    else:
      messages = [('TeslaRadarSguInfo', 10)]
      self.num_points = 32
      self.trigger_msg = 878

    for i in range(self.num_points):
      messages.extend([
        (f'RadarPoint{i}_A', 16),
        (f'RadarPoint{i}_B', 16),
      ])

    self.rcp = CANParser(DBC[CP.carFingerprint]['radar'], messages, CANBUS.radar)
    if CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      self._preap_radar_offset_m = get_float(PreApParamKeys.RADAR_OFFSET, 0.0)
      self._preap_radar_upside_down = get_bool(PreApParamKeys.RADAR_UPSIDE_DOWN, False)

  def update(self, can_strings):
    if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
      _feed_radar_vin_can(can_strings)
      # Live toggle: re-read each update so the UI takes effect without reboot.
      self._preap_radar_upside_down = get_bool(PreApParamKeys.RADAR_UPSIDE_DOWN, False)
      self._preap_radar_offset_m = get_float(PreApParamKeys.RADAR_OFFSET, 0.0)
    if self.rcp is None:
      return super().update(None)

    values = self.rcp.update_strings(can_strings)
    self.updated_messages.update(values)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = car.RadarData.new_message()

    # Errors
    errors = []
    if not self.rcp.can_valid:
      errors.append('canError')

    if self.CP.carFingerprint == CAR.TESLA_MODELS_RAVEN:
      radar_status = self.rcp.vl['RadarStatus']
      if radar_status['sensorBlocked'] or radar_status['shortTermUnavailable'] or radar_status['vehDynamicsError']:
        errors.append('fault')
    else:
      radar_status = self.rcp.vl['TeslaRadarSguInfo']
      # Pre-AP Bosch retrofit (NAP radar_interface + Tinkla Pre-AP):
      # SGUFail is a summary lamp (calibration / VIN reject / nosecone / etc.) and
      # must NOT block engage — tracks often stay live. HWFail is the dead-sensor
      # bit and still faults unless TeslaRadarIgnoreHwFail (NAP: NAPRadarIgnoreHwFail,
      # default off). SensorDirty is temporary in NAP; this cereal has no
      # radarUnavailableTemporary, so do not hard-fault on it for Pre-AP either.
      if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
        ignore_hw = get_bool(PreApParamKeys.RADAR_IGNORE_HW_FAIL, False)
        if radar_status['RADC_HWFail'] and not ignore_hw:
          errors.append('fault')
      elif radar_status['RADC_HWFail'] or radar_status['RADC_SGUFail'] or radar_status['RADC_SensorDirty']:
        errors.append('fault')

    ret.errors = errors

    # Radar tracks
    for i in range(self.num_points):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      # Check if it's a valid track
      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        self._preap_frozen.forget(i)
        continue

      if self.CP.carFingerprint == CAR.TESLA_PREAP_MODELS:
        raw = (msg_a['LongDist'], msg_a['LatDist'], msg_a['LongSpeed'], msg_a['LongAccel'], msg_b['LatSpeed'])
        if not self._preap_frozen.keep(i, raw, time.monotonic()):
          self.pts.pop(i, None)
          continue

      # New track!
      if i not in self.pts:
        self.pts[i] = car.RadarData.RadarPoint.new_message()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      # Parse track data. Upside-down mount inverts lateral dist/vel (NAP).
      self.pts[i].dRel = msg_a['LongDist']
      y_rel = msg_a['LatDist'] + self._preap_radar_offset_m
      yv_rel = msg_b['LatSpeed']
      if self._preap_radar_upside_down:
        y_rel = -y_rel
        yv_rel = -yv_rel
      self.pts[i].yRel = y_rel
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = yv_rel
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
