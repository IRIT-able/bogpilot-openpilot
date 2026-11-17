#!/usr/bin/env python3
import os
import time
import threading

import openpilot.cereal.messaging as messaging

from openpilot.cereal import log
from opendbc.car.structs import car

from openpilot.common.params import Params
from openpilot.common.traffic_mode_hold import TrafficModeHold
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.selfdrive.car.step_overrun import StepOverrunCounter
from openpilot.selfdrive.car.preap_card_step import (PREAP_ENGAGE_SET_REFRESH_S, use_10hz_distance_button,
                                                      use_step_overrun_counter)
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.tesla_ap1.phase3_stalk_controls import apply_experimental_toggle, twist_follow_on, experimental_supported
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import VCruiseHelper, PREAP_FULL_USE_SET_SPEED_LIMIT
from openpilot.selfdrive.car.ap1_runtime import apply_ap1_volume_defaults
from openpilot.selfdrive.car.bogpilot_card import bogpilot_car_state_msg

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing, block=True)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'deviceState', 'bogpilotPlan'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'radarTracks', 'bogpilotCarState'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = self.params.get_bool("IsReleaseBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      from openpilot.selfdrive.ui.bogpilot.phase2_car_selection import selected_fingerprint

      def fingerprint_override(detected):
        return selected_fingerprint(self.params, detected)

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, cached_params,
                        fingerprint_override=fingerprint_override)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP)
      self.CP = self.CI.CP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True, block=True)
    else:
      self.CI, self.CP = CI, CI.CP
      self.RI = RI

    apply_ap1_volume_defaults(self.CP, self.params)
    if self.CP.carFingerprint == "TESLA_PREAP_MODELS":
      from openpilot.selfdrive.car import preap_aol
      preap_aol.install(self.CP)   # Pre-AP double pull rides AOL only while the AOL lane says it may; nothing is enabled here
    self.ap1_model = None
    if self.CP.brand == "tesla_ap1" and self.CP.carFingerprint != "TESLA_PREAP_MODELS" and self.CI.CC is not None:
      self.ap1_model = messaging.SubMaster(["modelV2"])
      self.CI.CC.model_v2_provider = lambda: self.ap1_model["modelV2"] if self.ap1_model.all_checks() else None

    self.CP.alternativeExperience = 0
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key, block=True)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp, block=True)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes, block=True)
    self.params.put("CarParamsCache", cp_bytes)
    self.params.put("CarParamsPersistent", cp_bytes)

    self.v_cruise_helper = VCruiseHelper(self.CP)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")
    self.ap1_cluster_lift = bool(self.CP.brand == "tesla_ap1" and self.CP.carFingerprint != "TESLA_PREAP_MODELS" and self.CP.openpilotLongitudinalControl)
    # Pre-AP with pedal (pcmCruise=False): 0.9.7 SetSpeedLimit engage set from a fresh posted limit.
    self.preap_engage_set_limit = None
    self.preap_tx_sample = None
    self.preap_refusals = None
    self.preap_sent_560 = 0
    self.preap_safety_expect = None
    self.preap_safety_expects: list[tuple[int, int]] = []
    self.preap_car_mode_prev = False
    self.preap_560_dropped = 0
    self.preap_ps_seen_sample = None   # pandaStates logMonoTime last seen, and card's clock when it changed (freshness)
    self.preap_ps_seen_at: float | None = None
    self.preap_vin_read = None
    self.preap_vin_read_rx = []
    if self.CP.carFingerprint == "TESLA_PREAP_MODELS":
      from opendbc.car.tesla_ap1.preap.panda_link import RateLimitedLog, TxRefusalWatch
      from opendbc.car.tesla_ap1.preap.radar_vin_read import RadarVinReadRunner
      from opendbc.car.tesla_ap1.safety_flags import FLAG_TESLA_PREAP_RADAR_EMULATION
      param = int(self.CP.safetyConfigs[-1].safetyParam) if len(self.CP.safetyConfigs) else 0
      self.preap_vin_read = RadarVinReadRunner(bool(param & FLAG_TESLA_PREAP_RADAR_EMULATION))
      self.preap_refusals = TxRefusalWatch()
      self.preap_warn = RateLimitedLog(cloudlog.warning, 10.0)
      # no /data file IO in the 100 Hz step: Pre-AP params through a 10 Hz background snapshot (preap/params.py StepCache)
      from opendbc.car.tesla_ap1.preap.params import start_step_cache
      start_step_cache()
      if len(self.CP.safetyConfigs):
        # safetyModel is a capnp enum on the device (no int()); .raw is its number, as for pandaStates in preap_link_update
        sm_ = self.CP.safetyConfigs[-1].safetyModel
        self.preap_safety_expect = (int(getattr(sm_, "raw", sm_)), int(self.CP.safetyConfigs[-1].safetyParam))
        # per panda (safetyConfigs[i] is what pandad sets on panda i), for the host-side 0x560 gate
        self.preap_safety_expects = [(int(getattr(c.safetyModel, "raw", c.safetyModel)), int(c.safetyParam)) for c in self.CP.safetyConfigs]
    if self.CP.brand == "tesla_ap1" and self.CP.carFingerprint == "TESLA_PREAP_MODELS" and not self.CP.pcmCruise:
      from openpilot.selfdrive.controls.lib.phase3_speed_control import PreapEngageSetLimit, warm_engage_helpers
      self.preap_engage_set_limit = PreapEngageSetLimit(self.CP)
      # Warm now (offroad), as the AP1 helpers below already are: PreapEngageSetLimit.kph() runs on the enable and
      # FULL-capture edges inside the 100 Hz step, and its first call paid two lazy imports, a ParamStore (with its
      # first all_keys()) and dozens of param reads there. Saved logs of this build show a 120 ms card step 4 ms
      # after longActive first rose, which is enough on its own to push selfdrived's 100-sample average over the
      # 11.11 ms Ratekeeper.lagging threshold and raise "System Lagging" (a soft disable). params_thread keeps the
      # snapshot fresh from here on, so later engages do not read /data in the step either.
      warm_engage_helpers(self.preap_engage_set_limit)
    self._cluster_floor = None
    if self.ap1_cluster_lift:
      from openpilot.selfdrive.controls.lib.phase3_speed_control import EngageClusterFloor
      self._cluster_floor = EngageClusterFloor()
    if getattr(self.CI.CS, "op_long_set", None) is not None:
      # AP1 openpilot long: speed-limit offset for the engage set, from params (not bogpilotPlan: no loop).
      from openpilot.selfdrive.controls.lib.phase3_speed_control import EngageLimitOffset, EngageTapFromZero, warm_engage_helpers
      self.CI.CS.op_long_offset = EngageLimitOffset(self.CP)
      self.CI.CS.op_long_tap_from_zero = EngageTapFromZero()
      # Warm now (offroad) so the first engage does not pay lazy imports + param reads inside the loop.
      warm_engage_helpers(self.CI.CS.op_long_offset, self.CI.CS.op_long_tap_from_zero)
    self.ap1_personality_sync = None
    self.traffic_mode_hold = TrafficModeHold()
    if self.CP.brand == "tesla_ap1" and self.CP.carFingerprint == "TESLA_AP1_MODELS":
      from openpilot.selfdrive.controls.lib.phase3_following import Ap1PersonalitySync, personality_adopted
      # stock-equivalent default: stalk detents only write LongitudinalPersonality after the user adopted Driving Personalities
      if personality_adopted():
        self.ap1_personality_sync = Ap1PersonalitySync(self.params)

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    # 07e8bd68f moved OnroadDistanceButtonPressed to the 10 Hz params thread and
    # added a log-only step-overrun counter on every car. Gate both to Pre-AP:
    # AP1 and unknown keep the 92aa9ee1f path (read the button every 100 Hz step,
    # no overrun timer).
    if use_10hz_distance_button(self.CP.carFingerprint):
      self.onroad_distance_button = False
    self.step_timer = StepOverrunCounter(cloudlog.warning) if use_step_overrun_counter(self.CP.carFingerprint) else None

  def state_update(self) -> tuple[car.CarState, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)
    self.preap_vin_read_rx = []
    if getattr(self, "preap_vin_read", None) is not None and self.preap_vin_read.reader is not None:
      # Read Radar VIN button, only while a read runs: radar UDS replies (0x651, bus 1) for this step (opendbc tesla_ap1/preap/radar_vin_read.py).
      # can_capnp_to_list gives (nanos, [(address, dat, src), ...]) plain tuples (selfdrive/pandad/pandad_api_impl.py).
      try:
        self.preap_vin_read_rx = [CanData(addr, dat, src) for _, msgs in can_list for addr, dat, src in msgs if addr == 0x651 and src == 1]
      except Exception:
        self.preap_vin_read_rx = []   # the read then times out on its own; card keeps running

    if self.ap1_model is not None:
      self.ap1_model.update(0)

    # Read live device/control state before arming experimental-only gestures.
    self.sm.update(0)
    started = bool(self.sm.valid["deviceState"] and self.sm.alive["deviceState"] and self.sm["deviceState"].started)
    if hasattr(self.CI.CS, "stalk_controls_allowed"):
      # Arm detection without ExperimentalModeConfirmed so a completed hold can
      # still raise the "confirm in Settings" alert; apply_experimental_toggle keeps the gate.
      self.CI.CS.stalk_controls_allowed = started and experimental_supported(self.CP)
    if self.preap_refusals is not None:
      self.preap_link_update(can_list)
    # Update carState from CAN. Physical buttons/cancel remain untouched.
    CS = self.CI.update(can_list)

    mode = apply_experimental_toggle(self.params, self.CP, CS, started)
    if mode is not None:
      self.experimental_mode = mode

    stalk_traffic = None
    if self.ap1_personality_sync is not None and twist_follow_on(self.CI.CS):
      # 04d560f3 bogpilot_card._apply_ap1_stalk: stalk detents 3/5/7 drive LongitudinalPersonality (read-back sync)
      # Stalk Twist Following Distance off: the wheel must not write the personality icon/param either.
      try:
        stalk_traffic = self.ap1_personality_sync.update(getattr(self.CI.CS, "stalk_follow", None))
      except Exception:
        cloudlog.exception("card: AP1 personality sync failed")
    self.traffic_mode_hold.apply_stalk(stalk_traffic)
    if use_10hz_distance_button(self.CP.carFingerprint):
      distance_pressed = self.onroad_distance_button
    else:
      # 92aa9ee1f: read every 100 Hz step on AP1 and unknown (07e8bd68f gated to Pre-AP)
      distance_pressed = self.params.get_bool("OnroadDistanceButtonPressed")
    CS.bogpilot.trafficModeEnabled = self.traffic_mode_hold.update_button(
      distance_pressed, time.monotonic())

    # Update radar tracks from CAN
    if self.preap_refusals is not None and hasattr(self.RI, "preap_v_ego"):
      self.RI.preap_v_ego = float(CS.vEgo)      # radar-valid gate's late label only (opendbc tesla_ap1/preap/radar_valid.py)
    RD: structs.RadarDataT | None = self.RI.update(can_list)
    if self.preap_refusals is not None:
      # Pre-AP radar, advisory only (opendbc tesla_ap1/preap/radar_valid.py): the one warning while its points are kept out of radard
      # and speed control runs on camera leads. Only with openpilot longitudinal (the pedal); no engage or speed-control input.
      CS.bogpilot.preapRadarNotReady = bool(self.CP.openpilotLongitudinalControl and getattr(self.RI, "preap_gate", None) is not None
                                            and not getattr(self.RI, "preap_radar_valid", True))

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
    if self.sm['carControl'].enabled and not self.CC_prev.enabled:
      # Use CarState w/ buttons from the step selfdrived enables on
      set_limit_kph = 0.0
      if self.preap_engage_set_limit is not None:
        set_limit_kph = self.preap_engage_set_limit.kph(self.CS_prev, getattr(self.CI.CS, "engage_limit_ms", 0.0))
      self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode, set_limit_kph)

    if getattr(self.CI.CS, "preap_capture_edge", False) and not self.CP.pcmCruise:
      # Pre-AP pedal FULL capture edge (opendbc tesla_ap1/preap/engagement.py capture_edge). Pedal capture speed choice: SetSpeedLimit gives limit + offset
      # through the same freshness / units / standstill / gas gates as the engage set; otherwise the exact vEgo.
      set_limit_kph = 0.0
      if PREAP_FULL_USE_SET_SPEED_LIMIT and self.preap_engage_set_limit is not None:
        set_limit_kph = self.preap_engage_set_limit.kph(CS, getattr(self.CI.CS, "engage_limit_ms", 0.0))
      self.v_cruise_helper.capture_preap_full(float(CS.vEgo), set_limit_kph)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)
    if self.ap1_cluster_lift:
      # AP1 SLC / software stalk set: cluster + hudControl.setSpeed follow the planner set when it is higher
      # (0.9.7 controlsd cluster_display_kph). Floor at vEgo from enable until the plan has caught up so the
      # Tesla IC never flashes the DI_cruiseSet seed (8-10 mph / ~vEgo/2). Stalk reads are unchanged.
      from openpilot.selfdrive.controls.lib.phase3_speed_control import cluster_set_kph
      fresh = bool(self.sm.alive['bogpilotPlan'] and self.sm.valid['bogpilotPlan'])
      plan = self.sm['bogpilotPlan'] if fresh else None
      plan_v = float(getattr(plan, "vCruise", 0.0) or 0.0) if plan is not None else 0.0
      enabled = bool(self.sm['carControl'].enabled)
      # AP1 openpilot long: the cluster base is openpilot's engage set. Floor the IC at it (and vEgo) from the enable
      # edge until the plan reaches it or the driver tips, then show the plan set as is (down tips and SLC drops too).
      op_long = getattr(self.CI.CS, "op_long_set", None) is not None
      tip = op_long and any(be.type in (car.CarState.ButtonEvent.Type.accelCruise, car.CarState.ButtonEvent.Type.decelCruise)
                            for be in CS.buttonEvents)
      engage_floor = bool(self._cluster_floor is not None and
                          self._cluster_floor.update(enabled, fresh, plan_v, float(CS.vEgo),
                                                     floor_ms=float(CS.cruiseState.speed) if op_long else 0.0, tip=tip))
      if fresh or engage_floor:
        CS.vCruiseCluster = float(cluster_set_kph(CS.vCruiseCluster, plan, fresh,
                                                  v_ego_ms=float(CS.vEgo), engage_floor=engage_floor,
                                                  follow_plan=op_long))

    return CS, RD

  def preap_link_update(self, can_list) -> None:
    """Pre-AP: what the panda reports, for the pull FSM and the 0x560 stream (opendbc tesla_ap1/preap/panda_link.py).
    The FSM gets refusals of our own 0x551 / 0x488 frames, read from their rejected TX echoes, not the all-address
    safetyTxBlocked total, so our 0x560 (always refused by design) can never read as a pedal refusal. The total stays a
    cross-check: blocked frames whose echo never came are passed on (fail-safe) unless an own 0x560 was sent nearby (logged)."""
    from opendbc.car.tesla_ap1.preap.panda_link import LinkInput, scan_echoes
    echoes = scan_echoes(can_list)
    valid = bool(self.sm.valid["pandaStates"])
    pss = list(self.sm["pandaStates"]) if self.sm.seen["pandaStates"] else []
    sample = self.sm.logMonoTime["pandaStates"]
    new_sample = valid and len(pss) > 0 and sample != self.preap_tx_sample
    totals = None
    if new_sample:
      self.preap_tx_sample = sample
      totals = [int(ps.safetyTxBlocked) for ps in pss]
    # Panda 0 gets bus 0 and 2 (our 0x551 / 0x488 and the 0x560 config). Refusals count only while it shows the car's mode on
    # this AND the previous step, so an echo of a frame refused before the switch (ELM327) never reads as a pedal refusal.
    car_mode = self.preap_panda_mode_ok(0, pss, valid)
    refusals, logged = self.preap_refusals.step(echoes, self.preap_sent_560, totals, car_mode=car_mode and self.preap_car_mode_prev)
    self.preap_car_mode_prev = car_mode
    self.preap_sent_560 = 0
    if logged:
      self.preap_warn(time.monotonic(), f"card: Pre-AP {logged} blocked TX without an echo next to an own 0x560, not a refusal")
    if refusals > 0:
      self.CI.CS.preap_panda_tx_blocked = int(getattr(self.CI.CS, "preap_panda_tx_blocked", 0) or 0) + refusals
    link = LinkInput(gtw_echo=echoes.gtw > 0, new_sample=new_sample, controls_allowed=True)
    if pss:
      ps = pss[0]
      key = (int(getattr(ps.safetyModel, "raw", ps.safetyModel)), int(ps.safetyParam))
      link.safety_key = key
      link.safety_ok = car_mode
      link.uptime = int(ps.uptime)
      link.heartbeat_lost = bool(ps.heartbeatLost)
      # fail closed: no valid pandaStates reads as controls allowed, so no 0x560 goes out
      link.controls_allowed = (not valid) or any(bool(p.controlsAllowed) for p in pss)
    self.CI.CS.preap_link = link

  PREAP_PANDA_STATES_MAX_AGE_S = 0.5   # pandaStates is 10 Hz (pandad): five missed samples

  def preap_panda_states_fresh(self) -> bool:
    """pandaStates is alive (SubMaster) and a new sample arrived within PREAP_PANDA_STATES_MAX_AGE_S on card's clock. Stale but
    valid reads as not fresh, so the Pre-AP gates below fail closed (no 0x560, no refusal counting)."""
    sample = self.sm.logMonoTime["pandaStates"]
    now = time.monotonic()
    if self.sm.seen["pandaStates"] and sample != self.preap_ps_seen_sample:
      self.preap_ps_seen_sample = sample
      self.preap_ps_seen_at = now
    return (bool(self.sm.alive["pandaStates"]) and self.preap_ps_seen_at is not None
            and now - self.preap_ps_seen_at <= self.PREAP_PANDA_STATES_MAX_AGE_S)

  def preap_panda_mode_ok(self, panda: int, pss=None, valid=None) -> bool:
    """pandaStates (valid, alive, fresh, latest entry for this panda) shows the Pre-AP car mode with the param card set
    (panda_link.preap_mode_ok)."""
    from opendbc.car.tesla_ap1.preap.panda_link import preap_mode_ok
    if valid is None:
      valid = bool(self.sm.valid["pandaStates"])
    if pss is None:
      pss = list(self.sm["pandaStates"]) if self.sm.seen["pandaStates"] else []
    if not self.preap_panda_states_fresh():
      return False
    if not valid or panda >= len(pss) or panda >= len(self.preap_safety_expects):
      return False
    ps = pss[panda]
    return preap_mode_ok(int(getattr(ps.safetyModel, "raw", ps.safetyModel)), int(ps.safetyParam), self.preap_safety_expects[panda])

  def preap_gate_560(self, can_sends) -> list:
    """Host-side 0x560 gate, on the final sendcan of this step: a 0x560 goes out only while the latest pandaStates shows the panda
    it goes to (bus // 4) in the Pre-AP car mode, never in allOutput, noOutput, ELM327 or any other mode (that panda's tesla_preap
    tx hook consumes and refuses it, tesla_preap.h:291-299; in allOutput no hook would stop it, safety.h:249-250)."""
    from opendbc.car.tesla_ap1.preap.panda_link import PANDA_BUSES, RADAR_VIN_ADDR
    out = []
    for m in can_sends:
      if m[0] == RADAR_VIN_ADDR and not self.preap_panda_mode_ok(int(m[2]) // PANDA_BUSES):
        self.preap_560_dropped += 1
        continue
      out.append(m)
    return out

  def state_publish(self, CS: car.CarState, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    self.pm.send("bogpilotCarState", bogpilot_car_state_msg(CS))

    if RD is not None:
      tracks_msg = messaging.new_message('radarTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.radarTracks = RD
      self.pm.send('radarTracks', tracks_msg)

  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos)
      if getattr(self, "preap_vin_read", None) is not None:
        # Manual, disengaged-only, read-only F190 (never started here by itself). Engaged = openpilot enabled, lateral active,
        # the panda reporting controls allowed, or no valid pandaStates (fail closed); any of them stops the read at once.
        panda_ok = bool(self.sm.valid["pandaStates"]) and not any(ps.controlsAllowed for ps in self.sm["pandaStates"])
        engaged = bool(CC.enabled) or bool(CC.latActive) or not panda_ok
        try:
          can_sends = list(can_sends) + self.preap_vin_read.update(getattr(self, "preap_vin_read_rx", []), time.monotonic(), engaged)
        except Exception:
          # Never let the read stop card: end it and free the button. Fixed text only, no exception text or frame data in logs.
          self.preap_vin_read.fail_safe()
          cloudlog.error("card: Pre-AP radar read ended on an internal error")
      if self.preap_refusals is not None:
        can_sends = self.preap_gate_560(can_sends)
        self.preap_sent_560 += sum(1 for m in can_sends if m[0] == 0x560)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  def step(self):
    CS, RD = self.state_update()

    self.state_publish(CS, RD)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'])

    self.initialized_prev = initialized
    self.CS_prev = CS

  def params_thread(self, evt):
    last_engage_set = 0.0
    while not evt.is_set():
      if use_10hz_distance_button(self.CP.carFingerprint):
        self.onroad_distance_button = self.params.get_bool("OnroadDistanceButtonPressed")
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      # Pre-AP engage set: keep the SetSpeedLimit toggles snapshot fresh here, not on the engage frame in the step.
      # Once a second, not at 10 Hz: it is a few dozen param reads and the toggles only change in Settings.
      if self.preap_engage_set_limit is not None:
        now = time.monotonic()
        if now - last_engage_set >= PREAP_ENGAGE_SET_REFRESH_S:
          last_engage_set = now
          self.preap_engage_set_limit.refresh()
      time.sleep(0.1)

  def card_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        if self.step_timer is not None:
          t0 = time.monotonic()
          self.step()
          self.step_timer.record(time.monotonic() - t0, t0)
        else:
          self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
