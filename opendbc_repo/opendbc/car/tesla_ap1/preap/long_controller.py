"""NAP-style Pre-AP pedal longitudinal controller (VirtualDAS + authority).

Credit: NotAutopilot (NAP) nap-release e99f4dc75 preap/carcontroller.py (cancel triggers, 50 Hz path, engage grace); PedalAuthority/RegenDecelMonitor and gating are BogPilot. Software-only GAS_SENSOR handshake; no panda loosening.
Passive BogPilot PedalCalib* stays; di_to_pedal uses current_pedal_transform.
"""
from __future__ import annotations

import logging
import math
import time

from pathlib import Path
from opendbc.car.tesla_ap1.preap.accel_limits import get_preap_accel_limits
from opendbc.car.tesla_ap1.preap.engagement import Mode
from opendbc.car.tesla_ap1.preap.pedal_authority import (
  ENGAGE_GRACE_FRAMES,
  ENGAGE_GRACE_PEDAL_RAMP_RATE_UP,
  PedalAuthority,
  PedalAuthorityState,
  PedalCommandAction,
  RegenDecelMonitor,
)
from opendbc.car.tesla_ap1.preap.pedal_can import create_pedal_command
from opendbc.car.tesla_ap1.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla_ap1.preap.pedal_controller import (
  di_to_pedal,
  get_zero_torque,
)
from opendbc.car.tesla_ap1.preap.vdas_constants import PEDAL_RAMP_RATE_UP
from opendbc.car.tesla_ap1.preap.virtual_das import VirtualDAS

_log = logging.getLogger("preap.long")

PERSONALITY_PATHS = (Path("/data/params_bogpilot/LongitudinalPersonality"), Path("/data/params/d/LongitudinalPersonality"))
# How often the stored personality is re-read, off the engage edge (see PreAPLongController._personality_refresh).
PERSONALITY_REFRESH_S = 2.0


class PreAPLongController:
  """Pedal-mode longitudinal: VirtualDAS, zero-torque, command authority.

  Stalk-CC spoofs (CANCEL) stay in StockCCSpoofer via CarState flags.
  """

  def __init__(self, pedal_counter, pedal_bus_fn):
    self.prev_pedal_di = 0.0
    self.prev_requested_long = False
    self.preap_long_engage_frame = -1000000
    self.engage_a_max = 0.0
    self.preap_long_handoff_slew_active = False
    self.vdas = VirtualDAS(dt=0.02)
    self.pedal_authority = PedalAuthority()
    self.regen_decel_monitor = RegenDecelMonitor()
    self.pedal_counter = pedal_counter
    self._pedal_bus_fn = pedal_bus_fn
    # Driving Personality, cached. _personality() stats and reads up to two /data files, and it used to run on the
    # pedal ACQUIRE edge, inside card's 100 Hz step: exactly the frame that must not be slow. Primed here (card
    # start, offroad) and refreshed from update() only on frames that are NOT acquiring, so the edge reads memory.
    self._personality_cache = self._personality_read()
    self._personality_read_mono = 0.0

  @staticmethod
  def _fsm_full(CS) -> bool:
    """True only when the pull FSM is in FULL (steering + speed control). A missing or invalid mode reads as not FULL."""
    try:
      return Mode(int(getattr(CS, "preap_fsm_mode", Mode.OFF))) == Mode.FULL
    except (TypeError, ValueError):
      return False

  @staticmethod
  def _driver_holds_pedal(CS) -> bool:
    """ENABLE=1 is never sent while braking, while the accelerator is pressed (by our test or the panda's), during the one update
    after it is released (the panda refuses ENABLE while gas_pressed_prev), or during the Q1 (b) standstill hold. Missing = hold."""
    return (bool(getattr(CS, "real_brake_pressed", True)) or bool(getattr(getattr(CS, "out", None), "gasPressed", True))
            or bool(getattr(CS, "preap_panda_gas_pressed", True)) or int(getattr(CS, "preap_gas_clear_updates", 0)) < 2
            or bool(getattr(CS, "preap_standstill_hold", True)))

  def _append_pedal_command(self, can_sends, CS, pedal_cmd: float, enable: int):
    # The one place a pedal frame is built (single choke point). ENABLE=1 only in FULL: anything else is sent as
    # (0.0, enable=0), the passthrough frame the disabled path already sends, and counted so CarState demotes FULL
    # with the speed-blocked warning. Mirrors NAP, where only the second pull of a double pull starts the pedal
    # (NAP preap/engagement.py:131, e99f4dc75). Userspace only; the panda checks are unchanged.
    if enable and not self._fsm_full(CS):
      enable, pedal_cmd = 0, 0.0
      CS.preap_pedal_guard_blocks = int(getattr(CS, "preap_pedal_guard_blocks", 0)) + 1
    elif enable and self._driver_holds_pedal(CS):
      enable, pedal_cmd = 0, 0.0           # driver input, not a fault: not counted, FULL stays
    elif enable and getattr(CS, "preap_pedal_bus_ok", True) is not True:
      # interceptor never seen, or 0x552 arrived on a bus other than the latched one (tesla_preap.h:128-135)
      enable, pedal_cmd = 0, 0.0
      CS.preap_pedal_guard_blocks = int(getattr(CS, "preap_pedal_guard_blocks", 0)) + 1
    cmd = create_pedal_command(pedal_cmd, enable, self.pedal_counter,
                               int(getattr(CS, "pedal_bus", self._pedal_bus_fn())))
    can_sends.append(cmd)
    # counter lives in dat[4] low nibble
    try:
      CS.pedal_command_counter = cmd.dat[4] & 0x0F
    except Exception:
      pass

  def _personality_read(self) -> int:
    """Driving Personality (0 aggressive, 1 standard, 2 relaxed). opendbc cannot read Params, so this
    reads the stored value as a file (the same pattern as toggles.disable_openpilot_long). Default 1.

    Touches /data: call it from _personality_refresh(), never from the engage edge."""
    for p in PERSONALITY_PATHS:
      try:
        if p.is_file():
          return max(0, min(2, int(p.read_text(encoding="utf-8", errors="replace").strip() or "1")))
      except (OSError, ValueError):
        continue
    return 1

  def _personality_refresh(self) -> None:
    """Re-read the stored personality at most every PERSONALITY_REFRESH_S. Called only on frames that are not
    taking pedal authority, so the file IO never lands on the ACQUIRE edge. A read error keeps the last value."""
    now = time.monotonic()
    if now - self._personality_read_mono < PERSONALITY_REFRESH_S:
      return
    self._personality_read_mono = now
    try:
      self._personality_cache = self._personality_read()
    except Exception:
      pass

  def _personality(self) -> int:
    """The cached personality. No file IO: see _personality_refresh()."""
    return int(self._personality_cache)

  def update(self, CC, CS, frame, *, long_ok: bool, now_nanos: int = 0):
    """Append pedal CAN frames for this control step. Returns can_sends list."""
    can_sends = []
    actuators = CC.actuators

    # requested_long needs CC.enabled + longActive + gating AND the pull FSM in FULL (the second pull of a NAP-style
    # double pull, NAP preap/engagement.py:131), so the authority FSM never asks for the pedal on steering only.
    # Q1 (b) standstill hold: not requested, so the pedal authority acquires (with its engage grace) only after the accelerator.
    requested_long = bool(CC.enabled) and bool(CC.longActive) and bool(long_ok) and self._fsm_full(CS) \
      and not bool(getattr(CS, "preap_standstill_hold", False))
    long_active = requested_long
    pedal_long_allowed = bool(long_ok)

    if (not long_active
        or getattr(CS, "real_brake_pressed", False)
        or getattr(CS.out, "gasPressed", False)):
      self.regen_decel_monitor.reset()

    requested_long_rising = (not self.prev_requested_long) and requested_long
    if requested_long_rising:
      zero_torque_di = get_zero_torque().get(CS.out.vEgo)
      interceptor = float(getattr(CS, "pedal_interceptor_value", 0.0) or 0.0)
      self.prev_pedal_di = max(interceptor, zero_torque_di)
      CS.pedal_first_enabled_mono_time = 0
      # Clear prior authority failure so a fresh engage can re-acquire.
      if getattr(CS, "pedal_unavailable", False):
        CS.pedal_unavailable = False
        CS.pedal_fail_cause = 0
        if self.pedal_authority.state == PedalAuthorityState.FAILED:
          self.pedal_authority.state = PedalAuthorityState.INACTIVE
    elif not hasattr(CS, "pedal_first_enabled_mono_time"):
      CS.pedal_first_enabled_mono_time = 0

    # Stock CC cancel triggers (same as prior BogPilot StockCCSpoofer path)
    if pedal_long_allowed:
      cruise_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
      prev_btn = int(getattr(CS, "prev_cruise_buttons", 0) or 0)
      pedal_button_press = (cruise_btn != prev_btn) and (cruise_btn != 0)
      pedal_long_falling = self.prev_requested_long and not requested_long
      if requested_long_rising or pedal_long_falling or pedal_button_press:
        CS.preap_cc_cancel_needed = True

    self.prev_requested_long = requested_long

    # 50 Hz pedal path (even frames), matching NAP preap/carcontroller.py:88
    if frame % 2 == 0:
      brake_pressed = bool(getattr(CS, "real_brake_pressed", False))
      gas_pressed = bool(getattr(CS.out, "gasPressed", False))
      authority_requested = (
        pedal_long_allowed and long_active and not brake_pressed and not gas_pressed
      ) # a failed acquisition no longer blocks the request; the authority retries with back-off while it is asked for
      feedback = getattr(CS, "pedal", None)
      if feedback is None:
        # Should be wired by carstate; fail soft to inactive. Imported at module scope, not here: this branch
        # first runs inside card's 100 Hz step, and a first-time module import there is a step overrun.
        feedback = PedalFeedback()
        CS.pedal = feedback

      pedal_action = self.pedal_authority.update(authority_requested, feedback)
      if pedal_action != PedalCommandAction.ACQUIRE:
        # Keep the stored personality fresh on ordinary frames, so the ACQUIRE edge below reads memory.
        self._personality_refresh()
      if not authority_requested and getattr(CS, "pedal_unavailable", False):
        CS.pedal_unavailable = False          # nothing is asking for the pedal any more: drop the retry state and its alert
        CS.pedal_fail_cause = 0
      in_engage_grace = False

      if pedal_action == PedalCommandAction.ACQUIRE:
        CS.pedal_unavailable = False          # a retry that worked: the alert clears itself
        CS.pedal_fail_cause = 0
        self.preap_long_engage_frame = frame
        self.preap_long_handoff_slew_active = True
        zero_torque_di = get_zero_torque().get(CS.out.vEgo)
        interceptor = float(getattr(CS, "pedal_interceptor_value", 0.0) or 0.0)
        self.prev_pedal_di = max(interceptor, zero_torque_di)
        self.vdas.reset(
          measured_accel=CS.out.aEgo,
          commanded_accel=0.0,
          pedal_di_init=self.prev_pedal_di,
          preserve_grade=True,
        )
        _, self.engage_a_max = get_preap_accel_limits(CS.out.vEgo, self._personality())

      orientation = list(getattr(CC, "orientationNED", None) or [])
      if pedal_long_allowed and pedal_action not in (
        PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE,
      ):
        self.vdas.observe(CS.out.aEgo, orientation)

      if pedal_long_allowed:
        get_zero_torque().update(
          float(getattr(feedback, "torque_level", 0.0) or 0.0),
          self.prev_pedal_di,
          CS.out.vEgo,
          control_active=pedal_action in (
            PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE,
          ),
          accel_command=self.vdas.jerk_limiter.a_limited,
        )

      if pedal_action == PedalCommandAction.RESET:
        self._append_pedal_command(can_sends, CS, 0.0, 0)
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      elif pedal_action == PedalCommandAction.RELEASE:
        self._append_pedal_command(can_sends, CS, 0.0, 0)
        self.prev_pedal_di = 0.0
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      elif pedal_action in (PedalCommandAction.ACQUIRE, PedalCommandAction.ENABLE):
        try:
          engage_elapsed_frames = frame - self.preap_long_engage_frame
          in_engage_grace = engage_elapsed_frames < ENGAGE_GRACE_FRAMES
          accel_request = float(actuators.accel)
          accel_effort_limits = None
          pedal_ramp_rate_up = (
            ENGAGE_GRACE_PEDAL_RAMP_RATE_UP
            if self.preap_long_handoff_slew_active
            else PEDAL_RAMP_RATE_UP
          )
          if in_engage_grace:
            # Cap at grace_progress * engage_a_max so the ceiling is the
            # personality accel envelope scaled by grace progress (NAP preap/carcontroller.py:105-111).
            grace_progress = engage_elapsed_frames / ENGAGE_GRACE_FRAMES
            accel_cap = grace_progress * self.engage_a_max
            accel_request = max(0.0, min(accel_request, accel_cap))
            accel_effort_limits = (0.0, accel_cap)
            pedal_ramp_rate_up = ENGAGE_GRACE_PEDAL_RAMP_RATE_UP

          self.prev_pedal_di = self.vdas.update(
            accel_request, CS.out.vEgo, self.prev_pedal_di,
            a_ego=CS.out.aEgo, freeze_integrator=in_engage_grace,
            orientation_ned=orientation,
            accel_effort_limits=accel_effort_limits,
            pedal_ramp_rate_up=pedal_ramp_rate_up,
          )
          handoff_slew_complete = (
            self.preap_long_handoff_slew_active
            and not in_engage_grace
            and not self.vdas.pedal_ramp_limited_up
          )
          if handoff_slew_complete:
            self.preap_long_handoff_slew_active = False

          pedal_cmd = di_to_pedal(self.prev_pedal_di)
          if not math.isfinite(pedal_cmd):
            raise ValueError("non-finite pedal command")
          self._append_pedal_command(can_sends, CS, float(pedal_cmd), 1)
          if pedal_action == PedalCommandAction.ACQUIRE and CS.pedal_first_enabled_mono_time == 0:
            CS.pedal_first_enabled_mono_time = now_nanos
          self.regen_decel_monitor.update(
            pedal_control_active=True,
            in_engage_grace=in_engage_grace,
            pedal_di=self.prev_pedal_di,
            limited_accel=self.vdas.jerk_limiter.a_limited,
            actual_accel=CS.out.aEgo,
            v_ego=CS.out.vEgo,
          )

        except Exception:
          _log.exception("Pre-AP pedal command failed; sending disabled")
          self.pedal_authority.command_failed()
          CS.pedal_unavailable = True
          CS.pedal_fail_cause = int(self.pedal_authority.fail_cause)
          self._append_pedal_command(can_sends, CS, 0.0, 0)
          self.prev_pedal_di = 0.0
          self.preap_long_handoff_slew_active = False
          self.regen_decel_monitor.reset()
          pedal_action = PedalCommandAction.FAILURE

      elif pedal_action == PedalCommandAction.FAILURE:
        _log.error("Pre-AP pedal authority acquisition failed: %s (retry %d)",
                   self.pedal_authority.fail_cause.name, self.pedal_authority.retries + 1)
        CS.pedal_unavailable = True
        CS.pedal_fail_cause = int(self.pedal_authority.fail_cause)
        self.preap_long_handoff_slew_active = False
        self.regen_decel_monitor.reset()

      else:
        self.regen_decel_monitor.reset()

      CS.pedal_authority_requested = authority_requested
      CS.pedal_authority_active = (
        self.pedal_authority.state == PedalAuthorityState.ACTIVE
        and not bool(getattr(CS, "pedal_unavailable", False))
      )
      CS.pedal_authority_state = int(self.pedal_authority.state)
      CS.pedal_authority_action = int(pedal_action)

    CS.vdas_limited_accel = float(self.vdas.jerk_limiter.a_limited)
    CS.pedal_command_di = float(self.prev_pedal_di)
    CS.pedal_brake_required = bool(self.regen_decel_monitor.active)
    return can_sends
