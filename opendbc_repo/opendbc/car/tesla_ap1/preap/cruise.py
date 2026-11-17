"""Pre-AP cruiseState helpers for comma-pedal longitudinal.

NAP (preap/carstate.py:75, e99f4dc75) sets cruiseState.available=True always and engages on SpdCtrlLvr_Stat=2
(RWD / pull toward driver). With pcmCruise=False we cannot leave available
tied to DI_cruiseState STANDBY/ENABLED or the UI shows wrongCarMode
("Enable Adaptive Cruise to Engage"). Panda SAFETY_TESLA_PREAP already
arms controls_allowed on stalk lever==2 (not DI_cruiseState).
"""
from __future__ import annotations

from opendbc.car import structs

_ButtonType = structs.CarState.ButtonEvent.Type

# STW_ACTN_RQ.SpdCtrlLvr_Stat — same as NAP CruiseButtons (tesla/values.py:223-224) / DBC
SPD_CTRL_CANCEL = 1  # FWD — push away
SPD_CTRL_MAIN = 2    # RWD — pull toward driver (engage)


def pedal_cruise_available(*, detected: bool, calibrated: bool, faulted: bool = False) -> bool:
  """True when pedal long may show cruise available (not stock DI-armed)."""
  return bool(detected and calibrated and not faulted)


def remap_preap_pedal_button_type(event_type):
  """Map stalk MAIN/RWD (resumeCruise in shared BUTTONS) to setCruise.

  Shared BUTTONS keep SpdCtrl=2 as resumeCruise for AP1 pcmCruise resume.
  Pre-AP pedal uses setCruise so buttonEnable initializes v_cruise from ego
  (resumeCruise alone would hit resumeBlocked: "Press Set to Engage").

  Called by PreApCarState on every button event it publishes, so it uses the
  opendbc structs enum (the one the events are built from) and imports nothing
  per call; opendbc never imports cereal.
  """
  if event_type == _ButtonType.resumeCruise:
    return _ButtonType.setCruise
  return event_type

# Echo windows (NAP preap/engagement.py:9-10,189-190): suppress SpdCtrl CANCEL that is our own
# spoofed STW_ACTN_RQ so openpilot does not treat it as a driver FWD cancel.
CANCEL_ECHO_WINDOW_MS = 600
SPOOF_ECHO_WINDOW_MS = 300


def is_preap_cancel_echo(*, now_ms: int, last_pull_ms: int,
                         last_cc_spoof_ms: int, cruise_enabled: bool = False) -> bool:
  """True when a SpdCtrl=CANCEL edge is likely our stalk spoof echo.

  A cancel is real once more than CANCEL_ECHO_WINDOW_MS (600) have passed since the last
  counted pull, matching tesla_preap.h:114-120 (the panda ignores only a cancel shortly
  after a pull, not after an up or down tip).

  The SPOOF_ECHO_WINDOW_MS (300) window starts only when our own CANCEL frame is sent
  (StockCCSpoofer stamps last_cc_spoof_ms on TX) and only swallows a cancel that is also
  inside the pull window, so a cancel the panda counts as real is never swallowed.
  """
  in_pull_window = last_pull_ms > 0 and 0 <= (now_ms - last_pull_ms) <= CANCEL_ECHO_WINDOW_MS
  if cruise_enabled and in_pull_window:
    return True
  if in_pull_window and last_cc_spoof_ms and 0 <= (now_ms - last_cc_spoof_ms) < SPOOF_ECHO_WINDOW_MS:
    return True
  return False
