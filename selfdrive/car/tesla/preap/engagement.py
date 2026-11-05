"""Pre-AP optional double-pull engagement (NAP-style).

When enabled (TeslaPreApDoublePull): first MAIN/setCruise pull arms lateral
only (via Always On Lateral); a second pull within DOUBLE_PULL_WINDOW_MS
adds longitudinal by allowing buttonEnable. Brake drops long (stock
pedalPressed) while AOL keeps lat.

When disabled: button events pass through unchanged so AOL is not fought.
Credit: NAP-style (window capped at 400 ms).
"""
from __future__ import annotations

from cereal import car

ButtonType = car.CarState.ButtonEvent.Type

# NAP hard-caps the window at 400 ms (natural double-pulls land 250–400 ms).
DOUBLE_PULL_WINDOW_MS = 400


def preap_steer_drop(hands_on_level: int, eac_status: int, eac_error_code: int) -> bool:
  """Same conditions as the 0x370 handler in safety_tesla_preap.h."""
  return hands_on_level >= 3 or (eac_status == 0 and 6 <= eac_error_code <= 9)


class DoublePullEngagement:
  def __init__(self):
    self.pending = False
    self.lat_only = False
    self.long_latched = False
    self.last_pull_ms = -10_000
    self._swallow_release = False

  def reset(self):
    self.pending = False
    self.lat_only = False
    self.long_latched = False
    self.last_pull_ms = -10_000
    self._swallow_release = False

  def filter_button_events(self, button_events, *, enabled: bool, now_ms: int,
                           brake_pressed: bool, cancel: bool):
    """Return filtered button events.

    enabled=False → identity (no AOL interference).
    """
    if not enabled:
      # Clear any stale double-pull state so flipping the toggle off is clean.
      if self.pending or self.lat_only or self._swallow_release:
        self.reset()
      return button_events

    if cancel:
      self.reset()
      return button_events

    # Brake drops longitudinal intent; lat_only / AOL keeps lateral.
    # (OP disable itself comes from card pedalPressed → USER_DISABLE.)
    if brake_pressed and self.long_latched:
      self.long_latched = False
      self.lat_only = True

    out = []
    for be in button_events:
      if be.type == ButtonType.cancel and be.pressed:
        self.reset()
        out.append(be)
        continue

      if be.type != ButtonType.setCruise:
        out.append(be)
        continue

      if be.pressed:
        dt = now_ms - self.last_pull_ms
        self.last_pull_ms = now_ms
        if self.pending and dt < DOUBLE_PULL_WINDOW_MS:
          # Second pull within window — keep event; release will buttonEnable.
          self.pending = False
          self.lat_only = False
          self.long_latched = True
          self._swallow_release = False
          out.append(be)
        else:
          # First pull — lateral only; swallow press + matching release.
          self.pending = True
          self.lat_only = True
          self.long_latched = False
          self._swallow_release = True
      else:
        if self._swallow_release:
          self._swallow_release = False
          # Swallow release so create_common_events does not buttonEnable.
        else:
          out.append(be)

    if self.pending and (now_ms - self.last_pull_ms) > DOUBLE_PULL_WINDOW_MS:
      self.pending = False
      # lat_only remains True: first pull armed lateral until cancel.

    return out
