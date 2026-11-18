"""Independent driver-override detector for the legacy Teslas. No CAN.

The EPAS hands-on flag is slow and coarse: the AP1 EPAS reports levels 0, 1 and 3
only, and the pause (hso.AP1_HANDS_ON_LEVEL) needs level 2 or more. A hard swerve
can reach -130 deg/s and -5.7 Nm while the flag still reads 0. This detector uses
what the EPAS and the steering angle sensor measure directly, on every control
step (100 Hz), and the angle openpilot last asked for.

Signals, checked in this order. The first one that holds this frame is the
reported trigger (carState.bogpilot.ap1OverrideSignal):

  HANDS         EPAS handsOnLevel >= 2 (the existing flag, unchanged).
  TORQUE        |driver torque| >= OVERRIDE_TORQUE_NM for OVERRIDE_TORQUE_FRAMES.
  RATE          |wheel rate| >= OVERRIDE_RATE_DPS together with
                |driver torque| >= OVERRIDE_RATE_TORQUE_NM for OVERRIDE_RATE_FRAMES.
  ANGLE_ERROR   openpilot is commanding ANGLE, the wheel is at least
                OVERRIDE_ERROR_DEG away from that command, and it is moving away
                from it at OVERRIDE_ERROR_RATE_DPS or more, for
                OVERRIDE_ERROR_FRAMES.

Debounce is counted in control frames. steeringTorque and the EPAS angle arrive
at about 25 Hz and are held between samples, so a 3 frame debounce needs at most
one fresh sample plus the hold of the previous one: it filters a single bad
sample, not a real push.

Override Sensitivity (a driver setting, levels 1 to 5, default 2) scales the torque and rate numbers above: level 3
(SENSITIVITY_BASE_LEVEL) is exactly the numbers in this file, a lower level needs a harder push, a higher level a lighter one. See
override_thresholds(). The shipped default is 2 (DIVERGENCES.md), one step firmer than 3: 2.25 Nm instead of 2.0 Nm. The hands flag,
the angle error signal, the debounce frame counts, the hold before resuming and the panda limits do not change with it.

`quiet` is the release test used for the hand-back: hands below the pause level (QUIET_HANDS_MAX_LEVEL), torque and
wheel rate both low. The numbers were chosen from logged engaged Model S driving
so that normal driving does not trigger them. Model X and Pre-AP use the same
signals and the same numbers; neither has engaged logs yet, so the numbers are
unproven there.
"""
import math

from opendbc.car.tesla_ap1.preap import params

SIGNAL_NONE = 0
SIGNAL_HANDS = 1
SIGNAL_TORQUE = 2
SIGNAL_RATE = 3
SIGNAL_ANGLE_ERROR = 4
# Set by the EPAS gate (epas_gate.py), never by this detector.
SIGNAL_EPAS = 5

SIGNAL_NAMES = {
  SIGNAL_NONE: "none",
  SIGNAL_HANDS: "hands",
  SIGNAL_TORQUE: "torque",
  SIGNAL_RATE: "rate",
  SIGNAL_ANGLE_ERROR: "angle_error",
  SIGNAL_EPAS: "epas",
}

OVERRIDE_HANDS_LEVEL = 2          # same as hso.AP1_HANDS_ON_LEVEL

OVERRIDE_TORQUE_NM = 2.0
OVERRIDE_TORQUE_FRAMES = 3        # 30 ms

OVERRIDE_RATE_DPS = 30.0
OVERRIDE_RATE_TORQUE_NM = 1.0
OVERRIDE_RATE_FRAMES = 2          # 20 ms

OVERRIDE_ERROR_DEG = 4.0
OVERRIDE_ERROR_RATE_DPS = 20.0
OVERRIDE_ERROR_FRAMES = 2         # 20 ms

# Release: below all of these, with the hands flag at 0.
QUIET_TORQUE_NM = 1.0
QUIET_RATE_DPS = 30.0
# Highest EPAS_handsOnLevel that can still count as released. The pause fires at hso.AP1_HANDS_ON_LEVEL (2), so
# this is that threshold minus one: the hand-back asks for hands *below the level that would pause*, not for an
# absolute 0. Level 1 is any driver torque the EPAS reports at all; it does not pause lateral and openpilot steers
# through it for most of a normal drive (hso.ap1_lat_active), so demanding 0 to come back made the release test
# stricter than the pause test. On logged AP1 EPAS the levels seen are 0, 1 and 3 only, and 9 of 14 level 3
# overrides dropped to 1 first and reached 0 up to 2.9 s later: a brief nudge cost seconds of no steering while
# the driver was already off the wheel. The substantive checks are unchanged and still have to pass -- torque
# under quiet_torque_nm and wheel rate under quiet_rate_dps -- so a driver still leaning on or turning the wheel
# does not get the command back, and the pause itself is untouched: the driver still wins on the frame they act.
# hso.AP1_HANDS_ON_LEVEL is not imported here because hso imports this module (test_preap_steer_quiet_hands
# keeps the two in step).
QUIET_HANDS_MAX_LEVEL = 1


# ---- Override Sensitivity ---------------------------------------------------------------------

SENSITIVITY_DEFAULT = 2        # shipped default, and the value for a missing or bad setting
SENSITIVITY_BASE_LEVEL = 3     # the level whose numbers are the module constants above (scale 1.0); not the default
SENSITIVITY_LEVELS = (1, 2, 3, 4, 5)
# Level 1 is the least sensitive (hardest push to override), level 5 the most. Level 3 is the numbers above, unscaled.
SENSITIVITY_SCALE = {1: 1.25, 2: 1.125, 3: 1.0, 4: 0.875, 5: 0.75}

# Hard clamps: whatever the level or the constants, the detector can never be switched off or made hair-triggered.
TORQUE_MIN_NM, TORQUE_MAX_NM = 1.5, 2.5
RATE_TORQUE_MIN_NM, RATE_TORQUE_MAX_NM = 0.75, 1.25
RATE_MIN_DPS, RATE_MAX_DPS = 25.0, 35.0
QUIET_TORQUE_MIN_NM = 0.5            # and never below this, so a resting wheel can always count as quiet
QUIET_TORQUE_MAX_FRACTION = 0.5     # the release torque is at most half the fire torque, so release always sits below fire


def sanitize_level(value):
  """A level 1..5 from an int or a string holding one (a file value). Anything else is the default, never "off"."""
  if isinstance(value, bool):
    return SENSITIVITY_DEFAULT
  if isinstance(value, str):
    text = value.strip()
    if not text or not (text.isascii() and text.isdigit()):
      return SENSITIVITY_DEFAULT
    value = int(text)
  if isinstance(value, float):
    if not math.isfinite(value) or value != int(value):
      return SENSITIVITY_DEFAULT
    value = int(value)
  if isinstance(value, int) and value in SENSITIVITY_LEVELS:
    return value
  return SENSITIVITY_DEFAULT


def read_sensitivity_level():
  """The saved level (file-backed Override Sensitivity), read once when a controller is built. Missing, unreadable or bad is the default (2)."""
  try:
    return sanitize_level(params.get_str(params.PreApParamKeys.OVERRIDE_SENSITIVITY, ""))
  except Exception:
    return SENSITIVITY_DEFAULT


def _clamp(value, low, high):
  return max(low, min(high, value))


class Thresholds:
  """The numbers one detector uses. Built by override_thresholds() only."""
  __slots__ = ("torque_nm", "rate_torque_nm", "rate_dps", "quiet_torque_nm", "quiet_rate_dps")

  def __init__(self, torque_nm, rate_torque_nm, rate_dps, quiet_torque_nm, quiet_rate_dps):
    self.torque_nm = torque_nm
    self.rate_torque_nm = rate_torque_nm
    self.rate_dps = rate_dps
    self.quiet_torque_nm = quiet_torque_nm
    self.quiet_rate_dps = quiet_rate_dps

  def as_tuple(self):
    return (self.torque_nm, self.rate_torque_nm, self.rate_dps, self.quiet_torque_nm, self.quiet_rate_dps)


def override_thresholds(level=SENSITIVITY_DEFAULT):
  """Torque, torque-with-rate, rate and release thresholds for a level (a bad level is the default).

  s = SENSITIVITY_SCALE[level]. Fire torque = 2.0 Nm * s (2.5 .. 1.5), the torque that must go with a fast wheel = 1.0 Nm * s,
  fire rate = 30 deg/s * (1 + (s - 1) / 2) (about 33.8 .. 26.3), release torque = 1.0 Nm * s (0.5 Nm at least, at most half the fire torque), release rate
  = the fire rate. The module constants are read here, so level 3 is always exactly them. Every value is clamped.
  """
  s = SENSITIVITY_SCALE[sanitize_level(level)]
  torque = _clamp(OVERRIDE_TORQUE_NM * s, TORQUE_MIN_NM, TORQUE_MAX_NM)
  rate_torque = _clamp(OVERRIDE_RATE_TORQUE_NM * s, RATE_TORQUE_MIN_NM, RATE_TORQUE_MAX_NM)
  rate = _clamp(OVERRIDE_RATE_DPS * (1.0 + (s - 1.0) / 2.0), RATE_MIN_DPS, RATE_MAX_DPS)
  quiet_torque = _clamp(QUIET_TORQUE_NM * s, QUIET_TORQUE_MIN_NM, torque * QUIET_TORQUE_MAX_FRACTION)
  quiet_rate = _clamp(QUIET_RATE_DPS * (1.0 + (s - 1.0) / 2.0), RATE_MIN_DPS, rate)
  return Thresholds(torque, rate_torque, rate, quiet_torque, quiet_rate)


class DriverOverrideDetector:
  """One update per CarController step. `signal` is the trigger holding this step."""

  def __init__(self, level=SENSITIVITY_DEFAULT):
    self.level = sanitize_level(level)
    self.thresholds = override_thresholds(self.level)
    self.signal = SIGNAL_NONE
    self.quiet = True
    self._torque_frames = 0
    self._rate_frames = 0
    self._error_frames = 0

  def reset(self):
    self.signal = SIGNAL_NONE
    self.quiet = True
    self._torque_frames = 0
    self._rate_frames = 0
    self._error_frames = 0

  def update(self, engaged, hands_on_level, torque_nm, rate_dps, angle_deg, command_deg=None):
    """Return the signal that holds this frame (SIGNAL_NONE when none does).

    engaged: a lateral session is on. Not engaged clears every counter.
    command_deg: the angle of the last ANGLE (type 1) frame this controller sent,
    or None when the last frame was not an active command. The angle error signal
    only exists while openpilot is actually steering.
    """
    torque = float(torque_nm)
    rate = float(rate_dps)
    abs_torque = abs(torque)
    abs_rate = abs(rate)
    hands = int(hands_on_level or 0)
    th = self.thresholds
    self.quiet = hands <= QUIET_HANDS_MAX_LEVEL and abs_torque < th.quiet_torque_nm and abs_rate < th.quiet_rate_dps
    if not engaged:
      self._torque_frames = self._rate_frames = self._error_frames = 0
      self.signal = SIGNAL_NONE
      return self.signal

    self._torque_frames = self._torque_frames + 1 if abs_torque >= th.torque_nm else 0
    self._rate_frames = (self._rate_frames + 1
                         if (abs_rate >= th.rate_dps and abs_torque >= th.rate_torque_nm) else 0)
    away = False
    if command_deg is not None:
      error = float(command_deg) - float(angle_deg)
      away = abs(error) >= OVERRIDE_ERROR_DEG and error * rate < 0.0 and abs_rate >= OVERRIDE_ERROR_RATE_DPS
    self._error_frames = self._error_frames + 1 if away else 0

    if hands >= OVERRIDE_HANDS_LEVEL:
      self.signal = SIGNAL_HANDS
    elif self._torque_frames >= OVERRIDE_TORQUE_FRAMES:
      self.signal = SIGNAL_TORQUE
    elif self._rate_frames >= OVERRIDE_RATE_FRAMES:
      self.signal = SIGNAL_RATE
    elif self._error_frames >= OVERRIDE_ERROR_FRAMES:
      self.signal = SIGNAL_ANGLE_ERROR
    else:
      self.signal = SIGNAL_NONE
    return self.signal
