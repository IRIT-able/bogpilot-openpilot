"""EPAS refusal gate for the legacy Teslas. No CAN.

When the EPAS is INHIBITED, FAULT or SNA, or reports an error other than the
two codes that stay latched after a normal hand-back, it is not following
angle commands. Sending ANGLE anyway (or holding the last target) does not move
the wheel and, with the EPB watching, ended in a permanent "LKAS Fault".
This gate says so on the first frame it sees it, and the controller stops
commanding at once: type NONE at the measured wheel, never the old target.

Not blocking: EAC_AVAILABLE and EAC_ACTIVE with error IDLE, HANDS_ON (3) or
HIGH_ANGLE_REQ (6). Those two are latched by the EPAS after a hands-on or a
large request and clear once openpilot commands ANGLE at the measured wheel
(hso / actuator_plan hold). Every other error blocks, including
HIGH_ANGLE_RATE_SAFETY (9).

Recovery needs EPAS_CLEAN_FRAMES in a row of a clean state (about 7 EPAS
samples at 25 Hz) while the lateral session is still on. A lateral session
that ended starts clean: the gate does not carry a block across it, it
re-reads the EPAS on the next engagement. `resumed` is true for the one step the block
ends so the controller can restart the measured-angle soft-start.

Pre-AP has no AP ECU and its EPAS status flaps outside an engaged session, so
it builds the gate with arm_on_active: nothing blocks until the EPAS has
reported EAC_ACTIVE in this lateral session.
"""
import math
from opendbc.car.tesla_ap1.override import SIGNAL_EPAS, SIGNAL_NONE
from opendbc.car.tesla_ap1.steer_fault import EAC_LIMIT_ERRORS

EAC_ACTIVE = "EAC_ACTIVE"
EAC_BLOCKING_STATUS = ("EAC_INHIBITED", "EAC_FAULT", "EAC_SNA")
# Idle, hands-on, and the codes that only mean the EPAS clamped the request while still steering
# (steer_fault.EAC_LIMIT_ERRORS). Blocking on those sent type NONE mid-corner and the wheel self-centred.
# The status check below is unchanged: EAC_INHIBITED / EAC_FAULT / EAC_SNA still block, which is how NAP
# decides a real rejection (tesla/preap/carstate.py:66).
# Idle, hands-on and code 6 exactly as before, plus the two codes that only mean the assist motor ran out of
# torque (steer_fault.EAC_LIMIT_ERRORS). Codes 6-9 must keep blocking here even though 6-8 are request-origin:
# blocking is what stops the ANGLE frames, and the EPAS only reaches INHIBITED when they stop, which the hard
# disengage in preap/engagement.py preap_epas_refusing requires. See the long note in steer_fault.py.
EAC_OK_ERRORS = ("EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON", "EAC_ERROR_HIGH_ANGLE_REQ") + EAC_LIMIT_ERRORS

EPAS_CLEAN_FRAMES = 30   # 0.3 s at 100 Hz


def epas_not_ready(eac_status, steer_warning, eac_fault=False):
  """True when this EPAS state must not be commanded. None (no frame decoded yet) is not ready."""
  if eac_fault or eac_status is None or steer_warning is None:
    return True
  return eac_status in EAC_BLOCKING_STATUS or steer_warning not in EAC_OK_ERRORS


class EpasGate:
  def __init__(self, arm_on_active=False):
    self.arm_on_active = arm_on_active
    self.reset()

  def reset(self):
    self.blocked = False
    self.resumed = False
    self.clean_frames = 0
    self.armed = not self.arm_on_active

  @property
  def signal(self):
    return SIGNAL_EPAS if self.blocked else SIGNAL_NONE

  def update(self, lat_session, eac_status, steer_warning, eac_fault=False, refusing=False):
    """One call per control step. Returns True while commands must stay off.

    refusing: the panda is rejecting ANGLE (hands at level 3, or INHIBITED with error 6..9). That arms the gate even before the EPAS
    has been ACTIVE, so a refusal at the start of a session is waited out with the same clean frames as any other.
    """
    self.resumed = False
    if not lat_session:
      self.reset()
      return False
    if eac_status == EAC_ACTIVE or refusing:
      self.armed = True
    if self.armed and epas_not_ready(eac_status, steer_warning, eac_fault):
      self.blocked = True
      self.clean_frames = 0
    elif self.blocked:
      self.clean_frames += 1
      if self.clean_frames >= EPAS_CLEAN_FRAMES:
        self.blocked = False
        self.clean_frames = 0
        self.resumed = True
    return self.blocked


# ---- AP1 EAC_ERROR_TMP_FAULT (4) recovery ---------------------------------------------------------
#
# The EPAS latches code 4 the same way it latches HANDS_ON (3) and HIGH_ANGLE_REQ (6): it stays in
# EPAS_eacErrorCode while the EPAS is AVAILABLE and clears only when a new EAC session is commanded.
# Code 4 is a temporary fault (steer_fault.py), so latActive stays false, nothing sends ANGLE and the
# latch never clears: lateral is lost until a car power cycle. Tinkla (BogGyver 501c7de
# carcontroller.py:86-106) and BogPilot 0.9.7 (actuator_plan.py ap1_hold_measured_angle) kept sending
# ANGLE at the measured wheel while engaged, which is what clears a latched code.
#
# Divergence (stricter than both references). All of the recovery rules live in this class; the panda
# (tesla_ap1.h) only allows ANGLE with code 4 while the EPAS is AVAILABLE or ACTIVE and EPB_epasEACAllow (0x214)
# was 1 within the last 250 ms. It does not limit retries, so every limit below is enforced here. This code needs
# that panda rule: a panda without it drops these frames, and after 100 ms without an accepted ANGLE frame it
# forwards the stock DAS 0x488 again, so the two must ship together (a package-time check enforces it).
#  1. Onset: a code 4 that appears while the EPAS is ACTIVE still drops lateral on that frame, as before
#     (steer_fault.py makes it a temporary fault, EpasGate blocks it, the panda refuses it while INHIBITED).
#     Nothing here runs while latActive is on.
#  2. Start: only from EAC_AVAILABLE with a latched code 4, while openpilot is engaged and latActive is off,
#     and only after that state has held for ERR4_SETTLE_FRAMES in a row (see the value below).
#  3. Each request: ANGLE at the measured wheel only (actuator_plan hold, existing rate limit and clip),
#     at most ERR4_PROBE_FRAMES (0.3 s).
#  4. Spacing and cap: a request never starts less than ERR4_WAIT_FRAMES (2 s) after the previous request
#     started, whatever that request's result (cleared, timeout or abort); one clock, counted from the start.
#     After a timeout or an abort there is also the ERR4_WAIT_FRAMES wait and then the settle again. At most
#     ERR4_MAX_PROBES (3) per engagement. A new engagement starts the count and the clock from zero.
#  5. Abort on the frame it is seen: INHIBITED / FAULT / SNA, EAC_FAULT, hands level 2+, a driver override,
#     EPB_epasEACAllow other than 1, any error code other than 4 and IDLE, or vEgo below MIN_ERR4_PROBE_SPEED.
#  6. EPB: a request needs a 0x214 sample with EPB_epasEACAllow 1 no older than ERR4_EPB_FRESH_FRAMES, as the
#     panda does. A car without 0x214 (or with a stale one) gets no request, so it behaves as before this
#     recovery. Everywhere else in this port 0x214 stays optional.
#  7. Speed: no request at a standstill or below MIN_ERR4_PROBE_SPEED; a running request ends the moment vEgo
#     drops below it.
# A clear (error IDLE) hands over to the normal path: steerFaultTemporary drops, latActive returns and
# the controller restarts from the measured wheel through the engage soft-start.

ERR4_PROBE_FRAMES = 30    # 0.3 s at 100 Hz
ERR4_WAIT_FRAMES = 200    # 2 s at 100 Hz
ERR4_MAX_PROBES = 3       # per engagement
# 0.5 s at 100 Hz. In the recorded event the EPAS went ACTIVE/4 -> INHIBITED/4 (39 ms) -> AVAILABLE/4
# (118 ms after the onset). 0x370 comes at 25 Hz, so 50 frames is about 12 EPAS samples in a row that
# all report AVAILABLE + code 4: past that transient, and short next to the 2 s spacing.
ERR4_SETTLE_FRAMES = 50
# EPB_epasEACAllow (0x214, 10 Hz) must be 1 and no older than this. The panda allows code 4 only with an EPB
# allow of 1 received in the last 250 ms (tesla_ap1.h); 200 ms keeps the car side inside that window.
ERR4_EPB_FRESH_FRAMES = 20
# Lowest vEgo (m/s, about 5 mph) for a code 4 request. A replay of the recorded drive showed a request
# starting during a creep from a stop at 0.24 m/s, below controlsd's 0.3 m/s lateral floor (minSteerSpeed is 0
# here, so that floor did not apply to this path). No measured-wheel request at a standstill or a creep.
MIN_ERR4_PROBE_SPEED = 2.2
EAC_AVAILABLE = "EAC_AVAILABLE"
EAC_TMP_FAULT = "EAC_ERROR_TMP_FAULT"
EAC_IDLE = "EAC_ERROR_IDLE"
# While a request is running the EPAS may go ACTIVE before it drops the code.
_PROBE_STATUS = (EAC_AVAILABLE, EAC_ACTIVE)
_PROBE_ERRORS = (EAC_TMP_FAULT, EAC_IDLE)

R_IDLE, R_PROBE, R_WAIT, R_DONE = "idle", "probe", "wait", "done"


class EpasTmpFaultRecovery:
  """Bounded measured-wheel ANGLE request for a latched EAC_ERROR_TMP_FAULT. One update per control step."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.state = R_IDLE
    self.probes = 0
    self.frames = 0
    self.cleared = False
    self.settle = 0
    self.since_start = None   # control steps since the last request started; None before the first one
    self.last_result = None   # "cleared", "timeout" or "abort:<reason>" for the last request

  @property
  def active(self):
    return self.state == R_PROBE

  @staticmethod
  def abort_reason(eac_status, steer_warning, hands_on_level, epb_eac_allow, eac_fault, driver_yield, v_ego=None):
    """Why a request may not run this frame. None when it may.

    epb_eac_allow: the last EPB_epasEACAllow, or None when 0x214 is missing or older than ERR4_EPB_FRESH_FRAMES.
    v_ego: m/s; None (unknown) blocks.
    """
    if eac_fault or eac_status == "EAC_FAULT":
      return "eac_fault"
    if epb_eac_allow != 1:
      return "epb_allow"
    if v_ego is None or not math.isfinite(v_ego) or v_ego < MIN_ERR4_PROBE_SPEED:
      return "speed"
    if eac_status not in _PROBE_STATUS:
      return "status"
    if hands_on_level is None or hands_on_level >= 2:
      return "hands"
    if driver_yield:
      return "override"
    if steer_warning not in _PROBE_ERRORS:
      return "error_code"
    return None

  def _end(self, result):
    self.last_result = result
    self.frames = 0
    self.state = R_DONE if self.probes >= ERR4_MAX_PROBES else R_WAIT

  def update(self, engaged, lat_session, eac_status, steer_warning, hands_on_level, epb_eac_allow,
             eac_fault=False, driver_yield=False, v_ego=None):
    """True on the frames that must carry an ANGLE request at the measured wheel."""
    if not engaged:
      was_probe = self.state == R_PROBE
      self.reset()
      if was_probe:
        self.last_result = "abort:disengaged"
      return False
    if self.since_start is not None:
      self.since_start += 1
    reason = self.abort_reason(eac_status, steer_warning, hands_on_level, epb_eac_allow, eac_fault, driver_yield, v_ego)
    if self.state == R_PROBE:
      if reason is not None:
        self._end("abort:" + reason)
        return False
      if steer_warning == EAC_IDLE:
        self.cleared = True
      self.frames += 1
      # After a clear, hold until latActive is back (the next soft-start takes over), never past the window.
      if self.frames >= ERR4_PROBE_FRAMES or (self.cleared and lat_session):
        self._end("cleared" if self.cleared else "timeout")
        if self.cleared:
          self.state = R_IDLE if self.probes < ERR4_MAX_PROBES else R_DONE
        return False
      return True
    if self.state == R_WAIT:
      self.frames += 1
      if self.frames < ERR4_WAIT_FRAMES:
        return False
      self.state = R_IDLE
      self.frames = 0
      self.settle = 0
    if self.state == R_DONE:
      return False
    # R_IDLE: start only on a latched code 4 with an AVAILABLE EPAS and latActive held off by it, held for the settle.
    if reason is None and not lat_session and eac_status == EAC_AVAILABLE and steer_warning == EAC_TMP_FAULT:
      self.settle += 1
    else:
      self.settle = 0
    spaced = self.since_start is None or self.since_start >= ERR4_WAIT_FRAMES
    if self.settle >= ERR4_SETTLE_FRAMES and spaced:
      self.settle = 0
      self.since_start = 0
      self.state = R_PROBE
      self.probes += 1
      self.frames = 1
      self.cleared = False
      return True
    return False
