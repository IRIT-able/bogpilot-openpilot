"""Temporary steer-fault gate. No CAN, no parser.

Tinkla carstate.py (501c7de) sets steerWarning unless the EPAS error name
is EAC_ERROR_IDLE. An unknown name is a warning. HANDS_ON (3) is a warning
on that tree. EAC_FAULT is steerFaultPermanent and is not decided here.

AP1 does not treat EAC_ERROR_HIGH_ANGLE_REQ (6) or EAC_ERROR_HANDS_ON (3)
as a temporary fault. Both stay latched after the wheel is released, so
warning on them keeps lat_active false and 0x488 never returns to ANGLE.
The hands-on pause is hands_on_level, not this flag.

The Tinkla rule ("anything that is not IDLE is a warning") is not kept for
the codes that only mean the EPAS clamped the request while still steering:
see EAC_LIMIT_ERRORS below. Everything else non-idle still warns.
"""

# The EPAS clamped the request and is STILL STEERING. These are limits, not faults: the request was larger than
# the EPAS would act on, so it acted on as much as it would and said so. Treating them as faults released the
# wheel mid-corner -- lateral stopped and the wheel self-centred under caster, which on a left turn means going
# straight on. Holding the command instead finishes the turn at whatever the EPAS will give.
#
# This is NAP's rule. NAP Pre-AP (opendbc tesla/preap/carstate.py:62-70) only calls the EPAS a rejection when the
# *status* has gone EAC_INHIBITED as well, and even then only for the four angle/rate validators:
#   epas_rejecting = eac_status == "EAC_INHIBITED" and eac_error_code in (
#     "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HIGH_ANGLE_RATE_REQ",
#     "EAC_ERROR_HIGH_ANGLE_SAFETY", "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
# The torque codes (10 HIGH_MMOT_SAFETY, 11 HIGH_TORSION_SAFETY) and 12 LOW_ASSIST are not in NAP's set at all.
# An error code alone never stops NAP from commanding; only the status does. The status is still checked here and
# in epas_gate (EAC_INHIBITED / EAC_FAULT / EAC_SNA all still block), so the INHIBITED case is unchanged.
#
# Codes deliberately left as faults, because they are not "I clamped your request": 4 TMP_FAULT (latched, with its
# own recovery in epas_gate.EpasTmpFaultRecovery), 13 PINION_VEL_DIFF (the two angle sensors disagree, an
# integrity fault), 14 EPB_INHIBIT, 15 SNA, and 1 MIN_SPEED / 2 MAX_SPEED (outside the EPAS's speed window, where
# commanding achieves nothing). And the driver-origin codes, which are their own list below.
# !! Before adding anything here, read "why tolerating a code can remove the disengage" below. !!
#
# Request-origin AND safe to keep commanding through: the EPAS is saying "your request wanted more torque than I
# will give", nothing about the driver, and nothing that the hard disengage depends on.
EAC_LIMIT_ERRORS = (
  "EAC_ERROR_HIGH_MMOT_SAFETY",        # 10 assist motor torque limit -- the reported mid-corner straighten
  "EAC_ERROR_LOW_ASSIST",              # 12 reduced assist available
)

# Why tolerating a code can remove the disengage, learned the hard way on this build:
#
# The hard disengage is preap/engagement.py preap_epas_refusing, mirrored by tesla_preap_epas_blocking in the
# panda, and it is numeric, so nothing in this file feeds it directly:
#     hands_on_level >= 3 or (eac_status == INHIBITED and 6 <= eac_error_code <= 9)
# The EPAS only reaches INHIBITED when openpilot STOPS sending ANGLE ("EAC_INHIBITED is the normal Pre-AP idle
# state", NAP tesla/preap/carstate.py:56). Blocking the gate is what stops the ANGLE frames. So tolerating codes
# 6-9 kept the gate open, kept ANGLE flowing, kept the EPAS ACTIVE, and the INHIBITED half of that condition could
# then never become true -- the disengage was intact and unreachable. Driven on this build: turning the wheel no
# longer handed over. Codes 6, 7 and 8 are therefore NOT tolerated here even though they are request-origin.
#
# Driver-origin, and additionally the codes the disengage condition is written in terms of. These must keep
# blocking, both because blocking is the override signal (EpasGate.signal = SIGNAL_EPAS, and steerFaultTemporary ->
# steerTempUnavailable, a SOFT_DISABLE) and because blocking is what lets the EPAS reach INHIBITED at all:
#   11 HIGH_TORSION_SAFETY  the torsion bar measures DRIVER torque, not the motor's: the driver fighting the wheel.
#    9 HIGH_ANGLE_RATE_SAFETY  the wheel moved faster than commanded, so something else moved it. NAP uses exactly
#                           this as its fast override ("FSD disengages using union of handsOnLevel (slow
#                           overrides) and high angle rate faults (fast overrides, high speed)").
#    8 HIGH_ANGLE_SAFETY  }  request-origin, but inside the 6..9 window the disengage is written in terms of, so
#    7 HIGH_ANGLE_RATE_REQ}  tolerating them removes the INHIBITED precondition as described above.
#                            (6 HIGH_ANGLE_REQ is the exception: the gate has tolerated it since before this work,
#                             so it never produced INHIBITED and is left exactly as it was.)
#    5 MAX_STEER_DELTA     the command is far from the measured wheel. _ap1_limited_angle clips the command to the
#                           measured wheel +/- 20 deg, so openpilot cannot open a gap that big: the wheel moved.
# Codes 7, 8, 9 of that window. Code 6 is NOT here: EAC_OK_ERRORS has tolerated it since before any of this work,
# so code 6 never blocked the gate, never produced INHIBITED, and never contributed to the disengage. Only 7, 8
# and 9 did, and only those three needed putting back.
EAC_DISENGAGE_WINDOW = ("EAC_ERROR_HIGH_ANGLE_RATE_REQ", "EAC_ERROR_HIGH_ANGLE_SAFETY",
                        "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")
EAC_DRIVER_ORIGIN_ERRORS = ("EAR_ERROR_MAX_STEER_DELTA", "EAC_ERROR_HIGH_TORSION_SAFETY") + EAC_DISENGAGE_WINDOW

# Pre-AP took this branch (is_ap1_family is False for TESLA_PREAP_MODELS), so before the limit codes moved out it
# warned on everything except IDLE and HANDS_ON -- code 6 included, which is stricter than AP1.
_BENIGN = ("EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON") + EAC_LIMIT_ERRORS
# AP1: idle, code 6 and latched code 3 so an angle can be sent again, plus the limit codes. Code 6 here is
# unchanged from before any of this work.
_AP1_NOT_WARNING = ("EAC_ERROR_IDLE", "EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HANDS_ON") + EAC_LIMIT_ERRORS
# Latched names that should be commanded as ANGLE once hands are below 2.
AP1_LATCHED_ANGLE_ERRORS = ("EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HANDS_ON")
# AP1: EAC_ERROR_TMP_FAULT (4) stays a temporary fault here, so latActive stays false while it is set.
# The EPAS keeps it latched while AVAILABLE and clears it only when a new EAC session is commanded
# (pre-AP 0.9.7-era capture: AVAILABLE + code 4 from boot, EAC_ACTIVE + IDLE on the first ANGLE frame).
# epas_gate.EpasTmpFaultRecovery sends that bounded measured-wheel request; nothing else does.
AP1_RECOVERABLE_TMP_FAULT = "EAC_ERROR_TMP_FAULT"


# Belt and braces: a driver-origin code must never reach the tolerated sets, however they are edited later.
assert not (set(EAC_LIMIT_ERRORS) & set(EAC_DRIVER_ORIGIN_ERRORS)), "driver-origin code in the limit list"
assert not (set(_BENIGN) & set(EAC_DRIVER_ORIGIN_ERRORS)), "driver-origin code tolerated on Pre-AP"
assert not (set(_AP1_NOT_WARNING) & set(EAC_DRIVER_ORIGIN_ERRORS)), "driver-origin code tolerated on AP1"


def steer_fault_temporary(steer_warning, ap1):
  """True when this error name should set CarState.steerFaultTemporary."""
  allowed = _AP1_NOT_WARNING if ap1 else _BENIGN
  return steer_warning not in allowed
