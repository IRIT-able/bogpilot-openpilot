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
# !! Read EAC_DRIVER_ORIGIN_ERRORS below before adding anything here. !!
# Request-origin only: the EPAS is saying "your request was bigger than I will act on". Nothing about the driver.
EAC_LIMIT_ERRORS = (
  "EAC_ERROR_HIGH_ANGLE_REQ",          # 6  requested angle above the request validator
  "EAC_ERROR_HIGH_ANGLE_RATE_REQ",     # 7  requested rate above the request validator
  "EAC_ERROR_HIGH_ANGLE_SAFETY",       # 8  requested angle above the safety validator
  "EAC_ERROR_HIGH_MMOT_SAFETY",        # 10 assist motor torque limit -- the reported mid-corner straighten
  "EAC_ERROR_LOW_ASSIST",              # 12 reduced assist available
)

# Driver-origin. These are how the EPAS reports that something other than openpilot is moving the wheel, so they
# MUST keep blocking: blocking is what sets EpasGate.signal = SIGNAL_EPAS and steerFaultTemporary, and that is the
# wheel-turn override. Putting them in EAC_LIMIT_ERRORS removes the override -- hauling on the wheel then does not
# disengage, which was a regression driven on this build. Do not "unify" these with the list above.
#   11 HIGH_TORSION_SAFETY  the torsion bar measures DRIVER torque, not the motor's. This is the driver fighting
#                           the wheel, and it is the code the regression was reported against.
#    9 HIGH_ANGLE_RATE_SAFETY  the wheel moved faster than commanded, so something else moved it. NAP uses exactly
#                           this as its fast override: "FSD disengages using union of handsOnLevel (slow
#                           overrides) and high angle rate faults (fast overrides, high speed)"
#                           (opendbc tesla/carstate.py). NAP pairs it with EAC_INHIBITED; here the code itself is
#                           kept blocking, which is the more cautious of the two.
#    5 MAX_STEER_DELTA     the command is far from the measured wheel. _ap1_limited_angle already clips the
#                           command to measured +/- 20 deg, so openpilot's own request cannot open a gap this big:
#                           the wheel was moved. Treated as driver-origin for that reason.
EAC_DRIVER_ORIGIN_ERRORS = (
  "EAR_ERROR_MAX_STEER_DELTA",         # 5
  "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY",  # 9
  "EAC_ERROR_HIGH_TORSION_SAFETY",     # 11
)

# Pre-AP took this branch (is_ap1_family is False for TESLA_PREAP_MODELS), so before the limit codes moved out it
# warned on everything except IDLE and HANDS_ON -- code 6 included, which is stricter than AP1.
_BENIGN = ("EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON") + EAC_LIMIT_ERRORS
# AP1: idle, latched code 3 so an angle can be sent again, and the limit codes.
_AP1_NOT_WARNING = ("EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON") + EAC_LIMIT_ERRORS
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
