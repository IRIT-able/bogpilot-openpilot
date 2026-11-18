"""An EPAS limit code must not release the wheel mid-corner.

Reported on this build, not reproducible on NAP: halfway through a left turn the EPAS hit a torque limit, all
lateral control stopped, and the wheel straightened on its own under caster — the car carried straight on toward
the curb. The EPAS had clamped the request and was still steering; it was openpilot that let go.

Two independent gates did it, both keyed on EPAS_eacErrorCode regardless of EPAS_eacStatus:
  - epas_gate.epas_not_ready: anything outside EAC_OK_ERRORS -> the gate blocks -> CarController sends type NONE,
    which is no angle control at all, so the EPAS releases;
  - steer_fault.steer_fault_temporary: anything outside the allowed set -> CarState.steerFaultTemporary ->
    steerTempUnavailable -> lateral inactive. Pre-AP took the *stricter* `_BENIGN` branch, because
    is_ap1_family("TESLA_PREAP_MODELS") is False, so on Pre-AP every code except IDLE and HANDS_ON did this.

NAP Pre-AP (opendbc tesla/preap/carstate.py:62-70) only calls it a rejection when the status has gone
EAC_INHIBITED as well, and then only for the four angle/rate validators; the torque codes are not in its set at
all. The status check is what decides, not the code. That is the rule these tests hold BogPilot to.
"""
import pytest

from opendbc.car.tesla_ap1.epas_gate import EAC_OK_ERRORS, EpasGate, epas_not_ready
from opendbc.car.tesla_ap1.override import SIGNAL_EPAS, SIGNAL_NONE
from opendbc.car.tesla_ap1.steer_fault import (EAC_DRIVER_ORIGIN_ERRORS, EAC_LIMIT_ERRORS,
                                               steer_fault_temporary)

# The code the reported turn hit (10), its sibling (11), and the reduced-assist code (12).
# The request-origin torque/assist codes. NOT 11 HIGH_TORSION_SAFETY: the torsion bar measures driver torque, so
# that one is the wheel-turn override (see the driver-origin tests at the bottom).
TORQUE_CODES = ("EAC_ERROR_HIGH_MMOT_SAFETY", "EAC_ERROR_LOW_ASSIST")
# Codes that must STILL be faults: not "I clamped your request".
STILL_FAULTS = ("EAC_ERROR_TMP_FAULT", "EAC_ERROR_PINION_VEL_DIFF", "EAC_ERROR_EPB_INHIBIT", "SNA",
                "EAC_ERROR_MIN_SPEED", "EAC_ERROR_MAX_SPEED")
STEERING_STATUSES = ("EAC_ACTIVE", "EAC_AVAILABLE")
REFUSING_STATUSES = ("EAC_INHIBITED", "EAC_FAULT", "EAC_SNA")


# ---- the reported failure ------------------------------------------------------------------------

@pytest.mark.parametrize("code", TORQUE_CODES)
@pytest.mark.parametrize("status", STEERING_STATUSES)
def test_a_torque_limit_does_not_block_the_gate(status, code):
  """The gate staying open is what keeps 0x488 on ANGLE instead of NONE, so the wheel is still held."""
  assert not epas_not_ready(status, code), (status, code)
  g = EpasGate()
  assert g.update(True, status, code) is False, (status, code)


@pytest.mark.parametrize("code", TORQUE_CODES)
@pytest.mark.parametrize("ap1", [True, False])
def test_a_torque_limit_is_not_a_temporary_steer_fault(ap1, code):
  """Pre-AP is the ap1=False branch, which used to warn on everything but IDLE and HANDS_ON."""
  assert not steer_fault_temporary(code, ap1), (ap1, code)


@pytest.mark.parametrize("code", TORQUE_CODES)
def test_a_torque_limit_mid_corner_keeps_steering_for_as_long_as_it_lasts(code):
  """A sustained limit must not start blocking after N frames either."""
  g = EpasGate()
  for frame in range(500):
    assert g.update(True, "EAC_ACTIVE", code) is False, (frame, code)


# ---- the whole limit family -----------------------------------------------------------------------

@pytest.mark.parametrize("code", EAC_LIMIT_ERRORS)
@pytest.mark.parametrize("status", STEERING_STATUSES)
def test_no_limit_code_blocks_while_the_epas_is_still_steering(status, code):
  assert not epas_not_ready(status, code), (status, code)
  assert not steer_fault_temporary(code, False), code
  assert not steer_fault_temporary(code, True), code


def test_the_limit_family_is_request_origin_only():
  assert set(TORQUE_CODES) <= set(EAC_LIMIT_ERRORS)
  assert {"EAC_ERROR_HIGH_ANGLE_REQ", "EAC_ERROR_HIGH_ANGLE_RATE_REQ",
          "EAC_ERROR_HIGH_ANGLE_SAFETY"} <= set(EAC_LIMIT_ERRORS)


# ---- the wheel-turn override, which making these codes "clean" removed -----------------------------

def test_the_two_lists_are_disjoint():
  assert not (set(EAC_LIMIT_ERRORS) & set(EAC_DRIVER_ORIGIN_ERRORS))


@pytest.mark.parametrize("code", EAC_DRIVER_ORIGIN_ERRORS)
@pytest.mark.parametrize("status", STEERING_STATUSES)
def test_a_driver_origin_code_still_blocks_the_gate(status, code):
  """Blocking is the override: it is what sets SIGNAL_EPAS and steerFaultTemporary."""
  assert epas_not_ready(status, code), (status, code)
  assert code not in EAC_OK_ERRORS, code


@pytest.mark.parametrize("code", EAC_DRIVER_ORIGIN_ERRORS)
def test_a_driver_origin_code_still_raises_the_epas_override_signal(code):
  g = EpasGate()
  assert g.update(True, "EAC_ACTIVE", code) is True, code
  assert g.signal == SIGNAL_EPAS, code


@pytest.mark.parametrize("code", EAC_DRIVER_ORIGIN_ERRORS)
@pytest.mark.parametrize("ap1", [True, False])
def test_a_driver_origin_code_is_still_a_temporary_steer_fault(ap1, code):
  """steerFaultTemporary -> steerTempUnavailable is the disengage the driver feels on a wheel turn."""
  assert steer_fault_temporary(code, ap1), (ap1, code)


def test_turning_the_wheel_past_the_torsion_limit_disengages():
  """The exact regression: code 11 is driver torque, and it must not be treated as a request limit."""
  code = "EAC_ERROR_HIGH_TORSION_SAFETY"
  assert code in EAC_DRIVER_ORIGIN_ERRORS
  assert code not in EAC_LIMIT_ERRORS
  assert epas_not_ready("EAC_ACTIVE", code)
  assert steer_fault_temporary(code, False) and steer_fault_temporary(code, True)
  g = EpasGate()
  assert g.update(True, "EAC_ACTIVE", code) is True
  assert g.signal == SIGNAL_EPAS


def test_a_limit_code_raises_no_override_signal():
  """The other half: a motor-torque limit must not look like a driver override."""
  g = EpasGate()
  assert g.update(True, "EAC_ACTIVE", "EAC_ERROR_HIGH_MMOT_SAFETY") is False
  assert g.signal == SIGNAL_NONE


# ---- what must NOT have changed -------------------------------------------------------------------

@pytest.mark.parametrize("status", REFUSING_STATUSES)
@pytest.mark.parametrize("code", EAC_LIMIT_ERRORS)
def test_a_refusing_status_still_blocks_whatever_the_code(status, code):
  """This is how NAP decides a real rejection, and it is untouched: the EPAS has actually stopped."""
  assert epas_not_ready(status, code), (status, code)
  g = EpasGate()
  assert g.update(True, status, code) is True, (status, code)


@pytest.mark.parametrize("code", STILL_FAULTS)
def test_the_real_faults_still_block(code):
  assert epas_not_ready("EAC_ACTIVE", code), code
  assert code not in EAC_OK_ERRORS, code
  assert steer_fault_temporary(code, False), code


def test_tmp_fault_is_still_a_fault_so_its_recovery_path_is_unchanged():
  """epas_gate.EpasTmpFaultRecovery is the only thing that clears code 4; it must still see it as a fault."""
  assert "EAC_ERROR_TMP_FAULT" not in EAC_OK_ERRORS
  assert steer_fault_temporary("EAC_ERROR_TMP_FAULT", True)
  assert steer_fault_temporary("EAC_ERROR_TMP_FAULT", False)


def test_an_eac_fault_flag_still_blocks_on_a_limit_code():
  assert epas_not_ready("EAC_ACTIVE", "EAC_ERROR_HIGH_MMOT_SAFETY", eac_fault=True)


def test_an_undecoded_state_still_blocks():
  assert epas_not_ready(None, "EAC_ERROR_HIGH_MMOT_SAFETY")
  assert epas_not_ready("EAC_ACTIVE", None)


def test_an_unknown_code_is_still_treated_as_a_fault():
  """Fail closed: a name not in the DBC is not assumed to be a limit."""
  assert epas_not_ready("EAC_ACTIVE", "EAC_ERROR_SOMETHING_NEW")
  assert steer_fault_temporary("EAC_ERROR_SOMETHING_NEW", False)


def test_idle_and_hands_on_are_still_clean():
  for code in ("EAC_ERROR_IDLE", "EAC_ERROR_HANDS_ON"):
    assert not epas_not_ready("EAC_ACTIVE", code), code
    assert not steer_fault_temporary(code, True), code
