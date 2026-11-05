"""Gates, validation and packing for the NAP-style pedal calibration tool."""
import pathlib

from openpilot.selfdrive.car.tesla.preap.calibrate_pedal import (
  BRAKE_MESSAGE_ID,
  STALE_LIMIT_MS,
  CarSignals,
  gate_failure,
  pack_gas_command,
  validate_result,
)

FRESH = dict.fromkeys(STALE_LIMIT_MS, 0.0)
READY = CarSignals(car_on=True, brake_pressed=True, gear_neutral=True, di_gas=0.0)
GOOD = {t: float(t) for t in range(10, 100, 10)}


def test_gates_pass_when_ready():
  assert gate_failure(READY, FRESH, True) is None


def test_gates_require_brake_neutral_and_car_on():
  assert "brake" in gate_failure(CarSignals(True, False, True), FRESH, True)
  assert "Neutral" in gate_failure(CarSignals(True, True, False), FRESH, True)
  assert "on" in gate_failure(CarSignals(False, True, True), FRESH, True)


def test_stale_can_fails_even_if_last_values_were_good():
  assert "CAN" in gate_failure(READY, {**FRESH, BRAKE_MESSAGE_ID: 501.0}, True)
  assert "CAN" in gate_failure(READY, {}, True)


def test_gas_gate_only_applies_before_commanding():
  pressed = CarSignals(True, True, True, 12.0)
  assert "accelerator" in gate_failure(pressed, FRESH, True)
  assert gate_failure(pressed, FRESH, False) is None


def test_validate_saves_measured_values():
  result, _ = validate_result(-3.0, 5.0, 55.0, GOOD)
  assert result["ok"] and result["pedal_calib_zero"] == 5.0 and result["pedal_calib_factor"] == 2.0


def test_validate_rejects_the_narrow_span_bad_capture_produced():
  result, why = validate_result(0.76, 0.76, 6.45, GOOD)
  assert result is None and "span" in why


def test_validate_rejects_when_car_response_disagrees():
  result, why = validate_result(-3.0, 5.0, 55.0, {**GOOD, 50: 70.0})
  assert result is None and "50%" in why


def test_disabled_command_is_zero_with_enable_clear():
  dat = pack_gas_command(40.0, 0, 3)
  assert dat[:4] == b"\x00\x00\x00\x00" and dat[4] == 3 and len(dat) == 6


def test_enabled_command_sets_enable_bit():
  assert pack_gas_command(40.0, 1, 3)[4] == 0x83


def test_carcontroller_has_no_press_capture():
  src = (pathlib.Path(__file__).parents[1] / "preap" / "carcontroller.py").read_text()
  assert "compute_calibration" not in src and "apply_calibration_result" not in src
