#!/usr/bin/env python3
"""Comma Pedal calibration against the car's own DI gas reading.

Ported from NotAutopilot scripts/nap/calibrate_pedal.py (MIT). Keep this notice.
Commands the pedal instead of asking the driver to press it, so PedalCalib* come
from measured DI response. Differences from NAP: any gate failure after the sweep
starts aborts instead of resuming, stale CAN fails the gates, validation can
reject a result, and the default run only listens.

Needs openpilot stopped, car on, Neutral, brake held. Uses SAFETY_ALLOUTPUT.
  listen only:  calibrate_pedal.py
  calibrate:    calibrate_pedal.py --confirm-at-car
"""
import argparse
import signal
import subprocess
import sys
import time
from dataclasses import dataclass

from openpilot.selfdrive.car.tesla.preap.calib import apply_calibration_result, pedal_can_bus
from openpilot.selfdrive.car.tesla.preap.params import PreApParamKeys, set_bool
from openpilot.selfdrive.car.tesla.preap.pedal_can import PEDAL_D, PEDAL_M1, PEDAL_M2, GAS_COMMAND_ID, pedal_checksum

SAFETY_SILENT = 0
SAFETY_ALLOUTPUT = 17

GTW_STATUS_ID = 0x348
BRAKE_MESSAGE_ID = 0x20A
DI_TORQUE1_ID = 0x108
DI_TORQUE2_ID = 0x118
GAS_SENSOR_ID = 0x552
GEAR_NEUTRAL = 0x30

CHASSIS_BUS = 0
SEND_PERIOD_MS = 20
HEARTBEAT_PERIOD_MS = 500
PEDAL_TIMEOUT_MS = 500
STALE_LIMIT_MS = {GTW_STATUS_ID: 1000, BRAKE_MESSAGE_ID: 500, DI_TORQUE1_ID: 500, DI_TORQUE2_ID: 500}
MAX_RUNTIME_S = 180.0

ZERO_SAMPLES = 100
AVG_SAMPLES = 10
TARGET_MAX = 99.6
TARGET_ZERO = 0.4
FINETUNE_STEP = 0.1
SWEEP_CEILING = 150.0  # abort rather than keep raising the command without a DI response
MIN_SPAN = 10.0
VALIDATION_TOLERANCE = 10.0


@dataclass
class CarSignals:
  car_on: bool = False
  brake_pressed: bool = False
  gear_neutral: bool = False
  di_gas: float = 0.0


def gate_failure(sig: CarSignals, ages_ms: dict[int, float], require_gas_released: bool) -> str | None:
  """Reason calibration must not command the pedal, or None when all gates pass."""
  for addr, limit in STALE_LIMIT_MS.items():
    if ages_ms.get(addr, float("inf")) > limit:
      return f"no recent CAN 0x{addr:03X}"
  if not sig.car_on:
    return "car is not on"
  if not sig.brake_pressed:
    return "brake is not held"
  if not sig.gear_neutral:
    return "not in Neutral"
  if require_gas_released and sig.di_gas > 0:
    return "accelerator is pressed"
  return None


def validate_result(pedal_min: float, pedal_pressed: float, pedal_max: float,
                    samples: dict[int, float]) -> tuple[dict[str, float] | None, str]:
  """Build the result to save, or (None, reason) when it should be rejected."""
  span = pedal_max - pedal_pressed
  if span < MIN_SPAN:
    return None, f"span {span:.2f} below {MIN_SPAN:.0f}"
  for target, got in samples.items():
    if abs(got - target) > VALIDATION_TOLERANCE:
      return None, f"asked for {target}% gas, car reported {got:.1f}%"
  return {
    "pedal_calib_zero": float(pedal_pressed),
    "pedal_calib_factor": 100.0 / span,
    "pedal_calib_min": float(pedal_min),
    "pedal_calib_max": float(pedal_max),
    "ok": 1.0,
  }, "ok"


def pack_gas_command(value: float, enable: int, idx: int) -> bytes:
  int_a = max(0, min(65534, int((value - PEDAL_D) / PEDAL_M1))) if enable else 0
  int_b = max(0, min(65534, int((value - PEDAL_D) / PEDAL_M2))) if enable else 0
  dat = bytes([(int_a >> 8) & 0xFF, int_a & 0xFF, (int_b >> 8) & 0xFF, int_b & 0xFF,
               ((enable << 7) | (idx & 0x0F)) & 0xFF])
  return dat + bytes([pedal_checksum(GAS_COMMAND_ID, dat)])


def now_ms() -> float:
  return time.monotonic() * 1000.0


def log(msg: str) -> None:
  print(msg, flush=True)


class Abort(Exception):
  pass


class Calibrator:
  def __init__(self, panda, pedal_bus: int):
    self.panda = panda
    self.pedal_bus = pedal_bus
    self.sig = CarSignals()
    self.seen_ms: dict[int, float] = {}
    self.pedal_seen_ms = 0.0
    self.pedal_state = 0
    self.pedal_value = 0.0
    self.pedal_idx_rx = -1
    self.pedal_idx_used = -1
    self.pedal_idx_tx = 0
    self.last_sent_ms = 0.0
    self.last_heartbeat_ms = 0.0
    self.command = 0.0
    self.enabled = 0

  def send(self, value: float, enable: int) -> None:
    self.panda.can_send(GAS_COMMAND_ID, pack_gas_command(value, enable, self.pedal_idx_tx), self.pedal_bus)
    self.pedal_idx_tx = (self.pedal_idx_tx + 1) % 16
    self.last_sent_ms = now_ms()

  def read_can(self) -> None:
    t = now_ms()
    for msg in self.panda.can_recv():
      addr, dat, src = msg[0], msg[-2], msg[-1]
      if src == CHASSIS_BUS:
        if addr == GTW_STATUS_ID:
          self.sig.car_on = (dat[0] & 0x01) == 1
        elif addr == BRAKE_MESSAGE_ID:
          self.sig.brake_pressed = ((dat[0] >> 2) & 0x03) != 1
        elif addr == DI_TORQUE2_ID:
          self.sig.gear_neutral = (dat[1] & 0x70) == GEAR_NEUTRAL
        elif addr == DI_TORQUE1_ID:
          self.sig.di_gas = dat[6] * 0.4
        else:
          continue
        self.seen_ms[addr] = t
      elif src == self.pedal_bus and addr == GAS_SENSOR_ID:
        self.pedal_state = (dat[4] >> 7) & 0x01
        self.pedal_value = ((dat[0] << 8) + dat[1]) * PEDAL_M1 + PEDAL_D
        self.pedal_idx_rx = dat[4] & 0x0F
        self.pedal_seen_ms = t

  def service(self) -> None:
    """Read CAN, keep the panda heartbeat alive, keep commanding, enforce gates."""
    self.read_can()
    t = now_ms()
    if t - self.last_heartbeat_ms >= HEARTBEAT_PERIOD_MS:
      self.panda.send_heartbeat(engaged=False)
      self.last_heartbeat_ms = t
    if self.enabled:
      ages = {a: t - self.seen_ms.get(a, -1e12) for a in STALE_LIMIT_MS}
      reason = gate_failure(self.sig, ages, require_gas_released=False)
      if reason:
        raise Abort(reason)
      if t - self.pedal_seen_ms > PEDAL_TIMEOUT_MS:
        raise Abort("pedal stopped responding")
      if self.pedal_state != 0:
        raise Abort("pedal reports a fault")
      if self.command > SWEEP_CEILING:
        raise Abort(f"no full-gas response by command {SWEEP_CEILING:.0f}")
    if t - self.last_sent_ms >= SEND_PERIOD_MS:
      self.send(self.command, self.enabled)

  def next_sample(self) -> None:
    """Wait for a fresh pedal message, servicing the loop meanwhile."""
    while self.pedal_idx_rx == self.pedal_idx_used:
      self.service()
      time.sleep(0.002)
    self.pedal_idx_used = self.pedal_idx_rx

  def average_gas(self) -> float:
    total = 0.0
    for _ in range(AVG_SAMPLES):
      self.next_sample()
      total += self.sig.di_gas
    return total / AVG_SAMPLES

  def wait_for_gates(self) -> None:
    log("Waiting for: car on, Neutral, brake held, accelerator released")
    last = None
    while True:
      self.service()
      t = now_ms()
      ages = {a: t - self.seen_ms.get(a, -1e12) for a in STALE_LIMIT_MS}
      reason = gate_failure(self.sig, ages, require_gas_released=True)
      if t - self.pedal_seen_ms > PEDAL_TIMEOUT_MS:
        reason = reason or "no pedal messages"
      if reason is None:
        return
      if reason != last:
        log(f"  waiting: {reason}")
        last = reason
      time.sleep(0.01)

  def finetune(self, start: float, target: float) -> float:
    best_val, best_delta = start, float("inf")
    self.command = start
    while self.command <= start + 1.0:
      avg = self.average_gas()
      if abs(target - avg) < best_delta:
        best_val, best_delta = self.command, abs(target - avg)
      self.command += FINETUNE_STEP
    log(f"  target {target}%: command {best_val:.2f} (off by {best_delta:.1f})")
    return best_val

  def run(self) -> dict[str, float] | None:
    self.wait_for_gates()

    total = 0.0
    for _ in range(ZERO_SAMPLES):
      self.next_sample()
      t = now_ms()
      reason = gate_failure(self.sig, {a: t - self.seen_ms.get(a, -1e12) for a in STALE_LIMIT_MS}, True)
      if reason:
        raise Abort(reason)
      total += self.pedal_value
    pedal_min = total / ZERO_SAMPLES
    log(f"Pedal min: {pedal_min:.2f}")

    # The gas-released gate only applies before commanding; from here on DI gas is the response.
    self.command, self.enabled = pedal_min, 1
    pressed_at = None
    while self.sig.di_gas < TARGET_MAX:
      self.command += 1.0
      self.next_sample()
      self.next_sample()
      if pressed_at is None and self.sig.di_gas > 0:
        pressed_at = self.command
        log(f"First gas response at command {pressed_at:.2f}")
    if pressed_at is None:
      raise Abort("car never reported gas")
    max_at = self.command
    log(f"Full gas at command {max_at:.2f}")

    pedal_max = self.finetune(max_at - 0.5, TARGET_MAX)
    pedal_pressed = self.finetune(pressed_at - 0.5, TARGET_ZERO)

    samples = {}
    span = pedal_max - pedal_pressed
    for target in range(10, 100, 10):
      self.command = pedal_pressed + target * span / 100.0
      samples[target] = self.average_gas()
      log(f"Validate {target}%: car reports {samples[target]:.1f}%")

    self.command, self.enabled = pedal_min, 0
    result, why = validate_result(pedal_min, pedal_pressed, pedal_max, samples)
    if result is None:
      raise Abort(f"validation failed: {why}")
    return result


def listen(cal: Calibrator, seconds: float) -> int:
  log(f"Listening only for {seconds:.0f}s, no pedal commands will be sent")
  end, last = now_ms() + seconds * 1000, 0.0
  while now_ms() < end:
    cal.service()
    t = now_ms()
    if t - last >= 1000:
      ages = {a: t - cal.seen_ms.get(a, -1e12) for a in STALE_LIMIT_MS}
      log(f"  on={cal.sig.car_on} neutral={cal.sig.gear_neutral} brake={cal.sig.brake_pressed} "
          f"di_gas={cal.sig.di_gas:.1f} pedal={cal.pedal_value:.2f} state={cal.pedal_state} "
          f"pedal_age_ms={t - cal.pedal_seen_ms:.0f} gates={gate_failure(cal.sig, ages, True) or 'OK'}")
      last = t
    time.sleep(0.01)
  return 0


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--confirm-at-car", action="store_true",
                  help="command the pedal; driver is in the seat, in Neutral, holding the brake")
  ap.add_argument("--listen-seconds", type=float, default=10.0)
  args = ap.parse_args()

  if subprocess.run(["pgrep", "-f", "manager.py"], capture_output=True).returncode == 0:
    log("openpilot is running. Stop it first: tmux kill-session -t comma")
    return 2

  from panda import Panda

  def on_signal(signum, _frame):
    raise Abort(f"signal {signum}")
  signal.signal(signal.SIGTERM, on_signal)
  signal.signal(signal.SIGHUP, on_signal)

  panda = Panda()
  cal = Calibrator(panda, pedal_can_bus())
  code = 1
  try:
    if not args.confirm_at_car:
      panda.set_safety_mode(SAFETY_SILENT)
      cal.enabled = 0
      cal.send = lambda *_: None
      code = listen(cal, args.listen_seconds)
    else:
      signal.signal(signal.SIGALRM, on_signal)
      signal.alarm(int(MAX_RUNTIME_S))
      panda.set_safety_mode(SAFETY_ALLOUTPUT)
      result = cal.run()
      signal.alarm(0)
      apply_calibration_result(result)
      set_bool(PreApParamKeys.PEDAL_CALIB_ARMED, False)
      log(f"Saved: zero={result['pedal_calib_zero']:.3f} factor={result['pedal_calib_factor']:.4f} "
          f"min={result['pedal_calib_min']:.3f} max={result['pedal_calib_max']:.3f}")
      code = 0
  except (Abort, KeyboardInterrupt) as e:
    log(f"ABORTED, nothing saved: {e}")
  finally:
    signal.alarm(0)
    if args.confirm_at_car:
      for _ in range(5):
        try:
          cal.send(0.0, 0)
        except Exception as e:
          log(f"Warning, disable send failed: {e}")
        time.sleep(0.02)
    panda.set_safety_mode(SAFETY_SILENT)
    panda.close()
    log("Pedal disabled, panda back to SILENT. Reboot the comma to restart openpilot.")
  return code


if __name__ == "__main__":
  sys.exit(main())
