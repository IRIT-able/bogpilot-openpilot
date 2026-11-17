"""The Pre-AP pedal engage edge does no file IO and no first-time module import.

card runs the long controller inside its 100 Hz step. Two things used to happen on the pedal ACQUIRE edge, the one frame
that must not be slow:
  - _personality() stat'ed and read up to two /data files;
  - PedalFeedback was imported on first use (a first-time module import inside the step is a step overrun).
Both are now done away from the edge: the personality is primed when the controller is built (card start, offroad) and
refreshed only on frames that are not acquiring, and the import is at module scope. The value and the default are
unchanged.
"""
import ast
from pathlib import Path

from opendbc.car.tesla_ap1.preap import long_controller as lc

SRC_PATH = Path(lc.__file__)
SRC = SRC_PATH.read_text()


def _controller():
  return lc.PreAPLongController(pedal_counter=None, pedal_bus_fn=lambda: 2)


def test_the_personality_is_primed_when_the_controller_is_built():
  c = _controller()
  assert c._personality() in (0, 1, 2)
  assert c._personality_cache == c._personality()


def test_personality_does_no_file_io(monkeypatch):
  c = _controller()
  monkeypatch.setattr(lc.PreAPLongController, "_personality_read",
                      lambda self: (_ for _ in ()).throw(AssertionError("read on the engage edge")))
  for _ in range(10):
    assert c._personality() in (0, 1, 2)


def test_the_refresh_is_rate_limited(monkeypatch):
  c = _controller()
  reads = []
  monkeypatch.setattr(lc.PreAPLongController, "_personality_read", lambda self: reads.append(1) or 2)
  t = [1000.0]
  monkeypatch.setattr(lc.time, "monotonic", lambda: t[0])
  c._personality_read_mono = 0.0
  c._personality_refresh()
  assert len(reads) == 1 and c._personality() == 2
  c._personality_refresh()
  assert len(reads) == 1, "inside the window: no second read"
  t[0] += lc.PERSONALITY_REFRESH_S + 0.01
  c._personality_refresh()
  assert len(reads) == 2


def test_a_failing_refresh_keeps_the_last_value(monkeypatch):
  c = _controller()
  c._personality_cache = 2
  monkeypatch.setattr(lc.PreAPLongController, "_personality_read",
                      lambda self: (_ for _ in ()).throw(OSError))
  c._personality_read_mono = 0.0
  c._personality_refresh()
  assert c._personality() == 2


def test_a_bad_or_missing_file_still_defaults_to_standard(monkeypatch, tmp_path):
  monkeypatch.setattr(lc, "PERSONALITY_PATHS", (tmp_path / "missing",))
  assert _controller()._personality_read() == 1
  p = tmp_path / "LongitudinalPersonality"
  monkeypatch.setattr(lc, "PERSONALITY_PATHS", (p,))
  p.write_text("not a number")
  assert _controller()._personality_read() == 1
  p.write_text("2")
  assert _controller()._personality_read() == 2
  p.write_text("9")
  assert _controller()._personality_read() == 2, "clamped to 0-2"


def test_the_refresh_is_called_off_the_acquire_edge():
  assert "if pedal_action != PedalCommandAction.ACQUIRE:" in SRC
  assert "self._personality_refresh()" in SRC


def test_no_function_level_imports_are_left_in_the_controller():
  """A first-time import inside card's step is a step overrun; the module imports PedalFeedback at the top now."""
  tree = ast.parse(SRC)
  inner = []
  for node in ast.walk(tree):
    if not isinstance(node, ast.FunctionDef):
      continue
    for sub in ast.walk(node):
      if isinstance(sub, ast.Import | ast.ImportFrom):
        inner.append((node.name, sub.lineno))
  assert not inner, inner
  assert "from opendbc.car.tesla_ap1.preap.pedal_feedback import PedalFeedback" in SRC
