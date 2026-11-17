"""card warms the Pre-AP engage-set helper offroad and refreshes it off the 100 Hz step.

Source assertions in the style of test_preap_distance_button_read.py: card.py cannot be imported here (it needs the
compiled Params library and messaging), so the wiring is checked against the file, as that test already does for the
10 Hz distance-button read.

Why: PreapEngageSetLimit.kph() runs on the enable edge and the FULL capture edge, both inside card's step. Reading the
SetSpeedLimit toggles there cost two lazy imports, a ParamStore and dozens of param reads on the first engage and a
re-read on every engage after it. warm_engage_helpers was already doing exactly this for the AP1 helpers (its docstring
records a ~145 ms first-engage stall); the Pre-AP helper was constructed and never warmed.
"""
from pathlib import Path

from openpilot.selfdrive.car.preap_card_step import PREAP_ENGAGE_SET_REFRESH_S

CARD = Path(__file__).resolve().parents[1] / "card.py"
SRC = CARD.read_text()
INIT = SRC[SRC.index("def __init__"):SRC.index("def state_update")]
STEP = SRC[SRC.index("def state_update"):SRC.index("def params_thread")]
PARAMS_THREAD = SRC[SRC.index("def params_thread"):SRC.index("def card_thread")]


def test_the_preap_helper_is_warmed_at_card_start():
  block = INIT[INIT.index("PreapEngageSetLimit"):]
  assert "warm_engage_helpers(self.preap_engage_set_limit)" in block, (
      "the Pre-AP engage-set helper must be warmed offroad, as the AP1 helpers are"
  )


def test_the_snapshot_is_refreshed_from_the_params_thread():
  assert "self.preap_engage_set_limit.refresh()" in PARAMS_THREAD
  assert "PREAP_ENGAGE_SET_REFRESH_S" in PARAMS_THREAD


def test_the_refresh_never_runs_in_the_step():
  assert "refresh()" not in STEP, "a toggles read must never land on the engage frame"


def test_the_step_still_reads_the_engage_set_on_the_edges():
  """The fix is where the toggles are read, not whether the set is still applied."""
  assert "self.preap_engage_set_limit.kph(self.CS_prev" in STEP
  assert "self.preap_engage_set_limit.kph(CS," in STEP


def test_the_refresh_period_is_sane():
  assert 0.2 <= PREAP_ENGAGE_SET_REFRESH_S <= 10.0, PREAP_ENGAGE_SET_REFRESH_S
  assert PREAP_ENGAGE_SET_REFRESH_S >= 0.1, "the params thread itself runs at 10 Hz"


def test_the_ap1_warm_up_is_still_wired():
  assert "warm_engage_helpers(self.CI.CS.op_long_offset, self.CI.CS.op_long_tap_from_zero)" in INIT
