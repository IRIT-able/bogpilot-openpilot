"""PreapEngageSetLimit reads its toggles off card's control step, not on the engage frame.

kph() runs on the enable edge and on the Pre-AP FULL capture edge, both inside card's 100 Hz step. It used to call
_toggles() there, which on the first engage paid two lazy imports, a ParamStore (and its first all_keys()) and dozens of
param reads, and on every engage after that re-read them all. Saved logs of this build show a 120 ms card step 4 ms
after longActive first rose; selfdrived's Ratekeeper.lagging trips when its 100-sample average period passes 11.11 ms,
so one stall of that size is enough on its own to raise "System Lagging" (a soft disable).

warm_engage_helpers already did this for the AP1 helpers ("the first engage after boot stalled card for ~145 ms" in its
docstring); the Pre-AP helper was never warmed. refresh() now fills a snapshot, card warms it at start and keeps it
fresh from params_thread, and kph() reads memory. Values and gating are unchanged.
"""
import unittest
from types import SimpleNamespace as NS

from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.bogpilot_onroad_toggles import onroad_toggles
from openpilot.selfdrive.controls.lib import phase3_speed_control as sc
from openpilot.selfdrive.controls.tests.test_phase3_speed_control import FakeStore

MPH = CV.MPH_TO_MS
CP_PREAP = NS(carFingerprint="TESLA_PREAP_MODELS", openpilotLongitudinalControl=True, pcmCruise=False,
              radarUnavailable=True, enableBsm=False)
CP_AP1 = NS(carFingerprint="TESLA_AP1_MODELS", openpilotLongitudinalControl=True, pcmCruise=True,
            radarUnavailable=False, enableBsm=True)
MOVING = NS(standstill=False, gasPressed=False, vEgo=10.0)


def _store(**vals):
  vals.setdefault("SpeedLimitController", "1")
  vals.setdefault("SetSpeedLimit", "1")
  return FakeStore(**vals)


class Counted:
  """A toggles_fn that records how often it is called."""

  def __init__(self, store, cp):
    self.store, self.cp, self.calls = store, cp, 0

  def __call__(self):
    self.calls += 1
    return onroad_toggles(self.store, self.cp)


def _engage(cp=CP_PREAP, adopted=True, **vals):
  st = _store(**vals)
  fn = Counted(st, cp)
  return sc.PreapEngageSetLimit(cp, store=st, toggles_fn=fn, adopted=adopted), fn


class TestEngageSetSnapshot(unittest.TestCase):
  def test_kph_reads_no_toggles_once_refreshed(self):
    e, fn = _engage(Offset3="3")
    self.assertTrue(e.refresh())
    before = fn.calls
    for _ in range(5):                                   # five engage edges
      e.kph(MOVING, 40 * MPH)
    self.assertEqual(fn.calls, before, "kph() must not read the toggles again")

  def test_the_value_is_the_same_as_reading_inline(self):
    e, _ = _engage(Offset3="3")
    inline = e.kph(MOVING, 40 * MPH)                     # no refresh yet: the old path
    self.assertTrue(e.refresh())
    self.assertAlmostEqual(e.kph(MOVING, 40 * MPH), inline, places=6)
    self.assertAlmostEqual(inline, (40 + 3) * MPH * 3.6, places=3)

  def test_without_a_refresh_it_still_reads_inline(self):
    """Any other caller, and the tests above, must keep working with no refresh() at all."""
    e, fn = _engage(Offset3="3")
    self.assertAlmostEqual(e.kph(MOVING, 40 * MPH), (40 + 3) * MPH * 3.6, places=3)
    self.assertGreater(fn.calls, 0)

  def test_a_refresh_picks_up_a_changed_setting(self):
    st = _store(Offset3="3")
    e = sc.PreapEngageSetLimit(CP_PREAP, store=st, toggles_fn=lambda: onroad_toggles(st, CP_PREAP), adopted=True)
    self.assertTrue(e.refresh())
    st.vals["Offset3"] = "5"
    self.assertAlmostEqual(e.kph(MOVING, 40 * MPH), (40 + 3) * MPH * 3.6, places=3, msg="snapshot still in force")
    self.assertTrue(e.refresh())
    self.assertAlmostEqual(e.kph(MOVING, 40 * MPH), (40 + 5) * MPH * 3.6, places=3)

  def test_refresh_on_an_inactive_car_does_nothing(self):
    e, fn = _engage(cp=CP_AP1)
    self.assertFalse(e.active)
    self.assertFalse(e.refresh())
    self.assertEqual(fn.calls, 0)

  def test_a_failing_refresh_never_raises_and_keeps_the_last_snapshot(self):
    st = _store(Offset3="3")
    state = {"boom": False}

    def toggles():
      if state["boom"]:
        raise RuntimeError
      return onroad_toggles(st, CP_PREAP)

    e = sc.PreapEngageSetLimit(CP_PREAP, store=st, toggles_fn=toggles, adopted=True)
    self.assertTrue(e.refresh())
    state["boom"] = True
    self.assertFalse(e.refresh(), "a failed refresh reports False")
    self.assertAlmostEqual(e.kph(MOVING, 40 * MPH), (40 + 3) * MPH * 3.6, places=3)

  def test_a_failing_read_with_no_snapshot_still_fails_closed(self):
    def boom():
      raise RuntimeError
    e = sc.PreapEngageSetLimit(CP_PREAP, store=_store(), toggles_fn=boom, adopted=True)
    self.assertEqual(e.kph(MOVING, 20.0), 0.0)

  def test_warm_engage_helpers_fills_the_snapshot(self):
    e, fn = _engage(Offset3="3")
    sc.warm_engage_helpers(e)
    self.assertGreater(fn.calls, 0, "warming must do the read")
    before = fn.calls
    e.kph(MOVING, 40 * MPH)
    self.assertEqual(fn.calls, before, "the engage frame reads memory after warming")

  def test_warm_engage_helpers_still_warms_the_ap1_helpers_and_tolerates_none(self):
    calls = []

    class Ap1Helper:                                     # EngageLimitOffset / EngageTapFromZero shape
      def _toggles(self):
        calls.append("toggles")

    class Plain:
      def __call__(self):
        calls.append("call")

    def boom():
      raise RuntimeError

    sc.warm_engage_helpers(Ap1Helper(), Plain(), None, boom)
    self.assertEqual(calls, ["toggles", "call"])


if __name__ == "__main__":
  unittest.main()
