"""Vehicle Settings > Pre-AP: Donor VIN — type the VIN in by hand.

Until now the donor VIN (TeslaRadarDonorVin) could only be set by the Read Radar VIN button, which asks the radar for
its F190 over UDS. A radar that was VIN-learned elsewhere, or one that will not answer the read, left no way to set it
from the device. radar_donor.py has always described the value as "17 letters or digits, entered by hand"; this is the
row for it. Same stored key, same validation as the 0x560 stream (normalize_radar_donor_vin), offroad only, because the
value is read once when the car starts.
"""
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

from opendbc.car.tesla_ap1.preap import params as preap_params
from openpilot.selfdrive.ui.bogpilot.phase2_vehicle import (PREAP, RADAR_DONOR_VIN, RADAR_DONOR_VIN_BAD,
                                                            VehicleSettings)

VIN = "5YJSA1E26HF000000"      # 17 valid VIN characters; not a real car


class TestDonorVinRow(unittest.TestCase):
  def setUp(self):
    self.temp = TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)
    self.files = self.root / "bogpilot"
    self.files.mkdir()
    # Point the Pre-AP file-backed params at the temp dir; never touch /data in a test.
    self.prev_dir = preap_params._params_dir
    preap_params._params_dir = self.root / "params_bogpilot"
    self.addCleanup(lambda: setattr(preap_params, "_params_dir", self.prev_dir))

    self.cp = SimpleNamespace(carFingerprint=PREAP, dashcamOnly=False, openpilotLongitudinalControl=True,
                              alphaLongitudinalAvailable=False)
    self.offroad = True
    self.model = VehicleSettings(SimpleNamespace(put_bool=lambda *a, **k: None), lambda: self.cp,
                                 lambda: self.offroad, self.files, self.root / "long")
    preap_params.set_bool(preap_params.PreApParamKeys.RADAR_ENABLED, True)

  def stored(self):
    return preap_params.get_str(preap_params.PreApParamKeys.RADAR_DONOR_VIN)

  # ---- visibility ----
  def test_shown_on_a_preap_car_with_the_radar_on(self):
    self.assertTrue(self.model.visible(RADAR_DONOR_VIN))
    self.assertTrue(self.model.enabled(RADAR_DONOR_VIN))

  def test_hidden_while_the_radar_is_off(self):
    preap_params.set_bool(preap_params.PreApParamKeys.RADAR_ENABLED, False)
    self.assertFalse(self.model.visible(RADAR_DONOR_VIN))

  def test_hidden_on_an_ap1_car(self):
    self.cp = SimpleNamespace(carFingerprint="TESLA_AP1_MODELS", dashcamOnly=False,
                              openpilotLongitudinalControl=True, alphaLongitudinalAvailable=False)
    self.assertFalse(self.model.visible(RADAR_DONOR_VIN))

  def test_disabled_while_the_car_is_on(self):
    self.offroad = False
    self.assertTrue(self.model.visible(RADAR_DONOR_VIN))
    self.assertFalse(self.model.enabled(RADAR_DONOR_VIN))

  # ---- writing ----
  def test_a_valid_vin_is_saved(self):
    self.assertIsNone(self.model.set_donor_vin(VIN))
    self.assertEqual(self.stored(), VIN)
    self.assertEqual(self.model.donor_vin_stored(), VIN)
    self.assertEqual(self.model.donor_vin_text(), VIN)

  def test_lowercase_and_separators_are_normalized(self):
    self.assertIsNone(self.model.set_donor_vin(" 5yjsa1e26-hf0 00000 "))
    self.assertEqual(self.stored(), VIN)

  def test_an_empty_box_clears_it_back_to_this_car(self):
    self.assertIsNone(self.model.set_donor_vin(VIN))
    for blank in ("", "   ", "---", None):
      self.assertIsNone(self.model.set_donor_vin(VIN))
      self.assertIsNone(self.model.set_donor_vin(blank), blank)
      self.assertEqual(self.stored(), "", blank)
      self.assertEqual(self.model.donor_vin_text(), "Not set")

  def test_a_short_or_long_vin_is_refused_and_changes_nothing(self):
    self.assertIsNone(self.model.set_donor_vin(VIN))
    for bad in ("5YJSA1E26HF", VIN + "00", "5YJSA1E26HF00000!"):
      self.assertEqual(self.model.set_donor_vin(bad), RADAR_DONOR_VIN_BAD, bad)
      self.assertEqual(self.stored(), VIN, bad)

  def test_refused_while_the_car_is_on(self):
    self.offroad = False
    self.assertIsNotNone(self.model.set_donor_vin(VIN))
    self.assertEqual(self.stored(), "")

  def test_refused_while_the_radar_is_off(self):
    preap_params.set_bool(preap_params.PreApParamKeys.RADAR_ENABLED, False)
    self.assertIsNotNone(self.model.set_donor_vin(VIN))
    self.assertEqual(self.stored(), "")

  def test_the_refusal_reason_never_contains_the_vin(self):
    self.offroad = False
    self.assertNotIn(VIN, self.model.set_donor_vin(VIN) or "")
    self.assertNotIn(VIN, RADAR_DONOR_VIN_BAD)

  def test_it_is_the_same_key_the_read_button_writes(self):
    self.assertEqual(RADAR_DONOR_VIN, preap_params.PreApParamKeys.RADAR_DONOR_VIN)

  def test_what_is_saved_is_what_the_0x560_stream_accepts(self):
    from opendbc.car.tesla_ap1.preap.radar_donor import normalize_radar_donor_vin
    self.assertIsNone(self.model.set_donor_vin(VIN))
    self.assertEqual(normalize_radar_donor_vin(self.stored()), VIN)

  def test_the_row_has_help(self):
    from openpilot.selfdrive.ui.bogpilot.toggle_help import resolve
    self.assertIsNotNone(resolve(RADAR_DONOR_VIN))


if __name__ == "__main__":
  unittest.main()
