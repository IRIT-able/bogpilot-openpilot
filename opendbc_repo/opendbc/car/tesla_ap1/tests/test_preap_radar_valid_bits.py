"""The radar status line names WHICH 0x501 bit latched F2, and says whether the ITC code is a known one.

Saved logs of this build showed `fault=F2 ... itc=100` for a whole drive, with the radar tracking 10 points the whole
time, and there was no way to tell from the log which of the eleven F2 bits was set: the remedies differ per bit
(vinValidity wants the donor VIN, radPositionMismatch / configMismatch want another Radar Donor Type, a *MIA bit means
the radar is not getting a vehicle-data frame). The ITC was neither in ITC_OK nor in ITC_FAULT, which holds VALID off
without being a FAULT and also read as a bare valid=0.

Diagnostics only: no gate decision, threshold or control path changes here.
"""
from opendbc.car.tesla_ap1.preap import radar_valid as rv

CLEAN = {b: 0 for b in rv.ALERT_BITS}


def bits(*names):
  return {**CLEAN, **{n: 1 for n in names}}


def _gate():
  return rv.RadarValidGate(radar_emulation=True)


def test_short_bit_drops_the_dbc_prefix():
  assert rv.short_bit("RADC_a037_vinValidity") == "vinValidity"
  assert rv.short_bit("RADC_a061_radPositionMismatch") == "radPositionMismatch"
  assert rv.short_bit("odd") == "odd"


def test_itc_state_classifies_the_three_cases():
  assert rv.itc_state(0) == "ok"
  assert rv.itc_state(107) == "ok"
  assert rv.itc_state(67) == "fault"
  assert rv.itc_state(96) == "fault"
  assert rv.itc_state(100) == "unknown", "the code the saved logs showed for a whole drive"
  assert rv.itc_state(None) == "none"


def test_no_bits_reads_as_a_dash():
  g = _gate()
  g.on_alert_matrix(0.0, CLEAN)
  assert g.status()["bits"] == "-"


def test_the_latching_bit_is_named():
  g = _gate()
  for t in (0.0, 0.1):                                   # F2 needs 2 consecutive frames
    g.on_alert_matrix(t, bits("RADC_a037_vinValidity"))
  g.update(0.2)
  st = g.status()
  assert st["fault"] == "F2"
  assert st["bits"] == "vinValidity", st["bits"]


def test_several_bits_are_all_named_in_dbc_order():
  g = _gate()
  for t in (0.0, 0.1):
    g.on_alert_matrix(t, bits("RADC_a061_radPositionMismatch", "RADC_a009_configMismatch"))
  g.update(0.2)
  assert g.status()["bits"] == "configMismatch,radPositionMismatch", g.status()["bits"]


def test_the_name_is_kept_while_f2_holds_through_one_clear_frame():
  """F2 clears only after 2 clear frames, so the bit that latched it must survive a single clear frame."""
  g = _gate()
  for t in (0.0, 0.1):
    g.on_alert_matrix(t, bits("RADC_a013_gtwMIA"))
  g.on_alert_matrix(0.2, CLEAN)
  g.update(0.3)
  assert g.status()["fault"] == "F2", "still faulted after one clear frame"
  assert g.status()["bits"] == "gtwMIA"


def test_the_name_is_dropped_once_f2_clears():
  g = _gate()
  for t in (0.0, 0.1):
    g.on_alert_matrix(t, bits("RADC_a013_gtwMIA"))
  for t in (0.2, 0.3):
    g.on_alert_matrix(t, CLEAN)
  g.update(0.4)
  assert g.status()["fault"] != "F2"
  assert g.status()["bits"] == "-"


def test_adjustment_bits_are_named_with_a_prefix():
  """These hold VALID off without being a FAULT, so a bare valid=0 never said why."""
  g = _gate()
  g.on_alert_matrix(0.0, bits("RADC_a005_adjustmentReq"))
  g.update(0.1)
  assert g.status()["fault"] is None or g.status()["fault"] == "-"
  assert g.status()["bits"] == "adj:adjustmentReq", g.status()["bits"]


def test_status_keeps_its_existing_keys():
  """The log line and anything else reading status() must not lose a field."""
  g = _gate()
  g.on_alert_matrix(0.0, CLEAN)
  g.update(0.1)
  assert {"echoed", "pending", "valid", "fault", "late", "itc", "faults"} <= set(g.status())
  assert {"bits", "itc_state"} <= set(g.status())
