# Pre-AP radar-valid filter (advisory, never an engage gate)

Code: `opendbc_repo/opendbc/car/tesla_ap1/preap/radar_valid.py` (`RadarValidGate`), wired in `tesla_ap1/radar_interface.py`;
card (`selfdrive/car/card.py:284`) turns a radar that is not VALID into `carState.bogpilot.preapRadarNotReady`, one warning.
The radar is never an engage input: the Pre-AP pull FSM (`preap/carstate.py` speed_block :300, long_allowed :312) has no radar
term. Pre-AP only; AP1 has no filter. Lateral is never touched. See DIVERGENCES.md, "radar health is advisory, not an engage
gate".

## What it does
- Until the radar is VALID, and while any fault below is active, radar_interface publishes no radar points
  (`radar_interface.py:179`): speed control runs on camera leads and, while engaged with openpilot longitudinal, one warning
  shows, "Radar not ready – using camera lead" (event `preapRadarNotReady`, ET.WARNING only, `selfdrived/events.py:1013`;
  `ap1_events.py:78`). Never a no-entry, soft disable or disengage. Only HWFail is a radarFault (`radar_interface.py:142`; none with the Ignore radar HWFail option on, default off).
- A returned gateway echo (flag 256) only proves we sent the config frames. Whether the radar ACCEPTED it is read from the
  radar itself. The log-only status line (`preap radar: echoed= pending= valid= fault= bits= late= itc= ...`, every 5 s
  and on each change) reports echoed, pending, valid and the fault name separately.
- The status line also names WHICH 0x501 bits are set (`bits=`) and classifies the ITC code (`itc=100(unknown)`). A drive
  on `7a683247` logged `fault=F2 ... itc=100` for 28 segments with the radar tracking 10 points throughout, and neither
  question could be answered from the log: F2 covers eleven bits whose remedies differ (a037_vinValidity wants a donor
  VIN, a061_radPositionMismatch / a009_configMismatch want another Radar Donor Type, a *MIA bit means the radar is not
  getting a vehicle-data frame), and an ITC outside both ITC_OK and the F1 set holds VALID off without being a FAULT,
  which read as a bare `valid=0`. `bits=` shows the bits that latched F2 (kept while F2 holds, since it needs two clear
  frames to clear) plus any adjustment bits prefixed `adj:`, which keep VALID off without being a fault; `-` when none
  are set. Both are part of the line's change key, so a change of bit or ITC code is logged at once. Diagnostics only:
  no criterion, threshold, ITC set or control path is changed, and no VIN or other identifier is in the line.
  Tests: `opendbc/car/tesla_ap1/tests/test_preap_radar_valid_bits.py`.

## Criteria
From a CAN analysis of both tester packs, a 0.9.7 Pre-AP log and 67 AP1 power-ups (666 unique AP1 files). DBC:
`opendbc/dbc/generator/tesla/tesla_radar_bosch.py` (0x301 TeslaRadarSguInfo :46-55, 0x501 TeslaRadarAlertMatrix :64-130) and
`_radar_common.py:6-15` (point slots 0x310-0x36D). Time 0 is the radar's first 0x301.

FAULT (non-latching, any one; a fault only keeps the points out, only F4 is a radarFault):
- F1: `RADC_SGU_ITC` (:50) in {67, 68, 70, 83} (seen on starved radars) or {96} (seen with a rejected position), held 1.0 s.
- F2: any of 0x501 a009_configMismatch (:77), a027_CntryCdValidity (:95), a037_vinValidity (:105), a042_xwdValidity (:110),
  a061_radPositionMismatch (:129), a062_strRackMismatch (:130), or the bdy / esp / gtw / sccm / adas MIA bits (:79-83), set in
  2 consecutive 0x501 frames. It clears after 2 consecutive clear frames.
- F3 frozen table: 1 or more points whose rounded (dRel, yRel, vRel) set is unchanged for 1.5 s, counted only while the ITC
  is in the F1 set. NAP `BoschTableFreezeWatch` (`opendbc car/tesla/preap/radar_table_freeze.py` @ `7eb09d2a7`) with three
  changes: 1 point instead of 4 (tester 2's frozen table had 3 points, so 4 never fires), 1.5 s (2.4 s or more misses tester
  2's 2 s target; 1.9 s meets it by only 20-30 ms), and the ITC gate instead of NAP's SGUFail gate (ungated, it false-faults
  6-33 times on parked AP1 static scenes; ITC-gated, 0 of 666 files).
- F4: `RADC_HWFail` (:53), immediately (same signal as the existing radarFault).
- NO_ECHO: with flag 256, no returned gateway echo for 2 s (the radar then gets no vehicle data at all).

VALID: ITC in {0, 107}, `RADC_SGUFail` (:54), HWFail and `RADC_SensorDirty` (:52) all 0, a 0x501 frame seen and the last
one clear of the F2 bits and the adjustment bits a004-a006 (:72-74), and a moving track in the last 2 s, all held 2.0 s. It
latches until a FAULT fires (AP1's longest moving-ego gap with no moving track was 22.5 s). ITC 107 sat on 14 of 67 healthy
AP1 power-ups for 12-207 s with SGUFail 0 and 0x501 clear, so a strict ITC = 0 would block healthy radars. **What 107 means
is UNPROVEN.**

PENDING: neither, for example SGUFail or adjustment bits with moving tracks (aligning), or a parked radar with no moving
track yet. Not a fault. Like every state that is not VALID it only keeps the points out (camera leads); engage and speed control are
allowed, as NAP allows engage while its radar aligns (`opendbc/car/tesla/radar_interface.py:203-210` @ `508d26fcb`).

Moving track: a tracked point (bit 62, `_radar_common.py:15`) with 0 < LongDist <= 250 m and ProbExist >= 50 % (the filters
in radar_interface), seen in the last 0.3 s, with |LongSpeed| >= 0.5 m/s or a LongDist change >= 0.5 m against the same slot
2 s earlier.

LATE (label only): not VALID 20 s after the later of the radar's first 0x301 and the car first moving (vEgo >= 0.3 m/s, fed
by card). Two parked AP1 power-ups reached VALID about 2 s after moving off.

## Divergences
- Stock and NAP: Pre-AP radar points always reach radard; NAP faults a frozen table only with 4 points and SGUFail. Tinkla
  and BogPilot 0.9.7: no such filter. Ours filters more (no radar points until VALID, camera leads meanwhile) but, like NAP
  and Tinkla, radar health never blocks engage or speed control. NAP (`f00c8cb9`, `e512a07c`) raises radarFault on a frozen
  table with SGUFail; ours only drops those points.
- Not proven on a car: the ITC meanings, why ITC 107 clears at standstill, how long a parked ITC 90 lasts, an aligning radar
  with SGUFail and live tracks (not seen in any AP1 log), and the tester 2 time base (its packs start mid-drive).

## Evidence (offline replays and tests)
- Tests: `opendbc_repo/opendbc/car/tesla_ap1/tests/test_preap_radar_valid.py` (the analysis' numbers: 1 point fires and 4
  don't on tester 2's phantoms; 1.5 s fires within 2 s and 2.4 s doesn't; ungated F3 false-faults on a parked static scene and
  gated doesn't; ITC 107 reaches VALID; aligning SGUFail with moving tracks is PENDING, then VALID; no PENDING setting left), `test_preap_radar_advisory.py`
  (every cause that is not VALID keeps engage and only filters; HWFail still faults) and
  `openpilot/selfdrive/selfdrived/tests/test_preap_radar_not_ready.py` (warning only).
- Replays of the tester packs and a 30-file AP1 sample through `RadarInterface` are private (raw CAN). Tester 2 faults 1.02 s
  after its pack starts and never reaches VALID; tester 1 never reaches VALID; every decoded AP1 file reaches VALID with no
  fault.
