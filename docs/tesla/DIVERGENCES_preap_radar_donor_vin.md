# Pre-AP Bosch radar: donor VIN, config gate, 0x2A9 byte 7, Read Radar VIN

References: NAP `naponsp-alpha-prebuilt` `7eb09d2a7` (opendbc `5dbe36d37`, panda `a8f6ff22d`), NAP commits `347ac59b` /
`edd064aa` (donor VIN + F190), `604b9c49` (config gate), `f00c8cb9` (position from the donor config), `a2cdda8e` (no byte-7
checksum); NAP nap-alpha `92b42f772` (opendbc `508d26fcb`); BogPilot na-preap `c68fda744`; Tinkla/BogGyver `501c7de`.

## 1. 0x2A9 byte 7 (now matches all references)
- Before: the panda overwrote 0x2A9 byte 7 with an address + data byte sum (ported from NAP `cc39c7fa`), which also changed
  GTW_autopilot and the other byte-7 bits.
- Now: no checksum. Byte 7 = (car 0x398 byte 7 & 0xCF) | 0x10, i.e. GTW_autopilot = 1. Bosch BO_681 is "all fixed bytes, no
  checksum" (`opendbc/dbc/generator/tesla/tesla_radar_bosch.py:158,249`). Same as NAP `a2cdda8e`
  (`opendbc/safety/modes/tesla_preap.h:346-367` @ `5dbe36d37`), na-preap `c68fda744` and Tinkla (`safety_tesla.h:462-484`).

## 2. Host 0x560 donor config and the config gate (NAP, one rule stricter)
- 0x560 is host to panda only: on the TX list as {0x560, bus 0, 8}, the tx hook consumes it and returns false, so it never goes
  on the car. Three records: 0 = useRadar, position (0..2), EPAS type, VIN[0:3]; 1 = VIN[3:10]; 2 = VIN[10:17]
  (NAP `tesla_preap.h:199-228`, `teslacan.py:83-97`).
- Gate (NAP `604b9c49`, `tesla_preap.h:178-184`): with flag 256 the panda sends no radar-bus GTW frame until all three records
  have arrived with useRadar set. All donor state is cleared on every safety init.
- Stricter than NAP: 0x560 is ignored without flag 256 and while controls are allowed or the steering latch is armed, so the radar
  contract cannot change mid-engagement (NAP accepts it at any time: `opendbc/safety/modes/tesla_preap_tx.h:33-37` @ `508d26fcb`).
- Host stream, v2 (`tesla_ap1/preap/radar_donor.py` RadarDonorStream, a link state machine keyed on what the panda reports;
  card feeds it `pandaStates` and the panda's TX echoes, `tesla_ap1/preap/panda_link.py`). One record every 10 frames (10 Hz),
  only with flag 256, in any gear, and only while:
  - a panda runs the car's safety model with the expected param (not in ELM327, where the tx hook refuses it anyway);
  - the gateway is not seen open: no returned echo (src 129) of any frame the gateway puts on bus 1 in the last 0.3 s
    (the address set is read from `tesla_preap_radar.h` by a test);
  - openpilot is disengaged and not steering, the pull FSM is not in an engaged mode, and the panda does not allow controls.
  It stops as soon as the gateway frames come back, and re-arms on a safety model or param change, a panda uptime decrease
  (reboot), `heartbeatLost`, or gateway echoes absent for 0.3 s while the panda does not allow controls (a same-mode re-init
  shows nothing else in `pandaStates`). Simulated through the compiled panda safety code: the gateway opens within 1 s after
  the switch from ELM327 (about 15 s after the car starts, in any gear), after a re-init or a panda reboot in Drive, and within
  1 s of disengaging after a re-init while engaged. Not yet seen on a real car.
- NAP streams it at 10 Hz whenever the radar is on, engaged or not (`opendbc/car/tesla/preap/sp/carcontroller.py:69-77` @
  `5dbe36d37`; nap-alpha `92b42f772`, opendbc `508d26fcb`: `opendbc/car/tesla/carcontroller.py:143-152`). Divergence: no 0x560
  while engaged or steering (the panda ignores it then anyway, the config set before engaging stays until the next safety init)
  and none once the gateway is seen open. Limit: after a safety init while engaged, the radar stays silent until openpilot is
  disengaged; the radar-valid filter (`DIVERGENCES_preap_radar_valid.md`) keeps its points out meanwhile (camera leads, one
  warning), never blocking engage.
- card restart in Drive: if card's first `pandaStates` sample already shows the car's mode, the panda kept a config from an
  earlier card process that this one cannot see. Then nothing is sent outside Park until the panda shows a re-init or reboot,
  so a donor setting changed meanwhile is never applied at speed; logged once. The panda (`tesla_preap.h:235-262`) would
  accept a changed config while disengaged in Drive; a panda-side check is left for a later panda image.
- Before (`92aa9ee1f` and earlier builds with the gate): sent only while disengaged and not in Drive. A shift to Drive before
  the switch to the car's mode (or less than one full 0x560 cycle after it), or a safety re-init in Drive, kept the gateway
  closed, so there was no radar for the whole drive. Variant `dc5e7d665` (local, never released) streamed in any gear and took
  its own 0x560 back out of the panda's blocked-TX total; that subtraction is replaced below.
- Refusals, v2: the panda counts every consumed 0x560 as a blocked TX (`panda/board/drivers/can_common.h:169`) and returns it as
  a rejected echo (src bus + 0xC0, `selfdrive/pandad/panda.cc:265-266`). The Pre-AP pull FSM no longer reads the all-address
  total. card counts the rejected echoes of our own pedal 0x551 (bus 0 or 2) and steering 0x488 (bus 0) frames each step and
  feeds only those (`panda_link.py` TxRefusalWatch). A 0x560 echo has another address, so it can never read as a refusal,
  whether it is counted, late or lost; and a real refusal reaches the FSM on the step its echo arrives, never held for a counter
  sample. Rejected frames of other addresses (for example 0x214, stock cruise cancel) no longer drop FULL.
- Cross-check, fail-safe: once per `pandaStates` sample the blocked-TX increase is matched against the rejected echoes seen
  (every address). A shortfall is carried one sample (an echo can arrive after the sample that counted it); one still open after
  that means echoes were lost (rx queue overflow) and is passed to the FSM as refusals, unless card sent a 0x560 in that or the
  two previous sample windows, in which case it is logged only (rate-limited to one warning per 10 s, counts only). In FULL card
  sends no 0x560, so a refused frame whose echo was lost still drops FULL, within two samples. Residual: a refusal whose echo
  is lost, within 0.3 s after our last 0x560, is not passed on from the counter; the next refused frame (the pedal runs at
  50 Hz) is.
- Why NAP needs none of this: no NAP car or control code reads `safetyTxBlocked` (git grep: openpilot nap-alpha `92b42f772`,
  where only pandad sets it and debug plot layouts show it; opendbc `508d26fcb` and `5dbe36d37`). Our Pre-AP pull FSM drops
  FULL on a panda refusal (`DIVERGENCES_preap_nap_fsm.md`).

## 3. Donor VIN substitution (NAP)
- `TeslaRadarDonorVin` (BogPilot params dir, empty by default). Empty = 17 spaces = this car: no substitution, the car's own
  0x405 VIN records reach the radar on 0x2B9 as before.
- With a donor VIN: 0x405 records 0x10 / 0x11 / 0x12 go to the radar on 0x2B9 with the donor VIN, and 0x2A9 declares 4WD when
  donor VIN character 8 is '2' or '4' (NAP `tesla_preap.h:346-389`). Still firmware-originated on bus 1 only; host 0x2B9 / 0x2A9
  stay blocked; nothing is forwarded (`disable_forwarding` stays on).

## 4. Radar position
- Before: position = 1 when flags 256 and 128 are both set (`tesla_preap.h:276` @ `8636c8044`), so a nosecone install with the
  radar on (safety param 480 = 32 | 64 | 128 | 256) sent 1. Tinkla sets 1 for its behind-nosecone flag; na-preap the same.
- NAP `f00c8cb9` takes the position from 0x560 record 0 (its Donor Radar Position setting, 0 pre-facelift S, 1 post-facelift S,
  2 Model X, default 0) and keeps the nosecone flag only as a leftover bit.
- The references disagree on what the field means. Tinkla `501c7de` (`safety_tesla.h:1355-1357`, its behind-nosecone flag) and
  na-preap treat it as the mounting: 1 behind the nosecone. NAP `@7eb09d2a7` treats it as the donor car type: `NAPRadarPosition`
  (`nap_conf.py:41,256-261`, default 0), shown as 0 pre-facelift Model S, 1 facelift Model S, 2 Model X (sunnypilot
  `nap.py:287-288`).
- Now (as NAP): a separate Radar Donor Type setting (`TeslaRadarDonorType`, /data/params_bogpilot, 0 / 1 / 2, default 0, a bad
  value is 0) goes in 0x560 record 0, and the panda takes the position from 0x560. The nosecone toggle and flag 128 no longer
  pick the value (the panda init value from flag 128 is replaced by record 0 before the gateway opens). No panda C change was
  needed for this: the panda already took position from 0x560 (only its comments were updated to say so).
- Radar Behind Nosecone is hidden from Vehicle Settings (a project decision), since it has no effect on the car any more. Its stored
  value (`TeslaRadarBehindNosecone`) is kept as is, not deleted or migrated: it still sets flag 128, which nothing reads except
  the replaced init value (NAP keeps its nosecone bit as a leftover too). Divergence from NAP and Tinkla, which both show it.
- Behavior change: before, a nosecone install with the radar on (480) sent 1; with the default it now sends 0. Keep Donor Type 0:
  the old setting sent 1, and decoded tester data shows the radar rejected 1 (radPositionMismatch). If a position fault
  persists with 0, try Model X (2); there is no promise that any value clears it. Same disengaged, not-steering rule as the
  rest of 0x560.

## 5. Radar UDS: read-only F190 and the Read Radar VIN button
- Panda: {0x641, bus 1, 8} on the TX list; the tx hook allows only NAP's six exact frames (tester present, default and extended
  session, read DID F190, flow control, tester present without reply), only with flag 256 and only while controls are not
  allowed and the steering latch is not armed (NAP `tesla_preap.h:137-172`; NAP also checks its lateral-only flag, we check the
  latch). No write, routine or security access frame can pass. 0x641 on bus 0 / 2 and any other UDS address stay blocked.
- Host: NAP's F190 reader (`radar_donor_vin.py`) without its Commissioner: no automatic start (NAP starts a read by itself on the
  0x501 vinValidity alarm). It starts only from Vehicle Settings > "Pre-AP: Read Radar VIN" (car on, not engaged, radar on).
  Engaging, lateral, the panda reporting controls allowed, or missing pandaStates stop it at once.
- The VIN read is stored only in `TeslaRadarDonorVin` and used from the next car start. It is shown on the device under the button
  only. It is kept out of log text (carlog / cloudlog / print), crash reports, the status text (NAP `card.py:216` puts it there;
  not ported), openpilot Params (so never in the initData params snapshot), settings backups and the settings dump. It IS present
  in raw CAN in the drive logs, as below. Accepted as a project decision: the donor VIN in raw CAN is treated like the car's own VIN on
  0x405, so the 0x560 path is kept and the panda does not store it any other way.
- Raw CAN in the drive logs does carry VIN bytes. The car's own VIN is on 0x405 on every Tesla. The radar's 0x651 reply and the
  panda's 0x2B9 frames are raw bus-1 traffic. The host's 0x560 frames, which carry the donor VIN, are logged in `sendcan`
  (`cereal/services.py:39`, logged), and the panda consumes each one and returns it as a rejected echo in `can` (src + 0xC0;
  `panda/board/drivers/can_common.h:169-175`, `selfdrive/pandad/panda.cc:265-266`).
- Not ported: NAP's radar VIN-learn tool (`vin_learn_radar.py`), which reprograms the radar.

## 6. Donor VIN typed by hand: Vehicle Settings > "Pre-AP: Donor VIN"
- Before: `TeslaRadarDonorVin` could only be set by the Read Radar VIN button (section 5) or over ssh, even though this
  file and `preap/radar_donor.py` both describe the value as "17 letters or digits, entered by hand". A radar that was
  VIN-learned somewhere else (so its F190 is not the VIN the owner needs sent), or one that will not answer the read,
  left no way to set it on the device.
- Now: a `button_item` row under Read Radar VIN opens the stock `Keyboard` widget
  (`openpilot/system/ui/widgets/keyboard.py`, the same widget the screen-recording rename uses) with the saved VIN in it.
  Confirm saves it; an empty box clears it (empty = 17 spaces = this car, no substitution). Same stored key as the read
  button, so the two cannot disagree.
- Validation is the same function the 0x560 stream applies, `normalize_radar_donor_vin`: upper-cased, separators dropped,
  and it must come to exactly 17 of `A-Z0-9` or it is refused with a message and nothing is written. So a half-typed VIN
  can never reach the radar. The box takes 24 characters so separators can be typed and normalized away.
- Shown only on a Pre-AP car with Bosch Radar on, and editable offroad only, as Radar Donor Type: the value is read once
  when the car starts (`interface.py` / `radar_donor.py`), so a mid-drive change would not apply anyway.
- The VIN keeps the same handling as section 5: shown on the device only, never in a refusal string, log text, crash
  reports, openpilot Params, backups or the settings dump; still present in raw CAN in drive logs.
- Tests: `openpilot/selfdrive/ui/tests/test_preap_donor_vin_row.py` (visibility per car / radar / offroad, a valid VIN
  saved, lower case and separators normalized, an empty box clearing it, short / long / bad refused with nothing written,
  refused while the car is on or the radar is off, the reason never containing the VIN, the same key as the read button,
  and what is saved being what the 0x560 stream accepts).
- Divergence from NAP: NAP's equivalent setting is edited in its own config UI; this is the same stored value behind a
  BogPilot Vehicle Settings row, with the stricter offroad-only gate.
