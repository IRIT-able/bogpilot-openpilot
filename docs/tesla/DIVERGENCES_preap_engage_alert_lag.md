# Pre-AP: the resumeBlocked no-entry, the steering-only banner's lifetime, and the engage-frame stall

Entries in the `docs/tesla/DIVERGENCES.md` style, kept in their own file so they can be folded into `DIVERGENCES.md`.

All three come out of one drive on `bogpilot-tesla-fix` @ `7a683247` (a Pre-AP Model S, pedal longitudinal, Bosch radar on,
28 segments, debug pack `00000012`). None of them touches the panda, safety, the DBC or any cereal schema.

### Pre-AP pedal: the stalk pull is published as setCruise, so resumeBlocked cannot fire

- Platforms: Pre-AP Model S with pedal longitudinal (`pcmCruise` False). AP1 and no-pedal Pre-AP (`pcmCruise` True) are
  untouched: `selfdrived.py:267` is gated on `not CP.pcmCruise`, so the rule never reached them.
- Symptom: "Press Set to Engage" (`resumeBlocked`, NO_ENTRY, `events.py:775-777`) on the first stalk pull of every drive.
- Cause: `values.py:99` maps `SpdCtrlLvr_Stat == 2` (RWD, pull toward the driver) to `ButtonType.resumeCruise` for the AP1
  pcmCruise resume, and `PreApCarState` published that type unchanged. `selfdrived.py:267-268` raises `resumeBlocked` when
  `not CP.pcmCruise and CS.vCruise > 250` and an `accelCruise` / `resumeCruise` edge is seen; `CS.vCruise` is
  `V_CRUISE_UNSET` (255) until the first engage (`selfdrive/car/cruise.py:13,36`), so the first pull always matched. Pre-AP
  pedal longitudinal has no stock set to resume to — the set is captured at engage — so there was nothing for the driver to
  do about it.
- `preap/cruise.py` has carried `remap_preap_pedal_button_type` for exactly this since `e2aa1b2`, with `resumeBlocked` named
  in its docstring, and nothing ever called it (`git grep`: one hit, its own definition).
- Our behavior: with pedal longitudinal, `PreApCarState` runs every published button event through that helper, so the pull
  leaves carstate as `setCruise`. The remap is the last thing done to `button_events` before `ret.buttonEvents`
  (`preap/carstate.py`), after `pull_edge` and `cancel_edge`; everything Pre-AP reads itself (`update_button_enable`, the
  pull FSM, `fp_ret.resumePressed`, the cancel-echo filter) reads `button_states` or those edges, never the published
  types. The helper now uses the opendbc `structs` enum instead of importing `cereal` per call (opendbc never imports
  cereal, and it is called at 100 Hz).
- Side effect, intended, and NAP parity: `cruise.py:144` restores `v_cruise_kph_last` on an `accelCruise` / `resumeCruise`
  edge when `v_cruise_initialized`. A re-engage inside one drive therefore re-captures the set from ego (or the posted
  limit, `PreapEngageSetLimit`) instead of restoring the previous set. That is what the helper's docstring asks for
  ("buttonEnable initializes v_cruise from ego") and what NAP does (`engagement.py:142,243-246` @ `e99f4dc75` captures the
  current speed at engage and never a higher target). The first engage of a drive is unchanged either way:
  `v_cruise_initialized` is False at `V_CRUISE_UNSET`, so that branch was never taken then.
- Safety-relevant: no. No CAN, panda or DBC change; no change to what engages or when. The set value on the first engage
  of a drive is unchanged.
- Tests: `opendbc/car/tesla_ap1/tests/test_preap_engage_button_remap.py` (the pull publishes `setCruise` and not
  `resumeCruise`; `selfdrived`'s rule no longer matches with vCruise unset; the pull still engages and still only steers;
  `resumePressed` and the cancel / tip types unchanged; the helper maps only `resumeCruise`).
- Alternative, if the re-engage set change is not wanted: exempt Pre-AP from the `resumeBlocked` rule in `selfdrived.py`
  instead and leave the published type alone. That keeps the restore-last-set behavior and leaves the dead helper dead.

### Pre-AP: the steering-only banner stops after 3 seconds

- Platforms: Pre-AP Model S, lateral-only (the first pull of a double pull, or a demote out of FULL).
- Reference behavior: NAP has no steering-only banner at all (`DIVERGENCES_preap_nap_fsm.md`, "Display and sound").
- Before: `preapSteerOnly` is an `ET.PERMANENT` alert ("Steering only" / "Double pull for speed control",
  `events.py:1009-1011`) and `ap1_events.py` appended it on every frame while enabled and `preapLatOnly`, so a driver who
  chose to stay on steering only kept a mid-size banner for the rest of the drive.
- Our behavior: `Ap1Events` counts the frames it has shown the banner in one lateral-only stretch and stops appending it
  after `PREAP_STEER_ONLY_SHOW_S` (3 s), which is long enough to read it once: it says which mode you are in and what a
  second pull does, and neither changes while you stay in the mode. The counter resets whenever lateral-only ends — a
  disengage, or FULL — so the banner is shown again on the next first pull.
- Nothing about the state is hidden when it goes:
  - the lateral-only path colour stays, because both model renderers read `carState.bogpilot.preapLatOnly` directly
    (`ui/onroad/model_renderer.py:36-40`, `ui/mici/onroad/model_renderer.py:39`), not this event;
  - `LONG_OVERRIDE` (`gasPressedOverride`) is still appended every frame, so `carControl.longActive` stays off;
  - the actionable warnings are never timed out and do not spend the window: while `preapSpeedBlocked` is set,
    `preapSpeedBlocked` / `preapCruiseNotArmed` show instead and the banner's counter does not advance, so a stretch that
    showed a blocked warning and then cleared still has banner time left.
- Safety-relevant: no. Display only; no event that gates entry, disables or disengages is changed.
- Tests: `opendbc/car/tesla_ap1/tests/test_preap_steer_only_banner_timeout.py`.

### Pre-AP: the engage frame no longer reads params (the "System Lagging" soft disable)

- Platforms: Pre-AP Model S with pedal longitudinal.
- Symptom: "System Lagging" (`selfdrivedLagging`, SOFT_DISABLE, `events.py:888-890`) at a double pull, once, with no
  obvious trigger and no repeat for the rest of the drive.
- Evidence, same drive: `card`'s overrun counter (`selfdrive/car/step_overrun.py`, Pre-AP only) logged a **120 ms** step
  4 ms after `carControl.longActive` first rose, and a 91 ms step during the startup firmware query. The rest of the drive
  sat at a 14-16 ms worst case with about half of all steps over their 10 ms slot (about 500 per 10 s).
  `Ratekeeper.lagging` (`common/realtime.py:71-74`) trips when the 100-sample average period passes
  `interval / 0.9` = 11.11 ms, so against a baseline already near 10 ms one 120 ms stall is enough on its own to raise it.
- Cause: `PreapEngageSetLimit.kph()` runs on the enable edge (`card.py:313`) and on the Pre-AP FULL capture edge
  (`card.py:322`), both inside card's 100 Hz step, and it called `_toggles()` there. On the first call that is two lazy
  imports (`bogpilot_onroad_toggles`, `bogpilot_store`), a `ParamStore` (and its first `all_keys()`), `slc_adopted` and the
  few dozen param reads of `onroad_toggles`; on every engage after it, `invalidate()` plus the same reads again.
  `TESLA_PREAP_MODELS` is in `SLC_PLATFORMS`, so with openpilot longitudinal the helper is `active` and none of it was
  skipped.
- This is the bug `warm_engage_helpers` already documents for the AP1 helpers — "the first engage after boot stalled card
  for ~145 ms ... the next carOutput was invalid and selfdrived raised commIssue; the second press then engaged"
  (`phase3_speed_control.py:933-941`) — on a path that was never warmed: `card.py:209` built `PreapEngageSetLimit` and the
  warm-up call sat in the `op_long_set` branch, which Pre-AP does not take (`PreApCarState` has no `op_long_set`).
- Our behavior:
  - `PreapEngageSetLimit.refresh()` reads the toggles and keeps them as a snapshot; `kph()` reads the snapshot and touches
    no params. With no snapshot it still reads inline, so every other caller and the existing tests are unchanged.
  - card warms it at start, while offroad (`warm_engage_helpers(self.preap_engage_set_limit)`), and refreshes it from
    `params_thread` every `PREAP_ENGAGE_SET_REFRESH_S` (1.0 s, `preap_card_step.py`) — the same "through card's params
    snapshot, so no file IO in the step" pattern the Ignore radar HWFail option already uses. `warm_engage_helpers` takes
    any number of helpers now and prefers a helper's `refresh()`.
  - A changed setting is picked up within a second instead of at the next engage. A failed refresh keeps the previous
    snapshot and `kph()` keeps its own fail-closed 0.0.
- Two smaller first-call costs on the same frame, in `preap/long_controller.py`:
  - `PedalFeedback` was imported inside the 50 Hz pedal path (`long_controller.py:157`), so a first-time module import
    could land on the engage frame. It is a module-scope import now (`preap/carstate.py` already imported it at module
    scope, so this only ever fired on the fail-soft branch).
  - `_personality()` stat'ed and read up to two `/data` files on the pedal ACQUIRE edge. The value is primed when the
    controller is built (card start, offroad) and re-read only on frames that are **not** acquiring, at most every
    `PERSONALITY_REFRESH_S` (2.0 s); the edge reads memory. A read error keeps the last value; the 0-2 clamp and the
    default of 1 are unchanged.
- Not addressed here: the chronic baseline. About half of card's steps still run over 10 ms with a 14-16 ms worst case on
  this device, which is what makes a single hitch enough to trip the threshold. That is a separate piece of work; the
  overrun counter is the instrument for it.
- Safety-relevant: no. Same engage set from the same inputs; no CAN, panda, DBC or schema change. The only behavioral
  difference is that a settings change applies within a second rather than exactly at the next engage.
- Tests: `openpilot/selfdrive/controls/tests/test_preap_engage_set_snapshot.py` (the snapshot, that `kph()` stops reading
  toggles, that the value matches the inline read, the no-snapshot fallback, a failing refresh, and that
  `warm_engage_helpers` fills it and still warms the AP1 helpers);
  `openpilot/selfdrive/car/tests/test_preap_card_engage_warm.py` (card warms it and refreshes it off the step, and no
  refresh is in the step);
  `opendbc/car/tesla_ap1/tests/test_preap_long_engage_edge_io.py` (the engage edge does no file IO, the refresh is rate
  limited and off the ACQUIRE edge, a bad or missing file still defaults to 1, and no function-level imports are left).
- Credits: the `warm_engage_helpers` design and its AP1 fix (this tree).
