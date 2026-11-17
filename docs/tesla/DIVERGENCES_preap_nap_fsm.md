# Pre-AP pull FSM (NAP-style double pull) — divergences

Credit: NotAutopilot (NAP) nap-release e99f4dc75 (opendbc cc39c7fa) for the double-pull engagement, the steering-only first pull,
the brake behavior and the no-pedal cancel; BogPilot 0.9.7 na-preap 568fea947 for the earlier port. NAP paths below are
`opendbc_repo/opendbc/car/tesla/preap/` in NAP. The panda safety mode (`tesla_preap.h`) is unchanged by this work.

## Behavior
| Rule | NAP | This tree |
|---|---|---|
| One pull = steering only, a second pull < 400 ms later = speed control | `engagement.py:131` (strict `<`), `:154-161` | `preap/engagement.py` `WINDOW_MS` (:32), `_window_open` (:156), modes OFF / LOCKOUT / LAT_PENDING / LAT_ONLY / FULL |
| Pull counts only in D with the doors closed | `carstate.py:139` `check_can_engage` (engage, not the pair counter) | `engagement.py:194` `valid_pull`; DI_torque2.DI_gear and GTW_carState.DOOR_STATE_* (`tesla_ap1_can.dbc:177,232`). Stricter: an ignored pull never opens or closes a window |
| Brake in speed control drops to steering only | `engagement.py:100-110`, pedal mode only | `engagement.py:245` (T13), pedal and no pedal. No pedal: demoted on the brake edge itself and the DI's own cruise cancel is expected (`_brake_drop_until_ms`, :255); bench check B10 confirms it |
| A single re-pull in speed control drops to steering only | `engagement.py:154-161` | `engagement.py:259` (T12) |
| Hands on / EPAS refusal / EPAS fault | full disengage, `carstate.py:60-68` | LOCKOUT, fresh pull needed (`engagement.py:197`), mirrors `tesla_preap.h:52-59` |
| Pedal ENABLE only in speed control | ENABLE=1 also while "pedal engaged but not long" at zero torque (`carcontroller.py:123-128`) | single choke point `long_controller.py` `_append_pedal_command`: ENABLE=1 only in FULL (`_fsm_full`, :54), otherwise (0.0, enable=0) and counted; a counted block or a panda TX block drops FULL (T15) |
| Never ENABLE=1 while braking or with the accelerator | gas: passthrough enable=0 (`carcontroller.py:97-100`); brake: left to the panda | `_driver_holds_pedal` (:62): brake, accelerator by our test or by the panda's own test (raw INTERCEPTOR_GAS > 650, `tesla_preap.h:135`, DBC scale `tesla_ap1_can.dbc:938`, `carstate.py:31, :158-160`), and the first ENABLE=1 only one update after release (`carstate.py:248, :262`), because the panda refuses ENABLE while `gas_pressed_prev` |
| No pedal: first pull cancels a stock cruise it armed | immediate CANCEL (`engagement.py:164-173`) | same (`_nopedal_steer_only_cc`), cancel-only 0x45; a pending CANCEL is dropped if the second pull beats it |
| No pedal: second pull | spoofer re-engages stock cruise with a non-cancel 0x45 (SET_ACCEL) | the panda allows CANCEL only, so the driver's real second pull arms stock cruise; FULL only when DI_cruiseState confirms within 600 ms, otherwise "Stock cruise did not arm" |
| No pedal: stock cruise on without a counted pull | `cruiseState.enabled` only from a pull (`carstate.py:139`) | same: openpilot stays off and the stock cruise is left alone |

## Standstill (Q1, option (b))
- NAP: a double pull at a standstill enters pedal control with the current speed as the set speed (`engagement.py:142,243-246`,
  rounded to the display unit at :245) and holds with ENABLE=1 at zero torque (`carcontroller.py:123-128`). The accelerator is
  passed through with enable=0 (`carcontroller.py:97-100`); the set speed stays as captured until a stalk tip (`engagement.py:218-234`).
- This tree (`engagement.py` `PREAP_FULL_AT_STANDSTILL`, :48; option (a) refuses FULL until moving): FULL is entered but held at 0 with
  ENABLE=0 (raw <= 500, the panda's disabled limit `tesla_preap.h:249,256`), never ENABLE=1. The hold ends on the accelerator RELEASE
  edge, and only if vEgo >= 0.1 m/s then; released while still stopped = keep holding and wait for the next press. A fresh brake
  press at the stop drops to steering only (the panda drops `controls_allowed` on that edge, `safety.h:354-355`). Stricter than NAP.
- Set speed on that edge (`card.py` capture edge, `cruise.py` `capture_preap_full`): SetSpeedLimit on and the limit fresh (same gates
  as the engage set: `PreapEngageSetLimit.kph`) = limit + offset; otherwise the exact vEgo (NAP rounds to the display unit). Never
  the initial 40 km/h, never the 8 km/h floor. Q6 constant `PREAP_FULL_USE_SET_SPEED_LIMIT` (`cruise.py:17`). The same capture runs on
  a moving double pull.
- Unknown speed is refused, not held (fail closed). A non-finite speed (NaN, +-inf) is read as unknown (`engagement.py:188`).

## Display and sound
- "Steering only / Double pull for speed control" (`events.py` preapSteerOnly), "Speed control off / Steering only. Double pull to retry"
  (preapSpeedBlocked), "Stock cruise did not arm / Double pull again" (preapCruiseNotArmed, NAP teslaCCNotArmed parity). No text
  mentions braking or a hold: Pre-AP has no brake actuation. NAP has no steering-only banner.
- The steering-only banner is dropped after `PREAP_STEER_ONLY_SHOW_S` (3 s) in one lateral-only stretch and re-armed when
  lateral-only ends, so staying on steering only no longer keeps a permanent banner for the whole drive. The lateral-only
  path colour, `LONG_OVERRIDE` and the preapSpeedBlocked / preapCruiseNotArmed warnings are unaffected; see
  `DIVERGENCES_preap_engage_alert_lag.md`.
- User-facing text (these alerts and PREAP_PORT.md) names the gestures "single pull" and "double pull", after the "Double-pull vs
  single-pull" section of NAP docs-nap/engagement.md:49-57 @ e99f4dc75, and leaves the timing out. NAP's doc itself describes a
  first and a second pull inside a window; the window (strict `< 400 ms`) is unchanged and stays in the table above.
- Steering-only path colour in both model renderers (`preap_steer_only`). Speed on/off chimes on the preapLatOnly edges; Q5
  `PREAP_SPEED_CHIMES_FOLLOW_TOGGLE` (`cruise_chimes.py:69`).

## Pending decisions (constants)
Q2 `PREAP_NOPEDAL_CANCEL_ON_PULL` (:51), Q3 `PREAP_NOPEDAL_CANCEL_ON_HANDS_ON` (:54), Q4 `PREAP_NOPEDAL_FOLLOW_DRIVER_CC` (:57),
Q7 `PREAP_NOPEDAL_PULL_CANCELS_ACTIVE_CC` (:60) in `preap/engagement.py`; Q5 and Q6 as above.

## Brake held at the second pull
Decision: follow NAP; a brake-held double pull at standstill engages FULL with the standstill hold.
- NAP: the second pull has no brake term (`engagement.py:131-142`) and a brake drops speed control only on its rising edge
  (`engagement.py:101-103`), always on with no setting, so a double pull with the brake held enters pedal control. NAP sends
  ENABLE=1 under the held brake (`carcontroller.py:103-118` while long is active, enable=1 at :118; zero-torque hold at
  `:123-128` otherwise) and hides brakePressed from openpilot
  (`carstate.py:134-135`) and the brake from its panda (`tesla_preap.h:399-403`).
- This tree does not: ENABLE stays 0 (raw <= 500) under the standstill hold of Q1 (b) above, brakePressed stays visible to
  openpilot, and `tesla_preap.h:254` is unchanged, so the panda still refuses ENABLE=1 under a brake. Releasing the brake does
  not end the hold; only the accelerator does (`preap/engagement.py:281-294`). The panda allows the pull: a brake held at a stop
  has no rising edge (`safety.h:354`) and the pull still sets controls (`tesla_preap.h:107`).
- Still refused, steering only: the brake held while moving, a brake pressed on the second pull itself (a press, not a held
  brake: `safety.h:354` drops controls on that edge), unknown speed. FULL entered under the held brake drops to steering only
  if the car moves with the brake still held (`engagement.py:245`, mirror of the `safety.h:354` vehicle_moving term,
  `tesla_preap.h:151`). A new brake press after engaging drops to steering only as before (T13).
- openpilot: pedal (Pre-AP) cars lack DI STANDSTILL (carState.standstill is the DI STANDSTILL state, which a Comma Pedal car
  never reports), so the stock standstill-based brake rule would drop FULL on the held brake every frame. `selfdrived/preap_brake.py`
  `PreapHeldBrake` (tightened to track the brake held since FULL entry) is the minimal exemption: Pre-AP fingerprint
  only, preapFsmMode FULL only, the brake pressed on the frame before FULL entry and on every frame since, vEgo and vEgoRaw
  finite and < 0.1 m/s on every one of those frames. A release, a fresh press, any motion >= 0.1 m/s (filtered or raw) or a
  non-finite speed ends it and the stock rule applies; every other car (AP1, AP1 Model X) keeps the stock rule unchanged.
  Compared with NAP e99f4dc75 (`engagement.py:101-103,131-142`: no brake term on the second pull, rising edge only): NAP sends
  ENABLE=1 under a held brake (`carcontroller.py:103-118,123-128`) and hides brakePressed (`carstate.py:134-135`); this tree
  keeps ENABLE=0 and brakePressed visible.

## Not ported
NAP's pedal calibration (`nap_conf.py`): the BogPilot calibration (`preap/calib.py`) and pedal checks stay; FULL needs the pedal
detected and `is_pedal_calibrated()`.

## Interceptor bus latch
NAP @ e99f4dc75 `tesla_preap.h`:349-355 and BogPilot na-preap @ 568fea947 `safety_tesla_preap.h`:302-307 record the first
0x552 bus (`preap_pedal_can`) but still set `gas_pressed` from every 0x552. NAP userspace parses a single configured bus
(`nap_conf.pedal_can_zero`, NAP `carstate.py`:186-194) and has no live latch. This tree latches the first 0x552 as
`tesla_preap.h`:128-135 does (`tesla_preap_pedal_bus`), uses only that bus for presence and gas, and if 0x552 later
arrives on the other bus, drops FULL to steering only and never sends ENABLE=1. Stricter than both references.

## Panda TX refusals
`card.py` feeds the pull FSM (`preap_panda_tx_blocked`) the refusals of our own pedal 0x551 and steering 0x488 frames (the
v2 input below), so a refused steering frame drops speed control from FULL to steering only as a refused pedal frame does
(T15, `carstate.py` speed_block, `engagement.py` FULL demote). Stricter than counting only a refused 0x551.
A refusal within `TX_BLOCK_PRESS_IGNORE_MS` (300 ms) after a gas or brake press edge is ignored: the panda
refuses ENABLE=1 at `tesla_preap.h:254` (get_longitudinal_allowed / brake_pressed) from the first frame with INTERCEPTOR_GAS
> 650 (gas_pressed, `:135`) and while the brake is pressed, and that expected block must not drop FULL. Guard blocks and a bus mismatch still demote
immediately. A block with no recent press, or one after the window, still demotes.
The input, v2: the FSM reads refusals of our own pedal 0x551 and steering 0x488 frames, counted per address from the
panda's rejected TX echoes (src bus + 0xC0, `selfdrive/pandad/panda.cc:265-266`; `preap/panda_link.py` TxRefusalWatch,
fed by `card.py`), not the all-address `PandaState.safetyTxBlocked` total (`cereal/log.capnp:568`). So the 300 ms press
window now ignores only refusals of those two addresses; a refused frame of any other address (our 0x560 radar config, which
the panda always consumes as blocked, 0x214, the stock cruise cancel) never drops FULL. The total is kept as a fail-safe
cross-check: a blocked frame whose echo never arrived (rx overflow) is passed on as a refusal within two counter samples,
unless an own 0x560 was sent in the last 0.3 s (then logged only; card sends no 0x560 in FULL). NAP reads no blocked-TX data
at all. Before (`92aa9ee1f`): any increase of the total in FULL demoted, including our own 0x560 and other addresses.

## Own-spoof cancel echo
NAP stamps its 300 ms spoof-echo window when a stalk spoof is requested (`engagement.py:88,153`, tested at :189-190), not
when a frame is sent. This tree stamps it only when our own CANCEL frame is put on the bus (`stock_cc_spoofer.py:54`), so a
tip or a pending flag with no frame sent never opens it, and the window only swallows a cancel that is also inside the
600 ms pull window (`cruise.py:52-56`). A cancel more than 600 ms after the last pull is always real, as in
`tesla_preap.h:114-120`, even right after our own spoof, so openpilot never stays engaged while the panda drops controls.
Stricter than NAP.

## Schema
car.capnp BogPilotCarState `preapFsmMode @37`, `preapSpeedBlocked @38`, `preapSteerLatch @39`; log.capnp EventName `preapSteerOnly
@110`, `preapSpeedBlocked @111`, `preapCruiseNotArmed @112`. `ap1SpeedLimitUnknown` (@36 / @109) is carried with the identical
line of the AP1 set-speed change so the ordinals stay contiguous.
