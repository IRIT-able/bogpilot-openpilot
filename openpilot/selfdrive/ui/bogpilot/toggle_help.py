"""Tap-to-explain help for BogPilot settings rows.

Tapping the TEXT area of a settings row expands the help under the row (what it does, when to use it, safety note, default,
range) and tapping it again collapses it; opening another row collapses the previous one (accordion.py). Tapping the switch /
buttons on the right of a row still only does its normal job. Opening or closing help never reads or writes a preference.

All help text lives in the HELP table below, keyed by the row's param key (or a stable page-row id), so it is easy to
edit in one place. Every string goes through tr_noop() and is translated with tr() when the row is drawn.

Facts in this table come from the consumers in this tree (planner / controls / opendbc / onroad UI). When a consumer
changes, update the matching entry; test_bogpilot_toggle_help.py checks coverage, ASCII-only text and defaults.

Text rules (the font atlas has ASCII plus a few symbols only): ASCII text, "m/s^2" and "m/s^3" for units, no angle
brackets (the renderer treats them as tags).
"""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.system.ui.lib.multilang import tr_noop

POPUP_FONT_SIZE = 42
DEFAULT_LABEL = tr_noop("Default")
RANGE_LABEL = tr_noop("Range")
WHEN_LABEL = tr_noop("When to use it")
SAFETY_LABEL = tr_noop("Safety")
RECOMMENDED_LABEL = tr_noop("Recommended")

# Shared sentences.
_ADOPT = tr_noop("This section stays off, and openpilot drives stock, until you change a setting here.")
_OFFROAD = tr_noop("Can only be changed while the car is off.")


@dataclass(frozen=True)
class HelpEntry:
  title: str
  what: str
  when: str = ""
  note: str = ""          # safety / limitation
  default: str = ""
  range: str = ""
  recommended: str = ""
  default_param: str = ""  # param key whose shipping default the tests compare with `default` (On/Off)


HELP: dict[str, HelpEntry] = {}


def _add(key, title, what, when="", note="", default="", range="", recommended="", default_param=None):
  assert key not in HELP, key
  HELP[key] = HelpEntry(tr_noop(title), tr_noop(what), tr_noop(when) if when else "", tr_noop(note) if note else "",
                        tr_noop(default) if default else "", tr_noop(range) if range else "",
                        tr_noop(recommended) if recommended else "", default_param or "")


# ---------------------------------------------------------------- Alerts and Sounds
_add("GreenLightAlert", "Green Light Alert",
     "Plays a short alert, \"Light turned green\", when you are stopped with no lead car and the driving model predicts that the "
     "car can move off again. It does not read traffic lights: it only follows what the model predicts, so it can be wrong.",
     when="If you often wait at lights and want a nudge when traffic starts moving.",
     note="Always check the road yourself. The alert is lowest priority and is dropped while any stock alert is showing. "
          "It uses the Prompt volume.",
     default="On", default_param="GreenLightAlert")
_add("LeadDepartingAlert", "Lead Departed Alert",
     "Alerts, \"Lead departed\", when you are stopped behind a tracked lead car and it pulls away: it has moved more than "
     "1 m from where it stopped and is moving faster than 1 m/s while you are still stopped.",
     when="If you tend to miss the car in front moving off.",
     note="Lowest priority; dropped while any stock alert is showing. It uses the Prompt volume.",
     default="On", default_param="LeadDepartingAlert")
_add("LoudBlindspotAlert", "Loud Car in Blindspot Alert",
     "When you start a lane change toward a lane where the blind spot sensor sees a car, the \"Car Detected in Blindspot\" alert "
     "uses the loud warning sound instead of the short prompt sound. The lane change is blocked either way.",
     when="If the normal prompt sound is easy to miss.",
     note="Only for cars that report blind spot monitoring, so the row is hidden on other cars. The sound keeps playing while "
          "the car is detected and uses the Warning Soft volume, which can never go below 25 percent.",
     default="On", default_param="LoudBlindspotAlert")
_add("SpeedLimitChangedAlert", "Speed Limit Changed Alert",
     "Shows \"Speed limit changed\" and plays the prompt sound when the posted limit seen by the Speed Limit Controller changes.",
     when="If you use the Speed Limit Controller and want to know when its target moves.",
     note="Needs the Speed Limit Controller, so the row is hidden while that is off. It uses the Prompt volume.",
     default="On", default_param="SpeedLimitChangedAlert")
_add("CruiseEngageDisengageChimes", "Cruise Engage and Disengage Chimes",
     "Plays the stock openpilot engage and disengage chimes (engage.wav and disengage.wav) when cruise turns on or off. "
     "These are the same sounds 0.9.7 used on the stock enable and disable events. On Pre-AP cars it also chimes when "
     "longitudinal drops or returns while steering stays on, a change stock openpilot does not sound.",
     when="Turn this off if the car already chimes on its own, so the two sounds do not play together.",
     note="Shown only on AP1 and Pre-AP Teslas. It does not change warning or prompt sounds. It does not add a second "
          "chime on edges that already play the stock engage or disengage sound.",
     default="On", default_param="CruiseEngageDisengageChimes")
_add("AlertVolumeControl", "Alert Volume Controller",
     "Lets you set a separate volume for each kind of alert sound. With it off, every alert follows the automatic volume, which "
     "tracks how loud the cabin is. With it on, the rows below set each category: Auto, Muted or a fixed percent.",
     when="If some sounds, such as engage and disengage chimes, are too loud or too quiet for you.",
     note="The safety alert categories (Prompt, Prompt Distracted, Warning Soft and Warning Immediate) can never go below 25 "
          "percent, even if a lower value is saved. Can only be changed while the car is off.",
     default="Off (every alert uses the automatic volume)", default_param="AlertVolumeControl")
_VOL_COMMON = " Auto follows the cabin noise level."
_add("EngageVolume", "Engage Volume", "Volume of the sound played when openpilot engages." + _VOL_COMMON,
     when="Mute it if your car already makes its own engage sound.",
     default="Auto", range="Muted, 1 to 100 percent, or Auto")
_add("DisengageVolume", "Disengage Volume", "Volume of the sound played when openpilot disengages." + _VOL_COMMON,
     when="Mute it if your car already makes its own disengage sound.",
     default="Auto", range="Muted, 1 to 100 percent, or Auto")
_add("RefuseVolume", "Refuse Volume", "Volume of the sound played when openpilot refuses an engage request." + _VOL_COMMON,
     default="Auto", range="Muted, 1 to 100 percent, or Auto")
_add("PromptVolume", "Prompt Volume",
     "Volume of prompt alerts: the low-level sounds for things like a lane departure warning, steering assist unavailable, or a "
     "car detected in the blind spot when a lane change is blocked. The Green Light and Lead Departed alerts use it too."
     + _VOL_COMMON,
     note="Safety alert: cannot go below 25 percent. The Min button sets 25 percent.",
     default="Auto", range="25 to 100 percent, or Auto")
_add("PromptDistractedVolume", "Prompt Distracted Volume",
     "Volume of the driver monitoring prompts, such as Pay Attention and Touch Steering Wheel." + _VOL_COMMON,
     note="Safety alert: cannot go below 25 percent. The Min button sets 25 percent.",
     default="Auto", range="25 to 100 percent, or Auto")
_add("WarningSoftVolume", "Warning Soft Volume",
     "Volume of the warning sound used for forward collision warnings and soft take-control requests." + _VOL_COMMON,
     note="Safety alert: cannot go below 25 percent. The Min button sets 25 percent. Critical alerts also ramp up toward "
          "full volume the longer they sound.",
     default="Auto", range="25 to 100 percent, or Auto")
_add("WarningImmediateVolume", "Warning Immediate Volume",
     "Volume of the urgent warning sound used for immediate take-control requests and the last driver monitoring warnings." + _VOL_COMMON,
     note="Safety alert: cannot go below 25 percent. The Min button sets 25 percent. Critical alerts also ramp up toward "
          "full volume the longer they sound.",
     default="Auto", range="25 to 100 percent, or Auto")
_add("AlertVolumeFloor", "Warning minimum",
     "The warning categories cannot be muted. Prompt, Prompt Distracted, Warning Soft and Warning Immediate stay at 25 percent or "
     "louder. This limit is enforced where the sound is played, so a lower saved value is ignored.",
     default="25 percent", range="25 to 100 percent")

_add("section.reset_defaults", "Reset section to defaults",
     "Sets every setting in this section back to its default, after you confirm. Settings in other sections keep their values.",
     note="Only works while the car is off.")

# ---------------------------------------------------------------- Vehicle
_add("DisableOpenpilotLongitudinal", "Disable openpilot Longitudinal Control",
     "Uses the car's own adaptive cruise control for speed and following distance instead of openpilot's gas and brake control. "
     "Steering stays with openpilot.",
     when="If you prefer the car's stock cruise behavior.",
     note="Takes effect the next time the car starts, and asks you to confirm before turning on. Features that need openpilot "
          "longitudinal control, such as the Gas / Brake menu, stop applying and the Gas / Brake pill is hidden. Turn this off here "
          "and restart the car to bring them back. Can only be changed while the car is off.",
     default="Off", default_param="DisableOpenpilotLongitudinal")
_add("EnableICIntegration", "Tesla AP1: Dash Cluster Integration",
     "Shows openpilot information on the Tesla instrument cluster. Turning it off stops all cluster messages, so the cluster stays "
     "completely stock.",
     when="Turn it off if you want the dash to look exactly as Tesla made it.",
     note="Takes effect immediately. Model S only. " + _OFFROAD,
     default="On")
_add("RegenComfortBrake", "Tesla AP1: Earlier, Gentler Regen Stops",
     "The Model S AP1 brake pedal is friction only, and regen is limited to about 1 to 1.5 m/s^2. With this on, openpilot starts "
     "slowing for a lead earlier and uses regen-sized braking instead of the stock 2.5 m/s^2 comfort braking.",
     when="If approaches to a slower lead feel late or need friction brakes.",
     note="Emergency braking limits and the stopped distance are unchanged, so collision braking still has full authority. "
          "Needs openpilot longitudinal control. Takes effect the next time the planner starts, normally the next drive.",
     default="On")
_add("TeslaPreApDoublePull", "Pre-AP: Double Pull to Engage Pedal Longitudinal",
     "Chooses how the cruise stalk engages. On: the first pull starts Always On Lateral steering only (openpilot is not engaged and the "
     "pedal is never commanded), and a second pull within 400 ms engages openpilot with Comma Pedal longitudinal. A pull or the brake "
     "while engaged drops back to steering only. Off: a single pull engages steering and pedal longitudinal together.",
     when="Turn it on if you want to be able to engage steering without the pedal controlling speed.",
     note="Shown only while Always On Lateral is available. Takes effect immediately. Pre-AP Model S with Comma Pedal longitudinal only. "
          "Cancel (push the stalk away) still ends everything. " + _OFFROAD,
     default="On")
_add("TeslaRadarEnabled", "Pre-AP: Bosch Radar",
     "For a Pre-AP Model S with a retrofitted Tesla Bosch radar. On: openpilot reads the radar, and radar leads feed Comma Pedal "
     "longitudinal and Human-Like Following. The panda passes the car data the radar needs to it. If the radar does not answer or "
     "reports a hardware fault, openpilot will not engage and shows an alert.",
     when="Only after the radar is fitted, wired to the radar bus and the panda firmware supports it.",
     note="Pre-AP Model S only. Takes effect the next time the car starts. " + _OFFROAD,
     default="Off")
_add("TeslaRadarBehindNosecone", "Pre-AP: Radar Behind Nosecone",
     "Marks the retrofit radar as mounted behind the nosecone. It no longer changes what the radar is told: the panda now "
     "takes that from Radar Donor Type, as in NAP, which keeps this flag only as a leftover.",
     when="If your retrofit radar sits behind the nosecone.",
     note="Shown only while Bosch Radar is on. Takes effect the next time the car starts. " + _OFFROAD,
     default="Off")
_add("TeslaRadarReadVin", "Pre-AP: Read Radar VIN",
     "Reads the VIN stored in the retrofit Bosch radar (the car it came from) and saves it as the donor VIN. From the next car start "
     "the panda then shows the radar that VIN, so a salvage radar does not fault on a VIN mismatch. Read only: it never writes to "
     "the radar, never reprograms it and never flashes it. The saved VIN is shown under this row on this device. It is kept out "
     "of log text, crash reports, settings and backups, but the raw radar CAN messages in drive logs contain it, as they "
     "already contain the car's own VIN.",
     when="Once after fitting a salvage radar, if the radar reports a VIN fault. Leave it alone if the radar works.",
     note="Car on, in Park, openpilot not engaged, Bosch Radar on. Engaging stops the read. Restart the car afterwards.")
_add("TeslaRadarDonorVin", "Pre-AP: Donor VIN",
     "The VIN the retrofit Bosch radar was learned to. From the next car start the panda puts this VIN in the records it "
     "sends the radar, so a salvage radar sees the car it came from instead of this one and does not fault on a VIN "
     "mismatch. Press the row to type it in: 17 letters or digits. Leave the box empty to use this car's own VIN (the "
     "default, no substitution). This is the same stored value Read Radar VIN writes; use this row when you already know "
     "the VIN, for example a radar you VIN-learned yourself, or when the radar will not answer a read. The VIN is shown "
     "on this device only. It is kept out of log text, crash reports, settings and backups, but the raw radar CAN messages "
     "in drive logs contain it, as they already contain the car's own VIN.",
     when="If the radar reports a VIN fault and you know the VIN it was learned to.",
     note="Shown only while Bosch Radar is on. Takes effect the next time the car starts. " + _OFFROAD,
     default="Empty (this car)")
_add("TeslaRadarDonorType", "Pre-AP: Radar Donor Type",
     "The car the retrofit Bosch radar came from: Model S before the facelift (the default), Model S after it, or Model X. "
     "The panda tells the radar this car type, so a salvage radar matches the car it was built for.",
     when="If the radar reports a position or configuration fault and you know which car it came from.",
     note="Shown only while Bosch Radar is on. Press to step through the three types. Takes effect the next time the car starts. "
          + _OFFROAD,
     default="Model S pre-facelift")
_add("TeslaRadarIgnoreHwFail", "Ignore Radar Hardware Fault (Pre-AP)",
     "For a Pre-AP Model S with a retrofitted Tesla Bosch radar that reports a persistent hardware fault "
     "(Radar Error: Restart the Car) even though the radar is working. Off (the default): that fault is a "
     "Radar Error, so openpilot will not engage and will soft-disable if already engaged. On: that fault is no "
     "longer a Radar Error, so openpilot can engage. The radar-valid gate still treats the hardware fault as a "
     "fault, so radar points stay out of speed control, camera leads are used, and the one Radar not ready - "
     "using camera lead warning shows while engaged. Other radar faults and a frozen track are unchanged: they "
     "never raise Radar Error here, and frozen tracks are still dropped. Takes effect immediately "
     "(the next radar cycle).",
     when="If the car shows a persistent Radar Error: Restart the Car alert from the retrofit radar even though "
          "the radar is fitted and working.",
     note="Shown only on a detected Pre-AP Model S, and only while Bosch Radar is on. Hidden until the car is "
          "identified, and never shown on AP1 or an unknown car. " + _OFFROAD,
     default="Off")
_add("TeslaHasIBooster", "Pre-AP: iBooster Swap",
     "For a Pre-AP Model S with an aftermarket iBooster. On: the brake bar of Pedals on Screen follows how far the brake pedal is "
     "pressed, read from the iBooster. Off: the bar is full while the brake is pressed and empty otherwise. If the iBooster "
     "stops reporting, the bar stays empty. Display only: braking, engagement and safety are unchanged.",
     when="Only if an iBooster is fitted and connected to the chassis bus.",
     note="Pre-AP Model S only. Takes effect the next time the car starts. " + _OFFROAD,
     default="Off")
_add("PedalStatus", "Comma Pedal",
     "Live status of the Comma Pedal. Not detected: the car has not heard the pedal lately (also shown with the car off). Needs "
     "calibration: the pedal is heard but not calibrated, so pedal longitudinal stays off. Recalibrate: the saved pedal scale is out of "
     "range, so pedal longitudinal stays off until you calibrate again. Calibrated: detected and calibrated.",
     when="Check this before relying on pedal longitudinal.",
     note="Read-only. It refreshes while this page is open.")
_add("PreApBuild", "Software",
     "The BogPilot version and the commit this build was made from, for example 0.11.2 (6476431ac). A build with changes that were not committed "
     "adds \"+ local edits\".",
     when="Quote it when you report a problem, so it can be matched to the exact code.",
     note="Read-only. Pre-AP Model S only.")
_add("PreApPandaFirmware", "Panda Firmware",
     "The firmware version the panda reported when pandad last started it. \"(out of date)\" means the panda is not running the firmware shipped "
     "with this build; \"Not reported yet\" means the panda has not answered since this build started.",
     when="Quote it when you report a problem. Cruise or steering that never arms after a pull can come from a mismatched panda.",
     note="Read-only. Pre-AP Model S only.")
_add("PedalCalibrate", "Calibrate Pedal",
     "Starts the guided calibration: keep your foot off the accelerator and brake pedals, then press the accelerator pedal fully once. The screen shows each step as it "
     "happens and whether the result was saved or why it failed.",
     when="Once after installing the pedal, and again if Pre-AP pedal longitudinal feels off or the status says Needs calibration.",
     note="Put the car in Park with the ignition on. Cancel, the car moving, a 90 second timeout or leaving the screen leave the current "
          "calibration unchanged. A reading that fails the sanity check is rejected and never saved.")
_add("CarSelection", "Car Selection",
     "Picks the car openpilot should assume when automatic detection finds no car. Auto is the normal setting. The Tesla AP1 "
     "Model S and Model X entries are fallbacks for bench or bring-up use.",
     when="Only if openpilot does not recognize your car on its own.",
     note="A car that is positively detected always wins over this choice. Tesla AP1 Model X is steering only (the car's own "
          "cruise control stays in charge) and has not been verified on a real car. Takes effect the next time the car starts. "
          + _OFFROAD,
     default="Auto")
_add("CarSelection.current", "Current selection",
     "Shows what is saved: Auto, or a fallback car. \"(fallback)\" or \"(forced)\" means a car is saved and is used only if "
     "automatic detection finds none.")

# ---------------------------------------------------------------- Stalk Controls
_add("twist_follow", "Stalk Twist Following Distance",
     "Uses the twist detent of the AP1 distance stalk to set the following time. Off uses the normal driving profile. It does not "
     "change engaging, cancelling or the stalk's speed steps.",
     when="If you like to change following distance with the stalk.",
     note="Only for Tesla AP1. " + _OFFROAD,
     default="On")
_add("tip_round", "Round Set Speed Tips",
     "When on, a stalk tip first rounds the set speed to a whole mph (whole km/h with metric units), then steps by 1 or by 5. "
     "A 14.8 mph set tipped up by 5 goes to 20 instead of 15, and 15.2 tipped down by 5 goes to 10. Off steps from the exact set speed. "
     "The cluster and the screen only show whole numbers, so the rounded set is what you read.",
     note="Needs openpilot longitudinal control and applies to the software set only. Does not change engagement, cancel, what a pull "
          "does. A value saved earlier is kept. " + _OFFROAD,
     default="On")
_add("experimental_pull", "Long Pull: Experimental Mode",
     "While openpilot is engaged, holding the stalk toward you flips Experimental Mode once. The pull that engages openpilot "
     "never flips modes, and stock resume and set-speed behavior is unchanged.",
     note="Needs openpilot longitudinal control. The hold time is set in the Experimental Pull Hold row. " + _OFFROAD,
     default="On")
_add("experimental_push", "Long Push: Experimental Mode",
     "While openpilot is disengaged, holding the stalk away from you flips Experimental Mode once. A push that starts while "
     "engaged still cancels immediately and never flips modes on that hold.",
     note="Needs openpilot longitudinal control. The hold time is set in the Experimental Push Hold row. " + _OFFROAD,
     default="On")
_add("pull_hold_s", "Experimental Pull Hold",
     "How long you hold the stalk toward you before Experimental Mode flips. Longer is harder to trigger by accident.",
     note="Only Experimental Mode timing changes. Normal engagement and cancel timing stay fixed. A damaged or invalid saved "
          "value turns the gesture off rather than making it shorter.",
     default="1.23 s", range="1.10 to 5.00 seconds, in 0.01 second steps")
_add("push_hold_s", "Experimental Push Hold",
     "How long you hold the stalk away from you before Experimental Mode flips. Longer is harder to trigger by accident.",
     note="Only Experimental Mode timing changes. Normal engagement and cancel timing stay fixed. A damaged or invalid saved "
          "value turns the gesture off rather than making it shorter.",
     default="1.23 s", range="1.10 to 5.00 seconds, in 0.01 second steps")
_add("standard_engage_cancel_fixed", "Standard Stalk Engagement / Cancel",
     "Shows that normal engage and cancel on the stalk are fixed. No timing here can be changed, and the pre-AP double pull is "
     "not supported by this build.")

# ---------------------------------------------------------------- Lane Changes
_BSM_NOTE = ("A lane change, nudged or automatic, is blocked while the blind-spot monitor reports the side occupied. If the "
             "blind-spot data is missing, stale or invalid (Tesla AP1), the side counts as occupied.")
_add("LaneChanges", "Lane Changes",
     "Master switch for lane changes with the turn signal. Off turns lane changes off. The settings below only apply while this "
     "is on.",
     note="Off disables lane changes entirely: the turn signal alone will not move the car. Until you change a row on this "
          "page, the stock behavior applies (signal, then a nudge on the wheel). " + _BSM_NOTE,
     default="On", default_param="LaneChanges")
_add("NudgelessLaneChange", "Automatic Lane Changes",
     "Changes lanes by itself after you turn on the signal, with no nudge on the wheel, once the Lane Change Delay has passed.",
     when="If you are comfortable with openpilot starting the move once it judges the lane is clear.",
     note="Only starts when steering is active, you are above the minimum speed, the blind-spot monitor reports the side clear "
          "and you are not steering against the move. If blind-spot data is missing, stale or invalid (Tesla AP1), the side "
          "counts as occupied and the change is blocked. You are still responsible for checking the lane.",
     default="Off", recommended="Off until you have tried it somewhere safe", default_param="NudgelessLaneChange")
_add("LaneChangeTime", "Lane Change Delay",
     "How long the turn signal must be on before an automatic lane change starts.",
     note="Applies to Automatic Lane Changes only.",
     default="1.0 s", range="0 to 5 seconds, in 0.1 steps")
_add("MinimumLaneChangeSpeed", "Minimum Lane Change Speed",
     "The lowest speed at which a lane change is allowed.",
     note="A built-in floor of 20 mph (32.2 km/h) always applies, so lower values do not go below that.",
     default="20 mph (32 km/h)", range="0 to 99 mph (0 to 150 km/h)")
_add("LaneDetectionWidth", "Minimum Lane Width",
     "Blocks automatic lane changes into a lane narrower than this width. The width comes from the driving model's lane lines "
     "and road edge; if it cannot be measured, the change is blocked.",
     note="Applies to Automatic Lane Changes only. 0 turns this extra check off.",
     default="0 (off)", range="0 to 15 feet (0 to 5 m)")
_add("OneLaneChange", "One Lane Change Per Signal",
     "After an automatic lane change completes, another one does not start until the turn signal has been switched off and on "
     "again.",
     default="On", default_param="OneLaneChange")

# ---------------------------------------------------------------- Gas / Brake (openpilot longitudinal control only)
_GB = "Needs openpilot longitudinal control and can only be changed while the car is off."
_add("AdvancedLongitudinalTune", "Advanced Longitudinal Tuning",
     "Opens fine tuning of the actuator delay, the strongest acceleration, and how the car brakes to a stop. " + _ADOPT,
     note=_GB + " Stock safety limits still apply.",
     default="Off")
_add("LongitudinalActuatorDelay", "Actuator Delay",
     "The time between openpilot's throttle or brake command and the car's response. Raise it if the car feels slow to react, "
     "lower it if it feels too eager or overshoots.",
     note="Until you change it, the car's own value is used. " + _GB,
     default="The car's own value", range="0 to 1 second, in 0.01 steps")
_add("MaxDesiredAcceleration", "Maximum Acceleration",
     "Limits the strongest acceleration openpilot can command.",
     when="Lower it for gentler launches.",
     note=_GB,
     default="4.0 m/s^2", range="0.1 to 4.0 m/s^2, in 0.1 steps")
_add("StopAccel", "Stop Acceleration",
     "How hard openpilot brakes to hold a stop. More negative brakes harder.",
     note="Never softer than -0.5 m/s^2, so the car always holds a stop. Out-of-range saved values are clamped. Until you change "
          "this row, the car's own value is used. " + _GB,
     default="The car's own value", range="-4.0 to -0.5 m/s^2, in 0.01 steps")
_add("StoppingDecelRate", "Stopping Rate",
     "How fast the stopping brake ramps in as the car comes to a stop. Lower is gentler.",
     note="Never below 0.1 m/s^3, because a near-zero rate would ramp braking far too slowly. Out-of-range saved values are "
          "clamped. Until you change this row, the stock rate is used. " + _GB,
     default="1.0 m/s^3", range="0.1 to 1.0 m/s^3, in 0.01 steps")
_add("LongitudinalTune", "Longitudinal Tuning",
     "Opens the acceleration and deceleration profiles, Human-Like Acceleration, Human-Like Following, Human-Like Lane Changes "
     "and Lead Detection Confidence Threshold. " + _ADOPT,
     note=_GB,
     default="Off")
_add("AccelerationProfile", "Acceleration Profile",
     "How quickly openpilot speeds up. Standard is the stock limits, Eco is gentle, Sport is firmer, and Sport+ allows the "
     "highest acceleration, up to 4 m/s^2 at low speed.",
     note=_GB,
     default="Sport", range="Standard, Eco, Sport or Sport+")
_add("DecelerationProfile", "Deceleration Profile",
     "How gently openpilot slows down. Standard is the stock limit. Eco allows half of the normal braking and Eco+ a quarter, "
     "so the car coasts more.",
     note="When following a lead car, full braking is always restored, so a gentler profile never weakens braking behind a "
          "lead. " + _GB,
     default="Eco", range="Standard, Eco or Eco+")
_add("HumanAcceleration", "Human-Like Acceleration",
     "Eases the throttle at low set speeds and as you approach the set speed, instead of accelerating at the full limit.",
     note="Only limits acceleration. Braking is not affected. " + _GB,
     default="On", default_param="HumanAcceleration")
_add("HumanFollowing", "Human-Like Following",
     "Follows the way a person does: closes the gap behind a lead car that is pulling away, for quicker takeoffs, and adjusts "
     "the following distance for gentler braking.",
     note="Needs the Longitudinal Tuning switch on, and takes effect on the next drive. On a Pre-AP Model S it also needs "
          "the Bosch radar (the Pre-AP: Bosch Radar row in Vehicle Settings) on when the car starts; without it the row shows Off "
          "and cannot be changed. A saved Off is kept. " + _GB,
     default="On", default_param="HumanFollowing")
_add("HumanLaneChanges", "Human-Like Lane Changes",
     "Anticipates and tracks the cars in the next lane during a lane change, using the radar.",
     note="Needs radar and openpilot longitudinal control, so the row is hidden on a car without radar. On by default on an "
          "AP1 Model S and on a Pre-AP Model S that started with the Bosch radar on; on other cars it stays off until you "
          "change this row. A saved Off is kept. " + _GB,
     default="On", default_param="HumanLaneChanges")
_add("LeadDetectionThreshold", "Lead Detection Confidence Threshold",
     "The driving model confidence, in percent, a vehicle needs before it counts as the lead car. Lower values detect leads "
     "sooner but may react to false leads; higher values are more conservative.",
     note="Takes effect on the next drive. Stock behaviour is 50 percent. " + _GB,
     default="35 percent", range="25 to 50 percent, in 1 percent steps")
_add("CustomPersonalities", "Driving Personalities",
     "Opens the Aggressive, Standard and Relaxed profiles. Each profile sets how many seconds the car follows behind a lead car "
     "and five smoothness rows for acceleration, braking, safety gap and speed changes. The profile you pick with the distance "
     "button or the stalk is the one that applies.",
     when="If the stock following distance or the way the car speeds up and brakes behind a lead car does not suit you.",
     note="Takes effect on the next drive. Reset to Defaults restores one profile and asks you to confirm. " + _GB,
     default="Off", default_param="CustomPersonalities")
_add("SpeedLimitController", "Speed Limit Controller",
     "Follows the speed limit shown on the car's dashboard: the speed the car drives to follows the limit plus the offset you set "
     "for that speed band. You choose what it does when no limit is known and after you drive faster than the limit.",
     when="If you want the set speed to follow the posted limit.",
     note="AP1 Model S, and Pre-AP Model S with the pedal (openpilot longitudinal control). Needs a limit from the dashboard; "
          "maps and navigation sources are not available. Takes effect on the next drive. " + _OFFROAD,
     default="On", default_param="SpeedLimitController")
_add("ConditionalExperimental", "Conditional Experimental Mode",
     "Switches Experimental Mode on automatically when the conditions you choose below are met, and back off when they are "
     "not. " + _ADOPT,
     note="Warning: this has not worked on Tesla AP1 so far, so we recommend you do not turn it on. "
          "Pressing the gas or brake pedal resets it. Needs openpilot longitudinal control. " + _OFFROAD,
     default="Off", recommended="Off (not working on Tesla AP1 yet)")
_add("CESpeed", "Below (no lead)",
     "Switches to Experimental Mode when you drive below this speed and there is no lead car (above 1 m/s).",
     when="Helps the model handle slow, stop-and-go driving.",
     default="0 (off)", range="0 to 99 mph (0 to 150 km/h)")
_add("CESpeedLead", "Below, With Lead",
     "Switches to Experimental Mode when you drive below this speed while following a lead car.",
     default="0 (off)", range="0 to 99 mph (0 to 150 km/h)")
_add("CECurves", "Curve Detected Ahead",
     "Switches to Experimental Mode when a curve is detected ahead and you are driving faster than about 5 m/s, so the model "
     "can pick a suitable speed.",
     default="Off", default_param="CECurves")
_add("CECurvesLead", "Curve Detected Ahead, With Lead",
     "Also uses the curve condition while you are following a lead car. Without it, curves are ignored when following a lead.",
     default="Off", default_param="CECurvesLead")
_add("CEStopLights", "\"Detected\" Stop Lights/Signs",
     "Switches to Experimental Mode when the driving model predicts a stop and there is no lead car. openpilot does not detect "
     "traffic lights or stop signs, so it may stop when there is no clear reason.",
     note="Hidden when the Predicted Stop In row is shown. The model's stop prediction is not traffic-light recognition.",
     default="On", default_param="CEStopLights")
_add("CELead", "Lead Detected Ahead",
     "Switches to Experimental Mode when the lead car is much slower than you (more than 5 m/s, about 11 mph) or is stopped "
     "(under 1 m/s). This build uses the radar's lead track.",
     default="Off", default_param="CELead")
_add("CESlowerLead", "Slower Lead",
     "Use the condition for a lead that is more than 5 m/s slower than you.",
     default="Off", default_param="CESlowerLead")
_add("CEStoppedLead", "Stopped Lead",
     "Use the condition for a lead that is stopped, moving under 1 m/s.",
     default="Off", default_param="CEStoppedLead")
_add("CEModelStopTime", "Predicted Stop In",
     "Switches to Experimental Mode when the driving model predicts the car will stop within this many seconds and there is no "
     "lead car. This is usually the model seeing a red light or stop sign ahead.",
     note="openpilot does not detect traffic lights or stop signs; in Experimental Mode the model makes end-to-end decisions "
          "and may stop without a clear reason.",
     default="8 seconds", range="0 to 9 seconds (0 is off)")
_add("CESignalSpeed", "Turn Signal Below",
     "Switches to Experimental Mode when you use a turn signal below this speed, so the model can choose a suitable speed for "
     "turns.",
     default="55 mph", range="0 to 99 mph (0 to 150 km/h; 0 is off)")
_add("CESignalLaneDetection", "Not For Detected Lanes",
     "Skips the turn signal condition when a lane wide enough for a lane change is detected on the signalled side, so ordinary "
     "lane changes do not switch modes.",
     default="On", default_param="CESignalLaneDetection")
_add("ShowCEMStatus", "Status Widget",
     "Shows on the driving screen which condition switched Experimental Mode on.",
     default="On", default_param="ShowCEMStatus")
_add("QOLLongitudinal", "Quality of Life",
     "Opens extra stopping options: Force Stop, Increase Stopped Distance and manual weather offsets.",
     note=_GB,
     default="On", default_param="QOLLongitudinal")
_add("ForceStops", "Force Stop at \"Detected\" Stop Lights/Signs",
     "When the driving model has predicted a stop for about 1 second and there is no lead car, openpilot lowers its cruise "
     "target so the car comes to a stop.",
     note="The model does not detect traffic lights or stop signs and may stop without a clear reason. Pressing the "
          "accelerator or an acceleration button overrides it, and it then waits 10 seconds before it can try again. " + _GB,
     default="Off", default_param="ForceStops")
_add("IncreasedStoppedDistance", "Increase Stopped Distance by",
     "Stops the car farther behind a lead car by treating the lead as that much closer. It never makes a present lead's "
     "distance smaller than 0.5 m.",
     note="Applies only to a lead that is present. " + _GB,
     default="0", range="0 to 10 feet (0 to 3 m)")
_add("WeatherPresets", "Weather Condition Offsets",
     "Lets you apply extra stopped distance for a weather condition you pick by hand. There is no automatic weather lookup, no "
     "location upload and no API key.",
     note="In this build only the stopped distance offsets take effect. " + _GB,
     default="Off", default_param="WeatherPresets")
_add("WeatherPresets.page", "Weather Condition Offsets",
     "Opens the manual weather condition and the per-condition offsets.")
_add("BogPilotManualWeather", "Manual Weather Condition",
     "Which condition the offsets use: Clear, Low visibility, Rain, Storm or Snow. Clear turns the offsets off. The choice stays "
     "on this device until you change it.",
     note=_GB,
     default="Clear")
for _cond, _name in (("LowVisibility", "Low Visibility"), ("Rain", "Rain"), ("RainStorm", "Rainstorms"), ("Snow", "Snow")):
  _add("weather." + _cond, _name,
       "Opens the offsets used when the manual weather condition is " + _name.lower() + ".")
  _add("IncreasedStoppedDistance" + _cond, "Increase Stopped Distance by (" + _name + ")",
       "Extra distance to stop behind a lead car while the manual weather condition is " + _name.lower() + ". Added to the "
       "regular Increase Stopped Distance setting.",
       note=_GB,
       default="0", range="0 to 10 feet (0 to 3 m)")

# ---------------------------------------------------------------- Steering Tuning
_ST = "Offroad only; applies next drive. Stock safety limits are unchanged."
_add("AdvancedLateralTune", "Advanced Lateral Tuning",
     "Unlocks manual steering values: actuator delay, steer ratio and, on torque-controlled cars, friction and lateral "
     "acceleration. Values you do not set keep the car's or the learned value.",
     note="Needs a previously detected car. " + _ST,
     default="Off", default_param="AdvancedLateralTune")
_add("SteerDelay", "Actuator Delay (steering)",
     "Seconds between a steering command and the car's response. If you never set it, the auto-learned value is kept.",
     note="Stays inside the learner's own limits. " + _ST,
     default="Auto-learned (starts at the car's delay plus 0.2 s)", range="0.15 to 0.65 seconds, in 0.01 steps")
_add("SteerRatio", "Steer Ratio",
     "The ratio of steering wheel angle to road wheel angle. Reset returns to the auto-learned value.",
     note=_ST,
     default="Auto-learned", range="0.5 to 1.5 times the car's own value, in 0.01 steps")
_add("SteerFriction", "Friction",
     "Torque-controlled cars only. Compensates for steering friction.",
     note="Needs Force Auto-Tune Off on cars that have live tuning. " + _ST,
     default="The car's own value", range="0 to 0.5, in 0.01 steps")
_add("SteerLatAccel", "Lateral Acceleration",
     "Torque-controlled cars only. Maps steering torque to turning response.",
     note="Needs Force Auto-Tune Off on cars that have live tuning. " + _ST,
     default="The car's own value", range="0.75 to 1.25 times the car's own value, in 0.01 steps")
_add("ForceAutoTuneOff", "Force Auto-Tune Off",
     "For torque cars that already tune themselves live: use the Friction and Lateral Acceleration values you set instead of "
     "the live ones.",
     note="Only shown for torque cars with live tuning. " + _ST,
     default="Off", default_param="ForceAutoTuneOff")
_add("ForceAutoTune", "Force Auto-Tune On",
     "For torque cars without built-in live tuning: learn Friction and Lateral Acceleration live while driving. The usual "
     "validity checks still apply.",
     note="Only shown for torque cars without live tuning. " + _ST,
     default="Off", default_param="ForceAutoTune")
_add("QOLLateral", "Quality of Life (steering)",
     "Opens the steering comfort options: Pause Steering Below and Turn Signal Only.",
     note=_ST,
     default="On", default_param="QOLLateral")
_add("PauseLateralSpeed", "Pause Steering Below",
     "Below this speed openpilot does not steer, and you steer yourself. 0 turns it off.",
     note="This only ever stops steering; it never adds steering. Be ready to steer below the set speed. Uses the speed units "
          "you have chosen. " + _ST,
     default="0 (off)", range="0 to 99 mph (0 to 150 km/h)")
_add("PauseLateralOnSignal", "Turn Signal Only",
     "Only pause steering below the speed while a turn signal is on. Without it, steering pauses below the speed at all times.",
     note="Has no effect while Pause Steering Below is 0. " + _ST,
     default="Off", default_param="PauseLateralOnSignal")

_add("OverrideSensitivity", "Override Sensitivity",
     "How hard you have to push the steering wheel for openpilot to let go and let you steer. 1 needs the hardest push, 5 the lightest, "
     "and the default is 2, a little firmer than 3. Steering gives control back as soon as it sees your push, and resumes by itself once you let go.",
     when="If steering lets go when you only meant to rest your hands (choose a lower number), or if you have to fight the wheel to "
          "take over (choose a higher number).",
     note="It cannot be turned off and it only moves the push needed within a safe range. The car's own hands-on detection "
          "always counts as taking over too. The safety limits on steering do not change. Only for the Tesla Model S and Model X "
          "with the original Autopilot hardware, and the Pre-AP Model S. " + _OFFROAD + " Applies the next drive.",
     default="2 (a little firmer than 3)", range="1 to 5")
_add("OverrideSensitivityPreAp", "Override Sensitivity",
     "How hard you have to push the steering wheel for openpilot to let go and let you steer. 1 needs the hardest push, 5 the lightest, "
     "and the default is 2, a little firmer than 3. If steering lets go for a light push, it comes back by itself shortly after you let go. A firm "
     "takeover (or the car refusing a steering request) ends the session: release the stalk and pull it again to re-engage. "
     "On stock cruise, also cancel and pull once more to turn TACC back on. With the pedal, a fresh pull alone is enough. "
     "A held stalk never re-engages.",
     when="If steering lets go when you only meant to rest your hands (choose a lower number), or if you have to fight the wheel to "
          "take over (choose a higher number).",
     note="It cannot be turned off and it only moves the push needed within a safe range. A light push that only pauses steering comes back by itself. "
          "A firm takeover or a refused steering request ends the session until you release the stalk and pull it again. The safety limits on steering do not change. "
          "Only for the Pre-AP Model S. " + _OFFROAD + " Applies the next drive.",
     default="2 (a little firmer than 3)", range="1 to 5")


def override_sensitivity_help_key(car_fingerprint=None):
  """Pre-AP uses the stalk-reengage help; every other car keeps the existing OverrideSensitivity entry."""
  from opendbc.car.tesla_ap1.values import is_preap
  return "OverrideSensitivityPreAp" if is_preap(car_fingerprint) else "OverrideSensitivity"

# ---------------------------------------------------------------- Driving Model

_add("driving_model.status", "Driving Model",
     "Shows which driving model is in use. Only the Default (stock) model ships with BogPilot. Models added later appear on "
     "this page.",
     note="A model is only used if it passes a hash check and the 0.11.2 compatibility check. If one fails, the stock model "
          "is used instead and an alert tells you why.")
_add("driving_model.default", "Default (stock) model",
     "The stock openpilot driving model. It is always listed first, it is the default, and it is what the car uses whenever "
     "another model is missing or fails its checks.",
     when="Pick it to go back to stock after trying another model.",
     default="Selected")
_add("driving_model.entry", "Installed model",
     "A model installed on this device. Its hashes and compatibility with 0.11.2 are checked before it can be selected. "
     "\"Not usable\" means a check failed and it cannot be selected.",
     note="A switch only takes effect when the car is off, from the next drive. It is checked again at start, and the stock "
          "model is used if it fails.")

# ---------------------------------------------------------------- Visuals
_add("ConfidenceBall", "Confidence Ball",
     "Shows a small ball on the edge of the driving screen (the left edge by default) whenever the car is on. Its height follows "
     "how sure the driving model is that it can keep driving without help: high when sure, low when not. While openpilot is "
     "engaged it is green above 50 percent, orange under 50 percent and red under 20 percent. While you are overriding it is "
     "a consistent white ball with a grey shade. While openpilot is off it is slate grey when the model is sure, so it blends "
     "in, and fades to white as the model gets less sure, so it stands out. Its colour never suggests openpilot is driving; "
     "the screen border shows whether openpilot is engaged. It hides while a full-screen alert is showing.",
     when="If you want a glance-able sign of how sure the driving model is, like the comma four's ball.",
     note="Display only: it never steers, brakes or changes how the car drives. The ball is the model's own prediction that "
          "you will need to brake or steer soon, not a guarantee. With no model data it sits at the bottom (red only while engaged).",
     default="On", default_param="ConfidenceBall")
_add("ConfidenceBallSide", "Confidence Ball Side",
     "Picks the left or right edge of the driving screen for the confidence ball.",
     when="Pick Right if something in the car, such as a mirror, hides the left edge.",
     note="Has no effect while the Confidence Ball is off. Display only.",
     default="Left")
_add("AdjacentLeadsUI", "Adjacent Lead Speed Boxes",
     "Shows boxes with the speed of radar-tracked cars in the lanes beside you while openpilot is engaged.",
     note="Needs a car with radar.",
     default="On", default_param="AdjacentLeadsUI")
_add("BlindSpotPath", "Blind Spot Lane Fill",
     "Tints the lane beside you red while the car's blind spot monitor reports a vehicle there.",
     note="Only on cars that report blind spot data (Tesla AP1). Needs Driving Screen Widgets.",
     default="On", default_param="BlindSpotPath")
_add("BlindSpotMetrics", "Blind Spot Border",
     "Draws the red blind-spot half-border: the half of the screen border on the side where the car's blind spot monitor "
     "reports a vehicle turns red. This switch alone controls it. The other border colors always draw: green while "
     "engaged, gray while you override openpilot, dark blue while disengaged, red while engaged in Traffic Mode, and the "
     "flashing lane-keep fault border.",
     note="Only on cars that report blind spot data. The lane-keep fault border and critical alerts take the whole border.",
     default="On", default_param="BlindSpotMetrics")
_add("BSMFlameTracks", "Bog to the Future",
     "Draws two burning tire tracks down the lane beside you, for 3 seconds, behind a car that passes you faster than 88 mph "
     "(about 142 km/h). Display only: it never steers, brakes or changes how the car drives.",
     when="It fires when the blind spot lane fill on that side switches off, with a radar-tracked car in that lane going over "
          "88 mph. So it needs a car that reports blind spot data (Tesla AP1), the Blind Spot Lane Fill on, and a radar "
          "that tracks the car in the next lane. On most drives it never shows.",
     note="Display only, no effect on control. Needs Blind Spot Lane Fill and Driving Screen Widgets.",
     default="On", default_param="BSMFlameTracks")
_add("OnroadDistanceButton", "Driving Personality Button",
     "Shows a button in the driver-side corner that shows and changes the driving personality.",
     note="Needs Quality of Life Visuals and openpilot longitudinal control.",
     default="On", default_param="OnroadDistanceButton")
_add("PedalsOnUI", "Pedals on Screen",
     "Shows three vertical bars on the driving screen. Left: the brake pedal, red, or mauve while openpilot is braking. Middle: "
     "regen power from the motor, blue, or light blue while openpilot controls your speed; full at 55 kW. Right: acceleration, "
     "mint for the accelerator pedal or dark green for openpilot's request; your pedal always wins. On a Pre-AP Model S the "
     "brake bar is full while the brake is pressed, or follows the pedal with Pre-AP: iBooster Swap on. A bar stays empty when "
     "its signal is not available.",
     note="Needs Driving Screen Widgets and openpilot longitudinal control.",
     default="On", default_param="PedalsOnUI")
_add("ScreenRecorder", "Screen Recorder Button",
     "Shows a button on the driving screen that starts and stops a screen recording of what the UI draws.",
     note="Recordings are saved on the device in 5 minute segments. Recording will not start, or move on to a new segment, with "
          "under 1 GiB free. Nothing is recorded until you press the button.",
     default="On", default_param="ScreenRecorder")
_add("DriverCamera", "Show Driver Camera When In Reverse",
     "Shows the driver camera feed while the car is in reverse, after the car has been in reverse for half a second, and returns "
     "to the road view when you leave reverse.",
     note="Needs Quality of Life Visuals. Alerts still draw on top of the camera view.",
     default="On", default_param="DriverCamera")
# Camera View: BogPilot 0.9.7 wording (frogpilot/ui/qt/offroad/visual_settings.cc:118). Help text carries no tags (see _clean),
# so the bold first sentence of the 0.9.7 description is plain text here.
_add("CameraView", "Camera View",
     "Select the active camera view. This is purely a visual change and doesn't impact how openpilot drives!",
     default="Auto")
_add("QOLVisuals", "Quality of Life Visuals",
     "Parent switch for the reverse camera and the driving personality button. When off, both stay off.",
     default="On", default_param="QOLVisuals")
_add("CustomUI", "Driving Screen Widgets",
     "Parent switch for the blind spot lane fill and the pedals on screen. When off, those stay hidden.",
     default="On", default_param="CustomUI")
_add("LowerSpeedLimitsBox", "Lower Speed Limits Box",
     "Lower speed limits box (if obstructed by rear view mirror). Moves the top-left speed row (MAX, speed, and the "
     "LIMIT tile) down to the slot below where the accel graph sits, so it stays readable under a rear-view mirror. "
     "Display only.",
     default="Off", default_param="LowerSpeedLimitsBox")
_add("ShowAccelGraph", "Show Accel Graph",
     "Shows the acceleration request graph on the driving screen. It can slow UI draw time, so it is off by default "
     "and lives under Developer. Display only.",
     default="Off", default_param="ShowAccelGraph")
_add("DeveloperHUD", "Developer HUD",
     "Shows a live data panel at the top right of the driving screen, two columns of ten values: acceleration, distance to the "
     "lead car, lead speed, speed relative to the lead, desired lateral acceleration, real steering angle, desired steering "
     "angle, actual lateral acceleration, GPS altitude and memory use. Display only: it never changes how the car drives.",
     when="Use it while tuning on the road or on the bench. A dash means that value has no live source right now.",
     note="Colors warn: orange or red for a close or closing lead car, a large steering angle, or high memory use.",
     default="Off", default_param="DeveloperHUD")
_add("FPSCounter", "FPS Display",
     "Shows one line along the bottom of the driving screen: the frames per second the screen is drawing now, the lowest and "
     "highest for this drive, and the average over the last minute. Display only: it never changes how the car drives.",
     when="Use it to check that the screen keeps up while you change visual settings.",
     default="Off")
_add("DeveloperSidebar", "Developer Sidebar",
     "Shows a 300 pixel panel on the right of the driving screen while you drive, with up to seven live metrics. The driving view "
     "gets narrower by the width of the panel. Each tile shows one metric you pick in the Metric rows below the switch: "
     "acceleration (now and the highest this drive), the steering delay, the steer ratio and stiffness the car has learned, the "
     "share of the drive spent with steering or the gas and brake engaged, the steering angle and the acceleration command. "
     "Display only: it never changes how the car drives.",
     when="Use it while tuning on the road or on the bench.",
     note="A dash means that value has no live source right now. Engagement shares and the maximum start over with each drive. "
          "Acceleration is shown in m/s squared. On a car that steers by angle, the torque based metrics are not offered.",
     default="Off", default_param="DeveloperSidebar")
for _n in range(1, 8):
  _add(f"DeveloperSidebarMetric{_n}", f"Developer Sidebar Metric {_n}",
       "Chooses the metric shown in this tile of the Developer Sidebar, or None to leave the tile out. The tiles you leave out "
       "close up, and the rest are spaced evenly down the panel.",
       when="Pick the values you are watching while you tune.",
       default="Tiles 1 to 7 show acceleration, max acceleration, steering delay, friction, lateral acceleration, steer ratio "
               "and steer stiffness. Torque based ones are not offered on a car that steers by angle.")
_add("ShowIP", "Show IP in Sidebar",
     "Shows the Wi-Fi address of the device in place of the five signal dots in the sidebar. The network type stays below it. "
     "It reads the address from the device itself and sends nothing over the network.",
     when="Use it to find the address for an SSH session without opening the Wi-Fi settings.",
     note="It shows -- when the device has no Wi-Fi address.",
     default="Off", default_param="ShowIP")
_add("NavigationUI", "Navigation Widgets",
     "Parent switch for the posted speed limit tile on the driving screen.",
     default="On", default_param="NavigationUI")
_add("ShowSpeedLimits", "Show Speed Limits",
     "Shows the posted speed limit tile. The limit comes from the speed limit the Tesla dashboard reports, not from maps or "
     "the camera, and only a limit the dash reports as valid and current is used.",
     note="Needs Navigation Widgets. It only displays the limit and never changes the car's speed.",
     default="On", default_param="ShowSpeedLimits")

# ---------------------------------------------------------------- Device Controls
_add("DeviceManagement", "Device Settings",
     "Master switch for the shutdown timer and the low-voltage cutoff. Off keeps stock power protection.",
     note="Cannot make the device stay on longer or run the battery lower than stock allows. " + _OFFROAD,
     default="Stock behavior")
_add("DeviceShutdown", "Device Shutdown Timer",
     "How long the device stays on after the car is turned off before it powers down.",
     note="Never longer than the stock 30 hours. Needs Device Settings on. Applies at the next power check. " + _OFFROAD,
     default="30 hours (stock)", range="5 minutes, 15 / 30 / 45 minutes, then 1 to 30 hours")
_add("NoLogging", "Disable Logging",
     "Stops the device recording drives: no rlog / qlog, no camera video and no boot log. Recorded data cannot be recovered later.",
     note="Needs Device Settings on and Tuning Level Advanced or higher; otherwise logging runs as stock. "
          "Applies from the next drive. " + _OFFROAD,
     default="Off (drives are recorded)")
_add("LowVoltageShutdown", "Low-Voltage Cutoff",
     "The car battery voltage below which the device shuts down to protect the battery. Higher shuts down earlier.",
     note="Never lower than the stock 11.8 V. Needs Device Settings on. " + _OFFROAD,
     default="11.8 V (stock)", range="11.8 to 12.5 V, in 0.1 steps")
_add("ScreenManagement", "Screen Settings",
     "Master switch for screen brightness, timeout and standby. Off keeps stock display behavior.",
     note=_OFFROAD,
     default="Stock behavior")
_add("ScreenBrightness", "Screen Brightness (Offroad)",
     "Screen brightness while the car is off. Auto uses the stock automatic brightness.",
     note="Needs Screen Settings on. " + _OFFROAD,
     default="Auto", range="1 to 100 percent, or Auto")
_add("ScreenBrightnessOnroad", "Screen Brightness (Onroad)",
     "Screen brightness while driving. Auto uses the stock automatic brightness.",
     note="Alerts raise the screen to at least 30 percent so they stay visible. Needs Screen Settings on. " + _OFFROAD,
     default="Auto", range="0 to 100 percent, or Auto")
_add("ScreenTimeout", "Screen Timeout (Offroad)",
     "How long the screen stays on after your last touch while the car is off.",
     note="Needs Screen Settings on. " + _OFFROAD,
     default="30 seconds", range="5 to 60 seconds, in 5 second steps")
_add("ScreenTimeoutOnroad", "Screen Timeout (Onroad)",
     "How long the screen stays on after your last touch while driving.",
     note="Needs Screen Settings on. " + _OFFROAD,
     default="10 seconds (stock)", range="5 to 60 seconds, in 5 second steps")
_add("StandbyMode", "Standby Mode",
     "Lets the screen go to sleep after the timeout even while the car is on. Alerts and engagement changes wake it.",
     when="If you want a dark screen while driving.",
     note="Needs Screen Settings on. " + _OFFROAD,
     default="Off", default_param="StandbyMode")
_add("ScreenRecorderControl", "Screen Recorder",
     "Starts or stops a screen recording of what the UI draws. Recordings are saved on the device in 5 minute segments.",
     note="Refuses to start with under 1 GiB free. " + _OFFROAD)

# ---------------------------------------------------------------- Utilities
_add("DebugMode", "Debug Mode",
     "Shows BogPilot developer metrics on the next drive. It is not a permanent setting: it clears itself on the following "
     "offroad transition.",
     when="Only when you are troubleshooting.",
     note=_OFFROAD,
     default="Off", recommended="Off")
_add("BogPilotDefaultsReset", "Reset BogPilot Settings to Defaults",
     "Restores BogPilot's shipping defaults. A local backup is written first, and the reset is refused if the backup fails. "
     "Vehicle identity, calibration and cloud credentials are kept. No reboot.",
     note="Driving preferences change. Tuning sections such as Gas / Brake and Lane Changes go back to stock behavior until you "
          "change a setting in them again. " + _OFFROAD)
_add("DoToggleResetStock", "Reset Toggles to Stock openpilot",
     "Queues a reset of the openpilot toggles to stock values for the next time openpilot restarts. A backup is written first. "
     "Nothing changes right now and the device does not reboot.",
     note="Driving preferences change at the next restart. " + _OFFROAD)
_add("ClearCaches", "Clear Caches",
     "Deletes the download and model caches and the soft API cache. Settings backups, device identity and the operating system "
     "are never touched.",
     note=_OFFROAD)
_add("LocalDebugReport", "Save Debug Report Locally",
     "Writes a diagnostics file on this device with a few non-secret settings, the version and the names of error logs. Nothing "
     "is uploaded and the dongle ID and VIN are left out.",
     note=_OFFROAD)

# ---------------------------------------------------------------- Data
_add("data:Delete Driving Data", "Delete Driving Data",
     "Deletes the local driving data and footage stored on this device. Drives you have marked as preserved are kept.",
     when="To free space.",
     note="Cannot be undone. Asks you to confirm. " + _OFFROAD)
_add("data:Delete Error Logs", "Delete Error Logs",
     "Deletes the error logs stored on this device.",
     note="Cannot be undone. Asks you to confirm. " + _OFFROAD)
_add("data:Screen Recordings", "Screen Recordings",
     "Delete, delete all or rename the screen recordings saved on this device.",
     note="Recording must be stopped. Management is refused while a recording lock or partial file exists. " + _OFFROAD)
_add("data:Storage Use", "Storage Use",
     "Shows how much space driving data, error logs and screen recordings use. Read only.")
_add("data:Toggle Backups", "Toggle Backups",
     "Back up, restore or delete saved copies of your BogPilot feature toggles. Only audited, non-secret preferences are saved, "
     "and nothing leaves the device.",
     note="A backup only restores when the installed software, schema, platform and vehicle match exactly. " + _OFFROAD)
_add("data:Toggle Backups:BACKUP", "Back Up Toggles",
     "Saves a copy of your current feature toggles on this device.",
     note=_OFFROAD)
_add("data:Toggle Backups:DELETE", "Delete a Toggle Backup",
     "Deletes one compatible toggle backup that you pick.",
     note="Cannot be undone. " + _OFFROAD)
_add("data:Toggle Backups:DELETE ALL", "Delete All Toggle Backups",
     "Deletes every validated toggle backup. Older, system or executable archives are kept.",
     note="Cannot be undone. " + _OFFROAD)
_add("data:Toggle Backups:RESTORE", "Restore a Toggle Backup",
     "Lets you pick a backup and shows a preview of the changes first. Nothing changes until you apply the preview.",
     note=_OFFROAD)
_add("data:Toggle Backups.scope", "Scope",
     "Which preferences a toggle backup covers. Security, calibration, identity, driver monitoring and cloud keys are excluded, "
     "and nothing is uploaded.")
_add("data:Apply Preview", "Apply Preview",
     "Applies only the listed, compatible preference changes after writing a backup of your current settings. No restart or "
     "reboot is done.",
     note="Driving behavior preferences may change. " + _OFFROAD)
_add("data:BogPilot Stats", "BogPilot Stats",
     "View or reset the usage statistics BogPilot has collected on this device.",
     note="Stats stay local. Resetting keeps driving logs and recordings. " + _OFFROAD)
_add("data:BogPilot Stats:RESET", "Reset BogPilot Stats",
     "Clears the collected BogPilot statistics. Driving logs and recordings are kept.",
     note="Cannot be undone. Asks you to confirm. " + _OFFROAD)
_add("data:BogPilot Stats:EXPORT", "Export BogPilot Stats",
     "Writes a local JSON copy of the BogPilot statistics. It never uploads anything, and an existing file is not overwritten.",
     note="Asks you to confirm. " + _OFFROAD)
_add("data:View Stats", "View Stats", "Shows the saved BogPilot statistics. Read only.")
_add("data:Screen Recordings:DELETE", "Delete a Screen Recording",
     "Deletes one screen recording that you pick.",
     note="Cannot be undone. Recording must be stopped. " + _OFFROAD)
_add("data:Screen Recordings:DELETE ALL", "Delete All Screen Recordings",
     "Deletes all completed screen recordings on this device.",
     note="Cannot be undone. Recording must be stopped. " + _OFFROAD)
_add("data:Screen Recordings:RENAME", "Rename a Screen Recording",
     "Renames one screen recording. An existing file is never overwritten.",
     note="Recording must be stopped. " + _OFFROAD)

# Rows that deliberately have NO help popup (they keep the stock inline description, or have nothing to explain).
SKIPPED: dict[str, str] = {
  "Reset Steering row buttons (Reset Actuator Delay / Steer Ratio / Friction / Lateral Acceleration / Override Sensitivity)":
    "action buttons named by the value they reset; the value row above carries the help",
  "-0.01 / +0.01 and -1 / +1 stepper rows": "no text area (the whole row is the buttons)",
  "Weather IncreaseFollowing / ReduceAcceleration / ReduceLateralAcceleration rows": "hidden: no consumer in this build",
  "CENavigation, CENavigationIntersections, CENavigationTurns, CENavigationLead rows": "hidden: no navigation producer in this build",
  "TurnDesires": "row item exists but is not registered on any page in this build",
  "Alert Volume Controller: Storage row": "informational text about the file store; no choice to explain",
  "Vehicle Info rows": "read-only CarParams readouts",
  "High-Quality Recording, Raise Temperature Limits, Offroad Screensaver, Flash Panda, Force Drive State, "
  "BogPilot Backups": "unavailable placeholders; their inline description already says why they are unavailable",
  "Status rows, Total local data, per-category Storage Use rows, Preview Changes rows, Protected rows, Baseline, "
  "Affected keys": "live readouts, not settings",
  "BACK header rows and the Gas / Brake, Steering and Device 'Back' rows": "navigation",
  "Lock notes (Locked: needs longitudinal control, Not all rows are shown)":
    "the row is the explanation: it names what is missing and which setting, if any, unlocks the page",
  "Stock Toggles, Device, Software and Developer pages (stock openpilot)":
    "stock rows already expand their own description when you tap the title; left unchanged",
  "MICI layout": "BogPilot settings pages are TICI layout only in this tree",
}

# Registered param keys that deliberately have no help entry (key -> reason). Checked by the coverage test.
SKIPPED_KEYS: dict[str, str] = {
  "CENavigation": "hidden: no navigation producer in this build",
  "CENavigationIntersections": "hidden: no navigation producer in this build",
  "CENavigationTurns": "hidden: no navigation producer in this build",
  "CENavigationLead": "hidden: no navigation producer in this build",
  "TurnDesires": "not registered on any page in this build",
}

# Row titles (text only rows) that deliberately have no help popup. Checked by the native page-walk test.
SKIPPED_TITLES: frozenset[str] = frozenset({
  "Status", "Back", "BACK", "Storage", "Total local data", "Baseline", "Affected keys",
  # Lock notes: the row is the explanation.
  "Locked: needs longitudinal control", "Not all rows are shown",
  # Steering Tuning reset buttons: named by the value they reset; the value row above carries the help.
  "Reset Actuator Delay", "Reset Steer Ratio", "Reset Friction", "Reset Lateral Acceleration", "Reset Override Sensitivity",
  # Unavailable placeholders: their inline description already says why they are unavailable.
  "Flash Panda", "Force Drive State", "BogPilot Backups", "High-Quality Recording",
  "Raise Temperature Limits", "Offroad Screensaver",
})


# ---------------------------------------------------------------- Main menu rows (0.9.7 layout)
_add("menu.tuning_level", "Tuning Level",
     "Chooses how many settings the BogPilot menu shows. Minimal shows the basics, Standard adds the common options, Advanced "
     "adds the fine tuning, and Developer shows every setting. A setting that is hidden keeps its value; only the menu row is "
     "hidden, and a setting above your level reads as its default.",
     when="Pick Minimal or Standard for a simpler menu. Pick Developer to see everything.",
     note="Developer shows settings that change how the car drives. The level can only be changed while the car is off.",
     default="Developer, until you pick a level; your choice is then saved")
_add("menu.alerts_sounds", "Alerts and Sounds",
     "The alert volume controller and the BogPilot alerts.")
_add("menu.driving", "Driving Controls",
     "Choose the driving model, tune gas and brake, and adjust steering and lane changes.")
_add("menu.navigation", "Navigation",
     "The navigation widgets shown on the driving screen, such as the posted speed limit tile.")
_add("menu.system", "System Settings",
     "Local data and statistics, device and screen controls, and utilities such as resetting settings to the BogPilot defaults.")
_add("menu.theme", "Appearance",
     "The driving screen visuals: blind spot warnings, widgets, sidebar metrics and Bog to the Future.")
_add("menu.vehicle", "Vehicle Settings",
     "Pick the car, change vehicle controls, set up the stalk controls and see vehicle info.")


# Row ids that appear on a page more than once under different titles map to the same entry here.
ALIASES: dict[str, str] = {
  "group:advanced": "AdvancedLongitudinalTune",
  "group:conditional": "ConditionalExperimental",
  "group:profiles": "LongitudinalTune",
  "group:qol": "QOLLongitudinal",
  "group:personalities": "CustomPersonalities",
  "group:slc": "SpeedLimitController",
  "qol": "QOLLongitudinal",
  "longitudinal": "LongitudinalTune",
  "weather": "WeatherPresets.page",
  "system:Device Settings": "DeviceManagement",
  "system:Screen Settings": "ScreenManagement",
  "driving_model": "driving_model.entry",
}


def resolve(key: str) -> HelpEntry | None:
  return HELP.get(ALIASES.get(key, key))


# ---------------------------------------------------------------------------------------------------------------------
# Text building (pure, no raylib)
# ---------------------------------------------------------------------------------------------------------------------
def _clean(text: str) -> str:
  # The popup renderer reads angle brackets as tags; entries must never contain them, but never let one through.
  return text.replace("<", "(").replace(">", ")")


def help_html(entry: HelpEntry, translate=None, extra: str = "", heading: bool = True) -> str:
  """HTML (tags the HtmlRenderer understands) for one help text. `heading=False` drops the title (an inline row shows it)."""
  t = translate if translate is not None else (lambda s: s)
  parts = (["<h2>" + _clean(t(entry.title)) + "</h2>"] if heading else []) + ["<p>" + _clean(t(entry.what)) + "</p>"]
  if extra:
    parts.append("<p>" + _clean(extra) + "</p>")

  def section(label, body):
    if body:
      parts.append("<h4>" + _clean(t(label)) + "</h4>")
      parts.append("<p>" + _clean(t(body)) + "</p>")

  section(WHEN_LABEL, entry.when)
  section(SAFETY_LABEL, entry.note)
  section(RECOMMENDED_LABEL, entry.recommended)
  section(DEFAULT_LABEL, entry.default)
  section(RANGE_LABEL, entry.range)
  return "".join(parts)


def help_plain(entry: HelpEntry, translate=None) -> str:
  t = translate if translate is not None else (lambda s: s)
  bits = [t(entry.title), t(entry.what)]
  for label, body in ((WHEN_LABEL, entry.when), (SAFETY_LABEL, entry.note), (RECOMMENDED_LABEL, entry.recommended),
                      (DEFAULT_LABEL, entry.default), (RANGE_LABEL, entry.range)):
    if body:
      bits.append(t(label) + ": " + t(body))
  return "\n".join(bits)


# ---------------------------------------------------------------------------------------------------------------------
# Wiring
# ---------------------------------------------------------------------------------------------------------------------
def attach_help_first(item, *keys: str, extra=None):
  """attach_help with fallbacks: the first key that has an entry wins."""
  for key in keys:
    if resolve(key) is not None:
      return attach_help(item, key, extra)
  return item


def _gate(item, key: str):
  """Tuning Level: hide the row while the chosen level is below the level the row needs. Rows with no level always show."""
  if not hasattr(item, "set_level_gate"):
    return
  from openpilot.selfdrive.ui.bogpilot import tuning_level
  if tuning_level.row_level(key) is not None:
    item.set_level_gate(lambda k=key: tuning_level.row_visible(k))


def attach_help(item, key: str, extra=None, key_fn=None):
  """Make a ListItem's text area an accordion that shows the help for `key` under the row. The switch / buttons are untouched.

  extra: optional callable returning live text (for example a model's hash line) shown under the help.
  key_fn: optional callable returning a help key, re-resolved when the accordion opens (used to swap Pre-AP wording).
  Unknown keys leave the row alone, so a row without an entry keeps the stock inline description.
  """
  _gate(item, key)
  entry = resolve(key)
  if entry is None or not hasattr(item, "set_description") or not hasattr(item, "set_description_opened_callback"):
    return item  # unknown key, or a stand-in row that is not a ListItem: leave it alone
  from openpilot.selfdrive.ui.bogpilot import accordion
  from openpilot.system.ui.lib.multilang import tr
  def _html():
    live = resolve(key_fn()) if key_fn is not None else entry
    return help_html(live or entry, tr, extra() if callable(extra) else (extra or ""), heading=False)
  item.set_description(_html)
  accordion.attach(item)
  item.help_key = key
  return item
