"""BogPilot > Driving Controls > Gas / Brake > Speed Limit Controller, runtime consumer (phase 3).

Faithful port of 0.9.7 bogpilot-tesla 04d560f3e:
  bogpilot/controls/lib/speed_limit_controller.py   (SpeedLimitController: limits, confirmation, fallback,
                                                      override, Offset1..7 bands)
  bogpilot/controls/lib/bogpilot_vcruise.py:70-171 (merge order + AP1 software stalk authority)
  bogpilot/controls/bogpilot_planner.py:172-188    (bogpilotPlan SLC fields)
onto the 0.11.2 AP1 architecture: the AP1 helpers already copied to opendbc/car/tesla_ap1/slc_raise.py
(Ap1RaiseHoldoff, Ap1SlcLimitGuard, ap1_cruise_ms) were unconsumed (parity SPD01); this module is their consumer.
Toggle resolution is the existing common/bogpilot_onroad_toggles.onroad_toggles (bogpilot_variables :1127-1156).

Authority (unchanged from 0.9.7):
  * the stock DI still engages / cancels from the physical stalk (pcmCruise). Nothing here sends CAN, touches
    panda safety, or changes carState.vCruise / the cruise state machine. Only the planner's v_cruise changes.
  * SLC off, not a legacy Tesla (AP1 S/X or Pre-AP), no openpilot long, or stale/invalid carState -> v_cruise is returned bit-for-bit, AP1
    stalk latches are cleared, and every published SLC field is zero with source "" (inactive contract).
  * tips: +-1 mph (SpdCtrlLvr 16/32), next/prev multiple of 5 mph (4/8 or 0.45 s hold), floor 1 mph, cap
    V_CRUISE_MAX (145 km/h). These are 0.9.7 AP1 constants, not params; on AP1 (pcmCruise) 0.9.7 hid and ignored
    CustomCruise / CustomCruiseLong / SetSpeedOffset, so they are not consumed here.
  * the current planner applies AP1 CSC after this module (min only), equivalent to 0.9.7's csc re-min.

Map Data (mapd v1.12.0, system/bogpilot/osm) and the Mapbox filler (user key only) are consumed through
system.bogpilot.osm.consumer.MapContext, in the exact 0.9.7 update_limits order. Not ported (no source in this tree):
Navigation (no producer exists, the source stays 0 and keeps its place in the priority order), the SpeedLimitAccepted
on-screen tap (no raylib writer; stalk accept/deny is ported). The Experimental-Mode fallback consumer
is Ap1SpeedControl.resolve_experimental (planner), also exposed as .experimental_mode for a future CEM lane.
Units: m/s internally; Offset params are stored in the device's display unit (mph or km/h, IsMetric).

gasFollowSpeed additions:
  * ACTIVATION (default_on, ACTIVATION_POLICY below). The registry default of SpeedLimitController is "1" (0.9.7 shipped
    it ON for AP1) and slc_adopted() is True under default_on, so the consumer runs on AP1 Model S and on Pre-AP with the
    pedal whenever the toggle reads on; the switch in the Speed Limit Controller panel turns it off. Saved values are never
    rewritten and nothing here writes defaults. configure() still writes the SLC_ACK marker, which only the "adoption"
    policy reads.
  * EXPERIMENTAL FALLBACK semantics = "or" (EXPERIMENTAL_POLICY). With Fallback Speed = Experimental Mode the
    planner still turns experimental on when no limit is known, but a known SLC target must never cancel the
    user's ExperimentalMode toggle. Policy "replace" (0.9.7) remains selectable.
"""
from __future__ import annotations

import math
import time

from opendbc.car.tesla_ap1.slc_raise import Ap1RaiseHoldoff, Ap1SlcLimitGuard, ap1_cruise_ms

DT_MDL = 0.05                 # common/realtime.DT_MDL
CRUISING_SPEED = 5.0          # 0.9.7 bogpilot_variables.CRUISING_SPEED (== bogpilot_csc.CRUISING_SPEED)
KPH_TO_MS = 1 / 3.6
MS_TO_KPH = 3.6
MS_TO_MPH = 2.2369362920544
MPH_TO_MS = 0.44704
V_CRUISE_MAX_MS = 145 * KPH_TO_MS   # selfdrive/car/cruise.V_CRUISE_MAX (kph)
MAX_LIMIT_MS = 100.0          # speed_limit_display.MAX_LIMIT_MS: above this a dashboard limit is invalid
CONFIRM_TIMEOUT_S = 30.0      # speed_limit_controller.handle_limit_change: denied after 30 s
OFFSET_BOUNDS = {False: 99, True: 150}   # 0.9.7 longitudinal_settings.cc Offset range: +-99 mph / +-150 km/h
AP1 = "TESLA_AP1_MODELS"
# Legacy Teslas that run the SLC consumer and settings panel (Dashboard authority + OSM look-ahead).
SLC_PLATFORMS = frozenset({"TESLA_AP1_MODELS", "TESLA_AP1_MODELX", "TESLA_PREAP_MODELS"})
# Look-ahead: horizon is SLCLookaheadLower, 0-15 s, 0 = off. Higher is not used to raise early.
EARLY_DECEL_MPH_S = 2.0
EARLY_DECEL_MS2 = EARLY_DECEL_MPH_S * MPH_TO_MS
LOOKAHEAD_HORIZON_MIN = 0
LOOKAHEAD_HORIZON_MAX = 15
LOOKAHEAD_HORIZON_DEFAULT = 5
LOOKAHEAD_CONFIRM_MS = 1.0
LOOKAHEAD_DISAGREE_MPH = 5.0
LOOKAHEAD_DISAGREE_MS = LOOKAHEAD_DISAGREE_MPH * MPH_TO_MS
LOOKAHEAD_MIN_LIMIT_MPH = 15.0
LOOKAHEAD_MIN_LIMIT_MS = LOOKAHEAD_MIN_LIMIT_MPH * MPH_TO_MS
TOGGLE_CACHE_S = 1.0
INACTIVE_SOURCE = ""          # published while SLC is not controlling (0.9.7 used "None" for "active, no source")
SLC_ACK = "Phase3SpeedLimitEnabled"   # adoption marker (file store; written only by configure())
SLC_KEYS = ("SpeedLimitController", "SLCFallback", "SLCOverride", *[f"Offset{i}" for i in range(1, 8)],
            "SLCConfirmation", "SLCConfirmationLower", "SLCConfirmationHigher", "ShowSLCOffset",
            "SLCPriority1", "SLCPriority2", "SLCPriority3", "SLCLookaheadLower", "SetSpeedLimit")
# "or" = never cancel a manual Experimental Mode (previous release baseline); "replace" = 0.9.7 (SLC-derived flag replaces the manual one).
EXPERIMENTAL_POLICY = "or"
# ACTIVATION. "default_on": the saved/registry SpeedLimitController value
# decides (registry default "1"); saved prefs are never rewritten. "adoption": dormant until the panel writes SLC_ACK.
ACTIVATION_POLICY = "default_on"


def _put(store, method, key, value):
  """Blocking write so a read right after the tap sees the new value (the panels rebuild their child rows from it);
  small injected test stores without the keyword keep their plain write."""
  try:
    getattr(store, method)(key, value, block=True)
  except TypeError:
    getattr(store, method)(key, value)


def configure(store, key, value):
  """Only from an explicit panel action: write the user's value and adopt the Speed Limit Controller group."""
  if key not in SLC_KEYS:
    raise ValueError(key)
  _put(store, "put", key, value)
  _put(store, "put_bool", SLC_ACK, True)
  store.invalidate()


def slc_adopted(store) -> bool:
  """True once the user has used the panel. Fail closed (False) on any read problem."""
  if ACTIVATION_POLICY == "default_on":
    return True
  try:
    return bool(store.get_bool(SLC_ACK, False))
  except Exception:
    return False


def _finite(x, default=0.0) -> float:
  try:
    x = float(x)
  except (TypeError, ValueError):
    return default
  return x if math.isfinite(x) else default


V_CRUISE_UNSET_KPH = 255.   # selfdrive/car/cruise.V_CRUISE_UNSET
AP1_CLUSTER_LIFT_FPS = ("TESLA_AP1_MODELS", "TESLA_AP1_MODELX")   # card.py ap1_cluster_lift (AP1 S/X, op long)


def unlifted_cluster_kph(CS) -> float:
  """carState.vCruiseCluster before the AP1 display lift (card.py, VCruiseHelper pcmCruise branch, cruise.py).

  The card lifts vCruiseCluster for the IC (engage floor, plan follow). The planner reads the value
  VCruiseHelper had, rebuilt here from cruiseState, so the display lift never feeds back into the SLC override.
  """
  cruise = getattr(CS, "cruiseState", None)
  if cruise is None or not bool(getattr(cruise, "available", True)):
    return _finite(getattr(CS, "vCruise", 0.0))
  speed = _finite(getattr(cruise, "speed", 0.0))
  if speed == 0.0:
    return V_CRUISE_UNSET_KPH
  if speed == -1.0:
    return -1.0
  cluster = _finite(getattr(cruise, "speedCluster", 0.0))
  return (cluster if cluster != 0.0 else speed) * MS_TO_KPH   # interfaces.py: speedCluster 0 -> speed


def clip_offset_ms(offset_ms: float, is_metric: bool) -> float:
  """Stored offsets outside the 0.9.7 settings range are clipped to it; non-finite reads as 0."""
  conv = KPH_TO_MS if is_metric else MPH_TO_MS
  bound = OFFSET_BOUNDS[bool(is_metric)] * conv
  return float(min(max(_finite(offset_ms), -bound), bound))


def dashboard_limit_ms(value) -> float:
  """carState.bogpilot.dashboardSpeedLimit (m/s, 0 = none). Non-finite, negative or > MAX_LIMIT_MS -> 0 (none)."""
  v = _finite(value, 0.0)
  return v if 0.0 < v <= MAX_LIMIT_MS else 0.0



def slc_supported(CP) -> bool:
  """True when this car should run the SLC consumer and show the settings panel."""
  fp = str(getattr(CP, "carFingerprint", "") or "") if CP is not None else ""
  return fp in SLC_PLATFORMS and bool(getattr(CP, "openpilotLongitudinalControl", False))


def clamp_lookahead_horizon_s(value, default=LOOKAHEAD_HORIZON_DEFAULT) -> int:
  """User horizon in seconds. 0 = off. Non-finite / unreadable -> default. Clamped to 0-15."""
  try:
    v = int(round(float(value)))
  except (TypeError, ValueError):
    return int(default)
  if v < LOOKAHEAD_HORIZON_MIN:
    return LOOKAHEAD_HORIZON_MIN
  if v > LOOKAHEAD_HORIZON_MAX:
    return LOOKAHEAD_HORIZON_MAX
  return v


def adapt_distance_m(v_now, v_end, a_abs=EARLY_DECEL_MS2) -> float:
  """Distance needed to reach v_end from v_now at constant deceleration a_abs. 0 if no slowing is needed."""
  if not all(math.isfinite(x) for x in (v_now, v_end, a_abs)) or v_now <= v_end or a_abs <= 0.0:
    return 0.0
  return (v_now * v_now - v_end * v_end) / (2.0 * a_abs)


def trajectory_speed_ms(v_end, distance, a_abs=EARLY_DECEL_MS2) -> float:
  """Speed on a constant-a trajectory that arrives at v_end after `distance` metres."""
  if not all(math.isfinite(x) for x in (v_end, distance, a_abs)) or a_abs <= 0.0:
    return max(_finite(v_end), 0.0)
  return math.sqrt(max(0.0, v_end * v_end + 2.0 * a_abs * max(distance, 0.0)))


def dashboard_confirmed(dashboard_ms, upcoming_ms, tol=LOOKAHEAD_CONFIRM_MS) -> bool:
  """True when the dashboard already shows the upcoming posted limit (finish the drop)."""
  return dashboard_ms >= 1.0 and upcoming_ms >= 1.0 and abs(dashboard_ms - upcoming_ms) <= tol


def osm_disagrees_with_dashboard(dashboard_ms, osm_current_ms, tol=LOOKAHEAD_DISAGREE_MS) -> bool:
  return dashboard_ms >= 1.0 and osm_current_ms >= 1.0 and abs(dashboard_ms - osm_current_ms) > tol


def upcoming_in_horizon(next_ms, distance_m, v_ego, horizon_s) -> bool:
  """True when OSM reports a usable next limit whose distance is within the time horizon."""
  horizon_s = clamp_lookahead_horizon_s(horizon_s, 0)
  if horizon_s <= 0 or next_ms < LOOKAHEAD_MIN_LIMIT_MS or v_ego <= 0.0 or not math.isfinite(v_ego):
    return False
  if not math.isfinite(distance_m) or distance_m <= 1.0:
    return False
  return distance_m <= horizon_s * v_ego


def lookahead_ceiling_ms(*, dashboard_ms, osm_current_ms, osm_next_ms, distance_m, v_ego,
                         horizon_s, current_set_ms, next_set_ms, a_abs=EARLY_DECEL_MS2):
  """v_cruise ceiling (m/s) for an early slowdown toward a LOWER upcoming OSM limit, or None.

  Dashboard stays the current-limit authority. This never raises, never finishes the drop
  until the dashboard (or map current, when dashboard is missing) confirms the new limit,
  and is inert when the horizon is 0, OSM is stale/missing, or the next limit is below 15 mph.
  """
  horizon_s = clamp_lookahead_horizon_s(horizon_s, 0)
  if horizon_s <= 0 or not math.isfinite(v_ego) or v_ego < 0.0:
    return None
  dashboard_ms = _finite(dashboard_ms, 0.0)
  osm_current_ms = _finite(osm_current_ms, 0.0)
  osm_next_ms = _finite(osm_next_ms, 0.0)
  current_set_ms = _finite(current_set_ms, 0.0)
  next_set_ms = _finite(next_set_ms, 0.0)
  if osm_next_ms < LOOKAHEAD_MIN_LIMIT_MS or current_set_ms < 1.0 or next_set_ms < 1.0:
    return None
  if next_set_ms >= current_set_ms - 1e-6:
    return None
  # Already confirmed: the posted source has the new limit, so no early-slowdown ceiling.
  if dashboard_confirmed(dashboard_ms, osm_next_ms):
    return None
  if dashboard_ms < 1.0 and dashboard_confirmed(osm_current_ms, osm_next_ms):
    return None
  if not math.isfinite(distance_m) or distance_m < 0.0:
    return None
  # At or past the point without confirmation: do not finish the drop.
  if distance_m <= 1.0:
    return None
  adapt_d = adapt_distance_m(v_ego, next_set_ms, a_abs)
  horizon_d = horizon_s * v_ego
  if adapt_d <= 0.0 or distance_m > adapt_d or distance_m > horizon_d:
    return None
  ceil = trajectory_speed_ms(next_set_ms, distance_m, a_abs)
  return min(current_set_ms, max(ceil, next_set_ms))


def offset_for(target_ms: float, t) -> float:
  """speed_limit_controller.SpeedLimitController.offset: band chosen in rounded display units."""
  if t.is_metric:
    band = int(round(float(target_ms) * MS_TO_KPH))
    highs = (29, 49, 59, 79, 99, 119, 140)
  else:
    band = int(round(float(target_ms) * MS_TO_MPH))
    highs = (24, 34, 44, 54, 64, 74, 99)
  for i, high in enumerate(highs, start=1):
    if band <= high:
      return clip_offset_ms(getattr(t, f"speed_limit_offset{i}", 0.0), t.is_metric)
  return 0.0


class PreapEngageSetLimit:
  """Pre-AP with pedal (pcmCruise=False): the 0.9.7 SetSpeedLimit engage set (drive_helpers.py:146-158 and
  frogpilot_variables.py:1143 @568fea947): Speed Limit Controller on (and adopted) and SetSpeedLimit on -> the
  engage set is the posted limit plus the SLC offset. Read once per engage edge by the card.

  limit_ms must already be fresh with known units (PreApCarState.engage_limit_ms). Stricter than 0.9.7 (NAP
  e99f4dc75 captures the current speed at engage and never a higher target): no limit set at a standstill
  (CS.vEgo < 0.1 m/s or CS.standstill; missing or non-finite vEgo fails closed) or with the gas pressed;
  then (and for a stale / unknown limit) the stock engage set applies. 0 = no limit set.
  """

  def __init__(self, CP, store=None, toggles_fn=None, adopted=None):
    self.CP = CP
    self._store, self._toggles_fn, self._adopted_fixed = store, toggles_fn, adopted
    fp = str(getattr(CP, "carFingerprint", "") or "")
    self.active = (fp == "TESLA_PREAP_MODELS" and not bool(getattr(CP, "pcmCruise", True))
                   and slc_supported(CP))
    # Snapshot of (toggles, adopted), refreshed off card's 100 Hz step (card.params_thread calls refresh()).
    # kph() runs on the engage / FULL-capture edge inside that step; reading the toggles there meant the lazy
    # imports below, a ParamStore (and its first all_keys()) and dozens of param reads landed on the engage
    # frame. None (no refresh yet) falls back to reading inline, so tests and any other caller still work.
    self._snapshot = None

  def refresh(self) -> bool:
    """Read the toggles now and keep them for kph(). Call this off the control step (card's params thread, and
    once at card start while offroad, the same warm-up warm_engage_helpers does for the AP1 helpers).

    Returns whether the snapshot was taken. Never raises: a param problem leaves the previous snapshot in place
    and kph() keeps its own fail-closed 0.0 fallback.
    """
    if not self.active:
      return False
    try:
      self._snapshot = self._read_toggles()
      return True
    except Exception:
      return False

  def _read_toggles(self):
    if self._toggles_fn is not None:
      t = self._toggles_fn()
    else:
      from openpilot.common.bogpilot_onroad_toggles import onroad_toggles
      if self._store is None:
        from openpilot.common.bogpilot_store import ParamStore
        self._store = ParamStore()
      self._store.invalidate()
      t = onroad_toggles(self._store, self.CP)
    adopted = bool(self._adopted_fixed) if self._adopted_fixed is not None else slc_adopted(self._store)
    return t, adopted

  def _toggles(self):
    snap = self._snapshot
    return snap if snap is not None else self._read_toggles()

  def kph(self, CS, limit_ms) -> float:
    if not self.active:
      return 0.0
    limit_ms = dashboard_limit_ms(limit_ms)
    # Pre-AP with the pedal keeps CS.standstill as the DI STANDSTILL state, which is False when DI
    # cruise is not engaged, so a vEgo < 0.1 m/s gate is required at a stop. Missing or non-finite
    # vEgo fails closed. CS.standstill is still honored (the stricter of the two wins).
    v_ego = _finite(getattr(CS, "vEgo", None), default=float("nan"))
    stopped = (not math.isfinite(v_ego)) or (v_ego < 0.1) or bool(getattr(CS, "standstill", True))
    if not math.isfinite(limit_ms) or limit_ms <= 0.0 or stopped or bool(getattr(CS, "gasPressed", True)):
      return 0.0
    try:
      t, adopted = self._toggles()
      if not (adopted and getattr(t, "speed_limit_controller", False) and getattr(t, "set_speed_limit", False)):
        return 0.0
      return (limit_ms + offset_for(limit_ms, t)) * MS_TO_KPH
    except Exception:
      return 0.0


class SpeedLimitController:
  """0.9.7 SpeedLimitController minus Mapbox / map data / navigation (no producers in this tree)."""

  def __init__(self, previous_target: float = 0.0, persist=None):
    self.override_slc = False
    self.denied_target = 0.0
    self.map_speed_limit = 0.0
    self.next_speed_limit = 0.0
    self.next_distance = 0.0
    self.segment_distance = 0.0
    self.map_ctx = None     # osm.consumer.MapContext (None = Map Data / Mapbox sources absent)
    self.mapbox = None      # osm.mapbox.MapboxFiller
    self.overridden_speed = 0.0
    self.speed_limit_changed_timer = 0.0
    self.target = 0.0
    self.unconfirmed_speed_limit = 0.0
    self.previous_source = "None"
    self.source = "None"
    self.previous_target = max(_finite(previous_target), 0.0)
    self._persist = persist  # callable(float) -> None, 0.9.7 params.put_float_nonblocking("PreviousSpeedLimit")
    self.t = None

  @property
  def mapbox_limit(self) -> float:
    return float(self.mapbox.mapbox_limit) if self.mapbox is not None else 0.0

  @property
  def offset(self) -> float:
    return offset_for(self.target, self.t)

  def update_map_speed_limit(self, v_ego):
    """Map Data current + next + distance. Look-ahead is a cruise ceiling in Ap1SpeedControl, not a swap."""
    ctx = self.map_ctx
    if ctx is None:
      self.map_speed_limit = 0.0
      self.next_speed_limit = 0.0
      self.next_distance = 0.0
      return
    m = ctx.update(v_ego, getattr(self.t, "map_speed_lookahead_higher", 0), getattr(self.t, "map_speed_lookahead_lower", 0))
    # Prefer the unswapped current way limit; fall back to .limit if a stub only exposes that.
    current = getattr(m, "current", None)
    self.map_speed_limit = float(current if current is not None else m.limit)
    self.next_speed_limit = float(m.next_limit)
    dist = _finite(getattr(m, "distance_to_next", 0.0), 0.0)
    self.next_distance = dist if dist >= 0.0 else 0.0

  @property
  def experimental_mode(self) -> bool:
    return self.target == 0 and bool(self.t.slc_fallback_experimental_mode)

  def handle_limit_change(self, desired_source, desired_target, accel_pressed, decel_pressed, long_active,
                          accepted_flag=False, dt=DT_MDL):
    self.speed_limit_changed_timer += dt
    speed_limit_accepted = (accel_pressed and long_active) or accepted_flag
    speed_limit_denied = decel_pressed or (self.speed_limit_changed_timer >= CONFIRM_TIMEOUT_S)

    if speed_limit_accepted:
      self.overridden_speed = 0.0
      self.source = desired_source
      self.target = desired_target
    elif speed_limit_denied:
      self.denied_target = desired_target
      self.previous_source = desired_source
      self.previous_target = desired_target
    elif desired_target < self.target and not self.t.speed_limit_confirmation_lower:
      self.source = desired_source
      self.target = desired_target
    elif desired_target > self.target and not self.t.speed_limit_confirmation_higher:
      self.source = desired_source
      self.target = desired_target
    else:
      self.source = "None"
      self.unconfirmed_speed_limit = desired_target

    if self.target != self.previous_target and self.target > 0 and not speed_limit_denied:
      self.denied_target = 0.0
      self.previous_source = self.source
      self.previous_target = self.target
      if self._persist is not None:
        try:
          self._persist(float(self.target))
        except Exception:
          pass

  def update_limits(self, dashboard_speed_limit, v_cruise, enabled, accel_pressed, decel_pressed, long_active,
                    navigation_speed_limit=0.0, accepted_flag=False, dt=DT_MDL, v_ego=0.0, steering_deg=0.0):
    t = self.t
    self.update_map_speed_limit(v_ego)
    horizon_s = clamp_lookahead_horizon_s(getattr(t, "map_speed_lookahead_lower", LOOKAHEAD_HORIZON_DEFAULT))
    map_limit = self.map_speed_limit
    # If OSM's current disagrees with Dashboard by more than 5 mph and there is no upcoming
    # point in the horizon, Dashboard wins: Map Data is ignored for the posted current limit.
    if osm_disagrees_with_dashboard(dashboard_speed_limit, map_limit) and not upcoming_in_horizon(
        self.next_speed_limit, self.next_distance, v_ego, horizon_s):
      map_limit = 0.0
    limits = {"Dashboard": dashboard_speed_limit, "Map Data": map_limit, "Navigation": navigation_speed_limit}
    filtered = {s: v for s, v in limits.items() if v >= 1}

    # Dashboard is the current-limit authority whenever it is present, including Highest/Lowest.
    if dashboard_speed_limit >= 1:
      desired_source, desired_target = "Dashboard", dashboard_speed_limit
    elif t.speed_limit_priority_highest:
      desired_source = max(filtered, key=filtered.get, default="None")
      desired_target = filtered.get(desired_source, 0)
    elif t.speed_limit_priority_lowest:
      desired_source = min(filtered, key=filtered.get, default="None")
      desired_target = filtered.get(desired_source, 0)
    elif filtered:
      for priority in (t.speed_limit_priority1, t.speed_limit_priority2, t.speed_limit_priority3):
        if priority in filtered:
          desired_source = priority
          desired_target = filtered[desired_source]
          break
      else:
        desired_source, desired_target = "None", 0
    else:
      desired_source, desired_target = "None", 0

    if desired_target == 0 or self.target == 0:
      mb = self.mapbox
      if mb is not None and mb.budget_ok and getattr(t, "slc_mapbox_filler", False):
        ctx = self.map_ctx
        now_dt = ctx.now_dt()
        mb.update(ctx.gps_position, now_dt, _time_validated(now_dt), v_ego, steering_deg)

        if self.mapbox_limit >= 1:
          desired_source = "Mapbox"
          desired_target = self.mapbox_limit

      if desired_target == 0 or self.target == 0:
        if self.denied_target != self.previous_target > 0 and t.slc_fallback_previous_speed_limit:
          desired_source = self.previous_source
          desired_target = self.previous_target
          self.target = desired_target
        elif enabled and t.slc_fallback_set_speed:
          desired_source = "None"
          desired_target = v_cruise
    else:
      if self.mapbox is not None:
        self.mapbox.reset()

    if abs(desired_target - self.previous_target) >= 1:
      self.handle_limit_change(desired_source, desired_target, accel_pressed, decel_pressed, long_active,
                               accepted_flag, dt)
    elif desired_source != self.source and abs(desired_target - self.target) < 1:
      self.source = desired_source
    else:
      self.speed_limit_changed_timer = 0.0
      self.unconfirmed_speed_limit = 0.0

  def update_override(self, v_cruise, v_cruise_diff, v_ego, v_ego_diff, gas_pressed, enabled, driver_set_ms=None):
    """Gas override of the SLC target (0.9.7 speed_limit_controller.py:344-356 @568fea947).

    driver_set_ms: None = Max Set Speed returns to v_cruise (0.9.7: v_cruise is the driver's cruise set).
    A number = the set the driver chose (> 0), or 0 for none: then Max Set Speed returns to the current
    limit + offset. AP1 openpilot long passes it because there v_cruise is the car-layer engage latch, which after a
    pull is the engage-time limit + offset and is never refreshed on a limit change.
    """
    t = self.t
    self.override_slc = self.overridden_speed > self.target + self.offset > 0
    self.override_slc |= bool(gas_pressed) and v_ego > self.target + self.offset > 0
    self.override_slc &= bool(enabled)

    if self.override_slc:
      if t.speed_limit_controller_override_manual:
        if gas_pressed:
          self.overridden_speed = max(v_ego + v_ego_diff, self.overridden_speed)
        lo, hi = self.target + self.offset, v_cruise + v_cruise_diff
        self.overridden_speed = float(min(max(self.overridden_speed, lo), hi)) if lo <= hi else float(hi)
      elif t.speed_limit_controller_override_set_speed:
        if driver_set_ms is None:
          self.overridden_speed = v_cruise + v_cruise_diff
        elif math.isfinite(driver_set_ms) and driver_set_ms > 0.0:
          self.overridden_speed = float(driver_set_ms)
        else:
          self.overridden_speed = max(float(self.target + self.offset), 0.0)
      self.source = "None"
    else:
      self.overridden_speed = 0.0


def _time_validated(now_dt) -> bool:
  return now_dt.year >= 2025


def _service_ok(sm, name) -> bool:
  try:
    return bool(sm.alive[name]) and bool(sm.valid[name])
  except (AttributeError, KeyError, TypeError):
    return False


def _tip_round_pref(fingerprint) -> bool:
  """AP1TipRoundSet (default on, saved value kept); never raises."""
  try:
    from opendbc.car.tesla_ap1.toggles import ap1_tip_round_set
    return bool(ap1_tip_round_set(str(fingerprint or "")))
  except Exception:
    return False


class LongEngageGate:
  """Which engage pull hands the set speed to SLC: the one that actually engages longitudinal.

  Single-pull mode: the one pull engages lat + long, so cruiseState.enabled and carControl.longActive rise together (0.9.7).
  Double-pull mode (pre-AP style: first pull lateral only, second pull lat + long): the first pull raises cruiseState.enabled
  but longActive stays False, so nothing is handed over and the set speed is untouched; the second pull raises longActive and
  is then the engage pull. Ap1RaiseHoldoff classifies the stalk action on the cruise edge (cruise_on rising + stalk within
  0.8 s, BogPilot 0.9.7 frogpilot_vcruise.py:119), remembers it (newer presses win) and applies it when this gate reports
  engaged, so a gas-held tap whose longActive rises seconds later still latches vEgo and never falls back to SLC.
  A pull while long is engaged keeps the 0.9.7 rule (clears tip + sticky, SLC re-latches). A longActive dip shorter than DROP_S
  (brief overrideLongitudinal alerts) does not drop a tip; a longer one is a lateral-only state again.
  longActive lags cruiseState.enabled by a tick or two on a normal engage: those ticks pass the set through (no cap, no raise);
  the hand-over starts the tick longActive is seen, like every other planner input on engage.
  """
  DROP_S = 1.0

  def __init__(self):
    self.long_on = False
    self._drop_s = 0.0
    self.was_on = False   # longitudinal was ours in this cruise session and openpilot engagement

  def update(self, cruise_on: bool, long_active: bool, dt: float):
    """-> (engaged, lat_only)."""
    if not cruise_on:
      self.long_on, self._drop_s, self.was_on = False, 0.0, False
      return False, False
    if long_active:
      self.long_on, self._drop_s, self.was_on = True, 0.0, True
    elif self.long_on:
      self._drop_s += dt
      if self._drop_s >= self.DROP_S:
        self.long_on = False
    return self.long_on, not self.long_on


class Ap1SpeedControl:
  """Planner bridge: SLC + AP1 software stalk set authority (bogpilot_vcruise.py:70-171)."""

  def __init__(self, CP, store=None, toggles_fn=None, clock=time.monotonic, adopted=None, map_context=None, tip_round=None):
    """adopted: None = read SLC_ACK from the store (fail closed when there is no store); bool = fixed (tests/replay)."""
    self.CP = CP
    self._adopted_fixed = adopted
    self.ap1 = slc_supported(CP)  # S, X, and Pre-AP with openpilot longitudinal
    # card lifts carState.vCruiseCluster for display on these; the planner uses the unlifted value.
    self._cluster_lifted = bool(str(getattr(CP, "carFingerprint", "") or "") in AP1_CLUSTER_LIFT_FPS and
                                getattr(CP, "openpilotLongitudinalControl", False) and getattr(CP, "pcmCruise", True))
    self._store = store
    self._toggles_fn = toggles_fn
    self._clock = clock
    self._toggles = None
    self._toggles_t = -1e9
    self._adopted = False
    self.experimental_policy = EXPERIMENTAL_POLICY
    if self.ap1 and store is None and toggles_fn is None:
      try:
        from openpilot.common.bogpilot_store import ParamStore
        self._store = store = ParamStore()
      except Exception:
        store = None
    prev = 0.0
    if self.ap1 and store is not None:
      try:
        prev = float(store.get_float("PreviousSpeedLimit", 0.0) or 0.0)
      except Exception:
        prev = 0.0
    self.slc = SpeedLimitController(prev, persist=self._persist_previous if store is not None else None)
    self._tip_round_fixed = tip_round    # None = read AP1TipRoundSet with the toggles; bool = fixed (tests/replay)
    self._tip_round = True if tip_round is None else bool(tip_round)
    self._map_context = map_context
    self._map_tried = map_context is not None or toggles_fn is not None   # injected toggles = hermetic (no /dev/shm reads)
    # AP1 (S/X, not Pre-AP) with openpilot long: a stalk tap from 0 sets 15 mph and a sticky 0 is no set
    # (op_long_set_speed.TAP_FROM_ZERO_KPH). Pre-AP keeps the 0.9.7 behavior.
    self._ap1_op_long = bool(self.ap1 and getattr(CP, "carFingerprint", "") != "TESLA_PREAP_MODELS"
                             and getattr(CP, "openpilotLongitudinalControl", False))
    self._tap_from_zero = EngageTapFromZero(store=self._store, clock=self._clock)
    self.holdoff = self._new_holdoff()
    self.guard = Ap1SlcLimitGuard()
    self.engage = LongEngageGate()
    self._tip_override_active = False
    self._sd_enabled_prev = False
    self._gas_since_long = False   # gas seen since longActive was last True (the gas override and its release ticks)
    self._reset_outputs()

  def _new_holdoff(self):
    hold = Ap1RaiseHoldoff()
    if self._ap1_op_long:
      hold.tap_from_zero_ms = self._tap_from_zero.ms()
    return hold

  # ---- toggles ----
  def _persist_previous(self, value):
    self._store.put("PreviousSpeedLimit", str(float(value)))

  def toggles(self):
    now = self._clock()
    if self._toggles is None or now - self._toggles_t >= TOGGLE_CACHE_S:
      try:
        if self._toggles_fn is not None:
          self._toggles = self._toggles_fn()
        else:
          from openpilot.common.bogpilot_onroad_toggles import onroad_toggles
          if self._store is None:
            from openpilot.common.bogpilot_store import ParamStore
            self._store = ParamStore()
          self._store.invalidate()
          self._toggles = onroad_toggles(self._store, self.CP)
        self._tip_round = (_tip_round_pref(getattr(self.CP, "carFingerprint", "")) if self._tip_round_fixed is None
                           else bool(self._tip_round_fixed))
        if self._adopted_fixed is not None:
          self._adopted = bool(self._adopted_fixed)
        else:
          self._adopted = slc_adopted(self._store)
      except Exception:
        self._toggles = None
        self._adopted = False
      self._toggles_t = now
      if self._toggles is not None and self._adopted and getattr(self._toggles, "speed_limit_controller", False):
        self._wire_map_sources(self._toggles)
    return self._toggles

  def _wire_map_sources(self, t):
    """Map Data + Mapbox filler inputs (osm.consumer). Any failure leaves the sources absent (= 0), never raises."""
    try:
      if not self._map_tried:
        self._map_tried = True
        from openpilot.system.bogpilot.osm.consumer import MapContext
        self._map_context = MapContext(store=self._store)
      ctx = self._map_context
      if ctx is None:
        return
      self.slc.map_ctx = ctx
      want_mapbox = bool(getattr(t, "slc_mapbox_filler", False))
      token = None
      if want_mapbox and self._store is not None:
        token = self._store.get("MapboxSecretKey", "")
      if want_mapbox and token:
        self.slc.mapbox = ctx.mapbox
        ctx.mapbox.set_token(token)
        ctx.want_own_gps = True
      else:
        ctx.mapbox.set_token(None)
        ctx.want_own_gps = False
        ctx.mapbox.reset()
        self.slc.mapbox = None
    except Exception:
      self.slc.map_ctx = None
      self.slc.mapbox = None

  # ---- outputs ----
  def _reset_outputs(self):
    self.active = False
    self.slc_target = 0.0
    self.slc_offset = 0.0
    self.v_cruise_out = 0.0

  def _inactive(self, v_cruise):
    self.engage = LongEngageGate()
    self.holdoff = self._new_holdoff()
    self.guard = Ap1SlcLimitGuard()
    self._tip_override_active = False
    self.slc.overridden_speed = 0.0
    self.slc.override_slc = False
    self.slc.speed_limit_changed_timer = 0.0
    self.slc.unconfirmed_speed_limit = 0.0
    self._reset_outputs()
    self.v_cruise_out = v_cruise
    return v_cruise

  def update(self, sm, v_cruise: float, v_ego: float, dt: float = DT_MDL) -> float:
    """v_cruise (m/s, already min(carState.vCruise, V_CRUISE_MAX)) -> SLC / stalk set (m/s)."""
    t = self.toggles() if self.ap1 else None
    if t is None or not self._adopted or not getattr(t, "speed_limit_controller", False) or not getattr(t, "slc_platform", self.ap1):
      return self._inactive(v_cruise)
    if not _service_ok(sm, "carState") or not math.isfinite(v_cruise) or not math.isfinite(v_ego):
      return self._inactive(v_cruise)

    CS = sm["carState"]
    bp = CS.bogpilot
    self.slc.t = t
    enabled = bool(sm["selfdriveState"].enabled) if _service_ok(sm, "selfdriveState") else False
    long_active = bool(sm["carControl"].longActive) if _service_ok(sm, "carControl") else False
    up, dn, rwd = bool(bp.accelPressed), bool(bp.decelPressed), bool(bp.resumePressed)
    gas = bool(CS.gasPressed)

    cluster_kph = unlifted_cluster_kph(CS) if self._cluster_lifted else _finite(CS.vCruiseCluster)
    v_cruise_cluster = max(cluster_kph * KPH_TO_MS, v_cruise)
    v_cruise_diff = v_cruise_cluster - v_cruise
    v_ego_cluster = max(_finite(CS.vEgoCluster), v_ego)
    v_ego_diff = v_ego_cluster - v_ego

    try:
      steering_deg = float(CS.steeringAngleDeg) - float(sm["vehicleParameters"].angleOffsetDeg)
    except Exception:
      steering_deg = 0.0
    self.slc.update_limits(dashboard_limit_ms(bp.dashboardSpeedLimit), v_cruise, enabled, up, dn, long_active, dt=dt,
                           v_ego=max(float(v_ego), 0.0), steering_deg=steering_deg if math.isfinite(steering_deg) else 0.0)
    # AP1 openpilot long: Max Set Speed returns to a set the driver chose (tap latch / tip), else to the current
    # limit + offset, never to the engage-time limit + offset still held in carState.vCruise after a pull.
    driver_set = float(self.holdoff.driver_set_ms) if self._ap1_op_long else None
    self.slc.update_override(v_cruise, v_cruise_diff, v_ego, v_ego_diff, gas, enabled, driver_set_ms=driver_set)
    # AP1: ignore bogus <15 mph limits, confirm sudden big drops (e.g. a 5 mph blip).
    self.slc_target, self.slc_offset = self.guard.update(self.slc.target, self.slc.offset, dt)
    self.active = True

    cruise_on = bool(CS.cruiseState.enabled)
    if self._sd_enabled_prev and not enabled:
      self.holdoff.forget_engage()   # openpilot disengaged: never carry the engage stalk action across it
      self.engage.was_on = False     # a re-engage with cruise still on is an engage again, not a return from override
      self._gas_since_long = False
    self._sd_enabled_prev = enabled
    engaged, lat_only = self.engage.update(cruise_on, long_active, dt)
    if long_active or not cruise_on:
      self._gas_since_long = False
    elif gas:
      self._gas_since_long = True
    if lat_only:
      # cruise on but openpilot longitudinal never engaged (double-pull mode, first pull): lateral only. No hand-over, no cap, no
      # raise, no tip: the set speed is not ours. Limits above keep updating (display, Previous Limit bookkeeping).
      # Stalk levels still feed the holdoff (long-disengaged update). The cruise edge classifies the engage stalk action and a
      # newer press updates it, so the long edge that follows (second pull, or gas release) applies it.
      self.holdoff.update(False, dn, up, rwd, float(self.slc_target), float(v_ego), dt=dt, cruise_on=cruise_on)
      self._tip_override_active = False
      self.slc.overridden_speed = 0.0
      self.slc.override_slc = False
      if self._ap1_op_long and enabled and self.engage.was_on and self._gas_since_long:
        # Gas override after longitudinal was ours in this engagement: the gas held past DROP_S, the release tick and
        # any further ticks until longActive is seen again. carState.vCruise is the car-layer engage latch (after a
        # pull, the engage-time limit + offset), not a driver set. Publish the set the Max Set Speed override returns
        # to: the driver's own set (tip / tap latch), else the current limit + offset.
        held_ms = float(self.holdoff.driver_set_ms)
        if not (math.isfinite(held_ms) and held_ms > 0.0):
          held_ms = float(self.slc_target) + float(self.slc_offset) - v_ego_diff
        if math.isfinite(held_ms) and held_ms > 0.0:
          self.v_cruise_out = float(min(held_ms, V_CRUISE_MAX_MS))
          return self.v_cruise_out
      # Gas-held engage (longActive 0 while the gas is down, or the few ticks before longActive is seen): the
      # pass-through is carState.vCruise. Floor it at vEgo, the same floor as the long branch below, so the
      # plan (and the IC lift from it) never shows a ~vEgo/2 DI_cruiseSet seed. Raises only, never lowers.
      if cruise_on and math.isfinite(v_ego) and v_ego > 0.0 and v_cruise < v_ego:
        v_cruise = float(v_ego)
      self.v_cruise_out = float(v_cruise)
      return self.v_cruise_out

    # Longitudinal is ours. Under the OP overlay DI_cruiseSet is often ~vEgo/2,
    # or Tesla's 8-10 mph ACC minimum (both below CRUISING_SPEED, so min()
    # would keep that seed). Start the software set at vEgo; sticky / tip /
    # SLC raise still apply below. Does not change stalk reads or carState.vCruise.
    if engaged and math.isfinite(v_ego) and v_ego > 0.0 and v_cruise < v_ego:
      v_cruise = float(v_ego)

    csc_target = v_cruise  # current planner applies AP1 CSC afterwards (min only)
    posted = max(self.slc.overridden_speed, self.slc_target + self.slc_offset)
    # Look-ahead is a v_cruise ceiling only. It must not go through handle_limit_change
    # (posted target stays Dashboard). Skip it while a gas / max-set override is in charge.
    if not self.slc.override_slc:
      next_lim = float(self.slc.next_speed_limit)
      next_posted = (next_lim + offset_for(next_lim, t)) if next_lim >= 1.0 else 0.0
      ceil = lookahead_ceiling_ms(
        dashboard_ms=dashboard_limit_ms(bp.dashboardSpeedLimit),
        osm_current_ms=float(self.slc.map_speed_limit),
        osm_next_ms=next_lim,
        distance_m=float(self.slc.next_distance),
        v_ego=max(float(v_ego), 0.0),
        horizon_s=getattr(t, "map_speed_lookahead_lower", LOOKAHEAD_HORIZON_DEFAULT),
        current_set_ms=posted,
        next_set_ms=next_posted,
      )
      if ceil is not None:
        posted = min(posted, ceil)
    slc_desired = posted - v_ego_diff
    targets = [csc_target, v_cruise, slc_desired]
    out = min(x if x >= CRUISING_SPEED else v_cruise for x in targets)

    hold = self.holdoff
    hold.round_whole = bool(self._tip_round)
    hold.is_metric = bool(getattr(t, "is_metric", False))
    if self._ap1_op_long:
      hold.tap_from_zero_ms = self._tap_from_zero.ms()
    if hold.sticky_vego and float(hold.latched_vego_ms) > 0.0:
      set_hint = float(hold.latched_vego_ms)
    elif float(hold.tip_ms) > 0.0:
      set_hint = float(hold.tip_ms)
    else:
      set_hint = float(out)
      if slc_desired >= CRUISING_SPEED:
        set_hint = max(set_hint, float(slc_desired))
    spd = int(_finite(bp.spdCtrlLvr))
    allow_raise, sticky = hold.update(engaged and cruise_on, dn, up, rwd, float(self.slc_target), float(v_ego),
                                      current_set_ms=set_hint, dt=dt, tip_full=spd in (4, 8), cruise_on=cruise_on)
    tip_ms, tip_dir = float(hold.tip_ms), int(hold.tip_dir)
    if tip_ms > 0:
      # Max Set Speed gas override returns to the software tip set.
      self.slc.overridden_speed = tip_ms if tip_dir < 0 else max(float(self.slc.overridden_speed), tip_ms)
      self._tip_override_active = True
    elif self._tip_override_active:
      self.slc.overridden_speed = 0.0
      self._tip_override_active = False
      slc_desired = max(0.0, float(self.slc_target) + float(self.slc_offset)) - v_ego_diff
    out = ap1_cruise_ms(out, sticky, tip_ms, allow_raise, slc_desired, self.slc_target, CRUISING_SPEED, False, csc_target,
                        sticky_zero_is_none=self._ap1_op_long)

    if not math.isfinite(out):
      return self._inactive(v_cruise)
    self.v_cruise_out = float(min(max(out, 0.0), V_CRUISE_MAX_MS))
    return self.v_cruise_out

  @property
  def experimental_mode(self) -> bool:
    return bool(self.active and self.slc.experimental_mode)

  @property
  def experimental_override(self):
    """None unless SLC is controlling AND Fallback Speed = Experimental Mode; then the SLC-derived flag
    (True only while no limit is known). 0.9.7 bogpilot_variables slc_fallback_experimental_mode."""
    if not self.active or not bool(getattr(self.slc.t, "slc_fallback_experimental_mode", False)):
      return None
    return bool(self.slc.experimental_mode)

  def resolve_experimental(self, manual: bool) -> bool:
    """The planner's experimental flag. No override (SLC off / not adopted / fallback != Experimental) -> `manual`
    unchanged. Policy "replace" (0.9.7): the SLC-derived flag replaces the manual one. Policy "or": manual stays on."""
    manual = bool(manual)
    ov = self.experimental_override
    if ov is None:
      return manual
    if self.experimental_policy == "or":
      return manual or ov
    return ov

  def fill(self, plan) -> None:
    """bogpilotPlan SLC fields (bogpilot_planner.py:172-180). Inactive -> zeros, source ""."""
    a = self.active
    plan.slcSpeedLimit = float(self.slc_target) if a else 0.0
    plan.slcSpeedLimitOffset = float(self.slc_offset) if a else 0.0
    plan.slcOverriddenSpeed = float(self.slc.overridden_speed) if a else 0.0
    plan.slcSpeedLimitSource = str(self.slc.source) if a else INACTIVE_SOURCE
    plan.speedLimitChanged = bool(a and self.slc.speed_limit_changed_timer > DT_MDL)
    plan.unconfirmedSlcSpeedLimit = float(self.slc.unconfirmed_speed_limit) if a else 0.0
    plan.slcMapSpeedLimit = float(self.slc.map_speed_limit) if a else 0.0
    plan.slcMapboxSpeedLimit = float(self.slc.mapbox_limit) if a else 0.0
    plan.slcNextSpeedLimit = float(self.slc.next_speed_limit) if a else 0.0
    # experimentalMode is NOT written here: the planner publishes its effective flag (resolve_experimental) so the
    # reported value is exactly what the planner used (0.9.7 bogpilot_planner.py:149 `cem or slc.experimental_mode`).


class EngageLimitOffset:
  """Card -> AP1 CarState: the speed-limit offset (m/s) for the openpilot-long engage set (op_long_set_speed.py).

  Same Offset1..7 toggles and bands as the planner's SLC (offset_for). Read from params (1 s cache), never
  from bogpilotPlan, so the engage set does not feed back from the plan. Any failure gives 0 (no offset).
  """

  def __init__(self, CP, toggles_fn=None, clock=time.monotonic):
    self.CP = CP
    self._toggles_fn = toggles_fn
    self._clock = clock
    self._store = None
    self._t = None
    self._toggles_t = -1e9

  def _toggles(self):
    now = self._clock()
    if now - self._toggles_t >= TOGGLE_CACHE_S:
      self._toggles_t = now
      try:
        if self._toggles_fn is not None:
          self._t = self._toggles_fn()
        else:
          from openpilot.common.bogpilot_onroad_toggles import onroad_toggles
          if self._store is None:
            from openpilot.common.bogpilot_store import ParamStore
            self._store = ParamStore()
          self._store.invalidate()
          self._t = onroad_toggles(self._store, self.CP)
      except Exception:
        self._t = None
    return self._t

  def __call__(self, limit_ms: float) -> float:
    t = self._toggles()
    if t is None or not math.isfinite(limit_ms) or limit_ms <= 0.0:
      return 0.0
    try:
      return float(offset_for(limit_ms, t))
    except Exception:
      return 0.0


class EngageTapFromZero:
  """Card / planner: AP1TapFromZeroMph (whole mph 5-30) as kph, live with a 1 s cache.

  Missing, non-integer, or out-of-range values fall back to 15 mph. Any read failure does the same.
  """

  def __init__(self, store=None, clock=time.monotonic):
    self._store = store
    self._clock = clock
    self._t = -1e9
    from opendbc.car.tesla_ap1.op_long_set_speed import TAP_FROM_ZERO_KPH
    self._kph = float(TAP_FROM_ZERO_KPH)

  def __call__(self) -> float:
    now = self._clock()
    if now - self._t >= TOGGLE_CACHE_S:
      self._t = now
      try:
        if self._store is None:
          from openpilot.common.bogpilot_store import ParamStore
          self._store = ParamStore()
        self._store.invalidate("AP1TapFromZeroMph")
        from opendbc.car.tesla_ap1.op_long_set_speed import tap_from_zero_kph_from_param
        self._kph = float(tap_from_zero_kph_from_param(self._store.raw("AP1TapFromZeroMph")))
      except Exception:
        from opendbc.car.tesla_ap1.op_long_set_speed import TAP_FROM_ZERO_KPH
        self._kph = float(TAP_FROM_ZERO_KPH)
    return self._kph

  def ms(self) -> float:
    """Same setting as m/s. __call__ returns kph for the card; the planner holdoff field is m/s."""
    return float(self()) * KPH_TO_MS


def warm_engage_helpers(*helpers) -> None:
  """Card start: run the engage-set helpers once while offroad, so their lazy imports, the ParamStore
  and the first param reads happen here instead of on the first engage frame inside card's 100 Hz loop.

  Takes any number of helpers (AP1: the offset and tap-from-zero helpers; Pre-AP: PreapEngageSetLimit).

  Without this the first engage after boot stalled card for ~145 ms (seen in saved logs of two earlier builds), the next
  carOutput was invalid and selfdrived raised commIssue; the second press then engaged. Failure-tolerant:
  any error is swallowed so a param read problem can never block card start (the helpers already fall
  back to 0 offset / 15 mph on their own).
  """
  for fn in helpers:
    if fn is None:
      continue
    try:
      # A helper that keeps a snapshot (PreapEngageSetLimit) is warmed by filling it, so the engage frame
      # reads memory; the others just need their first call done here.
      refresh = getattr(fn, "refresh", None)
      warm = getattr(fn, "_toggles", None)
      if callable(refresh):
        refresh()
      elif callable(warm):
        warm()
      else:
        fn()
    except Exception:
      pass


class EngageClusterFloor:
  """Hold the Tesla IC set digit at least at vEgo from the moment openpilot enables
  until bogpilotPlan.vCruise has reached vEgo (sticky latch or SLC raise).

  pcmCruise copies DI_cruiseSet into vCruiseCluster. Under the OP overlay that
  seed is often ~vEgo/2, or Tesla's 8-10 mph ACC minimum, so the IC would flash
  8-10 mph and the planner would aim there for the ticks before SLC lifts.
  0.9.7 started the set at vEgo and then raised to the SLC target. The floor is
  released once the plan set is at or above vEgo so a later down tip can show
  (cluster_display_kph never lowers). Does not change carState.vCruise.
  """

  CATCHUP_MS = 0.25  # ~0.5 mph; plan has reached current speed

  def __init__(self):
    self.active = False
    self.enabled_prev = False

  def update(self, enabled: bool, plan_fresh: bool, plan_v_cruise_ms: float, v_ego_ms: float,
             floor_ms: float = 0.0, tip: bool = False) -> bool:
    """floor_ms: AP1 openpilot long, the engage set (cruiseState.speed); the floor holds until the plan reaches
    max(vEgo, floor_ms). tip: a stalk tip releases the floor at once so the tipped set shows."""
    rising = bool(enabled) and not self.enabled_prev
    self.enabled_prev = bool(enabled)
    if not enabled:
      self.active = False
      return False
    if rising:
      # Arm only on the enable edge. Re-arming on every enabled tick put the floor back after the
      # release, so a later down tip below vEgo showed vEgo on the IC instead of the tipped set.
      self.active = True
    target = max(_finite(v_ego_ms, 0.0), _finite(floor_ms, 0.0))
    if self.active and (tip or (plan_fresh and target > 0.0 and
                                _finite(plan_v_cruise_ms, -1.0) >= target - self.CATCHUP_MS)):
      self.active = False
    return self.active


def cluster_set_kph(v_cruise_cluster_kph: float, plan, plan_fresh: bool,
                    v_ego_ms: float | None = None, engage_floor: bool = False,
                    plan_v_ms: float | None = None, follow_plan: bool = False) -> float:
  """card.py consumer for slc_raise.cluster_display_kph (0.9.7 controlsd.py:536-543).

  0.9.7 lifted whenever frogpilotPlan had been received and plan.vCruise was a
  usable set. It did not wait for an SLC source name. Waiting for a non-empty
  source (0.11.2) left the IC on the raw DI_cruiseSet seed until a limit
  appeared. An empty source is "active, no name yet", not "do not lift".

  engage_floor: the displayed set is at least vEgo (see EngageClusterFloor).
  plan_v_ms: the plan set to display instead of plan.vCruise, when given.
  follow_plan: AP1 openpilot long. The cluster base is then openpilot's engage set (op_long_set_speed), not the
  low DI seed, so lifting with max(base, plan) would hide every down tip or SLC drop below the engage set.
  Outside the engage floor, a fresh plan set is shown as is, including the curve-speed (CSC) cap: plan.vCruise is
  published after that cap, so the IC digit follows the curve target down and back up, as the DI-based base did in
  0.9.7 (DI_cruiseSet followed the CSC-reduced DAS_setSpeed). No hold or smoothing.
  """
  from opendbc.car.tesla_ap1.slc_raise import cluster_display_kph
  cluster = float(v_cruise_cluster_kph)
  if engage_floor and v_ego_ms is not None and math.isfinite(v_ego_ms) and v_ego_ms > 0.0:
    cluster = max(cluster, float(v_ego_ms) * MS_TO_KPH)
  if not plan_fresh or plan is None:
    return cluster
  v = _finite(plan.vCruise if plan_v_ms is None else plan_v_ms, -1.0)
  if not 0.0 < v <= V_CRUISE_MAX_MS + 1e-3:
    return cluster
  if follow_plan and not engage_floor:
    return float(v) * MS_TO_KPH
  return float(cluster_display_kph(cluster, v))
