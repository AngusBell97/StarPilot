"""Source-independent CEM/Conditional Chill mode selection.

Only a validated adapter may construct ``SceneEvidence``. This pure policy
never reads Params, IPC, or car hardware; the runtime adapter uses it after
qualifying fresh scene and vehicle authority.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import StrEnum
import math

from openpilot.starpilot.conditional_mode.open_road import OpenRoad


MPH_TO_MPS = 0.44704
SCENE_MAX_AGE_S = 0.25  # Provisional adapter freshness budget; not a qualified detector threshold.
CEM_HOLD_S = 0.5
CEM_EXIT_BUFFER_S = 0.25
CEM_SLOW_LEAD_HOLD_S = 1.5
CEM_OPEN_ROAD_HANDOFF_S = 0.75
CEM_STOP_RELEASE_SUPPRESS_S = 2.0
CURVE_CLEAR_S = 2.0
SPEED_EXIT_MARGIN_MPS = 1.0 * MPH_TO_MPS
CCM_SPEED_CONFIRM_S = 0.35
CCM_LEAD_CONFIRM_S = 1.0
CCM_LAUNCH_CONFIRM_S = 0.0
CCM_MIN_DWELL_S = 1.2
CCM_EXIT_BUFFER_S = 0.35
# Climb mode: Experimental by default; Chill only to bring the car up to the set speed on a clear road.
CLIMB_CONFIRM_S = 1.0           # every start condition must hold this long
CLIMB_LEAD_CLEAR_S = 1.5        # no lead car seen for this long
CLIMB_DONE_MPS = 0.3 * MPH_TO_MPS
CLIMB_REENTRY_S = 3.0           # Experimental for at least this long after a climb ends
CLIMB_HELD_S = 3.0              # this long at the target counts as holding speed
CLIMB_HELD_BAND_MPS = 0.5 * MPH_TO_MPS
CLIMB_TARGET_RISE_MPS = 1.0 * MPH_TO_MPS
CLIMB_CALM_ACCEL = -0.05        # model asks at least this much to start a climb
CLIMB_CALM_3S_MPS = 0.3         # its 3 s plan may sit at most this far below current speed
CLIMB_CALM_8S_MPS = 0.5         # and its 8 s plan at most this far
CLIMB_SLOWING_ACCEL = -0.3      # a climb ends when the model asks for this or less
CLIMB_SLOWING_8S_MPS = 3.0 * MPH_TO_MPS
CLIMB_SIGNAL_VETO_MPS = 35.0 * MPH_TO_MPS  # below this an indicator means a turn: Experimental
CLIMB_RECOVER_SPEED_MPS = 15.0 * MPH_TO_MPS
# Lead pulling away (optional): only a lead well beyond the following distance that keeps moving away.
CLIMB_LEAD_MIN_GAP_S = 3.0      # start only with the lead at least this far ahead in time
CLIMB_LEAD_END_GAP_S = 2.5      # hand back to Experimental once the gap closes to this
CLIMB_LEAD_AWAY_MPS = 1.0       # the lead must be this much faster than us...
CLIMB_LEAD_AWAY_S = 2.0         # ...for this long before a climb starts
CLIMB_LEAD_STOP_AWAY_MPS = 0.3  # the climb ends once the lead is no longer pulling away
CLIMB_LEAD_BRAKE = -0.5         # or the lead brakes


class ModeChoice(StrEnum):
  STOCK = 'stock'
  CEM = 'conditional_experimental'
  CCM = 'conditional_chill'


class ManualIntent(StrEnum):
  NONE = 'none'
  FORCE_EXPERIMENTAL = 'force_experimental'
  FORCE_CHILL = 'force_chill'


class Reason(StrEnum):
  STOCK = 'stock'
  UNAVAILABLE = 'unavailable'
  MANUAL_EXPERIMENTAL = 'manual_experimental'
  MANUAL_CHILL = 'manual_chill'
  CEM_SPEED = 'cem_speed'
  CEM_OPEN_ROAD = 'cem_open_road'
  CEM_SIGNAL = 'cem_signal'
  CEM_CURVE = 'cem_curve'
  CEM_LEAD = 'cem_lead'
  CEM_STOP = 'cem_stop'
  CEM_SPEED_LIMIT = 'cem_speed_limit'  # Historical route labels only.
  CEM_HOLD = 'cem_hold'
  CEM_LANE_CHANGE = 'cem_lane_change'  # close-gap lane change in progress: Chill for the manoeuvre
  CCM_SPEED = 'ccm_speed'
  CCM_LEAD = 'ccm_lead'
  CCM_LAUNCH = 'ccm_launch'
  CCM_HOLD = 'ccm_hold'
  CCM_VETO = 'ccm_veto'
  SCENE_UNAVAILABLE = 'scene_unavailable'
  NO_TRIGGER = 'no_trigger'


@dataclass(frozen=True)
class Authority:
  """Fresh capability is separate from current lateral/longitudinal actuation."""

  fresh: bool
  system_long_capable: bool
  safe_mode: bool
  driving_enabled: bool
  long_active: bool
  lat_active: bool


@dataclass(frozen=True)
class LeadEvidence:
  present: bool
  tracked: bool
  radar: bool
  distance_m: float | None = None
  speed_mps: float | None = None
  accel_mps2: float | None = None
  model_probability: float | None = None


@dataclass(frozen=True)
class SceneEvidence:
  """Optional means unavailable, never an implicit negative observation."""

  observed_mono_s: float | None = None
  speed_mps: float | None = None
  set_speed_mps: float | None = None
  lead: LeadEvidence | None = None
  following_lead: bool | None = None
  speed_lead_present: bool | None = None
  left_blinker: bool | None = None
  right_blinker: bool | None = None
  lane_available: bool | None = None
  curve_detected: bool | None = None
  curve_raw: bool | None = None
  slow_lead_detected: bool | None = None
  stop_light_detected: bool | None = None
  standstill: bool | None = None
  traffic_mode: bool | None = None
  stop_sign_confirmed: bool | None = None
  forcing_stop: bool | None = None
  committed_stop: bool | None = None
  adjacent_lead_ambiguous: bool | None = None
  lane_change_chill: bool | None = None  # planner's close-gap lane-change window (SP-040); None when unknown
  low_speed_stop_scene: bool | None = None
  launch_candidate: bool | None = None
  standstill_stop_hold: bool | None = None
  launch_forced_exit: bool | None = None
  launch_lead: bool | None = None
  tesla_open_road: bool = False
  open_road_observed_s: float | None = None
  open_road_model_s: float | None = None
  open_road_model_accel: float | None = None
  open_road_model_stop: bool | None = None
  open_road_headway_s: float | None = None
  open_road_lateral_m: float | None = None
  # Climb mode evidence; None is unknown, never a negative observation.
  climb_target_mps: float | None = None  # the speed the planner actually holds for the set speed (Tesla dash-corrected)
  model_accel_mps2: float | None = None
  model_speed_3s_mps: float | None = None
  model_speed_8s_mps: float | None = None


@dataclass(frozen=True)
class ModeSettings:
  """Frozen defaults converted to SI; selection and input validation are external."""

  cem_speed_mps: float = 0.0
  cem_speed_with_lead_mps: float = 0.0
  cem_signal_mps: float = 0.0
  cem_open_road: bool = False
  cem_curves: bool = False
  cem_curves_with_lead: bool = False
  cem_lead: bool = True
  cem_stop: bool = True
  ccm_speed_mps: float = 45 * MPH_TO_MPS
  ccm_speed_with_lead_mps: float = 35 * MPH_TO_MPS
  ccm_set_speed_margin_mps: float = 3 * MPH_TO_MPS
  ccm_lead: bool = True
  ccm_launch: bool = False
  ccm_allow_chill_in_bends: bool = False
  ccm_climb: bool = False
  ccm_climb_on_bend_exit: bool = False
  ccm_climb_lead: bool = False


@dataclass(frozen=True)
class Decision:
  requested_experimental: bool
  effective_longitudinal: bool
  qualified: bool
  warm: bool
  reason: Reason
  status_code: int


def _finite(value: object, *, low: float = 0.0, high: float = 1000.0) -> bool:
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return False
  try:
    return math.isfinite(value) and low <= value <= high
  except OverflowError:
    return False


def _bool(value: object) -> bool:
  return type(value) is bool


def _valid_settings(settings: ModeSettings) -> bool:
  numeric = (
    settings.cem_speed_mps,
    settings.cem_speed_with_lead_mps,
    settings.cem_signal_mps,
    settings.ccm_speed_mps,
    settings.ccm_speed_with_lead_mps,
    settings.ccm_set_speed_margin_mps,
  )
  flags = (
    settings.cem_open_road,
    settings.cem_curves,
    settings.cem_curves_with_lead,
    settings.cem_lead,
    settings.cem_stop,
    settings.ccm_lead,
    settings.ccm_launch,
    settings.ccm_allow_chill_in_bends,
    settings.ccm_climb,
    settings.ccm_climb_on_bend_exit,
    settings.ccm_climb_lead,
  )
  return all(_finite(value, high=70.0) for value in numeric) and all(_bool(value) for value in flags)


def active_settings_key(choice: ModeChoice, settings: ModeSettings) -> tuple:
  prefix = 'ccm_' if choice is ModeChoice.CCM else 'cem_'
  return tuple(getattr(settings, f.name) for f in fields(settings) if f.name.startswith(prefix))


def restore_manual_status(choice: ModeChoice, session_code: int, persisted_code: int, persist: bool) -> ManualIntent:
  """Pure frozen CEStatus/CCStatus normalization; no storage side effects."""
  if type(persist) is not bool:
    return ManualIntent.NONE
  allowed = (
    {1: ManualIntent.FORCE_CHILL, 2: ManualIntent.FORCE_EXPERIMENTAL}
    if choice is ModeChoice.CEM
    else ({1: ManualIntent.FORCE_EXPERIMENTAL, 2: ManualIntent.FORCE_CHILL} if choice is ModeChoice.CCM else {})
  )
  if type(session_code) is int and session_code in allowed:
    return allowed[session_code]
  if persist and type(persisted_code) is int:
    return allowed.get(persisted_code, ManualIntent.NONE)
  return ManualIntent.NONE


def next_manual_status(choice: ModeChoice, current_code: int, current_experimental: bool) -> int:
  """Frozen wheel-button cycle: manual -> automatic; automatic -> opposite manual mode."""
  if type(current_experimental) is not bool:
    return 0
  if choice is ModeChoice.CEM:
    return 0 if type(current_code) is int and current_code in (1, 2) else (1 if current_experimental else 2)
  if choice is ModeChoice.CCM:
    return 0 if type(current_code) is int and current_code in (1, 2) else (2 if current_experimental else 1)
  return 0


def _manual_code(choice: ModeChoice, manual: ManualIntent) -> int:
  if choice is ModeChoice.CEM:
    return {ManualIntent.FORCE_CHILL: 1, ManualIntent.FORCE_EXPERIMENTAL: 2}.get(manual, 0)
  if choice is ModeChoice.CCM:
    return {ManualIntent.FORCE_EXPERIMENTAL: 1, ManualIntent.FORCE_CHILL: 2}.get(manual, 0)
  return 0


def _scene_current(now_s: float, scene: SceneEvidence) -> bool:
  observed = scene.observed_mono_s
  speed = scene.speed_mps
  return observed is not None and speed is not None and _finite(observed, high=1e12) and 0 <= now_s - observed <= SCENE_MAX_AGE_S and _finite(speed, high=80.0)


def _lead_valid(lead: LeadEvidence | None) -> bool:
  return lead is not None and all(_bool(value) for value in (lead.present, lead.tracked, lead.radar))


def _stable_lead(scene: SceneEvidence) -> bool:
  lead = scene.lead
  speed = scene.speed_mps
  if lead is None or speed is None or not _lead_valid(lead) or not lead.present or not lead.tracked or not _finite(speed, high=80.0):
    return False
  distance = lead.distance_m
  lead_speed = lead.speed_mps
  acceleration = lead.accel_mps2
  probability = lead.model_probability
  if distance is None or lead_speed is None or acceleration is None:
    return False
  if not all(_finite(value, high=500.0) for value in (distance, lead_speed)):
    return False
  if not _finite(acceleration, low=-20.0, high=20.0):
    return False
  if not lead.radar and not _finite(probability, high=1.0):
    return False
  confident = lead.radar or probability is not None and probability >= 0.9
  max_distance = min(90.0, max(35.0, speed * 4.5))
  max_closing_speed = max(1.25, 0.05 * speed)
  return bool(
    confident and distance < max_distance and lead_speed > 1.5 and max(0.0, -acceleration) <= 0.2 and max(0.0, speed - lead_speed) <= max_closing_speed
  )


def _credible_slow_lead(scene: SceneEvidence) -> bool:
  lead, speed = scene.lead, scene.speed_mps
  if lead is None or speed is None or not _lead_valid(lead) or not lead.present or not _finite(speed, high=80.0):
    return False
  if not lead.radar and not (_finite(lead.model_probability, high=1.0) and lead.model_probability is not None and lead.model_probability >= 0.85):
    return False
  return bool(_finite(lead.distance_m, high=500.0) and lead.distance_m is not None and lead.distance_m < max(40.0, speed * 4.0))


def _open_road_handoff(scene: SceneEvidence) -> bool:
  lead, speed = scene.lead, scene.speed_mps
  if lead is None or speed is None or not _lead_valid(lead) or not lead.present or not _finite(speed, high=80.0):
    return False
  if lead.distance_m is None or lead.speed_mps is None or not all(_finite(value, high=500.0) for value in (lead.distance_m, lead.speed_mps)):
    return False
  return lead.speed_mps >= 1.0 and lead.distance_m >= max(35.0, speed * 1.5) and max(0.0, speed - lead.speed_mps) <= 3.0


class ConditionalModePolicy:
  """One stateful policy per drive/session; old scene state never crosses modes."""

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.choice = ModeChoice.STOCK
    self.last_now: float | None = None
    self.last_settings: ModeSettings | None = None
    self.curve_held = False
    self.curve_cem_current = False
    self.curve_clear_since: float | None = None
    self._reset_cem()
    self.ccm_candidate: Reason | None = None
    self.ccm_candidate_since = 0.0
    self.ccm_chill = False
    self.ccm_active_reason = Reason.NO_TRIGGER
    self.ccm_launch_lead = False
    self.ccm_hold_until = 0.0
    self.ccm_false_since: float | None = None
    self._reset_climb()

  def _reset_climb(self) -> None:
    self.climbing = False
    self.climb_allowed = True       # a fresh engagement may climb
    self.climb_since: float | None = None
    self.climb_ended_at = -1e9
    self.climb_lead_seen = -1e9
    self.climb_held_since: float | None = None
    self.climb_last_target: float | None = None
    self.climb_end_reason = ''
    self.climb_kind = ''            # 'clear' or 'lead'
    self.climb_away_since: float | None = None

  def reset_transition(self, *, settings_edit: bool = False) -> None:
    # Preserve bend evidence across a temporary authority/data interruption.
    # An interruption cancels clearance; it cannot count as observed clear time.
    curve_held, cem_current, clear_since = self.curve_held, self.curve_cem_current, self.curve_clear_since
    self.reset()
    self.curve_held = curve_held
    if settings_edit:
      self.curve_cem_current, self.curve_clear_since = cem_current, clear_since

  def _reset_cem(self) -> None:
    self.open_road = OpenRoad()
    self.cem_hold_until = 0.0
    self.cem_false_since: float | None = None
    self.cem_active = False
    self.cem_active_reason = Reason.NO_TRIGGER
    self.cem_slow_lead_until = 0.0
    self.cem_open_road_until = 0.0
    self.cem_previous_open_road = False
    self.cem_previous_standstill = False
    self.cem_previous_stop_hold = False
    self.cem_stop_release_pending = False
    self.cem_launch_suppress_until = 0.0
    self.cem_speed_active = False
    self.cem_speed_until = 0.0

  @staticmethod
  def _decision(requested: bool, authority: Authority, qualified: bool, warm: bool, reason: Reason, status: int) -> Decision:
    return Decision(requested, bool(requested and qualified and authority.driving_enabled and authority.long_active), qualified, warm, reason, status)

  def step(
    self,
    now_s: float,
    choice: ModeChoice,
    manual: ManualIntent,
    authority: Authority,
    scene: SceneEvidence,
    settings: ModeSettings | None = None,
    *,
    stock_experimental: bool = False,
  ) -> Decision:
    if settings is None:
      settings = ModeSettings()
    if not isinstance(choice, ModeChoice):
      self.reset()
      return self._decision(False, authority, False, False, Reason.UNAVAILABLE, 0)
    if not _finite(now_s, high=1e12) or (self.last_now is not None and now_s < self.last_now):
      self.reset()
      return self._decision(False, authority, False, False, Reason.UNAVAILABLE, 0)
    if choice is not self.choice:
      self.reset()
      self.choice = choice
    elif self.last_now is not None and now_s - self.last_now > SCENE_MAX_AGE_S:
      self.reset_transition()
      self.choice = choice
    if self.last_settings is not None and active_settings_key(choice, settings) != active_settings_key(choice, self.last_settings):
      self.reset_transition(settings_edit=True)
      self.choice = choice
    self.last_settings = settings
    self.last_now = now_s
    authority_valid = all(
      _bool(value)
      for value in (authority.fresh, authority.system_long_capable, authority.safe_mode, authority.driving_enabled, authority.long_active, authority.lat_active)
    )
    qualified = bool(authority_valid and authority.fresh and authority.system_long_capable and not authority.safe_mode)
    warm = bool(authority_valid and authority.fresh and (authority.driving_enabled or authority.lat_active))
    if choice is ModeChoice.STOCK and scene.committed_stop is not True:
      return self._decision(bool(stock_experimental) if type(stock_experimental) is bool else False, authority, qualified, warm, Reason.STOCK, 0)
    if choice not in (ModeChoice.CEM, ModeChoice.CCM, ModeChoice.STOCK) or not qualified or not _valid_settings(settings):
      self.reset()
      return self._decision(False, authority, False, False, Reason.UNAVAILABLE, 0)
    if warm and _scene_current(now_s, scene):
      self._update_curve(now_s, scene)
    else:
      self.curve_clear_since = None
      self.curve_cem_current = False
    if (warm and authority.driving_enabled and authority.long_active and
        _scene_current(now_s, scene) and scene.committed_stop is True):
      self.open_road.reset()
      if choice is ModeChoice.CEM and self._speed_trigger(scene, settings):
        self.cem_speed_until = now_s + CEM_HOLD_S
      return self._decision(True, authority, True, warm, Reason.CEM_STOP, 8)
    if not isinstance(manual, ManualIntent):
      manual = ManualIntent.NONE
    if manual is not ManualIntent.NONE:
      # Manual intent clears confirmation/dwell but preserves the observed stop-release quiet period.
      suppress_until = self.cem_launch_suppress_until if choice is ModeChoice.CEM else 0.0
      current_scene = _scene_current(now_s, scene)
      if (
        choice is ModeChoice.CEM
        and current_scene
        and scene.standstill is False
        and (self.cem_previous_standstill and self.cem_previous_stop_hold or self.cem_stop_release_pending)
      ):
        suppress_until = now_s + CEM_STOP_RELEASE_SUPPRESS_S
      curve_held, clear_since, cem_current = self.curve_held, self.curve_clear_since, self.curve_cem_current
      self.reset()
      self.curve_held, self.curve_clear_since, self.curve_cem_current = curve_held, clear_since, cem_current
      self.choice = choice
      self.last_now = now_s
      if choice is ModeChoice.CEM:
        self.cem_launch_suppress_until = suppress_until
        self.cem_previous_standstill = current_scene and scene.standstill is True
      requested = manual is ManualIntent.FORCE_EXPERIMENTAL
      return self._decision(
        requested, authority, qualified, warm, Reason.MANUAL_EXPERIMENTAL if requested else Reason.MANUAL_CHILL, _manual_code(choice, manual)
      )
    if not warm or not _scene_current(now_s, scene):
      self._reset_cem()
      self.ccm_chill = False
      self.ccm_candidate = None
      self.ccm_false_since = None
      self._end_climb(now_s, 'unavailable')
      return self._decision(choice is ModeChoice.CCM, authority, qualified, warm, Reason.SCENE_UNAVAILABLE, 0)
    return self._cem(now_s, authority, warm, scene, settings) if choice is ModeChoice.CEM else self._ccm(now_s, authority, warm, scene, settings)

  def _update_curve(self, now_s: float, scene: SceneEvidence) -> None:
    if scene.curve_detected is True:
      self.curve_held = True
      self.curve_cem_current = True
      self.curve_clear_since = None
    elif scene.curve_detected is False and scene.curve_raw is False:
      if self.curve_clear_since is None:
        self.curve_clear_since = now_s
      if now_s - self.curve_clear_since >= CURVE_CLEAR_S:
        self.curve_held = False
    else:
      # Unknown is not clear; CCM separately vetoes unavailable evidence.
      self.curve_clear_since = None

  def _speed_trigger(self, scene: SceneEvidence, settings: ModeSettings) -> bool:
    lead = scene.speed_lead_present
    limit = (settings.cem_speed_with_lead_mps if lead is True else settings.cem_speed_mps if lead is False
             else min(settings.cem_speed_mps, settings.cem_speed_with_lead_mps))
    # When the lead class is unknown only the overlap of both rules is certain.
    # Enter exactly below the selected limit; release above a small speed band.
    margin = SPEED_EXIT_MARGIN_MPS if self.cem_speed_active else 0.0
    self.cem_speed_active = bool(limit > 0.0 and scene.speed_mps is not None and scene.speed_mps < limit + margin)
    return self.cem_speed_active

  def _cem_trigger(self, scene: SceneEvidence, settings: ModeSettings, *, launch_suppressed: bool = False) -> Reason | None:
    speed = scene.speed_mps
    if speed is None:
      return None
    lead = scene.lead
    following = scene.following_lead
    if self._speed_trigger(scene, settings):
      return Reason.CEM_SPEED
    set_speed = scene.set_speed_mps
    if (
      not launch_suppressed
      and settings.cem_open_road
      and not scene.tesla_open_road
      and lead is not None
      and _lead_valid(lead)
      and _bool(following)
      and not (lead.present or lead.tracked or following)
      and set_speed is not None
      and _finite(set_speed, high=80.0)
    ):
      if 0 < set_speed <= speed <= set_speed + MPH_TO_MPS:
        return Reason.CEM_OPEN_ROAD
    if (
      _bool(scene.left_blinker)
      and _bool(scene.right_blinker)
      and _bool(scene.lane_available)
      and settings.cem_signal_mps > speed
      and (scene.left_blinker or scene.right_blinker)
      and not scene.lane_available
    ):
      return Reason.CEM_SIGNAL
    if settings.cem_curves and self.curve_held and self.curve_cem_current and (settings.cem_curves_with_lead or scene.speed_lead_present is False):
      return Reason.CEM_CURVE
    if not launch_suppressed and settings.cem_lead and scene.slow_lead_detected is True and speed <= 35.31:
      return Reason.CEM_LEAD
    if settings.cem_stop and (scene.stop_light_detected is True or scene.forcing_stop is True):
      return Reason.CEM_STOP
    return None

  def _cem(self, now_s: float, authority: Authority, warm: bool, scene: SceneEvidence, settings: ModeSettings) -> Decision:
    committed = settings.cem_stop and scene.forcing_stop is True and authority.driving_enabled and authority.long_active
    if scene.standstill is True and scene.standstill_stop_hold is None and not committed and self.cem_previous_stop_hold:
      return self._decision(True, authority, True, warm, Reason.CEM_STOP, 8)
    speed_requested = self._speed_trigger(scene, settings)
    if speed_requested:
      self.cem_speed_until = now_s + CEM_HOLD_S
    speed_hold = scene.speed_lead_present is None and now_s < self.cem_speed_until
    if not _bool(scene.standstill) or (scene.standstill and settings.cem_stop and not committed and not speed_requested and not speed_hold and not _bool(scene.standstill_stop_hold)):
      self._reset_cem()
      return self._decision(False, authority, True, warm, Reason.SCENE_UNAVAILABLE, 0)
    released = not scene.standstill and (self.cem_previous_standstill and self.cem_previous_stop_hold or self.cem_stop_release_pending)
    if released:
      self.cem_launch_suppress_until = now_s + CEM_STOP_RELEASE_SUPPRESS_S
      self.cem_hold_until = self.cem_slow_lead_until = 0.0
      self.cem_false_since = None
      self.cem_active = False
      self.cem_stop_release_pending = False
    if scene.standstill:
      self.open_road.reset()
      stop_held = committed or settings.cem_stop and scene.standstill_stop_hold is True
      speed_held = speed_requested or speed_hold
      if speed_requested:
        self.cem_hold_until = now_s + CEM_HOLD_S
      elif not speed_hold:
        self.cem_hold_until = 0.0
      held = stop_held or speed_held
      self.cem_slow_lead_until = self.cem_open_road_until = 0.0
      self.cem_false_since = None
      self.cem_previous_open_road = False
      if held:
        self.cem_stop_release_pending = False
      elif self.cem_previous_stop_hold:
        self.cem_stop_release_pending = True
      if self.cem_stop_release_pending:
        self.cem_launch_suppress_until = now_s + CEM_STOP_RELEASE_SUPPRESS_S
      self.cem_active = held
      self.cem_previous_standstill = True
      self.cem_previous_stop_hold = stop_held
      self.cem_active_reason = Reason.CEM_STOP if stop_held else Reason.CEM_SPEED if speed_held else Reason.NO_TRIGGER
      return self._decision(held, authority, True, warm, self.cem_active_reason, 8 if stop_held else 6 if speed_held else 0)
    self.cem_previous_standstill = self.cem_previous_stop_hold = False
    if scene.lane_change_chill is True:
      # Close-gap lane change (SP-040): Chill for the manoeuvre so the end-to-end model cannot keep braking for
      # the car in the lane being left. Automatic holds are dropped; a slower lead re-triggers afterwards.
      self.open_road.reset()
      self.cem_hold_until = self.cem_slow_lead_until = self.cem_open_road_until = 0.0
      self.cem_false_since = None
      self.cem_active = False
      self.cem_previous_open_road = False
      self.cem_active_reason = Reason.NO_TRIGGER
      return self._decision(False, authority, True, warm, Reason.CEM_LANE_CHANGE, 0)
    launch_suppressed = now_s < self.cem_launch_suppress_until
    if (scene.tesla_open_road is True and settings.cem_open_road and not launch_suppressed and
        authority.driving_enabled and authority.long_active):
      open_road_active = self.open_road.step(now_s, scene)
    else:
      self.open_road.reset()
      open_road_active = False
    trigger = self._cem_trigger(scene, settings, launch_suppressed=launch_suppressed)
    # Preserve independent condition holds; an already-confirmed Open Road
    # release must not acquire an extra generic mode hold of its own.
    if scene.tesla_open_road and self.cem_active_reason is Reason.CEM_OPEN_ROAD:
      self.cem_hold_until = 0.0
      self.cem_false_since = None
      self.cem_active = False
    if trigger is not None:
      self.cem_hold_until = now_s + CEM_HOLD_S
      self.cem_false_since = None
      self.cem_open_road_until = 0.0
      self.cem_slow_lead_until = now_s + CEM_SLOW_LEAD_HOLD_S if trigger is Reason.CEM_LEAD else 0.0
      self.cem_active_reason = trigger
    else:
      if self.cem_active and self.cem_false_since is None:
        self.cem_false_since = now_s
      elif not self.cem_active:
        self.cem_false_since = None
    if scene.tesla_open_road or not settings.cem_open_road:
      self.cem_open_road_until = 0.0
    elif self.cem_previous_open_road and _open_road_handoff(scene):
      self.cem_open_road_until = now_s + CEM_OPEN_ROAD_HANDOFF_S
    open_road_hold = trigger is None and now_s < self.cem_open_road_until and _open_road_handoff(scene)
    slow_lead_hold = settings.cem_lead and now_s < self.cem_slow_lead_until and _credible_slow_lead(scene)
    if not slow_lead_hold:
      self.cem_slow_lead_until = 0.0
    if open_road_hold:
      self.cem_active_reason = Reason.CEM_OPEN_ROAD
    if slow_lead_hold and trigger is None:
      self.cem_active_reason = Reason.CEM_LEAD
    self.cem_active = bool(
      trigger is not None
      or slow_lead_hold
      or open_road_hold
      or now_s < self.cem_hold_until
      or self.cem_false_since is not None
      and now_s - self.cem_false_since < CEM_EXIT_BUFFER_S
    )
    self.cem_previous_open_road = trigger is Reason.CEM_OPEN_ROAD
    reason = trigger if trigger is not None else Reason.CEM_HOLD if self.cem_active else Reason.NO_TRIGGER
    if open_road_active and not self.cem_active:
      self.cem_active = True
      self.cem_active_reason = reason = Reason.CEM_OPEN_ROAD
    status_reason = self.cem_active_reason if self.cem_active else Reason.NO_TRIGGER
    status = {
      Reason.CEM_CURVE: 3,
      Reason.CEM_LEAD: 4,
      Reason.CEM_SIGNAL: 5,
      Reason.CEM_SPEED: 6,
      Reason.CEM_OPEN_ROAD: 6,
      Reason.CEM_STOP: 8,
    }.get(status_reason, 0)
    return self._decision(self.cem_active, authority, True, warm, reason, status)

  def _end_climb(self, now_s: float, reason: str) -> None:
    if self.climbing:
      self.climbing = False
      self.climb_ended_at = now_s
      self.climb_end_reason = reason
    self.climb_since = None
    self.climb_kind = ''

  def _climb_target(self, scene: SceneEvidence) -> float | None:
    target = scene.climb_target_mps
    if target is None or not _finite(target, high=80.0) or target <= 0.0:
      target = scene.set_speed_mps
    return target if target is not None and _finite(target, high=80.0) and target > 0.0 else None

  def _climb_observe(self, now_s: float, authority: Authority, scene: SceneEvidence, speed: float) -> None:
    """Track what lets a climb recover speed.

    Climbs only recover speed that something took away: engaging, a set-speed rise, a lead, a bend, a turn,
    a stop or slow traffic, or the driver's own input. Experimental slowing on its own from a speed it was
    holding is treated as deliberate, so no climb follows it.
    """
    lead = scene.lead
    lead_now = lead is not None and (lead.present is True or lead.tracked is True)
    if lead_now:
      self.climb_lead_seen = now_s
    target = self._climb_target(scene)
    rose = target is not None and self.climb_last_target is not None and target > self.climb_last_target + CLIMB_TARGET_RISE_MPS
    events = (
      lead_now,
      scene.curve_detected is True or scene.curve_raw is True,
      scene.left_blinker is True or scene.right_blinker is True,
      scene.standstill is True or speed < CLIMB_RECOVER_SPEED_MPS,
      scene.stop_light_detected is True or scene.forcing_stop is True or scene.stop_sign_confirmed is True,
      scene.slow_lead_detected is True or scene.low_speed_stop_scene is True,
      authority.long_active is not True,  # the driver took over (accelerator) or control paused
      rose,
    )
    if any(events):
      self.climb_allowed = True
      self.climb_held_since = None
    if target is None:
      self.climb_held_since = None
      return
    if self.climb_last_target is None or rose or target < self.climb_last_target:
      self.climb_last_target = target
    if not self.climbing and abs(target - speed) <= CLIMB_HELD_BAND_MPS and not any(events):
      if self.climb_held_since is None:
        self.climb_held_since = now_s
      if now_s - self.climb_held_since >= CLIMB_HELD_S:
        self.climb_allowed = False  # holding speed: from here a slowdown is the model's choice
    else:
      self.climb_held_since = None

  def _lead_pulling_away(self, now_s: float, scene: SceneEvidence, speed: float, gap_s: float, away_mps: float) -> bool:
    """A credible lead far ahead that keeps moving away (the model is not keeping up with it)."""
    lead = scene.lead
    if (lead is None or not _lead_valid(lead) or lead.present is not True or lead.distance_m is None or
        lead.speed_mps is None or lead.accel_mps2 is None or not _finite(lead.distance_m, high=500.0) or
        not _finite(lead.speed_mps, high=100.0) or not _finite(lead.accel_mps2, low=-20.0, high=20.0)):
      return False
    confident = lead.radar or (lead.model_probability is not None and _finite(lead.model_probability, high=1.0) and
                               lead.model_probability >= 0.9)
    return bool(confident and lead.distance_m >= gap_s * max(speed, 1.0) and
                lead.speed_mps >= speed + away_mps and lead.accel_mps2 >= CLIMB_LEAD_BRAKE)

  def _climb_step(self, now_s: float, scene: SceneEvidence, settings: ModeSettings, speed: float, signal: bool) -> bool:
    """Return True while a climb is active."""
    target = self._climb_target(scene)
    lead = scene.lead
    lead_clear = (lead is not None and _lead_valid(lead) and lead.present is False and lead.tracked is False and
                  now_s - self.climb_lead_seen >= CLIMB_LEAD_CLEAR_S)
    accel, plan3, plan8 = scene.model_accel_mps2, scene.model_speed_3s_mps, scene.model_speed_8s_mps
    model_known = all(v is not None and _finite(v, low=-20.0, high=100.0) for v in (accel, plan3, plan8))
    short = target - speed if target is not None else None
    away = settings.ccm_climb_lead and self._lead_pulling_away(now_s, scene, speed, CLIMB_LEAD_MIN_GAP_S, CLIMB_LEAD_AWAY_MPS)
    if away:
      if self.climb_away_since is None:
        self.climb_away_since = now_s
    else:
      self.climb_away_since = None
    if self.climbing:
      if self.climb_kind == 'lead':
        lead_ok = (lead is not None and _lead_valid(lead) and lead.present is False or
                   self._lead_pulling_away(now_s, scene, speed, CLIMB_LEAD_END_GAP_S, CLIMB_LEAD_STOP_AWAY_MPS))
      else:
        lead_ok = lead_clear
      reason = ('lead' if not lead_ok else
                'unknown' if not model_known or short is None else
                'done' if short <= CLIMB_DONE_MPS else
                'model slowing' if accel <= CLIMB_SLOWING_ACCEL or plan8 < speed - CLIMB_SLOWING_8S_MPS else
                '')
      if reason:
        self._end_climb(now_s, reason)
        if reason == 'done':
          self.climb_allowed = False  # reached the target: holding from here
      return self.climbing
    calm = (model_known and accel >= CLIMB_CALM_ACCEL and plan3 >= speed - CLIMB_CALM_3S_MPS and
            plan8 >= speed - CLIMB_CALM_8S_MPS)
    kind = 'clear' if lead_clear else 'lead' if (self.climb_away_since is not None and
                                                 now_s - self.climb_away_since >= CLIMB_LEAD_AWAY_S) else ''
    ready = (bool(kind) and self.climb_allowed and calm and not signal and short is not None and
             speed >= settings.ccm_speed_mps and short > settings.ccm_set_speed_margin_mps and
             now_s - self.climb_ended_at >= CLIMB_REENTRY_S)
    if not ready:
      self.climb_since = None
      return False
    if self.climb_since is None:
      self.climb_since = now_s
    if now_s - self.climb_since >= CLIMB_CONFIRM_S:
      self.climbing = True
      self.climb_kind = kind
      self.climb_end_reason = ''
    return self.climbing

  def _ccm(self, now_s: float, authority: Authority, warm: bool, scene: SceneEvidence, settings: ModeSettings) -> Decision:
    speed = scene.speed_mps
    if speed is None:
      return self._decision(True, authority, True, warm, Reason.SCENE_UNAVAILABLE, 0)
    climb = settings.ccm_climb
    signal = scene.left_blinker is True or scene.right_blinker is True
    if climb:
      self._climb_observe(now_s, authority, scene, speed)
    veto_fields = (
      scene.standstill,
      scene.left_blinker,
      scene.right_blinker,
      scene.traffic_mode,
      scene.curve_detected,
      scene.slow_lead_detected,
      scene.stop_light_detected,
      scene.stop_sign_confirmed,
      scene.forcing_stop,
      scene.adjacent_lead_ambiguous,
      scene.low_speed_stop_scene,
    )
    launch = settings.ccm_launch and speed < 15.0 * MPH_TO_MPS and scene.launch_candidate is True
    # Climb mode: an indicator forces Experimental only below the turn speed (a lane change at speed
    # just pauses new climbs); "climb on bend exit" lets a climb start once the bend itself is over.
    signal_veto = signal and (not climb or speed < CLIMB_SIGNAL_VETO_MPS)
    hard_veto = (signal_veto, scene.traffic_mode,
                 scene.slow_lead_detected, scene.stop_light_detected, scene.stop_sign_confirmed,
                 scene.forcing_stop, scene.adjacent_lead_ambiguous)
    bend_held = self.curve_held and not (climb and settings.ccm_climb_on_bend_exit and scene.curve_raw is False)
    curve_veto = not settings.ccm_allow_chill_in_bends and (bend_held or scene.curve_detected is True)
    if curve_veto or any(value is True for value in hard_veto) or (scene.standstill is True and not launch) or (scene.low_speed_stop_scene is True and not launch):
      self.ccm_chill = False
      self.ccm_candidate = None
      self.ccm_false_since = None
      self._end_climb(now_s, 'veto')
      return self._decision(True, authority, True, warm, Reason.CCM_VETO, 0)
    if any(not _bool(value) for value in veto_fields):
      self.ccm_chill = False
      self.ccm_candidate = None
      self.ccm_false_since = None
      self._end_climb(now_s, 'unavailable')
      return self._decision(True, authority, True, warm, Reason.SCENE_UNAVAILABLE, 0)
    launch_relevant = speed <= 15.0 * MPH_TO_MPS or self.ccm_active_reason is Reason.CCM_LAUNCH
    if settings.ccm_launch and launch_relevant and (not _bool(scene.launch_candidate) or not _bool(scene.launch_forced_exit) or launch and not _bool(scene.launch_lead)):
      self.ccm_chill = False
      self.ccm_candidate = None
      self.ccm_false_since = None
      self._end_climb(now_s, 'unavailable')
      return self._decision(True, authority, True, warm, Reason.SCENE_UNAVAILABLE, 0)
    if climb and not launch and self._climb_step(now_s, scene, settings, speed, signal):
      # The climb owns Chill; the lead and launch rules restart from scratch afterwards.
      self.ccm_chill = False
      self.ccm_candidate = None
      self.ccm_false_since = None
      self.ccm_active_reason = Reason.NO_TRIGGER
      return self._decision(False, authority, True, warm, Reason.CCM_SPEED, 6)
    lead = scene.lead
    candidate = None
    if launch:
      candidate = Reason.CCM_LAUNCH
    elif (
      not climb
      and lead is not None
      and _lead_valid(lead)
      and not lead.present
      and not lead.tracked
      and scene.set_speed_mps is not None
      and _finite(scene.set_speed_mps, high=80.0)
      and speed >= settings.ccm_speed_mps
      and scene.set_speed_mps - speed >= settings.ccm_set_speed_margin_mps
    ):
      candidate = Reason.CCM_SPEED
    elif settings.ccm_lead and speed >= settings.ccm_speed_with_lead_mps and _stable_lead(scene):
      candidate = Reason.CCM_LEAD
    if candidate is not None:
      if candidate is not self.ccm_candidate:
        self.ccm_candidate = candidate
        self.ccm_candidate_since = now_s
      self.ccm_false_since = None
      confirm = {Reason.CCM_SPEED: CCM_SPEED_CONFIRM_S, Reason.CCM_LEAD: CCM_LEAD_CONFIRM_S, Reason.CCM_LAUNCH: CCM_LAUNCH_CONFIRM_S}[candidate]
      if self.ccm_chill or now_s - self.ccm_candidate_since >= confirm:
        self.ccm_chill = True
        self.ccm_active_reason = candidate
        self.ccm_launch_lead = candidate is Reason.CCM_LAUNCH and scene.launch_lead is True
        self.ccm_hold_until = max(self.ccm_hold_until, now_s + CCM_MIN_DWELL_S)
    else:
      self.ccm_candidate = None
      if settings.ccm_launch and scene.launch_forced_exit is True:
        self.ccm_chill = False
        self.ccm_active_reason = Reason.NO_TRIGGER
        self.ccm_false_since = None
        self.ccm_hold_until = 0.0
      if self.ccm_chill and self.ccm_false_since is None:
        self.ccm_false_since = now_s
      if self.ccm_chill and not (now_s < self.ccm_hold_until or self.ccm_false_since is not None and now_s - self.ccm_false_since < CCM_EXIT_BUFFER_S):
        self.ccm_chill = False
        self.ccm_active_reason = Reason.NO_TRIGGER
    reason = self.ccm_active_reason if self.ccm_chill and candidate else Reason.CCM_HOLD if self.ccm_chill else Reason.NO_TRIGGER
    lead_status = self.ccm_active_reason is Reason.CCM_LEAD or self.ccm_active_reason is Reason.CCM_LAUNCH and self.ccm_launch_lead
    status = 4 if self.ccm_chill and lead_status else 6 if self.ccm_chill else 0
    return self._decision(not self.ccm_chill, authority, True, warm, reason, status)
