"""Bounded lane-centering contribution for lateral control.

The caller owns model freshness, axis authority, and final curvature limiting.
This module neither reads settings nor sends commands.
"""

from dataclasses import dataclass
from enum import StrEnum
import math
from numbers import Real
from typing import Any

import numpy as np

from openpilot.cereal import log
from openpilot.common.scalar_clip import clip_scalar
from openpilot.starpilot.lateral.lane_tuning import valid_tuning, STOCK_CORRECTION, MAX_CORRECTION


STRENGTH_BASELINE = 1.5
STRENGTH_DEFAULT = 2.1
STRENGTH_MAX = 2 * STRENGTH_BASELINE

MIN_SPEED = 5.0
MIN_LANE_PROB = 0.6
MAX_LANE_STD = 0.3
MIN_LANE_WIDTH = 2.6
MAX_LANE_WIDTH = 4.8
MAX_OFFSET = 0.3
MIN_CENTER_TO_LINE = 1.1
CENTER_ERROR_DEADBAND = 0.02
E2E_MAX_PATH_STD = 0.35
E2E_BREAK_IN_START = 0.15
E2E_BREAK_IN_FULL = 0.50
ACQUIRE_TAU = 0.35
RELEASE_TAU = 0.2


class ControlMode(StrEnum):
  OFF = "off"
  LATERAL_ONLY = "lateral_only"
  LONGITUDINAL_ONLY = "longitudinal_only"
  COMBINED = "combined"

  @property
  def lateral_authority(self) -> bool:
    return self in (ControlMode.LATERAL_ONLY, ControlMode.COMBINED)


@dataclass(frozen=True)
class LaneCenteringSettings:
  enabled: bool = False
  offset_m: float = 0.0
  e2e_authority: float = 1.0
  pause_on_signal: bool = True
  strength: float = STRENGTH_DEFAULT
  min_speed_mps: float = 0.0
  max_correction: float = 0.0018
  max_width: float = 4.8
  confidence_split: float = 0.2
  min_confidence: float = 0.1
  high_std: float = 1.0
  low_std: float = 0.6
  edge_std: float = 1.0
  car_width: float = 1.85
  clearance: float = 0.0
  smoothing_tau: float = 0.2
  smoothing_distance: float = 0.25
  response_tau: float = 0.35
  release_tau: float = 0.2
  single_clearance: float = 0.45
  single_deadband: float = 0.02  # Saved name retained; applies to every mode.
  lookahead_time: float = 1.0
  override_start: float = E2E_BREAK_IN_START
  override_full: float = E2E_BREAK_IN_FULL
  curve_adaptation: bool = True
  response_min: float = 0.1
  e2e_min: float = 0.0
  curve_start_radius: float = 2000.0
  curve_full_radius: float = 250.0
  # Retained for historical replay; live preferences always enable all three.
  mode1: bool = True
  mode2: bool = True
  mode3: bool = True

  @property
  def min_width(self) -> float:
    return self.car_width + 2 * self.clearance


@dataclass(frozen=True)
class LaneCenteringRequest:
  """Optional host configuration; vehicle/model facts come from the host itself."""

  mode: ControlMode
  settings: LaneCenteringSettings
  elapsed_seconds: float
  time_discontinuity: bool = False
  version: int = 4


@dataclass(frozen=True)
class LaneCenteringInput:
  model: Any
  model_sample_id: int
  model_valid: bool
  base_curvature: float
  speed_mps: float
  mode: ControlMode
  lateral_active: bool
  settings: LaneCenteringSettings
  elapsed_seconds: float
  turn_signal_active: bool = False
  driver_override: bool = False
  time_discontinuity: bool = False


@dataclass(frozen=True)
class LaneCenteringResult:
  # None means the caller's proposed curvature was invalid and must be rejected.
  candidate_curvature: float | None
  correction: float
  direction: int
  reason: str
  correction_limit: float = STOCK_CORRECTION


def _finite_number(value: Any) -> float | None:
  if type(value) is float:
    return value if math.isfinite(value) else None
  if isinstance(value, bool) or not isinstance(value, Real):
    return None
  try:
    result = float(value)
  except (TypeError, ValueError, OverflowError):
    return None
  return result if math.isfinite(result) else None


def _path(model_path: Any) -> tuple[np.ndarray, np.ndarray] | None:
  try:
    return _path_xy(model_path.x, model_path.y)
  except (AttributeError, TypeError, ValueError, OverflowError):
    return None


def _path_xy(xs: Any, ys: Any) -> tuple[np.ndarray, np.ndarray] | None:
  try:
    x = np.asarray(xs)
    y = np.asarray(ys)
  except (TypeError, ValueError, OverflowError):
    return None
  if x.ndim != 1 or y.ndim != 1 or len(x) < 2 or len(x) != len(y):
    return None
  if x.dtype.kind not in "fiu" or y.dtype.kind not in "fiu":
    return None
  x, y = x.astype(float), y.astype(float)
  if not np.isfinite(x).all() or not np.isfinite(y).all() or not np.all(np.diff(x) > 0):
    return None
  return x, y


def _at(path: tuple[np.ndarray, np.ndarray], distance: float) -> float | None:
  x, y = path
  if not x[0] <= distance <= x[-1]:
    return None
  return float(np.interp(distance, x, y))


def raw_correction(model: Any, speed_mps: float, offset_m: float, e2e_authority: float, *, _paths=None) -> float | None:
  """Return a finite raw curvature correction, or None for unqualified model data."""
  speed = _finite_number(speed_mps)
  offset = _finite_number(offset_m)
  authority = _finite_number(e2e_authority)
  if speed is None or offset is None or authority is None or speed < MIN_SPEED:
    return None
  try:
    lines = model.laneLines
    probs = model.laneLineProbs
    stds = model.laneLineStds
    if len(lines) < 3 or len(probs) < 3 or len(stds) < 3:
      return None
    for i in (1, 2):
      prob = _finite_number(probs[i])
      std = _finite_number(stds[i])
      if prob is None or std is None or not MIN_LANE_PROB <= prob <= 1.0 or not 0.0 <= std <= MAX_LANE_STD:
        return None
    left_path, right_path, model_path = (_path(lines[1]), _path(lines[2]), _path(model.position)) if _paths is None else _paths[:3]
    if left_path is None or right_path is None or model_path is None:
      return None
    lookahead = clip_scalar(speed, 8.0, 35.0)
    left, right, model_y = _at(left_path, lookahead), _at(right_path, lookahead), _at(model_path, lookahead)
    if left is None or right is None or model_y is None:
      return None
    width = right - left
    if not MIN_LANE_WIDTH <= width <= MAX_LANE_WIDTH:
      return None
    safe_offset = min(MAX_OFFSET, max(0.0, width * 0.5 - MIN_CENTER_TO_LINE))
    target_y = (left + right) * 0.5 + clip_scalar(offset, -safe_offset, safe_offset)
    return _error_correction(model, speed, target_y - model_y, authority, lookahead)
  except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
    return None


def _error_correction(model, speed, error, authority, lookahead, deadband=CENTER_ERROR_DEADBAND,
                      override_start=E2E_BREAK_IN_START, override_full=E2E_BREAK_IN_FULL, diagnostic=None, _paths=None):
  error_abs = abs(error)
  if diagnostic is not None:
    diagnostic.update(error_m=error, model_attenuation=1.0, path_std_m=None)
  error = math.copysign(max(0.0, error_abs - deadband), error)
  try:
    std_path = _path_xy(model.position.x, model.position.yStd)
    if _paths is not None: _paths["std"] = std_path
    path_std = _at(std_path, lookahead) if std_path is not None else None
    if diagnostic is not None:
      diagnostic["path_std_m"] = path_std
    if path_std is not None and 0.0 <= path_std <= E2E_MAX_PATH_STD:
      break_in = clip_scalar((error_abs - override_start) / (override_full - override_start), 0.0, 1.0)
      error *= 1.0 - clip_scalar(authority, 0.0, 1.0) * break_in
      if diagnostic is not None:
        diagnostic["model_attenuation"] = 1.0 - clip_scalar(authority, 0.0, 1.0) * break_in
  except (AttributeError, TypeError, ValueError, OverflowError):
    pass
  return 2.0 * error / lookahead ** 2


def _boundaries(model, lookahead, settings, _paths=None, *, include_edges=True):
  # Fixed order supplies line-before-edge, then left-before-right tie breaking.
  accepted = []
  for kind, index, side in (("line", 1, 0), ("line", 2, 1), ("edge", 0, 0), ("edge", 1, 1)):
    if kind == "edge" and not include_edges:
      continue
    try:
      std = _finite_number((model.laneLineStds if kind == "line" else model.roadEdgeStds)[index])
      limit = settings.edge_std
      if kind == "line":
        confidence = _finite_number(model.laneLineProbs[index])
        if confidence is None or not settings.min_confidence <= confidence <= 1.0:
          continue
        limit = settings.high_std if confidence >= settings.confidence_split else settings.low_std
      if std is None or not 0.0 <= std < limit:
        continue
      path = _path((model.laneLines if kind == "line" else model.roadEdges)[index])
      if _paths is not None: _paths[side, kind] = path
      y = _at(path, lookahead) if path is not None else None
      if y is not None:
        accepted.append((side, kind, y))
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
      continue
  return accepted


def _bend_target(model, chosen, lookahead, settings, diagnostic=None, _paths=None):
  """Road-boundary bend weight; None means unknown, zero means straight."""
  def reason(value):
    if diagnostic is not None:
      diagnostic["bend_reason"] = value

  reason("no_pair")
  if len(chosen) != 2:
    return None
  fits = []
  lo, hi = max(0., lookahead - 10.), lookahead + 10.
  origin, scale = (lo + hi) * .5, (hi - lo) * .5
  probes = (np.linspace(lo, hi, 9) - origin) / scale
  try:
    for side, kind, _ in chosen:
      path = (_paths[side, kind] if _paths is not None else
              _path((model.laneLines if kind == "line" else model.roadEdges)[side + 1 if kind == "line" else side]))
      reason("path_coverage")
      if path is None or path[0][0] > lo or path[0][-1] < hi:
        return None
      x, y = path
      inside = (x >= lo) & (x <= hi)
      reason("too_few_points")
      if np.count_nonzero(inside) < 4:
        return None
      u = (x[inside] - origin) / scale
      fit = _paths.get("bend_fit", np.polynomial.polynomial.polyfit) if _paths is not None else np.polynomial.polynomial.polyfit
      c = fit(u, y[inside], 3)
      reason("fit_residual")
      if not np.isfinite(c).all() or np.max(np.abs(np.polynomial.polynomial.polyval(u, c) - y[inside])) > .10:
        return None
      at = (lookahead - origin) / scale
      slope = (c[1] + 2*c[2]*at + 3*c[3]*at*at) / scale
      curvature = (2*c[2] + 6*c[3]*at) / (scale*scale) / (1 + slope*slope)**1.5
      reason("invalid_curvature")
      if not math.isfinite(curvature) or abs(curvature) > .1:
        return None
      fits.append((curvature, np.polynomial.polynomial.polyval(probes, c),
                   (c[1] + 2*c[2]*probes + 3*c[3]*probes**2) / scale))
    left, right = fits
    widths = (right[1] - left[1]) / np.hypot(1., (left[2] + right[2]) * .5)
    reason("width_or_curvature_disagreement")
    min_width, max_width = np.min(widths), np.max(widths)
    if diagnostic is not None:
      diagnostic.update(bend_curvatures=[left[0], right[0]], bend_width_range=[float(min_width), float(max_width)])
    if (not np.isfinite(widths).all() or min_width < settings.min_width or
        max_width > settings.max_width or np.ptp(widths) > .6 or
        abs(left[0] - right[0]) > max(.001, .3 * max(abs(left[0]), abs(right[0])))):
      return None
    k = abs((left[0] + right[0]) * .5)
    fraction = clip_scalar((k - 1/settings.curve_start_radius) /
                           (1/settings.curve_full_radius - 1/settings.curve_start_radius), 0., 1.)
    reason("valid")
    return fraction*fraction*(3 - 2*fraction)
  except (AttributeError, IndexError, TypeError, ValueError, OverflowError, FloatingPointError, np.linalg.LinAlgError):
    reason("invalid_fit_input")
    return None


class LaneCenteringController:
  use_road_edges = True
  use_single_boundary = True

  def __init__(self) -> None:
    self._correction = 0.0
    self._limit = STOCK_CORRECTION
    self._clear_boundaries()

  def reset(self) -> None:
    self._correction = 0.0
    self._limit = STOCK_CORRECTION
    self._clear_boundaries()

  def _clear_boundaries(self):
    self.diagnostic = {}
    self._bend_weight = 0.0
    self._preference_weight = 0.0
    self._response_tau = ACQUIRE_TAU
    self._e2e_authority = 1.0
    self._smoothed = [None, None]
    self._sample_id = None
    self._sample_elapsed = 0.0
    self._raw = None
    self.selected_mode = 0
    self.selected_sources = ()

  def _adapt(self, model, settings, chosen, lookahead, _paths=None):
    self.diagnostic["bend_reason"] = "single_boundary_hold" if self.selected_mode == 3 else "disabled"
    if not settings.curve_adaptation:
      self._bend_weight = 0.0
      self._preference_weight = 0.0
    elif self.selected_mode != 3:
      target = _bend_target(model, chosen, lookahead, settings, self.diagnostic, _paths)
      self.diagnostic["bend_target"] = target
      if target is not None:
        self._bend_weight += (target - self._bend_weight) * (1.0 - math.exp(-self._sample_elapsed / .20))
        # Lower preference weight means higher model preference.
        tau = -1.0 / math.log(.05) if target < self._preference_weight else .20
        self._preference_weight += (target - self._preference_weight) * (1.0 - math.exp(-self._sample_elapsed / tau))
    self._response_tau = settings.response_tau + (settings.response_min - settings.response_tau) * self._bend_weight
    self._e2e_authority = settings.e2e_authority + (settings.e2e_min - settings.e2e_authority) * self._preference_weight

  def _frame_correction(self, observation, _paths=None):
    self.diagnostic = {"frame": observation.model_sample_id}
    model, settings, speed = observation.model, observation.settings, observation.speed_mps
    lookahead = clip_scalar(speed * settings.lookahead_time, 8.0, 35.0)
    path = _path(getattr(model, "position", None))
    if _paths is not None: _paths["position"] = path
    model_y = _at(path, lookahead) if path is not None else None
    boundaries = _boundaries(model, lookahead, settings, _paths, include_edges=self.use_road_edges)
    self.diagnostic.update(lookahead_m=lookahead, model_y_m=model_y, accepted=boundaries, geometry="invalid_model_or_crossed")
    left = max((b[2] for b in boundaries if b[0] == 0), default=None)
    right = min((b[2] for b in boundaries if b[0] == 1), default=None)
    if model_y is None or (left is not None and right is not None and right <= left):
      self._smoothed = [None, None]
      self.selected_mode, self.selected_sources = 0, ()
      self._adapt(model, settings, (), lookahead, _paths)
      return None
    lines = [b for b in boundaries if b[1] == "line"]
    wide = len(lines) == 2 and lines[1][2] - lines[0][2] > settings.max_width
    pairs = [(l, r) for l in boundaries if l[0] == 0 for r in boundaries if r[0] == 1
             if settings.min_width <= r[2] - l[2] <= settings.max_width]
    chosen, mode = (), 0
    narrow = left is not None and right is not None and right - left < settings.min_width
    if not wide and not narrow:
      if settings.mode1:
        chosen = next((pair for pair in pairs if all(b[1] == "line" for b in pair)), ())
        mode = 1 if chosen else 0
      if not chosen and settings.mode2:
        mixed = [pair for pair in pairs if pair[0][1] != pair[1][1]]
        chosen = min(mixed, key=lambda pair: pair[1][2] - pair[0][2], default=())
        mode = 2 if chosen else 0
    self.diagnostic["geometry"] = "narrow" if narrow else "wide" if wide else "accepted"
    clearance = settings.car_width * .5 + settings.single_clearance
    if not chosen and self.use_single_boundary and settings.mode3 and boundaries:
      if left is not None and right is not None and left + clearance > right - clearance:
        self.diagnostic["geometry"] = "single_clearance_conflict"
        self._smoothed = [None, None]
        self.selected_mode, self.selected_sources = 0, ()
        self._adapt(model, settings, (), lookahead, _paths)
        return None
      chosen = (max(boundaries, key=lambda b: clearance - ((model_y - b[2]) if b[0] == 0 else (b[2] - model_y))),)
      mode = 3
    self.diagnostic["selected"] = chosen
    self.selected_mode = mode
    self.selected_sources = tuple((b[0], b[1]) for b in chosen)
    self._adapt(model, settings, chosen, lookahead, _paths)
    for side in (0, 1):
      boundary = next((b for b in chosen if b[0] == side), None)
      if boundary is None:
        self._smoothed[side] = None
        continue
      raw, previous = boundary[2], self._smoothed[side]
      if previous is None or settings.smoothing_tau == 0 or settings.smoothing_distance == 0:
        self._smoothed[side] = raw
      else:
        filtered = previous + (raw - previous) * (1.0 - math.exp(-self._sample_elapsed / settings.smoothing_tau))
        self._smoothed[side] = clip_scalar(filtered, raw - settings.smoothing_distance, raw + settings.smoothing_distance)
    self.diagnostic["smoothed_m"] = list(self._smoothed)
    if not chosen:
      self.diagnostic["geometry"] = "no_selection"
      return None
    if mode in (1, 2):
      sl, sr = self._smoothed
      width = sr - sl
      if not settings.min_width <= width <= settings.max_width:
        self.diagnostic["geometry"] = "smoothed_width"
        return None
      safe_offset = min(MAX_OFFSET, max(0.0, width * .5 - MIN_CENTER_TO_LINE))
      error = (sl + sr) * .5 + clip_scalar(settings.offset_m, -safe_offset, safe_offset) - model_y
      raw_width = chosen[1][2] - chosen[0][2]
      raw_offset = min(MAX_OFFSET, max(0.0, raw_width * .5 - MIN_CENTER_TO_LINE))
      self.diagnostic.update(raw_target_m=(chosen[0][2] + chosen[1][2]) * .5 + clip_scalar(settings.offset_m, -raw_offset, raw_offset),
                             smoothed_target_m=error + model_y)
    else:
      side, _, raw = chosen[0]
      away = 1 if side == 0 else -1
      target = self._smoothed[side] + away * clearance
      self.diagnostic.update(raw_target_m=raw + away * clearance, smoothed_target_m=target)
      raw_shortfall = clearance - away * (model_y - raw)
      smoothed_shortfall = away * (target - model_y)
      opposite = right if side == 0 else left
      self.diagnostic.update(raw_shortfall_m=raw_shortfall, smoothed_shortfall_m=smoothed_shortfall)
      if opposite is not None and away * (opposite - target) < clearance:
        self.diagnostic["geometry"] = "smoothed_opposite_conflict"
        return None
      # A clear raw boundary or a clear smoothed boundary requests zero, never attraction.
      if raw_shortfall <= 0 or smoothed_shortfall <= 0:
        self.diagnostic["geometry"] = "single_clear"
        return 0.0
      error = away * smoothed_shortfall
    return _error_correction(model, speed, error, self._e2e_authority, lookahead,
                             settings.single_deadband, settings.override_start, settings.override_full, self.diagnostic, _paths)

  def _result(self, base: float, reason: str) -> LaneCenteringResult:
    correction = self._correction
    return LaneCenteringResult(base + correction, correction, (correction > 0) - (correction < 0), reason, self._limit)

  def _fade(self, base: float, dt: float, reason: str, tau: float) -> LaneCenteringResult:
    self._correction *= math.exp(-dt / tau) if tau else 0.0
    if abs(self._correction) < 1e-12:
      self._correction = 0.0
    return self._result(base, reason)

  def _valid_settings(self, settings):
    return valid_tuning(settings)

  def update(self, observation: LaneCenteringInput) -> LaneCenteringResult:
    if not isinstance(observation, LaneCenteringInput):
      self.reset()
      return LaneCenteringResult(None, 0.0, 0, "invalid_input")
    base = _finite_number(observation.base_curvature)
    speed = _finite_number(observation.speed_mps)
    dt = _finite_number(observation.elapsed_seconds)
    settings = observation.settings
    offset = _finite_number(settings.offset_m) if isinstance(settings, LaneCenteringSettings) else None
    authority = _finite_number(settings.e2e_authority) if isinstance(settings, LaneCenteringSettings) else None
    strength = _finite_number(settings.strength) if isinstance(settings, LaneCenteringSettings) else None
    if (base is None or speed is None or dt is None or dt <= 0 or
        offset is None or authority is None or strength is None or not 0.5 <= strength <= STRENGTH_MAX or type(observation.model_sample_id) is not int or
        observation.model_sample_id < 0 or not isinstance(observation.mode, ControlMode) or
        type(observation.model_valid) is not bool or type(observation.lateral_active) is not bool or
        type(observation.turn_signal_active) is not bool or type(observation.driver_override) is not bool or
        type(observation.time_discontinuity) is not bool or
        type(settings.enabled) is not bool or type(settings.pause_on_signal) is not bool or not self._valid_settings(settings)):
      self.reset()
      return LaneCenteringResult(base, 0.0, 0, "invalid_input")
    self._limit = settings.max_correction
    self._correction = clip_scalar(self._correction, -self._limit, self._limit)
    if observation.time_discontinuity:
      self.reset()
      return self._result(base, "time_discontinuity")
    if not observation.mode.lateral_authority or not settings.enabled or not observation.lateral_active or \
       not observation.model_valid or speed < settings.min_speed_mps:
      self.reset()
      return self._result(base, "inactive")
    if observation.driver_override:
      self.reset()
      return self._result(base, "driver_override")
    if settings.pause_on_signal and observation.turn_signal_active:
      self._clear_boundaries()
      return self._fade(base, dt, "signal_release", settings.release_tau)
    try:
      if observation.model.meta.laneChangeState != log.LaneChangeState.off:
        self.reset()
        return self._result(base, "lane_change")
    except (AttributeError, TypeError, ValueError):
      self.reset()
      return self._result(base, "invalid_model")
    self._sample_elapsed += dt
    if self._sample_id != observation.model_sample_id:
      self._raw = self._frame_correction(observation)
      self._sample_id = observation.model_sample_id
      self._sample_elapsed = 0.0
    raw = self._raw
    if raw is None or not math.isfinite(raw):
      return self._fade(base, dt, "unqualified_model", settings.release_tau)
    target = clip_scalar(raw * 0.30, -self._limit, self._limit)
    if strength != 1.0:
      target = clip_scalar(target * strength, -self._limit, self._limit)
    self._correction += (target - self._correction) * (1.0 - math.exp(-dt / self._response_tau) if self._response_tau else 1.0)
    self._correction = clip_scalar(self._correction, -self._limit, self._limit)
    return self._result(base, "qualified")
