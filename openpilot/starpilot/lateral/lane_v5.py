"""SP-016 v5 explicit target path. V4 owns authority, output filtering and resets."""
from dataclasses import dataclass, replace
from functools import lru_cache
import math
import time
import numpy as np
import numpy.polynomial.polynomial  # Resolve lazy imports before control updates.
from openpilot.starpilot.lateral.lane_centering import (LaneCenteringController, LaneCenteringInput,
  LaneCenteringSettings, LaneCenteringResult, _finite_number, _path, _path_xy, _boundaries, MAX_OFFSET, MIN_CENTER_TO_LINE, STRENGTH_MAX)
from openpilot.starpilot.lateral.lane_motion import Motion, warp_profile
from openpilot.starpilot.lateral.lane_path import build_path, gain_normalization, CubicFit, _unit_shape, _free_fit
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan

@lru_cache(maxsize=1)
def _warm_matrices():
  # Controller construction runs before control ticks; settings-only UI imports
  # need no numerical setup. Fractional v5 horizons retain their usual fallback.
  for horizon in (1., 2., 3., 4.):
    for near in (False, True):
      _unit_shape(horizon, round(horizon*20)+1, near)
      _free_fit(horizon, round(horizon*20)+1, near)


@dataclass(frozen=True)
class LaneCenteringV5Settings(LaneCenteringSettings):
  preview_time: float = 3.
  # Used only by v5.1; v5/v6 retain their original reference calculation.
  recovery_time: float = 3.
  recovery_min: float = 2.


@dataclass(frozen=True)
class LaneCenteringV5Input(LaneCenteringInput):
  motion: Motion | None = None


class LaneCenteringV5Controller(LaneCenteringController):
  use_road_edges = False
  use_single_boundary = False

  def __init__(self, *, clock=time.perf_counter):
    _warm_matrices()
    self.clock = clock
    self._validated_settings = None
    super().__init__()

  def _valid_settings(self, settings):
    # Live settings are frozen value objects, replaced by the existing refresh.
    # Cache only this exact immutable class; malformed/subclass inputs still validate.
    if type(settings) is LaneCenteringV5Settings and settings is self._validated_settings:
      return True
    valid = super()._valid_settings(settings)
    self._validated_settings = settings if valid and type(settings) is LaneCenteringV5Settings else None
    return valid

  def _clear_boundaries(self):
    super()._clear_boundaries()
    self._profiles = [None, None]
    self.target_path = None
    self.target_z = None
    self._motion = np.zeros(3)
    self._motion_ok = True
    self._motion_ns = None

  def update(self, observation):
    if not isinstance(observation, LaneCenteringV5Input) or not isinstance(observation.settings, LaneCenteringV5Settings):
      self.reset()
      return LaneCenteringResult(None, 0., 0, "invalid_v5_input")
    if (_finite_number(observation.settings.preview_time) is None or
        not 1. <= observation.settings.preview_time <= 4.):
      self.reset()
      return LaneCenteringResult(_finite_number(observation.base_curvature), 0., 0, "invalid_v5_settings")
    motion = observation.motion
    valid = isinstance(motion, Motion) and motion.valid()
    dt = observation.elapsed_seconds
    if valid and self._motion_ns is not None:
      dt = (motion.timestamp_ns - self._motion_ns) / 1e9
      valid = 0. < dt <= .05
    self._motion_ns = motion.timestamp_ns if isinstance(motion, Motion) and type(motion.timestamp_ns) is int and motion.timestamp_ns > 0 else None
    self._motion_ok &= valid
    if not valid:
      # A bad motion tick cannot keep building on a formerly valid request.
      self._raw = None
      self.target_path = None
      self.target_z = None
      self.diagnostic["v5_reason"] = "v5_motion_unqualified"
    if valid and _finite_number(observation.speed_mps) is not None and _finite_number(dt) is not None and 0 < dt <= .05:
      yaw = observation.speed_mps * motion.measured_curvature * dt
      heading = self._motion[2] + yaw / 2
      self._motion += (observation.speed_mps * math.cos(heading) * dt,
                       observation.speed_mps * math.sin(heading) * dt, yaw)
    # Parent keeps all authority/reset checks and identical output filtering/caps.
    result = super().update(observation)
    self.diagnostic["algorithm"] = 5
    if result.reason != "qualified":
      self.target_path = None
      self.target_z = None
    if result.reason == "unqualified_model":
      result = replace(result, reason=self.diagnostic.get("v5_reason", result.reason))
    return result

  def _frame_correction(self, obs):
    start = self.clock()
    dx, dy, yaw = self._motion
    aligned = self._motion_ok
    self._motion[:] = 0.
    self._motion_ok = True
    self.target_path = self.target_z = None
    selection = replace(obs, settings=replace(obs.settings, lookahead_time=1.))
    paths = {"bend_fit": CubicFit()}  # Frame-local geometry and fit reuse only.
    raw = super()._frame_correction(selection, paths)

    def reject(reason):
      self._profiles = [None, None]
      self.target_path = self.target_z = None
      self.diagnostic["v5_reason"] = reason
      return None

    if raw is None: return reject("v5_geometry_unavailable")
    motion, settings, model = obs.motion, obs.settings, obs.model
    if not aligned or motion is None or not motion.valid() or obs.speed_mps < 1.:
      return reject("v5_motion_unavailable")
    try:
      path = paths["position"]
      mt = np.asarray(model.position.t, dtype=float)
      mz = np.asarray(model.position.z, dtype=float)
      if (path is None or mt.shape != path[0].shape or mz.shape != mt.shape or
          not np.isfinite(mt).all() or not np.isfinite(mz).all() or mt[0] != 0. or not np.all(np.diff(mt) > 0)):
        return reject("v5_time_geometry")
      accepted = self.diagnostic["accepted"]  # Already validated by the near selection above.
      curves = {(side, kind): paths[side, kind]
                for side, kind, _ in accepted}
      if any(curve is None for curve in curves.values()): return reject("v5_boundary_path")
      max_x = min(60., path[0][-1], *(curve[0][-1] for curve in curves.values()))
      horizon = min(settings.preview_time, float(np.interp(max_x, path[0], mt)))
      horizon = math.floor(horizon*20)/20
      if horizon < 1.: return reject("v5_short_preview")
      times = np.linspace(0., horizon, round(horizon*20)+1)
      stations = np.interp(times, mt, path[0]); py = np.interp(times, mt, path[1])
      if any(stations[0] < curve[0][0] for curve in curves.values()): return reject("v5_near_support")
      raw_sides, smooth_sides = {}, {}
      for side, kind in self.selected_sources:
        curve = curves[side, kind]
        detected = np.interp(stations, *curve); filtered = detected.copy()
        previous = self._profiles[side]
        if previous is not None and settings.smoothing_tau and settings.smoothing_distance:
          warped = warp_profile(*previous, dx, dy, yaw, stations)
          if warped is not None:
            old, overlapping = warped
            filtered[overlapping] = old[overlapping] + (detected[overlapping]-old[overlapping]) * (1-math.exp(-self._sample_elapsed/settings.smoothing_tau))
            filtered = np.clip(filtered, detected-settings.smoothing_distance, detected+settings.smoothing_distance)
        raw_sides[side], smooth_sides[side] = detected, filtered
      nearest = {}
      for side in (0, 1):
        samples = [np.interp(stations, *curve) for (bside, _), curve in curves.items() if bside == side]
        nearest[side] = (np.max(samples, axis=0) if side == 0 else np.min(samples, axis=0)) if samples else np.full(len(times), -100. if side == 0 else 100.)
      valid = np.ones(len(times), dtype=bool); away = 0
      if self.selected_mode in (1, 2):
        width = raw_sides[1]-raw_sides[0]; swidth = smooth_sides[1]-smooth_sides[0]
        valid &= (width >= settings.min_width) & (width <= settings.max_width)
        valid &= (swidth >= settings.min_width) & (swidth <= settings.max_width)
        valid &= nearest[1]-nearest[0] >= settings.min_width
        bound = np.minimum(MAX_OFFSET, np.maximum(0., swidth/2-MIN_CENTER_TO_LINE))
        error = (smooth_sides[0]+smooth_sides[1])/2+np.clip(settings.offset_m, -bound, bound)-py
        distance = settings.car_width/2+settings.clearance
      else:
        side = self.selected_sources[0][0]; away = 1 if side == 0 else -1
        distance = settings.car_width/2+settings.single_clearance
        shortfall = distance-away*(py-smooth_sides[side])
        raw_shortfall = distance-away*(py-raw_sides[side])
        target = smooth_sides[side]+away*distance
        valid &= nearest[1]-nearest[0] >= 2*distance
        valid &= away*(nearest[1-side]-target) >= distance
        error = away*np.where((shortfall > 0) & (raw_shortfall > 0), shortfall, 0.)
      bad = np.flatnonzero(~valid)
      n = int(bad[0]) if len(bad) else len(times)
      if n < 21: return reject("v5_near_geometry")
      times, stations, py, error = [a[:n] for a in (times, stations, py, error)]
      # Truncation preserves a uniform grid; no extrapolation past rejected geometry.
      for side in (0, 1):
        self._profiles[side] = (stations.copy(), smooth_sides[side][:n].copy()) if side in smooth_sides else None
      if away:
        # Anticipate the largest already-accepted shortfall in the next second.
        # Keep clear stations at zero; raw/smoothed and model-yield gates still apply.
        near = times <= 1.
        urgent = float(np.max(away*error[near]))
        error[near & (away*error > 0.)] = away*urgent
      attenuation = np.ones(n)
      std = paths.get("std")
      if "std" not in paths: std = _path_xy(model.position.x, model.position.yStd)
      if std is not None:
        path_std = np.interp(stations, *std)
        confident = (path_std >= 0.) & (path_std <= .35)
        attenuation[confident] -= self._e2e_authority*np.clip((np.abs(error[confident])-settings.override_start)/(settings.override_full-settings.override_start), 0., 1.)
      desired, normalization = self._desired_profile(error, attenuation, times, obs, away)
      lower = np.minimum(0., nearest[0][:n]+distance-py)
      upper = np.maximum(0., nearest[1][:n]-distance-py)
      lower[attenuation < 1e-9] = upper[attenuation < 1e-9] = 0.
      if away:
        lower[error == 0] = upper[error == 0] = 0.
      target = build_path(frame=obs.model_sample_id, t=times, x=stations, model_y=py,
          desired_offset=desired, lower=lower, upper=upper, speed=obs.speed_mps,
          action_delay=motion.delay, base_curvature=obs.base_curvature,
          cap=settings.max_correction*min(settings.strength, 1.), converter=get_curvature_from_plan, away=away, near_priority=bool(away))
      elapsed = self.clock()-start
      self.diagnostic.update(v5_compute_s=elapsed, v5_points=n, v5_gain_normalization=normalization,
          v5_cap_scale=target.cap_scale, v5_horizon_s=float(times[-1]), v5_target_correction=target.correction)
      if elapsed > .030: return reject("v5_compute_budget")
      self.target_path = target
      self.target_z = np.interp(times, mt, mz)
      # Parent retains response/release, cap and handover semantics. No second gain.
      return target.correction/(.30*settings.strength)
    except (AttributeError, ValueError, TypeError, IndexError, OverflowError, FloatingPointError, ZeroDivisionError, np.linalg.LinAlgError):
      return reject("v5_invalid_path")

  def _desired_profile(self, error, attenuation, times, obs, away):
    settings = obs.settings
    normalization = gain_normalization(float(times[-1]), len(times), obs.speed_mps, obs.motion.delay, get_curvature_from_plan, near_priority=bool(away))
    return normalization*.30*settings.strength*attenuation*np.sign(error)*np.maximum(np.abs(error)-settings.single_deadband, 0.), normalization


class LaneCenteringV51Controller(LaneCenteringV5Controller):
  """Gradual approach to the full paired centre; v5 limits and filters remain.

  Strength is fixed at normal solely for the inherited scalar bridge.
  It never scales the paired target; production v5/v5.1 require two lane lines.
  A full reference is not a promise that the bounded fitted path reaches it.
  Road edges are excluded from selection, constraints and bend evidence.
  """
  use_road_edges = False

  def __init__(self, *, clock=time.perf_counter):
    super().__init__(clock=clock)
    self._strength_source = self._normal_settings = None

  def _valid_settings(self, settings):
    normal = _finite_number(getattr(settings, "recovery_time", None))
    bend = _finite_number(getattr(settings, "recovery_min", None))
    return (normal is not None and bend is not None and 1. <= bend <= normal <= 4.
            and super()._valid_settings(settings))

  def update(self, observation):
    if (type(observation) is LaneCenteringV5Input and
        type(observation.settings) is LaneCenteringV5Settings and observation.settings.strength != 1.5):
      settings = observation.settings
      strength = _finite_number(settings.strength)
      if (settings is not self._strength_source and strength is not None and
          .5 <= strength <= STRENGTH_MAX and super()._valid_settings(settings)):
        self._strength_source = settings
        self._normal_settings = replace(settings, strength=1.5)
      if settings is self._strength_source:
        observation = replace(observation, settings=self._normal_settings)
    result = super().update(observation)
    self.diagnostic["algorithm"] = 51
    return result

  def _desired_profile(self, error, attenuation, times, obs, away):
    if away:
      return super()._desired_profile(error, attenuation, times, obs, away)
    # Approach the full midpoint + Lane offset, rather than a gain-reduced centre.
    # Do not compress recovery when geometry shortens the available preview.
    settings = obs.settings
    recovery = settings.recovery_time + (settings.recovery_min-settings.recovery_time)*self._bend_weight
    u = np.clip(times/recovery, 0., 1.)
    approach = u*u*u*(10. + u*(-15. + 6.*u))
    return attenuation*error*approach, 1.
