"""SP-016 v6: spatial eligibility, one interval-constrained preview fit."""

import math

import time

from dataclasses import replace

from functools import lru_cache

import numpy as np

import numpy.ma  # Resolve np.unique masked-array support before control updates.

from openpilot.starpilot.lateral.lane_v5 import LaneCenteringV5Controller, LaneCenteringV5Settings, LaneCenteringV5Input

from openpilot.starpilot.lateral.lane_centering import _path, _path_xy, _boundaries, MAX_OFFSET, MIN_CENTER_TO_LINE

from openpilot.starpilot.lateral.lane_motion import warp_profile

from openpilot.starpilot.lateral.lane_path_v6 import build_path, gain_normalization, CubicFit, _free_fit, _unit_shape, _interval_support, _local_basis

from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan



LaneCenteringV6Settings = LaneCenteringV5Settings

LaneCenteringV6Input = LaneCenteringV5Input

SOURCES = ((0, "line"), (1, "line"), (0, "edge"), (1, "edge"))

@lru_cache(maxsize=1)

def _warm_matrices():

  # Setup finishes during controller construction, never the first control tick.

  for count in range(21, 82):

    horizon = (count-1)/20

    for local in (False, True):

      for near in (False, True):

        _free_fit(horizon, count, near, local); _unit_shape(horizon, count, near, local)

      _interval_support(horizon, count, local)





def select_sections(raw, model_y, settings):

  """Raw selection only. IDs are stable source identities; -1 is unselected."""

  n = len(model_y)

  nearest = np.array([np.maximum(raw[0], raw[2]), np.minimum(raw[1], raw[3])])

  gap = nearest[1]-nearest[0]

  mode = np.zeros(n, dtype=np.int8); ids = np.full((2, n), -1, dtype=int)

  wide = raw[1]-raw[0] > settings.max_width

  line_pair = np.isfinite(raw[0]) & np.isfinite(raw[1])

  pair_ok = (gap >= settings.min_width) & ~(wide & line_pair)

  best = np.full(n, np.inf)

  for left, right, kind, enabled in ((0, 1, 1, settings.mode1), (0, 3, 2, settings.mode2), (2, 1, 2, settings.mode2)):

    width = raw[right]-raw[left]

    valid = (enabled & pair_ok & np.isfinite(width) & (width >= settings.min_width) & (width <= settings.max_width))

    take = valid & ((mode == 0) | ((mode == 2) & (kind == 2) & (width < best)))

    ids[0, take], ids[1, take], mode[take], best[take] = left, right, kind, width[take]

    if kind == 1 and np.all(mode == 1): return mode, ids, nearest

  if np.all(mode > 0): return mode, ids, nearest

  distance = settings.car_width/2 + settings.single_clearance

  shortfall = np.array([distance-(model_y-raw[i]) if side == 0 else distance-(raw[i]-model_y) for i, (side, _) in enumerate(SOURCES)])

  choice = np.argmax(shortfall, axis=0)  # fixed source order resolves ties

  has = np.any(np.isfinite(raw), axis=0)

  single = (mode == 0) & settings.mode3 & has & (gap >= 2*distance)

  for i, (side, _) in enumerate(SOURCES):

    take = single & (choice == i)

    ids[side, take], mode[take] = i, 3

  return mode, ids, nearest





class LaneCenteringV6Controller(LaneCenteringV5Controller):
  use_road_edges = True  # V6 retains road-edge support.
  use_single_boundary = True



  def __init__(self, *, clock=time.perf_counter):

    _warm_matrices()

    super().__init__(clock=clock)

  def _clear_boundaries(self):

    super()._clear_boundaries()

    self._profile_masks = [None, None]

    self.planned_path = None
    self._bend_fit = CubicFit()  # Projection only; cleared with boundary/reset history.



  def update(self, observation):

    result = super().update(observation)

    self.diagnostic["algorithm"] = 6

    return result



  def _frame_correction(self, obs):

    start, cpu = self.clock(), time.thread_time()

    phases = {}

    self.diagnostic = {"frame": obs.model_sample_id, "algorithm": 6}

    self.target_path = self.target_z = self.planned_path = None

    dx, dy, yaw = self._motion

    aligned = self._motion_ok

    self._motion[:] = 0.; self._motion_ok = True

    self.selected_mode, self.selected_sources = 0, ()



    def reject(reason, clear=True):

      if clear:

        self._profiles = [None, None]; self._profile_masks = [None, None]

      self.target_path = self.target_z = None

      self.diagnostic["v5_reason"] = "v6_"+reason

      return None



    try:

      model, s, motion = obs.model, obs.settings, obs.motion

      if not aligned or motion is None or not motion.valid() or obs.speed_mps < 1.:

        return reject("motion_unavailable")

      path = _path(model.position)

      if path is None: return reject("model_path")

      mt, mz = np.asarray(model.position.t, dtype=float), np.asarray(model.position.z, dtype=float)

      if (mt.shape != path[0].shape or mz.shape != mt.shape or not np.isfinite(mt).all() or

          not np.isfinite(mz).all() or mt[0] != 0. or not np.all(np.diff(mt) > 0)):

        return reject("time_geometry")

      paths = {"position": path, "bend_fit": self._bend_fit}

      _boundaries(model, float(path[0][0]), s, paths)

      # The helper caches every quality-accepted path before its point-coverage

      # test. Keep those paths: a near-starting edge must reject support, not

      # silently disappear from nearest-boundary/clearance checks.

      curves = {key: paths[key] for key in SOURCES if paths.get(key) is not None}

      accepted = [(side, kind, float(curve[1][0])) for (side, kind), curve in curves.items()]

      if not curves: return reject("no_boundaries")

      end = min(60., path[0][-1], *(curve[0][-1] for curve in curves.values()))

      count = math.floor(min(s.preview_time, float(np.interp(end, path[0], mt)))*20)+1

      if count < 21: return reject("short_preview")

      times = np.arange(count)/20

      x, py = np.interp(times, mt, path[0]), np.interp(times, mt, path[1])

      if any(x[0] < curve[0][0] for curve in curves.values()): return reject("near_support")

      raw = np.array([np.interp(x, *curves[key]) if key in curves else np.full(count, -np.inf if key[0] == 0 else np.inf) for key in SOURCES])

      mode, ids, nearest = select_sections(raw, py, s)

      # Widths are piecewise linear in x: inspect original boundary knots too.

      # A failed/intermediate selection cannot be hidden between uniform samples.

      grid = next(iter(curves.values()))[0]

      if all(np.array_equal(grid, curve[0]) for curve in curves.values()):

        inside = (grid >= x[0]) & (grid <= x[-1])

        knots = grid[inside]

        knot_raw = np.array([curves[key][1][inside] if key in curves else

          np.full(len(knots), -np.inf if key[0] == 0 else np.inf) for key in SOURCES])

      else:

        knots = np.unique(np.concatenate([curve[0][(curve[0] >= x[0]) & (curve[0] <= x[-1])] for curve in curves.values()]))

        knot_raw = np.array([np.interp(knots, *curves[key]) if key in curves else np.full(len(knots), -np.inf if key[0] == 0 else np.inf) for key in SOURCES])

      kmode, kids, knearest = select_sections(knot_raw, np.interp(knots, *path), s)

      hi = np.clip(np.searchsorted(x, knots), 1, count-1); lo = hi-1

      left_match = (kmode == mode[lo]) & np.all(kids == ids[:, lo], axis=0)

      right_match = (kmode == mode[hi]) & np.all(kids == ids[:, hi], axis=0)

      different = (kmode == 0) | ~(left_match | right_match)

      forbidden = np.zeros(count, dtype=bool)

      forbidden[lo[different]] = True; forbidden[hi[different]] = True

      crossed = np.flatnonzero(knearest[1] <= knearest[0])

      if len(crossed): forbidden[lo[crossed[0]]:] = True

      mode[forbidden] = 0; ids[:, forbidden] = -1

      phases["selection_cpu_s"] = time.thread_time()-cpu

      smooth = np.tile(py, (2, 1))

      selected = ids >= 0

      for side in (0, 1):

        used = selected[side]

        smooth[side, used] = raw[ids[side, used], np.flatnonzero(used)]

        previous = self._profiles[side]

        if previous is not None and s.smoothing_tau and s.smoothing_distance:

          warped = warp_profile(*previous, dx, dy, yaw, x)

          if warped is not None:

            overlap = warped[1] & used

            if not self._profile_masks[side].all():

              xx = math.cos(yaw)*(previous[0]-dx)+math.sin(yaw)*(previous[1]-dy)

              overlap &= np.interp(x, xx, self._profile_masks[side].astype(float)) > 1.-1e-10

            detected = smooth[side, overlap].copy()

            smooth[side, overlap] = warped[0][overlap]+(detected-warped[0][overlap])*(-math.expm1(-self._sample_elapsed/s.smoothing_tau))

            smooth[side, overlap] = np.clip(smooth[side, overlap], detected-s.smoothing_distance, detected+s.smoothing_distance)

        self._profiles[side] = (x.copy(), smooth[side].copy()) if used.any() else None

        self._profile_masks[side] = used.copy() if used.any() else None

      # Adapt only from the real pair near the existing bend evaluation distance.

      lookahead = float(np.clip(obs.speed_mps, 8., 35.))

      at = min(count-1, int(np.searchsorted(x, lookahead)))

      self.selected_mode = int(mode[at]) if lookahead <= x[-1] else 0

      chosen = tuple((side, SOURCES[ids[side, at]][1], float(raw[ids[side, at], at])) for side in (0, 1) if self.selected_mode and ids[side, at] >= 0)

      self._adapt(model, s, chosen, lookahead, paths)

      self.selected_sources = tuple((side, kind) for side, kind, _ in chosen)

      paired = (mode == 1) | (mode == 2)

      width = smooth[1]-smooth[0]

      valid = paired & (width >= s.min_width) & (width <= s.max_width)

      error = np.zeros(count); distance = np.full(count, s.car_width/2+s.clearance)

      bound = np.minimum(MAX_OFFSET, np.maximum(0., width/2-MIN_CENTER_TO_LINE))

      error[valid] = ((smooth[0]+smooth[1])/2+np.clip(s.offset_m, -bound, bound)-py)[valid]

      regions = np.where(valid, 2, 0).astype(np.int8)

      for side, away in ((0, 1), (1, -1)):

        single = (mode == 3) & selected[side]

        ii = np.flatnonzero(single)

        if not len(ii): continue

        clearance = s.car_width/2+s.single_clearance

        distance[ii] = clearance

        raw_short = clearance-away*(py[ii]-raw[ids[side, ii], ii])

        short = clearance-away*(py[ii]-smooth[side, ii])

        target = smooth[side, ii]+away*clearance

        opposite_ok = away*(nearest[1-side, ii]-target) >= clearance

        valid[ii] = opposite_ok

        ok = (raw_short > 0) & (short > 0) & opposite_ok

        regions[ii[opposite_ok]] = away

        jj = ii[ok]; error[jj] = away*short[ok]

        near = jj[times[jj] <= 1.]

        if len(near): error[near] = away*np.max(away*error[near])

      std = _path_xy(model.position.x, model.position.yStd)

      attenuation = np.ones(count)

      if std is not None:

        ps = np.interp(x, *std); confident = (ps >= 0.) & (ps <= .35)

        attenuation[confident] -= self._e2e_authority*np.clip((np.abs(error[confident])-s.override_start)/(s.override_full-s.override_start), 0., 1.)

      # Do not grow a sharp re-entry from a short isolated or terminal island.

      eligible = regions != 0

      starts = np.flatnonzero(eligible & ~np.concatenate((np.array([False]), eligible[:-1])))

      ends = np.flatnonzero(eligible & ~np.concatenate((eligible[1:], np.array([False]))))

      for first, last in zip(starts, ends):

        if first and times[last]-times[first] < 1.:

          regions[first:last+1] = 0





      priority = bool(np.any((np.abs(regions) == 1) & (times <= 1.)))

      norm = gain_normalization(float(times[-1]), count, obs.speed_mps, motion.delay, get_curvature_from_plan, priority, _local_basis(float(times[-1]), regions, priority))

      desired = norm*.30*s.strength*attenuation*np.sign(error)*np.maximum(np.abs(error)-s.single_deadband, 0.)

      lower, upper = np.minimum(0., nearest[0]+distance-py), np.maximum(0., nearest[1]-distance-py)

      lower = np.maximum(lower, -100.); upper = np.minimum(upper, 100.)

      lower[regions == 1] = 0.; upper[regions == -1] = 0.

      # Preserve v5 sampled model-yield/clearance bounds on valid geometry.

      # Only invalid geometry gets continuous zero-offset interval constraints.

      sampled_zero = (attenuation < 1e-9) | ((mode == 3) & (error == 0.))

      lower[sampled_zero] = upper[sampled_zero] = 0.

      lower[regions == 0] = upper[regions == 0] = 0.

      phases["pre_fit_cpu_s"] = time.thread_time()-cpu

      target = build_path(frame=obs.model_sample_id, t=times, x=x, model_y=py, desired_offset=desired,

        lower=lower, upper=upper, speed=obs.speed_mps, action_delay=motion.delay, base_curvature=obs.base_curvature,

        cap=s.max_correction*min(s.strength, 1.), converter=get_curvature_from_plan,

        away=int(regions[0]) if np.all(regions == regions[0]) and abs(regions[0]) == 1 else 0,

        near_priority=priority, regions=regions)

      self.planned_path = target

      codes = np.vstack((mode, regions, ids))

      cuts = np.concatenate((np.array([0], dtype=np.intp),
        np.flatnonzero(np.any(codes[:, 1:] != codes[:, :-1], axis=0))+1, np.array([count], dtype=np.intp)))

      sections = [[float(times[a]), float(times[b-1]), *codes[:, a].tolist()] for a, b in zip(cuts[:-1], cuts[1:])]



      action_end = min(count-1, int(np.searchsorted(times, max(1., motion.delay))))

      current = regions[:action_end+1]

      self.diagnostic.update(v6_sections=sections, v6_modes=mode.tolist(), v6_eligible=valid.tolist(), v6_regions=regions.tolist(), v6_sources=ids.tolist(),

        v6_cap_peak_s=float(times[int(np.argmax(np.abs(target.curvature_delta)))]),

        v6_paired_opposes_near=bool(np.all(mode[:action_end+1] < 3) and np.any(mode[action_end+1:] == 3)

          and abs(float(np.mean(error[:action_end+1]))) > s.single_deadband and target.correction*float(np.mean(error[:action_end+1])) < 0),

        v6_fit_iterations=target.fit_iterations, v6_horizon_s=float(times[-1]), v6_points=count,

        v6_target_correction=target.correction, v6_cap_scale=target.cap_scale,

        accepted=accepted, selected=chosen, geometry="preview_sections")

      # Reject completed results over 30 ms; 10 ms remains the performance goal.
      if self.clock()-start > .030: return reject("compute_budget")

      if not np.all(valid[:action_end+1]): return reject("model_section", clear=False)

      signs = current[np.abs(current) == 1]

      if (np.any(signs == 1) and np.any(signs == -1)) or np.any(signs*target.correction < -1e-10): return reject("single_direction")

      self.target_path, self.target_z = target, np.interp(times, mt, mz)

      self.diagnostic["v5_reason"] = "qualified"

      return target.correction/(.30*s.strength)

    except (AttributeError, ValueError, TypeError, IndexError, OverflowError, FloatingPointError, ZeroDivisionError, np.linalg.LinAlgError):

      return reject("invalid_path")

    finally:

      self.diagnostic.update(phases, v6_compute_s=self.clock()-start, v6_cpu_s=time.thread_time()-cpu)

