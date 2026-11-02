"""SP-016 v6 interval-constrained lane target and original-model curvature bridge. No I/O."""
from openpilot.starpilot.lateral.lane_path import TargetPath, CubicFit, _basis, _fit
from functools import lru_cache
import math
import numpy as np

# Fixed near support; the remaining knots extend smoothly with the preview.
_NEAR_KNOTS = (.25, .5, .75, 1., 1.25, 1.5)


def _local_basis(horizon, regions, near_priority=False):
  # Keep short previews and single-boundary near response on the original fit.
  return not near_priority and horizon > 2.5 and regions is not None and bool(np.any(regions == 0))


@lru_cache(maxsize=256)
def _knots(horizon, local=False):
  if not local or horizon <= 2.5:
    interior = np.linspace(0., horizon, 11)[1:-1]
  else:
    # Nine interior knots, matching the original grid exactly at the 2.5 s gate.
    interior = np.r_[_NEAR_KNOTS, 1.5+(horizon-1.5)*np.array((.25, .5, .75))]
  knots = np.r_[np.zeros(4), interior, np.full(4, horizon)]
  knots.setflags(write=False)
  return knots


@lru_cache(maxsize=256)
def _matrices(horizon, count, near_priority=False, local=False):
  t = np.linspace(0., horizon, count)
  knots = _knots(horizon, local)
  # Left endpoint limit at the final knot; retain derivatives there.
  at = np.minimum(t, np.nextafter(horizon, 0.))
  # First three coefficients fixed zero => offset/heading/curvature start at zero.
  basis_cache = {}
  b, d, dd, ddd = [_basis(at, knots, 3, k, basis_cache)[:, 3:] for k in range(4)]
  weight = np.where(t <= 1., 64. if near_priority else 4., 1.)
  h = b.T@(weight[:, None]*b) + .02*dd.T@dd + .002*ddd.T@ddd + np.eye(b.shape[1])*1e-8
  for a in (t, b, d, dd, weight, h): a.setflags(write=False)
  return t, b, d, dd, weight, h


@lru_cache(maxsize=256)
def _free_fit(horizon, count, near_priority, local=False):
  _, b, _, _, weight, h = _matrices(horizon, count, near_priority, local)
  projection = np.linalg.solve(h, b.T*weight)
  projection.setflags(write=False)
  return projection



@lru_cache(maxsize=256)
def _interval_support(horizon, count, local=False):
  t = np.arange(count)/20
  if t[-1] != horizon:
    t = np.linspace(0., horizon, count)
  knots = _knots(horizon, local)
  width = len(knots)-7
  starts, ends = knots[3:3+width], knots[7:7+width]
  overlap = (starts[:, None] < t[1:]) & (ends[:, None] > t[:-1])
  t.setflags(write=False); overlap.setflags(write=False)
  return t, overlap


def _gradient_uniform(a, step):
  # np.gradient(..., edge_order=2) for the validated one-dimensional grid.
  result = np.empty_like(a)
  result[1:-1] = (a[2:]-a[:-2])/(2.*step)
  result[0] = (-1.5/step)*a[0]+(2./step)*a[1]+(-.5/step)*a[2]
  result[-1] = (.5/step)*a[-3]+(-2./step)*a[-2]+(1.5/step)*a[-1]
  return result


def build_path(*, frame, t, x, model_y, desired_offset, lower, upper,
               speed, action_delay, base_curvature, cap, converter, away=0, near_priority=False, regions=None):
  """Fit one supported frame. Bounds are offsets and must include zero.

  Input is already accepted geometry and attenuated spatial target. This function
  does not implement confidence selection, temporal filtering or host authority.
  Returns the steering target, NOT predicted actual vehicle motion.
  """
  arrays = [np.asarray(v, dtype=float) for v in (t, x, model_y, desired_offset, lower, upper)]
  t, x, model_y, desired, lower, upper = arrays
  if (any(v.shape != t.shape or v.ndim != 1 or not np.isfinite(v).all() for v in arrays)
      or len(t) < 21 or len(t) > 401 or t[0] != 0. or not np.all(np.diff(t) > 0)
      or not np.all(np.diff(x) > 0) or not 1. <= t[-1] <= 4.
      or np.max(np.abs(t-np.linspace(0., t[-1], len(t)))) > 1e-8
      or not all(math.isfinite(v) for v in (speed, action_delay, base_curvature, cap))
      or type(frame) is not int or frame < 0
      or speed < 1. or not 0. < action_delay <= min(t[-1], 1.5) or not 0. < cap <= .01
      or np.any(lower > 0.) or np.any(upper < 0.) or away not in (-1, 0, 1)):
    raise ValueError("invalid path inputs")
  if away == 1: lower = np.maximum(lower, 0.)
  if away == -1: upper = np.minimum(upper, 0.)
  if regions is not None:
    regions = np.asarray(regions)
    if regions.shape != t.shape or not (((regions == -1) | (regions == 0) | (regions == 1) | (regions == 2)).all()
        if regions.dtype.kind in "biu" else np.isin(regions, (-1, 0, 1, 2)).all()):
      raise ValueError("invalid spatial regions")
  # Preserve the established fit on fully valid previews, including model yield.
  local = _local_basis(float(t[-1]), regions, near_priority)
  _, b, d, dd, weight, h = _matrices(float(t[-1]), len(t), near_priority, local)
  zero = np.zeros(b.shape[1], dtype=bool)
  signs = np.zeros(b.shape[1])
  if regions is not None:
    if not np.all(regions == 2):
      # A coefficient affects its entire B-spline support. Constrain supports,
      # not only samples: zero spans and away-only signs then hold between samples.
      # A fully paired preview has no sign/zero support restrictions.
      canonical_t, overlap = _interval_support(float(t[-1]), len(t), local)
      if not np.array_equal(t, canonical_t):
        knots = _knots(float(t[-1]), local)
        overlap = (knots[3:3+b.shape[1], None] < t[None, 1:]) & (knots[7:7+b.shape[1], None] > t[None, :-1])
      suppressed = (regions[:-1] == 0) | (regions[1:] == 0)
      positive = (regions[:-1] == 1) | (regions[1:] == 1)
      negative = (regions[:-1] == -1) | (regions[1:] == -1)
      pos, neg = np.any(overlap & positive, axis=1), np.any(overlap & negative, axis=1)
      zero = np.any(overlap & suppressed, axis=1) | (pos & neg)
      signs = pos.astype(float)-neg.astype(float)
  clipped = np.clip(desired, lower, upper)
  c = _free_fit(float(t[-1]), len(t), near_priority, local)@clipped
  offset = b@c
  if (np.any(offset < lower) or np.any(offset > upper) or
      (away and np.any(away*c < 0.)) or np.any(np.abs(c[zero]) > 1e-12) or np.any(signs*c < 0.)):
    # Fully suppressed spans are equalities. Solve in their nullspace, rather
    # than cycling between dependent opposing inequalities at zero width.
    equality = np.r_[b[np.abs(upper-lower) < 1e-12], np.eye(b.shape[1])[zero]]
    null = np.eye(b.shape[1])
    if len(equality):
      _, singular, vh = np.linalg.svd(equality, full_matrices=True)
      rank = int(np.sum(singular > 1e-9))
      null = vh[rank:].T
    if null.shape[1]:
      if away:
        # Nonnegative B-spline basis: coefficient signs guarantee away-only
        # displacement between sample stations too, without redundant zero rows.
        a = np.r_[away*b, -away*np.eye(b.shape[1])]@null
        bound = np.r_[upper if away == 1 else -lower, np.zeros(b.shape[1])]
      else:
        a = np.r_[b, -b]@null; bound = np.r_[upper, -lower]
      if regions is not None and not away:
        ids = np.flatnonzero((signs != 0) & ~zero)
        a = np.r_[a, (-signs[ids, None]*np.eye(b.shape[1])[ids])@null]
        bound = np.r_[bound, np.zeros(len(ids))]
      keep = np.max(np.abs(a), axis=1) > 1e-9
      reduced, iterations = _fit(null.T@h@null, null.T@b.T@(weight*clipped), a[keep], bound[keep])
      c = null@reduced
    else:
      c = np.zeros(b.shape[1]); iterations = 0
  else:
    iterations = 0
  if np.any(np.abs(c[zero]) > 1e-8) or np.any(signs*c < -1e-8):
    raise ValueError("spatial interval constraint failed")
  fitted_offset = b@c
  if np.any(fitted_offset < lower-1e-7) or np.any(fitted_offset > upper+1e-7):
    raise ValueError("path bounds failed")
  step = t[-1]/(len(t)-1)  # Uniform input grid validated above.
  vx, vy = _gradient_uniform(x, step), _gradient_uniform(model_y, step)
  ax, ay = _gradient_uniform(vx, step), _gradient_uniform(vy, step)
  if np.min(vx) < .5:
    raise ValueError("unsupported path motion")
  psi = np.arctan2(vy, vx)
  rate = (vx*ay-vy*ax)/(vx*vx+vy*vy)
  original_k = rate/np.hypot(vx, vy)

  velocity_offset, acceleration_offset = d@c, dd@c

  def evaluate(scale):
    dy, ddy = velocity_offset*scale, acceleration_offset*scale
    ty, tay = vy+dy, ay+ddy
    hd = np.arctan2(ty, vx)-psi  # vx > 0: angles cannot cross the unwrap seam.
    rd = (vx*tay-ty*ax)/(vx*vx+ty*ty)-rate
    kd = (rate+rd)/np.hypot(vx, ty)-original_k
    correction = float(converter(hd, rd, t, speed, action_delay))
    return hd, rd, kd, correction

  scale = 1.; result = evaluate(scale)
  # Apply cap to the path itself as well as its current converter request.
  if max(abs(result[3]), float(np.max(np.abs(result[2])))) > cap:
    low, high = 0., 1.
    safe_result = evaluate(0.)
    for _ in range(12):  # Always retain the safe side; scale resolution is 1/4096.
      mid = (low+high)*.5; probe = evaluate(mid)
      if max(abs(probe[3]), float(np.max(np.abs(probe[2])))) <= cap: low, safe_result = mid, probe
      else: high = mid
    scale = low; result = safe_result
  hd, rd, kd, correction = result
  if away and correction*away < -1e-10:
    raise ValueError("single-boundary request is not away-only")
  offset = fitted_offset*scale
  output = [v.copy() for v in (t, x, model_y, model_y+offset, hd, rd, kd)]
  for a in output: a.setflags(write=False)
  return TargetPath(frame, *output, correction, base_curvature+correction, scale, iterations)


@lru_cache(maxsize=256)
def _unit_shape(horizon, count, near_priority=False, local=False):
  t, b, d, dd, weights, h = _matrices(horizon, count, near_priority, local)
  c = np.linalg.solve(h, b.T@weights)
  return t, d@c, dd@c


def gain_normalization(horizon, count, speed, action_delay, converter, near_priority=False, local=False):
  """Keep v4's nominal small constant-error gain, independent of preview length.

  Calibrate the unconstrained straight-path small-signal response only. Actual
  curved/bounded geometry still determines the requested correction.
  """
  t, velocity, acceleration = _unit_shape(horizon, count, near_priority, local)
  sensitivity = float(converter(velocity/speed, acceleration/speed, t, speed, action_delay))
  reference = 2. / np.clip(speed, 8., 35.)**2
  if not math.isfinite(sensitivity) or sensitivity <= 1e-9:
    raise ValueError("unsupported path gain")
  return reference/sensitivity
