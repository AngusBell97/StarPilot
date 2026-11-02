"""SP-016 smooth lane target and original-model curvature bridge. No I/O."""
from dataclasses import dataclass
from functools import lru_cache
import math
import numpy as np


@dataclass(frozen=True)
class TargetPath:
  frame: int
  t: np.ndarray
  x: np.ndarray
  model_y: np.ndarray
  target_y: np.ndarray
  heading_delta: np.ndarray
  yaw_rate_delta: np.ndarray
  curvature_delta: np.ndarray
  correction: float
  candidate_curvature: float
  cap_scale: float
  fit_iterations: int

  def display(self, corrected=True):
    # Drawing consumes the actual target, never a curve invented from correction.
    return self.x.copy(), (self.target_y if corrected else self.model_y).copy()


class CubicFit:
  """Same scaled cubic least squares as polyfit; share x work across a pair."""
  def __init__(self):
    self.x = self.projection = None

  def __call__(self, x, y, degree):
    if degree != 3:
      raise ValueError("only cubic bend fits are supported")
    if self.x is None or not np.array_equal(x, self.x):
      x2 = x*x
      design = np.column_stack((np.ones(len(x)), x, x2, x2*x))
      scale = np.sqrt(np.sum(design*design, axis=0))
      scale[scale == 0.] = 1.
      u, singular, vh = np.linalg.svd(design/scale, full_matrices=False)
      inverse = np.divide(1., singular, out=np.zeros_like(singular),
                          where=singular > len(x)*np.finfo(float).eps*singular[0])
      self.projection = ((vh.T*inverse)@u.T)/scale[:, None]
      self.x = x.copy()
    return self.projection@y


def _basis(t, knots, degree, derivative=0, cache=None):
  cache = {} if cache is None else cache
  key = degree, derivative
  if key not in cache:
    if derivative > degree:
      out = np.zeros((len(t), len(knots)-degree-1))
    elif degree == 0:
      out = ((t[:, None] >= knots[:-1]) & (t[:, None] < knots[1:])).astype(float)
    else:
      n = len(knots)-degree-1
      a = knots[degree:degree+n]-knots[:n]
      c = knots[degree+1:degree+n+1]-knots[1:n+1]
      ia = np.divide(1., a, out=np.zeros(n), where=a != 0.)
      ic = np.divide(1., c, out=np.zeros(n), where=c != 0.)
      b = _basis(t, knots, degree-1, max(0, derivative-1), cache)
      out = (degree*(ia*b[:, :n]-ic*b[:, 1:n+1]) if derivative else
             (t[:, None]-knots[:n])*ia*b[:, :n] + (knots[degree+1:degree+n+1]-t[:, None])*ic*b[:, 1:n+1])
    cache[key] = out
  return cache[key]


@lru_cache(maxsize=128)
def _matrices(horizon, count, near_priority=False):
  t = np.linspace(0., horizon, count)
  knots = np.r_[np.zeros(4), np.linspace(0., horizon, 11)[1:-1], np.full(4, horizon)]
  # Left endpoint limit at the final knot; retain derivatives there.
  at = np.minimum(t, np.nextafter(horizon, 0.))
  # First three coefficients fixed zero => offset/heading/curvature start at zero.
  basis_cache = {}
  b, d, dd, ddd = [_basis(at, knots, 3, k, basis_cache)[:, 3:] for k in range(4)]
  weight = np.where(t <= 1., 64. if near_priority else 4., 1.)
  h = b.T@(weight[:, None]*b) + .02*dd.T@dd + .002*ddd.T@ddd + np.eye(b.shape[1])*1e-8
  for a in (t, b, d, dd, weight, h): a.setflags(write=False)
  return t, b, d, dd, weight, h


@lru_cache(maxsize=128)
def _free_fit(horizon, count, near_priority):
  _, b, _, _, weight, h = _matrices(horizon, count, near_priority)
  projection = np.linalg.solve(h, b.T*weight)
  projection.setflags(write=False)
  return projection


def _fit(h, f, a, bound):
  """Small feasible active-set QP. Zero must be feasible; fail on nonconvergence."""
  norm = np.linalg.norm(a, axis=1)
  a, bound = a/norm[:, None], bound/norm
  c = np.zeros(len(f)); active = []
  for iteration in range(100):
    g = h@c-f
    if active:
      aw = a[active]
      # Feasible nullspace steps avoid drift along nearly dependent constraints.
      _, _, vh = np.linalg.svd(aw, full_matrices=True)
      null = vh[len(active):].T
      direction = null@np.linalg.solve(null.T@h@null, -null.T@g) if null.shape[1] else np.zeros(len(c))
      multipliers = np.linalg.lstsq(aw.T, -g, rcond=None)[0]
    else:
      direction = np.linalg.solve(h, -g); multipliers = np.empty(0)
    if np.max(np.abs(direction)) < 1e-9:
      if len(active) and np.min(multipliers) < -1e-8:
        active.pop(int(np.argmin(multipliers))); continue
      return c, iteration+1
    travel = a@direction; slack = bound-a@c
    eligible = travel > 1e-10
    if active: eligible[active] = False
    ids = np.flatnonzero(eligible)
    ratios = np.maximum(slack[ids], 0.)/travel[ids]
    block = int(np.argmin(ratios)) if len(ids) else None
    alpha = min(1., float(ratios[block])) if block is not None else 1.
    c += alpha*direction
    if alpha < 1.-1e-9:
      index = int(ids[block])
      trial = a[active+[index]]
      if np.linalg.matrix_rank(trial, tol=1e-9) <= len(active):
        raise ValueError("dependent path constraint")
      active.append(index)
  raise ValueError("path fit did not converge")


def build_path(*, frame, t, x, model_y, desired_offset, lower, upper,
               speed, action_delay, base_curvature, cap, converter, away=0, near_priority=False):
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
  _, b, d, dd, weight, h = _matrices(float(t[-1]), len(t), near_priority)
  clipped = np.clip(desired, lower, upper)
  c = _free_fit(float(t[-1]), len(t), near_priority)@clipped
  offset = b@c
  if (np.any(offset < lower) or np.any(offset > upper) or
      (away and np.any(away*c < 0.))):
    # Fully suppressed spans are equalities. Solve in their nullspace, rather
    # than cycling between dependent opposing inequalities at zero width.
    equality = b[np.abs(upper-lower) < 1e-12]
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
      keep = np.max(np.abs(a), axis=1) > 1e-9
      reduced, iterations = _fit(null.T@h@null, null.T@b.T@(weight*clipped), a[keep], bound[keep])
      c = null@reduced
    else:
      c = np.zeros(b.shape[1]); iterations = 0
  else:
    iterations = 0
  if np.any(b@c < lower-1e-7) or np.any(b@c > upper+1e-7):
    raise ValueError("path bounds failed")
  step = t[-1]/(len(t)-1)  # Uniform input grid validated above.
  vx, vy = np.gradient(x, step, edge_order=2), np.gradient(model_y, step, edge_order=2)
  ax, ay = np.gradient(vx, step, edge_order=2), np.gradient(vy, step, edge_order=2)
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
    for _ in range(12):  # Always retain the safe side; scale resolution is 1/4096.
      mid = (low+high)*.5; probe = evaluate(mid)
      if max(abs(probe[3]), float(np.max(np.abs(probe[2])))) <= cap: low = mid
      else: high = mid
    scale = low; result = evaluate(scale)
  hd, rd, kd, correction = result
  if away and correction*away < -1e-10:
    raise ValueError("single-boundary request is not away-only")
  offset = b@c*scale
  output = [v.copy() for v in (t, x, model_y, model_y+offset, hd, rd, kd)]
  for a in output: a.setflags(write=False)
  return TargetPath(frame, *output, correction, base_curvature+correction, scale, iterations)


@lru_cache(maxsize=128)
def _unit_shape(horizon, count, near_priority=False):
  t, b, d, dd, weights, h = _matrices(horizon, count, near_priority)
  c = np.linalg.solve(h, b.T@weights)
  return t, d@c, dd@c


def gain_normalization(horizon, count, speed, action_delay, converter, near_priority=False):
  """Keep v4's nominal small constant-error gain, independent of preview length.

  Calibrate the unconstrained straight-path small-signal response only. Actual
  curved/bounded geometry still determines the requested correction.
  """
  t, velocity, acceleration = _unit_shape(horizon, count, near_priority)
  sensitivity = float(converter(velocity/speed, acceleration/speed, t, speed, action_delay))
  reference = 2. / np.clip(speed, 8., 35.)**2
  if not math.isfinite(sensitivity) or sensitivity <= 1e-9:
    raise ValueError("unsupported path gain")
  return reference/sensitivity
