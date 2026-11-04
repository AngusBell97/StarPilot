"""Host side of the lane obstacle detector: packet decode, ground projection, in-lane decision.

Pure numpy so it runs (and is tested) anywhere. Frames follow modelV2 as the device receives it:
x forward, y RIGHT, z DOWN, origin at the camera; laneLines[1] is the left line of the ego lane,
laneLines[2] the right one.
"""
from dataclasses import dataclass, field
import math

import numpy as np

# detector packet layout (lane_obstacle_graph.py)
PACKET_HEAD = 6
PROTO_W, PROTO_STRIDE = 128, 4
PAD_TOP = 32
# the warped road image is the calibrated medmodel camera
MED_FX, MED_CX, MED_CY = 910.0, 256.0, 47.6

CLASSES = ("car", "van", "truck", "bus", "traffic cone", "person", "wheelchair", "mobility scooter", "pushchair",
           "bicycle", "motorcycle", "scooter", "trailer", "caravan", "motorhome", "tractor", "construction vehicle",
           "traffic barrel", "plastic road barrier", "concrete road barrier", "portable road sign", "bollard", "gate",
           "traffic island marker", "bin", "box", "pallet", "tyre", "ladder", "dog", "cat", "deer", "sheep", "cow", "horse")
VEHICLES = frozenset((0, 1, 2, 3, 10, 12, 13, 14, 15, 16))
VULNERABLE = frozenset((5, 6, 7, 8, 9, 11, 29, 30, 31, 32, 33, 34))
ROADWORKS = frozenset((4, 17, 18, 19, 20, 21, 23))


def class_threshold(cls: int) -> float:
  if cls in VEHICLES or cls in VULNERABLE:
    return 0.35
  if cls in ROADWORKS:
    return 0.30
  return 0.45


@dataclass
class Detection:
  cls: int
  score: float
  box: tuple[float, float, float, float]      # detector input px
  u: np.ndarray                               # contact points, model image px (512x256)
  v: np.ndarray


@dataclass
class Placed:
  det: Detection
  x_near: float          # m ahead of the camera, nearest in-lane contact (inf if none)
  intrusion: float       # m inside the nearest lane line (<= 0: outside the lane)
  side: int              # -1 enters from the left line, +1 from the right, 0 spans the lane
  y_min: float
  y_max: float
  lead: bool = False     # matched to the moving lead the car follows
  candidate: bool = False  # in the lane enough, within the look-ahead and not traffic: may block the lane


@dataclass
class Assessment:
  healthy: bool
  blocked: bool = False
  distance: float = math.inf
  intrusion: float = 0.0
  side: int = 0
  cls: int = -1
  score: float = 0.0
  objects: list[Placed] = field(default_factory=list)


@dataclass(frozen=True)
class Config:
  min_intrusion: float = 0.30       # m inside the lane line before the lane counts as blocked
  min_range: float = 15.0           # m, always look at least this far
  time_window: float = 5.0          # s ahead at the current speed
  max_range: float = 90.0
  lead_gap: float = 6.0             # m, how close a vehicle must sit to the model's lead to be that lead
  moving_lead_speed: float = 3.0    # m/s, a lead faster than this is traffic, not an obstacle
  nms_iou: float = 0.5
  min_lane_prob: float = 0.3
  default_width: float = 3.5
  x_min: float = 5.0                # m, contacts nearer than this are ignored (bonnet, road paint, objects being passed)
  lane_check: bool = True           # trust the lane lines only out to where they still look like a lane
  # tracking (ObstacleTracker): an object blocks the lane only once seen persist times in the last second and, above
  # close_v, closing at close_lo..close_hi times the car's speed for confirm results in a row (a static object closes
  # at 1, slower traffic ahead well below it, oncoming traffic near 2)
  persist: int = 2
  close_lo: float = 0.8
  close_hi: float = 1.6
  close_v: float = 4.0
  confirm: int = 3


def decode(packet: np.ndarray) -> list[Detection]:
  dets = []
  rows = np.asarray(packet, dtype=np.float32)
  rows = rows[np.isfinite(rows[:, 4]) & (rows[:, 4] >= 0.30)]       # below the lowest class threshold
  for row in rows:
    if not np.isfinite(row).all():
      continue
    x1, y1, x2, y2, score, cls = (float(v) for v in row[:PACKET_HEAD])
    cls = int(round(cls))
    if not 0 <= cls < len(CLASSES) or score < class_threshold(cls) or x2 <= x1 or y2 <= y1:
      continue
    low = row[PACKET_HEAD:PACKET_HEAD + PROTO_W]
    cols = np.flatnonzero(low >= 0)
    u = cols * PROTO_STRIDE + PROTO_STRIDE / 2
    v = (low[cols] + 1) * PROTO_STRIDE - PAD_TOP     # bottom edge of the lowest outline cell
    dets.append(Detection(cls, score, (x1, y1, x2, y2), u.astype(np.float32), v.astype(np.float32)))
  return nms(dets)


def nms(dets: list[Detection], iou_max: float = Config.nms_iou) -> list[Detection]:
  kept: list[Detection] = []
  for d in sorted(dets, key=lambda d: -d.score):
    a = d.box
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    if all(_iou(a, area_a, k.box) <= iou_max for k in kept):
      kept.append(d)
  return kept


def _iou(a, area_a, b) -> float:
  iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
  ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
  inter = iw * ih
  return inter / max(area_a + (b[2] - b[0]) * (b[3] - b[1]) - inter, 1e-6)


def ground_points(u: np.ndarray, v: np.ndarray, xs: np.ndarray, z_road: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """Intersect camera rays through model-image pixels with the road height profile z_road(x)."""
  yr = (np.asarray(v, dtype=np.float64) - MED_CY) / MED_FX
  xr = (np.asarray(u, dtype=np.float64) - MED_CX) / MED_FX
  ok = yr > 1e-3
  x = np.full(yr.shape, np.inf)
  ys = np.full(yr.shape, np.nan)
  if ok.any():
    d = np.full(ok.sum(), 1.22) / yr[ok]
    for _ in range(4):           # fixed point: the road height varies slowly with distance
      d = np.interp(np.clip(d, xs[0], xs[-1]), xs, z_road) / yr[ok]
    x[ok] = d
    ys[ok] = xr[ok] * d
  return x, ys


def lane_bounds(xs, lane_y, lane_prob, width_hint, cfg: Config):
  """Left/right ego-lane lines as functions of x; None when neither line is usable."""
  pl, pr = lane_prob[1], lane_prob[2]
  if pl < cfg.min_lane_prob and pr < cfg.min_lane_prob:
    return None
  if pl >= cfg.min_lane_prob and pr >= cfg.min_lane_prob:
    return lane_y[1], lane_y[2]
  w = width_hint if np.isfinite(width_hint) and 2.5 <= width_hint <= 5.0 else cfg.default_width
  return (lane_y[1], lane_y[1] + w) if pl >= cfg.min_lane_prob else (lane_y[2] - w, lane_y[2])


def lane_valid_x(xs, left, right) -> float:
  """Furthest x out to which the ego lane still looks like a lane: plausible width and no sudden sideways swing.
  At junctions and the far end of bends the model's lines wrap across the road; objects there are not in the lane."""
  w = right - left
  bad = (w < 2.2) | (w > 5.5)
  c = (left + right) / 2
  bad[1:] |= np.abs(np.diff(c)) > np.maximum(np.diff(xs) / 3.0, 0.3)
  k = np.flatnonzero(bad & (xs > 5))
  return float(xs[k[0]]) if len(k) else float(xs[-1])


def assess(packet, xs, lane_y, lane_z, lane_prob, v_ego: float, lead=None, width_hint: float = math.nan,
           cfg: Config = Config()) -> Assessment:
  """lead: (x, v, prob) of the model's first lead, or None. blocked/distance give the nearest candidate in this
  single frame; ObstacleTracker decides across frames."""
  xs = np.asarray(xs, dtype=np.float64)
  lane_y, lane_z, lane_prob = np.asarray(lane_y), np.asarray(lane_z), np.asarray(lane_prob)
  bounds = lane_bounds(xs, lane_y, lane_prob, width_hint, cfg)
  if bounds is None:
    return Assessment(healthy=False)
  left, right = bounds
  z_road = (lane_z[1] + lane_z[2]) / 2 if min(lane_prob[1], lane_prob[2]) >= cfg.min_lane_prob else lane_z[1 if lane_prob[1] >= lane_prob[2] else 2]
  reach = min(cfg.max_range, max(cfg.min_range, v_ego * cfg.time_window))
  if cfg.lane_check:
    reach = min(reach, lane_valid_x(xs, np.asarray(left, dtype=np.float64), np.asarray(right, dtype=np.float64)))
  result = Assessment(healthy=True)
  for det in decode(packet):
    if det.u.size == 0:
      continue
    gx, gy = ground_points(det.u, det.v, xs, z_road)
    good = np.isfinite(gx) & (gx > max(0.5, cfg.x_min)) & (gx < cfg.max_range)
    if not good.any():
      continue
    gx, gy = gx[good], gy[good]
    lo = np.interp(gx, xs, left)
    hi = np.interp(gx, xs, right)
    depth = np.minimum(gy - lo, hi - gy)          # > 0 inside the lane
    inside = depth > 0
    intr = float(depth.max())
    x_near = float(gx[inside].min()) if inside.any() else math.inf
    if gy.min() <= lo[gy.argmin()] and gy.max() >= hi[gy.argmax()]:
      side = 0
    else:
      side = -1 if (gy - lo)[depth.argmax()] < (hi - gy)[depth.argmax()] else 1
    placed = Placed(det, x_near, intr, side, float(gy.min()), float(gy.max()))
    traffic = False
    if (lead is not None and det.cls in VEHICLES and lead[2] > 0.3 and math.isfinite(x_near) and
        abs(x_near - lead[0]) < max(cfg.lead_gap, 0.2 * lead[0])):
      # The car being followed: an obstacle only if the car closes on it within the look-ahead time (stopped or much
      # slower traffic); one moving with us is traffic, whatever the lane lines say.
      placed.lead = True
      closing = v_ego - lead[1]
      traffic = lead[1] > cfg.moving_lead_speed and (closing <= 0.5 or x_near / closing > cfg.time_window)
    placed.candidate = not traffic and intr >= cfg.min_intrusion and x_near <= reach
    result.objects.append(placed)
    if placed.candidate and (not result.blocked or x_near < result.distance):
      result.blocked, result.distance, result.intrusion = True, x_near, intr
      result.side, result.cls, result.score = side, det.cls, det.score
  return result


class ObstacleTracker:
  """Follows in-lane candidates from one detector result to the next and says which one blocks the lane.

  A track survives 0.6 s unseen. It blocks once seen cfg.persist times in the last second and, while the car moves
  faster than cfg.close_v, only if its range closes at about the car's own speed: parked cars, cones and stopped
  vehicles close at 1x, the traffic ahead hardly closes and oncoming traffic closes at about 2x. The closing speed is
  a robust (Theil-Sen) fit over the last 1.5 s (four sightings over 0.4 s at least) and must be in range for
  cfg.confirm results in a row, because one reading at 50 m is noisy."""

  KEEP_S = 0.6
  HIST_S = 1.5

  def __init__(self, cfg: Config = Config()):
    self.cfg = cfg
    self.tracks: list[dict] = []

  def reset(self):
    self.tracks = []

  @staticmethod
  def closing_ratio(hist) -> float | None:
    if len(hist) < 4 or hist[-1][0] - hist[0][0] < 0.4:
      return None
    t = np.array([h[0] for h in hist])
    x = np.array([h[1] for h in hist])
    v = float(np.mean([h[2] for h in hist]))
    if v < 1e-3:
      return None
    i, j = np.triu_indices(len(t), 1)
    ok = (t[j] - t[i]) > 0.15
    if not ok.any():
      return None
    return -float(np.median((x[j][ok] - x[i][ok]) / (t[j][ok] - t[i][ok]))) / v

  def update(self, t: float, v_ego: float, objects: list[Placed]) -> Placed | None:
    """t in seconds (monotonic); returns the nearest blocking object, or None."""
    cfg = self.cfg
    for tr in self.tracks:
      tr['matched'] = False
    for p in (o for o in objects if o.candidate):
      x, ym = p.x_near, (p.y_min + p.y_max) / 2
      best = None
      for tr in self.tracks:
        if tr['matched']:
          continue
        tt, tx = tr['hist'][-1][0], tr['hist'][-1][1]
        gap = t - tt
        # static, moving with us, or oncoming: any of the three predictions may continue a track
        dx = min(abs(tx - v_ego * gap - x), abs(tx - x), abs(tx - 2 * v_ego * gap - x))
        if dx < max(4.0, 0.25 * x) and abs(ym - tr['ym']) < 1.5 and (best is None or dx < best[0]):
          best = (dx, tr)
      tr = best[1] if best is not None else {'hist': []}
      if best is None:
        self.tracks.append(tr)
      tr.update(matched=True, ym=ym, placed=p)
      tr['hist'] = [h for h in tr['hist'] if t - h[0] <= self.HIST_S] + [(t, x, v_ego)]
    self.tracks = [tr for tr in self.tracks if t - tr['hist'][-1][0] <= self.KEEP_S]
    blocking = None
    for tr in self.tracks:
      if not tr['matched']:
        continue
      if sum(1 for h in tr['hist'] if t - h[0] <= 1.0) < cfg.persist:
        continue
      if v_ego > cfg.close_v:
        r = self.closing_ratio(tr['hist'])
        tr['ok_run'] = tr.get('ok_run', 0) + 1 if r is not None and cfg.close_lo <= r <= cfg.close_hi else 0
        if tr['ok_run'] < cfg.confirm:
          continue
      if blocking is None or tr['placed'].x_near < blocking.x_near:
        blocking = tr['placed']
    return blocking
