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


def assess(packet, xs, lane_y, lane_z, lane_prob, v_ego: float, lead=None, width_hint: float = math.nan,
           cfg: Config = Config()) -> Assessment:
  """lead: (x, v, prob) of the model's first lead, or None."""
  xs = np.asarray(xs, dtype=np.float64)
  lane_y, lane_z, lane_prob = np.asarray(lane_y), np.asarray(lane_z), np.asarray(lane_prob)
  bounds = lane_bounds(xs, lane_y, lane_prob, width_hint, cfg)
  if bounds is None:
    return Assessment(healthy=False)
  left, right = bounds
  z_road = (lane_z[1] + lane_z[2]) / 2 if min(lane_prob[1], lane_prob[2]) >= cfg.min_lane_prob else lane_z[1 if lane_prob[1] >= lane_prob[2] else 2]
  reach = min(cfg.max_range, max(cfg.min_range, v_ego * cfg.time_window))
  result = Assessment(healthy=True)
  for det in decode(packet):
    if det.u.size == 0:
      continue
    gx, gy = ground_points(det.u, det.v, xs, z_road)
    good = np.isfinite(gx) & (gx > 0.5) & (gx < cfg.max_range)
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
    result.objects.append(placed)
    if (not traffic and intr >= cfg.min_intrusion and x_near <= reach and
        (not result.blocked or x_near < result.distance)):
      result.blocked, result.distance, result.intrusion = True, x_near, intr
      result.side, result.cls, result.score = side, det.cls, det.score
  return result
