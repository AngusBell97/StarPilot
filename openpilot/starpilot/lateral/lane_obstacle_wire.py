"""laneObstacleWire: modeld's lane obstacle result, a small fixed binary record (Event.laneObstacleWire, Data)."""
from dataclasses import dataclass, field
import math
import struct

SERVICE = "laneObstacleWire"
MAGIC = b"LOB1"
VERSION = 1
MAX_OBJECTS = 8
# magic, version, seq, frame_id, detect_mono_ns, healthy, blocked, cls, side, n_objects, distance, intrusion, length, score, gpu_ms
_HEAD = struct.Struct("<4sHIIqBBBbB3xfffff")
# cls, flags (1 in-lane blocker, 2 matched to the moving lead), score, x_near, intrusion, y_min, y_max
_OBJ = struct.Struct("<BB2xfffff")
FLAG_BLOCKING, FLAG_LEAD = 1, 2


@dataclass
class WireObject:
  cls: int
  flags: int
  score: float
  x_near: float
  intrusion: float
  y_min: float
  y_max: float


@dataclass
class LaneObstacleWire:
  seq: int
  frame_id: int
  detect_mono_ns: int          # monotonic time the detected camera frame was processed by the driving model
  healthy: bool                # detector ran and the lane was measurable
  blocked: bool                # an obstacle sits inside the ego lane within the look-ahead
  cls: int = -1
  side: int = 0                # -1 from the left line, +1 from the right, 0 across the lane
  distance: float = math.inf   # m ahead of the camera
  intrusion: float = 0.0       # m inside the nearest lane line
  length: float = 0.0          # m, assumed obstacle length (by class) for the pass-by hold
  score: float = 0.0
  gpu_ms: float = 0.0          # host time the detector cost this frame
  objects: list[WireObject] = field(default_factory=list)


def _f(v: float) -> float:
  return v if math.isfinite(v) else 1e9


def encode(w: LaneObstacleWire) -> bytes:
  objs = w.objects[:MAX_OBJECTS]
  out = [_HEAD.pack(MAGIC, VERSION, w.seq & 0xFFFFFFFF, w.frame_id & 0xFFFFFFFF, int(w.detect_mono_ns), int(w.healthy),
                    int(w.blocked), max(0, min(255, w.cls)) if w.cls >= 0 else 255, int(w.side), len(objs),
                    _f(w.distance), _f(w.intrusion), _f(w.length), _f(w.score), _f(w.gpu_ms))]
  out += [_OBJ.pack(o.cls & 0xFF, o.flags & 0xFF, _f(o.score), _f(o.x_near), _f(o.intrusion), _f(o.y_min), _f(o.y_max)) for o in objs]
  return b"".join(out)


def decode(raw: bytes) -> LaneObstacleWire | None:
  try:
    if len(raw) < _HEAD.size:
      return None
    (magic, version, seq, frame_id, mono, healthy, blocked, cls, side, n, distance, intrusion, length, score,
     gpu_ms) = _HEAD.unpack_from(raw, 0)
    if magic != MAGIC or version != VERSION or n > MAX_OBJECTS or len(raw) != _HEAD.size + n * _OBJ.size:
      return None
    inf = lambda v: math.inf if v >= 1e9 else v  # noqa: E731
    objs = [WireObject(*_OBJ.unpack_from(raw, _HEAD.size + i * _OBJ.size)) for i in range(n)]
    for o in objs:
      o.x_near = inf(o.x_near)
    return LaneObstacleWire(seq, frame_id, mono, bool(healthy), bool(blocked), -1 if cls == 255 else cls, side,
                            inf(distance), intrusion, length, score, gpu_ms, objs)
  except struct.error:
    return None
