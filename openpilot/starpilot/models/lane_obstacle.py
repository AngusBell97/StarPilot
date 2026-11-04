"""Lane obstacle detector on Chestnut, run inside modeld beside the driving model (change 37).

The driving model (a stateful Chestnut catalog model) copies the warped road image to Chestnut every frame
(`mapped_gpu_image`). On even frames, after modelV2 is published, the detector graph is queued on that image
without waiting; on the next frame its small result (16 objects with outline contact profiles, ~8.6 KB) is read
back and turned into a laneObstacleWire. The driving model always comes first: the detector never runs before
the driving model's outputs are read, and any failure switches the detector off for the drive.
"""
import hashlib
import json
import math
import time
from collections import deque
from pathlib import Path

import numpy as np

from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.lateral import lane_obstacle_geometry as GEO
from openpilot.starpilot.lateral.lane_obstacle_wire import FLAG_BLOCKING, FLAG_LEAD, LaneObstacleWire, WireObject, encode

ARTIFACT_DIR = Path('/data/models/lane_obstacle')
ARTIFACT = ARTIFACT_DIR / 'lane_obstacle_yoloe26n_seg35_mm512x256.pkl'
CONFIG = Path('/data/starpilot/lane_obstacle.json')
INPUT_SHAPE = (2, 6, 128, 256)
ARTIFACT_ABI = 'starpilot_lane_obstacle_v1'
_LONG = frozenset((2, 3, 12, 13, 14))            # truck, bus, trailer, caravan, motorhome


def object_length(cls: int) -> float:
  if cls in _LONG:
    return 12.0
  if cls in GEO.VEHICLES:
    return 6.0
  return 1.5


def enabled() -> bool:
  try:
    return bool(json.loads(CONFIG.read_text()).get('enabled', False))
  except (OSError, ValueError, AttributeError):
    return False


def _sha256(path: Path) -> str:
  h = hashlib.sha256()
  with path.open('rb') as f:
    while block := f.read(1 << 20):
      h.update(block)
  return h.hexdigest()


def load_artifact(path: Path = ARTIFACT):
  """Load the pickled detector JIT through the driving runtime's Chestnut loader and guard its USB submissions,
  exactly as the catalog loader does for the driving model's own captures."""
  from tinygrad import Context
  from tinygrad.uop.ops import Ops
  from tinygrad.runtime.support.hcq2 import HCQInfo
  from openpilot.starpilot.models.runtime_bridge.runtime import _guard, _load_chestnut, selected_legacy
  if not selected_legacy():
    raise RuntimeError('lane obstacle detector requires the pinned driving runtime')
  expected = (path.parent / (path.name + '.sha256')).read_text().split()[0]
  if _sha256(path) != expected:
    raise ValueError('lane obstacle artifact changed')
  protection = _guard(True)
  with Context(DEV='USB+AMD:LLVM'):
    artifact = _load_chestnut(path)
  if not isinstance(artifact, dict) or artifact.get('artifact_abi') != ARTIFACT_ABI or tuple(artifact.get('input_shape', ())) != INPUT_SHAPE:
    raise ValueError('unexpected lane obstacle artifact')
  jit = artifact['run']
  cap = jit.captured
  if 'linear' in cap.__dict__:
    raise ValueError('protection must precede captured execution')
  replacements = {u: protection.protect_call(u) for u in cap._linear.toposort()
                  if u.op is Ops.CALL and isinstance(u.arg.aux, HCQInfo)}
  if not replacements:
    raise ValueError('lane obstacle artifact has no native submission')
  cap._linear = cap._linear.substitute(replacements, enter_calls=True)
  return jit, artifact


class LaneObstacleDetector:
  """Runs at most every MIN_PERIOD_S, and only in time modeld has spare before its next camera frame."""
  FRAME_PERIOD_NS = 50_000_000
  MARGIN_NS = 4_000_000          # always leave this much before the next frame is due
  MIN_PERIOD_S = 0.095           # 10 Hz cap
  MAX_PENDING_S = 0.6            # a queued result not read by then is dropped
  LEAD_MEMORY_S = 2.0            # a confident lead is remembered this long through probability dips

  def __init__(self, model, jit):
    self.model = model
    self.jit = jit
    self.image = model.input_queues['mapped_gpu_image']
    self.pending = None
    self.context = None
    self.seq = 0
    self.failed = False
    self.width_hint = math.nan
    self.last_log = 0.0
    self.last_submit = -math.inf
    self.pipeline_ns = math.inf  # least frame-end to frame-ready delay seen: the camera pipeline plus any clock offset
    self.deadline_ns = 0
    self.samples = {'submit': deque([6e-3], maxlen=20), 'read': deque([12e-3], maxlen=20)}   # s, host time of each phase
    self.waited = 0                # frames in a row the budget said no: the estimate relaxes so a spike cannot starve it
    self.lead_memory = None
    self.tracker = GEO.ObstacleTracker()
    self.stats = {'frames': 0, 'submits': 0, 'reads': 0, 'wait_budget': 0, 'dropped': 0}
    self.stats_t = time.monotonic()

  def begin_frame(self, frame_eof_ns: int) -> None:
    """Call as soon as modeld has the camera frame: sets when the next frame will be ready."""
    now = time.monotonic_ns()
    delay = now - int(frame_eof_ns)
    # The least delay seen is the camera pipeline (modeld waiting idle); it rises 0.2 ms a frame so a clock step heals.
    self.pipeline_ns = min(delay, self.pipeline_ns + 200_000) if math.isfinite(self.pipeline_ns) else delay
    self.deadline_ns = int(frame_eof_ns) + self.FRAME_PERIOD_NS + int(self.pipeline_ns)
    self.stats['frames'] += 1

  def cost(self, phase: str) -> float:
    return float(np.percentile(self.samples[phase], 75)) * 0.85 ** self.waited

  def _fits(self, phase: str) -> bool:
    if self.deadline_ns - time.monotonic_ns() - self.MARGIN_NS >= self.cost(phase) * 1e9:
      self.waited = 0
      return True
    self.waited += 1
    self.stats['wait_budget'] += 1
    return False

  def _learn(self, phase: str, seconds: float) -> None:
    self.samples[phase].append(seconds)

  def _log_stats(self) -> None:
    now = time.monotonic()
    if now - self.stats_t >= 10.0:
      dt = now - self.stats_t
      cloudlog.event('laneObstacleStats', hz=round(self.stats['reads'] / dt, 2), frames=self.stats['frames'],
                     waits=self.stats['wait_budget'], dropped=self.stats['dropped'],
                     cost_submit_ms=round(self.cost('submit') * 1e3, 1), cost_read_ms=round(self.cost('read') * 1e3, 1),
                     pipeline_ms=round(self.pipeline_ns / 1e6, 1))
      self.stats = {k: 0 for k in self.stats}
      self.stats_t = now

  @classmethod
  def create(cls, model):
    """None unless enabled, the driving model is a stateful Chestnut model and the artifact loads and runs."""
    if not enabled():
      return None
    try:
      queues = getattr(model, 'input_queues', {})
      if not getattr(model, 'chestnut', False) or 'mapped_gpu_image' not in queues:
        cloudlog.warning('lane obstacle detector: driving model has no Chestnut image, off')
        return None
      if tuple(queues['mapped_gpu_image'].shape) != INPUT_SHAPE:
        cloudlog.warning('lane obstacle detector: unexpected driving image shape, off')
        return None
      st = time.monotonic()
      jit, _ = load_artifact()
      det = cls(model, jit)
      packet = det.jit(det.image).numpy()
      if packet.shape != (16, GEO.PACKET_HEAD + GEO.PROTO_W) or not np.isfinite(packet).all():
        raise ValueError('lane obstacle detector warmup output invalid')
      cloudlog.warning(f'lane obstacle detector ready in {time.monotonic() - st:.1f}s')
      return det
    except Exception:
      cloudlog.exception('lane obstacle detector failed to start, off')
      return None

  def _context(self, modelv2, v_ego: float):
    lines = modelv2.laneLines
    xs = np.asarray(lines[1].x, dtype=np.float64)
    lane_y = np.zeros((4, len(xs)))
    lane_z = np.zeros((4, len(xs)))
    for i in (1, 2):                 # only the ego lane's lines are used
      lane_y[i] = np.asarray(lines[i].y, dtype=np.float64)
      lane_z[i] = np.asarray(lines[i].z, dtype=np.float64)
    probs = np.asarray(modelv2.laneLineProbs, dtype=np.float64)
    lead = None
    now = time.monotonic()
    if len(modelv2.leadsV3):
      ld = modelv2.leadsV3[0]
      if len(ld.x) and len(ld.v):
        lead = (float(ld.x[0]), float(ld.v[0]), float(ld.prob))
    if lead is not None and lead[2] > 0.5:
      self.lead_memory = (lead, now)
    elif self.lead_memory is not None and now - self.lead_memory[1] < self.LEAD_MEMORY_S:
      # The followed car survives a short dip in the model's lead probability, moved on by the relative speed.
      (x, v, _), t = self.lead_memory
      lead = (x + (v - v_ego) * (now - t), v, 0.5)
    return dict(frame_id=int(modelv2.frameId), mono_ns=time.monotonic_ns(), xs=xs, lane_y=lane_y, lane_z=lane_z,
                probs=probs, lead=lead, v_ego=float(v_ego))

  def step(self, run_count: int, modelv2, v_ego: float) -> bytes | None:
    """Call once per published modelV2, after begin_frame. Returns a laneObstacleWire payload when a result is read."""
    if self.failed:
      return None
    try:
      self._log_stats()
      if self.pending is None:
        if time.monotonic() - self.last_submit < self.MIN_PERIOD_S:
          return None
        if not self._fits('submit'):
          return None
        t0 = time.perf_counter()
        self.context = self._context(modelv2, v_ego)
        self.pending = self.jit(self.image)
        self.last_submit = time.monotonic()
        dt = time.perf_counter() - t0
        self._learn('submit', dt)
        self.context['submit_ms'] = dt * 1e3
        self.stats['submits'] += 1
        return None
      if time.monotonic() - self.last_submit > self.MAX_PENDING_S:
        self.pending = None            # too old to steer by; the next submit starts afresh (no readback needed)
        self.stats['dropped'] += 1
        return None
      if not self._fits('read'):
        return None
      t0 = time.perf_counter()
      packet = self.pending.numpy()
      self.pending = None
      if not np.isfinite(packet).all():
        raise ValueError('lane obstacle detector output not finite')
      c = self.context
      a = GEO.assess(packet, c['xs'], c['lane_y'], c['lane_z'], c['probs'], c['v_ego'], lead=c['lead'], width_hint=self.width_hint)
      # across frames: only an object seen repeatedly and closing like a static one blocks the lane (traffic ahead and
      # oncoming traffic do not)
      block = self.tracker.update(c['mono_ns'] / 1e9, c['v_ego'], a.objects if a.healthy else [])
      a.blocked = block is not None
      if block is not None:
        a.distance, a.intrusion, a.side, a.cls, a.score = block.x_near, block.intrusion, block.side, block.det.cls, block.det.score
      else:
        a.distance, a.intrusion, a.side, a.cls, a.score = math.inf, 0.0, 0, -1, 0.0
      if min(c['probs'][1], c['probs'][2]) >= 0.5:
        w = float(c['lane_y'][2][0] - c['lane_y'][1][0])
        if 2.5 <= w <= 5.0:
          self.width_hint = w
      self.seq += 1
      objs = sorted(a.objects, key=lambda p: p.x_near)[:8]
      read_s = time.perf_counter() - t0
      self._learn('read', read_s)
      self.stats['reads'] += 1
      wire = LaneObstacleWire(self.seq, c['frame_id'], c['mono_ns'], a.healthy, a.blocked, a.cls, a.side, a.distance,
                              a.intrusion, object_length(a.cls) if a.blocked else 0.0, a.score, c['submit_ms'] + read_s * 1e3,
                              [WireObject(p.det.cls, (FLAG_BLOCKING if p is block else 0) |
                                          (FLAG_LEAD if p.lead else 0), p.det.score, p.x_near, p.intrusion, p.y_min, p.y_max)
                               for p in objs])
      now = time.monotonic()
      if a.blocked and now - self.last_log > 1.0:
        self.last_log = now
        cloudlog.event('laneObstacle', distance=round(a.distance, 1), intrusion=round(a.intrusion, 2), cls=GEO.CLASSES[a.cls],
                       score=round(a.score, 2), side=a.side, frame=c['frame_id'])
      return encode(wire)
    except Exception:
      self.failed = True
      self.pending = None
      cloudlog.exception('lane obstacle detector failed, off for this drive')
      return encode(LaneObstacleWire(self.seq + 1, 0, time.monotonic_ns(), False, False))
