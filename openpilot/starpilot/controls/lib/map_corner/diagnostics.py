"""Bounded operational metadata. Never carry location, geometry IDs or exceptions."""
from dataclasses import dataclass,asdict,replace
import json
import math

# Append-only order: compact counter schema1 uses these stable numeric slots.
COUNTER_NAMES=(
  'unknown','ready','gps_missing','gps_stale','gps_invalid','tile_missing','tile_decode',
  'tile_capacity','tile_changed','tile_error','tile_retry','qualifying','contested',
  'no_corner','late_target','preview_incomplete','worker_error','input_expired',
  'input_fenced','output_full','result_missing','result_expired','result_epoch',
  'result_travel','queue_full','worker_failed','proven_continuity','normal_match',
  'unavailable','available','pending','disabled','no_unique_corner','waiting',
  'active','blocked','observation_ready','stored','store_rejected','store_fenced',
  'invalid sample','gap or reverse','confounded','reengaged','ownership',
  'late takeover','repeat passage','timeout','incomplete profile','speed outside scope',
  'no speed change','sequence_gap','map_changed','match_lost','invalid_fix','reset',
  'better competitor','fresh_match','no_match','motion_mismatch','provenance_changed',
  'input_overflow','worker_unavailable','no_intervention','stored_applied',
  'stored_unapplied','duplicate','geometry_mismatch')
REASONS=frozenset(COUNTER_NAMES)
MAX_COUNT=2147483647


@dataclass(frozen=True)
class DiagnosticSnapshot:
  sequence: int=0
  control_epoch: int=0
  observation_fence: int=0
  input_age_ms: float=-1
  compute_ms: float=0
  provider_ms: float=0
  preview_ms: float=0
  gps_age_ms: float=-1
  gps_producer_age_ms: float=-1  # Clock mapping is not established by planner receipt.
  accuracy_m: float=-1
  control_reason: str='unknown'
  control_trusted: bool=False
  match_reason: str='unknown'
  fork_free_metres: float=0
  learning_reason: str='waiting'
  learning_state: str='waiting'
  memory_status: str='unavailable'
  reset_reason: str='unknown'
  parent_reason: str='pending'
  result_age_ms: float=-1
  worker_age_ms: float=-1
  confounds: int=0
  faster_allowed: bool=False
  preview_complete: int=-1
  common_points: int=0
  options: int=0
  option_points: int=0
  late_points: int=0
  decodes: int=0
  reindexes: int=0
  counters: tuple[tuple[str,int],...]=()
  parent_counters: tuple[tuple[str,int],...]=()


def encode_snapshot(snapshot):
  """Supervisor-only serialization; strict vocabulary and total payload limit."""
  data=asdict(snapshot)
  for key,value in tuple(data.items()):
    if isinstance(value,str):data[key]=value if value in REASONS else 'unknown'
    elif isinstance(value,float):data[key]=round(min(1e9,max(-1,value)),2) if math.isfinite(value) else -1
    elif isinstance(value,int):data[key]=min(MAX_COUNT,max(-1,value))
  data['counters']={k:min(MAX_COUNT,max(0,int(v))) for k,v in snapshot.counters if k in REASONS}
  data['parent_counters']={k:min(MAX_COUNT,max(0,int(v))) for k,v in snapshot.parent_counters if k in REASONS}
  encoded=json.dumps(data,separators=(',',':'),allow_nan=False)
  if len(encoded.encode())>2048:
    data['counters']=[data['counters'].get(name,0) for name in COUNTER_NAMES]
    data['counter_schema']=1
    encoded=json.dumps(data,separators=(',',':'),allow_nan=False)
  return encoded


class DiagnosticEmitter:
  def __init__(self,emit):
    self.emit_callback=emit
    self.last=-math.inf
    self.window=None
    self.bytes=0

  def emit(self,snapshot,now):
    if snapshot is None or not math.isfinite(now) or now-self.last<1:return
    if self.window is None or now-self.window>=3600:self.window=now;self.bytes=0
    payload=encode_snapshot(snapshot)
    cost=len(payload.encode())+512  # Reserve space for existing logger metadata.
    if self.bytes+cost>8*1024*1024:return
    self.last=now;self.bytes+=cost
    self.emit_callback(payload)


def parent_snapshot(frame,result,reason):
  snapshot=result.diagnostics if result is not None and result.diagnostics is not None else DiagnosticSnapshot()
  age=(frame.now-result.now)*1000 if result is not None else -1
  return replace(snapshot,parent_reason=reason,result_age_ms=age,worker_age_ms=age)


class ParentDiagnostics:
  """Planner-side bounded scalar accounting; serialization stays in supervisor."""
  def __init__(self):
    self.counts={}
    self.previous=None

  def update(self,frame,result,reason):
    if reason not in ('queue_full','worker_failed','result_missing','result_epoch','result_expired','result_travel','ready'):
      reason='unknown'
    if reason!=self.previous:
      self.counts[reason]=min(MAX_COUNT,self.counts.get(reason,0)+1)
      self.previous=reason
    return replace(parent_snapshot(frame,result,reason),parent_counters=tuple(self.counts.items()))
