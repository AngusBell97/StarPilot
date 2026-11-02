from dataclasses import replace,asdict
import json,queue
from types import SimpleNamespace
from openpilot.starpilot.controls.lib.map_corner.worker import Engine
from openpilot.starpilot.controls.lib.map_corner.provider import OfflineProvider
from test_map_corner_runtime import frame,Provider


def test_diagnostics_do_not_change_control():
  a=Engine(Provider());b=Engine(Provider())
  for i in range(20):
    f=frame(now=1+i*.05,sequence=i+1)
    x=a.step(f);y=b.step(f)
    assert getattr(x,'diagnostics',None) is not None, 'worker must provide cause metadata'
    assert replace(x,diagnostics=None)==replace(y,diagnostics=None)
    assert x.diagnostics.sequence==f.sequence
    assert x.diagnostics.compute_ms>=0


def test_diagnostics_bounded_private():
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import DiagnosticSnapshot,encode_snapshot
  snapshot=DiagnosticSnapshot(sequence=10,control_reason='GPS unavailable',counters=(('worker_error',2),))
  payload=encode_snapshot(snapshot)
  assert len(payload.encode())<=1536
  d=json.loads(payload)
  assert d['sequence']==10 and 'latitude' not in d and 'key' not in d
  assert encode_snapshot(replace(snapshot,control_reason='private location 51.5,-2.6')).find('51.5')<0


def test_stale_vs_provider_error_distinguished(tmp_path):
  p=OfflineProvider(tmp_path);e=Engine(p)
  f=frame(fix=None);a=e.step(f)
  b=e.step(frame(now=2,sequence=2,fix=replace(frame().fix,received_at=2,identity=2)))
  assert a.diagnostics.control_reason=='gps_missing'
  assert b.diagnostics.control_reason=='tile_missing'
  assert a.diagnostics.memory_status=='unavailable'


def test_worker_dropped_intervention_fences_learning():
  from openpilot.starpilot.controls.lib.map_corner.worker import process_frame
  e=Engine(Provider());flags=[1,1,1]
  first=process_frame(e,frame(),flags,clock=lambda:1)
  assert first is not None
  dropped=process_frame(e,frame(now=1.05,sequence=2,automated=False,intervention=True),flags,clock=lambda:2)
  assert dropped is None
  fresh=process_frame(e,frame(now=2,sequence=3,automated=False),flags,clock=lambda:2)
  assert not e.observer.active
  assert dict(fresh.diagnostics.counters)['input_expired']==1


def test_worker_error_has_no_exception_text_or_false_freshness():
  from openpilot.starpilot.controls.lib.map_corner.worker import process_frame
  p=Provider();p.update=lambda f,n:(_ for _ in ()).throw(ValueError('private map coordinate'))
  r=process_frame(Engine(p),frame(),[1,1,1],clock=lambda:1.1)
  assert r.now==1 and not r.points
  assert r.diagnostics.control_reason=='worker_error'
  assert 'private' not in json.dumps(asdict(r.diagnostics))


def test_observer_rejection_survives_reset():
  from openpilot.starpilot.controls.lib.map_corner.learning import Observer
  from openpilot.starpilot.controls.lib.map_corner.types import Sample
  from test_map_corner_runtime import corner
  o=Observer();o.update(corner(),Sample(1,0,20,True,'map',confounded=True));o.reset()
  assert o.last_reason=='confounded'
  assert o.rejections['confounded']==1


def test_supervisor_diagnostics_rate_and_byte_bound():
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import DiagnosticSnapshot,DiagnosticEmitter
  events=[];emitter=DiagnosticEmitter(events.append)
  for i in range(1000):emitter.emit(DiagnosticSnapshot(sequence=i),i/100)
  assert len(events)<=10
  assert all(len(x.encode())<=1536 for x in events)


def test_parent_reports_expired_worker_snapshot_without_refreshing_it():
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import parent_snapshot
  from openpilot.starpilot.controls.lib.map_corner.types import Result
  old=Engine(Provider()).step(frame())
  snapshot=parent_snapshot(frame(now=2),old,'result_expired')
  assert snapshot.parent_reason=='result_expired'
  assert snapshot.worker_age_ms==1000
  assert snapshot.sequence==1
  assert old.now==1


def test_runtime_record_is_assignment_only():
  from openpilot.starpilot.controls.lib.map_corner.runtime import Runtime
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import DiagnosticSnapshot
  runtime=Runtime.__new__(Runtime)
  value=DiagnosticSnapshot(sequence=123)
  runtime.record_diagnostics(value)
  assert runtime.diagnostic_snapshot is value


def test_parent_failure_episode_survives_a_later_ready_snapshot():
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import ParentDiagnostics,encode_snapshot
  observer=ParentDiagnostics();result=Engine(Provider()).step(frame())
  observer.update(frame(),result,'queue_full')
  snapshot=observer.update(frame(now=1.05),result,'ready')
  assert dict(snapshot.parent_counters)['queue_full']==1
  assert 'queue_full' in encode_snapshot(snapshot)


def test_control_proof_is_distinct_from_retained_identity():
  p=Provider();p.tracker=SimpleNamespace(contested=True,control_trusted=True,control_reason='proven_continuity',fork_free_metres=140)
  result=Engine(p).step(frame())
  assert result.diagnostics.control_trusted
  assert result.diagnostics.match_reason=='proven_continuity'
  assert result.diagnostics.fork_free_metres==140


def test_worst_case_diagnostic_payload_keeps_all_fixed_parent_failure_counters():
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import DiagnosticSnapshot,encode_snapshot
  reasons=('queue_full','worker_failed','result_missing','result_epoch','result_expired','result_travel','ready')
  snapshot=DiagnosticSnapshot(parent_counters=tuple((r,2147483647) for r in reasons),counters=tuple((r,2147483647) for r in reasons))
  payload=encode_snapshot(snapshot)
  assert len(payload.encode())<=1536
  assert json.loads(payload)['parent_counters']['queue_full']==2147483647


def test_many_reason_counters_cannot_hide_transient_worker_or_tile_failures():
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import DiagnosticSnapshot,encode_snapshot,REASONS
  reasons=sorted(REASONS)
  payload=json.loads(encode_snapshot(DiagnosticSnapshot(control_reason='ready',counters=tuple((r,1) for r in reasons))))
  counters=payload['counters']
  if not isinstance(counters,dict):
    from openpilot.starpilot.controls.lib.map_corner.diagnostics import COUNTER_NAMES
    counters=dict(zip(COUNTER_NAMES,counters))
  assert counters['worker_error']==1 and counters['tile_error']==1


def test_full_reason_vocabulary_is_recoverable_at_maximum_counts():
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import DiagnosticSnapshot,encode_snapshot,REASONS
  d=DiagnosticSnapshot(counters=tuple((r,2147483647) for r in sorted(REASONS)),parent_counters=tuple((r,2147483647) for r in ('queue_full','worker_failed','result_missing','result_epoch','result_expired','result_travel','ready','unknown')))
  payload=encode_snapshot(d);data=json.loads(payload)
  assert len(payload.encode())<=2048
  if isinstance(data['counters'],dict):
    assert set(data['counters'])==REASONS
    assert all(v==2147483647 for v in data['counters'].values())
  else:
    from openpilot.starpilot.controls.lib.map_corner.diagnostics import COUNTER_NAMES
    assert data['counter_schema']==1 and set(COUNTER_NAMES)==REASONS
    assert len(data['counters'])==len(COUNTER_NAMES)
    assert all(v==2147483647 for v in data['counters'])


def test_supervisor_logging_failure_cannot_prevent_worker_start_or_reaping(monkeypatch):
  import multiprocessing
  from openpilot.starpilot.controls.lib.map_corner.runtime import Runtime
  import openpilot.starpilot.controls.lib.map_corner.worker as worker
  import openpilot.starpilot.controls.lib.map_corner.diagnostics as diagnostics
  class Q(queue.Queue):
    def cancel_join_thread(self):pass
    def close(self):pass
  class Process:
    pid=123
    def __init__(self):self.started=False;self.checks=0;self.joined=False
    def start(self):self.started=True
    def is_alive(self):self.checks+=1;return self.checks==1
    def join(self,timeout=None):self.joined=True
    def terminate(self):pass
  process=Process();context=SimpleNamespace(Queue=lambda maxsize:Q(maxsize),RawArray=lambda kind,values:list(values),Process=lambda **kw:process)
  monkeypatch.setattr(multiprocessing,'get_context',lambda method:context)
  monkeypatch.setattr(worker,'demote',lambda:None)
  def fail(*args):raise OSError('logger unavailable')
  monkeypatch.setattr(diagnostics.DiagnosticEmitter,'emit',fail)
  runtime=Runtime.__new__(Runtime);runtime.stopped=False;runtime.failed=False;runtime.root='unused';runtime.map_root='unused';runtime.diagnostic_snapshot=None
  runtime._supervise()
  assert process.started and process.joined and runtime.failed
  assert runtime.inputs is None and runtime.outputs is None and runtime.flags is None


def test_extreme_scalars_and_all_worker_counters_stay_bounded_and_decodable():
  from dataclasses import fields
  from openpilot.starpilot.controls.lib.map_corner.diagnostics import DiagnosticSnapshot,encode_snapshot,REASONS,COUNTER_NAMES
  d=DiagnosticSnapshot();values={}
  for field in fields(d):
    value=getattr(d,field.name)
    if isinstance(value,bool):values[field.name]=True
    elif isinstance(value,int):values[field.name]=2147483647
    elif isinstance(value,float):values[field.name]=1e9
    elif isinstance(value,str):values[field.name]=max(REASONS,key=len)
  values['counters']=tuple((name,2147483647) for name in COUNTER_NAMES)
  values['parent_counters']=tuple((name,2147483647) for name in ('queue_full','worker_failed','result_missing','result_epoch','result_expired','result_travel','ready','unknown'))
  payload=encode_snapshot(DiagnosticSnapshot(**values));assert len(payload.encode())<=2048
  data=json.loads(payload);counters=data['counters']
  if not isinstance(counters,dict):counters=dict(zip(COUNTER_NAMES,counters))
  assert counters['worker_error']==2147483647 and len(counters)==len(REASONS)


def test_worker_error_retains_all_accumulated_learning_rejections():
  from openpilot.starpilot.controls.lib.map_corner.worker import process_frame
  p=Provider();engine=Engine(p);engine.observer.rejections['confounded']=7
  first=engine.step(frame());before=dict(first.diagnostics.counters)
  p.update=lambda f,n:(_ for _ in ()).throw(RuntimeError('provider failed'))
  error=process_frame(engine,frame(now=1.05,sequence=2),[1,1,1],clock=lambda:1.05)
  after=dict(error.diagnostics.counters)
  assert all(after.get(key,0)>=value for key,value in before.items())
  assert after['worker_error']==1


def test_invalid_coordinates_cannot_reuse_ready_status(tmp_path):
  provider=OfflineProvider(tmp_path);provider.status='Map ready'
  f=frame(fix=replace(frame().fix,latitude=86))
  result=Engine(provider).step(f)
  assert result.status=='GPS unavailable'
  assert result.diagnostics.control_reason=='gps_invalid'
