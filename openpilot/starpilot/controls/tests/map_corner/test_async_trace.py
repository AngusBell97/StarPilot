from dataclasses import replace
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'work'))
from replay_async import VirtualWorker
from openpilot.starpilot.controls.lib.map_corner.worker import Engine,process_frame
from test_map_corner_runtime import frame,Provider


def test_virtual_results_preserve_original_age_through_delay():
  w=VirtualWorker(Engine(Provider()),process_frame,cost=lambda f:.6 if f.sequence==1 else 0)
  w.exchange(frame())
  w.advance(1.6)
  assert w.outputs[0].now==1  # Completion never fabricates a fresh timestamp.
  results=[]
  for i in range(1,30):
    f=frame(now=1+i*.05,sequence=i+1)
    result,full=w.exchange(f)
    if result:results.append((f.now,result))
  assert w.expired>=1
  assert all(result.now<=now for now,result in results)


def test_virtual_overflow_is_bounded_and_explicit():
  w=VirtualWorker(Engine(Provider()),process_frame,cost=lambda f:10)
  overflows=0
  for i in range(150):
    _,overflow=w.exchange(frame(now=1+i*.05,sequence=i+1));overflows+=overflow
  assert len(w.inputs)==64 and overflows>0


def test_dropped_manual_sample_never_becomes_an_observation():
  w=VirtualWorker(Engine(Provider()),process_frame,cost=lambda f:1 if f.sequence==1 else 0)
  for i in range(60):
    w.exchange(frame(now=1+i*.05,sequence=i+1,automated=i<4,intervention=i==4))
  assert not w.engine.observer.active and w.engine.stats['commits']==0


def test_virtual_fences_cannot_authorize_old_queued_learning():
  w=VirtualWorker(Engine(Provider()),process_frame,cost=lambda f:.2)
  for i in range(10):
    w.exchange(frame(now=1+i*.05,sequence=i+1,observation_fence=1 if i<3 else 2))
  w.advance(3)
  assert w.engine.diagnostic_counts.get('input_fenced',0)>0
  assert w.engine.stats['commits']==0


def test_identical_virtual_schedules_produce_identical_control():
  a=VirtualWorker(Engine(Provider()),process_frame,cost=lambda f:.12)
  b=VirtualWorker(Engine(Provider()),process_frame,cost=lambda f:.12)
  for i in range(60):
    f=frame(now=1+i*.05,sequence=i+1)
    x,xa=a.exchange(f);y,ya=b.exchange(f)
    assert xa==ya
    assert x==y  # Diagnostic metadata is deliberately not part of control equality.
