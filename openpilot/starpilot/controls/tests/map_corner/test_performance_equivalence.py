"""Frozen rejected candidate is the behavioral oracle, including float bits."""
import dataclasses
from collections import deque
import importlib.util
import math
import os
from pathlib import Path
import random
import struct
import sys
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from openpilot.starpilot.controls.lib.map_corner import controller as C, geometry as G, routes as R, worker as W, provider as P
from openpilot.starpilot.controls.lib.map_corner.types import PointCap, Way, Match
from test_map_corner_provider import bend_way, fix, ll
from test_route_worker import frame

ROOT=next((parent for parent in Path(__file__).resolve().parents if (parent/'COMMA-TO-DO.txt').exists()),Path('/'))
BASE=Path(__file__).resolve().parent/'fixtures/performance'
spec=importlib.util.spec_from_file_location('sp034_frozen',BASE/'__init__.py',submodule_search_locations=[str(BASE)])
package=importlib.util.module_from_spec(spec);sys.modules[spec.name]=package;spec.loader.exec_module(package)
from sp034_frozen import controller as BC, geometry as BG, routes as BR, provider as BP, worker as BW

def bits(x):
  # Compare common behavioral fields with the frozen pre-metadata schema.
  # Admission metadata is exercised separately by acquisition/transport tests.
  if dataclasses.is_dataclass(x) and hasattr(x,'admission_speed'):
    assert x.admission_speed==0 or 0<x.admission_speed<=x.prior
  if isinstance(x,float):return ('float',struct.pack('!d',x).hex())
  if dataclasses.is_dataclass(x):return tuple((f.name,bits(getattr(x,f.name))) for f in dataclasses.fields(x) if f.compare and f.name not in ('ceiling','curvature','admission_speed','admission_prior'))
  if isinstance(x,dict):return tuple((bits(k),bits(v)) for k,v in x.items())
  if isinstance(x,(tuple,list,deque)):return tuple(bits(v) for v in x)
  return x

def envelope_kwargs(**kw):
  return dict(cruise=35.,speed=24.,acceleration=.3,uncertainty=17.,travelled=2.5,**kw)

def legacy_point(distance,speed,prior,key):
  return PointCap(distance,speed,prior,key,ceiling=prior*math.sqrt(4/1.8))
def test_common_allowances_calculated_once_per_envelope(monkeypatch):
  original=C.math.sqrt;calls=[]
  monkeypatch.setattr(C.math,'sqrt',lambda x:(calls.append(x),original(x))[1])
  common=tuple(PointCap(100.+i,10.,10.,'a',ceiling=(10.)*(4/1.8)**0.5) for i in range(24))
  C.route_envelope(common,tuple(() for _ in range(32)),**envelope_kwargs())
  # One prior allowance and the existing speed-ratio sqrt per common point.
  assert len(calls)<=48

def test_envelope_bitwise_oracle_random_and_capacity_edges():
  rng=random.Random(34021)
  for n in range(350):
    points=tuple(legacy_point(rng.choice([0.,-0.,rng.uniform(-20,1200)]),rng.uniform(0,40),rng.uniform(0,40),rng.choice(['','a','b','c'])) for _ in range(15))
    common=points[:rng.randrange(12)]
    options=tuple(tuple(rng.choice(points) for _ in range(rng.randrange(12))) for _ in range(rng.randrange(8)))
    kw=dict(cruise=rng.uniform(0,45),speed=rng.uniform(0,55),acceleration=rng.uniform(-4,4),uncertainty=rng.uniform(0,50),travelled=rng.uniform(0,30),skip_key=rng.choice(['','a']),allow_faster=bool(n%2))
    assert bits(C.route_envelope(common,options,**kw))==bits(BC.route_envelope(common,options,**kw))
  a=PointCap(100.,10.,10.,'a',ceiling=(10.)*(4/1.8)**0.5);b=dataclasses.replace(a,key='b')
  for common,options in [((),()),((a,b),((a,),(b,))),((a,)*25,((b,),)),((a,),((b,),)*33),((a,),((b,)*129,)),((a,),((b,)*128,)*32)]:
    for skip in ('','a'):
      for faster in (False,True):
        kw=envelope_kwargs(skip_key=skip,allow_faster=faster)
        assert bits(C.route_envelope(common,options,**kw))==bits(BC.route_envelope(common,options,**kw))

def test_envelope_signed_zero_nonfinite_and_deadband_order():
  for v in (-0.,0.,float('nan'),float('inf')):
    common=(PointCap(-0.,v,v,'a',ceiling=(v)*(4/1.8)**0.5),PointCap(0.,v,v,'b',ceiling=(v)*(4/1.8)**0.5))
    kw=envelope_kwargs()
    assert bits(C.route_envelope(common,((),common),**kw))==bits(BC.route_envelope(common,((),common),**kw))
  for delta in (0.,.299999999,.3,.300000001):
    common=(PointCap(0.,10.,10.,'a',ceiling=(10.)*(4/1.8)**0.5),PointCap(0.,10.+delta,10.+delta,'b',ceiling=(10.+delta)*(4/1.8)**0.5))
    assert bits(C.route_envelope(common,((),),**envelope_kwargs()))==bits(BC.route_envelope(common,((),),**envelope_kwargs()))

def test_preview_identical_progress_reuses_render_and_is_bounded():
  tile=G.build_tile((bend_way(),),'r');match=G.find_match(tile,fix())
  first=R.preview_routes(tile,match)
  assert R.preview_routes(tile,match) is first
  for offset in (1.,2.,-1.,0.):
    current=dataclasses.replace(match,progress=match.progress+offset)
    before=R.preview_routes(G.build_tile((bend_way(),),'r'),current)
    assert bits(R.preview_routes(tile,current))==bits(before)
  road=Way(1,tuple(ll(x,0) for x in range(-150,1001,10)),False,'secondary','')
  many=G.build_tile((road,),'r')
  for i in range(100):R.preview_routes(many,Match(1,1,i*10.,0.,10.,(),'r'))
  assert len(many.route_cache)==64
  changed=G.build_tile((bend_way(),),'new')
  assert R.preview_routes(changed,match) is not first

def test_build_tile_unindexed_retains_normalization_and_capacity_validation():
  way=bend_way();way=dataclasses.replace(way,points=way.points[:5]+(way.points[4],)+way.points[5:])
  expected=BG.build_tile((way,),'r');actual=G.build_tile((way,),'r',index=False)
  assert bits(actual.roads)==bits(expected.roads)
  assert actual.origin==expected.origin and actual.nodes=={} and actual.cells=={}
  main=Way(1,(ll(-400,0),ll(0,0),ll(.01,0),ll(400,0)),False,'secondary','')
  side=Way(2,(ll(.01,0),ll(.01,200)),False,'secondary','')
  cases=[(main,side),(dataclasses.replace(way,identity=0),),(dataclasses.replace(way,points=(ll(0,0),ll(10000,0))),),(way,dataclasses.replace(way,points=way.points[:-2]))]
  for ways in cases:
    with pytest.raises(Exception) as expected:BG.build_tile(ways,'r')
    with pytest.raises(type(expected.value),match=str(expected.value)):G.build_tile(ways,'r',index=False)

def test_route_build_memoizes_shared_graph_queries(monkeypatch):
  from test_route_graph import straight
  ways=(straight(),Way(2,tuple(ll(0,y) for y in range(0,301,10)),False,'secondary',''))
  tile=G.build_tile(ways,'r');match=G.find_match(tile,fix())
  calls=[];original=R.outgoing
  def counted(tile,node,previous=None):
    calls.append((node,previous));return original(tile,node,previous)
  monkeypatch.setattr(R,'outgoing',counted)
  assert bits(R.preview_routes(tile,match))==bits(BR.preview_routes(BG.build_tile(ways,'r'),match))
  assert len(calls)==len(set(calls))

def test_worker_reuses_unchanged_published_envelope(monkeypatch):
  p=PointCap(400.,12.,12.,'bend',ceiling=(12.)*(4/1.8)**0.5)
  monkeypatch.setattr(R,'preview_routes',lambda tile,match:R.Preview('route',0.,((p,),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0.,0.,0.,(),'r'))
  calls=[];original=W.route_envelope
  def counted(*a,**kw):calls.append(1);return original(*a,**kw)
  monkeypatch.setattr(W,'route_envelope',counted)
  engine=W.Engine(provider);engine.step(frame(speed=5,acceleration=-.6))
  assert len(calls)==1
  calls.clear();engine.step(frame(1.05,speed=25,acceleration=-.6,applied_map_cap=5))
  assert len(calls)==2

def test_decode_bitwise_and_error_priority(tmp_path):
  import capnp
  schema=capnp.load(str(Path(P.__file__).with_name('offline.capnp')))
  variants=[[(.1,-0.),(.1,0.),(.15,.1)],[(3.,3.),(float('nan'),0.)],[(3.,3.),(4.,4.)],[(.1,.1),(3.,3.)],[]]
  for i,points in enumerate(variants):
    msg=schema.Offline.new_message(minLat=0,minLon=0,maxLat=.25,maxLon=.25,overlap=0,
      ways=[dict(id=1,highwayClass='secondary',nodes=[dict(latitude=a,longitude=b) for a,b in points])])
    path=tmp_path/str(i);path.write_bytes(msg.to_bytes_packed())
    providers=(BP.OfflineProvider(tmp_path),P.OfflineProvider(tmp_path));out=[]
    for provider in providers:
      provider.schema=schema
      try:out.append(bits(provider._decode(path)))
      except Exception as e:out.append((type(e).__name__,str(e)))
    assert out[0]==out[1]

def test_actual_drive_tiles_decode_bitwise():
  import capnp
  schema=capnp.load(str(Path(P.__file__).with_name('offline.capnp')))
  paths=sorted(p for p in Path(os.environ.get('SP034_REPLAY_TILES',str(ROOT/'Current evidence/Longitudinal/SP-034-first-drive-20261021/tiles'))).rglob('*') if p.is_file())
  assert len(paths)==2
  for path in paths:
    a=BP.OfflineProvider();b=P.OfflineProvider();a.schema=b.schema=schema
    assert bits(a._decode(path))==bits(b._decode(path))

@pytest.mark.parametrize('scenario',['slow','faster','confounded','visual_owner','wrong_binding','invalid_commit'])
def test_learning_calls_and_state_match_old_with_weather_satisfied(monkeypatch,scenario):
  from openpilot.starpilot.controls.lib.map_corner.types import Corner
  c=Corner('common',1,1,'r',100.,145.,190.,(20.,20.,20.),((0,0),(1,1)),0,curvature=tuple(1.8/(v*v) for v in (20.,20.,20.)))
  monkeypatch.setattr(R,'preview_routes',lambda tile,match:R.Preview('route',1000.,((),),True))
  monkeypatch.setattr(BR,'preview_routes',lambda tile,match:BR.Preview('route',1000.,((),),True))
  progress=[0.];observed=[[],[]];engines=[]
  for module,calls in zip((BW,W),observed):
    provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,progress[0],0.,0.,(c,),'r'))
    store=SimpleNamespace(lookup=lambda c:None,observe=lambda *args,calls=calls:calls.append(bits(args[:4])))
    engine=module.Engine(provider,store);engine.session='fixed-session:';engines.append(engine)
  for i in range(205):
    progress[0]=float(i);auto=i<75
    owner='visual' if scenario=='visual_owner' else 'map'
    f=frame(1+i*.05,speed=22. if auto else 25. if scenario=='faster' else max(15.,22.-(i-75)*.25),acceleration=-.6,
      odometer=float(i),automated=auto,owner=owner if auto else 'manual',binding_key='wrong' if scenario=='wrong_binding' else 'common' if auto else '',
      clear_conditions=True,intervention=i==75,observation_id='takeover' if i==75 else '',applied_map_cap=20. if auto else 0.,
      control_epoch=0 if auto else 1,confounded=scenario=='confounded' and i==90)
    results=[e.step(f,lambda:scenario!='invalid_commit') for e in engines]
    assert bits(results[0])==bits(results[1])
    for field in ('stats','late_points','late_overflow','previous_safe','continuation_deficit'):
      assert bits(getattr(engines[0],field))==bits(getattr(engines[1],field))
    assert bits({k:v for k,v in engines[0].observer.__dict__.items() if k!='last_reason'})==bits({k:v for k,v in engines[1].observer.__dict__.items() if k!='last_reason'})
  assert observed[0]==observed[1]
  assert len(observed[1])==(1 if scenario in ('slow','faster') else 0)

def test_worker_deduplication_cannot_bypass_overflow_recalculation(monkeypatch):
  point=PointCap(400.,12.,12.,'bend',ceiling=(12.)*(4/1.8)**0.5);preview=R.Preview('route',0.,((point,),)*33,True)
  monkeypatch.setattr(R,'preview_routes',lambda tile,match:preview)
  monkeypatch.setattr(BR,'preview_routes',lambda tile,match:preview)
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0.,0.,0.,(),'r'))
  old=BW.Engine(provider);new=W.Engine(provider)
  assert bits(old.step(frame(speed=35.,automated=False)))==bits(new.step(frame(speed=35.,automated=False)))
  assert old.previous_safe==new.previous_safe==False

def test_unindexed_tile_still_enforces_full_index_capacity(monkeypatch):
  # Sparse fake cell storage lets the frozen implementation hit its real
  # million-entry guard without retaining a million Python allocations.
  class Bucket:
    def append(self,value):pass
  class Cells(dict):
    def setdefault(self,key,value):return Bucket()
  original=BG.Tile
  def tile(*a,**kw):
    result=original(*a,**kw);result.cells=Cells();return result
  monkeypatch.setattr(BG,'Tile',tile)
  monkeypatch.setattr(BG,'CELL',.001);monkeypatch.setattr(G,'CELL',.001)
  way=Way(1,(ll(0,0),ll(10,10)),False,'secondary','')
  for function,kw in ((BG.build_tile,{}),(G.build_tile,dict(index=False)),(G.build_tile,{})):
    with pytest.raises(ValueError,match='Map index capacity'):function((way,),'r',**kw)

def test_decoder_count_limit_precedes_node_materialization(tmp_path):
  class Nodes:
    def __len__(self):return 300001
    def __iter__(self):raise AssertionError('Must reject before reading coordinates')
  data=SimpleNamespace(minLat=0.,minLon=0.,maxLat=.25,maxLon=.25,overlap=0.,ways=[SimpleNamespace(nodes=Nodes())])
  schema=SimpleNamespace(Offline=SimpleNamespace(from_bytes_packed=lambda *a,**k:data))
  path=tmp_path/'tile';path.write_bytes(b'x'*100)
  for module in (BP,P):
    provider=module.OfflineProvider();provider.schema=schema
    with pytest.raises(ValueError,match='Unsupported tile size'):provider._decode(path)

def test_only_latest_render_is_retained_across_segments():
  import gc,weakref
  tile=G.build_tile((bend_way(),),'r');match=G.find_match(tile,fix())
  first=R.preview_routes(tile,match);reference=weakref.ref(first)
  del first
  R.preview_routes(tile,dataclasses.replace(match,progress=match.progress+30))
  gc.collect()
  assert reference() is None

def test_path_memo_preserves_signed_zero_repr(monkeypatch):
  import hashlib
  from test_route_graph import straight
  tokens=[('v',(1,0,1),(1,1,1),((0.,0.),(0.,1.),(1.,1.))),
          ('v',(1,0,1),(1,1,1),((-0.,0.),(0.,1.),(1.,1.)))]
  calls=[]
  def targets(tile,nodes,edges,root,direction,edges_at,path_key):
    calls.extend(path_key(token) for token in tokens);return ()
  monkeypatch.setattr(R,'_targets',targets)
  tile=G.build_tile((straight(),),'r');match=G.find_match(tile,fix())
  R.preview_routes(tile,match)
  assert calls[:2]==['path:'+hashlib.sha256(repr(token).encode()).hexdigest() for token in tokens]

def test_inline_envelope_allowance_matches_public_formula_bitwise():
  rng=random.Random(3455)
  for _ in range(500):
    p=PointCap(rng.uniform(-10,1100),rng.uniform(0,40),rng.uniform(0,40),'a',ceiling=(rng.uniform(0,40))*(4/1.8)**0.5)
    speed=rng.uniform(0,55);accel=rng.uniform(-4,4);uncertainty=rng.uniform(0,60);travelled=rng.uniform(0,30)
    for faster in (True,False):
      limit=min(p.speed,p.ceiling)
      if not faster:limit=min(limit,p.prior)
      expected=C.approach_speed(limit,p.distance-travelled,speed,accel,uncertainty)
      actual=C.route_envelope((p,),((),),cruise=math.inf,speed=speed,acceleration=accel,uncertainty=uncertainty,travelled=travelled,allow_faster=faster)[0]
      assert bits(actual)==bits(expected)

def test_direct_point_construction_requires_exact_message_fields():
  with pytest.raises(TypeError,match='ceiling'):
    PointCap(0,10,10)
