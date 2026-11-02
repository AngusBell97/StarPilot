"""Global calibration and independently specified geometry/physical safeguards."""
import math
import hashlib
import importlib.util
from pathlib import Path
import sys
from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.controls.lib.map_corner import geometry as G
from openpilot.starpilot.controls.lib.map_corner.controller import route_envelope
from openpilot.starpilot.controls.lib.map_corner.worker import visual_agrees
from openpilot.starpilot.controls.lib.map_corner.types import Way, MPH

BASE=Path(__file__).resolve().parent/'fixtures/seed'
spec=importlib.util.spec_from_file_location('seed_old',BASE/'__init__.py',submodule_search_locations=[str(BASE)])
package=importlib.util.module_from_spec(spec);sys.modules[spec.name]=package;spec.loader.exec_module(package)
from seed_old import geometry as OLD_G, routes as OLD_R, store as OLD_S

def arc(radius):
  xy=[(x,0) for x in range(-150,1,10)]
  step=2*math.asin(10/(2*radius));count=math.ceil(math.pi/(2*step))
  xy += [(radius*math.sin(i*step),radius*(1-math.cos(i*step))) for i in range(1,count+1)]
  end=xy[-1];angle=count*step
  xy += [(end[0]+y*math.cos(angle),end[1]+y*math.sin(angle)) for y in range(10,151,10)]
  points=tuple((51+y/111195,-.1+x/(111195*math.cos(math.radians(51)))) for x,y in xy)
  return Way(700,points,False,'secondary','')

def corner(radius):
  tile=G.build_tile((arc(radius),),'calibration')
  return G._corners(tile,tile.roads[700],1)[0]

@pytest.mark.parametrize('radius,low,high',[(20,14.0,14.3),(77.5656,33.9,34.1),(220,46.8,47.1)])
def test_general_seed_uplift_in_tight_medium_and_gentle_bends(radius,low,high):
  # Fails if only the reported bend or only medium speeds receive the uplift.
  v=min(corner(radius).prior)/MPH
  assert low < v < high

def test_faster_memory_ceiling_is_independent_of_comfort_seed():
  # Actual curvature0.02/m means physical4m/s2 ceiling sqrt(200), regardless
  # of a higher comfort prior. Old inference would wrongly allow >20m/s.
  p=SimpleNamespace(distance=0,speed=30,prior=34*MPH,ceiling=math.sqrt(200),key='c')
  value=route_envelope((p,),((),),cruise=40,speed=15,acceleration=-.6,uncertainty=0,allow_faster=True)[0]
  assert value==pytest.approx(14.1421356237)

@pytest.mark.parametrize('visual_k,expected',[(.007,False),(.03,True),(-.02,False)])
def test_visual_agreement_uses_geometry_not_new_seed(visual_k,expected):
  c=SimpleNamespace(apex=100,prior=(34*MPH,)*3,curvature=(.02,)*3,turn=1)
  frame=SimpleNamespace(visual_valid=True,visual_profile=((50,visual_k,True),))
  assert visual_agrees(frame,c,50) is expected

def test_corner_carries_physical_curvature_independent_of_prior():
  c=corner(77.5656)
  assert getattr(c,'curvature',None)==pytest.approx((1/77.5656,)*3,rel=.03)

def test_tight_to_gentle_seed_is_monotonic_and_below_physical_ceiling():
  previous=0
  for radius in (10,15,20,30,40,50,60,77.5656,90,110,140,180,220,300,500):
    v=min(corner(radius).prior)
    assert v>=previous-1e-4
    assert v<=math.sqrt(4*radius)+.02
    previous=v

@pytest.mark.parametrize('delta',[-1,0,1])
def test_old_database_absolute_profiles_survive_recalibration_learning_and_undo(tmp_path,delta):
  from openpilot.starpilot.controls.lib.map_corner.store import CornerStore
  old_tile=OLD_G.build_tile((arc(77.5656),),'old')
  old=OLD_G._corners(old_tile,old_tile.roads[700],1)[0];new=corner(77.5656)
  assert new.key==old.key and new.revision==old.revision
  with OLD_S.CornerStore(tmp_path) as store:
    for i in range(3):store.observe(old,tuple(v+delta for v in old.prior),str(i),delta>0)
    original=store.lookup(old);stats=store.stats()
  with CornerStore(tmp_path) as store:
    before=hashlib.sha256(store.path.read_bytes()).hexdigest()
    assert store.lookup(new).profile==original.profile
    assert store.lookup(new).count==original.count
    assert hashlib.sha256(store.path.read_bytes()).hexdigest()==before
    assert store.stats()['observations']==stats['observations']
    assert min(new.prior)>max(old.prior)
    target=tuple(v-.5 for v in original.profile)
    store.observe(new,target,'after-calibration',False)
    assert store.lookup(new).profile==pytest.approx(target)
    store.undo()
    assert store.lookup(new).profile==original.profile
    assert store.lookup(new).count==original.count
    store.forget(new.key)
    assert store.lookup(new) is None

def test_junction_targets_are_bitwise_unchanged_and_carry_prior_only_ceiling():
  from test_map_corner_provider import ll,fix
  from openpilot.starpilot.controls.lib.map_corner.routes import preview_routes
  ways=(Way(1,tuple(ll(x,0) for x in range(-400,1,10)),False,'secondary',''),
        Way(2,tuple(ll(0,y) for y in range(0,201,10)),False,'secondary',''),
        Way(3,tuple(ll(0,-y) for y in range(0,201,10)),False,'secondary',''))
  old=OLD_G.build_tile(ways,'same');new=G.build_tile(ways,'same')
  a=OLD_R.preview_routes(old,OLD_G.find_match(old,fix()))
  b=preview_routes(new,G.find_match(new,fix()))
  def values(preview):return [[(p.distance.hex(),p.speed.hex(),p.prior.hex(),p.key) for p in path] for path in preview.options]
  assert values(a)==values(b)
  assert all(p.ceiling==p.speed for path in b.options for p in path)

def test_point_ceilings_survive_route_cache_render_worker_and_extrapolation():
  from test_map_corner_provider import fix
  from openpilot.starpilot.controls.lib.map_corner.routes import preview_routes
  from openpilot.starpilot.controls.lib.map_corner.worker import Engine
  from openpilot.starpilot.controls.lib.map_corner.types import Frame
  tile=G.build_tile((arc(77.5656),),'test');match=G.find_match(tile,fix())
  preview=preview_routes(tile,match)
  assert preview is preview_routes(tile,match)
  points=[p for path in preview.options for p in path]
  assert points and all(p.ceiling>=p.prior for p in points)
  for has_tile in (False,True):
    provider=SimpleNamespace(status='Map ready',update=lambda f,n:match)
    if has_tile:provider.tile=tile
    f=Frame(1,1,0,0,fix(.9),5,0,0,True,True,'other',False,'',False,False,False,False)
    result=Engine(provider).step(f)
    active=result.points+tuple(p for path in result.route_options for p in path)
    assert active
    assert all(p.ceiling>=p.prior for p in active)
    assert min(p.ceiling for p in active)==pytest.approx(17.614,abs=.02)

def test_worker_queue_serialization_preserves_geometry_and_physical_authority():
  import pickle
  from multiprocessing.reduction import ForkingPickler
  from openpilot.starpilot.controls.lib.map_corner.types import PointCap,Result
  c=corner(77.5656)
  p=PointCap(0,40,c.prior[1],c.key,ceiling=G.physical_speed(c.curvature[1]))
  restored,result=pickle.loads(ForkingPickler.dumps((c,Result(points=(p,)))))
  assert restored.curvature==c.curvature
  f=SimpleNamespace(visual_valid=True,visual_profile=((50,.02,True),))
  assert visual_agrees(f,restored,restored.apex-50)
  speed=route_envelope(result.points,((),),cruise=40,speed=15,acceleration=-.6,uncertainty=0,allow_faster=True)[0]
  assert speed==pytest.approx(17.614,abs=.02)
