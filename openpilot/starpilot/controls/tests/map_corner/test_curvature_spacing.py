"""Analytic geometry independent of estimator implementation."""
import math
import pytest
from openpilot.starpilot.controls.lib.map_corner import geometry as G
from openpilot.starpilot.controls.lib.map_corner.types import Way

def road(points):
  along=[0.0]
  for a,b in zip(points,points[1:]):along.append(along[-1]+math.dist(a,b))
  return G.Road(Way(1,(),False,'secondary',''),tuple(points),tuple(along))

def circle(radius,distances):
  return road([(radius*math.sin(s/radius),radius*(1-math.cos(s/radius))) for s in distances])

def legacy(r,i):
  s=r.along[i];window=max(10,min(30,(r.along[i+1]-r.along[i-1])/2))
  a,b,c=G._sample(r,s-window),r.xy[i],G._sample(r,s+window)
  den=math.dist(a,b)*math.dist(b,c)*math.dist(a,c)
  return 2*((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/den if den>1 else 0

@pytest.mark.parametrize('radius',[80,220,1000])
@pytest.mark.parametrize('before,after',[(20,20),(40,40),(60,60),(80,80),(15,60),(60,15),(30,80),(80,30)])
def test_known_circle_independent_of_sparse_or_asymmetric_nodes(radius,before,after):
  r=circle(radius,[-2*before,-before,0,after,2*after])
  assert G._sparse_curvature(r,2)==pytest.approx(1/radius,rel=1e-10)
  rr=road(tuple(reversed(r.xy)))
  assert G._sparse_curvature(rr,2)==pytest.approx(-1/radius,rel=1e-10)

@pytest.mark.parametrize('step',[.2,1,3,6,9])
def test_dense_geometry_retains_previous_noise_protection(step):
  r=road([(j*step,.3*math.sin(j*1.7)+.0001*(j*step)**2) for j in range(-60,61)])
  assert max(b-a for a,b in zip(r.along,r.along[1:]))<10
  for i in range(1,len(r.xy)-1):assert G._curvature(r,i)==legacy(r,i)

@pytest.mark.parametrize('step',[3,6,10,20])
def test_genuine_tight_circle_remains_tight(step):
  r=circle(20,[j*step for j in range(-6,7)])
  assert G._curvature(r,6)==pytest.approx(.05,rel=.10)
  assert G.seed_speed(abs(G._curvature(r,6)))<7

def test_straight_and_reversal_cannot_create_bend():
  r=road([(-100,0),(-60,0),(0,0),(20,0),(100,0)])
  assert all(G._curvature(r,i)==0 for i in range(1,4))
  rr=road(tuple(reversed(r.xy)))
  assert all(G._curvature(rr,i)==0 for i in range(1,4))

def test_sparse_s_bend_keeps_both_turn_signs():
  r=road([(-100,0),(-60,0),(0,10),(60,0),(100,0)])
  assert G._curvature(r,1)>0
  assert G._curvature(r,2)<0
  assert G._curvature(r,3)>0

def test_short_dense_endpoints_remain_finite():
  r=road([(0,0),(1,0),(2,1),(3,1)])
  assert all(math.isfinite(G._curvature(r,i)) for i in (1,2))

@pytest.mark.parametrize('sign',[1,-1])
def test_cluster_beside_sparse_chord_uses_real_support(sign):
  # The10m point lands seven metres inside a60m chord on the left.
  r=circle(1000,[-123,-63,-3,0,60,120])
  if sign<0:r=road(tuple(reversed(r.xy)))
  assert G._sparse_curvature(r,3 if sign>0 else 2)==pytest.approx(sign*.001,rel=1e-10)

def test_remote_unsupported_chord_cannot_supply_dense_side_support():
  r=circle(1000,[-203,-103,-3,0,60,120])
  assert G._sparse_curvature(r,3) is None

def test_asymmetric_vertex_noise_has_bounded_error():
  r=circle(1000,[-80,-15,0,80,160]);xy=list(r.xy);xy[2]=(0,.05);r=road(xy)
  assert abs(G._sparse_curvature(r,2)-.001)<.0001

def test_boundary_identity_and_bins_stay_parent_compatible_when_gentle_prior_changes(monkeypatch):
  import json
  from pathlib import Path
  from test_map_corner_provider import ll
  way=Way(900,tuple(ll(x,0) for x in range(0,221,20)),False,'secondary','')
  monkeypatch.setattr(G,'_legacy_curvature',lambda r,i:.002 if 2<=i<=8 else 0)
  monkeypatch.setattr(G,'_curvature',lambda r,i:.0009 if 2<=i<=8 else 0)
  tile=G.build_tile((way,),'same');corner=G._corners(tile,tile.roads[900],1)[0]
  golden=json.loads(Path(__file__).with_name('parent-boundary-golden.json').read_text())
  assert corner.key==golden['key'] and corner.revision==golden['revision']
  assert corner.entry==golden['entry'] and corner.exit==golden['exit']
  assert corner.prior==(40,40,40)


def test_gentle_entry_to_tighter_bend_keeps_all_bins_and_prior(monkeypatch):
  from test_map_corner_provider import ll
  way=Way(900,tuple(ll(x,0) for x in range(0,401,20)),False,'secondary','')
  monkeypatch.setattr(G,'_legacy_curvature',lambda r,i:.0011 if 3<=i<=5 else .008 if 6<=i<=10 else 0)
  def unexpected(*args):raise AssertionError('a tighter bend must retain its whole entry and speed profile')
  monkeypatch.setattr(G,'_curvature',unexpected)
  tile=G.build_tile((way,),'same');c=G._corners(tile,tile.roads[900],1)[0]
  assert c.entry==pytest.approx(60) and c.exit==pytest.approx(200)
  assert c.curvature==(.0011,.008,.008)
  assert c.prior==tuple(G.seed_speed(k) for k in (.0011,.008,.008))


@pytest.mark.parametrize("production_route",[False,True])
def test_sparse_corner_stored_absolute_profile_survives_new_prior_and_undo(tmp_path,monkeypatch,production_route):
  import struct
  from dataclasses import replace
  from types import SimpleNamespace
  from test_map_corner_provider import ll
  from test_route_worker import frame
  from openpilot.starpilot.controls.lib.map_corner.store import CornerStore
  from openpilot.starpilot.controls.lib.map_corner.worker import Engine
  from openpilot.starpilot.controls.lib.map_corner.types import Match
  radius=300;step=40/radius
  xy=[(x,0) for x in range(-400,1,40)]
  xy += [(radius*math.sin(j*step),radius*(1-math.cos(j*step))) for j in range(1,13)]
  end=xy[-1]
  xy += [(end[0]+s*math.cos(12*step),end[1]+s*math.sin(12*step)) for s in range(40,401,40)]
  way=Way(700,tuple(ll(*p) for p in xy),False,'secondary','')
  actual=G._curvature
  monkeypatch.setattr(G,'_curvature',legacy)
  oldtile=G.build_tile((way,),'same');old=G._corners(oldtile,oldtile.roads[700],1)[0]
  monkeypatch.setattr(G,'_curvature',actual)
  tile=G.build_tile((way,),'same');new=G._corners(tile,tile.roads[700],1)[0]
  import json
  from pathlib import Path
  golden=json.loads(Path(__file__).with_name("parent-memory-golden.json").read_text())
  assert new.key==old.key==golden["key"]
  assert new.revision==old.revision==golden["revision"]
  assert (old.entry,old.exit,old.shape)==(new.entry,new.exit,new.shape)
  assert min(new.prior)>max(old.prior)+.1
  with CornerStore(tmp_path) as store:
    store.observe(old,old.prior,'unadjusted-old-seed',False)
    original=store.conn.execute('SELECT profile FROM corners').fetchone()[0]
    assert struct.pack('<3d',*store.lookup(new).profile)==original
    # Actual worker recall must also honor the absolute stored value.
    provider=SimpleNamespace(status='Map ready',update=lambda f,n:Match(700,1,0,0,0,(new,),'r'))
    if production_route:
      from openpilot.starpilot.controls.lib.map_corner.routes import Preview
      import openpilot.starpilot.controls.lib.map_corner.routes as routes
      provider.tile=object()
      monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',1200,((),),True))
    result=Engine(provider,store).step(frame())
    assert result.status=='Remembered corner'
    assert tuple(p.speed for p in result.points)==old.prior
    store.observe(new,tuple(v-.5 for v in old.prior),'new-slower',False)
    assert store.lookup(new).profile!=old.prior
    store.undo()
    assert store.conn.execute('SELECT profile FROM corners').fetchone()[0]==original
    assert store.lookup(new).profile==old.prior
    # A later hidden corner never gets faster recall authority from the first.
    first=replace(new,key='first-corner',entry=100,apex=150,exit=200)
    store.observe(first,first.prior,'first',False)
    later=replace(new,entry=600,apex=650,exit=700,prior=tuple(v-1 for v in old.prior))
    provider.update=lambda f,n:Match(700,1,0,0,0,(first,later),'r')
    result=Engine(provider,store).step(frame())
    assert [p.speed for p in result.points if p.key==later.key]==list(later.prior)
    store.forget(new.key)
    assert store.lookup(new) is None

@pytest.mark.parametrize('delta',[-5e-8,0,5e-8])
def test_projection_roundoff_at_dense_boundary_does_not_snap_across_corner(delta):
  r=road([(-20,0),(-10,0),(0,0),(0,10+delta),(0,20+delta)])
  # At the first point after the turn, floating error must not look a whole
  # extra segment back through the junction and invent another sharp target.
  assert abs(G._curvature(r,3))<1e-7


@pytest.mark.parametrize('delta',[-5e-8,0,5e-8])
def test_exact_support_vertex_ignores_unused_long_chord_and_reverses(delta):
  r=road([(-100,0),(-10+delta,0),(0,0),(10,1),(60,5)])
  rr=road(tuple(reversed(r.xy)))
  value=G._sparse_curvature(r,2)
  assert value is not None and value>0
  assert value==pytest.approx(-G._sparse_curvature(rr,2),abs=1e-9)
  assert G._curvature(r,2)==pytest.approx(-G._curvature(rr,2),abs=1e-9)


def legacy_radius_vertex(radius):
  theta=2*math.asin(30/(2*radius))
  return road([(-120,0),(-60,0),(0,0),(60*math.cos(theta),60*math.sin(theta)),(120*math.cos(theta),120*math.sin(theta))])


@pytest.mark.parametrize('radius',[20,40,80,120,150,199.9,200])
def test_production_retains_existing_tight_bend_estimate(radius):
  r=legacy_radius_vertex(radius)
  assert G._curvature(r,2)==pytest.approx(legacy(r,2),rel=1e-14)


@pytest.mark.parametrize('radius',[350,406,548,1000])
def test_production_corrects_gentle_bends_fully(radius):
  r=legacy_radius_vertex(radius)
  assert G._curvature(r,2)==pytest.approx(1/(2*radius),rel=1e-12)


@pytest.mark.parametrize('threshold',[200,350])
def test_production_transition_has_no_speed_step(threshold):
  radii=[threshold-1e-4,threshold,threshold+1e-4]
  values=[G.seed_speed(abs(G._curvature(legacy_radius_vertex(r),2))) for r in radii]
  assert max(values)-min(values)<1e-4
  assert values==sorted(values)


def test_transition_stays_between_parent_and_geometric_estimates():
  for radius in range(201,350):
    r=legacy_radius_vertex(radius)
    value=G._curvature(r,2)
    assert 1/(2*radius)-1e-14<=value<=legacy(r,2)


def test_sparse_right_angle_retains_parent_speed_in_production():
  r=road([(-100,0),(-50,0),(0,0),(0,50),(0,100)])
  assert 1/abs(G._sparse_curvature(r,2))==pytest.approx(50/math.sqrt(2))
  assert 1/abs(legacy(r,2))==pytest.approx(30/math.sqrt(2))
  assert G._curvature(r,2)==legacy(r,2)


@pytest.mark.parametrize('value',[None,-.001,.01])
def test_unsupported_opposite_or_sharper_geometry_keeps_parent(monkeypatch,value):
  r=legacy_radius_vertex(250)
  monkeypatch.setattr(G,'_sparse_curvature',lambda r,i:value)
  assert G._curvature(r,2)==legacy(r,2)


def test_supported_collinear_relaxation_is_distinct_from_missing_support(monkeypatch):
  r=legacy_radius_vertex(400)
  monkeypatch.setattr(G,'_sparse_curvature',lambda r,i:0.0)
  assert G._curvature(r,2)==0


def test_tight_geometry_does_not_consult_sparse_estimator(monkeypatch):
  def unexpected(*args):raise AssertionError('tight geometry must retain parent structurally')
  monkeypatch.setattr(G,'_sparse_curvature',unexpected)
  r=legacy_radius_vertex(150)
  assert G._curvature(r,2)==legacy(r,2)


def test_remote_unsupported_support_keeps_existing_production_target():
  r=circle(1000,[-203,-103,-3,0,60,120])
  assert G._sparse_curvature(r,3) is None
  assert G._curvature(r,3)==legacy(r,3)>0



def test_recalibration_cannot_admit_previously_late_target(monkeypatch):
  from types import SimpleNamespace
  from test_route_worker import frame
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  from openpilot.starpilot.controls.lib.map_corner.types import PointCap,Match
  from openpilot.starpilot.controls.lib.map_corner.worker import Engine
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  # Corrected target is initially comfortable; original target is already late.
  p=PointCap(50,24,24,'curve',ceiling=35,admission_speed=12)
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((p,),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  result=Engine(provider).step(frame(speed=25,acceleration=-.6))
  assert result.route_options==((),)
  assert result.status=='Corner acquired too late'


def test_admission_metadata_never_commands_the_old_slower_speed():
  from openpilot.starpilot.controls.lib.map_corner.types import PointCap
  from openpilot.starpilot.controls.lib.map_corner.controller import route_envelope
  p=PointCap(0,24,24,'curve',ceiling=35,admission_speed=12)
  kw=dict(cruise=30,speed=25,acceleration=-.6,uncertainty=0)
  assert route_envelope((p,),(),**kw)[0]==24
  assert route_envelope((p,),(),admission=True,**kw)[0]==12


def test_metadata_preserves_anticipation_and_publishes_corrected_target(monkeypatch):
  from types import SimpleNamespace
  from test_route_worker import frame
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  from openpilot.starpilot.controls.lib.map_corner.types import PointCap,Match
  from openpilot.starpilot.controls.lib.map_corner.worker import Engine
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=PointCap(1000,24,24,'curve',ceiling=35,admission_speed=12)
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((p,),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);r=engine.step(frame(speed=25,acceleration=-.6))
  assert r.route_options[0][0].speed==24 and r.route_options[0][0].admission_speed==12
  assert engine.previous_safe
  r=engine.step(frame(1.05,speed=25,acceleration=-.6,odometer=1.25))
  assert r.route_options[0][0].speed==24



def test_actual_preview_and_worker_transport_retain_parent_admission_prior():
  import pickle
  from types import SimpleNamespace
  from test_map_corner_provider import ll,fix
  from test_route_worker import frame
  from openpilot.starpilot.controls.lib.map_corner.routes import preview_routes
  from openpilot.starpilot.controls.lib.map_corner.worker import Engine
  radius=300;step=40/radius
  xy=[(x,0) for x in range(-400,1,40)]
  xy += [(radius*math.sin(j*step),radius*(1-math.cos(j*step))) for j in range(1,13)]
  end=xy[-1]
  xy += [(end[0]+s*math.cos(12*step),end[1]+s*math.sin(12*step)) for s in range(40,401,40)]
  way=Way(700,tuple(ll(*p) for p in xy),False,'secondary','')
  tile=G.build_tile((way,),'transport');match=G.find_match(tile,fix(x=-350))
  assert match is not None and match.corners
  preview=preview_routes(tile,match)
  points=[p for path in preview.options for p in path]
  assert points and any(p.speed>p.admission_speed>0 for p in points)
  provider=SimpleNamespace(tile=tile,status='Map ready',update=lambda f,n:match)
  result=Engine(provider).step(frame())
  assert result.points and any(p.speed>p.admission_speed>0 for p in result.points)
  restored=pickle.loads(pickle.dumps(result))
  assert restored==result
  assert [p.admission_speed for p in restored.points]==[p.admission_speed for p in result.points]
