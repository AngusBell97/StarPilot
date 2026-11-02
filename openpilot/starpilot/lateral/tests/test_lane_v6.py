from dataclasses import replace
import numpy as np
import pytest
from openpilot.starpilot.lateral.tests.test_lane_v5 import scene, obs
from openpilot.starpilot.lateral.lane_v6 import LaneCenteringV6Controller, select_sections
from openpilot.starpilot.lateral.lane_v5 import LaneCenteringV5Controller
from openpilot.starpilot.lateral.lane_path_v6 import (build_path, gain_normalization, _local_basis,
 _basis, _interval_support, _knots, _matrices)
from openpilot.starpilot.lateral.lane_versions import LaneCenteringVersions
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan


def controller(): return LaneCenteringV6Controller(clock=lambda: 0.)

def widening(start=1., end=1.5, narrow=False):
 m=scene(.2);t=np.array(m.position.t)
 half=np.where((t>=start)&(t<=end), .5 if narrow else 3., 1.8)
 m.laneLines[1].y=(.2-half).tolist();m.laneLines[2].y=(.2+half).tolist()
 return m


def test_valid_road_retains_v5_output():
 a,b=controller(),LaneCenteringV5Controller(clock=lambda:0.)
 for i in range(35):
  o=obs(i);ra,rb=a.update(o),b.update(o)
  assert ra.reason==rb.reason=='qualified'
  assert ra.correction==pytest.approx(rb.correction,abs=1e-12)

@pytest.mark.parametrize('narrow',[False,True])
def test_retain_valid_preview_after_rejected_section(narrow):
 c=controller();o=obs(model=widening(narrow=narrow),settings=replace(obs().settings,mode3=False));c.update(o)
 p=c.planned_path
 assert p is not None and p.t[-1]==3.
 modes=np.array(c.diagnostic['v6_regions'])
 assert np.any(modes==0) and modes[-1]==2
 assert np.max(np.abs((p.target_y-p.model_y)[modes==0]))<1e-8
 assert c.diagnostic['v6_modes'][-1]==1


def test_wide_near_model_release_keeps_future_plan():
 c=controller();c._correction=.001
 r=c.update(obs(model=widening(0.,1.),settings=replace(obs().settings,mode3=False)))
 assert r.reason=='v6_model_section'
 assert r.correction==pytest.approx(.001*np.exp(-.01/.2))
 assert c.planned_path is not None and c.diagnostic['v6_modes'][-1]==1

@pytest.mark.parametrize('side',[0,1])
def test_single_boundary_away_only(side):
 m=scene();m.laneLineProbs=[0,0,0,0];m.roadEdgeStds=[2,2]
 m.laneLineProbs[side+1]=.9
 m.laneLines[side+1].y=np.full(33,-1. if side==0 else 1.).tolist()
 c=controller();r=c.update(obs(model=m))
 assert r.reason=='qualified'
 assert r.correction*(1 if side==0 else -1)>0
 assert np.all(np.array(c.diagnostic['v6_modes'])==3)


def test_clear_single_never_attracts():
 c=controller();c.update(obs(model=scene(single=1)))
 assert c._raw is None or c._raw==0
 assert np.max(np.abs(c.planned_path.target_y-c.planned_path.model_y))<1e-10


def test_two_line_wide_veto_and_mixed_tie():
 s=obs().settings;raw=np.array([[-3.], [3.],[-1.4],[1.4]])
 mode,_,_=select_sections(raw,np.array([0.]),s)
 assert mode[0]==3
 raw=np.array([[-1.4],[1.4],[-1.4],[1.4]])
 mode,ids,_=select_sections(raw,np.array([0.]),replace(s,mode1=False))
 assert mode[0]==2 and tuple(ids[:,0])==(0,3)


def test_crossed_and_single_conflict():
 c=controller();m=scene();m.laneLines[1].y=np.full(33,1.).tolist();m.laneLines[2].y=np.full(33,-1.).tolist()
 c.update(obs(model=m));assert not np.any(c.diagnostic['v6_modes'])


def test_no_far_extrapolation():
 c=controller();m=scene();m.laneLines[1].x=np.linspace(0,8,33).tolist()
 r=c.update(obs(model=m));assert r.reason=='v6_short_preview' and c.planned_path is None


def test_filtered_invalid_pair_does_not_change_raw_mode():
 c=controller();s=replace(obs().settings,smoothing_tau=1.,smoothing_distance=.5)
 c.update(obs(0,model=scene(),settings=s))
 m=scene();m.laneLines[1].y=np.full(33,-3.).tolist();m.laneLines[2].y=np.full(33,3.).tolist()
 c.update(obs(5,model=m,settings=s))
 assert all(v==3 for v in c.diagnostic['v6_modes'])


def test_repeated_ticks_do_not_refit_and_signal_clears():
 c=controller();o=obs();c.update(o);p=c.planned_path
 c.update(obs(1));assert c.planned_path is p
 c.update(obs(2,turn_signal_active=True));assert c.planned_path is None and c._profiles==[None,None]


def test_live_version_handover_keeps_scalar_not_profiles():
 c=LaneCenteringVersions();c.v5.clock=lambda:0.;c.v6.clock=lambda:0.
 for i in range(10):c.update(obs(i),version=5,motion=obs(i).motion)
 prior=c._correction
 r=c.update(obs(10),version=6,motion=obs(10).motion)
 assert c.version==6 and c.active is c.v6 and c.v5._correction==0
 assert abs(r.correction-prior)<.0002


def test_bad_motion_and_override_clear():
 for option in ('driver_override','motion'):
  c=controller();c.update(obs())
  kw={'driver_override':True} if option=='driver_override' else {'motion':replace(obs(1).motion,base_supported=False)}
  c.update(obs(1,**kw));assert c.target_path is None


def test_interval_zero_and_opposite_single_regions():
 t=np.arange(61)/20;regions=np.full(61,2);regions[22:30]=0;regions[40:]= -1
 p=build_path(frame=1,t=t,x=15*t,model_y=np.zeros(61),desired_offset=np.where(t<1.5,.3,-.3),
  lower=np.where(regions==0,0.,-.5),upper=np.where(regions==0,0.,.5),speed=15.,action_delay=.31,
  base_curvature=0.,cap=.0018,converter=get_curvature_from_plan,regions=regions)
 assert np.max(np.abs(p.target_y[regions==0]))<1e-8
 assert np.max(p.target_y[regions==-1])<1e-8


def test_budget_rejects_and_records_cpu_on_early_exit():
 values=iter([0.,.031,.032]);c=LaneCenteringV6Controller(clock=lambda:next(values))
 r=c.update(obs());assert r.reason=='v6_compute_budget' and c.target_path is None
 assert c.diagnostic['v6_cpu_s']>=0
 c=controller();r=c.update(obs(model=scene(),motion=None));assert c.diagnostic['algorithm']==6


def test_canonical_one_second_endpoint():
 c=controller();s=replace(obs().settings,preview_time=1.)
 c.update(obs(settings=s));assert c.planned_path.t[-1]==1. and len(c.planned_path.t)==21


def test_short_terminal_return_is_suppressed():
 c=controller();c.update(obs(model=widening(1.,2.7),settings=replace(obs().settings,mode3=False)))
 assert c.planned_path is not None
 assert c.diagnostic['v6_regions'][-1]==0
 assert abs(c.planned_path.target_y[-1]-c.planned_path.model_y[-1])<1e-9


def test_original_knot_narrowing_between_samples_is_not_skipped():
 m=scene();xx=np.array(m.laneLines[1].x);xx[10]=15.375
 xx.sort();m.laneLines[1].x=xx.tolist();m.laneLines[2].x=xx.tolist()
 yy=np.full(33,-1.8);zz=np.full(33,1.8);yy[10]=-.1;zz[10]=.1
 m.laneLines[1].y=yy.tolist();m.laneLines[2].y=zz.tolist()
 c=controller();c.update(obs(model=m))
 at=np.searchsorted(c.planned_path.x,xx[10])
 assert c.diagnostic['v6_regions'][at]==0 and c.diagnostic['v6_regions'][at-1]==0


def test_raw_selector_matches_v4_constant_geometry():
 from openpilot.starpilot.lateral.lane_centering import LaneCenteringController
 rng=np.random.default_rng(16)
 for _ in range(120):
  values=rng.uniform(-4.,4.,4);accepted=rng.random(4)>.2
  m=scene();m.roadEdgeStds=[.1 if accepted[2] else 2.,.1 if accepted[3] else 2.]
  m.laneLineProbs=[0,.9 if accepted[0] else 0,.9 if accepted[1] else 0,0]
  for curve,value in zip([m.laneLines[1],m.laneLines[2],*m.roadEdges],values):curve.y=np.full(33,value).tolist()
  o=obs(model=m);v4=LaneCenteringController();v4._frame_correction(o)
  raw=values.copy();raw[~accepted]=np.array([-np.inf,np.inf,-np.inf,np.inf])[~accepted]
  mode,ids,_=select_sections(raw[:,None],np.zeros(1),o.settings)
  assert int(mode[0])==v4.selected_mode
  expected=tuple((side,('line' if idx<2 else 'edge')) for side,idx in enumerate(ids[:,0]) if idx>=0)
  assert expected==v4.selected_sources


@pytest.mark.parametrize('count',[21,31,45,61,69,81])
def test_dense_interval_zero_and_sign_guarantees(count):
 from openpilot.starpilot.lateral.lane_path_v6 import _matrices, _basis
 t=np.arange(count)/20;h=t[-1];regions=np.full(count,2);regions[count//3:count//2]=0;regions[3*count//4:]=-1
 p=build_path(frame=1,t=t,x=15*t,model_y=np.zeros(count),desired_offset=np.where(t<h/2,.3,-.3),
  lower=np.where(regions==0,0.,-.5),upper=np.where(regions==0,0.,.5),speed=15.,action_delay=.31,
  base_curvature=0.,cap=.0018,converter=get_curvature_from_plan,regions=regions)
 b=_matrices(h,count,False,h>2.5)[1];c=np.linalg.lstsq(b,p.target_y,rcond=None)[0]
 dense=np.arange(0,h,.001);knots=_knots(h,h>2.5)
 offset=_basis(dense,knots,3)[:,3:]@c
 gap=(dense>=t[count//3])&(dense<=t[count//2-1])
 assert np.max(np.abs(offset[gap]))<1e-8
 assert np.max(offset[dense>=t[3*count//4]])<1e-8


def test_converter_does_not_read_beyond_delay_window():
 t=np.arange(61)/20;hd=np.zeros(61);rd=np.zeros(61)
 baseline=get_curvature_from_plan(hd,rd,t,15.,.31)
 hd[t>.4]=1.;rd[t>.4]=2.
 assert get_curvature_from_plan(hd,rd,t,15.,.31)==baseline


def test_near_starting_accepted_edge_cannot_disappear():
 m=scene();m.roadEdgeStds[0]=.1;m.roadEdges[0].x=(np.array(m.roadEdges[0].x)+1.).tolist()
 c=controller();r=c.update(obs(model=m))
 assert r.reason=='v6_near_support' and c.planned_path is None


def test_real_pair_to_mixed_transition_is_not_a_model_hole():
 m=scene();t=np.array(m.position.t)
 # Line pair becomes too narrow, but opposite edge supplies a valid mixed pair.
 # Use narrower paired-clearance requirement than single to expose real selection.
 m.roadEdgeStds[1]=.1;m.roadEdges[1].y=np.full(33,2.).tolist()
 m.laneLines[2].y=np.where(t<1.4,1.8,-.2).tolist()
 # Nearest boundary still prevents paired mode where too narrow; instead make
 # selected pair transition through the existing mixed narrower tie, with mode1 off.
 m.laneLines[2].y=np.full(33,1.8).tolist();m.roadEdgeStds[0]=.1
 m.roadEdges[0].y=np.where(t<1.4,-2.,-1.7).tolist()
 s=replace(obs().settings,mode1=False)
 c=controller();c.update(obs(model=m,settings=s))
 assert all(v==2 for v in c.diagnostic['v6_modes'])
 assert all(v==2 for v in c.diagnostic['v6_regions'])

@pytest.mark.parametrize('delay',[.2,.3,1.2,1.5])
def test_gate_covers_converter_delay(delay):
 o=obs(model=widening(.7,1.5,narrow=True));o=replace(o,motion=replace(o.motion,delay=delay))
 c=controller();r=c.update(o)
 assert r.reason=='v6_model_section' and c._raw is None

@pytest.mark.parametrize('field',['boundary','std','z'])
def test_nonfinite_geometry_fails_closed(field):
 m=scene()
 if field=='boundary':m.laneLines[1].y[5]=float('nan')
 if field=='std':m.position.yStd[5]=float('inf')
 if field=='z':m.position.z[5]=float('nan')
 c=controller();r=c.update(obs(model=m))
 assert np.isfinite(r.correction)
 if field=='z':assert r.reason=='v6_time_geometry'


@pytest.mark.parametrize('side',[0,1])
@pytest.mark.parametrize('pattern',['clear_near','clear_far','clear_middle','brief_need'])
def test_valid_single_clearance_sections_retain_v5_output(side,pattern):
 a,b=controller(),LaneCenteringV5Controller(clock=lambda:0.)
 m=scene();t=np.array(m.position.t)
 m.laneLineProbs=[0,0,0,0];m.roadEdgeStds=[2,2];m.laneLineProbs[side+1]=.9
 need={'clear_near':t>.5,'clear_far':t<1.5,'clear_middle':(t<.7)|(t>1.4),'brief_need':(t>1.3)&(t<1.8)}[pattern]
 clearance=obs().settings.car_width/2+obs().settings.single_clearance
 m.laneLines[side+1].y=((1 if side else -1)*np.where(need,clearance-.25,clearance+.1)).tolist()
 for i in range(25):
  o=obs(i,model=m);ra,rb=a.update(o),b.update(o)
  assert ra.reason==rb.reason=='qualified'
  assert ra.correction==pytest.approx(rb.correction,abs=1e-10)
 assert not any(v==0 for v in a.diagnostic['v6_regions'])


def test_valid_fully_attenuated_pair_does_not_create_geometry_gap():
 s=replace(obs().settings,e2e_authority=1.,curve_adaptation=False)
 m=scene(.7);m.position.yStd=np.full(33,.1).tolist()
 a,b=controller(),LaneCenteringV5Controller(clock=lambda:0.)
 for i in range(15):
  o=obs(i,model=m,settings=s);ra,rb=a.update(o),b.update(o)
  assert ra.reason==rb.reason=='qualified'
  assert ra.correction==pytest.approx(rb.correction,abs=1e-10)
 assert all(v==2 for v in a.diagnostic['v6_regions'])


def test_wide_clear_single_is_valid_zero_demand_not_rejected_geometry():
 c=controller();c.update(obs(model=widening()))
 modes=np.array(c.diagnostic['v6_modes']);regions=np.array(c.diagnostic['v6_regions'])
 assert np.any(modes==3) and not np.any(regions==0)
 assert np.max(np.abs((c.planned_path.target_y-c.planned_path.model_y)[modes==3]))<1e-8


@pytest.mark.parametrize('side',[0,1])
def test_single_rejected_direction_clears_profiles(monkeypatch,side):
 import openpilot.starpilot.lateral.lane_v6 as module
 original=module.build_path
 def contrary(**kwargs):
  p=original(**kwargs)
  return replace(p,correction=-.0001 if side==0 else .0001)
 monkeypatch.setattr(module,'build_path',contrary)
 m=scene();m.laneLineProbs=[0,0,0,0];m.laneLineProbs[side+1]=.9;m.roadEdgeStds=[2,2]
 m.laneLines[side+1].y=np.full(33,-1. if side==0 else 1.).tolist()
 c=controller();r=c.update(obs(model=m))
 assert r.reason=='v6_single_direction' and c._profiles==[None,None]


@pytest.mark.parametrize('side',[0,1])
def test_single_line_edge_same_source_choice_retains_v5(side):
 m=scene();m.laneLineProbs=[0,0,0,0];m.laneLineProbs[side+1]=.9;m.roadEdgeStds=[2,2];m.roadEdgeStds[side]=.2
 m.laneLines[side+1].y=np.full(33,-1. if side==0 else 1.).tolist()
 m.roadEdges[side].y=np.full(33,-1.1 if side==0 else 1.1).tolist()
 a,b=controller(),LaneCenteringV5Controller(clock=lambda:0.)
 for i in range(25):
  ra,rb=a.update(obs(i,model=m)),b.update(obs(i,model=m))
  assert ra.reason==rb.reason=='qualified'
  assert ra.correction==pytest.approx(rb.correction,abs=1e-10)


def test_clear_single_next_to_invalid_span_keeps_dense_zero():
 from openpilot.starpilot.lateral.lane_path_v6 import _matrices,_basis
 t=np.arange(61)/20;regions=np.ones(61,dtype=int);regions[25:35]=0
 lower=np.zeros(61);upper=np.full(61,.5);upper[:8]=0.;upper[regions==0]=0.
 p=build_path(frame=1,t=t,x=15*t,model_y=np.zeros(61),desired_offset=np.where(t<.4,0.,.2),
  lower=lower,upper=upper,speed=15.,action_delay=.31,base_curvature=0.,cap=.0018,
  converter=get_curvature_from_plan,near_priority=True,regions=regions)
 b=_matrices(3.,61,True,False)[1];c=np.linalg.lstsq(b,p.target_y,rcond=None)[0]
 dense=np.arange(0,3.,.001);knots=_knots(3.,False)
 offset=_basis(dense,knots,3)[:,3:]@c
 assert np.max(abs(offset[(dense>=t[25])&(dense<=t[34])]))<1e-8
 assert np.min(offset)>-1e-8


@pytest.mark.parametrize('side',[0,1])
def test_pair_to_clear_single_retains_new_geometry_constraints(side):
 m=scene(-.2 if side==0 else .2);t=np.array(m.position.t)
 # Pair near the car requests toward the side that becomes the sole boundary.
 # This is intentionally different eligibility/horizon from truncated v5.
 curve=m.laneLines[2 if side==0 else 1]
 curve.y=np.where(t<1.5,np.array(curve.y),6. if side==0 else -6.).tolist()
 c=controller();c.update(obs(model=m));regions=np.array(c.diagnostic['v6_regions'])
 assert np.any(regions==2) and np.any(regions==(1 if side==0 else -1))
 assert not np.any(regions==0)
 single=np.array(c.diagnostic['v6_modes'])==3
 offset=c.planned_path.target_y-c.planned_path.model_y
 assert np.max(np.abs(offset[single]))<1e-8


HORIZONS=[n/20 for n in range(20,81)]

def straight(t,desired,lower,upper,cap=.0018,**kw):
 return build_path(frame=1,t=t,x=15*t,model_y=np.zeros(len(t)),desired_offset=desired,lower=lower,upper=upper,
  speed=15.,action_delay=.31,base_curvature=0.,cap=cap,converter=get_curvature_from_plan,**kw)


@pytest.mark.parametrize('h',HORIZONS)
def test_knot_rule_and_real_basis_invariants(h):
 knots=_knots(h,True);inner=knots[4:-4];n=round(h*20)+1
 assert len(inner)==9 and knots[:4].tolist()==[0.]*4 and knots[-4:].tolist()==[h]*4
 assert np.all(np.diff(inner)>0) and inner[-1]<h
 if h<=2.5:
  assert np.array_equal(knots,_knots(h,False))
  assert all(np.array_equal(a,b) for a,b in zip(_matrices(h,n,False,True),_matrices(h,n,False,False)))
 else:
  assert inner[:6].tolist()==[.25,.5,.75,1.,1.25,1.5]
  assert np.allclose(inner[6:],1.5+(h-1.5)*np.array([.25,.5,.75]),rtol=0.,atol=1e-12)
 t,b,d,dd,_,_=_matrices(h,n,False,True)
 assert b.shape==(n,10) and np.max(np.abs(np.r_[b[0],d[0],dd[0]]))<1e-12
 assert b[-1,-1]==pytest.approx(1.,abs=1e-9) and np.max(np.abs(b[-1,:-1]))<1e-9
 assert d[-1,-1]==pytest.approx(3./(h-inner[-1]),rel=1e-6)
 dense=np.arange(0,h,.001);full=_basis(dense,knots,3)
 assert np.allclose(full.sum(axis=1),1.,rtol=0.,atol=1e-12)
 for j in range(3,13):
  lo,hi=knots[j],knots[j+4]
  assert np.all(full[(dense<=lo)|(dense>=hi),j]==0.) and np.all(full[(dense>lo+1e-6)&(dense<hi-1e-6),j]>0.)
 for k in inner:
  for order in range(3):
   v=_basis(np.array([k-1e-10,k+1e-10]),knots,3,order)
   assert np.allclose(v[0],v[1],rtol=0.,atol=1e-5)
 st,overlap=_interval_support(h,n,True);mid=(st[:-1]+st[1:])/2
 assert np.array_equal(st,np.arange(n)/20)
 # Tail knots need not coincide with stations; an interval midpoint alone can
 # miss a support that occupies only part of that interval. Ignore ulp slivers.
 assert not np.any((_basis(mid,knots,3)[:,3:].T>1e-12)&~overlap)
 for j in range(10):
  lo=np.maximum(st[:-1],knots[j+3]);hi=np.minimum(st[1:],knots[j+7])
  active=hi-lo>1e-10
  assert np.all(overlap[j,active])
  assert np.all(_basis((lo[active]+hi[active])/2,knots,3)[:,j+3]>0.)
  assert not np.any(overlap[j,hi<lo-1e-10])


def test_knot_grid_is_continuous_at_horizon_gate():
 at=_knots(2.5,True)
 assert np.array_equal(at,_knots(2.5,False))
 assert np.max(np.abs(_knots(float(np.nextafter(2.5,3.)),True)-at))<1e-12


@pytest.mark.parametrize('h,count',[(1.,30),(2.2,37),(3.4,41)])
def test_noncanonical_grid_matches_cached_support(h,count):
 exact=np.linspace(0.,h,count);jitter=exact.copy();jitter[1:-1]+=1e-11
 regions=np.full(count,2);regions[count//2:count//2+2]=0;regions[3*count//4:]=1
 lower=np.where(regions==0,0.,-.5);upper=np.where(regions==0,0.,.5);desired=np.where(exact<h/3,.01,-.01)
 a,b=(straight(v,desired,lower,upper,cap=.01,regions=regions) for v in (exact,jitter))
 assert a.cap_scale==b.cap_scale
 assert np.max(np.abs(a.target_y-b.target_y))<1e-9 and np.max(np.abs(a.target_y[regions==0]))<1e-8


@pytest.mark.parametrize('near',[False,True])
def test_gain_normalization_calibrates_every_horizon(near):
 for h in HORIZONS:
  n=round(h*20)+1;t=np.arange(n)/20
  gain=gain_normalization(h,n,15.,.31,get_curvature_from_plan,near)
  p=straight(t,np.full(n,1e-4),np.full(n,-.5),np.full(n,.5),cap=.01,near_priority=near)
  assert np.isfinite(gain) and gain>0 and p.cap_scale==1.
  assert gain*p.correction==pytest.approx(1e-4*2/15.**2,rel=1e-3)


@pytest.mark.parametrize('h',[1.,1.1,1.15,2.25,3.2,3.4,4.])
@pytest.mark.parametrize('away',[-1,1])
def test_dense_away_only_between_stations(h,away):
 n=round(h*20)+1;t=np.arange(n)/20
 p=straight(t,away*np.where(t<h/2,.3,-.3),np.full(n,-.5),np.full(n,.5),away=away)
 c=np.linalg.lstsq(_matrices(h,n,False)[1],p.target_y,rcond=None)[0]
 assert np.min(away*(_basis(np.arange(0,h,.001),_knots(h),3)[:,3:]@c))>-1e-8


@pytest.mark.parametrize('h',[3.,3.2,3.4,4.])
@pytest.mark.parametrize('invalid',[True])
@pytest.mark.parametrize('side',[-1,1])
def test_later_invalid_station_and_clear_single_keep_near_freedom(h,invalid,side):
 # Logged shape: pair through 1.35 s, optional rejected station at 1.40 s, then
 # already-clear single geometry with sampled zero bounds. Not the replayed frame.
 n=round(h*20)+1;t=np.arange(n)/20;later=t>1.39
 regions=np.where(later,side,2);regions[28]=0 if invalid else side
 p=build_path(frame=1,t=t,x=17.8*t,model_y=np.zeros(n),desired_offset=np.where(later,0.,.186),
  lower=np.where(later,0.,-.5),upper=np.where(later,0.,.5),speed=17.8,action_delay=.327,
  base_curvature=0.,cap=.0018,converter=get_curvature_from_plan,regions=regions)
 offset=p.target_y-p.model_y
 assert p.correction>0 and np.max(offset[t<1.])>1e-3
 c=np.linalg.lstsq(_matrices(h,n,False,True)[1],offset,rcond=None)[0];dense=np.arange(0,h,.001)
 assert np.max(np.abs((_basis(dense,_knots(h,True),3)[:,3:]@c)[dense>=1.4]))<1e-8


def test_near_local_selection_keeps_valid_and_short_previews_unchanged():
 from openpilot.starpilot.lateral.lane_path_v6 import _local_basis
 for h in HORIZONS:
  n=round(h*20)+1;t=np.arange(n)/20
  assert not _local_basis(h,None)
  assert not _local_basis(h,np.full(n,2))
  assert not _local_basis(h,np.full(n,1))
  regions=np.full(n,2);regions[-1]=0
  assert _local_basis(h,regions)==(h>2.5)
  assert not _local_basis(h,regions,True)
 for h in (1.05,1.1):
  t=np.arange(round(h*20)+1)/20;regions=np.where(t>=1.05-1e-8,0,2)
  p=straight(t,np.where(regions==0,0.,.1),np.where(regions==0,0.,-.5),np.where(regions==0,0.,.5),regions=regions)
  assert p.correction>0 and np.max(p.target_y[t<=1.])>1e-5
