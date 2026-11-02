from dataclasses import replace
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from openpilot.cereal import log
from openpilot.starpilot.lateral.lane_centering import LaneCenteringController, LaneCenteringSettings, LaneCenteringInput, ControlMode
from openpilot.starpilot.lateral.lane_v5 import LaneCenteringV5Controller, LaneCenteringV5Settings, LaneCenteringV5Input
from openpilot.starpilot.lateral.lane_motion import Motion, warp_profile
from openpilot.starpilot.lateral.lane_versions import LaneCenteringVersions
from openpilot.starpilot.lateral.lane_path import build_path, gain_normalization
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan as converter


def scene(mid=.2, speed=15., single=0):
 t=np.linspace(0,5,33);x=t*speed
 def path(y): return NS(x=x.tolist(),y=(np.full(33,y) if np.isscalar(y) else y).tolist(),z=np.zeros(33).tolist())
 return NS(position=NS(x=x.tolist(),y=np.zeros(33).tolist(),z=np.zeros(33).tolist(),t=t.tolist(),yStd=np.full(33,.1).tolist()),
   laneLines=[path(-5),path(np.asarray(mid)-1.8),path(np.asarray(mid)+1.8),path(5)],roadEdges=[path(-2),path(2)],
   laneLineProbs=[0,.9 if single!=-1 else 0,.9 if single!=1 else 0,0],laneLineStds=[0,.1,.1,0],roadEdgeStds=[2,2],
   orientation=NS(z=np.zeros(33).tolist()),orientationRate=NS(z=np.zeros(33).tolist()),
   meta=NS(laneChangeState=log.LaneChangeState.off),action=NS(desiredCurvature=0.))


def obs(i=0, model=None, **kwargs):
 defaults=dict(model=model or scene(),model_sample_id=1_000_000_000+(i//5)*50_000_000,model_valid=True,base_curvature=.001,
   speed_mps=15.,mode=ControlMode.LATERAL_ONLY,lateral_active=True,
   settings=LaneCenteringV5Settings(enabled=True,e2e_authority=0.,strength=1.5,response_min=.2,smoothing_distance=.2),elapsed_seconds=.01,
   motion=Motion(1_000_000_000+i*10_000_000,0.,0.,0.,.31))
 defaults.update(kwargs);return LaneCenteringV5Input(**defaults)


def controller(): return LaneCenteringV5Controller(clock=lambda:0.)


def test_real_path_drives_parent_filter_and_preserves_base():
 c=controller();o=obs();r=c.update(o);p=c.target_path
 assert r.reason=='qualified' and p is not None
 expected=p.correction*(1-math.exp(-.01/c._response_tau))
 assert r.correction==pytest.approx(expected) and r.candidate_curvature==pytest.approx(o.base_curvature+expected)
 assert p.correction==pytest.approx(converter(p.heading_delta,p.yaw_rate_delta,p.t,o.speed_mps,o.motion.delay))


def test_zero_adjustment_is_exact_identity():
 c=controller();r=c.update(obs(model=scene(0.)))
 assert r.correction==0 and r.candidate_curvature==.001
 np.testing.assert_array_equal(c.target_path.target_y,c.target_path.model_y)


@pytest.mark.parametrize('priority',[False,True])
@pytest.mark.parametrize('speed',[3.,8.,15.,30.])
@pytest.mark.parametrize('horizon',[1.,2.,3.,4.])
def test_nominal_gain_matches_v4_small_constant_error(speed,horizon,priority):
 t=np.linspace(0,horizon,round(horizon*20)+1);error=1e-4
 gain=gain_normalization(horizon,len(t),speed,.31,converter,near_priority=priority)
 p=build_path(frame=1,t=t,x=speed*t,model_y=np.zeros(len(t)),desired_offset=np.full(len(t),error*.45*gain),
  lower=np.full(len(t),-1.),upper=np.full(len(t),1.),speed=speed,action_delay=.31,base_curvature=0.,cap=.0018,converter=converter,near_priority=priority)
 assert p.correction==pytest.approx(.45*2*error/np.clip(speed,8.,35.)**2,rel=1e-5)


def test_model_frame_cadence_repeated_ticks_only_filter_output():
 c=controller();c.update(obs());p=c.target_path;profiles=c._profiles;previous=c._correction
 for i in range(1,5):
  r=c.update(obs(i));assert c.target_path is p and c._profiles is profiles and r.correction>previous;previous=r.correction
 c.update(obs(5));assert c.target_path is not p


def test_motion_aligned_straight_profile_and_turn_rotation():
 x=np.linspace(-5,100,100)
 a,valid=warp_profile(x,np.full(len(x),-1.8),.75,0,0,np.arange(60.));assert valid.all();np.testing.assert_allclose(a,-1.8)
 a,valid=warp_profile(x,.1*x+2,0,0,math.atan(.1),np.arange(50.));assert valid.all();np.testing.assert_allclose(a,2*math.cos(math.atan(.1)))


def test_lane_edge_replacement_is_not_used():
 c=controller();c.update(obs())
 m=scene();m.laneLineProbs[1]=0.;m.roadEdgeStds[0]=.1;m.roadEdges[0].y=(np.full(33,-1.9)).tolist()
 for i in range(1,6):c.update(obs(i,model=m))
 assert c.selected_mode==0 and c.selected_sources==()
 assert c._profiles[0] is None


@pytest.mark.parametrize('field,value',[('driver_override',True),('lateral_active',False),('model_valid',False),('time_discontinuity',True)])
def test_reset_clears_target_and_history(field,value):
 c=controller();c.update(obs());r=c.update(obs(1,**{field:value}))
 assert r.correction==0 and c.target_path is None and c._profiles==[None,None]


def test_signal_pause_fades_and_clears_target():
 c=controller();c.update(obs());before=c._correction;r=c.update(obs(1,turn_signal_active=True))
 assert r.correction==pytest.approx(before*math.exp(-.01/.2)) and c.target_path is None and c._profiles==[None,None]


def test_malformed_motion_withdraws_even_repeated_frame():
 c=controller();c.update(obs());before=c._correction
 r=c.update(obs(1,motion=replace(obs(1).motion,measured_curvature=float('nan'))))
 assert 0<r.correction<before and c.target_path is None


def test_frame_invalidity_does_not_extrapolate():
 c=controller();m=scene();m.position.t[8]=m.position.t[7]
 r=c.update(obs(model=m));assert r.reason=='v5_time_geometry' and c.target_path is None


def test_far_invalid_geometry_truncates_supported_target():
 m=scene();m.laneLines[2].y[-17:]=[10.]*17
 c=controller();r=c.update(obs(model=m))
 assert r.reason=='qualified' and 1<=c.target_path.t[-1]<3.


def test_opposing_preview_changes_path_before_near_target_changes():
 straight=controller();straight.update(obs(model=scene(.2)))
 t=np.linspace(0,5,33);mid=np.interp(t,[0,1,2,5],[.2,.2,-.2,-.2])
 c=controller();r=c.update(obs(model=scene(mid)));assert r.reason=='qualified'
 assert c.target_path.correction<straight.target_path.correction
 assert np.min(c.target_path.target_y)<-.005 and np.max(c.target_path.target_y)>.005


@pytest.mark.parametrize('side',[1,-1])
def test_single_line_is_not_used(side):
 m=scene(mid=side*1.,single=side);c=controller();r=c.update(obs(model=m))
 assert c.selected_mode==0 and c.target_path is None and r.correction==0.


def test_single_opposite_conflict_has_no_target():
 m=scene(mid=1.,single=1);m.laneLineProbs[2]=.9;m.laneLines[2].y=[1.]*33
 c=controller();r=c.update(obs(model=m));assert c.target_path is None and r.correction==0


def test_budget_guard_rejects_new_target():
 values=iter([0.,.031]);c=LaneCenteringV5Controller(clock=lambda:next(values));r=c.update(obs())
 assert r.reason=='v5_compute_budget' and c.target_path is None and r.correction==0


def test_live_settings_refresh_applies_next_frame_without_reset():
 c=controller();c.update(obs());before=c._correction
 s=replace(obs().settings,smoothing_tau=0.,response_tau=.1,response_min=.1)
 for i in range(1,6):r=c.update(obs(i,settings=s,model=scene(.1)))
 assert r.reason=='qualified' and c.target_path.frame==obs(5).model_sample_id and c._correction>0 and before>0
 np.testing.assert_allclose(c._profiles[0][1],-1.7)


def test_v4_exact_dispatch_and_live_handover():
 a=LaneCenteringController();b=LaneCenteringVersions();b.v5.clock=lambda:0.
 for i in range(50):
  v=obs(i);o=LaneCenteringInput(**{k:getattr(v,k) for k in LaneCenteringInput.__dataclass_fields__})
  assert a.update(o)==b.update(o)
 before=b._correction;r=b.update(o,version=5,motion=obs(50).motion)
 assert r.reason=='qualified' and abs(r.correction-before)<.0001 and b.target_path is not None
 r=b.update(o,version=4);assert b.v5.target_path is None and r.reason=='qualified'


def test_motion_uses_actual_elapsed_and_accepts_normal_tick_jitter():
 c=controller();c.update(obs());c.update(obs(1,motion=replace(obs(1).motion,timestamp_ns=1_004_000_000),elapsed_seconds=.01))
 assert c.target_path is not None and c._motion[0]==pytest.approx(.004*15.)
 c.update(obs(2,motion=replace(obs(2).motion,timestamp_ns=1_028_000_000),elapsed_seconds=.01))
 assert c.target_path is not None and c._motion[0]==pytest.approx(.028*15.)
 r=c.update(obs(3,motion=replace(obs(3).motion,timestamp_ns=1_100_000_000)))
 assert c.target_path is None and r.reason=='v5_motion_unqualified'


def test_negative_model_uncertainty_does_not_enable_override():
 m=scene(.6);m.position.yStd=[-1.]*33
 s=replace(obs().settings,e2e_authority=1.,e2e_min=1.)
 c=controller();c.update(obs(model=m,settings=s));p=c.target_path
 d=controller();d.update(obs(model=m,settings=replace(s,e2e_authority=0.,e2e_min=0.)))
 assert p is not None and p.correction==pytest.approx(d.target_path.correction)


@pytest.mark.parametrize('side',[1,-1])
def test_retained_single_policy_math_when_explicitly_opted_in(monkeypatch,side):
 import openpilot.starpilot.lateral.lane_v5 as lv
 original=lv.build_path;captured=[]
 def capture(**kwargs):
  captured.append(kwargs);return original(**kwargs)
 monkeypatch.setattr(lv,'build_path',capture)
 t=np.linspace(0,5,33)
 m=scene(mid=side*np.interp(t,[0,.25,1,5],[.1,.1,.8,.8]),single=side)
 c=controller();c.use_single_boundary=True;r=c.update(obs(model=m))
 assert r.reason=='qualified' and captured[-1]['near_priority']
 a=captured[-1];desired=side*a['desired_offset'];near=(a['t']<=1)&(desired>0)
 assert np.count_nonzero(near)>2 and np.ptp(desired[near])<1e-10
 assert desired[0]==0 and np.min(side*(c.target_path.target_y-c.target_path.model_y))>=-1e-8
 # Full model preference still suppresses an independently confident model path.
 s=replace(obs().settings,e2e_authority=1.,e2e_min=1.,override_start=.05,override_full=.1)
 c=controller();c.use_single_boundary=True;m=scene(mid=side*1.2,single=side);r=c.update(obs(model=m,settings=s))
 assert r.reason=='qualified' and c.target_path.correction==0
 assert np.max(np.abs(captured[-1]['desired_offset']))==0
