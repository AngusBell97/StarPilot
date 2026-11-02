"""SP-034 7.0 ownership/frame adapter regressions; Dom VCruise is no longer an owner."""
from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from openpilot.cereal import messaging
from opendbc.car.structs import car
from openpilot.starpilot.controls.lib.map_corner.host_v7 import MapHost, visual_evidence
from openpilot.starpilot.controls.lib.map_corner.types import Result, PointCap
from openpilot.starpilot.curve_speed.runtime import Runtime
from openpilot.starpilot.curve_speed.host import CurveHost
from openpilot.starpilot.curve_speed.tests.test_runtime import frame
from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling

class QuietTransport:
  failed = False
  def record_diagnostics(self, snapshot): self.diagnostics = snapshot
  def exchange(self, frame): self.frame = frame; return None, False
  def fence(self, *args): self.fenced = args
  def close(self): pass

class Master(dict):
  pass

def setup():
  host = MapHost(None); host.enabled = host.learn = True; host.runtime_factory = QuietTransport
  curve = CurveHost(enabled=True, replay=True)
  cp = car.CarParams.new_message(); cp.openpilotLongitudinalControl = True
  planner = NS(CP=cp, force_stop_plan=NS(forcing=False), last_curve_ceiling_applied=True)
  cs = car.CarState.new_message(aEgo=0., vEgo=20., canValid=True)
  model = messaging.new_message('modelV2').modelV2
  model.orientationRate.z = [.02]; model.velocity.x = [20.]; model.position.x = [0.]
  gps = messaging.new_message('gpsLocationExternal').gpsLocationExternal
  gps.latitude=51.; gps.longitude=-1.; gps.bearingDeg=90.; gps.horizontalAccuracy=1.; gps.speed=20.; gps.hasFix=True
  sm = Master(carState=cs, modelV2=model, onroadEvents=[], gpsLocationExternal=gps)
  sm.valid={key:True for key in sm}; sm.alive=dict(sm.valid); sm.logMonoTime={}
  return host, curve, planner, sm

def tick(host, curve, planner, sm, n, *, visual=18., map_speed=10., applied=True, **kwargs):
  f=replace(frame(n), ego_mps=20., cruise_mps=25., **kwargs)
  sm.logMonoTime={key:f.now_ns for key in sm}
  host.begin(1)
  co=host.coordinator
  host.latest_result=Result(f.now_ns/1e9,n,co.control_epoch,co.odometer,'bend',100.,200.,
                           (PointCap(0.,map_speed,map_speed,key='bend',ceiling=30.),),'Map corner',visual_confirmed=True)
  suppressed=host.sample(sm,curve,planner,f,visual)
  chosen=host.select(CurveCeiling(visual,f.model_ns) if visual else None)
  planner.last_curve_ceiling_applied=applied
  host.confirm(planner)
  return chosen, suppressed

def test_map_can_go_below_vision_floor_but_never_raise_a_tighter_visual_cap():
  h,c,p,s=setup()
  for n in range(1,250): chosen,_=tick(h,c,p,s,n)
  assert chosen.speed_mps==pytest.approx(10.)
  chosen,_=tick(h,c,p,s,250,visual=8.)
  assert chosen.speed_mps==8. and h.winner=='visual'

def test_worker_ownership_requires_actual_planner_application():
  h,c,p,s=setup()
  for n in range(1,250): tick(h,c,p,s,n)
  assert h.runtime.frame.owner=='map' and h.runtime.frame.applied_map_cap>0
  tick(h,c,p,s,250,applied=False)
  assert h.runtime.frame.owner=='other' and h.runtime.frame.applied_map_cap==0

def test_frame_uses_model_identity_not_wall_sampling_time():
  h,c,p,s=setup()
  for n in range(1,250): chosen,_=tick(h,c,p,s,n,model_ns=frame(n).now_ns-10_000_000)
  assert chosen.model_ns==frame(249).now_ns-10_000_000

def test_stale_worker_result_cannot_claim_a_new_applied_learning_sample():
  h,c,p,s=setup()
  for n in range(1,250): tick(h,c,p,s,n)
  h.begin(1); f=frame(250); s.logMonoTime={key:f.now_ns for key in s}
  h.latest_result=replace(h.latest_result,now=h.latest_result.now-1)
  h.sample(s,c,p,f,18.)
  h.select(CurveCeiling(18.,f.model_ns));h.confirm(p)
  assert h.runtime.frame.applied_map_cap==0

def test_parent_off_does_not_construct_worker_or_change_visual_cap():
  h,c,p,s=setup();h.enabled=False
  chosen,_=tick(h,c,p,s,1)
  assert h.runtime is None and chosen==CurveCeiling(18.,frame(1).model_ns)

def test_manual_takeover_is_sent_without_automated_map_cap():
  h,c,p,s=setup()
  for n in range(1,250): tick(h,c,p,s,n)
  chosen,suppressed=tick(h,c,p,s,250,long_active=False,brake_pressed=True)
  assert suppressed and h.runtime.frame.intervention
  assert not h.runtime.frame.automated and h.runtime.frame.applied_map_cap==0

def test_invalid_input_cycle_fences_previous_worker_learning():
  h,c,p,s=setup();tick(h,c,p,s,1)
  fence=h.coordinator.observation_fence
  h.begin(1);h.confirm(p)
  assert h.coordinator.observation_fence>fence and h.latest_result is None

def test_gps_duplicate_publication_does_not_renew_fix_age():
  h,c,p,s=setup();tick(h,c,p,s,1)
  original=h.fix.received_at;stamp=s.logMonoTime['gpsLocationExternal']
  f=frame(2);s.logMonoTime={key:f.now_ns for key in s};s.logMonoTime['gpsLocationExternal']=stamp
  h.begin(1);h.sample(s,c,p,f,18.)
  assert h.fix.received_at==original

def test_map_exclusion_blocks_vision_learning_watch_and_manual_training():
  rt=Runtime(enabled=True)
  for n in range(1,200):
    f=frame(n,long_active=False)
    result=rt.step(f,before_learning=lambda *_:True)
    rt.confirm_applied(f.now_ns,applied=False)
  assert rt.curve.revision==0 and not result.training and result.learning_reason=='map_corner_exclusion'

def test_no_map_exclusion_retains_original_runtime_outputs():
  a=Runtime(enabled=True);b=Runtime(enabled=True)
  for n in range(1,100):
    f=frame(n)
    assert a.step(f)==b.step(f,before_learning=lambda *_:False)
    a.confirm_applied(f.now_ns,applied=True);b.confirm_applied(f.now_ns,applied=True)

def test_weather_is_not_fabricated_for_worker():
  h,c,p,s=setup();tick(h,c,p,s,1)
  assert h.runtime.frame.clear_conditions is False
