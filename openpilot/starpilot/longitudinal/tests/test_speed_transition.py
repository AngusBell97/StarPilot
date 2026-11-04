"""SP-031 selected-speed continuity, native arbitration and forecast tests."""
import math
from types import SimpleNamespace as NS
import pytest
from openpilot.starpilot.longitudinal.set_speed_comfort import CruiseTransition
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel, LongitudinalPlanner
from openpilot.starpilot.longitudinal.tests.test_conditional_handoff import Frame, NOW
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR

CP=NS(carFingerprint='TESLA_MODEL_3',openpilotLongitudinalControl=True,steerRatio=15.,wheelbase=2.9,longitudinalActuatorDelay=.15)
def request(h,a,selected,speed,tick,**kw):
 return get_cruise_accel(False,selected,speed,a,0.,CP,.05,0.,True,
   selected_acceleration_max=1.6,base_brake_floor=-1.2,a_target_prev=a,
   transition=h,transition_now_ns=NOW+tick*50_000_000,transition_enabled=True,**kw)

@pytest.mark.parametrize('a',[-1.,-.5,0.,.6,.812,1.3])
def test_entry_has_no_fixed_acceleration_cap_or_brake_release_step(a):
 h=CruiseTransition();out=request(h,a,30.,15.,0)
 assert out>=a and out-a<=.001

@pytest.mark.parametrize('cap',[.3,.6,1.,1.6,2.])
def test_long_clear_road_step_reaches_selected_profile(cap):
 h=CruiseTransition();a=0.
 for i in range(400):
  old=a;a=get_cruise_accel(False,40.,15.,a,0.,CP,.05,0.,True,
   selected_acceleration_max=cap,a_target_prev=a,transition=h,
   transition_now_ns=NOW+i*50_000_000,transition_enabled=True)
  assert a<=cap+1e-8 and -.400001<=(a-old)/.05<=.350001
 from openpilot.selfdrive.controls.lib.longitudinal_planner import _A_TOTAL_MAX_V
 assert a==pytest.approx(min(cap,_A_TOTAL_MAX_V[0]),abs=1e-5)  # the total-acceleration budget below 20 m/s bounds the step

@pytest.mark.parametrize('targets',[[20.,30.,25.,30.]*30,[15.1,14.9,15.05,14.99]*30,[30.]*20+[10.]*100])
def test_repeated_and_near_target_changes_retain_continuity(targets):
 h=CruiseTransition();a=0.
 for i,target in enumerate(targets):
  old=a;a=request(h,a,target,15.,i)
  assert -.400001<=(a-old)/.05<=.350001

@pytest.mark.parametrize('protection',[True,False])
def test_protected_braking_and_fresh_recovery(protection):
 h=CruiseTransition();a=0.
 for i in range(100):a=request(h,a,30.,15.,i)
 if protection:
  direct=get_cruise_accel(False,5.,15.,a,0.,CP,.05,0.,True,selected_acceleration_max=1.6,base_brake_floor=-1.2,a_target_prev=a)
  assert request(h,a,5.,15.,100,transition_protected=True)==pytest.approx(direct)
 # Model/lead final arbitration lowered the actual request, even though our
 # cruise output was high. Once that lower request has persisted for the dip
 # hold (0.3 s), release must start from that lower value.
 for i in range(101,107):request(h,-1.,30.,15.,i)
 out=request(h,-1.,30.,15.,107)
 assert -1.<out<=-.89+1e-9
 assert .05<=out+1.<=.054 # Native nominal50ms slew remains the outer bound.


def test_short_dip_keeps_the_ramp_instead_of_restarting():
 h=CruiseTransition();a=0.
 for i in range(40):a=request(h,a,30.,15.,i)
 before=a
 # A two-cycle lower final request (model blip, radar swap) must not restart the ramp.
 for i in (40,41):request(h,-.3,30.,15.,i)
 resumed=request(h,before,30.,15.,42)
 assert resumed>before and resumed-before<=3*.35*.05+1e-9
 # The same dip held for longer than the hold does restart from the lower request.
 h=CruiseTransition();a=0.
 for i in range(40):a=request(h,a,30.,15.,i)
 for i in range(40,47):out=request(h,-.3,30.,15.,i)
 assert out<0.

@pytest.mark.parametrize('delta_ns',[0,-50_000_000,1_000_000_000])
def test_clock_discontinuity_cannot_reuse_a_large_elapsed_step(delta_ns):
 h=CruiseTransition();request(h,0.,30.,15.,0)
 x=h.shape(selected=30.,speed=15.,desired=2.,previous=0.,now_ns=NOW+delta_ns,enabled=True)
 assert 0.<=x<=.001

@pytest.mark.parametrize('fault',['off','stopped','nan','protected'])
def test_bypass_for_invalid_stopped_and_protected_context(fault):
 h=CruiseTransition();args=dict(selected=20.,speed=15.,desired=-2.,previous=.5,now_ns=NOW,enabled=True)
 if fault=='off':args['enabled']=False
 if fault=='stopped':args['speed']=0.
 if fault=='nan':args['selected']=math.nan
 if fault=='protected':args['protected']=True
 assert h.shape(**args)==-2. and h.time_ns is None

@pytest.mark.parametrize('context',['ordinary','model','lead','force','curve','gas','brake','standstill'])
def test_real_planner_native_arbitration(context):
 from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling
 from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import ACTIVE
 cp=CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3);cp.openpilotLongitudinalControl=True
 planner=LongitudinalPlanner(cp,init_v=15.)
 for i in range(25):
  sm=Frame();now=NOW+i*50_000_000;sm.logMonoTime=dict.fromkeys(sm,now-5_000_000)
  sm['carState'].vEgo=sm['carState'].vEgoCluster=15.;sm['carState'].vCruise=108.
  sm['modelV2'].action.shouldStop=False;sm['modelV2'].action.desiredAcceleration=2.
  kw={}
  if i>=20:
   if context=='model':
    sm['selfdriveState'].experimentalMode=True;sm['modelV2'].action.desiredAcceleration=-2.
   if context=='lead':
    for lead in [sm['radarState'].leadOne,sm['radarState'].leadTwo]:
     lead.present=True;lead.dRel=10.;lead.vLead=lead.vLeadK=2.;lead.vRel=-13.
   if context=='force':sm['controlsState'].forceDecel=True
   if context=='curve':kw['curve_ceiling']=CurveCeiling(8.,sm.logMonoTime['modelV2'])
   if context=='gas':sm['carState'].gasPressed=True
   if context=='brake':sm['carState'].brakePressed=True
   if context=='standstill':sm['carState'].standstill=True;sm['carState'].vEgo=sm['carState'].vEgoCluster=0.
  planner.update(sm,now_ns=now,drive_id=1,**kw)
  assert math.isfinite(planner.output_a_target)
 if context=='model':assert planner.output_a_target<=-2.
 if context=='lead':assert planner.output_a_target<0.
 if context in ('force','curve','gas','brake','standstill'):assert planner.cruise_transition.time_ns is None
 if context=='ordinary':assert 0.<planner.output_a_target<.5


def test_pulse_glide_cruise_ceiling_uses_same_transition():
 from openpilot.starpilot.controllers.coast import CoastPlan
 cp=CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3);cp.openpilotLongitudinalControl=True
 planner=LongitudinalPlanner(cp,init_v=15.,init_a=.8)
 class Owner:
  def sample(self,*args,**kw):return CoastPlan(brake_floor=-.03,speed_ceiling_mps=12.,pulse_coasting=True)
 previous=.8
 for i in range(20):
  sm=Frame();now=NOW+i*50_000_000;sm.logMonoTime=dict.fromkeys(sm,now-5_000_000)
  sm['carState'].vEgo=sm['carState'].vEgoCluster=15.;sm['carState'].vCruise=108.
  planner.update(sm,now_ns=now,drive_id=1,wheel_coast=(Owner(),None))
  assert planner.cruise_transition.time_ns==now
  # The cruise candidate keeps its own comfort rate; a lower final output
  # (lead/model/coast) only re-anchors it after the 0.3 s dip hold.
  assert -.400001<=(planner.a_cruise-previous)/.05<=.350001
  assert planner.output_a_target<=planner.a_cruise+1e-9
  previous=planner.a_cruise


def test_drive_boundary_resets_transition_history():
 cp=CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3);cp.openpilotLongitudinalControl=True
 planner=LongitudinalPlanner(cp,init_v=15.)
 for i in range(20):
  sm=Frame();sm['carState'].vCruise=108.;sm['carState'].vEgo=15.
  planner.update(sm,now_ns=NOW+i*50_000_000,drive_id=1)
 prior=planner.output_a_target
 planner.update(sm,now_ns=NOW+20*50_000_000,drive_id=2)
 assert abs(planner.a_cruise-prior)<=.001


def test_external_braking_recovery_is_not_held_at_comfort_release_rate():
 h=CruiseTransition();a=0.
 for i in range(100):a=request(h,a,30.,15.,i)
 # A stronger lead/model request won the previous final arbitration.
 a=-1.5;first=request(h,a,30.,15.,100)
 assert first>a+.04
 a=first
 for i in range(101,132):a=request(h,a,30.,15.,i)
 assert a>0. # Native release, rather than >4s of continued comfort braking.
 previous=a;a=request(h,a,30.,15.,132)
 assert a-previous<=.35*.05+1e-9
