from dataclasses import replace
from types import SimpleNamespace as NS
import math
import pytest
from openpilot.starpilot.longitudinal.slow_creep import Frame, SlowCreep, project
from openpilot.starpilot.longitudinal.lead_takeoff import Lead
from openpilot.starpilot.longitudinal.force_stop import StopPlan

NOW=100_000_000_000
DRIVE=90_000_000_000

def frame(i=0, **changes):
 n=NOW+i*50_000_000
 return replace(Frame(n,n-5_000_000,DRIVE,True,0.,10.,.04,1.2,1.6,
                      (Lead(0,-1,False,.999,5.6+.02*i,.2,0.,0.),)),**changes)

def armed():
 p=SlowCreep()
 for i in range(40):result=p.step(frame(i),.04,True)
 assert result==(.04,False)
 return p

def test_constant_speed_creep_is_confirmed_without_changing_native_acceleration():
 p=SlowCreep();results=[p.step(frame(i),.04,True) for i in range(60)]
 first=next(i for i,x in enumerate(results) if not x[1])
 assert first>=20
 assert all(a==.04 for a,_ in results)
 assert results[-1]==(.04,False)


def test_stationary_lead_never_releases_even_with_noisy_opening_range():
 p=SlowCreep()
 for i in range(80):
  f=frame(i);f=replace(f,leads=(replace(f.leads[0],speed=.02),))
  assert p.step(f,.04,True)==(.04,True)

@pytest.mark.parametrize('field,value', [('mpc_accel',-.001),('cruise_accel',0.),('selected_speed',0.),
 ('speed',.8),('speed',-.01),('speed',math.nan),('follow_seconds',4.),('drive_id',0)])
def test_invalid_or_native_braking_inputs_retain_original(field,value):
 p=armed();assert p.step(frame(40,**{field:value}),.04,True)==(.04,True)

@pytest.mark.parametrize('field,value',[('speed',0.),('acceleration',-.2),('probability',.94),
 ('lateral',1.1),('distance',3.),('track_id',22),('distance',12.)])
def test_lead_stop_brake_replacement_or_cut_in_cancels_immediately(field,value):
 p=armed();f=frame(40);f=replace(f,leads=(replace(f.leads[0],**{field:value}),))
 assert p.step(f,.04,True)==(.04,True)

@pytest.mark.parametrize('gap,speed,prob',[(5.,0.,1.),(6.,0.,.99),(6.,.3,.5)])
def test_possible_nearer_obstacle_cannot_be_ignored(gap,speed,prob):
 p=armed();f=frame(40);other=Lead(1,-1,False,prob,gap,speed,0.,0.)
 assert p.step(replace(f,leads=f.leads+(other,)),.04,True)==(.04,True)

def test_negative_target_is_not_raised_and_new_confirmation_required():
 p=armed();assert p.step(frame(40),-.001,True)==(-.001,True)
 assert p.step(frame(41),.04,True)==(.04,True)

def test_stale_frozen_and_reversed_clock_reset():
 for change in [dict(model_ns=NOW),dict(now_ns=NOW),dict(drive_id=DRIVE+1)]:
  p=armed();assert p.step(frame(40,**change),.04,True)==(.04,True)
 p=armed();assert p.step(None,.04,True)==(.04,True)
 assert p.step(frame(41),.04,True)==(.04,True)

@pytest.mark.parametrize('target',[1e-6,.005,.02,.04,.09,.1,.15,.3,.6,1.2])
def test_native_acceleration_is_exactly_preserved_for_all_positive_requests(target):
 p=SlowCreep()
 for i in range(70):a,stop=p.step(frame(i,cruise_accel=target,mpc_accel=target),target,True)
 assert a==target and stop is False
 assert p.step(frame(70),target,False)==(target,False)

@pytest.mark.parametrize('value',[0.,-.000001,-.04])
@pytest.mark.parametrize('field',['target','mpc_accel','cruise_accel'])
def test_zero_or_negative_request_never_releases_and_reconfirmation_required(field,value):
 p=armed();f=frame(40);target=.04
 if field=='target':target=value
 else:f=replace(f,**{field:value})
 assert p.step(f,target,True)==(target,True)
 assert p.step(frame(41),.04,True)==(.04,True)


def test_cannot_admit_a_car_already_rolling_without_observed_stop():
 p=SlowCreep()
 for i in range(80):assert p.step(frame(i,standstill=False,speed=.2),.04,True)==(.04,True)

def test_release_remains_clear_below_old_threshold_while_confirmed_creep_continues():
 p=armed();f=frame(40,speed=.15,standstill=False)
 a,stop=p.step(f,.04,True)
 assert a==.04 and stop is False

class SM(dict):pass

def messages():
 cp=NS(carFingerprint='TESLA_MODEL_3',openpilotLongitudinalControl=True,passive=False,dashcamOnly=False,notCar=False)
 lead=NS(present=True,radarTrackId=-1,radar=False,modelProb=.999,dRel=6.5,vLead=.2,aLeadK=0.,yRel=0.)
 sm=SM(carState=NS(canValid=True,canTimeout=False,gasPressed=False,brakePressed=False,gearShifter='drive',
                   standstill=True,vEgo=0.,vEgoRaw=0.),carControl=NS(longActive=True),controlsState=NS(forceDecel=False),
       selfdriveState=NS(enabled=True,experimentalMode=False),radarState=NS(leadOne=lead,leadTwo=NS(present=False)),modelV2=NS())
 sm.valid=dict.fromkeys(sm,True);sm.alive=dict.fromkeys(sm,True);sm.logMonoTime=dict.fromkeys(sm,NOW-5_000_000)
 sm.recv_time=dict.fromkeys(sm,(NOW-5_000_000)/1e9)
 return sm,cp

def projected(sm,cp,**changes):
 kw=dict(now_ns=NOW,drive_id=DRIVE,selected_speed=10.,mpc_accel=.04,cruise_accel=1.2,follow_seconds=1.6,stop_plan=StopPlan(),threshold_only=True)
 kw.update(changes);return project(sm,cp,**kw)

def test_current_tesla_stationary_noise_projection():
 sm,cp=messages();sm['carState'].vEgo=.025;sm['carState'].vEgoRaw=.025
 assert projected(sm,cp).speed==0.

@pytest.mark.parametrize('channel,field,value', [('carState','canValid',False),('carState','canTimeout',True),
 ('carState','gasPressed',True),('carState','brakePressed',True),('carState','gearShifter','reverse'),
 ('carControl','longActive',False),('controlsState','forceDecel',True),('selfdriveState','enabled',False)])
def test_runtime_ownership_and_pedal_guards(channel,field,value):
 sm,cp=messages();setattr(sm[channel],field,value);assert projected(sm,cp) is None

@pytest.mark.parametrize('field,value',[('carFingerprint','HYUNDAI_IONIQ_6'),('openpilotLongitudinalControl',False),
 ('passive',True),('dashcamOnly',True),('notCar',True)])
def test_other_vehicles_and_non_controlling_modes_are_exact_passthrough(field,value):
 sm,cp=messages();setattr(cp,field,value);assert projected(sm,cp) is None

@pytest.mark.parametrize('field',['forcing','should_stop','manual_hold','obstacle_m','speed_ceiling_mps'])
def test_force_stop_and_approach_authority(field):
 sm,cp=messages();assert projected(sm,cp,stop_plan=replace(StopPlan(),**{field:True if field in ['forcing','should_stop','manual_hold'] else 1.})) is None

@pytest.mark.parametrize('service',['carState','carControl','controlsState','selfdriveState','radarState','modelV2'])
@pytest.mark.parametrize('kind',['valid','alive','stamp','receipt'])
def test_every_input_requires_fresh_valid_observation(service,kind):
 sm,cp=messages()
 if kind in ['valid','alive']:getattr(sm,kind)[service]=False
 elif kind=='stamp':sm.logMonoTime[service]=NOW-150_000_001
 else:sm.recv_time[service]=(NOW-150_000_001)/1e9
 assert projected(sm,cp) is None


def test_experimental_never_releases_and_chill_reentry_requires_confirmation():
 p=SlowCreep()
 for i in range(40):assert p.step(frame(i,release_allowed=False),.04,True)==(.04,True)
 for i in range(40,50):assert p.step(frame(i),.04,True)==(.04,True)
 for i in range(50,70):p.step(frame(i),.04,True)
 assert p.step(frame(70),.04,True)==(.04,False)
 assert p.step(frame(71,release_allowed=False),.04,True)==(.04,True)
 assert p.step(frame(72),.04,True)==(.04,True)
 sm,cp=messages();sm['selfdriveState'].experimentalMode=True
 assert projected(sm,cp).release_allowed is False
 assert projected(*messages(),threshold_only=False).release_allowed is False


def test_lead_speed_without_opening_range_cannot_release():
 p=SlowCreep()
 for i in range(80):
  f=frame(i);f=replace(f,leads=(replace(f.leads[0],distance=6.2),))
  assert p.step(f,.04,True)==(.04,True)


def test_return_inside_selected_gap_does_not_release():
 p=armed();f=frame(40,speed=.25,standstill=False)
 f=replace(f,leads=(replace(f.leads[0],distance=6.39),))
 assert p.step(f,.04,True)==(.04,True)


def test_repeated_zero_positive_requests_do_not_chatter_release():
 p=SlowCreep()
 for i in range(80):
  target=0. if i%4==0 else .04
  assert p.step(frame(i),target,True)==(target,True)


def test_opening_is_not_reapplied_after_admission_above_normal_clearance():
 p=SlowCreep()
 for i in range(40):
  f=frame(i);p.step(replace(f,leads=(replace(f.leads[0],distance=6.5+.02*i),)),.04,True)
 assert p.released
 # Ego catches the lead; gap shrinks below the old 6.5+.4 opening threshold,
 # but remains above the actual 6+.15*1.6 following clearance.
 for i in range(40,80):
  f=frame(i,speed=.15,standstill=False);gap=max(6.5,7.28-.02*(i-39))
  assert p.step(replace(f,leads=(replace(f.leads[0],distance=gap),)),.04,True)==(.04,False)


def test_correlated_stationary_vision_jitter_does_not_accumulate_opening():
 p=SlowCreep()
 for i in range(200):
  f=frame(i);phase=i%8;offset=(-.5+.125*phase);speed=.15 if phase<4 else -.15
  f=replace(f,leads=(replace(f.leads[0],distance=7.+offset,speed=speed),))
  assert p.step(f,.04,True)==(.04,True)


def test_diagnostics_are_transition_only_and_capture_cancellation():
 p=SlowCreep()
 for i in range(40):
  p.step(frame(i),.04,True);d=p.diagnostic(.04,True)
  if d is not None:assert d['admitted'] and d['native_target']==.04
 assert p.diagnostic(.04,True) is None
 p.step(None,.04,True);assert p.diagnostic(.04,True)['admitted'] is False
 assert p.diagnostic(.04,True) is None
