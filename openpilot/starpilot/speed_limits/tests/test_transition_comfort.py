from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from unittest.mock import patch
from openpilot.starpilot.speed_limits.transition_comfort import Adjustment,TransitionComfort,percentage
from openpilot.starpilot.speed_limits.custom_settings import CustomSettings
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages,V_EGO
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR

@pytest.mark.parametrize('changed', ['stale','future','manual','settled','blocked','bad_percent'])
def test_vetoes(changed):
  r=Adjustment(1_000_000_000,20.,-1,-50.)
  args=dict(now_ns=r.now_ns,selected=20.,speed=25.,blocked=False)
  if changed=='stale':args['now_ns']+=100_000_001
  if changed=='future':args['now_ns']-=1
  if changed=='manual':args['selected']=21.
  if changed=='settled':args['speed']=20.
  if changed=='blocked':args['blocked']=True
  if changed=='bad_percent':r=replace(r,percent=-51.)
  assert percentage(r,**args)==0

def test_native_cruise_shaping_does_not_relax_lead_stop_or_curve_braking():
  cp=CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  def run(request,lead=False,stop=False,force=False):
    planner=LongitudinalPlanner(cp,init_v=V_EGO)
    for _ in range(65):
      sm,_=messages(lead=lead,force=force)
      class Checked(dict):
        def all_checks(self,services):return True
      sm=Checked(sm)
      sm['carState'].vCruise=50.
      sm['carState'].vEgoCluster=V_EGO
      sm['carControl'].longActive=True
      sm['modelV2'].action.shouldStop=stop
      planner.update(sm,now_ns=1_000_000_000,slc_comfort=request,global_braking_response='standard')
    return planner
  request=Adjustment(1_000_000_000,50/3.6,-1,-50.)
  normal,soft=run(None),run(request)
  assert soft.a_cruise==pytest.approx(-.6) and normal.a_cruise==pytest.approx(-1.2)
  for kwargs in ({'lead':True},{'stop':True},{'force':True}):
    a,b=run(None,**kwargs),run(request,**kwargs)
    assert (a.output_a_target,a.output_should_stop,a.a_cruise)==(b.output_a_target,b.output_should_stop,b.a_cruise)


def test_manual_confirmation_retains_old_limit_baseline_until_ack():
  from openpilot.starpilot.speed_limits import acceptance as acc
  from openpilot.starpilot.speed_limits.runtime_settings import parse
  from openpilot.starpilot.speed_limits.runtime import Runtime,Action
  from openpilot.starpilot.speed_limits.tests.test_runtime_replay import RuntimeReplayTests
  from openpilot.starpilot.speed_limits.tests.test_set_speed import tick,command_receipt
  f=RuntimeReplayTests();f.setUp();sm,cp=f.sm,f.cp
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=90.;sm['carState'].vEgoCluster=25.
  r=Runtime(parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard','SLCTransitionOffsets':True,'SLCDecelOffsetPct':-50.}))
  for i in range(62):tick(r,sm,cp,i*.05)
  assert r.comfort.baseline==('dashboard',25.)
  r.settings=replace(r.settings,acceptance=replace(r.settings.acceptance,confirm_lower=True))
  sm['slcDashboardObservation'].speedMps=20.
  pending=tick(r,sm,cp,3.1).message.slcState
  assert pending.hasPending and r.comfort.baseline==('dashboard',25.)
  out=tick(r,sm,cp,3.2,request=Action(r.session_id,1,pending.decisionId,pending.presentationId,'accept'))
  cmd=out.command.slcCruiseCommand
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=72.
  tick(r,sm,cp,3.25,cruise_event=command_receipt(r,cmd,3.25,90.,72.))
  assert r.comfort_request is not None and r.comfort_request.percent==-50.
