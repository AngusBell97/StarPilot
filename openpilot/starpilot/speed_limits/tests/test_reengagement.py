from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.speed_limits.reengagement import Engagement
from openpilot.starpilot.speed_limits import physical_actions
from openpilot.starpilot.speed_limits.runtime import Runtime, CruiseEvent
from openpilot.starpilot.speed_limits.runtime_settings import parse
from openpilot.starpilot.speed_limits.tests.test_set_speed import drive, tick, command_receipt, START

ZONE=('map','producer',20.)
def observe(e,t,**kw):
  values=dict(now_ns=int((t+1)*1e9),zone=ZONE,enabled=True,fresh=True,selected=22.,remember=True,match=True)
  values.update(kw);e.observe(**values)

def test_matching_never_lowers_faster_engagement_and_no_pedal_input():
  e=Engagement();observe(e,0)
  assert e.resolve(limit_target=20.,provisional=False,now_ns=1_000_000_000)==22.
  e.applied();observe(e,.1,selected=22.)
  assert e.resolve(limit_target=25.,provisional=False,now_ns=1_100_000_000) is None

@pytest.mark.parametrize('memory',[15.,30.])
def test_manual_recall_both_sides_of_limit(memory):
  e=Engagement();observe(e,0);e.applied()
  observe(e,.1,manual=memory,selected=memory)
  observe(e,.2,enabled=False,selected=memory)
  observe(e,.3,selected=23.)
  assert e.resolve(limit_target=20.,provisional=False,now_ns=1_300_000_000)==memory

@pytest.mark.parametrize('veto',['zone','gap','source','disabled','explicit'])
def test_memory_is_invalidated_by_current_zone_gaps_and_new_intent(veto):
  e=Engagement();observe(e,0);e.applied();observe(e,.1,manual=30.);observe(e,.2,enabled=False)
  kw={'enabled':False}
  kw.update({'zone':('map','producer',15.)} if veto=='zone' else {'zone':('map','restarted',20.)} if veto=='source' else
            {'fresh':False} if veto=='gap' else {'remember':False} if veto=='disabled' else {'explicit':True})
  observe(e,.3,**kw);observe(e,.4)
  assert e.resolve(limit_target=20.,provisional=False,now_ns=1_400_000_000)==22.

def test_recall_cannot_restore_prior_approach_target_or_survive_stall():
  e=Engagement();observe(e,0);e.applied();observe(e,.1,manual=30.);observe(e,.2,enabled=False);observe(e,.3)
  assert e.resolve(limit_target=15.,provisional=True,now_ns=1_300_000_000)==22.
  observe(e,2,enabled=False);observe(e,2.1)
  assert e.remembered is None


def test_real_runtime_resume_command_is_bound_separately_and_acknowledged(drive):
  sm,cp=drive
  runtime=Runtime(parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard','SLCReengageSpeed':True,'SLCRememberManualOffset':True}))
  # Establish normal adoption first, without dropping publisher continuity.
  first=None
  for i in range(62):
    out=tick(runtime,sm,cp,i*.05)
    if out.command is not None:first=out;break
  assert first is not None
  command=first.command.slcCruiseCommand
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=90.
  tick(runtime,sm,cp,3.1,cruise_event=command_receipt(runtime,command,3.1,60.,90.))
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=100.
  at=START+3_200_000_000
  tick(runtime,sm,cp,3.2,cruise_event=CruiseEvent(2,at,25.,100./3.6,'accel','card-producer',at))
  sm['selfdriveState'].enabled=sm['carControl'].enabled=sm['carControl'].longActive=False
  sm['controlsState'].longControlState='off'
  tick(runtime,sm,cp,3.3)
  sm['selfdriveState'].enabled=sm['carControl'].enabled=sm['carControl'].longActive=True
  sm['controlsState'].longControlState='pid'
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=80.
  resumed=tick(runtime,sm,cp,3.4)
  assert resumed.command is not None
  cmd=resumed.command.slcCruiseCommand
  assert str(cmd.kind)=='reengageZone' and cmd.targetMps==pytest.approx(100./3.6)
  assert resumed.message.slcState.hasReengageTarget
  assert resumed.message.slcState.acceptedSpeedLimit==25.
  # The normal adoption kind cannot smuggle a remembered target.
  from opendbc.car.dashboard_speed_limit import Status
  d=sm['slcDashboardObservation']
  observation=SimpleNamespace(status=Status.VALID,speed_mps=d.speedMps,episode=d.episode,
    observed_ns=d.observedMonoTime,valid_until_ns=d.validUntilMonoTime)
  assert physical_actions.command_applicable(cmd,resumed.message.slcState,observation,'card-producer',START+3_400_000_000,80./3.6,long_active=True,button_event=False)
  cmd.kind='adoptAcceptedLimit'
  assert not physical_actions.command_applicable(cmd,resumed.message.slcState,observation,'card-producer',START+3_400_000_000,80./3.6,long_active=True,button_event=False)
  cmd.kind='reengageZone'
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=100.
  ack=tick(runtime,sm,cp,3.45,cruise_event=command_receipt(runtime,cmd,3.45,80.,100.,event_id=3))
  assert ack.command is None and ack.message.slcState.commandStatus=='applied'
  sm['carState'].gasPressed=True;sm['carState'].vEgo=35.
  assert tick(runtime,sm,cp,3.5).command is None
  sm['carState'].gasPressed=False
  assert tick(runtime,sm,cp,3.55).command is None
