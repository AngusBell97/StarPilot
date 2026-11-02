"""Real cereal source -> planner -> Card binding -> acknowledgement regressions."""
from dataclasses import replace
import pytest
from openpilot.starpilot.speed_limits.runtime import Runtime, CruiseEvent, Action
from openpilot.starpilot.speed_limits.runtime_settings import parse
from openpilot.starpilot.speed_limits import physical_actions
from openpilot.starpilot.speed_limits.tests.test_set_speed import drive, command_receipt, START
from openpilot.starpilot.speed_limits.tests.test_maps_runtime import packet


def step(runtime, sm, cp, t, *, current=20., upcoming=0., distance=0., stale=False, way=123, producer=41, **kw):
  now=START+round(t*1e9)
  sm.advance(now)
  sm['slcDashboardObservation'].validUntilMonoTime=now+1_000_000_000
  event=packet(speedLimit=current,nextSpeedLimit=upcoming,nextSpeedLimitDistance=distance,wayId=way,
               producerSession=producer,gpsMonoTime=now-100_000_000,computedMonoTime=now,roadName='Road')
  sm.data['mapdOut']=event.mapdOut
  sm.valid['mapdOut']=sm.alive['mapdOut']=not stale
  sm.updated['mapdOut']=True
  sm.logMonoTime['mapdOut']=now
  return runtime.step(sm,cp,now_ns=now,now_boot_ns=now,**kw)


def begin(drive, *, manual=None):
  sm,cp=drive
  runtime=Runtime(parse({'SpeedLimitController':True,'SLCPriority1':'Maps','SLCPriority2':'Dashboard'}))
  sm['carState'].canValid=True
  sm['carState'].vEgo=sm['carState'].vEgoCluster=20.
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=72.
  stable=step(runtime,sm,cp,0)
  assert not stable.result.errors and stable.message.slcState.acceptedSpeedLimit==20.
  if manual is not None:
    sm['carState'].vCruise=sm['carState'].vCruiseCluster=manual*3.6
    at=START+50_000_000
    step(runtime,sm,cp,.05,cruise_event=CruiseEvent(1,at,20.,manual,'accel','card-producer',at))
    assert runtime.state.override.persistent_selected_mps==pytest.approx(manual)
  preview=step(runtime,sm,cp,.1,upcoming=10.,distance=20.)
  assert not preview.result.errors
  assert preview.command is not None
  cmd=preview.command.slcCruiseCommand
  assert cmd.targetMps==10.
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=36.
  ack=step(runtime,sm,cp,.15,upcoming=10.,distance=19.,
    cruise_event=command_receipt(runtime,cmd,.15,(manual or 20.)*3.6,36.,event_id=2))
  assert ack.command is None and runtime.rollback.checkpoint.acknowledged
  return runtime,sm,cp


@pytest.mark.parametrize('manual',[None,25.,15.])
def test_fresh_original_way_restores_exact_target_and_manual_intent(drive,manual):
  runtime,sm,cp=begin(drive,manual=manual)
  before=runtime.state.acceptance.next_presentation_id
  out=step(runtime,sm,cp,.2)
  assert not out.result.errors
  cmd,state=out.command.slcCruiseCommand,out.message.slcState
  assert str(cmd.kind)=='previewRollback' and state.hasRollbackTarget
  assert cmd.targetMps==pytest.approx(manual or 20.) and state.acceptedSpeedLimit==20.
  assert state.presentationId>=before and not state.hasPending
  assert physical_actions.command_applicable(cmd,state,None,'card-producer',START+200_000_000,10.,long_active=True,button_event=False)
  if manual is not None:
    cmd.kind='adoptAcceptedLimit'
    assert not physical_actions.command_applicable(cmd,state,None,'card-producer',START+200_000_000,10.,long_active=True,button_event=False)
    cmd.kind='previewRollback'
  # Held publication stays bound while waiting for Card; no repeated command.
  held=step(runtime,sm,cp,.23)
  assert held.command is None and held.message.slcState.hasRollbackTarget
  assert held.message.slcState.presentationId==state.presentationId
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=(manual or 20.)*3.6
  ack=step(runtime,sm,cp,.25,cruise_event=command_receipt(runtime,cmd,.25,36.,(manual or 20.)*3.6,event_id=3))
  assert ack.command is None and ack.message.slcState.commandStatus=='applied'
  assert runtime.rollback.checkpoint is None
  assert runtime.state.override.persistent_selected_mps==pytest.approx(manual) if manual is not None else runtime.state.override.persistent_selected_mps is None
  assert step(runtime,sm,cp,.3).command is None


@pytest.mark.parametrize('veto',['manual','unclassified','way','producer','clock_gap','offset','disabled'])
def test_intervening_changes_cannot_turn_returned_limit_into_automatic_acceleration(drive,veto):
  runtime,sm,cp=begin(drive)
  kw={};t=.2
  if veto in ('manual','unclassified'):
    sm['carState'].vCruise=sm['carState'].vCruiseCluster=39.6
    if veto=='manual':
      at=START+200_000_000
      kw['cruise_event']=CruiseEvent(3,at,10.,11.,'accel','card-producer',at)
  elif veto=='way':kw['way']=456
  elif veto=='producer':kw['producer']=42
  elif veto=='clock_gap':t=1.
  elif veto=='offset':
    from openpilot.starpilot.speed_limits.speed_domain import OffsetSchedule, OffsetBand
    runtime.update_offsets(OffsetSchedule((OffsetBand(0., None, 1.),)))
  else:
    sm['selfdriveState'].enabled=sm['carControl'].enabled=sm['carControl'].longActive=False
    sm['controlsState'].longControlState='off'
  out=step(runtime,sm,cp,t,**kw)
  assert not out.result.errors and out.command is None and not out.message.slcState.hasRollbackTarget
  assert step(runtime,sm,cp,t+.05).command is None


def test_stale_map_holds_until_fresh_original_way_confirmation(drive):
  runtime,sm,cp=begin(drive)
  for t in (.2,.25,.3):
    out=step(runtime,sm,cp,t,stale=True)
    assert out.command is None and not out.message.slcState.hasRollbackTarget
  restored=step(runtime,sm,cp,.35)
  assert str(restored.command.slcCruiseCommand.kind)=='previewRollback'


def test_reached_lower_zone_commits_without_rollback(drive):
  runtime,sm,cp=begin(drive)
  reached=step(runtime,sm,cp,.2,current=10.,way=456)
  assert not reached.result.errors and reached.command is None
  assert runtime.rollback.checkpoint is None and reached.message.slcState.acceptedSpeedLimit==10.


def test_new_explicit_adoption_can_release_cancelled_checkpoint(drive):
  runtime,sm,cp=begin(drive)
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=39.6
  held=step(runtime,sm,cp,.2)
  shown=held.message.slcState
  out=step(runtime,sm,cp,.25,request=Action(runtime.session_id,9,0,shown.presentationId,'adopt'))
  assert out.command is not None and str(out.command.slcCruiseCommand.kind)=='adoptAcceptedLimit'
  assert out.command.slcCruiseCommand.targetMps==20.


@pytest.mark.parametrize('failure',['no_ack','stale_map','button','changed_target','wrong_kind','missing_binding'])
def test_card_and_acknowledgement_barriers(drive,failure):
  runtime,sm,cp=begin(drive)
  if failure=='no_ack':
    runtime.rollback.checkpoint.acknowledged=False
    assert step(runtime,sm,cp,.2).command is None
    return
  out=step(runtime,sm,cp,.2);cmd,state=out.command.slcCruiseCommand,out.message.slcState
  now=START+200_000_000
  if failure=='missing_binding':state.hasRollbackTarget=False
  elif failure=='wrong_kind':cmd.kind='reengageZone'
  elif failure=='changed_target':cmd.targetMps=30.
  elif failure=='stale_map':
    stale=step(runtime,sm,cp,.25,stale=True)
    state=stale.message.slcState
  assert not physical_actions.command_applicable(cmd,state,None,'card-producer',now,10.,long_active=True,button_event=failure=='button')


def test_stale_confirmation_request_cannot_release_rollback_hold(drive):
  runtime,sm,cp=begin(drive)
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=39.6
  held=step(runtime,sm,cp,.2)
  shown=held.message.slcState
  out=step(runtime,sm,cp,.25,request=Action(runtime.session_id,9,999,shown.presentationId,'accept'))
  assert out.command is None and runtime.rollback.checkpoint is not None
  assert step(runtime,sm,cp,.3).command is None


@pytest.mark.parametrize('failure',['expired','rejected'])
def test_checked_restoration_retries_only_while_target_and_map_remain_current(drive,failure):
  runtime,sm,cp=begin(drive)
  first=step(runtime,sm,cp,.2);cmd=first.command.slcCruiseCommand
  receipt=command_receipt(runtime,cmd,.25,36.,36.,applied=False,event_id=3) if failure=='rejected' else None
  t=.25 if failure=='rejected' else .35
  out=step(runtime,sm,cp,t,cruise_event=receipt)
  assert out.command is not None and out.command.slcCruiseCommand.commandId>cmd.commandId
  assert out.command.slcCruiseCommand.targetMps==20.
  assert step(runtime,sm,cp,t+.02,stale=True).command is None
