from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from openpilot.cereal import messaging
from openpilot.starpilot.audio.slc_warning import SlcWarning
from openpilot.starpilot.ui.slc_warning_visual import WarningVisual
from openpilot.starpilot.ui.onroad_state import OnroadState,SpeedLimitObservation

NOW=10_000_000_000
class SM(dict):pass

def fixture():
  msg=messaging.new_message('slcState');v=msg.slcState
  v.enabled=True;v.changeWarningId='boundary';v.changeWarningIssuedMonoTime=NOW;v.changeWarningDeadlineMonoTime=NOW+250_000_000
  sm=SM(slcState=v,selfdriveState=NS(enabled=True))
  sm.valid=sm.alive={'slcState':True,'selfdriveState':True};sm.logMonoTime={'slcState':NOW}
  return sm

@pytest.mark.parametrize('failure',['busy','muted','old','future','off','invalid','disabled'])
def test_audio_delivery_never_replays_an_ineligible_event(failure):
  sm=fixture();gate=SlcWarning(started=9.)
  if failure=='old':sm.logMonoTime['slcState']=NOW-100_000_001
  if failure=='future':sm.logMonoTime['slcState']=NOW+1
  if failure=='off':sm['selfdriveState'].enabled=False
  if failure=='invalid':sm.valid['slcState']=False
  if failure=='disabled':sm['slcState'].enabled=False
  assert not gate.update(sm,NOW,busy=failure=='busy',enabled=failure!='muted')
  sm=fixture();assert not gate.update(sm,NOW+10_000_000,busy=False,enabled=True)


def test_actual_payload_once_and_late_process_start_cannot_play_old_warning():
  sm=fixture();gate=SlcWarning(started=9.)
  assert gate.update(sm,NOW,busy=False,enabled=True)
  assert not gate.update(sm,NOW+50_000_000,busy=False,enabled=True)
  assert not SlcWarning(started=10.1).update(sm,NOW+150_000_000,busy=False,enabled=True)


def test_visual_uses_original_publication_freshness_and_resets_at_drive_boundary():
  obs=SpeedLimitObservation(warning_id='boundary',warning_issued_ns=NOW,warning_frame_ns=NOW,
    warning_pin='boundary',warning_pin_issued_ns=NOW)
  state=OnroadState(True,False,20.,72.,obs,observed_ns=NOW,longitudinal_active=True,drive_frame=1)
  visual=WarningVisual();assert visual.update(state)==(True,1.)
  assert not visual.update(replace(state,observed_ns=NOW+100_000_001))[0]
  assert not visual.update(replace(state,engaged=False))[0]
  assert not visual.update(replace(state,drive_frame=2,speed_limit=SpeedLimitObservation()))[0]
