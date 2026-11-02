"""SP-011 deterministic regressions for the 29Oct input failure paths."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import json
import pytest
from openpilot.starpilot.controllers.runtime import ControllerRuntime
from openpilot.starpilot.controllers.tests.test_tesla_worker import setup, Channel
from openpilot.starpilot.controllers.tests.test_tesla_decoder import wheel
from openpilot.starpilot.common.car_control_config import PARAM, SPEED_UP, software_cruise_requested


def bare_runtime(presses=(), capture=None):
  r=ControllerRuntime.__new__(ControllerRuntime)
  r._closed=False; r._next_warm=float('inf'); r._warm_until=0
  r.reader=NS(poll=lambda _:presses,devices=lambda:[],close=Mock())
  r.owner=NS(set_devices=Mock(),tick=Mock(),feed=Mock())
  r.tesla=NS(poll=Mock(),close=Mock());r.service=NS(poll=Mock(),close=Mock())
  r.authority=NS(close=Mock());r.input_capture=lambda:capture
  return r


def test_tesla_exception_without_button_press_does_not_close_all_controllers():
  r=bare_runtime();r.tesla.poll.side_effect=ValueError('injected dispatch failure')
  r.poll()
  assert not r._closed and r.tesla.next_start > 0
  r.tesla.close.assert_called_once();r.service.poll.assert_called_once()


def test_modal_capture_receives_each_button_once():
  capture=Mock();press=object();r=bare_runtime([press],capture)
  r.poll();r.owner.feed.assert_called_once_with(press,capture=capture)


def test_identical_periodic_carparams_does_not_drop_held_scroll(setup):
  s=setup;s.feed(wheel());s.feed(wheel(right_scroll=1))
  s.pump.set_vehicle(s.cp.as_reader())
  s.feed(wheel());assert s.calls==[0]


def test_speed_input_does_not_build_unrelated_action_catalogue(setup):
  s=setup;s.receiver.speed_actions=lambda:s.actions
  s.receiver.actions=Mock(side_effect=ValueError('expensive or failing UI page'))
  s.feed(wheel());s.feed(wheel(right_scroll=1));s.feed(wheel())
  assert s.calls==[0];s.receiver.actions.assert_not_called()


def test_worker_recovery_is_bounded_and_requires_fresh_drive(setup):
  s=setup;created=[]
  def factory():
    c=Channel();created.append(c);return c
  s.receiver.channel_factory=factory
  s.receiver.close()
  s.clock[0]+=3;s.sm.refresh(s.clock[0]);s.receiver.poll()
  assert len(created)==1
  for _ in range(3):
    s.receiver.close();s.clock[0]+=3;s.sm.refresh(s.clock[0]);s.receiver.poll()
  assert len(created)==3 and s.receiver.channel is None
  s.clock[0]+=10;s.receiver.poll();assert len(created)==3


def test_assignment_size_is_consistent_with_ownership(setup):
  s=setup;s.cp.openpilotLongitudinalControl=True;s.pump.set_vehicle(s.cp)
  raw=(' '*17000+json.dumps(s.doc)).encode()
  s.params.put(PARAM,raw,block=True);s.params.put('SpeedLimitController',b'1',block=True)
  assert software_cruise_requested(s.params,s.cp)
  s.feed(wheel());s.feed(wheel(right_scroll=1));s.feed(wheel());assert s.calls==[0]


@pytest.mark.parametrize('sign,key', [(1,SPEED_UP),(-1,'decrease_comma_set_speed')])
def test_physical_speed_catalogue_steps_and_replay_guard(setup,monkeypatch,sign,key):
  # Execute the exact UI method without loading a graphics window.
  import ast
  from pathlib import Path
  import time
  from openpilot.starpilot.favorites.state import FavoriteAction
  from openpilot.starpilot.common.car_control_config import SPEED_DOWN
  key=SPEED_UP if sign>0 else SPEED_DOWN
  tree=ast.parse((Path(__file__).parents[2]/'ui/runtime_app.py').read_text())
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='StarShellSession')
  fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_comma_speed_actions')
  setup.params.get_bool=lambda key: (setup.params.path/key).read_bytes()==b'1' if (setup.params.path/key).exists() else False
  state=NS(started_frame=123,CP=setup.cp,params=setup.params,sm=setup.sm,replay_clock=None)
  publisher=object();dispatch=Mock(return_value=True);owner=NS(cruise_actions=NS(dispatch=dispatch),_favorite_authority=lambda:False)
  scope=dict(ui_state=state,time=time,FavoriteAction=FavoriteAction,
    cruise_action_authority=lambda *a,**k:True,_slc_action_publisher=lambda:publisher)
  exec(compile(ast.Module(body=[fn],type_ignores=[]),'runtime_app._comma_speed_actions','exec'),scope)
  actions=scope['_comma_speed_actions'](owner)
  assert actions[key].available
  favorites=scope['_comma_speed_actions'](owner,require_display=True)
  assert not favorites[key].available and favorites[key].invoke_value(1) is False
  assert actions[key].invoke_value(0) is True
  assert dispatch.call_args.kwargs['step_units']==sign
  assert actions[key].invoke_value(1) is True
  assert dispatch.call_args.kwargs['step_units']==sign*5
  # A previously discovered callback must reject playback as well.
  state.replay_clock=object()
  assert actions[key].invoke_value(0) is False
  assert not scope['_comma_speed_actions'](owner)[key].available
  assert dispatch.call_count==2


def test_recovered_worker_must_see_neutral_before_a_new_gesture(setup):
  s=setup
  s.feed(wheel(right_scroll=2));s.feed(wheel())
  assert not s.calls
  s.feed(wheel(right_scroll=1));s.feed(wheel())
  assert s.calls==[0]
