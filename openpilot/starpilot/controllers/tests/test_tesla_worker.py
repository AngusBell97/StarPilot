"""SP-041 native message, lifecycle and handoff regressions."""
import collections
import json
import os
import time
from types import SimpleNamespace as NS

import pytest
from openpilot.cereal import messaging
from opendbc.car.structs import car
from opendbc.car.tesla.values import TeslaFlags
from openpilot.starpilot.common.car_control_config import PARAM, SPEED_UP
from openpilot.starpilot.controllers.tesla_receiver import TeslaReceiver, WorkerChannel
from openpilot.starpilot.controllers.tesla_input_worker import InputPump
from openpilot.starpilot.controllers.tesla_input_protocol import encode_event, decode_event, MAX_PACKET
from openpilot.starpilot.controllers.tests.test_tesla_decoder import wheel
from openpilot.starpilot.tests.port_settings_fixture import Saved


class Messages(dict):
  def refresh(self, now, *, started=True):
    self['deviceState'] = NS(started=started, startedMonoTime=100_000_000)
    self['carState'] = NS(canValid=True, canTimeout=False)
    self.seen = self.alive = self.valid = {k: True for k in self}
    self.logMonoTime = {k: int(now * 1e9) for k in self}
    self.recv_time = {k: now for k in self}


class Channel:
  def __init__(self):
    self.queue = collections.deque()
    self.closed = False
    self.receives = 0

  def receive(self):
    self.receives += 1
    return self.queue.popleft() if self.queue else None

  def close(self):
    self.closed = True


@pytest.fixture
def setup(tmp_path):
  params = Saved(tmp_path)
  cp = car.CarParams.new_message(carFingerprint='TESLA_MODEL_3', brand='tesla',
                                flags=int(TeslaFlags.HAS_VEHICLE_BUS), pcmCruise=True)
  doc = dict(version=1, vehicle='TESLA_MODEL_3', bindings={
    'right_wheel_up': dict(key=SPEED_UP, value=None, label='Up')})
  params.put(PARAM, json.dumps(doc).encode(), block=True)
  clock = [1.]
  sm = Messages()
  sm.refresh(clock[0])
  calls = []
  channel = Channel()
  frames = collections.deque()
  actions = {SPEED_UP: NS(available=True, invoke_value=calls.append, invoke=None)}
  receiver = TeslaReceiver(params, lambda: actions, lambda: cp, clock=lambda: clock[0],
                           messages=lambda: sm, channel_factory=lambda: channel)
  pump = InputPump(params, channel.queue.append, clock=lambda: clock[0],
                   socket_factory=object, receive=lambda _: frames.popleft() if frames else None)
  pump.set_vehicle(cp)

  def feed(data, delta=.1, bus=1, valid=True, address=0x3c2, dispatch=True):
    clock[0] += delta
    sm.refresh(clock[0])
    ev = messaging.new_message('can', 1, valid=valid, logMonoTime=int(clock[0]*1e9))
    ev.can[0] = dict(address=address, dat=data, src=bus)
    frames.append(ev)
    pump.poll(sm)
    if dispatch:
      receiver.poll()

  return NS(params=params, cp=cp, doc=doc, clock=clock, sm=sm, calls=calls,
            channel=channel, frames=frames, receiver=receiver, pump=pump, feed=feed, actions=actions)


@pytest.mark.parametrize('ticks', [1, 2, 3, 6])
def test_end_to_end_release_preserves_speed_action(setup, ticks):
  s = setup
  s.feed(wheel())
  s.feed(wheel(right_scroll=ticks))
  assert not s.calls
  s.feed(wheel())
  s.feed(wheel())
  assert s.calls == [0 if ticks == 1 else 1]


@pytest.mark.parametrize('fault', ['safe', 'config', 'invalid', 'wrong_bus', 'offroad', 'vehicle', 'stale'])
def test_held_gesture_cannot_cross_context(setup, fault):
  s = setup
  s.feed(wheel())
  s.feed(wheel(right_scroll=2))
  if fault == 'safe':
    s.params.put('SafeMode', b'1', block=True)
  if fault == 'config':
    s.doc['bindings']['right_wheel_up']['label'] = 'Changed'
    s.params.put(PARAM, json.dumps(s.doc).encode(), block=True)
  if fault == 'vehicle':
    s.cp.mass = 1234.
    s.pump.set_vehicle(s.cp)
  if fault == 'offroad':
    s.sm.refresh(s.clock[0], started=False)
    s.pump.poll(s.sm)
  s.feed(wheel(), valid=fault != 'invalid', bus=0 if fault == 'wrong_bus' else 1,
         delta=1 if fault == 'stale' else .1)
  assert not s.calls


@pytest.mark.parametrize('fault', ['safe', 'config', 'vehicle', 'offroad', 'stale', 'can_invalid', 'availability'])
def test_ui_revalidates_worker_event(setup, fault):
  s = setup
  s.feed(wheel())
  s.feed(wheel(right_scroll=2))
  s.feed(wheel(), dispatch=False)
  assert len(s.channel.queue) == 1
  if fault == 'safe':
    s.params.put('SafeMode', b'1', block=True)
  if fault == 'config':
    s.doc['bindings']['right_wheel_up']['label'] = 'New'
    s.params.put(PARAM, json.dumps(s.doc).encode(), block=True)
  if fault == 'vehicle':
    s.cp.mass = 1234.
  if fault == 'offroad':
    s.sm['deviceState'].started = False
  if fault == 'stale':
    s.clock[0] += .251
  if fault == 'can_invalid':
    s.sm['carState'].canValid = False
  if fault == 'availability':
    s.actions[SPEED_UP].available = False
  s.receiver.poll()
  assert not s.calls


def test_revalidation_after_action_discovery(setup):
  s = setup
  s.feed(wheel())
  s.feed(wheel(right_scroll=2))
  s.feed(wheel(), dispatch=False)
  def actions():
    s.params.put('SafeMode', b'1', block=True)
    return s.actions
  s.receiver.actions = actions
  s.receiver.poll()
  assert not s.calls


def test_duplicate_event_does_not_repeat(setup):
  s = setup
  s.feed(wheel())
  s.feed(wheel(right_scroll=2))
  s.feed(wheel(), dispatch=False)
  packet = s.channel.queue[0]
  s.channel.queue.append(packet)
  s.receiver.poll()
  assert s.calls == [1]


def test_no_can_or_settings_work_without_event(setup):
  s = setup
  def forbidden(*a, **kw):
    raise AssertionError('idle UI accessed source context')
  s.receiver.params = NS(get_param_path=forbidden)
  s.receiver.vehicle = s.receiver.messages = s.receiver.actions = forbidden
  for _ in range(1000):
    s.receiver.poll()
  assert s.channel.receives == 1000


def test_backlog_is_bounded_and_discarded(setup):
  s = setup
  s.channel.queue.extend([b'{}'] * 500)
  s.receiver.poll()
  assert s.channel.receives == 5
  assert s.channel.closed
  assert not s.calls
  # Never immediately replay a discarded backlog. Recovery has a backoff.
  s.receiver.poll()
  assert s.receiver.channel is None


@pytest.mark.parametrize('patch', [
  {'stamp':float('nan')}, {'stamp':float('inf')}, {'stamp':1.3}, {'stamp':.7},
  {'sequence':True}, {'sequence':0}, {'version':True}, {'fast':1}, {'drive':0},
  {'document':'x'*64}, {'vehicle':None}, {'extra':'field'}, {'name':3},
])
def test_protocol_rejects_malformed_or_stale(patch):
  d=json.loads(encode_event(1,'right_wheel_up',True,1.,100_000_000,'a'*64,'b'*64))
  d.update(patch)
  assert decode_event(json.dumps(d).encode(),1.) is None


def test_protocol_size_and_boundary():
  packet=encode_event(1,'right_wheel_up',True,1.,100_000_000,'a'*64,'b'*64)
  assert decode_event(packet,1.25) is not None
  assert decode_event(packet,1.250001) is None
  assert decode_event(b' '*(MAX_PACKET+1),1) is None


def test_unrelated_frames_never_convert_payload(setup):
  s=setup
  class Payload:
    def __bytes__(self):
      raise AssertionError('unrelated payload converted')
  s.frames.append(NS(valid=True,logMonoTime=int(s.clock[0]*1e9),
                     can=[NS(src=0,address=0x3c2,dat=Payload()),
                          NS(src=1,address=0x555,dat=Payload())]))
  s.pump.poll(s.sm)
  assert not s.calls


def test_worker_tick_is_bounded(setup):
  s=setup
  s.frames.extend(NS(valid=True,logMonoTime=int(s.clock[0]*1e9),can=[]) for _ in range(1000))
  s.pump.poll(s.sm)
  assert len(s.frames)==744


def test_transport_overflow_fails_without_replay(setup):
  s=setup
  s.feed(wheel())
  s.feed(wheel(right_scroll=2))
  def fail(_):
    raise BlockingIOError()
  s.pump.send=fail
  with pytest.raises(BlockingIOError):
    s.feed(wheel())
  assert s.pump.decoder is None
  assert not s.calls


def test_worker_real_process_lifecycle():
  channel=WorkerChannel()
  try:
    # Real imports, isolated Params/MSGQ, scheduler reset and socket closure.
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
      assert channel.receive() is None
      if channel.ready:
        break
      time.sleep(.01)
    assert channel.ready, "worker did not complete native startup"
    pid=channel.process.pid
    assert pid!=os.getpid()
    assert os.sched_getscheduler(pid)==os.SCHED_OTHER
    assert os.getpriority(os.PRIO_PROCESS,pid)==10
    assert os.sched_getaffinity(pid).issubset({0,1,2,3})
  finally:
    channel.close()
  assert channel.process.poll() is not None



def test_lost_telemetry_does_not_relaunch_failed_worker(setup):
  s=setup
  s.receiver.close()
  s.receiver.next_start=0
  s.sm['carState'].canValid=False
  s.receiver.poll()
  assert s.receiver.channel is None
  s.sm.refresh(s.clock[0],started=False)
  s.receiver.poll()
  assert s.receiver.channel is s.channel


@pytest.mark.parametrize('value',[None,0,2.5])
def test_generic_assigned_actions_keep_value_semantics(setup,value):
  s=setup
  s.doc['bindings']['right_wheel_up']=dict(key='custom',value=value,label='Action')
  s.params.put(PARAM,json.dumps(s.doc).encode(),block=True)
  s.actions['custom']=NS(available=True,invoke=lambda:s.calls.append('invoke'),invoke_value=s.calls.append)
  s.feed(wheel());s.feed(wheel(right_scroll=1));s.feed(wheel())
  assert s.calls==['invoke' if value is None else value]


def test_peer_close_stops_worker_without_gesture():
  channel=WorkerChannel()
  try:
    deadline=time.monotonic()+15
    while time.monotonic()<deadline and not channel.ready:
      assert channel.receive() is None
      time.sleep(.01)
    assert channel.ready
    channel.socket.close()
    assert channel.process.wait(timeout=5)==0
  finally:
    channel.close()


def test_real_worker_can_to_ui_dispatch(tmp_path, monkeypatch):
  # Explicit private Params root; the child inherits it. No live device settings.
  from openpilot.common.params import Params
  from openpilot.starpilot.controllers.tesla_input_protocol import drive_id
  root=tmp_path/'native-params'
  root.mkdir()
  monkeypatch.setenv('PARAMS_ROOT',str(root))
  params=Params(str(root))
  cp=car.CarParams.new_message(carFingerprint='TESLA_MODEL_3',brand='tesla',
                              flags=int(TeslaFlags.HAS_VEHICLE_BUS),pcmCruise=True)
  params.put(PARAM,dict(version=1,vehicle='TESLA_MODEL_3',bindings={
    'right_wheel_up':dict(key=SPEED_UP,value=None,label='Up')}),block=True)
  pm=messaging.PubMaster(['deviceState','carState','carParams','can'])
  channel=WorkerChannel()
  sm=Messages();calls=[]
  receiver=TeslaReceiver(params,lambda:{SPEED_UP:NS(available=True,invoke=None,invoke_value=calls.append)},
                         lambda:cp,messages=lambda:sm,channel_factory=lambda:channel)
  try:
    deadline=time.monotonic()+15
    while time.monotonic()<deadline and not channel.ready:
      assert channel.receive() is None
      time.sleep(.01)
    assert channel.ready
    start=time.monotonic()
    drive=int((start-1)*1e9)
    # Fresh telemetry and real CAN neutral -> press -> neutral across processes.
    while time.monotonic()-start<1.2:
      now=time.monotonic();elapsed=now-start;stamp=int(now*1e9)
      sm.refresh(now);sm['deviceState'].startedMonoTime=drive
      state=messaging.new_message('deviceState',valid=True,logMonoTime=stamp)
      state.deviceState.started=True;state.deviceState.startedMonoTime=drive
      pm.send('deviceState',state)
      state=messaging.new_message('carState',valid=True,logMonoTime=stamp)
      state.carState.canValid=True
      pm.send('carState',state)
      # Publish CP only during startup, not continually resetting a held gesture.
      if elapsed<.2:
        state=messaging.new_message('carParams',valid=True,logMonoTime=stamp)
        state.carParams=cp
        pm.send('carParams',state)
      event=messaging.new_message('can',1,valid=True,logMonoTime=stamp)
      event.can[0]=dict(address=0x3c2,src=1,
                        dat=wheel(right_scroll=2 if .4<=elapsed<.6 else 0))
      pm.send('can',event)
      receiver.poll()
      time.sleep(.01)
    assert calls==[1]
  finally:
    receiver.close()

