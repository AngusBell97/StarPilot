"""SP-041: bounded UI dispatch; raw CAN decoding lives in a separate process."""
from openpilot.common.swaglog import cloudlog as LOGGER
import os
import socket
import subprocess
import sys
import time

from openpilot.starpilot.common.car_control_config import (
  PARAM, MAX_ASSIGNMENT_BYTES, parse_assignments, vehicle_supported, screen_reserved, SPEED_UP, SPEED_DOWN,
)
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.controllers.wheel_actions import cp_fingerprint
from openpilot.starpilot.controllers.tesla_input_protocol import (
  MAX_PACKET, MAX_EVENTS, decode_event, document_hash, drive_id,
)



class WorkerChannel:
  """An inherited private socketpair: no listening port, file socket or network."""
  def __init__(self):
    self.ready = False
    self.socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    self.socket.setblocking(False)
    try:
      self.process = subprocess.Popen(
        [sys.executable, '-B', '-m', 'openpilot.starpilot.controllers.tesla_input_worker',
         str(child.fileno()), str(os.getpid())],
        pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL,
        # The ordinary process logger retains failures; never pipe unconsumed output.
        close_fds=True,
      )
    except BaseException:
      self.socket.close()
      raise
    finally:
      child.close()

  def receive(self):
    if self.process.poll() is not None:
      raise EOFError('Tesla input worker exited')
    try:
      value = self.socket.recv(MAX_PACKET + 1)
    except BlockingIOError:
      return None
    if not value:
      raise EOFError('Tesla input worker disconnected')
    if value == b'SP041_READY':
      self.ready = True
      return None
    return value

  def close(self):
    self.socket.close()
    if self.process.poll() is None:
      self.process.terminate()
    # close is shutdown/failure only, never normal frame work.
    try:
      self.process.wait(timeout=.1)
    except subprocess.TimeoutExpired:
      self.process.kill()
      self.process.wait(timeout=.1)


class TeslaReceiver:
  def __init__(self, params, actions, vehicle, *, clock=time.monotonic,
               messages=lambda: None, channel_factory=WorkerChannel, speed_actions=None, keyed_actions=None):
    self.params, self.actions, self.vehicle = params, actions, vehicle
    self.speed_actions = speed_actions
    # Looks up only the requested keys. The full catalog checks every settings
    # page (including the maps package), which took 2.3 s on a cold first press
    # and outlived MAX_AGE, so the press was dropped.
    self.keyed_actions = keyed_actions
    self._restart_drive = None
    self._restarts = 0
    self._diagnostic_at = 0.
    self.clock, self.messages, self.channel_factory = clock, messages, channel_factory
    self.channel = None
    self.sequence = 0
    self.next_start = 0.
    self.failed = False
    self._start()

  def _start(self):
    try:
      self.channel = self.channel_factory()
      self.sequence = 0
      self.failed = False
    except (OSError, RuntimeError):
      LOGGER.exception('Tesla input worker could not start')
      self.failed = True
    self.next_start = self.clock() + 2.

  def close(self):
    if self.channel is not None:
      channel, self.channel = self.channel, None
      channel.close()

  def drive(self):
    return drive_id(self.messages(), self.clock())

  def _offroad(self):
    sm = self.messages()
    now = self.clock()
    try:
      return bool(sm.seen['deviceState'] and sm.alive['deviceState'] and sm.valid['deviceState']
                  and not sm['deviceState'].started
                  and 0 <= now - sm.logMonoTime['deviceState'] / 1e9 <= 1.
                  and 0 <= now - sm.recv_time['deviceState'] <= 1.)
    except (AttributeError, KeyError, TypeError, ValueError):
      return False

  def poll(self):
    if self.channel is None:
      # Receive-only recovery is bounded per drive; a new worker must observe
      # a fresh neutral gesture and exact current authority before dispatch.
      drive = self.drive()
      if drive != self._restart_drive and drive is not None:
        self._restart_drive, self._restarts = drive, 0
      if self.clock() >= self.next_start and (self._offroad() or drive is not None and self._restarts < 3):
        self._restarts += 1
        self._start()
      return
    try:
      packets = []
      for _ in range(MAX_EVENTS + 1):
        raw = self.channel.receive()
        if raw is None:
          break
        packets.append(raw)
      if len(packets) > MAX_EVENTS:
        # Never replay a backlog across frames. Discard and reset the worker.
        raise RuntimeError('Tesla input event backlog')
      for raw in packets:
        self._dispatch(raw)
    except (OSError, EOFError, RuntimeError):
      LOGGER.exception('Tesla input worker stopped; bounded fresh-context recovery scheduled')
      self.failed = True
      self.close()
      self.next_start = self.clock() + 2.

  def _dispatch(self, raw):
    event = decode_event(raw, self.clock())
    if event is None or event['sequence'] <= self.sequence:
      self._diagnostic('invalid_expired_or_duplicate', event or {})
      return
    self.sequence = event['sequence']
    cp = self.vehicle()
    if not vehicle_supported(cp) or self.drive() != event['drive']:
      self._diagnostic('vehicle_or_drive_unavailable', event)
      return
    if read_saved(self.params, 'SafeMode', 8) not in ((None, True), (b'0', True)):
      return
    current, readable = read_saved(self.params, PARAM, MAX_ASSIGNMENT_BYTES)
    if not readable or current is None or document_hash(current) != event['document']:
      self._diagnostic('assignment_changed_or_unreadable', event)
      return
    doc = parse_assignments(current)
    if not doc or doc['vehicle'] != cp.carFingerprint or cp_fingerprint(cp) != event['vehicle']:
      self._diagnostic('assignment_vehicle_identity_mismatch', event)
      return
    name = event['name']
    if name == 'screen_three_finger_tap' and screen_reserved(cp):
      return
    binding = doc['bindings'].get(name)
    if binding is None:
      return
    if binding['key'] in (SPEED_UP, SPEED_DOWN) and self.speed_actions is not None:
      action = self.speed_actions().get(binding['key'])
    elif self.keyed_actions is not None:
      action = self.keyed_actions({binding['key']}).get(binding['key'])
    else:
      action = self.actions().get(binding['key'])
    if action is None or not action.available:
      self._diagnostic('action_unavailable', event)
      return
    # Action discovery may perform work. Recheck exact context at dispatch.
    if (decode_event(raw, self.clock()) is None or self.drive() != event['drive']
        or read_saved(self.params, PARAM, MAX_ASSIGNMENT_BYTES) != (current, True)
        or read_saved(self.params, 'SafeMode', 8) not in ((None, True), (b'0', True))
        or not vehicle_supported(self.vehicle()) or cp_fingerprint(self.vehicle()) != event['vehicle']):
      self._diagnostic('context_changed_or_expired', event)
      return
    value = binding['value']
    if binding['key'] in (SPEED_UP, SPEED_DOWN):
      if value is None and action.invoke_value is not None:
        from openpilot.starpilot.controllers.tesla_gesture_context import source_scope
        with source_scope(event):
          applied = action.invoke_value(1 if event['fast'] else 0)
        self._diagnostic('dispatched' if applied is not False else 'publisher_rejected', event)
    elif value is None:
      if action.invoke is not None:
        action.invoke()
    elif action.invoke_value is not None:
      action.invoke_value(value)


  def _diagnostic(self, reason, event):
    now = self.clock()
    if now < self._diagnostic_at:
      return
    self._diagnostic_at = now + .1
    LOGGER.info({'event':'tesla_speed_input', 'reason':reason, 'input':event.get('name',''),
                 'sequence':event.get('sequence',0), 'age_ms':round((now-event.get('stamp',now))*1000, 1)})
