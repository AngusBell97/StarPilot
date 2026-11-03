"""Late real authority, never freshness extensions or cached mode holds."""

from types import SimpleNamespace
import unittest

from openpilot.cereal import messaging
from openpilot.starpilot.conditional_mode.planner_host import AuthorityRefresher
from openpilot.starpilot.conditional_mode.projection import _fresh
from openpilot.starpilot.conditional_mode.tests.test_projection import (
  BOOT, CP, MONO, FakeSubMaster, SceneProjector, serialized_scene,
)


class TestAuthorityRefresh(unittest.TestCase):
  def setUp(self):
    self.now = MONO + 200_000_000
    self.sm = FakeSubMaster(serialized_scene(stamp=BOOT + 190_000_000), self.now - 10_000_000)
    self.sm.sock = {service: service for service in ('carControl', 'selfdriveState', 'carState')}
    self.sm.logMonoTime['carControl'] = self.now - 55_000_000
    self.sm.recv_time['carControl'] = (self.now - 54_000_000) / 1e9
    self.events = {}
    self.calls = []
    self.refresher = AuthorityRefresher(dict(self.sm.sock))
    self.sm.sock = {}  # A conditional refresh must never drain native sockets.

  def receive(self, service):
    self.calls.append(service)
    return self.events.get(service)

  def event(self, service, *, age=2_000_000, valid=True, **changes):
    payload = self.sm[service].as_builder()
    for key, value in changes.items():
      setattr(payload, key, value)
    self.events[service] = SimpleNamespace(logMonoTime=self.now - age, valid=valid, **{service: payload.as_reader()})

  def refresh(self):
    return self.refresher.sample(self.sm, self.receive, monotonic=lambda: self.now / 1e9)

  def project(self, sm):
    projector = SceneProjector()
    projector.offset_ns = BOOT - MONO
    projector.barrier_mono_ns = MONO
    return projector.project(sm, CP, now_mono_ns=self.now, now_boot_ns=BOOT + 200_000_000,
                             sample_skew_ns=1000, safe_mode=False)

  def test_recorded_55ms_gap_recovers_only_from_real_new_event(self):
    self.assertIsNone(self.project(self.sm).authority)
    self.event('carControl')
    view = self.refresh()
    self.assertTrue(self.project(view).authority.long_active)
    self.assertEqual(self.calls, ['carControl', 'selfdriveState', 'carState'])
    self.assertEqual(self.sm.logMonoTime['carControl'], self.now - 55_000_000)
    for service in ('modelV2', 'carState', 'radarState', 'longitudinalPlan'):
      self.assertIs(view[service], self.sm[service])
      self.assertEqual(view.logMonoTime[service], self.sm.logMonoTime[service])
      self.assertEqual(view.recv_time[service], self.sm.recv_time[service])

  def test_stale_native_car_state_recovers_from_real_new_event(self):
    # 2 Nov drive: planner work before the conditional sample let carState age past 50 ms, so the
    # projection lost speed and authority for one cycle. A real newer carState restores it; the
    # native frame and its timestamps are left untouched, and no freshness bound is extended.
    self.event('carControl')
    self.sm.logMonoTime['carState'] = self.now - 55_000_000
    self.sm.recv_time['carState'] = (self.now - 54_000_000) / 1e9
    self.assertIsNone(self.project(self.refresh()).authority)
    native_car = self.sm['carState']
    self.events.clear()  # carControl stays cached from its one real delivery
    self.event('carState')
    view = self.refresh()
    self.assertTrue(self.project(view).authority.long_active)
    self.assertIs(self.sm['carState'], native_car)
    self.assertEqual(self.sm.logMonoTime['carState'], self.now - 55_000_000)
    self.assertEqual(view.logMonoTime['carState'], self.now - 2_000_000)

  def test_stale_or_invalid_refreshed_car_state_fails_closed(self):
    self.event('carControl')
    self.sm.logMonoTime['carState'] = self.now - 55_000_000
    self.sm.recv_time['carState'] = (self.now - 54_000_000) / 1e9
    for kwargs in ({'age': 50_000_001}, {'valid': False}, {'age': -1}):
      with self.subTest(kwargs=kwargs):
        self.refresher.reset()
        self.event('carState', **kwargs)
        self.assertIsNone(self.project(self.refresh()).authority)

  def test_no_event_does_not_renew_receipt_or_authority(self):
    view = self.refresh()
    self.assertEqual(view.recv_time, self.sm.recv_time)
    self.assertIsNone(self.project(view).authority)

  def test_stale_invalid_backwards_and_future_events_fail_closed(self):
    for kwargs in ({'age': 50_000_001}, {'valid': False}, {'age': 60_000_000}, {'age': -1}):
      with self.subTest(kwargs=kwargs):
        self.event('carControl', **kwargs)
        self.assertIsNone(self.project(self.refresh()).authority)

  def test_current_disengagement_and_long_inactive_take_effect(self):
    self.event('carControl', longActive=False)
    self.event('selfdriveState', enabled=False)
    authority = self.project(self.refresh()).authority
    self.assertFalse(authority.long_active)
    self.assertFalse(authority.driving_enabled)

  def test_original_50ms_expiry_and_barrier_remain_strict(self):
    self.event('carControl', age=50_000_000)
    view = self.refresh()
    self.assertTrue(_fresh(view, 'carControl', self.now, MONO))
    self.assertFalse(_fresh(view, 'carControl', self.now + 1, MONO))
    self.assertFalse(_fresh(view, 'carControl', self.now, self.now))

  def test_consumed_packet_keeps_original_receipt_and_expires_without_successor(self):
    self.event('carControl')
    first = self.refresh()
    self.events.clear()
    self.now += 20_000_000
    second = self.refresh()
    self.assertEqual(second.recv_time['carControl'], first.recv_time['carControl'])
    self.assertTrue(_fresh(second, 'carControl', self.now, MONO))
    self.now += 30_000_000
    self.assertFalse(_fresh(self.refresh(), 'carControl', self.now, MONO))

  def test_native_same_stamp_invalid_frame_supersedes_cached_valid_frame(self):
    self.event('carControl')
    self.refresh()
    self.events.clear()
    self.sm.logMonoTime['carControl'] = self.now - 2_000_000
    self.sm.valid['carControl'] = False
    self.assertIsNone(self.project(self.refresh()).authority)

  def test_reset_and_close_remove_cached_packets(self):
    self.event('carControl')
    self.refresh()
    self.refresher.reset()
    self.events.clear()
    self.assertEqual(self.refresher.latest, {})
    self.assertIsNone(self.project(self.refresh()).authority)
    self.refresher.close()
    self.assertEqual(self.refresher.sockets, {})

  def test_future_packet_cannot_become_valid_when_clock_catches_up(self):
    self.event('carControl', age=-1_000_000)
    self.assertIsNone(self.project(self.refresh()).authority)
    self.events.clear()
    self.now += 2_000_000
    self.assertFalse(_fresh(self.refresh(), 'carControl', self.now, MONO))

  def test_late_native_duplicate_cannot_hide_cached_disengagement(self):
    self.sm.logMonoTime['carControl'] = self.now - 10_000_000
    self.sm.recv_time['carControl'] = (self.now - 9_000_000) / 1e9
    self.event('carControl', longActive=False)
    self.assertFalse(self.project(self.refresh()).authority.long_active)
    self.event('carControl', age=10_000_000, longActive=True)
    self.assertFalse(self.project(self.refresh()).authority.long_active)

  def test_invalid_late_duplicate_cannot_erase_newer_authority_evidence(self):
    self.sm.logMonoTime['carControl'] = self.now - 10_000_000
    self.sm.recv_time['carControl'] = (self.now - 9_000_000) / 1e9
    self.event('carControl', longActive=False)
    self.refresh()
    self.event('carControl', age=10_000_000, valid=False)
    self.assertIsNone(self.project(self.refresh()).authority)
    self.events.clear()
    self.assertIsNone(self.project(self.refresh()).authority)

  def test_real_submaster_snapshot_preserves_native_payload_and_validity(self):
    # Exercise actual SubMaster mappings and serialized events, without owning
    # publishers or reading the native planner's sockets.
    native = messaging.SubMaster(list(self.sm.payloads))
    events = []
    for service, payload in self.sm.payloads.items():
      event = messaging.new_message(service, valid=self.sm.valid[service])
      event.logMonoTime = self.sm.logMonoTime[service]
      setattr(event, service, payload)
      events.append(event.as_reader())
    native.update_msgs((self.now - 1_000_000) / 1e9, events)
    original_payload = native['carControl']
    self.event('carControl', longActive=False)
    view = self.refresher.sample(native, self.receive, monotonic=lambda: self.now / 1e9)
    self.assertFalse(self.project(view).authority.long_active)
    self.assertIs(native['carControl'], original_payload)
    self.assertTrue(native['carControl'].longActive)
    self.assertEqual(native.logMonoTime['carControl'], self.now - 55_000_000)
    self.assertIs(view['modelV2'], native['modelV2'])

  def test_actual_selfdrived_rejection_log_uses_schema_and_throttles(self):
    from unittest.mock import patch
    from opendbc.car.honda.interface import CarInterface
    from opendbc.car.honda.values import CAR
    from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
    from openpilot.starpilot.conditional_mode.tests.test_status_consumer import TestModeTransport

    fixture = TestModeTransport()
    fixture.setUp()
    self.addCleanup(fixture.doCleanups)
    drive = SelfdriveD.__new__(SelfdriveD)
    drive.params = fixture.params
    drive.CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_BOSCH)
    drive.CP.openpilotLongitudinalControl = True
    drive.conditional_replay = True
    drive.conditional_settings, drive.conditional_consumer = fixture.owner, fixture.consumer
    drive.conditional_car_state_valid = drive.initialized = drive.enabled = True
    drive.aol_replay = False
    records = []
    for delta in (50_000_000, 250_000_000, 1_050_000_000):
      now = MONO + delta
      event, _, sm = fixture.produce(now)
      event.slcState.conditionalMode.hasOverride = False
      device = messaging.new_message('deviceState', valid=True)
      device.deviceState.started = True
      device.deviceState.startedMonoTime = fixture.drive_id
      sm.payloads['deviceState'] = messaging.log_from_bytes(device.to_bytes()).deviceState
      sm.payloads['slcState'] = messaging.log_from_bytes(event.to_bytes()).slcState
      for service in ('deviceState', 'slcState'):
        sm.logMonoTime[service] = now
        sm.recv_time[service] = now / 1e9
        sm.seen[service] = sm.alive[service] = sm.valid[service] = True
      drive.sm, drive.aol_car_state_log_ns = sm, now - 2_000_000
      with patch('openpilot.selfdrive.selfdrived.selfdrived.paired_clocks_ns',
                 return_value=(now, BOOT + delta, 1000)), \
           patch('openpilot.selfdrive.selfdrived.selfdrived.cloudlog.event',
                 side_effect=lambda *args, **kwargs: records.append((args, kwargs))):
        drive.update_conditional_mode(sm['carState'])
      self.assertFalse(drive.conditional_result.accepted)
      self.assertFalse(drive.experimental_mode)
    self.assertEqual(len(records), 2)
    for args, fields in records:
      self.assertEqual(args, ('conditional.unavailable',))
      self.assertEqual(fields['rejection'], 'proposal_mismatch')
      self.assertTrue(fields['authority'])
      self.assertGreater(fields['proposal_model_ns'], 0)


if __name__ == '__main__':
  unittest.main()
