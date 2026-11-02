"""Pure protocol regressions; no device or inferred command acknowledgements."""
import pytest
from opendbc.car.tesla.speed_sync import SpeedSynchronizer, scroll_request

NEUTRAL = bytes.fromhex('2955000000000040')


class Rig:
  def __init__(self):
    self.sync = SpeedSynchronizer(experimental_test=True)
    self.now = 1_000_000_000
    self.counter = 0
    self.speed, self.target, self.units = 45, 50, 1
    self.sent = []

  def tick(self, dt=10_000_000, *, long=True, gas=False, safe=True, enabled=True,
           feedback=True, wheel=NEUTRAL, wheel_bus=1):
    self.now += dt
    frames = []
    if feedback:
      self.counter = (self.counter + 1) % 16
      bits = (self.counter << 8) | (2 << 12) | (round(self.speed * 2) << 15) | (self.units << 24)
      di = bytearray(bits.to_bytes(8, 'little'))
      di[0] = (0x88 + sum(di[1:])) & 255
      frames.append((0x286, bytes(di), 0))
    if wheel is not None:
      frames.append((0x3c2, wheel, wheel_bus))
    self.sync.observe([(self.now, frames)])
    msg = self.sync.update(self.now, target_kph=self.target, enabled=enabled,
                           safe=safe, longitudinal_active=long, gas_pressed=gas)
    if msg:
      self.sent.append((self.now, msg))
    return msg


@pytest.mark.parametrize('before', [False, True])
def test_pending_response_on_either_side_of_release(before):
  r = Rig()
  assert r.tick(long=False, gas=True)
  if before:
    r.speed += 1
  assert r.tick(long=False) is None
  assert r.sync.blocked is None
  r.speed = 46
  assert r.tick(long=True) is None
  assert r.sync.pending is None and r.sync.blocked is None
  for _ in range(15):
    msg = r.tick()
    if msg:
      break
  assert msg is not None and len(r.sent) == 2


def test_no_initial_authority_and_held_pedal_engage():
  r = Rig()
  for _ in range(20):
    assert r.tick(long=False) is None
  assert not r.sent and r.sync.handover_until == 0
  assert r.tick(long=False, gas=True) is not None


def test_no_new_command_during_handover_even_after_response():
  r = Rig()
  assert r.tick(long=False, gas=True)
  r.speed += 1
  for _ in range(8):
    assert r.tick(long=False) is None
  assert r.sync.pending is None and len(r.sent) == 1
  assert r.tick() is None  # Minimum spacing still enforced.


def test_grace_expires_once_not_renewed_by_false_samples():
  r = Rig()
  r.tick(long=False, gas=True)
  r.tick(long=False)
  deadline = r.sync.handover_until
  for _ in range(9):
    r.tick(long=False)
    assert r.sync.handover_until == deadline and r.sync.blocked is None
  r.tick(long=False)
  assert r.now == deadline and r.sync.blocked == 'authority_lost_during_request'
  r.speed += 1
  for _ in range(20):
    assert r.tick() is None


def test_hard_failure_and_disengagement_clear_edge_history():
  r = Rig()
  r.target = r.speed
  r.tick(gas=True)
  r.tick(safe=False)
  r.target = 50
  assert r.tick(long=False) is None and r.sync.handover_until == 0
  r.tick(gas=True, enabled=False)
  assert r.tick(long=False) is None and r.sync.handover_until == 0


def test_feedback_loss_still_faults_during_grace():
  r = Rig()
  r.tick(gas=True)
  # A malformed DI message invalidates fresh native evidence immediately.
  r.sync.native_ns = 0
  assert r.tick(long=False, feedback=False) is None
  assert r.sync.blocked == 'feedback_lost'


def test_units_change_inflight_is_not_handover():
  r = Rig()
  r.tick(gas=True)
  r.units = 0
  assert r.tick(long=False) is None
  assert r.sync.blocked == 'units_changed_during_request'


def test_manual_input_always_wins_during_handover():
  r = Rig()
  r.tick(gas=True)
  assert r.tick(long=False, wheel=scroll_request(NEUTRAL, -1)[1]) is None
  assert r.sync.blocked == 'input_during_request'


def test_no_response_latches_and_later_change_does_not_retry():
  r = Rig()
  r.tick()
  for _ in range(20):
    assert r.tick(dt=100_000_000) is None
  assert r.sync.blocked == 'no_native_response'
  r.speed += 1
  r.target += 10
  for _ in range(20):
    assert r.tick() is None
  assert len(r.sent) == 1


def test_post_response_neutral_must_be_physical_and_strictly_newer():
  r = Rig()
  r.tick()
  r.tick(dt=100_000_000)  # New neutral BEFORE response is insufficient.
  r.speed += 1
  assert r.tick(dt=60_000_000, wheel=None) is None
  assert r.tick(wheel=NEUTRAL, wheel_bus=129) is None
  assert r.tick() is not None


def test_same_packet_neutral_does_not_count_as_post_response():
  r = Rig()
  r.tick()
  r.speed += 1
  assert r.tick(dt=150_000_000) is None
  assert r.tick() is not None


def test_template_age_is_tighter_than_receive_freshness():
  r = Rig()
  r.target = r.speed
  r.tick()
  r.target = 50
  assert r.tick(dt=50_000_001, wheel=None) is None
  assert r.sync.status == 'waiting_for_neutral_wheel'
  assert r.tick() is not None


@pytest.mark.parametrize('direction', [-1, 1])
def test_convergence_respects_spacing_and_unchanged_target(direction):
  r = Rig()
  r.target = r.speed + 10 * direction
  for _ in range(400):
    msg = r.tick()
    if msg:
      r.speed += direction
    if r.sync.status == 'matched':
      break
  assert r.speed == r.target and len(r.sent) == 10
  assert all(b[0] - a[0] >= 150_000_000 for a, b in zip(r.sent, r.sent[1:]))
  assert len(r.sync.history) <= 32


def test_same_rounded_target_never_sends():
  r = Rig()
  r.speed = 45.5
  r.target = 46
  for _ in range(10):
    assert r.tick() is None
  assert r.sync.status == 'matched'


def test_gap_fault_retains_existing_time_safety():
  r = Rig()
  r.tick(gas=True)
  assert r.tick(dt=250_000_001, long=False) is None
  assert r.sync.blocked == 'controller_clock_gap'
