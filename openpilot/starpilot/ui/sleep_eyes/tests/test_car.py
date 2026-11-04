"""Sleep Eyes car watcher: Park puts the eyes to sleep, leaving Park wakes them, and the other triggers fire."""
import random
from types import SimpleNamespace as NS

from openpilot.starpilot.ui.sleep_eyes.car import CarWatcher
from openpilot.starpilot.ui.sleep_eyes.engine import Engine


def sm(gear='drive', enabled=False, standstill=False, blinker=0, a_ego=0.0, v_ego=0.0, curvature=0.0, lane_change='off'):
  return {
    'selfdriveState': NS(enabled=enabled, alertStatus='normal', alertSize='none', alertType='', alertText1='', alertText2=''),
    'carState': NS(gearShifter=gear, standstill=standstill, leftBlinker=blinker < 0, rightBlinker=blinker > 0,
                   aEgo=a_ego, vEgo=v_ego, leftBlindspot=False),
    'modelV2': NS(meta=NS(laneChangeState=lane_change, laneChangeDirection='left'), laneLines=[]),
    'controlsState': NS(curvature=curvature),
  }


def finish(engine, start, ms=6000):
  now = start
  while now < start + ms:
    now += 50
    engine.step(now)
  return now


def test_a_drive_that_starts_in_park_is_asleep_until_shifted_out():
  engine, watcher = Engine(random.Random(1)), CarWatcher()
  watcher.update(engine, 0, sm('park'), True)
  assert engine.parked and engine.current is None
  watcher.update(engine, 100, sm('drive'), True)
  assert engine.current == 'ignOn' and not engine.parked


def test_shifting_into_park_dozes_off_and_back_out_wakes():
  engine, watcher = Engine(random.Random(2)), CarWatcher()
  watcher.update(engine, 0, sm('drive'), True)
  assert not engine.parked
  watcher.update(engine, 100, sm('park'), True)
  assert engine.current == 'ignOff'
  now = finish(engine, 100)
  assert engine.parked
  watcher.update(engine, now, sm('reverse'), True)
  assert engine.current == 'ignOn' and not engine.parked


def test_driving_triggers():
  engine, watcher = Engine(random.Random(3)), CarWatcher()
  watcher.update(engine, 0, sm('drive'), True)
  watcher.update(engine, 50, sm('drive', enabled=True), True)
  assert engine.current == 'eng'
  watcher.update(engine, 100, sm('drive', enabled=True, blinker=1), True)
  assert engine.ctx['blinker'] == 1
  watcher.update(engine, 150, sm('drive', enabled=True, lane_change='laneChangeStarting'), True)
  assert engine.current == 'alcL' and engine.ctx['blinker'] == 0
  engine.stop()
  for t in (200, 300, 500, 700):
    watcher.update(engine, t, sm('drive', enabled=True, a_ego=-4.0), True)
  assert engine.current == 'brake'
  engine.stop()
  for t in range(5000, 5700, 100):  # 30 m/s in a bend at 0.005 1/m = 4.5 m/s^2 to the left
    watcher.update(engine, t, sm('drive', enabled=True, v_ego=30, curvature=0.005), True)
  assert engine.current == 'latL' and engine.ctx['lat_hold']
  watcher.update(engine, 6000, sm('drive', enabled=True, v_ego=30), True)
  assert not engine.ctx['lat_hold']
  watcher.update(engine, 6100, sm('drive', enabled=False), True)
  watcher.update(engine, 6150, sm('drive', enabled=True, standstill=True), True)
  assert engine.stopped


def test_boot_wakes_then_dozes_off_in_park_or_stays_awake_in_drive():
  for gear, asleep in (('park', True), ('drive', False)):
    engine, watcher = Engine(random.Random(4)), CarWatcher()
    engine.auto = False  # no random events in the way
    engine.play('boot', 0, force=True)
    now = 0
    while engine.current == 'boot':  # the boot animation runs to the end whatever the gear says
      now += 50
      watcher.update(engine, now, sm(gear), True)
      engine.step(now)
    assert now > 5900
    watcher.update(engine, now + 50, sm(gear), True)
    if asleep:
      assert engine.current == 'ignOff'
      finish(engine, now + 50)
      assert engine.parked
    else:
      assert engine.current is None and not engine.parked
