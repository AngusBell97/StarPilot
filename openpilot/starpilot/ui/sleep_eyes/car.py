"""Turns live car messages into Sleep Eyes reactions (signals, lane changes, braking, bends), and decides which
alerts bring back the full driving screen."""
from __future__ import annotations

from openpilot.starpilot.ui.sleep_eyes.engine import Engine

HARD_BRAKE = -3.0        # m/s^2
HARD_ACCEL = 2.5         # m/s^2
ACCEL_HOLD = 250         # ms the acceleration must last
ACCEL_COOLDOWN = 4000    # ms between brake/accel reactions
LAT_ON = 3.2             # m/s^2 of lateral acceleration that pins the eyes to the edge
LAT_OFF = 2.4
LAT_HOLD = 400           # ms the bend must last

# Orange alerts that are information rather than a request: the eyes stay.
IGNORED_TEXT = ('regen', 'pedal cruise', 'branch is not tested', 'maneuver')


def _status(ss) -> str:
  return str(getattr(ss, 'alertStatus', 'normal'))


def wakes_screen(ss) -> bool:
  """For safety, every red and every orange alert brings back the full driving screen (the eyes never stand in
  for an alert). Only orange information (regen, pedal cruise, untested-branch banner) leaves the eyes up."""
  status = _status(ss)
  if status == 'critical':
    return True
  if status != 'userPrompt' or str(getattr(ss, 'alertSize', 'none')) == 'none':
    return False
  text = f"{getattr(ss, 'alertText1', '')} {getattr(ss, 'alertText2', '')}".lower()
  return not any(word in text for word in IGNORED_TEXT)


class CarWatcher:
  """Sleep and wake follow the Park gear: the comma loses power the moment the car switches off, so going
  offroad is never seen. Shifting into Park dozes the eyes off; shifting out of Park wakes them up."""

  SLEEP_WAKE = ('boot', 'ignOn', 'ignOff', 'morphIn')

  def __init__(self, grace_until: float = 0.0):
    self.started: bool | None = None
    self.first = True
    self.grace_until = grace_until  # ms: until then, offroad counts as "not known yet" (the comma just booted)
    self.engaged: bool | None = None
    self.lane_change = 'off'
    self.accel_since: float | None = None
    self.accel_sign = 0
    self.accel_cooldown = 0.0
    self.lat_since: float | None = None
    self.lat_active = False

  def update(self, engine: Engine, now: float, sm, started: bool) -> None:
    cs = sm['carState'] if started else None
    asleep = not started or str(cs.gearShifter) == 'park'
    # Doze off or wake up whenever the wanted state differs, unless a sleep/wake animation is already playing
    # (the boot animation then settles: into Park it goes to sleep). The very first look sets it silently.
    settled = started or now >= self.grace_until
    if settled and engine.current not in self.SLEEP_WAKE and engine.parked != asleep:
      if self.first and engine.current is None:
        engine.parked = asleep
      else:
        engine.play('ignOff' if asleep else 'ignOn', now, force=True)
    self.first = False
    if started != self.started:
      self.started = started
      if not started:
        engine.ctx.update(blinker=0, alert=None, lat_hold=False)
        self.engaged = None
    if not started:
      return

    ss = sm['selfdriveState']
    engaged = bool(ss.enabled)
    if self.engaged is None:
      engine.engaged = engaged
    elif engaged != self.engaged:
      engine.play('eng' if engaged else 'dis', now)
    self.engaged = engaged
    engine.stopped = engaged and bool(cs.standstill)

    # Lane changes: the automatic manoeuvre gets its own reaction; the blinker loop pauses meanwhile.
    meta = sm['modelV2'].meta
    lane_change = str(meta.laneChangeState)
    if lane_change == 'laneChangeStarting' and self.lane_change != 'laneChangeStarting':
      engine.play('alcL' if str(meta.laneChangeDirection) == 'left' else 'alcR', now)
    self.lane_change = lane_change
    changing = lane_change in ('laneChangeStarting', 'laneChangeFinishing')
    engine.ctx['blinker'] = 0 if changing else -1 if cs.leftBlinker else 1 if cs.rightBlinker else 0

    # Hard braking or acceleration, held briefly, with a cooldown.
    a_ego = float(cs.aEgo)
    sign = -1 if a_ego < HARD_BRAKE else 1 if a_ego > HARD_ACCEL else 0
    if sign and sign == self.accel_sign:
      if self.accel_since is not None and now - self.accel_since > ACCEL_HOLD and now > self.accel_cooldown:
        engine.play('brake' if sign < 0 else 'accel', now)
        self.accel_cooldown = now + ACCEL_COOLDOWN
    else:
      self.accel_since = now if sign else None
    self.accel_sign = sign

    # Bends: lateral acceleration from the actual path curvature.
    lat = float(sm['controlsState'].curvature) * float(cs.vEgo) ** 2
    if not self.lat_active:
      if abs(lat) > LAT_ON:
        self.lat_since = now if self.lat_since is None else self.lat_since
        if now - self.lat_since > LAT_HOLD:
          self.lat_active = engine.play('latL' if lat > 0 else 'latR', now)
      else:
        self.lat_since = None
    elif abs(lat) < LAT_OFF:
      self.lat_active = False
      self.lat_since = None
    engine.ctx['lat_hold'] = self.lat_active
    # Alerts are never shown as eyes: red and orange ones wake the full screen instead (see wakes_screen). The
    # engine's amber alert animations stay in engine.py but nothing triggers them.
