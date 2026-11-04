"""Turns live car messages into Sleep Eyes reactions (signals, lane changes, braking, bends, orange alerts)."""
from __future__ import annotations

from openpilot.starpilot.ui.sleep_eyes.engine import Engine

HARD_BRAKE = -3.0        # m/s^2
HARD_ACCEL = 2.5         # m/s^2
ACCEL_HOLD = 250         # ms the acceleration must last
ACCEL_COOLDOWN = 4000    # ms between brake/accel reactions
LAT_ON = 3.2             # m/s^2 of lateral acceleration that pins the eyes to the edge
LAT_OFF = 2.4
LAT_HOLD = 400           # ms the bend must last

ALERT_BY_EVENT = {
  'driverDistracted': 'alAttn',
  'driverUnresponsive': 'alWheel',
  'steerSaturated': 'alSteer',
  'ldw': 'alLdw',
  'laneChangeBlocked': 'alBlind',
  'resumeRequired': 'alResume',
  'brakeHold': 'alResume',
}
ALERT_BY_TEXT = (
  ('blindspot', 'alBlind'),
  ('press resume', 'alResume'),
  ('pay attention', 'alAttn'),
  ('touch steering', 'alWheel'),
  ('exceeds', 'alSteer'),
  ('lane departure', 'alLdw'),
)
# Orange alerts that are information rather than a request: no reaction.
IGNORED_TEXT = ('regen', 'pedal cruise', 'branch is not tested', 'maneuver')


def _status(ss) -> str:
  return str(getattr(ss, 'alertStatus', 'normal'))


def wakes_screen(ss) -> bool:
  """Red alerts, and the full-screen orange TAKE CONTROL IMMEDIATELY, always wake the full screen."""
  status = _status(ss)
  return status == 'critical' or (status == 'userPrompt' and str(getattr(ss, 'alertSize', 'none')) == 'full')


def orange_alert(ss) -> str | None:
  if _status(ss) != 'userPrompt' or str(getattr(ss, 'alertSize', 'none')) in ('none', 'full'):
    return None
  text = f"{getattr(ss, 'alertText1', '')} {getattr(ss, 'alertText2', '')}".lower()
  if any(word in text for word in IGNORED_TEXT):
    return None
  event = str(getattr(ss, 'alertType', '')).split('/')[0]
  for prefix, name in ALERT_BY_EVENT.items():
    if event.startswith(prefix):
      return name
  for word, name in ALERT_BY_TEXT:
    if word in text:
      return name
  return 'alTake'


class CarWatcher:
  def __init__(self):
    self.started: bool | None = None
    self.engaged: bool | None = None
    self.lane_change = 'off'
    self.accel_since: float | None = None
    self.accel_sign = 0
    self.accel_cooldown = 0.0
    self.lat_since: float | None = None
    self.lat_active = False

  def update(self, engine: Engine, now: float, sm, started: bool) -> None:
    if self.started is None:
      self.started = started
      engine.parked = not started
    elif started != self.started:
      self.started = started
      engine.play('ignOn' if started else 'ignOff', now, force=True)
      if not started:
        engine.ctx.update(blinker=0, alert=None, lat_hold=False)
        self.engaged = None
        return
    if not started:
      return

    ss, cs = sm['selfdriveState'], sm['carState']
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

    # Orange alerts become amber eyes; which way they look depends on the alert.
    alert = orange_alert(ss)
    engine.ctx['alert'] = alert
    if alert == 'alSteer':
      engine.ctx['alert_dir'] = -1 if lat > 0 else 1
    elif alert == 'alBlind':
      engine.ctx['alert_dir'] = -1 if cs.leftBlinker or getattr(cs, 'leftBlindspot', False) else 1
    elif alert == 'alLdw':
      lines = sm['modelV2'].laneLines
      if len(lines) >= 3 and len(lines[1].y) and len(lines[2].y):
        engine.ctx['alert_dir'] = -1 if abs(lines[1].y[0]) < abs(lines[2].y[0]) else 1
