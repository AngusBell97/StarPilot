"""Tesla stationary release evidence; never a source of acceleration or stop acquisition."""
import math
from openpilot.starpilot.longitudinal.lead_takeoff import Lead, confident
from openpilot.starpilot.model_geometry import tesla_stationary_zero

TESLAS = frozenset(('TESLA_MODEL_3', 'TESLA_MODEL_Y'))
# DI_vehicleSpeed: 0.08 km/h per count. Stay below two counts (0.04444 m/s).
STATIONARY_COUNT_MAX = .025


def number(value, low, high):
  return (float(value) if type(value) in (int, float) and math.isfinite(value) and low <= value <= high else None)


def forward_tail(model):
  try:
    values = tuple(model.velocity.x)
    if len(values) != 33 or any(number(v, -.05, 100.) is None for v in values):
      return False
    tail = values[15:]  # ModelConstants.T_IDXS[15] == 2.197265625 seconds.
    # A creeping departure need not accelerate monotonically to 5 m/s.
    # Keep finite/non-reversing model evidence and a positive future speed.
    return max(tail) >= .3 and tail[-1] >= .1
  except (AttributeError, TypeError, ValueError):
    return False


class TeslaLaunchRelease:
  def __init__(self):
    self.reset()

  def reset(self):
    self.last_ns = 0
    self.previous = None
    self.minimum_gaps = None
    self.absent_since = None
    self.stationary_presence = None
    self.lead_from_stop = False
    self.continuous_since = None
    self.lateral_origin = None

  def pause_motion(self):
    presence, eligible = self.stationary_presence, self.lead_from_stop
    self.reset()
    self.stationary_presence, self.lead_from_stop = presence, eligible

  def invalidate(self):
    # Invalid evidence removes all accumulated authority. Fresh observations
    # must qualify again; one missing sample must not latch a whole-stop veto.
    self.reset()

  def sample(self, *, car, cp, model, radar, stamp, now, horizon, native_clear, tracked):
    try:
      if car.standstill is not True:
        self.reset()
        return False
      presence = (radar.leadOne.present, radar.leadTwo.present)
      if self.stationary_presence is None:
        self.stationary_presence = presence
        self.lead_from_stop = any(v is True for v in presence)
      elif presence != self.stationary_presence:
        self.reset()
        self.stationary_presence = presence
        self.lead_from_stop = any(v is True for v in presence)
      speed = 0. if tesla_stationary_zero(car, cp) else number(car.vEgo, 0., .01)
      if speed is None or (car.vEgoRaw != 0. and not tesla_stationary_zero(car, cp)):
        self.pause_motion()
        return False
      if (cp.carFingerprint not in TESLAS or car.standstill is not True or str(car.gearShifter) != 'drive' or
          not car.canValid or car.canTimeout or car.gasPressed or car.brakePressed or speed is None or
          (car.vEgoRaw != 0. and not tesla_stationary_zero(car, cp)) or
          type(stamp) is not int or not 0 < stamp <= now or now-stamp > 150_000_000 or
          stamp <= self.last_ns or self.last_ns and stamp-self.last_ns > 100_000_000):
        self.invalidate()
        return False
      dt = (stamp-self.last_ns)/1e9 if self.last_ns else 0.
      self.last_ns = stamp
      leads = []
      for lead in (radar.leadOne, radar.leadTwo):
        if type(lead.present) is not bool:
          self.invalidate()
          return False
        if lead.present:
          values = tuple(number(getattr(lead, k), low, high) for k, low, high in
            (('dRel', 0., 100.), ('vLead', -30., 100.), ('vRel', -100., 100.),
             ('aLeadK', -20., 20.), ('modelProb', 0., 1.), ('yRel', -20., 20.)))
          if any(v is None for v in values):
            self.invalidate()
            return False
          leads.append(values)
      current = tuple(leads)
      consistent = self.previous is not None and len(current) == len(self.previous)
      if current:
        self.absent_since = None
        if consistent:
          for row, old in zip(current, self.previous):
            delta = row[0]-old[0]
            if (abs(delta-old[2]*dt) > .3+(abs(old[2])+1.)*dt or abs(row[5]-old[5]) > .15 or abs(row[1]-old[1]) > 1.+5.*dt):
              consistent = False
        if not consistent:
          self.minimum_gaps = tuple(row[0] for row in current)
          self.continuous_since = stamp
          self.lateral_origin = tuple(row[5] for row in current)
        else:
          self.minimum_gaps = tuple(min(a, row[0]) for a, row in zip(self.minimum_gaps, current))
      else:
        if self.previous is None or self.previous:
          self.absent_since = stamp
        self.minimum_gaps = None
      self.previous = current
      # The native planner is a same-cycle consistency check, not an independent sensor.
      if (not native_clear or horizon is None or model.action.shouldStop is not False or not forward_tail(model)):
        self.continuous_since = stamp
        self.absent_since = stamp if not current else None
        return False
      accel = number(model.action.desiredAcceleration, -10., 10.)
      if not current:
        if accel is None or accel < .15:
          self.absent_since = stamp
          return False
        return bool(self.absent_since is not None and stamp-self.absent_since >= 500_000_000)
      # Dom-derived stationary departure classes. Native positive/non-stop
      # authority is still required, and this helper never raises acceleration.
      def departing(row):
        distance, velocity, relative, acceleration, probability, lateral = row
        vehicle = Lead(0, 0, False, probability, distance, velocity, acceleration, lateral)
        if not vehicle.credible(.9) or distance > 30. or relative < .2:
          return False
        creep = vehicle.credible(.95) and distance >= 5.6 and velocity >= .25 and acceleration >= .08
        regular = distance >= 6.3 and velocity >= .6 and acceleration >= -.2 and accel >= .08
        return bool(creep or regular or confident(vehicle, speed))
      qualified = accel is not None and accel >= 0. and all(departing(row) for row in current)
      if not qualified:
        self.continuous_since = stamp
        return False
      return bool(consistent and self.continuous_since is not None and
        stamp-self.continuous_since >= 300_000_000 and
        all(abs(row[5]-origin) <= .5 for row, origin in zip(current, self.lateral_origin)) and
        accel is not None and accel >= 0. and all(departing(row) for row in current))
    except (AttributeError, TypeError, ValueError, OverflowError):
      self.invalidate()
      return False
