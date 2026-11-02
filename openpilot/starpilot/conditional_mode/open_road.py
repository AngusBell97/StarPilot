"""SP-021 Tesla Open Road state: mode selection only, never actuator output.

Restores the installed pre-7.0 entry/recovery/lead confirmation rules. The
following gap uses current 7.0 MPC semantics (6 m stop gap, 2.5 m/s2 comfort
braking, same-cycle selected headway), not the old acceleration controller.
"""
from __future__ import annotations

import math
from numbers import Real
from typing import TYPE_CHECKING

if TYPE_CHECKING:
  from openpilot.starpilot.conditional_mode.policy import SceneEvidence

MPH = 0.44704
MAX_AGE = 0.2
TESLAS = frozenset(('TESLA_MODEL_3', 'TESLA_MODEL_Y'))


def finite(value, low=0., high=1000.):
  return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value) and low <= value <= high


def following_gap(speed, lead_speed, headway):
  return max(6., speed * headway, 6. + speed * headway + (speed ** 2 - lead_speed ** 2) / 5.)


class OpenRoad:
  def __init__(self):
    self.reset()

  def reset(self):
    self.active = self.approach = self.released = False
    self.entry_since = self.below_since = self.match_since = self.absent_since = None
    self.last_tick = self.last_lead = self.set_speed = None

  def _clear_timers(self):
    self.entry_since = self.below_since = self.match_since = self.absent_since = None
    self.last_lead = None

  def step(self, now: float, scene: SceneEvidence) -> bool:
    self.released = False
    speed, cruise, stamp = scene.speed_mps, scene.set_speed_mps, scene.open_road_observed_s
    if not finite(speed, high=80.) or not finite(cruise, .001, 80.):
      self.reset()
      return False
    if self.set_speed is not None and abs(cruise - self.set_speed) > .001:
      self.reset()
    self.set_speed = cruise
    # Unknown/repeated observations never contribute dwell or release an
    # established approach. The containing policy owns authority/stale resets.
    if not finite(stamp, high=1e12) or not 0 <= now - stamp <= MAX_AGE:
      self._clear_timers()
      self.last_tick = None
      return self.active
    if self.last_tick is not None:
      gap = stamp - self.last_tick
      if gap <= 0:
        self._clear_timers()
        return self.active
      if gap > MAX_AGE + 1e-6:
        self._clear_timers()
    self.last_tick = stamp
    lead = scene.lead
    if lead is None or any(type(v) is not bool for v in (lead.present, lead.tracked, lead.radar, scene.following_lead)):
      self._clear_timers()
      return self.active
    tracked = lead.tracked or scene.following_lead
    if not self.active:
      model_stamp = scene.open_road_model_s
      ready = (finite(model_stamp, high=1e12) and 0 <= now - model_stamp <= MAX_AGE and
               finite(scene.open_road_model_accel, high=10.) and scene.open_road_model_stop is False)
      if not tracked and cruise <= speed <= cruise + MPH and ready:
        if self.entry_since is None:
          self.entry_since = stamp
        self.active = stamp - self.entry_since >= 2. - 1e-6
      else:
        self.entry_since = None
      self.approach = False
      self.below_since = self.match_since = self.absent_since = self.last_lead = None
      return self.active

    if lead.present or tracked:
      self.approach = True
      self.below_since = self.absent_since = None
      headway, lateral = scene.open_road_headway_s, scene.open_road_lateral_m
      stable = (lead.present and lead.tracked and
                all(finite(v, high=500.) for v in (lead.distance_m, lead.speed_mps)) and
                finite(lead.accel_mps2, -.2, 20.) and finite(lateral, -1.8, 1.8) and
                finite(headway, .5, 3.) and
                (lead.radar or finite(lead.model_probability, .9, 1.)) and
                speed - lead.speed_mps <= 5. * MPH + 1e-6 and
                lead.distance_m >= following_gap(speed, lead.speed_mps, headway))
      if stable:
        sample = (stamp, lead.distance_m, lead.speed_mps)
        if self.last_lead is not None:
          last_t, last_d, last_v = self.last_lead
          predicted = last_d + (last_v - speed) * (stamp - last_t)
          if abs(lead.distance_m - predicted) > 5. or abs(lead.speed_mps - last_v) > 2.5:
            self.match_since = None
        self.last_lead = sample
        if self.match_since is None:
          self.match_since = stamp
        if stamp - self.match_since >= .5 - 1e-6:
          self.active, self.released = False, True
          self.entry_since = None
      else:
        self.match_since = self.last_lead = None
      return self.active

    self.match_since = self.last_lead = None
    if self.approach:
      if self.absent_since is None:
        self.absent_since = stamp
      if stamp - self.absent_since < .5 - 1e-6:
        return True
      self.approach = False
      self.below_since = None
    if cruise - speed > MPH + 1e-6:
      if self.below_since is None:
        self.below_since = stamp
      if stamp - self.below_since > 2. + 1e-6:
        self.active, self.released = False, True
        self.entry_since = None
    else:
      self.below_since = None
    return self.active
