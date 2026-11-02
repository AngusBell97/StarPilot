"""SP-016 v9: Dom geometry with custom acceptance and asymmetric response."""
from dataclasses import dataclass
import math
from openpilot.starpilot.lateral.stock_dom.lane_centering import (
  LaneCenteringController, LaneCenteringSettings, _finite_number)


@dataclass(frozen=True)
class V9Settings(LaneCenteringSettings):
  buildup_time: float = .4
  unwind_time: float = .2
  release_time: float = .2


def approach(current, target, dt, tau):
  return target if tau == 0 else current + (target-current) * -math.expm1(-dt/tau)


def filter_correction(current, target, dt, buildup, unwind):
  """Use unwind until zero on a reversal, then buildup for the remaining time.

  Both phases approach the requested target exponentially. The exact crossing
  time avoids a zero hold or tick-size-dependent jump between filter phases.
  """
  if current * target < 0:
    crossing = unwind * math.log1p(abs(current / target)) if unwind else 0.
    if dt <= crossing:
      return approach(current, target, dt, unwind)
    return approach(0., target, dt-crossing, buildup)
  tau = buildup if abs(target) > abs(current) else unwind
  return approach(current, target, dt, tau)


class V9LaneController(LaneCenteringController):
  min_speed = 0.
  use_deadband = False

  @staticmethod
  def accept_line(prob, std):
    return .1 <= prob <= 1. and 0. <= std < (1. if prob >= .2 else .6)

  def reset(self):
    super().reset()
    self.filter_diagnostic = {}

  def _valid_settings(self, settings):
    return isinstance(settings, V9Settings) and all(
      (value := _finite_number(getattr(settings, key))) is not None and 0 <= value <= 1
      for key in ('buildup_time', 'unwind_time', 'release_time'))

  def _filter_target(self, target, dt):
    settings = self._clearance_observation.settings
    previous = self._correction
    phase = ('reversal' if previous * target < 0 else
             'buildup' if abs(target) > abs(previous) else 'unwind')
    self._correction = filter_correction(previous, target, dt, settings.buildup_time, settings.unwind_time)
    self.filter_diagnostic = dict(phase=phase, target=target, before=previous, after=self._correction)

  def _fade(self, base, dt, reason):
    previous = self._correction
    self._correction = approach(previous, 0., dt, self._clearance_observation.settings.release_time)
    if abs(self._correction) < 1e-12:
      self._correction = 0.
    self.filter_diagnostic = dict(phase='return_to_model', target=0., before=previous, after=self._correction)
    return self._result(base, reason)
