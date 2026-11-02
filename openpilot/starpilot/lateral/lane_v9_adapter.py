"""Independent v9 settings and Dom host/vehicle-limit integration."""
from dataclasses import fields
import math
from pathlib import Path
from openpilot.common.params import UnknownKeyName
from openpilot.starpilot.lateral.lane_v9 import V9Settings, V9LaneController
from openpilot.starpilot.lateral.stock_dom_adapter import StockController
from openpilot.starpilot.lateral.stock_dom.lane_runtime import LaneCenteringHost, read_settings
from openpilot.starpilot.lateral.lane_centering import ControlMode, LaneCenteringRequest

V9_VERSION = 90  # Internal 9 already belongs to Chubbs; never reinterpret saved choices.
V9_MAP = {'LaneCentering': 'LaneCentering', 'LaneCenteringPauseOnSignal': 'LaneCenteringV9PauseOnSignal',
          'LaneCenterOffset': 'LaneCenteringV9Offset', 'LaneCenteringE2EAuthority': 'LaneCenteringV9E2EAuthority',
          'LaneCenteringStrength': 'LaneCenteringV9Strength'}
TIMERS = {'LaneCenteringV9BuildupTime': 'buildup_time', 'LaneCenteringV9UnwindTime': 'unwind_time',
          'LaneCenteringV9ReleaseTime': 'release_time'}
V9_DEFAULTS = {'LaneCenteringV9PauseOnSignal': True, 'LaneCenteringV9Offset': 0.,
               'LaneCenteringV9E2EAuthority': .5, 'LaneCenteringV9Strength': 1.5,
               'LaneCenteringV9BuildupTime': .4, 'LaneCenteringV9UnwindTime': .2, 'LaneCenteringV9ReleaseTime': .2}


class V9Params:
  def __init__(self, params): self.params = params
  def get_param_path(self, key): return self.params.get_param_path(V9_MAP.get(key, key))


def settings(params):
  base = read_settings(V9Params(params))
  timers = {}
  try:
    for key, field in TIMERS.items():
      try: value = Path(params.get_param_path(key)).read_bytes()
      except FileNotFoundError: value = str(V9_DEFAULTS[key]).encode()
      value = float(value)
      if not math.isfinite(value) or not 0 <= value <= 1: raise ValueError('Invalid v9 timing')
      timers[field] = value
    return V9Settings(**{f.name: getattr(base, f.name) for f in fields(base)}, **timers)
  except (OSError, ValueError, TypeError, UnknownKeyName):
    return V9Settings()


class V9Host:
  def __init__(self, params):
    self.params = params
    self.host = LaneCenteringHost(V9Params(params))
    self.last_refresh = None
    self.settings = V9Settings()

  def sample(self, sm, **kwargs):
    request = self.host.sample(sm, **kwargs)
    if self.last_refresh != self.host.last_refresh_ns:
      self.settings = settings(self.params)
      self.last_refresh = self.host.last_refresh_ns
    if request is None or not self.settings.enabled: return None
    return LaneCenteringRequest(ControlMode(request.mode.value), self.settings,
                                request.elapsed_seconds, request.time_discontinuity, V9_VERSION)


class V9Controller(StockController):
  def __init__(self):
    self.controller = V9LaneController()
    self.diagnostic = {}

  def update(self, observation, **kwargs):
    result = super().update(observation, **kwargs)
    self.diagnostic.update(algorithm=V9_VERSION, **getattr(self.controller, 'filter_diagnostic', {}))
    return result
