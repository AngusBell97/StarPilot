"""Stock Dom comparison boundary; custom tuning never enters the stock controller."""
from dataclasses import fields
from openpilot.starpilot.lateral.stock_dom import lane_centering as stock
from openpilot.starpilot.lateral.stock_dom.lane_runtime import LaneCenteringHost, read_settings
from openpilot.starpilot.lateral.lane_centering import ControlMode, LaneCenteringRequest, LaneCenteringResult

STOCK_VERSION = 7
STOCK_LABEL = "Stock Dom"
STOCK_BASE = "e8256ee5b2418a7b74218bbe1cfb3d6b961e542e"
STOCK_MAP = {"LaneCentering": "LaneCentering", "LaneCenteringPauseOnSignal": "LaneCenteringDomPauseOnSignal",
             "LaneCenterOffset": "LaneCenteringDomOffset", "LaneCenteringE2EAuthority": "LaneCenteringDomE2EAuthority",
             "LaneCenteringStrength": "LaneCenteringDomStrength"}
STOCK_DEFAULTS = {"LaneCenteringDomPauseOnSignal": True, "LaneCenteringDomOffset": 0.,
                  "LaneCenteringDomE2EAuthority": .5, "LaneCenteringDomStrength": 1.5}

class StockParams:
  def __init__(self, params): self.params = params
  def get_param_path(self, key): return self.params.get_param_path(STOCK_MAP.get(key, key))

def settings(params):
  return read_settings(StockParams(params))

class StockHost:
  def __init__(self, params): self.host = LaneCenteringHost(StockParams(params))
  def sample(self, sm, **kwargs):
    request = self.host.sample(sm, **kwargs)
    if request is None: return None
    return LaneCenteringRequest(ControlMode(request.mode.value), request.settings,
                               request.elapsed_seconds, request.time_discontinuity, STOCK_VERSION)

class StockController:
  def __init__(self):
    # Stock controlsd's normal production path does not opt into body prediction.
    self.controller = stock.LaneCenteringController()
    self.diagnostic = {}
  @property
  def _correction(self): return self.controller._correction
  @_correction.setter
  def _correction(self, value): self.controller._correction = value
  def reset(self):
    self.controller.reset()
    self.diagnostic = {}
  def _result(self, base, reason):
    result = self.controller._result(base, reason)
    return LaneCenteringResult(result.candidate_curvature, result.correction, result.direction, result.reason, stock.MAX_CORRECTION)
  def update(self, observation, *, measured_curvature=None, lateral_delay_s=None):
    values = {f.name: getattr(observation, f.name) for f in fields(observation)}
    values.update(mode=stock.ControlMode(observation.mode.value) if isinstance(observation.mode, ControlMode) else observation.mode, measured_curvature=measured_curvature,
                  lateral_delay_s=lateral_delay_s)
    result = self.controller.update(stock.LaneCenteringInput(**values))
    self.diagnostic = {"algorithm": STOCK_VERSION, "frame": observation.model_sample_id, "base": STOCK_BASE}
    return LaneCenteringResult(result.candidate_curvature, result.correction, result.direction, result.reason, stock.MAX_CORRECTION)
  def guard_applied(self, *args): return self.controller.guard_applied(*args)
