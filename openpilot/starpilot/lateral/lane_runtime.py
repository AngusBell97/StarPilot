"""Saved lane-centering preferences and fresh host authority, opt-in at startup."""

import math
import os
from pathlib import Path

from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.realtime import DT_CTRL
from openpilot.common.params import UnknownKeyName
from openpilot.starpilot.lateral.lane_centering import ControlMode, LaneCenteringRequest, LaneCenteringSettings, STRENGTH_DEFAULT, STRENGTH_MAX
from openpilot.starpilot.lateral.lane_tuning import NUMBERS, BOOLS, PREFIX, valid_tuning
from openpilot.starpilot.feature_capabilities import lateral_supported


def read_settings(params, *, defaults=None) -> LaneCenteringSettings:
  def raw(key, default):
    if defaults is not None and key in defaults:
      value = defaults[key]
      default = str(int(value) if isinstance(value, bool) else value).encode()
    try:
      return Path(params.get_param_path(key)).read_bytes()
    except FileNotFoundError:
      return default

  def boolean(key, default):
    value = raw(key, default)
    if value not in (b'0', b'1'):
      raise ValueError('Invalid lane preference')
    return value == b'1'

  try:
    enabled = boolean('LaneCentering', b'1')
    pause = boolean('LaneCenteringPauseOnSignal', b'1')
    offset = float(raw('LaneCenterOffset', b'0.0'))
    authority = float(raw('LaneCenteringE2EAuthority', b'1.0'))
    try:
      strength = float(raw('LaneCenteringStrength', str(STRENGTH_DEFAULT).encode()))
    except UnknownKeyName:
      strength = STRENGTH_DEFAULT
    if not math.isfinite(strength) or not 0.5 <= strength <= STRENGTH_MAX:
      raise ValueError('Invalid lane strength')
    if not math.isfinite(offset) or not -0.3 <= offset <= 0.3 or not math.isfinite(authority) or not 0 <= authority <= 1:
      raise ValueError('Invalid lane tuning')
    tuning = {field: float(raw(PREFIX + spec[0], str(spec[1]).encode())) for field, spec in NUMBERS.items()}
    tuning.update({field: boolean(key, b'1') for field, key in BOOLS.items()})
    settings = LaneCenteringSettings(enabled, offset, authority, pause, strength, **tuning)
    if not valid_tuning(settings):
      raise ValueError('Invalid lane v4 tuning')
    return settings
  except (OSError, ValueError, UnknownKeyName):
    return LaneCenteringSettings()


def runtime_supported(CP) -> bool:
  """Keep the preference reader available for every configured steering owner."""
  return os.getenv('LANE_CENTERING_REPLAY_RUNTIME') == '1' or lateral_supported(CP)


class LaneCenteringHost:
  REFRESH_NS = 1_000_000_000
  MAX_TICK_NS = 50_000_000
  REQUIRED = ('carState', 'selfdriveState', 'modelV2', 'vehicleParameters')

  def __init__(self, params):
    self.params = params
    from openpilot.starpilot.lateral.chubbs_selector import selected
    self.chubbs_session = selected(params)
    self.stock_host = None
    self.version = 4
    self.settings = LaneCenteringSettings()
    self.last_refresh_ns: int | None = None
    self.last_tick_ns: int | None = None

  def inputs_need_refresh(self, sm, *, now_ns: int) -> bool:
    # Only the active Stock Dom host adopts upstream's queued-input refresh.
    # Custom v4/v5/v6 retain their existing subscriber and freshness behavior.
    return (self.version in (7, 90) and self.stock_host is not None and
            self.stock_host.host.inputs_need_refresh(sm, now_ns=now_ns))

  def sample(self, sm, *, now_ns: int, lateral_active: bool, longitudinal_active: bool) -> LaneCenteringRequest | None:
    # Freeze exclusion for this process session, matching modeld. A parked
    # selection change takes effect next drive; never stack on a Chubbs action.
    if self.chubbs_session:
      return None
    previous = self.last_tick_ns
    self.last_tick_ns = now_ns
    discontinuity = previous is not None and not 0 < now_ns - previous <= self.MAX_TICK_NS
    if now_ns <= 0:
      self.settings = LaneCenteringSettings()
      self.last_refresh_ns = None
      self.stock_host = None
      return None
    if self.last_refresh_ns is None or now_ns < self.last_refresh_ns or now_ns - self.last_refresh_ns >= self.REFRESH_NS:
      from openpilot.starpilot.lateral.lane_versions import read_selected
      self.version, self.settings = read_selected(self.params)
      self.last_refresh_ns = now_ns
    if self.version in (7, 90):
      from openpilot.starpilot.lateral.stock_dom_adapter import StockHost
      from openpilot.starpilot.lateral.lane_v9_adapter import V9Host
      host_type = V9Host if self.version == 90 else StockHost
      if type(self.stock_host) is not host_type: self.stock_host = host_type(self.params)
      return self.stock_host.sample(sm, now_ns=now_ns, lateral_active=lateral_active, longitudinal_active=longitudinal_active)
    self.stock_host = None
    if not self.settings.enabled or not sm['carState'].canValid or sm['carState'].canTimeout:
      return None
    if not sm.all_checks(list(self.REQUIRED)) or any(
      int(sm.logMonoTime[name]) <= 0 or not 0 <= now_ns - int(sm.logMonoTime[name]) <= int(2.5e9 / SERVICE_LIST[name].frequency)
      for name in self.REQUIRED
    ):
      if self.version != 8:
        return None
      # v8 handles stale inputs itself (hold on ego motion, then fade to the model); the host's
      # model_valid flag carries the staleness. Returning None here would reset the controller.
    mode = (ControlMode.COMBINED if longitudinal_active else ControlMode.LATERAL_ONLY) if lateral_active else (
      ControlMode.LONGITUDINAL_ONLY if longitudinal_active else ControlMode.OFF)
    dt = DT_CTRL if previous is None or discontinuity else (now_ns - previous) / 1e9
    return LaneCenteringRequest(mode, self.settings, dt, discontinuity, self.version)
