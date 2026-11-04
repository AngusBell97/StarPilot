"""Lane version 8 (lane-led centring): saved settings, host request and the dispatcher-facing controller.

The controller itself is lane_led.py. This module reads its settings from params (one LaneCenteringV8* key per
slider), converts the host's LaneCenteringInput plus Motion into a LaneLedObservation, and returns the
LaneCenteringResult shape the host, the feedback message and the diagnostics expect.
"""
from dataclasses import fields
import json
import math
from pathlib import Path

import numpy as np

from openpilot.common.params import UnknownKeyName
from openpilot.starpilot.lateral.lane_centering import LaneCenteringResult, MAX_CORRECTION
from openpilot.starpilot.lateral.lane_led import LaneLedController, LaneLedObservation, boundaries_from_model, ALGORITHM
from openpilot.starpilot.lateral.lane_led_settings import LaneLedSettings, RANGES, valid as settings_valid

V8_VERSION = 8
V8_LABEL = "v8"
OBSTACLE_MAX_AGE_NS = 350_000_000             # a detection older than this (10 Hz, read ~0.1 s after its frame) is no detector


class ObstacleFeed:
  """Latest laneObstacleWire from modeld (change 37). Never raises: any problem reads as 'no detector'."""

  def __init__(self, sock=None):
    self.sock = sock
    self._tried = sock is not None
    self.last = None
    self.last_ns = 0

  def poll(self, now_ns: int):
    try:
      if self.sock is None and not self._tried:
        self._tried = True
        import openpilot.cereal.messaging as messaging
        from openpilot.starpilot.lateral.lane_obstacle_wire import SERVICE
        self.sock = messaging.sub_sock(SERVICE, conflate=True)
      if self.sock is not None:
        import openpilot.cereal.messaging as messaging
        from openpilot.starpilot.lateral.lane_obstacle_wire import decode
        msg = messaging.recv_one_or_none(self.sock)
        if msg is not None and msg.valid:
          wire = decode(bytes(msg.laneObstacleWire))
          if wire is not None:
            self.last, self.last_ns = wire, now_ns
    except Exception:
      self.sock, self.last = None, None
    w = self.last
    # Age from when the detected frame was processed (carried in the result), not from when the result arrived.
    ok = w is not None and w.healthy and 0 <= now_ns - w.detect_mono_ns <= OBSTACLE_MAX_AGE_NS
    if not ok:
      return False, False, math.inf, 0.0
    return True, bool(w.blocked), float(w.distance), float(w.length)
PREFIX = "LaneCenteringV8"
LEVER_STATE_KEY = PREFIX + "LeverState"      # the lever learner's regression state (JSON), carried across drives
_STORE_INTERVAL_S = 60.0
_STORE_MIN_FRAMES = 20

# Slider fields and their saved keys. "enabled" uses the common LaneCentering master switch.
_FIELDS = tuple(f.name for f in fields(LaneLedSettings) if f.name != "enabled")


def _camel(name: str) -> str:
  return "".join(part.capitalize() for part in name.split("_"))


V8_MAP = {name: PREFIX + _camel(name) for name in _FIELDS}        # field -> key
V8_FIELDS = {key: name for name, key in V8_MAP.items()}           # key -> field
_DEFAULT_SETTINGS = LaneLedSettings()
V8_DEFAULTS = {key: getattr(_DEFAULT_SETTINGS, name) for name, key in V8_MAP.items()}
V8_BOOLS = frozenset(key for key, name in V8_FIELDS.items() if isinstance(getattr(_DEFAULT_SETTINGS, name), bool))


def _raw(params, key):
  try:
    return Path(params.get_param_path(key)).read_bytes()
  except (FileNotFoundError, UnknownKeyName):
    return None   # an unregistered key (library not rebuilt for a new field) reads as its default


def settings(params) -> LaneLedSettings:
  """Saved v8 settings; any unreadable or out-of-range value yields the defaults (with the master switch)."""
  try:
    enabled_raw = _raw(params, "LaneCentering")
    enabled = enabled_raw is None or enabled_raw == b"1"
    values = {}
    for key, name in V8_FIELDS.items():
      raw = _raw(params, key)
      default = V8_DEFAULTS[key]
      if raw is None:
        values[name] = default
      elif key in V8_BOOLS:
        if raw not in (b"0", b"1"):
          raise ValueError(key)
        values[name] = raw == b"1"
      else:
        value = float(raw)
        if not math.isfinite(value):
          raise ValueError(key)
        values[name] = value
    result = LaneLedSettings(enabled=enabled, **values)
    if not settings_valid(result):
      raise ValueError("invalid v8 settings")
    return result
  except (OSError, ValueError, TypeError, UnknownKeyName):
    return LaneLedSettings(enabled=False)


class V8Controller:
  """Dispatcher-facing wrapper: LaneCenteringVersions calls update(observation, motion=...) and reads .diagnostic."""

  def __init__(self, params=None):
    self.controller = LaneLedController()
    self.diagnostic = {}
    self._correction = 0.0          # the dispatcher writes an inherited scalar here on a version switch; unused
    self._settings = None
    self._params = params
    self._params_tried = params is not None
    self._state_loaded = False
    self._last_store_t = math.nan
    self._stored_frames = 0
    self._was_active = False
    self.obstacles = ObstacleFeed()

  def reset(self):
    self.controller.reset()      # the lever learner survives: it describes the car, not the engagement
    self.diagnostic = {}

  # -- lever learner persistence (best effort; never lets a params problem reach steering)
  def _handle(self):
    if not self._params_tried:
      self._params_tried = True
      try:
        from openpilot.common.params import Params
        self._params = Params()
      except Exception:
        self._params = None
    return self._params

  def _load_state(self):
    self._state_loaded = True
    p = self._handle()
    if p is None:
      return
    raw = _raw(p, LEVER_STATE_KEY)
    if not raw:
      return
    try:
      self.controller.load_lever_state(json.loads(raw))
    except (ValueError, TypeError):
      pass
    self._stored_frames = self.controller.lever_learner.accepted

  def _store_state(self, t: float, force: bool = False):
    accepted = self.controller.lever_learner.accepted
    if accepted - self._stored_frames < _STORE_MIN_FRAMES:
      return
    if not force and math.isfinite(self._last_store_t) and t - self._last_store_t < _STORE_INTERVAL_S:
      return
    p = self._handle()
    if p is None:
      return
    try:
      p.put(LEVER_STATE_KEY, json.dumps(self.controller.lever_state()))
    except Exception:
      return
    self._last_store_t = t
    self._stored_frames = accepted

  def _result(self, base, reason):
    return LaneCenteringResult(base, 0.0, 0, reason, MAX_CORRECTION)

  def update(self, observation, *, motion=None, measured_curvature=None, lateral_delay_s=None):
    base = float(observation.base_curvature) if observation.base_curvature is not None else 0.0
    if motion is None or not motion.valid():
      self.diagnostic = {"algorithm": V8_VERSION, "frame": observation.model_sample_id, "reason": "no_motion"}
      return self._result(base, "v8_no_motion")
    if isinstance(observation.settings, LaneLedSettings) and observation.settings is not self._settings:
      self.controller.set_settings(observation.settings)
      self._settings = observation.settings
    if not self._state_loaded:
      self._load_state()
    model = observation.model
    try:
      position = model.position
      path_x = np.asarray(position.x, dtype=float)
      path_y = np.asarray(position.y, dtype=float)
      path_std = np.asarray(position.yStd, dtype=float)
      if not (len(path_x) == len(path_y) == len(path_std)) or not np.isfinite(path_x).all() or not np.isfinite(path_y).all():
        path_x = path_y = path_std = None
    except (AttributeError, TypeError, ValueError):
      path_x = path_y = path_std = None
    try:
      lane_change = int(model.meta.laneChangeState) != 0
    except (AttributeError, TypeError, ValueError):
      lane_change = False
    pause = bool(getattr(observation.settings, "pause_on_signal", True))
    import time
    ob_ok, ob_blocked, ob_dist, ob_len = self.obstacles.poll(time.monotonic_ns())
    obs = LaneLedObservation(
      t=motion.timestamp_ns / 1e9, speed=float(observation.speed_mps),
      measured_curvature=float(motion.measured_curvature if measured_curvature is None else measured_curvature),
      model_curvature=base, delay=float(motion.delay), boundaries=boundaries_from_model(model),
      model_sample_id=int(observation.model_sample_id), published_curvature=float(motion.previous_curvature),
      model_path_x=path_x, model_path_y=path_y, model_path_ystd=path_std,
      lateral_active=bool(observation.lateral_active) and observation.mode.lateral_authority,
      lane_change=lane_change or (pause and bool(observation.turn_signal_active)),
      driver_override=bool(observation.driver_override), inputs_stale=not bool(observation.model_valid),
      obstacle_ok=ob_ok, obstacle_blocked=ob_blocked, obstacle_distance=ob_dist, obstacle_length=ob_len)
    r = self.controller.update(obs)
    # Persist the lever learner's state once a minute while it gathers frames, and when lateral control ends.
    self._store_state(obs.t, force=self._was_active and not obs.lateral_active)
    self._was_active = obs.lateral_active
    reason = r.state if r.state in ("lane_led", "blended") else f"{r.state}:{r.reason}"
    candidate = r.candidate_curvature if math.isfinite(r.candidate_curvature) else base
    correction = candidate - base
    fl = r.diagnostics.get("flags") or {}
    rd = lambda v, n=6: (round(v, n) if isinstance(v, float) else v)
    # Compact on purpose: this is logged at 20 Hz for a whole drive. Curvatures keep 7 places (1e-7 1/m), metres 3.
    self.diagnostic = {
      "algorithm": V8_VERSION, "frame": observation.model_sample_id, "state": r.state, "reason": r.reason,
      "weight": rd(r.weight, 3), "trust": rd(r.trust, 3), "e_y": rd(r.e_y, 3), "e_psi": rd(r.e_psi, 4),
      "kappa_road": rd(r.kappa_road, 7), "kappa_lane": rd(r.kappa_lane, 7), "bias": rd(r.bias, 7), "width": rd(r.width, 2),
      "mode": r.mode, "gap_s": rd(r.diagnostics.get("gap_s", 0.), 2), "residual": rd(r.diagnostics.get("residual"), 3),
      "conf": rd(r.diagnostics.get("conf"), 2), "sigma": rd(r.diagnostics.get("bandwidth_scale"), 2), "bridge": rd(r.diagnostics.get("bridge_s", 0.), 2),
      "lim_a": bool(fl.get("accel_limited", False)), "lim_j": bool(fl.get("jerk_limited", False)), "lim_t": bool(fl.get("total_limited", False)),
      "lever": rd(r.diagnostics.get("lever"), 3), "lever_n": int(r.diagnostics.get("lever_n", 0) or 0),
      "guard_l": rd(r.diagnostics.get("guard_l"), 2), "guard_r": rd(r.diagnostics.get("guard_r"), 2), "guard": bool(r.diagnostics.get("guard_active", False)),
      "ob_ok": ob_ok, "ob_blk": ob_blocked, "ob_d": rd(ob_dist if math.isfinite(ob_dist) else -1.0, 1),
      "ob_hold": bool(r.diagnostics.get("obstacle_hold", False)), "ob_left": rd(r.diagnostics.get("obstacle_left_m", 0.0), 1),
    }
    theta = r.diagnostics.get("lever_theta")
    if theta is not None:
      self.diagnostic["lever_theta"] = [rd(float(v), 6) for v in theta]
    return LaneCenteringResult(candidate, correction, (correction > 0) - (correction < 0), reason, MAX_CORRECTION)
