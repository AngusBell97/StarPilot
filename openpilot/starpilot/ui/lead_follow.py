"""Lead follow time: the live gap in profile seconds, its target, the lead widget text, and the button nudge."""

import json
import math
from copy import deepcopy

from openpilot.common.constants import CV
from openpilot.starpilot.longitudinal.profile_document import (CURVE_BOUNDS, FOLLOWING_SPEEDS_MPH, active_personality_id,
                                                               strict_profile_document)

# The planner holds this on top of the follow time (long_mpc STOP_DISTANCE), so a settled gap measures
# follow_seconds * speed + STOP_DISTANCE_M. Removing it makes the live value comparable with the profile.
STOP_DISTANCE_M = 6.0
MIN_SPEED_MPS = 2.0
# get_T_FOLLOW defaults when no personality profile applies; traffic from the traffic follow table.
DEFAULT_FOLLOW_S = {'traffic': 0.75, 'aggressive': 1.25, 'standard': 1.45, 'relaxed': 1.75}
NUDGE_STEP_S = 0.05
# Bands of live / target: on target within 15%, slightly close down to 70%, closer is too close.
TARGET_BAND = 0.15
CLOSE_RATIO = 0.7


def follow_time_s(distance_m, speed_mps) -> float | None:
  """Live follow time in profile seconds; None when stopped, crawling or without a usable gap."""
  try:
    distance, speed = float(distance_m), float(speed_mps)
  except (TypeError, ValueError, OverflowError):
    return None
  if not math.isfinite(distance) or not math.isfinite(speed) or distance <= 0.0 or speed < MIN_SPEED_MPS:
    return None
  return max(0.0, (distance - STOP_DISTANCE_M) / speed)


def default_follow_s(personality, traffic_mode: bool) -> float:
  personality_id = active_personality_id(bool(traffic_mode), personality)
  return DEFAULT_FOLLOW_S[personality_id if personality_id in DEFAULT_FOLLOW_S else 'standard']


def follow_state(current: float | None, target: float | None) -> str:
  """'none', 'far', 'target', 'near' or 'close' for the live follow time against its target."""
  if current is None or target is None or not target > 0.0:
    return 'none'
  ratio = current / target
  if ratio > 1.0 + TARGET_BAND:
    return 'far'
  if ratio >= 1.0 - TARGET_BAND:
    return 'target'
  return 'near' if ratio >= CLOSE_RATIO else 'close'


def _finite(value) -> float | None:
  try:
    number = float(value)
  except (TypeError, ValueError, OverflowError):
    return None
  return number if math.isfinite(number) else None


def lead_speed_text(lead, metric: bool) -> str:
  speed = _finite(getattr(lead, 'speed_mps', None)) if lead is not None else None
  if speed is None:
    return ""
  speed = max(speed, 0.0)
  return f"{round(speed * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))} {'km/h' if metric else 'mph'}"


def lead_distance_text(lead, metric: bool) -> str:
  distance = _finite(getattr(lead, 'distance_m', None)) if lead is not None else None
  if distance is None:
    return ""
  distance = max(distance, 0.0)
  return f"{round(distance if metric else distance * 3.28084)} {'m' if metric else 'ft'}"


def lead_time_text(lead, speed_mps) -> str:
  current = follow_time_s(getattr(lead, 'distance_m', None), speed_mps) if lead is not None else None
  return "" if current is None else f"{current:.1f} s"


def follow_target_text(target: float | None) -> str:
  return "" if target is None else f"{target:.1f} s"


class FollowTargetReader:
  """The follow time the planner is aiming for: the saved profile at this speed, else the personality default."""

  def __init__(self, params):
    self.params = params
    self._host = None

  def sample(self, now_ns: int, personality, speed_mps, cp, *, traffic_mode: bool) -> float | None:
    personality_int = _personality_int(personality)
    if personality_int is None:
      return None
    fallback = default_follow_s(personality_int, traffic_mode)
    speed = _finite(speed_mps)
    if cp is None or speed is None or speed < 0.0:
      return fallback
    try:
      if self._host is None:
        from openpilot.starpilot.longitudinal.profile_runtime import ProfileHost
        self._host = ProfileHost(self.params)
      tuning = self._host.sample(int(now_ns), personality_int, speed, cp, traffic_mode=bool(traffic_mode))
    except (AttributeError, OSError, RuntimeError, TypeError, ValueError, KeyError):
      tuning = None
    follow = _finite(getattr(tuning, 'follow_seconds', None)) if tuning is not None else None
    return follow if follow is not None and follow > 0.0 else fallback


def _personality_int(personality) -> int | None:
  raw = getattr(personality, 'raw', personality)
  try:
    value = int(raw)
  except (TypeError, ValueError):
    return None
  return value if value in (0, 1, 2) and not isinstance(raw, bool) else None


def nudge_following_curve(curve, speed_mph, delta_s: float) -> list[float] | None:
  """The custom following curve with the value at this speed moved by delta_s.

  The grid is every 10 mph, so the two breakpoints around the speed share the change in proportion to
  how close each is, scaled so the interpolated value at the current speed moves by exactly delta_s.
  None when nothing changes (the curve is already at a bound) or the curve is not the custom grid."""
  try:
    values = [float(value) for value in curve]
    speed = float(speed_mph)
  except (TypeError, ValueError, OverflowError):
    return None
  if len(values) != len(FOLLOWING_SPEEDS_MPH) or not math.isfinite(speed) or not math.isfinite(delta_s) or delta_s == 0.0:
    return None
  step = FOLLOWING_SPEEDS_MPH[1] - FOLLOWING_SPEEDS_MPH[0]
  if speed <= FOLLOWING_SPEEDS_MPH[0]:
    low, fraction = 0, 0.0
  elif speed >= FOLLOWING_SPEEDS_MPH[-1]:
    low, fraction = len(values) - 1, 0.0
  else:
    low = int((speed - FOLLOWING_SPEEDS_MPH[0]) // step)
    fraction = (speed - FOLLOWING_SPEEDS_MPH[low]) / step
  weight = (1.0 - fraction) ** 2 + fraction ** 2
  shares = {low: (1.0 - fraction) / weight}
  if fraction > 0.0:
    shares[low + 1] = fraction / weight
  minimum, maximum = CURVE_BOUNDS["following"]
  updated = list(values)
  for index, share in shares.items():
    updated[index] = round(min(max(values[index] + delta_s * share, minimum), maximum), 4)
  return None if updated == values else updated


def nudged_profile_document(document: dict, personality, traffic_mode: bool, speed_mps, delta_s: float) -> dict | None:
  """A canonical copy of the saved profile document with the active custom following curve nudged; None when not applicable."""
  personality_id = active_personality_id(bool(traffic_mode), personality)
  speed = _finite(speed_mps)
  if personality_id is None or speed is None or type(document) is not dict:
    return None
  try:
    config = document['profiles'][personality_id]['following']
    preset, curve = config['preset'], config['curve']
  except (KeyError, TypeError):
    return None
  if preset != 'custom':
    return None
  updated_curve = nudge_following_curve(curve, speed * CV.MS_TO_MPH, delta_s)
  if updated_curve is None:
    return None
  updated = deepcopy(document)
  updated['profiles'][personality_id]['following']['curve'] = updated_curve
  return strict_profile_document(updated)


def encode_profile_document(document: dict) -> bytes:
  return json.dumps(document, separators=(",", ":"), sort_keys=True, allow_nan=False).encode()


def interpolated_follow_s(curve, speed_mph) -> float | None:
  """The custom curve's value at this speed, for showing what a nudge changed."""
  try:
    values = [float(value) for value in curve]
    speed = float(speed_mph)
  except (TypeError, ValueError, OverflowError):
    return None
  if len(values) != len(FOLLOWING_SPEEDS_MPH) or not math.isfinite(speed):
    return None
  if speed <= FOLLOWING_SPEEDS_MPH[0]:
    return values[0]
  if speed >= FOLLOWING_SPEEDS_MPH[-1]:
    return values[-1]
  step = FOLLOWING_SPEEDS_MPH[1] - FOLLOWING_SPEEDS_MPH[0]
  low = int((speed - FOLLOWING_SPEEDS_MPH[0]) // step)
  fraction = (speed - FOLLOWING_SPEEDS_MPH[low]) / step
  return values[low] + (values[low + 1] - values[low]) * fraction
