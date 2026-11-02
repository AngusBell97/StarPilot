"""Validate lane feedback for request-cap usage and host-limited direction."""

import math

from openpilot.cereal import messaging
from openpilot.starpilot.lateral.lane_centering import MAX_CORRECTION, STOCK_CORRECTION

SERVICE = 'starpilotLateralState'
MAX_AGE_NS = 250_000_000
MAX_PAIR_SKEW_NS = 100_000_000
DIRECTION_EPSILON = 1e-6
BLUE = (0, 176, 220)


def feedback_message(result, applied, lateral_active, *, model_mono_time, car_control_mono_time, valid):
  event = messaging.new_message(SERVICE, valid=valid)
  state = event.starpilotLateralState.laneCentering
  state.version = 2
  state.correctionLimit = result.correction_limit if result is not None else STOCK_CORRECTION
  state.modelMonoTime = model_mono_time
  state.carControlMonoTime = car_control_mono_time
  state.lateralActive = lateral_active
  state.requestedCorrection = result.correction if result is not None else 0.0
  state.appliedCorrection = applied if lateral_active else 0.0
  state.reason = result.reason if result is not None else 'inactive'
  return event


def correction_feedback(sm, now_ns: int, after_frame: int, *, use_request: bool = False) -> tuple[float, float]:
  try:
    clocks = {}
    for service in (SERVICE, 'carControl', 'modelV2'):
      stamp = int(sm.logMonoTime[service])
      if (not sm.alive[service] or not sm.valid[service] or sm.recv_frame[service] <= after_frame or
          not 0 < stamp <= now_ns or now_ns - stamp > MAX_AGE_NS):
        return 0.0, STOCK_CORRECTION
      clocks[service] = stamp
    state = sm[SERVICE].laneCentering
    requested, applied = float(state.requestedCorrection), float(state.appliedCorrection)
    limit = float(state.correctionLimit)
    if (state.version != 2 or not state.lateralActive or not sm['carControl'].latActive or
        not 0 < state.modelMonoTime <= now_ns or now_ns - state.modelMonoTime > MAX_AGE_NS or
        not 0 < state.carControlMonoTime <= clocks[SERVICE] or
        abs(state.carControlMonoTime - clocks['carControl']) > MAX_PAIR_SKEW_NS or
        abs(state.modelMonoTime - clocks['modelV2']) > MAX_PAIR_SKEW_NS or
        not math.isfinite(requested) or not math.isfinite(applied) or
        not math.isfinite(limit) or not STOCK_CORRECTION - 1e-8 <= limit <= MAX_CORRECTION + 1e-8 or
        abs(requested) > limit + 1e-8 or abs(applied) > abs(requested) + 1e-8 or
        requested * applied < 0):
      return 0.0, STOCK_CORRECTION
    correction = requested if use_request else applied
    return (correction if abs(correction) > DIRECTION_EPSILON else 0.0), limit
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return 0.0, STOCK_CORRECTION


def applied_correction(sm, now_ns: int, after_frame: int) -> float:
  return correction_feedback(sm, now_ns, after_frame)[0]


def direction(sm, now_ns: int, after_frame: int) -> int:
  applied = applied_correction(sm, now_ns, after_frame)
  return (applied > 0) - (applied < 0)
