"""SLC-only fixed comfort ceilings, before protective candidate arbitration."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Adjustment:
  now_ns: int
  target: float
  direction: int
  limit: float


class TransitionComfort:
  def __init__(self):
    self.baseline = None
    self.target = None
    self.started = self.deadline = self.release_until = 0
    self.direction = 0
    self.armed = self.pedal_was_pressed = False

  def step(self, result, frame, *, settings, selected, speed, acknowledged, manual, reengage,
           selected_acknowledged=False, limits=None):
    now = frame.now_ns
    valid_limits = (isinstance(limits, dict) and set(limits) == {'acceleration', 'braking'} and
                    all(type(v) in (int, float) and math.isfinite(v) and .1 <= v <= 3.5 for v in limits.values()))
    if (not settings.transition_offsets or not frame.authority.can_control or frame.host.force_decel or result.errors or reengage or
        not all(math.isfinite(v) for v in (selected, speed)) or not 1 <= selected <= 80):
      self.__init__()
      return None
    accepted = result.state.acceptance.accepted
    candidate = result.selection.observation.candidate if result.selection is not None else None
    eligible = (accepted is not None and candidate is not None and
                accepted.candidate.speed_mps == candidate.speed_mps and
                result.domain is not None and result.domain.context is not None)
    if eligible:
      key = (accepted.candidate.source, accepted.candidate.speed_mps)
      old, self.baseline = self.baseline, key
      if old is not None and abs(old[1] - key[1]) >= .1:
        # Preserve the existing policy for a recalled/manual target present
        # before this transition. Intervention after arming is retained below.
        if result.state.override.persistent_selected_mps is not None:
          self.target = None
          self.direction = 0
          return None
        self.target = result.domain.context.effective_cluster_mps
        error = self.target - speed
        self.direction = 1 if error > .05 else -1 if error < -.05 else 0
        self.started = now
        self.release_until = 0
        self.pedal_was_pressed = False
        self.deadline = now + int(min(600., max(30., 1.5 * abs(error) / .1 + 10.)) * 1e9)
        self.armed = False
    if self.target is None or not self.started <= now <= self.deadline:
      self.target = None
      return None
    # After SLC has handed over a target, the observed selected speed owns it.
    # Wheel/Bluetooth/pedal targets may revise it without cancelling the episode.
    # No manual-only change can create an episode or extend its original expiry.
    if not self.armed:
      self.armed = bool((acknowledged or selected_acknowledged) and abs(selected - self.target) <= .15)
      if not self.armed:
        if now - self.started > 1_000_000_000:
          self.target = None
        return None
    if abs(selected - self.target) > .01:
      self.target = selected
      error = selected - speed
      self.direction = 1 if error > .05 else -1 if error < -.05 else 0
    if self.pedal_was_pressed and not frame.pedal_pressed:
      self.release_until = now + 500_000_000  # Allow the existing pedal capture to reach card.
    self.pedal_was_pressed = frame.pedal_pressed
    if frame.pedal_pressed:
      return None
    if self.direction * (self.target - speed) <= .05:
      if not manual and now >= self.release_until:
        self.target = None
      return None
    # A temporarily unqualified profile frame suppresses output, not the
    # accepted transition. Fresh settings can resume it before its old deadline.
    if not valid_limits:
      return None
    return Adjustment(now, self.target, self.direction, limits['acceleration' if self.direction > 0 else 'braking'])


def ceiling(request, *, now_ns, selected, speed, blocked):
  if (not isinstance(request, Adjustment) or blocked or request.direction not in (-1, 1) or
      not 0 <= now_ns - request.now_ns <= 100_000_000 or
      not all(math.isfinite(x) for x in (request.target, request.limit, selected, speed)) or
      not .1 <= request.limit <= 3.5 or abs(selected - request.target) > .15 or
      request.direction * (request.target - speed) <= .05):
    return None
  return request.limit
