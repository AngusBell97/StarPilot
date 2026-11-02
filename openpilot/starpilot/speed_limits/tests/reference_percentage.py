"""SP-011 temporary cruise shaping, before ordinary MPC/model arbitration."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Adjustment:
  now_ns:int
  target:float
  direction:int
  percent:float

class TransitionComfort:
  def __init__(self):
    self.baseline=None;self.target=None;self.started=0;self.deadline=0;self.direction=0

  def step(self, result, frame, *, settings, selected, speed, acknowledged, manual, reengage, selected_acknowledged=False):
    now=frame.now_ns
    accepted=result.state.acceptance.accepted
    candidate=result.selection.observation.candidate if result.selection is not None else None
    # Keep only the shaping of an already acknowledged, unchanged selected
    # target. Requalification of map acceptance must not drop its percentage.
    # This grants no map authority and cannot select or restore a speed.
    carry = (self.target is not None and selected_acknowledged and settings.transition_offsets and
      frame.authority.can_control and not frame.host.force_decel and
      not result.errors and not manual and not reengage and
      result.state.override.persistent_selected_mps is None and abs(selected-self.target)<=.15 and
      self.started<=now<=self.deadline and self.direction*(self.target-speed)>.05)
    if carry:
      return None if frame.pedal_pressed else Adjustment(now,self.target,self.direction,settings.accel_percent if self.direction>0 else settings.decel_percent)
    eligible=(settings.transition_offsets and accepted is not None and candidate is not None and
      accepted.candidate.speed_mps==candidate.speed_mps and result.domain is not None and result.domain.context is not None and
      frame.authority.can_control and not frame.host.force_decel and not result.errors)
    if not eligible:
      # A source gap cannot issue a new target, but need not interrupt shaping
      # of a target already acknowledged by card. Vehicle authority, driver
      # intent, selected-speed identity and the completion deadline still apply.
      carry = (self.target is not None and acknowledged and settings.transition_offsets and
        frame.authority.can_control and not frame.host.force_decel and
        not result.errors and not manual and not reengage and
        result.state.override.persistent_selected_mps is None and abs(selected-self.target)<=.15 and
        self.started<=now<=self.deadline and self.direction*(self.target-speed)>.05)
      if carry:
        return None if frame.pedal_pressed else Adjustment(now,self.target,self.direction,settings.accel_percent if self.direction>0 else settings.decel_percent)
      self.target=None;self.direction=0
      if not frame.authority.can_control or not settings.transition_offsets or frame.host.force_decel:
        self.baseline=None
      return None
    key=(accepted.candidate.source,accepted.candidate.speed_mps)
    target=result.domain.context.effective_cluster_mps
    old=self.baseline;self.baseline=key
    if old is not None and abs(old[1]-key[1])>=.1:
      error=target-speed
      self.direction=1 if error>.05 else -1 if error<-.05 else 0
      self.target=target;self.started=now
      self.deadline=now+int(min(600.,max(30.,1.5*abs(error)/.1+10.))*1e9)
    if (manual or reengage or result.state.override.persistent_selected_mps is not None or self.target is None or
        abs(target-self.target)>.1 or now>self.deadline or self.direction*(target-speed)<=.05):
      self.target=None;self.direction=0
      return None
    if not acknowledged or abs(selected-target)>.15:
      if now-self.started>1_000_000_000:self.target=None;self.direction=0
      return None
    percent=settings.accel_percent if self.direction>0 else settings.decel_percent
    # Pedal intervention pauses shaping, not the acknowledged target's history.
    # A replacement manual target, lost authority or completion still cancels
    # above. Neither the original deadline nor the acknowledgment is renewed.
    return None if frame.pedal_pressed else Adjustment(now,target,self.direction,percent)


def percentage(request, *, now_ns, selected, speed, blocked):
  if (not isinstance(request,Adjustment) or blocked or request.direction not in (-1,1) or
      not 0<=now_ns-request.now_ns<=100_000_000 or
      not all(math.isfinite(x) for x in (request.target,request.percent,selected,speed)) or
      not -50<=request.percent<=50 or abs(selected-request.target)>.15 or
      request.direction*(request.target-speed)<=.05):return 0.
  return request.percent
