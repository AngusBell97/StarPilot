"""Fresh primary-lead classification for speed settings only, never braking authority.

The perception producer already selects leadOne. Do not require the full model
path or a speed-scaled following distance to choose between user speed limits.
"""

import math

MAX_AGE_NS = 150_000_000
ABSENCE_CONFIRM_NS = 500_000_000


def _number(value, low, high):
  return type(value) in (int, float) and math.isfinite(value) and low <= value <= high


class SpeedLeadSelector:
  def __init__(self):
    self.reset()

  def reset(self):
    self.present = None
    self.last_ns = None
    self.absent_since = None

  def step(self, lead, *, observed_ns, now_ns, fresh):
    if (fresh is not True or type(observed_ns) is not int or type(now_ns) is not int or
        not 0 < observed_ns <= now_ns or now_ns - observed_ns > MAX_AGE_NS):
      self.reset()
      return None
    if self.last_ns is not None and (observed_ns < self.last_ns or observed_ns - self.last_ns > MAX_AGE_NS):
      self.reset()
    if self.last_ns == observed_ns:
      return self.present  # Reuse only within the original stamp age; never advance confirmation.
    self.last_ns = observed_ns
    try:
      if type(lead.present) is not bool or type(lead.radar) is not bool:
        raise ValueError
      if lead.present:
        probability = lead.modelProb
        if (not _number(lead.dRel, 0.1, 250.) or not _number(lead.vLead, -30., 100.) or
            not _number(lead.yRel, -20., 20.) or not _number(probability, 0., 1.) or
            not (lead.radar or probability >= (0.7 if self.present is True else 0.9))):
          raise ValueError
        self.present = True
        self.absent_since = None
      elif self.present is not False:
        if self.absent_since is None:
          self.absent_since = observed_ns
        if observed_ns - self.absent_since >= ABSENCE_CONFIRM_NS:
          self.present = False
      else:
        self.present = False
      return self.present
    except (AttributeError, TypeError, ValueError, OverflowError):
      self.reset()
      return None
