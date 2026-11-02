"""SP-016 selector bridge; upstream Chubbs policy remains unchanged."""
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.lateral.chubbs_lane import ChubbsLaneCentering


def selected(params):
  value, readable = read_saved(params, "LaneCenteringVersion", 8)
  return readable and value == b"9"


class ModelLanePolicy:
  """Freeze the algorithm for the drive; refresh only the common master switch."""
  def __init__(self, params):
    self.params = params
    self.policy = ChubbsLaneCentering(selected(params))
    self.refresh_ns = None
    self.enabled = False

  def apply(self, outputs, action, speed, *, now_ns, active, desire, lane_change):
    if not self.policy.enabled:
      return action
    if self.refresh_ns is None or now_ns < self.refresh_ns or now_ns - self.refresh_ns >= 1_000_000_000:
      value, readable = read_saved(self.params, "LaneCentering", 8)
      self.enabled = readable and value in (None, b"1")
      self.refresh_ns = now_ns
    return self.policy.apply(outputs, action, speed, active=active and self.enabled,
                             desire=desire, lane_change=lane_change)
