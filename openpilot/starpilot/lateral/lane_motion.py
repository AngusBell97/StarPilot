"""Measured ego-motion for aligning boundary profiles; no future vehicle prediction."""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class Motion:
  timestamp_ns: int
  measured_curvature: float
  previous_curvature: float
  roll: float
  delay: float
  rate_factor: float = 1.
  base_supported: bool = True

  def valid(self):
    return (type(self.timestamp_ns) is int and self.timestamp_ns > 0 and
            all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
                for v in (self.measured_curvature, self.previous_curvature, self.roll, self.delay, self.rate_factor)) and
            abs(self.measured_curvature) <= .2 and abs(self.previous_curvature) <= .2 and
            abs(self.roll) <= .5 and .01 <= self.delay <= 1.5 and 0 <= self.rate_factor <= 1 and
            self.base_supported is True)


def warp_profile(x, y, dx, dy, yaw, stations):
  c, s = math.cos(yaw), math.sin(yaw)
  xx, yy = c * (x - dx) + s * (y - dy), -s * (x - dx) + c * (y - dy)
  if not np.isfinite(xx).all() or not np.isfinite(yy).all() or not np.all(np.diff(xx) > 0):
    return None
  valid = (stations >= xx[0]) & (stations <= xx[-1])
  return np.interp(stations, xx, yy), valid


