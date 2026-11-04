"""The stop detector's model horizon is the path's arc length, not its x extent (4 Nov 2026)."""
import math
from types import SimpleNamespace
import unittest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.starpilot.conditional_mode.projection import _horizon

N = ModelConstants.IDX_N


def model(x, y):
  return SimpleNamespace(position=SimpleNamespace(x=list(x), y=list(y)))


class StopHorizonTests(unittest.TestCase):
  def test_straight_path_keeps_its_x_extent(self):
    x = [150.0 * i / (N - 1) for i in range(N)]
    self.assertAlmostEqual(_horizon(model(x, [0.0] * N)), 150.0)

  def test_curved_path_measures_arc_length_not_x(self):
    # A constant-curvature arc of radius 100 m over 1.4 rad: the car travels 140 m but the path's x extent is 98.5 m.
    radius, total = 100.0, 1.4
    x = [radius * math.sin(total * i / (N - 1)) for i in range(N)]
    y = [radius * (1 - math.cos(total * i / (N - 1))) for i in range(N)]
    horizon = _horizon(model(x, y))
    self.assertGreater(horizon, 139.0)
    self.assertLess(horizon, 140.1)
    self.assertLess(x[-1], 99.0)

  def test_invalid_x_still_rejected_and_bad_y_falls_back_to_x(self):
    x = [150.0 * i / (N - 1) for i in range(N)]
    bad = list(x)
    bad[5] = bad[4] - 1.0  # x going backwards is not a usable path
    self.assertIsNone(_horizon(model(bad, [0.0] * N)))
    self.assertAlmostEqual(_horizon(model(x, [math.nan] * N)), 150.0)
    self.assertAlmostEqual(_horizon(model(x, [0.0] * (N - 1))), 150.0)


if __name__ == '__main__':
  unittest.main()
