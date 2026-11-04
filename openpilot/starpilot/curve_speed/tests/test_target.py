import math
from types import SimpleNamespace
import unittest
from openpilot.starpilot.curve_speed.learning import LearnedCurve
from openpilot.starpilot.curve_speed.target import COMFORT_SCALE, CurveProfile, Envelope, TargetFilter, evaluate


class TestTarget(unittest.TestCase):
  def test_full_profile_selects_binding_apex_and_distance(self):
    result = evaluate(CurveProfile((0.0, 0.02, 0.01), (0.0, 150.0, 200.0), 100), LearnedCurve(), 30.0)
    assert result.binding_distance_m == 150
    apex = max(math.sqrt(LearnedCurve().comfort(0.02) * COMFORT_SCALE / 0.02), 25 * 0.44704)
    self.assertAlmostEqual(result.speed_mps, math.sqrt(apex ** 2 + 0.6 * 150))
    assert evaluate(CurveProfile((0.02,), (2000.0,), 100), LearnedCurve(), 30.0) == Envelope(30.0, 0.0)

  def test_comfort_scale_raises_the_apex_but_not_the_physical_cap(self):
    curve = LearnedCurve()
    result = evaluate(CurveProfile((0.01,), (0.0,), 100), curve, 40.0)
    self.assertAlmostEqual(result.apex_speed_mps, math.sqrt(curve.comfort(0.01) * COMFORT_SCALE / 0.01))
    assert result.apex_speed_mps > math.sqrt(curve.comfort(0.01) / 0.01)
    assert result.apex_speed_mps <= math.sqrt(4.0 / 0.01)

  def test_farfield_correction_and_weather_only_lower_envelope(self):
    curve = LearnedCurve()
    profile = CurveProfile((0.004,), (30.0,), 100)
    corrected = evaluate(profile, curve, 35.0)
    expected = math.sqrt(curve.comfort(0.004 * 1.23) * COMFORT_SCALE / (0.004 * 1.23) + 0.6 * 30)
    self.assertAlmostEqual(corrected.speed_mps, expected)
    assert evaluate(profile, curve, 35.0, weather_reduction=0.3).speed_mps < corrected.speed_mps
    assert evaluate(CurveProfile((-0.004,), (30.0,), 100), curve, 35.0) == corrected

  def test_invalid_profile_rejected_before_arithmetic(self):
    for curvatures, distances in [
      ((), ()),
      ((0.1,), ()),
      ((math.nan,), (0.0,)),
      ((0.1,), (-1.0,)),
      ((0.1,), (math.inf,)),
      ((0.1, 0.2), (1.0, 0.0)),
      ((0.1,) * 130, (0.0,) * 130),
    ]:
      with self.subTest(curvatures=curvatures, distances=distances):
        with self.assertRaises(ValueError):
          CurveProfile(curvatures, distances, 100)

  def test_model_extraction_checks_whole_profile_and_low_speed(self):
    model = SimpleNamespace(
      orientationRate=SimpleNamespace(z=[0.3, -0.3, 0.3]), velocity=SimpleNamespace(x=[0.0, 10.0, 2.99]), position=SimpleNamespace(x=[0.0, 10.0, 20.0])
    )
    profile = CurveProfile.from_model(model, 100)
    assert profile == CurveProfile((0.0, 0.03, 0.0), (0.0, 10.0, 20.0), 100)
    model.velocity.x[1] = math.inf
    assert CurveProfile.from_model(model, 100) is None

  def test_seed_rate_limits_and_exit_recovery(self):
    target = TargetFilter()
    previous = 32.0
    for _ in range(300):
      current = target.step(Envelope(15.0, 20.0), ego_mps=30.0, cruise_mps=35.0, dt=0.05)
      assert 0 <= previous - current <= 1.5 * 0.05 + 1e-12
      previous = current
    assert current < 15.1
    # The target settled more than RISE_HOLD_SECONDS ago, so the exit recovers at the (slower) up rate.
    released = target.step(Envelope(35.0, 0.0), ego_mps=15.0, cruise_mps=35.0, dt=0.05)
    self.assertAlmostEqual(released - current, 1.0 * 0.05)

  def test_rise_is_held_above_ego_speed_after_a_fall(self):
    target = TargetFilter()
    for _ in range(60):
      current = target.step(Envelope(15.0, 20.0), ego_mps=20.0, cruise_mps=35.0, dt=0.05)
    assert 15.0 < current < 22.0 and target._since_fall == 0.0
    # The envelope opens again (prediction wobble): the target may rise to the car's speed but not above it.
    held = [target.step(Envelope(35.0, 0.0), ego_mps=20.0, cruise_mps=35.0, dt=0.05) for _ in range(39)]
    assert max(held) <= 20.0 + 1e-9
    assert held[-1] == 20.0
    # After RISE_HOLD_SECONDS the ordinary up rate applies.
    after = target.step(Envelope(35.0, 0.0), ego_mps=20.0, cruise_mps=35.0, dt=0.05)
    self.assertAlmostEqual(after - held[-1], 1.0 * 0.05)
    # A renewed fall (once the filter has drained) restarts the hold.
    for _ in range(40):
      fallen = target.step(Envelope(15.0, 20.0), ego_mps=20.0, cruise_mps=35.0, dt=0.05)
    assert fallen < after and target._since_fall == 0.0
    assert target.step(Envelope(35.0, 0.0), ego_mps=20.0, cruise_mps=35.0, dt=0.05) <= max(fallen, 20.0) + 1e-9

  def test_falling_envelope_is_filtered_faster_than_rising(self):
    down, up = TargetFilter(), TargetFilter()
    for filt in (down, up):
      filt.step(Envelope(25.0, 0.0), ego_mps=23.0, cruise_mps=30.0, dt=0.05)
      assert filt._filtered == 25.0
    down.step(Envelope(20.0, 0.0), ego_mps=23.0, cruise_mps=30.0, dt=0.05)
    up.step(Envelope(30.0, 0.0), ego_mps=23.0, cruise_mps=30.0, dt=0.05)
    assert 25.0 - down._filtered > up._filtered - 25.0 > 0  # the fall moved further in one step than the rise

  def test_model_origin_roundoff_is_normalized_without_relaxing_future_geometry(self):
    model = SimpleNamespace(orientationRate=SimpleNamespace(z=[0.1, 0.2, 0.3]),
                            velocity=SimpleNamespace(x=[10.0, 10.0, 10.0]),
                            position=SimpleNamespace(x=[-6.64888977208733e-11, 10.0, 20.0]))
    profile = CurveProfile.from_model(model, 100)
    self.assertEqual(profile.distances, (0.0, 10.0, 20.0))
    self.assertLess(model.position.x[0], 0.0)
    for distances in ([-1.0001e-6, 10.0, 20.0], [0.0, -1e-11, 20.0], [-1e-11, 10.0, 9.0]):
      with self.subTest(distances=distances):
        model.position.x = distances
        self.assertIsNone(CurveProfile.from_model(model, 100))

  def test_driver_reduction_is_immediate_ceiling_and_clock_gap_resets(self):
    target = TargetFilter()
    target.step(Envelope(35.0, 0.0), ego_mps=30.0, cruise_mps=35.0, dt=0.05)
    assert target.step(Envelope(15.0, 0.0), ego_mps=30.0, cruise_mps=15.0, dt=0.05) == 15.0
    with self.assertRaises(ValueError):
      target.step(Envelope(15.0, 0.0), ego_mps=30.0, cruise_mps=35.0, dt=1.0)
    assert target.target is None

  def test_unknown_weather_cannot_enter_envelope(self):
    for weather in [-0.1, 1.1, math.nan, True]:
      with self.subTest(weather=weather):
        with self.assertRaises(ValueError):
          evaluate(CurveProfile((0.01,), (0.0,), 100), LearnedCurve(), 30.0, weather_reduction=weather)
