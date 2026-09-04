import sys
from enum import IntEnum
from types import ModuleType, SimpleNamespace

import pytest

from openpilot.starpilot.common.longitudinal_personality_profiles import default_personality_profiles, profile_document


def _module(name, **attributes):
  module = ModuleType(name)
  for key, value in attributes.items():
    setattr(module, key, value)
  return module


class LaneChangeState(IntEnum):
  off = 0
  preLaneChange = 1
  laneChangeStarting = 2
  laneChangeFinishing = 3


class LaneChangeDirection(IntEnum):
  none = 0
  left = 1
  right = 2


sys.modules["cereal"] = _module(
  "cereal",
  log=SimpleNamespace(LaneChangeState=LaneChangeState, LaneChangeDirection=LaneChangeDirection),
)
sys.modules["openpilot.common.constants"] = _module(
  "openpilot.common.constants", CV=SimpleNamespace(MPH_TO_MS=0.44704),
)
sys.modules["openpilot.common.realtime"] = _module("openpilot.common.realtime", DT_MDL=0.05)
sys.modules["openpilot.selfdrive.controls.lib.lead_behavior"] = _module(
  "openpilot.selfdrive.controls.lib.lead_behavior", should_disable_far_lead_throttle=lambda *_args: False,
)
sys.modules["openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc"] = _module(
  "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc",
  COMFORT_BRAKE=2.5,
  LEAD_DANGER_FACTOR=0.8,
  desired_follow_distance=lambda v_ego, _v_lead, t_follow: v_ego * t_follow,
  get_jerk_factor=lambda *_args: (1.0, 1.0, 1.0),
  get_T_FOLLOW=lambda *_args: 1.45,
)
sys.modules["openpilot.starpilot.common.starpilot_variables"] = _module(
  "openpilot.starpilot.common.starpilot_variables", CITY_SPEED_LIMIT=11.176, MAX_T_FOLLOW=3.0,
)

from openpilot.starpilot.controls.lib.starpilot_following import StarPilotFollowing


class Personality(IntEnum):
  aggressive = 0
  standard = 1
  relaxed = 2


def _planner(*, weather_id=0, weather_increase=0.0):
  lead = SimpleNamespace(status=False, dRel=1000.0, vLead=0.0, aLeadK=0.0)
  return SimpleNamespace(
    lead_one=lead,
    starpilot_weather=SimpleNamespace(weather_id=weather_id, increase_following_distance=weather_increase),
    tracking_lead=False,
  )


def _sm(*, traffic=False, personality=Personality.standard):
  return {
    "carState": SimpleNamespace(aEgo=0.0, standstill=False, leftBlindspot=False, rightBlindspot=False),
    "selfdriveState": SimpleNamespace(personality=personality),
    "starpilotCarState": SimpleNamespace(trafficModeEnabled=traffic),
  }


def _toggles(document):
  return SimpleNamespace(
    aggressive_follow=1.25,
    aggressive_jerk_acceleration=1.0,
    aggressive_jerk_danger=1.0,
    aggressive_jerk_deceleration=1.0,
    aggressive_jerk_speed=1.0,
    aggressive_jerk_speed_decrease=1.0,
    conditional_slower_lead=False,
    custom_personalities=True,
    lane_change_close_gap=False,
    lane_change_close_gap_seconds=0.75,
    longitudinal_personality_profiles=document,
    minimum_lane_change_speed=0.0,
    personality_ev_tuning=False,
    relaxed_follow=1.6,
    relaxed_jerk_acceleration=1.0,
    relaxed_jerk_danger=1.0,
    relaxed_jerk_deceleration=1.0,
    relaxed_jerk_speed=1.0,
    relaxed_jerk_speed_decrease=1.0,
    standard_follow=1.45,
    standard_jerk_acceleration=1.0,
    standard_jerk_danger=1.0,
    standard_jerk_deceleration=1.0,
    standard_jerk_speed=1.0,
    standard_jerk_speed_decrease=1.0,
    traffic_mode_follow=[0.75, 1.0],
    traffic_mode_jerk_acceleration=[1.0, 1.0],
    traffic_mode_jerk_danger=[1.0, 1.0],
    traffic_mode_jerk_deceleration=[1.0, 1.0],
    traffic_mode_jerk_speed=[1.0, 1.0],
    traffic_mode_jerk_speed_decrease=[1.0, 1.0],
  )


def _document(*, enabled=True):
  return profile_document(default_personality_profiles(False), enabled=enabled)


def test_explicit_following_curve_selects_active_personality_and_linear_speed_point():
  document = _document()
  document["profiles"]["standard"]["following"] = {
    "preset": "custom",
    "curve": [0.75 + 0.1 * index for index in range(10)],
  }
  controller = StarPilotFollowing(_planner())

  controller.update(True, 5.0 * 0.44704, _sm(), _toggles(document))

  assert controller.t_follow == pytest.approx(0.80)
  assert controller.base_acceleration_jerk == 1.0


def test_enabled_following_document_is_independent_of_legacy_custom_personalities_toggle():
  document = _document()
  document["profiles"]["standard"]["following"] = {"preset": "custom", "curve": [0.9] * 10}
  toggles = _toggles(document)
  toggles.custom_personalities = False

  controller = StarPilotFollowing(_planner())
  controller.update(True, 10.0, _sm(), toggles)

  assert controller.t_follow == pytest.approx(0.9)


def test_traffic_profile_wins_over_cereal_personality_without_changing_jerk():
  document = _document()
  document["profiles"]["traffic"]["following"] = {"preset": "far", "curve": []}
  controller = StarPilotFollowing(_planner())

  controller.update(True, 0.0, _sm(traffic=True, personality=Personality.aggressive), _toggles(document))

  assert controller.t_follow == pytest.approx(1.55)
  assert controller.base_acceleration_jerk == 1.0


@pytest.mark.parametrize("document", [None, {}, _document(enabled=False)])
def test_absent_malformed_or_disabled_document_keeps_legacy_standard_follow(document):
  controller = StarPilotFollowing(_planner())

  controller.update(True, 0.0, _sm(), _toggles(document))

  assert controller.t_follow == pytest.approx(1.45)


def test_dom_default_category_keeps_legacy_traffic_follow():
  controller = StarPilotFollowing(_planner())

  controller.update(True, 0.0, _sm(traffic=True), _toggles(_document()))

  assert controller.t_follow == pytest.approx(0.75)


def test_existing_weather_modifier_runs_after_profile_and_retains_maximum_bound():
  document = _document()
  document["profiles"]["relaxed"]["following"] = {"preset": "custom", "curve": [2.9] * 10}
  controller = StarPilotFollowing(_planner(weather_id=1, weather_increase=0.5))

  controller.update(True, 10.0, _sm(personality=Personality.relaxed), _toggles(document))

  assert controller.t_follow == pytest.approx(3.0)
