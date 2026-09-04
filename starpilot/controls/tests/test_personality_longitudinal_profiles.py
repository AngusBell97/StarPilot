import sys
from types import ModuleType, SimpleNamespace

from openpilot.starpilot.common.accel_profile import (
  ACCELERATION_PROFILES,
  DECELERATION_PROFILES,
)
from openpilot.starpilot.common.longitudinal_personality_profiles import (
  default_personality_profiles,
)

_MISSING_MODULE = object()
_STUBBED_MODULE_NAMES = (
  "openpilot.common.constants",
  "openpilot.common.params",
  "openpilot.selfdrive.car.cruise",
  "openpilot.selfdrive.controls.lib.longitudinal_planner",
  "openpilot.starpilot.controls.lib.starpilot_vcruise",
  "cereal",
  "cereal.log",
  "openpilot.common.realtime",
  "openpilot.selfdrive.controls.lib.lead_behavior",
  "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc",
  "openpilot.starpilot.common.starpilot_variables",
  "openpilot.starpilot.controls.lib.starpilot_acceleration",
  "openpilot.starpilot.controls.lib.starpilot_following",
)
_ORIGINAL_MODULES = {name: sys.modules.get(name, _MISSING_MODULE) for name in _STUBBED_MODULE_NAMES}


def _module(name, **attributes):
  module = ModuleType(name)
  for key, value in attributes.items():
    setattr(module, key, value)
  return module


class _Params:
  def __init__(self, *args, **kwargs):
    self.writes = []

  def put_nonblocking(self, key, value):
    self.writes.append((key, value))

  def put_bool(self, key, value):
    self.writes.append((key, value))


sys.modules["openpilot.common.constants"] = _module(
  "openpilot.common.constants",
  CV=SimpleNamespace(KPH_TO_MS=1 / 3.6, MPH_TO_MS=0.44704),
)
sys.modules["openpilot.common.params"] = _module("openpilot.common.params", Params=_Params)
sys.modules["openpilot.selfdrive.car.cruise"] = _module(
  "openpilot.selfdrive.car.cruise", V_CRUISE_MAX=145, V_CRUISE_UNSET=255,
)
sys.modules["openpilot.selfdrive.controls.lib.longitudinal_planner"] = _module(
  "openpilot.selfdrive.controls.lib.longitudinal_planner", A_CRUISE_MIN=-1.0, get_max_accel=lambda _v_ego: 2.0,
)
sys.modules["openpilot.starpilot.controls.lib.starpilot_vcruise"] = _module(
  "openpilot.starpilot.controls.lib.starpilot_vcruise",
  get_active_slc_control_target=lambda enabled, set_speed_limit, target, offset, overridden_speed, *_args, **_kwargs: (
    float(overridden_speed or target) + float(offset) if enabled and set_speed_limit else 0.0
  ),
)

from openpilot.starpilot.controls.lib.starpilot_acceleration import (
  StarPilotAcceleration,
)

_lane_change_state = SimpleNamespace(preLaneChange=1, laneChangeStarting=2, laneChangeFinishing=3)
_lane_change_direction = SimpleNamespace(left=1, right=2)
sys.modules["cereal"] = _module("cereal", log=SimpleNamespace(LaneChangeState=_lane_change_state, LaneChangeDirection=_lane_change_direction))
sys.modules["cereal.log"] = _module("cereal.log", LaneChangeState=_lane_change_state, LaneChangeDirection=_lane_change_direction)
sys.modules["openpilot.common.realtime"] = _module("openpilot.common.realtime", DT_MDL=0.05)
sys.modules["openpilot.selfdrive.controls.lib.lead_behavior"] = _module(
  "openpilot.selfdrive.controls.lib.lead_behavior", should_disable_far_lead_throttle=lambda *args: False,
)


def _strict_legacy_personality(*args):
  if args[-1] not in (0, 1, 2):
    raise NotImplementedError("Longitudinal personality not supported")


def _strict_get_jerk_factor(*args):
  _strict_legacy_personality(*args)
  return 1.0, 1.0, 1.0


def _strict_get_t_follow(*args):
  _strict_legacy_personality(*args)
  return 1.45


sys.modules["openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc"] = _module(
  "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc",
  COMFORT_BRAKE=2.5,
  LEAD_DANGER_FACTOR=0.75,
  desired_follow_distance=lambda v_ego, _v_lead, t_follow: v_ego * t_follow,
  get_jerk_factor=_strict_get_jerk_factor,
  get_T_FOLLOW=_strict_get_t_follow,
)
sys.modules["openpilot.starpilot.common.starpilot_variables"] = _module(
  "openpilot.starpilot.common.starpilot_variables", CITY_SPEED_LIMIT=15.0, MAX_T_FOLLOW=3.0,
)

from openpilot.starpilot.controls.lib.starpilot_following import StarPilotFollowing

for _module_name, _original_module in _ORIGINAL_MODULES.items():
  if _original_module is _MISSING_MODULE:
    sys.modules.pop(_module_name, None)
  else:
    sys.modules[_module_name] = _original_module


def test_stubbed_dependency_modules_are_restored_after_collection():
  from openpilot.common.params import ParamKeyType

  assert ParamKeyType.JSON.name == "JSON"


def _sm(*, traffic=False, personality=1):
  inactive_lead = SimpleNamespace(status=False, vLead=0.0, aLeadK=0.0, dRel=1000.0)
  return {
    "carControl": SimpleNamespace(orientationNED=[0.0, 0.0, 0.0]),
    "carState": SimpleNamespace(
      aEgo=0.0,
      leftBlindspot=False,
      rightBlindspot=False,
      vCruise=50.0,
      vEgoCluster=0.0,
      standstill=False,
    ),
    "controlsState": SimpleNamespace(forceDecel=False),
    "modelV2": SimpleNamespace(meta=SimpleNamespace(laneChangeState=0, laneChangeDirection=0)),
    "radarState": SimpleNamespace(leadOne=inactive_lead, leadTwo=inactive_lead),
    "selfdriveState": SimpleNamespace(personality=personality),
    "starpilotCarState": SimpleNamespace(
      ecoGear=False,
      forceCoast=False,
      pulseAndGlide=False,
      sportGear=False,
      trafficModeEnabled=traffic,
    ),
  }


def _planner():
  return SimpleNamespace(
    lead_one=SimpleNamespace(status=False),
    starpilot_cem=SimpleNamespace(stop_light_detected=False),
    starpilot_following=SimpleNamespace(disable_throttle=False),
    starpilot_vcruise=SimpleNamespace(slc_target=0.0, slc_offset=0.0, slc=SimpleNamespace(overridden_speed=0.0), forcing_stop=False),
    starpilot_weather=SimpleNamespace(weather_id=0, reduce_acceleration=0.0, increase_following_distance=0.0),
    tracking_lead=False,
    v_cruise=20.0,
  )


def _toggles(profiles):
  toggles = SimpleNamespace(
    acceleration_profile=ACCELERATION_PROFILES["STANDARD"],
    conditional_slower_lead=False,
    custom_accel_profile=False,
    custom_accel_profile_values=[],
    custom_personalities=True,
    deceleration_profile=DECELERATION_PROFILES["STANDARD"],
    ev_tuning=False,
    lane_change_close_gap=False,
    lane_change_close_gap_seconds=0.75,
    longitudinal_personality_profiles=profiles,
    map_acceleration=False,
    map_deceleration=False,
    minimum_lane_change_speed=5.0,
    pulse_glide_speed_delta=0.0,
    redneck_cruise=False,
    set_speed_limit=False,
    set_speed_offset=0.0,
    speed_limit_controller=False,
    speed_limit_controller_override_set_speed=False,
    truck_tuning=False,
  )
  for personality in ("aggressive", "standard", "relaxed"):
    setattr(toggles, f"{personality}_follow", [1.45, 1.2])
    for suffix in ("acceleration", "deceleration", "danger", "speed", "speed_decrease"):
      setattr(toggles, f"{personality}_jerk_{suffix}", 1.0)
  toggles.traffic_mode_follow = [0.75, 1.45]
  toggles.traffic_mode_jerk_acceleration = [1.0, 1.0]
  toggles.traffic_mode_jerk_deceleration = [1.0, 1.0]
  toggles.traffic_mode_jerk_danger = [1.0, 1.0]
  toggles.traffic_mode_jerk_speed = [1.0, 1.0]
  toggles.traffic_mode_jerk_speed_decrease = [1.0, 1.0]
  return toggles


def test_traffic_mode_uses_its_personality_acceleration_and_braking_curves():
  profiles = default_personality_profiles(False)
  profiles["traffic"]["acceleration"] = {"preset": "custom", "curve": [2.0] * 7}
  profiles["traffic"]["braking"] = {"preset": "custom", "curve": [0.8] * 7}
  controller = StarPilotAcceleration(_planner())

  controller.update(0.0, _sm(traffic=True), _toggles(profiles))

  assert controller.max_accel == 2.0
  assert controller.min_accel == -0.8


def test_following_uses_the_active_personality_curve_at_every_ten_mph():
  profiles = default_personality_profiles(False)
  profiles["standard"]["following"] = {
    "preset": "custom",
    "curve": [1.0 + index * 0.2 for index in range(10)],
  }
  controller = StarPilotFollowing(_planner())

  controller.update(True, 5.0 * 0.44704, _sm(personality=1), _toggles(profiles))

  assert controller.t_follow == 1.1


def test_drive_mode_mapping_overrides_profiles_without_reviving_hidden_legacy_custom_curve():
  profiles = default_personality_profiles(False)
  profiles["standard"]["acceleration"] = {"preset": "custom", "curve": [2.0] * 7}
  toggles = _toggles(profiles)
  toggles.map_acceleration = True
  toggles.custom_accel_profile = True
  toggles.custom_accel_profile_values = [5.0] * 7
  sm = _sm(traffic=False, personality=1)
  sm["starpilotCarState"].ecoGear = True
  controller = StarPilotAcceleration(_planner())

  controller.update(0.0, sm, toggles)

  assert controller.max_accel == 1.5


def test_drive_mode_mapping_overrides_configured_traffic_profile():
  profiles = default_personality_profiles(False)
  profiles["traffic"]["acceleration"] = {"preset": "custom", "curve": [2.8] * 7}
  profiles["traffic"]["braking"] = {"preset": "custom", "curve": [0.8] * 7}
  cases = (
    (True, False, 1.5, -0.5),
    (False, False, 2.0, -1.0),
    (False, True, 3.5, -2.0),
  )

  for eco_gear, sport_gear, expected_max, expected_min in cases:
    toggles = _toggles(profiles)
    toggles.map_acceleration = True
    toggles.map_deceleration = True
    sm = _sm(traffic=True)
    sm["starpilotCarState"].ecoGear = eco_gear
    sm["starpilotCarState"].sportGear = sport_gear
    controller = StarPilotAcceleration(_planner())

    controller.update(0.0, sm, toggles)

    assert controller.max_accel == expected_max
    assert controller.min_accel == expected_min


def test_malformed_in_memory_profiles_fall_back_to_legacy_controllers():
  malformed = default_personality_profiles(False)
  malformed["standard"]["acceleration"]["curve"] = [float("nan")] * 7
  malformed["standard"]["following"]["curve"] = [0.1] * 10

  acceleration = StarPilotAcceleration(_planner())
  acceleration.update(10.0, _sm(personality=1), _toggles(malformed))
  assert acceleration.max_accel == 1.55
  assert acceleration.min_accel == -1.0

  following = StarPilotFollowing(_planner())
  following.update(True, 10.0, _sm(personality=1), _toggles(malformed))
  assert following.t_follow == 1.45


def test_invalid_personality_falls_back_to_standard_legacy_following():
  following = StarPilotFollowing(_planner())

  following.update(True, 10.0, _sm(personality=99), _toggles({}))

  assert following.t_follow == 1.45


def test_personality_braking_curve_keeps_speed_limit_coast_shaping():
  profiles = default_personality_profiles(False)
  profiles["standard"]["braking"] = {"preset": "custom", "curve": [2.0] * 7}
  planner = _planner()
  planner.v_cruise = 30.0
  planner.starpilot_vcruise.slc_target = 19.5
  sm = _sm(traffic=False, personality=1)
  sm["carState"].vCruise = 120.0
  sm["carState"].vEgoCluster = 20.0
  toggles = _toggles(profiles)
  toggles.speed_limit_controller = True
  toggles.set_speed_limit = True

  controller = StarPilotAcceleration(planner)
  controller.update(20.0, sm, toggles)

  assert -2.0 < controller.min_accel < 0.0
