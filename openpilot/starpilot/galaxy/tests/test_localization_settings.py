from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import cast

import pytest

from openpilot.common.params import Params
from openpilot.starpilot.galaxy.localization_settings import LocalizationSettingsOwner
from openpilot.starpilot.galaxy.settings import AuthorityContext, ContextSource, SettingsChanged, SettingsGateway
from openpilot.starpilot.speed_limits import offset_document as od
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
from openpilot.starpilot.ui.slc_offset_feature import DOCUMENT_KEY


def test_real_gateway_units_and_wheel_side_without_a_vehicle(tmp_path):
  params = Params(str(tmp_path))
  context = SimpleNamespace(value=AuthorityContext(True, None, None))
  context.sample = lambda: context.value
  gateway = SettingsGateway(params, cast(ContextSource, context))
  token, generation = "localization-session", b"generation"

  def preview(index, value):
    page = gateway.page("localization", token, generation)
    assert page["title"] == "Localization" and not page["vehicleIdentified"]
    assert [row["label"] for row in page["rows"]] == ["Comma speed units", "Comma distance units", "Galaxy speed units", "Galaxy distance units",
                                                      "Right-Hand Drive", "Language"]
    return gateway.preview(page["view"], index, 0, token, generation, value=value)["intent"]

  assert gateway.confirm(preview(2, "Metric"), token, generation)
  assert params.get("GalaxySpeedUnits") == 1 and not params.get_bool("IsMetric")
  assert gateway.confirm(preview(1, "Metric"), token, generation)
  assert params.get("GalaxyRoadDistanceUnits") == 1 and not params.get_bool("IsMetric"), "comma distance units are their own choice"
  assert gateway.confirm(preview(0, "Metric"), token, generation)
  assert params.get_bool("IsMetric"), "Comma speed units is the comma's own setting"
  assert not Path(params.get_param_path(DOCUMENT_KEY)).exists()
  assert gateway.confirm(preview(4, "On"), token, generation)
  assert params.get_bool("IsRhdDetected") and params.get_bool("OnroadCycleRequested")
  pending = preview(4, "Off")
  context.value = replace(context.value, parked=False)
  with pytest.raises(SettingsChanged):
    gateway.confirm(pending, token, generation)
  assert params.get_bool("IsRhdDetected")


def test_localization_preserves_current_offsets(tmp_path):
  params = Params(str(tmp_path))
  params.put_bool("IsMetric", False, block=True)
  document = od.OffsetDocument(od.IMPERIAL_BOUNDS, (5 * od.MPH_TO_MPS, 0, 0, 0, 0, 0, 0))
  params.put(DOCUMENT_KEY, od.to_value(document), block=True)
  params.put_bool("SpeedLimitController", False, block=True)
  owner = LocalizationSettingsOwner(params, lambda: True, lambda: True)
  assert owner.apply(FeatureSettingsRequest("IsMetric", b"0", "Metric"))
  assert od.decode(Path(params.get_param_path(DOCUMENT_KEY)).read_bytes()) == document
  assert owner.apply(FeatureSettingsRequest("IsMetric", b"1", "Imperial"))
  assert od.decode(Path(params.get_param_path(DOCUMENT_KEY)).read_bytes()) == document


@pytest.mark.parametrize("condition", ["onroad", "preview", "unauthorized", "stale", "invalid"])
def test_wheel_side_cannot_change_during_a_drive_or_stale_request(tmp_path, condition):
  params = Params(str(tmp_path))
  params.put_bool("IsRhdDetected", False, block=True)
  params.put_bool("IsDriverViewEnabled", condition == "preview", block=True)
  owner = LocalizationSettingsOwner(params, lambda: condition != "unauthorized", lambda: condition != "onroad")
  request = FeatureSettingsRequest("IsRhdDetected", b"1" if condition == "stale" else b"0",
                                   "invalid" if condition == "invalid" else "On")
  assert not owner.apply(request)
  assert not params.get_bool("IsRhdDetected")
  assert not params.get_bool("OnroadCycleRequested")


def test_settings_session_rechecked_before_localization_commit(tmp_path):
  params = Params(str(tmp_path))
  context = SimpleNamespace(sample=lambda: AuthorityContext(True, None, None))
  gateway = SettingsGateway(params, cast(ContextSource, context))
  page = gateway.page("localization", "session", b"generation")
  pending = gateway.preview(page["view"], 4, 0, "session", b"generation", value="On")["intent"]
  with pytest.raises(SettingsChanged):
    gateway.confirm(pending, "session", b"generation", session_valid=lambda: False)
  assert not params.get_bool("IsRhdDetected")


def test_units_without_offsets_work_onroad_but_wheel_side_waits(tmp_path):
  params = Params(str(tmp_path))
  owner = LocalizationSettingsOwner(params, lambda: True, lambda: False)
  rows = owner.snapshot().rows
  assert rows[0].available and rows[0].key == "IsMetric" and not rows[4].available
  assert owner.apply(FeatureSettingsRequest("IsMetric", None, "Metric"))
  assert params.get_bool("IsMetric")
  assert not Path(params.get_param_path(DOCUMENT_KEY)).exists()


def test_galaxy_units_do_not_require_legacy_offset_migration(tmp_path):
  params = Params(str(tmp_path))
  Path(params.get_param_path("Offset1")).write_bytes(b"5.0")
  owner = LocalizationSettingsOwner(params, lambda: True, lambda: False)
  row = owner.snapshot().rows[2]
  assert row.available and row.key == "GalaxySpeedUnits"
  assert owner.apply(FeatureSettingsRequest("GalaxySpeedUnits", None, "Metric"))
  assert not owner.apply(FeatureSettingsRequest("IsMetric", None, "On")), "the old On/Off values are not accepted"
  assert not params.get_bool("IsMetric")


def test_comma_and_galaxy_units_rows(tmp_path):
  params = Params(str(tmp_path))
  params.put_bool("IsMetric", False, block=True)
  owner = LocalizationSettingsOwner(params, lambda: True, lambda: True)
  rows = owner.snapshot().rows
  assert [(r.key, r.label, r.value, r.choices, r.default_value) for r in rows[:4]] == [
    ("IsMetric", "Comma speed units", "Imperial", ("Metric", "Imperial"), "Imperial"),
    ("GalaxyRoadDistanceUnits", "Comma distance units", "Same as speed", ("Same as speed", "Metric", "Imperial"), "Same as speed"),
    ("GalaxySpeedUnits", "Galaxy speed units", "Same as comma", ("Same as comma", "Metric", "Imperial"), "Same as comma"),
    ("GalaxySmallDistanceUnits", "Galaxy distance units", "Same as comma", ("Same as comma", "Metric", "Imperial"), "Same as comma")]
  assert owner.apply(FeatureSettingsRequest("GalaxySmallDistanceUnits", None, "Imperial"))
  assert params.get("GalaxySmallDistanceUnits") == 2


def _width_row(metric):
  from openpilot.starpilot.ui.feature_settings_state import FeatureRow
  return FeatureRow("lane_change:width", "Minimum adjacent lane width", "5", b"x", step=0.1, minimum=0.0, maximum=15.0, unit="ft",
                    available=True, default_value="0")


@pytest.mark.parametrize("comma_metric,comma_distance,galaxy_distance,unit", [
  (False, None, None, "ft"),     # everything follows the comma's (imperial) speed units
  (True, None, None, "m"),       # metric comma: distances follow
  (False, b"1", None, "m"),      # comma distance metric, Galaxy follows the comma
  (True, b"2", None, "ft"),      # comma distance imperial on a metric comma
  (False, None, b"1", "m"),      # Galaxy's own choice wins
  (True, None, b"2", "ft"),
])
def test_galaxy_distance_same_as_comma_follows_comma_distance(tmp_path, comma_metric, comma_distance, galaxy_distance, unit):
  from openpilot.starpilot.galaxy import slider_units
  params = Params(str(tmp_path))
  params.put_bool("IsMetric", comma_metric, block=True)
  if comma_distance is not None:
    Path(params.get_param_path("GalaxyRoadDistanceUnits")).write_bytes(comma_distance)
  if galaxy_distance is not None:
    Path(params.get_param_path("GalaxySmallDistanceUnits")).write_bytes(galaxy_distance)
  shown = slider_units.project(params, _width_row(comma_metric))
  assert shown.unit == unit
  assert float(shown.value) == pytest.approx(5 if unit == "ft" else 1.524)
  if galaxy_distance is None:
    assert ("GalaxyRoadDistanceUnits", comma_distance) in shown.dependencies, "a comma units change refreshes the view"
