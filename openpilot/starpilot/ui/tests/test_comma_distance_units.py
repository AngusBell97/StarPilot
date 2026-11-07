"""Comma distance units (Galaxy Language & Units): driving-screen distances can differ from the comma's speed units."""
from pathlib import Path

import pytest

from openpilot.common.params import Params
from openpilot.starpilot.ui.appearance_preferences import (LeadInfoMode, comma_distance_metric, distance_units, onroad_appearance)
from openpilot.starpilot.ui.navigation_state import distance_text


def _params(tmp_path, metric, comma_distance=None):
  params = Params(str(tmp_path))
  params.put_bool("IsMetric", metric, block=True)
  if comma_distance is not None:
    Path(params.get_param_path("GalaxyRoadDistanceUnits")).write_bytes(comma_distance)
  return params


@pytest.mark.parametrize("raw,expected", [(None, None), (b"0", None), (b"1", True), (b"2", False), (b"9", None)])
def test_comma_distance_choice(tmp_path, raw, expected):
  assert comma_distance_metric(_params(tmp_path, False, raw)) is expected


@pytest.mark.parametrize("metric,choice,expected", [(False, None, False), (True, None, True), (False, True, True), (True, False, False)])
def test_distance_units_fall_back_to_speed_units(metric, choice, expected):
  assert distance_units(metric, choice) is expected


def test_navigation_distance_text_uses_the_chosen_units():
  assert distance_text(1609.344 * 2, distance_units(False, True)) == "3.2 km"
  assert distance_text(3000.0, distance_units(True, False)) == "1.9 mi"


@pytest.mark.parametrize("mode,expected", [(LeadInfoMode.DISTANCE, True), (LeadInfoMode.SPEED, False)])
def test_lead_info_distance_follows_distance_units_and_speed_follows_speed_units(tmp_path, mode, expected, monkeypatch):
  params = _params(tmp_path, False, b"1")
  from dataclasses import replace
  import openpilot.starpilot.ui.appearance_preferences as ap
  saved = ap.read_lead_info(params)
  monkeypatch.setattr(ap, "read_lead_info", lambda _params: replace(saved, mode=mode))
  appearance = onroad_appearance(params)
  assert appearance.distance_metric is True
  assert appearance.lead_info_metric is expected


def test_default_follows_speed_units(tmp_path):
  appearance = onroad_appearance(_params(tmp_path, False))
  assert appearance.distance_metric is None
