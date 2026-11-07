"""Every Galaxy slider shown in converted units (Galaxy speed / distance units) uses clean steps, and a value picked on
it saves and then shows exactly that value again. Runs every settings page for a Tesla Model 3 and a Toyota."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import cast
import math
import tempfile

import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.values import CAR as TESLA
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.params import Params
from openpilot.starpilot.galaxy import slider_units
from openpilot.starpilot.galaxy.settings import AuthorityContext, ContextSource, PAGES, SettingsChanged, SettingsGateway, SettingsUnavailable


def _gateway(params, platform, metric):
  from opendbc.car import gen_empty_fingerprint
  cp = interfaces[platform].get_params(platform, gen_empty_fingerprint(), [], False, False, False)
  cp.openpilotLongitudinalControl = True
  context = SimpleNamespace(value=AuthorityContext(True, cp, cp.to_bytes(), metric=metric))
  context.sample = lambda: context.value
  return SettingsGateway(params, cast(ContextSource, context), clock=lambda: 100.0)


def _converted(gateway, page_name, session, generation):
  """Rows Galaxy shows in other units than the setting's own (compared with the unconverted page)."""
  try:
    page = gateway.page(page_name, session, generation)
    native = {row.label: row.unit for row in gateway._base_state(page_name, gateway._context(page_name)).rows}
  except (SettingsUnavailable, SettingsChanged):
    return None, []
  rows = []
  for index, row in enumerate(page["rows"]):
    if (row["step"] and row["available"] and row["action"] and row["unit"] in slider_units.SI and
        native.get(row["label"], row["unit"]) != row["unit"]):
      rows.append((index, row))
  return page, rows


def _on_grid(value, step):
  return math.isclose(value / step, round(value / step), abs_tol=1e-6)


CASES = [(platform, metric, speed, distance) for platform in (TESLA.TESLA_MODEL_3, TOYOTA.TOYOTA_COROLLA_TSS2)
         for metric in (False, True) for speed, distance in ((b"1", b"1"), (b"2", b"2"))]


@pytest.mark.parametrize("platform,metric,speed,distance", CASES)
def test_converted_sliders_are_clean_and_round_trip(platform, metric, speed, distance):
  with tempfile.TemporaryDirectory() as folder:
    params = Params(folder)
    params.put_bool("IsMetric", metric, block=True)
    Path(params.get_param_path("GalaxySpeedUnits")).write_bytes(speed)
    Path(params.get_param_path("GalaxySmallDistanceUnits")).write_bytes(distance)
    gateway = _gateway(params, platform, metric)
    session, generation = "units-session", b"generation"
    checked = []
    try:
      for name in sorted(PAGES):
        page, rows = _converted(gateway, name, session, generation)
        for index, row in rows:
          native_unit_shown = row["unit"]
          step, low, high = float(row["step"]), float(row["minimum"]), float(row["maximum"])
          assert any(math.isclose(step, n) for n in slider_units.NICE_STEPS), (name, row["label"], step)
          assert _on_grid(low, step) and _on_grid(high, step), (name, row["label"], low, high, step)
          if row["value"] not in ("Auto",):
            assert _on_grid(float(row["value"]), step), (name, row["label"], row["value"], step)
          picks = sorted({round(low + step, 6), round((low + high) / 2 / step) * step, round(high - step, 6)} - {float(row["value"])})
          for pick in [p for p in picks if low <= p <= high][:2]:
            page, rows_now = _converted(gateway, name, session, generation)
            index_now = next(i for i, r in rows_now if r["label"] == row["label"])
            intent = gateway.preview(page["view"], index_now, 0, session, generation, value=round(pick, 6))["intent"]
            assert gateway.confirm(intent, session, generation), (name, row["label"], pick)
            again = gateway.page(name, session, generation)["rows"]
            shown = next(r for r in again if r["label"] == row["label"])
            assert math.isclose(float(shown["value"]), pick, abs_tol=1e-6), (name, row["label"], native_unit_shown, pick, shown["value"])
            checked.append((name, row["label"], native_unit_shown, pick))
    finally:
      gateway.close()
    print(f"\n{platform} metric={metric} speed={speed} distance={distance}: {len(checked)} saves checked")
    for item in checked:
      print("  ", item)
