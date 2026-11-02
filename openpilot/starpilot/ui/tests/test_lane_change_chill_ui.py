"""The close-gap lane-change Chill reason renders as Chill everywhere the screen shows the mode."""

from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pyray as rl

from openpilot.starpilot.conditional_mode.policy import ModeChoice, Reason
from openpilot.starpilot.ui.onroad import axis_status_color, driving_mode_description
from openpilot.starpilot.ui.onroad_compact_widgets import MiciSidebarWidgets
from openpilot.starpilot.ui.onroad_conditional import status, stop_active
from openpilot.starpilot.ui.onroad_state import OnroadState, SpeedLimitObservation

GREEN = (22, 127, 64)
ORANGE = (218, 111, 37)


def road(**changes):
  state = OnroadState(True, True, 15.0, 80.0, SpeedLimitObservation(), lateral_active=True, longitudinal_active=True)
  return replace(state, **changes)


def accepted(reason, experimental):
  return NS(choice=ModeChoice.CEM, effective_experimental=experimental, reason=reason, status_code=0)


def test_lane_change_reason_colours_the_edge_green_like_chill():
  lead = road(conditional_effective=accepted(Reason.CEM_LEAD.value, True))
  chill = road(conditional_effective=accepted(Reason.CEM_LANE_CHANGE.value, False))
  assert (axis_status_color(lead).r, axis_status_color(lead).g, axis_status_color(lead).b) == ORANGE
  assert (axis_status_color(chill).r, axis_status_color(chill).g, axis_status_color(chill).b) == GREEN
  assert driving_mode_description(lead) == "ACC - Experimental"
  assert driving_mode_description(chill) == "ACC - Chill"
  assert not stop_active(chill)


def test_lane_change_reason_has_a_label_in_the_chill_colour():
  value = status(road(conditional_effective=accepted(Reason.CEM_LANE_CHANGE.value, False)))
  assert value is not None
  family, detail, colour = value
  assert (family, detail) == ('CEM', 'LANE CHANGE')
  assert (colour.r, colour.g, colour.b) == (255, 155, 63)


def test_compact_widget_uses_the_chill_icon_for_the_lane_change_reason():
  fonts = Mock()
  fonts.measure.return_value = NS(width=20, height=20)
  widget = MiciSidebarWidgets(fonts)
  methods = ('_lead_icon', '_stop_icon', '_curve_icon', '_turn_icon', '_speed_icon', '_chill_icon', '_active_icon')
  with patch.object(rl, 'draw_rectangle'), patch.object(widget, '_confidence_ball'), patch.object(widget, '_personality'):
    patches = [patch.object(widget, method) for method in methods]
    mocks = [item.__enter__() for item in patches]
    try:
      widget.render(rl.Rectangle(0, 0, 536, 240),
                    road(conditional_effective=accepted(Reason.CEM_LANE_CHANGE.value, False)))
      mocks[methods.index('_chill_icon')].assert_called_once()
      assert sum(mock.call_count for mock in mocks) == 1
    finally:
      for item in reversed(patches):
        item.__exit__(None, None, None)
