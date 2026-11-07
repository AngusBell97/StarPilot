"""SLC presentation groups shared by native controls and Galaxy.

Only navigation changes here. Rows retain the original saved sources, capability,
compare-and-save dependencies, edit permissions and confirmation requirements.
"""
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

SECTIONS = (
  ("slc/offsets", "Speed-limit offsets", "Offsets for each speed range", frozenset(("slc_reset", *(f"slc:offset:{i}" for i in range(1, 8))))),
  ("slc/sources", "Limit sources", 'Where speed limits come from', frozenset(("SLCPriority1", "SLCPriority2"))),
  ("slc/timing", "Timing and confirmation", 'When to slow down or speed up for a new limit', frozenset(("SLCWarningLeadTime", "SLCLowerLimitTiming", "SLCHigherLimitDistance", "SLCConfirmationLower", "SLCConfirmationHigher"))),
  ("slc/reengagement", "Manual speed and re-engagement", 'Remember and restore your own speed for each limit', frozenset(("SLCRememberManualOffset", "SLCReengageSpeed"))),
)
SLC_PAGES = tuple(section[0] for section in SECTIONS)


def menu_rows(page, rows):
  if page == "slc":
    # Keep unknown future settings visible until explicitly assigned to a group.
    grouped = frozenset().union(*(section[3] for section in SECTIONS))
    root = [row for row in rows if row.key and row.key not in grouped]
    root.extend(FeatureRow("", title, description, page=child, available=True)
                for child, title, description, _ in SECTIONS)
    return "Speed Limit Controller", root
  for child, title, _, keys in SECTIONS:
    if page == child:
      return title, [row for row in rows if row.key in keys or (child == "slc/offsets" and not row.key)]
  raise ValueError("Unknown SLC submenu")
