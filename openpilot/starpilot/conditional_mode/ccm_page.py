"""Conditional Chill settings page: one source for its layout, labels, help and summary (7 Nov 2026).

The comma's settings screen and Galaxy both read this, so each row has exactly one description, written for
the layout it appears in. The page changes shape with "Use Chill for": rows that cannot apply are hidden, and
a row that depends on another is shown greyed with a one-line reason. Every option that affects driving in
the current layout is visible.

Two rows are choices over existing saved fields, written together in one validated document:
  use         -> ccm.climb
  after_bend  -> ccm.allow_chill_in_bends, ccm.climb_on_bend_exit
"""
from __future__ import annotations

USE = "use"
AFTER_BEND = "after_bend"
CHOICE_FIELDS = frozenset((USE, AFTER_BEND))

USE_CLIMB = "Climbing to set speed"
USE_CLASSIC = "Classic conditions"
USE_CHOICES = (USE_CLIMB, USE_CLASSIC)

BEND_WAIT = "Wait 2 s, then climb"
BEND_EXIT = "Climb as the bend ends"
BEND_THROUGH = "Stay in Chill through bends"
BEND_CHOICES = (BEND_WAIT, BEND_EXIT, BEND_THROUGH)


def bend_choice(allow_chill_in_bends: bool, climb_on_bend_exit: bool) -> str:
  return BEND_THROUGH if allow_chill_in_bends else BEND_EXIT if climb_on_bend_exit else BEND_WAIT


def bend_fields(choice: str) -> dict[str, bool]:
  """Saved fields for an "After a bend" choice; KeyError for anything else."""
  allow, on_exit = {BEND_WAIT: (False, False), BEND_EXIT: (False, True), BEND_THROUGH: (True, False)}[choice]
  return {"allow_chill_in_bends": allow, "climb_on_bend_exit": on_exit}


USE_HELP = ('Climbing to set speed: Experimental drives, and Chill only brings you up to set speed. Classic: ' +
              'Chill above a speed and behind steady leads.')
MANUAL_HELP = 'Keeps your manual mode choice across drives.'
LAUNCH_HELP = ("Uses Chill to pull away from a stop when it's clear. Off: Experimental handles stops.")

# (field, label, help) in display order.
CLIMB_LAYOUT = (
  (USE, "Use Chill for", USE_HELP),
  ("speed_mps", "Climb from",
   "Climbs only start at or above this speed. Below it, Experimental always drives."),
  ("set_speed_margin_mps", "Climb when below set speed by",
   ('Starts a climb once this far below set speed on a clear road. Hands back at set speed, or for a ' +
     'lead, bend or slowdown.')),
  (AFTER_BEND, "After a bend",
   ('When Chill may climb again after Experimental slows for a bend: 2 s after, at the end of the ' +
     'bend, or straight through.')),
  ("climb_lead", "Keep up with a lead pulling away",
   'Also climbs when a distant car ahead keeps pulling away.'),
  ("launch_assist", "Launch assist from a stop", LAUNCH_HELP),
  ("lead", "Also use Chill behind a steady lead",
   'Also uses Chill while following a steady car.'),
  ("speed_with_lead_mps", "Chill behind a lead above",
   'Lowest speed for Chill behind a steady car. 0 means any speed.'),
  ("persist_manual", "Remember manual choice", MANUAL_HELP),
)
CLASSIC_LAYOUT = (
  (USE, "Use Chill for", USE_HELP),
  ("speed_mps", "Chill above: no lead",
   "Chill at or above this speed with no lead, while the car is below the set speed by the margin. 0 means any speed."),
  ("speed_with_lead_mps", "Chill above: with lead",
   "Chill at or above this speed behind a steady lead, when Lead vehicle is on. 0 means any speed."),
  ("set_speed_margin_mps", "Set speed margin",
   "How far below the set speed the car must be before the no-lead Chill condition applies. 0 allows it right up "
   "to the set speed."),
  ("lead", "Lead vehicle",
   "Allows Chill behind a steady lead above the with-lead speed. Slower or braking leads, signals and stops still "
   "bring Experimental back."),
  ("launch_assist", "Launch assist", LAUNCH_HELP),
  ("allow_chill_in_bends", "Allow Chill in bends",
   "Lets Chill continue through bends instead of returning to Experimental until the bend is clear. CSC still limits "
   "corner speed when Curve Speed Control is on."),
  ("persist_manual", "Remember manual choice", MANUAL_HELP),
)

# A row that only works with another row on: field -> (needed field, reason shown while it is off), per layout.
DEPENDS = {
  True: {"speed_with_lead_mps": ("lead", 'Turn on "Also use Chill behind a steady lead" to use this.')},
  False: {"speed_with_lead_mps": ("lead", 'Turn on "Lead vehicle" to use this.')},
}

HELP_BY_LABEL = {label: text for layout in (CLIMB_LAYOUT, CLASSIC_LAYOUT) for _, label, text in layout}


def layout(climb: bool):
  return CLIMB_LAYOUT if climb else CLASSIC_LAYOUT


def summary(ccm, speed_text: str) -> str:
  """One-line description of what Conditional Chill will do, for the Chill conditions link."""
  if not ccm.climb:
    return "Classic conditions"
  parts = [f"Climb to set speed from {speed_text}" if ccm.speed_mps > 0 else "Climb to set speed"]
  if ccm.climb_lead:
    parts.append("keeps up with leads")
  if ccm.lead:
    parts.append("Chill behind leads")
  if ccm.launch_assist:
    parts.append("launch assist")
  return " · ".join(parts)
