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


USE_HELP = ("Chooses what Conditional Chill uses Chill for. Climbing to set speed keeps Experimental driving and uses "
            "Chill only to bring the car up to your set speed. Classic conditions uses Chill above a speed and behind "
            "steady leads.")
MANUAL_HELP = "Keeps your manual driving-mode choice across drives. Changing this resets that choice to Automatic."
LAUNCH_HELP = ("Uses Chill to pull away from a stop when the driving model and the lead show it is clear to go. "
               "Off keeps Experimental for stops and departures.")

# (field, label, help) in display order.
CLIMB_LAYOUT = (
  (USE, "Use Chill for", USE_HELP),
  ("speed_mps", "Climb from",
   "Climbs only start at or above this speed. Below it, Experimental always drives."),
  ("set_speed_margin_mps", "Climb when below set speed by",
   "A climb starts once the car is this far below the set speed (as the dash shows it) on a clear road, while the "
   "driving model is not slowing. It hands back to Experimental at the set speed, or as soon as a lead appears, a "
   "bend starts or the model slows."),
  (AFTER_BEND, "After a bend",
   "Experimental slows for bends. Choose when Chill may bring the speed back: 2 seconds after the bend, as soon as "
   "the bend ends, or stay in Chill through bends. Chill itself only slows for bends when Curve Speed Control is on."),
  ("climb_lead", "Keep up with a lead pulling away",
   "Also climbs when a lead far ahead (3 seconds or more) keeps pulling away and Experimental is not keeping up. "
   "Hands back once the gap closes to 2.5 seconds or the lead stops pulling away."),
  ("launch_assist", "Launch assist from a stop", LAUNCH_HELP),
  ("lead", "Also use Chill behind a steady lead",
   "Adds Chill while following a steady lead, outside climbs. Off keeps Experimental whenever you follow a lead."),
  ("speed_with_lead_mps", "Chill behind a lead above",
   "Minimum speed for Chill behind a steady lead. 0 allows it at any speed."),
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
