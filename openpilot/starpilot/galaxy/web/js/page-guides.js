// Page guides: group a settings page's rows into titled cards and explain each setting in plain words.
// The rows and saving come from the settings API unchanged; this file only decides which card a row sits in,
// its text, how a choice is worded, and which rows are greyed (their switch is off) or hidden (not in use).
// Rows are matched by label. A row a guide does not list still shows, in a "More settings" card at the end.

const LANE_TURN_SPEED = row => (row.unit === "km/h" ? "32 km/h" : "20 mph")
const ALLOW = { label: "Allow lane changes", value: "On" }
// Wheel nudge required is the device's Automatic Lane Changes switch the other way round (nudge required = automatic off).
const NUDGELESS = { label: "Automatic Lane Changes", value: "On", note: "Turn off Wheel nudge required to use this." }
const CLOSE_GAP = { label: "Close lane-change gap", value: "On" }
const SLC = { label: "Speed Limit Controller", value: "On" }

// Row options: help (text or row => text; none keeps the device's text), display (label shown instead of the device's),
// values (device value → shown value), needs (rows that must have a value, else greyed, with the need's note or
// "Turn on <row>"), showWhen (same, else hidden), hide (never shown here), sub (indented under the row above), placeholder (greyed stand-in while
// the device leaves the row out because its switch is off). A card with no title has no header. A card's `drops` are
// dropdown sections that each show listed rows of another page, while its `dropsWhen` row has that value.
// Driving Personalities (personalities, aggressive, standard, relaxed).
const PROFILES = ["aggressive", "standard", "relaxed"]
const TITLE = name => name[0].toUpperCase() + name.slice(1)
const SPEEDS = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90]
const JERKS = ["Acceleration jerk", "Deceleration jerk", "Speed jerk", "Speed decrease jerk", "Danger jerk"]
// A personality's following and smoothness rows, in the order its dropdown shows them.
const FOLLOWING_ROWS = ["Following preset", "Low-speed follow", "High-speed follow", ...SPEEDS.map(speed => `Following at ${speed} mph`),
  ...JERKS, "Reset Profile to Default"]

export const PAGE_GUIDES = Object.freeze({
  lane_change: {
    intro: "Signal, and StarPilot steers you into the lane on that side without disengaging.",
    cards: [
      {
        id: "lane-changes", title: "",
        rows: [
          { label: "Allow lane changes", display: "Auto lane changes",
            help: "Main switch. On: signalling can start a lane change. Off: it never does, and the settings below do nothing." },
          { label: "One change per signal", needs: [ALLOW],
            help: "On: one lane change per signal; signal again for the next lane. Off: keeps moving over while you keep signalling." },
          { label: "Lane Change Speed", needs: [ALLOW],
            help: "How quickly the car moves across. 10 is the quickest (about 3 s); 5 takes about 6 s, 1 about 8 s." },
          { label: "Automatic Lane Changes", display: "Wheel nudge required", values: { Off: "On", On: "Off" }, needs: [ALLOW],
            help: "On: nudge the wheel toward the new lane to start. Off: it starts by itself after the Nudgeless delay. " +
              "Never starts with a car in the blind spot." },
          { label: "Automatic lane-change delay", display: "Nudgeless delay", needs: [ALLOW, NUDGELESS], sub: true,
            help: "How long after you signal before it starts, so you can cancel. 0 starts straight away." },
          { label: "Minimum adjacent lane width", needs: [ALLOW, NUDGELESS], sub: true,
            help: "Only starts if the next lane is at least this wide; otherwise nudge the wheel. 0 skips the check." },
          { label: "Minimum lane-change speed", needs: [ALLOW],
            help: row => "Lane changes only start above this speed (from " + LANE_TURN_SPEED(row) + " while Turn Signal Guidance is on)." },
          { label: "Close lane-change gap", needs: [ALLOW],
            help: "When you signal to overtake, the car stops dropping back and can speed up as it moves across. " +
              "Still slows for close or braking cars. Needs StarPilot speed control." },
          { label: "Lane-change follow gap", needs: [ALLOW, CLOSE_GAP], sub: true,
            help: "Closest following time while you wait to pull out. 0.75 s is closest; 1.0 s leaves more room." },
          { label: "Faster lane change when close", needs: [ALLOW, CLOSE_GAP], sub: true, placeholder: { value: "Off", choices: ["Off", "On"] },
            help: "Uses the quickest steering when you pull out while closing on the car ahead." },
        ],
        // Turn Signal Guidance: saved with the Steering and Torque page's settings, shown at the end of this list.
        include: { page: "torque", rows: ["Turn Signal Guidance"] },
      },
    ],
    // Conditional Experimental's signal rows: served with this page but shown under Experimental conditions (menu.js
    // PAGE_COMPANIONS); the device's own text, the width greyed while detection is off.
    rows: [
      { label: "Signal lane detection" },
      { label: "Signal lane width", needs: [{ label: "Signal lane detection", value: "On" }] },
    ],
  },
  // Visuals: the comma screen options, grouped as the layout editor grouped them. Device text, saved straight away.
  appearance: {
    intro: "Options for the comma's driving screen. Widget positions and colours are on Colors & Layout.",
    cards: [
      // One card, no sub-headings (the user's choice, 7 Nov); rows keep the layout editor's order.
      { id: "appearance", title: "", rows: ["Developer Sidebar", "Use StarPilot Widgets", "Wheel brake and acceleration colors",
        "Lead Indicator", "Lead Info", "Camera View", "Driver Camera on Reverse", "Hide Personality Alerts", "C4 amber signal border",
        "C4 red blind-spot border"].map(label => ({ label })) },
    ],
  },
  // Speed Limit Controller: what only works while the controller is on is greyed while it is off. Show speed limit signs
  // and Limit sources stay active (the signs use the sources). The page keeps its own layout and text.
  slc: {
    rows: [
      { label: "Speed-limit offsets", needs: [SLC] },
      { label: "Timing and confirmation", needs: [SLC] },
      { label: "Manual speed and re-engagement", needs: [SLC] },
    ],
  },
  "slc/reengagement": {
    rows: [
      // The memory is only used when restoring the SLC speed on re-engagement (both must be on).
      { label: "Remember manual speed for each limit", needs: [{ label: "Restore SLC speed on re-engagement", value: "On" }] },
    ],
  },
  // Turn Signal Guidance is saved with Steering and Torque but shown on Lane Changes (above); only its text changes here.
  torque: {
    rows: [
      { label: "Turn Signal Guidance",
        help: "On: below 20 mph (32 km/h), your signal tells the model which way you're turning so it follows that path. " +
          "Lane changes then start from 20 mph." },
    ],
  },
  personalities: {
    intro: "One personality drives at a time; switch on the comma or the wheel. Each has its own following, smoothness, " +
      "acceleration and braking.",
    cards: [
      {
        id: "custom-following", title: "",
        rows: [
          { label: "Custom Following and Jerk",
            help: "On: Aggressive, Standard and Relaxed use your own following distance and smoothness, set below. " +
              "Off: StarPilot's defaults." },
          { label: "Restore saved profiles switch to Off" },
          { label: "Invalid profile document" },
        ],
        dropsWhen: { label: "Custom Following and Jerk", value: "On" },
        drops: PROFILES.map(name => ({ id: name, title: TITLE(name), page: name, hint: "Following distance and smoothness",
          rows: FOLLOWING_ROWS })),
      },
      {
        id: "personalities", title: "Acceleration and braking", icon: "bi-speedometer2",
        rows: [...PROFILES, "traffic"].map(name => ({ label: `${TITLE(name)} Personality`, display: TITLE(name) })),
      },
    ],
  },
  ...Object.fromEntries(PROFILES.map(name => [name, personalityGuide(name)])),
})

// Aggressive, Standard and Relaxed share one layout. A category's speed points exist only while its preset is Custom.
function personalityGuide(name) {
  const points = (category, extra = {}) => SPEEDS.map(speed => ({ label: `${category} at ${speed} mph`, sub: true, ...extra }))
  return {
    intro: `How ${TITLE(name)} speeds up and brakes. Following and smoothness are on Driving Personalities.`,
    cards: [
      {
        id: "acceleration", title: "Acceleration", icon: "bi-speedometer2",
        rows: [{ label: "Acceleration preset", display: "Acceleration" }, { label: "SLC acceleration limit", sub: true }, ...points("Acceleration")],
      },
      {
        id: "braking", title: "Braking", icon: "bi-sign-stop",
        rows: [{ label: "Braking preset", display: "Braking" }, { label: "SLC braking limit", sub: true }, ...points("Braking")],
      },
    ],
    // Following and smoothness live in the Driving Personalities dropdown (it embeds this page's rows); the comma's
    // screen shows "Following" / "Acceleration" / "Braking" as section titles, which the cards replace here.
    rows: [
      { label: "Following preset", hide: true,
        help: "How the gap to the car ahead is set. StarPilot Default uses Low- and High-speed follow; Close, Medium, Far and " +
          "Traffic are ready-made; Custom sets the gap at each speed." },
      { label: "Low-speed follow", sub: true, hide: true },
      { label: "High-speed follow", sub: true, hide: true },
      ...points("Following", { hide: true }),
      ...JERKS.map(label => ({ label, hide: true })),
      { label: "Reset Profile to Default", hide: true },
      ...["Following", "Acceleration", "Braking"].map(label => ({ label, hide: true })),
    ],
  }
}

export const pageGuide = page => (typeof page === "string" && Object.hasOwn(PAGE_GUIDES, page) ? PAGE_GUIDES[page] : null)

// A guide has `cards` (the page's layout) and/or plain `rows` (new text only, the page keeps its own layout).
function rowSpecs(guide) {
  const specs = new Map()
  ;(guide.cards || []).forEach((card, cardIndex) => card.rows.forEach((spec, order) => specs.set(spec.label, { ...spec, cardIndex, order })))
  ;(guide.rows || []).forEach(spec => specs.set(spec.label, spec))
  return specs
}

const shownLabel = (specs, label) => specs.get(label)?.display || label

// The page's rows as the guide presents them. Row positions never move (saving goes by position), so a
// placeholder is added after the device's rows, and a hidden row is only marked (guidedCards leaves it out).
// A row the device has made unavailable keeps the device's reason: it says why.
export function guidedRows(page, rows) {
  const guide = pageGuide(page)
  if (!guide || !Array.isArray(rows)) return rows
  const specs = rowSpecs(guide)
  const values = new Map(rows.map(row => [row.label, row.value]))
  const unmet = spec => (spec.needs || []).find(need => values.has(need.label) && values.get(need.label) !== need.value)
  const present = (spec, row) => {
    const help = typeof spec.help === "function" ? spec.help(row) : spec.help
    const blocked = unmet(spec)
    const note = need => need.note || `Turn on ${shownLabel(specs, need.label)} to use this.`
    const map = value => spec.values?.[value] ?? value
    const shown = { ...row, label: spec.display || row.label, guideLabel: row.label, guideSub: !!spec.sub,
      guideHidden: !!spec.hide || !!spec.showWhen && values.get(spec.showWhen.label) !== spec.showWhen.value }
    if (spec.values) Object.assign(shown, { value: map(row.value), choices: (row.choices || []).map(map),
      defaultValue: row.defaultValue == null ? row.defaultValue : map(row.defaultValue),
      guideSave: Object.fromEntries(Object.entries(spec.values).map(([saved, label]) => [label, saved])) })
    if (!row.available) return shown
    if (help === undefined) return blocked ? { ...shown, available: false, reason: `${note(blocked)} ${row.reason}` } : shown
    return blocked ? { ...shown, available: false, reason: `${note(blocked)} ${help}` }
      : { ...shown, reason: help }
  }
  const out = rows.map(row => (specs.has(row.label) && !row.heading ? present(specs.get(row.label), row) : row))
  if (!rows.length) return out
  for (const [label, spec] of specs) {
    if (!spec.placeholder || values.has(label) || !unmet(spec)) continue
    out.push(present(spec, { label, value: spec.placeholder.value, choices: spec.placeholder.choices, available: false, action: false,
      reason: (unmet(spec).note || `Turn on ${shownLabel(specs, unmet(spec).label)} to use this.`) + " " + (typeof spec.help === "string" ? spec.help : ""),
      guidePlaceholder: true }))
  }
  return out
}

// What to save for a value picked on a guided row (a reworded choice goes back to the device's own value).
export const guideSaveValue = (row, value) => row?.guideSave?.[value] ?? value

// Cards for a guided page, built from the rows still visible after search: [{ id, title, icon, intro, include, entries }].
// A card with nothing to show is left out, but there is always one card (it holds the page search box).
// Rows the guide does not list go in "More settings": last, or first when one of them is a repair or reset.
export function guidedCards(page, entries, query = "") {
  const guide = pageGuide(page)
  if (!guide?.cards) return null
  const specs = rowSpecs(guide)
  const needle = String(query || "").trim().toLocaleLowerCase()
  // A card showing another page's rows is searched by its title, intro, and those rows' names and text.
  const includeText = card => {
    const other = pageGuide(card.include.page), otherSpecs = other ? rowSpecs(other) : new Map()
    return [card.title, card.intro, ...card.include.rows.flatMap(label => [label, typeof otherSpecs.get(label)?.help === "string"
      ? otherSpecs.get(label).help : ""])].join(" ").toLocaleLowerCase()
  }
  // Dropdowns show while their switch (`dropsWhen`, a row on this page) has its value; never during a page search.
  const current = new Map(entries.map(({ row }) => [row.guideLabel ?? row.label, row.value]))
  const dropsShown = card => !needle && (!card.dropsWhen || current.get(card.dropsWhen.label) === card.dropsWhen.value)
  const cards = guide.cards.map(card => ({ id: card.id, title: card.title, icon: card.icon, intro: card.intro, guide: true,
    include: card.include && (!needle || includeText(card).includes(needle)) ? card.include : null,
    drops: card.drops && dropsShown(card) ? card.drops : null, entries: [] }))
  const more = { id: "more", title: "More settings", icon: "bi-sliders", intro: "", guide: true, include: null, entries: [] }
  const specOf = entry => specs.get(entry.row.guideLabel ?? entry.row.label)
  for (const entry of entries) {
    if (entry.row.guideHidden) continue
    const spec = specOf(entry)
    ;(spec?.cardIndex !== undefined ? cards[spec.cardIndex] : more).entries.push(entry)
  }
  for (const card of cards) card.entries.sort((a, b) => specOf(a).order - specOf(b).order)
  const repair = more.entries.some(({ row }) => row.confirm || row.repairValue)
  const shown = (repair ? [more, ...cards] : [...cards, more]).filter(card => card.entries.length || card.include || card.drops)
  return shown.length ? shown : [cards[0]]
}
