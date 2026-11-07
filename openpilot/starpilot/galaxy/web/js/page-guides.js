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
// "Turn on <row>"), showWhen (same, else hidden), sub (indented under the row above), placeholder (greyed stand-in while
// the device leaves the row out because its switch is off). A card with no title has no header.
export const PAGE_GUIDES = Object.freeze({
  lane_change: {
    intro: "Signal, and StarPilot steers you into the lane on that side. Changes save straight away; a lane change that has " +
      "already started finishes on the settings it started with.",
    cards: [
      {
        id: "lane-changes", title: "",
        rows: [
          { label: "Allow lane changes", display: "Auto lane changes",
            help: "The main switch. On: when you signal, StarPilot can move the car into the next lane. Off: signalling never starts " +
              "a lane change, and the settings below do nothing. Called Allow lane changes on the comma's screen." },
          { label: "One change per signal", needs: [ALLOW],
            help: "On: one lane change each time you signal. To cross another lane, cancel the signal and signal again. " +
              "Off: keep signalling and it carries on into the next lane (with Wheel nudge required on, nudge again for each lane)." },
          { label: "Lane Change Speed", needs: [ALLOW],
            help: "How fast the car moves across once a lane change starts (not a road speed). 10 is the quickest: the driving model's " +
              "own pace, about 3 seconds. Lower numbers soften the steering so it takes longer: 5 is about 6 seconds, 1 about 8." },
          { label: "Automatic Lane Changes", display: "Wheel nudge required", values: { Off: "On", On: "Off" }, needs: [ALLOW],
            help: "On: after you signal, nudge the wheel towards the new lane to start. Off (nudgeless): it starts by itself after the " +
              "Nudgeless delay, if the camera sees a lane there. Neither starts with a car in the blind spot. On the comma's screen " +
              "this is Automatic Lane Changes, the other way round (Automatic on = no nudge needed)." },
          { label: "Automatic lane-change delay", display: "Nudgeless delay", needs: [ALLOW, NUDGELESS], sub: true,
            help: "How long after you signal before the lane change starts, giving you time to cancel. The wait starts again if the " +
              "lane or blind spot stops being clear. 0 starts straight away." },
          { label: "Minimum adjacent lane width", needs: [ALLOW, NUDGELESS], sub: true,
            help: "A nudgeless lane change only starts when the camera measures the next lane at least this wide; otherwise nudge " +
              "the wheel. 0 skips the width check." },
          { label: "Minimum lane-change speed", needs: [ALLOW],
            help: row => "Lane changes only start above this speed. While Turn Signal Guidance is on (below), they start from " +
              LANE_TURN_SPEED(row) + " whatever this says, because a signal below that means a turn." },
          { label: "Close lane-change gap", needs: [ALLOW],
            help: "For overtaking a slower car: works only when you pull out to overtake (to the right in a right-hand-drive car) and " +
              "only when StarPilot controls the speed; moving back in works as normal. On: once you signal to overtake, the car stops " +
              "easing back from the car in front, and can speed up as soon as it starts moving across rather than once it is in the new " +
              "lane. In Experimental mode it runs as Chill for the overtake, then switches back. It still slows for a car that is close " +
              "or braking, and for stops." },
          { label: "Lane-change follow gap", needs: [ALLOW, CLOSE_GAP], sub: true,
            help: "The closest it follows the car in front while you wait to pull out, as following time. 0.75 s is the closest; " +
              "1.0 s leaves more room. It keeps more space when you are closing fast or the car in front brakes." },
          { label: "Faster lane change when close", needs: [ALLOW, CLOSE_GAP], sub: true, placeholder: { value: "Off", choices: ["Off", "On"] },
            help: "When you pull out while closing in on the car in front, use the quickest steering (Lane Change Speed 10) for that " +
              "overtake. Your saved Lane Change Speed stays as it is." },
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
    intro: "Options for the comma's driving screen. Changes save straight away. Widget positions and colours are on Colors & Layout.",
    cards: [
      { id: "big", title: "Big layout", icon: "bi-layout-sidebar-reverse", rows: [{ label: "Developer Sidebar" }] },
      { id: "widgets", title: "Widgets", icon: "bi-grid", rows: [{ label: "Use StarPilot Widgets" }, { label: "Wheel brake and acceleration colors" }] },
      { id: "lead", title: "Lead car (Small layout)", icon: "bi-car-front", rows: [{ label: "Lead Indicator" }, { label: "Lead Info" }] },
      { id: "camera", title: "Camera (Small layout)", icon: "bi-camera-video", rows: [{ label: "Camera View" }, { label: "Driver Camera on Reverse" }] },
      { id: "alerts", title: "Alerts and borders", icon: "bi-bell", rows: [{ label: "Hide Personality Alerts" }, { label: "C4 amber signal border" },
        { label: "C4 red blind-spot border" }] },
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
        help: "On: below 20 mph (32 km/h), your signal tells the driving model which way you are turning at a junction, so it steers " +
          "along that path (not while the blind spot on that side shows a car). Lane changes then only start from 20 mph. " +
          "Off: the signal does not guide turns, and lane changes can start from the Minimum lane-change speed." },
    ],
  },
})

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
      guideHidden: !!spec.showWhen && values.get(spec.showWhen.label) !== spec.showWhen.value }
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
  const cards = guide.cards.map(card => ({ id: card.id, title: card.title, icon: card.icon, intro: card.intro, guide: true,
    include: card.include && (!needle || includeText(card).includes(needle)) ? card.include : null, entries: [] }))
  const more = { id: "more", title: "More settings", icon: "bi-sliders", intro: "", guide: true, include: null, entries: [] }
  const specOf = entry => specs.get(entry.row.guideLabel ?? entry.row.label)
  for (const entry of entries) {
    if (entry.row.guideHidden) continue
    const spec = specOf(entry)
    ;(spec?.cardIndex !== undefined ? cards[spec.cardIndex] : more).entries.push(entry)
  }
  for (const card of cards) card.entries.sort((a, b) => specOf(a).order - specOf(b).order)
  const repair = more.entries.some(({ row }) => row.confirm || row.repairValue)
  const shown = (repair ? [more, ...cards] : [...cards, more]).filter(card => card.entries.length || card.include)
  return shown.length ? shown : [cards[0]]
}
