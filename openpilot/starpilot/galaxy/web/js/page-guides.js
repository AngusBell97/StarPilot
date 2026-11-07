// Page guides: group a settings page's rows into titled cards and explain each setting in plain words.
// The rows, values and saving are unchanged (they come from the settings API); this file only decides
// which card a row sits in, the text under it, and greys a row whose switch is off.
// Rows are matched by label. A row a guide does not list still shows, in a "More settings" card at the end.

const LANE_TURN_SPEED = row => (row.unit === "km/h" ? "32 km/h" : "20 mph")

export const PAGE_GUIDES = Object.freeze({
  lane_change: {
    intro: "Signal, and StarPilot steers you into the lane on that side. The settings are grouped by what they " +
      "change. Changes save straight away; a lane change that has already started finishes on the settings it started with.",
    cards: [
      {
        id: "assist", title: "Lane change assist", icon: "bi-signpost-split",
        intro: "Whether StarPilot changes lanes when you signal, from what speed, and how quickly it moves across.",
        rows: [
          { label: "Allow lane changes",
            help: "On: when you signal, StarPilot can move the car into the next lane. Off: signalling never starts a lane change and the car stays in its lane." },
          { label: "Minimum lane-change speed",
            help: row => "Lane changes only start above this speed. While Turn Signal Guidance is on (Turns at junctions, below), they start from " +
              LANE_TURN_SPEED(row) + " whatever this says, because a signal below that means a turn." },
          { label: "One change per signal",
            help: "On: one lane change each time you signal. To cross another lane, cancel the signal and signal again. " +
              "Off: keep signalling and it carries on into the next lane (with Automatic Lane Changes off, nudge the wheel again for each lane)." },
          { label: "Lane Change Speed",
            help: "How fast the car moves across once a lane change starts (not a road speed). 10 is the quickest: the driving model's own pace, " +
              "about 3 seconds. Lower numbers soften the steering so it takes longer: 5 is about 6 seconds, 1 about 8." },
        ],
      },
      {
        id: "start", title: "How a lane change starts", icon: "bi-hand-index",
        intro: "Either you nudge the wheel towards the new lane, or StarPilot starts by itself after a short wait. " +
          "It never starts while the car reports something in the blind spot on that side.",
        rows: [
          { label: "Automatic Lane Changes",
            help: "On: starts by itself once the delay below has passed, if the camera sees a lane on that side and the blind spot is clear. " +
              "Off: nudge the wheel towards the new lane to start." },
          { label: "Automatic lane-change delay", needs: { label: "Automatic Lane Changes", value: "On" },
            help: "How long after you signal before an automatic lane change starts, giving you time to cancel. " +
              "The wait starts again if the lane or blind spot stops being clear. 0 starts straight away." },
          { label: "Minimum adjacent lane width", needs: { label: "Automatic Lane Changes", value: "On" },
            help: "An automatic lane change only starts when the camera measures the next lane at least this wide; " +
              "otherwise nudge the wheel. 0 skips the width check." },
        ],
      },
      {
        id: "overtake", title: "Overtaking a slower car", icon: "bi-fast-forward",
        intro: "For when you are following a slower car and signal to pull out and pass. Works only when you pull out to overtake " +
          "(to the right in a right-hand-drive car) and only when StarPilot controls the speed. Moving back in works as normal.",
        rows: [
          { label: "Close lane-change gap",
            help: row => "On: once you signal to overtake, the car stops easing back from the car in front, and can speed up as soon as it " +
              "starts moving across rather than once it is in the new lane. In Experimental mode it runs as Chill for the overtake, then " +
              "switches back. It still slows for a car that is close or braking, and for stops." +
              (row.value === "On" ? "" : " Turn on to also see Faster lane change when close.") },
          { label: "Lane-change follow gap", needs: { label: "Close lane-change gap", value: "On" },
            help: "The closest it follows the car in front while you wait to pull out, as following time. 0.75 s is the closest; " +
              "1.0 s leaves more room. It keeps more space when you are closing fast or the car in front brakes." },
          { label: "Faster lane change when close",
            help: "When you pull out while closing in on the car in front, use the quickest steering (Lane Change Speed 10) for that " +
              "overtake. Your saved Lane Change Speed stays as it is." },
        ],
      },
      {
        id: "turns", title: "Turns at junctions", icon: "bi-sign-turn-right",
        intro: "With Turn Signal Guidance on, a signal below 20 mph (32 km/h) means a turn, not a lane change.",
        // Saved with the Steering and Torque page's settings, shown here.
        include: { page: "torque", rows: ["Turn Signal Guidance"] },
        rows: [],
      },
      {
        id: "signal-mode", title: "Experimental mode when signalling", icon: "bi-sign-merge-right",
        intro: "Conditional Experimental mode can switch to Experimental when you signal below its Signal speed (Long Planner › " +
          "Conditional Driving Modes › Experimental conditions). These two stop that happening for lane changes, so it only switches " +
          "for turns. They do nothing in the other driving modes.",
        rows: [
          { label: "Signal lane detection",
            help: "On: when you signal, it looks for a lane on that side. A lane there means a lane change, so the mode stays as it is; " +
              "no lane means a turn, so it switches to Experimental. Off: every signal below the Signal speed switches to Experimental." },
          { label: "Signal lane width", needs: { label: "Signal lane detection", value: "On" },
            help: "How wide the space beside the car must be to count as a lane (a lane change) rather than a turn. " +
              "0 counts nothing as a lane, so every signal switches to Experimental." },
        ],
      },
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

// The page's rows with the guide's text, and rows whose switch is off greyed with a note saying which.
// A row the device has made unavailable keeps the device's reason: it says why.
export function guidedRows(page, rows) {
  const guide = pageGuide(page)
  if (!guide || !Array.isArray(rows)) return rows
  const specs = rowSpecs(guide)
  const values = new Map(rows.map(row => [row.label, row.value]))
  return rows.map(row => {
    const spec = specs.get(row.label)
    if (!spec || !row.available || row.heading) return row
    const help = typeof spec.help === "function" ? spec.help(row) : spec.help
    const off = spec.needs && values.has(spec.needs.label) && values.get(spec.needs.label) !== spec.needs.value
    return off
      ? { ...row, reason: `Turn on ${spec.needs.label} to use this. ${help}`, available: false, guideOff: true }
      : { ...row, reason: help }
  })
}

// Cards for a guided page, built from the rows still visible after search: [{ id, title, icon, intro, include, entries }].
// A card with nothing to show is left out, but there is always one card (it holds the page search box).
// Rows the guide does not list go in "More settings": last, or first when one of them is a repair or reset.
export function guidedCards(page, entries, query = "") {
  const guide = pageGuide(page)
  if (!guide?.cards) return null
  const specs = rowSpecs(guide)
  const needle = String(query || "").trim().toLocaleLowerCase()
  const cards = guide.cards.map(card => ({ id: card.id, title: card.title, icon: card.icon, intro: card.intro, guide: true,
    include: card.include && (!needle || `${card.title} ${card.include.rows.join(" ")}`.toLocaleLowerCase().includes(needle))
      ? card.include : null, entries: [] }))
  const more = { id: "more", title: "More settings", icon: "bi-sliders", intro: "", guide: true, include: null, entries: [] }
  for (const entry of entries) {
    const spec = specs.get(entry.row.label)
    ;(spec ? cards[spec.cardIndex] : more).entries.push(entry)
  }
  for (const card of cards) card.entries.sort((a, b) => specs.get(a.row.label).order - specs.get(b.row.label).order)
  const repair = more.entries.some(({ row }) => row.confirm || row.repairValue)
  const shown = (repair ? [more, ...cards] : [...cards, more]).filter(card => card.entries.length || card.include)
  return shown.length ? shown : [cards[0]]
}
