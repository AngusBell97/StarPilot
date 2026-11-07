import assert from "node:assert/strict"
import { readFileSync } from "node:fs"
import { PAGE_GUIDES, pageGuide, guidedRows, guidedCards, guideSaveValue } from "../web/js/page-guides.js"
import { SettingsPage } from "../web/js/settings.js"
import { PAGE_COMPANIONS, PAGE_HIDDEN_ROWS } from "../web/js/menu.js"

// The Lane Changes page as the device sends it (labels from ui/lane_change_feature.py and ui/conditional_feature.py).
const row = (label, value, extra = {}) => ({ label, value, reason: "device text", available: true, action: true, choices: ["Off", "On"],
  defaultValue: value, resetAvailable: true, ...extra })
const number = (label, value, extra = {}) => row(label, value, { choices: [], step: 1, minimum: 0, maximum: 10, ...extra })
const laneRows = (overrides = {}) => [
  number("Lane Change Speed", "10"), row("Allow lane changes", "On"), number("Minimum lane-change speed", "5", { unit: "mph" }),
  row("One change per signal", "On"), row("Automatic Lane Changes", "On", { defaultValue: "Off" }),
  number("Automatic lane-change delay", "0.2"), number("Minimum adjacent lane width", "0"), row("Close lane-change gap", "Off"),
  number("Lane-change follow gap", "0.75"), row("Signal lane detection", "On"), number("Signal lane width", "1.524"),
].map(r => (r.label in overrides ? { ...r, ...overrides[r.label] } : r))
const layout = (rows, query = "") => {
  // The settings page's own search (SettingsPage.visibleRows) runs on the guided rows, then the cards are built.
  const view = { state: { query, data: { page: "lane_change", rows } } }
  const visible = SettingsPage.computed.visibleRows.call(view)
  return guidedCards("lane_change", visible, query)
    .map(card => [card.title, card.entries.map(({ row: r, index }) => `${index} ${r.label}=${r.value}${r.available ? "" : " grey"}${r.guideSub ? " sub" : ""}`)])
}

// Nudgeless (the comma's saved choice): the toggle on top, the selector under it, the nudgeless settings under that.
assert.deepEqual(layout(laneRows()), [
  ["Lane changes", ["1 Allow lane changes=On", "4 Nudge or nudgeless=Nudgeless", "5 Nudgeless delay=0.2 sub", "6 Minimum adjacent lane width=0 sub",
    "2 Minimum lane-change speed=5", "3 One change per signal=On", "0 Lane Change Speed=10"]],
  ["Overtaking a slower car", ["7 Close lane-change gap=Off", "8 Lane-change follow gap=0.75 grey sub", "11 Faster lane change when close=Off grey sub"]],
  ["Turns at junctions", []],
  ["Experimental mode when signalling", ["9 Signal lane detection=On", "10 Signal lane width=1.524 sub"]],
], "rows keep their device positions (saving goes by position); the greyed stand-in for Faster comes after them")

// Nudge hides the nudgeless settings; Allow off greys everything that needs it, including close gap.
const nudgeOff = layout(laneRows({ "Automatic Lane Changes": { value: "Off" }, "Allow lane changes": { value: "Off" } }))
assert.deepEqual(nudgeOff[0][1], ["1 Allow lane changes=Off", "4 Nudge or nudgeless=Nudge grey", "2 Minimum lane-change speed=5 grey",
  "3 One change per signal=On grey", "0 Lane Change Speed=10 grey"])
assert.deepEqual(nudgeOff[1][1], ["7 Close lane-change gap=Off grey", "8 Lane-change follow gap=0.75 grey sub", "11 Faster lane change when close=Off grey sub"])
assert.deepEqual(layout(laneRows({ "Signal lane detection": { value: "Off" } }))[3][1], ["9 Signal lane detection=Off", "10 Signal lane width=1.524 grey sub"])

// Close gap on: the device sends the real Faster row, so no stand-in, and both rows are usable.
const closeRows = laneRows({ "Close lane-change gap": { value: "On" } })
closeRows.splice(9, 0, row("Faster lane change when close", "Off"))
assert.deepEqual(layout(closeRows)[1][1], ["7 Close lane-change gap=On", "8 Lane-change follow gap=0.75 sub", "9 Faster lane change when close=Off sub"])

// Greyed rows say which switch to turn on; a row the device made unavailable keeps the device's reason.
const shown = guidedRows("lane_change", laneRows({ "Allow lane changes": { value: "Off" }, "Close lane-change gap": { available: false, reason: "Needs speed control" } }))
assert.match(shown.find(r => r.label === "One change per signal").reason, /^Turn on Allow lane changes to use this\. On: one lane change/)
assert.equal(shown.find(r => r.label === "Close lane-change gap").reason, "Needs speed control")
assert.match(shown.find(r => r.label === "Faster lane change when close").reason, /^Turn on Allow lane changes to use this\./)

// The selector shows Nudge / Nudgeless and saves the device's own Off / On.
const selector = guidedRows("lane_change", laneRows()).find(r => r.guideLabel === "Automatic Lane Changes")
assert.deepEqual([selector.label, selector.value, selector.choices, selector.defaultValue], ["Nudge or nudgeless", "Nudgeless", ["Nudge", "Nudgeless"], "Nudge"])
assert.equal(guideSaveValue(selector, "Nudge"), "Off")
assert.equal(guideSaveValue(selector, "Nudgeless"), "On")
assert.equal(guideSaveValue(laneRows()[0], "7"), "7", "other rows save what was picked")
assert.equal(guideSaveValue(undefined, "On"), "On")
assert.match(SettingsPage.template, /feed\.previewValue\(index, guideSaveValue\(row, value\)\)/)
assert.equal(SettingsPage.methods.guideSaveValue, guideSaveValue)

// Speed text follows the device's units.
assert.match(guidedRows("lane_change", laneRows({ "Minimum lane-change speed": { unit: "km/h" } }))
  .find(r => r.label === "Minimum lane-change speed").reason, /start from 32 km\/h/)

// Nothing disappears: a row the guide does not know goes to "More settings" (first when it is a repair).
assert.deepEqual(layout([...laneRows(), row("A new Dom setting", "Off")]).at(-1), ["More settings", ["11 A new Dom setting=Off"]])
assert.deepEqual(layout([{ label: "Restore Lane Change defaults", value: "", available: true, action: true, confirm: true, choices: [] }])[0],
  ["More settings", ["0 Restore Lane Change defaults="]])
// Search keeps one card for the search box, and matches the new wording.
assert.deepEqual(layout(laneRows(), "zzz"), [["Lane changes", []]])
assert.deepEqual(layout(laneRows(), "blind spot").map(([title]) => title), ["Lane changes", "Turns at junctions"])

// Turn Signal Guidance: shown inside the Turns card from the Steering page, with new text there too.
assert.deepEqual(PAGE_GUIDES.lane_change.cards.find(c => c.id === "turns").include, { page: "torque", rows: ["Turn Signal Guidance"] })
assert.equal(PAGE_COMPANIONS.lane_change, undefined)
assert.deepEqual(PAGE_HIDDEN_ROWS.torque, ["Turn Signal Guidance"])
assert.match(guidedRows("torque", [row("Turn Signal Guidance", "On")])[0].reason, /^On: below 20 mph/)
assert.equal(guidedCards("torque", []), null, "the Steering page keeps its own layout")
assert.match(SettingsPage.template, /<SettingsPage v-if="card\.include && mode === 'local'"[^>]* bare embedded menu-mode/)
assert.equal(SettingsPage.name, "SettingsPage")

// Pages without a guide are untouched.
const plain = [row("Something", "On")]
assert.equal(guidedRows("profiles", plain), plain)
assert.equal(guidedCards("profiles", []), null)
assert.equal(pageGuide("constructor"), null)

// Every label a guide relies on still exists in the device code (a Dom rename would otherwise drop a row into "More settings").
const source = ["../../ui/lane_change_feature.py", "../../ui/conditional_feature.py", "../../ui/feature_settings_owner.py"]
  .map(path => readFileSync(new URL(path, import.meta.url), "utf8")).join("\n")
for (const guide of Object.values(PAGE_GUIDES)) {
  for (const spec of [...(guide.cards || []).flatMap(card => card.rows), ...(guide.rows || [])]) {
    assert.ok(source.includes(`"${spec.label}"`) || source.includes(`'${spec.label}'`), `device code no longer has the row "${spec.label}"`)
    for (const need of [...(spec.needs || []), ...(spec.showWhen ? [spec.showWhen] : [])]) {
      assert.ok(source.includes(`"${need.label}"`), `device code no longer has the row "${need.label}"`)
    }
  }
}
console.log("page guides ok")
