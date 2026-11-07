import assert from "node:assert/strict"
import { guidedRows, guidedCards } from "../web/js/page-guides.js"
import { SettingsPage } from "../web/js/settings.js"

// Driving Personalities as the device sends it (ui/feature_settings_owner.py), and Aggressive's combined page.
const row = (label, value, extra = {}) => ({ label, value, reason: "device text", available: true, action: true, choices: [],
  defaultValue: value, resetAvailable: true, ...extra })
const hub = master => [row("Custom Following and Jerk", master, { choices: ["Off", "On"] }),
  ...["Aggressive", "Standard", "Relaxed", "Traffic"].map(name => row(`${name} Personality`, "Acceleration: Sport · Braking: Normal",
    { page: name.toLowerCase(), action: false }))]
const cards = (page, rows, query = "") =>
  guidedCards(page, guidedRows(page, rows).map((r, index) => ({ row: r, index })), query)

// The switch's card carries one dropdown per personality only while it is on, and never during a page search.
const on = cards("personalities", hub("On"))
assert.deepEqual(on.map(card => card.id), ["custom-following", "personalities"])
assert.deepEqual(on[0].drops.map(drop => [drop.id, drop.page]), [["aggressive", "aggressive"], ["standard", "standard"], ["relaxed", "relaxed"]])
assert.equal(on[0].drops[0].rows[0], "Following preset")
assert.ok(on[0].drops[0].rows.includes("Following at 90 mph") && on[0].drops[0].rows.at(-1) === "Reset Profile to Default")
assert.equal(cards("personalities", hub("Off"))[0].drops, null)
assert.equal(cards("personalities", hub("On"), "aggressive")[0]?.drops ?? null, null)
assert.deepEqual(on[1].entries.map(({ row }) => row.label), ["Aggressive", "Standard", "Relaxed", "Traffic"])

// A personality page shows Acceleration and Braking; following and smoothness live in the hub dropdown.
const aggressive = [
  row("Low-speed follow", "1.25", { step: 0.05, minimum: 0.5, maximum: 3 }), row("Acceleration jerk", "100", { step: 5, minimum: 25, maximum: 200 }),
  row("Reset Profile to Default", "", { confirm: true }), row("Acceleration", "", { available: false, action: false }),
  row("Acceleration preset", "Sport", { choices: ["Sport", "Normal"] }), row("SLC acceleration limit", "0.7", { step: 0.1, minimum: 0.1, maximum: 3.5 }),
  row("Braking", "", { available: false, action: false }), row("Braking preset", "Normal", { choices: ["Sport", "Normal"] }),
  row("Following", "", { available: false, action: false }), row("Following preset", "Custom", { choices: ["Custom", "StarPilot Default"] }),
  row("Following at 0 mph", "1.4", { step: 0.05, minimum: 0.5, maximum: 3 }),
]
const page = cards("aggressive", aggressive)
assert.deepEqual(page.map(card => [card.id, card.entries.map(({ row }) => row.label)]),
  [["acceleration", ["Acceleration", "SLC acceleration limit"]], ["braking", ["Braking"]]])

// Embedded in the hub (only these rows), the same page lists them in the dropdown's order, sub-rows indented.
const view = { onlyRows: ["Following preset", "Low-speed follow", "Following at 0 mph", "Acceleration jerk", "Reset Profile to Default"],
  hideRows: null, state: { query: "", data: { page: "aggressive", rows: aggressive } } }
view.visibleRows = SettingsPage.computed.visibleRows.call(view)
const embedded = SettingsPage.computed.rowCards.call(view)
assert.deepEqual(embedded[0].entries.map(({ row }) => [row.label, !!row.guideSub]),
  [["Following preset", false], ["Low-speed follow", true], ["Following at 0 mph", true], ["Acceleration jerk", false],
   ["Reset Profile to Default", false]])
console.log("personalities guide ok")
