import assert from "node:assert/strict"
import { readFileSync } from "node:fs"
import { ModelsPage, MODEL_SORTS } from "../web/js/models.js"

const template = ModelsPage.template

// Single model / Model pair selector: one mode's settings at a time.
assert.ok(template.includes(`@click="setDriveMode('single')"><i aria-hidden="true" class="bi bi-cpu"></i> Single model</button>`))
assert.match(template, /<template v-if="driveMode === 'single'">/)
assert.match(template, /<div v-else id="gx-model-laboratory" ref="laboratory"/)
assert.doesNotMatch(template, /aria-label="Enable model pair"/, "the selector replaces the pair switch")

const page = (enabled, available = () => true) => {
  const calls = []
  const view = { ...ModelsPage.data(), lab: { configuration: { enabled } }, labActionAvailable: available,
    runLabAction: action => { calls.push(action); return Promise.resolve({ message: "ok" }) } }
  Object.defineProperty(view, "driveMode", { get() { return ModelsPage.computed.driveMode.call(this) } })
  view.enablePair = () => ModelsPage.methods.enablePair.call(view)
  return { view, calls }
}
{
  const { view, calls } = page(false)
  assert.equal(view.driveMode, "single")
  await ModelsPage.methods.setDriveMode.call(view, "pair")
  assert.deepEqual(calls, ["enable"], "Model pair switches the pair on when it can run")
}
{
  const { view, calls } = page(false, () => false)
  await ModelsPage.methods.setDriveMode.call(view, "pair")
  assert.deepEqual(calls, [], "a pair that cannot run yet is not switched on")
  assert.equal(view.driveMode, "pair", "the pair settings show so the missing parts can be chosen")
}
{
  const { view, calls } = page(true)
  assert.equal(view.driveMode, "pair")
  await ModelsPage.methods.setDriveMode.call(view, "single")
  assert.deepEqual(calls, ["disable"], "Single model switches the pair off")
  assert.equal(view.pairView, null)
}

// Compact list with Filters and Sort by; no Download Missing Models; catalog check is a small refresh button.
assert.match(template, /<ul v-else class="gx-model-list" aria-label="Models">/)
assert.match(template, /class="gx-model-filters__label">Filters</)
assert.match(template, /class="gx-model-filters__label">Sort by</)
assert.doesNotMatch(template, /Download Missing Models/)
assert.doesNotMatch(template, /Check model catalog/)
assert.match(template, /class="gx-icon-btn gx-model-refresh"[^>]*aria-label="Check for new models"[^>]*@click="runAction\('refresh'\)"/)
assert.match(template, /Cancel download/)
assert.match(template, /title="'Model format ' \+ m\.version/)

// Sorts: newest, name, model format version, size; ties fall back to the name.
const models = [
  { value: "a", label: "Alpha", version: "v9", released: "2025-01-01", artifactSize: 300 },
  { value: "b", label: "Beta", version: "v16", released: "2026-11-05", artifactSize: 100 },
  { value: "c", label: "Gamma", version: "v12", released: "2026-03-01", fileSizeBytes: 900 },
]
const order = sortMode => ModelsPage.computed.sorted.call({ ...ModelsPage.data(), hardwareFilter: "both", labRow: () => null, models, sortMode }).map(m => m.value)
assert.deepEqual(order("release_date"), ["b", "c", "a"])
assert.deepEqual(order("alphabetical"), ["a", "b", "c"])
assert.deepEqual(order("version"), ["b", "c", "a"])
assert.deepEqual(order("size"), ["c", "a", "b"])
assert.deepEqual(Object.keys(MODEL_SORTS), ["release_date", "alphabetical", "version", "size"])

// eGPU variant details only with Model pair or Pairable only.
assert.equal(ModelsPage.computed.showLabVariants.call({ driveMode: "single", pairableFilter: false }), false)
assert.equal(ModelsPage.computed.showLabVariants.call({ driveMode: "pair", pairableFilter: false }), true)
assert.equal(ModelsPage.computed.showLabVariants.call({ driveMode: "single", pairableFilter: true }), true)

const html = readFileSync(new URL("../web/index.html", import.meta.url), "utf8")
assert.match(html, /css\/model-list\.css/)
console.log("Model Manager selector, compact list, filters and sorts passed")
