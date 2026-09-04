export const PROFILE_CLIPBOARD_SCHEMA_VERSION = 1

export function formatSpeedMph(speed) {
  const numeric = Number(speed)
  if (!Number.isFinite(numeric)) return ""
  return numeric.toFixed(Number.isInteger(numeric) ? 0 : 1)
}

export function copyCurve(category, curve) {
  if (!category || !Array.isArray(curve) || curve.some(value => !Number.isFinite(value))) return null
  return {
    schemaVersion: PROFILE_CLIPBOARD_SCHEMA_VERSION,
    category: String(category),
    curve: curve.map(value => Number(value)),
  }
}

export function pasteCurve(clipboard, category, expectedLength) {
  if (!clipboard || clipboard.schemaVersion !== PROFILE_CLIPBOARD_SCHEMA_VERSION) return null
  if (clipboard.category !== category || !Array.isArray(clipboard.curve)) return null
  if (clipboard.curve.length !== expectedLength || clipboard.curve.some(value => typeof value !== "number" || !Number.isFinite(value))) return null
  return clipboard.curve.map(value => Number(value))
}

export function valueFromPointer(clientY, rect, minimum, maximum, step) {
  const height = Number(rect?.height)
  if (!Number.isFinite(height) || height <= 0) return Number(minimum)

  const fraction = Math.min(1, Math.max(0, (Number(clientY) - Number(rect.top)) / height))
  const raw = Number(maximum) - fraction * (Number(maximum) - Number(minimum))
  const snapped = Number(minimum) + Math.round((raw - Number(minimum)) / Number(step)) * Number(step)
  const precision = Math.max(0, String(step).split(".")[1]?.length || 0)
  return Number(Math.min(Number(maximum), Math.max(Number(minimum), snapped)).toFixed(precision))
}
