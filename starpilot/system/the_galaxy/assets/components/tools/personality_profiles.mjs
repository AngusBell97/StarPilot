export function formatProfileSpeed(speedMph, isMetric) {
  const numeric = Number(speedMph);
  if (!Number.isFinite(numeric)) return "—";
  if (!isMetric) return Number.isInteger(numeric) ? String(numeric) : numeric.toFixed(1).replace(/\.0$/, "");
  return (numeric * 1.609344).toFixed(1).replace(/\.0$/, "");
}

export function profileSpeedUnit(isMetric) {
  return isMetric ? "km/h" : "mph";
}

export function valueFromPointer(clientY, rect, minimum, maximum, step) {
  const height = Number(rect?.height);
  const top = Number(rect?.top);
  if (!Number.isFinite(clientY) || !Number.isFinite(height) || height <= 0 || !Number.isFinite(top)) {
    return Number(minimum);
  }
  const ratio = Math.max(0, Math.min(1, 1 - ((clientY - top) / height)));
  const raw = Number(minimum) + ratio * (Number(maximum) - Number(minimum));
  const snapped = Math.round(raw / Number(step)) * Number(step);
  return Number(Math.max(Number(minimum), Math.min(Number(maximum), snapped)).toFixed(4));
}
