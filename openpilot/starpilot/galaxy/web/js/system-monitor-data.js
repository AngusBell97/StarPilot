// Shared boundary; the selected transport must explicitly choose its source mode.
const object = (value) => value !== null && typeof value === "object" && !Array.isArray(value)
const number = (value, min = 0, max = Infinity) => typeof value === "number" && Number.isFinite(value) && value >= min && value <= max ? value : null
const text = (value) => typeof value === "string" && value.trim() ? value : null
const percent = (value) => number(value, 0, 100)
const integer = (value, min = 0) => Number.isSafeInteger(value) && value >= min ? value : null

export function normalizeSnapshot(raw, expectedMode = "synthetic-preview") {
  if (!["synthetic-preview", "local-runtime"].includes(expectedMode) ||
      !object(raw) || raw.mode !== expectedMode || raw.schemaVersion !== 1 ||
      !Array.isArray(raw.cores) || !Array.isArray(raw.processes) || !object(raw.memory) || !object(raw.storage) ||
      number(raw.sampledAt, 1) === null || !text(raw.source)) {
    throw new Error("Invalid System Monitor sample")
  }
  const cores = raw.cores.map((core) => {
    if (!object(core) || !text(core.name)) throw new Error("Invalid CPU core in sample")
    return { name: core.name, percent: percent(core.percent) }
  })
  if (new Set(cores.map((core) => core.name)).size !== cores.length) throw new Error("Duplicate CPU core in sample")
  const processes = raw.processes.map((process) => {
    if (!object(process) || integer(process.pid, 1) === null || !text(process.name) || !text(process.user) ||
        typeof process.kernel !== "boolean" || !text(process.state)) throw new Error("Invalid process in sample")
    return { instanceId: text(process.instanceId), pid: process.pid, name: process.name, user: process.user, kernel: process.kernel,
      state: process.state, cpu: percent(process.cpu), memoryMiB: number(process.memoryMiB) }
  })
  if (new Set(processes.map((process) => process.pid)).size !== processes.length) throw new Error("Duplicate process PID in sample")
  const memory = { totalMiB: number(raw.memory.totalMiB, 0.01), usedMiB: number(raw.memory.usedMiB),
    availableMiB: number(raw.memory.availableMiB), percent: percent(raw.memory.percent) }
  const storage = { usedGiB: number(raw.storage.usedGiB), totalGiB: number(raw.storage.totalGiB) }
  if (memory.totalMiB !== null && (memory.usedMiB !== null && memory.usedMiB > memory.totalMiB ||
      memory.availableMiB !== null && memory.availableMiB > memory.totalMiB)) {
    memory.usedMiB = null; memory.availableMiB = null; memory.percent = null
  }
  if (storage.totalGiB !== null && storage.usedGiB !== null && storage.usedGiB > storage.totalGiB) storage.usedGiB = null
  const v = object(raw.vitals) ? raw.vitals : {}
  const vitals = Object.fromEntries(["cpuTempC", "gpuTempC", "hotspotTempC", "gpuEdgeTempC", "memoryUsedBytes", "memoryTotalBytes",
    "onboardMaxAgeMs", "maxAgeMs", "memoryMaxAgeMs"].map((key) => [key, number(v[key])]))
  return { sampleMono: number(raw.sampleMono), sourceId: text(raw.sourceId), intervalSeconds: number(raw.intervalSeconds, 0, 10), source: raw.source, sampledAt: raw.sampledAt, cpuPercent: percent(raw.cpuPercent), cores, memory, storage,
    batteryVoltage: number(raw.batteryVoltage, 0.01, 60),
    uptimeSeconds: number(raw.uptimeSeconds), processCount: integer(raw.processCount), processes, vitals }
}

export const processFeature = (process) => processInfo(process).title === process.name ? "" : processInfo(process).title
const PROCESS_STATES = new Map([["R", "Running"], ["S", "Sleeping"], ["D", "Waiting"], ["T", "Stopped"],
  ["t", "Tracing"], ["Z", "Zombie"], ["I", "Idle"]])
export const processState = (state) => PROCESS_STATES.get(state) || state
export const displayNumber = (value, suffix = "") => value === null || value === undefined ? "—" : `${value.toFixed(1)}${suffix}`

export function vital(snapshot, field, budget, elapsedMs = 0) {
  const v = snapshot?.vitals
  if ((field === "memoryUsedBytes" || field === "memoryTotalBytes") && v?.memoryUsedBytes != null && v?.memoryTotalBytes != null &&
      v.memoryUsedBytes > v.memoryTotalBytes) return null
  return v && v[budget] > 0 && elapsedMs >= 0 && elapsedMs < v[budget] ? v[field] ?? null : null
}

export function processRows(snapshot, { query = "", scope = "all", category = "", sort = "cpu", descending = true } = {}) {
  const q = query.trim().toLowerCase()
  const keys = new Set(["name", "category", "pid", "cpu", "memoryMiB", "user", "state"])
  const key = keys.has(sort) ? sort : "cpu"
  return (snapshot?.processes || []).filter((p) =>
    (scope === "all" || (scope === "users" ? !p.kernel : p.user === "comma")) &&
    (!category || processInfo(p).category === category) &&
    (!q || `${p.name} ${processFeature(p)} ${processInfo(p).description} ${processInfo(p).category} ${p.pid} ${p.user} ${processState(p.state)}`.toLowerCase().includes(q))
  ).slice().sort((a, b) => {
    const av = key === "category" ? processInfo(a).category : key === "name" ? processInfo(a).title : a[key], bv = key === "category" ? processInfo(b).category : key === "name" ? processInfo(b).title : b[key]
    if (av === null) return bv === null ? a.pid - b.pid : 1
    if (bv === null) return -1
    const cmp = typeof av === "number" ? av - bv : String(av).localeCompare(String(bv))
    return (descending ? -cmp : cmp) || a.pid - b.pid
  })
}

export const PROCESS_CATEGORIES = ["Driving assistance", "Cameras & recording", "Navigation & location",
  "Display, sound & media", "Galaxy & connectivity", "Device services", "Other / unclassified"]
const catalogue = new Map()
function describe(aliases, title, category, description) {
  for (const alias of aliases.split("|")) catalogue.set(alias, { title, category: PROCESS_CATEGORIES[category], description })
}
describe("selfdrive.controls.controlsd", "Steering and speed control", 0, "Calculates steering and acceleration/braking requests from plans and vehicle state.")
describe("selfdrive.controls.plannerd", "Speed planning", 0, "Plans requested speed and acceleration, including speed-limit and curve-speed features.")
describe("selfdrive.selfdrived.selfdrived", "Engagement and alerts", 0, "Coordinates driving-assistance engagement and driver alerts.")
describe("selfdrive.car.card", "Vehicle interface", 0, "Reads vehicle state and translates control requests for the car.")
describe("selfdrive.controls.radard", "Lead tracking", 0, "Combines lead-vehicle estimates from available model and sensor data; the name does not imply a fitted radar.")
describe("selfdrive.modeld.modeld", "Driving model", 0, "Runs the driving model on road-camera images.")
describe("selfdrive.modeld.dmonitoringmodeld", "Driver monitoring model", 0, "Runs the cabin-camera driver monitoring model.")
describe("selfdrive.monitoring.dmonitoringd", "Driver attention", 0, "Uses driver monitoring results to assess attention and request alerts.")
describe("selfdrive.locationd.locationd", "Vehicle motion estimation", 0, "Estimates vehicle motion from available sensor and model inputs.")
describe("selfdrive.locationd.calibrationd", "Camera calibration", 0, "Estimates camera alignment relative to the car.")
describe("selfdrive.locationd.paramsd", "Vehicle parameter learning", 0, "Estimates vehicle and steering parameters used by driving assistance.")
describe("selfdrive.locationd.torqued", "Steering torque learning", 0, "Learns steering torque response from driving measurements.")
describe("selfdrive.locationd.lagd", "Steering delay estimation", 0, "Estimates the delay in vehicle steering response.")
describe("starpilot.flm.steerlearnerd", "Steering model learning", 0, "Learns steering behaviour for the steering model.")
describe("selfdrive.pandad.pandad|./pandad", "Vehicle communications", 0, "Manages communication with the Panda vehicle interface.")
describe("./camerad|system.camerad.webcam.camerad", "Camera capture", 1, "Captures camera images for other processes.")
describe("./encoderd", "Video encoding", 1, "Encodes camera video for recording or live streaming.")
describe("./loggerd", "Drive recording", 1, "Writes driving logs and coordinates recording segments.")
describe("system.loggerd.uploader", "Upload service", 1, "Uploads eligible recordings and logs under configured rules.")
describe("system.loggerd.deleter", "Recording cleanup", 1, "Removes older recordings when storage space is needed.")
describe("system.logmessaged", "Application logs", 1, "Collects application log messages.")
describe("system.proclogd", "Process telemetry recording", 1, "Records process and system activity for diagnostics.")
describe("system.journald", "System log recording", 1, "Collects operating-system journal records.")
describe("system.tombstoned", "Crash collection", 1, "Collects native crash reports.")
describe("selfdrive.navigation.navd", "Navigation instructions", 2, "Provides navigation instructions to driving software.")
describe("selfdrive.navigation.mapsd", "Navigation maps", 2, "Provides navigation map data.")
describe("starpilot.navigation.runtime", "Route guidance", 2, "Maintains destination routes and guidance from location.")
describe("starpilot.navigation.navtilesd", "Map tiles", 2, "Serves map tiles used by navigation.")
describe("system.qcomgpsd.qcomgpsd|system.ubloxd.ubloxd|system.ubloxd.pigeond", "GPS service", 2, "Works with the device GPS receiver and its location data.")
describe("selfdrive.ui.ui", "Comma display", 3, "Runs the device's on-screen interface.")
describe("selfdrive.ui.soundd", "Alert sounds", 3, "Plays driving-assistance alert sounds.")
describe("system.micd", "Microphone input", 3, "Reads microphone audio for device features.")
describe("starpilot.soloist.supervisor", "Music service supervisor", 3, "Supervises the configured music playback service.")
describe("starpilot.galaxy.managed", "Galaxy web interface", 4, "Serves Galaxy pages and device APIs, including this monitor.")
describe("starpilot.galaxy_bluetooth.bootstrap", "Galaxy phone connection", 4, "Starts the Galaxy Bluetooth phone connection.")
describe("starpilot.galaxy.hotspot_daemon", "Galaxy hotspot", 4, "Manages the Galaxy device hotspot.")
describe("starpilot.galaxy.companion_uplink_daemon", "Companion connection", 4, "Manages the companion uplink.")
describe("starpilot.system.starport.daemon", "StarPort", 4, "Manages the StarPort phone projection service.")
describe("system.athena.manage_athenad", "Remote connection supervisor", 4, "Supervises the configured remote connection service.")
describe("system.athena.athenad", "Remote connection", 4, "Handles requests through the configured remote connection.")
describe("system.webrtc.webrtcd", "Live streaming", 4, "Serves WebRTC live streaming connections.")
describe("common.hardware.comma.modem", "Cellular modem", 4, "Manages the device cellular modem.")
describe("system.manager.manager|./manager.py|system/manager/manager.py|openpilot/system/manager/manager.py", "Process manager", 5, "Starts and supervises configured device processes.")
describe("system.hardware.hardwared", "Device hardware", 5, "Monitors temperature and power and manages cooling.")
describe("system.sensord.sensord", "Motion sensors", 5, "Reads device motion sensors.")
describe("system.timed", "Clock synchronisation", 5, "Maintains device time using available time sources.")
describe("system.updated.updated", "Software updates", 5, "Checks and prepares software updates under configured rules.")
describe("starpilot.analytics.runtime", "Usage statistics", 5, "Maintains StarPilot usage statistics.")
export function processInfo(process) {
  const name = process.name.replace(/^openpilot\./, "")
  return catalogue.get(name) || { title: process.name, category: PROCESS_CATEGORIES[6],
    description: process.kernel && process.state !== "Z" ? "Linux kernel thread; individual role not catalogued." : "Description not yet available." }
}
export const processIdentity = (p) => p.instanceId || String(p.pid) + ":" + p.name

// Histories belong to one producer lifetime and use source monotonic time.
// CPU values are interval means; gauges are means of distinct observed samples.
export class MonitorAverage {
  constructor() { this.reset() }
  reset() { this.history = []; this.sourceId = null }
  add(snapshot) {
    if (!snapshot.sourceId || snapshot.sampleMono === null) return snapshot // Legacy/preview: no invented timing.
    if (this.sourceId !== snapshot.sourceId) { this.reset(); this.sourceId = snapshot.sourceId }
    const end = snapshot.sampleMono, start = end - 10
    if (this.history.length && end <= this.history.at(-1).sampleMono) throw new Error("Non-advancing monitor sample")
    this.history.push(snapshot)
    this.history = this.history.filter(s => s.sampleMono >= start).slice(-128)
    const observations = this.history
    const indexes = new Map(observations.map(s => [s, new Map(s.processes.map(p => [processIdentity(p), p]))]))
    const mean = (get, cpu = false) => {
      let total = 0, weight = 0, first = null, last = null, samples = 0
      for (const s of observations) {
        const value = get(s)
        if (value === null || value === undefined || !Number.isFinite(value)) continue
        const duration = cpu ? Math.max(0, Math.min(end, s.sampleMono) - Math.max(start, s.sampleMono - (s.intervalSeconds || 0))) : 1
        if (!duration) continue
        total += value * duration; weight += duration; samples++
        first ??= s.sampleMono; last = s.sampleMono
      }
      return { value: weight ? total / weight : null, seconds: cpu ? weight : (samples ? last - first : 0), samples, last }
    }
    const result = { ...snapshot, latest: snapshot, averaging: true }
    const overall = mean(s => s.cpuPercent, true)
    result.cpuPercent = overall.value
    result.cpuCoverage = overall.seconds
    result.cpuStale = overall.last !== null && overall.last !== end
    result.windowSeconds = Math.min(10, end - observations[0].sampleMono)
    result.memory = { ...snapshot.memory }
    for (const key of ["usedMiB", "availableMiB", "percent"]) {
      const avg = mean(s => s.memory[key]); result.memory[key] = avg.value
      if (avg.last !== null && avg.last !== end) result.memoryStale = true
    }
    const disk = mean(s => s.storage.usedGiB)
    result.storage = { ...snapshot.storage, usedGiB: disk.value }
    result.storageStale = disk.last !== null && disk.last !== end
    result.cores = snapshot.cores.map(core => {
      const avg = mean(s => s.cores.find(c => c.name === core.name)?.percent, true)
      return { ...core, percent: avg.value, coverage: avg.seconds, stale: avg.last !== null && avg.last !== end }
    })
    result.processes = snapshot.processes.map(p => {
      const id = processIdentity(p)
      const values = s => indexes.get(s).get(id)
      const cpu = mean(s => values(s)?.cpu, true), memory = mean(s => values(s)?.memoryMiB)
      return { ...p, cpu: cpu.value, memoryMiB: memory.value, cpuCoverage: cpu.seconds,
        memoryCoverage: memory.seconds, cpuStale: cpu.last !== null && cpu.last !== end,
        memoryStale: memory.last !== null && memory.last !== end, latestCpu: p.cpu, latestMemory: p.memoryMiB }
    })
    result.vitals = { ...snapshot.vitals }
    for (const [field, budget] of [["cpuTempC","onboardMaxAgeMs"], ["gpuTempC","onboardMaxAgeMs"],
      ["hotspotTempC","maxAgeMs"], ["gpuEdgeTempC","maxAgeMs"], ["memoryUsedBytes","memoryMaxAgeMs"]]) {
      const avg = mean(s => vital(s, field, budget))
      result.vitals[field] = avg.value
      result.vitals[field + "Stale"] = avg.last !== null && (avg.last !== end || !(snapshot.vitals[budget] > 0))
    }
    return result
  }
}
