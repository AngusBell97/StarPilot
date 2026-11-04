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
      state: process.state, cpu: percent(process.cpu), memoryMiB: number(process.memoryMiB), ageSeconds: number(process.ageSeconds),
      openpilot: process.openpilot === true, managedName: text(process.managedName) }
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
  return { sampleMono: number(raw.sampleMono), sourceId: text(raw.sourceId), intervalSeconds: number(raw.intervalSeconds, 0, 10), source: raw.source, sampledAt: raw.sampledAt, cpuPercent: percent(raw.cpuPercent), cores, memory, storage,
    batteryVoltage: number(raw.batteryVoltage, 0.01, 60),
    uptimeSeconds: number(raw.uptimeSeconds), processCount: integer(raw.processCount), processes,
    managed: normalizeManaged(raw.managed),
    commaThermals: normalizeThermals(raw.commaThermals), externalGpus: Array.isArray(raw.externalGpus) ? raw.externalGpus.map(normalizeGpu).filter(Boolean) : [] }
}

const temperature = (value) => number(value, 1, 150)
const LEVELS = new Set(["good", "idle", "warning", "danger"])

function normalizeManaged(raw) {
  if (!object(raw) || integer(raw.expected) === null || integer(raw.running) === null || !Array.isArray(raw.stopped)) return null
  return { expected: raw.expected, running: raw.running,
    stopped: raw.stopped.filter(p => object(p) && text(p.name)).map(p => ({ name: p.name, exitCode: Number.isInteger(p.exitCode) ? p.exitCode : null })) }
}

const peak = (raw) => object(raw) && temperature(raw.value) !== null && number(raw.at, 0) !== null ? { value: raw.value, at: raw.at } : null

function normalizeThermals(raw) {
  if (!object(raw)) return null
  return { cpuTempC: temperature(raw.cpuTempC), gpuTempC: temperature(raw.gpuTempC), memoryTempC: temperature(raw.memoryTempC),
    maxTempC: temperature(raw.maxTempC), fanPercent: percent(raw.fanPercent),
    status: ["ok", "overheated", "critical"].includes(raw.status) ? raw.status : null, statusText: text(raw.statusText) }
}

function normalizeGpu(raw) {
  if (!object(raw) || !["chestnut", "jetlink"].includes(raw.kind) || !text(raw.name)) return null
  return { kind: raw.kind, name: raw.name, detail: text(raw.detail), status: LEVELS.has(raw.status) ? raw.status : "idle",
    statusText: text(raw.statusText) || "Connected", runningModel: raw.runningModel === true, readingsNote: text(raw.readingsNote),
    hotspotC: temperature(raw.hotspotC), memoryTempC: temperature(raw.memoryTempC),
    hotspotLimitC: temperature(raw.hotspotLimitC), memoryLimitC: temperature(raw.memoryLimitC),
    loadPercent: percent(raw.loadPercent), clockMhz: number(raw.clockMhz, 1), powerW: number(raw.powerW),
    powerLimitW: number(raw.powerLimitW, 0.1), fanRpm: number(raw.fanRpm), fanPercent: percent(raw.fanPercent),
    supplyVolts: number(raw.supplyVolts), supplyAmps: number(raw.supplyAmps, -100), frameMs: number(raw.frameMs),
    link: text(raw.link), usbSpeedMbps: number(raw.usbSpeedMbps, 1), usbLinkErrors: integer(raw.usbLinkErrors),
    hotspotPeak: peak(raw.hotspotPeak), memoryPeak: peak(raw.memoryPeak), peaksSince: number(raw.peaksSince, 0),
    peaksResettable: raw.peaksResettable === true }
}

// How close a temperature is to its limit: "hot" at the limit, "warm" within 10 °C of it.
export function heatLevel(value, limit) {
  if (value === null || value === undefined || limit === null || limit === undefined) return "normal"
  return value >= limit ? "hot" : value >= limit - 10 ? "warm" : "normal"
}

export const processFeature = (process) => processInfo(process).title === process.name ? "" : processInfo(process).title
const PROCESS_STATES = new Map([["R", "Running"], ["S", "Sleeping"], ["D", "Waiting"], ["T", "Stopped"],
  ["t", "Tracing"], ["Z", "Zombie"], ["I", "Idle"]])
export const processState = (state) => PROCESS_STATES.get(state) || state
export const displayNumber = (value, suffix = "") => value === null || value === undefined ? "—" : `${value.toFixed(1)}${suffix}`

export function processRows(snapshot, { query = "", scope = "all", sort = "cpu", descending = true } = {}) {
  const q = query.trim().toLowerCase()
  const keys = new Set(["name", "pid", "cpu", "memoryMiB", "user", "state", "ageSeconds"])
  const key = keys.has(sort) ? sort : "cpu"
  return (snapshot?.processes || []).filter((p) =>
    (scope === "all" || (scope === "openpilot" ? p.openpilot : scope === "users" ? !p.kernel : p.user === "comma")) &&
    (!q || `${p.name} ${processFeature(p)} ${processInfo(p).description} ${p.pid} ${p.user} ${processState(p.state)}`.toLowerCase().includes(q))
  ).slice().sort((a, b) => {
    const av = key === "name" ? processInfo(a).title : a[key], bv = key === "name" ? processInfo(b).title : b[key]
    if (av === null) return bv === null ? a.pid - b.pid : 1
    if (bv === null) return -1
    const cmp = typeof av === "number" ? av - bv : String(av).localeCompare(String(bv))
    return (descending ? -cmp : cmp) || a.pid - b.pid
  })
}

const catalogue = new Map()
function describe(aliases, title, description) {
  for (const alias of aliases.split("|")) catalogue.set(alias, { title, description })
}
describe("selfdrive.controls.controlsd", "Steering and speed control", "Calculates steering and acceleration/braking requests from plans and vehicle state.")
describe("selfdrive.controls.plannerd", "Speed planning", "Plans requested speed and acceleration, including speed-limit and curve-speed features.")
describe("selfdrive.selfdrived.selfdrived", "Engagement and alerts", "Coordinates driving-assistance engagement and driver alerts.")
describe("selfdrive.car.card", "Vehicle interface", "Reads vehicle state and translates control requests for the car.")
describe("selfdrive.controls.radard", "Lead tracking", "Combines lead-vehicle estimates from available model and sensor data; the name does not imply a fitted radar.")
describe("selfdrive.modeld.modeld", "Driving model", "Runs the driving model on road-camera images.")
describe("selfdrive.modeld.dmonitoringmodeld", "Driver monitoring model", "Runs the cabin-camera driver monitoring model.")
describe("selfdrive.monitoring.dmonitoringd", "Driver attention", "Uses driver monitoring results to assess attention and request alerts.")
describe("selfdrive.locationd.locationd", "Vehicle motion estimation", "Estimates vehicle motion from available sensor and model inputs.")
describe("selfdrive.locationd.calibrationd", "Camera calibration", "Estimates camera alignment relative to the car.")
describe("selfdrive.locationd.paramsd", "Vehicle parameter learning", "Estimates vehicle and steering parameters used by driving assistance.")
describe("selfdrive.locationd.torqued", "Steering torque learning", "Learns steering torque response from driving measurements.")
describe("selfdrive.locationd.lagd", "Steering delay estimation", "Estimates the delay in vehicle steering response.")
describe("starpilot.flm.steerlearnerd", "Steering model learning", "Learns steering behaviour for the steering model.")
describe("selfdrive.pandad.pandad|./pandad", "Vehicle communications", "Manages communication with the Panda vehicle interface.")
describe("./camerad|system.camerad.webcam.camerad", "Camera capture", "Captures camera images for other processes.")
describe("./encoderd", "Video encoding", "Encodes camera video for recording or live streaming.")
describe("./loggerd", "Drive recording", "Writes driving logs and coordinates recording segments.")
describe("system.loggerd.uploader", "Upload service", "Uploads eligible recordings and logs under configured rules.")
describe("system.loggerd.deleter", "Recording cleanup", "Removes older recordings when storage space is needed.")
describe("system.logmessaged", "Application logs", "Collects application log messages.")
describe("system.proclogd", "Process telemetry recording", "Records process and system activity for diagnostics.")
describe("system.journald", "System log recording", "Collects operating-system journal records.")
describe("system.tombstoned", "Crash collection", "Collects native crash reports.")
describe("selfdrive.navigation.navd", "Navigation instructions", "Provides navigation instructions to driving software.")
describe("selfdrive.navigation.mapsd", "Navigation maps", "Provides navigation map data.")
describe("starpilot.navigation.runtime", "Route guidance", "Maintains destination routes and guidance from location.")
describe("starpilot.navigation.navtilesd", "Map tiles", "Serves map tiles used by navigation.")
describe("system.qcomgpsd.qcomgpsd|system.ubloxd.ubloxd|system.ubloxd.pigeond", "GPS service", "Works with the device GPS receiver and its location data.")
describe("selfdrive.ui.ui", "Comma display", "Runs the device's on-screen interface.")
describe("selfdrive.ui.soundd", "Alert sounds", "Plays driving-assistance alert sounds.")
describe("system.micd", "Microphone input", "Reads microphone audio for device features.")
describe("starpilot.soloist.supervisor", "Music service supervisor", "Supervises the configured music playback service.")
describe("starpilot.galaxy.managed", "Galaxy web interface", "Serves Galaxy pages and device APIs, including this monitor.")
describe("starpilot.galaxy_bluetooth.bootstrap", "Galaxy phone connection", "Starts the Galaxy Bluetooth phone connection.")
describe("starpilot.galaxy.hotspot_daemon", "Galaxy hotspot", "Manages the Galaxy device hotspot.")
describe("starpilot.galaxy.companion_uplink_daemon", "Companion connection", "Manages the companion uplink.")
describe("starpilot.system.starport.daemon", "StarPort", "Manages the StarPort phone projection service.")
describe("system.athena.manage_athenad", "Remote connection supervisor", "Supervises the configured remote connection service.")
describe("system.athena.athenad", "Remote connection", "Handles requests through the configured remote connection.")
describe("system.webrtc.webrtcd", "Live streaming", "Serves WebRTC live streaming connections.")
describe("common.hardware.comma.modem", "Cellular modem", "Manages the device cellular modem.")
describe("system.manager.manager|./manager.py|system/manager/manager.py|openpilot/system/manager/manager.py", "Process manager", "Starts and supervises configured device processes.")
describe("system.hardware.hardwared", "Device hardware", "Monitors temperature and power and manages cooling.")
describe("system.sensord.sensord", "Motion sensors", "Reads device motion sensors.")
describe("system.timed", "Clock synchronisation", "Maintains device time using available time sources.")
describe("system.updated.updated", "Software updates", "Checks and prepares software updates under configured rules.")
describe("starpilot.analytics.runtime", "Usage statistics", "Maintains StarPilot usage statistics.")
export function processInfo(process) {
  const name = process.name.replace(/^openpilot\./, "")
  return catalogue.get(name) || { title: process.name, description: process.kernel && process.state !== "Z" ? "Linux kernel thread." : "" }
}
// A process-manager name (e.g. "controlsd") as a friendly title, when the catalogue knows its module.
export function managedTitle(name) {
  if (MANAGED_TITLES[name]) return MANAGED_TITLES[name]
  for (const [alias, entry] of catalogue) if (alias.endsWith("." + name)) return entry.title
  return name
}
const MANAGED_TITLES = { galaxy: "Galaxy web interface", galaxy_hotspot: "Galaxy hotspot", galaxy_phone: "Galaxy phone connection",
  companion_uplink: "Companion connection", updated: "Software updates", usage_stats: "Usage statistics", ui: "Comma display",
  manage_athenad: "Remote connection supervisor", pandad: "Vehicle communications", camerad: "Camera capture", loggerd: "Drive recording",
  encoderd: "Video encoding", modem: "Cellular modem" }

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
    return result
  }
}
