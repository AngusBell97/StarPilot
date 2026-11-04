import { GxState } from "./state.js"
import { GxNotice } from "./notice.js"
import { displayNumber, heatLevel, managedTitle, processInfo, processIdentity, processRows } from "./system-monitor-data.js"
import { MonitorFeed } from "./monitor-feed.js"

const whole = (value, suffix = "") => value === null || value === undefined ? "—" : `${Math.round(value)}${suffix}`
// Readings dated before 2020 come from a comma whose clock was not set yet
const when = (at) => at && at > 1577836800 ? new Date(at * 1000).toLocaleString([], { day: "numeric", month: "short", hour: "2-digit", minute: "2-digit" }) : ""
const duration = (seconds) => {
  if (seconds === null || seconds === undefined) return "—"
  if (seconds < 60) return `${Math.round(seconds)}s`
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m`
  if (seconds < 86400) return `${Math.floor(seconds / 3600)}h ${Math.floor(seconds / 60) % 60}m`
  return `${Math.floor(seconds / 86400)}d ${Math.floor(seconds / 3600) % 24}h`
}
// Below this a 12 V lead-acid battery is nearly flat and a 16 V lithium one is already in trouble
const LOW_BATTERY_V = 11.8

export const SystemMonitor = {
  name: "SystemMonitor",
  components: { GxState, GxNotice },
  props: { mode: { type: String, required: true }, unauthorized: { type: Function, required: true } },
  data: () => ({ snapshot: null, status: "loading", error: "", query: "", scope: null,
    sort: "cpu", descending: true, ageSeconds: 0, receivedAt: null, resetting: "", resetError: "" }),
  mounted() {
    this.feed = new MonitorFeed({ mode: this.mode, publish: state => this.receive(state), unauthorized: this.unauthorized })
    this.feed.start()
    this.ageTimer = setInterval(() => { if (this.receivedAt !== null) this.ageSeconds = Math.floor((performance.now() - this.receivedAt) / 1000) }, 1000)
  },
  beforeUnmount() { this.feed?.stop(); clearInterval(this.ageTimer) },
  computed: {
    view() { return this.scope ?? (this.snapshot?.processes.some(p => p.openpilot) ? "openpilot" : "all") },
    rows() { return processRows(this.snapshot, { query: this.query, scope: this.view, sort: this.sort, descending: this.descending }) },
    openpilotCount() { return this.snapshot?.processes.filter(p => p.openpilot).length ?? 0 },
    openpilotUse() {
      const mine = this.snapshot?.processes.filter(p => p.openpilot) || []
      return { cpu: mine.reduce((sum, p) => sum + (p.cpu ?? 0), 0), memory: mine.reduce((sum, p) => sum + (p.memoryMiB ?? 0), 0) }
    },
    health() { return this.snapshot?.managed || null },
    battery() { return this.snapshot?.batteryVoltage ?? null },
    batteryLow() { return this.battery !== null && this.battery < LOW_BATTERY_V },
    uptime() { const s = this.snapshot?.uptimeSeconds; return s == null ? "—" : `${Math.floor(s / 3600)}h ${Math.floor(s / 60) % 60}m` },
    chip() { return this.mode === "sample" ? "Preview" : this.status === "current" ? "Live" : this.status === "stale" ? "Reconnecting" : "Connecting" },
    comma() { return this.snapshot?.commaThermals || null },
    gpus() { return this.snapshot?.externalGpus || [] },
  },
  methods: {
    number: displayNumber, whole, when, duration, managedTitle, info: processInfo, identity: processIdentity,
    restartedRecently(p) { return !!p.managedName && p.ageSeconds != null && p.ageSeconds < 300 && (this.snapshot?.uptimeSeconds ?? 0) > 600 },
    exitText(code) { return code == null ? "stopped" : code < 0 ? `stopped by signal ${-code}` : `exited with code ${code}` },
    async resetPeaks(gpu) {
      if (this.resetting || !window.confirm(`Clear the highest logged temperatures for ${gpu.name}?`)) return
      this.resetting = gpu.kind; this.resetError = ""
      try {
        const response = await fetch("./api/system/monitor/reset-peaks", { method: "POST", credentials: "same-origin", cache: "no-store",
          headers: { "Content-Type": "application/json" }, body: JSON.stringify({ gpu: gpu.kind }) })
        if (response.status === 401) { this.unauthorized(); return }
        if (!response.ok) throw new Error("reset failed")
        gpu.hotspotPeak = gpu.memoryPeak = null
        gpu.peaksSince = Date.now() / 1000
      } catch {
        this.resetError = "The highest temperatures could not be cleared. Try again."
      } finally {
        this.resetting = ""
      }
    },
    percentBar(value) { return value == null ? 0 : Math.max(0, Math.min(100, value)) },
    usedPercent(used, total) { return used == null || !total ? null : 100 * used / total },
    tempBar(value, limit) { return value == null ? 0 : Math.max(4, Math.min(100, 100 * value / (limit || 110))) },
    heat: heatLevel,
    commaHeat(c) { return c?.status === "critical" ? "hot" : c?.status === "overheated" ? "warm" : "normal" },
    gpuStats(gpu) {
      const stats = []
      if (gpu.loadPercent != null) stats.push(["GPU load", whole(gpu.loadPercent, "%")])
      if (gpu.powerW != null) stats.push(["Power", gpu.powerLimitW != null ? `${this.number(gpu.powerW)} of ${whole(gpu.powerLimitW)} W` : `${this.number(gpu.powerW)} W`])
      if (gpu.clockMhz != null) stats.push(["GPU clock", whole(gpu.clockMhz, " MHz")])
      if (gpu.fanRpm != null) stats.push(["Fan", whole(gpu.fanRpm, " rpm")])
      else if (gpu.fanPercent != null) stats.push(["Fan", whole(gpu.fanPercent, "%")])
      if (gpu.frameMs != null) stats.push(["Model time per frame", `${this.number(gpu.frameMs)} ms`])
      if (gpu.supplyVolts != null) stats.push(["Power supply", gpu.supplyAmps != null ? `${gpu.supplyVolts.toFixed(1)} V · ${gpu.supplyAmps.toFixed(2)} A` : `${gpu.supplyVolts.toFixed(1)} V`])
      if (gpu.link) stats.push(["GPU link", gpu.link.replace(/^GPU link /, "").replace(/^./, c => c.toUpperCase())])
      if (gpu.usbSpeedMbps != null) stats.push(["USB speed", gpu.usbSpeedMbps >= 1000 ? `${this.number(gpu.usbSpeedMbps / 1000)} Gbps` : `${whole(gpu.usbSpeedMbps)} Mbps`])
      if (gpu.usbLinkErrors) stats.push(["USB link errors", String(gpu.usbLinkErrors)])
      return stats
    },
    sortBy(key) { if (this.sort === key) this.descending = !this.descending; else { this.sort = key; this.descending = ["cpu", "memoryMiB", "ageSeconds"].includes(key) } },
    arrow(key) { return this.sort === key ? (this.descending ? " ↓" : " ↑") : "" },
    ariaSort(key) { return this.sort !== key ? "none" : this.descending ? "descending" : "ascending" },
    receive(state) {
      const table = this.$refs.table
      const top = table?.getBoundingClientRect().top
      const anchor = table && table.scrollTop > 0 && Array.from(table.querySelectorAll("[data-instance]")).find(row => row.getBoundingClientRect().bottom > top + 45)
      const id = anchor?.dataset?.instance, offset = anchor ? anchor.getBoundingClientRect().top - top : 0
      Object.assign(this, state)
      if (state.snapshot && ["current", "sample"].includes(state.status)) { this.receivedAt = performance.now(); this.ageSeconds = 0 }
      this.$nextTick(() => {
        if (!id || !table || this.$refs.table !== table) return
        const row = Array.from(table.querySelectorAll("[data-instance]")).find(row => row.dataset.instance === id)
        if (row) table.scrollTop += row.getBoundingClientRect().top - table.getBoundingClientRect().top - offset
      })
    },
  },
  template: `
    <div class="gx-monitor">
      <div class="gx-monitor__toolbar gx-page-header"><h2>System Monitor</h2>
        <span class="gx-chip" :class="{ 'gx-monitor__chip--live': chip === 'Live' }" role="status">{{ chip }}</span></div>
      <p class="gx-note" v-if="mode === 'sample'">Illustrative values only. No device was read.</p>
      <p class="gx-note" v-else-if="status === 'stale'">Reconnecting… showing the readings from {{ ageSeconds }}s ago.</p>
      <p class="gx-note" v-else>Updates every 2 seconds. CPU and memory are averaged over the last 10 seconds.</p>
      <GxNotice tone="danger" v-if="error && !snapshot">{{ error }}</GxNotice>
      <GxState v-else-if="!snapshot" loading>Loading system activity…</GxState>
      <template v-else>
        <div class="gx-monitor__summary">
          <section class="gx-card gx-monitor__metric"><span>CPU</span><strong>{{ number(snapshot.cpuPercent, '%') }}</strong>
            <div class="gx-monitor__bar"><i :style="{ width: percentBar(snapshot.cpuPercent) + '%' }"></i></div><small>{{ snapshot.cores.length }} cores</small></section>
          <section class="gx-card gx-monitor__metric"><span>Memory</span><strong>{{ number(snapshot.memory.percent, '%') }}</strong>
            <div class="gx-monitor__bar"><i :style="{ width: percentBar(snapshot.memory.percent) + '%' }"></i></div>
            <small>{{ snapshot.memory.usedMiB == null ? '—' : number(snapshot.memory.usedMiB / 1024) }} of {{ snapshot.memory.totalMiB == null ? '—' : number(snapshot.memory.totalMiB / 1024) }} GB</small></section>
          <section class="gx-card gx-monitor__metric"><span>Storage</span><strong>{{ number(usedPercent(snapshot.storage.usedGiB, snapshot.storage.totalGiB), '%') }}</strong>
            <div class="gx-monitor__bar"><i :style="{ width: percentBar(usedPercent(snapshot.storage.usedGiB, snapshot.storage.totalGiB)) + '%' }"></i></div>
            <small>{{ number(snapshot.storage.usedGiB) }} of {{ number(snapshot.storage.totalGiB) }} GB</small></section>
          <section class="gx-card gx-monitor__metric" :data-heat="commaHeat(comma)"><span>Comma temperature</span><strong>{{ comma ? whole(comma.maxTempC, ' °C') : '—' }}</strong>
            <small class="gx-monitor__heat-text">{{ comma?.statusText || 'Unavailable' }}</small><small v-if="comma?.fanPercent != null">Fan {{ whole(comma.fanPercent, '%') }}</small></section>
          <section class="gx-card gx-monitor__metric" :data-heat="batteryLow ? 'hot' : 'normal'"><span>Car battery</span><strong>{{ battery == null ? '—' : battery.toFixed(1) + ' V' }}</strong>
            <small class="gx-monitor__heat-text">{{ battery == null ? 'Unavailable right now' : batteryLow ? 'Low · check the car battery' : 'Measured at the car harness' }}</small></section>
        </div>

        <section class="gx-monitor__section">
          <h3>External GPU</h3>
          <div v-if="!gpus.length" class="gx-card gx-monitor__empty"><strong>No external GPU connected</strong>
            <small>A Chestnut or JetLink device (Jetson, computer or phone) shows here with its temperatures once it is plugged in.</small></div>
          <article v-for="gpu in gpus" :key="gpu.kind" class="gx-card gx-monitor__gpu">
            <header><div><h4>{{ gpu.name }}</h4><small v-if="gpu.detail">{{ gpu.detail }}</small></div>
              <span class="gx-monitor__pill" :data-level="gpu.status">{{ gpu.statusText }}</span></header>
            <div class="gx-monitor__temps">
              <div class="gx-monitor__temp" :data-heat="heat(gpu.hotspotC, gpu.hotspotLimitC)"><span>GPU hotspot</span><strong>{{ whole(gpu.hotspotC, ' °C') }}</strong>
                <div class="gx-monitor__bar"><i :style="{ width: tempBar(gpu.hotspotC, gpu.hotspotLimitC) + '%' }"></i></div>
                <small>{{ gpu.hotspotLimitC != null ? 'Overheat warning at ' + whole(gpu.hotspotLimitC, ' °C') : 'Hottest point on the chip' }}</small>
                <small class="gx-monitor__peak" :data-heat="heat(gpu.hotspotPeak?.value, gpu.hotspotLimitC)">Highest logged <b>{{ gpu.hotspotPeak ? whole(gpu.hotspotPeak.value, ' °C') : '—' }}</b><template v-if="when(gpu.hotspotPeak?.at)"> · {{ when(gpu.hotspotPeak.at) }}</template></small></div>
              <div class="gx-monitor__temp" :data-heat="heat(gpu.memoryTempC, gpu.memoryLimitC)"><span>Memory temperature</span><strong>{{ whole(gpu.memoryTempC, ' °C') }}</strong>
                <div class="gx-monitor__bar"><i :style="{ width: tempBar(gpu.memoryTempC, gpu.memoryLimitC) + '%' }"></i></div>
                <small>{{ gpu.memoryLimitC != null ? 'Overheat warning at ' + whole(gpu.memoryLimitC, ' °C') : gpu.hotspotC != null && gpu.memoryTempC == null ? 'Not reported by this device' : 'GPU memory' }}</small>
                <small class="gx-monitor__peak" :data-heat="heat(gpu.memoryPeak?.value, gpu.memoryLimitC)">Highest logged <b>{{ gpu.memoryPeak ? whole(gpu.memoryPeak.value, ' °C') : '—' }}</b><template v-if="when(gpu.memoryPeak?.at)"> · {{ when(gpu.memoryPeak.at) }}</template></small></div>
            </div>
            <p v-if="gpu.readingsNote" class="gx-note gx-monitor__gpu-note">{{ gpu.readingsNote }}</p>
            <dl v-if="gpuStats(gpu).length" class="gx-monitor__stats"><div v-for="stat in gpuStats(gpu)" :key="stat[0]"><dt>{{ stat[0] }}</dt><dd>{{ stat[1] }}</dd></div></dl>
            <footer v-if="gpu.peaksResettable || gpu.peaksSince" class="gx-monitor__gpu-footer"><small>Highest temperatures are logged all the time, page open or not{{ when(gpu.peaksSince) ? ', since ' + when(gpu.peaksSince) : '' }}.</small>
              <button v-if="gpu.peaksResettable && mode !== 'sample'" type="button" class="gx-monitor__link" :disabled="!!resetting" @click="resetPeaks(gpu)">{{ resetting === gpu.kind ? 'Clearing…' : 'Reset' }}</button></footer>
          </article>
          <p v-if="resetError" class="gx-note gx-monitor__error">{{ resetError }}</p>
        </section>

        <details class="gx-card gx-monitor__cores"><summary>Comma temperatures and CPU cores</summary>
          <dl class="gx-monitor__stats" v-if="comma">
            <div><dt>CPU</dt><dd>{{ whole(comma.cpuTempC, ' °C') }}</dd></div><div><dt>Built-in GPU</dt><dd>{{ whole(comma.gpuTempC, ' °C') }}</dd></div>
            <div><dt>Memory</dt><dd>{{ whole(comma.memoryTempC, ' °C') }}</dd></div><div><dt>Fan</dt><dd>{{ whole(comma.fanPercent, '%') }}</dd></div></dl>
          <div class="gx-monitor__core-list"><span v-for="core in snapshot.cores" :key="core.name">{{ core.name.replace('cpu', 'Core ') }} <b>{{ number(core.percent, '%') }}</b>
            <div class="gx-monitor__bar"><i :style="{ width: percentBar(core.percent) + '%' }"></i></div></span></div>
        </details>

        <section class="gx-monitor__section">
          <div class="gx-monitor__toolbar"><h3>Processes</h3><small class="gx-note">Comma up {{ uptime }}</small></div>
          <div v-if="health" class="gx-monitor__health" :data-level="health.stopped.length ? 'warning' : 'good'" role="status">
            <strong v-if="!health.stopped.length">All {{ health.expected }} openpilot services that should be running are running.</strong>
            <template v-else><strong>{{ health.stopped.length === 1 ? '1 openpilot service is' : health.stopped.length + ' openpilot services are' }} not running</strong>
              <ul><li v-for="p in health.stopped" :key="p.name">{{ managedTitle(p.name) }} <code class="gx-monitor__process-name">{{ p.name }}</code> · {{ exitText(p.exitCode) }}</li></ul></template>
          </div>
          <div class="gx-monitor__filters">
            <div class="gx-monitor__segments" role="group" aria-label="Which processes">
              <button type="button" :aria-pressed="view === 'openpilot' ? 'true' : 'false'" @click="scope = 'openpilot'">openpilot <span>{{ openpilotCount }}</span></button>
              <button type="button" :aria-pressed="view === 'all' ? 'true' : 'false'" @click="scope = 'all'">Everything <span>{{ snapshot.processCount ?? snapshot.processes.length }}</span></button>
            </div>
            <input class="gx-field" type="search" v-model="query" aria-label="Search processes" placeholder="Search processes…"></div>
          <p class="gx-note gx-monitor__use" v-if="view === 'openpilot'">openpilot is using {{ number(openpilotUse.cpu, '%') }} of the CPU and {{ number(openpilotUse.memory / 1024) }} GB of memory.</p>
          <p class="gx-note gx-monitor__use" v-else>Everything includes Linux system services and kernel threads.</p>
          <section ref="table" class="gx-card gx-monitor__table" tabindex="0" aria-label="Processes">
            <table><thead><tr><th v-for="column in [['name','Process'],['cpu','CPU'],['memoryMiB','Memory'],['ageSeconds','Running for']]" :key="column[0]" :aria-sort="ariaSort(column[0])" :class="'gx-monitor__col-' + column[0]"><button type="button" @click="sortBy(column[0])">{{ column[1] }}{{ arrow(column[0]) }}</button></th></tr></thead>
            <tbody><tr v-for="process in rows" :key="identity(process)" :data-instance="identity(process)">
              <td><strong>{{ info(process).title }}</strong>
                <span v-if="restartedRecently(process)" class="gx-monitor__pill gx-monitor__pill--inline" data-level="warning">Restarted {{ duration(process.ageSeconds) }} ago</span>
                <p v-if="info(process).description" class="gx-monitor__description">{{ info(process).description }}</p>
                <code v-if="info(process).title !== process.name" class="gx-monitor__process-name">{{ process.name }}</code></td>
              <td class="gx-monitor__col-cpu">{{ number(process.cpu, '%') }}<div class="gx-monitor__bar gx-monitor__bar--small"><i :style="{ width: percentBar((process.cpu ?? 0) * 4) + '%' }"></i></div></td>
              <td class="gx-monitor__col-memoryMiB">{{ process.memoryMiB == null ? '—' : whole(process.memoryMiB, ' MB') }}</td>
              <td class="gx-monitor__col-ageSeconds">{{ duration(process.ageSeconds) }}</td>
            </tr><tr v-if="!rows.length"><td colspan="4">No matching processes.</td></tr></tbody></table>
          </section>
        </section>
      </template>
    </div>`,
}
