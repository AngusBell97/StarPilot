import { GxState } from "./state.js"
import { GxNotice } from "./notice.js"
import { displayNumber, processInfo, processIdentity, processRows, processState, vital, PROCESS_CATEGORIES } from "./system-monitor-data.js"
import { MonitorFeed } from "./monitor-feed.js"
import { GalaxySelect } from "./galaxy-select.js"

export const SystemMonitor = {
  name: "SystemMonitor",
  components: { GxState, GxNotice, GalaxySelect },
  props: { mode: { type: String, required: true }, unauthorized: { type: Function, required: true } },
  data: () => ({ snapshot: null, status: "loading", error: "", query: "", scope: "all", category: "",
    sort: "cpu", descending: true, expanded: {}, ageSeconds: 0, receivedAt: null, categories: PROCESS_CATEGORIES }),
  mounted() {
    this.feed = new MonitorFeed({ mode: this.mode, publish: state => this.receive(state), unauthorized: this.unauthorized })
    this.feed.start()
    this.ageTimer = setInterval(() => { if (this.receivedAt !== null) this.ageSeconds = Math.floor((performance.now() - this.receivedAt) / 1000) }, 1000)
  },
  beforeUnmount() { this.feed?.stop(); clearInterval(this.ageTimer) },
  computed: {
    rows() { return processRows(this.snapshot, this) },
    captured() { return this.snapshot ? new Date(this.snapshot.sampledAt * 1000).toLocaleString() : "—" },
    uptime() { const s = this.snapshot?.uptimeSeconds; return s == null ? "—" : `${Math.floor(s / 3600)}h ${Math.floor(s / 60) % 60}m` },
    vramUsed() { return this.sensor("memoryUsedBytes", "memoryMaxAgeMs") },
    vramTotal() { return this.sensor("memoryTotalBytes", "memoryMaxAgeMs") },
    averageLabel() {
      if (!this.snapshot?.averaging) return this.mode === "sample" ? "Illustrative readings" : "Latest readings · timing unavailable"
      return this.snapshot.windowSeconds < 10 ? `10s averages · collecting (${this.snapshot.windowSeconds.toFixed(0)}/10s)` : "10s rolling averages"
    },
  },
  methods: {
    number: displayNumber, info: processInfo, identity: processIdentity, state: processState,
    sensor(field, budget) { return this.snapshot?.averaging ? this.snapshot.vitals[field] : vital(this.snapshot, field, budget) },
    coverage(p) { return p.cpuStale ? "Last measured" : p.cpuCoverage == null ? "" : p.cpuCoverage < 9.95 ? `${p.cpuCoverage.toFixed(1)}/10s measured` : "10s measured" },
    sortBy(key) { if (this.sort === key) this.descending = !this.descending; else { this.sort = key; this.descending = ["cpu", "memoryMiB"].includes(key) } },
    arrow(key) { return this.sort === key ? (this.descending ? " ↓" : " ↑") : "" },
    ariaSort(key) { return this.sort !== key ? "none" : this.descending ? "descending" : "ascending" },
    receive(state) {
      const table = this.$refs.table
      const top = table?.getBoundingClientRect().top
      const anchor = table && table.scrollTop > 0 && Array.from(table.querySelectorAll("[data-instance]")).find(row => row.getBoundingClientRect().bottom > top + 45)
      const id = anchor?.dataset?.instance, offset = anchor ? anchor.getBoundingClientRect().top - top : 0
      const oldSource = this.snapshot?.sourceId
      Object.assign(this, state)
      if (!state.snapshot || oldSource && oldSource !== state.snapshot.sourceId) this.expanded = {}
      if (state.snapshot && ["current", "sample"].includes(state.status)) {
        this.receivedAt = performance.now(); this.ageSeconds = 0
        const ids = new Set(state.snapshot.processes.map(processIdentity))
        this.expanded = Object.fromEntries(Object.entries(this.expanded).filter(([key]) => ids.has(key)))
      }
      this.$nextTick(() => {
        if (!id || !table || this.$refs.table !== table) return
        const row = Array.from(table.querySelectorAll("[data-instance]")).find(row => row.dataset.instance === id)
        if (row) table.scrollTop += row.getBoundingClientRect().top - table.getBoundingClientRect().top - offset
      })
    },
  },
  template: `
    <div class="gx-monitor">
      <div class="gx-monitor__toolbar gx-page-header"><div><h2>System Monitor</h2><div class="gx-note">Captured {{ captured }} · device time</div></div>
        <span class="gx-chip" role="status">{{ mode === 'sample' ? 'Preview' : status === 'current' ? 'Live' : status === 'stale' ? 'Stale' : 'Connecting' }}</span></div>
      <p class="gx-note">{{ averageLabel }}. <span v-if="receivedAt !== null">Last update received {{ ageSeconds }}s ago.</span> Last readings stay visible while reconnecting.</p>
      <GxNotice tone="danger" v-if="error && !snapshot">{{ error }}</GxNotice>
      <GxState v-else-if="!snapshot" loading>Loading system activity…</GxState>
      <template v-else>
        <div class="gx-monitor__summary">
          <section class="gx-card gx-monitor__metric"><span>CPU · 10s avg</span><strong>{{ number(snapshot.cpuPercent, '%') }}</strong><small>{{ snapshot.cores.length }} cores · whole device</small><small>{{ snapshot.cpuStale ? 'Last measured' : snapshot.cpuCoverage != null ? number(snapshot.cpuCoverage, '/10s measured') : '' }}</small></section>
          <section class="gx-card gx-monitor__metric"><span>Memory · 10s avg</span><strong>{{ number(snapshot.memory.percent, '%') }}</strong><small v-if="snapshot.memoryStale">Last measured</small><small>{{ snapshot.memory.usedMiB == null ? '—' : number(snapshot.memory.usedMiB / 1024) }} / {{ snapshot.memory.totalMiB == null ? '—' : number(snapshot.memory.totalMiB / 1024) }} GiB</small></section>
          <section class="gx-card gx-monitor__metric"><span>Storage used · 10s avg</span><strong>{{ number(snapshot.storage.usedGiB) }} GiB</strong><small v-if="snapshot.storageStale">Last measured</small><small>{{ number(snapshot.storage.totalGiB) }} GiB total</small></section>
          <section class="gx-card gx-monitor__metric"><span>Processes</span><strong>{{ snapshot.processCount ?? '—' }}</strong><small>Uptime {{ uptime }}</small></section>
        </div>
        <details class="gx-card gx-monitor__cores"><summary>CPU cores</summary><div><span v-for="core in snapshot.cores" :key="core.name">{{ core.name.toUpperCase() }} <b>{{ number(core.percent, '%') }}</b><small v-if="core.stale">Last measured</small><progress v-if="core.percent != null" :value="core.percent" max="100" :aria-label="core.name + ' usage'"></progress></span></div></details>
        <details class="gx-card gx-monitor__cores"><summary>Temperatures and GPU memory</summary><p class="gx-note">10s means where available. Missing sensors are unavailable; retained readings may be stale.</p>
          <div><span v-for="metric in [['cpuTempC','onboardMaxAgeMs','CPU temperature'],['gpuTempC','onboardMaxAgeMs','Onboard GPU temperature'],['hotspotTempC','maxAgeMs','eGPU hotspot'],['gpuEdgeTempC','maxAgeMs','eGPU temperature']]" :key="metric[0]">{{ metric[2] }} <b>{{ sensor(metric[0],metric[1]) == null ? 'Unavailable' : number(sensor(metric[0],metric[1]), ' °C') }}</b><small v-if="snapshot.vitals[metric[0] + 'Stale'] || status === 'stale'">Last received</small></span>
          <span>eGPU VRAM <b>{{ vramUsed == null ? 'Unavailable' : number(vramUsed / 1073741824) + ' GiB' }}</b><small v-if="vramTotal != null">{{ number(vramTotal / 1073741824) }} GiB capacity</small></span></div>
        </details>
        <div class="gx-monitor__filters"><input class="gx-field" type="search" v-model="query" aria-label="Search processes" placeholder="Search process, purpose, category or PID…">
          <GalaxySelect class="gx-field gx-field--full" v-model="category" aria-label="Category"><option value="">All categories</option><option v-for="c in categories" :key="c" :value="c">{{ c }}</option></GalaxySelect></div>
        <p class="gx-note">{{ rows.length }} of {{ snapshot.processes.length }} processes shown · all Linux users included</p>
        <section ref="table" class="gx-card gx-monitor__table" tabindex="0" aria-label="Process table; scroll horizontally for more columns">
          <table><thead><tr><th v-for="column in [['name','Process'],['category','Category'],['cpu','CPU · 10s avg'],['memoryMiB','Memory · 10s avg']]" :key="column[0]" :aria-sort="ariaSort(column[0])"><button type="button" @click="sortBy(column[0])">{{ column[1] }}{{ arrow(column[0]) }}</button></th></tr></thead>
          <tbody><tr v-for="process in rows" :key="identity(process)" :data-instance="identity(process)">
            <td><strong>{{ info(process).title }}</strong><p class="gx-monitor__description">{{ info(process).description }}</p>
              <details :open="!!expanded[identity(process)]" @toggle="expanded[identity(process)] = $event.target.open"><summary>Technical details</summary>
                <div class="gx-note gx-monitor__raw">{{ process.name }}<br>PID {{ process.pid }} · User {{ process.user }}<br>{{ state(process.state) }} ({{ process.state }})<br>Latest CPU: {{ number(process.latestCpu === undefined ? process.cpu : process.latestCpu, '%') }}<br>Latest resident memory: {{ number(process.latestMemory === undefined ? process.memoryMiB : process.latestMemory) }} MiB</div>
              </details></td>
            <td class="gx-monitor__category">{{ info(process).category }}</td>
            <td>{{ number(process.cpu, '%') }}<small class="gx-monitor__coverage">{{ coverage(process) }}</small></td>
            <td>{{ number(process.memoryMiB) }} MiB<small class="gx-monitor__coverage">{{ process.memoryStale ? 'Last measured' : process.memoryCoverage != null && process.memoryCoverage < 9.95 ? number(process.memoryCoverage, '/10s collected') : '' }}</small></td>
          </tr><tr v-if="!rows.length"><td colspan="4">No matching processes.</td></tr></tbody></table>
        </section>
      </template>
    </div>`,
}
