"""Pixel-eye animation engine for Sleep Eyes (pure Python, no raylib).

A 40 x 18 grid of cells. Two "pill" eyes are drawn as solid cells; effects add light
to a background and a foreground buffer (bilinear splats with an optional glow).
Ported from the approved browser mock-up: D:/AI/Claude/Starpilot/sleep-eyes/mockup-v6.html.

Times are in milliseconds. Engine.step(now_ms) returns one RGB tuple per cell, row by row.
"""
from __future__ import annotations

import math
import random
from collections.abc import Callable

C, R = 40, 18
N = C * R

GLYPHS = {
  'star': ["...X...", "..XXX..", "XXXXXXX", ".XXXXX.", "..XXX..", ".XX.XX.", ".X...X."],
  'heart': [".XX.XX.", "XXXXXXX", "XXXXXXX", ".XXXXX.", "..XXX..", "...X..."],
  'gt': ["XX...", ".XX..", "..XX.", "...XX", "..XX.", ".XX..", "XX..."],
  'lt': ["...XX", "..XX.", ".XX..", "XX...", ".XX..", "..XX.", "...XX"],
  'bang': ["X", "X", "X", ".", "X"],
  'z': ["XXX", "..X", ".X.", "X..", "XXX"],
  'spark': [".X.", "XXX", ".X."],
  'note': ["..XX", "..X.", "..X.", "XXX.", "XXX."],
  'play': ["X..", "XX.", "XXX", "XX.", "X.."],
}

ENGAGED = '#b77aff'
DISENGAGED = '#a99ccc'
ASLEEP = '#7a66a8'
AMBER = '#fe8c34'
BG_PURPLE = (23, 15, 31)
BG_BLACK = (0, 0, 0)
RAINBOW = ('#f87171', '#fb923c', '#facc15', '#4ade80', '#38bdf8', '#818cf8', '#c084fc')
CONSTELLATION = ((2.5, 5), (5.5, 1.8), (10.5, 2.6), (15.5, 1), (21.5, 1.8), (27, 0.9), (32.5, 2.6), (36.5, 5.2),
                 (37.5, 10.5), (36, 15.8))

BASE = dict(gx=0.0, gy=0.0, open=1.0, openL=1.0, openR=1.0, scale=1.0, sx=1.0, sy=1.0, lidL=0.0, lidR=0.0, lidB=0.0,
            happy=0.0, tilt=0.0, jx=0.0, jy=0.0, cross=0.0, press=0.0, roll=0.0, stick=0.0, daze=0.0, worry=0.0)
RATE = dict(open=0.55, openL=0.45, openR=0.45, jx=0.35, jy=0.35, gx=0.18, gy=0.18)
NEIGHBOURS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, .5), (1, -1, .5), (-1, 1, .5), (-1, -1, .5))
TAU = 2 * math.pi
_ZEROS = [0.0] * (N * 3)


def clamp(v: float, a: float = -1.0, b: float = 1.0) -> float:
  return a if v < a else b if v > b else v


def tri(t: float, at: float, length: float) -> float:
  """1 outside the window, dipping to 0.05 at its middle (a blink)."""
  d = abs(t - at - length / 2) / (length / 2)
  return 1.0 if d >= 1 else max(0.05, d)


def lin(t: float, a: float, b: float) -> float:
  return clamp((t - a) / (b - a), 0.0, 1.0)


def eo(x: float) -> float:
  return 1 - (1 - x) ** 3


def eio(x: float) -> float:
  return 4 * x * x * x if x < 0.5 else 1 - ((-2 * x + 2) ** 3) / 2


_HEX: dict[str, tuple[float, float, float]] = {}


def rgb(c) -> tuple[float, float, float]:
  if isinstance(c, str):
    v = _HEX.get(c)
    if v is None:
      v = _HEX[c] = (int(c[1:3], 16), int(c[3:5], 16), int(c[5:7], 16))
    return v
  return c


def mix(a, b, k: float) -> tuple[float, float, float]:
  a, b = rgb(a), rgb(b)
  return (a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k)


def plot(buf: list[float], i: int, j: int, c, a: float) -> None:
  if 0 <= i < C and 0 <= j < R and a > 0:
    k = (j * C + i) * 3
    buf[k] += c[0] * a
    buf[k + 1] += c[1] * a
    buf[k + 2] += c[2] * a


def splat(buf: list[float], x: float, y: float, c, a: float) -> None:
  fx, fy = x - 0.5, y - 0.5
  x0, y0 = math.floor(fx), math.floor(fy)
  ax, ay = fx - x0, fy - y0
  plot(buf, x0, y0, c, a * (1 - ax) * (1 - ay))
  plot(buf, x0 + 1, y0, c, a * ax * (1 - ay))
  plot(buf, x0, y0 + 1, c, a * (1 - ax) * ay)
  plot(buf, x0 + 1, y0 + 1, c, a * ax * ay)


def light(buf: list[float], x: float, y: float, c, a: float, glow: float = 0.0) -> None:
  c = rgb(c)
  splat(buf, x, y, c, a)
  if glow:
    for dx, dy, w in NEIGHBOURS:
      splat(buf, x + dx, y + dy, c, a * glow * w)


def glyph_cells(gl: list[str], cx: float, cy: float):
  h, w = len(gl), len(gl[0])
  x0, y0 = round(cx - w / 2), round(cy - h / 2)
  for j, row in enumerate(gl):
    for i, ch in enumerate(row):
      if ch == 'X':
        yield x0 + i, y0 + j


def follow(T: dict, x: float, y: float) -> None:
  T['gx'] = clamp((x - 20) / 14)
  T['gy'] = clamp((y - 9) / 7)


def lids(T: dict, v: float) -> None:
  T['lidL'] = T['lidR'] = v


def yawn(T: dict) -> None:
  lids(T, 0)
  T.update(sy=0.35, sx=1.3, happy=0.7, jy=-1.2, gy=-0.3)


class Eye:
  __slots__ = ('s', 'St', 'w', 'h', 'x', 'y', 'eh', 'lid')

  def __init__(self, s, St, w, h, x, y, eh, lid):
    self.s, self.St, self.w, self.h, self.x, self.y, self.eh, self.lid = s, St, w, h, x, y, eh, lid


def eyes_of(St: dict, stick_dir: int = 1, daze_ph: float = 0.0) -> list[Eye]:
  out = []
  for s in (-1, 1):
    p = 1 + 0.12 * St['gx'] * s
    direction = (1 if St['jx'] > 0 else -1) if St['jx'] != 0 else 1
    outer = s * direction > 0
    pr = St['press'] * (1 if outer else 0.5)
    w = 7 * St['scale'] * St['sx'] * p * (1 - 0.38 * pr)
    h = 10.5 * St['scale'] * St['sy'] * p * (1 + 0.15 * St['press'])
    op = St['open'] * (St['openL'] if s < 0 else St['openR'])
    x = 20 + s * (7.6 - St['cross'] * 3.2) + St['gx'] * 4.2 + St['jx'] + (0 if outer else direction * St['press'] * 2.5)
    y = 9 + St['gy'] * 2.6 + St['jy'] + s * St['roll'] * 1.8
    if St['stick'] > 0.001:
      d = stick_dir
      out_side = s == d
      xs = (38.3 if out_side else 33.4) if d > 0 else (1.7 if out_side else 6.6)
      ws, hs = (3.2, 12.5) if out_side else (3.8, 11.6)
      x += (xs - x) * St['stick']
      w += (ws - w) * St['stick']
      h += (hs - h) * St['stick']
    if St['daze'] > 0.001:
      ph = daze_ph + (2.2 if s > 0 else 0)
      x += math.cos(ph) * 1.3 * St['daze']
      y += math.sin(ph * 1.3) * 0.9 * St['daze']
    out.append(Eye(s, St, w, h, x, y, h * op, St['lidL'] if s < 0 else St['lidR']))
  return out


def pill(e: Eye, px: float, py: float) -> bool:
  St = e.St
  dx = (px - e.x) / (e.w / 2)
  dy = (py - e.y) / (max(e.eh, 0.6) / 2)
  if dx ** 4 + dy ** 4 > 1:
    return False
  top, bot = e.y - e.eh / 2, e.y + e.eh / 2
  if py < top + e.eh * e.lid or py > bot - e.eh * St['lidB']:
    return False
  if St['tilt'] > 0.02 and py < top + e.eh * 0.42 * St['tilt'] + (px - e.x) * 0.5 * (-e.s) * St['tilt']:
    return False
  if St['worry'] > 0.02 and py < top + e.eh * 0.3 * St['worry'] + (px - e.x) * 0.5 * e.s * St['worry']:
    return False
  if St['happy'] > 0.05:
    a = (px - e.x) / (e.w * 0.85)
    b = (py - (e.y + e.eh * 0.62)) / (e.eh * 0.62)
    if a * a + b * b < St['happy']:
      return False
  return True


def each_cell(e: Eye):
  x0 = max(0, math.floor(e.x - e.w / 2 - 1))
  x1 = min(C - 1, math.ceil(e.x + e.w / 2 + 1))
  y0 = max(0, math.floor(e.y - e.eh / 2 - 1))
  y1 = min(R - 1, math.ceil(e.y + e.eh / 2 + 1))
  for j in range(y0, y1 + 1):
    for i in range(x0, x1 + 1):
      if pill(e, i + 0.5, j + 0.5):
        yield i, j


def cells_of(E: list[Eye]) -> list[tuple[int, int]]:
  return [cell for e in E for cell in each_cell(e)]


class Frame:
  """What one animation frame asks for: eye targets, glyph eyes, colour and light."""

  def __init__(self, engine: Engine, T: dict, col_now: str):
    self.engine = engine
    self.T = T
    self.gl: dict[int, tuple[list[str], object]] = {}
    self.col = None
    self.hide = False
    self.shade: Callable[[int, int], float] | None = None
    self.set_jx: float | None = None
    self.col_now = col_now
    self.sol: list[tuple] = []

  def l(self, x, y, c, a=1.0, glow=0.0, layer='b'):  # noqa: E743
    light(self.engine.fr if layer == 'f' else self.engine.bk, x, y, c, a, glow)

  def lg(self, gl, cx, cy, c, a=1.0, glow=0.0, layer='f'):
    h, w = len(gl), len(gl[0])
    x0, y0 = cx - w / 2, cy - h / 2
    buf = self.engine.fr if layer == 'f' else self.engine.bk
    c = rgb(c)
    for j, row in enumerate(gl):
      for i, ch in enumerate(row):
        if ch == 'X':
          light(buf, x0 + i + 0.5, y0 + j + 0.5, c, a, glow)

  def g(self, gl, cx, cy, c):
    self.sol.append(('g', gl, cx, cy, rgb(c)))

  def cell(self, k: int, c):
    self.sol.append(('c', k, rgb(c)))

  def wash(self, c, a):
    c = rgb(c)
    bk = self.engine.bk
    for k in range(N):
      bk[k * 3] += c[0] * a
      bk[k * 3 + 1] += c[1] * a
      bk[k * 3 + 2] += c[2] * a


class Act:
  """One playing animation; per-run state lives on the instance."""

  def __init__(self, name: str, spec: Spec):
    self.name = name
    self.spec = spec


class Spec:
  def __init__(self, label: str, dur: float, run, *, start=None, end=None, prio: int = 0, no_blink=False, no_z=False,
               park=False, event=False, loop: str | None = None, loop_value=None, hold: tuple | None = None):
    self.label, self.dur, self.run, self.start, self.end = label, dur, run, start, end
    self.prio, self.no_blink, self.no_z, self.park, self.event = prio, no_blink, no_z, park, event
    # loop: keep replaying while engine.ctx[loop] == loop_value, and stop as soon as it differs.
    self.loop, self.loop_value = loop, loop_value
    # hold: (ctx key, hold at ms, jump back to ms) keeps a phase going while the context flag is set.
    self.hold = hold


# --- helpers shared by animations -------------------------------------------------------------------------

def sparkles(o: Frame, now: float, col) -> None:
  for i, (x, y) in enumerate(((7, 2), (33, 3), (4, 14), (36, 15), (20, 1.5), (12, 16))):
    s = math.sin(now / 170 + i * 1.7)
    if s > 0.2:
      o.lg(GLYPHS['spark'], x, y, col, s, 0.3)


def zzz(o: Frame, now: float) -> None:
  for k in range(3):
    u = ((now / 1000) * 0.4 + k / 3) % 1
    o.lg(GLYPHS['z'], 29 + k * 2.6 + u * 2, 8 - u * 8, '#cba3ff', (1 - u) * 0.9, 0.1)


def streak(o: Frame, x, y, ang, c, length, a0=0.9) -> None:
  for n in range(length):
    q = n * 0.65
    a = (1 - n / length) ** 1.6
    o.l(x - math.cos(ang) * q, y - math.sin(ang) * q, mix('#ffffff', c, n / (length - 1)), a * a0, 0.35 if n < 2 else 0)


def alert_frame(o: Frame, t: float, soft: bool = False) -> None:
  o.col = AMBER
  a = (0.16 if soft else 0.28) + (0.08 if soft else 0.18) * math.sin(t / 160)
  for i in range(C):
    o.l(i + 0.5, 0.5, AMBER, a)
    o.l(i + 0.5, R - 0.5, AMBER, a)
  for j in range(1, R - 1):
    o.l(0.5, j + 0.5, AMBER, a)
    o.l(C - 0.5, j + 0.5, AMBER, a)


def tri_wave(v: float, lo: float, hi: float) -> float:
  s = hi - lo
  m = v % (2 * s)
  return lo + (m if m < s else 2 * s - m)


def heart_point(th: float) -> tuple[float, float]:
  return (20 + 0.42 * 16 * math.sin(th) ** 3,
          8.6 - 0.36 * (13 * math.cos(th) - 5 * math.cos(2 * th) - 2 * math.cos(3 * th) - math.cos(4 * th)))


def torque_bar_y(x: float) -> float:
  q = (x - 20) / 14
  return 16.4 - 1.9 * q * q


# --- animations ------------------------------------------------------------------------------------------
# Each run(a, e, u, t, o, now, E0): a = this run's Act, e = Engine, u = progress 0..1, t = ms since start,
# o = Frame, E0 = the eyes as drawn last frame.

def r_ign_on(a, e, u, t, o, now, E0):
  T = o.T
  if t < 700:
    lids(T, 0.88); T['gy'] = 0.5; T['jy'] = math.sin(t / 90) * 0.3
  elif t < 1300:
    T['lidL'] = 0.5; T['lidR'] = 0.6 if t > 950 else 0.88; T['gy'] = 0.35
  elif t < 2000:
    lids(T, 0.85 if t < 1650 else 0.45); T['gy'] = 0.3
  elif t < 2700:
    lids(T, 0.4); T['gy'] = 0.2 + 0.15 * math.sin((t - 2000) / 350); T['gx'] = math.sin((t - 2000) / 500) * 0.3
  elif t < 3300:
    lids(T, 0.15); T['open'] = tri(t, 2750, 170) * tri(t, 3050, 170)
  elif t < 4000:
    T.update(sy=1.15, scale=1.08, jy=-0.6)
  else:
    T['gx'] = -0.6 if t < 4350 else 0.6


BOOT_LIGHT_MS = 1300


def r_boot(a, e, u, t, o, now, E0):
  """Comma start-up: black, one pixel blooms into a line, the line splits into two closed eyes, then they wake."""
  if t < BOOT_LIGHT_MS:
    o.hide = True
    o.T.update(gx=0, gy=0)
    if t < 450:
      k = lin(t, 100, 450)
      o.l(20, 9, '#ffffff', k, 0.35 * k, 'f')
    elif t < 900:
      w = eo(lin(t, 450, 900)) * 14
      x = 20 - w
      while x <= 20 + w:
        o.l(x, 9, mix('#ffffff', ENGAGED, abs(x - 20) / 14), 1 - 0.4 * abs(x - 20) / 14, 0, 'f')
        x += 0.5
    else:
      k = eio(lin(t, 900, BOOT_LIGHT_MS))
      for s in (-1, 1):
        cx, half = 20 + s * 7.6 * k, 14 - 11 * k
        x = cx - half
        while x <= cx + half:
          o.l(x, 9 + 5.8 * k, mix('#ffffff', ENGAGED, k), 1, 0, 'f')  # lands on the closed eyelids
          x += 0.5
    return
  lids(o.T, 0.88)
  r_ign_on(a, e, u, t - BOOT_LIGHT_MS, o, now, E0)


def r_engaged(a, e, u, t, o, now, E0):
  if t < 1500:
    c = '#ffe08a' if int(t / 150) % 2 else '#f5b400'
    o.gl[-1] = o.gl[1] = (GLYPHS['star'], c)
    o.T['jy'] = -abs(math.sin(t / 160)) * 1.4
  else:
    o.T['happy'] = 1
  sparkles(o, now, '#f5b400')


def signal(d):
  def run(a, e, u, t, o, now, E0):
    T = o.T
    pulse = max(0.0, math.sin((t % 760) / 760 * TAU))
    T['jx'] = d * (7 + 1.1 * pulse); T['gx'] = d; T['gy'] = -0.1
    if 1500 < t < 1950:
      T['jx'] = d * 2; T['gx'] = 0
  return run


def lane_change(d):
  def run(a, e, u, t, o, now, E0):
    T = o.T
    if t < 1200:
      lids(T, 0.25); T['lidB'] = 0.12
      T['gx'] = d if t < 750 else 0
      T['jx'] = d * 3 if t < 750 else 0
    elif t < 2700:
      jx = d * 26 * eio(lin(t, 1200, 1950)) if t < 1950 else -d * 26 * (1 - eo(lin(t, 1950, 2700)))
      o.set_jx = jx; T['jx'] = jx; T['gx'] = d * 0.6; lids(T, 0.15)
      for k in range(1, 5):
        ghost = dict(e.S); ghost['jx'] = jx - d * k * 1.8
        for i, j in cells_of(eyes_of(ghost)):
          o.l(i + 0.5, j + 0.5, o.col_now, 0.3 / k)
      for n, r in enumerate((4, 9, 14)):
        q = (t * 0.09 + n * 17) % (C + 10)
        x = C + 5 - q if d > 0 else q - 5
        for m in range(5):
          o.l(x + d * m, r + 0.5, '#c4b5fd', 0.3 - m * 0.05)
    else:
      T['happy'] = 1
  return run


def lateral(d):
  def start(a, e):
    e.stick_dir = d

  def run(a, e, u, t, o, now, E0):
    T = o.T
    e.stick_dir = d
    e.daze_ph = t / 170
    if t < 1900:
      T['stick'] = 1; lids(T, 0.28); T['lidB'] = 0.18; T['gx'] = -d * 0.5; T['jy'] = math.sin(t / 38) * 0.18
      for k, r in enumerate((3, 7, 11, 15)):
        q = (t * 0.06 + k * 11) % (C + 8)
        x = q - 4 if d > 0 else C + 4 - q
        for n in range(3):
          o.l(x - d * n, r + 0.5, '#cba3ff', 0.5 - n * 0.15)
    elif t < 4200:
      T['daze'] = lin(t, 1900, 2300) if t < 2300 else 1
      lids(T, 0.42); T['gy'] = 0.15; T['gx'] = math.sin(t / 260) * 0.5
      if t > 2100:
        env = lin(t, 2100, 2400) * (1 - lin(t, 3900, 4200))
        for k in range(3):
          ang = t / 280 + k * 2.094
          o.lg(GLYPHS['spark'], 20 + math.cos(ang) * 9, 2.4 + math.sin(ang) * 1.4, '#f5b400',
               env * (1 if math.sin(ang) > 0 else 0.45), 0.25)
    else:
      T['jx'] = math.sin((t - 4200) / 45) * 2 * math.exp(-(t - 4200) / 220)
      T['open'] = tri(t, 4450, 170) * tri(t, 4720, 170)
  return start, run


def r_accel(a, e, u, t, o, now, E0):
  T = o.T
  if t < 1700:
    o.gl[-1] = (GLYPHS['gt'], None); o.gl[1] = (GLYPHS['lt'], None); T['jy'] = -1
    for k, r in enumerate((2, 4, 14, 16)):
      x = C + 4 - ((t * 0.05 + k * 13) % (C + 10))
      for q in range(4):
        o.l(x + q, r + 0.5, '#cba3ff', 0.7 - q * 0.16)
  else:
    T['happy'] = 1


def r_brake(a, e, u, t, o, now, E0):
  T = o.T
  if t < 1100:
    T['scale'] = 1.3; T['gy'] = -0.1
    if t < 160:
      T['jy'] = 3.5
    if int(t / 120) % 2 == 0:
      o.g(GLYPHS['bang'], 20, 2.5, '#ffa352')
  else:
    T['open'] = tri(t, 1250, 180)


def s_disengaged(a, e):
  e.engaged = False
  a.parts = None


def r_disengaged(a, e, u, t, o, now, E0):
  T = o.T
  if t < 1300:
    o.hide = True
    if a.parts is None:
      src = cells_of(E0)
      tgt = cells_of(eyes_of(dict(BASE)))
      a.parts = []
      for k, (tx, ty) in enumerate(tgt):
        sx, sy = src[(k * 7) % len(src)] if src else (20, 9)
        ang, sp = e.rng.random() * TAU, 5 + e.rng.random() * 11
        a.parts.append((sx + 0.5, sy + 0.5, math.cos(ang) * sp, math.sin(ang) * sp * 0.6, tx + 0.5, ty + 0.5, e.rng.random() * 6))
    for sx, sy, vx, vy, tx, ty, ph in a.parts:
      if t < 550:
        k = eo(t / 550)
        x, y = sx + vx * k, sy + vy * k
        c = '#ffffff' if t < 90 else ('#b77aff' if math.sin(t / 60 + ph) > 0 else '#e9d5ff')
      else:
        k = eio(lin(t, 550, 1300))
        mx, my = sx + vx, sy + vy
        x, y = mx + (tx - mx) * k, my + (ty - my) * k
        c = mix(ENGAGED, DISENGAGED, k)
      o.l(x, y, c, 1, 0.12, 'f')
  else:
    T['gx'] = 0.45; T['gy'] = 0.55
    if 1700 < t < 2500:
      T['gy'] = 0.5 + 0.3 * math.sin((t - 1700) / 400 * TAU)


def e_ign_off(a, e):
  e.parked = True
  e.engaged = False


def r_ign_off(a, e, u, t, o, now, E0):
  T = o.T
  if t < 1300:
    yawn(T)
  elif t < 2200:
    lids(T, 0.5); T['gy'] = 0.3
  elif t < 2800:
    lids(T, 0.82); T['gy'] = 0.6
  elif t < 3050:
    lids(T, 0.05); T['jy'] = -1.2; T['scale'] = 1.08
  elif t < 3800:
    lids(T, 0.5); T['open'] = tri(t, 3300, 400); T['gy'] = 0.2
  else:
    lids(T, 0.6 + 0.28 * lin(t, 3800, 4800)); T['gy'] = 0.5
  if t > 4500:
    zzz(o, now)


# Orange alerts
def r_al_attention(a, e, u, t, o, now, E0):
  T = o.T
  alert_frame(o, t)
  T['tilt'] = 0.5
  if t % 1400 < 800:
    T.update(gx=0.45, gy=0.55, scale=1.12)
  else:
    T.update(gx=0, gy=-0.95)


def r_al_wheel(a, e, u, t, o, now, E0):
  T = o.T
  alert_frame(o, t)
  T.update(gx=0.3, gy=1, scale=1.15, jy=1.6 if t % 500 < 150 else 0)


def r_al_take(a, e, u, t, o, now, E0):
  T = o.T
  alert_frame(o, t)
  T.update(worry=1, scale=1.2, gx=0.45, gy=0.4, jx=math.sin(t / 30) * 0.4)


def r_al_ldw(a, e, u, t, o, now, E0):
  T = o.T
  d = e.ctx.get('alert_dir') or -1
  alert_frame(o, t)
  T['worry'] = 0.6
  if t < 700:
    T['gx'] = d; T['jx'] = 4 * d
  else:
    k = t - 700
    T['jx'] = 4 * d * math.exp(-k / 300) * math.cos(k / 110)
    T['gx'] = 0.6 * d * math.exp(-k / 500)


def r_al_blind(a, e, u, t, o, now, E0):
  T = o.T
  d = e.ctx.get('alert_dir') or -1
  alert_frame(o, t)
  T['gx'] = d
  if t < 300:
    T['jx'] = 3 * d; T['scale'] = 1.3
  elif t < 2300:
    T['scale'] = 1.25
    T['jx'] = 3 * d + math.sin((t - 300) / 70) * 1.5 * math.exp(-(t - 300) / 900)
  else:
    T['jx'] = 2 * d; lids(T, 0.3)


def r_al_steer(a, e, u, t, o, now, E0):
  T = o.T
  d = e.ctx.get('alert_dir') or 1
  alert_frame(o, t)
  T.update(worry=1, scale=0.88, jy=-2.4)  # raised so the eyes clear the bar
  f = eo(lin(t, 0, 900))
  x = 6.0
  while x <= 34:
    o.l(x, torque_bar_y(x), '#3b2a4d', 0.6)
    x += 0.5
  xe = 20 + d * 14 * f
  x = 20.0
  while (x <= xe) if d > 0 else (x >= xe):
    fr = abs(x - 20) / 14
    c = mix('#e5e7eb', AMBER, fr * 2) if fr < 0.5 else mix(AMBER, '#ef4444', (fr - 0.5) * 2)
    fl = (1 if math.sin(t / 60) > 0 else 0.45) if f >= 1 and fr > 0.85 else 1
    o.l(x, torque_bar_y(x), c, fl, 0.3 if fr > 0.85 else 0, 'f')
    x += d * 0.5
  ph = t % 1700
  if t < 900 or ph < 600:
    T['gx'] = 0.45; T['gy'] = 0.4
  else:
    follow(T, xe, torque_bar_y(xe))
    T['gy'] = 1
    T['jy'] = -2.4 + (1.0 if (ph - 600) % 380 < 130 else 0)
    if ph > 700:
      o.lg(GLYPHS['spark'], xe, torque_bar_y(xe) - 2, '#ffe08a', 0.9 if math.sin(t / 90) > 0 else 0.3, 0.2)


def r_al_resume(a, e, u, t, o, now, E0):
  T = o.T
  p = 0.6 + 0.4 * math.sin(t / 200)
  alert_frame(o, t, True)
  T.update(gx=0.45, gy=0.5, happy=0.3, jy=-abs(math.sin(t / 250)))
  o.lg(GLYPHS['play'], 20.5, 3, AMBER, p, 0.2)


# Transitions to and from the driving view
def s_morph_in(a, e):
  a.parts = None


def r_morph_in(a, e, u, t, o, now, E0):
  if a.parts is None:
    tgt = cells_of(eyes_of(dict(BASE)))
    scene = e.scene_cells or [(20, 20, 40)] * N
    src = [((k % C) + 0.5, (k // C) + 0.5, scene[k]) for k in range(N)]
    e.rng.shuffle(src)
    a.parts = []
    for k, (x, y, c) in enumerate(src):
      if k < len(tgt):
        a.parts.append((x, y, c, tgt[k][0] + 0.5, tgt[k][1] + 0.5, e.rng.random() * 350))
      else:
        a.parts.append((x, y, c, None, None, e.rng.random() * 400))
  if t < 1150:
    o.hide = True
    for x, y, c, tx, ty, d in a.parts:
      if tx is None:
        k = lin(t, d, d + 500)
        if k < 1:
          o.l(x, y + k * 2, c, 1 - k, 0, 'f')
      else:
        k = eio(lin(t, d, d + 750))
        o.l(x + (tx - x) * k, y + (ty - y) * k, mix(c, o.col_now, k), 1, 0, 'f')
  else:
    o.T['open'] = tri(t, 1250, 200) * tri(t, 1550, 200)


def e_morph_out(a, e):
  e.exit_done = True


def r_morph_out(a, e, u, t, o, now, E0):
  if t < 260:
    o.T['open'] = 0.05
    return
  o.hide = t > 330
  if t < 420:
    w = lin(t, 260, 420) * 20
    x = 20 - w
    while x <= 20 + w:
      o.l(x, 9, '#ffffff', 0.8, 0, 'f')
      x += 0.5
  r = lin(t, 260, 950) * 24
  scene = e.scene_cells
  if scene:
    for k in range(N):
      x, y = (k % C) + 0.5, (k // C) + 0.5
      if math.hypot((x - 20) * 0.55, (y - 9) * 1.1) < r:
        o.cell(k, scene[k])


# Bored while stopped and engaged
def r_sigh(a, e, u, t, o, now, E0):
  T = o.T
  if t < 1000:
    T['sy'] = 1.08; T['jy'] = -0.8; lids(T, 0.15)
  elif t < 2600:
    lids(T, 0.5); T.update(gy=0.45, jy=0.8, sx=1.06)
    if t < 1900:
      k = lin(t, 1000, 1900)
      for s in (-1, 1):
        for m in (0, 1):
          o.l(20 + s * (1 + k * 5 + m), 15 + k * 2 + m * 0.5, '#cba3ff', (1 - k) * 0.5, 0.15, 'f')


def s_hum(a, e):
  a.notes = [(i * 650, 25 + e.rng.random() * 6, '#f9a8d4' if i % 2 else '#cba3ff') for i in range(6)]


def r_hum(a, e, u, t, o, now, E0):
  T = o.T
  T.update(happy=0.6, jy=-abs(math.sin(t / 500 * math.pi)) * 1.1, gx=math.sin(t / 1000) * 0.4)
  for t0, x0, c in a.notes:
    lt = (t - t0) / 1700
    if 0 < lt < 1:
      o.lg(GLYPHS['note'], x0 + lt * 5 + math.sin(lt * 6) * 0.6, 13 - lt * 12, c, math.sin(lt * math.pi), 0.15)


def r_pong(a, e, u, t, o, now, E0):
  env = 1 - lin(t, 4600, 5200)
  for k in range(6, -1, -1):
    q = t - k * 28
    x, y = tri_wave(q * 0.016 + 3, 1.2, 38.8), tri_wave(q * 0.011 + 2, 1.2, 16.8)
    o.l(x, y, '#cba3ff' if k else '#fdf4ff', env * (0.45 * (1 - k / 7) if k else 1), 0 if k else 0.35, 'f')
    if not k:
      follow(o.T, x, y)


def r_orbit(a, e, u, t, o, now, E0):
  T = o.T
  ang = t / 650
  T.update(jx=math.cos(ang) * 3, jy=math.sin(ang) * 1.6, gx=-math.sin(ang) * 0.6, gy=math.cos(ang) * 0.4)
  lids(T, 0.35)


def r_nod(a, e, u, t, o, now, E0):
  T = o.T
  if t < 2300:
    lids(T, 0.3 + 0.45 * lin(t, 0, 2300)); T['gy'] = 0.2 + 0.3 * lin(t, 0, 2300)
  elif t < 2600:
    T['scale'] = 1.18; T['jy'] = -1.2
  else:
    T['open'] = tri(t, 2800, 160) * tri(t, 3100, 160)


def r_doodle(a, e, u, t, o, now, E0):
  T = o.T
  env = 1 - lin(t, 5600, 6800)
  th = TAU * lin(t, 0, 3600)
  T['scale'] = 0.8
  q = 0.0
  while q <= th:
    x, y = heart_point(q)
    al = 0.8 * env
    if t > 3600:
      al *= 0.65 + 0.35 * math.sin(q * 8 - t / 110)
    o.l(x, y, '#f0abfc', al, 0.08, 'f')
    q += 0.05
  px, py = heart_point(th)
  if t < 3600:
    o.l(px, py, '#ffffff', 1, 0.4, 'f')
    follow(T, px, py)
  else:
    T['happy'] = 1


# Idle actions
def r_blink(a, e, u, t, o, now, E0):
  o.T['open'] = tri(t, 0, 190)


def r_double_blink(a, e, u, t, o, now, E0):
  o.T['open'] = tri(t, 0, 170) * tri(t, 290, 170)


def r_look_around(a, e, u, t, o, now, E0):
  p = ((-0.9, -0.4), (0.85, -0.35), (0.6, 0.55), (-0.3, 0.2))[min(3, int(t / 700))]
  o.T['gx'], o.T['gy'] = p


def r_roll(a, e, u, t, o, now, E0):
  ang = math.pi * (1 - lin(t, 0, 1100))
  if t < 1100:
    o.T['gx'] = math.cos(ang); o.T['gy'] = -math.sin(ang) * 0.95
  else:
    lids(o.T, 0.32)


def r_squint(a, e, u, t, o, now, E0):
  lids(o.T, 0.4); o.T['lidB'] = 0.35


def r_yawn(a, e, u, t, o, now, E0):
  if t < 1300:
    yawn(o.T)
  else:
    o.T['open'] = tri(t, 1450, 380)


def s_side(a, e):
  a.d = -1 if e.rng.random() < 0.5 else 1


def r_side(a, e, u, t, o, now, E0):
  if t < 1800:
    o.T['gx'] = a.d; lids(o.T, 0.5); o.T['gy'] = 0.1


def r_cross(a, e, u, t, o, now, E0):
  if t < 1400:
    o.T['cross'] = 1; o.T['gy'] = 0.25


def r_wiggle(a, e, u, t, o, now, E0):
  o.T['happy'] = 1; o.T['jy'] = -abs(math.sin(t / 110)) * 1.5


def r_peek(a, e, u, t, o, now, E0):
  if 400 < t < 2100:
    o.T['lidR'] = 0; o.T['gx'] = math.sin(t / 400) * 0.7; o.T['gy'] = 0.1


# Random events
def peek_right(e, T):
  if e.parked:
    T['lidR'] = 0


def r_aurora(a, e, u, t, o, now, E0):
  env = lin(t, 0, 1500) * (1 - lin(t, 6800, 8500))
  for x in range(C):
    xc = x + 0.5
    yc = 3.2 + 1.3 * math.sin(xc * 0.22 + t * 0.0013) + 0.8 * math.sin(xc * 0.09 - t * 0.0021)
    hue = 0.5 + 0.5 * math.sin(xc * 0.12 + t * 0.0008)
    top_c, low_c = mix('#f472b6', '#a855f7', hue), mix('#34d399', '#a855f7', hue)
    for y in range(max(0, math.floor(yc - 1.5)), min(R, math.ceil(yc + 7))):
      d = y + 0.5 - yc
      inten = max(0.0, 1 + d * 1.1) if d < 0 else math.exp(-d / 2.3)
      sh = 0.55 + 0.45 * math.sin(xc * 0.7 + t * 0.005 + y * 0.3)
      o.l(xc, y + 0.5, top_c if d < 0 else low_c, env * inten * sh * 0.7)
  o.T.update(gy=-0.75, scale=1.06, gx=0.45 * math.sin(t / 1800), happy=0.25 * env)
  o.col = mix(o.col_now, '#6ee7b7', 0.25 * env)
  peek_right(e, o.T)


def s_fireworks(a, e):
  rr = e.rng.random
  a.rockets = [dict(t0=t0, x=lo + rr() * (hi - lo), c=c, ty=3 + rr() * 3, parts=None)
               for t0, lo, hi, c in ((0, 6, 13, '#f5b400'), (1300, 27, 34, '#f472b6'), (2500, 17, 23, '#a855f7'),
                                     (3300, 9, 31, '#38bdf8'))]


def r_fireworks(a, e, u, t, o, now, E0):
  rr = e.rng.random
  fx, flash, fc, burst = None, 0.0, None, False
  for r in a.rockets:
    lt = t - r['t0']
    if lt < 0:
      continue
    if lt < 700:
      k = eo(lt / 700)
      x, y = r['x'] + math.sin(lt / 60) * 0.15, 18 - (18 - r['ty']) * k
      o.l(x, y, '#fff7d6', 1, 0.3)
      for m in range(1, 4):
        o.l(x + (rr() - 0.5) * 0.6, y + m * 0.7, '#fdba74', 0.5 / m)
      fx = (x, y)
    else:
      if r['parts'] is None:
        r['parts'] = []
        for k in range(24):
          ang, sp = k / 24 * TAU + (rr() - 0.5) * 0.2, 6 + rr() * 3.5
          r['parts'].append((math.cos(ang) * sp, math.sin(ang) * sp * 0.8, rr() * 6))
      s = (lt - 700) / 1000
      if s < 1.5:
        for vx, vy, twk in r['parts']:
          x = r['x'] + vx * s * (1 - s * 0.25)
          y = r['ty'] + vy * s * (1 - s * 0.25) + 3.2 * s * s
          al = max(0.0, 1 - s / 1.5) ** 1.2
          if s > 0.7:
            al *= 1 if math.sin(t / 45 + twk) > 0 else 0.25
          o.l(x, y, mix('#ffffff', r['c'], min(1.0, s * 3)), al, 0.25)
        if s < 0.18 and 1 - s / 0.18 > flash:
          flash, fc = 1 - s / 0.18, r['c']
        if s < 0.8:
          fx, burst = (r['x'], r['ty']), True
  if flash > 0:
    o.wash(fc, 0.12 * flash)
  if fx:
    follow(o.T, *fx)
  if burst:
    o.T['scale'] = 1.18
  if fc:
    o.col = mix(o.col_now, fc, 0.45 * flash)
  peek_right(e, o.T)


def s_snow(a, e):
  rr = e.rng.random
  a.flakes = [dict(x0=rr() * 40, d=rr() * 4200, sp=2 + rr() * 1.6, am=0.4 + rr() * 0.7, ph=rr() * 6.3, st=None, rel=None)
              for _ in range(34)]


def r_snow(a, e, u, t, o, now, E0):
  T = o.T
  for f in a.flakes:
    lt = (t - f['d']) / 1000
    if lt < 0:
      continue
    al = 0.9
    if f['rel']:
      r = (t - f['rel'][2]) / 1000
      x, y = f['rel'][0] + f['rel'][3] * r, f['rel'][1] + 4 * r * r + 1.5 * r
    elif f['st']:
      eye = E0[0 if f['st'][0] < 0 else 1]
      x, y = eye.x + f['st'][1], eye.y + f['st'][2]
      if t > 5800:
        f['rel'] = (x, y, t, (e.rng.random() - 0.5) * 4)
    else:
      y = -1 + f['sp'] * lt
      x = f['x0'] + math.sin(lt * 1.6 + f['ph']) * f['am']
      if t < 5600:
        for eye in E0:
          if pill(eye, math.floor(x) + 0.5, math.floor(y) + 0.5):
            f['st'] = (eye.s, x - eye.x, y - eye.y)
            break
    if y > R + 1:
      continue
    if t > 7300:
      al *= 1 - lin(t, 7300, 8200)
    o.l(x, y, '#eef2ff', al, 0.12, 'f')
  if t < 5800:
    T['gy'] = -0.45; T['happy'] = 0.35
  elif t < 6300:
    T['jx'] = math.sin(t / 30) * 1.4; T['open'] = tri(t, 5850, 200)
  else:
    T['happy'] = 0.6
  peek_right(e, T)


def s_rainbow(a, e):
  a.drops = [(e.rng.random() * 46, e.rng.random() * 700) for _ in range(46)]


def r_rainbow(a, e, u, t, o, now, E0):
  T = o.T
  if t < 3300:
    ra = 1 - lin(t, 2800, 3300)
    for x0, dl in a.drops:
      if t < dl:
        continue
      lt = ((t - dl) % 700) / 1000
      y, x = -1 + lt * 30, x0 - lt * 7
      o.l(x, y, '#93c5fd', 0.5 * ra)
      o.l(x + 0.23, y - 1, '#93c5fd', 0.22 * ra)
  env = lin(t, 3000, 4200) * (1 - lin(t, 7000, 8500))
  rev = lin(t, 3000, 4800)
  if env > 0:
    for j in range(R):
      for i in range(C):
        dx, dy = i + 0.5 - 20, j + 0.5 - 21.5
        band = 18.2 - math.hypot(dx, dy)
        if 0 <= band < 7 and math.atan2(-dy, dx) >= math.pi * (1 - rev):
          o.l(i + 0.5, j + 0.5, RAINBOW[int(band)], env * 0.55)
  if t < 3300:
    lids(T, 0.3); T['gy'] = -0.35; T['tilt'] = 0.3
  else:
    T['gy'] = -0.2
    if t > 3800:
      T['happy'] = 1
  peek_right(e, T)


def r_butterfly(a, e, u, t, o, now, E0):
  x = 20 + 15 * math.sin(t / 1300 + 1.2)
  y = 7 + 4 * math.sin(t / 700) + 1.2 * math.sin(t / 230)
  x = -3 + (x + 3) * eio(lin(t, 0, 1000))
  eye = E0[1]
  lx, ly = eye.x, eye.y - eye.eh / 2 - 0.7
  land = eio(lin(t, 3200, 3800)) * (1 - eio(lin(t, 5400, 5900)))
  x += (lx - x) * land
  y += (ly - y) * land
  if t > 5400:
    k = eio(lin(t, 5400, 7600))
    x += (44 - x) * k
    y += (-4 - y) * k
  f = abs(math.sin(t / (260 if land > 0.9 else 70)))
  sp = 0.4 + 1.5 * f
  o.l(x, y - 0.3, '#f5f3ff', 0.9, 0, 'f')
  o.l(x, y + 0.5, '#f5f3ff', 0.6, 0, 'f')
  for s in (-1, 1):
    o.l(x + s * sp, y - 0.7, '#f472b6', 0.95, 0.12, 'f')
    o.l(x + s * sp * 0.55, y - 1.2, '#f9a8d4', 0.7, 0, 'f')
    o.l(x + s * sp * 0.75, y + 0.6, '#fb923c', 0.85, 0.1, 'f')
  if land > 0.9:
    o.T.update(gx=0.35, gy=-1, lidR=0.12, happy=0.35, jy=math.sin(t / 90) * 0.25)
  else:
    follow(o.T, x, y)
  peek_right(e, o.T)


def s_bubbles(a, e):
  rr = e.rng.random
  a.bubbles = [dict(x0=20.5, d=1500, sp=2.6, r=1.5, pop=9.2, w=0, pt=None, px=0)]
  a.bubbles += [dict(x0=3 + rr() * 34, d=rr() * 5200, sp=2.6 + rr() * 1.6, r=0.8 + rr() * 0.7, pop=0.5 + rr() * 6.5,
                     w=rr() * 6, pt=None, px=0) for _ in range(8)]


def r_bubbles(a, e, u, t, o, now, E0):
  T = o.T
  tg = None
  for n, b in enumerate(a.bubbles):
    lt = (t - b['d']) / 1000
    if lt < 0:
      continue
    y = 19 - b['sp'] * lt
    x = b['x0'] + math.sin(lt * 2.2 + b['w']) * 0.7
    if y > b['pop'] and b['pt'] is None:
      for k in range(12):
        ang = k / 12 * TAU
        o.l(x + math.cos(ang) * b['r'], y + math.sin(ang) * b['r'] * 0.95,
            mix('#7dd3fc', '#f0abfc', 0.5 + 0.5 * math.sin(ang * 2 + t / 300)), 0.5, 0, 'f')
      o.l(x - b['r'] * 0.45, y - b['r'] * 0.5, '#ffffff', 0.85, 0, 'f')
      o.l(x, y, '#7dd3fc', 0.08, 0, 'f')
      if tg is None or n == 0:
        tg = (x, y, n)
    else:
      if b['pt'] is None:
        b['pt'], b['px'] = t, x
      pl = (t - b['pt']) / 380
      if pl < 1:
        for k in range(8):
          ang = k / 8 * TAU
          rr = b['r'] + pl * 1.6
          o.l(b['px'] + math.cos(ang) * rr, b['pop'] + math.sin(ang) * rr, '#e0f2fe', 1 - pl, 0, 'f')
  b0 = a.bubbles[0]
  if b0['pt'] is not None and t - b0['pt'] < 450:
    T['open'] = tri(t, b0['pt'], 220); T['jy'] = -0.6
  elif tg and tg[2] == 0 and tg[1] < 15:
    T.update(cross=0.9, gy=0.4, gx=0)
  elif tg:
    follow(T, tg[0], tg[1])
  peek_right(e, T)


def r_eclipse(a, e, u, t, o, now, E0):
  T = o.T
  env = lin(t, 0, 1200) * (1 - lin(t, 8000, 9500))
  cx, cy, rad = 20, 5, 4.2
  if t < 1500:
    mx = 7
  elif t < 3700:
    mx = 7 + 13 * eio(lin(t, 1500, 3700))
  elif t < 5200:
    mx = 20
  else:
    mx = 20 + 13 * eio(lin(t, 5200, 7600))
  tot = max(0.0, 1 - abs(mx - cx) / 3.5)
  for j in range(12):
    for i in range(8, 32):
      x, y = i + 0.5, j + 0.5
      d = math.hypot(x - cx, y - cy)
      if d > rad + 3.5:
        continue
      dm = math.hypot(x - mx, y - cy)
      if d < rad and dm > rad:
        o.l(x, y, '#fdf4ff', env * 0.85 * (0.75 + 0.25 * (1 - d / rad)))
      rim = math.exp(-((dm - rad) ** 2) / 0.35) * (1 if d < rad + 0.7 else 0)
      if rim > 0.02:
        o.l(x, y, '#c084fc', env * 0.95 * rim)
      cor = tot * (math.exp(-((d - rad - 0.7) ** 2) / 1.4) * 0.75 + (0.25 * math.exp(-(d - rad) / 1.6) if d > rad else 0))
      if cor > 0.02:
        o.l(x, y, '#c084fc' if d < rad + 1.6 else '#7e22ce', env * cor)
  if tot > 0.6:
    rr = random.Random(int(t / 110) * 977).random
    for _ in range(3):
      ang, r = rr() * TAU, rad + 0.9
      for s in range(5):
        ang += (rr() - 0.5) * 0.9
        r += 0.85
        o.l(cx + math.cos(ang) * r, cy + math.sin(ang) * r * 0.9, '#e9d5ff', 0.85 * env * (1 - s / 6), 0.2)
  T.update(scale=0.6 + 0.1 * tot, jy=6, gy=-1, gx=clamp((mx - 20) / 12) * 0.6)
  o.col = mix(o.col_now, '#c084fc', 0.5 * tot)
  peek_right(e, T)


def make_bolt(rng: random.Random, x: float):
  pts = []
  y = 0
  while y < 11.5:
    pts.append((x, y + 0.5))
    y += 1
    x += (rng.random() - 0.5) * 2.2
  fx, fy = pts[4][0], 4.5
  direction = -1 if rng.random() < 0.5 else 1
  fork = []
  for _ in range(4):
    fx += direction * (0.4 + rng.random() * 0.9)
    fy += 1
    fork.append((fx, fy))
  return pts, fork


def s_storm(a, e):
  a.strikes = [(t0, make_bolt(e.rng, 6 + e.rng.random() * 28)) for t0 in (700, 2500, 2750, 4700)]
  a.drops = [(e.rng.random() * 46, e.rng.random() * 700) for _ in range(30)]


def r_storm(a, e, u, t, o, now, E0):
  T = o.T
  env = 1 - lin(t, 6000, 6800)
  for x in range(C):
    for y in range(2):
      o.l(x + 0.5, y + 0.5, '#4c1d95', env * (0.25 + 0.12 * math.sin(x * 0.5 + t / 500 + y)))
  for x0, dl in a.drops:
    if t < dl:
      continue
    lt = ((t - dl) % 700) / 1000
    o.l(x0 - lt * 7, -1 + lt * 30, '#93c5fd', 0.3 * env)
  fl, bx = 0.0, None
  for t0, (pts, fork) in a.strikes:
    lt = t - t0
    if lt < 0 or lt > 320:
      continue
    al = 1 if lt < 60 else math.exp(-(lt - 60) / 80)
    for x, y in pts:
      o.l(x, y, '#ffffff', al, 0.35)
    for x, y in fork:
      o.l(x, y, '#d8b4fe', al * 0.8, 0.25)
    fl = max(fl, 1 - lt / 120 if lt < 120 else 0)
    bx = pts[5][0]
  if fl > 0:
    o.wash('#a78bfa', 0.22 * fl)
  if fl > 0.5:
    T['open'] = 0.15; T['jy'] = -1
  else:
    lids(T, 0.3); T['gy'] = -0.5; T['scale'] = 1.05
  if bx is not None:
    T['gx'] = clamp((bx - 20) / 14)
  peek_right(e, T)


def r_constellation(a, e, u, t, o, now, E0):
  env = 1 - lin(t, 7200, 8500)
  last = 0
  for i, (x, y) in enumerate(CONSTELLATION):
    ti = 300 + i * 480
    if t < ti - 350:
      continue
    if i > 0:
      px, py = CONSTELLATION[i - 1]
      lp = lin(t, ti - 350, ti)
      length = math.hypot(x - px, y - py)
      q = 0.0
      while q <= length * lp:
        k = q / length
        o.l(px + (x - px) * k, py + (y - py) * k, '#a78bfa', 0.28 * env)
        q += 0.4
    if t >= ti:
      last = i
      ap = lin(t, ti, ti + 250)
      fl = 1.4 - 0.4 * ap if ap < 1 else 1
      twk = 0.75 + 0.25 * math.sin(t / 260 + i * 1.9)
      o.l(x, y, '#f5f3ff', env * min(1.0, fl * twk), 0.35 * fl)
  if t > 5300:
    pos = ((t - 5300) / 1400) % 1 * (len(CONSTELLATION) - 1)
    i = int(pos)
    k = pos - i
    if i < len(CONSTELLATION) - 1:
      (ax, ay), (bx, by) = CONSTELLATION[i], CONSTELLATION[i + 1]
      o.l(ax + (bx - ax) * k, ay + (by - ay) * k, '#ffffff', env, 0.4)
    o.T['happy'] = 0.8
  else:
    follow(o.T, *CONSTELLATION[last])
  peek_right(e, o.T)


def s_shoot(a, e):
  a.dust = []
  a.y0 = 0.5 + e.rng.random() * 2.5
  a.dy = 4 + e.rng.random() * 3


def r_shoot(a, e, u, t, o, now, E0):
  k = min(1.0, t / 1500)
  x, y = -3 + k * 48, a.y0 + k * a.dy
  if t < 1500:
    streak(o, x, y, math.atan2(a.dy, 48), '#c084fc', 12)
    if e.rng.random() < 0.6:
      a.dust.append((x - e.rng.random() * 1.5, y + (e.rng.random() - 0.5) * 1.2, t, 0.2 + e.rng.random() * 0.6))
    follow(o.T, x, y)
  a.dust = [p for p in a.dust if t - p[2] < 900]
  for px, py, t0, vy in a.dust:
    lt = (t - t0) / 900
    o.l(px, py + lt * vy, '#e9d5ff', (1 - lt) * (0.8 if math.sin(t / 40 + px * 3) > 0 else 0.3))
  peek_right(e, o.T)


def r_meteor(a, e, u, t, o, now, E0):
  for s, y0, x0, c in ((0, 0.5, 0, '#c084fc'), (600, 3, 6, '#f0abfc'), (1100, 1.5, -5, '#fde68a'), (1800, 4.5, 3, '#7dd3fc')):
    v = (t - s) / 1400
    if 0 < v < 1:
      x, y = x0 - 3 + v * 48, y0 + v * 5
      streak(o, x, y, math.atan2(5, 48), c, 9)
      follow(o.T, x, y)
  peek_right(e, o.T)


def s_fireflies(a, e):
  rr = e.rng.random
  a.flies = [(8 + rr() * 10, 3 + rr() * 3, 0.0004 + rr() * 0.0005, 0.0006 + rr() * 0.0006, rr() * 6.3, rr() * 6.3,
              rr() * 6.3, 0.7 + rr() * 0.6) for _ in range(6)]


def r_fireflies(a, e, u, t, o, now, E0):
  T = o.T
  env = lin(t, 0, 800) * (1 - lin(t, 6800, 7600))
  landed, f0 = False, (20, 9)
  for n, (ax, ay, fx, fy, px, py, ph, sp) in enumerate(a.flies):
    x, y = 20 + ax * math.sin(t * fx + px), 9 + ay * math.sin(t * fy + py)
    al = 0.2 + 0.8 * max(0.0, math.sin(t / 380 * sp + ph)) ** 2
    if n == 0:
      land = lin(t, 3400, 3900) * (1 - lin(t, 5400, 5900))
      x += (20 - x) * land
      y += (13.6 - y) * land
      if land > 0.9:
        landed, al = True, 0.6 + 0.4 * math.sin(t / 200)
      f0 = (x, y)
    o.l(x, y, '#d9f99d', al * env, 0.45, 'f')
  if landed:
    T.update(cross=1, gy=0.8, gx=0)
  else:
    follow(T, *f0)
  peek_right(e, T)


def s_heart(a, e):
  a.sp = []


def r_heart(a, e, u, t, o, now, E0):
  x, y = 20 + math.sin(t / 320) * 1.3, 21 - u * 28
  o.lg(GLYPHS['heart'], x, y, '#ec4899', 0.75 + 0.25 * math.sin(t / 120), 0.25)
  if e.rng.random() < 0.4:
    a.sp.append((x + (e.rng.random() - 0.5) * 6, y + 3, t))
  a.sp = [s for s in a.sp if t - s[2] < 700]
  for sx, sy, t0 in a.sp:
    lt = (t - t0) / 700
    o.l(sx, sy + lt * 1.5, '#f9a8d4', (1 - lt) * 0.8, 0, 'f')
  follow(o.T, x, y)
  if 0.35 < u < 0.7 and not e.parked:
    o.gl[-1] = o.gl[1] = (GLYPHS['heart'], '#ec4899')
  peek_right(e, o.T)


def r_planet(a, e, u, t, o, now, E0):
  cx, cy, r, tl = -6 + u * 52, 2.8, 2.3, -0.25
  for j in range(7):
    for i in range(math.floor(cx - 4), math.floor(cx + 4) + 1):
      x, y = i + 0.5, j + 0.5
      d = math.hypot(x - cx, y - cy)
      if d > r + 0.4:
        continue
      nx, ny = (x - cx) / r, (y - cy) / r
      sh = clamp(0.5 + 0.5 * (-nx * 0.6 - ny * 0.6), 0, 1)
      band = 0.5 + 0.5 * math.sin((y - cy) * 2.6)
      o.l(x, y, mix('#3730a3', '#7dd3fc', sh * 0.8 + band * 0.2), clamp(r + 0.4 - d, 0, 1) * 0.95)
  ang = 0.0
  while ang < TAU:
    px, py = math.cos(ang) * 4.4, math.sin(ang) * 1
    X = cx + px * math.cos(tl) - py * math.sin(tl)
    Y = cy + px * math.sin(tl) + py * math.cos(tl)
    if not (math.sin(ang) < 0 and math.hypot(X - cx, Y - cy) < r):
      o.l(X, Y, '#fde68a', 0.6)
    ang += 0.2
  follow(o.T, cx, cy)
  o.T['scale'] = 1.1
  peek_right(e, o.T)


def r_sneeze(a, e, u, t, o, now, E0):
  T = o.T
  if t < 1300:
    k = t / 1300
    lids(T, 0.5 * k); T['sy'] = 1 - 0.3 * k; T['jy'] = -0.8 * k + math.sin(t / 40) * 0.15
  elif t < 1700:
    o.gl[-1] = (GLYPHS['gt'], None); o.gl[1] = (GLYPHS['lt'], None); T['jy'] = 3
  else:
    T['open'] = tri(t, 1850, 160) * tri(t, 2150, 160)


def r_hiccup(a, e, u, t, o, now, E0):
  if any(h < t < h + 140 for h in (300, 1200, 2100)):
    o.T['jy'] = -2.6; o.T['scale'] = 1.12


def s_ign_on(a, e):
  e.parked = False


def s_engaged(a, e):
  e.engaged = True


_lat_left = lateral(1)
_lat_right = lateral(-1)

# Priorities: 9 transitions, 7 ignition, 6 orange alerts, 5 lateral g / lane change, 4 brake / accel / engage,
# 2 signals, 0 idle and events.
SPECS: dict[str, Spec] = {
  'boot': Spec('Comma start-up', BOOT_LIGHT_MS + 4700, r_boot, start=s_ign_on, prio=9, no_blink=True, no_z=True),
  'ignOn': Spec('Waking up (out of Park)', 4700, r_ign_on, start=s_ign_on, prio=7, no_blink=True),
  'eng': Spec('Engaged', 2800, r_engaged, start=s_engaged, prio=4),
  'sigL': Spec('Signal left', 3100, signal(-1), prio=2, loop='blinker', loop_value=-1),
  'sigR': Spec('Signal right', 3100, signal(1), prio=2, loop='blinker', loop_value=1),
  'alcL': Spec('Auto lane change left', 3400, lane_change(-1), prio=5, no_blink=True),
  'alcR': Spec('Auto lane change right', 3400, lane_change(1), prio=5, no_blink=True),
  # A left bend throws the eyes to the right edge.
  'latL': Spec('High lateral g (left bend)', 5000, _lat_left[1], start=_lat_left[0], prio=5, no_blink=True,
               hold=('lat_hold', 1750, 700)),
  'latR': Spec('High lateral g (right bend)', 5000, _lat_right[1], start=_lat_right[0], prio=5, no_blink=True,
               hold=('lat_hold', 1750, 700)),
  'accel': Spec('Hard accel', 2300, r_accel, prio=4),
  'brake': Spec('Hard brake', 1900, r_brake, prio=4, no_blink=True),
  'dis': Spec('Disengaged', 3000, r_disengaged, start=s_disengaged, prio=4, no_blink=True),
  'ignOff': Spec('Going to sleep (into Park)', 5600, r_ign_off, end=e_ign_off, prio=7, no_blink=True),
  'alAttn': Spec('Pay attention', 4200, r_al_attention, prio=6, no_blink=True, loop='alert', loop_value='alAttn'),
  'alWheel': Spec('Touch steering wheel', 4200, r_al_wheel, prio=6, no_blink=True, loop='alert', loop_value='alWheel'),
  'alSteer': Spec('Turn exceeds steering limit', 5200, r_al_steer, prio=6, no_blink=True, loop='alert', loop_value='alSteer'),
  'alTake': Spec('Take control', 4200, r_al_take, prio=6, no_blink=True, loop='alert', loop_value='alTake'),
  'alLdw': Spec('Lane departure', 3200, r_al_ldw, prio=6, no_blink=True, loop='alert', loop_value='alLdw'),
  'alBlind': Spec('Car in blind spot', 3300, r_al_blind, prio=6, no_blink=True, loop='alert', loop_value='alBlind'),
  'alResume': Spec('Press resume', 3900, r_al_resume, prio=6, loop='alert', loop_value='alResume'),
  'morphIn': Spec('Driving view to eyes', 1900, r_morph_in, start=s_morph_in, prio=9, no_blink=True, no_z=True),
  'morphOut': Spec('Eyes to driving view', 1000, r_morph_out, end=e_morph_out, prio=9, no_blink=True, no_z=True),
  'sigh': Spec('Sigh', 3300, r_sigh),
  'hum': Spec('Humming', 4600, r_hum, start=s_hum),
  'pong': Spec('Bouncing ball', 5200, r_pong),
  'orbit': Spec('Eye orbit', 3800, r_orbit),
  'nod': Spec('Nodding off', 3600, r_nod, no_blink=True),
  'doodle': Spec('Doodle a heart', 6800, r_doodle),
  'blink': Spec('Blink', 220, r_blink, no_blink=True),
  'dbl': Spec('Double blink', 620, r_double_blink, no_blink=True),
  'around': Spec('Look around', 2800, r_look_around),
  'roll': Spec('Eye roll', 1600, r_roll, no_blink=True),
  'squint': Spec('Squint', 1500, r_squint),
  'yawn': Spec('Yawn', 2000, r_yawn, no_blink=True),
  'side': Spec('Side-eye', 2400, r_side, start=s_side),
  'cross': Spec('Cross-eyed', 1700, r_cross),
  'wiggle': Spec('Happy wiggle', 1500, r_wiggle),
  'peek': Spec('Peek', 2700, r_peek, no_z=True, park=True),
  'aurora': Spec('Aurora', 8500, r_aurora, no_z=True, event=True),
  'fireworks': Spec('Fireworks', 5800, r_fireworks, start=s_fireworks, no_z=True, event=True),
  'snow': Spec('Snowfall', 8200, r_snow, start=s_snow, no_z=True, event=True),
  'rainbow': Spec('Rain, then a rainbow', 8500, r_rainbow, start=s_rainbow, no_z=True, event=True),
  'butterfly': Spec('Butterfly lands', 7800, r_butterfly, no_z=True, event=True),
  'bubbles': Spec('Bubbles', 8000, r_bubbles, start=s_bubbles, no_z=True, event=True),
  'eclipse': Spec('StarPilot eclipse', 9500, r_eclipse, no_z=True, event=True),
  'storm': Spec('Purple lightning', 6800, r_storm, start=s_storm, no_z=True, event=True),
  'constellation': Spec('Constellation', 8500, r_constellation, no_z=True, event=True),
  'shoot': Spec('Shooting star', 2000, r_shoot, start=s_shoot, no_z=True, event=True),
  'meteor': Spec('Meteor shower', 4200, r_meteor, no_z=True, event=True),
  'fly': Spec('Fireflies', 7600, r_fireflies, start=s_fireflies, no_z=True, event=True),
  'heart': Spec('Floating heart', 4600, r_heart, start=s_heart, no_z=True, event=True),
  'planet': Spec('Ringed planet', 9500, r_planet, no_z=True, event=True),
  'sneeze': Spec('Sneeze', 2600, r_sneeze, no_blink=True, event=True),
  'hiccup': Spec('Hiccups', 2800, r_hiccup, event=True),
}

IDLE_DRIVE = ('blink', 'dbl', 'around', 'around', 'roll', 'squint', 'yawn', 'side', 'cross', 'wiggle', 'dbl')
IDLE_STOP = ('sigh', 'hum', 'pong', 'orbit', 'nod', 'doodle', 'blink', 'around', 'yawn')
EVENTS = ('eclipse', 'aurora', 'fireworks', 'snow', 'rainbow', 'butterfly', 'bubbles', 'storm', 'constellation', 'shoot',
          'meteor', 'fly', 'heart', 'planet', 'sneeze', 'hiccup')
EVENTS_PARKED = ('eclipse', 'aurora', 'snow', 'constellation', 'shoot', 'meteor', 'fly', 'planet', 'butterfly')


class Engine:
  def __init__(self, rng: random.Random | None = None):
    self.rng = rng or random.Random()
    self.S = dict(BASE)
    self.bk = [0.0] * (N * 3)
    self.fr = [0.0] * (N * 3)
    self.parked = False
    self.stopped = False
    self.engaged = False
    self.pure_black = False
    self.auto = True
    # Continuous context fed by the car: blinker (-1/0/1), alert (spec name or None), alert_dir, lat_hold.
    self.ctx: dict = {'blinker': 0, 'alert': None, 'alert_dir': 0, 'lat_hold': False}
    self.scene_cells: list | None = None
    self.exit_done = False
    self.stick_dir = 1
    self.daze_ph = 0.0
    self.cur: Act | None = None
    self.cur_start = 0.0
    self.queue: list[str] = []
    self.wander = [0.0, 0.0, 0.0]
    self.next_blink = 0.0
    self.blink_at = -1e9
    self.next_idle = 0.0
    self.next_event = 0.0
    self.last_now: float | None = None

  # --- control ---------------------------------------------------------------------------------------------
  def reset_timers(self, now: float) -> None:
    self.next_idle = now + 2500
    self.next_event = now + 14000 + self.rng.random() * 12000

  def play(self, name: str, now: float, *, force: bool = False) -> bool:
    """Start an animation if it outranks (or equals) the one playing."""
    spec = SPECS[name]
    if self.cur is not None and not force and spec.prio < self.cur.spec.prio:
      return False
    if self.cur is not None and self.cur.spec.end and self.cur.name != name:
      self.cur.spec.end(self.cur, self)
    self.cur = Act(name, spec)
    self.cur_start = now
    if spec.start:
      spec.start(self.cur, self)
    return True

  def stop(self) -> None:
    self.cur = None
    self.queue = []

  @property
  def current(self) -> str | None:
    return self.cur.name if self.cur else None

  def colour_now(self) -> str:
    return ASLEEP if self.parked else ENGAGED if self.engaged else DISENGAGED

  # --- per frame -------------------------------------------------------------------------------------------
  def _update_current(self, now: float) -> None:
    cur = self.cur
    if cur is None:
      return
    spec = cur.spec
    t = now - self.cur_start
    if spec.loop is not None and self.ctx.get(spec.loop) != spec.loop_value:
      self.cur = None
      return
    if spec.hold is not None and self.ctx.get(spec.hold[0]) and t > spec.hold[1]:
      self.cur_start = now - spec.hold[2]
      return
    if t > spec.dur:
      if spec.loop is not None:
        self.cur_start = now
        if spec.start:
          spec.start(cur, self)
        return
      self.cur = None
      if spec.end:
        spec.end(cur, self)
      if self.queue:
        self.play(self.queue.pop(0), now, force=True)
      else:
        self.next_idle = now + 1500

  def _ambient(self, now: float) -> None:
    want = None
    if self.ctx.get('alert'):
      want = self.ctx['alert']
    elif self.ctx.get('blinker'):
      want = 'sigL' if self.ctx['blinker'] < 0 else 'sigR'
    if want and (self.cur is None or self.cur.name != want):
      self.play(want, now)

  def _schedule(self, now: float) -> None:
    if self.cur is not None or self.queue or not self.auto:
      return
    if now > self.next_event:
      pool = EVENTS_PARKED if self.parked else EVENTS
      self.play(self.rng.choice(pool), now)
      self.next_event = now + 14000 + self.rng.random() * 12000
    elif now > self.next_idle:
      if self.parked:
        if self.rng.random() < 0.35:
          self.play('peek', now)
      else:
        self.play(self.rng.choice(IDLE_STOP if self.stopped else IDLE_DRIVE), now)
      self.next_idle = now + 2500 + self.rng.random() * 4000

  def step(self, now: float) -> list[tuple[int, int, int]]:
    self._update_current(now)
    if not self.parked:
      self._ambient(now)
    self._schedule(now)

    T = dict(BASE)
    if self.parked:
      lids(T, 0.88)
      T['gy'] = 0.5
    else:
      if now > self.wander[2]:
        m = 0.5 if self.stopped else 1.0
        self.wander[0] = (self.rng.random() * 2 - 1) * 0.7 * m
        self.wander[1] = (self.rng.random() * 2 - 1) * 0.45 * m + (0.15 if self.stopped else 0)
        self.wander[2] = now + (2500 if self.stopped else 1300) + self.rng.random() * 2200
      T['gx'], T['gy'] = self.wander[0], self.wander[1]
      if self.stopped:
        lids(T, 0.3)

    bk, fr = self.bk, self.fr
    bk[:] = _ZEROS
    fr[:] = _ZEROS
    col_now = self.colour_now()
    o = Frame(self, T, col_now)
    E0 = eyes_of(self.S, self.stick_dir, self.daze_ph)
    cur = self.cur
    if cur is not None:
      t = now - self.cur_start
      cur.spec.run(cur, self, min(1.0, t / cur.spec.dur), t, o, now, E0)
    if self.parked and not (cur and cur.spec.no_z):
      zzz(o, now)

    bf = 1.0
    if not self.parked and not (cur and cur.spec.no_blink):
      if now > self.next_blink:
        self.blink_at = now
        self.next_blink = now + 2400 + self.rng.random() * 3800
      bf = tri(now, self.blink_at, 150)

    # Easing rates were tuned per 60 Hz frame; scale them to the real step so any frame rate looks the same.
    frames = 1.0 if self.last_now is None else clamp((now - self.last_now) / (1000 / 60), 0.0, 12.0)
    self.last_now = now
    S = self.S
    for k, base in BASE.items():
      target = T.get(k, base)
      rate = 0.45 if k == 'stick' and target > S[k] else RATE.get(k, 0.15)
      S[k] += (target - S[k]) * (1 - (1 - rate) ** frames)
    if o.set_jx is not None:
      S['jx'] = o.set_jx

    E = eyes_of(S, self.stick_dir, self.daze_ph)
    for eye in E:
      eye.eh *= bf
    col = rgb(o.col or col_now)
    sol: list = [None] * N
    if not o.hide:
      for eye in E:
        glyph = o.gl.get(eye.s)
        if glyph:
          c = rgb(glyph[1] or col)
          for i, j in glyph_cells(glyph[0], eye.x, eye.y):
            if 0 <= i < C and 0 <= j < R:
              sol[j * C + i] = c
        else:
          for i, j in each_cell(eye):
            sol[j * C + i] = mix(col, '#ffffff', o.shade(i, j)) if o.shade else col
    for item in o.sol:
      if item[0] == 'g':
        _, gl, cx, cy, c = item
        for i, j in glyph_cells(gl, cx, cy):
          if 0 <= i < C and 0 <= j < R:
            sol[j * C + i] = c
      else:
        sol[item[1]] = item[2]

    bg = BG_BLACK if self.pure_black else BG_PURPLE
    out = []
    for k in range(N):
      s = sol[k]
      k3 = k * 3
      if s is not None:
        r, g, b = s[0] + fr[k3], s[1] + fr[k3 + 1], s[2] + fr[k3 + 2]
      else:
        r = bg[0] + bk[k3] + fr[k3]
        g = bg[1] + bk[k3 + 1] + fr[k3 + 1]
        b = bg[2] + bk[k3 + 2] + fr[k3 + 2]
      out.append((min(255, int(r)), min(255, int(g)), min(255, int(b))))
    return out
