"""Sleep Eyes engine: every animation runs in every state, colours stay valid, loops and holds follow the car."""
import random
import time

import pytest

from openpilot.starpilot.ui.sleep_eyes.engine import BASE, C, Engine, N, R, SPECS, cells_of, eyes_of


def run(engine: Engine, start: float, ms: float, step: float = 50.0) -> tuple[float, list]:
  now, frame = start, []
  while now < start + ms:
    now += step
    frame = engine.step(now)
  return now, frame


def valid(frame) -> bool:
  return len(frame) == N and all(len(c) == 3 and all(isinstance(v, int) and 0 <= v <= 255 for v in c) for c in frame)


@pytest.mark.parametrize("state", ["drive", "stop", "park"])
@pytest.mark.parametrize("black", [False, True])
def test_every_animation_runs_in_every_state(state, black):
  engine = Engine(random.Random(1))
  engine.parked, engine.stopped, engine.pure_black = state == "park", state == "stop", black
  engine.engaged = state == "stop"
  engine.scene_cells = [(30, 40, 60)] * N
  now = 0.0
  for name, spec in SPECS.items():
    engine.ctx.update(blinker=0, alert=None, lat_hold=False)
    if spec.loop == 'blinker':
      engine.ctx['blinker'] = spec.loop_value
    if spec.loop == 'alert':
      engine.ctx['alert'] = spec.loop_value
    engine.play(name, now, force=True)
    now, frame = run(engine, now, spec.dur + 200)
    assert valid(frame), name
    assert all(abs(v) < 1e6 for v in engine.S.values()), name
    engine.parked, engine.stopped = state == "park", state == "stop"  # ignition animations change these


def test_pure_black_background_is_unlit_and_eyes_are_lit():
  engine = Engine(random.Random(2))
  engine.pure_black, engine.auto = True, False
  _, frame = run(engine, 0, 600)
  lit = [k for k, c in enumerate(frame) if c != (0, 0, 0)]
  eyes = {j * C + i for i, j in cells_of(eyes_of(engine.S))}
  assert eyes and set(lit) <= eyes | {k for k in range(N)}  # only eyes lit when idle and no blink
  assert sum(1 for k in range(N) if frame[k] == (0, 0, 0)) > N * 0.6


def test_signal_loops_while_blinker_on_and_stops_when_off():
  engine = Engine(random.Random(3))
  engine.ctx['blinker'] = 1
  now, _ = run(engine, 0, 7000)
  assert engine.current == 'sigR'
  engine.ctx['blinker'] = 0
  run(engine, now, 100)
  assert engine.current != 'sigR'


def test_orange_alert_outranks_signal_and_clears():
  engine = Engine(random.Random(4))
  engine.ctx['blinker'] = -1
  now, _ = run(engine, 0, 500)
  engine.ctx['alert'] = 'alAttn'
  now, _ = run(engine, now, 300)
  assert engine.current == 'alAttn'
  engine.ctx['alert'] = None
  run(engine, now, 300)
  assert engine.current == 'sigL'


def test_high_g_holds_the_eyes_on_the_edge_until_the_bend_ends():
  engine = Engine(random.Random(5))
  engine.ctx['lat_hold'] = True
  engine.play('latL', 0)
  now, _ = run(engine, 0, 6000)
  assert engine.current == 'latL' and engine.S['stick'] > 0.9
  xs = [i for i, _ in cells_of(eyes_of(engine.S, engine.stick_dir))]
  assert max(xs) == C - 1  # pinned to the right edge
  engine.ctx['lat_hold'] = False
  now, _ = run(engine, now, 1000)
  assert engine.S['daze'] > 0.5
  run(engine, now, 4000)
  assert engine.current != 'latL'


def test_ignition_off_parks_and_ignition_on_wakes():
  engine = Engine(random.Random(6))
  engine.play('ignOff', 0)
  now, _ = run(engine, 0, 6000)
  assert engine.parked and not engine.engaged
  engine.play('ignOn', now)
  assert not engine.parked


def test_morph_out_reports_done():
  engine = Engine(random.Random(7))
  engine.scene_cells = [(10, 20, 30)] * N
  engine.play('morphOut', 0)
  run(engine, 0, 1200)
  assert engine.exit_done


def test_frame_time_budget():
  engine = Engine(random.Random(8))
  worst = 0.0
  for name in ('aurora', 'rainbow', 'fireworks', 'snow', 'eclipse', 'alSteer', 'dis', 'morphIn'):
    engine.scene_cells = [(10, 20, 30)] * N
    if SPECS[name].loop:
      engine.ctx['alert'] = SPECS[name].loop_value
    engine.play(name, 0, force=True)
    for k in range(40):
      start = time.perf_counter()
      engine.step(150.0 * k + 1)
      worst = max(worst, time.perf_counter() - start)
    engine.ctx['alert'] = None
  print(f"worst frame {worst * 1000:.1f} ms")
  assert worst < 0.05


def test_neutral_eyes_fit_the_grid():
  cells = cells_of(eyes_of(dict(BASE)))
  assert 80 < len(cells) < 200
  assert all(0 <= i < C and 0 <= j < R for i, j in cells)


def lit(frame, threshold=40):
  return sum(1 for c in frame if max(c) > threshold)


def test_an_interrupted_animation_fades_out_instead_of_vanishing():
  engine = Engine(random.Random(9))
  engine.auto = False
  engine.play('aurora', 0, force=True)
  now, before = run(engine, 0, 3000)
  engine.ctx['alert'] = 'alAttn'
  engine.play('alAttn', now)  # cuts the aurora short
  first = engine.step(now + 50)
  later = run(engine, now + 50, 600)[1]
  aurora_cells = [k for k, c in enumerate(before) if c[1] > c[0] + 20]  # green-ish aurora light
  assert aurora_cells
  assert sum(1 for k in aurora_cells if first[k][1] > 30) > len(aurora_cells) // 3  # still fading
  assert sum(1 for k in aurora_cells if later[k][1] > 60 and later[k][1] > later[k][0]) == 0  # gone


def test_eye_colour_eases_between_states():
  engine = Engine(random.Random(10))
  engine.auto = False
  now, _ = run(engine, 0, 1000)
  engine.engaged = True
  first = engine.step(now + 50)
  eye_cells = {j * C + i for i, j in cells_of(eyes_of(engine.S))}
  colours = {first[k] for k in eye_cells}
  assert all(c != (0xb7, 0x7a, 0xff) for c in colours)  # not snapped to engaged purple in one frame
  _, settled = run(engine, now + 50, 1500)
  eye_cells = {j * C + i for i, j in cells_of(eyes_of(engine.S))}
  assert any(settled[k] == (0xb7, 0x7a, 0xff) for k in eye_cells)


def test_dim_fades_the_whole_picture_to_black():
  engine = Engine(random.Random(11))
  engine.dim = 0.0
  _, frame = run(engine, 0, 500)
  assert all(c == (0, 0, 0) for c in frame)


def test_reappearing_eyes_do_not_crossfade_from_an_old_frame():
  engine = Engine(random.Random(12))
  engine.play('fireworks', 0, force=True)
  now, _ = run(engine, 0, 1500)
  engine.reset_display()
  engine.scene_cells = [(0, 0, 0)] * N
  engine.play('morphIn', now + 60000, force=True)
  assert engine.xfade_from is None and engine.shown is None
  frame = engine.step(now + 60050)
  assert lit(frame) < 40  # a black scene and the first morph frame: nothing left over from the fireworks


def test_morph_in_forms_closed_eyes_when_asleep():
  engine = Engine(random.Random(13))
  engine.parked, engine.auto = True, False
  engine.scene_cells = [(200, 200, 200)] * N
  engine.play('morphIn', 0, force=True)
  _, frame = run(engine, 0, 2000)
  open_eyes = len(cells_of(eyes_of(dict(BASE))))
  assert lit(frame) < open_eyes // 2  # thin closed lids, not two open eyes
