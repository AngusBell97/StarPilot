from dataclasses import replace
import sqlite3
import pytest
from openpilot.starpilot.controls.lib.map_corner.types import Corner
from openpilot.starpilot.controls.lib.map_corner.store import CornerStore, MemoryFull

MPH = .44704
def corner(key='a'):
  return Corner(key=key,way_id=100,direction=1,revision='test-revision',entry=100,apex=150,exit=200,
                prior=(40*MPH,)*3,shape=((0,0),(10,5),(20,20)),heading=90,curvature=tuple(1.8/(v*v) for v in (40*MPH,)*3))

def test_slower_observation_adopts_bounded_step_then_converges_from_previous_value(tmp_path):
  with CornerStore(tmp_path) as s:
    c=corner()
    s.observe(c,(32*MPH,)*3,'pass1',False)
    assert s.lookup(c).profile == pytest.approx((38*MPH,)*3)
    s.observe(c,(32*MPH,)*3,'pass2',False)
    assert s.lookup(c).profile == pytest.approx((36*MPH,)*3)
    s.observe(c,(32*MPH,)*3,'pass3',False)
    s.observe(c,(32*MPH,)*3,'pass4',False)
    assert s.lookup(c).profile == pytest.approx((32*MPH,)*3)
  with CornerStore(tmp_path) as s:
    assert s.lookup(c).profile == pytest.approx((32*MPH,)*3)

def test_faster_needs_three_independent_consistent_passes_and_only_one_mph(tmp_path):
  with CornerStore(tmp_path) as s:
    c=corner()
    for token in ('one','two'):
      s.observe(c,(44*MPH,)*3,token,True)
      assert s.lookup(c).profile == pytest.approx(c.prior)
    s.observe(c,(44*MPH,)*3,'two',True)
    assert s.lookup(c).profile == pytest.approx(c.prior)
    s.observe(c,(44*MPH,)*3,'three',True)
    assert s.lookup(c).profile == pytest.approx((41*MPH,)*3)

def test_unqualified_faster_observation_never_promotes_faster(tmp_path):
  with CornerStore(tmp_path) as s:
    c=corner()
    for n in range(10): s.observe(c,(44*MPH,)*3,str(n),False)
    p=s.lookup(c)
    assert p is None or p.profile == pytest.approx(c.prior)

def test_entry_only_reduction_does_not_lower_apex_or_exit(tmp_path):
  with CornerStore(tmp_path) as s:
    c=corner()
    s.observe(c,(36*MPH,40*MPH,40*MPH),'one',False)
    assert s.lookup(c).profile == pytest.approx((38*MPH,40*MPH,40*MPH))

def test_changed_map_never_recalls_old_preference(tmp_path):
  with CornerStore(tmp_path) as s:
    c=corner()
    s.observe(c,(36*MPH,)*3,'one',False)
    assert s.lookup(replace(c,revision='new')) is None

def test_undo_forget_and_clear_remove_history_transactionally(tmp_path):
  with CornerStore(tmp_path) as s:
    c=corner()
    s.observe(c,(32*MPH,)*3,'one',False)
    s.observe(c,(32*MPH,)*3,'two',False)
    s.undo()
    assert s.lookup(c).profile == pytest.approx((38*MPH,)*3)
    s.forget(c.key)
    assert s.lookup(c) is None
    assert s.stats()['observations'] == 0
    s.observe(c,(32*MPH,)*3,'three',False)
    s.clear()
    assert s.stats()['corners'] == s.stats()['observations'] == 0

def test_capacity_does_not_silently_evict_slower_preference(tmp_path):
  with CornerStore(tmp_path,max_corners=1) as s:
    s.observe(corner(),(32*MPH,)*3,'one',False)
    with pytest.raises(MemoryFull): s.observe(corner('b'),(32*MPH,)*3,'two',False)
    assert s.lookup(corner()).profile == pytest.approx((38*MPH,)*3)

def test_history_bound_and_nonfinite_input(tmp_path):
  with CornerStore(tmp_path) as s:
    for n in range(30): s.observe(corner(),(32*MPH,)*3,str(n),False)
    assert s.stats()['observations'] <= 8
    with pytest.raises(ValueError): s.observe(corner(),(float('nan'),15,15),'bad',False)

def test_second_writer_cannot_race_parked_management(tmp_path):
  with CornerStore(tmp_path):
    with pytest.raises(OSError): CornerStore(tmp_path)

def test_sqlite_full_rolls_back_and_keeps_existing_slower_record(tmp_path):
  with CornerStore(tmp_path) as s:
    s.observe(corner(),(32*MPH,)*3,'first',False)
    pages=s.conn.execute('PRAGMA page_count').fetchone()[0]
    s.conn.execute(f'PRAGMA max_page_count={pages}')
    failed=False
    for i in range(200):
      try:s.observe(corner(str(i)),(32*MPH,)*3,str(i),False)
      except sqlite3.DatabaseError:failed=True;break
    assert failed and not s.conn.in_transaction
    assert s.lookup(corner()).profile==pytest.approx((38*MPH,)*3)
    with pytest.raises(MemoryFull):s.observe(corner(),(32*MPH,)*3,'last',False)

def test_flag_change_before_commit_discards_observation(tmp_path):
  checks=iter([True,False])
  with CornerStore(tmp_path) as s:
    with pytest.raises(ValueError):s.observe(corner(),(32*MPH,)*3,'one',False,valid=lambda:next(checks))
    assert s.stats()['observations']==s.stats()['corners']==0
def test_matching_revision_recalls_even_after_partial_window_marked_inactive(tmp_path):
  with CornerStore(tmp_path) as s:
    c=corner();s.observe(c,(11,11,11),'one',False)
    s.mark_map(c.way_id,c.direction,())
    assert s.lookup(c) is not None

def test_repeated_large_slower_passes_adopt_bounded_slower_steps(tmp_path):
  from dataclasses import replace
  from openpilot.starpilot.controls.lib.map_corner.types import MPH
  with CornerStore(tmp_path) as s:
    c=replace(corner(),prior=(20,20,20))
    manual=(20-10*MPH,)*3
    s.observe(c,manual,'one',False)
    assert s.lookup(c).profile==(20,20,20)
    s.observe(c,manual,'two',False)
    assert s.lookup(c).profile==pytest.approx((20-2*MPH,)*3)

def test_fast_exit_does_not_discard_slower_entry_and_apex(tmp_path):
  from dataclasses import replace
  from openpilot.starpilot.controls.lib.map_corner.types import MPH
  with CornerStore(tmp_path) as s:
    c=replace(corner(),prior=(20,20,20))
    s.observe(c,(20-3*MPH,20-3*MPH,20+9*MPH),'one',False)
    assert s.lookup(c).profile==pytest.approx((20-2*MPH,20-2*MPH,20))
