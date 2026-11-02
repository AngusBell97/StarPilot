from dataclasses import replace
import pytest
from openpilot.starpilot.controls.lib.map_corner.store import CornerStore
from openpilot.starpilot.controls.lib.map_corner.learning import Observer
from openpilot.starpilot.controls.lib.map_corner.types import Sample
from test_map_corner_store import corner,MPH


def test_store_receipt_distinguishes_observation_from_applied_preference(tmp_path):
  with CornerStore(tmp_path) as store:
    c=corner()
    assert store.observe(c,(44*MPH,)*3,'one',False)=='stored_unapplied'
    assert store.stats()['observations']==1
    assert store.lookup(c).profile==pytest.approx(c.prior)
    assert store.observe(c,(36*MPH,)*3,'two',False)=='stored_applied'
    assert store.lookup(c).profile==pytest.approx((38*MPH,)*3)


def test_duplicate_and_cancelled_observations_cannot_claim_a_new_commit(tmp_path):
  with CornerStore(tmp_path) as store:
    c=corner();store.observe(c,(36*MPH,)*3,'one',False)
    assert store.observe(c,(36*MPH,)*3,'one',False)=='duplicate'
    with pytest.raises(ValueError):store.observe(c,(34*MPH,)*3,'two',False,lambda:False)
    assert store.stats()['observations']==1


def test_reset_during_manual_bend_cannot_resume_partial_learning():
  from test_map_corner_learning import corner as bend
  o=Observer();outputs=[]
  for i in range(181):
    progress=40+i
    if progress==140:o.reset()
    sample=Sample(1+i*.05,progress,14 if progress>=70 else 18,progress<70,'map' if progress<70 else 'manual',intervention=progress==70)
    result=o.update(bend(),sample)
    if result:outputs.append(result)
  assert not outputs
