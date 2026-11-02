from dataclasses import replace
import pytest
from openpilot.starpilot.controls.lib.map_corner.types import Corner, Sample
from openpilot.starpilot.controls.lib.map_corner.learning import Observer

def corner():
  return Corner('bend',100,1,'revision',100,150,200,(18,18,18),((0,0),(10,5),(20,20)),90,curvature=tuple(1.8/(v*v) for v in (18,18,18)))

def passage(*, automated_bend=False, before_owner='map', mixed=False, bad_at=None, intervention=True, visual=True, clear=True, learn=True):
  obs=Observer()
  output=[]
  for i in range(181):
    progress=40+i
    manual=progress>=70 and not automated_bend
    owner='visual' if mixed and 55<=progress<60 else before_owner
    sample=Sample(now=1+i*.05, progress=progress, speed=14 if manual else 18,
                  automated=not manual, owner=owner if not manual else 'manual',
                  intervention=intervention and progress==70, confounded=progress==bad_at,
                  fault=False, visual_valid=visual, clear_conditions=clear, learn=learn)
    result=obs.update(corner(),sample)
    if result: output.append(result)
  return output

def test_brake_takeover_and_slower_full_manual_bend_produces_one_profile():
  out=passage()
  assert len(out)==1
  assert out[0].profile == pytest.approx((14,14,14))

@pytest.mark.parametrize('kwargs', [{'automated_bend':True},{'before_owner':'visual'},{'before_owner':'other'},
                                  {'before_owner':'tie'},{'mixed':True},{'bad_at':170},{'intervention':False},{'learn':False}])
def test_rejects_self_training_other_owners_mixed_confounds_or_no_driver_event(kwargs):
  assert passage(**kwargs)==[]

def test_visual_unknown_blocks_faster_but_weather_is_not_a_gate():
  assert passage(visual=False)[0].faster_allowed is False
  assert passage(clear=False)[0].faster_allowed is True

def test_fault_and_gap_abort_instead_of_learning_partial_pass():
  obs=Observer()
  for i in range(100):
    s=Sample(1+i*.05,40+i,14,i<30,'map' if i<30 else 'manual',i==30,False,i==60,True,True,True)
    assert obs.update(corner(),s) is None
  for i in range(100,181):
    s=Sample(20+i*.05,40+i,14,False,'manual',False,False,False,True,True,True)
    assert obs.update(corner(),s) is None
