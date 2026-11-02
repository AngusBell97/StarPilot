"""Real geometry regressions: qualification retention without early authority."""
from dataclasses import replace
import math
import pytest
from openpilot.starpilot.controls.lib.map_corner.types import Way
from openpilot.starpilot.controls.lib.map_corner.geometry import build_tile,find_match
from openpilot.starpilot.controls.lib.map_corner.provider import MatchTracker
from test_map_corner_provider import ll,fix

def road(start=-300,end=500,identity=1,y=0,lanes=2):
    return Way(identity,tuple(ll(x,y) for x in range(start,end+1,5)),False,'secondary','',lanes)

def drive(tracker,tile,start=-45,count=56,y=lambda i:0,speed=20):
    result=None
    for i in range(count):
        result=tracker.update(tile,replace(fix(1+i*.1,x=start+2*i,y=y(i)),speed=speed))
        if i<50:assert result is None, 'Control authority before100m'
    return result

def test_qualifying_track_survives_connected_side_access_before100m():
    tile=build_tile((road(),Way(2,(ll(0,0),ll(0,100)),False,'residential','')),'r')
    tr=MatchTracker();m=drive(tr,tile)
    assert m is not None and m.way_id==1

@pytest.mark.parametrize('lanes,accuracy',[(1,.8),(2,1),(3,2)])
def test_qualifying_track_earns_bounded_corridor_from_motion(lanes,accuracy):
    tile=build_tile((road(lanes=lanes),),'r');tr=MatchTracker()
    target=lanes*3.5/2+accuracy+1.5
    for i in range(56):
        m=tr.update(tile,fix(1+i*.1,x=-150+2*i,y=target*min(1,i/50),accuracy=accuracy))
        if i<50:assert m is None
    assert m is not None
    assert tr.update(tile,fix(6.6,x=-38,y=lanes*3.5/2+accuracy+2.1,accuracy=accuracy)) is None

def test_qualifying_track_crosses_unique_degree_two_map_boundary():
    tile=build_tile((road(end=0),road(start=0,identity=2)),'r')
    tr=MatchTracker();m=drive(tr,tile)
    assert m is not None and m.way_id==2

def test_provisional_cannot_cross_multiway_join_even_with_consistent_motion():
    tile=build_tile((road(end=0),road(start=0,identity=2),
        Way(3,(ll(0,0),ll(0,100)),False,'residential','')),'r')
    tr=MatchTracker()
    for i in range(20):tr.update(tile,fix(1+i*.1,x=-40+2*i))
    assert tr.update(tile,fix(3,x=0)) is None
    assert tr.since==3, 'Unqualified multiway transition must reset confidence'

def test_stationary_forward_drift_cannot_earn_corridor_allowance():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    for i in range(12):tr.update(tile,replace(fix(1+i*.1,x=-100+2*i),speed=0))
    assert tr.update(tile,replace(fix(2.2,x=-76,y=5),speed=0)) is None
    assert tr.last_match is None

def test_oscillating_jitter_cannot_earn_corridor_allowance():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    for i in range(30):tr.update(tile,replace(fix(1+i*.1,x=-100+(i%2)*.9),speed=3))
    assert tr.update(tile,replace(fix(4,x=-99,y=5),speed=3)) is None
    assert tr.last_match is None

def test_failed_motion_cannot_keep_permissive_candidate_as_new_seed():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    assert drive(tr,tile,start=-200) is not None
    assert tr.update(tile,fix(6.6,x=100,y=5)) is None
    assert tr.last_match is None, 'Cold matching must reject5m residual after jump'

@pytest.mark.parametrize('change',[{'has_fix':False},{'latitude':float('nan')},{'bearing':180}])
def test_changed_duplicate_payload_invalidates_qualified_cache(change):
    tr=MatchTracker();tile=build_tile((road(),),'r')
    assert drive(tr,tile,start=-200) is not None
    assert tr.update(tile,replace(tr.last_fix,**change)) is None
    assert not tr.qualified

def test_older_identity_cannot_reenter_as_new_evidence():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    assert drive(tr,tile,start=-200) is not None
    assert tr.update(tile,replace(fix(6.6,x=-88),identity=100)) is None
    assert tr.last_match is None

@pytest.mark.parametrize('kind',['parallel','shallow_fork','unconnected_crossing'])
def test_provisional_still_rejects_true_competing_roads(kind):
    if kind=='parallel': other=road(start=0,identity=2,y=10)
    elif kind=='shallow_fork':
        other=Way(2,tuple(ll(x,x*math.tan(math.radians(10))) for x in range(0,101,5)),False,'secondary','')
    else:other=Way(2,(ll(0,-100),ll(0,100)),False,'secondary','')
    tr=MatchTracker();tile=build_tile((road(),other),'r')
    drive(tr,tile,count=20)
    assert tr.last_match is None

def test_real_side_turn_revokes_provisional_through_road():
    tile=build_tile((road(),Way(2,(ll(0,0),ll(0,100)),False,'residential','')),'r')
    tr=MatchTracker();drive(tr,tile,count=23)
    assert tr.update(tile,fix(3.3,x=0,y=2,bearing=0)) is None
    assert not tr.qualified

def test_nearby_crossing_is_not_excused_by_a_connection_elsewhere():
    other=Way(2,(ll(0,-100),ll(0,50),ll(40,50),ll(40,0)),False,'secondary','')
    tile=build_tile((road(),other),'r');tr=MatchTracker()
    drive(tr,tile,count=20)
    assert tr.last_match is None

def test_reordered_sequence_cannot_lower_identity_watermark():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    assert drive(tr,tile,start=-200) is not None
    assert tr.update(tile,replace(fix(6.6,x=-88),identity=100)) is None
    assert tr.update(tile,replace(fix(6.7,x=-86),identity=101)) is None
    assert tr.last_match is None

def test_folded_side_arm_cannot_masquerade_as_a_local_straight_connection():
    other=Way(2,(ll(0,0),ll(0,15),ll(.2,3),ll(.2,-30)),False,'residential','')
    tr=MatchTracker();tile=build_tile((road(),other),'r')
    drive(tr,tile,count=20)
    assert tr.last_match is None

def test_fast_travel_still_waits_four_seconds_for_control():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    for i in range(40):
        assert tr.update(tile,replace(fix(1+i*.1,x=-250+4*i),speed=40)) is None
    assert tr.update(tile,replace(fix(5.1,x=-86),speed=40)) is not None

def test_identical_duplicates_never_earn_early_protection():
    tr=MatchTracker();tile=build_tile((road(),),'r');f=fix(x=-100)
    for _ in range(100):assert tr.update(tile,f) is None
    assert tr.update(tile,fix(1.1,x=-98,y=5)) is None
    assert tr.last_match is None

def test_local_reindex_keeps_same_road_evidence_but_geometry_change_resets():
    tr=MatchTracker();base=road();tile=build_tile((base,),'r')
    for i in range(30):tr.update(tile,fix(1+i*.1,x=-200+2*i))
    other=road(identity=9,y=500)
    tile=build_tile((base,other),'r')
    for i in range(30,56):m=tr.update(tile,fix(1+i*.1,x=-200+2*i))
    assert m is not None
    changed=replace(base,points=base.points[:-1]+(ll(500,1),))
    assert tr.update(build_tile((changed,other),'r'),fix(6.6,x=-88)) is None

def test_transition_must_use_the_node_which_justified_local_exemption():
    from openpilot.starpilot.controls.lib.map_corner.types import Match
    from openpilot.starpilot.controls.lib.map_corner.geometry import transition_distance
    tile=build_tile((road(start=-100,end=0),road(start=0,end=100,identity=2)),'r')
    a=Match(1,1,98,0,11,(),'a');b=Match(2,1,2,0,11,(),'b')
    assert transition_distance(tile,a,b,unique_join_only=True,joining_node=ll(40,0)) is None
    assert transition_distance(tile,a,b,unique_join_only=True,joining_node=ll(0,0))==pytest.approx(4)

def test_two_motion_steps_do_not_earn_protection_despite_time_and_distance():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    for t,x in [(1,-100),(1.6,-88),(2.2,-76)]:tr.update(tile,fix(t,x=x,y=4))
    assert tr.update(tile,fix(2.3,x=-74,y=5)) is None
    assert tr.last_match is None

@pytest.mark.parametrize('speed,retained',[(2.9,False),(3,True)])
def test_low_speed_heading_does_not_earn_protection(speed,retained):
    tr=MatchTracker();tile=build_tile((road(),),'r')
    for i in range(60):tr.update(tile,replace(fix(1+i*.1,x=-100+speed*.1*i,y=4),speed=speed))
    assert tr.update(tile,replace(fix(7,x=-100+speed*6,y=5),speed=speed)) is None
    assert (tr.last_match is not None)==retained

def test_forward_drift_disagreeing_with_integrated_speed_cannot_earn_protection():
    tr=MatchTracker();tile=build_tile((road(),),'r')
    for i in range(20):tr.update(tile,replace(fix(1+i*.1,x=-100+2*i,y=4),speed=3))
    assert tr.update(tile,replace(fix(3,x=-60,y=5),speed=3)) is None
    assert tr.last_match is None

def test_same_road_strict_success_preserves_identity_with_protection():
    tile=build_tile((road(),),'r');previous=find_match(tile,fix(x=-100))
    for x in (-98,-90,-80):
        for y in (-4,-2,0,2,4):
            f=fix(2,x=x,y=y);strict=find_match(tile,f)
            protected=find_match(tile,f,previous,provisional=True)
            assert strict and protected
            assert (strict.way_id,strict.direction,strict.progress)==(protected.way_id,protected.direction,protected.progress)
