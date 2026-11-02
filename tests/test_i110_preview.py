import math
import pytest

from traffic.i110_preview import PreviewConfig, PreviewController, release_guard, release_speed_budget
from traffic.smooth_acc import limit_acceleration


def latch(config=PreviewConfig()):
    c=PreviewController.__new__(PreviewController)
    c.config=config; c.memory={}
    return c


def test_hysteresis_unknown_and_release_dwell():
    c=latch()
    assert not c.update_latch('ego',0,[11],True)['active']
    assert c.update_latch('ego',.1,[9],True)['active']
    assert c.update_latch('ego',.2,[19],True)['active']
    assert c.update_latch('ego',.3,[],False)['active']
    assert c.update_latch('ego',.4,[21],True)['active']
    assert c.update_latch('ego',1.3,[30],True)['active']
    assert not c.update_latch('ego',1.4,[30],True)['active']


def test_missing_candidate_release_does_not_reuse_other_egos_memory():
    c=latch()
    assert c.update_latch('a',0,[-5],True)['active']
    assert not c.update_latch('b',0,[],True)['active']
    assert c.update_latch('a',.1,[],True)['active']
    assert not c.update_latch('a',1.1,[],True)['active']
    with pytest.raises(ValueError):c.update_latch('a',1.1,[],True)
    assert not latch().update_latch('a',0,[],True)['active']


@pytest.mark.parametrize('speed',[25.,3.,.5,.02,0.])
def test_terminal_release_preserves_discrete_jerk_and_nonnegative_speed(speed):
    acceleration=0.
    for _ in range(300):
        filtered=limit_acceleration(-2.5,acceleration,speed,.1)
        applied,guard=release_guard(filtered,acceleration,speed,.1,2.)
        assert not guard['existing_state_conflict']
        assert abs(applied-acceleration)<=.2+1e-7
        speed+=applied*.1
        assert speed>=-1e-9
        acceleration=applied
    assert speed==pytest.approx(0.,abs=1e-8)
    assert acceleration==pytest.approx(0.,abs=1e-8)


def test_budget_matches_explicit_discrete_release_and_exposes_native_conflict():
    expected=sum(max(0.,2.5-k*.2)*.1 for k in range(13))
    assert release_speed_budget(-2.5,.1,2.)==pytest.approx(expected)
    _,guard=release_guard(-4.3,-4.5,.1,.1,2.)
    assert guard['active'] and guard['existing_state_conflict']


@pytest.mark.parametrize('kwargs',[dict(range_m=math.nan),dict(reserve_m=30),dict(other_model='oracle')])
def test_invalid_contract_rejected(kwargs):
    with pytest.raises(ValueError):PreviewConfig(**kwargs)
