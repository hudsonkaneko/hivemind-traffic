from copy import deepcopy
import json
from pathlib import Path

import pytest

from traffic.highway_merge_cutin_v07 import MIX, gap_check, training_row
from traffic.highway_merge_cutin_v07_contract import validate_config

ROOT = Path(__file__).resolve().parents[1]


def config():
    return json.loads((ROOT / 'configs/training/highway-merge-cutin-2026-11-15.json').read_text())


def test_historical_close_cutin_is_blocked_even_with_long_ttc():
    # Faster leader cuts in 3m ahead of an 8.75m/s follower: no closing TTC,
    # but 0.34s headway violates our requirement independently of Krauss.
    result = gap_check(19.05, {'rear': dict(id='av', bumper_gap_m=3., relative_speed_m_s=-10.3)})
    assert not result['permitted']
    assert result['checks']['rear']['required_gap_m'] == pytest.approx(13.125)


@pytest.mark.parametrize('side', ['front', 'rear'])
def test_equal_speed_boundary(side):
    assert gap_check(20., {side: dict(bumper_gap_m=30., relative_speed_m_s=0.)})['permitted']
    assert not gap_check(20., {side: dict(bumper_gap_m=29.9, relative_speed_m_s=0.)})['permitted']


def test_future_gap_closure_blocks_apparently_safe_start():
    assert not gap_check(20., {'rear': dict(bumper_gap_m=45., relative_speed_m_s=5.)})['permitted']
    assert gap_check(20., {'rear': dict(bumper_gap_m=60., relative_speed_m_s=5.)})['permitted']


def test_stress_admission_retains_minimum_gap_and_ttc():
    assert gap_check(20., {'rear': dict(bumper_gap_m=12., relative_speed_m_s=0.)}, headway=.5)['permitted']
    assert not gap_check(20., {'rear': dict(bumper_gap_m=2., relative_speed_m_s=-10.)}, headway=.5)['permitted']
    assert not gap_check(20., {'rear': dict(bumper_gap_m=15., relative_speed_m_s=5.)}, headway=.5)['permitted']


def test_decision_balance_uses_actual_experience_and_is_resume_deterministic():
    cfg = config()
    state = dict(coverage={})
    counts = dict.fromkeys(MIX, 0)
    for _ in range(300):
        row = training_row(cfg, state, False)
        assert row == training_row(cfg, deepcopy(state), False)
        actual = row['controlled_count'] * 170  # deliberately unequal episode sizes
        c = state['coverage'].setdefault(row['case']['id'], dict(ppo_decisions=0, episodes=0))
        c['ppo_decisions'] += actual
        c['episodes'] += 1
        counts[row['curriculum_group']] += actual
    total = sum(counts.values())
    for group, target in MIX.items():
        assert abs(counts[group] / total - target) < .025


def test_approved_contract():
    cfg = config()
    validate_config(cfg, require_training=True)
    bad = deepcopy(cfg)
    bad['lane_admission_settings']['headway_s'] = .3
    with pytest.raises(ValueError):
        validate_config(bad, True)
    bad = deepcopy(cfg)
    bad['budget']['per_round_decisions'] += 1
    with pytest.raises(ValueError):
        validate_config(bad, True)


def test_reservation_blocks_second_move_into_same_gap():
    from environments.highway_merge_cutin_v07_env import HighwayMergeCutinV07Env
    env = object.__new__(HighwayMergeCutinV07Env)
    class Graph:
        lanes = {'right': dict(length=100.), 'middle': dict(length=100.), 'left': dict(length=100.)}
        def neighbor(self, ego, states, lane, position, *, forward):
            candidates = [s for k, s in states.items() if k != ego['id'] and s['lane_id'] == lane]
            if not candidates or not forward:
                return None
            return dict(id=candidates[0]['id'], bumper_gap_m=-4.8, relative_speed_m_s=0.)
    env.graph = Graph()
    env.states = {'a': dict(id='a', speed=20., lane_id='right', lane_position=50.),
                  'b': dict(id='b', speed=20., lane_id='left', lane_position=50.)}
    env.reservations = {}
    assert env._evidence('b', 'middle')['permitted']
    env.reservations['a'] = dict(target='middle')
    assert not env._evidence('b', 'middle')['permitted']


def test_sparse_five_lane_missed_gap_keeps_a_blocker_without_reseeding(tmp_path):
    from experiments.run_highway_merge_cutin_v07 import env_for
    from traffic.highway_v05_demand import write_demand
    cfg = config()
    row = next(r for r in cfg['training_rows'] if r.get('cutin_stress') and r['case']['through_lanes'] == 5)
    env = env_for(cfg, row, tmp_path / 'env')
    assert env.v04_mask_audit['version'] == cfg['mask_version']
    reproduced = False
    for cursor in range(12):
        seed = row['seed'] + 50000 + 1000 * cursor
        try:
            write_demand(tmp_path / 'old.xml', seed, env.graph, 'merge', 1, 8,
                         speed_limit_m_s=row['speed_limit_m_s'], difficulty=row['difficulty'],
                         missed_gap=True, controlled_car_follow_model='Krauss')
        except ValueError as exc:
            if 'rightmost-lane' not in str(exc):
                raise
            reproduced = True
        env._write_demand(tmp_path / f'new-{cursor}.xml', seed)
        assert len(env.population_manifest) == 8
        assert env.population_manifest.scenario['missed_gap']['blocker_slot']
    assert reproduced
