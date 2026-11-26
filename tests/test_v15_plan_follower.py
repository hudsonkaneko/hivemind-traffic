"""The plan follower (traffic/v15_plan_follower.py): its rule, its interface, the rows it is scored on, and
(with libsumo and the LA networks) a pool run with a canary."""
import gzip
import importlib.util
import json
import shutil
import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from experiments import v15_pool as pool
from traffic import v15_plan_follower as follower

ROOT = Path(__file__).resolve().parents[1]
HAS_LIBSUMO = importlib.util.find_spec('libsumo') is not None


def test_the_rule_is_the_measured_one():
    assert follower.DRIVER == 'rule:plan-follower-v1' and follower.DRIVER in pool.RULE_DRIVERS
    assert follower.SETTINGS == dict(gain_per_s=3.0, request_min_m_s2=-4.0, request_max_m_s2=2.0, ceiling_input=43,
                                     ceiling_scale_m_s2=4.5, ceiling_margin_m_s2=0.05, rate_m_s2=0.35,
                                     free_road_m_s=26.8224)


def test_target_is_the_eligible_v14_gate_target_or_the_free_road():
    state = dict(speed=10., acceleration=0., allowed_speed=20.1)
    assert follower.target_speed(state, dict(eligible=True, target_m_s=7.5)) == 7.5
    assert follower.target_speed(state, dict(eligible=False, target_m_s=7.5)) == 20.1
    assert follower.target_speed(state, None) == 20.1
    assert follower.target_speed(dict(speed=10.), dict(eligible=True, target_m_s=float('nan'))) == 26.8224
    assert follower.target_speed(dict(speed=10., allowed_speed=33.), {}) == 26.8224


def test_request_gain_ceiling_and_rate_limit():
    ceiling = lambda m_s2: m_s2 / 4.5                                    # the observed input
    calm = dict(speed=10., acceleration=0.)
    assert follower.request(calm, 10.05, ceiling(2.)) == pytest.approx(.15)          # 3/s x 0.05 m/s
    assert follower.request(calm, 30., ceiling(2.)) == pytest.approx(.35)            # rate-limited
    assert follower.request(dict(speed=10., acceleration=1.9), 30., ceiling(2.)) == pytest.approx(1.95)  # ceiling - 0.05
    assert follower.request(dict(speed=10., acceleration=1.9), 30., ceiling(1.)) == pytest.approx(1.55)  # rate floor wins
    assert follower.request(dict(speed=20., acceleration=-3.9), 0., ceiling(2.)) == pytest.approx(-4.)   # request floor
    assert follower.normalized(-4.5) == -1. and follower.normalized(2.) == 1. and follower.normalized(9.) == 1.


def test_act_drives_controlled_cars_keeps_lanes_and_is_deterministic_only():
    env = SimpleNamespace(agents=['agent_0', 'agent_3'],
                          states=dict(agent_0=dict(speed=10., acceleration=0.), agent_3=dict(speed=5., acceleration=0.)),
                          _v14_cache=dict(agent_0=dict(gate_v14=dict(eligible=True, target_m_s=10.05))))
    local = torch.zeros(24, 121)
    local[:, 43] = 2. / 4.5
    actor = follower.PlanFollower(lambda: env)
    actions, logp, _, latent = actor.act(local, deterministic=True)
    assert actions.shape == (24, 2) and (actions[:, 1] == 0).all() and torch.equal(actions, latent)
    assert -4.5 + (actions[0, 0].item() + 1) * 3.25 == pytest.approx(.15, abs=1e-5)
    assert -4.5 + (actions[3, 0].item() + 1) * 3.25 == pytest.approx(.35, abs=1e-5)  # free road, rate-limited
    assert (actions[[1, 2] + list(range(4, 24)), 0] == 0).all() and actor.values(None, torch.arange(24)).shape == (24,)
    with pytest.raises(ValueError):
        actor.act(local, deterministic=False)
    env.states.pop('agent_3')
    with pytest.raises(ValueError):
        actor.act(local, deterministic=True)


def test_scored_on_the_60_rows_without_cases_0_and_4():
    from traffic import highway_flow_rl_v14_contract as contract
    from traffic import v15_benchmark
    cfg = contract.build_config(24)
    cases = v15_benchmark.cases(cfg)
    assert [cases[i]['case']['id'] for i in (0, 4)] == list(follower.LANE_CHANGE_CASES)
    rows = v15_benchmark.development_rows(cfg)
    scored = follower.scored_rows(rows)
    assert len(scored) == 60 and not {i % 14 for i in scored} & {0, 4}


def test_the_pool_runs_rule_drivers_for_validation_only():
    pool.Task('v15-development', 1, follower.DRIVER)
    with pytest.raises(ValueError):
        pool.Task('v15-development', 1, follower.DRIVER, mode='collect')
    with pytest.raises(ValueError):
        pool.Task('v15-development', 1, 'policy:abc', label_with=follower.DRIVER)


def _networks_present():
    from traffic import highway_flow_rl_v14_contract as contract
    rows = contract.build_config(24)['validation_rows']
    return all((ROOT / rows[i]['case']['network']).is_file() for i in (10, 11))


@pytest.mark.skipif(not HAS_LIBSUMO, reason='libsumo is not installed in this environment')
@pytest.mark.timeout(900)
def test_the_follower_through_the_pool_keeps_its_lane_and_rate_and_matches_its_canary():
    if not _networks_present():
        pytest.skip('the frozen LA networks are not in this checkout (outputs/ is not in Git)')
    out = ROOT / 'outputs' / 'test-runs' / f'pytest-v15-follower-{uuid.uuid4().hex[:8]}'
    try:
        with pool.WorkerPool(2, out, priority='below_normal') as workers:
            assert workers.worker_priorities == {0: 'below_normal', 1: 'below_normal'}
            batch = workers.run([pool.Task('v14-validation', r, follower.DRIVER) for r in (10, 11)])
        assert json.loads((out / 'batches.jsonl').read_text().splitlines()[0])['priority'] == 'below_normal'
        assert batch.canary['matched'] and batch.canary['different_worker']
        assert batch.canary_decisions > 0 and batch.charged_decisions == batch.decisions + batch.canary_decisions
        decisions = 0
        for record in batch.records:
            assert record['fields']['driver'] == follower.DRIVER and record['decisions'] > 0
            trace = next(Path(record['directory']).glob('episode-*/actions.jsonl.gz'))
            with gzip.open(trace, 'rt') as handle:
                for line in handle:
                    for decision in json.loads(line)['agents'].values():
                        decisions += 1
                        measured = decision['v04_reward_audit']['pre_action']['ego_state']['acceleration']
                        assert decision['lane_request'] == 0
                        assert -4. - 1e-4 <= decision['requested_acceleration'] <= 2. + 1e-4
                        assert abs(decision['requested_acceleration'] - measured) <= .35 + 1e-4
        assert decisions == batch.decisions
    finally:
        shutil.rmtree(out, ignore_errors=True)
