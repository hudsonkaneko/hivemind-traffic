"""v15 Phase 0: the flat environment (environments/highway_merge_v15_env.py) against v14 round 24.

Fast tests run anywhere. The parity test drives the v14 environment (experiments/run_highway_flow_rl_v14.py
env_for, round 24) and the v15 environment with the round-16 starting policy (deterministic) on the same
validation rows and compares every decision exactly: the 121-field actor observations, the 2928-field critic
state, the applied acceleration, lane request handling, override, the states of every vehicle on the road
(background included), the simulation time, terminations, truncations and the agent set. It runs both on
libsumo, one simulation at a time, and skips where the frozen networks (outputs/ is not in Git), the warm
start or libsumo are missing.
"""
import importlib.util
import os
import shutil
import time
import uuid
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

os.environ.setdefault('OMP_NUM_THREADS', '1')

from environments import highway_merge_v15_env as flat  # noqa: E402
from environments.highway_merge_v15_env import HighwayMergeV15Env  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
HAS_LIBSUMO = importlib.util.find_spec('libsumo') is not None
# (round-24 validation row, step cap or None for the full row). Rows 8 and 9 are the LA merges (9 is the
# southbound case), 13 the LA passing row (slow leader), 0 the flat I-110 merge (8 cars, 18 m/s ceiling),
# 5 the 8-car cruise with 10% packet loss (the only rows with loss and more than one car are cruises).
PARITY_ROWS = ((8, None), (9, None), (0, None), (13, None), (5, 400))
INFO_FIELDS = ('requested_acceleration', 'applied_acceleration', 'lane_request', 'lane_applied_delta',
               'lane_rejected', 'override', 'override_causes')


# --- fast tests ---------------------------------------------------------------------------------
def test_flat_class_has_no_legacy_environment_base():
    from pettingzoo import ParallelEnv
    from environments.i110_merge_env import I110MergeEnv
    from environments.highway_flow_rl_v14_env import HighwayFlowRLV14Env
    legacy = [cls for cls in HighwayFlowRLV14Env.__mro__ if issubclass(cls, I110MergeEnv)]
    assert len(legacy) >= 14     # V14 ... I110MergeEnv
    assert not any(issubclass(HighwayMergeV15Env, cls) for cls in legacy)
    assert HighwayMergeV15Env.__mro__[1:] == ParallelEnv.__mro__
    # nothing in the module is (or builds) one of the legacy environment classes
    assert not [name for name, value in vars(flat).items()
                if isinstance(value, type) and issubclass(value, I110MergeEnv)]


def test_action_mapping_matches_v14():
    from environments.sumo_mappo_env import action_to_acceleration
    assert flat.map_action([-1., 0.]) == (-4.5, 0)
    assert flat.map_action([1., 1.]) == (2., 1)
    assert flat.map_action(np.array([0., -1.], np.float32)) == (-1.25, -1)
    # the float32 cast comes first, as in v05-v14
    value = flat.map_action([.1234567891, 0.])[0]
    assert value == action_to_acceleration(np.float32(.1234567891)) and value != action_to_acceleration(.1234567891)
    for invalid in ([0., .5], [1.5, 0.], [0., 2.], [np.nan, 0.]):
        with pytest.raises(ValueError):
            flat.map_action(invalid)


def test_acceleration_bounds_add_the_drive_bound_as_v14():
    from environments.highway_stage1_env import acceleration_constraints
    state = dict(speed=20., allowed_speed=26.8224)
    front = dict(bumper_gap_m=60., relative_speed_m_s=-1.)
    base_bounds, base_ceiling = acceleration_constraints(state, front, 300.)
    assert flat.acceleration_bounds(state, front, 300.) == (base_bounds, base_ceiling)
    for drive in (1.72, -5., 3.):
        bounds, ceiling = flat.acceleration_bounds(state, front, 300., drive)
        assert bounds == dict(base_bounds, drive_grade=drive)
        assert ceiling == float(np.clip(min(base_ceiling, drive), -4.5, 2.))


def test_drive_saturation_exempts_overrides_caused_only_by_the_drive_bound():
    from traffic.highway_flow_rl_v14_saturation import SETTINGS
    common = dict(rejected=False, requested=1.9, applied=1.7, refused=False, interrupted=False)
    assert flat.decision_override(causes=['drive_grade'], realized=1.7, saturation=SETTINGS, **common) == (
        False, [], True, False)
    assert flat.decision_override(causes=['drive_grade'], realized=1.7, **common) == (
        True, ['drive_grade'], False, False)
    assert flat.decision_override(causes=['drive_grade', 'comfort_rate_limit'], realized=1.7,
                                  saturation=SETTINGS, **common)[:3] == (True, ['comfort_rate_limit', 'drive_grade'], False)
    # a native change also counts, and so does a refused lane request
    assert flat.decision_override(causes=['drive_grade'], realized=1.6, saturation=SETTINGS, **common)[:3] == (
        True, ['drive_grade', 'native_realization_unattributed'], False)
    assert flat.decision_override(causes=[], realized=None, saturation=SETTINGS,
                                  **dict(common, applied=1.9, refused=True))[:3] == (
        True, ['native_lane_request_refused'], False)
    assert flat.decision_override(causes=[], realized=1.9, saturation=SETTINGS,
                                  **dict(common, applied=1.9))[:3] == (False, [], False)


def _network(tmp_path):
    import xml.etree.ElementTree as ET
    net = ET.Element('net')
    for edge, count, length in [('sb1', 4, 400), ('sb2', 4, 600), ('sb3', 4, 400)]:
        element = ET.SubElement(net, 'edge', id=edge)
        for index in range(count):
            ET.SubElement(element, 'lane', id=f'{edge}_{index}', index=str(index), length=str(length), speed='25')
    for source, target in [('sb1', 'sb2'), ('sb2', 'sb3')]:
        for index in range(4):
            ET.SubElement(net, 'connection', {'from': source, 'to': target, 'fromLane': str(index), 'toLane': str(index)})
    path = tmp_path / 'network.xml'
    ET.ElementTree(net).write(path)
    return path


def _vehicle(graph, name, lane, position, speed=20.):
    return dict(id=name, x=0., y=-position, lane_id=lane, lane_position=position, route=('sb1', 'sb2', 'sb3'),
                speed=speed, acceleration=0., distance=position, allowed_speed=25., length=4.8,
                lateral_position=0., lateral_speed=0., angle=180., lane_width=3.2, lane_identity=graph.identity(lane))


def _bare_env(tmp_path):
    """A v15 environment on a synthetic straight road with a mock SUMO connection (no simulation)."""
    from environments.highway_environment_v05 import ContinuousMotionLaneGraph
    from traffic import highway_flow_rl_v11_rewards as v11, highway_flow_rl_v12_rewards as v12
    from traffic import highway_flow_rl_v14_rewards as v14
    env = HighwayMergeV15Env.__new__(HighwayMergeV15Env)
    env.graph = ContinuousMotionLaneGraph(_network(tmp_path), 4)
    env.road_grade = env.drive_limit = env.follow_gate = env.drive_saturation = env.speed_limit_m_s = None
    env.v11_settings, env.v12_settings, env.v14_settings = dict(v11.SETTINGS_BY_ROUND[17]), dict(v12.SETTINGS), dict(v14.SETTINGS)
    env.controlled_count, env.total_vehicles, env.horizon = 2, 8, 3
    env.communication, env.packet_loss, env.task, env.lane_change_duration = 'local', 0., 'cruise', 3.
    env.rng = np.random.default_rng(0)
    env.conn = MagicMock()
    env.time = env.start_time = 0.
    env.steps = env.collision_count = env.teleport_count = 0
    env.arrived, env.collisions = set(), set()
    env.states = {'agent_0': _vehicle(env.graph, 'agent_0', 'sb1_1', 100.),
                  'agent_1': _vehicle(env.graph, 'agent_1', 'sb1_2', 120.),
                  'main_0': _vehicle(env.graph, 'main_0', 'sb1_1', 150., speed=18.)}
    env.agents = ['agent_0', 'agent_1']
    env.previous = {a: (0., 0) for a in env.agents}
    env.last_change = {a: -5. for a in env.agents}
    env.inbox = {a: {} for a in env.agents}
    env.maneuvers, env.completed_maneuvers = {}, {}
    env.completed_this_step, env.refused_this_step, env.interrupted_this_step = set(), {}, {}
    env.flow_contexts, env.trackers, env.merge_agents = {}, {}, ()
    env.quality_history, env.stable_merged = {}, set()
    env._reference_targets, env._v14_cache, env._ramp, env._follow = {}, {}, {}, {}
    env._faded, env._target_inputs, env._augmented = {}, {}, {}

    def advance():      # every car holds its speed for one tick; the commanded acceleration is realized
        env.time = round(env.time + .1, 10)
        commands = {call.args[0]: call.args[1] for call in env.conn.vehicle.setAcceleration.call_args_list}
        env.states = {vid: dict(s, lane_position=s['lane_position'] + s['speed'] * .1, y=s['y'] - s['speed'] * .1,
                                distance=s['distance'] + s['speed'] * .1, acceleration=commands.get(vid, 0.))
                      for vid, s in env.states.items()}
        env.completed_this_step, env.refused_this_step, env.interrupted_this_step = set(), {}, {}
    env._advance = advance
    return env


def test_step_returns_zero_rewards_and_the_audited_command(tmp_path):
    env = _bare_env(tmp_path)
    observations = env._actor_observations(env._raw_observations(env.agents))
    assert set(observations) == {'agent_0', 'agent_1'} and all(o.shape == (121,) for o in observations.values())
    assert env.state().shape == (flat.STATE_SIZE,)
    result = env.step({'agent_0': np.array([1., 0.], np.float32), 'agent_1': np.array([-1., -1.], np.float32)})
    observations, rewards, terminated, truncated, infos = result
    assert rewards == {'agent_0': 0., 'agent_1': 0.}
    assert set(infos['agent_0']) == {*INFO_FIELDS, 'drive_saturated'}
    first, second = infos['agent_0'], infos['agent_1']
    # +2 m/s^2 requested from 0: the comfort rate limit applies 0.4 (v05/v06), an override
    assert first['requested_acceleration'] == 2. and first['applied_acceleration'] == pytest.approx(.4)
    assert first['override'] and first['override_causes'] == ['comfort_rate_limit']
    # full braking bypasses the rate limit; the lane request is refused (gap to main_0 behind on sb1_1)
    assert second['applied_acceleration'] == -4.5 and second['lane_request'] == -1
    assert second['lane_rejected'] and second['lane_applied_delta'] == 0
    env.conn.vehicle.changeLane.assert_not_called()
    applied = {call.args[0]: call.args[1:] for call in env.conn.vehicle.setAcceleration.call_args_list}
    assert applied == {'agent_0': (first['applied_acceleration'], .1), 'agent_1': (-4.5, .1)}
    assert terminated == truncated == {'agent_0': False, 'agent_1': False}
    assert env.previous == {'agent_0': (2., 0), 'agent_1': (-4.5, -1)}   # raw requests feed field 5 and messages
    # each car now holds the other's message (one tick latency): slot 0 starts at 69 + 4 target inputs,
    # carries the sender's previous raw request (0 before its first decision) and its present flag
    assert set(env.inbox['agent_0']) == {'agent_1'} and set(env.inbox['agent_1']) == {'agent_0'}
    assert observations['agent_0'][73 + 4] == 0. and observations['agent_0'][73 + 7] == 1.
    while env.agents:
        result = env.step({a: np.zeros(2, np.float32) for a in env.agents})
    assert env.steps == 3 and result[3] == {'agent_0': True, 'agent_1': True}
    assert env.state()[flat.OBSERVATION_SIZE::flat.OBSERVATION_SIZE + 1].sum() == 0.


# --- parity with v14 round 24 -------------------------------------------------------------------
@pytest.fixture(scope='module')
def round24():
    try:
        from experiments import run_highway_flow_rl_v12 as runner
        from experiments import run_highway_flow_rl_v14 as runner14
        from traffic import highway_flow_rl_v14_contract as contract
        cfg = contract.build_config(24)
    except (FileNotFoundError, OSError, ImportError) as error:
        pytest.skip(f'Round-24 configuration unavailable: {error}')
    missing = [row['case']['network'] for row in (cfg['validation_rows'][i] for i, _ in PARITY_ROWS)
               if not (ROOT / row['case']['network']).is_file()]
    if not (ROOT / cfg['warm_start']['path']).is_file():
        missing.append(cfg['warm_start']['path'])
    if missing:
        pytest.skip(f'Frozen networks or warm start missing (outputs/ is not in Git): {sorted(set(missing))}')
    if not HAS_LIBSUMO:
        pytest.skip('libsumo is not installed in this environment')
    import torch
    from traffic import sumo_coupling
    torch.set_num_threads(1)
    runner.install = runner14.install
    runner.install()
    model, _ = runner.load_warm_start(cfg)
    model.eval()
    sumo_coupling.use('libsumo')
    try:
        yield cfg, runner.base, model
    finally:
        sumo_coupling.use('traci')


def _act(model, obs, agents):
    import torch
    local = torch.tensor(np.stack([obs.get(a, np.zeros(flat.OBSERVATION_SIZE, np.float32)) for a in flat.HighwayMergeV15Env.possible_agents]))
    with torch.no_grad():
        actions = model.act(local, deterministic=True)[0]
    return {a: actions[i].numpy() for i, a in enumerate(flat.HighwayMergeV15Env.possible_agents) if a in agents}


def _record(env, obs):
    return dict(time=env.time, agents=list(env.agents), obs={a: np.array(o) for a, o in obs.items()},
                state=np.array(env.state()), vehicles={vid: dict(s) for vid, s in env.states.items()})


def _v14_trace(cfg, base, model, row, directory, cap):
    env = base.env_for(cfg, row, directory)
    try:
        obs, _ = env.reset(seed=row['seed'])
        trace = [_record(env, obs)]
        while env.agents and (cap is None or len(trace) <= cap):
            actions = _act(model, obs, env.agents)
            nxt, _, terminated, truncated, infos = env.step(actions)
            trace.append(dict(_record(env, nxt), actions=actions, terminated=terminated, truncated=truncated,
                infos={a: dict({k: info[k] for k in INFO_FIELDS}, drive_saturated='drive_saturation' in info)
                       for a, info in infos.items()}))
            obs = {a: nxt[a] for a in env.agents}
        return trace
    finally:
        env.close()


def _compare(where, expected, actual):
    assert actual['time'] == expected['time'], f'{where}: time'
    assert actual['agents'] == expected['agents'], f'{where}: agent set'
    assert set(actual['obs']) == set(expected['obs']), f'{where}: observed agents'
    for agent, value in expected['obs'].items():
        if not np.array_equal(actual['obs'][agent], value):
            fields = np.flatnonzero(actual['obs'][agent] != value).tolist()
            raise AssertionError(f'{where}: {agent} observation fields {fields}: v14 {value[fields]} v15 {actual["obs"][agent][fields]}')
    if not np.array_equal(actual['state'], expected['state']):
        raise AssertionError(f'{where}: critic state fields {np.flatnonzero(actual["state"] != expected["state"]).tolist()[:20]}')
    assert set(actual['vehicles']) == set(expected['vehicles']), f'{where}: vehicles on the road'
    for vid, state in expected['vehicles'].items():
        assert actual['vehicles'][vid] == state, f'{where}: vehicle {vid} state'


@pytest.mark.parametrize('index, cap', PARITY_ROWS)
def test_v15_matches_v14_round24_decision_by_decision(round24, index, cap):
    cfg, base, model = round24
    row = cfg['validation_rows'][index]
    directory = ROOT / 'outputs/test-runs/v15-env-parity' / f'row-{index:02d}-{uuid.uuid4().hex[:8]}'
    started = time.perf_counter()
    trace = _v14_trace(cfg, base, model, row, directory / 'v14', cap)     # libsumo: one simulation at a time
    v14_seconds = time.perf_counter() - started
    env = HighwayMergeV15Env.from_row(cfg, row, directory / 'v15')
    decisions = 0
    try:
        obs, infos = env.reset(seed=row['seed'])
        assert infos == {a: {} for a in env.agents}
        _compare(f'row {index} reset', trace[0], _record(env, obs))
        for step, expected in enumerate(trace[1:], 1):
            where = f'row {index} step {step}'
            actions = _act(model, obs, env.agents)
            assert set(actions) == set(expected['actions']) and all(
                np.array_equal(actions[a], expected['actions'][a]) for a in actions), f'{where}: policy actions'
            nxt, rewards, terminated, truncated, infos = env.step(expected['actions'])
            assert rewards == {a: 0. for a in expected['actions']}, f'{where}: rewards'
            assert terminated == expected['terminated'] and truncated == expected['truncated'], f'{where}: ends'
            for agent, info in expected['infos'].items():
                assert {k: infos[agent][k] for k in (*INFO_FIELDS, 'drive_saturated')} == info, f'{where}: {agent} info'
            _compare(where, expected, _record(env, nxt))
            decisions += len(expected['actions'])
            obs = {a: nxt[a] for a in env.agents}
    finally:
        env.close()
    if cap is None:
        assert not trace[-1]['agents'], 'A full row runs until every car has ended'
    print(f'row {index}: {len(trace) - 1} steps, {decisions} decisions identical '
          f'(v14 {v14_seconds:.1f} s, v15 {time.perf_counter() - started - v14_seconds:.1f} s)')
    shutil.rmtree(directory, ignore_errors=True)
