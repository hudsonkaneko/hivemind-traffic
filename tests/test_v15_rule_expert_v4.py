"""Expert v4 (traffic/v15_rule_expert_v4.py) and the Level 2 drivers on its executor (traffic/v15_level2_v4.py): the
gap logic on synthetic streams, the copied methods against v3's, and the pinned v3 files unchanged. No SUMO run."""
import ast
import difflib
import hashlib
import inspect
import json
import math
import random
import textwrap
from pathlib import Path

import pytest

from traffic import v15_level2 as l2
from traffic import v15_level2_v4 as l2v4
from traffic import v15_rule_expert as v3
from traffic import v15_rule_expert_v4 as v4

ROOT = Path(__file__).resolve().parents[1]
S = v4.SETTINGS
PARAMS = (2.0, 4.5, 1.75, 2.5)          # a balanced background car (accel, decel, tau, min gap)
# (seconds until the car reaches our stop-line point, speed, id): a close stream, the last car 18.6 s away
ROW_11_LIKE = [(2., 21., 'main_10'), (4.5, 22., 'main_7'), (7., 22., 'main_4'), (9.5, 22., 'main_3'),
               (12., 22., 'main_2'), (14.5, 22., 'main_9'), (17., 22., 'main_8'), (18.6, 22., 'main_1')]


def _stream(cars):
    """[(seconds until the car reaches our stop-line point, speed, id)] as select_gap's stream."""
    return [(t * v, v, vid, PARAMS) for t, v, vid in cars]


def _lines(function):
    tree = ast.parse(textwrap.dedent(inspect.getsource(function))).body[0]
    if isinstance(tree.body[0], ast.Expr) and isinstance(tree.body[0].value, ast.Constant):
        tree.body = tree.body[1:]
    return ast.unparse(tree).splitlines()


def _changed(old, new):
    return [line[0] + line[1:].strip() for line in difflib.unified_diff(_lines(old), _lines(new), lineterm='', n=0)
            if line[:1] in '+-' and not line.startswith(('+++', '---'))]


# --------------------------------------------------------------------------- v3 unchanged, v4 a new driver

def test_the_files_the_recorded_runs_pin_are_unchanged():
    """Expert v3, the Level 2 module and the pool keep the bytes Stage 5's configuration recorded."""
    from traffic.highway_flow_contract import file_digest
    sources = json.loads((ROOT / 'configs/evaluation/v15-stage5-held-out.json').read_text())['sources']
    for path in ('traffic/v15_rule_expert.py', 'traffic/v15_level2.py', 'experiments/v15_pool.py'):
        assert file_digest(ROOT / path) == sources[path], path


def test_v4_is_a_new_driver_with_v3s_settings_plus_four():
    from experiments import v15_pool as P
    assert v4.DRIVER == 'rule:expert-v4' and v4.DRIVER not in P.RULE_DRIVERS and v3.DRIVER == 'rule:expert-v3'
    assert {k: v for k, v in S.items() if k not in v4.NEW_SETTINGS} == v3.SETTINGS
    assert set(v4.NEW_SETTINGS) == set(S) - set(v3.SETTINGS)
    assert (S['wait_brake_m_s2'], S['wait_lead_s'], S['hold_step_m_s'], S['run_up_floor_m_s2']) == (1.5, .5, .25, .3)
    assert S['wait_brake_m_s2'] == v3.SETTINGS['run_up_m_s2'] < 2.0       # below movement check M1's threshold
    assert S['entry_abort_m_s2'] == 3.0 and S['entry_safe_m_s2'] == 2.0       # v3's gap test and abort, kept


def test_each_copied_method_differs_from_its_source_only_in_its_marked_lines():
    """decide: the limits stored for change 4 and the committed target (change 3); rule_choice: the gap selection
    with the run-up the car can make (change 4). Every changed line is marked v4 in the source."""
    assert _changed(v3.RuleExpert.decide, v4.RuleExpert.decide) == [
        '+self._limits[agent] = (ceiling, cap)',
        '-target = max(target, min(self._free_road(state), speed))',
        '+target = committed_target(target, junction, speed, self._free_road(state))']
    assert _changed(l2.Level2Driver.decide, l2v4.Level2Driver.decide) == [
        '+self._limits[agent] = (ceiling, cap)',
        '-target = max(target, min(self._free_road(state), speed))',
        '+target = v4.committed_target(target, junction, speed, self._free_road(state))']
    assert _changed(l2.Level2Driver.rule_choice, l2v4.Level2Driver.rule_choice) == [
        "-plan = select_gap_from(stop, speed, stream, state.get('length', 5.0), self._free_road(state))",
        "+limited, top = v4.achievable(*v4.run_up_limits(self, env, agent, state, stop), cap=self._free_road(state))",
        "+plan = v4.select_gap(stop, speed, stream, state.get('length', 5.0), top, s=limited)"]
    for module in (v4, l2v4):
        source = inspect.getsource(module)
        for line in ('self._limits[agent] = (ceiling, cap)', 'committed_target(target, junction, speed'):
            assert all('# v4' in row for row in source.splitlines() if line in row and 'def ' not in row), line


def test_the_level2_ids_on_v4():
    assert l2v4.is_driver('l2v4:rule-v4') and l2v4.is_driver('l2v4:bc7c050f6d3da3ce')
    assert not l2v4.is_driver('l2:bc7c050f6d3da3ce') and not l2v4.is_driver('l2v4:xyz')
    assert l2v4.network_id('l2v4:bc7c050f6d3da3ce') == 'l2:bc7c050f6d3da3ce'
    with pytest.raises(ValueError):
        l2v4.network_id(l2v4.RULE_DRIVER)
    assert l2v4.settings_digest() != l2.settings_digest()
    with pytest.raises(TypeError):
        l2v4.Level2Driver(lambda: None, collect=True)            # validates only


# --------------------------------------------------------------------------- change 2: the run-up plans

def _arrive(stop, speed, w, run):
    """The time and distance of brake (wait_brake) to w, hold, run up (run_up) for `run` s, ending at the line."""
    a, b = S['run_up_m_s2'], S['wait_brake_m_s2']
    braking = (speed - w) / b
    braked = (speed * speed - w * w) / (2 * b)
    climbed = w * run + a * run * run / 2
    hold = (stop - braked - climbed) / w if w > 0 else 0.
    return braking + hold + run, braked + w * hold + climbed


@pytest.mark.parametrize('stop, speed, arrival', [(102., 12.1, 19.6), (80., 10., 9.), (60., 8., 12.), (129., 12.1, 7.5)])
def test_every_run_up_plan_crosses_the_line_exactly_at_the_planned_time(stop, speed, arrival):
    plans = v4.wait_plans(stop, speed, arrival, 26.5)
    holds = [w for _, w, _ in plans]
    assert holds == sorted(holds, reverse=True)               # gentlest (highest hold) first
    for u, w, run in plans:
        assert w <= speed + 1e-9 and u <= 26.5 + 1e-9 and math.isclose(u, w + S['run_up_m_s2'] * run, abs_tol=1e-9)
        if w > 0:
            t, d = _arrive(stop, speed, w, run)
            assert math.isclose(t, arrival, abs_tol=1e-6) and math.isclose(d, stop, abs_tol=1e-6)
        reach = w * run + .5 * S['run_up_m_s2'] * run * run
        assert w * w / (2. * max(reach - S['stop_line_margin_m'], .1)) <= S['entry_abort_m_s2'] + 1e-9


def test_a_lower_hold_crosses_the_line_faster():
    plans = v4.wait_plans(102., 12.1, 19.6, 26.5)
    assert len(plans) > 5 and plans[-1][0] > plans[0][0]


def test_the_options_are_v4s_plans_then_v3s():
    found = v4.options(80., 10., 9., 26.5)
    kinds = [k for _, _, k in found]
    assert kinds == ['run-up'] * kinds.count('run-up') + ['v3'] * kinds.count('v3') and 'v3' in kinds
    assert [(u, h) for u, h, k in found if k == 'v3'] == v3.entry_options(80., 10., 9., 26.5)


# --------------------------------------------------------------------------- the gap selection

def _random_streams(n, seed):
    rng = random.Random(seed)
    for _ in range(n):
        cars = sorted((rng.uniform(0., 25.), rng.uniform(15., 26.), f'main_{i}') for i in range(rng.randint(0, 6)))
        yield rng.uniform(30., 140.), rng.uniform(0., 20.), _stream(cars)


def test_a_car_that_can_go_now_goes_as_v3_does():
    same = 0
    for stop, speed, stream in _random_streams(400, 1):
        old = v3.select_gap(stop, speed, stream, 5., 26.5)
        if old['waited_for']:
            continue
        new = v4.select_gap(stop, speed, stream, 5., 26.5)
        assert {k: new[k] for k in old} == old and new['kind'] == 'go'
        same += 1
    assert same > 50


def test_v4_accepts_every_gap_v3_accepts_and_never_enters_after_v3():
    earlier = 0
    for stop, speed, stream in _random_streams(600, 2):
        old = v3.select_gap(stop, speed, stream, 5., 26.5)
        new = v4.select_gap(stop, speed, stream, 5., 26.5)
        assert len(new['waited_for']) <= len(old['waited_for'])
        if old['rear'] is not None:
            assert new['rear'] is not None
        earlier += len(new['waited_for']) < len(old['waited_for'])
        if new['rear'] is not None:
            assert new['response'] >= -S['entry_safe_m_s2'] and new['gap_at_entry'] > 0.
    assert earlier > 0


def test_with_skip_v4_waits_at_least_as_many_cars_and_matches_level2_when_going_now():
    for stop, speed, stream in _random_streams(300, 3):
        for skip in range(4):
            new = v4.select_gap(stop, speed, stream, 5., 26.5, skip=skip)
            if new['rear'] is not None:
                assert len(new['waited_for']) >= skip
            old = l2.select_gap_from(stop, speed, stream, 5., 26.5, skip=skip)
            if not old['waited_for']:
                assert {k: new[k] for k in old} == old


def test_a_row_11_like_wait_slows_gently_where_v3_brakes_at_its_floor():
    """No visible gap is acceptable (a close stream, the last car 18.6 s away): v3 holds at once the steady speed that
    arrives 1 s after the last (102 m in 19.6 s, 5.2 m/s, as on row 11), so its free term asks its -3.0 m/s^2 floor; v4
    slows at wait_brake_m_s2 toward the highest hold speed that still arrives on time."""
    stream = _stream(ROW_11_LIKE)
    old = v3.select_gap(102., 12.1, stream, 5., 26.5)
    assert old['rear'] is None and math.isclose(old['T'], 19.6) and math.isclose(old['hold'], 102. / 19.6)
    assert v3.free_term(12.1, old['hold']) == -3.0
    decision = v4.junction_plan(102., 12.1, stream, 5., 26.5, {})
    assert not decision['committed'] and decision['kind'] == 'none' and not decision['keep_speed']
    assert decision['hold_w'] == 4.0 and decision['hold_m_s'] == round(12.1 - 1.5 / 2., 3)
    assert v3.free_term(12.1, decision['hold_m_s']) == -1.5


def test_a_car_with_slack_keeps_its_speed():
    """A gap 10 s away at 100 m: holding the current 10 m/s, then a short run-up, crosses as the gap arrives (change 1)."""
    decision = v4.junction_plan(100., 10., _stream([(9., 22., 'a'), (20., 22., 'b')]), 5., 26.5, {})
    assert not decision['committed'] and decision['kind'] == 'run-up' and decision['rear'] == 'b'
    assert decision['keep_speed'] and decision['hold_m_s'] == 10.


def _wait(stream, stop, speed):
    """A waiting car under v4's targets (ideal tracking of v3's free term, no other term); its decisions and speeds."""
    t, dt, previous, decisions, speeds = 0., .1, {}, [], []
    while stop > 3. and t < 30.:
        now = [(d - v * t, v, vid, p) for d, v, vid, p in stream]
        decision = v4.junction_plan(stop, speed, now, 5., 26.5, previous)
        previous = decision
        decisions.append(decision)
        if decision['committed']:
            break
        a = v3.free_term(speed, min(26.5, decision['hold_m_s']))
        speeds.append((speed, a))
        speed = max(0., speed + a * dt)
        stop -= speed * dt
        t += dt
    return decisions, speeds


def test_a_wait_slows_at_the_plan_rate_and_never_below_its_first_hold():
    decisions, speeds = _wait(_stream(ROW_11_LIKE), 102., 12.1)
    assert min(a for _, a in speeds) >= -S['wait_brake_m_s2'] - 1e-9
    assert min(v for v, _ in speeds) >= decisions[0]['hold_w'] - S['hold_step_m_s'] - 1e-9     # no drift to a crawl


def test_a_wait_for_a_later_gap_holds_a_moderate_speed_then_commits_at_it():
    stream = _stream([(3., 22., 'a'), (5.5, 22., 'b'), (8., 22., 'c'), (16., 22., 'd')])
    decisions, speeds = _wait(stream, 110., 14.)
    last = decisions[-1]
    assert last['committed'] and not last['dilemma'] and last['rear'] == 'd'      # a go-now commit, not a dilemma
    assert min(v for v, _ in speeds) >= 11.5 and min(a for _, a in speeds) >= -S['wait_brake_m_s2'] - 1e-9
    assert v3.select_gap(110., 14., stream, 5., 26.5)['rear'] == 'd'          # the same gap as v3


# --------------------------------------------------------------------------- change 4: the run-up it can make

def test_with_no_limits_the_run_up_is_v3s():
    assert v4.achievable() == (S, v3.SETTINGS['free_road_m_s'])
    assert v4.achievable(ceiling=math.inf, curve=math.inf, cap=26.5) == (S, 26.5)
    margin = S['ceiling_margin_m_s2']
    assert v4.achievable(ceiling=S['run_up_m_s2'] + margin, cap=26.5) == (S, 26.5)     # the threshold: 1.55 m/s^2
    assert v4.achievable(ceiling=2.0, cap=26.5)[0] is S                                    # an ample ceiling


def test_the_run_up_is_at_most_the_ceiling_less_v3s_margin_and_at_least_the_floor():
    margin = S['ceiling_margin_m_s2']
    limited, cap = v4.achievable(ceiling=1.3, cap=26.5)                 # the Adams ramp's drive bound
    assert limited['run_up_m_s2'] == pytest.approx(1.3 - margin) and cap == 26.5
    assert {k: v for k, v in limited.items() if k != 'run_up_m_s2'} == {k: v for k, v in S.items() if k != 'run_up_m_s2'}
    assert v4.achievable(ceiling=0.)[0]['run_up_m_s2'] == S['run_up_floor_m_s2']          # a stopped leader ahead
    assert v4.achievable(ceiling=-4.5)[0]['run_up_m_s2'] == S['run_up_floor_m_s2']


def test_the_run_up_tops_out_at_the_curve_cap():
    assert v4.achievable(curve=17.6, cap=26.5) == (S, 17.6)
    assert v4.achievable(curve=30., cap=26.5) == (S, 26.5)


def test_a_slower_run_up_never_plans_an_earlier_crossing():
    """With the run-up the car can make, the planned crossing is never earlier than with v3's 1.5 m/s^2 run-up. (It
    may take a gap the full run-up rejects: arriving later, it lets the cars the full run-up waited for go by.)"""
    rng = random.Random(4)
    for _ in range(300):
        stop, speed = rng.uniform(20., 150.), rng.uniform(0., 20.)
        stream = _stream(sorted((rng.uniform(.5, 20.), rng.uniform(15., 25.), f'main_{k}') for k in range(rng.randint(0, 6))))
        full = v4.select_gap(stop, speed, stream, 5., 26.5)
        limited, top = v4.achievable(ceiling=rng.uniform(-1., 1.55), curve=rng.choice([None, rng.uniform(10., 26.)]),
                                     cap=26.5)
        slow = v4.select_gap(stop, speed, stream, 5., top, s=limited)
        assert slow['T'] >= full['T'] - 1e-9, (stop, speed, stream)
        assert slow['u'] <= top + 1e-9


def test_the_line_cap_is_v3s_curve_cap_seen_from_the_line():
    window = S['curve_window_m']
    curve = [(stop_ahead, .02) for stop_ahead in (60., 62., 64.)]          # a bend just past a line 50 m ahead
    assert v4.line_cap(curve, 50.) == v3.curve_cap([(a - 50., k) for a, k in curve], S)
    assert v4.line_cap(curve, 50.) < math.inf and v3.curve_cap([], S) == math.inf
    behind = [(50. - window, .05)]                                          # passed before the run-up's end
    assert v4.line_cap(behind, 50.) == math.inf


class _Car:
    """A driver stub for run_up_limits: the limits decide stored, and the car's curve points."""
    def __init__(self, limits, points):
        self._limits, self._points = limits, points

    def _curve_points(self, env, agent, state):
        return self._points


def test_run_up_limits_take_the_stored_ceiling_and_the_lower_curve_cap():
    bend = [(60., .02), (62., .02)]
    at_line = v4.line_cap(bend, 50.)
    assert v4.run_up_limits(_Car({}, bend), None, 'agent_0', {}, 50.) == (None, None)
    assert v4.run_up_limits(_Car({'agent_0': (1.2, None)}, bend), None, 'agent_0', {}, 50.) == (1.2, None)
    assert v4.run_up_limits(_Car({'agent_0': (1.2, math.inf)}, bend), None, 'agent_0', {}, 50.) == (1.2, at_line)
    assert v4.run_up_limits(_Car({'agent_0': (1.2, 5.)}, bend), None, 'agent_0', {}, 50.) == (1.2, min(5., at_line))


def test_the_junction_plan_records_the_run_up_it_assumed():
    plan = v4.junction_plan(102., 12.1, _stream(ROW_11_LIKE), 5., 26.5, {}, ceiling=1.3)
    assert plan['run_up_m_s2'] == pytest.approx(1.3 - S['ceiling_margin_m_s2'])
    assert v4.junction_plan(102., 12.1, _stream(ROW_11_LIKE), 5., 26.5, {})['run_up_m_s2'] == S['run_up_m_s2']


# --------------------------------------------------------------------------- change 3: the committed run-up

def test_a_committed_car_runs_up_where_its_gap_needs_it_and_keeps_v3s_target_otherwise():
    free = 26.5
    junction = dict(committed=True, rear='main_4', dilemma=False, needs_run_up=True)
    assert v4.committed_target(6., junction, 6., free) == 6. + S['run_up_m_s2'] / S['free_gain_per_s']
    assert v4.committed_target(20., junction, 6., free) == 20.                 # never below v3's target
    assert v4.committed_target(5., dict(junction, needs_run_up=False), 6., free) == 6.     # v3: max(target, speed)
    assert v4.committed_target(5., dict(junction, dilemma=True), 6., free) == 6.
    assert v4.committed_target(5., junction, 26.3, free) == free
    slow = dict(junction, run_up_m_s2=1.25)                                   # change 4: the run-up it can make
    assert v4.committed_target(6., slow, 6., free) == 6. + 1.25 / S['free_gain_per_s']


def test_needs_run_up_marks_a_gap_that_fails_at_the_current_speed():
    tight = v4.select_gap(60., 4., _stream([(12., 22., 'main_4')]), 5., 26.5)    # crossing at 4 m/s, 15 s from now
    assert tight['kind'] == 'go' and tight['rear'] == 'main_4' and tight['needs_run_up']
    roomy = v4.select_gap(60., 15., _stream([(20., 22., 'main_4')]), 5., 26.5)
    assert roomy['kind'] == 'go' and roomy['rear'] == 'main_4' and not roomy['needs_run_up']
