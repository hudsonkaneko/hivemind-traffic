"""The drive limit: road grade from the model surface and the physics car's force balance (no Isaac Sim)."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from traffic.road_grade import (AMERICANO_V14, DRIVE_LIMIT, ModelSurface, RoadGrade, _walk, drive_ceiling,
                                validate_drive_limit)

ROOT = Path(__file__).resolve().parents[1]
CORRIDORS = ROOT / 'outputs/highway-flow-rl/v12/la-corridors-exact-01'


def test_drive_ceiling_is_the_force_balance_of_the_package_values():
    r = .401728
    effective = 1800. + 4 * 1.225 / r ** 2
    assert drive_ceiling(AMERICANO_V14, 0., 0.) == pytest.approx(2 * 700. / r / effective)
    # 23 m/s up 5%: drive 3485 N, wheel damping 4 * 0.25 * 23 / r^2 = 142.5 N, slope 880.6 N
    expected = (2 * 700. / r - 4 * .25 * 23. / r ** 2 - 1800. * 9.81 * .05 / math.sqrt(1.0025)) / effective
    assert drive_ceiling(AMERICANO_V14, 23., .05) == pytest.approx(expected)
    assert 1.33 < expected < 1.36                        # PhysX measured about 1.37 there at full torque
    assert drive_ceiling(AMERICANO_V14, 23., -.05) > drive_ceiling(AMERICANO_V14, 23., 0.) > expected


def test_only_the_reviewed_limit_is_accepted():
    assert validate_drive_limit(dict(DRIVE_LIMIT, lookahead_s=[0., .1, .2])) == DRIVE_LIMIT
    with pytest.raises(ValueError):
        validate_drive_limit(dict(DRIVE_LIMIT, headroom_m_s2=0.))
    with pytest.raises(ValueError):
        validate_drive_limit(dict(DRIVE_LIMIT, vehicle=dict(AMERICANO_V14, max_drive_nm_per_wheel=900.)))


def scalar_height(x, y, surface):
    """scripts/render_i110_merge_replay.surface_elevation, restated."""
    nearest = []
    for lane in surface.values():
        best = (math.inf, 0.)
        for a, b in zip(lane['points'], lane['points'][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            u = max(0., min(1., ((x - a[0]) * dx + (y - a[1]) * dy) / max(dx * dx + dy * dy, 1e-12)))
            best = min(best, ((x - a[0] - u * dx) ** 2 + (y - a[1] - u * dy) ** 2, a[2] + u * (b[2] - a[2])))
        nearest.append((math.sqrt(best[0]), best[1]))
    nearest.sort()
    w = [(1. / max(d, .01) ** 2, z) for d, z in nearest[:2]]
    return sum(a * z for a, z in w) / sum(a for a, _ in w)


def test_vectorised_heights_match_the_renderer_rule():
    surface = {'a': dict(points=[(0., 0., 0.), (50., 0., 2.), (100., 5., 6.)]),
               'b': dict(points=[(0., 3.6, .1), (100., 8.6, 6.2)])}
    rng = np.random.default_rng(3)
    xy = np.column_stack([rng.uniform(0, 100, 40), rng.uniform(-1, 9, 40)])
    got = ModelSurface(surface).heights(xy, limit=None)
    assert got == pytest.approx([scalar_height(x, y, surface) for x, y in xy], abs=1e-9)
    with pytest.raises(ValueError):
        ModelSurface(surface).heights([[50., 30.]])                      # off the modelled road
    assert ModelSurface(surface).heights([[50., 30.]], check=[False]).shape == (1,)


def test_walk_continues_straight_past_both_ends():
    points = _walk([(0., 0.), (10., 0.), (10., 10.)], np.array([-2., 5., 15., 22.]))
    assert points == pytest.approx(np.array([[-2., 0.], [5., 0.], [10., 5.], [10., 12.]]))


def synthetic_grade(z_of_s):
    grade = RoadGrade.__new__(RoadGrade)
    grade.spec = dict(AMERICANO_V14)
    s = np.arange(-6., 200., .5)
    grade.profiles = {'e_0': (s, np.array([z_of_s(v) for v in s]))}
    grade.info = dict(source='synthetic')
    return grade


def test_grade_is_taken_between_the_axles_and_the_steepest_one_ahead_limits():
    flat_then_climb = synthetic_grade(lambda s: 0. if s < 100. else .08 * (s - 100.))
    assert flat_then_climb.grade('e_0', 50.) == pytest.approx(0.)
    assert flat_then_climb.grade('e_0', 150.) == pytest.approx(.08)
    # front axle 0.9085 m and rear axle 3.9085 m behind the bumper: half on the climb at s = 102.4085
    assert flat_then_climb.grade('e_0', 102.4085) == pytest.approx(.08 * 1.5 / 3.)
    now = flat_then_climb.ceiling('e_0', 99., 20., headroom_m_s2=.1, lookahead_s=(0.,))
    ahead = flat_then_climb.ceiling('e_0', 99., 20., headroom_m_s2=.1, lookahead_s=(0., .1, .2))
    assert now['limiting_grade'] == pytest.approx(0.) and ahead['limiting_grade'] > .03     # 4 m ahead is climbing
    assert ahead['ceiling_m_s2'] < now['ceiling_m_s2'] == pytest.approx(drive_ceiling(AMERICANO_V14, 20., 0.) - .1)
    with pytest.raises(ValueError):
        flat_then_climb.ceiling('missing_0', 10., 20.)


def test_a_network_without_a_road_model_is_flat():
    grade = RoadGrade.__new__(RoadGrade)
    grade.spec, grade.profiles = dict(AMERICANO_V14), {}
    assert grade.grade('any_0', 12.) == 0.


@pytest.mark.parametrize('corridor', ['la-110-sb-adams-onramp', 'la-110-nb-pasadena-onramp'])
def test_la_corridor_grades_come_from_the_model(corridor):
    folder = CORRIDORS / corridor
    if not (folder / 'network.net.xml').is_file():
        pytest.skip('LA corridor networks are local outputs, not in Git')
    pytest.importorskip('sumolib')
    grade = RoadGrade(folder / 'network.net.xml', folder / 'corridor-manifest.json')
    assert grade.info['source'].startswith('model lane graph') and grade.info['lanes'] == len(grade.profiles) > 20
    steepest = max(abs(grade.grade(lane, p)) for lane, (s, _) in grade.profiles.items()
                   for p in np.arange(4., s[-1] - 6., 2.))
    assert .05 < steepest < .09                         # 7.2% (southbound) and 6.8% (northbound) measured


def test_the_v14_env_adds_the_drive_bound_only_while_it_runs():
    from environments import highway_environment_v05 as v05_module
    from environments import highway_flow_env as flow_module
    from environments.highway_flow_rl_v14_env import HighwayFlowRLV14Env
    from environments.highway_stage1_env import acceleration_constraints as base
    env = HighwayFlowRLV14Env.__new__(HighwayFlowRLV14Env)
    env.drive_limit = dict(DRIVE_LIMIT)
    env.road_grade = synthetic_grade(lambda s: .06 * s)
    env._drive_patched = False
    env.drive_stats = dict(decisions=0, limited=0, max_grade=-np.inf, min_ceiling_m_s2=np.inf)
    state = dict(lane_id='e_0', lane_position=50., speed=20., allowed_speed=30.)
    with env._drive_limited():
        bounds, ceiling = v05_module.acceleration_constraints(state, None, None)
        assert flow_module.acceleration_constraints is v05_module.acceleration_constraints
        with env._drive_limited():                       # nested use (observations inside step) is a no-op
            assert v05_module.acceleration_constraints(state, None, None) == (bounds, ceiling)
    expected = drive_ceiling(AMERICANO_V14, 20., .06) - .1
    assert bounds['drive_grade'] == pytest.approx(expected) and ceiling == pytest.approx(expected)
    assert v05_module.acceleration_constraints is base and flow_module.acceleration_constraints is base
    assert base(state, None, None)[1] == 2.               # unchanged outside the environment
    with pytest.raises(ValueError):
        with env._drive_limited():
            v05_module.acceleration_constraints(dict(state, lane_id='missing_0'), None, None)
    assert v05_module.acceleration_constraints is base   # restored after an error too
    off = SimpleNamespace(road_grade=None, _drive_patched=False)
    with HighwayFlowRLV14Env._drive_limited(off):
        assert v05_module.acceleration_constraints is base
