import pytest

pytest.importorskip('torch')

from environments.highway_flow_rl_v05_env import HANDOFF_MAX_STEPS, HighwayFlowRLV05Env


def env(variant):
    instance = HighwayFlowRLV05Env.__new__(HighwayFlowRLV05Env)  # logic only, no SUMO
    instance.v05_variant = variant
    instance._reset_v05()
    instance.states = {'agent_0': dict(acceleration=0.)}
    return instance


def drive(instance, requests, ceiling=2.):
    applied = []
    for request in requests:
        value = instance._condition_acceleration('agent_0', request, ceiling)
        instance.states['agent_0']['acceleration'] = min(value, ceiling)  # ideal realization
        applied.append(round(value, 6))
    return applied


def test_startup_ramps_first_command_then_returns_to_legacy():
    instance = env('startup')
    applied = drive(instance, [1.2, 1.2, 1.2, 1.2, -1.0])
    # 4 m/s3 * 0.1 s = 0.4 m/s2 per step until the request is reached...
    assert applied[:4] == [0.4, 0.8, 1.2, 1.2]
    # ...after which the actor's commands are applied unchanged (legacy).
    assert applied[4] == -1.0
    assert instance.v05_metrics['handoff'] == dict(cars=1, conditioned_decisions=2,
                                                    completed_by_catch_up=1, completed_by_limit=0)


def test_startup_jerk_never_exceeds_gate_during_handoff():
    instance = env('startup')
    applied = drive(instance, [2.0] * 10)
    jerks = [abs(b - a) / .1 for a, b in zip([0.] + applied, applied)]
    assert max(jerks) <= 5.0 + 1e-9


def test_maximum_braking_bypasses_handoff_limit():
    instance = env('startup')
    assert drive(instance, [-4.5]) == [-4.5]


def test_lower_safety_ceiling_is_applied_immediately():
    instance = env('startup')
    instance.states['agent_0']['acceleration'] = 1.0
    value = instance._condition_acceleration('agent_0', -3.0, -2.5)
    assert value == -3.0  # ceiling below the comfort window: request passes unconditioned


def test_handoff_stops_after_limit():
    instance = env('startup')
    instance.states['agent_0']['acceleration'] = 0.
    for _ in range(HANDOFF_MAX_STEPS):
        instance._condition_acceleration('agent_0', 2.0, 2.0)  # never realized: cannot catch up
    assert instance._handoff['agent_0'] is None
    assert instance._condition_acceleration('agent_0', 2.0, 2.0) == 2.0
    assert instance.v05_metrics['handoff']['completed_by_limit'] == 1


def test_legacy_never_conditions():
    instance = env('legacy')
    assert drive(instance, [1.2, -3.0, 2.0]) == [1.2, -3.0, 2.0]
