"""v15 event metrics: each stop, cut-in, emergency, collision and override episode counted once."""
import pytest

from traffic import v15_events as ev


def decision(*, stopped=False, free_road=False, override=False, causes=()):
    return dict(v14_stop=dict(stopped=stopped, free_road=free_road, asking_to_brake=False),
                override=override, override_causes=list(causes))


def trace(steps):
    """steps: list of {agent: decision} at 0.1 s spacing from t = 0.2 s."""
    return [dict(time=round(0.2 + 0.1 * i, 6), agents=agents, control_mode='policy') for i, agents in enumerate(steps)]


def report(**extra):
    base = dict(case_id='case', seed=1, collisions=0, intervention_fraction=None,
                v14_cut_in=dict(charged=0, events=[]), native_emergencies=dict(unique_events=0, events=[]))
    base.update(extra)
    return base


def test_step_comes_from_the_trace_and_needs_two_steps():
    assert ev.step_seconds(trace([{}, {}, {}])) == pytest.approx(0.1)
    with pytest.raises(ValueError):
        ev.step_seconds(trace([{}]))


def test_runs_join_neighbours_one_step_apart_only():
    assert ev.runs([0.2, 0.3, 0.4, 0.7, 0.8], 0.1) == [(0.2, 0.4, 3), (0.7, 0.8, 2)]
    assert ev.runs([], 0.1) == []


def test_a_long_stop_is_one_event_not_one_per_decision():
    steps = [{'agent_0': decision(stopped=True, free_road=True)} for _ in range(100)]
    result = ev.episode_events(report(), trace(steps))
    assert result['events']['stops'] == 1
    assert result['events']['free_road_stops'] == 1
    assert result['secondary']['free_road_stop_decisions'] == 100
    assert result['detail']['stops'][0]['seconds'] == pytest.approx(10.0)


def test_a_leader_passing_mid_stop_does_not_split_the_stop():
    flags = [True] * 5 + [False] * 3 + [True] * 5
    steps = [{'agent_0': decision(stopped=True, free_road=f)} for f in flags]
    result = ev.episode_events(report(), trace(steps))
    assert result['events']['free_road_stops'] == 1
    event = result['detail']['free_road_stops'][0]
    assert (event['decisions'], event['free_road_decisions']) == (13, 10)


def test_separate_stops_and_separate_cars_are_separate_events():
    moving, stopped = decision(), decision(stopped=True)
    steps = [{'agent_0': stopped, 'agent_1': stopped}] * 3 + [{'agent_0': moving, 'agent_1': stopped}] * 2 \
        + [{'agent_0': stopped, 'agent_1': moving}] * 2
    result = ev.episode_events(report(), trace(steps))
    assert result['events']['stops'] == 3          # agent_0 twice, agent_1 once
    assert result['events']['free_road_stops'] == 0


def test_override_episodes_carry_their_causes_and_keep_the_fraction_secondary():
    plain = decision()
    rate = decision(override=True, causes=['comfort_rate_limit'])
    drive = decision(override=True, causes=['drive_grade'])
    steps = [{'agent_0': rate}, {'agent_0': drive}, {'agent_0': plain}, {'agent_0': rate}]
    result = ev.episode_events(report(intervention_fraction=0.75), trace(steps))
    assert result['events']['override_episodes'] == 2
    assert result['detail']['override_episodes'][0]['causes'] == ['comfort_rate_limit', 'drive_grade']
    assert result['secondary']['override_decisions'] == 3
    assert result['secondary']['override_fraction'] == pytest.approx(0.75)
    assert result['secondary']['report_intervention_fraction'] == 0.75


def test_cut_ins_group_by_car_and_follower():
    rows = [dict(car='agent_0', follower='main_11', time_s=round(6.4 + 0.1 * i, 6)) for i in range(10)]
    rows += [dict(car='agent_0', follower='main_12', time_s=6.5), dict(car='agent_0', follower='main_11', time_s=9.0)]
    result = ev.episode_events(report(v14_cut_in=dict(charged=12, events=rows)), trace([{}, {}]))
    assert result['events']['cut_ins'] == 3
    assert result['secondary']['cut_in_charged_decisions'] == 12


def test_emergencies_group_by_vehicle_and_split_controlled():
    rows = [dict(vehicle_id='main_11', controlled=False, time_s=6.3), dict(vehicle_id='main_11', controlled=False, time_s=6.4),
            dict(vehicle_id='agent_2', controlled=True, time_s=8.0)]
    result = ev.episode_events(report(native_emergencies=dict(unique_events=3, events=rows)), trace([{}, {}]))
    assert result['events']['emergencies'] == 2
    assert result['events']['controlled_emergencies'] == 1
    assert result['secondary']['emergency_warning_lines'] == 3


def test_an_episode_has_at_most_one_collision_event():
    result = ev.episode_events(report(collisions=2), trace([{}, {}]))
    assert result['events']['collisions'] == 1
    assert result['secondary']['colliding_vehicle_steps'] == 2


def test_stop_threshold_is_only_a_report_and_pool_sums_episodes():
    long_stop = [{'agent_0': decision(stopped=True, free_road=True)} for _ in range(90)]
    short_stop = [{'agent_0': decision(stopped=True, free_road=True)} for _ in range(20)]
    a, b = ev.episode_events(report(), trace(long_stop)), ev.episode_events(report(), trace(short_stop))
    assert [ev.stops_longer_than(x, 8.0) for x in (a, b)] == [1, 0]
    pooled = ev.pool([a, b])
    assert pooled['episodes'] == 2 and pooled['decisions'] == 110
    assert pooled['events']['free_road_stops'] == 2 and pooled['episodes_with']['free_road_stops'] == 2
