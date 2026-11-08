import pytest

from experiments.diagnose_population_acceleration import duration_comparison, emergency_events, finite_difference


def test_acceleration_uses_matching_timestamps_and_native_units():
    assert finite_difference(20., 20.15, 8.1, 8.2) == pytest.approx(1.5)
    with pytest.raises(ValueError):
        finite_difference(20., 20.15, 8.1, 8.1)


def test_warning_events_keep_time_and_id_and_deduplicate_lines():
    first = "Warning: Vehicle 'agent_0' performs emergency braking with decel=6.50, time=12.20."
    other = "Warning: Vehicle 'background_3' performs emergency braking with decel=7.10, time=12.20."
    result = emergency_events('\n'.join([first,first,other,'Warning: unrelated']))
    assert result['warning_lines'] == 3
    assert result['unique_vehicle_time_events'] == 2
    assert {event['vehicle_id'] for event in result['events']} == {'agent_0','background_3'}
    assert {event['time_s'] for event in result['events']} == {12.2}


def test_lane_end_emergency_stop_is_not_missed_by_braking_parser():
    line = "Warning: Vehicle 'agent_4' performs emergency stop at the end of lane 'sb3_0' because there is no connection to the next edge (decel=-26.71, offset=0.60), time=54.20."
    result = emergency_events(line)
    assert result['warning_lines'] == 1
    event = result['events'][0]
    assert (event['vehicle_id'], event['time_s'], event['kind'], event['native_reported_decel']) == ('agent_4',54.2,'lane_end_emergency_stop',-26.71)


def test_duration_comparison_detects_changed_trace_and_misalignment():
    row = dict(step=1,agent='agent_0',time_before_s=1.,time_after_s=1.1,speed_before_m_s=10.,
        speed_after_m_s=10.1,requested_acceleration_m_s2=1.,filtered_acceleration_m_s2=1.,realized_acceleration_m_s2=1.)
    result = duration_comparison([row], [dict(row,realized_acceleration_m_s2=.5)])
    assert result['max_absolute_difference']['realized_acceleration_m_s2'] == .5
    with pytest.raises(ValueError):
        duration_comparison([row],[dict(row,step=2)])
