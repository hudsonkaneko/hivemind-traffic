from copy import deepcopy

import pytest

from traffic.sumo_warnings import native_emergency_report, parse_native_emergencies
from experiments import run_highway_population as runner


def test_native_lane_end_event_is_retained_even_without_acceleration_threshold_crossing(tmp_path):
    line = "Warning: Vehicle 'agent_4' performs emergency stop at the end of lane 'sb3_0' (decel=-26.71, offset=0.60), time=54.20."
    other = "Warning: Vehicle 'human_0' performs emergency braking with decel=6.50, time=54.20."
    path = tmp_path / 'sumo-errors.log'
    path.write_text('\n'.join((line, line, other)), encoding='utf-8')
    value = native_emergency_report(path)
    assert value['available'] and value['warning_lines'] == 3
    assert value['unique_events'] == 2 and value['controlled_events'] == 1 and value['background_events'] == 1
    assert value['events'][0]['time_s'] == 54.2
    assert value['events'][0]['native_reported_decel'] == -26.71
    assert value['events'][0]['kind'] == 'lane_end_emergency_stop'
    assert native_emergency_report(tmp_path / 'missing.log')['unique_events'] is None


def test_unknown_emergency_warning_is_not_discarded():
    value = parse_native_emergencies('Warning: emergency stop (unknown format)')
    assert value['unique_events'] == 1 and value['unclassified_events'] == 1


def clean_report():
    return dict(agent_steps=100, overrides=0, collisions=0, teleports=0, overspeed_steps=0,
        intervention_fraction=0., mean_fraction_of_speed_limit=.95, passes=0, successful_merges=0,
        accounting_comparison_valid=True, partial_control_horizon=False, task='cruise',
        case_id='train-cruise', seed=1, population_setting={}, native_warning_accounting_required=True,
        native_emergencies=dict(available=True, unique_events=0))


def test_v3_gates_on_native_events_and_missing_native_accounting():
    report = clean_report()
    assert runner.summarize([report])['nominal_gates_passed']
    report['native_emergencies']['unique_events'] = 1
    assert 'native_emergency' in runner.summarize([report])['failures'][0]['causes']
    del report['native_emergencies']
    assert 'native_accounting_unavailable' in runner.summarize([report])['failures'][0]['causes']


def test_legacy_gate_is_not_silently_redefined_but_new_events_are_visible():
    report = clean_report()
    report['native_warning_accounting_required'] = False
    report['native_emergencies']['unique_events'] = 1
    result = runner.summarize([report])
    assert result['nominal_gates_passed'] and not result['native_event_gate_applied']
    assert result['native_emergency_events'] == 1


def test_v3_config_requires_versioned_target_diagnostics_and_explicit_output():
    from environments.highway_stage1_env import REWARD_V3_VERSION
    from traffic.highway_merge_context import CONTROL_OBSERVATION_VERSION
    cfg = runner.load_config(runner.ROOT / 'experiments/configs/highway_population_v3.json')
    assert cfg['training_authorized'] is False
    assert cfg['environment']['control_target_version'] == 'v3'
    assert cfg['optimizer']['gradient_diagnostics'] is True
    assert cfg['optimizer']['imitation_acceleration_weight'] == 1.
    assert cfg['output_directory'] == 'outputs/highway-population/v03'
    assert cfg['reward_version'] == REWARD_V3_VERSION
    assert cfg['contract']['observation_semantics'] == CONTROL_OBSERVATION_VERSION
    for mutate in (
        lambda c: c['optimizer'].update(gradient_diagnostics=False),
        lambda c: c.update(output_directory='../outside'),
        lambda c: c.update(reward_version='highway_stage1_feasible_acceleration_v2'),
    ):
        bad = deepcopy(cfg); mutate(bad)
        with pytest.raises(ValueError):
            runner.validate_config(bad)
