import gzip
import json

import pytest

from experiments.summarize_highway_actions import summarize, summarize_journal


def write_rows(tmp_path, rows):
    path = tmp_path/'actions.jsonl.gz'
    with gzip.open(path, 'wt') as stream:
        for frame in rows:
            stream.write(json.dumps(frame)+'\n')
    return path


def decision(**updates):
    return dict(requested_acceleration=2., applied_acceleration=1.,
                realized_acceleration=.5, override=True, **updates)


def test_legacy_differences_do_not_invent_native_causes(tmp_path):
    path = write_rows(tmp_path, [dict(control_mode='policy', agents={'a':decision()})])
    report = summarize_journal(path)
    assert report['counts']['requested_filtered_difference'] == 1
    assert report['counts']['filtered_realized_difference'] == 1
    assert report['counts']['decisions_without_cause_telemetry'] == 1
    assert report['cause_decision_counts'] == {}


def test_causes_overlap_but_count_decisions_once_and_exclude_native(tmp_path):
    row = decision(intervention_causes=['following', 'speed_cap', 'following'])
    path = write_rows(tmp_path, [dict(control_mode='policy', agents={'a':row}),
                                 dict(control_mode='native', agents={'a':row})])
    report = summarize([path])
    assert report['counts']['agent_decisions'] == 1
    assert report['counts']['recorded_overrides'] == 1
    assert report['cause_decision_counts'] == {'following':1, 'speed_cap':1}
    assert report['counts']['non_policy_frames_excluded'] == 1


def test_nonfinite_commands_and_empty_audit_fail(tmp_path):
    row = decision()
    row['realized_acceleration'] = float('nan')
    path = write_rows(tmp_path, [dict(control_mode='policy', agents={'a':row})])
    with pytest.raises(ValueError, match='Nonfinite'):
        summarize_journal(path)
    with pytest.raises(ValueError, match='No action journals'):
        summarize([])


def test_departed_vehicle_realization_is_unknown_not_fabricated(tmp_path):
    row = decision(override_causes=['lane_mask'])
    row['realized_acceleration'] = None
    path = write_rows(tmp_path, [dict(control_mode='policy', agents={'a':row})])
    report = summarize_journal(path)
    assert report['counts']['realization_unobserved'] == 1
    assert report['acceleration_statistics_observed_decisions'] == 0
    assert report['cause_decision_counts'] == {'lane_mask':1}
    assert 'mean_requested_filtered_realized_m_s2' not in report
