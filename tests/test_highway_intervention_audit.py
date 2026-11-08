import gzip
import json

import pytest

from experiments.audit_highway_interventions import Accumulator, audit_reports, magnitude_bins


def row(request=1., realized=.2, target=.2, filtered=None, **extra):
    filtered = request if filtered is None else filtered
    result = dict(requested_acceleration=request, applied_acceleration=filtered,
        filtered_acceleration=filtered, realized_acceleration=realized,
        feasible_acceleration_target=target, override=realized is not None and abs(filtered-realized)>.05,
        override_causes=['native_realization_unattributed'] if realized is not None and abs(filtered-realized)>.05 else [],
        lane_request=0, lane_change_observed=False, lane_rejected=False)
    result.update(extra)
    return result


def frame(time, data=None, **extra):
    return dict(time=time, agents={'car': row() if data is None else data}, control_mode='policy', **extra)


def test_preserves_metric_and_distinguishes_acceleration_reduction_from_braking():
    a = Accumulator()
    a.add_frame(frame(.1))
    a.add_frame(frame(.2, row(request=.4, realized=-.6, target=-.6)))
    a.add_frame(frame(.3, row(request=.1, realized=.2, target=.1)))
    a.add_frame(frame(.4, row(request=.5, filtered=.2, realized=.2, target=.2,
                             override=True, override_causes=['speed_ceiling'])))
    report = a.report()
    counts = report['counts']
    assert counts['recorded_overrides'] == counts['recomputed_override_union'] == 4
    assert counts['override_union_mismatch'] == 0
    assert counts['native_changes'] == 3
    assert counts['native_reductions'] == 2 and counts['native_increases'] == 1
    assert counts['filter_changes'] == 1
    assert counts['native_changes_realized_braking'] == 1
    assert counts['native_changes_realized_nonbraking'] == 2
    assert counts['native_changes_requested_accel_realized_braking'] == 1
    assert counts['native_changes_target_matches_realized_within_0p05'] == 2
    assert report['native_change_runs']['duration_s']['maximum'] == pytest.approx(.3)


def test_unobserved_realization_is_not_fabricated_and_false_predictor_is_visible():
    a = Accumulator()
    a.add_frame(frame(.1, row(realized=None, target=None)))
    a.add_frame(frame(.2, row(request=1., realized=1., target=.1)))
    r = a.report()
    assert r['counts']['realization_unobserved'] == 1
    assert r['counts']['target_unobserved'] == 1
    assert r['counts']['native_changes'] == 0
    assert r['counts']['native_predicted_false_positive'] == 1
    assert r['acceleration']['filtered_minus_realized_m_s2']['signed']['count'] == 1
    assert r['native_target_agreement_fraction'] is None
    assert r['acceleration']['native_only_filtered_minus_realized_m_s2']['absolute']['count'] == 0


def test_runs_break_at_time_gaps_vehicle_disappearance_and_episode_boundaries():
    a = Accumulator()
    for t in (.1, .2, .4):
        a.add_frame(frame(t))
    a.add_frame(dict(time=.5, control_mode='policy', agents={}))
    a.add_frame(frame(.6))
    a.finish_episode()
    a.add_frame(frame(.7))
    r = a.report()
    assert r['native_change_runs']['count'] == 4
    assert r['native_change_runs']['decisions']['maximum'] == 2
    assert r['native_change_runs']['duration_s']['maximum'] == pytest.approx(.2)


def test_native_control_frames_excluded_and_break_runs():
    a = Accumulator()
    a.add_frame(frame(.1))
    a.add_frame(dict(time=.2, control_mode='native', agents={'car': row()}))
    a.add_frame(frame(.3))
    r = a.report()
    assert r['counts']['agent_decisions'] == 2
    assert r['counts']['non_policy_frames_excluded'] == 1
    assert r['native_change_runs']['count'] == 2


def test_magnitude_bins_cover_threshold_boundaries_once():
    r = magnitude_bins([0, -.05, .1, -.25, .5, -1, 2])
    assert list(r.values()) == [2, 1, 1, 1, 1, 1]


@pytest.mark.parametrize('field', ['requested_acceleration', 'realized_acceleration', 'feasible_acceleration_target'])
def test_nonfinite_values_fail_instead_of_silently_biasing_statistics(field):
    a = Accumulator()
    with pytest.raises(ValueError, match='Nonfinite'):
        a.add_frame(frame(.1, row(**{field: float('nan')})))


def test_journal_report_reconciliation_and_provenance(tmp_path):
    episode = tmp_path/'outputs'/'episode'
    episode.mkdir(parents=True)
    journal = episode/'actions.jsonl.gz'
    with gzip.open(journal, 'wt') as stream:
        stream.write(json.dumps(frame(.1))+'\n')
    report = dict(episode_directory='outputs/episode', case_id='fixture', task='following',
                  seed=4, agent_steps=1, overrides=1)
    source = tmp_path/'outputs'/'validation.json'
    source.write_text(json.dumps([report]))
    result = audit_reports(source, tmp_path)
    assert len(result['cases'][0]['journal_sha256']) == 64
    assert result['counts']['agent_decisions'] == 1
    report['overrides'] = 0
    source.write_text(json.dumps([report]))
    with pytest.raises(ValueError, match='counts disagree'):
        audit_reports(source, tmp_path)


def test_report_path_escape_is_rejected(tmp_path):
    source = tmp_path/'validation.json'
    source.write_text(json.dumps([dict(episode_directory='../foreign')]))
    with pytest.raises(ValueError, match='inside project outputs'):
        audit_reports(source, tmp_path)
