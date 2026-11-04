"""Derive compact evidence from immutable local runs; no simulator or learning."""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[3]
assert Path.cwd().resolve() == ROOT
sys.path.insert(0, str(ROOT))
from traffic.highway_selection import selection_diagnostics

OUT = Path(__file__).parent
OLD = ROOT / 'outputs/highway-stage1/v02/train-20261103T190020Z-8469072d'
FAILED = ROOT / 'outputs/highway-stage1/v03/train-20261103T230113Z-dbea1564'
RUN = ROOT / 'outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761'

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def reference(path):
    return dict(path=path.relative_to(ROOT).as_posix(), sha256=hashlib.sha256(path.read_bytes()).hexdigest())

def write(name, data):
    (OUT / name).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')

cfg = read(RUN / 'config.json')
keys = ('seed', 'case_id', 'task', 'density', 'through_lanes', 'overrides', 'agent_steps',
        'intervention_fraction', 'collisions', 'teleports', 'overspeed_steps', 'passes',
        'successful_merges', 'mean_fraction_of_speed_limit', 'lane_changes',
        'mean_abs_acceleration', 'mean_abs_jerk', 'minimum_ttc_s', 'minimum_headway_s',
        'primary_horizon_complete', 'accounting_comparison_valid', 'reward',
        'native_realization_changes', 'policy_filter_changes')

def compact(path):
    reports = read(path)
    diagnostics = selection_diagnostics(reports, cfg)
    cruise = [r['mean_fraction_of_speed_limit'] for r in reports if r['task'] == 'cruise']
    return dict(source=reference(path), diagnostics=diagnostics,
                totals={k: sum(r[k] for r in reports) for k in
                        ('collisions', 'teleports', 'overspeed_steps', 'passes', 'successful_merges',
                         'lane_changes', 'requested_trips', 'completed_trips')},
                cruise_speed_fraction_min=min(cruise), cruise_speed_fraction_max=max(cruise),
                cases=[{k: r[k] for k in keys} | {'trip_totals': r['accounting']['totals']}
                       for r in reports])

selected = read(RUN / 'selection.json')['generation']
old_selected = read(OLD / 'selection.json')['generation']
sidecar = read(RUN / selected / 'policy-contract.json')
verified_artifacts = {}
for filename, key in (('policy.pt', 'policy_sha256'), ('best.pt', 'checkpoint_sha256'),
                      ('validation.json', 'validation_sha256')):
    item = reference(RUN / selected / filename)
    assert item['sha256'] == sidecar[key]
    verified_artifacts[filename] = item
current = compact(RUN / selected / 'validation.json')
previous = compact(OLD / old_selected / 'validation.json')
assert sidecar['validation_rank'] == list(current['diagnostics']['rank'])
assert read(RUN / selected / 'validation.json') == read(RUN / 'validation-phase-3.json')
phases = {name: compact(RUN / (name + '.json')) for name in
          ['validation-after-bc'] + [f'validation-phase-{i}' for i in range(4)]}
paired = []
for before, after in zip(previous['cases'], current['cases']):
    assert (before['seed'], before['case_id']) == (after['seed'], after['case_id'])
    paired.append(dict(seed=after['seed'], case_id=after['case_id'],
        override_fraction_before=before['intervention_fraction'],
        override_fraction_after=after['intervention_fraction'],
        override_fraction_delta=after['intervention_fraction']-before['intervention_fraction'],
        speed_fraction_before=before['mean_fraction_of_speed_limit'],
        speed_fraction_after=after['mean_fraction_of_speed_limit']))
write('comparison.json', dict(scope='Matched validation only, one paired training seed; no held-out claim',
    v02_selected=previous, v03_selected=current, v03_phases=phases, paired_cases=paired,
    pooled_override_percentage_point_change=100*(current['diagnostics']['pooled_override_fraction']-
                                              previous['diagnostics']['pooled_override_fraction']),
    original_selections_preserved=True, held_out_test_run=False))

summaries = []
provenances = []
for directory in (FAILED, RUN):
    summary = read(directory / 'summary.json')
    provenance = read(directory / 'provenance.json')
    assert summary['sources_changed'] == [] and summary['held_out_test_run'] is False
    for path, expected in provenance['source_hashes'].items():
        assert reference(directory / 'source' / path)['sha256'] == expected
        assert reference(ROOT / path)['sha256'] == expected
    summaries.append(dict(source=reference(directory / 'summary.json'),
        metrics={k:v for k,v in summary.items() if k not in ('source_hashes', 'geometry')}))
    provenances.append(dict(source=reference(directory / 'provenance.json'), **provenance))
assert provenances[0]['source_hashes'] == provenances[1]['source_hashes']
assert provenances[0]['config_sha256'] == provenances[1]['config_sha256']
write('run-evidence.json', dict(scope='One authorized attempt with a budget-preserving checkpoint recovery',
    selected_generation=selected, selected_artifacts=verified_artifacts,
    runs=summaries, provenance=provenances,
    aggregate_reported_elapsed_seconds=summaries[1]['metrics']['elapsed_seconds'],
    sum_segment_wall_seconds=sum(s['metrics']['wall_seconds'] for s in summaries),
    actual_agent_transitions=summaries[1]['metrics']['actual_agent_transitions'],
    recovery_note='Initial Windows access-denied replacement error; rolled back to BC step 212, '
                  'then resumed without source/config changes. The log records one discarded BC '
                  'update at step 213, which is retried; counters describe committed optimizer state.',
    learning_budget_reset=False, held_out_test_run=False, source_hashes_and_artifacts_verified=True))

import torch
torch.set_num_threads(1)
data_path = RUN / 'demonstrations-6200015.pt'
actor_path = RUN / 'selected-00000-c127e64d/policy.pt'
data = torch.load(data_path, map_location='cpu', weights_only=False)
actor = torch.jit.load(str(actor_path), map_location='cpu').eval()
with torch.no_grad():
    prediction = torch.cat([actor(x) for x in data['observations'].split(512)])
error = (prediction[:, 0] - data['actions'][:, 0]) * 3.25
absolute = error.abs()
fit = dict(scope='Frozen after-BC actor on retained TRAINING demonstrations; not validation/control accuracy',
    learning=False, optimizer_steps=0, simulator_run=False, samples=len(prediction),
    acceleration_rmse_m_s2=float(error.square().mean().sqrt()),
    acceleration_mae_m_s2=float(absolute.mean()), acceleration_bias_m_s2=float(error.mean()),
    acceleration_error_p50_m_s2=float(absolute.quantile(.5)),
    acceleration_error_p95_m_s2=float(absolute.quantile(.95)),
    fraction_label_error_above_0p05=float((absolute > .05).float().mean()),
    lane_agreement=float((prediction[:, 1] == data['actions'][:, 1]).float().mean()),
    sources=[reference(actor_path), reference(data_path)], torch=torch.__version__)
write('bc-fit.json', fit)
print(json.dumps(dict(selected=current['diagnostics'], totals=current['totals'],
    phase_trend={name:{key: item['diagnostics'][key] for key in
                      ('pooled_override_fraction', 'worst_override_fraction', 'override_failure_cases')}
                 for name,item in phases.items()}, fit=fit), indent=2))
