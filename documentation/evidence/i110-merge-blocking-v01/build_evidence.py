"""Rebuild compact evidence from saved recordings only; never run SUMO."""
from collections import Counter, defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
BASELINE = ROOT / 'outputs/highway-environment/v05/visuals/i110-failure'
CORRECTED = ROOT / 'outputs/i110-merge-blocking/v01/replay-corrected-01'
DIAGNOSTIC = ROOT / 'outputs/i110-merge-blocking/v01'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def source(path):
    return dict(path=path.relative_to(ROOT).as_posix(), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def write(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def audit(directory):
    recording, report, manifest = (read(directory / name) for name in ('recording.json', 'evaluation.json', 'manifest.json'))
    episode = ROOT / report['episode_directory']
    times = [frame['time'] for frame in recording['frames']]
    samples = defaultdict(list)
    for frame in recording['frames']:
        for car in frame['vehicles']:
            samples[car['id']].append(dict(time=frame['time'], **car))
    trips = {trip['vehicle_id']: trip for trip in report['accounting']['trips']}
    vehicles = {}
    for identity, rows in sorted(samples.items()):
        stops, current, history = [], [], []
        def finish_stop():
            if current and current[-1]['time'] - current[0]['time'] >= 2. - 1e-7:
                stops.append(dict(start_s=current[0]['time'], end_s=current[-1]['time'],
                    observed_span_s=round(current[-1]['time'] - current[0]['time'], 6), samples=len(current),
                    lane_ids=list(dict.fromkeys(row['lane_id'] for row in current)),
                    still_stopped_at_horizon=abs(current[-1]['time'] - times[-1]) < 1e-7))
            current.clear()
        for row in rows:
            if not history or history[-1]['lane_id'] != row['lane_id']:
                history.append(dict(time=row['time'], lane_id=row['lane_id']))
            if current and row['time'] - current[-1]['time'] > .100001:
                finish_stop()
            if row['speed'] < .1:
                current.append(row)
            else:
                finish_stop()
        finish_stop()
        trip = trips[identity]
        vehicles[identity] = dict(controlled=identity in recording['controlled_ids'], origin=trip['origin'],
            requested_at=trip['requested_at'], departed_at=trip['departed_at'], arrived_at=trip['arrived_at'],
            ledger_state=trip['state'], first_observed_s=rows[0]['time'], last_observed_s=rows[-1]['time'],
            observed_samples=len(rows), stopped_samples=sum(row['speed'] < .1 for row in rows),
            minimum_speed_m_s=min(row['speed'] for row in rows), final_lane=rows[-1]['lane_id'],
            final_observed_speed_m_s=rows[-1]['speed'], present_at_horizon=abs(rows[-1]['time'] - times[-1]) < 1e-7,
            stops_at_least_2s=stops, lane_history=history)
    commands, lane_requests = defaultdict(Counter), []
    journal = episode / 'actions.jsonl.gz'
    with gzip.open(journal, 'rt', encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            for identity, info in row['agents'].items():
                values = commands[identity]
                values['decisions'] += 1
                values['overrides'] += int(info['override'])
                values['policy_filter_changes'] += int(info['policy_filter_changed'])
                values['native_realization_changes'] += int(info['native_realization_changed'])
                values['lane_request_' + str(info['lane_request'])] += 1
                if identity == 'agent_0' and info['lane_request']:
                    lane_requests.append(dict(time=row['time'], request=info['lane_request'],
                        rejected=info['lane_rejected'], applied_delta=info['lane_applied_delta']))
    assert sum(row['decisions'] for row in commands.values()) == report['agent_steps']
    assert sum(row['overrides'] for row in commands.values()) == report['overrides']
    warnings = (episode / 'sumo-errors.log').read_text(encoding='utf-8')
    selected = ('steps', 'agent_steps', 'overrides', 'intervention_fraction', 'policy_filter_changes', 'native_realization_changes',
        'collisions', 'teleports', 'overspeed_steps', 'emergency_braking_observed', 'completed_trips', 'requested_trips',
        'mean_fraction_of_speed_limit', 'primary_horizon_complete', 'accounting_comparison_valid', 'partial_control_horizon',
        'control_end_time', 'primary_horizon_time', 'post_control_native_steps', 'native_emergencies', 'post_control_events')
    summary = dict(directory=directory.relative_to(ROOT).as_posix(),
        artifacts={name: source(directory / name) for name in ('recording.json', 'evaluation.json', 'manifest.json',
            'recording-check.json', 'network.net.xml', 'policy.pt', 'source-config.json')},
        demand=source(episode / 'routes.rou.xml'), journal=source(journal), native_log=source(episode / 'sumo-errors.log'),
        raw_native_log_nonempty_lines=sum(bool(line.strip()) for line in warnings.splitlines()),
        raw_native_log_warning_lines=sum('warning' in line.lower() for line in warnings.splitlines()),
        frame_count=len(times), first_frame_s=times[0], last_frame_s=times[-1],
        exact_01s_frame_cadence=all(abs(b - a - .1) < 1e-7 for a, b in zip(times, times[1:])),
        observed_vehicle_count=len(vehicles), stopped_vehicle_count_at_least_2s=sum(bool(v['stops_at_least_2s']) for v in vehicles.values()),
        all_vehicle_stopped_samples=sum(v['stopped_samples'] for v in vehicles.values()),
        minimum_all_vehicle_observed_speed_m_s=min(v['minimum_speed_m_s'] for v in vehicles.values()),
        stopped_vehicle_ids_at_least_2s=[v for v, row in vehicles.items() if row['stops_at_least_2s']],
        stopped_at_horizon=[v for v, row in vehicles.items() if row['present_at_horizon'] and row['final_observed_speed_m_s'] < .1],
        ramp_vehicles=[v for v, row in vehicles.items() if row['origin'] == 'ramp'],
        remaining_ramp_vehicle_ids=[v for v, row in vehicles.items() if row['present_at_horizon'] and row['final_lane'] == 'mlk_on_0'],
        report={key: report.get(key) for key in selected}, accounting_totals=report['accounting']['totals'],
        accounting_groups=report['accounting']['groups'], stable_merge=report['stable_merge'],
        per_car_full_route=report['per_car_full_route'], journal_per_car={key: dict(value) for key, value in commands.items()},
        focal_nonzero_lane_requests=lane_requests,
        events={key: recording.get(key) for key in ('entry_events', 'junction_entry_events', 'merge_events', 'stable_events')},
        provenance={key: manifest.get(key) for key in ('seed', 'validation_case_index', 'selected_case', 'policy_sha256',
            'network_sha256', 'config_sha256', 'resolved_config_sha256', 'git_head', 'python', 'torch', 'readiness',
            'readiness_provenance', 'source_sha256', 'training', 'optimizer_steps')},
        sumo_runtime_version=report.get('sumo_runtime_version'), recording_check=read(directory / 'recording-check.json'))
    return summary, vehicles


def main():
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run from the canonical checkout root')
    summaries, vehicle_audits = {}, {}
    for name, directory in [('baseline', BASELINE), ('corrected', CORRECTED)]:
        summaries[name], vehicle_audits[name] = audit(directory)
    old, new = summaries['baseline'], summaries['corrected']
    assert old['demand']['sha256'] == new['demand']['sha256']
    assert old['provenance']['policy_sha256'] == new['provenance']['policy_sha256']
    assert old['provenance']['seed'] == new['provenance']['seed'] == 6900019
    assert old['frame_count'] == new['frame_count'] == 801
    previous_sources, current_sources = old['provenance']['source_sha256'], new['provenance']['source_sha256']
    pilot_source = ROOT / 'documentation/evidence/highway-environment-v05/results/comparison-summary.json'
    pilot = read(pilot_source)
    comparison = dict(schema='i110_merge_blocking_paired_saved_evidence_v01', training=False, optimizer_steps=0,
        held_out_test_run=False, scope='One fixed frozen-policy development case; not policy acceptance or retraining',
        same_actor=True, same_seed=True, byte_identical_demand=True,
        source_changes=[dict(path=key, before=previous_sources.get(key), after=current_sources.get(key))
                        for key in sorted(previous_sources.keys() | current_sources.keys())
                        if previous_sources.get(key) != current_sources.get(key)],
        evidence= summaries,
        accounting_note='Journal decisions include terminal arrival decisions; per_car_full_route only includes samples with an extant vehicle state',
        acceptance=dict(intervention_limit=.05, corrected_intervention_pass=new['report']['intervention_fraction'] <= .05,
                        candidate_accepted=False, historical_pilot_source=source(pilot_source),
                        unchanged_v05_pooled_initial_intervention_fraction=pilot['initial']['intervention_fraction'],
                        unchanged_v05_pooled_final_intervention_fraction=pilot['final']['intervention_fraction']))
    write('paired-summary.json', comparison)
    write('all-vehicle-audit.json', dict(definition='Speed <0.1 m/s in consecutive native frames; qualifying stop spans at least 2s between first and last observed samples. No interpolation or inferred absent-vehicle motion.',
        cadence_s=.1, vehicles=vehicle_audits))
    diagnostics = {}
    for name in ('acceleration-lifetime-01', 'native-handoff-01', 'network-ab-01', 'network-ab-02'):
        path = DIAGNOSTIC / name / 'summary.json'
        data = read(path)
        diagnostics[name] = dict(source=source(path), wall_seconds=data['wall_seconds'], policy_unchanged=data['policy_unchanged'],
            variants=[dict(**{key: variant.get(key) for key in ('duration_s', 'first_ramp_exit_s', 'stopped_samples', 'error',
                'network_sha256', 'demand_sha256', 'native_emergencies')},
                report={key: variant['report'].get(key) for key in ('overrides', 'agent_steps', 'intervention_fraction',
                    'successful_merges', 'completed_trips', 'collisions', 'teleports', 'overspeed_steps')}) for variant in data['variants']])
    write('diagnostic-history.json', diagnostics)
    for name in ('network-comparison.json', 'network-manifest.json'):
        shutil.copyfile(DIAGNOSTIC / 'network-v02' / name, OUT / name)
    shutil.copyfile(DIAGNOSTIC / 'diagnostic-getter-caveat.json', OUT / 'diagnostic-getter-caveat.json')
    write('source-index.json', dict(raw_root=str(ROOT), inputs=[source(path) for path in (
        DIAGNOSTIC / 'network-v02/network-comparison.json', DIAGNOSTIC / 'network-v02/network-manifest.json',
        DIAGNOSTIC / 'diagnostic-getter-caveat.json', ROOT / 'configs/diagnostics/i110-merge-blocking-v01.json')],
        builder=source(Path(__file__)), generated_files={path.name: source(path)['sha256'] for path in OUT.glob('*.json')
                                                     if path.name != 'source-index.json'}))
    print(json.dumps({name: dict(stopped_vehicles=row['stopped_vehicle_ids_at_least_2s'],
        stopped_at_horizon=row['stopped_at_horizon'], remaining_on_ramp=row['remaining_ramp_vehicle_ids'],
        overrides=row['report']['overrides'], decisions=row['report']['agent_steps']) for name, row in summaries.items()}, indent=2))


if __name__ == '__main__':
    main()
