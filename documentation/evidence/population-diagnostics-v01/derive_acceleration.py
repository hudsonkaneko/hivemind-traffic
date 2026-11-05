"""Derive acceleration evidence offline into a new ignored output directory.

Requires the preserved raw diagnostic run and historical population-v02 logs.
No simulator, optimizer, or training is started; accepted evidence is not edited.
"""
import argparse
import csv
from datetime import datetime, timezone
import gzip
import json
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from experiments.diagnose_population_acceleration import digest, emergency_events

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run', type=Path,
    default=Path('outputs/highway-population-diagnostics/v01/acceleration-20261105T051936Z-5526457f'),
    help='Preserved diagnostic run; relative paths resolve from the project root')
parser.add_argument('--output', type=Path,
    help='New directory under outputs; existing directories are rejected')
args = parser.parse_args()
RUN = (ROOT/args.run).resolve()
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
EVIDENCE = ((ROOT/args.output) if args.output else
    ROOT/'outputs/highway-population-diagnostics/v01'/f'offline-derived-{stamp}-{uuid.uuid4().hex[:8]}').resolve()
if not RUN.is_relative_to((ROOT/'outputs').resolve()) or not RUN.is_dir():
    raise ValueError('--run must be an existing retained run under the project outputs directory')
if not EVIDENCE.is_relative_to((ROOT/'outputs').resolve()):
    raise ValueError('--output must remain under the project outputs directory')
if EVIDENCE.exists():
    raise FileExistsError('Refusing to overwrite an existing output directory')
EVIDENCE.mkdir(parents=True, exist_ok=False)
raw = json.loads((RUN/'acceleration-summary.json').read_text())


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n',encoding='utf-8')


strata = {}
for path in sorted(RUN.glob('*.csv')):
    records = list(csv.DictReader(path.open(encoding='utf-8')))
    def choose(row, category):
        seam = row['road_seam']=='True'
        end = row['lane_end_distance_m']!=''
        near = float(row['speed_before_m_s'])>=float(row['speed_limit_m_s'])-.5
        return {'ordinary':not seam and not end and not near,'near_limit':near,'road_seam':seam,'ending_lane':end}[category]
    strata[path.stem] = {}
    for category in ('ordinary','near_limit','road_seam','ending_lane'):
        chosen = [r for r in records if r['realized_acceleration_m_s2']!='' and choose(r,category)]
        residual = [float(r['realized_acceleration_m_s2'])-min(float(r['filtered_acceleration_m_s2']),float(r['acc_model_prediction_m_s2'])) for r in chosen]
        strata[path.stem][category] = dict(samples=len(chosen),
            signed_residual_mean_m_s2=sum(residual)/len(residual) if residual else None,
            mean_absolute_residual_m_s2=sum(map(abs,residual))/len(residual) if residual else None,
            maximum_absolute_residual_m_s2=max(map(abs,residual),default=None),
            residual_beyond_point05=sum(abs(v)>.05 for v in residual))

episodes = []
for episode in raw['episodes']:
    ep_path = ROOT/episode['episode_directory']
    warnings = emergency_events((ep_path/'sumo-errors.log').read_text(encoding='utf-8'))
    report = episode['environment_report']
    episodes.append(dict(spec=episode['spec'],measurements=episode['measurements'],native_warnings=warnings,
        environment_outcomes={k:report.get(k) for k in ('successful_merges','passes','collisions','teleports','lane_changes','overspeed_steps','emergency_braking_observed','mean_fraction_of_speed_limit')},
        raw_csv_sha256=digest(RUN/(episode['spec']['id']+'.csv')),
        native_log_sha256=digest(ep_path/'sumo-errors.log')))

prior = ROOT/'outputs/highway-population/v01/train-20261105T045039Z-a6db8f1b'
history=[]
for native in sorted(prior.rglob('sumo-errors.log')):
    warnings=emergency_events(native.read_text(encoding='utf-8'))
    if not warnings['warning_lines']:
        continue
    journal=native.parent/'actions.jsonl.gz'
    rows=[json.loads(line) for line in gzip.open(journal,'rt',encoding='utf-8')]
    for event in warnings['events']:
        matches=[]
        for row in rows:
            if abs(row['time']-event['time_s'])<.101:
                a=row['agents'].get(event['vehicle_id'])
                if a:
                    matches.append(dict(journal_time_s=row['time'], time_minus_warning_s=round(row['time']-event['time_s'],6),
                        realized_acceleration=a['realized_acceleration'], requested_acceleration=a['requested_acceleration'],
                        filtered_acceleration=a['filtered_acceleration'], override_causes=a['override_causes']))
        history.append(dict(**event, episode=native.parent.relative_to(ROOT).as_posix(),
            controlled=event['vehicle_id'].startswith('agent_'), nearby_journal_samples=matches,
            native_log_sha256=digest(native), journal_sha256=digest(journal)))

historical=dict(schema='population_native_emergency_reconciliation_v01',prior_run=prior.relative_to(ROOT).as_posix(),
    warning_events=len(history),controlled_events=sum(x['controlled'] for x in history),
    background_events=sum(not x['controlled'] for x in history),
    controlled_nearby_samples_below_minus4point55=sum(s['realized_acceleration'] is not None and s['realized_acceleration'] < -4.55 for e in history for s in e['nearby_journal_samples']),
    note='Native lane-end emergency-stop deceleration/offset events are distinct from the post-step acceleration sample. Retain both; do not interpret zero sampled braking as zero native emergencies.',
    events=history)
write(EVIDENCE/'acceleration-historical-emergencies.json',historical)
compact=dict(schema='population_acceleration_evidence_v01',raw_run=RUN.relative_to(ROOT).as_posix(),
    raw_summary_sha256=digest(RUN/'acceleration-summary.json'),raw_executed_source_sha256=digest(RUN/'executed-diagnostic-source.py'),
    corrected_parser_source_sha256=digest(ROOT/'experiments/diagnose_population_acceleration.py'),
    source_commit=raw['source_commit'],git_status_observed_after_run=subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines(),
    git_status_limitation='Start-of-run dirty status was not captured. This is a post-run snapshot; exact source hashes in raw summary remain authoritative. Later runner revision captures start state.',
    full_resolved_config_path=(RUN/'resolved-config.json').relative_to(ROOT).as_posix(),
    full_resolved_config_sha256=digest(RUN/'resolved-config.json'),
    actor=raw['actor'],actor_sha256_after=raw['actor_sha256_after'],frozen_inputs_unchanged=raw['frozen_inputs_unchanged'],
    source_sha256_before=raw['source_sha256_before'],source_sha256_after=raw['source_sha256_after'],
    runtime=raw['runtime'],sumo_version=raw['episodes'][0]['runtime_version'],wall_seconds=raw['wall_seconds'],
    timing_scope='One bounded diagnostic wall time, not a throughput benchmark',
    decisions=sum(e['measurements']['agent_steps'] for e in episodes),
    diagnostic_speed_ratio_definition='CSV and measurements.speed_ratio use post-step speed / PRE-step limit to align with command context. Use environment_outcomes.mean_fraction_of_speed_limit for performance; it uses the post-step limit. Merge seams change limits.',
    maximum_derivative_error_m_s2=max(e['measurements']['derivative_max_error_m_s2'] for e in episodes),
    duration_comparisons=raw['duration_comparisons'],episodes=episodes,
    predictor_residual_definition='Realized acceleration minus min(filtered request, existing pure ACC response estimate); no fitted parameters',
    residual_strata_note='Ordinary excludes the other three categories. Near-limit, road-seam, and ending-lane categories may overlap. Ending-lane flag follows route-topology merge_distance, not literal nearest edge endpoint.',
    predictor_residuals=strata,primary_sources=raw['primary_sources'],api_limitations=raw['api_limitations'],
    native_warning_parser_correction='Initial raw summary searched emergency braking only. This corrected offline evidence searches emergency stop as well. All new diagnostic episodes still have zero of either kind.',
    historical_warning_evidence='acceleration-historical-emergencies.json')
write(EVIDENCE/'acceleration-summary.json',compact)
shutil.copyfile(RUN/'acceleration-curves.svg',EVIDENCE/'acceleration-curves.svg')
print(json.dumps(dict(output=str(EVIDENCE), historical_events=len(history),controlled=historical['controlled_events'],
                     sampled_emergencies=historical['controlled_nearby_samples_below_minus4point55'],new_run_warnings=sum(e['native_warnings']['warning_lines'] for e in episodes))))
