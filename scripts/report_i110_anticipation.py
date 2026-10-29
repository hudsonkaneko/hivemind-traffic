"""Replay frozen development traces through current-only shadow margins."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from traffic.i110_anticipation import headway_signal, prior_warning


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def verify_run(run):
    manifest = read(run/'manifest.json')
    if manifest['status'] != 'completed':
        raise ValueError('Source run did not complete: '+str(run))
    for p, digest in {**manifest['inputs'], **manifest['outputs']}.items():
        if sha(run/p) != digest:
            raise ValueError('Source bytes changed: '+str(run/p))
    for p, digest in manifest['source_sha256'].items():
        if sha(run/'source'/p) != digest:
            raise ValueError('Executed source bytes changed: '+p)
    return manifest


def derive_run(run, plan, output):
    verify_run(run)
    config, metrics = read(run/'resolved-config.json'), read(run/'metrics.json')
    expected = dict(seed=plan['seed'], controller=plan['controller'], step_s=plan['step_s'],
                    horizon_s=plan['horizon_s'], speed_mode=31, controlled_lane_change_mode=512)
    if any(config[k] != v for k, v in expected.items()):
        raise ValueError('Source run violates frozen development contract')
    variants = [(f'{mode}-{h:g}', mode, h) for mode in plan['variants'] for h in plan['headways_s']]
    rows = {key:[] for key, _, _ in variants}
    previous = {}; in_exceedance = {}; episodes = []; switches = []; peak = None
    counts = Counter(); semantic = Counter()
    with (run/'trace.jsonl').open(encoding='utf-8') as source, output.open('x', encoding='utf-8') as journal:
        for line in source:
            frame = json.loads(line); t = frame['time']; states = frame['states']
            observations = frame['policy_observations']
            for vid in sorted(observations):
                state, obs = states[vid], observations[vid]
                phase = 'primary' if t <= plan['horizon_s']+1e-8 else 'drain'
                semantic['packets'] += 1
                semantic[f"lane_count_{obs['lane_count']}"] += 1
                semantic['unknown_adjacency'] += not obs['adjacent_context_known']
                semantic['unknown_route'] += obs['remaining_route_m'] is None
                for key, mode, headway in variants:
                    signal = headway_signal(state['speed'], obs, headway_s=headway,
                        standstill_gap_m=plan['standstill_gap_m'],
                        closing_deceleration_m_s2=plan['closing_deceleration_m_s2'],
                        adjacent=mode == 'own_and_adjacent')
                    row = dict(time_s=t, vehicle=vid, variant=key, phase=phase, signal=signal)
                    rows[key].append(row)
                    journal.write(json.dumps(row, sort_keys=True, allow_nan=False)+'\n')
                    counts[(key, phase, 'samples')] += 1
                    counts[(key, phase, 'warning_samples')] += signal['warning']
                    counts[(key, phase, 'unknown_samples')] += not signal['context_known']
                leader = state['leader'][0] if state.get('leader') else None
                if vid in previous:
                    old_t, old = previous[vid]
                    if abs(t-old_t-plan['step_s']) > 1e-7:
                        raise ValueError('Noncontiguous ego trace')
                    old_leader = old['leader'][0] if old.get('leader') else None
                    jerk = (state['acceleration']-old['acceleration'])/plan['step_s']
                    label = dict(vehicle=vid, time_s=t, leader=leader, old_leader=old_leader,
                                 jerk_m_s3=jerk, road_id=state['road_id'])
                    if peak is None or abs(jerk) > abs(peak['jerk_m_s3']):
                        peak = dict(label)
                    exceeds = abs(jerk) > plan['comfort_bound_m_s3']+1e-5
                    if exceeds and not in_exceedance.get(vid, False):
                        episodes.append(dict(label))
                    in_exceedance[vid] = exceeds
                    if leader and leader != old_leader:
                        switches.append(dict(label))
                previous[vid] = (t, state)
    if abs(abs(peak['jerk_m_s3'])-metrics['max_controlled_abs_jerk_m_s3']) > 1e-8:
        raise ValueError('Recomputed realized jerk disagrees with source metrics')
    # Retrospective labels cannot enter the already emitted shadow journal.
    for label in [*episodes, *switches, peak]:
        label['prior_warnings'] = {key:prior_warning(records, label['vehicle'], label['time_s'],
            label['leader'], plan['warning_window_s']) for key, records in rows.items()}
    exposure = {key:{phase:{name:counts[(key, phase, name)] for name in
        ('samples', 'warning_samples', 'unknown_samples')} for phase in ('primary', 'drain')} for key in rows}
    for phases in exposure.values():
        for values in phases.values():
            values['warning_vehicle_seconds'] = round(values['warning_samples']*plan['step_s'], 9)
            values['warning_fraction'] = values['warning_samples']/values['samples'] if values['samples'] else None
    return dict(run=run.name, seed=config['seed'], peak=peak, jerk_exceedance_episodes=episodes,
        leader_switches=switches, warning_exposure=exposure, semantic_packet_counts=dict(semantic),
        unchanged_baseline=dict(primary=metrics['primary'], drain=metrics['drain'],
            network_gate_passed=metrics['network_gate_passed'], comfort_gate_passed=metrics['comfort_gate_passed'],
            collisions=metrics['collisions'], min_gap_m=metrics['min_controlled_bumper_gap_m'],
            min_ttc_s=metrics['min_controlled_ttc_s'], interaction=read(run/'interaction-summary.json'),
            event=read(run/'event-summary.json') if (run/'event-summary.json').exists() else None))


def report(output, plan_path=ROOT/'experiments/configs/i110_anticipation_shadow_v1.json'):
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    plan_path = Path(plan_path).resolve(); plan = read(plan_path)
    if (plan['runs'] != ['ramp-0','ramp-38','ramp-44','ramp-50','cutin-smooth_acc-0','cutin-smooth_acc-1']
            or plan['variants'] != ['own','own_and_adjacent'] or plan['headways_s'] != [1.5, 2.0]):
        raise ValueError('Changed v1 matrix requires explicit review')
    sources = ['traffic/i110_anticipation.py', 'scripts/report_i110_anticipation.py']
    for name in sources:
        dest=output/'source'/name; dest.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(ROOT/name, dest)
    shutil.copyfile(plan_path, output/'plan.json')
    git = lambda *args: subprocess.check_output(['git','-c',f'safe.directory={ROOT.as_posix()}',*args],cwd=ROOT,text=True)
    (output/'source.diff').write_text(git('diff','HEAD'),encoding='utf-8')
    manifest = dict(status='started', started_utc=datetime.now(timezone.utc).isoformat(),
        command=[sys.executable,*sys.argv], python=sys.version, platform=platform.platform(),
        git_commit=git('rev-parse','HEAD').strip(), git_status=git('status','--short'),
        source_sha256={p:sha(ROOT/p) for p in sources}, inputs={str(plan_path):sha(plan_path)},
        simulator_launched=False, actor_loaded=False, training=False)
    write(output/'manifest.json',manifest)
    try:
        results=[]
        for name in plan['runs']:
            run=ROOT/plan['source_suite']/name
            manifest['inputs'][str(run/'manifest.json')]=sha(run/'manifest.json')
            results.append(derive_run(run, plan, output/(name+'-warnings.jsonl')))
        summary=dict(study_id=plan['study_id'], phase=plan['phase'], runs=results, limits=plan['limits'])
        write(output/'summary.json',summary)
        table=['# Development shadow margins', '',
            'Read-only reanalysis; no new motion or demonstrated intervention benefit.', '',
            '| Run | Headway (s) | Peak jerk (m/s³) | Own warning seconds | Own + adjacent warning seconds | Own / adjacent lead to peak (s) |',
            '|---|---:|---:|---:|---:|---|']
        for r in results:
            p=r['peak']['prior_warnings']; e=r['warning_exposure']
            for h in plan['headways_s']:
                own, adjacent = f'own-{h:g}', f'own_and_adjacent-{h:g}'
                table.append(f"| {r['run']} | {h:g} | {abs(r['peak']['jerk_m_s3']):.4f} | {e[own]['primary']['warning_vehicle_seconds']:.1f} | {e[adjacent]['primary']['warning_vehicle_seconds']:.1f} | {p[own]['earliest_lead_s']} / {p[adjacent]['earliest_lead_s']} |")
        table += ['', 'Warning seconds sum over controlled vehicles through H. Lead targets the actual later leader, uses strictly prior frames within 3 seconds, and can be intermittent. Null means no qualifying prior warning, not no hazard. Full ledgers, both headways, unknown context, episodes and leader switches are in summary.json.']
        (output/'results.md').write_text('\n'.join(table)+'\n',encoding='utf-8')
        manifest.update(status='completed',finished_utc=datetime.now(timezone.utc).isoformat(),
            outputs={p.relative_to(output).as_posix():sha(p) for p in output.rglob('*') if p.is_file() and p.name!='manifest.json'})
        write(output/'manifest.json',manifest)
        print('\n'.join(table))
        return summary
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));write(output/'manifest.json',manifest);raise


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    report(**vars(parser.parse_args()))
