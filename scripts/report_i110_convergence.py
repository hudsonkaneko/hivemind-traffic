"""Reanalyse frozen development traces; emit signals before scoring future labels."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from traffic.i110_convergence import ConvergenceGraph, screen
from scripts.report_i110_anticipation import read, write, sha, verify_run

STATE_FIELDS = ('lane_id','lane_position','length','speed','acceleration','x','y')


def variants(plan):
    return [(f'r{radius:g}-{model}-h{h:g}', radius, f'{model}-h{h:g}')
            for radius in plan['ranges_m'] for model in plan['other_models'] for h in plan['headways_s']]


def score(label, histories, plan):
    result = {}
    records = histories.get((label['vehicle'], label['leader']), [])
    for key, radius, envelope_key in variants(plan):
        eligible = [(t, c) for t, c in records if c['distance_m'] <= radius
                    and 1e-8 < label['time_s']-t <= plan['warning_window_s']+1e-8]
        supported = [(t,c) for t,c in eligible if c['envelopes'].get(envelope_key, {}).get('known')]
        deficits = [(t,c) for t,c in supported if c['envelopes'][envelope_key]['deficit']]
        first = supported[0] if supported else None
        result[key] = dict(prior_candidate_samples=len(eligible), supported_samples=len(supported),
            deficit_samples=len(deficits),
            earliest_deficit_lead_s=round(label['time_s']-deficits[0][0],9) if deficits else None,
            latest_deficit_lead_s=round(label['time_s']-deficits[-1][0],9) if deficits else None,
            first_supported=(dict(time_s=first[0], lead_s=round(label['time_s']-first[0],9),
                projection=first[1]['projection'], envelope=first[1]['envelopes'][envelope_key]) if first else None))
    return result


def derive(run, plan, graph, output):
    verify_run(run)
    config, metrics = read(run/'resolved-config.json'), read(run/'metrics.json')
    expected = dict(seed=plan['seed'], controller=plan['controller'], step_s=plan['step_s'],
                    horizon_s=plan['horizon_s'], speed_mode=31, controlled_lane_change_mode=512)
    if any(config[k] != v for k,v in expected.items()):
        raise ValueError('Source run differs from frozen development contract')
    manifest = read(run/'manifest.json')
    network_digest = sha(ROOT/plan['network'])
    if network_digest not in manifest['inputs'].values():
        raise ValueError('Diagnostic network not hash-matched to source run')
    counts = Counter(); histories = defaultdict(list); previous = {}; exceeding = {}
    switches = []; episodes = []; peak = None
    with (run/'trace.jsonl').open(encoding='utf-8') as source, output.open('x', encoding='utf-8') as journal:
        for line in source:
            frame = json.loads(line); t = frame['time']; states = frame['states']
            # Only contemporaneous geometry/kinematics are exposed to the signal.
            current = {vid:{k:s[k] for k in STATE_FIELDS} for vid,s in states.items()}
            phase = 'primary' if t <= plan['horizon_s']+1e-8 else 'drain'
            for vid in sorted(frame['policy_observations']):
                signal = screen(vid, current, graph, plan)
                journal.write(json.dumps(dict(time_s=t, vehicle=vid, phase=phase, signal=signal),
                                         sort_keys=True, allow_nan=False)+'\n')
                for c in signal['candidates']:
                    histories[(vid, c['id'])].append((t,c))
                for key, radius, ek in variants(plan):
                    candidates = [c for c in signal['candidates'] if c['distance_m'] <= radius]
                    envelopes = [c['envelopes'][ek] for c in candidates if ek in c['envelopes']]
                    counts[(key,phase,'samples')] += 1
                    counts[(key,phase,'candidate_samples')] += bool(candidates)
                    counts[(key,phase,'forward_candidate_samples')] += bool(envelopes)
                    counts[(key,phase,'deficit_samples')] += any(e.get('deficit',False) for e in envelopes)
                    counts[(key,phase,'unknown_samples')] += (not signal['ego_context_known']
                        or not signal['stopping_profile_known']
                        or any(c['projection']['status']=='unknown_lane_context' for c in candidates))
                state = states[vid]
                leader = state['leader'][0] if state.get('leader') else None
                if vid in previous:
                    old_t, old = previous[vid]
                    if abs(t-old_t-plan['step_s']) > 1e-7:
                        raise ValueError('Noncontiguous controlled trace')
                    old_leader = old['leader'][0] if old.get('leader') else None
                    jerk = (state['acceleration']-old['acceleration'])/plan['step_s']
                    label = dict(vehicle=vid, time_s=t, leader=leader, old_leader=old_leader,
                                 jerk_m_s3=jerk, lane_id=state['lane_id'])
                    if peak is None or abs(jerk) > abs(peak['jerk_m_s3']):
                        peak = dict(label)
                    exceed = abs(jerk) > plan['comfort_bound_m_s3']+1e-5
                    if exceed and not exceeding.get(vid, False):
                        episodes.append(dict(label))
                    exceeding[vid] = exceed
                    if leader and old_leader != leader:
                        switches.append(dict(label))
                previous[vid] = (t,state)
    if abs(abs(peak['jerk_m_s3'])-metrics['max_controlled_abs_jerk_m_s3']) > 1e-8:
        raise ValueError('Jerk labels disagree with immutable source metrics')
    # Labels are scored only after the current-state journal is closed.
    for label in [*switches, *episodes, peak]:
        label['prior'] = score(label, histories, plan)
    exposure = {}
    for key, _, _ in variants(plan):
        exposure[key] = {}
        for phase in ('primary', 'drain'):
            v = {name:counts[(key,phase,name)] for name in ('samples','candidate_samples',
                 'forward_candidate_samples','deficit_samples','unknown_samples')}
            v['deficit_vehicle_seconds'] = round(v['deficit_samples']*plan['step_s'],9)
            v['deficit_fraction'] = v['deficit_samples']/v['samples'] if v['samples'] else None
            exposure[key][phase] = v
    return dict(run=run.name, seed=config['seed'], peak=peak, leader_switches=switches,
        jerk_exceedance_episodes=episodes, exposure=exposure,
        unchanged_baseline=dict(primary=metrics['primary'],drain=metrics['drain'],
            network_gate_passed=metrics['network_gate_passed'],comfort_gate_passed=metrics['comfort_gate_passed'],
            collisions=metrics['collisions'],min_gap_m=metrics['min_controlled_bumper_gap_m'],
            min_ttc_s=metrics['min_controlled_ttc_s'],interaction=read(run/'interaction-summary.json'),
            event=read(run/'event-summary.json') if (run/'event-summary.json').exists() else None))


def report(output):
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    plan_path = ROOT/'experiments/configs/i110_convergence_v1.json'; plan = read(plan_path)
    sources = ['traffic/i110_convergence.py','traffic/i110_policy_interface.py',
        'scripts/report_i110_convergence.py','scripts/report_i110_anticipation.py','traffic/i110_anticipation.py']
    for name in sources:
        dest = output/'source'/name; dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/name,dest)
    shutil.copyfile(plan_path,output/'plan.json')
    git = lambda *args: subprocess.check_output(['git','-c',f'safe.directory={ROOT.as_posix()}',*args],cwd=ROOT,text=True)
    (output/'source.diff').write_text(git('diff','HEAD'),encoding='utf-8')
    manifest = dict(status='started',started_utc=datetime.now(timezone.utc).isoformat(),
        command=[sys.executable,*sys.argv], python=sys.version,platform=platform.platform(),
        git_commit=git('rev-parse','HEAD').strip(),git_status=git('status','--short'),
        source_sha256={p:sha(ROOT/p) for p in sources},
        inputs={str(plan_path):sha(plan_path),plan['network']:sha(ROOT/plan['network'])},
        simulator_launched=False,actor_loaded=False,training=False,no_checkpoint_reason=plan['no_checkpoint_reason'])
    write(output/'manifest.json',manifest)
    try:
        graph = ConvergenceGraph(ROOT/plan['network'],plan['path_limit_m'])
        results = []
        for name in plan['runs']:
            run = ROOT/plan['source_suite']/name
            manifest['inputs'][str(run/'manifest.json')] = sha(run/'manifest.json')
            results.append(derive(run,plan,graph,output/(name+'-signals.jsonl')))
            print('Derived '+name,flush=True)
        summary = dict(study_id=plan['study_id'],phase=plan['phase'],runs=results,limits=plan['limits'])
        write(output/'summary.json',summary)
        table = ['# Conditional convergence and stopping envelopes','',
            'Offline development diagnostics. No motion changed; projected gaps across lanes are conditional.', '',
            '| Run | Range m | Peak jerk | First supported lead to peak s | First constant-speed h0 margin m | Constant-speed h0 deficit seconds | Hard-brake h0 deficit seconds |',
            '|---|---:|---:|---:|---:|---:|---:|']
        for r in results:
            for radius in plan['ranges_m']:
                key = f'r{radius:g}-constant_speed-h0'
                first = r['peak']['prior'][key]['first_supported']
                table.append(f"| {r['run']} | {radius:g} | {abs(r['peak']['jerk_m_s3']):.4f} | "
                    f"{first['lead_s'] if first else None} | {round(first['envelope']['margin_m'],4) if first else None} | "
                    f"{r['exposure'][key]['primary']['deficit_vehicle_seconds']} | "
                    f"{r['exposure'][f'r{radius:g}-brake_9-h0']['primary']['deficit_vehicle_seconds']} |")
        table += ['', 'First support is within the frozen 10 s scoring window and requires current forward projection and a constructible ego stop. Positive margin means this specific hypothetical stopping profile fits under its stated lead model, not a guaranteed safe merge. h0 preserves a 2.5 m residual; h2 additionally preserves a 2 s dynamic gap. Full h2, all switches/episodes, unknowns and untouched burden ledgers are in summary.json.']
        (output/'results.md').write_text('\n'.join(table)+'\n',encoding='utf-8')
        manifest.update(status='completed',finished_utc=datetime.now(timezone.utc).isoformat(),
            outputs={p.relative_to(output).as_posix():sha(p) for p in output.rglob('*')
                     if p.is_file() and p.name!='manifest.json'})
        write(output/'manifest.json',manifest)
        print('\n'.join(table)); return summary
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));write(output/'manifest.json',manifest);raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    report(**vars(parser.parse_args()))
