"""Run the frozen I-110 stress tranche, retaining acceptance failures."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from traffic.i110_events import load_event
from traffic.smooth_acc import ComfortConfig


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(output, config, phase='verification'):
    config = Path(config).resolve()
    plan = json.loads(config.read_text())
    if (plan['schema_version'] != 1 or plan['step_s'] != .1 or plan['horizon_s'] != 60
            or plan['drain_budget_s'] != 90 or plan['passing'] != 'parallel'
            or plan['comfort'] != asdict(ComfortConfig()) or plan['training'] is not False
            or plan['controllers'] != ['native_acc', 'smooth_acc']
            or plan['verification_seeds'] != [731101, 731102] or plan['development_seed'] != 731001
            or plan['wall_limit_s'] != 120
            or plan['repeat'] != dict(event_id='emergency-brake-v1',seed=731101,controller='smooth_acc')):
        raise ValueError('Changed plan requires a new explicit protocol implementation')
    events = [(ROOT/p, load_event(ROOT/p)) for p in plan['events']]
    if [e['event_id'] for _, e in events] != ['cutin-v1', 'hard-brake-v1', 'emergency-brake-v1']:
        raise ValueError('Unexpected event set')
    network = ROOT/plan['network']
    if not network.is_file():
        raise FileNotFoundError(network)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(config, output/'plan.json')
    shutil.copyfile(__file__, output/'supervisor.py')
    write(output/'input-hashes.json', {str(p): digest(p) for p in [config, network, *[p for p, _ in events]]})
    for path, spec in events:
        shutil.copyfile(path, output/path.name)
    if phase == 'development':
        off = dict(events[0][1], enabled=False)
        write(output/'event-off.json', off)
        runs = [('ordinary', None, 731001, 'smooth_acc'),
                ('event-off', output/'event-off.json', 731001, 'smooth_acc'),
                ('cutin-development', events[0][0], 731001, 'smooth_acc'),
                ('emergency-development', events[2][0], 731001, 'smooth_acc')]
    else:
        runs = [(f"{event['event_id']}-{seed}-{controller}", path, seed, controller)
                for path, event in events for seed in plan['verification_seeds'] for controller in plan['controllers']]
        runs.append(('emergency-repeat', events[2][0], 731101, 'smooth_acc'))
    records = []
    for name, event_path, seed, controller in runs:
        command = [sys.executable, '-m', 'traffic.i110_baseline', '--network', str(network),
            '--output', str(output/name), '--seed', str(seed), '--controller', controller,
            '--passing', plan['passing'], '--horizon', str(plan['horizon_s']),
            '--drain', str(plan['drain_budget_s']), '--observe-policy-interface',
            '--study-id', plan['study_id'], '--reason', plan['trigger']]
        if event_path:
            command.extend(['--event-spec', str(event_path)])
        start = time.monotonic()
        with (output/(name+'-process.log')).open('x') as log:
            proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            try:
                code = proc.wait(timeout=plan['wall_limit_s'])
            except subprocess.TimeoutExpired:
                if os.name == 'nt':
                    subprocess.run(['taskkill','/PID',str(proc.pid),'/T','/F'],capture_output=True,timeout=15)
                else:
                    proc.kill()
                proc.wait(timeout=15)
                code = 124
        row = dict(name=name, seed=seed, controller=controller, command=command,
                   exit_code=code, wall_s=time.monotonic()-start)
        records.append(row)
        write(output/'processes.json', records)
        print(json.dumps(row), flush=True)
        metrics_path = output/name/'metrics.json'
        if code not in (0, 1) or not metrics_path.exists():
            write(output/'verification.json', dict(status='infrastructure_failed', runs=records))
            raise RuntimeError('Retained infrastructure failure: '+name)
    metrics = {name: json.loads((output/name/'metrics.json').read_text()) for name, *_ in runs}
    if phase == 'development':
        repeat_exact = metrics['ordinary']['trace_sha256'] == metrics['event-off']['trace_sha256']
        matched = True
        event_repeat = None
    else:
        name = 'emergency-brake-v1-731101-smooth_acc'
        repeat_exact = metrics[name]['trace_sha256'] == metrics['emergency-repeat']['trace_sha256']
        event_repeat = digest(output/name/'event-journal.json') == digest(output/'emergency-repeat/event-journal.json')
        matched = all(digest(output/f"{e['event_id']}-{seed}-native_acc"/file) ==
                      digest(output/f"{e['event_id']}-{seed}-smooth_acc"/file)
                      for _, e in events for seed in plan['verification_seeds']
                      for file in ('network.net.xml','routes.rou.xml','event-spec.json'))
    unchanged = all(digest(Path(p)) == h for p,h in json.loads((output/'input-hashes.json').read_text()).items())
    result = dict(status='completed', phase=phase, runs=records, repeat_exact=repeat_exact,
        event_repeat_exact=event_repeat, matched_inputs=matched, inputs_unchanged=unchanged,
        network_all_passed=all(m['network_gate_passed'] for m in metrics.values()),
        event_all_passed=all(m['event_gate_passed'] is not False for m in metrics.values()),
        comfort_failures=[name for name,m in metrics.items() if m['comfort_gate_passed'] is False],
        fixture_failures=[name for name,m in metrics.items() if not m['passed']],
        plan_sha256=digest(output/'plan.json'))
    result['passed'] = unchanged and matched and repeat_exact and event_repeat is not False and not result['fixture_failures']
    write(output/'verification.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT/'experiments/configs/i110_acc_stress_v1.json')
    parser.add_argument('--phase', choices=['development','verification'], default='verification')
    result = verify(**vars(parser.parse_args()))
    print(json.dumps({k:v for k,v in result.items() if k != 'runs'}, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
