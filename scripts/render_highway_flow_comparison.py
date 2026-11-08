"""Serial, budgeted SimReady replays of explicit frozen comparison inputs."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / 'outputs/highway-flow/v01'
RUNTIME = Path('C:/isaacsim/python.bat')
FFMPEG = ROOT / 'outputs/i110-merge-visual/v01/encoder/imageio_ffmpeg/binaries/ffmpeg-win-x86_64-v7.1.exe'


def write(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-seconds-per-render', type=float, default=240.)
    args = parser.parse_args()
    source, output = args.comparison.resolve(), args.output.resolve()
    if Path.cwd().resolve() != ROOT or not source.is_relative_to(STUDY) or not output.is_relative_to(STUDY) or output.exists():
        raise ValueError('Use canonical root, existing study inputs and fresh output')
    if not 0 < args.max_seconds_per_render <= 300:
        raise ValueError('Render bound must be in (0,300] seconds')
    output.mkdir(parents=True)
    ledger_path = STUDY / 'preparation-ledger.json'
    jobs = []
    for agent in ('agent_0', 'agent_4'):
        for side in ('left', 'right'):
            ledger = json.loads(ledger_path.read_text())
            if any(a['status'] not in ('complete', 'completed', 'failed') for a in ledger['attempts']):
                raise ValueError('Native tests and renders must be serial')
            spent = sum(a['charged_wall_seconds'] for a in ledger['attempts'])
            if spent + args.max_seconds_per_render > 1800:
                raise ValueError('Insufficient preparation allowance; do not silently expand it')
            target = output / (agent + '-' + side)
            command = [str(RUNTIME), 'scripts/render_i110_merge_replay.py', '--recording', str(source / (side + '-5fps')),
                '--output', str(target), '--focal-agent', agent, '--simready-main',
                'vehicles/sim_ready/americano-i7-ev/v14/assets/vehicle.usda', '--highlight-agents', 'agent_0', 'agent_4',
                '--camera-mode', 'follow', '--fps', '5', '--ffmpeg', str(FFMPEG)]
            entry = dict(output=target.relative_to(ROOT).as_posix(), status='running', kind='simready_replay',
                reserved_wall_seconds=args.max_seconds_per_render, charged_wall_seconds=args.max_seconds_per_render,
                command=command, training=False, script_sha256=sha(Path(__file__)),
                renderer_sha256=sha(ROOT/'scripts/render_i110_merge_replay.py'))
            ledger['attempts'].append(entry)
            write(ledger_path, ledger)
            started = time.monotonic()
            error = None
            with (output / (agent + '-' + side + '.log')).open('w', encoding='utf-8') as log:
                process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                    creationflags=subprocess.CREATE_NO_WINDOW)
                try:
                    code = process.wait(timeout=args.max_seconds_per_render)
                    report = json.loads((target/'render-report.json').read_text()) if (target/'render-report.json').is_file() else {}
                    if code != 0 or report.get('passed') is not True or report.get('video_decode_verified') is not True:
                        error = 'Renderer failed or evidence incomplete; exit=' + str(code)
                except subprocess.TimeoutExpired:
                    # Only terminate the child tree launched by this job.
                    subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], capture_output=True)
                    process.wait(timeout=15)
                    error = 'Render exceeded reserved time; partial evidence retained'
                except Exception as exc:
                    error = repr(exc)
                finally:
                    elapsed = time.monotonic()-started
                    entry.update(status='failed' if error else 'complete', charged_wall_seconds=elapsed, error=error)
                    write(ledger_path, ledger)
            jobs.append(dict(entry))
            write(output/'summary.json', dict(training=False, optimizer_steps=0, jobs=jobs, passed=error is None))
            print(json.dumps(dict(agent=agent, side=side, wall_seconds=elapsed, error=error)), flush=True)
            if error:
                return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
