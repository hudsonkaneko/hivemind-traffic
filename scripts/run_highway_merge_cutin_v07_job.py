"""Wait for the prior process, qualify sources, then train; no progress polling."""
import argparse
import ctypes
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
CONFIG = 'configs/training/highway-merge-cutin-2026-11-15.json'


def wait_pid(pid):
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.OpenProcess.argtypes = [ctypes.c_ulong, ctypes.c_bool, ctypes.c_ulong]
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handle = kernel.OpenProcess(0x100000, False, pid)
    if not handle:
        if ctypes.get_last_error() == 87:  # process already exited
            return
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        if kernel.WaitForSingleObject(handle, 0xFFFFFFFF) != 0:
            raise ctypes.WinError(ctypes.get_last_error())
    finally:
        kernel.CloseHandle(handle)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait-pid', type=int, action='append', default=[])
    parser.add_argument('--name', required=True)
    args = parser.parse_args()
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run in the prepared worktree')
    out = ROOT / 'outputs/highway-flow-rl/v07' / args.name
    out.mkdir(parents=True, exist_ok=False)
    log_dir = ROOT / 'logs'
    log_dir.mkdir(exist_ok=True)
    status = dict(status='waiting_for_previous_run', wait_pids=args.wait_pid,
                  started_unix=time.time(), workspace=str(ROOT), output=str(out))
    status_path = out / 'job-status.json'
    def save():
        tmp = status_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(status, indent=2) + '\n')
        tmp.replace(status_path)
    save()
    def execute(command, name):
        status.update(status=name, command=command)
        save()
        with (log_dir / f'{args.name}-{name}.log').open('x', encoding='utf-8') as stream:
            subprocess.run(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=True)
    runner = [sys.executable, '-u', 'experiments/run_highway_merge_cutin_v07.py']
    relative = out.relative_to(ROOT).as_posix()
    try:
        for pid in args.wait_pid:
            wait_pid(pid)  # OS wait, no metrics or log polling.
        # Source-qualified smoke cases before spending the learning budget.
        for name, index in [('nominal-smoke', 4), ('stress-smoke', 8)]:
            execute(runner + ['evaluate', '--config', CONFIG, '--case-index', str(index),
                '--output', f'{relative}/{name}', '--max-wall-seconds', '180'], name)
            report = json.loads((out / name / 'evaluation.json').read_text())['reports'][0]
            if report.get('collisions') or report.get('teleports'):
                raise RuntimeError(f'{name} observed collision/teleport; see retained report')
            if not report.get('v07_admission', {}).get('native_requests'):
                raise RuntimeError(f'{name} did not exercise native lane-change admission')
            if name == 'stress-smoke' and not report['v07_admission']['stress_started']:
                raise RuntimeError('Stress fixture did not produce a measured cut-in; do not claim coverage')
        execute(runner + ['preflight', '--config', CONFIG, '--output', f'{relative}/preflight'], 'preflight')
        execute(runner + ['train', '--config', CONFIG, '--output', f'{relative}/train',
            '--preflight', f'{relative}/preflight/preflight.json'], 'training')
        status.update(status='complete', finished_unix=time.time())
        save()
        print(json.dumps(status))
        return 0
    except BaseException as exc:
        status.update(status='failed', error=repr(exc), finished_unix=time.time())
        save()
        print(json.dumps(status))
        raise


if __name__ == '__main__':
    raise SystemExit(main())
