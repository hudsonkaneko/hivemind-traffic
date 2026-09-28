"""Launch the live lidar demo using the external Isaac runtime."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import ROOT, find_isaac, find_sumo

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--check', action='store_true')
    args, extra = parser.parse_known_args()
    runtime, sumo = find_isaac(), find_sumo()
    command = [str(runtime), str(ROOT/'scripts/live_lidar_drive.py'), *extra]
    if not args.headless:
        command.append('--gui')
    print('SUMO:', sumo, '\nLaunch:', subprocess.list2cmdline(command), flush=True)
    if not args.check:
        raise SystemExit(subprocess.call(command, cwd=ROOT, env={**os.environ, 'SUMO_BINARY': str(sumo)}))
