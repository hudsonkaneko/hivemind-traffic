"""Portable launcher for the bounded two-car lidar experiment."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from traffic.live_runtime import ROOT,find_isaac,find_sumo


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true');p.add_argument('--check',action='store_true')
    args,extra=p.parse_known_args()
    command=[str(find_isaac()),str(ROOT/'scripts/cooperative_lidar_drive.py'),*extra]
    if not args.headless:command.append('--gui')
    print('Launch:',subprocess.list2cmdline(command),flush=True)
    if args.check:return 0
    child=subprocess.Popen(command,cwd=ROOT,env={**os.environ,'SUMO_BINARY':str(find_sumo())})
    try:return child.wait(timeout=240)
    except subprocess.TimeoutExpired:
        if os.name=='nt':subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],capture_output=True)
        else:child.kill()
        child.wait();print('Fleet run timed out; retained partial evidence.',flush=True);return 1


if __name__=='__main__':raise SystemExit(main())
