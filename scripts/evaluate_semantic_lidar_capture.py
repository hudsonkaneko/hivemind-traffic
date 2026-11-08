"""Offline fusion analysis in the existing Isaac Python; no SimulationApp."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    args=parser.parse_args()
    directory=args.capture.resolve()
    if Path.cwd().resolve()!=ROOT or not directory.is_relative_to(ROOT/'outputs'):
        parser.error('Use a project capture directory from the project root')
    config=json.loads((directory/'evaluation-config.json').read_text())
    from traffic.semantic_lidar_evaluation import evaluate_capture
    result=evaluate_capture(directory,config['fusion'],config['gates'])
    print(json.dumps(dict(passed=result['passed'],training=False)))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
