"""Write the declared v14 round configuration and verify its inputs (no traffic, no training).

The configuration is traffic/highway_flow_rl_v14_contract.build_config(round), written as-is.
Before writing, the inputs are checked as for v12 (every network and the warm start against
their pinned checksums; the LA corridor manifests name the same networks), and the drive
limit's road grade is built once for every network of a case with a road model, so a launch
cannot start from a corridor whose grade does not load.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import prepare_highway_flow_rl_v12 as v12prepare  # noqa: E402
from traffic import highway_flow_rl_v12_contract as v12c  # noqa: E402
from traffic import highway_flow_rl_v14_contract as contract  # noqa: E402
from traffic.road_grade import RoadGrade, validate_drive_limit  # noqa: E402


def verify_inputs(cfg):
    """v12's pinned-input checks for the base round, then the road grade of every road-model network."""
    base_round = contract.ROUNDS[cfg['round_index']]['base_round']
    verified = v12prepare.verify_inputs(dict(cfg, round_index=base_round)) if base_round in v12c.ROUNDS else None
    if verified is None:
        raise ValueError('The v14 round must name a declared v12 base round')
    limit = validate_drive_limit(cfg['v14_drive_limit'])
    grades = {}
    for row in cfg['training_rows'] + cfg['validation_rows']:
        case = row['case']
        if not case.get('road_model') or case['network'] in grades:
            continue
        network = ROOT / case['network']
        grade = RoadGrade(network, network.parent / 'corridor-manifest.json', spec=limit['vehicle'])
        grades[case['network']] = grade.info
    return verified, grades


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round', type=int, default=20)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    cfg = contract.build_config(args.round)
    contract.validate_config(cfg)
    verified, grades = verify_inputs(cfg)
    output = args.output or ROOT / f'configs/training/highway-flow-rl-v14-round{args.round}.json'
    output.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
    contract.validate_config(json.loads(output.read_text(encoding='utf-8')))
    print(json.dumps(dict(config=output.relative_to(ROOT).as_posix(), verified_inputs=len(verified),
                          road_grades=grades, training_rows=len(cfg['training_rows']),
                          validation_rows=len(cfg['validation_rows']))))


if __name__ == '__main__':
    main()
