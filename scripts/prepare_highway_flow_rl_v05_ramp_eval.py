"""Write a frozen-actor evaluation config: historical vs ramp-started ramp networks.

Starts from the draft's verified legacy comparison config (which reproduced the
v04 final validation exactly) and replaces only ``validation_rows``. Each of the
six v05-bank ramp layouts appears twice with the same merge row settings and
seed: once on the historical network and once on the ramp-started network built
by ``scripts/build_highway_flow_rl_v05_ramp_networks.py``. Speed limits follow
each layout's posted limit. Training and held-out rows are untouched, and no
training or held-out evaluation is authorized by this config.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from scripts.build_highway_environment_v05_bank import specifications

BASE_CASE_ID = 'v02-development-train-2-lane-ramp120-merge'
HISTORICAL_BANK = 'outputs/highway-environment/v05/network-bank'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def layouts():
    for spec in specifications():
        ident = f'{spec["split"]}-{spec["lanes"]}-lane-ramp{spec["ramp_upstream_distance"]:g}'
        yield ident, spec


def build_config(base, root, corrected_bank):
    template = next(r for r in base['validation_rows'] if r['case']['id'] == BASE_CASE_ID)
    rows = []
    for ident, spec in layouts():
        for label, bank in (('historical', HISTORICAL_BANK), ('ramp-started', corrected_bank)):
            network = f'{bank}/{ident}/network.net.xml'
            row = deepcopy(template)
            case = row['case']
            case.update(id=f'ramp-eval-{ident}-{label}', network=network,
                        network_sha256=digest(root / network), through_lanes=spec['lanes'],
                        speed_limit_mph=spec['speed_mph'], network_layout=label)
            case.pop('geometry_manifest', None)
            case.pop('geometry_manifest_sha256', None)
            row['speed_limit_m_s'] = round(spec['speed_mph'] * .44704, 4)
            rows.append(row)
    cfg = deepcopy(base)
    cfg.update(validation_rows=rows, training_authorized=False, held_out_authorized=False,
               development_gates_passed=False,
               authorization='Frozen evaluation only (user request, Nov 11 2026); no training or held-out use',
               evaluation_note=('Same merge row and seed on historical and ramp-started networks; '
                                'compares SUMO interventions and jerk, not a held-out generalization test'))
    return cfg


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--root', type=Path, required=True, help='Checkout whose outputs/ holds the networks')
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--corrected-bank', default='outputs/highway-flow-rl/v05/ramp-networks-02')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    cfg = build_config(json.loads(args.base.read_text()), args.root, args.corrected_bank)
    if args.output.exists():
        raise ValueError('Use a fresh config path')
    args.output.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(config=str(args.output), rows=len(cfg['validation_rows']), sha256=digest(args.output))))


if __name__ == '__main__':
    main()
