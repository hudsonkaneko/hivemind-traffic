"""Write the declared v12 round configuration and verify its inputs (no traffic, no training).

The configuration is traffic/highway_flow_rl_v12_contract.build_config(round), written
as-is. Before writing, every network and the warm start are checked against their
pinned checksums, and the LA corridor manifests must name the same networks, so a
launch can only start from the corridors the contract declares.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic import highway_flow_rl_v12_contract as contract  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402


def verify_inputs(cfg):
    pins = {cfg['warm_start']['path']: cfg['warm_start']['sha256']}
    for row in cfg['training_rows'] + cfg['validation_rows']:
        pins[row['case']['network']] = row['case']['network_sha256']
    for path, checksum in sorted(pins.items()):
        if file_digest(ROOT / path) != checksum:
            raise ValueError(f'Input differs from its pinned checksum: {path}')
    for ident, site in contract.LA_CORRIDORS.items():
        manifest = json.loads((ROOT / site['network']).with_name('corridor-manifest.json').read_text())
        if (manifest['id'] != ident or manifest['network_sha256'] != site['network_sha256']
                or manifest['compiled_checks']['passed'] is not True or manifest['model'] != contract.MODEL):
            raise ValueError(f'Corridor manifest does not match the contract: {ident}')
    return sorted(pins)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round', type=int, default=18)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    cfg = contract.build_config(args.round)
    contract.validate_config(cfg)
    verified = verify_inputs(cfg)
    output = args.output or ROOT / f'configs/training/highway-flow-rl-v12-round{args.round}.json'
    output.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(config=output.relative_to(ROOT).as_posix(), verified_inputs=len(verified),
                          training_rows=len(cfg['training_rows']), validation_rows=len(cfg['validation_rows']))))


if __name__ == '__main__':
    main()
