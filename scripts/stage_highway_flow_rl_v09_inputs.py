"""Stage the v09 run's frozen inputs into this worktree (copies only; no simulation, no ledger charge).

The v05 loop only reads artifacts inside the checkout it runs from. V09 runs from
Claude's worktree, so the declared training/validation networks and the pinned
round-9 warm start are copied from the canonical checkout, each verified by its
declared SHA-256 before and after copying. Held-out networks are not staged
(held-out evaluation is not authorized). An existing file with a different hash
is never overwritten.
"""
import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from traffic import highway_flow_rl_v09_contract as contract  # noqa: E402


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def declared_inputs(cfg):
    pins = {cfg['warm_start']['path']: cfg['warm_start']['sha256']}
    for split in ('training_rows', 'validation_rows'):
        for row in cfg[split]:
            case = row['case']
            if pins.setdefault(case['network'], case['network_sha256']) != case['network_sha256']:
                raise ValueError('One network path declared with two hashes')
    return pins


def stage(cfg, source_root, target_root=HERE):
    staged = []
    for relative, sha in sorted(declared_inputs(cfg).items()):
        source, target = Path(source_root) / relative, Path(target_root) / relative
        if digest(source) != sha:
            raise ValueError(f'Source does not match its declared hash: {relative}')
        if target.exists():
            if digest(target) != sha:
                raise ValueError(f'Refusing to overwrite a different file: {relative}')
            staged.append(dict(path=relative, sha256=sha, action='already_present'))
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        if digest(target) != sha:
            raise ValueError(f'Copy does not match its declared hash: {relative}')
        staged.append(dict(path=relative, sha256=sha, action='copied'))
    return staged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--source-root', type=Path, required=True, help='canonical checkout holding outputs/')
    args = parser.parse_args()
    cfg = contract.validate_config(json.loads(args.config.read_text(encoding='utf-8')))
    print(json.dumps(stage(cfg, args.source_root), indent=1))


if __name__ == '__main__':
    main()
