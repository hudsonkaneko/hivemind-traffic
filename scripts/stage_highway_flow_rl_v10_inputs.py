"""Stage the v10 run's frozen inputs into this worktree (copies only; no simulation, no ledger charge).

Same inputs and checks as scripts/stage_highway_flow_rl_v09_inputs.py (the pinned
round-9 warm start and the declared training/validation networks, each verified by
SHA-256 before and after copying, never overwriting a different file, held-out
networks never staged); only the configuration is validated by the v10 contract.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from scripts.stage_highway_flow_rl_v09_inputs import declared_inputs, stage  # noqa: E402,F401
from traffic import highway_flow_rl_v10_contract as contract  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--source-root', type=Path, required=True, help='canonical checkout holding outputs/')
    args = parser.parse_args()
    cfg = contract.validate_config(json.loads(args.config.read_text(encoding='utf-8')))
    print(json.dumps(stage(cfg, args.source_root), indent=1))


if __name__ == '__main__':
    main()
