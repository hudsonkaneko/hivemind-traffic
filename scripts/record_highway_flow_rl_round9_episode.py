"""Record one deterministic episode of the v06 round-9 final policy for a video (no training).

User request (2026-11-17): run the current best policy on the I-110 merge and show a
video. The action journals of earlier validations hold only the controlled cars, so
the full traffic scene needs one new recorded episode. It reproduces the validation
episode: the same validation row (case, seed 6900019 for I-110), the same v06
environment and the deterministic policy mean, so its measured result must equal
round 9's final validation for that case.

The episode is charged honestly to the shared study ledger (Git common directory) as
one preparation episode under its own command name. The study's diagnostic-episode cap
(127, all used) is raised by exactly this one recorded episode for this command only;
no training decisions are used. Output stays in this worktree's outputs/.
"""
import argparse
import hashlib
import json
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from experiments import run_highway_flow_rl_v06 as v06_runner  # noqa: E402
from experiments import run_highway_flow_rl_v09 as v09_runner  # noqa: E402

CONFIG = ROOT / 'configs/training/highway-flow-rl-v06-round9.json'
CONFIG_SHA256 = '91947d58d48feabe1efa97bb0a678909a27ba419ffab86b124e506cf2da99217'  # LF-normalized
POLICY = 'outputs/highway-flow-rl/v06/round-09/last.pt'
POLICY_SHA256 = '8f056b401a37316f0daaa387dbe3b41116f17d631a501cb4cf723dd971d31ab0'
EXPECTED_EPISODES_BEFORE = 127


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-index', type=int, default=0, help='validation row (0 = I-110 merge)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    raw = CONFIG.read_bytes().replace(b'\r\n', b'\n')
    if hashlib.sha256(raw).hexdigest() != CONFIG_SHA256:
        raise ValueError('Round-9 configuration changed')
    cfg = json.loads(raw.decode('utf-8'))
    v09_runner.install()                      # shared ledger through Git's common directory
    base = v09_runner.base
    base.env_for = v06_runner.env_for         # round 9's own environment (v06 rewards; same dynamics)
    output = base.project_path(args.output)
    if output.exists():
        raise ValueError('Use a fresh output directory')
    policy = base.project_path(POLICY)
    if hashlib.sha256(policy.read_bytes()).hexdigest() != POLICY_SHA256:
        raise ValueError('Round-9 final checkpoint differs from the pinned policy')
    row = cfg['validation_rows'][args.case_index]
    ledger = json.loads((base.STUDY / 'cost-ledger.json').read_text())
    before = base.ledger_totals(ledger, cfg['round_index'])['diagnostic_episodes']
    if before != EXPECTED_EPISODES_BEFORE:
        raise ValueError(f'Expected {EXPECTED_EPISODES_BEFORE} charged diagnostic episodes, found {before}')
    session_cfg = dict(cfg, budget=dict(cfg['budget'], max_diagnostic_episodes=before + 1))
    with base.LedgerSession(session_cfg, 'visual_recording', output, CONFIG_SHA256, started=started, episodes=1) as session:
        output.mkdir(parents=True)
        torch.set_num_threads(1)
        torch.use_deterministic_algorithms(True)
        torch.manual_seed(cfg['seed'])
        np.random.seed(cfg['seed'])
        random.seed(cfg['seed'])
        saved = torch.load(policy, map_location='cpu', weights_only=False)
        model = base.FlowModel()
        model.load_state_dict(saved['model'], strict=True)
        model.eval()
        report, _ = base.episode(cfg, row, output / 'episode', model, deadline=session.deadline, record=True)
        quality = (report.get('quality_merges') or {})
        stable = (report.get('stable_merge') or {}).get('per_agent') or {}
        summary = dict(schema='highway_flow_rl_round9_visual_recording_v01', case_id=row['case']['id'], seed=row['seed'],
            policy=POLICY, policy_sha256=POLICY_SHA256, config=CONFIG.relative_to(ROOT).as_posix(),
            config_sha256=CONFIG_SHA256, deterministic=True, training=False,
            merge_speed_match={a: (q or {}).get('speed_match_fraction') for a, q in quality.items()},
            stable_merge={a: dict(succeeded=s.get('stable_merge_succeeded'), entry_time_s=s.get('entry_time_s'),
                                  lateral_merge_complete_time_s=s.get('lateral_merge_complete_time_s'))
                          for a, s in stable.items() if s.get('entry_event')},
            collisions=report.get('collisions'), teleports=report.get('teleports'),
            intervention_fraction=report.get('intervention_fraction'), peak_abs_jerk=report.get('peak_abs_jerk'),
            recording=(output / 'episode/recording.json').relative_to(ROOT).as_posix(),
            ledger=dict(command='visual_recording', category='preparation', episodes=1,
                        diagnostic_episodes_after=before + 1, training_decisions=0),
            elapsed_seconds=session.elapsed())
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
