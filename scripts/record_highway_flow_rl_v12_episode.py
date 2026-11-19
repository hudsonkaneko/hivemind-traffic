"""Record one deterministic v12 validation episode of a saved v12 policy for a video (no training).

User request (2026-11-18): "make a new video of the policy like similar to this" (the
round-9 I-110 top-down video), with the LA I-10/I-110 model as the primary road. The
validation journals hold only the controlled cars, so the full traffic scene needs one
recorded episode. It reproduces a declared validation episode: the same row (case and
seed), the v12 environment and the deterministic policy mean. Its result must therefore
equal that checkpoint's validation of the same row.

The episode is charged to the shared study ledger (Git common directory) as one
preparation episode, under the study's existing diagnostic-episode cap. No training
decisions are used, and the output stays in this worktree's outputs/.
Render it with scripts/render_highway_flow_topdown.py.
"""
import argparse
import json
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from experiments import run_highway_flow_rl_v12 as runner  # noqa: E402
from policies.highway_flow_v12 import FlowModelV12  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402

CONFIG = 'configs/training/highway-flow-rl-v12-round18.json'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True, help='a v12 checkpoint (for example round-18/best.pt)')
    parser.add_argument('--case-index', type=int, default=8, help='validation row (8 = LA northbound merge, 8 of 16)')
    parser.add_argument('--config', default=CONFIG)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    started = time.monotonic()
    runner.install()
    base = runner.base
    config = base.project_path(args.config)
    cfg = json.loads(config.read_text())
    base.validate_config(cfg)
    fingerprint = file_digest(config)
    output = base.project_path(args.output)
    if not output.is_relative_to(base.OUTPUT_ROOT.resolve()) or output.exists():
        raise ValueError('Use a fresh output directory under outputs/highway-flow-rl/v12')
    checkpoint = base.project_path(args.checkpoint)
    saved = torch.load(checkpoint, map_location='cpu', weights_only=False)
    if saved.get('schema') != runner.CHECKPOINT_SCHEMA or saved.get('config_sha256') != fingerprint:
        raise ValueError('The checkpoint is not a v12 checkpoint of this configuration')
    if not 0 <= args.case_index < len(cfg['validation_rows']):
        raise ValueError('Case index is outside the declared validation rows')
    row = cfg['validation_rows'][args.case_index]
    with base.LedgerSession(cfg, 'visual_recording', output, fingerprint, started=started, episodes=1) as session:
        output.mkdir(parents=True)
        torch.set_num_threads(1)
        torch.use_deterministic_algorithms(True)
        torch.manual_seed(cfg['seed'])
        np.random.seed(cfg['seed'])
        random.seed(cfg['seed'])
        model = FlowModelV12()
        model.load_state_dict(saved['model'], strict=True)
        model.eval()
        report, _ = base.episode(cfg, row, output / 'episode', model, deadline=session.deadline, record=True)
        quality = report.get('quality_merges') or {}
        stable = (report.get('stable_merge') or {}).get('per_agent') or {}
        summary = dict(schema='highway_flow_rl_v12_visual_recording_v01', case_id=row['case']['id'], seed=row['seed'],
            road_model=row['case'].get('road_model'), corridor=row['case'].get('corridor'),
            checkpoint=checkpoint.relative_to(ROOT).as_posix(), checkpoint_sha256=file_digest(checkpoint),
            policy=checkpoint.relative_to(ROOT).as_posix(), policy_sha256=file_digest(checkpoint),
            checkpoint_state=dict(updates=saved['state'].get('updates'), decisions=saved['state'].get('decisions'),
                                  best_validation_label=saved['state'].get('best_validation_label')),
            config=config.relative_to(ROOT).as_posix(), config_sha256=fingerprint, deterministic=True, training=False,
            merge_target_ids=report.get('merge_target_ids'),
            merge_speed_match={a: (q or {}).get('speed_match_fraction') for a, q in quality.items()},
            quality_qualified={a: (q or {}).get('qualified') for a, q in quality.items()},
            stable_merge={a: dict(succeeded=s.get('stable_merge_succeeded'), entry_time_s=s.get('entry_time_s'),
                                  lateral_merge_complete_time_s=s.get('lateral_merge_complete_time_s'))
                          for a, s in stable.items() if s.get('entry_event')},
            target_band=report.get('v12_metrics'), collisions=report.get('collisions'),
            teleports=report.get('teleports'), intervention_fraction=report.get('intervention_fraction'),
            peak_abs_jerk=report.get('peak_abs_jerk'),
            recording=(output / 'episode/recording.json').relative_to(ROOT).as_posix(),
            ledger=dict(command='visual_recording', category='preparation', episodes=1, training_decisions=0),
            elapsed_seconds=session.elapsed())
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({k: summary[k] for k in ('case_id', 'merge_target_ids', 'merge_speed_match',
                                                   'stable_merge', 'collisions', 'intervention_fraction')}, indent=1))


if __name__ == '__main__':
    main()
