"""Record one v14 validation episode with the policy driving a PhysX car on the LA model (no training).

User decision (November 19, 2026): "Yes, policy drives it". Training stays in SUMO; this runs a
saved policy with the controlled car driven by rigid-body physics instead of SUMO's kinematics:

- the venv process runs SUMO (background traffic), the v14 environment and the policy, as in
  scripts/record_highway_flow_rl_v14_episode.py: the same row, seed and deterministic policy mean;
- each 0.1 s decision, the acceleration and lane change the policy and v14's limits produce go
  to environments/highway_flow_rl_v14_physical_env.py, which hands them with a reference path to
  scripts/physics_drive_server.py (Isaac Sim, PhysX 120 Hz): a low-level controller turns them
  into steering, drive torque and brake torque on the provisional Americano v14 vehicle package,
  which drives on the LA I-10/I-110 model's own deck colliders (the model file is not modified);
- the car's physics pose goes back into SUMO as its shadow, so the background traffic, the
  policy's observations and the v14 checks all see where the physics car actually is.

The other cars are SUMO vehicles drawn at their SUMO poses on the model's surface (kinematic).
Only the policy's car is physics-driven.

Without --development the episode is charged to the shared study ledger as one preparation
episode, like the v12 and v14 recordings, and the output must be a fresh directory under
outputs/highway-flow-rl/v14. --development runs are uncharged checks of the physics integration
and go under outputs/test-runs. Add the graph with scripts/compose_highway_flow_3d_graph.py
(--physics-log adds commanded vs measured acceleration).
"""
from __future__ import annotations

import argparse
import contextlib
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

from environments.highway_flow_rl_v14_physical_env import HighwayFlowRLV14PhysicalEnv  # noqa: E402
from experiments import run_highway_flow_rl_v12 as runner  # noqa: E402
from experiments import run_highway_flow_rl_v14 as runner14  # noqa: E402
from policies.highway_flow_v12 import FlowModelV12  # noqa: E402
from traffic import highway_flow_rl_v14_rewards as v14  # noqa: E402
from traffic.highway_flow_contract import file_digest  # noqa: E402
from traffic.physics_link import RemoteDriver, server_command  # noqa: E402

CONFIG = 'configs/training/highway-flow-rl-v12-round19.json'
CASE_INDEX = 11              # v12-val-la-sb-adams-model-merge-focused-1of16-medium, seed 6950004


def physics_summary(server_report, log):
    """Commanded vs measured acceleration and the car's contact with the deck, from the server's log."""
    rows = log.get('rows', [])
    errors = [abs(r['commanded_acceleration'] - r['measured_acceleration']) for r in rows]
    unsaturated = [e for e, r in zip(errors, rows) if not r['saturated_ticks']]
    off = [r['time'] for r in rows if not all(r['on_ground'])]
    return dict(decisions=len(rows), frames=len(log.get('frames', [])),
                acceleration_error=dict(mean_abs=sum(errors) / max(1, len(errors)), max_abs=max(errors, default=None),
                                        unsaturated_mean_abs=sum(unsaturated) / max(1, len(unsaturated)),
                                        saturated_decisions=sum(bool(r['saturated_ticks']) for r in rows)),
                max_abs_lateral_error_m=max((abs(r['lateral_error_m']) for r in rows), default=None),
                wheels_off_ground_decisions=len(off), wheels_off_ground_times=off[:50],
                max_abs_pitch_rad=max((abs(r['pitch_rad']) for r in rows), default=None),
                max_abs_roll_rad=max((abs(r['roll_rad']) for r in rows), default=None),
                server=server_report)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True, help='a v12 checkpoint (for example round-19/best.pt)')
    parser.add_argument('--case-index', type=int, default=CASE_INDEX)
    parser.add_argument('--config', default=CONFIG)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render-hz', type=int, default=10, choices=[0, 10])
    parser.add_argument('--development', action='store_true',
                        help='uncharged physics-integration check; output under outputs/test-runs')
    args = parser.parse_args(argv)
    started = time.monotonic()
    runner.install = runner14.install
    runner.install()
    base = runner.base
    config = base.project_path(args.config)
    cfg = json.loads(config.read_text())
    base.validate_config(cfg)
    fingerprint = file_digest(config)
    output = args.output.resolve()
    allowed_root = (ROOT / 'outputs/test-runs') if args.development else base.OUTPUT_ROOT.resolve()
    if not output.is_relative_to(allowed_root) or output.exists():
        raise ValueError(f'Use a fresh output directory under {allowed_root.relative_to(ROOT).as_posix()}')
    checkpoint = base.project_path(args.checkpoint)
    saved = torch.load(checkpoint, map_location='cpu', weights_only=False)
    if saved.get('schema') != runner.CHECKPOINT_SCHEMA or saved.get('config_sha256') != fingerprint:
        raise ValueError('The checkpoint is not a v12 checkpoint of this configuration')
    if not 0 <= args.case_index < len(cfg['validation_rows']):
        raise ValueError('Case index is outside the declared validation rows')
    row = cfg['validation_rows'][args.case_index]
    network = ROOT / row['case']['network']
    manifest = network.parent / 'corridor-manifest.json'
    driver = RemoteDriver(server_command(output / 'isaac', network=network, manifest=manifest, render_hz=args.render_hz),
                          log_path=output / 'isaac-server.log', cwd=ROOT)
    base.env_for = lambda cfg_, row_, directory: runner14.env_for(
        cfg_, row_, directory, env_class=HighwayFlowRLV14PhysicalEnv, driver=driver)
    ledger = (contextlib.nullcontext(None) if args.development else
              base.LedgerSession(cfg, 'visual_recording', output, fingerprint, started=started, episodes=1))
    with ledger as session:
        output.mkdir(parents=True)
        torch.set_num_threads(1)
        torch.use_deterministic_algorithms(True)
        torch.manual_seed(cfg['seed'])
        np.random.seed(cfg['seed'])
        random.seed(cfg['seed'])
        model = FlowModelV12()
        model.load_state_dict(saved['model'], strict=True)
        model.eval()
        server_report = None
        try:
            report, _ = base.episode(cfg, row, output / 'episode', model,
                                     deadline=None if session is None else session.deadline, record=True)
        finally:
            server_report = driver.close()
        log_path = output / 'isaac/physics-log.json'
        log = json.loads(log_path.read_text(encoding='utf-8')) if log_path.exists() else {}
        poses = [dict(time=f['time'], frame=f['file'], camera=f.get('camera')) for f in log.get('frames', [])]
        (output / 'isaac/camera-poses.json').write_text(json.dumps(poses, indent=1), encoding='utf-8')
        quality = report.get('quality_merges') or {}
        stable = (report.get('stable_merge') or {}).get('per_agent') or {}
        summary = dict(schema='highway_flow_rl_v14_physics_recording_v01', case_id=row['case']['id'], seed=row['seed'],
            road_model=row['case'].get('road_model'), corridor=row['case'].get('corridor'),
            environment='v14 physical', reward_version=v14.REWARD_VERSION, control=dict(v14.CONTROL),
            motion=('agent cars: Isaac Sim PhysX rigid-body vehicle (Americano v14, provisional physics) on the '
                    "LA model's deck colliders, driven by the low-level controller; other cars: SUMO poses (kinematic)"),
            checkpoint=checkpoint.relative_to(ROOT).as_posix(), checkpoint_sha256=file_digest(checkpoint),
            checkpoint_state=dict(updates=saved['state'].get('updates'), decisions=saved['state'].get('decisions'),
                                  best_validation_label=saved['state'].get('best_validation_label')),
            config=config.relative_to(ROOT).as_posix(), config_sha256=fingerprint, deterministic=True, training=False,
            merge_target_ids=report.get('merge_target_ids'),
            merge_speed_match={a: (q or {}).get('speed_match_fraction') for a, q in quality.items()},
            stable_merge={a: dict(succeeded=s.get('stable_merge_succeeded'), entry_time_s=s.get('entry_time_s'),
                                  lateral_merge_complete_time_s=s.get('lateral_merge_complete_time_s'))
                          for a, s in stable.items() if s.get('entry_event')},
            target_band=report.get('v12_metrics'), collisions=report.get('collisions'),
            teleports=report.get('teleports'), intervention_fraction=report.get('intervention_fraction'),
            peak_abs_jerk=report.get('peak_abs_jerk'), physical_steps=report.get('physical_steps'),
            physics=physics_summary(server_report and server_report.get('report'), log),
            recording=(output / 'episode/recording.json').relative_to(ROOT).as_posix(),
            ledger=(dict(command='visual_recording', category='preparation', episodes=1, training_decisions=0)
                    if session is not None else dict(charged=False, note='development check of the physics integration')),
            elapsed_seconds=time.monotonic() - started)
        (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({k: summary[k] for k in ('case_id', 'stable_merge', 'target_band', 'collisions',
                                                   'intervention_fraction', 'peak_abs_jerk', 'physical_steps')},
                         indent=1, default=str))
        print(json.dumps({k: v for k, v in summary['physics'].items() if k != 'server'}, indent=1))


if __name__ == '__main__':
    main()
