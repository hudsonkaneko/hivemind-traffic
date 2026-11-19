"""Record one deterministic validation episode under v14 control for a video (no training).

User request (2026-11-19): a 3D version of the top-down policy video, with the sim car and
the speed/target graph. This is scripts/record_highway_flow_rl_v12_episode.py with the v14
environment installed (experiments/run_highway_flow_rl_v14.py): the same row, seed and
deterministic policy mean, but SUMO's own checks are off for the controlled cars (speed mode
96, lane change mode 0) and the merge target is v14's. The policy is a saved v12 checkpoint
with its own configuration; v14 changes no observation width, so it loads unchanged.

Charged to the shared study ledger as one preparation episode, like the v12 recording. The
output must be a fresh directory under outputs/highway-flow-rl/v14. Render it with
scripts/stage_highway_flow_isaac_replay.py and scripts/render_i110_merge_replay.py, and add
the graph with scripts/compose_highway_flow_3d_graph.py.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments import run_highway_flow_rl_v12 as runner  # noqa: E402
from experiments import run_highway_flow_rl_v14 as runner14  # noqa: E402
from scripts import record_highway_flow_rl_v12_episode as recorder12  # noqa: E402
from traffic import highway_flow_rl_v14_rewards as v14  # noqa: E402


def main(argv=None):
    runner.install = runner14.install          # the v12 recorder installs the v14 pieces
    recorder12.main(argv)
    args = recorder12_args(argv)
    path = runner.base.project_path(args.output) / 'summary.json'
    summary = json.loads(path.read_text(encoding='utf-8'))
    summary.update(schema='highway_flow_rl_v14_visual_recording_v01', environment='v14',
                   reward_version=v14.REWARD_VERSION, control=dict(v14.CONTROL),
                   control_note=('SUMO checks off for controlled cars; the policy and the '
                                 "environment's own limits drive them"))
    path.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')


def recorder12_args(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint')
    parser.add_argument('--case-index', type=int)
    parser.add_argument('--config')
    parser.add_argument('--output', type=Path, required=True)
    return parser.parse_args(argv)


if __name__ == '__main__':
    main()
