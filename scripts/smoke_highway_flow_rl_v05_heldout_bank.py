"""Check that the environment accepts every held-out layout, without the policy.

For each layout: construct the v04 flow-RL environment with a single-AV merge
row, reset (demand generation, lane identities, speed limits), then drive a
bounded number of steps with SUMO driving the controlled car natively (control
mode 'native', speed -1, lane-change mode 1621, as the project runners do for
native handoff). The trained actor is never loaded, so the held-out geometry
stays unused for policy evaluation. Reports errors and coarse native events.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time


def smoke(code_root, network, layout, row, cfg, steps):
    from environments.highway_flow_rl_v04_env import HighwayFlowRLV04Env
    with tempfile.TemporaryDirectory(dir=code_root / 'outputs') as tmp:
        env = HighwayFlowRLV04Env(network, Path(tmp), controlled_count=row['controlled_count'],
            total_vehicles=row['total_vehicles'], horizon=row['horizon_steps'], communication=row['communication'],
            packet_loss=row['packet_loss'], through_lanes=layout['options']['lanes'], task='merge', density='medium',
            control_target_version='v3', speed_limit_m_s=round(layout['options']['speed_mph'] * .44704, 4),
            difficulty=row['difficulty'], traffic_mode='initial', lane_change_duration=3., stabilization_seconds=8.,
            jerk_reward_weight=cfg['reward_weights']['jerk'], v04_reward_weights=cfg['v04_reward_weights'])
        started = time.monotonic()
        try:
            import numpy as np
            env.control_mode = 'native'  # commands are recorded, never applied
            env.reset(seed=row['seed'])
            for agent in env.agents:  # same native handoff the project runners use
                env.conn.vehicle.setSpeed(agent, -1)
                env.conn.vehicle.setLaneChangeMode(agent, 1621)
            done_steps, merged = 0, False
            for _ in range(steps):
                if not env.agents:
                    break
                env.step({a: np.array([0., 0.], np.float32) for a in env.agents})
                done_steps += 1
                merged = merged or bool(env.merged)
            return dict(id=layout['id'], ok=True, steps=done_steps, merged_native=merged,
                        collisions=env.stats['collisions'], teleports=env.stats['teleports'],
                        seconds=round(time.monotonic() - started, 1))
        except Exception as error:  # report, do not hide
            return dict(id=layout['id'], ok=False, error=f'{type(error).__name__}: {error}')
        finally:
            env.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--code-root', type=Path, required=True)
    parser.add_argument('--bank', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True, help='Config supplying the merge row and weights')
    parser.add_argument('--steps', type=int, default=400)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    sys.path.insert(0, str(args.code_root))
    cfg = json.loads(args.config.read_text())
    row = next(r for r in cfg['validation_rows']
               if r['case']['task'] == 'merge' and r['controlled_count'] == 1)
    bank = json.loads((args.bank / 'bank.json').read_text())
    results = [smoke(args.code_root, args.bank / layout['id'] / 'network.net.xml', layout, row, cfg, args.steps)
               for layout in bank['layouts']]
    report = dict(schema='highway_flow_rl_v05_heldout_smoke', policy_loaded=False, actions='native SUMO driving (no commands applied)',
                  bank_sha256=__import__('hashlib').sha256((args.bank / 'bank.json').read_bytes()).hexdigest(),
                  steps_limit=args.steps, results=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(results, indent=1))


if __name__ == '__main__':
    main()
