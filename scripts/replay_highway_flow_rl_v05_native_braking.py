"""Bounded headless SUMO replay of one saved v04 case to explain native braking.

Re-runs the saved network/routes with the recorded SUMO options and seed, then
feeds the controlled car the *applied* accelerations from its saved action
journal (open loop). At each step it records SUMO's realized acceleration and
the car's junction state (next links, junction foes, leader). A second pass
repeats the replay with speed-mode bit 3 (right of way at intersections) turned
off, as a counterfactual check of the junction-yield explanation.

This is diagnostics only: no policy, training or reward is involved, and the
counterfactual speed mode is never used for control.
"""
import argparse
import gzip
import json
import os
from pathlib import Path

import traci

DT = 0.1


def sumo_binary():
    home = os.environ.get('SUMO_HOME')
    if home and (Path(home) / 'bin' / 'sumo.exe').exists():
        return str(Path(home) / 'bin' / 'sumo.exe')
    return 'sumo'


def load_journal(episode, agent):
    rows = [json.loads(line) for line in gzip.open(episode / 'actions.jsonl.gz', 'rt')]
    return [(round(r['time'], 1), r['agents'][agent]) for r in rows if agent in r['agents']]


def junction_state(conn, agent):
    links = []
    for link in conn.vehicle.getNextLinks(agent)[:2]:
        # TraCI order: lane, hasPrio, isOpen, hasFoe, viaLane, state, direction, length
        lane, has_prio, is_open, has_foe, via, state, direction, length = link[:8]
        links.append(dict(to_lane=lane, via=via, has_priority=bool(has_prio), is_open=bool(is_open),
                          has_approaching_foe=bool(has_foe), state=state, direction=direction,
                          link_length_m=round(length, 2)))
    try:
        foes = [dict(foe=f[0], ego_gap_to_conflict_m=round(f[1], 2), foe_gap_to_conflict_m=round(f[2], 2),
                     ego_eta_s=round(f[3], 2), foe_eta_s=round(f[4], 2), foe_lane=f[8])
                for f in conn.vehicle.getJunctionFoes(agent, 80.)]
    except traci.TraCIException:
        foes = None
    leader = conn.vehicle.getLeader(agent, 100.)
    return dict(next_links=links, junction_foes=foes,
                leader=dict(id=leader[0], gap_m=round(leader[1], 2)) if leader else None)


def replay(episode, journal, agent, seed, speed_mode, window):
    label = f'replay-{speed_mode}'
    traci.start([sumo_binary(), '-n', str(episode / 'network.net.xml'), '-r', str(episode / 'routes.rou.xml'),
                 '--seed', str(seed), '--step-length', str(DT), '--lanechange.duration', '3.0',
                 '--collision.action', 'warn', '--collision.check-junctions', 'true',
                 '--time-to-teleport', '-1', '--no-step-log', 'true', '--duration-log.disable', 'true',
                 '--xml-validation', 'never'], label=label)
    conn = traci.getConnection(label)
    samples = []
    try:
        conn.simulationStep()  # 0.1 s: initial cohort inserted, matching the saved start time
        conn.vehicle.setSpeedMode(agent, speed_mode)
        conn.vehicle.setLaneChangeMode(agent, 512)
        for time_s, saved in journal:
            if saved.get('lane_request'):
                break  # open-loop replay is only meaningful before the first lane maneuver
            conn.vehicle.setAcceleration(agent, saved['applied_acceleration'], DT)
            conn.simulationStep()
            now = round(conn.simulation.getTime(), 1)
            if agent not in conn.vehicle.getIDList():
                break
            row = dict(time_s=now, applied=saved['applied_acceleration'],
                       saved_realized=saved['realized_acceleration'],
                       replay_realized=conn.vehicle.getAcceleration(agent),
                       speed_m_s=conn.vehicle.getSpeed(agent), lane=conn.vehicle.getLaneID(agent),
                       lane_position_m=conn.vehicle.getLanePosition(agent))
            lane = row['lane']
            if not lane.startswith(':'):
                row['distance_to_lane_end_m'] = conn.lane.getLength(lane) - row['lane_position_m']
                # Distance needed to stop at the type's normal deceleration.
                row['stopping_distance_m'] = row['speed_m_s'] ** 2 / (2 * conn.vehicle.getDecel(agent))
            if window[0] <= now <= window[1]:
                row.update(junction_state(conn, agent))
                row['sb1_vehicles'] = sorted(
                    (dict(id=v, lane=conn.vehicle.getLaneID(v), pos_m=round(conn.vehicle.getLanePosition(v), 1),
                          speed_m_s=round(conn.vehicle.getSpeed(v), 2))
                     for v in conn.vehicle.getIDList() if conn.vehicle.getRoadID(v) in ('sb1', ':n1_0')),
                    key=lambda d: -d['pos_m'])
            samples.append(row)
            if now >= window[1] + 1.0:
                break
    finally:
        conn.close()
    return samples


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--agent', default='agent_0')
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--window', type=float, nargs=2, default=(1.4, 3.0))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    journal = load_journal(args.episode, args.agent)
    baseline = replay(args.episode, journal, args.agent, args.seed, 31, args.window)
    no_row = replay(args.episode, journal, args.agent, args.seed, 31 & ~8, args.window)
    matched = [abs(r['replay_realized'] - r['saved_realized']) <= 1e-3 for r in baseline]
    report = dict(schema='highway_flow_rl_v05_native_braking_replay', training=False, policy_used=False,
                  episode=str(args.episode).replace('\\', '/'), agent=args.agent, seed=args.seed,
                  reproduction=dict(samples=len(baseline), matched_within_1e3=sum(matched),
                                    first_mismatch_s=next((r['time_s'] for r, m in zip(baseline, matched) if not m), None)),
                  baseline_speed_mode_31=baseline, counterfactual_speed_mode_23=no_row,
                  limitation='Open-loop replay of saved commands; counterfactual mode is diagnostic only.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    report = main()
    print(json.dumps(report['reproduction']))
    for name in ('baseline_speed_mode_31', 'counterfactual_speed_mode_23'):
        print(name)
        for r in report[name]:
            if 1.4 <= r['time_s'] <= 3.0:
                links = r.get('next_links') or [{}]
                print(f"  {r['time_s']:.1f} applied {r['applied']:+.2f} saved {r['saved_realized']:+.2f} "
                      f"replay {r['replay_realized']:+.2f} v {r['speed_m_s']:.2f} {r['lane']}@{r['lane_position_m']:.1f} "
                      f"to-end {r.get('distance_to_lane_end_m', 0):.2f} stop {r.get('stopping_distance_m', 0):.2f} "
                      f"link {links[0].get('state')} prio={links[0].get('has_priority')} open={links[0].get('is_open')} "
                      f"foe={links[0].get('has_approaching_foe')} foes={[f['foe'] for f in r.get('junction_foes') or []]} "
                      f"sb1_lane0={[v['id'] for v in r.get('sb1_vehicles', []) if v['lane'].endswith('_0')]}")
