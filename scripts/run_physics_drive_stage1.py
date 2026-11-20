"""Stage 1 of the physics drive: the PhysX Americano follows scripted commands on the empty LA road.

User decision (November 19, 2026): "Yes, policy drives it". Before the policy drives, this checks
the physics side alone: SUMO holds only the physics car's shadow on the southbound Adams on-ramp
corridor (no other traffic), and a fixed schedule of accelerations and lane changes, of the kind
the v14 environment issues, goes through the same path builder, link and controller as the
policy will. It reports how closely the car carries out each command (acceleration error per
0.1 s decision, lateral error at the front bumper, lane-change completion by the v14 rule:
on the target lane within 0.05 m of its centre and under 0.05 m/s lateral speed), whether all
four wheels stay on the deck, and the shadow placement error.

A development check of the physics integration: no policy, no training, no other traffic,
not charged to the study ledger. Output: a fresh directory under outputs/.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from traffic.physical_drive import LaneNetwork, reference_path  # noqa: E402
from traffic.physics_link import RemoteDriver, server_command  # noqa: E402

CORRIDOR = 'outputs/highway-flow-rl/v12/la-corridors-exact-01/la-110-sb-adams-onramp'
ROUTE = ['mlk_on', 'sb2', 'sb3', 'sb4', 'sb5']
DT = .1
MAX_SPEED = 26.8          # 60 mph
SCHEDULE = [              # (from time s, acceleration m/s2 or None to hold 60 mph, lane change -1/0/+1 at that time)
    (0., 2., 0), (8., None, 0), (11., None, +1), (16., None, +1), (19.5, -3., 0), (21., 1., -1), (23., 0., 0)]


def lateral_speed(previous, lane, lateral, now, lanes, network):
    if previous is None:
        return 0.
    old_lane, old_lateral, then = previous
    if old_lane != lane and network.getLane(old_lane).getEdge() == network.getLane(lane).getEdge():
        step = network.getLane(lane).getIndex() - network.getLane(old_lane).getIndex()
        old_lateral -= math.copysign(.5 * (lanes.width(old_lane) + lanes.width(lane)), step)
    return (lateral - old_lateral) / (now - then)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render-hz', type=int, default=0, choices=[0, 10])
    parser.add_argument('--seconds', type=float, default=40.)
    parser.add_argument('--depart-pos', type=float, default=40.)
    parser.add_argument('--depart-speed', type=float, default=12.)
    parser.add_argument('--vehicle-package', type=Path)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'outputs') or output.exists():
        parser.error('Use a fresh output directory under outputs/')
    output.mkdir(parents=True)
    import sumolib
    import traci
    network_path, manifest = ROOT / CORRIDOR / 'network.net.xml', ROOT / CORRIDOR / 'corridor-manifest.json'
    lanes = LaneNetwork(network_path)
    net = lanes.net
    driver = RemoteDriver(server_command(output / 'isaac', render_hz=args.render_hz, network=network_path,
                                         manifest=manifest, vehicle_package=args.vehicle_package),
                          log_path=output / 'isaac-server.log', cwd=ROOT)
    traci.start([sumolib.checkBinary('sumo'), '-n', str(network_path), '--step-length', str(DT),
                 '--lanechange.duration', '3', '--no-step-log', 'true', '--collision.action', 'warn',
                 '--begin', '0', '--error-log', str(output / 'sumo-errors.log')], label='stage1')
    conn = traci.getConnection('stage1')
    rows, completions = [], []
    try:
        conn.route.add('r', ROUTE)
        conn.vehicle.add('agent_0', 'r', departLane='0', departPos=str(args.depart_pos), departSpeed=str(args.depart_speed))
        conn.vehicle.setSpeedMode('agent_0', 96)
        conn.vehicle.setLaneChangeMode('agent_0', 0)
        conn.simulationStep()
        driver.reset(lanes)
        x, y = conn.vehicle.getPosition('agent_0')
        spawn = driver.spawn('agent_0', dict(x=x, y=y, angle=conn.vehicle.getAngle('agent_0'),
                                             speed=conn.vehicle.getSpeed('agent_0'), acceleration=0.,
                                             lane_id=conn.vehicle.getLaneID('agent_0'), route=ROUTE))
        maneuver, previous, scheduled = None, None, set()
        for step in range(int(round(args.seconds / DT))):
            now = round(conn.simulation.getTime(), 6)
            lane, lateral = conn.vehicle.getLaneID('agent_0'), conn.vehicle.getLateralLanePosition('agent_0')
            speed, (x, y) = conn.vehicle.getSpeed('agent_0'), conn.vehicle.getPosition('agent_0')
            lat_speed = lateral_speed(previous, lane, lateral, now, lanes, net)
            previous = (lane, lateral, now)
            on_target = (maneuver is not None and not lane.startswith(':')
                         and net.getLane(lane).getIndex() == maneuver['target_index'])
            if on_target and abs(lateral) < .05 and abs(lat_speed) < .05:     # the v14 completion rule
                completions.append(dict(maneuver, completed_at=now, duration_s=now - maneuver['started_at']))
                maneuver = None
            acceleration, change = 0., 0
            for index, (start, accel, delta) in enumerate(SCHEDULE):
                if now + 1e-9 >= start:
                    acceleration = accel
                    if delta and index not in scheduled and maneuver is None:
                        change, _ = delta, scheduled.add(index)       # waits for a lane change in progress
            if acceleration is None or (acceleration > 0 and speed >= MAX_SPEED):
                acceleration = max(-2., min(1., (MAX_SPEED - speed) / 1.))   # hold 60 mph
            edge = net.getLane(lane).getEdge()
            if change and maneuver is None and not edge.getID().startswith(':'):
                target = net.getLane(lane).getIndex() + change
                if 0 <= target < edge.getLaneNumber():
                    maneuver = dict(source_lane=lane, target_lane=f'{edge.getID()}_{target}', target_index=target,
                                    started_at=now, duration_s=3.)
            path = reference_path(lanes, ROUTE, lane, x, y, speed, maneuver=maneuver, now=now)
            command = dict(acceleration=acceleration, path=path, lane_id=lane,
                           maneuver=None if maneuver is None else dict(maneuver))
            pose = driver.advance({'agent_0': command}, DT, time=now, background=[])['agent_0']
            remaining = (net.getLane(lane).getLength() - conn.vehicle.getLanePosition('agent_0')
                         if edge.getID() == ROUTE[-1] else math.inf)
            telemetry = driver.replies[-1]
            rows.append(dict(time=now, lane=lane, lateral_m=lateral, lateral_speed_m_s=lat_speed, sumo_speed=speed,
                             commanded=acceleration, maneuver=command['maneuver'], physics=telemetry))
            if remaining <= pose['speed'] * DT + 1.:
                break                                          # the end of the corridor
            conn.vehicle.setSpeed('agent_0', pose['speed'])
            conn.vehicle.moveToXY('agent_0', '', -1, pose['x'], pose['y'], angle=pose['angle'], keepRoute=3,
                                  matchThreshold=100.)
            conn.simulationStep()
            sx, sy = conn.vehicle.getPosition('agent_0')
            rows[-1]['shadow_error_m'] = math.hypot(sx - pose['x'], sy - pose['y'])
    finally:
        (output / 'stage1-log.json').write_text(json.dumps(rows, indent=1), encoding='utf-8')
        conn.close()
        server_report = driver.close()
    errors = [abs(r['commanded'] - r['physics']['measured_acceleration']) for r in rows[1:]]
    unsaturated = [abs(r['commanded'] - r['physics']['measured_acceleration']) for r in rows[1:]
                   if not r['physics']['saturated_ticks']]
    steady = [abs(r['lateral_m']) for r in rows if r['maneuver'] is None]
    report = dict(schema='physics_drive_stage1_v01', route=ROUTE, schedule=SCHEDULE, decisions=len(rows),
                  spawn=spawn, end_time=rows[-1]['time'] if rows else None,
                  max_speed=max(r['physics']['speed'] for r in rows) if rows else None,
                  acceleration_error=dict(mean_abs=sum(errors) / len(errors), max_abs=max(errors),
                                          unsaturated_mean_abs=sum(unsaturated) / max(1, len(unsaturated)),
                                          unsaturated_max_abs=max(unsaturated, default=None),
                                          saturated_decisions=sum(bool(r['physics']['saturated_ticks']) for r in rows)),
                  lateral_outside_lane_changes=dict(max_abs_m=max(steady, default=None),
                                                    mean_abs_m=sum(steady) / max(1, len(steady))),
                  lane_changes_requested=sum(1 for r in rows if r['maneuver'] and r['maneuver']['started_at'] == r['time']),
                  lane_changes_completed=completions,
                  max_shadow_error_m=max((r.get('shadow_error_m', 0.) for r in rows), default=None),
                  wheels_off_ground_decisions=sum(not all(r['physics']['on_ground']) for r in rows),
                  max_abs_pitch_rad=max(abs(r['physics']['pitch_rad']) for r in rows),
                  max_abs_roll_rad=max(abs(r['physics']['roll_rad']) for r in rows),
                  server=server_report and server_report.get('report'),
                  note='development check of the physics integration; not charged to the study ledger')
    (output / 'stage1-log.json').write_text(json.dumps(rows, indent=1), encoding='utf-8')
    (output / 'stage1-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('decisions', 'end_time', 'max_speed', 'acceleration_error',
                                             'lateral_outside_lane_changes', 'lane_changes_requested',
                                             'max_shadow_error_m', 'wheels_off_ground_decisions',
                                             'max_abs_pitch_rad', 'max_abs_roll_rad')}, indent=1))
    print('completed lane changes:', [(c['target_lane'], round(c['duration_s'], 1)) for c in completions])


if __name__ == '__main__':
    main()
