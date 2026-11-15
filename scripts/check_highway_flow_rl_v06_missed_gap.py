"""V06 Phase 1: does missed-gap demand really put a mainline car beside the ramp car?

Native SUMO only: every car, including the would-be AV, drives under SUMO's own
model. No policy, no environment episode, no ledger charge. For each network and
seed this records the blocker's longitudinal offset from the front ramp car when
the ramp car first reaches the acceleration lane (route edge sb2). A policy-driven
car will move differently, so this checks the demand construction, not training.
"""
import argparse
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from traffic.highway_merge_context import Stage1LaneGraph  # noqa: E402
from traffic.highway_v05_demand import write_demand  # noqa: E402
from traffic.live_runtime import find_sumo  # noqa: E402
import traci  # noqa: E402

LEVEL_M = 4.8 + 2.5   # within one car length plus minimum gap: the first gap is closed


def check(network, through_lanes, speed_limit, seed, missed_gap, difficulty):
    graph = Stage1LaneGraph(network, through_lanes)
    with tempfile.TemporaryDirectory() as tmp:
        routes = Path(tmp) / 'routes.rou.xml'
        manifest = write_demand(routes, seed, graph, 'merge', 1, 8, speed_limit_m_s=speed_limit,
                                difficulty=difficulty, horizon_seconds=60., missed_gap=missed_gap,
                                controlled_car_follow_model='Krauss')
        info = manifest.scenario['missed_gap']
        ramp = next(r for r in manifest if r['route'] == 'ramp' and r['initial_cohort'] and r['slot'] == 'ramp_0')
        main = [r for r in manifest if r['route'] == 'main' and r['initial_cohort']
                and graph.identity(r['lane_id']).get('index_from_right') == 0]
        traci.start([str(find_sumo()), '-n', str(network), '-r', str(routes), '--step-length', '.1',
                     '--no-step-log', 'true', '--no-warnings', 'true'])
        try:
            result = None
            for step in range(600):
                traci.simulationStep()
                ids = set(traci.vehicle.getIDList())
                if ramp['vehicle_id'] not in ids:
                    continue
                if traci.vehicle.getRoadID(ramp['vehicle_id']) != 'sb2':
                    continue
                x = traci.vehicle.getLanePosition(ramp['vehicle_id'])
                offsets = {r['vehicle_id']: traci.vehicle.getLanePosition(r['vehicle_id']) - x
                           for r in main if r['vehicle_id'] in ids and traci.vehicle.getRoadID(r['vehicle_id']) == 'sb2'}
                nearest = min(offsets.items(), key=lambda kv: abs(kv[1]), default=(None, None))
                result = dict(time_s=round((step + 1) * .1, 1), nearest_vehicle=nearest[0],
                              nearest_offset_m=nearest[1],
                              first_gap_closed=nearest[1] is not None and abs(nearest[1]) <= LEVEL_M,
                              blocker_offset_m=offsets.get((info or {}).get('blocker_slot')))
                break
        finally:
            traci.close()
    return dict(seed=seed, missed_gap=missed_gap, alignment=info, at_acceleration_lane=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--network', action='append', required=True, help='path:through_lanes:speed_limit_m_s')
    parser.add_argument('--seeds', type=int, nargs='+', required=True)
    parser.add_argument('--difficulty', type=int, default=1)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rows = []
    for spec in args.network:
        path, lanes, limit = spec.rsplit(':', 2)
        for seed in args.seeds:
            for mode in (False, True):
                try:
                    row = check(Path(path), int(lanes), float(limit), seed, mode, args.difficulty)
                except ValueError as error:
                    row = dict(seed=seed, missed_gap=mode, error=str(error))
                row['network'] = Path(path).parent.name
                rows.append(row)
                a = row.get('at_acceleration_lane') or {}
                print(row['network'], seed, 'missed_gap' if mode else 'normal', row.get('error') or
                      'offset %s closed %s' % (None if a.get('nearest_offset_m') is None else round(a['nearest_offset_m'], 1),
                                               a.get('first_gap_closed')))
    summary = {}
    for mode in (False, True):
        sel = [r for r in rows if r['missed_gap'] is mode and r.get('at_acceleration_lane')]
        summary['missed_gap' if mode else 'normal'] = dict(
            runs=sum(r['missed_gap'] is mode for r in rows), measured=len(sel),
            errors=sum(r['missed_gap'] is mode and 'error' in r for r in rows),
            first_gap_closed=sum(r['at_acceleration_lane']['first_gap_closed'] for r in sel))
    Path(args.output).write_text(json.dumps(dict(schema='highway_flow_rl_v06_missed_gap_check',
        level_threshold_m=LEVEL_M, scope=__doc__.strip().split('\n\n')[1], summary=summary, rows=rows), indent=1),
        encoding='utf-8')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
