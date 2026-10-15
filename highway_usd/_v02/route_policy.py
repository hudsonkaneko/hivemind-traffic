"""Seeded route plans over the static network; not a vehicle/merge controller.

Each plan starts on an auxiliary segment approaching a chosen diverge. It either
stays on the inner auxiliary ring for a lap, or takes the exit and rejoins at
one of the next four return opportunities. All routes end back on the auxiliary.
"""
import argparse
import hashlib
import json
import random
from pathlib import Path


def generate_plans(network,seed=20261006,vehicles=24,exit_probability=.65):
    if isinstance(seed,bool) or not isinstance(seed,int):
        raise ValueError('Seed must be an integer')
    if not 1<=vehicles<=10000 or not 0<=exit_probability<=1:
        raise ValueError('Use 1..10000 vehicles and exit_probability in [0,1]')
    rng=random.Random(seed)
    edges=network['edges']
    sides=['East','North','West','South']
    plans=[]
    def next_kind(edge,kind):
        matches=[key for key in edges[edge]['successors'] if edges[key]['kind']==kind]
        if len(matches)!=1:
            raise ValueError(f'Expected one {kind} continuation after {edge}')
        return matches[0]
    for vehicle in range(vehicles):
        side=rng.choice(sides)
        approaching=[key for key,e in edges.items() if e['kind']=='auxiliary' and e['end_node']==side+'Diverge']
        if len(approaching)!=1: raise ValueError('Invalid exit approach')
        start=approaching[0]
        route=[start]
        take_exit=rng.random()<exit_probability
        opportunity=rng.randint(1,4) if take_exit else None
        if take_exit:
            current=next_kind(start,'exit'); route.append(current)
            current=next_kind(current,'collector'); route.append(current)
            encountered=0
            # At most eight collector segments in one revolution.
            for _ in range(9):
                returns=[k for k in edges[current]['successors'] if edges[k]['kind']=='return']
                if returns:
                    encountered+=1
                    if encountered==opportunity:
                        current=returns[0]; route.append(current)
                        route.append(next_kind(current,'auxiliary'))
                        break
                current=next_kind(current,'collector'); route.append(current)
            else: raise ValueError('No return within one collector lap')
        else:
            current=start
            for _ in range(8):
                current=next_kind(current,'auxiliary'); route.append(current)
        for a,b in zip(route,route[1:]):
            if b not in edges[a]['successors']:
                raise ValueError('Disconnected generated route')
        plans.append(dict(vehicle_id=f'vehicle_{vehicle:04d}',start_side=side,take_exit=take_exit,
            return_opportunity=opportunity,return_edge=next((e for e in route if edges[e]['kind']=='return'),None),
            desired_speed_m_s=round(rng.uniform(26.8224,network['parameters']['target_speed_m_s']),4),
            release_delay_s=round(vehicle*4+rng.uniform(0,2),3),edges=route,
            route_length_m=sum(edges[k]['length_m'] for k in route)))
    return dict(schema_version=1,seed=seed,exit_probability=exit_probability,vehicles=vehicles,
        network_sha256=hashlib.sha256(json.dumps(network,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
        semantics='Plans on existing edges; no geometry changes, speed control, lane-change, yielding or collision avoidance. Release delays are per-plan requests, not a traffic safety guarantee.',plans=plans)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--network',type=Path,default=Path(__file__).with_name('navigation.json'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed',type=int,default=20261006)
    parser.add_argument('--vehicles',type=int,default=24)
    parser.add_argument('--exit-probability',type=float,default=.65)
    args=parser.parse_args()
    result=generate_plans(json.loads(args.network.read_text()),args.seed,args.vehicles,args.exit_probability)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(f'Saved {args.vehicles} seeded route plans to {args.output}')


if __name__=='__main__': main()
