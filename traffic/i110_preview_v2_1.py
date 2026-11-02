"""Continuous conditional-path preview after v2's late-leader failures.

Adjacent projections are possibilities, not known merge intent or physical gaps.
They may limit closing speed but never request opening a following gap below the
other vehicle's speed. Side-by-side/overlapping projections are not leaders.
"""
from dataclasses import dataclass
import math
from traffic.i110_convergence import ConvergenceGraph
from traffic.i110_preview_v2 import PreviewV2Config, PreviewV2Controller, following_request


@dataclass(frozen=True)
class PreviewV21Config(PreviewV2Config):
    path_limit_m: float = 300.
    standstill_gap_m: float = 2.5


def conditional_request(speed, other_speed, projected_gap, relation, config=PreviewV21Config()):
    if relation == 'adjacent_lane_path' and projected_gap <= config.standstill_gap_m:
        return None
    requested=following_request(speed,other_speed,projected_gap-config.standstill_gap_m,config)
    if relation == 'adjacent_lane_path':
        # Closing-speed matching only: no forced gap-opening relative to a car
        # that has not entered the ego path. This removes v1's stop/dwell target.
        requested=max(requested,config.speed_gain*(other_speed-speed))
    return requested


class PreviewV21Controller(PreviewV2Controller):
    def __init__(self, network, config=PreviewV21Config()):
        super().__init__(config)
        self.graph=ConvergenceGraph(network,config.path_limit_m)

    def observe(self,conn,vid,states,time_s):
        result=super().observe(conn,vid,states,time_s)
        ego=states[vid];native=result['limiting_vehicle'];options=[];excluded=0;unknown=0
        if result['requested_ceiling_m_s2'] is not None:
            options.append((result['requested_ceiling_m_s2'],native,dict(relation='native_route_local_leader')))
        fields=('lane_id','lane_position','length','speed','acceleration','x','y')
        current={k:{f:s[f] for f in fields} for k,s in states.items()}
        for oid,other in sorted(current.items()):
            if oid in (vid,native) or math.hypot(other['x']-ego['x'],other['y']-ego['y'])>self.config.range_m:
                continue
            pair=self.graph.pair(current[vid],other)
            if pair['status']=='unknown_lane_context':unknown+=1
            if not pair.get('forward_candidate'):
                excluded+=1;continue
            request=conditional_request(ego['speed'],other['speed'],pair['projected_gap_m'],pair['relation'],self.config)
            if request is None:excluded+=1;continue
            options.append((request,oid,pair))
        limiting=min(options,key=lambda row:(row[0],row[1])) if options else None
        result.update(version='2.1',active=bool(options),context_known=result['context_known'] and unknown==0 and self.graph.paths(current[vid]) is not None,
            requested_ceiling_m_s2=limiting[0] if limiting else None,
            native_leader_id=native,limiting_vehicle=limiting[1] if limiting else None,
            limiting_projection=limiting[2] if limiting else None,
            candidate_count=len(options),excluded_candidate_count=excluded,unknown_projection_count=unknown,
            information='current exact state and static same/adjacent paths; adjacent means possible merge, not known intent')
        return result
