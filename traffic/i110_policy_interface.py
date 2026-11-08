"""Graph-aware SI observations for I-110; NOT Hudson's 27-feature actor ABI.

Lane positions restart at edges. Directed lane distances include internal links
and route-compatible branches; coordinates alone never establish a leader.
"""
from __future__ import annotations

import heapq
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SCHEMA='i110_graph_v1'
FEATURES=(
    'speed_over_cap','acceleration_over_4p5','lane_index_fraction','remaining_route_fraction',
    'allowed_speed_over_cap','previous_request_over_4p5',
    *(f'{lane}_{direction}_{feature}' for lane in ('own','left','right')
      for direction in ('front','rear') for feature in ('gap_over_120','relative_speed_over_cap','present')),
    'left_lane_exists','right_lane_exists','elapsed_fraction','lane_count_over_5',
    'adjacent_context_known','remaining_route_known')


class LaneGraph:
    def __init__(self, path, through_lanes=4):
        self.path=Path(path)
        self.lanes={};self.edges={};self.successors={};self.predecessors={};self.bindings={}
        root=ET.parse(path).getroot()
        for edge in root.findall('edge'):
            ids=[]
            for lane in edge.findall('lane'):
                lid=lane.get('id');ids.append(lid)
                self.lanes[lid]=dict(edge=edge.get('id'),index=int(lane.get('index')),
                    length=float(lane.get('length')),internal=edge.get('function')=='internal')
                self.successors[lid]=set();self.predecessors[lid]=set()
            self.edges[edge.get('id')]=ids
        for connection in root.findall('connection'):
            a=f"{connection.get('from')}_{connection.get('fromLane')}"
            b=f"{connection.get('to')}_{connection.get('toLane')}"
            via=connection.get('via')
            target=via or b
            self.successors[a].add(target);self.predecessors[target].add(a)
            if via and not self.lanes[a]['internal']:
                self.bindings[via]=(a,b)
        self.through_lanes=through_lanes

    def identity(self,lid):
        lane=self.lanes[lid]
        if lane['internal']:
            binding=self.bindings.get(lid)
            if not binding:return {'kind':'internal_unknown','id':lid}
            a,b=(self.identity(x) for x in binding)
            return a if a==b else dict(kind='transition',source=a,destination=b)
        if lane['edge'].startswith('sb'):
            shift=len(self.edges[lane['edge']])-self.through_lanes
            index=lane['index']-shift
            return dict(kind='through',index_from_right=index) if index>=0 else dict(kind='auxiliary')
        return dict(kind='ramp',edge=lane['edge'])

    def allowed(self,route):
        pairs=set(zip(route,route[1:]));allowed={lid for lid,l in self.lanes.items() if l['edge'] in route}
        for lid,(a,b) in self.bindings.items():
            if (self.lanes[a]['edge'],self.lanes[b]['edge']) in pairs:allowed.add(lid)
        return allowed

    def distances(self,lid,position,route,*,forward=True,limit=math.inf):
        if lid not in self.lanes or not math.isfinite(position) or position<0:
            raise ValueError('Unknown lane or invalid front-bumper lane position')
        allowed=self.allowed(route);allowed.add(lid)
        adjacency=self.successors if forward else self.predecessors
        initial=-position if forward else -(self.lanes[lid]['length']-position)
        distances={lid:initial};queue=[(initial,lid)]
        while queue:
            cost,current=heapq.heappop(queue)
            if cost!=distances[current]:continue
            cost+=self.lanes[current]['length']
            if cost>limit:continue
            for target in sorted(adjacency[current] & allowed):
                if cost<distances.get(target,math.inf):
                    distances[target]=cost;heapq.heappush(queue,(cost,target))
        return distances

    def adjacent(self,lid,delta):
        lane=self.lanes[lid]
        if not lane['internal']:
            target=lane['index']+delta
            return (self.edges[lane['edge']][target] if 0<=target<len(self.edges[lane['edge']]) else None),True
        binding=self.bindings.get(lid)
        if not binding:return None,False
        a,b=binding
        # Mainline internal links retain neighboring paths through the same seam.
        # A merge from a different road is not invented as a parallel lane.
        if not self.lanes[a]['edge'].startswith('sb'):return None,False
        target=self.lanes[a]['index']+delta
        if not 0<=target<len(self.edges[self.lanes[a]['edge']]):return None,True
        choices=[x for x,(c,d) in self.bindings.items()
            if self.lanes[c]['edge']==self.lanes[a]['edge'] and self.lanes[c]['index']==target
            and self.lanes[d]['edge']==self.lanes[b]['edge']]
        return (choices[0],True) if len(choices)==1 else (None,False)

    def neighbor(self,ego,states,lid,position,*,forward):
        distances=self.distances(lid,position,ego['route'],forward=forward,limit=130.)
        candidates=[]
        for vid,other in states.items():
            if vid==ego['id'] or other['lane_id'] not in distances:continue
            lane=self.lanes[other['lane_id']]
            delta=distances[other['lane_id']]+(other['lane_position'] if forward else lane['length']-other['lane_position'])
            if delta<0:continue
            gap=max(0.,delta-(other['length'] if forward else ego['length']))
            if gap<=120.:
                candidates.append((gap,vid,other['speed']))
        if not candidates:return None
        gap,vid,speed=min(candidates)
        return dict(id=vid,bumper_gap_m=gap,relative_speed_m_s=speed-ego['speed'])

    def observe(self,vid,states,time,horizon=60.,previous_request=0.,cap=26.8224):
        if vid not in states:raise KeyError('Observation requested for inactive vehicle')
        ego=dict(states[vid],id=vid);lid=ego['lane_id'];lane=self.lanes[lid]
        forward=self.distances(lid,ego['lane_position'],ego['route'])
        ends=[cost+self.lanes[k]['length'] for k,cost in forward.items() if self.lanes[k]['edge']==ego['route'][-1]]
        remaining=min(ends) if ends else None
        neighbors={};adjacent_known=True;exists={}
        for name,delta in (('own',0),('left',1),('right',-1)):
            other_lane,known=(lid,True) if delta==0 else self.adjacent(lid,delta)
            if delta:adjacent_known &= known;exists[name]=bool(other_lane) if known else None
            # Comparable progress within a lateral lane, using the common edge's
            # SUMO length convention. This is not a physical lateral sensor model.
            position=(ego['lane_position']/lane['length']*self.lanes[other_lane]['length']) if other_lane else None
            neighbors[name]={direction:self.neighbor(ego,states,other_lane,position,forward=direction=='front')
                if other_lane else None for direction in ('front','rear')}
        normal_lane=self.lanes[self.bindings[lid][0]] if lane['internal'] and lid in self.bindings else lane
        count=len(self.edges[normal_lane['edge']]);index=normal_lane['index']
        clip=lambda x,lo,hi:max(lo,min(hi,float(x)))
        fraction=remaining/(max(0.,ego['distance'])+remaining) if remaining is not None and remaining+ego['distance']>0 else 0.
        vector=[clip(ego['speed']/cap,0,1),clip(ego['acceleration']/4.5,-1,1),index/max(1,count-1),
            fraction,clip(ego['allowed_speed']/cap,0,1),clip(previous_request/4.5,-1,1)]
        for name in ('own','left','right'):
            for direction in ('front','rear'):
                item=neighbors[name][direction]
                vector.extend([item['bumper_gap_m']/120,clip(item['relative_speed_m_s']/cap,-1,1),1.] if item else [0.,0.,0.])
        vector.extend([float(bool(exists['left'])),float(bool(exists['right'])),clip(time/horizon,0,1),
            count/5,float(adjacent_known),float(remaining is not None)])
        assert len(vector)==len(FEATURES)==30 and all(math.isfinite(v) for v in vector)
        connected={v['id'] for group in neighbors.values() for v in group.values() if v}
        nearby=[dict(id=oid,euclidean_distance_m=math.hypot(s['x']-ego['x'],s['y']-ego['y']),lane_id=s['lane_id'])
            for oid,s in sorted(states.items()) if oid!=vid and oid not in connected
            and math.hypot(s['x']-ego['x'],s['y']-ego['y'])<=120.]
        return dict(schema=SCHEMA,vehicle=vid,time_s=time,lane_identity=self.identity(lid),
            lane_id=lid,lane_count=count,neighbors=neighbors,lane_exists=exists,adjacent_context_known=adjacent_known,
            remaining_route_m=remaining,nearby_other_paths=nearby,features=vector,
            legacy_checkpoint_compatible=False,information_source='exact SUMO state, not perception',
            limitations=['Other-path proximity is not a longitudinal leader or a merge-gap decision.',
                'Unknown adjacency is explicitly masked; no merge or lane-change action implemented.'])


def require_actor_schema(metadata):
    if metadata.get('observation_schema')!=SCHEMA or metadata.get('observation_size')!=len(FEATURES):
        raise ValueError('Actor contract mismatch: Hudson v02 27-feature actor cannot consume I-110 graph_v1 observations')
