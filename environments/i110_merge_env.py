"""I-110 merge pilot: explicit new ABI, SUMO motion, no legacy actor loading."""
from __future__ import annotations
from functools import lru_cache
from pathlib import Path
import json
import gzip
import math
import shutil
import uuid
import xml.etree.ElementTree as ET
import gymnasium as gym
import numpy as np
import traci
from pettingzoo import ParallelEnv
from environments.sumo_mappo_env import ManeuverActionSpace, action_to_acceleration, physical_acceleration_to_action
from traffic.i110_policy_interface import LaneGraph, FEATURES
from traffic.sumo_backend import find_sumo_binary
from traffic.trip_accounting import TripLedger, read_requests

ROOT = Path(__file__).resolve().parents[1]
AGENTS = tuple(f'agent_{i}' for i in range(4))
SCHEMA = 'highway_curriculum_graph_messages_v1'
LOCAL_FEATURES = (*FEATURES[:26], 'lane_count_over_7', 'distance_to_required_merge_over_240', 'requires_merge',
                  'lane_cooldown_over_5', 'mask_right', 'mask_stay', 'mask_left')
MESSAGE_FEATURES = ('southbound_position_delta_over_150', 'sender_requires_merge',
                    'relative_speed_over_cap', 'acceleration_over_4p5',
                    'previous_acceleration_request_over_4p5', 'age_over_0p5',
                    'previous_lane_request', 'present')


class MergeLaneGraph(LaneGraph):
    def neighbor(self, ego, states, lid, position, *, forward):
        if forward: return super().neighbor(ego,states,lid,position,forward=True)
        candidates=[]
        for vid,other in states.items():
            if vid==ego['id']:continue
            # Rear traffic may approach on a different incoming route. Include
            # that route's real graph links, never compare restarted edge positions.
            distances=self.distances(lid,position,tuple(other['route'])+tuple(ego['route']),forward=False,limit=130.)
            if other['lane_id'] not in distances:continue
            delta=distances[other['lane_id']]+self.lanes[other['lane_id']]['length']-other['lane_position']
            if delta<0:continue
            gap=max(0.,delta-ego['length'])
            if gap<=120:candidates.append((gap,vid,other['speed']))
        if not candidates:return None
        gap,vid,speed=min(candidates)
        return dict(id=vid,bumper_gap_m=gap,relative_speed_m_s=speed-ego['speed'])


def write_demand(path, seed, graph, task='merge', density='medium'):
    rng = np.random.default_rng(seed)
    routes = ET.Element('routes')
    for name, model in [('controlled', 'ACC'), ('human', 'IDM')]:
        ET.SubElement(routes, 'vType', id=name, carFollowModel=model, length='4.8',
            minGap='2.5', tau='1.5', accel='2', decel='4.5', emergencyDecel='9',
            maxSpeed='26.8224', speedFactor='1', speedDev='0', sigma='0')
    for name, edges in [('main', 'sb1 sb2 sb3 sb4 sb5'),
                        ('ramp', 'mlk_on sb2 sb3 sb4 sb5')]:
        ET.SubElement(routes, 'route', id=name, edges=edges)
    vehicles = []
    ramp_length = graph.lanes['mlk_on_0']['length']
    if ramp_length < 90: raise ValueError('Ramp too short for declared initial platoon')
    for i in range(4):
        ramp = i >= 2 and task=='merge'
        position = (ramp_length-25 if i == 2 else ramp_length-85) if ramp else (110 if i == 0 else 45)
        if not ramp:position=110-20*(i//graph.through_lanes)
        position += float(rng.uniform(-10,10))
        vehicles.append((0., AGENTS[i], 'controlled', 'ramp' if ramp else 'main',
                         0 if ramp else 1+i%graph.through_lanes, position, float(rng.uniform(8,11) if ramp else rng.uniform(12,17))))
    count,interval={'light':(8,4.),'medium':(32,1.3),'dense':(56,.75)}[density]
    if task in ('passing','following'):
        # Deterministic route identity; varying initial gap, not a future label in observations.
        vehicles.append((0.,'slow_leader','human','main',1,220.,10. if task=='passing' else 16.))
    for i in range(count):
        vehicles.append((2+i*interval, f'human_{i}', 'human', 'main', int(rng.integers(1,graph.through_lanes+1)), 0., float(rng.uniform(12,17))))
    if task in ('following','passing'):
        for lane in range(1,graph.through_lanes+1):
            for j in range(2):
                # Front traffic exists before policy control, not only behind it.
                # Positions remain on the initial edge, including the short I-110 crop.
                length=graph.lanes[f'sb1_{lane}']['length']
                pos=min(length-12.,145.+j*35.)
                vehicles.append((0.,f'front_{lane}_{j}','human','main',lane,pos,float(rng.uniform(10,17))))
    for i in range(8 if task=='merge' else 0):
        vehicles.append((3+i*4.5, f'ramp_{i}', 'human', 'ramp', 0, 0., float(rng.uniform(7,11))))
    for depart, vid, typ, route, lane, pos, speed in sorted(vehicles, key=lambda x:(x[0],-x[5],x[1])):
        v = ET.SubElement(routes, 'vehicle', id=vid, type=typ, route=route, depart=f'{depart:.3f}',
            departLane=str(lane), departPos=str(pos), departSpeed=str(speed))
        ET.SubElement(v, 'param', key='research.origin', value=route)
    ET.ElementTree(routes).write(path, encoding='utf-8', xml_declaration=True)


class I110MergeEnv(ParallelEnv):
    metadata = {'name': SCHEMA, 'render_modes': [], 'is_parallelizable': True}
    possible_agents = list(AGENTS)
    observation_size = 81
    state_size = 328

    def __init__(self, network, run_directory, *, horizon=800, communication='local', packet_loss=0., through_lanes=4, task='merge', density='medium'):
        self.network = Path(network).resolve()
        self.output = Path(run_directory).resolve()
        if not self.output.is_relative_to(ROOT/'outputs'): raise ValueError('Outputs must stay inside project outputs')
        if communication not in ('none','local') or not 0 <= packet_loss <= 1: raise ValueError('Unsupported channel')
        if task not in ('cruise','following','passing','merge'):raise ValueError('Unknown driving task')
        self.graph = MergeLaneGraph(self.network,through_lanes=through_lanes)
        self.task,self.density=task,density
        self.horizon, self.communication, self.packet_loss = horizon, communication, packet_loss
        self.conn = None
        self.journal = None
        self.control_mode = 'policy'
        self.agents = []
        self.state_space = gym.spaces.Box(-1.,1.,(328,),np.float32)

    @lru_cache(None)
    def observation_space(self, agent): return gym.spaces.Box(-1.,1.,(81,),np.float32)

    @lru_cache(None)
    def action_space(self, agent): return ManeuverActionSpace()

    def close(self):
        if self.conn is not None:
            try: self.conn.close()
            finally: self.conn = None
        if self.journal is not None:
            self.journal.close();self.journal=None

    def _controlled_agents(self):
        """Stable episode participants; legacy environments retain four cars."""
        return AGENTS

    def _merge_agents(self):
        return ('agent_2', 'agent_3')

    def _write_demand(self, path, seed):
        write_demand(path, seed, self.graph, self.task, self.density)

    def _spawn_ready(self):
        return set(self._controlled_agents()).issubset(self.states)

    def reset(self, seed=None, options=None):
        self.close()
        seed = 0 if seed is None else int(seed)
        self.rng = np.random.default_rng(seed)
        self.directory = self.output / f'episode-{seed}-{uuid.uuid4().hex[:8]}'
        self.directory.mkdir(parents=True)
        self.journal=gzip.open(self.directory/'actions.jsonl.gz','wt',encoding='utf-8')
        shutil.copyfile(self.network, self.directory/'network.net.xml')
        self._write_demand(self.directory/'routes.rou.xml', seed)
        controlled = self._controlled_agents()
        self.ledger = TripLedger(read_requests(self.directory/'routes.rou.xml', controlled))
        label = 'i110-'+uuid.uuid4().hex
        command = [str(find_sumo_binary()), '-n', str(self.directory/'network.net.xml'),
            '-r', str(self.directory/'routes.rou.xml'), '--seed', str(seed), '--step-length', '.1',
            '--no-step-log', 'true', '--duration-log.disable', 'true', '--time-to-teleport', '-1',
            '--collision.action', 'warn', '--collision.check-junctions', 'true',
            '--error-log', str(self.directory/'sumo-errors.log')]
        self.previous = {a:(0.,0) for a in controlled}
        self.last_change = {a:-5. for a in controlled}
        self.inbox = {a:{} for a in controlled}
        self.merged = set()
        self.passed = set()
        self.steps = 0
        self.stats = dict(collisions=0,teleports=0,overrides=0,agent_steps=0,lane_changes=0,
                          successful_merges=0,delivered_messages=0,dropped_messages=0,
                          delivered_bytes=0,overspeed_steps=0,peak_abs_jerk=0.,reward_terms={},
                          passes=0,speed_ratio_sum=0.,speed_samples=0)
        try:
            traci.start(command, label=label, verbose=False)
            self.conn = traci.getConnection(label)
            self.conn._socket.settimeout(30)
            for _ in range(100):
                self._advance()
                if self.stats['collisions'] or self.stats['teleports']: raise RuntimeError('Invalid spawn event')
                if self._spawn_ready(): break
            else: raise RuntimeError('Controlled spawn timeout; no vehicle discarded')
            for a in controlled:
                self.conn.vehicle.setSpeedMode(a,31)
                self.conn.vehicle.setLaneChangeMode(a,512)
            self.agents = list(controlled)
            self.start_time = self.time
            self.obs = self._observations()
            return self.obs, {a:{} for a in self.agents}
        except BaseException:
            self.close(); raise

    def _advance(self):
        self.conn.simulationStep()
        sim = self.conn.simulation
        self.time = sim.getTime()
        self.arrived = set(sim.getArrivedIDList())
        self.collisions = set(sim.getCollidingVehiclesIDList())
        tele = sim.getStartingTeleportIDList()
        active = self.conn.vehicle.getIDList()
        self.ledger.observe(self.time, departed={a:self.conn.vehicle.getDeparture(a) for a in sim.getDepartedIDList()},
            arrived=self.arrived, active=active, teleport_started=tele, teleport_ended=sim.getEndingTeleportIDList())
        self.stats['collisions'] += len(self.collisions)
        self.stats['teleports'] += len(tele)
        self.states = {}
        for a in active:
            v = self.conn.vehicle
            x,y = v.getPosition(a)
            self.states[a] = dict(id=a,x=x,y=y,lane_id=v.getLaneID(a),lane_position=v.getLanePosition(a),
                route=v.getRoute(a),speed=v.getSpeed(a),acceleration=v.getAcceleration(a),
                distance=v.getDistance(a),allowed_speed=v.getAllowedSpeed(a),length=v.getLength(a))
        if self.task in ('passing','following') and 'slow_leader' in self.states:
            self.conn.vehicle.setSpeed('slow_leader',10. if self.task=='passing' else 16.)

    def merge_distance(self, s):
        distances = self.graph.distances(s['lane_id'],s['lane_position'],s['route'])
        if any(self.graph.lanes[l]['edge']==s['route'][-1] for l in distances): return None
        ends = [d+self.graph.lanes[l]['length'] for l,d in distances.items()
                if not (self.graph.successors[l] & set(distances))]
        return max(0.,min(ends)) if ends else 0.

    def _context(self, a):
        return self.graph.observe(a,self.states,self.time-self.start_time,self.horizon*.1,self.previous[a][0])

    def _mask(self, a, context):
        s=self.states[a];mask=[0.,1.,0.]
        if self.graph.lanes[s['lane_id']]['internal'] or self.time-self.last_change[a]<5: return mask
        for label,idx in [('right',0),('left',2)]:
            if not context['lane_exists'][label] or not context['adjacent_context_known']: continue
            neighbors=context['neighbors'][label]
            safe=True
            for direction in ('front','rear'):
                n=neighbors[direction]
                if n:
                    follower=s['speed'] if direction=='front' else max(0.,s['speed']+n['relative_speed_m_s'])
                    leader=max(0.,s['speed']+n['relative_speed_m_s']) if direction=='front' else s['speed']
                    required=2.5+max(1.5*follower,follower*.1+follower*follower/9-leader*leader/18)
                    safe &= n['bumper_gap_m']>=required
            if safe: mask[idx]=1.
        return mask

    def _observations(self):
        result={}
        for a in self.agents:
            if a not in self.states: continue
            s=self.states[a];c=self._context(a);distance=self.merge_distance(s)
            local=c['features'][:26]+[c['lane_count']/7,min(1.,distance/240) if distance is not None else 1.,
                float(distance is not None),max(0.,1-(self.time-self.last_change[a])/5),*self._mask(a,c)]
            slots=[]
            if self.communication=='local':
                for sender,m in sorted(self.inbox[a].items()):
                    age=self.time-m['time']
                    if age>.5+1e-8:continue
                    slots.extend([np.clip((s['y']-m['y'])/150,-1,1),m['requires_merge'],
                        np.clip((m['speed']-s['speed'])/26.8224,-1,1),np.clip(m['acceleration']/4.5,-1,1),
                        m['request']/4.5,min(1.,age/.5),m['lane_intent'],1.])
            vector=np.asarray(local+slots+[0.]*(48-len(slots)),np.float32)
            if not self.observation_space(a).contains(vector):raise ValueError('Invalid graph observation')
            result[a]=vector
        return result

    def state(self):
        return np.concatenate([np.r_[self.obs[a] if a in self.agents else np.zeros(81),float(a in self.agents)] for a in AGENTS]).astype(np.float32)

    def scripted_actions(self):
        actions={}
        for a,o in self.obs.items():
            speed=o[0]*26.8224;target=o[4]*26.8224
            if o[8]>.5: target=min(target,speed+o[7]*26.8224+.5*(o[6]*120-2.5-1.5*speed))
            delta=1. if o[28]>.5 and o[32]>.5 else 0.
            if not delta and o[8]>.5 and o[7]*26.8224 < -1 and o[32]>.5 and o[6]*120<90:
                delta=1.
            acceleration=np.clip((target-speed)*.8,-4.5,2.)
            actions[a]=np.array([physical_acceleration_to_action(acceleration),delta],np.float32)
        return actions

    def step(self, actions):
        agents=list(self.agents)
        if set(actions)!=set(agents):raise ValueError('One action for each active agent required')
        before=self.states; requested={}; applied={}; lane_requests={}; rejected={}
        # Validate the whole joint batch before issuing any command.
        for a in agents:
            action=np.asarray(actions[a],np.float32)
            if not self.action_space(a).contains(action):raise ValueError('Invalid hybrid action')
            requested[a]=action_to_acceleration(action[0]);lane_requests[a]=int(action[1])
        snapshots={a:dict(time=self.time,x=before[a]['x'],y=before[a]['y'],speed=before[a]['speed'],
            acceleration=before[a]['acceleration'],requires_merge=float(self.merge_distance(before[a]) is not None),
            request=self.previous[a][0],lane_intent=self.previous[a][1]) for a in agents}
        for a in agents:
            s=before[a];c=self._context(a);delta=lane_requests[a]
            rejected[a]=bool(delta and not self._mask(a,c)[delta+1])
            if self.control_mode=='policy' and delta and not rejected[a]:
                self.conn.vehicle.changeLaneRelative(a,delta,.1)
            accel=min(requested[a],(s['allowed_speed']-s['speed'])/.1)
            front=c['neighbors']['own']['front']
            if front:
                leader=max(0.,s['speed']+front['relative_speed_m_s'])
                safe=max(0.,leader+.5*(front['bumper_gap_m']-2.5-1.5*s['speed']))
                accel=min(accel,(safe-s['speed'])/.1)
            # Stop at an ending lane if a safe merge has not yet been possible.
            distance=self.merge_distance(s)
            if distance is not None:
                safe=math.sqrt(2*4.5*max(0.,distance-2.5))
                accel=min(accel,(safe-s['speed'])/.1)
            applied[a]=float(np.clip(accel,-4.5,2.))
            if self.control_mode=='policy':self.conn.vehicle.setAcceleration(a,applied[a],.1)
            self.previous[a]=(requested[a],delta)
        self._advance();self.steps+=1
        # Exactly one tick latency, 10 Hz, finite range, seeded independent loss.
        for receiver in agents:
            if receiver not in self.states or self.communication=='none':continue
            for sender,m in snapshots.items():
                if sender==receiver:continue
                if math.hypot(m['x']-before[receiver]['x'],m['y']-before[receiver]['y'])>150:continue
                if self.rng.random()<self.packet_loss:self.stats['dropped_messages']+=1;continue
                self.inbox[receiver][sender]=m
                self.stats['delivered_messages']+=1
                self.stats['delivered_bytes']+=len(json.dumps(m,sort_keys=True,separators=(',',':')).encode())
        rewards={};terminated={};truncated={};infos={}
        for a in agents:
            s=self.states.get(a);old=before[a]
            completed=a in self.arrived
            changed=bool(s and self.graph.identity(s['lane_id'])!=self.graph.identity(old['lane_id'])
                         and self.graph.lanes[s['lane_id']]['edge']==self.graph.lanes[old['lane_id']]['edge'])
            if changed:self.last_change[a]=self.time;self.stats['lane_changes']+=1
            merged=bool(self.task=='merge' and a in ('agent_2','agent_3') and s and a not in self.merged and self.graph.identity(s['lane_id']).get('kind')=='through')
            if merged:self.merged.add(a);self.stats['successful_merges']+=1
            passed=bool(self.task=='passing' and a not in self.passed and s and 'slow_leader' in self.states
                        and s['y']<self.states['slow_leader']['y']-10)
            if passed:self.passed.add(a);self.stats['passes']+=1
            realized=s['acceleration'] if s else applied[a]
            override=self.control_mode=='policy' and (rejected[a] or abs(applied[a]-requested[a])>1e-4 or (s is not None and abs(realized-applied[a])>.05))
            jerk=abs(realized-old['acceleration'])/.1
            progress=max(0.,s['distance']-old['distance']) if s else old['speed']*.1 if completed else 0.
            terms=dict(progress=progress/2.68224,merge=5.*merged,passing=2.*passed,arrival=2.*completed,
                collision=-20.*(a in self.collisions),intervention=-.2*override,jerk=-.01*jerk,lane_change=-.1*changed)
            rewards[a]=sum(terms.values())
            for k,v in terms.items():self.stats['reward_terms'][k]=self.stats['reward_terms'].get(k,0.)+v
            self.stats['overrides']+=int(override);self.stats['agent_steps']+=1
            self.stats['peak_abs_jerk']=max(self.stats['peak_abs_jerk'],jerk)
            self.stats['overspeed_steps']+=int(s is not None and s['speed']>s['allowed_speed']+1e-4)
            if s:
                self.stats['speed_ratio_sum']+=s['speed']/max(.1,s['allowed_speed']);self.stats['speed_samples']+=1
            terminated[a]=completed or a not in self.states or bool(self.collisions) or self.stats['teleports']>0
            truncated[a]=self.steps>=self.horizon and not terminated[a]
            infos[a]=dict(reward_terms=terms,requested_acceleration=requested[a],applied_acceleration=applied[a],
                realized_acceleration=realized,lane_request=lane_requests[a],lane_rejected=rejected[a],
                override=bool(override),merge_completed=merged,route_completed=completed)
        self.obs=self._observations()
        observations={a:self.obs.get(a,np.zeros(81,np.float32)) for a in agents}
        self.agents=[a for a in agents if not terminated[a] and not truncated[a]]
        self.obs={a:observations[a] for a in self.agents}
        self.journal.write(json.dumps(dict(time=self.time,control_mode=self.control_mode,agents=infos),allow_nan=False)+'\n')
        return observations,rewards,terminated,truncated,infos

    def report(self):
        return dict(self.stats, schema=SCHEMA,steps=self.steps,simulation_time=self.time,
            task=self.task,density=self.density,through_lanes=self.graph.through_lanes,
            mean_fraction_of_speed_limit=self.stats['speed_ratio_sum']/max(1,self.stats['speed_samples']),
            intervention_fraction=self.stats['overrides']/max(1,self.stats['agent_steps']),
            requested_trips=len(self.ledger.trips),completed_trips=sum(t['state']=='completed' for t in self.ledger.trips.values()),
            accounting=self.ledger.snapshot())

    def finish_accounting(self):
        """Keep the complete requested cohort through a fixed horizon, then drain.

        Controlled cars left at truncation are released to native SUMO during
        drain. Never count that completion as a learned merge or route success.
        """
        primary=self.start_time+self.horizon*.1
        while self.time<primary-1e-8:self._advance()
        report=self.report()
        for a in self._controlled_agents():
            if a in self.states:
                self.conn.vehicle.setSpeed(a,-1)
                self.conn.vehicle.setLaneChangeMode(a,1621)
        deadline=primary+60
        while self.conn.simulation.getMinExpectedNumber()>0 and self.time<deadline-1e-8:self._advance()
        report['drain_accounting']=self.ledger.snapshot()
        report['drain_is_native_uncontrolled']=True
        return report
