"""Fixed-cohort population randomization for the structured Stage 1 actor.

The pilot changes controller assignment at reset only. It does not implement a
live GUI slider or mid-episode controller handoff. SUMO remains motion owner.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import gymnasium as gym
import numpy as np

from environments.highway_stage1_env import HighwayStage1Env, OBSERVATION_SIZE

SCHEMA = 'highway_population_graph_messages_v1'
MAX_AGENTS = 8
STATE_SIZE = MAX_AGENTS * (OBSERVATION_SIZE + 1)


def _main_start(graph, distance, lane):
    """Map a cohort position onto the same route without excess insertion flow."""
    for edge_index, edge in enumerate(('sb1', 'sb2', 'sb3', 'sb4', 'sb5')):
        lane_index = lane if edge_index < 3 else lane - 1
        lane_data = graph.lanes.get(f'{edge}_{lane_index}')
        if lane_data is None:
            raise ValueError('Population route requires sb1..sb5 through lanes')
        if distance < lane_data['length'] - 5.:
            return edge_index, lane_index, distance
        # A seam may cut through the deterministic grid on a cropped road.
        # Put that start five metres into the next edge, moving at most ten
        # metres; sixty-metre row spacing retains native insertion clearance.
        distance = max(5., distance - lane_data['length'])
    raise ValueError('Population exceeds available route length')


def write_population_demand(path, seed, graph, task, controlled_count, total_vehicles):
    """Keep positions/routes/speeds fixed across AV counts for a given seed.

    AV assignment is a seeded permutation (merge tasks stratify one ramp car
    first). The slow leader, where required, is included in the requested total
    and excluded from AV assignment. No extra vehicles are inserted later.
    """
    rng = np.random.default_rng(np.random.SeedSequence([seed, 901]))
    assignment_rng = np.random.default_rng(np.random.SeedSequence([seed, 902]))
    leader_count = int(task in ('following', 'passing'))
    if controlled_count > total_vehicles - leader_count:
        raise ValueError('Following/passing requires a native slow leader within the total population')
    ramp_count = 2 if task == 'merge' else 0
    main_count = total_vehicles - ramp_count - leader_count
    physical = []
    for i in range(main_count):
        lane = 1 + i % graph.through_lanes
        distance = 25. + 60. * (i // graph.through_lanes)
        edge, lane, position = _main_start(graph, distance, lane)
        # Keep insertion feasible behind the 10 m/s passing leader for both
        # native IDM and ACC cars. Runtime speed targets/guards are unchanged.
        physical.append(dict(slot=f'main_{i}', route='main', edge=edge, lane=lane,
                             position=position, speed=float(rng.uniform(11., 13.))))
    if leader_count:
        distance = 25. + 60. * math.ceil(main_count / graph.through_lanes)
        edge, lane, position = _main_start(graph, distance, 1)
        physical.append(dict(slot='slow_leader', route='main', edge=edge, lane=lane,
                             position=position, speed=10. if task == 'passing' else 16.))
    ramp_length = graph.lanes['mlk_on_0']['length']
    if ramp_count and ramp_length < 90.:
        raise ValueError('Ramp too short for population fixture')
    for i in range(ramp_count):
        physical.append(dict(slot=f'ramp_{i}', route='ramp', edge=0, lane=0,
                             position=max(5., ramp_length - 35. - 60. * i),
                             speed=float(rng.uniform(7., 9.))))
    eligible = [v['slot'] for v in physical if v['slot'] != 'slow_leader']
    order = list(assignment_rng.permutation(eligible))
    if ramp_count:
        first_ramp = next(v for v in order if v.startswith('ramp_'))
        order.remove(first_ramp)
        order.insert(0, first_ramp)
    assignment = {slot: f'agent_{i}' for i, slot in enumerate(order[:controlled_count])}
    root = ET.Element('routes')
    for name, model in [('controlled', 'ACC'), ('human', 'IDM')]:
        ET.SubElement(root, 'vType', id=name, carFollowModel=model, length='4.8',
                      minGap='2.5', tau='1.5', accel='2', decel='4.5', emergencyDecel='9',
                      maxSpeed='26.8224', speedFactor='1', speedDev='0', sigma='0')
    ET.SubElement(root, 'route', id='main', edges='sb1 sb2 sb3 sb4 sb5')
    ET.SubElement(root, 'route', id='ramp', edges='mlk_on sb2 sb3 sb4 sb5')
    manifest = []
    # Front-to-back insertion order is independent of the selected AV subset.
    for item in sorted(physical, key=lambda v: (v['route'], -v['edge'], -v['position'], v['slot'])):
        controlled = item['slot'] in assignment
        vid = assignment.get(item['slot'], item['slot'])
        vehicle = ET.SubElement(root, 'vehicle', id=vid, type='controlled' if controlled else 'human',
            route=item['route'], depart='0', departEdge=str(item['edge']), departLane=str(item['lane']),
            departPos=str(item['position']), departSpeed=str(item['speed']))
        ET.SubElement(vehicle, 'param', key='research.origin', value=item['route'])
        ET.SubElement(vehicle, 'param', key='research.population_slot', value=item['slot'])
        manifest.append(dict(item, vehicle_id=vid, controlled=controlled))
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
    return manifest


class HighwayPopulationEnv(HighwayStage1Env):
    metadata = dict(HighwayStage1Env.metadata, name=SCHEMA)
    state_size = STATE_SIZE

    def __init__(self, network, run_directory, *, controlled_count, total_vehicles,
                 max_agents=MAX_AGENTS, **kwargs):
        if max_agents != MAX_AGENTS:
            raise ValueError('Population v1 declares exactly eight stable critic slots')
        if (isinstance(controlled_count, bool) or int(controlled_count) != controlled_count
                or not 0 <= controlled_count <= MAX_AGENTS):
            raise ValueError('controlled_count must be an integer from zero to eight')
        if (isinstance(total_vehicles, bool) or int(total_vehicles) != total_vehicles
                or total_vehicles not in (8, 16, 24) or controlled_count > total_vehicles):
            raise ValueError('total_vehicles must be 8, 16 or 24 and include all AVs')
        if kwargs.get('task', 'merge') in ('following', 'passing') and controlled_count == total_vehicles:
            raise ValueError('Following/passing requires a native slow leader within the total population')
        self.controlled_count, self.total_vehicles = int(controlled_count), int(total_vehicles)
        self.possible_agents = [f'agent_{i}' for i in range(MAX_AGENTS)]
        self.population_manifest = []
        self.merge_agents = ()
        self.population_stats = {}
        super().__init__(network, run_directory, **kwargs)
        self.state_space = gym.spaces.Box(-1., 1., (STATE_SIZE,), np.float32)

    def _controlled_agents(self):
        return tuple(self.possible_agents[:self.controlled_count])

    def _merge_agents(self):
        return self.merge_agents

    def _spawn_ready(self):
        return (super()._spawn_ready() and len(self.states) == self.total_vehicles)

    def _write_demand(self, path, seed):
        self.population_manifest = write_population_demand(path, seed, self.graph, self.task,
                                                         self.controlled_count, self.total_vehicles)
        self.merge_agents = tuple(v['vehicle_id'] for v in self.population_manifest
                                  if v['controlled'] and v['route'] == 'ramp')

    def reset(self, seed=None, options=None):
        if options and set(options) & {'controlled_count', 'total_vehicles', 'max_agents',
                                      'communication', 'packet_loss'}:
            raise ValueError('Population is configured at construction; live slider handoff is not implemented')
        self.population_stats = dict(road_samples=0, total_on_road_sum=0, controlled_on_road_sum=0,
                                    on_road_av_fraction_sum=0., maximum_on_road=0,
                                    peer_observations=0, selected_peer_sum=0, maximum_selected_peers=0,
                                    overflow_peer_observations=0, zero_peer_observations=0,
                                    selected_peer_histogram={str(i): 0 for i in range(7)})
        result = super().reset(seed=seed, options=options)
        if len(self.states) != self.total_vehicles:
            self.close()
            raise RuntimeError('Population fixture did not spawn the entire fixed cohort at reset')
        return result

    def _advance(self):
        super()._advance()
        stats = self.population_stats
        controlled = sum(a in self.states for a in self._controlled_agents())
        total = len(self.states)
        stats['road_samples'] += 1
        stats['total_on_road_sum'] += total
        stats['controlled_on_road_sum'] += controlled
        stats['on_road_av_fraction_sum'] += controlled / max(1, total)
        stats['maximum_on_road'] = max(stats['maximum_on_road'], total)

    def _selected_messages(self, agent):
        state = self.states[agent]
        eligible = [(sender, message) for sender, message in self.inbox[agent].items()
                    if sender in self.states and sender != agent and
                    -1e-8 <= self.time - message['time'] <= .5 + 1e-8]
        eligible.sort(key=lambda pair: (math.hypot(state['x'] - pair[1]['x'], state['y'] - pair[1]['y']), pair[0]))
        return eligible[:6]

    def _observations(self):
        observations = super()._observations()
        for agent, observation in observations.items():
            peers = int(observation[45:].reshape(6, 8)[:, -1].sum())
            fresh = sum(sender in self.states and sender != agent and
                        -1e-8 <= self.time - message['time'] <= .5 + 1e-8
                        for sender, message in self.inbox[agent].items())
            self.population_stats['peer_observations'] += 1
            self.population_stats['selected_peer_sum'] += peers
            self.population_stats['maximum_selected_peers'] = max(self.population_stats['maximum_selected_peers'], peers)
            self.population_stats['overflow_peer_observations'] += int(fresh > 6)
            self.population_stats['zero_peer_observations'] += int(peers == 0)
            self.population_stats['selected_peer_histogram'][str(peers)] += 1
        return observations

    def state(self):
        return np.concatenate([np.r_[self.obs[a] if a in self.agents else np.zeros(OBSERVATION_SIZE),
                                     float(a in self.agents)] for a in self.possible_agents]).astype(np.float32)

    def report(self):
        report = super().report()
        samples = max(1, self.population_stats['road_samples'])
        peers = max(1, self.population_stats['peer_observations'])
        report.update(schema=SCHEMA, controlled_count=self.controlled_count,
            total_vehicles=self.total_vehicles, requested_av_fraction=self.controlled_count / self.total_vehicles,
            merge_target_count=len(self.merge_agents), merge_target_ids=list(self.merge_agents),
            population_assignment=self.population_manifest,
            population_metrics=dict(self.population_stats,
                mean_total_on_road=self.population_stats['total_on_road_sum'] / samples,
                mean_controlled_on_road=self.population_stats['controlled_on_road_sum'] / samples,
                mean_on_road_av_fraction=self.population_stats['on_road_av_fraction_sum'] / samples,
                mean_selected_peers=self.population_stats['selected_peer_sum'] / peers),
            population_contract='Fixed requested cohort, controller assignment only at reset; actual on-road counts change with arrivals',
            message_selection='Nearest six fresh received sender snapshots by Euclidean distance, stable sender-ID tie break',
            communication_mode=self.communication, packet_loss=self.packet_loss)
        return report
