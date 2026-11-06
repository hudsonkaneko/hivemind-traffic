"""Aggregate saved control residuals without simulation, inference or learning.

Validation traces are diagnostic evidence only. Output contains aggregate
statistics and provenance, never observation/action demonstration rows. Native
causes below are explicitly analytical matches to pinned SUMO model equations,
not queries of hidden simulator decisions. No getFollowSpeed call is made.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from environments.highway_environment_v05 import ContinuousMotionLaneGraph
from traffic.highway_stage1_teacher import acc_response_ceiling, discrete_stop_speed

SOURCE_PATHS = ('environments/highway_environment_v05.py',
    'traffic/highway_stage1_teacher.py', 'traffic/highway_merge_context.py',
    'traffic/i110_policy_interface.py', 'environments/i110_merge_env.py')
MATCH_TOLERANCE = 1e-4  # model-match audit only; never an intervention threshold


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def distribution(values):
    values = sorted(values)
    if not values:
        return dict(samples=0, mean=None, median=None, p95=None, maximum=None)
    index = .95 * (len(values)-1)
    lower, upper = math.floor(index), math.ceil(index)
    return dict(samples=len(values), mean=statistics.fmean(values),
        median=statistics.median(values),
        p95=values[lower]+(values[upper]-values[lower])*(index-lower), maximum=values[-1])


def analytical_cause(ego, leader, info, *, yield_entry):
    """Classify evidence from current recorded state, leaving unknowns explicit."""
    actual = info['realized_acceleration']
    if actual is None:
        return 'no_realization_after_arrival', None
    limit = ego['speed'] + .1*info['acceleration_bounds']['speed_ceiling']
    if leader is not None:
        prediction = acc_response_ceiling(ego['speed'], limit,
            (leader['bumper_gap_m'], ego['speed']+leader['relative_speed_m_s']))
        if abs(prediction-actual) < MATCH_TOLERANCE:
            return 'acc_following_response_match', prediction
    if yield_entry:
        prediction = max(-4.5, min(2., (discrete_stop_speed(ego['lane_remaining_m']-.1)-ego['speed'])/.1))
        if abs(prediction-actual) < MATCH_TOLERANCE:
            return 'minor_approach_stopping_response_match', prediction
    return 'unresolved_native_difference', None


class Aggregate:
    def __init__(self):
        self.counts = Counter()
        self.metrics = defaultdict(list)
        self.causes = Counter()
        self.reward_terms = defaultdict(list)

    def add(self, info, ego, *, cause, previous_request=None):
        self.counts['decisions'] += 1
        for name in ('override', 'policy_filter_changed', 'native_realization_changed'):
            self.counts[name] += int(info[name])
        requested, filtered = info['requested_acceleration'], info['filtered_acceleration']
        target, actual = info['feasible_acceleration_target'], info['realized_acceleration']
        self.metrics['absolute_request_target_error_m_s2'].append(abs(requested-target))
        self.metrics['absolute_filtered_target_error_m_s2'].append(abs(filtered-target))
        self.metrics['requested_acceleration_m_s2'].append(requested)
        if previous_request is not None:
            self.metrics['absolute_requested_jerk_m_s3'].append(abs(requested-previous_request)/.1)
        if actual is not None:
            self.counts['realized_samples'] += 1
            self.metrics['absolute_native_target_error_m_s2'].append(abs(actual-target))
            self.metrics['absolute_filtered_native_error_m_s2'].append(abs(filtered-actual))
            self.metrics['realized_acceleration_m_s2'].append(actual)
            self.metrics['absolute_realized_jerk_m_s3'].append(abs(actual-ego['acceleration'])/.1)
            self.metrics['speed_m_s'].append(ego['speed'])
        else:
            self.counts['arrival_without_realization'] += 1
        if info['override']:
            self.causes[cause] += 1
        for key, value in info['reward_terms'].items():
            self.reward_terms[key].append(value)

    def report(self):
        return dict(counts=dict(self.counts),
            intervention_fraction=self.counts['override']/max(1,self.counts['decisions']),
            analytical_match_causes=dict(self.causes),
            residuals={k:distribution(v) for k,v in sorted(self.metrics.items())},
            mean_weighted_reward_terms={k:statistics.fmean(v) for k,v in sorted(self.reward_terms.items())})


def analyze(recording):
    recording = Path(recording)
    manifest = json.loads((recording/'manifest.json').read_text())
    for name in SOURCE_PATHS:
        if digest(ROOT/name) != manifest['source_sha256'][name]:
            raise ValueError(f'Current analytical helper differs from recorded source: {name}')
    saved = json.loads((recording/'recording.json').read_text())
    evaluation = json.loads((recording/'evaluation.json').read_text())
    config = json.loads((recording/'config.json').read_text())
    journals = list((recording/'simulation').glob('*/actions.jsonl.gz'))
    if len(journals) != 1:
        raise ValueError('Exactly one saved episode journal is required')
    journal = journals[0]
    with gzip.open(journal,'rt',encoding='utf-8') as file:
        rows = [json.loads(line) for line in file]
    frames = {round(frame['time'],6):frame for frame in saved['frames']}
    if len(frames) != len(saved['frames']):
        raise ValueError('Duplicate recording times')
    graph = ContinuousMotionLaneGraph(recording/'network.net.xml')
    widths = {lane.get('id'):float(lane.get('width','3.2'))
              for lane in ET.parse(recording/'network.net.xml').iter('lane')}
    all_steps, changed_steps = Aggregate(), Aggregate()
    agents, stages = defaultdict(Aggregate), defaultdict(Aggregate)
    previous = {}
    intervention_times = defaultdict(list)
    leaders = defaultdict(Counter)
    lateral_leader_overrides = Counter()
    type_root = ET.parse(journal.parent/'routes.rou.xml').getroot()
    types = {node.get('id'):dict(node.attrib) for node in type_root.findall('vType')}
    vehicle_types = {node.get('id'):node.get('type') for node in type_root.findall('vehicle')}
    for row in rows:
        before = frames[round(row['time']-.1,6)]
        states = {v['id']:dict(v,lane_position=v['lane_position_m'],lane_width=widths[v['lane_id']])
                  for v in before['vehicles']}
        for identity, info in row['agents'].items():
            ego = states[identity]
            leader = graph.neighbor(ego,states,ego['lane_id'],ego['lane_position'],forward=True)
            cause, _ = analytical_cause(ego,leader,info,yield_entry=ego['lane_id'] in graph.yield_entries)
            kind = ego['lane_identity']['kind']
            stage = ('ramp_approach' if kind == 'ramp' else 'auxiliary_lane' if kind == 'auxiliary'
                     else 'during_lane_change' if info['maneuver']['in_progress'] else 'through_lane')
            for aggregate in (all_steps, agents[identity], stages[stage]):
                aggregate.add(info,ego,cause=cause,previous_request=previous.get(identity))
            if info['override']:
                changed_steps.add(info,ego,cause=cause,previous_request=previous.get(identity))
                intervention_times[identity].append(row['time'])
                if leader:
                    leaders[identity][leader['id']] += 1
                    lateral_leader_overrides[identity] += int(abs(states[leader['id']]['lateral_position']) > .02)
            previous[identity] = info['requested_acceleration']
    overall = all_steps.report()
    if (overall['counts']['override'] != evaluation['overrides']
            or overall['counts']['decisions'] != evaluation['agent_steps']):
        raise ValueError('Saved journal counts differ from frozen evaluation')
    per_agent = {}
    for identity, aggregate in sorted(agents.items()):
        times = intervention_times[identity]
        per_agent[identity] = dict(**aggregate.report(),
            first_intervention_time_s=min(times) if times else None,
            last_intervention_time_s=max(times) if times else None,
            leaders_during_interventions=dict(leaders[identity]),
            interventions_with_laterally_displaced_leader=lateral_leader_overrides[identity],
            vehicle_type=types[vehicle_types[identity]])
    inputs = [recording/name for name in ('manifest.json','recording.json','evaluation.json','config.json','network.net.xml')]
    inputs += [journal,journal.parent/'routes.rou.xml']
    return dict(schema='highway_smooth_control_saved_diagnostic_v01',training=False,optimizer_steps=0,
        native_runs=0, policy_inference_calls=0, demonstration_export=False,
        allowed_use='Validation diagnosis and objective review only; never use as demonstration/training data',
        policy_sha256=manifest['policy_sha256'],seed=manifest['seed'],
        inputs={str(p.relative_to(ROOT)):digest(p) for p in inputs},
        analytical_sources={name:digest(ROOT/name) for name in SOURCE_PATHS},
        intervention_definition='Unchanged recorded override/policy-filter/native-realization booleans',
        denominator_note='Includes final arrival decisions. Native residual/jerk statistics exclude only the eight decisions with no post-arrival vehicle state; no decision is removed from the intervention denominator.',
        analytical_match_tolerance_m_s2=MATCH_TOLERANCE,
        analytical_match_limit='Numerical agreement with pinned model equations supports cause attribution; it does not change the recorded unattributed native flags',
        overall=overall,intervention_steps=changed_steps.report(),per_agent=per_agent,
        per_stage={k:v.report() for k,v in sorted(stages.items())},
        stage_definition='Pre-step recorded ramp/auxiliary identity takes precedence; other roads use the recorded post-step maneuver-in-progress flag, then through_lane.',
        objective_review=dict(evidence_scope='One fixed validation case; not a training or generalization result',
            priorities=[
                dict(topic='Following response consistency',
                    observed_interventions=overall['analytical_match_causes'].get('acc_following_response_match',0),
                    interpretation='Existing actor-visible following target already explains native response; preserve exact intervention accounting.'),
                dict(topic='Anticipatory minor-entry braking and smooth acceleration recovery',
                    observed_interventions=overall['analytical_match_causes'].get('minor_approach_stopping_response_match',0),
                    interpretation='Evaluate realized jerk as well as overrides: a released native constraint can cause a large jerk without an override on that tick.')],
            proposed_training_source='Separately generated training-only scenarios after explicit configuration/budget approval; never these validation trajectories'),
        reward_weights=config['reward_weights'],
        references=[dict(url='https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/cfmodels/MSCFModel_ACC.cpp',lines='110-123,185-220,224-286',role='Ordinary ACC following response and mode thresholds'),
            dict(url='https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/MSVehicle.cpp',lines='2777-2779,2801-2849',role='Minor-junction approach and waiting speed'),
            dict(url='https://raw.githubusercontent.com/eclipse-sumo/sumo/v1_27_1/src/microsim/cfmodels/MSCFModel.cpp',role='Discrete safe stop-speed calculation')],
        decision='Do not change the native car-following model, intervention threshold, or raw actor accounting to conceal this residual. Existing actor-visible teacher targets explain the mismatches; improving frozen weights needs separately authorized training or an explicitly different controller.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    source, output = args.recording.resolve(), args.output.resolve()
    if (Path.cwd().resolve()!=ROOT or not source.is_relative_to(ROOT/'outputs')
            or not output.is_relative_to(ROOT/'outputs') or output.exists()):
        raise ValueError('Use project-root execution, existing output recording, and a fresh output JSON')
    report = analyze(source)
    report['analysis_script_sha256'] = digest(__file__)
    report['command'] = [sys.executable,*sys.argv]
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',encoding='utf-8') as file:
        json.dump(report,file,indent=2,allow_nan=False)
        file.write('\n')
    print(json.dumps(dict(output=str(output),counts=report['overall']['counts'],
        causes=report['overall']['analytical_match_causes']),indent=2))


if __name__ == '__main__':
    main()
