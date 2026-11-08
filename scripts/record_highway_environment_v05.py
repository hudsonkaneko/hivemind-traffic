"""Record one fixed v05 validation row with a checksum-verified frozen actor.

Actual SUMO front-bumper poses are retained for the complete 80-second primary
horizon, independently of the short suggested video crop. No optimizer is used.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
import torch

from experiments import run_highway_environment_v05 as runner
from traffic.sumo_warnings import native_emergency_report

SCHEMA = 'highway_environment_v05_frozen_recording'
EVENT_SCHEMA = 'observed_junction_entry_and_settled_merge_v2'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def readiness_provenance(cfg):
    """Never reuse stale training preflight as proof for a changed treatment."""
    ready = cfg.get('readiness')
    if ready is not None:
        if not isinstance(ready, dict) or not ready:
            raise ValueError('Readiness must be a valid proof object or explicit null')
        runner.readiness_check(cfg)
    return dict(status='verified' if ready is not None else 'not_provided',
        proof=deepcopy(ready), training=False, optimizer_steps=0,
        scope='Runner source/config preflight only; recorder source is separately snapshotted',
        interpretation=('Provided preflight matches the current runner sources and configuration' if ready is not None else
                        'Frozen inference diagnostic; no source/config preflight proof supplied'),
        training_authorization_used=False)


class JunctionEntryTracker:
    """Observe ramp -> internal link -> mainline joins without changing scores.

    A completed entry means the front bumper is observed on the connected
    external successor. Auxiliary entry is not a completed through-lane merge.
    No events are inferred for vehicles first observed downstream of the join.
    """
    def __init__(self, graph, controlled_ids):
        self.graph, self.controlled = graph, set(controlled_ids)
        self.joins = {}
        for via, (source, target) in graph.bindings.items():
            if (graph.identity(source).get('kind') == 'ramp'
                    and graph.identity(target).get('kind') in ('auxiliary', 'through')):
                self.joins.setdefault((source, target), set()).add(via)
        self.previous, self.pending = {}, {}

    def observe(self, states, time_s):
        events = []
        for vehicle, state in sorted(states.items()):
            lane, previous = state['lane_id'], self.previous.get(vehicle)
            for (source, target), vias in self.joins.items():
                pending = self.pending.get(vehicle)
                same_pending = pending is not None and pending['source_lane'] == source and pending['target_lane'] == target
                if lane in vias and previous == source and not same_pending:
                    event = dict(agent=vehicle, time=float(time_s), kind='native_ramp_junction_entry',
                        phase='started', controlled=vehicle in self.controlled, source_lane=source,
                        target_lane=target, target_lane_identity=deepcopy(self.graph.identity(target)),
                        lane_id=lane, internal_link_observed=True, internal_entry_time_s=float(time_s))
                    self.pending[vehicle] = event
                    events.append(deepcopy(event))
                elif lane == target and (previous == source or same_pending):
                    events.append(dict(agent=vehicle, time=float(time_s), kind='native_ramp_junction_entry',
                        phase='completed', controlled=vehicle in self.controlled, source_lane=source,
                        target_lane=target, target_lane_identity=deepcopy(self.graph.identity(target)),
                        lane_id=lane, internal_link_observed=bool(same_pending),
                        internal_entry_time_s=pending['internal_entry_time_s'] if same_pending else None))
                    self.pending.pop(vehicle, None)
            pending = self.pending.get(vehicle)
            if pending and lane not in self.joins[(pending['source_lane'], pending['target_lane'])]:
                self.pending.pop(vehicle, None)
        self.previous = {vehicle: state['lane_id'] for vehicle, state in states.items()}
        self.pending = {vehicle: value for vehicle, value in self.pending.items() if vehicle in states}
        return events


def merge_completion_event(env, agent, info, junction_events):
    """Describe the existing settled-through event using measured provenance."""
    state = env.states.get(agent)
    maneuver = getattr(env, 'completed_maneuvers', {}).get(agent)
    lateral = bool(info.get('maneuver', {}).get('completed_this_step') and maneuver
                   and math.isclose(maneuver['completed_at'], env.time, abs_tol=1e-8))
    native_join = any(event['agent'] == agent and event['phase'] == 'completed'
                      and event['target_lane_identity']['kind'] == 'through' for event in junction_events)
    completion_kind = ('policy_requested_lateral_maneuver' if lateral else
                       'native_route_join' if native_join else 'settled_through_lane_occupancy')
    kind = 'completed_lateral_merge' if lateral else 'completed_through_lane_merge'
    location = dict(lane_id=state['lane_id'], lane_identity=deepcopy(state['lane_identity']),
                    x=state['x'], y=state['y']) if state else {}
    return dict(agent=agent, time=float(env.time), kind=kind, completion_kind=completion_kind,
        commanded_lateral_completion_observed=lateral,
        completed_maneuver=deepcopy(maneuver) if lateral else None, **location)


def frame(env):
    """Read native current positions and heading, without smoothing or inference."""
    api = env.conn.vehicle
    controlled = set(env._controlled_agents())
    vehicles = []
    for identity, state in sorted(env.states.items()):
        x, y, z = map(float, api.getPosition3D(identity))
        values = dict(x=x, y=y, z=z, heading=float(api.getAngle(identity)),
            speed=float(state['speed']), acceleration=float(state['acceleration']),
            length=float(state['length']), width=float(api.getWidth(identity)),
            lateral_position=float(state['lateral_position']), lateral_speed=float(state['lateral_speed']))
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError('Nonfinite native vehicle pose or motion measurement')
        vehicles.append(dict(id=identity, **values, lane_id=state['lane_id'],
            lane_identity=deepcopy(state['lane_identity']), lane_position_m=float(state['lane_position']),
            lane_remaining_m=max(0., env.graph.lanes[state['lane_id']]['length'] - state['lane_position']),
            route=list(state['route']), distance_m=float(state['distance']),
            controlled=identity in controlled, merge_complete=identity in env.merged,
            stable_merge_status=env.merge_trackers[identity].status if identity in env.merge_trackers else None,
            maneuver_in_progress=identity in env.maneuvers))
    return dict(time=float(env.time), vehicles=vehicles)


def frozen_actions(actor, observations, agents):
    values = np.asarray([observations[agent] for agent in agents], dtype=np.float32)
    if values.shape != (len(agents), 99) or not np.isfinite(values).all() or not agents:
        raise ValueError('Frozen v05 policy requires nonempty finite 99-field observations')
    with torch.inference_mode():
        actions = actor(torch.from_numpy(values))
    if (not isinstance(actions, torch.Tensor) or actions.shape != (len(agents), 2)
            or not torch.isfinite(actions).all() or bool((actions.abs() > 1.).any())
            or not bool(((actions[:, 1] == -1.) | (actions[:, 1] == 0.) | (actions[:, 1] == 1.)).all())):
        raise ValueError('Frozen policy must return finite normalized acceleration and discrete lane intent')
    actions = actions.detach().cpu().numpy().astype(np.float32)
    return {agent: actions[index] for index, agent in enumerate(agents)}


def choose_clip(frames, merge_events):
    if not frames or any(b['time'] <= a['time'] for a, b in zip(frames, frames[1:])):
        raise ValueError('Recording needs strictly increasing native timestamps')
    first, last = frames[0]['time'], frames[-1]['time']
    event = min(merge_events, key=lambda row: (row['time'], row['agent'])) if merge_events else None
    start = max(first, event['time'] - 5.) if event else first
    end = min(last, event['time'] + 20.) if event else min(last, first + 25.)
    selected = [row for row in frames if start - 1e-6 <= row['time'] <= end + 1e-6]
    # The nominal 25-second window has 251 frames at 10 Hz; retain a strict
    # renderer-compatible cap even if a future recorder changes sampling rate.
    if len(selected) > 300:
        selected = selected[:300]
        end = selected[-1]['time']
    if not selected or end - start > 30. + 1e-8:
        raise ValueError('Suggested video crop must contain 1..300 frames within 30 seconds')
    return dict(clip_start_time=float(start), clip_end_time=float(end), clip_frame_count=len(selected),
        clip_label='First observed settled through-lane merge; stability is scored separately' if event else
                   'No settled through-lane merge observed; showing the first 25 seconds',
        crop_rule='First settled through-lane merge minus 5/plus 20 seconds; absent event: first 25 seconds; never reroll',
        crop_event=deepcopy(event))


def collect_episode(env, row, actor, *, deadline, frames, merge_events, stable_events, entry_events,
                    junction_entry_events=None):
    """Collect all native frames, including post-control primary accounting."""
    observations, _ = env.reset(seed=row['seed'])
    env.control_mode = 'policy'
    focal = next(iter(env.merge_agents), 'agent_0')
    controlled = list(env._controlled_agents())
    junction_entry_events = junction_entry_events if junction_entry_events is not None else []
    junctions = JunctionEntryTracker(env.graph, controlled)
    junctions.observe(env.states, env.time)
    frames.append(frame(env))
    previous_status, entered = {}, set()
    transitions, reward_sum = 0, 0.
    while env.agents:
        runner.check_deadline(deadline)
        agents = list(env.agents)
        actions = frozen_actions(actor, observations, agents)
        observations, rewards, _, _, infos = env.step(actions)
        new_junction_events = junctions.observe(env.states, env.time)
        junction_entry_events.extend(new_junction_events)
        transitions += len(rewards)
        reward_sum += sum(float(value) for value in rewards.values())
        if not math.isfinite(reward_sum):
            raise ValueError('Nonfinite recording reward')
        for agent, tracker in env.merge_trackers.items():
            if tracker.entry_time_s is not None and agent not in entered:
                entered.add(agent)
                entry_events.append(dict(agent=agent, time=tracker.entry_time_s, kind='through_lane_entry'))
        for agent, info in infos.items():
            state = env.states.get(agent)
            location = dict(lane_id=state['lane_id'], x=state['x'], y=state['y']) if state else {}
            if info.get('merge_completed'):
                merge_events.append(merge_completion_event(env, agent, info, new_junction_events))
            status = info.get('stable_merge_status')
            if status in ('succeeded', 'failed') and previous_status.get(agent) != status:
                stable_events.append(dict(agent=agent, time=float(env.time), status=status,
                    failure_reasons=env.merge_trackers[agent].summary()['failure_reasons'], **location))
            previous_status[agent] = status
        frames.append(frame(env))
    # The common accounting helper advances any remaining background traffic to
    # the fixed horizon. Hook only that drain so its frames are preserved too.
    original_advance = env._advance
    def recorded_advance():
        original_advance()
        junction_entry_events.extend(junctions.observe(env.states, env.time))
        frames.append(frame(env))
    env._advance = recorded_advance
    try:
        report = runner.primary_accounting(env, partial=False, deadline=deadline)
    finally:
        env._advance = original_advance
    report.update(seed=row['seed'], controller='learned', case_id=row['case']['id'],
        agent_transitions=transitions, reward=reward_sum, partial_control_horizon=False,
        population_setting={key: row[key] for key in ('controlled_count', 'total_vehicles', 'communication', 'packet_loss')},
        difficulty=row.get('difficulty', 0), traffic_mode=row.get('traffic_mode', 'initial'),
        training=False, optimizer_steps=0, policy_mode='frozen TorchScript deterministic inference',
        sumo_runtime_version=env.conn.getVersion() if hasattr(env.conn, 'getVersion') else None,
        junction_entry_accounting=dict(
            started=sum(e['phase'] == 'started' for e in junction_entry_events),
            completed=sum(e['phase'] == 'completed' for e in junction_entry_events),
            controlled_completed=sum(e['phase'] == 'completed' and e['controlled'] for e in junction_entry_events),
            definition='Observed ramp-to-connected-external-successor front-bumper entry; auxiliary/through identity retained',
            historical_through_entry_and_stable_merge_metrics_unchanged=True),
        full_report_is_separate_from_video_crop=True)
    return report, focal, controlled


def record(args):
    config_path, policy_path, output = args.config.resolve(), args.policy.resolve(), args.output.resolve()
    if Path.cwd().resolve() != ROOT or not output.is_relative_to(ROOT / 'outputs') or output == ROOT / 'outputs':
        raise ValueError('Run from project root and use a fresh child directory under outputs/')
    if not config_path.is_relative_to(ROOT) or not policy_path.is_relative_to(ROOT):
        raise ValueError('Frozen configuration and policy must be inside the project')
    if output.exists():
        raise ValueError('Recording output already exists; preserve prior evidence and choose a fresh directory')
    if not 1 <= args.max_wall_seconds <= 600:
        raise ValueError('Recording wall-time bound must be 1..600 seconds')
    if len(args.expected_policy_sha256) != 64 or any(c not in '0123456789abcdef' for c in args.expected_policy_sha256):
        raise ValueError('Expected policy SHA256 must be 64 lowercase hexadecimal characters')
    if sha256(policy_path) != args.expected_policy_sha256:
        raise ValueError('Frozen policy checksum mismatch')
    cfg = runner.load_config(config_path)
    if not 0 <= args.case_index < len(cfg['validation_cases']):
        raise ValueError('case-index must identify one fixed validation row')
    row = deepcopy(cfg['validation_cases'][args.case_index])
    readiness = readiness_provenance(cfg)
    network = (ROOT / row['case']['network']).resolve()
    if not network.is_relative_to(ROOT) or sha256(network) != row['case']['network_sha256']:
        raise ValueError('Selected frozen network checksum/path mismatch')
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(row['seed'])
    actor = torch.jit.load(str(policy_path), map_location='cpu').eval()
    # Validate the deployment interface before starting SUMO.
    probe = np.zeros(99, dtype=np.float32)
    probe[30:33] = 1.
    frozen_actions(actor, {'probe': probe}, ['probe'])
    output.mkdir(parents=True, exist_ok=False)
    hashes = runner.sources()
    hashes['scripts/record_highway_environment_v05.py'] = sha256(Path(__file__))
    scoped = [*hashes, config_path.relative_to(ROOT).as_posix()]
    manifest = dict(schema=SCHEMA, training=False, optimizer_steps=0, observation_size=99, max_agents=24,
        seed=row['seed'], validation_case_index=args.case_index, selected_case=row,
        selection='Exactly the specified validation row; no seed search, reroll or success-based reselection',
        policy=str(policy_path.relative_to(ROOT)), policy_sha256=args.expected_policy_sha256,
        checkpoint_sha256=args.expected_policy_sha256, config_path=str(config_path.relative_to(ROOT)),
        config_sha256=sha256(config_path), resolved_config_sha256=runner.config_digest(cfg),
        network=str(network.relative_to(ROOT)), network_sha256=row['case']['network_sha256'],
        readiness='verified runner source/config preflight proof' if readiness['status'] == 'verified' else 'not_provided; frozen inference only',
        readiness_provenance=readiness, event_schema=EVENT_SCHEMA,
        event_semantics='entry_events retain historical through entry; junction_entry_events are separate native route joins; merge_events retain settled through-merge timing with explicit completion provenance',
        source_sha256=hashes, motion_owner='SUMO', position_reference='SUMO front-bumper center',
        heading_convention='SUMO degrees clockwise from north; renderer rotationZ=90-heading',
        source_timestep=.1, horizon_steps=cfg['environment']['horizon'], horizon_seconds=80.,
        max_wall_seconds=args.max_wall_seconds, communication=row['communication'], packet_loss=row['packet_loss'],
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short', '--untracked-files=all', '--', *scoped], cwd=ROOT, text=True),
        python=platform.python_version(), torch=torch.__version__, renderer_compatible='scripts/render_i110_merge_replay.py',
        rendering='Actual recorded SUMO poses; no physical closed-loop or sensor-driven validation')
    runner.atomic_json(output / 'manifest.json', manifest)
    runner.atomic_json(output / 'config.json', cfg)
    shutil.copyfile(config_path, output / 'source-config.json')
    shutil.copyfile(network, output / 'network.net.xml')
    shutil.copyfile(policy_path, output / 'policy.pt')
    (output / 'working-tree.diff').write_bytes(subprocess.check_output(['git', 'diff', 'HEAD', '--binary', '--', *scoped], cwd=ROOT))
    for name, expected in hashes.items():
        destination = output / 'source' / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / name, destination)
        if sha256(destination) != expected:
            raise ValueError('Source changed while preparing recording snapshot')
    if sha256(output / 'policy.pt') != args.expected_policy_sha256 or sha256(output / 'network.net.xml') != row['case']['network_sha256']:
        raise ValueError('Copied frozen inputs failed checksum verification')
    started = time.monotonic()
    frames, merge_events, stable_events, entry_events, junction_entry_events = [], [], [], [], []
    report, error, focal, controlled = None, None, 'agent_0', []
    env = runner.env_for(cfg, output / 'simulation', row)
    try:
        report, focal, controlled = collect_episode(env, row, actor, deadline=started + args.max_wall_seconds,
            frames=frames, merge_events=merge_events, stable_events=stable_events, entry_events=entry_events,
            junction_entry_events=junction_entry_events)
    except Exception as exc:
        error = dict(error=repr(exc), traceback=traceback.format_exc())
        try:
            report = env.report()
        except Exception as report_error:
            report = dict(report_error=repr(report_error))
        report.update(primary_horizon_complete=False, accounting_comparison_valid=False, incomplete_episode=True)
        focal = next(iter(getattr(env, 'merge_agents', ())), 'agent_0')
        controlled = list(env._controlled_agents())
    finally:
        env.close()
    report['native_emergencies'] = native_emergency_report(env.directory / 'sumo-errors.log')
    report.update(policy_sha256=args.expected_policy_sha256, seed=row['seed'], training=False, optimizer_steps=0,
        wall_seconds=time.monotonic() - started, episode_directory=str(env.directory.relative_to(ROOT)),
        merge_events=merge_events, stable_events=stable_events, entry_events=entry_events,
        junction_entry_events=junction_entry_events, event_schema=EVENT_SCHEMA)
    unchanged = (sha256(policy_path) == args.expected_policy_sha256 and sha256(config_path) == manifest['config_sha256']
                 and sha256(network) == row['case']['network_sha256']
                 and sha256(output / 'policy.pt') == args.expected_policy_sha256
                 and sha256(output / 'source-config.json') == manifest['config_sha256']
                 and sha256(output / 'network.net.xml') == row['case']['network_sha256'])
    changed_sources = [name for name, expected in hashes.items() if sha256(ROOT / name) != expected]
    recording = dict(manifest, frames=frames, merge_events=merge_events, stable_events=stable_events,
        entry_events=entry_events, junction_entry_events=junction_entry_events,
        focal_vehicle=focal, controlled_ids=controlled,
        completed_recording=error is None, full_report_is_separate_from_video_crop=True,
        **(choose_clip(frames, merge_events) if frames else dict(clip_label='No frames recorded')))
    runner.atomic_json(output / 'recording.json', recording)
    runner.atomic_json(output / 'evaluation.json', report)
    complete = bool(report.get('primary_horizon_complete')) and bool(frames) and frames[-1]['time'] >= report.get('primary_horizon_time', math.inf) - 1e-8
    monotonic_time = all(a['time'] < b['time'] for a, b in zip(frames, frames[1:]))
    check = dict(passed=error is None and complete and unchanged and not changed_sources and monotonic_time,
        frames=len(frames), full_primary_horizon_retained=complete,
        recording_sha256=sha256(output / 'recording.json'), network_sha256=sha256(output / 'network.net.xml'),
        policy_unchanged=sha256(policy_path) == args.expected_policy_sha256, frozen_inputs_unchanged=unchanged,
        sources_changed=changed_sources, source_unchanged=not changed_sources,
        monotonic_time=monotonic_time,
        training=False, optimizer_steps=0, held_out_test_run=False,
        driving_acceptance='See full evaluation.json; successful recording does not imply accepted driving')
    runner.atomic_json(output / 'recording-check.json', check)
    if error:
        runner.atomic_json(output / 'failure.json', error)
    return dict(output=str(output), **check, focal_vehicle=focal, merge_events=merge_events,
        stable_events=stable_events, junction_entry_events=junction_entry_events,
        clip_start_time=recording.get('clip_start_time'),
        clip_end_time=recording.get('clip_end_time'), clip_label=recording['clip_label'], error=error and error['error'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--expected-policy-sha256', required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--case-index', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-wall-seconds', type=float, default=180.)
    result = record(parser.parse_args(argv))
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
