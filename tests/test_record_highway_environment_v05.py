from types import SimpleNamespace

import numpy as np
import pytest
import torch

from scripts.record_highway_environment_v05 import choose_clip, collect_episode, frame, frozen_actions
from scripts import record_highway_environment_v05 as recorder


def test_clip_keeps_fixed_first_event_and_does_not_require_successful_stability():
    frames = [dict(time=index / 10) for index in range(801)]
    events = [dict(time=20., agent='agent_1'), dict(time=35., agent='agent_0')]
    clip = choose_clip(frames, events)
    assert (clip['clip_start_time'], clip['clip_end_time'], clip['clip_frame_count']) == (15., 40., 251)
    assert clip['crop_event'] == events[0]
    assert 'stability is scored separately' in clip['clip_label']


def test_absent_merge_is_labeled_and_uses_first_25_seconds_without_reroll():
    frames = [dict(time=.7 + index / 10) for index in range(801)]
    clip = choose_clip(frames, [])
    assert clip['clip_start_time'] == .7 and clip['clip_end_time'] == pytest.approx(25.7)
    assert 'No settled through-lane merge observed' in clip['clip_label']
    assert clip['clip_frame_count'] <= 300


def test_clip_rejects_duplicate_time_and_enforces_renderer_frame_cap():
    with pytest.raises(ValueError, match='strictly increasing'):
        choose_clip([dict(time=0.), dict(time=0.)], [])
    clip = choose_clip([dict(time=index / 100) for index in range(1000)], [])
    assert clip['clip_frame_count'] == 300


def test_frozen_actor_contract_is_99_finite_inputs_and_two_bounded_actions():
    observation = {'agent_0': np.zeros(99, np.float32)}
    actor = lambda values: torch.tensor([[.2, -1.]], dtype=values.dtype)
    result = frozen_actions(actor, observation, ['agent_0'])
    assert result['agent_0'].tolist() == pytest.approx([.2, -1.])
    with pytest.raises(ValueError, match='99-field'):
        frozen_actions(actor, {'agent_0': np.zeros(93)}, ['agent_0'])
    for values in ([[0., .5]], [[2., 0.]], [[float('nan'), 0.]], [[0.]]):
        with pytest.raises(ValueError, match='discrete lane intent'):
            frozen_actions(lambda _: torch.tensor(values), observation, ['agent_0'])


class FakeGraph:
    bindings = {'internal_0': ('ramp_0', 'auxiliary_0')}
    lanes = {lane: dict(length=100.) for lane in ('ramp_0', 'internal_0', 'auxiliary_0', 'through_1')}

    def identity(self, lane):
        return dict(kind=lane.split('_')[0])


class FakeEnv:
    def __init__(self):
        self.time = 0.
        self.agents = ['agent_0']
        self.merge_agents = ('agent_0',)
        self.merged, self.maneuvers = set(), {}
        self.graph = FakeGraph()
        self.merge_trackers = {'agent_0': SimpleNamespace(status='pending', entry_time_s=None,
            summary=lambda: dict(failure_reasons=[]))}
        self.conn = SimpleNamespace(vehicle=SimpleNamespace(
            getPosition3D=lambda _: (100. + self.time, 4., 2.5),
            getAngle=lambda _: 87., getWidth=lambda _: 1.8))
        self.states = {'agent_0': dict(x=-999., y=-999., speed=10., acceleration=.2,
            length=4.6, lateral_position=.1, lateral_speed=.2, lane_id='through_1',
            lane_identity=dict(kind='through'), lane_position=10., route=('ramp', 'through'), distance=20.)}

    def _controlled_agents(self):
        return ('agent_0',)

    def reset(self, seed):
        self.seed = seed
        return {'agent_0': np.zeros(99, np.float32)}, {}

    def _advance(self):
        self.time = round(self.time + .1, 6)

    def step(self, actions):
        self._advance()
        self.merged.add('agent_0')
        self.merge_trackers['agent_0'].entry_time_s = .1
        if self.time >= .2:
            self.agents = []
        return {'agent_0': np.zeros(99, np.float32)}, {'agent_0': 1.}, {}, {}, {
            'agent_0': dict(merge_completed=self.time == .1, stable_merge_status='pending')}


def test_frames_use_actual_native_xyz_and_heading_not_cached_or_interpolated_pose():
    result = frame(FakeEnv())['vehicles'][0]
    assert (result['x'], result['y'], result['z'], result['heading']) == (100., 4., 2.5, 87.)
    assert result['controlled'] and result['lateral_position'] == .1
    assert result['lane_identity'] == dict(kind='through')
    assert (result['lane_position_m'], result['lane_remaining_m'], result['distance_m']) == (10., 90., 20.)


def test_full_primary_drain_is_recorded_and_focal_is_selected_from_reset(monkeypatch):
    env = FakeEnv()
    original = env._advance
    def accounting(env, partial, deadline):
        assert not partial
        while env.time < .4:
            env._advance()
        return dict(primary_horizon_complete=True, primary_horizon_time=.4)
    monkeypatch.setattr(recorder.runner, 'primary_accounting', accounting)
    frames, merge, stable, entries = [], [], [], []
    row = dict(seed=123, case=dict(id='fixed'), controlled_count=1, total_vehicles=8,
               communication='none', packet_loss=0.)
    report, focal, controlled = collect_episode(env, row, lambda value: torch.zeros(len(value), 2),
        deadline=None, frames=frames, merge_events=merge, stable_events=stable, entry_events=entries)
    assert [value['time'] for value in frames] == [0., .1, .2, .3, .4]
    assert env._advance == original
    assert focal == 'agent_0' and controlled == ['agent_0']
    assert report['agent_transitions'] == 2 and report['optimizer_steps'] == 0
    assert len(merge) == len(entries) == 1 and not stable
    assert merge[0]['completion_kind'] == 'settled_through_lane_occupancy'
    assert merge[0]['kind'] != 'completed_lateral_merge'


def test_native_junction_entry_is_distinct_from_auxiliary_to_through_merge():
    tracker = recorder.JunctionEntryTracker(FakeGraph(), ['agent_0'])
    snapshot = lambda lane: {'agent_0': dict(lane_id=lane)}
    assert tracker.observe(snapshot('ramp_0'), 0.) == []
    started = tracker.observe(snapshot('internal_0'), .1)
    assert len(started) == 1 and started[0]['phase'] == 'started'
    assert tracker.observe(snapshot('internal_0'), .2) == []
    completed = tracker.observe(snapshot('auxiliary_0'), .3)
    assert completed[0]['phase'] == 'completed'
    assert completed[0]['target_lane_identity'] == dict(kind='auxiliary')
    assert completed[0]['internal_entry_time_s'] == .1
    assert completed[0]['controlled']
    assert tracker.observe(snapshot('auxiliary_0'), .4) == []
    assert tracker.observe(snapshot('through_1'), .5) == []


def test_junction_entry_handles_skipped_internal_tick_without_inventing_initial_entry():
    tracker = recorder.JunctionEntryTracker(FakeGraph(), [])
    assert tracker.observe({'already_downstream': dict(lane_id='auxiliary_0'),
                            'native': dict(lane_id='ramp_0')}, 0.) == []
    events = tracker.observe({'native': dict(lane_id='auxiliary_0')}, .1)
    assert len(events) == 1 and events[0]['phase'] == 'completed'
    assert not events[0]['controlled'] and not events[0]['internal_link_observed']
    assert events[0]['internal_entry_time_s'] is None


def test_junction_entry_removal_or_return_to_ramp_never_completes():
    tracker = recorder.JunctionEntryTracker(FakeGraph(), ['agent_0'])
    snapshot = lambda lane: {'agent_0': dict(lane_id=lane)}
    tracker.observe(snapshot('ramp_0'), 0.)
    tracker.observe(snapshot('internal_0'), .1)
    assert tracker.observe({}, .2) == []
    assert tracker.observe(snapshot('auxiliary_0'), .3) == []
    tracker.observe(snapshot('ramp_0'), .4)
    tracker.observe(snapshot('internal_0'), .5)
    assert tracker.observe(snapshot('ramp_0'), .6) == []
    assert not tracker.pending


def test_merge_kind_requires_actual_current_maneuver_completion_evidence():
    env = FakeEnv()
    info = dict(maneuver=dict(completed_this_step=True))
    env.completed_maneuvers = {'agent_0': dict(completed_at=0., started_at=-3., source_lane='auxiliary_0', target_lane='through_1')}
    event = recorder.merge_completion_event(env, 'agent_0', info, [])
    assert event['kind'] == 'completed_lateral_merge'
    assert event['completion_kind'] == 'policy_requested_lateral_maneuver'
    assert event['completed_maneuver'] == env.completed_maneuvers['agent_0']
    env.time = 1.
    event = recorder.merge_completion_event(env, 'agent_0', info, [])
    assert not event['commanded_lateral_completion_observed']
    assert event['completion_kind'] == 'settled_through_lane_occupancy'
    native_join = dict(agent='agent_0', phase='completed', target_lane_identity=dict(kind='through'))
    event = recorder.merge_completion_event(env, 'agent_0', {}, [native_join])
    assert event['completion_kind'] == 'native_route_join'
    assert event['kind'] != 'completed_lateral_merge'


def test_native_junction_events_continue_during_post_control_drain(monkeypatch):
    env = FakeEnv()
    def advance():
        env.time = round(env.time + .1, 6)
        lane = {0.1: 'ramp_0', 0.2: 'internal_0', 0.3: 'auxiliary_0', 0.4: 'auxiliary_0'}[env.time]
        env.states['native'] = dict(env.states['agent_0'], lane_id=lane, lane_identity=env.graph.identity(lane))
    env._advance = advance
    def accounting(env, partial, deadline):
        while env.time < .4:
            env._advance()
        return dict(primary_horizon_complete=True, primary_horizon_time=.4)
    monkeypatch.setattr(recorder.runner, 'primary_accounting', accounting)
    events = []
    report, _, _ = collect_episode(env, dict(seed=123, case=dict(id='fixed'), controlled_count=1,
        total_vehicles=8, communication='none', packet_loss=0.), lambda value: torch.zeros(len(value), 2),
        deadline=None, frames=[], merge_events=[], stable_events=[], entry_events=[], junction_entry_events=events)
    assert [(event['phase'], event['time']) for event in events] == [('started', .2), ('completed', .3)]
    assert report['junction_entry_accounting']['completed'] == 1
    assert report['junction_entry_accounting']['controlled_completed'] == 0


def test_readiness_is_optional_but_invalid_or_stale_proofs_never_bypass_validation(monkeypatch):
    assert recorder.readiness_provenance({})['status'] == 'not_provided'
    assert recorder.readiness_provenance(dict(readiness=None))['optimizer_steps'] == 0
    for malformed in ({}, False, ''):
        with pytest.raises(ValueError, match='valid proof'):
            recorder.readiness_provenance(dict(readiness=malformed))
    def reject(cfg):
        raise ValueError('Preflight used different sources')
    monkeypatch.setattr(recorder.runner, 'readiness_check', reject)
    with pytest.raises(ValueError, match='different sources'):
        recorder.readiness_provenance(dict(readiness=dict(summary_path='old-proof.json')))
    checked = []
    monkeypatch.setattr(recorder.runner, 'readiness_check', lambda cfg: checked.append(cfg))
    cfg = dict(readiness=dict(summary_path='verified.json'))
    proof = recorder.readiness_provenance(cfg)
    assert checked == [cfg] and proof['status'] == 'verified'
    assert not proof['training_authorization_used']
