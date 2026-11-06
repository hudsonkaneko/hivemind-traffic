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
    assert 'No completed lateral merge observed' in clip['clip_label']
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


class FakeEnv:
    def __init__(self):
        self.time = 0.
        self.agents = ['agent_0']
        self.merge_agents = ('agent_0',)
        self.merged, self.maneuvers = set(), {}
        self.merge_trackers = {'agent_0': SimpleNamespace(status='pending', entry_time_s=None,
            summary=lambda: dict(failure_reasons=[]))}
        self.conn = SimpleNamespace(vehicle=SimpleNamespace(
            getPosition3D=lambda _: (100. + self.time, 4., 2.5),
            getAngle=lambda _: 87., getWidth=lambda _: 1.8))
        self.states = {'agent_0': dict(x=-999., y=-999., speed=10., acceleration=.2,
            length=4.6, lateral_position=.1, lateral_speed=.2, lane_id='through_1')}

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
