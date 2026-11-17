from copy import deepcopy
import json
from pathlib import Path

import pytest
import torch

from experiments.run_shared_highway_evaluation import (
    ROOT, accumulate, checked_actions, inside, load_actor, native_evidence, validate_spec,
)


def approved():
    spec = json.loads((ROOT / 'configs/evaluation/highway-shared-2026-11-17.json').read_text())
    spec.update(evaluation_authorized=True, training_authorized=False, held_out_authorized=False)
    spec['budget'].update(approved=True, max_episode_runs=30, max_wall_seconds=900, training_decisions=0)
    return spec


@pytest.mark.parametrize('change', ['training', 'heldout', 'episode_cap', 'time_cap', 'startup'])
def test_no_training_or_silent_budget_expansion(change):
    spec = approved()
    validate_spec(spec)
    if change == 'training': spec['training_authorized'] = True
    if change == 'heldout': spec['held_out_authorized'] = True
    if change == 'episode_cap': spec['budget']['max_episode_runs'] = 31
    if change == 'time_cap': spec['budget']['max_wall_seconds'] = 901
    if change == 'startup': spec['environment']['raw_request_scope'] = 'after_first_decision'
    with pytest.raises(ValueError): validate_spec(spec)


def test_actor_rejects_invalid_or_masked_actions():
    observations = torch.zeros(24, 117)
    observations[:, 31] = 1.
    actor = lambda _: torch.zeros(24, 2)
    assert checked_actions(actor, observations).shape == (24, 2)
    with pytest.raises(ValueError, match='masked lane'):
        checked_actions(lambda _: torch.ones(24, 2), observations)
    with pytest.raises(ValueError, match='Invalid normalized'):
        checked_actions(lambda _: torch.full((24, 2), float('nan')), observations)


def test_startup_diagnostics_do_not_hide_full_route_peak():
    stats = {}
    for i, value in enumerate([30., 8., 4., 3., 2., 1.]):
        accumulate(stats, 'agent_0', i, dict(realized_jerk=value,
            command_slew=None if i == 0 else .5))
    assert stats['agent_0']['0']['realized_jerk']['peak_abs'] == 30.
    assert stats['agent_0']['1']['realized_jerk']['peak_abs'] == 8.
    assert stats['agent_0']['5']['realized_jerk']['peak_abs'] == 1.
    assert stats['agent_0']['0']['command_slew']['samples'] == 5


def test_paths_cannot_escape_project(tmp_path):
    with pytest.raises(ValueError): inside(tmp_path, '../elsewhere.pt')


def test_actor_loading_rejects_checksum_before_unpickling(tmp_path):
    path = tmp_path / 'untrusted.pt'; path.write_bytes(b'not a checkpoint')
    candidate = dict(id='test', path_base='canonical_checkout', path='untrusted.pt',
        sha256='0'*64, artifact_type='full_checkpoint_actor_only')
    with pytest.raises(ValueError, match='checksum'):
        load_actor(candidate, tmp_path)


def test_native_evidence_uses_runtime_episode_child_not_case_parent(tmp_path):
    from types import SimpleNamespace
    child = tmp_path / 'case-00' / 'episode-seed-unique'
    child.mkdir(parents=True)
    (child / 'sumo-errors.log').write_text('')
    result = native_evidence(SimpleNamespace(directory=child))
    assert result['available'] and result['unique_events'] == 0
