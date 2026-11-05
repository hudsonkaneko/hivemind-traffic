"""Offline sentinels must not update weights, RNG, simulator state or labels."""
import json
import pytest
import torch

from experiments.diagnose_near_limit import (
    ROOT, boundary_bank, boundary_report, objective_probe, saved_physical_audit,
)


def test_bank_covers_boundary_recovery_and_following_without_saved_inputs():
    rows=boundary_bank()
    assert len(rows)==352
    assert {row['limit_m_s'] for row in rows}=={6,9,12,15,18,22.352,26.8224}
    assert {row['speed_offset_m_s'] for row in rows}=={-3,-1,-.5,-.1,0,.1}
    assert sum(row['case']=='slower-leader' for row in rows)==24
    assert all(len(row['observation'])==93 for row in rows)
    assert all(row['delivered_message_count']==0 for row in rows)
    assert all(max(abs(v) for v in row['observation'])<=1 for row in rows)


def test_boundary_report_uses_raw_actions_without_guard_or_training():
    class ConstantActor(torch.nn.Module):
        def forward(self,obs):
            # +2 m/s² regardless of limit, intentionally unsafe at boundary.
            return torch.stack((torch.ones(len(obs)),torch.zeros(len(obs))),dim=1)
    rng=torch.get_rng_state().clone()
    report,rows=boundary_report(ConstantActor())
    assert torch.equal(rng,torch.get_rng_state())
    assert report['rows']==352
    assert all(group['at_limit_positive_fraction']==1 for group in report['groups'])
    assert all(row['requested_acceleration_m_s2']==2 for row in rows)
    assert all(row['target_v3_m_s2']<0 for row in rows if row['speed_offset_m_s']==0)


def test_gradient_probe_preserves_state_and_targets_only_acceleration():
    cfg=json.loads((ROOT/'experiments/configs/highway_population_v4.json').read_text())
    if not (ROOT/cfg['initial_checkpoint']['path']).exists():
        pytest.skip('Frozen full checkpoint is an ignored local artifact')
    rng=torch.get_rng_state().clone()
    report=objective_probe(cfg,boundary_bank())
    assert torch.equal(rng,torch.get_rng_state())
    assert report['selected_count']>0
    for key in ('gradient_descent_moves_toward_target','direct_lane_output_derivative_zero',
                'outside_band_derivative_zero','weights_unchanged','actions_unchanged',
                'parameter_gradients_untouched','zero_optimizer_steps'):
        assert report[key], key
    assert report['optimizer_state_entries']==0
    assert report['coefficient']==cfg['optimizer']['near_limit_coef']
    assert report['band_m_s']==cfg['optimizer']['near_limit_band_m_s']


def test_saved_physical_evidence_is_diagnostic_only():
    if not (ROOT/'outputs/mappo-physical-adapter/v01/20261105T204908Z-32331979').exists():
        pytest.skip('Original physical traces are local evidence')
    reports=saved_physical_audit()
    assert [r['override_decisions'] for r in reports]==[312,161]
    assert all(not r['used_for_fitting'] for r in reports)
    assert all(set(r['correction_causes'])=={'speed_ceiling'} for r in reports)
