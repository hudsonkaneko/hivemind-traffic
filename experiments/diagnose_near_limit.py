"""Offline frozen-actor boundary audit; no simulator and no optimizer step.

Saved physical traces are diagnosis only. Synthetic developmental probes are
constructed from the observation contract; neither is training data in this tool.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import uuid

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from traffic.mappo_physical_policy import FrozenPhysicalActor, StraightRoadPolicyBridge, CAP_M_S
from traffic.highway_stage1_teacher import teacher_action, longitudinal_target_v3

PHYSICAL_RUN = 'outputs/mappo-physical-adapter/v01/20261105T204908Z-32331979'
POLICY = 'policies/versions/highway-stage1-v03-phase0/policy.pt'
LIMITS = (6., 9., 12., 15., 18., 22.352, CAP_M_S)
OFFSETS = (-3., -1., -.5, -.1, 0., .1)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def boundary_bank():
    """Named current-state probes; no saved simulator trajectories are loaded."""
    rows = []
    for limit in LIMITS:
        bridge = StraightRoadPolicyBridge(road_length_m=2000, lane_count=3,
                                          lane_width_m=3.6, speed_limit_m_s=limit)
        for offset in OFFSETS:
            if limit + offset > CAP_M_S:
                continue  # Analytic schema clips speed above cap: not a distinct actor input.
            for lane in (0, 1):
                for previous in (-.5, 0., .5, 1.):
                    ego = dict(id='ego', x_m=100., y_m=lane*3.6, yaw_rad=0.,
                               speed_m_s=limit+offset, acceleration_m_s2=0., length_m=4.8, width_m=1.8)
                    observed = bridge.observe(ego, time_s=30., distance_travelled_m=100.,
                                              previous_request_m_s2=previous, last_lane_change_s=0.)
                    rows.append(dict(case='empty-road', limit_m_s=limit, speed_offset_m_s=offset,
                                     previous_request_m_s2=previous, **observed))
    # Preserve interactions with following/passing labels near the limit.
    for limit in (12., 18., 22.352, CAP_M_S):
        bridge = StraightRoadPolicyBridge(road_length_m=2000, lane_count=3,
                                          lane_width_m=3.6, speed_limit_m_s=limit)
        for offset in (-1., 0.):
            for gap in (20., 60., 100.):
                ego = dict(id='ego', x_m=100., y_m=0., yaw_rad=0., speed_m_s=limit+offset,
                           acceleration_m_s2=0., length_m=4.8, width_m=1.8)
                lead = dict(ego, id='lead', x_m=104.8+gap, speed_m_s=max(0.,limit-3.))
                observed = bridge.observe(ego, [lead], time_s=30., distance_travelled_m=100.)
                rows.append(dict(case='slower-leader', limit_m_s=limit, speed_offset_m_s=offset,
                                 previous_request_m_s2=0., **observed))
    return rows


def saved_physical_audit():
    reports = []
    for case in ('frozen_cruise', 'frozen_pass'):
        path = ROOT/PHYSICAL_RUN/case/'policy-decisions.json'
        rows = json.loads(path.read_text())
        correction = np.array([r['requested_acceleration_m_s2']-r['filtered_acceleration_m_s2'] for r in rows])
        near = [r for r in rows if (r['observation'][4]-r['observation'][0])*CAP_M_S <= .5]
        reports.append(dict(case=case, source=str(path.relative_to(ROOT)).replace('\\','/'),
            source_sha256=sha(path), decisions=len(rows), near_limit_decisions=len(near),
            override_decisions=sum(bool(r['causes']) for r in rows),
            correction_causes=dict(Counter(c for r in rows for c in r['causes'])),
            median_correction_m_s2=float(np.median(correction)), maximum_correction_m_s2=float(correction.max()),
            corrections_above_0p05_m_s2=int((correction>.05).sum()),
            positive_request_within_0p05_m_s_of_limit=sum(
                r['requested_acceleration_m_s2']>.05 and
                (r['observation'][4]-r['observation'][0])*CAP_M_S<.05 for r in rows),
            used_for_fitting=False))
    return reports


def model_digest(model):
    digest = hashlib.sha256()
    for key, value in sorted(model.state_dict().items()):
        digest.update(key.encode()); digest.update(value.detach().cpu().numpy().tobytes())
    return digest.hexdigest()


def boundary_report(actor):
    """Reusable no-update developmental sentinel for a batched exported/local actor.

    Returns compact aggregate metrics and explicitly synthetic rows. The caller
    retains ownership of model mode/state. No rollout, learning or RNG call.
    """
    rows = boundary_bank()
    observations = torch.tensor([r['observation'] for r in rows], dtype=torch.float32)
    with torch.no_grad():
        actions = actor(observations).cpu().numpy()
    if actions.shape != (len(rows), 2) or not np.isfinite(actions).all():
        raise ValueError('Boundary actor must produce finite normalized acceleration/lane pairs')
    for row, action in zip(rows, actions):
        row.update(requested_acceleration_m_s2=-4.5+(float(action[0])+1)*3.25,
                   lane_delta=int(action[1]), target_v3_m_s2=longitudinal_target_v3(row['observation']))
    groups = []
    for limit in LIMITS:
        part = [r for r in rows if r['case']=='empty-road' and r['limit_m_s']==limit]
        exact = [r for r in part if r['speed_offset_m_s']==0]
        near = [r for r in part if r['speed_offset_m_s']>=-1.]
        groups.append(dict(limit_m_s=limit, probes=len(part), at_limit_probes=len(exact),
            at_limit_raw_acceleration_mean_m_s2=float(np.mean([r['requested_acceleration_m_s2'] for r in exact])),
            at_limit_raw_acceleration_range_m_s2=[min(r['requested_acceleration_m_s2'] for r in exact),max(r['requested_acceleration_m_s2'] for r in exact)],
            at_limit_positive_requests=sum(r['requested_acceleration_m_s2']>.05 for r in exact),
            at_limit_positive_fraction=sum(r['requested_acceleration_m_s2']>.05 for r in exact)/len(exact),
            near_limit_teacher_mse_m2_s4=float(np.mean([(r['requested_acceleration_m_s2']-r['target_v3_m_s2'])**2 for r in near])),
            near_limit_raw_positive_fraction=sum(r['requested_acceleration_m_s2']>.05 for r in near)/len(near),
            at_limit_teacher_range_m_s2=[min(r['target_v3_m_s2'] for r in exact),max(r['target_v3_m_s2'] for r in exact)]))
    report = dict(rows=len(rows), source='synthetic developmental current-state probes; not held-out or training samples', groups=groups)
    return report, rows


def objective_probe(cfg, rows):
    """Differentiate the real new term at frozen weights; never call step/backward."""
    from experiments.run_highway_population import model_for, warm_start
    from policies.mappo import MappoTrainer
    with torch.random.fork_rng():
        torch.manual_seed(0)
        model = model_for(cfg)
        warm_start(model, cfg)
        coefficient = float(cfg['optimizer']['near_limit_coef'])
        band_m_s = float(cfg['optimizer']['near_limit_band_m_s'])
        trainer = MappoTrainer(model, near_limit_coef=coefficient,
                               near_limit_band_m_s=band_m_s)
        observations = torch.tensor([r['observation'] for r in rows], dtype=torch.float32)
        labels = torch.tensor(np.stack([teacher_action(r['observation'], version='v3') for r in rows]))
        before = model_digest(model)
        with torch.no_grad():
            actions_before = model.actor(observations).clone()
        raw_outputs = []
        handle = model.actor.mean_net.register_forward_hook(lambda module, arguments, output: raw_outputs.append(output))
        try:
            loss = trainer.near_limit_coef * trainer._near_limit_loss(observations, labels)
            raw = raw_outputs[-1]
            derivative = torch.autograd.grad(loss, raw)[0]
        finally:
            handle.remove()
        # Match the trainer's actual float32 predicate at the inclusive boundary.
        selected = observations[:,0] >= observations[:,4] - band_m_s/CAP_M_S
        difference = (raw[:,0].tanh()-labels[:,0])*3.25
        nonzero = selected & (difference.abs()>1e-6)
        direction = bool((derivative[nonzero,0]*difference[nonzero]>0).all())
        with torch.no_grad():
            actions_after = model.actor(observations)
        return dict(weighted_loss_m2_s4=float(loss.detach()), coefficient=coefficient, band_m_s=band_m_s,
            selected_count=int(selected.sum()), directional_rows=int(nonzero.sum()),
            gradient_descent_moves_toward_target=direction,
            direct_lane_output_derivative_zero=bool((derivative[:,1:]==0).all()),
            outside_band_derivative_zero=bool((derivative[~selected]==0).all()),
            weights_unchanged=before==model_digest(model), actions_unchanged=torch.equal(actions_before,actions_after),
            optimizer_state_entries=len(trainer.optimizer.state), parameter_gradients_untouched=all(p.grad is None for p in model.parameters()),
            zero_optimizer_steps=True,
            normalized_to_si_mse_factor=3.25**2,
            lane_retention_limit='Zero direct lane-logit derivative does not guarantee retention after future shared-trunk updates')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'experiments/configs/highway_population_v4.json')
    parser.add_argument('--skip-objective', action='store_true', help='Audit frozen responses before new objective API is available')
    args = parser.parse_args()
    if not Path.cwd().resolve().is_relative_to(ROOT) or not args.config.resolve().is_relative_to(ROOT):
        raise ValueError('Use only the current checkout')
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    actor = FrozenPhysicalActor(ROOT/POLICY)
    before = sha(ROOT/POLICY)
    boundary, rows = boundary_report(actor.actor)
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
    output=ROOT/'outputs/near-limit-v01'/('diagnostic-'+stamp)
    output.mkdir(parents=True, exist_ok=False)
    result=dict(no_training=True, optimizer_steps=0, checkpoint_sha256=before,
        config_sha256=sha(args.config), source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        physical=saved_physical_audit(), synthetic_grouped=boundary['groups'], synthetic_rows=len(rows),
        objective=None if args.skip_objective else objective_probe(cfg,rows),
        checkpoint_unchanged=before==sha(ROOT/POLICY),
        limitations=['Synthetic developmental probes are not a closed-loop evaluation or a training dataset.',
                    'Saved physical validation traces are diagnosis-only; no fitting or optimizer step.',
                    'The original speed feature clips above 60 mph; probes exceeding that cap are omitted.',
                    'Unchanged weights still need training and complete matched evaluation before improvement can be claimed.'])
    (output/'rows.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf-8')
    result['rows_sha256']=sha(output/'rows.json')
    result['local_output']=str(output.relative_to(ROOT)).replace('\\','/')
    result['source_sha256']={name:sha(ROOT/name) for name in ('experiments/diagnose_near_limit.py',
        'traffic/mappo_physical_policy.py','traffic/highway_stage1_teacher.py','policies/mappo.py')}
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
