"""Read saved v05 updates and emit compact objective-scale evidence; no training."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
from statistics import mean, median

ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / 'outputs/highway-environment/v05/train-20261106T024828Z-90b25880'
OUT = Path(__file__).with_name('objective-scale-audit.json')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    return dict(samples=len(values), mean=mean(values), median=median(values),
                minimum=min(values), maximum=max(values)) if values else dict(samples=0)


def main():
    if Path.cwd().resolve() != ROOT:
        raise ValueError('Run from the canonical checkout root')
    sources, learned, warmup = [], [], []
    for path in sorted(RUN.glob('update-*.json')):
        data = json.loads(path.read_text(encoding='utf-8'))
        (warmup if data['critic_only'] else learned).append(data)
        sources.append(dict(path=path.relative_to(ROOT).as_posix(), sha256=digest(path),
            critic_only=data['critic_only'], decisions=data['report']['agent_steps'],
            optimizer_steps=data['optimization']['optimizer_steps']))
    decisions = sum(row['report']['agent_steps'] for row in learned)
    rewards = defaultdict(float)
    for row in learned:
        for name, value in row['report']['reward_terms'].items():
            rewards[name] += value
    steps = [step for row in learned for step in row['optimization']['gradient_diagnostics']['actual_steps']]
    components = [row['optimization']['gradient_diagnostics']['representative_minibatch'] for row in learned]
    norms = {group: stats([step['before_clip'][group] for step in steps]) for group in ('actor', 'critic', 'joint')}
    component_norms = {name: stats([row['components'][name]['actor'] for row in components])
                       for name in ('ppo', 'acceleration', 'near_limit')}
    ratios = {name: stats([row['components'][name]['actor'] / row['components']['ppo']['actor']
                          for row in components if row['components']['ppo']['actor'] > 0])
              for name in ('acceleration', 'near_limit')}
    cosines = {}
    for name in ('ppo__acceleration', 'ppo__near_limit', 'acceleration__near_limit'):
        values = [row['actor_cosines'][name] for row in components if row['actor_cosines'][name] is not None]
        cosines[name] = dict(**stats(values), negative=sum(value < 0 for value in values),
                             undefined=len(components) - len(values))
    assert len(steps) == sum(row['optimization']['optimizer_steps'] for row in learned)
    assert abs(sum(rewards.values()) - sum(row['report']['reward'] for row in learned)) < 1e-6
    output = dict(schema='highway_smooth_proposal_saved_objective_audit_v01', training=False, optimizer_steps=0,
        native_runs=0, run=RUN.relative_to(ROOT).as_posix(), sources=sources,
        computation=dict(
            selection='All update-*.json in saved run; critic_only true excluded from PPO reward/gradient summaries',
            reward_mean='Sum each recorded weighted reward component across PPO reports divided by summed active agent_steps',
            gradient_statistics='Unweighted distribution over every actual optimizer-step record, not weighted by minibatch sample count',
            component_statistics='One first accepted minibatch per PPO update; ratio computed per minibatch then summarized',
            cosine_statistics='Only defined non-null cosine values; undefined zero-gradient pairs counted separately',
            limitations='Gradient norms/clipping are derivatives, not Adam parameter steps. Two partial PPO rollouts are included. Recorded jerk reward includes native dynamics and terminal no-state zeros; no requested/native causal separation.'),
        ppo=dict(rollouts=len(learned), decisions=decisions, optimizer_steps=len(steps),
            partial_rollouts=sum(row['report']['partial_control_horizon'] for row in learned),
            reward_terms={name: dict(sum=value, per_active_decision=value / decisions) for name, value in sorted(rewards.items())},
            reward_sum=sum(rewards.values()),
            jerk_cost_percent_of_progress=-rewards['jerk'] / rewards['progress'] * 100),
        excluded_critic_warmup=dict(rollouts=len(warmup), decisions=sum(row['report']['agent_steps'] for row in warmup),
            optimizer_steps=sum(row['optimization']['optimizer_steps'] for row in warmup)),
        actual_step_gradients=dict(clipped_steps=sum(step['global_clip_factor'] < 1. for step in steps),
            clipped_fraction=sum(step['global_clip_factor'] < 1. for step in steps) / len(steps),
            global_clip_factor=stats([step['global_clip_factor'] for step in steps]),
            before_clip_norms=norms, critic_exceeds_actor_steps=sum(step['before_clip']['critic'] > step['before_clip']['actor'] for step in steps)),
        first_minibatch_components=dict(observations=len(components), samples=[row['samples'] for row in components],
            weighted_actor_norms=component_norms, per_minibatch_actor_norm_ratios_to_ppo=ratios, actor_cosines=cosines),
        builder=dict(path=Path(__file__).relative_to(ROOT).as_posix(), sha256=digest(Path(__file__))))
    OUT.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(dict(output=OUT.relative_to(ROOT).as_posix(), ppo_rollouts=len(learned),
                         decisions=decisions, optimizer_steps=len(steps), bytes=OUT.stat().st_size)))


if __name__ == '__main__':
    main()
