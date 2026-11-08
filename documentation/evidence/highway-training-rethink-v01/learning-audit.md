# Learning and objective audit — read-only findings

2026-11-06. Scope: saved evidence and source inspection only. No simulator run,
model update, optimizer step, implementation change, or training authorization.
This file is an audit and a proposed sequence of experiments, not a launch config.
Project root: `C:/Users/hudso/Documents/highwaysim`.

## Main findings

1. The most recent learned v05 policy did **not generally become slower**. On the
   same 24 validation rows, pooled speed utilization rose from 0.915692582 to
   0.931405504, while interventions rose from 13.3749427% to 19.8287213%. Stable
   merges improved from 2/9 to 8/9. It was a mixed result with a clear intervention
   regression, not accepted driving. Current smooth reference v4 has never been
   trained into a new policy; its scripted behavior must not be described as a
   learned-policy result. Scripted actions come from its reference/lane formulas,
   not reward maximization: reward weights cannot be the cause of a zero-update
   scripted trajectory. They matter when deciding whether later learning would
   reinforce that behavior.
2. Slower behavior can be locally rational under the current objective. Progress
   is distance per tick, while intervention, acceleration-reference error and
   jerk are charged per tick. The smooth bundle multiplies acceleration alignment
   by four and roughly triples large-jerk cost, yet contains no explicit delay,
   stopped-time or all-trip boundary-time reward. It does not reward stopping
   outright, but the immediate costs of changing speed can outweigh a modest
   immediate progress gain. Whether that explains a particular trajectory remains
   a hypothesis until compared on matched trajectories and returns.
3. More imitation is not a remedy for an incorrect reference. The saved v4 frozen
   gradient audit has **strongly opposing PPO and acceleration-imitation gradients**
   on its first 256 active samples. The coefficient was raised fivefold while the
   reference changed; the weighted acceleration-gradient norm actually decreased.
   The prior bundle therefore cannot isolate the effect of imitation weight.
4. The completed v05 run learned only difficulty 0, including just four merge
   PPO episodes and no I-110 merge episode, then evaluated eight cases at each
   difficulty 0/1/2. That is a demonstrated exposure shortfall, not evidence that
   the harder cases are unlearnable.
5. I found no verified PPO sign, tanh-log-probability, inactive-slot, or
   termination-bootstrap bug in the inspected implementation. There are important
   design limitations: reward/evaluation mismatch, reference quality, sparse
   completion credit, memory-dependent residual-slew reward, pooled sampling and
   actor/critic clipping coupling. They should be isolated, not all changed at once.
6. Route utility and observation sufficiency come before stronger imitation.
   The native-reference audit independently identifies unnecessary passing into
   an ending auxiliary lane and an ending-lane feed-forward brake when its speed
   target is above the relevant limit. Those are concrete controller/visibility
   issues; a stronger equality loss would encode them more strongly. See the
   parent/native agent's evidence for trajectory-level causality. This audit
   independently verifies the teacher's bidirectional passing rule and that its
   lane mask establishes immediate admissibility, not long-term route utility.

## Exact learning contract and math

The actor receives 99 fixed analytically scaled fields; there is no running
observation normalizer. It uses a shared `(128,128)` MLP with a message encoder,
one Gaussian/tanh acceleration head, and three masked categorical lane choices.
The critic is a separate `(256,256)` value head over a focal row and pooled
controlled-agent row encodings. It consumes 24 rows of 99 observations plus a
presence bit: 2,400 values. It is not the complete native simulator state and is
not exported. See `policies/mappo.py:86`, `:124`, `:162`; observation construction
is `environments/highway_environment_v05.py:402` and `:439`.

Acceleration is `u = -1.25 + 3.25*z` m/s², where `z=tanh(latent)` lies in [-1,1].
Physical zero is z=5/13, not zero. A fresh zero-mean actor therefore initially
brakes, but this is **not** the current frozen actor's observed mean. The saved
v05 checkpoint has log_std=-2.043247699737549 (latent std 0.129607101668871), and
the saved frozen stochastic batch has positive mean requested acceleration.
See `environments/sumo_mappo_env.py:37`, `policies/mappo.py:205`, `:222`.

The likelihood ratio uses the sum of corrected tanh-Gaussian and categorical
log probabilities. Stored pre-tanh samples avoid inversion at saturation; the
stored observation supplies the lane mask. PPO uses clip=0.2, value_clip=0.2,
entropy coefficient 0.001, value coefficient 0.5, actor LR=1e-4, critic LR=3e-4,
four epochs, minibatches of 256, and a pre-step approximate-KL stop at 0.02.
The single Adam has separate parameter groups, but **one global 0.5 norm clip**
couples actor and critic derivative scale (`policies/mappo.py:661–708`). This is a
design choice, not a coding error. Adam parameter displacement is not determined
by gradient norm alone.

For each active transition, `delta = r + gamma*V(next) - V(now)`; actual
termination sets next value to zero, while time-limit truncation uses the final
observation value. Both end flags stop the GAE trace, inactive slots have zero
advantage, and final-state presence is retained for truncations. Gamma=.99 and
lambda=.95; the actor normalizes advantages over the active rollout **before**
minibatching. Returns remain unnormalized and detached. See
`policies/mappo.py:295–334`, `:576–603`, and
`experiments/run_highway_environment_v05.py:188–215`.

At a 0.1 s step, gamma's half-life is 6.89676 s and an event eight seconds away
is weighted by 0.447523. The geometric GAE trace scale 1/(1-gamma*lambda) is
16.8067 ticks, or 1.68067 s; the critic bootstrap carries longer-horizon value,
so this is not a hard 1.68 s planning limit. Weak value estimates make distant
merge/stability credit more dependent on successful bootstrapping.

Reset creates a fresh native session and fixed controlled cohort, with no silent
removal of failed spawns. Native warmup is zero in this configuration; the
initial real acceleration is retained for the first jerk measurement. The
episode ends for a controlled agent on arrival/disappearance, any observed
collision, or any teleport; the horizon is a truncation. Whole-route accounting
continues to 80 s after the last controlled vehicle finishes, but that native
tail supplies evaluation evidence rather than new controlled reward samples.
See `environments/i110_merge_env.py:144–186`,
`environments/highway_environment_v05.py:178–214`, `:582–610`, and
`experiments/run_highway_stage1.py:150–166`.

## Rewards and competing scales

For the historical v05 objective, each controlled tick receives these weighted
terms (`environments/highway_environment_v05.py:545–553`):

| Term | Exact weighted formula / event amount |
| --- | --- |
| Progress | max(0, distance_after-distance_before)/2.68224; arrival step uses old speed×0.1 |
| First completed merge / passing / arrival | +5 / +2 / +2 |
| Collision | -20 for the involved vehicle |
| Intervention | -0.2 per counted tick |
| Native jerk | -0.01×abs(realized_accel_after-realized_accel_before)/0.1 |
| Completed lane change | -0.1 |
| Feasible-reference error | -0.25×abs(requested_accel-reference_accel)/4.5 |
| Unsafe request | -0.5×max(requested_accel-analytical_ceiling,0)/4.5 |
| Cooperation | +0.1×team merge events |
| Stable merge / classified unnecessary reversal | +2 once / -0.5 per classified event |

Stable success requires the complete eight-second window; arrival before that
window does not manufacture success (`environments/highway_environment_v05.py:612–648`).
There is no explicit headway/TTC reward apart from guards/reference effects;
headway/TTC are stable-window conditions. There is no direct time-to-finish,
stopped-time, boundary-time, background throughput or route-utility reward.
The small cooperation term is not a general fleet-delay objective.

The untrained smooth bundle replaces alignment with
`-abs(u-reference)/4.5`, native jerk with `-0.03 H(j/1 m/s³)`, and adds
`-0.01 H((residual_now-residual_previous)/(0.1 s × 1 m/s³))`, where
`residual=u-reference`, H(x)=x²/2 if |x|≤1 and |x|-1/2 otherwise.
The tail is uncapped. All other terms remain unchanged.
See `environments/highway_smooth_control.py:28–35`, `:125–156`.

Useful scale comparisons, not simulations:

- At approximately constant speed, progress reward per tick is v/26.8224.
  One extra m/s gains only **0.0372823** per tick.
- A 0.2 intervention charge equals the immediate progress reward of
  **5.36448 m/s** extra speed. A smooth alignment error of 1 m/s² costs
  0.222222 per tick, equivalent to **5.96053 m/s** of immediate progress reward.
- A jerk of 2 m/s³ costs 0.02 historically and 0.045 in the smooth bundle;
  jerk 45 costs 0.45 versus 1.335; jerk 65 costs 0.65 versus 1.935.
- On a fixed route, undiscounted distance reward is almost fixed regardless of
  travel time. Discounting and arrival timing favor earlier progress, but merely
  raising progress weight does not create a direct fleet-delay objective.
- Necessary braking must retain priority. These arithmetic comparisons do not
  justify avoiding braking, relaxing native guards, suppressing counted overrides,
  or censoring observed jerk to meet a gate.

The reward is based partly on native realized motion while PPO differentiates
the distribution of requested actions. That is a legitimate model-free setup;
it is also a credit-assignment problem when native ACC/junction logic overrides
requests. An intervention is not automatically a safety violation: the current
definition includes ordinary requested-versus-native differences over 0.05 m/s².
Keep that historical metric intact and separate its causes from unsafe requests.

The residual-slew term has a narrower observability issue. Ego previous requested
acceleration **is** present at obs[5] (`traffic/i110_policy_interface.py:16,135`).
Previous reference acceleration is not a declared actor field; the environment
stores previous residual privately. Thus identical current actor input can have
different residual-slew cost depending on earlier context. The actor is
feedforward. Do not claim the actor lacks its previous request; either remove
this extra memory-dependent reward for the first ablation or explicitly version
the observation/history contract before relying on it.

## Actual optimization evidence

Historical run:
`outputs/highway-environment/v05/train-20261106T024828Z-90b25880`.
The 48 PPO rollout reports contain 245,903 active decisions, 2,025 joint PPO
optimizer steps and 24 early-KL-stopped updates. Critic warmup adds 4,096 decisions
and 68 critic-only steps. All 2,025 actual PPO gradients were clipped; median
clip factor=0.1205569, mean=0.1462623. Median pre-clip actor/critic norms were
3.48464/0.376217. Critic exceeded actor on 360/2,025 steps: claiming the critic
generally dominated this completed run would be wrong.

Across the first accepted minibatch of each PPO update, median component norms
were PPO=3.27462, acceleration imitation=0.0332943, near-limit=0.307916.
The median acceleration/PPO ratio was **1.14053%**. PPO–acceleration cosine was
negative in 15/48 samples. This component evidence is not an average over all
2,025 optimizer steps. Critic median explained variance improved within updates
from 0.193838 to 0.214955; median MSE from 34.1109 to 28.1125. These are modest
fits on different frozen-return distributions, not proof that value error caused
the driving regression.

Historical PPO reward totals / active decision:

| Term | Total | Per decision |
| --- | ---: | ---: |
| Progress | 150811.541465 | +0.613296875 |
| Jerk | -6861.383060 | -0.027902803 |
| Intervention | -9214.600000 | -0.037472499 |
| Feasible acceleration | -8924.409898 | -0.036292399 |
| Unsafe request | -357.883136 | -0.001455383 |
| Lane change | -135.9 | -0.000552657 |
| Merge / passing | +20 each | +0.000081333 each |
| Arrival | +102 | +0.000414798 |
| Stable merge | +2 | +0.000008133 |

This demonstrates sparse successful-event credit in the *training data*. It
does not show that the corresponding event bonus is mathematically incapable
of influencing an individual decision. Reward totals are not gradient norms.

Final v4 frozen diagnostic:
`outputs/highway-smooth-control/v02/gradients-02/summary.json`, matching
`config.json`, `batches.pt`. Both arms contain identical 512 active stochastic
samples; derivatives use only their first 256. Zero optimizer steps and unchanged
parameters/optimizer/gradients/RNG were verified.

| Weighted component | Control loss / gradient norm | Smooth loss / gradient norm |
| --- | ---: | ---: |
| PPO | -0.531660 / 5.564572 actor | -0.186968 / 6.357264 actor |
| Value | 13.687343 / 37.302975 critic | 8.479546 / 23.392658 critic |
| Acceleration imitation | 0.006329 / 0.066850 actor | 0.024581 / 0.053264 actor |
| Entropy | 0.001011 / 0.006769 | same |
| Lane imitation | 0.000540 / 0.003540 | same |
| Near-limit | 0 / 0 | 0 / 0 |

PPO–acceleration cosine is **-0.840824 → -0.955379**; the weighted
acceleration/PPO norm ratio is **1.20135% → 0.83785%**, despite the fivefold
coefficient increase. The changed reference/reward/returns also change the
gradient. No inference that coefficient 5 alone reduced the gradient is valid.
Derived hypothetical global clipping factors are 0.0132598 and 0.0206377;
neither was an executed optimizer step. A lower total loss (13.16356→8.31871)
compares different targets and is not evidence of better driving.

Acceleration imitation is MSE in normalized action units. Baseline coefficient
0.04 corresponds to **0.00378698×MSE(m/s²)** because the physical scale is 3.25.
Smooth 0.04×5 corresponds to 0.0189349×physical MSE. The separately normalized
near-limit subset uses 0.1×physical MSE, about 26.4 times the baseline coefficient
per comparable selected sample. It is zero if that subset is empty, as in this
frozen diagnostic. This helps explain why near-limit behavior can improve while
ordinary following alignment remains poor; it is not causal proof by itself.
See `policies/mappo.py:406–482`, `:623–635`.

The 512 saved requested accelerations average +0.308142 m/s² (population SD
0.602618); 28.3203% are negative. The old teacher labels average +0.622058,
while v4 labels average +0.215912. This is a concrete local shift toward lower
requested acceleration, not a full-route speed comparison. Latent std and
across-state action spread must not be conflated.

## Exposure, ancestry, and interpretation limits

Historical v05 PPO exposure was cruise 20 episodes/105829 decisions, following
18/102607, passing 6/9467, merge 4/28000. Every episode used difficulty 0;
46 were complete and two budget-truncated. Validation used eight cases at each
difficulty 0/1/2. All population/speed/channel values appeared somewhere, but
there was no full factorial coverage and no I-110 merge or passing training
episode. The grade gate was never met. Fresh failed-case retries were 20% of
collected episodes; saved PPO samples were not replayed as fresh experience.
See `experiments/highway_v05_curriculum.py:18–24`, `:126–160`, and the saved
`curriculum.json`, `training-coverage.json`, `update-*.json`.

The new unrun smooth schedule fixes **case shares**, not decision shares;
different populations/lifetimes necessarily change active-decision weighting.
It records actual shortfalls. Increasing an early rollout's controlled count can
change its learning influence even with identical case probability. A future
comparison should keep matched complete case order and report both case and
decision exposure instead of silently truncating every case to a tiny quota.

The older Stage1 v03 run collected 49,998 teacher demonstrations plus 449,992 PPO
decisions, with 1,176 BC and 6,555 PPO optimizer steps, and was also unaccepted.
Its config trained a fresh 93-field actor, used acceleration-weighted BC, then
decayed PPO imitation from 0.05 toward 0.01. The v05 initialization specifically
uses `selected-00036-d97376b9/best.pt`, not that run's final selected generation
`selected-00162-80ce2ebf`. V05 copies old actor features and initializes the six
new motion-feature columns to zero; the new critic is not the old 4-slot critic.
See saved Stage1 `config.json`/`summary.json`, v05 `config.json`, and
`experiments/highway_v05_support.py:60–82`. Historical comparisons across changed
road, lateral-motion and observation semantics are not controlled A/B results.

The v05 learned endpoint failed 19/24 cases versus **23/24**, not 21/24,
initially. Speed rose in cruise/following/passing; merge speed changed only
0.931666563→0.930856885. Policy-filter changes fell 1692→391 while native
realization differences rose 24554→38464. Those overlapping counters must not
be summed as independent events. The corrected-I-110 replay of the same frozen
weights is a map intervention, not a training improvement.

## Recommended controlled stages — proposals only

Do not launch the old two-arm v4 bundle merely because it is implemented. Its
readiness proof is failed. Do not strengthen a teacher that still selects an
unnecessary ending-lane pass, or tune jerk upward to hide a route decision error.

The minimum necessary work is (a) route-useful candidate-lane information and
targets that the deployed actor can actually observe, (b) correctly conditioned
ending-lane braking, (c) all-controlled-car progress/stop acceptance, and (d)
behavior/return checks before resolving a new launch config. Preserve the
existing PPO probability/GAE machinery and frozen comparison actors while doing
that. Larger models, recurrence, value normalization, separate gradient clipping,
higher imitation weights and a new jerk reward are **optional isolated
experiments**, not simultaneous prerequisites or verified fixes. Previous
reference history is needed only if the residual-slew term is retained.

1. **Behavior contract and reference/visibility repair, zero learning.** Define
   route-useful lane choices and achievable longitudinal targets separately from
   immediate safety permission. If the actor cannot see adjacent-lane continuation
   or ending distance, explicitly version the needed current-map fields; do not
   give only the teacher undisclosed map knowledge. Correct the unconditional
   ending-profile feed-forward activation. Keep required braking immediate and
   report uncensored native jerk. On a predeclared train-only fixture bank,
   require every controlled car's progress/completion and prolonged-stop audit,
   not only the focal merge target. Check reward ordering for staying versus
   unnecessary detours, needed versus gratuitous braking, waiting versus feasible
   progress, and whole-route returns. Suggested new preparatory ceiling: 600 s
   runtime, zero updates; failures charged. This work requires its own resolved
   scope/approval before new runs.
2. **Longitudinal learning pilot with one identifiable factor.** Keep lanes fixed
   in explicit cruise/following training cases until validated targets work over
   speeds 12/18/22.352/26.8224 and difficulty 0/1/2. Use a common actor, fresh
   matched critic, common reference and common rewards for both arms. Compare
   baseline acceleration-imitation weight 1 versus 5 **without simultaneously
   changing the target, jerk term, alignment reward or residual-slew term**. Only
   use supervision where the qualified reference defines desired behavior; a
   conservative feasibility ceiling alone is not an equality target. This tests
   the old weight hypothesis cleanly rather than assuming it is correct. If the
   target cannot be qualified, do not launch this comparison. Proposed cap:
   2×30,000 active decisions, each including at most 2,048 critic warmup, and
   3,600 s total including that preparation, shared initial 24 and both final 24;
   reserve 900 s for final evaluation. Caps are not promised achieved samples.
3. **Route-choice/merge learning after longitudinal competence.** Introduce
   route-aware passing and generic merge before I-110, retain foundation cases,
   and guarantee a predeclared block includes every task/difficulty rather than
   waiting indefinitely for one aggregate grade. Still report competency
   separately from exposure. Use one selected longitudinal objective, and compare
   only the route-information/reference treatment. Proposed separate approval:
   2×100,000 decisions, 7,200 s including evaluations/failures and a 1,800 s final
   reserve. Validate all 24 existing rows; add a separately frozen stress bank
   for non-focal ending-lane vehicles and merges with background traffic.
4. **Comfort objective and optimization ablations, then replication.** After
   route choices and braking targets pass, test added Huber jerk alone on the
   same reward baseline; retain the exact no-stop/progress/safety gates. Add
   residual-slew only after resolving its previous-reference state requirement.
   A separate actor/critic clipping or value-normalization ablation is warranted
   by the frozen fresh-critic scale, but the completed run does not justify
   changing it concurrently with the reward. Each comparison needs a predeclared
   bounded budget and fresh on-policy samples; no old rollout reuse. Replicate a
   qualified candidate across at least three training RNG seeds before a claim
   of robustness (suggested later ceiling: six 100k-decision control/treatment
   runs, six hours aggregate, all costs included). This is a later proposal,
   not authorization now.

For every stage: preserve native car-following/safety modes, count all overrides,
retain reset/startup and full 80 s accounting, keep validation out of training,
report all fixed rows plus per-task tails, and compare safety/comfort with
progress, stops, passes, stable merges and all-car completed trips. Use component
losses, gradient directions, actual parameter changes, KL, clipping and critic
fit as diagnostics—not acceptance substitutes. Show the exact new config and
remaining charged budget before asking to launch. A failed stage stops the
sequence; there is no automatic escalation or budget extension.

## Evidence inventory and source identity

All numeric observations above are saved measurements or explicitly labeled
arithmetic derivations. The independent saved-JSON extraction was performed by
the existing `results_audit` collaborator. No new derivatives or forward native
episodes were computed for this audit. Exact current source hashes below were
checked against the v05 run's manifest: `policies/mappo.py`, diagnostics, v05
runner, v05 environment, Stage1 environment and v05 curriculum are byte-identical
to their recorded v05 training sources. This makes line-level objective
inspection applicable to that completed run.

- `policies/mappo.py`: `76a63715ba46b87e023e5e5d0bea7b328d154b2c044e27d086aa446b9f741c5a`
- `policies/mappo_diagnostics.py`: `f4d146964f9879292befa3b8082383cebb0248c62cf290f2b7d404b156ad71fc`
- `experiments/run_highway_environment_v05.py`: `483bb28a6360e6b55e1e56dfcc270499a83f09777cbcbe4157e627d2419383ce`
- `environments/highway_environment_v05.py`: `ef2175752c880ae120da38349da4c2df0138e5e69705997581d45e7ab3703453`
- `environments/highway_stage1_env.py`: `da92aa1ee67a6b229c2462fec6f261eca7b24ce231c70cbb9af8dd7d04112076`
- `experiments/highway_v05_curriculum.py`: `0946d6f8b4dba54810058852e96b10a11fca78099679fc0992449d303982debb`

Primary saved artifacts:

- `outputs/highway-environment/v05/train-20261106T024828Z-90b25880/{config.json,summary.json,initial-validation.json,final-validation.json,curriculum.json,training-coverage.json,update-*.json,last.pt}`.
- `documentation/evidence/highway-smooth-training-plan-v01/objective-scale-audit.json`: includes raw update paths, SHA256s, formulas and derivative-sampling limitations.
- `outputs/highway-stage1/v03/train-20261103T230506Z-04ba1761/{config.json,summary.json}` and its explicitly named selected-00036 checkpoint in the v05 config.
- `outputs/highway-smooth-control/v02/gradients-02/{summary.json,config.json,manifest.json,batches.pt}`: final retained v4 frozen diagnostic, not a training run.
- `configs/training/highway-smooth-v01.json`: disabled retained-v4 config, semantic SHA256 `5d515c15a66c6a9f459356056f2eced7d77d072b6d1d8d0cb466fded27f9b578`.
- `outputs/highway-smooth-control/v02/matrix-02-offline/gates-monotone-bound-revision.json`: retained v4 measured gates. Four comfort failures; six unchanged safety/task passes.
- `outputs/highway-smooth-control/v02/matrix-03-offline/gates.json`: rejected v5 experiment, not the active v4 source. Keep its five comfort failures separate.
- `outputs/highway-smooth-control/v02/final-readiness-failed.json`: failed readiness, no training; source/ledger/evidence hashes retained.
- `documentation/highway-environment-v05-results.md` and `documentation/highway-smooth-preparation-v01.md`: interpretation/history indexes, not substitutes for the raw artifacts above.
