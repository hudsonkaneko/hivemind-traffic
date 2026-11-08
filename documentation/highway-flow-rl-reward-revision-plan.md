# Flow117 reward revision plan: steady speed and useful lane choices

November 8, 2026. **Proposal only: no policy, runnable configuration or reward
implementation changed; no simulator or training launched.** This revises the
recommended reward design for the prepared v02 pilot, not its historical results.
Two sub-agents researched primary sources and audited the actual reward math.

## What the current evidence says

The latest replay still uses frozen v01 round-two weights. V02's new rewards
have never trained an actor. Its clear-approach visibility improvement remains
separate from the remaining learned speed/lane problems.

- At 6.6-10.2 s, agent_0 brakes from 34.44 to 20.76 mph entirely by actor request;
  neither the explicit filter nor SUMO changes these acceleration commands.
- At 17.4-21.1 s, the actor again requests the slowdown, while its leader opens
  the gap. The optional ending-lane move begins at 19.6 s; its lane ID changes
  at 21.1 s. At 26.4-32 s it brakes to a stop in that ending lane, with additional
  lane-end protection near the end. A command trace identifies who brakes, not
  whether every merge-related slowdown is unnecessary.
- The proposed v02 cruise bonus pays **zero in both initial dip windows**.
  The first is excluded by auxiliary/route/entry conditions. During nine journal
  rows at 18.8-19.6 s, only the car's own braking excludes it, despite a leader
  pulling away and a positive feasible acceleration estimate.
- Across 545 focal decisions, progress contributes +213.618, intervention -23.4,
  jerk -12.224, cruise +1.935, and detour -0.5. Only 69 decisions receive cruise
  reward. These are evaluation-time component sums, not training-gradient shares.
- Progress is distance/2.68224 each 0.1 s. Increasing that weight alone mostly
  increases the value of distance already rewarded. For a fixed distance its
  undiscounted sum is nearly fixed; discounting and termination still matter.
- Current comfort is -0.03 times absolute physical jerk. Constant braking costs
  almost no jerk after acceleration settles. For a monotonic change of
  acceleration, the summed absolute-jerk cost at fixed cadence depends on total
  acceleration change, rather than how gradually that change is spread out.

[Saved speed comparison](highway-flow-rl-v02.md#november-8---speed-comparison-with-adjacent-native-traffic).
[Reward audit and source hashes](evidence/highway-flow-rl-v02/reward-plan-audit.json).

## Research and what we apply

[SECRM](https://journals.sagepub.com/doi/full/10.1177/03611981231171899)
combines a dynamic target speed and normalized comfort objective while enforcing
an independent acceleration safety constraint. Apply that separation; its
car-following proof and numerical weights do not validate our multi-lane system.
Our Huber comfort proposal below is an engineering choice, not SECRM's squared
comfort formula.

[HighwayEnv's merge implementation](https://highway-env.farama.org/_modules/highway_env/envs/merge_env/)
combines speed, merging-traffic effects and lane-change costs. Apply the principle
of measuring traffic outcomes, rather than rewarding maneuver counts alone.
[MetaDrive](https://metadrive-simulator.readthedocs.io/en/latest/reward_cost_done.html)
separates progress/speed and terminal rewards. Use these as design precedents,
not coefficients to copy between simulators with different units and cadence.

The audit also checked two hypotheses. Merge/pass bonuses are already one-time
per agent, so repeated merge farming is not an established bug here. Normalized
action maps as a=3.25u-1.25: u=0 means braking, while coasting is u=0.384615.
This is asymmetric but not evidence that it caused the observed learned actions.
Preserve that ABI for the warm-start pilot; do not reinterpret frozen weights.

## Proposed starting weights at 10 Hz

These are candidates for a saved-state scale audit, not validated optima.

| Term | Current/prepared | Proposed |
| --- | --- | --- |
| Progress | +1.0 times distance/2.68224 | Unchanged |
| Cruise | +0.1 times (v/legal)^2, with restrictive exclusions | Replace with speed-target error below; coefficient 0.30 |
| Comfort | -0.03 times absolute jerk | -0.09 H(j/2), using physical jerk and unit-threshold Huber |
| Optional through-to-ending-lane detour | -0.5 once at first observed motion | -2.0 once, with existing route/passing exceptions |
| General lane change / unnecessary reversal | -0.1 / -0.5 per existing event | Unchanged; avoid suppressing necessary merges and useful passing |
| Merge / stable merge / arrival / passing | +5 / +2 / +2 / +2 once | Unchanged, with existing eligibility |
| Collision / intervention / unsafe request | -20 / -0.2 / existing -0.5 normalized excess | Unchanged; native guards and analytical shield stay active |
| Cooperation | +0.1 times existing team-merge fraction | Unchanged |

### Speed target and eligibility come before tuning its coefficient

For a valid pre-action target v_ref, use post-action measured speed v_next:

`r_speed = -0.30 * min(1, max(0, abs(v_next - v_ref) - 0.5) / max(v_ref, 5))`.

Units are m/s; the 0.5 m/s deadband is approximately 1.1 mph. Maximum cost is
0.30 per 0.1-second decision. This replaces the old cruise addition, rather than
stacking two speed objectives. At v_ref=18 and v_next=13, it costs 0.075;
at v_next=9.3, approximately 0.137; within the deadband it costs zero.

Construct and log v_ref **before** observing the requested action:

1. Clear cruising uses the effective legal target: global 26.8224 m/s ceiling,
   executed scenario/current-lane limits and upcoming lower limits all apply.
2. Following uses a lower target when current leader motion, gaps and the existing
   stopping-distance constraints require it. A sufficiently separated opening
   leader does not automatically disqualify speed recovery at a 3-second cutoff.
3. Ramp/auxiliary/merge context is evaluated using current front/rear gaps,
   relative speeds, the existing maneuver horizon, route runway and right of way.
   Align with an admissible traffic gap rather than chasing the adjacent car's
   speed regardless of its position. A known entry or auxiliary-lane label alone
   is not evidence that acceleration is unsafe.
4. Unknown/conflicting evidence or no defensible merge target means abstain and
   log the reason. Necessary slowing for a yield, obstacle or lane end lowers
   the target where it can be established, otherwise the term abstains.

The precise target helper and gap-bound equations must be frozen and tested
before producing a runnable configuration. Reuse verified current-state/map
constraints; do not revive the failed full scripted planner, infer future motion
from replay, or introduce a teacher action/loss. A target is reward supervision,
not an instruction sent to the car or a new actor input. Its construction is the
main implementation risk in this plan; it is not already a qualified controller.

Do not switch this term off merely because the policy brakes, or because the
post-step realized acceleration is negative. Otherwise the actor can remove its
speed objective through its own action. Required braking must remain preferable
to forcing speed in occupied/conflicting cases. Test continuity across corridor
changes; freeze the chosen target for each transition, not indefinitely across
new hazards. Report eligibility/abstention independently of actor behavior.

### Comfort with explicit units

Use j=(a_next-a_previous)/0.1 in m/s3, and
H(x)=0.5*x^2 for |x|<=1, otherwise |x|-0.5.
The proposed penalty is -0.09 H(j/2). Its magnitude is 0.01125*j^2 up to
|j|=2, then 0.045*|j|-0.045. These coefficients have different normalization
from the old 0.03 and must not be compared as bare numbers.

| Absolute jerk (m/s3) | Old penalty magnitude | Proposed magnitude |
| --- | ---: | ---: |
| 1 | 0.030 | 0.01125 |
| 3 | 0.090 | 0.090 |
| 5 | 0.150 | 0.180 |
| 10 | 0.300 | 0.405 |

Gradual acceleration becomes cheaper, while abrupt changes above 3 m/s3 cost
more. Do not add a general penalty on acceleration or negative acceleration.
Do not impose a new execution filter that hides poor actor output. Keep measured
jerk uncapped, including forced/emergency braking; log requested, filtered and
realized action separately. Safety takes precedence over comfort, and all raw
comfort failures remain in evaluation.

## Implementation and evaluation sequence, after plan review

1. **Offline reward audit first.** Use saved train/development states and
   explicit clear-road, opening-leader, occupied-merge, necessary-brake, lower-limit
   and ending-lane fixtures. Compare old/proposed terms individually and combined,
   with component sums and gate coverage. Check numerical candidate responses
   without treating them as simulated rollouts. Audit target continuity and
   collision/early-termination incentives. If abstention still hides the known
   dips, resolve the information/target problem before increasing the weight.
2. **Version and preflight.** Add a separate reward version/configuration after
   the audit, preserving v02 history. Keep 117 actor inputs, acceleration/lane
   intent, native modes 31/512, masks and 60 mph/lower local limits. Existing
   contracts pin the old weights, so explicit contract/schema changes and tests
   are required. Show the resolved configuration before launching learning.
3. **One bounded pilot.** Warm-start the same frozen round-two checkpoint,
   fresh optimizer, actor LR 1e-4, critic LR 3e-4, PPO clip 0.2, four epochs,
   batch 256, KL 0.02, entropy 0.001, gamma 0.99 and GAE lambda 0.95.
   Keep the existing 12-row curriculum, 2-5-lane diversity, explicit speed caps,
   controlled counts/communications and split seeds. Verify all scheduled rows
   are reached and retain complete I-110 focal episodes. Cap at 30,000 active-agent
   decisions / 900 seconds including development evaluation. This is a screening
   pilot, not assurance of convergence. No grid of extra training runs.
4. **Matched behavioral comparison.** Use the frozen baseline, revised actor
   and native control of the same focal spawn/route/demand on declared development
   cases. The adjacent native car is still useful visually, but its different
   starting state is not the controlled baseline. Score the entire drive and
   every controlled car; preserve separate held-out seeds for one final check
   only if development passes. Retain actual passes/merges/arrivals, not reward
   alone. A combined pilot cannot isolate the contribution of each new term.

Retain existing gates: zero collisions/teleports/overspeed, per-car interventions
<=5%, uncensored peak jerk <=5 m/s3, minimum measured leader headway >=1.5 s,
closing TTC >=3 s and full-route completion. Preserve necessary braking even
when its comfort metric fails; do not hide the failure or relax protection.
Also require no avoidable ending-lane detours/stops, no regression in required
merges/passes, at least 90% legal speed in free-cruise evaluation, and report
time to 90% of the valid target plus mean/p95 jerk against matched native control.
Report actual numbers without claiming equality to native before measuring it.

Use the existing v02 total ceiling of 2,760 seconds; do not reset its 115.734 s
already spent. The last preparation simulation slot is reserved for the fresh
zero-update preflight. Proposed allocation: up to 120 s of the existing 600 s
evaluation reserve for matched native comparisons, leaving 480 s for final
evaluation. This is an explicit reserve reallocation proposal, not extra budget.
Remaining preparation work uses offline data/unit checks. A second 30,000/900 s
round remains conditional and unauthorized; review terminal results and propose
one evidenced change rather than automatically cycling weights.

Training, if subsequently approved, runs headlessly with script-managed
checkpointing. No language-model polling or agent monitoring while it runs;
review only completion or failure. Keep source/config hashes, raw per-car metrics
and reward components. Render one full-route SimReady replay after terminal
evaluation, not repeated videos during training.

Next: review this proposal, then implement the offline audit and resolved
configuration. No training authorization is inferred from this plan. Google
engineering-guide synchronization remains pending; its generated mirror was
not edited.
