# Highway reward revision v03: smooth speed recovery

Implementation follow-up: [v03 code, saved-trace results and final configuration](highway-flow-rl-v03.md). The proposal below is retained as the pre-implementation record. Training remains disabled.

November 9, 2026, America/Los_Angeles. **Proposal for review only.** No reward
implementation, executable training configuration, actor, simulator run or
training authorization changes in this milestone. This is a version of the
plan, not a trained v03 policy. It refines the [November 8 proposal](highway-flow-rl-reward-revision-plan.md).

## Starting point and diagnosis

Warm-start candidate: frozen Highway Flow RL v01 round-two actor/critic
(`888a380b7c6741aea623e9b99ad155f7cb9a2ef8fdc2e590823b7e2629031819`
actor SHA-256), with a fresh optimizer. It remains experimental. The prepared
v02 environment is not a trained v02 actor. The accepted straight-road v04r2
actor and its clipped -0.02 jerk term are a different experiment and contract.

The actual highway actor was trained with uncapped `-0.03 * abs(jerk)`.
Constant deceleration costs no jerk after acceleration settles. For a monotonic
acceleration change, summed absolute jerk at fixed cadence is independent of
how many steps spread that change. Raising its coefficient alone does not
establish a preference for a steady, appropriate cruising speed.

The [saved audit](evidence/highway-flow-rl-v02/reward-plan-audit.json) finds
actor-requested braking in the 6.6-10.2 s and 17.4-21.1 s windows. The first
reduces speed from 34.44 to 20.76 mph without command overrides. V02's proposed
cruise addition pays zero in both windows; some rows exclude themselves solely
because the actor brakes. Optional ending-lane entry later causes further
braking/stopping. These observations identify failure modes, not a controlled
proof of reward causation. The visibility correction and its synthetic sight
assumption remain separately documented in [v02](highway-flow-rl-v02.md).

## Proposed weights at 10 Hz

Values below are signed reward contributions or coefficients, not all literal
configuration signs: the existing highway implementation negates cost features
before multiplying positive configuration weights. Freeze one convention in the
new schema and test against double negation.

| Term | Proposed contribution | Change |
| --- | --- | --- |
| Progress | +1.0 * distance_m / 2.68224 | Preserve |
| Valid safe-target-speed error | -0.30 * E | New trained objective; replace untrained v02 cruise bonus |
| Comfort | -0.09 * H(jerk / 2) | Replace -0.03 * abs(jerk) |
| Avoidable ending-lane detour | -2.0 once at first lateral motion | V01 has none; prepared v02 uses -0.5 |
| Applied lane change | -0.1 per existing event | Preserve |
| Unnecessary lane reversal | -0.5 per existing event | Preserve |
| Merge / stable merge / pass / arrival | +5 / +2 / +2 / +2 under existing one-time eligibility | Preserve |
| Collision / intervention | -20 / -0.2 under existing indicators | Preserve |
| Unsafe acceleration request | -0.5 * max(0, request - safe_ceiling) / 4.5 | Preserve |
| Cooperation | +0.1 * existing team-merge fraction | Preserve |

Here `j = (a_next - a_previous) / 0.1` is measured longitudinal jerk in m/s3.
`H(x) = 0.5*x*x` for abs(x) <= 1, otherwise `abs(x)-0.5`. No jerk clipping:

| Absolute jerk, m/s3 | Current penalty magnitude | Proposed magnitude |
| --- | ---: | ---: |
| 1 | 0.030 | 0.01125 |
| 3 | 0.090 | 0.090 |
| 5 | 0.150 | 0.180 |
| 10 | 0.300 | 0.405 |

The candidate makes gentle changes cheaper and sharper changes above 3 m/s3
more costly. Its linear tail does not guarantee that every large-jerk split is
strictly preferred; validate sequence-level behavior. Do not add a general
braking/acceleration penalty or a new output smoothing filter. Necessary braking
must remain available, and raw comfort failures remain visible.

For post-step speed v_next and a valid reference v_ref established **before**
the action, `E = min(1, max(0, abs(v_next-v_ref)-0.5) / max(v_ref,5))`.
All speeds are m/s. The deadband is 0.5 m/s (about 1.1 mph). At v_ref=18 and
v_next=13 the penalty is -0.075; at v_next=9.3 it is about -0.137. These are
candidate scales, not evidence of training effectiveness.

## Qualify the speed target before training

1. Clear road: use the effective legal cruise target, including upcoming lower
   limits and the 60 mph global ceiling. A 40 mph scenario is not a 60 mph target.
2. Following and merging: lower the target using current leader motion,
   conservative stopping bounds, front/rear target-lane gaps, route runway and
   right of way. Both speed recovery and necessary yielding must be represented.
3. Compute eligibility from pre-action context. Braking itself cannot disable
   the speed objective; ramp/auxiliary labels alone cannot disqualify clear road.
   Never read future traffic state to construct the target.
4. Unknown or contradictory context: abstain with a specific logged reason.
   Report coverage so abstention cannot hide the known dips or all difficult
   states. Test target continuity at merge entry/exit and lane transitions.
5. Keep the 117-input actor/2,832-input critic interface, action mapping, native
   modes 31/512, lane masks and analytical safety shield. The target is reward
   supervision, not a teacher action, new actor input or replacement controller.

Exact gap-to-target equations remain an implementation deliverable. Do not
launch learning until fixtures establish that they produce defensible targets.

## Work sequence and decision gates

1. **Offline qualification:** use saved development traces and synthetic clear
   road, opening leader, occupied merge, necessary braking, lower-limit and
   ending-lane cases. Compare reward totals for smooth recovery versus needless
   braking; verify necessary braking remains preferable to unsafe acceleration.
   Exercise early termination, target abstention, jerk units, one-time detours
   and the existing merge/pass guards. Offline counterfactual calculations are
   not new closed-loop performance evidence. Do not inspect held-out data.
2. **Separate implementation/configuration:** version the target helper and
   reward schema; add boundary/regression tests and refresh the zero-update
   preflight. Log each component, target, eligibility, and requested/filtered/
   realized acceleration. Show the resolved configuration and audit results to
   the user before any learning. Training remains disabled until authorized.
3. **One bounded pilot after approval:** same frozen round-two initialization,
   fresh optimizer, no imitation or extra critic warmup; MAPPO actor LR 1e-4,
   critic LR 3e-4, clip 0.2, four epochs, minibatch 256, KL 0.02, entropy 0.001,
   gamma 0.99 and GAE 0.95. Maximum 30,000 active-car decisions and 900 seconds
   including development evaluation. Retain the 12-row, 2-5-lane curriculum,
   speed limits, AV counts and communication conditions; verify coverage and
   complete focal I-110 episodes. This is a screening pilot, not a convergence
   claim. No automatic additional rounds.
4. **Matched evaluation:** frozen parent, revised actor and native control of
   the same focal spawn/route/demand. Review whole routes and every controlled
   car. Use validation to select the checkpoint; only consume held-out seeds
   after development passes. A combined pilot cannot identify which term caused
   a gain. Keep communication-benefit claims outside this reward pilot.

Retain zero collisions/teleports/overspeed, full-route completion, per-car
interventions <=5%, uncensored peak jerk <=5 m/s3, measured leader headway >=1.5 s
and closing TTC >=3 s. Necessary emergency braking remains enabled even when
its jerk makes a case fail. Require at least 90% of legal speed in qualified
free-cruise cases, no avoidable ending-lane stops/detours, and no regression in
required merges or passing. Report time to 90% of valid target speed and
mean/p95/peak jerk against the matched native baseline.

Add a proposed avoidable-braking diagnostic: at least three consecutive 0.1 s
steps with requested acceleration below -0.2 m/s2 while a qualified pre-action
target exceeds current speed by >=0.5 m/s and the safe acceleration ceiling is
>=+0.2 m/s2, with no yield/obstacle/lane-end obligation. Count events and later
reacceleration, with raw traces. Require zero such events in explicitly clear
test fixtures; validate this classifier before using it as a gate. Unknown
target coverage must not produce a false pass.

## Usage, evidence and next action

Preserve the existing v02 study ceiling of 2,760 seconds and its 115.734 seconds
already charged, verified from cost-ledger.json on November 9. Do not reset
the ledger. The previous 120-second native-comparison allocation proposal comes
from the existing 600-second final reserve, leaving 480 seconds, not extra time.
The second 30,000-decision round remains separately unauthorized. Checkpoint and
record metrics automatically; inspect only completion or failure while learning
runs. Keep failures, per-car/component metrics, exact code/configuration hashes
and one final full-route replay. No repeated renders or language-model polling.

Next action: user review of these starting weights, followed by offline target/
reward qualification and a resolved configuration. This planning request does
not launch training. The living roadmap is v35. Google guide incremental sync
remains pending; this milestone does not edit its generated mirror or claim
a fresh native-document synchronization.
