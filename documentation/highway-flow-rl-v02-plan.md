# Proposed flow117 v02: correct the ramp, then improve learned driving

November 8, 2026. **Plan only. No implementation, simulation or training launched.**
This follows the [recorded velocity diagnosis](highway-flow-rl-velocity-analysis.md).
The existing round-two actor and accepted historical baseline remain unchanged.
This proposal is not an extension of the completed two-round training budget.

## 1. Isolate the ramp-junction problem with the frozen actor

Preserve the original network. Inspect the ramp-to-auxiliary connection, priority,
conflict links and sight distance. The 4.5 m minor-link visibility rule strongly
matches the hard braking, but has not yet been isolated experimentally.

Create one versioned candidate with a defensible visibility distance based on
available geometry. Document any synthetic-fixture assumption; the illustrative
road model is not proof of real I-110 sight distance. Change topology only if
the intended lane continuation and connection semantics require it, as a
separate candidate. Do not remove right-of-way or collision protection.

Compare original and candidate using the same frozen actor, seed 6900019, demand,
speed limits and full route. Then check a clear approach and an occupied/conflicting
approach. Keep native speed mode 31 and lane-change mode 512. Save requested,
filtered and realized acceleration, gap/TTC, entry progress and all interventions.

Exit: the empty-approach brake is explained and reduced without unsafe entry in
the conflict case. A successful short comparison is development evidence, not a
general safety claim. If visibility does not isolate the cause, retain the result
and inspect native junction behavior before altering policy rewards.

## 2. Address optional detours into ending lanes

Use the existing route-continuation, runway and lane-context inputs first; retain
the 117-input ABI unless a documented information gap requires a new version.
Train on through-lane cruising beside ending auxiliary lanes, necessary ramp
merging, valid passing and required route changes. Penalize avoidable detours
that sacrifice route progress without a passing or route benefit.

Audit route-aware action masks independently of the reward change. Exclude only
options that are incompatible with the required route or lack sufficient complete
maneuver/recovery runway. Do not ban every ending lane or block a required exit.
Version any changed mask semantics. Preserve observations used for masks and log
the allowed actions. Report mask statistics separately from the existing executed
action intervention rate; do not redefine that denominator to manufacture a pass.

Exit: the known sb3_1 -> sb3_0 -> sb3_1 detour is addressed, while required merges,
route changes and useful passing remain possible in explicit counterexamples.

## 3. Improve cruise speed and comfort with separate reward tests

Keep native safety active. Add a bounded legal-speed tracking term in demonstrably
unconstrained cruising, alongside progress and route completion. Continue rewarding
measured comfort across approach, lane change and recovery, rather than only the
eight-second merge window. Do not penalize every slowdown: yielding, safe following,
lower limits and emergency braking remain necessary behaviors.

The current starting weights are progress 1.0, jerk 0.03, intervention 0.2,
unsafe request 0.5, lane change 0.1; merge 5.0, passing 2.0 and arrival 2.0.
No new numerical weights are declared final here. Before training, specify the
new terms, normalization, conditions and coefficients, inspect their values on
saved examples, and present the resolved configuration. Avoid another blind
increase in jerk weight: the previous increase did not improve every case.

Distinguish the global 60 mph ceiling, local legal limits and training speed caps.
The diagnosed episode uses 40.3 mph highway / 30 mph ramp limits. Introduce explicit
40/50/60 mph highway cases only where the scenario supports them; do not raise
every ramp or connector to 60 mph. Vary traffic density and connected-AV participation
within the existing supported population and 24-slot bounds. Include isolated AVs
and missing/delayed messages, without claiming support for unlimited fleets.

## 4. Validate the full drive before and after learning

Use a small declared development set: clear merge, occupied merge, slow leader/pass,
ending-lane trap, free-flow cruise and dense traffic. Pair comparisons on identical
seeds/demand and preserve separate held-out cases for final acceptance. Test network,
mask and reward changes separately so their effects remain attributable.

Retain existing safety and comfort criteria, including the 5% intervention check
and nominal 5 m/s3 peak-jerk check. Report each case and each controlled car,
including uncensored braking/jerk maxima, complete-route completion/progress,
speed tracking, headway/TTC and unnecessary lane changes. Do not hide a failure
inside a fleet average or score only the initial stable-merge window. Set scenario
horizons to cover the complete route or explicitly record unfinished trips.

Save one full focal-route replay using the SimReady main car and lightweight
backgrounds after CPU checks. Include the post-25-second behavior missed by the
previous clip. The render remains visualization, not physical/sensor validation.

## 5. Present configuration, then run a bounded quiet pilot if approved

Before launch, show the exact network/mask version, actor initialization/hash,
observation/action contract, reward coefficients, scenario mix, speed settings,
optimizer, seeds, budget and acceptance tests. If the ABI remains compatible,
use the current round-two checkpoint as a candidate initializer; reset/declare
optimizer handling when objectives or action-mask semantics change.

Proposed budget, subject to that configuration review:

- Initial network screen: at most four short CPU episodes and 180 seconds total.
- Additional diagnostic development runs: at most eight episodes and another
  180 seconds; report unresolved findings instead of expanding silently.
- Initial learning round: 30,000 active-agent decisions, 15-minute wall cap
  including its development evaluation. This is a bounded pilot, not a claim
  that 30,000 decisions will solve the behavior.
- At most one further 30,000-decision / 15-minute round if terminal evidence
  identifies a specific revision and that conditional restart is approved.
- Reserve up to 10 CPU minutes for final evaluation, with seeds frozen before
  learning. If pilot development gates fail, do not spend held-out cases selecting
  a new candidate. No automatic third round or open-ended tuning.

Training runs headlessly in SUMO. Scripts save checkpoints and metrics and signal
completion/failure; no language-model monitoring or repeated log reading during
learning. Review once at termination, fix evidenced errors, and resume/restart only
within the agreed budget. Use agents for bounded preparation/review, not training
watchers. Preserve failed attempts and parameter/configuration versions.

Implementation and launch require the next task instruction. Google engineering
guide synchronization of this planning milestone remains pending.
