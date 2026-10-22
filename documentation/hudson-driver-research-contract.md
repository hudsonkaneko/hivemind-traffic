# Hudson's driving controller and the communication study

22 October 2026. Codex for Sam. Source: Hudson's message forwarded by Sam,
corroborated by read-only inspection of
[`1fb5d1d`](https://github.com/hudsonkaneko/hivemind-traffic/commit/1fb5d1d)
on `codex/rsl-road-hazard-integration`. No checkout/merge or training was performed.
The local checkpoint was not accessed. Committed performance summaries are
reported evidence, not independently reproduced driving results on Sam's machine.

## Current interface and maturity

The experimental RSL-RL PPO controller takes 42 simulator-state features: 22 ego,
route/task/control values plus two ten-field object slots. Inputs include object
relative position/velocity, dimensions, heading, validity and age. An engineered
obstacle-braking calculation contributes to the target-speed input. This is
simulator-state assisted actuator control; LiDAR perception is not currently the
policy input.

Two normalized actions map to steering within ±0.5 radians and signed
longitudinal control (mutually exclusive throttle/brake within 0–1). Physics runs
at 120 Hz and control at 60 Hz. The configured cruise is 3 m/s, curve radius
500 m and usual driving goal 60 m. Reset is exposed; general destination setting
and a research-level dynamic target-speed/headway interface are not established.

Reported development probes passed clear, static-stop and crossing stages using
successive checkpoints. They are not one frozen candidate's held-out evaluation.
The first full-scene learned-policy transfer departed the lane after 6.659 m;
a separate scripted controller completed 180 m continuously. The backgrounds
in that deployment were scripted and nonreactive. Full-scene training remains
in progress; neither car following nor merging is trained/validated. The pushed
configuration gives actor and critic the same policy observation group; it does
not implement the planned multi-agent centralized-training comparison.

## How it supports the research

The two learning problems have distinct acceptance criteria. The driving layer
must execute lane/speed/spacing objectives reliably within a measured envelope.
The coordination layer tests whether vehicle messages improve outcomes over a
matched local-information controller. Reusing one driving policy across cars can
support this separation, but replicating weights alone does not create reactive
traffic, safe merging or communication.

For Sam's first confirmatory comparison, fix the fleet share and keep the same
base driver, physics/response model, safety logic, traffic requests and learning
budget in both arms. Both learned arms receive matched centralized training
support; only permitted execution messages differ. Multiple fleet shares can be
a later explicitly scoped extension. Prespecify whole-traffic time, human/ramp
outcomes and unfinished/failure guardrails, rather than choosing a favorable
traffic pattern after training.

Define the local information boundary before using the current object slots:
allowed range/visibility, update age, masks and any engineered brake target must
be identical in both arms. Simulator-wide object access or hidden planning
information must not leak into a supposedly local actor. Messages need a defined
payload, range, rate, latency/expiry and missing-message behavior. Adding LiDAR
later is a separate perception change to evaluate consistently across arms.

## Proposed integration gates

1. **Accept one frozen driver in the full scene.** Evaluate new held-out cases
   with speed/curvature/obstacle ranges, lane containment, completion, contacts,
   clearance, timeouts and interventions. Fix the current transfer failure before
   calling the policy a working general driver. Specify what is learned versus
   provided by the engineered braking target.
2. **Expose and test the research command boundary.** Document units, limits,
   reset/route/goal semantics, per-car state isolation and a target-speed/headway
   or equivalent longitudinal interface. Validate following and then merge
   interactions with reactive background cars at the intended study speeds.
   A 3 m/s demonstration does not establish highway-speed competence. Preserve
   one pose owner: SUMO in traffic runs, PhysX in the bounded execution test.
3. **Freeze and compare coordination.** Use the accepted base in matched local
   and communicating conditions. In the approved bounded approach, identify its
   command-response behavior in Isaac, validate a response surrogate independently,
   then feed it into selected SUMO comparisons. An all-physical Isaac fleet would
   need separate evidence for coupled reactive traffic, demand insertion, queue
   boundaries, compute capacity and reproducibility before replacing that approach.

These gates are proposed integration requirements, not new assignments or
permission for unattended training. Merge qualification and the complete-trip
ledger proceed now while Hudson works on driver acceptance. The detailed
[development plan](sam-development-plan.md) and [current accounting evidence](merge-accounting-2026-10-22.md)
remain the handoff for Sam's branch.
