# RSL-RL road hazard integration

Updated: 2026-10-22. Status: implementation and validation in progress.

The requested milestone is a trained native PhysX car that can continue along
the road, stop for a randomly entering object, and resume after it clears. The
first deployment target is the existing full highway showcase: one physical
ego among scripted background traffic. Multiple physical learned cars and RTX
perception require their own gates; neither is implied by a one-car pass.

## Version history

### v02 — resume after the PPO/evaluation tensor-context failure

Run `v01-train-20261022T202419Z-7322db03` passed all 10 clear-road development
cases after 2,000 imitation updates. RSL-RL then completed 16 PPO updates and
saved `checkpoints/clear-0016.pt` plus `policy-clear-0016.pt` (32,768 transitions).
The switch into its next evaluation failed because the task's action tensor had
been allocated in Torch inference mode, while reset tried to clear it outside
that mode. The supervisor correctly marked the run incomplete despite the
runtime's exit0 behavior. The original package is retained.

The new task's reset and step boundaries now consistently use inference mode;
the original frozen lane task remains unchanged. Version2 restores the complete
actor, critic and PPO optimizer state at update16, with no repeated imitation
training. Its remaining maximum is 1,015,808 transitions, preserving the initial
1,048,576-transition lifetime cap. The postponed clear-road probe runs first.
Every promotion now also requires valid independent evidence for all ten cases.
This is an implementation repair, not an extension of the training budget.

### v01 — measured-object task and shared policy boundary

This version preserves the previous lane task, its immutable experiments, and
the v09 showcase. New modules share the same observation/action contract in
training and deployment. Training uses a 100 m road segment with the full
showcase's 500 m radius, 120 Hz native physics and 60 Hz controls. The policy
controls steering and signed throttle/brake; only PhysX moves the ego.

The 42 privileged observations contain the existing 22 measured ego/known-map
values and two masked current-object slots. Each slot contains validity,
chassis-relative displacement and velocity, size, relative heading, and age.
Future activation times, seeds, identities and intended trajectories are not
actor inputs. Missing/stale telemetry is a fault, not evidence of an empty road.
The target-speed observation includes an engineered braking target computed
from current object geometry and velocity. The neural policy learns actuator
control using that assistance; this is not end-to-end learned perception or
planning. Feature scaling is fixed in physical units and embedded in the export.
The first object is a fixed 1 m cube with native collision; it starts outside
the lane and enters continuously. No active collider is teleported into a car.

RSL-RL 5.0.0 PPO will use a clear-road, static-stop, crossing-and-resume
curriculum. The initial maximum is 512 updates of 2,048 transitions (1,048,576
total), allocated 64/128/320 updates across the stages. Promotion requires
separate development probes, at least 9/10 successes and zero contacts or lane
departures. A stage that exhausts its budget without passing is retained as a
failed version and revised explicitly. Checkpoints are never overwritten by
later attempts. A scripted controller may supply an explicitly recorded
imitation warm start if the existing policy fails to learn meaningful progress;
its contribution must be reported separately from PPO optimization.

Rewards combine signed progress, time-scaled tracking/control costs, a current
object-based safe speed target, success bonus and physical-failure penalty.
Parking at the start cannot satisfy either a route or obstacle-stop goal.
Clear and crossing trials finish by passing 60 m at driving speed, allowing
continuous full-loop deployment; static trials require a measured stop and
one-second hold before the object. Crossing trials require progress after the
object clears. Optional emergency
interventions must be recorded and evaluated separately from the unshielded
policy; successful shielding is not proof of learned avoidance.

Development seeds are 31001–31010; final evaluation seeds are 41001–41020.
Clear, static, left-crossing and right-crossing cases compare the frozen policy
and unchanged scripted hazard baseline. Feasibility is declared before rollout
using a conservative braking envelope. Impossible late entries are retained in
a separate diagnostic set. Outcomes include success, contacts, lane departures,
timeouts, progress, clearance, post-clear waiting, clipping and interventions.

Evidence is written to unique `outputs/rsl_integration/` run directories and
runtime logs to `logs/rsl_integration/`. Source/configuration hashes, seeds,
runtime versions, pre-reset terminal outcomes, exit and cleanup results identify
each run. Compact results and actual screenshots will be added here after the
native gates pass. Raw policies and logs are local ignored artifacts.

## Prior policy baseline

The earlier `lane-follow-rsl-v1-20261022T185540Z-05fa4bbe` run completed all
128 updates/262,144 transitions and exported `policy.pt`. Its logs show native
context cleanup, but did not retain a process exit-code manifest or driving
outcomes. The rolling reward ended at -12.83 versus -10.70 initially. Completion
of optimization therefore does not establish useful driving. A separate frozen
policy evaluation is being added before reuse decisions. Audit evidence:
`outputs/rsl_integration/review/rsl-v1-completed-run-audit.json`.

## Current limitations

The isolated hazard curriculum has passed, but full-sim policy acceptance has
not passed: the first transfer attempt left the lane. This is
simulator-state control at 3 m/s, not sensor autonomy, highway-speed validation,
SUMO transfer, multi-agent training, or real-world safety certification. The
Google engineering guide has not been synchronized for this milestone.

## v01 initial native checks

Run `v01-probe-20261022T201030Z-be025799` passed four scripted native cases:
clear road (60.040 m), static stop (22.850 m), left crossing (60.030 m), and
right crossing (60.026 m). All had zero contacts. The composed scene checks,
contact-setting restoration, process exit0 and no remaining Kit process passed;
source hashes were unchanged. This establishes task plumbing and a baseline,
not a trained-policy result. The plain OpenUSD/native Python scene and inference
tests also passed all 32 cases, including real TorchScript load/normalization.

The deliberately non-braking static-object control
`v01-probe-20261022T201947Z-f680a73d` produced a native rigid contact at 26.506 m,
terminated promptly and passed independent evidence checks. The obstacle is
therefore physically collidable rather than merely visible geometry.

Collector `v01-collect-20261022T201215Z-e1b7d509` produced 40 independently
qualified successes (10 clear, 10 static, 20 crossing), zero contacts and 50,669
paired native observation/action samples. Every saved demonstration pair was
checked against raw pre-action observations and actions. The separate original
RSL policy evaluation `20261022T201631Z-8dc8d6ee` completed 15 valid cases with
15 timeouts and zero successes. See the compact
[baseline evidence](evidence/rsl-road-hazard-v01-baseline-20261022.json).

The fresh training config uses a 2,000-gradient-step imitation initialization,
then RSL-RL PPO with 128/64 tanh networks, fixed feature scales, signed physical
actions, learning rate 0.00003 and initial action standard deviation 0.06.
Development checks run every 16 PPO updates; a passing stage promotes early,
while the stated stage budgets remain hard upper bounds. Imitation and PPO
optimizer calls are counted separately. Training has not yet passed its gates.

## v02 — resumed training and an unsuccessful full-scene transfer

The v01 attempt completed 2,000 imitation updates and 16 PPO updates before
evaluation reset hit a Torch inference-tensor mutation error. Its checkpoint,
logs and failed result remain intact. The reset/step boundary now uses the same
inference context as RSL; the regression reproduces the old failure and passes
with real CPU Torch. The interrupted run's result had a stale zero optimizer
counter; its separate counter file correctly records 1,280 PPO optimizer calls.
See [the interruption audit](evidence/rsl-road-hazard-v01-training-20261022.json).

`v02-train-20261022T203726Z-61afe5d7` restored the full checkpoint and optimizer,
without repeating imitation. Its clear/static/crossing development gates each
passed 10/10, with clean process exit, source hashes and native cleanup. Across
both attempts there were 48 PPO updates, 98,304 transitions and 3,840 PPO
optimizer calls, plus the original 2,000 imitation updates. Early promotion
completed the curriculum in less than its maximum budget; this does not mean
that the remaining budget was executed.

The frozen candidate is
`outputs/rsl_integration/v02-train-20261022T203726Z-61afe5d7/policy.pt`, SHA256
`f2e51984bce3a13d43e20f146b4b00f5d406f6ce1aa48167bb384773317123a5`.
Full-scene trial `v01-20261022T204641Z-101d29f0` completed cleanly but failed
driving: lane departure after 6.659 m, zero contacts. Twelve background cars
populate object slots and sizes absent from the isolated demonstrations. The
observed failure is evidence of a transfer problem; its exact causal weight
has not been isolated by an ablation. Do not deploy this candidate as accepted.

The scripted continuous baseline `v01-20261022T204758Z-f4202c5e` passed 180 m
on the complete highway through three task windows, using one physical reset.
This validates a deployment mechanism, not learned continuous driving. The
12 background cars remain scripted and nonreactive.

## v03 — full-scene training in progress

The next configuration uses the same full-highway/native-background observation
context for demonstrations, training and deployment. It starts a fresh network
with a separately qualified imitation dataset; it does not silently mask away
background vehicles to recover the old result. The new PPO cap is 950,272
transitions, leaving cumulative experiment consumption within the original
1,048,576 cap after v01/v02's 98,304 transitions. Budgets are 48/96/320 updates
for clear/static/crossing, with the same development promotion rule. Final
held-out seeds have not been used for policy selection.
