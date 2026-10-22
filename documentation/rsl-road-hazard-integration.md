# RSL-RL road hazard integration

Updated: 2026-10-22. Status: implementation and validation in progress.

The requested milestone is a trained native PhysX car that can continue along
the road, stop for a randomly entering object, and resume after it clears. The
first deployment target is the existing full highway showcase: one physical
ego among scripted background traffic. Multiple physical learned cars and RTX
perception require their own gates; neither is implied by a one-car pass.

## Version history

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

No hazard-trained policy or full-sim acceptance result is claimed yet. This is
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
