# Apply GPUDrive findings to MAPPO — plan v01

**November8 update:** the user authorized revising this plan and training with
terminal review/restart. A separately versioned [teacher-free MAPPO pilot](highway-flow-rl-v01.md)
completed two rounds without changing the original failed imitation-study gates.
It removes all teacher losses/reward alignment and uses native safety/runtime
preflight instead of claiming the teacher is qualified. The original plan below
remains historical evidence; its teacher-assisted launch path is still blocked.

November 7, 2026. **Planning only: no runtime, weights or training configuration
changed; no simulation or training launched.** This supplements the
[living roadmap](sensor-driven-training-roadmap.md) and
[flow preparation](highway-flow-preparation-v01.md), preserving their failed
experiments, acceptance criteria and budget ledger.

## Starting point and evidence

Use the prepared `HighwayFlowEnv`/flow117 path, not a fresh GPUDrive port or a
rollback to the old Stage1-v03 actor. Stage1-v03 has 93 inputs; the rejected
environment-v05 actor has 99; flow117 prepares 69 local fields plus six received
eight-field V2V slots. Its training-only critic has 24 masked slots/2,832 inputs.
There is **no trained flow117 policy or selected initializer yet**. Existing
explicit 93/99/117 migration and frozen-inference checks must govern loading.

[GPUDrive v05](gpudrive-i110-v05.md) reached the destination with smoother
longitudinal acceleration, but crossed onto an unconnected upstream road about
33 m before the ramp ended. Its comfort controller changed 99.29% of raw actor
commands. Zero extra guard corrections after filtering did not establish learned
driving competence. Transfer these audit and control lessons, not its weights,
91-action steering interface, external pose ownership or comfort-filter gains.

Our own reference already requests a 2 m/s³ jerk limit, yet measured nominal
jerk reaches 45 m/s³ and only 17/27 combined cases pass. Low override counts alone
have not fixed execution or flow. The last trained v05 pilot also collected no
I-110 merge/passing training episode: planned curriculum coverage is not actual
experience. These are prerequisites to solve before stronger imitation.

## 1. Establish honest merge and control measurements

- Adapt `traffic/gpudrive_merge_audit.py` to MAPPO's native SUMO recording format.
  Check original route/link continuity, the intended ramp-entry/auxiliary merge
  corridor, valid adjacent-lane requests, full-maneuver occupied corridors and
  completion beyond the merge. Keep destination arrival, mainline occupancy and
  verified merge separate. Missing corridor/marking data is an open gate, not
  permission to cross. Do not invent links to turn a failed maneuver into success.
- Audit saved I-110 failure windows first. Record actor output, physical request,
  any comfort conditioning, explicit guard result, TraCI command and measured
  acceleration, with timing, chosen leader/gap and route identity.
- Preserve the existing raw intervention metric. Also report per-stage counts,
  modification magnitude, final binding constraint and rejected actions. Never
  substitute a post-filter rate for the existing 5% acceptance gate.
- Retain fixed-time distance, low-speed/stopped exposure, fixed-cohort boundary
  time including unfinished trips, gaps/TTC, peak/RMS/p95 longitudinal jerk and
  braking imposed on other vehicles. Report all peaks, including startup and
  emergencies; identify them separately. Scalar jerk is not full vehicle jerk.

Primary files: `environments/highway_flow_env.py`,
`traffic/highway_flow_context.py`, existing flow metrics and experiment reports.

## 2. Fix a feasible reference and execution before training

- Inspect one recorded failed gap window. Add persistent pre-maneuver gap
  identity and longitudinal slot alignment to `traffic/highway_flow_reference.py`.
  Align speed early enough to enter the selected gap; reselect when that gap
  becomes infeasible. Keep current six-second planning, three-second lane
  execution, both occupied corridors and complete-maneuver gap checks.
- Anticipate the actual current-state native car-following response and upcoming
  mapped internal-lane speed limits. Resolve the source of measured oscillation
  and avoid target changes merely because an edge ID changes.
- Keep acceleration at −4.5…+2 m/s², 10 Hz decisions, 60 mph/lower legal caps,
  existing 1.5 s/native following protections and nominal 5 m/s³ comfort gate.
  Start with the existing reference's 2 m/s³ jerk target, speed gain 0.4,
  comfort deceleration 2 m/s² and gap buffer 3 m. GPUDrive's 1.8 s/3 m/s³
  conditioner is evidence to investigate, not a preset to copy into native SUMO.
- SUMO retains motion ownership. Any tested conditioning runs before final
  safety checks, is visible in training observations/transition accounting as
  appropriate, and remains separately measurable. Emergency braking cannot be
  delayed to satisfy comfort. Do not train through an undisclosed external driver.
- If new preview/commitment information is required by the actor, version the
  observation/export contract and test its migration. Use current map/state and
  the ego's causal commitment; do not expose teacher actions, future trajectories
  or global critic information as actor inputs.

Do not repeat the rejected remove-braking patches: they caused 12.3 s stopped
exposure and worsened arrival from 49.3 to 63.2 s. Do not increase imitation of an
unqualified reference or add a stronger jerk penalty as a substitute for this fix.

## 3. Use a small qualification ladder

First reuse compatible saved traces and unit tests. Proposed first runtime
allocation: **at most two same-seed, 80-simulation-second CPU checks sharing a
180-second wall ceiling**, charged to the existing preparation ledger. Stop and
inspect a failed candidate; no automatic parameter sweep or render campaign.
Freeze these cases and comparisons before running them.

If the focused case improves with no safety, arrival or gap regression, rerun
the existing reference qualification, including its 27 nominal/counterfactual
cases and full-route completion checks. Source-changed evidence is not reusable
merely because a prior run passed. Keep failure and event-coverage records.
Passing the small screen is not permission to bypass the full reference gates.

## 4. Teach the qualified behavior through the existing MAPPO pilot

- Keep shared actors, centralized training/decentralized execution, bounded
  acceleration and masked right/stay/left intent. No steering-to-lane translation
  or GPUDrive weight transplantation.
- After reference qualification, perform the existing fixed 48-case initializer
  comparison (24 development rows per candidate), migration/parity and gradient
  checks. Neither the rejected final v05 actor nor its earlier initializer is
  selected in advance. No held-out test data enters selection or demonstrations.
- Initially retain existing reward coefficients and the prepared comparison of
  acceleration imitation weight **1 versus 5**, overall coefficient **0.04**.
  Gate merge/arrival credit on the correct maneuver/route semantics. Version any
  semantic reward change; log every component before considering weight changes.
  Preserve incentives for progress and penalties for unsafe requests/stopping so
  smoothness cannot be achieved by refusing to drive. A standalone BC phase or
  new network/reward bundle would need a separate resolved configuration.
- Keep foundation cruise/following exposure, then passing/merge retention and
  varied AV participation. Verify actual completed training events, not just the
  schedule: an episode that never reaches the merge does not count as merge
  exposure. Abort/review a stalled curriculum before spending the remaining budget.
- Retain delivered-message validity/age, message-free fallback and communication
  ablations. Vary traffic density independently of active controlled-car fraction.
  Log the realized fraction and sender count; zero AV is a baseline, not training
  experience. Respect the existing cohort/message-slot limits. Live slider-driven
  handoff and closer-than-current following remain separate future milestones.

The existing proposed study remains **2 × 30,000 active decisions**, at most
2,048 critic-warmup decisions per arm, with actor/critic learning rates
1e-4/3e-4, PPO clip 0.2, four epochs, batch 256, gamma 0.99 and GAE 0.95.
Its shared wall budget is 5,400 s, including 1,800 s preparation and 900 s final
evaluation reserve. Reconcile already charged preparation/failures/ablations
before allocating more; this plan does not reset or enlarge that budget. If the
remaining budget cannot cover the required qualification and evaluation, present
a revised allocation for review; do not omit gates or silently extend the run.

## 5. Review the resolved config, then run with low chat usage

Only after source/config-bound readiness passes, present the exact initializer,
observation/action contract, losses, scenario/event coverage, checkpoints,
evaluation and remaining budget for review before launch. The current
`configs/training/highway-flow-v01.json` remains `training_authorized: false`,
`readiness: null`, with no initializer selected.

Once explicitly launched, scripts should write metrics/checkpoints and enforce
the agreed stop conditions. No model polling or status analysis while training
is healthy; inspect on completion or failure. Preserve failed attempts and resume
only with compatible checkpoint/config/state and within the same remaining budget.
Do not silently change rewards or increase runtime on failure.

Evaluate matched development cases with native/reference/no-communication/MAPPO
baselines. Reserve held-out seeds/topologies for the final selected candidate.
Keep the 5% per-case intervention criterion and existing safety/flow/comfort
gates; report actual proper merges, retained passing and communication behavior.
Omniverse/physical validation comes after the inexpensive SUMO gates.

## Status and next action

**Next proposed implementation:** steps 1–2 and the bounded screen
in step 3, beginning with the saved failed gap window. This turn only saves the
plan. No policy improvement, readiness or training is claimed. Keep the accepted
baseline and all historical checkpoints intact. Update this plan and the living
roadmap with evidence after each milestone. Google guide incremental sync remains
pending; the generated engineering-guide mirror was not edited.
