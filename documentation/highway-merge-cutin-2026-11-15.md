# November 15, 2026: merge and cut-in training revision

Branch: `2026-11-15-codex-merge-cutin`. User approved implementation and one
300,000-decision / 5,400-second training round after reviewing the plan.
This checkpoint prepares the run; it does not claim learned improvement.

## Changes

- Keep the audited v06 rewards, flow117 actor, centralized critic, Krauss,
  10 Hz control, 60 mph/lower posted limits, native safety and original gates.
- Select complete training episodes by deficits in **actual agent decisions**:
  normal merge 30%, missed-gap recovery 20%, cut-in recovery 10%, cruise/follow
  25%, passing 15%. Report finite-episode sampling error rather than claiming
  exact proportions. Critic and actor exposure are balanced independently.
- Background vehicles retain native longitudinal control. Native lane-change
  model intentions pass an additional admission layer, using mode 512 to
  prevent unreviewed native lateral moves. AV requests use that layer too.
  Require front and rear time headway >=1.5 s, TTC >=3 s and bumper gap >=2.5 m,
  at the current and projected end state of the 3.1-second maneuver.
- Reservations include admitted vehicles before their lateral displacement is
  measurable, preventing simultaneous moves into one gap. Remaining-maneuver
  checks run every tick and record violations. Constant-speed prediction is
  not a guarantee against later acceleration; SUMO retains collision avoidance.
  Ongoing moves are not teleported or abruptly canceled to hide a violation.
- Separately labeled stress rows can request one close cut-in ahead of an AV,
  using 0.5 s headway but retaining 2.5 m clearance, 3 s TTC and native guards.
  Requested, started and completed stress events are distinct counters.
  Stress failures remain visible and are summarized separately from nominal
  traffic. Low event counts must be reported, not counted as successful coverage.
- Fix sparse five-lane missed-gap demand: guarantee that the blocker lane is
  occupied without resampling the seed, changing vehicle counts or rebuilding
  roads. The new option is disabled for older environments.

## Approved learning settings

Resolved configuration: [highway-merge-cutin-2026-11-15.json](../configs/training/highway-merge-cutin-2026-11-15.json).
Warm-start v04 `best.pt` (79e36b54...), fresh optimizer; 20,000 critic-only
decisions inside the 300k budget. Actor/critic LR 1e-4/3e-4, 20 maximum epochs,
minibatch 256, clip 0.2, KL 0.02, entropy 0.001, gamma/lambda 0.99/0.95.
Whole-episode routine jerk conditioning remains 4 m/s3; emergency safety
limits bypass it. Raw requested jerk remains an acceptance check.

Rewards retain progress 1, merge 5, quality merge 5, passing/arrival 2 each,
speed error 0.30, merge tracking 0.25, Huber jerk 0.09, excess jerk 1.50
(capped), avoidable braking 0.25, gap potential 2, intervention 0.20,
unsafe request 0.50, collision 20 and the existing lane/cooperation terms.

Full initial/final and automatic 100k validation use matched inputs and the
same admission rules. The merge-regression stop compares to the new run's
initial nominal baseline, not a score from different traffic rules. Collisions
stop the run. No automatic second round or held-out evaluation is authorized.
Short-ramp physical ceilings and original failures are retained; no gate is
relaxed to manufacture acceptance.

## Verification and execution

- 155 targeted tests pass in two isolated processes (116 legacy, 39 v06/v07).
  The existing v06 runner-install test changes module globals, so combining it
  before a source-inspection v05 test causes order dependence; isolation avoids
  altering either runtime to accommodate that test side effect.
- 72 deterministic missed-gap/stress demand seeds prepared without simulation.
  The first attempt exposed the empty blocker lane; the regression is retained
  in the new test and fixed explicitly.
- Before learning, the job requires nominal and stress simulator checks,
  measured native admission and stress motion, and a source/config-bound
  zero-update preflight. Any failure ends the job with evidence.
- An existing v06 process is left untouched. The launcher waits on its OS
  process handles, then acquires the existing shared study ledger/lock. The
  historical ledger is never reset. Round six adds at most the approved 300k
  and 5,400 seconds to the study ceilings (4.93M / 46,800 s).
- New code executes from this branch's existing worktree. Pinned network and
  checkpoint inputs are copied there with hash verification by the runner.
  New outputs stay under its `outputs/highway-flow-rl/v07/`; logs under `logs/`.
  Requested/applied/realized control, rewards, per-car safety, merge quality,
  curriculum coverage, checkpoints, versions and raw failures are retained.
- The launcher blocks on process completion, with no model-driven progress
  polling. Review only after completion/failure. Google engineering-guide
  synchronization for this milestone is pending; its generated mirror is unchanged.

References: [SUMO lane-change state](https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html#change_lane_information_0x13),
[lane-change control mode](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane_change_mode_0xb6).
