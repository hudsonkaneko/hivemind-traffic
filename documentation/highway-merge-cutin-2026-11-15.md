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

## First native qualification and preflight correction

`train-20261115-01` retained both completed smoke cases: zero collisions,
teleports or overspeed. Nominal combined interventions 0.5249%, two admitted
native moves, 161 rejected native requests, zero ongoing gap violations.
The stress case executed and completed one measured cut-in, interventions
4.1754%; eight ongoing gap-violation samples remain recorded. These are
frozen-actor diagnostic episodes, not learned improvement or full acceptance.

Preflight stopped before learning because the v07 mask had retained the old
v04 audit-version label. Corrected the version at audit initialization and
added a regression assertion. Also withdraw queued native requests if the gap
is lost before motion starts; remaining-maneuver prediction now starts from
measured motion onset. Repeat source-bound qualification before launching.
All artifacts and charges from the failed attempt remain in the study ledger.

## Training outcome: stopped at the declared regression gate

Source commit `1ecba3b`; run `outputs/highway-flow-rl/v07/train-20261115-02/train`.
The second job passed both native smoke checks and the zero-update preflight,
then trained without interactive monitoring. At 100,728 decisions (23,955
critic-only), 62 updates and 696 actor/2,000 critic optimizer steps, the first
periodic validation triggered the configured merge-regression stop. There was
no runtime training defect to bypass. No second round or automatic promotion.

| Matched nominal validation (8 cases) | Initial frozen actor | Trained candidate |
| --- | ---: | ---: |
| Merge speed matching, four mergers | 43.52% | 39.66% |
| Stable merges | 4/4 | 4/4 |
| Pooled combined overrides | 0.265% | 0.327% |
| Peak realized jerk | 4.762 m/s3 | 4.000 m/s3 |
| Collision / teleport events | 0 / 0 | 0 / 0 |
| Cases passing every original + raw-request gate | 1/8 | 1/8 |

I-110 speed matching improved 52.80% -> 54.38%, but all three corrected-ramp
cases declined, so aggregate merge quality failed the predeclared rule. Raw
requested jerk still fails despite smoother executed motion; no claim that
the actor itself is sufficiently smooth. One ongoing nominal gap-violation
sample remains in both evaluations; admission prediction does not guarantee
all later gaps under changing speeds.

The two stress validation cases produced one actual cut-in initially and zero
after learning. Their lower aggregate interventions are therefore **not proof
of better cut-in recovery**. Across 11 stress-labeled training episodes, two
cut-ins actually started and completed. That is insufficient stress coverage;
a subsequent approved revision should create reliable recoverable cut-in
encounters before increasing learning time. All 62 training episodes and both
validation sets recorded zero collisions and teleports.

Actual PPO decision shares at the early stop: normal merge 26.51%, missed gap
18.65%, stress 9.68%, cruise/follow 26.62%, passing 18.54%. Full episodes cause
sampling overshoot; these measured values, not the target percentages, describe
this stopped run. Missed-gap exposure is higher than the previous ~5% proposal.

Saved evidence: [result.json](evidence/highway-merge-cutin-2026-11-15/result.json)
and [paired cases.csv](evidence/highway-merge-cutin-2026-11-15/cases.csv), including
input/output hashes, actual coverage, per-case failures and event counts.
The full checkpoint `last.pt` and exported `candidate-policy.pt` remain local
under the run directory. The candidate is experimental and not accepted;
`best-policy.pt` remains the initial actor. No held-out test, 3D run or new
training was started after the stop. Google guide synchronization is pending.
