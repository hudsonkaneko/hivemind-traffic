# Highway merge training v09: round-11 proposal

Proposed by Claude on November 17, 2026. **Approved by the user the same day
("Approve and launch")**: the configuration below and a 300,000-decision
increase to the shared study ceiling. The authorized config differs from the
proposal only in `training_authorized` (now true) and its `authorization` text;
a test rebuilds the proposal from it and checks the approved SHA-256. Its own
file hash is `c02a2c33c8464560cf74a99e2cbffa8acbf8e978dedc4d8caf7a2168d7c3629a`.
One round only. Held-out evaluation is not part of this proposal and is not
authorized.

## Decision requested

| Item | Proposed value |
| --- | --- |
| Configuration | [`configs/training/highway-flow-rl-v09-round11.json`](../configs/training/highway-flow-rl-v09-round11.json), SHA-256 `918c313fdcccac74267c65c8df71247eaa6df00303e19c3e62fb079f5f073f13` |
| Starting policy | Complete v06 round-9 final checkpoint (`outputs/highway-flow-rl/v06/round-09/last.pt`, SHA-256 `8f056b40…1d31ab0`), the strongest actor in the shared 30-episode comparison |
| Change | Reward only: three v09 terms on the unchanged v06 environment |
| Length | One round, at most 300,000 learning decisions |
| Study decision ceiling | 4,930,000 → 5,230,000 (8,054 decisions are left today) |
| Study wall-clock ceiling | 46,800 s, unchanged. 4,388 s is available for training after the 900 s final reserve; about 2,900 s is expected |
| Diagnostic-episode cap | 125 → 126, for the one zero-update preflight episode |
| Round index | 11. Index 10 records the shared evaluation in the ledger. |

## Why round 9 stopped improving

Measured from round 9's own validation journals in the v06 environment
(`outputs/highway-flow-rl/v06/round-09/{initial,final}-validation`). No new
simulation was run.

1. **Merge-speed tracking stops giving a signal after 2 m/s of error.** The v06 term
   is `-0.25 * min(1, |v - target| / 2)`. After round 9, tracking is saturated on
   37% of I-110 merge decisions and on 53–63% of ramp-start merge decisions. Mean
   speed error while merging is -1.1 m/s on I-110 and -2.3 to -3.9 m/s on the
   ramp starts. Below 2 m/s of error, the penalty is flat, so the policy gets no
   gradient for closing the gap. Ramp cars request about 1.5 of the 2.0 m/s²
   allowed. The I-110 car holds about 14.7 m/s while the target rises to
   16.9 m/s, with +2.0 m/s² of headroom the whole time.
2. **The quality-merge bonus is never paid.** The v06 `quality_merge` +5 is
   all-or-nothing on reaching 90% speed match. No validation car reaches it; the
   best is 59%. So it adds no learning signal. On I-110, 5 of the 161 samples in
   the quality window also have a neighbor braking at -4.5 m/s². That alone fails
   the neighbor condition, whatever the car does.
3. **The jerk penalty lands at insertion.** 99.6% of round 9's `jerk_excess`
   charge falls in the first 5 decisions. During that fixed insertion lag, the
   raw request cannot reach the vehicle yet, so the penalty teaches nothing
   about driving.
4. **Progress makes up about 97% of the per-decision signal.** Progress is
   0.758 per decision; tracking is -0.006 and speed error -0.004. On the I-110
   merge case, round 9 collects 1,772 progress, -37.1 tracking, -22.1 speed error
   and -8.5 jerk excess.
5. **Updates were small.** Round 9's 4,608 optimizer steps changed the hidden
   actor weights by 1.4–3.6% and the output layer by about 10%. `log_std` stayed
   at -3.23 (σ ≈ 0.04 normalized, ±0.13 m/s²), so exploration is narrow.

The shared 30-episode comparison agrees. 93% of round 9's gain over the frozen
v04 actor is the I-110 case, and the ramp-start cars barely moved.

## Reward changes (v09)

Code: [`traffic/highway_flow_rl_v09_rewards.py`](../traffic/highway_flow_rl_v09_rewards.py)
and [`environments/highway_flow_rl_v09_env.py`](../environments/highway_flow_rl_v09_env.py),
a subclass of the v06 environment that changes only these three terms.
Vehicle dynamics, observations, actions, the shield and the rate limit are
unchanged.

| Term | v06 (round 9) | v09 (proposed) |
| --- | --- | --- |
| `merge_tracking` | `-0.25 * min(1, err / 2)`: flat after 2 m/s | `-0.25 * min(4, err / 2)`: the same slope out to 8 m/s, at most -1.0 per decision |
| `quality_merge` | +5 once, only if every condition holds (never paid) | Kept as is. New `quality_merge_graded`: once per car when its quality window closes, `5 * speed-match fraction * neighbor-calm fraction`, paid only when the car's own conditions hold (stopped ≤ 1 s in the window and on approach, every \|jerk\| ≤ 5 m/s³, overrides ≤ 5%) |
| `jerk_excess` | Charges max(requested, realized) from the first decision | Ignores the raw request during the fixed 5-decision insertion lag; realized jerk is still charged. Unchanged from decision 5 on |

Every other weight is the round-9 value (`v06_reward_weights`, `reward_weights`).

A worked example of the graded payment: at round 9's final I-110 match (0.590)
and neighbor-calm fraction (156/161), the payment would be 2.86, *if* the car's
own conditions hold. A car's own conditions are fully under its control. The
neighbor fraction scales the payment down but never removes it.

## What stays the same as round 9

[`traffic/highway_flow_rl_v09_contract.py`](../traffic/highway_flow_rl_v09_contract.py)
reverts each v09 field to its round-9 value and then runs the unchanged v06
round-9 validator on the result. So the following cannot drift without failing
validation:

- the 20 training rows and 8 validation rows, their networks and hashes, and
  the missed-gap curriculum;
- the observation and action contracts, the safety shield, the comfort rate
  limit, Krauss car-following for controlled cars, and the road-end rule;
- the optimizer: actor learning rate 1e-4, critic 3e-4, 10 epochs, minibatch
  256, target KL 0.1, 4 episodes per update, entropy 0.001;
- critic warm-up of 20,000 decisions, and the evaluation interval;
- acceptance, including the raw-request check after the insertion lag.

Only these fields differ from round 9:

- schema and round index;
- the learning profile and the reward version and settings;
- the warm start;
- the budget;
- the stop rules;
- provenance;
- the publication branch;
- the execution block;
- the missed-gap seed check;
- the authorization text.

The round-9 config is pinned by hash (`91947d58…`), and so is the warm-start
checkpoint. The loader also checks that the checkpoint was produced by that
round-9 config.

Round index 11 shifts every training seed. The preparation script rebuilt the
missed-gap demand for every cycle a 300k round can reach: PPO cycles 0–3 and
critic cycle 0, 15 seeds in total. Every one built with its blocker present.

## Stop rules

The run checks these at every periodic validation (100k and 200k decisions):

- **Collision:** any collision stops the run at once, at any episode.
- **Merge floor:** merge-speed match below **0.4343** stops the run. The floor
  is round 9's final validation (0.4543) minus 0.02. Unlike round 9, where the
  floor was report-only, it stops and saves the rest of the budget.
- **Interventions:** pooled interventions above 5% stop the run.

The final checkpoint is kept either way. Nothing is promoted without a
separate user decision.

## Execution

The run happens only from Claude's worktree,
`outputs/worktrees/claude-policy-2026-11-17` on branch
`2026-11-17-claude-policy-candidate`. Steps, once approved:

1. Set `training_authorized=true` in a reviewed commit on this branch. Any
   other edit changes the config hash and needs fresh approval.
2. `scripts/stage_highway_flow_rl_v09_inputs.py` copies in the warm start and
   the declared training and validation networks. Each copy is checked against
   its SHA-256 before and after copying, and an existing file is never
   overwritten. Held-out networks are not staged.
3. Run a zero-update preflight: one diagnostic episode, bound to this config
   and these source hashes.
4. Train with `experiments/run_highway_flow_rl_v09.py`. It writes outputs to
   the worktree's `outputs/highway-flow-rl/v09/round-11`. The shared ledger and
   lock are the canonical ones (`outputs/highway-flow-rl/v02`, found through
   Git's common directory), so Codex and Claude charge one budget.
5. Report results to the coordination master, update the logs, and stop.
   Nothing runs after that without a new decision.

Nothing in the canonical checkout or in Codex's worktree is edited.

## What would count as progress

Results are compared per case against round 9's final validation, not on the
pooled mean. Most of round 9's comparison gain came from one I-110 seed.

- Ramp-start merge matching (0.37–0.46 today) should rise. This is the direct
  test of the unsaturated tracking term.
- Overall merge-speed match should rise above 0.454 with no collisions, no
  regression in stable merges, and pooled interventions below 5%.
- The graded quality payment should be earned on more cars over the run.
- Gate counts should stay at 4/8 (original rules) and 3/8 (raw check) or
  improve.

This is not expected to reach the 90% merge-quality gate in one round. The gap
is too large: the best car is at 59%. The round measures whether removing the
saturation moves the ramp cars at all. That tells us whether reward shaping
can still help, or whether the next step has to be something else, such as a
larger exploration σ or a change to the merge target.

## Risks

- **A stronger tracking pull may raise jerk.** At most -1.0 per decision, the
  term can now outweigh progress (0.76 per decision) for a car more than 6 m/s
  off target. v08's retrain regressed on jerk. The rate limit (4.0 m/s³
  realized) and the unchanged `jerk_excess` after the lag bound this, and the
  merge floor stops a run that trades away merging.
- **Timing gaming.** A car could try to shorten its time in the merge window
  instead of matching speed. Merge timing will be reported against round 9.
- **The I-110 neighbor gate.** The original quality gate on I-110 can still
  fail because of neighbor braking, whatever the car does. The graded payment
  rewards partial credit, but the gate itself is unchanged.
- **Narrow exploration** (σ ≈ ±0.13 m/s²) may limit how far one round moves.
  This proposal does not change it, to keep the round a single-variable test of
  the reward.

## Tests

`tests/test_highway_flow_rl_v09.py`: 26 tests pass. They cover:

- the tracking slope and cap, the jerk lag, and the graded-quality math and
  gates;
- the environment hook: tracking -0.5 at 4 m/s slow; no raw jerk charge before
  decision 5; the graded payment of 2.5 made exactly once;
- the settings guard; that the config validates and stays unauthorized; that
  only the listed fields differ from round 9; 11 rejected mutations;
- runner install (the ledger resolves outside the worktree); the warm-start
  loader; and staging.

The broader run of the highway-flow, v07 and v08 tests gave 481 passed and
2 failed:

- `test_highway_merge_cutin_v07.py::test_sparse_five_lane_missed_gap_keeps_a_blocker_without_reseeding`
- `test_highway_merge_recovery_v08.py::test_seeded_cutins_are_real_adjacent_spawn_pairs`

Both fail with FileNotFoundError: they read network files under `outputs/`,
which a fresh worktree does not have (for example,
`outputs/highway-flow-rl/v05/ramp-networks-02/train-5-lane-ramp240/network.net.xml`).
They do not touch v09 code. They will pass once the inputs are staged, or when
run in the canonical checkout.

## Status

- Approved and run on November 17, 2026. The result is below.
- Held-out evaluation is not authorized.
- Google engineering guide sync is still pending.

## Round 11 result (November 17, 2026): stopped by the merge floor at 100k

The run stopped itself at its first periodic validation, after 100,457
decisions and 1,084 s, with no collisions. Pooled merge-speed match was 0.211,
below the 0.4343 floor. The round-9 policy (the starting point) stays the best
policy; nothing is promoted. The stop message says "below the frozen-v04
baseline 0.4343". That is the v06 runner's wording: the v09 floor is round 9's
final validation minus 0.02.

Evidence: [round-11-results.json](evidence/highway-merge-training-v09/round-11-results.json)
(validation summaries with hashes, per-car traces, updates, ledger and
checkpoint hashes). Run outputs stay in Claude's worktree,
`outputs/highway-flow-rl/v09/round-11`, and are not versioned.

### Per merging car

"Before" is the run's initial validation of the round-9 policy in the v09
environment. It reproduces round 9's final scores exactly. "After" is the
validation at 100k decisions.

| Merging car | Before | After 100k | What changed |
| --- | --- | --- | --- |
| I-110 | 0.590, stable | Stable merge failed; no quality window | Merged 6.1 s later (entry 23.0 s vs 16.9 s). In the 8 s after merging, 32 of 81 decisions had a native-realization override |
| Ramp 120 m, 2 lanes | 0.461 | 0.468 | Mean speed error while merging -2.33 → -2.25 m/s |
| Ramp 180 m, 2 lanes | 0.397, stable | Stable merge failed; no quality window | 6 of 81 post-merge decisions clamped by the speed ceiling (7.4%; the gate allows 5%). Error -3.74 → -3.63 m/s |
| Ramp 180 m, 4 lanes | 0.369 | 0.376 | Error -3.94 → -3.82 m/s |

The pooled score counts a car whose stable merge fails as zero:
(0 + 0.468 + 0 + 0.376) / 4 = 0.211.

Other measures:
- Gate counts unchanged: 4/8 under the original rules, 3/8 with the raw check.
- Pooled interventions rose from 0.26% to 0.51%. Peak jerk stayed 4.0 m/s³.
- Stable merges fell from 4 to 2.
- The graded quality payment was already earned by the round-9 policy at the
  start (2.86, 2.31, 1.99, 1.84). After training: 0, 2.34, 0, 1.88.

Updates:
- 43 update records, 10 of them critic-only.
- 8 merged PPO updates, 2,583 optimizer steps; 2 stopped early on KL (0.108
  and 0.188).
- `log_std` barely moved: -3.227 → -3.220.

### What this shows

- **The tracking change pushed the right way, but only weakly.** On the two ramp
  cars that still merged stably, matching rose by 0.007 each. Mean speed error
  improved by only about 0.1 m/s, and ramp cars requested about the same
  acceleration as before (1.00–1.16 m/s² while merging).
- **The drop is the stable-merge gate, not a safety problem.** Both failures
  are override counts in the 8 s after merging, which the gate caps at 5%
  (4 of 81 decisions).
  - On the 2-lane ramp 180 case, the car reaches its target at the speed limit
    (24.59 m/s) and keeps asking for small positive acceleration that the speed
    ceiling clamps.
  - On I-110, the car merged 6 s later. It then sat just under its 18.0 m/s
    target, asking for +0.1–0.2 m/s² that the native vehicle model did not
    realize.
  - In both cases the requests press against a limit the car cannot pass. The
    I-110 timing change is one episode on one seed; it is not attributed
    causally.
- **What a further attempt would need.** The tracking term should not reward
  pushing against limits. Two candidates: clip the tracking target to what the
  car can actually reach (the speed ceiling and the native leader), or charge
  requests that the speed ceiling or native realization would cut in the
  post-merge window. That would be a new proposal with its own approval.

### Ledger after round 11

- Training decisions: 5,022,403 of the raised 5,230,000 (207,597 left).
- Time: 42,599.8 s of 46,800 s charged (3,300 s left for training after the
  final reserve).
- Diagnostic episodes: 126 of 126.

The approval covered this one round only. Nothing else runs without a new
decision.
