# Highway merge training v10: round-12 proposal

Proposed by Claude on November 17, 2026, after
[v09 round 11](highway-merge-training-plan-v09.md#round-11-result-november-17-2026-stopped-by-the-merge-floor-at-100k)
was stopped by its merge floor. **Approved by the user the same day ("Approve and
launch").** The approval covers the configuration below and the use of 200,000 of
the remaining decisions. The authorized config differs from the proposal only in
`training_authorized` (now true) and its `authorization` text. A test rebuilds the
proposal from it and checks the approved SHA-256. Its own file hash is
`779181ac22d7ce36a710b9eece3160336b979b65affc157265f02da4b68ad208`. One round
only. Held-out evaluation is not part of this proposal and is not authorized.

## Decision requested

| Item | Proposed value |
| --- | --- |
| Configuration | [`configs/training/highway-flow-rl-v10-round12.json`](../configs/training/highway-flow-rl-v10-round12.json), SHA-256 `08730eafc1c3bec89f9b916e6081f06f9e19d7a5cb5ad6804927ddfafec6bcd6` |
| Starting policy | The complete v06 round-9 final checkpoint again (`8f056b40…1d31ab0`). Not the round-11 checkpoint, which regressed |
| Change | v09's reward terms plus one new term: a post-merge override cost aligned with the stable-merge gate |
| Length | One round, at most 200,000 learning decisions |
| Study decision ceiling | **No increase.** It stays at 5,230,000, which leaves 207,597 decisions after round 11 |
| Study wall-clock ceiling | 46,800 s, unchanged. 3,300 s is available for training; about 2,100 s is expected |
| Diagnostic-episode cap | 126 → 127, for the one zero-update preflight episode |
| Round index | 12. Index 11 is v09 round 11. |

After this round, about 7,600 decisions would be left under the current
ceiling. Any later round would need a new budget decision.

## What round 11 showed

All evidence below comes from round 11's own records: the validation JSON, the
action journals and the 43 update records. It is summarized in
[round-11-results.json](evidence/highway-merge-training-v09/round-11-results.json).
No new simulation was run for this proposal.

- **Validation.** Two of four merging cars failed the stable-merge gate. The
  gate allows at most 5% override decisions in the 8 s after the lateral merge,
  which is 4 of 81 decisions. Neither failure was a safety problem.
  - **I-110:** 32 of 81 post-merge decisions had a native-realization change.
    The car was just under its 18.0 m/s target behind traffic and kept asking
    for +0.1 to +0.2 m/s² that the native vehicle model did not deliver.
  - **Ramp 180 m, 2 lanes:** 6 of 81 decisions were speed-ceiling clamps. The
    car was at its target, which equals the speed limit, and kept asking for a
    little more.
- **Training episodes show the same change.** In episodes 16–43, 6 of 17 merging
  cars failed the stable-merge gate. Round 9 had 2 of 17 at the same point.
  Four of the six had 47–54 of 81 post-merge decisions with native-realization
  changes. Round 9 had no failure of that kind. In those decisions the car was
  within 0.1–0.5 m/s of its target, where tracking costs only -0.005 to
  -0.05. So the push is not the tracking slope acting there. It is a general
  shift toward small positive requests, learned where the stronger pull did
  help: far below target on the ramps.
- **Why the reward did not stop it.** Every override costs 0.2 through the
  unchanged intervention term, so five extra post-merge overrides cost about 1.
  The gate result they break is worth the whole graded quality payment (up to 5),
  but that payment is sparse: it comes once per episode, so the signal is weak
  and noisy.
- **The tracking change itself helped only slightly.** On the two ramp cars that
  stayed stable, matching rose by 0.007 each.

## Reward change (v10)

Code: [`traffic/highway_flow_rl_v10_rewards.py`](../traffic/highway_flow_rl_v10_rewards.py)
and [`environments/highway_flow_rl_v10_env.py`](../environments/highway_flow_rl_v10_env.py),
a subclass of the v09 environment that adds one term.

**`post_merge_override`:** -1.0 for each decision that the stable-merge tracker
counts as an intervention inside its 8 s post-merge window.

- Added to the unchanged 0.2 intervention cost, each such decision costs 1.2.
- Five of them, the first count that fails the gate, cost 6.0. That is more
  than the largest graded payment.
- The count is read from the tracker's own post-window intervention count. So
  the charged decisions are exactly the gate's numerator: policy-filter changes
  (including speed-ceiling clamps) and native-realization changes.

All v09 terms stay as they were: unsaturated tracking, the graded quality
payment, and no jerk charge on the raw request during the insertion lag. So do
all v04 and v06 weights.

## What stays the same as round 9

[`traffic/highway_flow_rl_v10_contract.py`](../traffic/highway_flow_rl_v10_contract.py)
reverts each v10 field and runs the unchanged v06 round-9 validator, the same way
the v09 contract does. The following are therefore unchanged: rows, networks,
the missed-gap curriculum, observations, actions, the safety shield, the rate
limit, Krauss car-following, the road-end rule, the optimizer (actor learning
rate 1e-4, 10 epochs, target KL 0.1, 4 episodes per update), the 20,000-decision
critic warm-up, and acceptance.

Round index 12 shifts every training seed. A test checks 40 cycles of round-12
training and critic seeds against every validation and held-out seed and finds
no collision. The missed-gap demand was rebuilt for 15 seeds (PPO cycles 0–3 and
critic cycle 0), and every one has its blocker.

## Stop rules

The same as round 11:

- **Collision:** any collision stops the run at once.
- **Merge floor:** merge-speed match below **0.4343** stops the run. The floor
  is round 9's final validation minus 0.02. It is checked at the periodic
  validation (100k decisions); the final validation records a below-floor score
  without stopping.
- **Interventions:** pooled interventions above 5% stop the run.

Because the pooled score counts a car whose stable merge fails as zero, losing
any one of the four stable merges at 100k puts the score below the floor and
stops the run.

## Execution

The same as round 11, from Claude's worktree on branch
`2026-11-17-claude-policy-candidate`, once approved:

1. Authorize the config in a reviewed commit.
2. Stage inputs with `scripts/stage_highway_flow_rl_v10_inputs.py`. It runs the
   v09 staging checks; the inputs are already present and hash-verified.
3. Run a zero-update preflight (one diagnostic episode).
4. Train with `experiments/run_highway_flow_rl_v10.py`. Outputs go to
   `outputs/highway-flow-rl/v10/round-12`. The run uses the canonical shared
   ledger and lock.
5. Report per case against round 9, then stop.

## What would count as progress

- All four validation merging cars keep their stable merge. Each car's
  post-merge override count stays at 4 of 81 or fewer.
- Per-case merge matching is at or above round 9's on every merging car (0.590,
  0.461, 0.397, 0.369). The ramp cars should also show round 11's small
  tracking gains.
- Stable-merge failures in training episodes return to about round 9's rate
  (2 of 17 in episodes 16–43).
- No collisions; gate counts at 4/8 and 3/8 or better.

**Expectation, stated plainly:** even if v10 removes the regression, the gain is
likely small. v09's tracking change moved the ramp cars by about 0.007 in 100k
decisions, and the optimizer, exploration and updates per decision are
unchanged. This round tests whether that small gain survives once the post-merge
regression is priced in. It will not reach the 90% quality gate.

## Risks

- **Over-correction.** The car may become timid after merging and give up speed
  to avoid native-realization changes. Tracking and progress both penalize
  that, and the merge floor stops a run that loses merge matching.
- **Native limits are only partly visible.** The car must learn when a leader
  will hold it back from its existing leader observations. If it cannot, the
  cost becomes noise rather than signal.
- **One seed per case.** Validation runs one seed per case. A single post-merge
  episode can decide the stop rule, as it did in round 11.
- **This is the last round the current ceiling allows.**

## Tests

- `tests/test_highway_flow_rl_v10.py`: 22 tests pass. They cover:
  - the cost math and its size against the graded payment, and that v10's
    settings extend v09's without changing them;
  - the environment hook: it charges exactly the tracker's new post-window
    interventions (including two at once), leaves the v09 terms unchanged, and
    resets each episode;
  - the settings guard; that the config validates and stays unauthorized; that
    only the listed fields differ from round 9; round-12 seed separation; the
    missed-gap coverage; 13 rejected mutations;
  - the runner install (v10 environment, the v09 warm-start loader, the
    canonical ledger), and staging reuse.
- `tests/test_highway_flow_rl_v09.py`: 26 pass.
- Broader run of the highway-flow, v07 and v08 tests: 393 passed and 2 failed.
  The two failures (`test_sparse_five_lane_missed_gap_keeps_a_blocker_without_reseeding`
  and `test_seeded_cutins_are_real_adjacent_spawn_pairs`) write environments to
  pytest's temp directory, which the environment rejects because it is outside
  `outputs/`. With `--basetemp` inside `outputs/`, both pass. They do not
  involve v09 or v10 code.

## Not done and not authorized

- No training, staging run, preflight or ledger write has happened for v10.
- Held-out evaluation is not authorized.
- Google engineering guide sync is still pending.
