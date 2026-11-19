# Highway merge training v13: the merge target follows SUMO's yield decision

Started by Claude on November 19, 2026, after
[v12 round 19](highway-merge-training-plan-v12.md#round-19-result) stopped on the
intervention rule on the LA roads exactly as the 3D model has them.

**User direction:**
- November 18: "dont need to verify, just use that road for the training" ...
  "the exusting 3d model we have". The 3D model is not modified.
- November 19: "fix", the go-ahead to make the target speed follow SUMO's yield
  decision at these merges and to check it with a short test run without training.

No training round is declared yet. A round of useful size needs a study-budget
decision from the user: 100,868 decisions remain.

## The problem, as measured

On the model-exact LA corridors, the ramp joins the right lane at a yield junction
(connection state `m`); the model has no acceleration lane. The gate's merge-speed
target (`merge_speed_target`,
[`traffic/highway_flow_rl_v04_rewards.py`](../traffic/highway_flow_rl_v04_rewards.py))
does not consider whether SUMO will let the car through. Its gap plan projects the
car forward at the target speed, wherever the yield line is.

Round 19's focused northbound validation case (`v12-val-la-nb-pasadena-model-merge-focused-1of16-medium`,
seed 6950003), from its step journal:
- **Start.** The ramp car starts 30 m before the yield line at 9.0 m/s. A mainline
  car is 28 m behind it in the target lane, 10.6 m/s faster.
- **Approach.** The target says 16–20 m/s ("behind_rear": go in behind that car), and
  the policy accelerates.
- **Brake.** At t = 1.8 s, 14 m from the line at 11.3 m/s, SUMO overrides the command
  (+1.67 m/s²) with −4.5 m/s² until the car stops at the line (t = 4.3 s).
- **Held.** SUMO holds the car at the line for 13 s. In that time three mainline cars
  pass it at about 22 m/s. The target stays between 9.4 and 13.8 m/s (mostly 13.8).
  Every held decision costs the full band penalty, and every request to accelerate
  counts as an intervention.
- **Release.** The car moves at t = 17.4 s. From t = 17.1 s the journal shows no
  mainline car within 120 m upstream.
- **Result.** 41% interventions, 0 of 301 band decisions in band, peak jerk 52.8 m/s³.

## What SUMO reports

TraCI `vehicle.getNextLinks` (SUMO 1.27.1) lists the links ahead of a car. The tuple
order, read from `traci/_lane.py`, is: lane, has priority, is open, has foe, via lane,
state, direction, length.

A replay of the same case with the same policy logged that link every step. The
replay reproduced the round-19 result exactly (41.1% interventions), so the logging
query did not change the run. The link to `sb2_0`:
- read **closed from t = 0.6 s**, about 26 m before the line: 1.2 s before SUMO's
  brake at t = 1.8 s;
- stayed **closed the whole time the car was held** (t = 4.3–17.0 s);
- read **open at t = 17.1 s**, 0.3 s before the car moved;
- read **open at t = 3.2–3.7 s** while SUMO was still braking the car to a stop.
  The SUMO source is not installed here, so the reason for this mismatch is not
  verified. v13 counts these reads (`open_while_braking`, link open while the car's
  measured acceleration is at or below −3 m/s², the v04 gate's braking threshold).

## The v13 rule

[`traffic/highway_flow_rl_v13_rewards.py`](../traffic/highway_flow_rl_v13_rewards.py),
[`environments/highway_flow_rl_v13_env.py`](../environments/highway_flow_rl_v13_env.py):

- **When.** The car's pending merge is a yield entry (`entry_active`), and SUMO reports
  the link to the entry's target lane closed.
- **Cap.** The target is capped by a stop at the yield line:
  `sqrt(2 · 2 m/s² · max(0, distance − 2.5 m))`. This is the same form and the same
  constants as v03's existing `unseen_yield_stop` bound. Waiting at the line while the
  link is closed is therefore on target (0 m/s).
- **The cap only lowers the target.** When SUMO reports the link open, when the link
  is not listed, or when the merge is not a yield entry, the target is exactly v12's.
  Acceleration-lane merges are unchanged.
- **One target for everything.** The capped target feeds the four actor target
  inputs, the `target_band` and `merge_tracking` rewards, and the gate's speed match.
  Both target hooks (`_reward_target` before the decision and `current_target` for the
  actor inputs) query SUMO once per car and step.
- **Nothing else changes.** Observation widths, dynamics, the safety shield, native
  guards, masks, gate thresholds and reward weights are v12's.

What this does not change: the gate still fails a merge whose approach includes more
than 1 s stopped. When SUMO holds the car at the line in dense traffic, that merge
fails the quality gate no matter how the car is driven.

## Development smoke check

Outside the ledger, as v12's smoke check was; no training, not a validation. It ran
round 19's 14 validation cases through the v12 runner's own `evaluate`, with the v13
environment and round 19's starting policy: 14 episodes, 26,507 decisions, 122 s.
The one-episode link replay above was also outside the ledger.

**Driving is identical in all 14 cases** (collisions, interventions, jerk, lane
changes, steps). The starting policy was migrated with zero weights on the four
target inputs, so it ignores them. The check therefore shows what the target and the
grading now say about the same driving, not how training will change it.

**Unchanged where it should be.** The eight non-LA cases match round 19 in every
target, reward and gate value. In the I-110 merge, SUMO reported its yield link open
on all 36 yield decisions, so its target was never capped.

**The LA merges** (round 19's initial validation → v13, same driving):

| Case | Link closed / yield decisions | Target lowered | In band | Gate speed match per merging car |
|---|---|---|---|---|
| NB merge 8 of 16 | 86 / 351 | 86 | 0.22 → 0.30 | 0.13, 0.29, 0.65, 0.00 → 0.13, 0.29, 0.65, 0.17 |
| SB merge 8 of 16 | 246 / 376 | 199 | 0.03 → 0.22 | 0.00, 0.00 → 0.04, 0.00 |
| NB focused merge | 159 / 177 | 159 | 0.00 → 0.46 | 0.00 → 0.18 |
| SB focused merge | 145 / 153 | 145 | 0.00 → 0.38 | 0.00 → 0.05 |

- **Link state:** SUMO listed the yield link on every yield decision (none missing).
  It read open while the car braked at −3 m/s² or harder on 8, 11, 6 and 0 decisions.
- **Totals:** the pooled merge speed match went from 0.381 to 0.418. Gates stay at
  8 of 14 and pooled interventions at 2.3%, because the driving is the same.

**What still disagrees, in the focused NB case** (decision times; the journal logs
each decision 0.1 s later):
- **Approach.** The cap applies from the first decision on which SUMO reports the
  link closed (t = 0.6 s). The starting policy keeps accelerating, so it is out of
  band until SUMO brakes it. This is what training on the new target would have to
  change.
- **Open while braking.** At t = 3.2–3.7 s the link reads open while SUMO brakes
  the car to a stop, and the target jumps back to 16–17.6 m/s.
- **Held.** Target 0, in band for the whole 13 s wait.
- **Release (the largest remaining gap).** Once SUMO opens the link, the target is
  v12's again: 13.8 m/s with the car at standstill, rising to 24.6 and then
  26.8 m/s. The car accelerates at about 1.9 m/s². Up to the end of the stop, 138
  of 173 band decisions are in band (SB: 112 of 149). After it, 0 of 128 (SB: 0 of
  149). That post-release target is not part of the yield cap. It comes from v12's
  rules, which do not limit the target to what the car can reach from a stop.
