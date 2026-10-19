# Bounded live-showcase gap creation — October 19, 2026

Status: experimental controller capability, with complete CPU regression,
independent source review, the original native endpoint, its repeat and the
legacy control smoke passed. Independent statistics and lifetime audits passed;
**matched performance validation remains pending**.
Its purpose is to test whether a brief, guarded slowdown followed by ordinary
acceleration can create a passing opportunity in the existing scripted scene.
It does not promise that every blocked formation can be passed.

The prior [admission checkpoint](showcase-validation-20261019.md#dormant-entry-repair-population-fixed-passing-still-fails)
is preserved. Commit `396f7ef0b400752500a57f2e6d53251babb9f507` repaired dormant
entry placement. Native endpoint `20261019T164433Z-563caac3` then reached every
4/80/12/80 population deadline, but its settled 135–180-second tail completed no
new pass or lane change. Overall endpoint acceptance remains **failed**. This
separate controller experiment does not reinterpret that result or move active
traffic to manufacture an opening.

## Scope and unchanged authority

The change is confined to `LiveShowcaseController` in
[`traffic/live_showcase.py`](../traffic/live_showcase.py), used by v09. The
ordinary [`ShowcaseController`](../traffic/loop_showcase_control.py) retains
steering, path selection, front-following/emergency behavior, state and track
validation, sequencing, command expiry and the full configured cruise target in
every candidate forecast. V07/v08 do not gain this supervisor. The background
entry rules, kinematic motion owner, physical car, scene composition, endpoint
schedule and acceptance thresholds remain unchanged.

The ordinary planner runs first. The supervisor may lower its longitudinal target
with `min(ordinary_target, deliberate_cap)` while a bounded attempt is active.
It never lowers `config.target_speed_m_s` to make a lane-change forecast easier.
After releasing the cap, it uses the ordinary following target and speed servo.
Only a freshly accepted ordinary maneuver can supply the actual lane-change
path; a hypothetical preflight result never authorizes a future path.
The cap floor is a target limit, not a guaranteed lower bound on measured
physical speed; ordinary servo response can undershoot it.

This remains one PhysX car with privileged measured simulator state, a known
circle and scripted nonreactive kinematic backgrounds. It introduces no sensed
driving, SUMO interaction, learned policy or training.

## Why slowdown must include recovery

The repaired endpoint's recorded 135-second formation differs from the original
failed endpoint. Ego is in lane 2 at approximately 6.8059 m/s behind
`background_006_g001`, whose center is 17.8762 base-station metres ahead.
Same-lane rear `background_074_g002` is about 449.3627 m behind. Lane 1 rear
`background_005_g001` is only 38.6636 m behind, and lane 3 has a car nearly
alongside. Those adjacent cars remain part of the ordinary merge forecast.

Counterfactual scans rotate ego backward relative to the recorded fleet and
query the unchanged planner at declared speeds. They do not integrate physical
braking, reproduce an exact planner tick or prove that the real car reaches that
state. At measured following speed, no full ordinary-plan opening was found over
the scanned 0–200 m added lag. At 12 m/s a lane-1 opening appeared at 31.5 m lag;
at 14 m/s it appeared at 26.5 m. Recovery speed matters: holding a low cap until
lane acceptance is not the implemented strategy.

The retained analyses are
`outputs/showcase_validation/oct19-review/gap-creation-repaired-analysis.json`
and `gap-creation-repaired-recovery-analysis.json`, with source and input hashes.
They are feasibility evidence for this recorded formation, not a successful
native maneuver or general solution for slow traffic.

## Bounded profile

| Condition or budget | Implemented limit |
| --- | --- |
| Entry dwell | 2 simulated seconds of continuous eligible blocked following |
| Settled speed | Measured lead at least 5 m/s; ego within 0.35 m/s of the lead |
| Strict straight-lane tracking | Radial error at most 0.2 m; heading error at most 0.04 rad |
| Attempts and search memory | At most 3 actual attempts and 16 examined formation signatures |
| Repeated formation | Once per current lane, lead identity, configuration and complete membership signature |
| Preflight | At most 12 queries: 24/30/34 m added lag × measured ego speed +2/+4/+6/+8 m/s, each clipped to unchanged cruise |
| Deliberate cap | Target falls at most 1 m/s², at most 4.5 m/s below its initial value, never below 2.5 m/s |
| Cap release | At 34 m measured added lag, 11 simulated seconds, or an earlier release condition |
| Whole attempt | At most 21 simulated seconds, including recovery; at most 50 m added lag |

The dwell requires the same blocked lead, stable tracking and settled population
and traffic speed. Approach transients reset the dwell without consuming the
formation assessment. A preflight succeeds only if the ordinary planner selects
a safe adjacent lane **and** its following target supports the hypothetical
probe speed. The bounded grid is a possible-opening check, not a dynamics model.
There is no absolute endpoint-time trigger or repeated blind search each tick.

The states are idle, waiting, rejected, slowing, recovering, complete and failed.
Requested target/gap changes, population/lifecycle changes and speed ramps
release an active cap into guarded recovery. Front emergency braking retains
authority. A fresh ordinary maneuver ends the supervisor only after the strict
guard has been checked. There is no separate fixed cooldown in this profile;
the once-per-formation and episode/search limits bound repeated attempts.

All budgets use simulation ticks. A native pause advances neither elapsed time
nor measured lead/ego progress. Physics, safety and statistics remain at 120 Hz,
control at 60 Hz and normal tactical planning at 10 Hz. Guard checks run at every
control command; cancellation/deadline handling can request an immediate planning
update rather than waiting for the next normal interval.

## Measured lead progress and rear reserve

Added lag is the original lead's measured unwrapped base-circle progress minus
ego progress since attempt start. It does not integrate the requested speed cap
as if it were actual vehicle motion. If that lead disappears, the reference
switches permanently to a conservative maximum-speed bound for the remainder;
it does not silently substitute a new lead identity.

The rear certificate is a bounded runtime guard for the straight-lane attempt,
under the declared scripted-traffic envelope. It checks every measured rear
object whose inflated footprint could overlap the certified ego corridor.
It includes full body dimensions and collision-center offset, curvature,
0.3 m tracking margin, 0.8 m clearance, object yaw allowance of 0.05 rad,
canonical lane radius with 0.1 m radial tolerance, sample-travel padding and
progress slack. A wide adjacent object can enter this corridor and reject the
attempt. Adjacent merge traffic is also checked by the unchanged lane-change
forecast when that planner considers a path.

The initial reserve assumes **zero ego progress for the entire 21-second window**
and the configured global maximum observed rear speed, 20 m/s in this profile.
In base-circle metres, the conservative rear speed is
`20 * 500 / (500 - 0.1)`. Each rear's required center separation is that speed
times the remaining window plus the body, offset, clearance, tracking,
sample-travel and progress margins. It does not assume that a rear car brakes
or that ego achieves a guaranteed recovery acceleration.

The author's reduction of the repaired 135-second snapshot gives 449.3627 m
available center gap, 428.7145 m required and 20.6482 m initial reserve for the
limiting straight-corridor rear. These are calculations from a recorded snapshot,
not a newly observed successful slowdown. The current reserve is recalculated
from fresh tracks at every control command. The original formation's larger
rear gap cannot be reused after membership changes.

## Recovery and explicit failure

Slowing ends on its lag/time cap, a changed formation/setting, or loss of the
ordinary blocked-following condition. Recovery restores only the ordinary base
command. At the 21-second deadline, measured ego speed must be no more than
0.15 m/s below the current lead/relevant-rear reference, and the ordinary target must not
demand a further deficit greater than 0.15 m/s. Otherwise the bounded experiment
fails with `recovery_deadline`; merely removing the cap and continuing
indefinitely is not accepted recovery.

Loss of the rear/tracking certificate, excess or nonfinite added lag, or a
changed geometric configuration terminates the experiment with an explicit
error. An unexpected base stop/fallback while the attempt is active also fails
the experiment before another physics step. It must not silently abandon the
rear reserve and brake indefinitely while scripted backgrounds keep moving.
Explicit requested stop retains the existing coordinated rundown handoff.
The wider ordinary tracking emergency remains authoritative; handoff cannot
clear the attempt before that emergency is evaluated.

Diagnostics identify state, reason, attempt count, fixed profile, start/deadline
ticks, measured speed, cap, original lead, lag/reference mode, rear center gap,
required gap, reserve and the latest event. Saved planner records are decimated
under the presentation evidence profile, so they are not a complete 60 Hz history
of every guard decision. A controlled failure must remain a failed run with its
error and retained artifacts, not a skipped scenario or an implied safe outcome.
The raised error includes a compact finite JSON certificate because the runner
persists the error even when `command()` cannot return its diagnostics. If the
full diagnostic object is nonfinite or unserializable, a minimal finite payload
preserves the original failure reason instead of masking it with formatting.

## Required evidence before accepting the behavior

[`tests/test_live_showcase_gap_creation.py`](../tests/test_live_showcase_gap_creation.py)
contains CPU contract fixtures for bounded search, rear-window refusal, wide
adjacent footprints, low-speed rejection, dwell stability, cap release and
ordinary recovery, new rear admission, tracking violations, removed-lead lag,
configuration changes, time/lag/attempt limits, circular seam behavior, command
expiry and stop/emergency handoffs. Counterfactual states in these fixtures do
not qualify native physical response. The author-recorded focused checkpoint is
**138 passed and 34 subtests passed**, retained in
`outputs/showcase_validation/oct19-review/gap-creation-focused-checks.json`.
The frozen controller source SHA256 is
`7b3778e9635422f7773f63cde5ecee308d5fb4f48a49312363913d245d0d4042`;
the new fixture file SHA256 is
`e67b0a849a462ac72e2a1986169fbb5f0b0489aab1e6da9f42e273de7c444764`.
The complete frozen-source CPU checkpoint passed **1,750 tests**, with **181
optional skips and 116 subtests passed**, in **71.74 seconds**. Its result and
JUnit report are retained under
`outputs/showcase_validation/cpu-gap-20261019T170809Z/`. Independent source review
is clear and recorded in
`outputs/showcase_validation/oct19-review/gap-creation-source-review.json`.
These results apply to the hashes above, on admission checkpoint `396f7ef` plus
the gap-creation changes; they do not imply that a later source revision passed.
Native endpoint `20261019T170934Z-049ddcb3` completed from that frozen source and
passed 33/33 gates. It reached all four population/speed/gap deadlines, completed
11 passes and five lane changes overall, and added eight passes and four lane
changes in the settled 135–180 s tail. Minimum clearance was 1.779726 m, with
zero reported contacts and verified cleanup. Independent statistics/lifecycle
reduction passed 94 checks, and exact deadline targets and all 47 source hashes
matched. [The measured report](showcase-validation-20261019.md#bounded-gap-creation-original-endpoint-passes)
retains the details and limitations. This documentation task did not run tests
or simulation.

The first unchanged native endpoint satisfied all population/speed/gap
deadlines, pause/mailbox checks, physical/scene/process guards and **an additional
pass and lane change in the settled tail**. Repeat
`20261019T171541Z-e2c26f1d` also passed 33/33 native gates and 94 independent
consistency checks. Its configuration, trajectory, statistics, lifecycle and
control-validation files are byte-identical to the original endpoint's files;
all 192 saved planner frames match after per-run episode identity is excluded.
Wall-clock equivalence is not claimed. The legacy
90-second smoke `20261019T171947Z-9f61a18c` passed 33/33 gates and 88 consistency
checks, with zero recorded deliberate attempts. The measured report retains
these runs as distinct records and preserves both earlier endpoint failures.
Its 12,240-row trajectory also matches the earlier admission-only legacy smoke.
The control review corroborates the attempt's 14.4 s total duration and 10.5 s
slowdown, with maximum reconstructed added lag 38.373119 m below the 50 m bound.
The reconstruction uses measured ego motion plus declared background angular
motion and agrees with saved lead diagnostics; full lead tracks at every physics
tick were not retained.
Timing remains a separate result: this run failed strict display qualification
with RTF 0.950207 and lag p95 10.782066 s. The matched display matrix must use one new
frozen source snapshot across its cases; old admission-only runs cannot be
substituted into those pairs. No forecast margin or endpoint threshold is reduced
to make this experiment pass.

Existing failed endpoint evidence and the [measured admission report](showcase-validation-20261019.md)
remain authoritative for completed runs. New native results, screenshots,
source hashes and any failure must be appended as distinct evidence before this
experimental status changes. Training remains outside this milestone.
