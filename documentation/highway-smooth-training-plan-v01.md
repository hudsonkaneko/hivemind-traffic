# Smooth highway control v01 — proposed training plan

**Proposal only. Training is not authorized, and this specification is not
launchable.** First request approval to implement the soft reference and reward
proposal. Learning requires a passing no-update preflight, a resolved configuration
shown to the user, and explicit training approval; approval of preparation alone
does not authorize learning. Keep the frozen 99-field
v05 actor, two actions, 24 critic slots, SUMO motion and existing safety gates.
Smooth acceleration means reducing avoidable sharp changes while making useful
progress; it does not mean constant acceleration or avoiding necessary braking.

The paired experiment will compare an unchanged-objective control with a
combined alignment-and-comfort objective, starting from the same frozen actor.
It tests the combined proposal; it cannot identify each component's individual
effect. The disabled [configuration](../configs/training/highway-smooth-v01.proposed.json)
records the budget, formulas, required implementation and unresolved launch gates.

## Evidence and the reference decision

The [I-110 network correction](i110-merge-blocking-v01.md) removed the prolonged
ramp blockage without changing the actor. Its 244/2,864 interventions (8.52%)
still fail 5%. The saved analytical audit at
`outputs/highway-smooth-control/v01/saved-control-analysis-02.json` matches the
existing target to measured native acceleration within 0.0001 m/s² at all 244
intervention samples: 233 following responses and 11 minor-entry responses.
That supports learning the existing native constraints, not weakening them.

It does **not** establish that the teacher is a desirable trajectory everywhere.
Across all measured samples, mean native-to-target acceleration difference is
0.19085 m/s². The v3 target conservatively limits entry until the 4.5 m visibility
boundary and can jump from braking to acceleration there. The focal car slowed
from about 11.12 to 6.41 m/s before entry and recorded a 58.9257 m/s³ brake-release
jerk. Stronger equality imitation could reinforce this behavior. A train-only,
zero-update comparison was run on training seed 6800019. Both controllers
completed the full horizon and one stable merge with identical demand. The
existing teacher reached zero interventions versus the actor's 17.50%, and
improved mean/p95 jerk (0.1281/0.2886 versus 0.1861/0.4459 m/s³), but its peak
jerk was **64.9999 m/s³**, worse than the actor's 59.9553. It fails the proposed
nominal peak gate below. Results are preserved in
`outputs/highway-smooth-control/v01/reference-check-01/summary.json`; no weights
changed. Therefore, do not increase equality fitting to the existing v3 jump.
The stronger coefficients below apply only to a new consistent soft reference
that passes the preparation gates.

Across the prior pilot's 48 PPO rollouts and 245,903 active decisions, the mean
weighted progress reward was +0.61330 per decision, versus −0.02790 jerk,
−0.03629 feasible-acceleration error and −0.03747 intervention. Jerk cost was
about 4.55% of progress. These are measurements of previous behavior, not an
estimate of the proposed policy's performance.

All 2,025 PPO optimizer steps were gradient-clipped; median global clip factor
was 0.12056. First-minibatch acceleration-imitation actor-gradient norm was
only 0.01141 of PPO at the median; near-limit supervision was 0.08154 of PPO.
These component measurements cover the first accepted minibatch of each update,
not every optimizer step. The critic exceeded the actor's norm on only 360/2,025
actual steps, so the evidence does not support a blanket critic-dominance claim.
Retain the existing weighted gradient, clipping and critic-fit diagnostics.
These totals, definitions and every raw update path/SHA256 are preserved in the
[compact objective audit](evidence/highway-smooth-training-plan-v01/objective-scale-audit.json).
Rebuild it without simulation or optimization by running
`python documentation/evidence/highway-smooth-training-plan-v01/build_objective_audit.py`
with the project's traffic Python from the checkout root.

## Current objective and proposed changes

All formulas below are per active controlled decision at `dt=0.1 s`. Let `u`
be raw requested acceleration, `a` measured native acceleration, `a_ref` the
resolved observation-only reference, `a_max` the unchanged analytical ceiling,
`j=(a_t-a_previous)/dt`, and `d` route distance in metres. The existing actor
maps its normalized output with `u=-1.25+3.25*tanh(z)`, giving −4.5 to +2 m/s².

| Component | Existing weighted term | Proposed combined arm |
| --- | --- | --- |
| Progress | `+1.0 * max(0, delta_d) / 2.68224` | Unchanged |
| Settled merge event | `+5.0` once | Unchanged |
| Passing / route arrival | `+2.0 / +2.0` on events | Unchanged |
| Collision | `-20.0` | Unchanged; safety remains a hard gate |
| Intervention union | `-0.2` per flagged decision | Unchanged definition and coefficient |
| Native longitudinal jerk | `-0.01 * abs(j)` | `-0.03 * H(j / 1 m/s³)` |
| Completed lane change | `-0.1` | Unchanged |
| Reference alignment | `-0.25 * abs(u-a_ref)/4.5` | `-1.0 * abs(u-a_ref)/4.5`, using the new soft reference after preflight |
| Unsafe request | `-0.5 * max(0,u-a_max)/4.5` | Unchanged |
| Cooperation | `+0.1 * new_team_merges / controlled_count` | Unchanged |
| Eight-second stable completion | `+2.0` once | Unchanged |
| Credibly unnecessary reversal | `-0.5` per classified event | Unchanged; route/safety exceptions retained |
| Command/reference consistency | Absent | `-0.01 * H(delta(u-a_ref)/(dt * 1 m/s³))` |

Here `H(x)=0.5*x²` for `abs(x)<=1`, otherwise `abs(x)-0.5`. It has an unbounded
linear tail, not a cap. At jerk 5 m/s³, the jerk cost rises from 0.05 to 0.135;
at the observed 58.9257 m/s³, from 0.58926 to 1.75277. A cap of 10 would have
made that extreme event cheaper and is explicitly rejected. The 1 m/s³ scale
is an engineering normalization for this experiment, not a certified comfort
threshold. Preserve unclipped jerk in every report.

Command consistency penalizes changes in the **error relative to the reference**,
not every change in requested acceleration. Following a legitimately changing
braking reference does not itself produce this error-slew cost. Skip the term
where a consecutive previous request/reference is unavailable; do not invent a
zero previous value at reset or across disappearance. It is a reward term, not
an actuator slew limiter. Native braking and the shield can act immediately.

At 18 m/s, unchanged progress is approximately +0.6711 per step. A 1 m/s²
reference error costs 0.05556 now and would cost 0.22222 at weight 1.0. The
proposed jerk and alignment terms are meaningful relative to progress without
rewarding a stopped, inactive policy. Necessary braking carries no new reward
for being avoided: unsafe requests, native/filter interventions, collision,
speed/headway/TTC constraints and braking-reference alignment remain in force.
Preflight must explicitly compare a required-braking request against holding or
increasing acceleration; smoothing must not override the safety response.

Keep progress as the speed incentive. Do not add an unconditional free-speed or
positive-acceleration bonus. Report legal-speed utilization, task progress and
one-step feasible-speed tracking, using
`v_ref_next=clip(v_before+dt*a_ref,0,legal_limit)` and the measured next speed.
This diagnostic is not a claim that the reference defines an optimal trip speed.

The present optimizer adds normalized-action acceleration MSE and weighted lane
cross entropy with imitation coefficient 0.04; non-stay lane labels have weight
10. Near-limit acceleration MSE is separately weighted 0.1 in SI units over
rows within 1 m/s below the legal limit, including overspeed rows. Candidate
all-state acceleration-imitation weight 5 against the new soft reference would change its effective SI MSE
coefficient from `0.04/3.25²=0.003787` to `0.20/3.25²=0.018935`, while leaving
lane and near-limit coefficients unchanged. **Do not activate that increase
until the reference gate passes.** At a fixed batch its component gradient
scales fivefold; normalized PPO advantages mean reward changes do not imply
proportional changes in the PPO gradient.

## Proposed soft reference and implementation preflight

The next preparation step is a pure current-observation reference, not a change
to native commands, car-following, visibility or the hard shield. These numerical
choices are proposals to test, not validated capabilities:

1. Use the existing 99 fields: measured speed/acceleration, legal limit, current
   own/projected-front and rear gaps, route/entry distance, masks and delivered
   messages. Keep the current 4.5 m minor-entry visibility and 0.1 m stop offset.
2. Separate the **mandatory feasible acceleration ceiling** (existing hard
   analytical guard plus the pinned current-state native ACC/minor-entry
   response equations) from a **soft desired speed**. A conservative desired
   speed is not automatically evidence that every greater request is unsafe.
3. For a known minor entry at distance `d`, look ahead in the existing stop
   envelope using `d_look=max(d-2.0*v,d_floor)`. Set `d_floor=4.5 m` only when
   current projected front/rear gaps meet the unchanged safety requirements;
   otherwise use the 0.1 m stop offset. Set
   `v_entry=discrete_stop_speed(max(0,d_look-0.1))`. This evaluates a known map
   distance earlier; it does not predict future traffic or grant entry early.
4. Take `v_soft` as the minimum of the existing legal-speed buffer, current
   following/target-lane speed reference, ending-lane taper and active entry
   reference. Use `a_des=clip(0.4*(v_soft-v),-4.5,2)`.
5. Recover smoothly from **measured** acceleration:
   `a_ramp=clip(a_des,a_measured-2.0*dt,a_measured+2.0*dt)`, then
   `a_ref=clip(min(a_ramp,a_mandatory),-4.5,2)`. The nominal 2 m/s³ reference
   slew is soft. A stricter mandatory braking ceiling overrides it immediately.
   Log that event; never suppress its jerk or safety accounting.

The new reference changes reward/teacher labels, not the deployed actor's ABI
or its action application. The old-objective arm retains the old v3 labels;
the proposed arm uses this new reference. That is part of the explicitly
combined treatment. Verify that frozen-actor observations, raw actions and
native trajectories remain identical when only the reward/reference version
changes. Do not query native follow-speed getters or future state.

Before learning, compare native, existing-teacher and proposed-reference driving
on predeclared training-only nominal cruise, dense following, passing and entry
fixtures, including the matched seed above and force-braking boundary cases.
Keep complete 80-second traffic accounting. Required braking must override
comfort in danger tests. On the **predeclared nominal fixtures**, require
maximum measured longitudinal jerk **at most 5 m/s³**, zero safety events,
case interventions at most 5%, passing/stable-merge success, cruise ratio at
least .9, and no regression in completed trips. Report startup and later jerk
separately **and together**; reset/startup samples are not silently excluded from
the nominal peak gate. Danger tests retain all jerk measurements but test prompt
safety response, not the nominal comfort ceiling. Also compare approach speed
loss and total travel time: nominal merge speed drawdown may exceed the frozen
actor by at most 0.1 m/s, and mean boundary time across all requested trips may
exceed the existing teacher by at most 5%, including unfinished trips through
the horizon. A smooth but unnecessarily slow teacher fails.
Passing low intervention alone is insufficient. If these proposals fail, retain
the evidence and revise the resolved plan before seeking training approval.

Implementation must add versioned reward terms, previous-reference lifecycle,
raw/filtered/native jerk distributions, merge-phase metrics and immutable
matched schedule state. Test reset/arrival boundaries, required-braking priority,
formula units, target discontinuities and exact budget/resume charging. Run
zero-update weighted-loss/gradient diagnostics on training-only samples. Existing
v05's trainer cannot launch this new proposal schema; no runtime reward or
optimizer code is changed by writing this note.

## Matched training exposure and budget

Use the exact frozen v05 actor SHA256
`fd639c39f0c514256a458b2b85d35baba242125f44dcc3d9542d767eb2dd46ec` for both
arms, with identically initialized fresh critics/optimizers and saved RNG state.
Keep 99 actor inputs, 24 stable critic slots, the 0.1 s control interval and
three-second native lane-change setting. Actor/critic rates remain
0.0001/0.0003, four epochs, minibatch 256, PPO/value clips 0.2, entropy 0.001,
value coefficient 0.5, gradient norm cap 0.5, KL limit 0.02, gamma 0.99 and
GAE lambda 0.95. Reward totals are not separately standardized; active PPO
advantages retain their existing batch normalization.

The old pilot never left competence grade 0. For this matched experiment,
separate *exposure difficulty* from a reported *mastery grade*. Use one saved,
predeclared training-only sequence for both arms, targeting active-decision
shares of cruise/following/passing/merge 20/30/20/30% and difficulty 0/1/2
40/30/30%. Reserve 20% of PPO decisions for difficulty-2 following with 24 total
vehicles, within the following/difficulty shares. Keep foundational tasks in
every cycle. This ensures dense-following exposure without pretending the
competence gates were passed.

Retain totals 8/16/24; controlled fractions .125/.25/.5/.75/1; speed ceilings
12/18/22.352/26.8224 m/s; and none/local/local-with-30%-loss channels. Full-AV
cases use initial traffic and exclude scripted-leader tasks. Other cases retain
sustained inflow. Rotate dimensions independently within eligible strata, using
training seeds 7000000–7000999, and preserve the fixed validation seeds.
Disable adaptive failure resampling for this paired ablation so objectives do
not receive different scenario schedules; never replay old PPO samples.
Save case indices, per-stratum decisions, RNG and pending reservations. Report
actual exposure and shortfalls—50k decisions cannot establish full factorial
coverage. This schedule is common to both arms, so their difference isolates
the objective bundle within the new exposure protocol.

Proposed ceiling: **50,000 active decisions per arm, 100,000 total; 3,600 seconds
wall time total**. Each arm includes at most 2,048 frozen-actor critic-warmup
decisions, leaving at most 47,952 PPO decisions. Failures, interrupted
reservations and warmup count; evaluations count against wall time. Reserve
900 seconds for both final 24-case evaluations. The shared initial 24-case
evaluation and any fresh preparation consume the same wall budget. Split the
remaining pre-reserve learning time equally between arms; never extend it
silently. If either budget prevents complete comparisons, report an incomplete
pilot, not acceptance. No extension or second training seed is automatic.

## Evaluation and decision rules

Evaluate the starting actor once and both endpoints on **all 24 fixed validation
rows**, with every I-110 reference using the corrected network and all other
case identities, seeds, demand, speed/channel/population settings unchanged.
This is a new environment treatment; preserve the original rejected 24-case
results separately. Use deterministic frozen inference, full 80-second traffic
accounting and native-tail recordings. Validation may compare endpoints but
must never become demonstrations, teacher-fitting data or retry scenarios.

Keep every existing acceptance gate: zero collisions, teleports, overspeed,
measured emergency braking and native emergency warnings; at most 5%
interventions in **each** case; cruise speed ratio at least .9; at least one
pass per passing case; at least 90% stable merges per merge case; full valid
horizons. Keep the eight-second window, 1.5 s headway, 3 s TTC, 5% window
interventions and .25 progress-ratio gates. Pending/early-ended windows stay
pending. Audit all warnings and every vehicle's stops of at least two seconds.

Add smoothness reporting for each car, task, case, merge approach, commanded
lateral maneuver and post-merge window: mean/RMS/p95/peak absolute requested,
filtered and realized longitudinal jerk; acceleration total variation;
reference error; speed utilization; travel progress/completion; and stopped
time. Use consecutive measured samples only, retain denominator counts and
report safety-constrained intervals separately without deleting them.

Define **observed merge slowdown** as the largest observed speed loss from a
peak to a later trough in the fixed window from five seconds before first
junction entry through settled through-lane completion plus eight seconds:
`max_{s<t}(max(0,v_s-v_t))`. Report m/s, fraction of the corresponding preceding
peak, both timestamps, available window coverage and legal-limit changes.
Also report approach, native join, lateral maneuver and post-merge phases
separately so late approach braking is not hidden. If there is no entry or
completion, or insufficient history, mark coverage incomplete and report the
available ramp trajectory rather than reporting zero slowdown.
Slowdown alone is not proof of bad driving; label whether the contemporaneous
reference required braking or the legal limit fell, with unknown classifications
retained. Do not reward suppressing a necessary slowdown.

The combined arm is a useful comfort improvement only if all existing safety/
task gates pass, median paired-case p95 realized jerk improves by at least 10%
against the trained control, no task's mean realized jerk worsens by more than
5%, task mean speed ratio falls by no more than .02, and every predeclared
nominal approach window has peak measured jerk at most 5 m/s³. Freeze those
window identities before evaluation; do not exclude adverse samples afterward.
Require no reduction
in total stable merges, passes or completed trips relative to the control.
Report any tradeoff rather than changing these thresholds after seeing results.
These are proposed engineering comparison criteria, not human-comfort or
physical-vehicle certification. A single training seed and 24 development
scenarios do not establish statistical generalization or held-out acceptance.

The present frozen actor remains at 8.52% on the corrected I-110 case. Reward
changes alone cannot verify that it meets 5%; that requires the approved,
implemented and completed learning comparison. No training has been launched.
