# Highway merge training plan v16: merges that look like driving, on the 3D road

Written by the main Claude session on November 28, 2026, on branch
`2026-11-17-claude-policy-candidate` after commit `a2aaed4`.

**Status: proposed.** Nothing in this plan has been built, run or charged, and nothing is
authorized beyond writing it. Each phase below needs your go-ahead.

**User request (November 28, 2026):** "Yes, plan v16", in answer to "Move the remaining work into a
v16 plan?". In the same message: "lookat how we previosuly fixed the merge speed. would that method
help what we are currently doing?" That question is answered in
[The earlier merge-speed fix](#the-earlier-merge-speed-fix-would-it-help-now) below.

v16 keeps v15's rules-first design and its tools. It changes what counts as success, and it takes
on the three things v15 left open: the slow merges into traffic, physics on the 3D road, and our
own cars making room for each other.

## Where v15 left off

All from [plan v15](highway-merge-training-plan-v15.md) and the shared ledger.

| Area | Result | Kind |
|---|---|---|
| SUMO, development rows (70) | The policy and rule expert v3 pass 62. Round 16 passes 36 | Re-baselined gates |
| SUMO, held-out rows (42, run once) | The policy passes 34 (81%, 67–90%). Round 16 passes 25 (60%). +21.4 points, p = 0.012 | Re-baselined gates |
| Learning | The policy is an exact copy of expert v3. No RL was trained: the Phase 4 probe found room on only 2 of the 8 development rows the policy fails | — |
| Merging into traffic (12 ramp cars) | A median 24.4 s to merge, at 24 mph, crawling for about 16 s. Round 16: 6.2 s at 43 mph, but 2 cars never merged and 4 of the 9 rows had SUMO emergencies | Development rows, matched traffic |
| Why the merges are slow | On 6 of the 12 cars, no gap passes the gap rule until the traffic has passed, for any driver. On the five short ramps, 7–10 s below 11 mph is forced | Offline, from saved SUMO records |
| The I-110 merge case | No driver passes it at any held-out seed | Re-baselined gates |
| Physics (PhysX on the LA model) | No row passes. Controller C2 halves the failed gates on row 11 and on the southbound corridor | Physics, not a replay |
| Coordination | "Open a gap for a merging car we control" was in the plan but never built | — |
| Study time | 1,414.8 s left of 64,700 | Ledger |

**The v15 code that v16 starts from:**
- Expert v3 (`traffic/v15_rule_expert.py`) and the policy `l2:bc7c050f6d3da3ce`.
- Expert v4 is kept in the code but not used.
- The movement checks M1–M4 (`traffic/v15_movement.py`) are reported, not used as pass/fail gates.
- C2 is the default physics controller.
- The spawn-speed fix is in place but not yet confirmed by a drive.

## The earlier merge-speed fix: would it help now?

### What fixed the merge speed before (v11, November 17)

There were two parts. Both are in [plan v11](highway-merge-training-plan-v11.md).

1. **The merge-speed rule (rounds 13–14).**
   - **What it does:** a car that must change lanes to merge may start the lane change only after
     it has matched the speed target, within 2 m/s, for 40 decisions in a row (4 s). Near the lane
     end the rule lets go, so it can delay a merge but never prevent one.
   - **Why it was needed:** the round-9 policy merged on the first decision it was allowed. That
     was at 29 mph into 50–55 mph traffic, 0.1 s after entering an auxiliary lane about 800 m long.
   - **What it did, with no training:**
     - the 120 m ramp case's speed match went from 0.461 to 0.938, with the lane change at 12.4 s
       and 49 mph instead of 3.2 s and 29 mph;
     - the 4-lane 180 m ramp case went from 0.369 to 0.944;
     - gates went from 3 of 8 to 5 of 8.
   - **Where it failed:** the I-110 merge, whose ending lane was about 360 m long. The rule blocked
     the only gap, and the car stopped at the lane end.
2. **The scripted merge approach (probes 01–07).**
   - **What it does:** while a merge is pending, the car tracks 0.7–0.9 m/s under the speed target,
     then hands back after the lateral merge.
   - **Why under the target:** a faster car was ahead in the target lane. Waiting a little slower
     lets the gap behind it open in time.
   - **What it did:** probe 07 qualified all four merging cars, including I-110 (0.994), on 1 seed
     per case.

**Both work the same way.** The car drives alongside the traffic in a lane of its own, builds up
speed, and slides into a gap at traffic speed.

### Is it in v15 already? Yes, for lane-change merges

- **The rule:** v15's environment keeps it in the lane mask, with 40 decisions, a 15 s release and a
  150 m minimum. This is in `environments/highway_merge_v15_env.py` (`_merge_speed_gate`) and in the
  round-24 settings v15 runs with. Expert v3 requests a merging lane change only where that mask
  allows it.
- **Where it helps:** on the three ramp cases with an auxiliary lane, expert v3 passes 10 of 15
  development rows on round 24's gates (4, 2 and 4 of 5). Round 16 passes 8.
- **Not carried over:** the scripted approach is not in v15. Expert v3 follows its own target
  instead.

### Would it help the slow merges? Not on today's roads

- **The slow merges aren't lane changes.** All 12 cars are on the LA ramps, where SUMO joins the
  ramp to the right mainline lane at a yield line. There's no added lane: the car crosses a
  junction, and the lane mask never acts.
- **The model has no acceleration lane.** Plan v12 (round 19) built these roads exactly as the 3D
  model has them: "The model has no acceleration lane, so neither do these roads." So the yield
  line comes from the model itself.
- **There's no room to match speed.** On the five short ramps the car starts 42–50 m from the line
  at about 29 mph.
  - The gaps in the traffic need 40–69 mph at the line.
  - The car can reach at most 36–40 mph by the first gap.
  - Holding it back until its speed matches would only make it wait longer.
- **The scripted approach needs the same thing:** a lane alongside the traffic, where gaps slide
  past while the car waits at nearly traffic speed. At a yield line the car can only wait at the
  line.

### Where it would help

- **With an acceleration lane on the LA ramps.**
  - **The same test once had one.** Round 18 ran on an invented 250 m auxiliary lane. Round 19 ran
    on the model's roads with none.
  - **Same policy and cases, different road:** both validated the same round-16 policy on the same
    14 cases. The pooled merge-speed match was 0.567 with the invented lane and 0.381 without.
    - On the model's roads, the single-car merges had 35–41% interventions and peak jerk above
      50 m/s³: the car braked to the line, waited, and pulled away from a stop.
    - Plan v12's verdict: "The policy has only ever merged from an acceleration lane."
  - **Not a clean comparison:** the speed limits also changed between those rounds, from 60 and
    30 mph to 65 and 45 mph.
  - **An expectation, not tested:** with an acceleration lane, v15's existing merge-speed rule would
    apply as it does on the ramp cases.
  - **The cost:** an acceleration lane means changing the 3D model as well as SUMO's network, so
    physics drives stay on the same road. You have ruled out changing the model so far (decision 18
    in v15). It's your call: see decision v16-2.
- **On the I-110 merge case.**
  - **Why it's a candidate:** it has a lane where the car waits beside the traffic before the lane
    ends. That's where the scripted approach qualified the car in v11, at 1 seed.
  - **Today:** no v15 driver passes the case at any held-out seed.
  - **The plan:** Phase 0a tests the approach offline first.

## Goals

1. **Merges that look like sensible driving.** Movement counts alongside the safety, comfort and
   merge-quality gates: no hard braking on an open road, no long crawl, no speed-up-then-brake, and
   merge speed close to the traffic's.
2. **Driving on the 3D road in physics.** Rows that pass in SUMO should pass in PhysX.
3. **Our cars make room for each other.** A mainline car we control eases off to open a gap for a
   merging car we control.
4. **Learning only where the rules leave room.** v15 showed that the rules alone set the score.

## Kept from v15

- **The controller:** rules first. A rule executor drives every decision, and a learned layer
  chooses the maneuver. Each new behaviour is written as a rule, tested, then imitated.
- **Gates:** each gate is checked against the rule expert before anything trains against it.
- **Evaluation:**
  - development rows for tuning;
  - new held-out rows, run once at the end (v15's 42 have been used);
  - Wilson intervals and paired comparisons on matched rows;
  - results reported by ramp situation (clear, traffic, blocked, I-110).
- **Judging the driving:** by the actual movement (speed traces and frames), not only the numbers.
  Every video is labelled physics or kinematic replay.
- **Tools:**
  - libsumo, with at most 16 workers at below-normal priority;
  - the shared runner module `experiments/v15_runner.py`;
  - slim committed configs.
- **Unchanged unless you decide otherwise:** the road model, the 3D model and the car's provisional
  physics settings.

## Phases

### Phase 0: offline checks (uncharged, no simulation)

From saved records and code only.
- **0a, the I-110 merge case:**
  - Why does expert v3 fail it at every held-out seed?
  - Would the v11 scripted approach (0.7–0.9 m/s under the target while waiting) change that?
  - The test: an open-loop replay on the saved records, with its caveat stated.
- **0b, gap-opening:** on rows 8, 9, 22 and 37, the four development rows where our own cars are in
  the traffic stream:
  - how much our mainline car would have to slow to open a gap that passes the gap rule;
  - what that does to its own movement (M1, M4) and to the cars behind it.
- **0c, movement checks as gates:**
  - Score M1–M4 on the 70 development rows for expert v3, from the saved records.
  - Propose limits and any exemption where the road forces the movement. For example, M2 can't be
    met on the short yield ramps.
- **0d, what an acceleration lane would take:**
  - Where it would go and how long: drawn from v15's ramp cases and from round 18's 250 m lane, not
    invented.
  - What changes in SUMO's network and in the 3D model.
  - Which recorded results it would invalidate.
  - Nothing is changed.

**Output:** a section in this plan with the results and the sizes of the phases below.

### Phase 1: the road (depends on decision v16-2)

- **Option A, keep the model's roads:** nothing changes. On the short yield ramps, the wait is set by
  the road and the gap rule.
- **Option B, add an acceleration lane to the LA ramps,** in SUMO's network and the 3D model alike.
  - Every SUMO baseline (expert v3, the policy, round 16) is re-run on the new roads.
  - Recorded results on the old roads stay as they were, labelled.
  - **Cost:** about 238 s of SUMO time, the side session's estimate from v15's 405,397 recorded
    decisions. The model work isn't sized yet.

### Phase 2: rules (expert v5)

- **Gap-opening:** our mainline car eases off within the comfort limits to open a gap for our
  merging car. It's a rule first, used only where Phase 0b shows it helps.
- **The I-110 approach:** only if Phase 0a supports it.
- **Testing:** an offline replay first, then a SUMO check on the 70 development rows.
- **Cost:** about 105 s per driver, from v15's Phase 2 runs (90–112 s each).

### Phase 3: imitation (only if the learned layer's choices change)

- **Why it would be needed:** gap-opening is a maneuver the learned layer would choose, so the
  policy would need to imitate it.
- **The method:** v15's, behaviour cloning then DAgger.
- **Cost:** v15's Phase 3 took 1,690.0 s. Starting from v15's policy should take less; it will be
  sized after Phase 2.

### Phase 4: physics

- **The torque-limit rule** (decision 16 in v15): a physics-only rule for decisions where the car is
  at its torque limit, tested on the 5 northbound rows.
  - At most 400 s.
  - It also confirms the spawn-speed fix.
- **More rows:** the drivers that pass in SUMO, on matched rows. Each drive costs about 46 s (the
  physics gap's 10 drives took 462.7 s).
- **The car's physics settings** stay provisional unless you decide otherwise (decision v16-6).

### Phase 5: held-out check

- **What:** new held-out rows, built for v16 and run once, for the final drivers and round 16.
- **Cost:** v15's held-out check took 110.8 s for three drivers on 42 rows, plus building the rows.

## Budget

- **Left:** 1,414.8 s of study time.
- **Rough total for Phases 1–5:**

  | Item | Cost |
  |---|---|
  | Option B's re-run | 238 s |
  | Phase 2, two drivers | about 210 s |
  | Phase 4's rule | 400 s |
  | A held-out check | about 110 s |
  | Phase 3, if needed | up to 1,690 s |

- **So a raise is likely needed** if Phase 3 runs. That's your decision (v16-5), sized after
  Phase 0. Phase 0 is uncharged.

## Decisions for you

- **v16-1, Phase 0:** run the four offline checks, uncharged (recommended).
- **v16-2, the road:**
  - **Option A:** keep the model's roads.
  - **Option B:** add an acceleration lane to the LA ramps in both SUMO and the 3D model.
  - If you're willing to change the model, B is the change most likely to fix the slow merges: the
    merge-speed method is already built for it. Otherwise A, and the waits on the short ramps stay.
  - You can decide now or after Phase 0d.
- **v16-3, gap-opening:** as a rule first, then imitated (recommended), or learned directly.
- **v16-4, movement checks:** keep them as reported checks, or make them gates once Phase 0c shows
  that expert v3 can meet them where the road allows.
- **v16-5, budget:** a raise for Phases 1–5, sized after Phase 0.
- **v16-6, the car's physics settings:** keep them provisional (as now), or measure them.

**Separate from v16, each needing your approval:**
- merging the v15 branch to main;
- sending the portfolio session a caveat on the slow merges. Its cause would be named as the
  yield-line ramps and the gap rule, not the merge logic.
