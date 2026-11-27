# Highway merge plan v15: rules first, learning on top

Proposed by Claude on November 25, 2026, after v14 round 24. Every phase that charges the
shared ledger needs the user's approval of its resolved configuration first, as before.

**Status (November 27, 2026):** the user decided all seven questions at the end of this plan.
- **Phase 0** (tools only, no training, nothing charged) passed its exit check; see
  [Phase 0 result](#phase-0-result-november-25).
- **Phase 1** (baselines, evaluation only, charged) ran and met its exit check; see
  [Phase 1 result](#phase-1-result-november-25).
- **Phase 2** (the rule expert, evaluation only, charged; the user approved up to 5 runs) met its
  exit check in 3 runs; see [Phase 2 result](#phase-2-result-november-26).
  - The rule expert v3 passes 54 of 70 development rows under round 24's gates and 62 of 70 under
    the gate re-baseline, now frozen. Round 16 passes 33 and 36.
- **Phase 3** (imitation; the user approved the recommended size) met its exit check, by my reading;
  see [Phase 3 result](#phase-3-result-november-26).
  - The learned Level 2 passes 60 of 70 development rows re-baselined (51–52 under round 24's gates)
    at each of 3 training seeds. The expert passes 62 (54).
  - **After the relabel fix** (the gap labelled 0; you approved it on November 26), one refit passes
    62 (54), the expert's numbers, on the same rows; see
    [The relabel step](#the-relabel-step-november-26). It ran at one training seed only.
  - No learner showed a new kind of failure.
  - Only Level 2 was trained, by imitation; Level 1, the rule executor, is unchanged.
- **Phase 4 step A** (you approved it on November 26) ran in full; see
  [Phase 4 step A result](#phase-4-step-a-result-november-26).
  - **The seed check:** the relabel configuration scores 62 (54) at all 3 fit seeds.
  - **The probe's first batch** stopped at its stop rule: its slower variants stopped on a free road
    on 3 rows.
  - **Its second batch** ran after you approved it, with no stop-rule event.
  - **What it found:** room on 2 of the 8 failing rows, 9 and 37. Row 37 also passes the recorded
    gates, with a constant +1 or +2 m/s offset.
- **Step B is skipped** (you decided on November 26: "Yes, run the check").
- **The +1 offset check** (you approved it on November 26) ran on all 70 development rows; see
  [Phase 4 offset check](#phase-4-offset-check-november-26).
  - It passes 61 of 70 re-baselined and 54 recorded. It gains row 37 but loses rows 48 and 50, both
    on headway.
  - So it doesn't join the held-out check, by the reading set before the run.
- **Stage 5** (you approved the recommended size on November 27: "Approve recommended"; decision 10):
  the held-out check and the physics drives, with the Phase 3 policy as the only candidate; see
  [Stage 5 proposal](#stage-5-proposal-november-26-the-held-out-check-and-physics-drives-for-your-approval).
  - **Part 1, the held-out check, ran** (SUMO); see
    [Stage 5 part 1](#stage-5-part-1-november-27-the-held-out-check).
    - The policy has the same verdict as expert v3 on every held-out row: 34 of 42 re-baselined and
      29 recorded.
    - Round 16 passes 25 and 24.
    - By the reading set before the run, the policy beats v14's best: +21.4 points re-baselined,
      with a 95% interval of +6.3 to +35.5.
    - By the same reading, it doesn't "hold up as well as its teacher". The only reason is a stable
      merge failure on two rows, a failure it never had on the development rows. Expert v3 fails
      the same two rows the same way.
  - **Part 2, the physics drives, ran** (PhysX, not a replay); see
    [Stage 5 part 2](#stage-5-part-2-november-27-the-physics-drives).
    - On development row 11, expert v3 and the policy drove identically.
    - Both fail the row under physics, though both pass it in SUMO. The stable merge fails on its
      intervention fraction: PhysX's realized acceleration differs from the command on the climb
      and crest at the merge.
    - No collision. Charged 176.2 s of the 500 s cap.
  - **Stage 5 is complete.**
- **The physics gap** (you chose "Physics gap first" on November 27): proposed, approved ("Yes, as
  recommended") and run (PhysX, not a replay); see [Physics gap result](#physics-gap-result-november-27-c1-and-c2-in-physx).
  - **The diagnosis:** all four gate failures on row 11 came from the two command ramps, three sharp
    grade breaks in the road model, and the controller's learned force bias.
  - **The fix and the pick:** controller-only versions C1 and C2; C2 was picked on row 11 by the rule
    set before the run.
  - **What C2 changed:** it halves the failed gates on row 11 (2 per gate set, not 4) and on the
    southbound corridor (12 per set over the 5 rows, not 20). It removes every overspeed and
    torque-limit decision, and cuts the realization flags from 150 to 67.
  - **No row passes** under either gate set, with either controller.
    - The stable merge still fails on every row: 5 to 9 of the 81 decisions after the merge are
      interventions, against at most 4. Merge quality fails with it.
    - By the reading set before the run, the gap is not closed on row 11 and the corridor is "not
      improved".
  - **Charged:** 462.7 s of the 850 s cap; no collision.
- **The ramp driving** (you asked on November 27 whether success is judged by numbers or by the
  car's movement, and chose "Yes, driving first"): written, nothing run; see
  [Ramp driving](#ramp-driving-november-27-the-slow-merges-for-your-decision).
  - **The saved SUMO records:** the policy drives exactly like expert v3.
    - **Where mainline cars are coming** (12 of the 31 ramp cars on the development rows), it brakes
      early and crawls: a median of 16.1 s below 5 m/s. It merges at a median of 24.4 s, at 10.9 m/s.
    - **Round 16 merges at 6.2 s at 19.4 m/s,** but with more SUMO emergencies.
    - **On a clear ramp** the two drive alike.
    - **No gate looks at this movement.**
  - **Recommended** (decisions 13 to 15):
    - movement checks M1, M2 and M4, reported from now on;
    - a Level 1 change (expert v4), written and replayed offline first, then a 70-row SUMO re-check
      (about 150 s, no raise).
- **The options for the physics gap** (you chose "Write up the options" on November 27): written,
  nothing run; see [Physics gap options](#physics-gap-options-november-27-what-next-for-your-decision).
  - **Recommended:** C2 as the default physics controller, and a physics-only torque-limit rule for
    realization interventions, tested on the 5 northbound rows (decisions 16 to 20).

It replaces the approach of v06–v14: PPO fine-tuning of one network that outputs an
acceleration every decision, with reward and gate patches after each round. Their evidence
is kept unchanged.

**User request (November 25, 2026):** "make like a new plan thats basically completely
revised and different from previous plans. even think about like tools/ training methods
that shoudl be used instead of the ones we have if they aren't the most optimal. use things
from the video that you analyzed as well".

The video is "Simulating Tesla FSD in C++" (summarized in the November 19 video review);
times below link to it as `m:ss`. Its numbers are the creator's own, not reproduced here.

## Why a new plan

What rounds 11–24 showed (all SUMO; details in the [v14 plan](highway-merge-training-plan-v14.md)
and earlier plans):

1. **Nothing has beaten round 16 since November 17.** No round since has been promoted.
   Round 16 itself was a supervised fit (shield projection from round 9), not PPO.
2. **PPO fine-tuning keeps falling into a braking loop.** Rounds 20, 22, 23 and 24 all ended
   with the southbound LA merges braking to a stop after a few PPO updates. Only round 21
   escaped, and it gained no gate.
3. **The policy is mostly a feedback loop on its own last request.** After round 21, the first
   layer's weights for the car's own acceleration and previous request have norms 2.85 and
   1.98. Its four target-speed inputs have 0.02–0.06. So reward changes to the target
   (rounds 22 and 23) reach it only through a tiny weight nudge. The diagnosis after
   round 24 found the same in merge states: the starting actor works like a latch, and its
   request hardly depends on speed. Its 3 updates in round 24 made stopped merging cars
   stay stopped, and the shield-consistency loss, whose ceiling includes the drive limit
   since round 20, pushes those requests down.
4. **Results hinge on tiny changes.** Training moved the weights 0.5–2.5% in every round.
   Case 9 passes only within a 0.15 m/s² window of acceleration request. An offline
   settling-point check on case 9's recorded states matched its result at all 13 saved
   checkpoints of rounds 20–24.
5. **One seed per case overstates the policy.** On 5 other seeds per case the round-16
   policy passes about 4 of 14 cases, not 8. With 3 seeds (round 24) it passes 19 of 42
   rows, and no merge case passes at all three.
6. **One training run per configuration can't separate a change from chance.** Each round's
   training differs from its first episode (round 20's critic warm-up had 33,763 decisions,
   round 21's 32,312).
7. **A gate was set that no driver could pass.** The merge-quality target in the four LA
   cases couldn't be followed within the comfort and drive limits (fixed in round 23).
8. **Rules already beat the policy on the LA emergencies.** They are cut-ins by our merging
   cars. A scripted car that follows the plan's yields passed every gate in 7 of 12
   one-car LA runs with no emergencies; the round-16 policy passed in 0 of 12, with 4
   emergencies.
9. **The tools are slow and serial.** Training runs one SUMO process over TraCI's socket:
   round 21 took 2,127 s for 193,714 decisions (about 90 per second, validations included).
   A 42-row validation takes about 413 s. This machine has 32 hardware threads, an RTX 5070
   and 62 GB of RAM. The network is small (two 128-unit layers) and trains on CPU PyTorch,
   so the simulator, not the network, is the bottleneck.
10. **The code has grown in layers.** The v14 environment builds on v12, v06, v05 and
    earlier, and one decision's reward has 24 terms.

The video reached the same place on a smaller problem. Its rules (116 lines) needed 3
interventions on two unused drives; its network (174,824 weights) needed 42 (12:29). It then
taught the network from the rules with DAgger (11:19). It found its bugs by asking what the
network was looking at (8:57), and picked models by driving them, not by their error
score (11:40).

## What changes

| Area | v06–v14 | v15 |
|---|---|---|
| Controller | One network outputs an acceleration every decision, plus lane intent; environment limits filter it | A rule executor drives every decision; a learned policy chooses the maneuver: which gap, commit or yield, target speed, lane change |
| How learning starts | Warm start from round 16, then PPO | Qualify a rule expert, imitate it (behaviour cloning, then DAgger), then PPO anchored to the imitation policy |
| Reward | 24 terms, patched after failures | A few event terms at the maneuver level |
| Validation | 14 cases at 1 seed (3 in round 24), one training run | 14 cases × 5 seeds, reserved held-out seeds, 3 training seeds per configuration, confidence intervals, paired comparisons |
| Headline metric | Per-decision intervention fraction | Events: stuck, cut-in, emergency, collision. Per-decision fractions kept as secondary |
| Gates | Set first, then trained against | Each gate checked with the rule expert before any training against it |
| Checkpoint screening | Full validation only | Free offline screens on every checkpoint, then validation |
| Simulator coupling | TraCI, one process | libsumo, one simulation per process, many processes in parallel |
| Code | v14 environment on top of 10 earlier versions | A flat v15 environment that reuses scenarios, road networks, limits and messages, with new rewards |

## The controller: two levels, always on

The user's direction from November 19 stands: one controller on the car at all times, no
hand-back to SUMO, with SUMO's checks off for the controlled cars. In v15 that one controller
has two levels. The executor plays the part that the environment's limits play today, so the
car is never swapped to another controller.

### Level 1: the rule executor (every decision)

- **Speed and following: IDM** (0:58; Treiber, Hennecke and Helbing, 2000). It tracks a target
  speed behind a chosen leader: the car that defines the gap the car is aiming for. It keeps
  today's limits: speed limit, grade-limited drive (the v14 drive limit), comfort rate limit,
  jerk limit and lane end.
- **Commit or yield at the merge: the dilemma-zone rule** (3:00). The video stops for a yellow
  light only if that needs no more than 3.5 m/s² of braking, otherwise it goes. Here: yield to
  the next mainline car only if that needs no more than a set braking level (start at
  3.5 m/s², then tune); otherwise commit. Once committed, the car re-decides only when a
  safety check fails. That removes the half-braking that has stopped our southbound cars.
- **Gap acceptance: MOBIL's safety test** (Kesting, Treiber and Helbing, 2007), in place of the
  video's 5 s time gap for left turns (3:30). Take a gap only if the car that would follow us
  in the target lane needs to brake no harder than a safe level. Set at 3 m/s², the threshold
  of round 24's cut-in charge, this makes a cut-in impossible by rule instead of penalizing
  it afterwards.
- **Lane changes: MOBIL** with a politeness factor, for the cases that need one (0 and 4).
  The scripted follower couldn't change lanes, which is why it skipped those two.
- **Stuck rule** (3:58). The video's fix for deadlock: if nobody has moved for 8 s, go. Here:
  if the car is stopped on a free road for more than a set time, release the brake and pull
  away. This changes what the car does, so it's **your decision**; the alternative is to only
  count it.
- **Ramp curvature bound** (2:09). The video keeps sideways acceleration under 2.5 m/s²,
  looking 40 m ahead. Here: cap target speeds on the curved LA ramps the same way. Whether
  the map preview already includes curvature has not been checked.

### Level 2: the learned maneuver policy (every 0.5–1 s, or at events)

- **Actions:**
  - which gap to aim for: behind or ahead of one of the nearest three mainline cars;
  - commit now, or wait;
  - a target-speed offset of a few m/s;
  - lane-change intent;
  - for mainline cars we control: open a gap for a merging car we control.
- **Inputs:** today's local fields and V2V messages, plus the executor's state (committed or
  not, chosen gap). The car's own previous request and acceleration are left out as direct
  inputs, since they drive the feedback loop.
- **A rule version of Level 2** picks the gap by MOBIL's incentive and commits by the
  dilemma rule. It is both the expert to imitate and the baseline every learned policy must
  beat.

Why this split:
- **A braking loop like rounds 20–24 is much harder to fall into.** On a free road the
  executor always accelerates toward the target, whatever the policy chose a second earlier.
- **Learning goes where rules are weakest:** which gap to take and how our cars cooperate.
  That is the multi-car coordination the project is about.
- **The learned part can't make an unsafe cut-in or a hard stop by itself.** The executor's
  safety tests are part of every action.

## Training method

### Stage 1: qualify a rule expert (no learning)

- Build Level 1 with the rule Level 2, and run it on the v15 benchmark (below).
- **Targets:**
  - no collisions;
  - no cut-in emergencies in the one-car LA cases;
  - no free-road stops;
  - a higher pass rate than the round-16 policy on the same 70 rows.
- **When the expert fails a gate, find out whether the gate or the driver is wrong before
  any learning**, as with the merge-quality target. Look at the map and scene at the failure
  spot first. In the video, a car stuck at one spot was staring at a hole in the world, an
  unbuilt gap at the city's edge (9:27).

### Stage 2: behaviour cloning with perturbations

- **Data:** the expert drives the 20 training rows on many seeds. Each decision records the
  observation and the expert's Level-2 choice.
- **Perturbations** (5:08 and 11:31). The video nudged the steering off course with a chance
  of 0.02 per 50 ms tick, for 0.6 + 0.8·u s, then let the rules recover; the recovery became
  the label. Here:
  - start some episodes slow, stopped or braking near the merge;
  - add random speed offsets and late commits;
  - label the expert's recovery.

  Today the merging cars start at speed, so recovery from a stop is practised only after the
  policy stops itself.
- **Balance the data** (8:14, the inertia problem; Codevilla et al., 2019). In the video the
  car stood still in 13% of frames, and a copying network learned that a stopped car stays
  stopped. Down-weight samples where a stopped car stays stopped, and keep every
  "stopped on a free road, so go" label.
- **Split by seed.** Watch the held-back error for memorising (10:39), but choose models only
  by closed-loop driving (11:40): in the video two networks with the same held-back error
  drove very differently.

### Stage 3: DAgger

- The learner drives with probability 0.7 and the expert labels every state it reaches. That
  is the video's setting (11:31) and Ross, Gordon and Bagnell's method (2011).
- Add the labels, retrain, and repeat for 3–5 iterations.
- Stop when the closed-loop pass rate on the development rows stops rising.
- **Target:** close to the expert's pass rate, with no new kind of failure.

### Stage 4: anchored multi-agent RL (only where the expert is weak)

- **MAPPO on Level 2 only.** It keeps the existing centralized critic, plus a behaviour-cloning
  or KL term toward the Stage-3 policy. The term decays slowly, so training can't drift far
  in a few updates, as it did in rounds 20–24.
- **Reward:** a few event terms, each logged:
  - progress or flow, and merges completed;
  - cut-in events, emergencies and collisions;
  - stuck events and comfort events (jerk exceedances);
  - a cooperation term for gaps opened.
- **3 training seeds per configuration,** with today's stop rules (free-road stops, collision
  regression, speed-match floor) and the screens below.
- **A change is accepted only if it beats the Stage-3 policy across seeds on the same rows,**
  and no safety event gets worse.

### Stage 5: physics and held-out evaluation

- Drive the expert and the best policy on the PhysX car on the LA model, and label each video
  as physics or kinematic replay.
- Run the reserved held-out seeds once, for the final candidate only, and only with your
  authorization.

## Evaluation and tools

### The v15 benchmark

- **Rows:** the 14 v14 validation cases at 5 seeds each (70 rows) for development. 3 more
  seeds per case are reserved as held-out and never used to choose anything.
- **Report:**
  - each case's pass rate, with a 95% Wilson interval;
  - the pooled pass rate;
  - the paired difference from the rule expert and from the round-16 policy, on the same rows.
- **Background traffic:** SUMO's default Krauss drivers, plus an IDM variant (0:58) as a
  robustness check.
  **Correction (Phase 0):** the background cars are already IDM. The v14 demand writer gives
  each one of three IDM profiles (`traffic/highway_v05_demand.py`, `PROFILES`), as the routes
  files of round 24's validation show; only our own cars' native model is Krauss. So the
  recorded traffic is the IDM case, and the robustness variant is Krauss: the same three
  profiles with SUMO's Krauss model (`experiments/v15_pool.py`, background `krauss`).
- **Events first** (10:00). The video counted one stuck car up to 100 times as interventions
  until it counted events. Here each stop counts once, as do each cut-in, emergency,
  collision and override episode. Per-decision fractions stay as secondary numbers.

### Free checks on every checkpoint

- **Settling-point screen:** where each car's request settles on recorded southbound ramp
  states. It matched case 9 at all 13 checkpoints of rounds 20–24, including at round 24's
  new seed.
- **Per-input sensitivity** (8:57): the gradient of each output with respect to each input,
  over recorded states. It flags a dominant self-feedback input or an unused one, as the
  video's saliency map found the stuck car's cause.
- **Training versus validation gap** (10:39).

### Tools to change

| Now | Change to | Why | Check first |
|---|---|---|---|
| TraCI over a socket | libsumo (`pip install libsumo`, pinned to SUMO 1.27.1), switched in with `LIBSUMO_AS_TRACI` | SUMO's docs describe it as "more efficient coupling without the need for socket communication"; they give no speedup figure | It isn't installed in `.venv-traffic` today. Measure the speed, and confirm identical results on fixed seeds. sumo-gui doesn't work with libsumo on Windows, so keep TraCI for the GUI |
| One SUMO process | 16–24 worker processes, one simulation each, for rollouts and validation | 32 hardware threads; about 90 decisions per second now. SUMO's docs say parallel libsumo instances need Python's multiprocessing | How the ledger charges parallel wall time (your decision); identical results per seed |
| PPO from a warm start as the only trainer | A plain-PyTorch supervised trainer for cloning and DAgger; MAPPO kept for Stage 4 | Imitation is supervised learning: stable and cheap. It's the method behind round 16 and the video | — |
| The v14 environment on top of v05–v12 | A flat v15 environment reusing the scenarios, road networks, limits and messages | 24 stacked reward terms are hard to reason about | Parity test: with the same driver, v15 reproduces v14's executions |
| One training run per configuration | 3 training seeds | Separate a change from chance | Budget |

**Kept:**
- SUMO as the traffic simulator;
- the LA I-10/I-110 model, unmodified;
- Isaac Sim with PhysX for physics checks;
- CPU PyTorch, since the networks are small.

**Not recommended now:**
- **GPU-batched driving simulators such as GPUDrive or Waymax.** They are built around their
  own map formats, so our SUMO networks and the LA model would have to be ported. The
  November GPUDrive v05 study also drove onto an unconnected road.
- **Isaac Lab for traffic training.** Keep it for physics validation. Policy training there
  needs agreement with you first, as recorded in `AGENTS.md`.

## Phases, exit checks and budget

| Phase | Work | Exit check | Ledger |
|---|---|---|---|
| 0. Tools | libsumo and parallel workers; v15 environment and parity test; event metrics; screens; benchmark rows | Same results as serial TraCI on fixed seeds; throughput measured | Engineering checks only, to be agreed with you |
| 1. Baselines | Round-16 policy, round-21 final and the plan follower on the 70 rows | Pass rates with intervals | Evaluation budget, about 210 episodes |
| 2. Rule expert | Level 1 and the rule Level 2; gate audit | Stage-1 targets met, or each failing gate explained | Evaluation budget |
| 3. Imitation | Behaviour cloning, then DAgger | Close to the expert, with no new kind of failure | Expert and DAgger decisions; sized after Phase 0 |
| 4. Anchored RL | MAPPO on Level 2, 3 seeds | Beats Phase 3 across seeds; no safety regression | Training decisions; sized after Phase 0 |
| 5. Physics and held-out | PhysX drives; held-out seeds | Recorded and labelled; held-out run once | Diagnostic episodes; held-out needs authorization |

Decision and time budgets are deliberately left open until Phase 0 measures throughput.

## Phase 0 result (November 25)

**Tools only.** The user started it with "tell the other session to start phase 0". They
directed the worker pool with "ok we should run multiple sims and have them work together to
improve learning speed and cross references".
- No training and no optimizer update; nothing charged.
- The ledger and its lock were not touched.
- Every run is SUMO, not physics.
- Outputs are in `outputs/test-runs/v15-p0/`, which is not in Git.

**Exit check: passed.** The plan's check is "Same results as serial TraCI on fixed seeds;
throughput measured".
- Every backend, worker count and pool batch reproduced serial TraCI exactly.
- The flat v15 environment reproduced v14 at every decision on the rows tested.
- Throughput is measured below.
- All 52 v15 tests pass, the SUMO end-to-end ones included, with none skipped.

**Phase 1 is approved but not started.** It still needs the items under "What Phase 1 needs"
below, and its resolved configuration and expected charge must be in the coordination master
first.

### What was built

| File | What it does |
|---|---|
| `traffic/sumo_coupling.py` | Switches the v14 environments between TraCI and libsumo 1.27.1 (`HIGHWAYSIM_SUMO_BACKEND`) without editing a hash-pinned file. TraCI stays the default and the sumo-gui path |
| `experiments/v15_parallel.py` | Runs validation rows serially or across N worker processes. `compare` checks two runs, or a run against a training round's own record, report by report and decision by decision |
| `experiments/v15_pool.py` | The worker pool, below |
| `environments/highway_merge_v15_env.py` | The flat v15 environment, below |
| `traffic/v15_events.py` | Event metrics |
| `scripts/v15_checkpoint_screens.py` | Settling points, per-input sensitivity, and the training-versus-validation gap |
| `traffic/v15_benchmark.py` | The benchmark rows |

### Parity: identical everywhere

- **Sweep:** round 24's 42 validation rows (14 cases × 3 seeds), driven by the round-16 policy.
  - It ran 20 times: serial TraCI, serial libsumo, and 4, 8, 16 and 24 workers on each
    backend, plus the same on the first 14 rows.
  - Every run gave identical reports and per-decision traces: 79,981 decisions per 42-row
    run.
  - The only field removed before comparing is the episode's own directory.
- **Trainer's record:** serial TraCI matches round 24's own initial-validation record on all
  42 rows.
- **Pool:** its 42-row validation at 1, 4, 8, 16 and 24 workers matched serial TraCI on every
  row. Every batch's canary matched: on a different worker each time, except at 1 worker,
  where it reran on the same one.

### Throughput

The sweep below ran from 15:49 to 16:20, before the heavier programs started on this PC. Each run
includes starting its workers. Each cell gives the clock time and the decisions per second.

| Workers | TraCI, 42 rows | libsumo, 42 rows | TraCI, 14 rows | libsumo, 14 rows |
|---|---|---|---|---|
| 1 | 478.0 s (167/s) | 289.1 s (277/s) | 157.9 s (163/s) | 108.9 s (237/s) |
| 4 | 145.6 s (549/s) | 82.2 s (973/s) | 54.5 s (474/s) | 28.8 s (895/s) |
| 8 | 91.6 s (873/s) | 44.7 s (1,790/s) | 37.6 s (686/s) | 30.8 s (839/s) |
| 16 | 52.3 s (1,530/s) | 36.0 s (2,220/s) | 40.8 s (632/s) | 31.6 s (815/s) |
| 24 | 46.2 s (1,733/s) | 34.9 s (2,292/s) | 38.8 s (664/s) | 26.5 s (975/s) |

- **libsumo vs TraCI:** libsumo is 1.65× faster serially. 24 libsumo workers run the 42 rows
  13.7× faster than serial TraCI.
- **Past about 16 workers, the longest row sets the time.** A 2,000-step row takes 18.7 s on
  its own and about 25 s on a busy machine, plus about 9 s to start the workers.
- **The 14-row set** has only 14 rows, so it can't use more than 14 workers.
- **Unexplained gap:** round 24's trainer ran the same 42 rows serially on TraCI in 389 s (122 s
  for the first 14). Here they took 478 s, and 147.7 s for the first 14 on a re-measure.
  - Threads are ruled out: the trainer also limits PyTorch to one thread
    (`experiments/run_highway_flow_rl_v05.py`).
  - The cause hasn't been found. It doesn't affect parity, and Phase 1 will use libsumo.

### Worker pool (`experiments/v15_pool.py`)

**Design:**
- **One learner and N SUMO workers.** The loop is synchronous: publish the weights, collect
  in parallel, then update. That keeps PPO on-policy and reproducible. Phase 0 ran no
  update.
- **Keyed results.** Each result is stored under a key built from what decides it: round,
  row set, row, case, seed, driver (a policy's content hash), background model, mode, and the
  sampling seed when collecting. The worker that ran it is never part of the key. Collection
  seeds PyTorch from that key, so samples don't depend on the worker.
- **Shared labelled dataset.** It serves cloning and DAgger. Collected episodes can carry
  another driver's labels; they are indexed in `dataset/index.jsonl`.
- **Repeated keys.** A key is simulated at most once. A later task with the same key reuses the
  stored result, at no cost. Asked to rerun instead, the pool runs it into a new folder and
  checks it against the stored result, like the canary.
  - The first full pool rerun stopped on this: its paired check repeated the validation's
    round-16 rows under IDM, and the pool then had no rule for a repeated key.
- **Canary.** Every batch reruns one task on a different worker. A mismatch stops the pool.
- **Paired validation.** Drivers run on the same rows and seeds, under IDM and Krauss
  background traffic.
- **Interleaving.** Several runs share one pool, row by row.
- **Time records.** Every batch records its clock time, the time summed across workers, the
  simulated time, the worker count and the decisions, as the clock-time ledger decision
  needs.

**Measurements:** round 24's 42 validation rows; then 3 learner iterations, each collecting
min(N, 20) training rows. A learner iteration's overhead is its clock time minus its slowest
episode: publishing, dispatch, writing and loading samples.

The measurements were taken at 17:44–18:08. Other programs (Isaac Sim's `kit.exe` and
background tools) used about 2–3.5 of the 32 hardware threads. Worker counts 1–16 come from
one run; 24 workers come from the rerun after the repeated-key fix.

| Workers | 42-row validation | Decisions/s | Canary | Learner iteration (rows collected) | Overhead per iteration |
|---|---|---|---|---|---|
| 1 | 327.1 s | 244 | matched (same worker; there is only one) | 5.6–5.8 s (1) | 0.44–0.49 s |
| 4 | 90.7 s | 882 | matched on another worker | 9.2–9.4 s (4) | 0.74–0.87 s |
| 8 | 47.1 s | 1,698 | matched on another worker | 11.5–11.8 s (8) | 0.97–1.11 s |
| 16 | 33.7 s | 2,376 | matched on another worker | 15.7–17.1 s (16) | 1.65–1.77 s |
| 24 | 33.7 s | 2,371 | matched on another worker | 45.9–54.4 s (20) | 4.9–6.8 s |

- **Every worker count reproduced serial TraCI on all 42 rows.**
- **Starting 24 workers** took 6.8 s.
- **Past 16 workers there's no gain on 42 rows.** The longest row sets the time, as in the
  sweep.
- **The 24-worker learner iterations are slow because of one row.** Training row 17
  (`train-4-lane-cruise`, 11,509 decisions, about 6× the median row) took 41–48 s on its own.
  The 16-worker iterations collect rows 0–15, so they leave it out.
- **The 24-worker overhead hasn't been explained.**
  - It's measured after each episode's own timer stops. That window covers the longest
    episode's post-processing: reading its trace, extracting events, and compacting, hashing and
    writing its samples. Row 17's samples are 25 MB of the iteration's 271 MB.
  - It hasn't been profiled. Phase 3–4 sizing should split such long rows, or balance the rows
    per iteration.
- **Interleaving at 24 workers:** 3 runs of the same 8 training rows, each with its own
  sampling seed. The 24 tasks gave 24 distinct keys in 17.5 s, and the canary matched on another
  worker.
- **Paired check at 24 workers:** the round-16 policy and round 21's final policy on validation
  rows 8 and 9, under IDM and Krauss.
  - 8 tasks; the canary matched.
  - The 2 round-16 IDM tasks reused the validation's results.
  - Neither policy passed any of the 4 pairings. Round 21's final policy collided on row 8 under
    Krauss only.
  - This is a tool check on 2 rows, not an evaluation.
- **The pool's own cost** was measured back to back on the first 14 validation rows, all three
  identical:

  | Runner | Clock time |
  |---|---|
  | `v15_parallel`, serial TraCI | 147.7 s |
  | `v15_parallel`, serial libsumo | 92.9 s |
  | The pool at 1 worker | 102.2 s, plus 2.7 s to start |

  The pool's per-episode bookkeeping (hashes, events, gates) costs about 10%.
- **The first pool attempt** ran at 16:34–16:58 under heavier load: a game used about 6 of the
  32 threads. Its 1-worker time was 478.6 s. It also matched serial TraCI and its canaries
  everywhere. The user paused it, and it was replaced by the run above.

### Flat v15 environment

**`HighwayMergeV15Env` is one flat class.** It uses no v04–v14 environment base class.
- It reuses the v14 scenarios, road networks, limits and messages.
- It reaches SUMO through `sumo_coupling`.
- Its rewards are all 0.0. Rewards and gates come later, from the plan's events and the rule
  expert.

**Parity with v14: identical on every row tested.**
- **The comparison:** one recorded policy drives round 24's configuration on libsumo. At every
  decision the test compares, exactly:
  - the time and the car set;
  - each car's 121-field observation and the 2,928-field critic state;
  - every vehicle's state;
  - the episode ends and the info fields;
  - the policy's actions.

| Row | Case | Steps | Decisions | v14 | v15 |
|---|---|---|---|---|---|
| 8 | LA northbound Pasadena on-ramp merge | 300 | 1,968 | 8.6 s | 2.7 s |
| 9 | LA southbound Adams on-ramp merge | 341 | 2,173 | 6.8 s | 2.4 s |
| 0 | I-110 merge | 414 | 2,888 | 11.0 s | 3.7 s |
| 13 | LA southbound Adams passing | 335 | 1,075 | 9.9 s | 1.2 s |
| 5 | 60 mph cruise, 5 lanes, packet loss 0.1 (first 400 of 2,000 steps) | 400 | 3,200 | 8.4 s | 2.6 s |

- **Speed:** v15 ran these rows about 3× faster than v14. It skips v14's reward and audit
  reads; that is the likely reason, but it hasn't been profiled.
- **Not carried over,** because round 24 doesn't use them: v14's startup rate limit, its drive
  target and its end-of-episode accounting.

### Events, screens and benchmark rows

- **Events** on round 24's initial validation (42 rows):

  | Event | Episodes | Rows | Per-decision count |
  |---|---|---|---|
  | Override | 68 | 27 | 545 decisions |
  | Free-road stop | 8 | 5 | 43 decisions |
  | Stop (any) | 19 | 8 | — |
  | Cut-in | 10 | 8 | — |
  | Emergency braking | 8 (1 by our car) | 7 | — |
  | Collision | 3 | 3 | 6 colliding vehicle-steps |

  The longest stop lasted 4.1 s (row 14), 1.1 s of it on a free road. Every stop counts as
  an event, as decided; the pull-away belongs to the Phase 2 executor.
- **Settling screen:** the port matches the parent session's round-21 script on all 104 rows
  it printed.
- **Per-input sensitivity** on cases 9 and 11:
  - the measured acceleration (input 1) holds 19–20% of the request's saliency;
  - with the previous request (input 5), the self-feedback share is 23–24%;
  - the v12 target inputs (69–72) hold 0% for the round-16 policy and 0.7% for round 24's
    update 32.
- **Training versus validation gap** for round 24's starting policy:
  - its 22 training episodes passed 3 of 22, against 19 of 42 in validation;
  - they averaged 8.9 override episodes per episode, against 1.6.
  - Training samples its actions on the training rows, so the gap mixes exploration noise with
    generalization.
- **Benchmark rows:**
  - 70 development rows: each of the 14 cases at seeds 6952001–6952005. Development rows
    0–27 are round 24's validation rows 14–41.
  - The cases' own v14 seeds are left out, since rounds 20–23 were judged on them.
  - Held-out seeds 6953001–6953003 are reserved. The code refuses to build them without an
    explicit authorization flag, and none was run.

### What Phase 1 needs

All of this was built, decided and recorded before Phase 1 ran; see
[Phase 1 result](#phase-1-result-november-25).

- **The plan follower as a pool driver.** It exists only as scratch scripts from round 23's
  check (copies in `outputs/test-runs/v15-p0/scratch-copies/`).
  - It reads the follow gate's reference (`traffic/highway_flow_rl_v14_follow_gate.py`) from the
    environment's internal state.
  - It can't change lanes, so round 23's check skipped cases 0 and 4. How to score it on those
    two cases (10 of the 70 rows) needs deciding before launch.
- **Round 21's final policy** is `outputs/highway-flow-rl/v14/round-21/last.pt`; round 21's
  summary names it as its checkpoint. The round-16 policy is the warm start of round 24's
  configuration.
- **A ledger writer for evaluation runs, using the shared lock.**
  - It uses the study ledger and `active.lock`, as `experiments/run_highway_flow_rl_v05.py` does.
  - It charges the clock time and every decision across workers, and records the summed worker
    time, the simulated time and the worker count.
  - Its category has to pass the ledger's own totals check.
- **Whether Krauss runs are included.** The approved ~210 episodes cover one background model.
  Adding the Krauss variant would double it to ~420, which needs the user.
- **Wilson intervals and paired differences** for the report. `paired_table` already lines
  rows up; the interval code isn't written.
- **Phase 1's resolved configuration and expected charge** go in the coordination master
  before launch.
  - On 42 rows the round-16 policy makes 79,981 decisions, so the 70 rows should be about
    133,000 decisions per driver. That's an estimate, not a measurement.
  - From the sweep, 24 libsumo workers need well under a minute per 42 rows.

## Phase 1 result (November 25)

**Exit check met:** pass rates with 95% intervals for the three baselines on the development rows.
- **Evaluation only:** no training and no update. All runs are SUMO (libsumo), not physics.
- **Background:** IDM only.
- **Held out:** the held-out rows were not built or run.

**User decisions**, relayed by the main Claude session and recorded in the coordination master:
- **"Yes, all four":**
  1. commit Phase 0;
  2. raise the decision allowance for Phase 1 only, sized to its resolved charge and at most
     400,000, with the time cap unchanged;
  3. IDM background only;
  4. score the plan follower on the 60 rows without cases 0 and 4.
- **The launch:** "start phjase one after those modifications have been made".
- **The CPU limit:** "don't use all cpu cores i need some to do other tasks". So the run used 16
  workers, and the workers and the learner ran at below-normal priority.

### What was built

- **The plan follower** (`traffic/v15_plan_follower.py`, driver `rule:plan-follower-v1`,
  validation only). It is round 23's rule:
  - it requests 3/s × (target − speed), within −4 to 2 m/s²;
  - it stays below the acceleration ceiling the actor observes, minus 0.05;
  - it stays within 0.35 m/s² of the measured acceleration;
  - it never changes lanes.

  Its target is v14's own ramped merge target, the `gate_v14` entry of the environment's
  per-car target cache. Off the merge it uses the free-road speed.
  **Correction to Phase 0's note above:** it reads v14's target, not the follow gate's reference.
- **The evaluation ledger writer** (`experiments/v15_ledger.py`):
  - **Lock:** it takes the shared `active.lock` with exclusive create and never overrides
    another run's lock.
  - **Charge:** clock time against the study total, and every decision across workers (canary
    reruns included) against the phase's own allowance.
  - **Records:** summed worker time, simulated time, the worker count and the process priority.
  - **Compatibility:** its segment is a `preparation` segment (command `v15_evaluation`,
    `evaluation_only`), so the ledger's own totals check and every existing runner still read the
    ledger.
  - **Episodes:** they go in `evaluation_episodes`, not the diagnostic-episode count.
- **The statistics** (`traffic/v15_stats.py`):
  - **Pass rates:** Wilson score intervals.
  - **Paired differences:** Newcombe's hybrid score interval, with the corrected phi (ad − bc
    reduced by n/2 when positive). Without the correction, two drivers that agree on every row
    get a zero-width interval.
  - **Coverage, measured by simulation** (2,000 tables each at n = 60–70): about 95% where
    drivers disagree often; 97–100% (conservative) where they rarely do.
  - **Tests:** an exact McNemar test.
- **The runner** (`experiments/v15_phase1.py`) and its pinned configuration
  (`configs/evaluation/v15-phase1.json`, sha256 `a37fdaf9…`). The run refuses to start if what it
  resolves differs from the pinned file.
- **The pool:**
  - it accepts rule drivers for validation;
  - it counts the canary's rerun decisions;
  - it sets and checks each worker's process priority.
- **Tests:** 79 v15 tests pass, the SUMO end-to-end ones included. One of them runs the whole
  charged path (lock, session, pool, canary, parity, ledger segment, report) against a temporary
  ledger.

### The run

Five batches, one per development seed. Each batch ran every driver on that seed's 14 rows
(the follower on 12), with the cheapest short row as the canary. 16 workers, below-normal
priority, started at 23:23 (America/Los_Angeles).

| Batch | Seed | Clock time | Decisions (expected) | Canary | Parity rows |
|---|---|---|---|---|---|
| 0 | 6952001 | 39.5 s | 80,315 (77,687) | matched, other worker | 14 of 14 identical |
| 1 | 6952002 | 34.7 s | 78,483 (79,071) | matched, other worker | 14 of 14 identical |
| 2 | 6952003 | 35.2 s | 80,582 (77,183) | matched, other worker | — |
| 3 | 6952004 | 34.6 s | 76,400 (77,183) | matched, other worker | — |
| 4 | 6952005 | 35.0 s | 78,358 (77,183) | matched, other worker | — |

- **Total:** 185.3 s of clock time; 2,231.1 s of summed worker time; 26,220 s simulated.
- **Decisions:** 395,240, against 388,307 expected (+1.8%). 1,102 of them are canary reruns.
- **Episodes:** 205.
- **Parity:** development rows 0–27 are round 24's validation rows 14–41. The round-16 policy's
  reports and per-decision traces there are identical to Phase 0's.

### Pass rates

| Driver | Rows passed | Pass rate (95% Wilson) | Decisions |
|---|---|---|---|
| Round-16 policy | 33 of 70 | 47% (36–59%) | 134,530 |
| Round 21's final policy | 31 of 70 | 44% (33–56%) | 135,760 |
| Plan follower | 21 of 60 | 35% (24–48%) | 123,848 |

On the follower's 60 rows, the round-16 policy passes 29 and round 21's final policy 27.

| Case | Round 16 | Round 21 final | Plan follower |
|---|---|---|---|
| 0. i110 merge | 0/5 | 0/5 | not scored |
| 1. Cruise | 3/5 | 3/5 | 3/5 |
| 2. Passing | 5/5 | 5/5 | 4/5 |
| 3. i110 following | 5/5 | 5/5 | 0/5 |
| 4. 2-lane ramp 120 | 4/5 | 4/5 | not scored |
| 5. 60 mph cruise | 3/5 | 4/5 | 5/5 |
| 6. 2-lane ramp 180 | 3/5 | 2/5 | 0/5 |
| 7. 4-lane ramp 180 | 1/5 | 1/5 | 0/5 |
| 8. LA NB 8-car merge | 0/5 | 0/5 | 2/5 |
| 9. LA SB 8-car merge | 1/5 | 1/5 | 1/5 |
| 10. LA NB 1-car merge | 3/5 | 1/5 | 3/5 |
| 11. LA SB 1-car merge | 1/5 | 1/5 | 2/5 |
| 12. LA NB following | 2/5 | 2/5 | 1/5 |
| 13. LA SB passing | 2/5 | 2/5 | 0/5 |

Paired differences use the same rows, seeds and background for both drivers:

| First vs second | Rows | Both pass | First only | Second only | Neither | Difference (95%) | McNemar p |
|---|---|---|---|---|---|---|---|
| Round 21 final vs round 16 | 70 | 30 | 1 | 3 | 36 | −2.9 pts (−8.9 to +3.3) | 0.625 |
| Plan follower vs round 16 | 60 | 16 | 5 | 13 | 26 | −13.3 pts (−26.4 to +0.5) | 0.096 |
| Plan follower vs round 21 final | 60 | 15 | 6 | 12 | 27 | −10.0 pts (−23.3 to +4.0) | 0.238 |

Failing gates and events (rows with at least one; totals in brackets):

| | Round 16 (70 rows) | Round 21 final (70) | Plan follower (60) |
|---|---|---|---|
| Per-car route comfort or control | 24 | 24 | 38 |
| Rolling merge quality | 23 | 29 | 19 |
| Raw request jerk | 24 | 22 | 0 |
| Stable merge | 7 | 7 | 11 |
| Native emergency | 4 | 5 | 1 |
| Override episodes | 40 (94) | 35 (79) | 30 (190) |
| Stops / free-road stops | 11 / 5 | 14 / 7 | 14 / 0 |
| Cut-ins | 8 (10) | 10 (11) | 3 (3) |
| Collisions | 2 | 2 | 1 |

### What it shows

- **Round 21's final policy is not distinguishable from the round-16 policy.** Only 4 of 70 rows
  differ (1 against 3).
- **The plan follower passes fewer rows than the round-16 policy, but not significantly.** On
  the same 60 rows it passes 21 against 29, so 8 fewer: −13.3 points (−26.4 to +0.5), p = 0.096.
- **Where the follower loses:** all of case 3 (i110 following, 0 of 5 against 5 of 5), both
  ramp-180 cases (0 of 10 against 4 of 10) and case 13 (0 of 5 against 2 of 5).
- **Where it gains:** case 5 (5 of 5 against 3 of 5), case 8 (2 of 5 against 0 of 5) and case 11
  (2 of 5 against 1 of 5).
- **It has fewer safety events but more overrides.**
  - Fewer rows with emergencies (1 against 4–5), cut-ins (3 against 8–10) and free-road stops
    (0 against 5–7).
  - Override episodes reach 190 in 30 rows, against 79–94.
  - Its most common failing gate is per-car route comfort or control: 38 of 60 rows, against 24
    of 70 for each policy.
  - In round 23's check, case 8's comfort failures came from jerk when the environment's safety
    limit cut in. That was not checked here.
- **No driver passes case 0** (i110 merge): 0 of 5 for both policies. The follower isn't scored
  there.
- **Caveat:** rows of one case share its network and traffic setting and differ only in seed.
  Pooled intervals that treat the 70 rows as independent are narrower than the case structure
  justifies, and per-case intervals on 5 rows are wide.
- **For Phase 2:**
  - The rule expert has to beat the round-16 policy's 33 of 70.
  - A lane-keeping target follower is not that expert as it stands: it can't change lanes, and
    it fails comfort or control on most rows.
  - Phase 2's budget still needs the user.

### Ledger

| | Before | After |
|---|---|---|
| Study time | 60,195.7 of 64,700 s | 60,381.0 of 64,700 s (4,319.0 s left) |
| Training decisions | 6,722,376 of 6,871,150 | unchanged |
| Phase 1 evaluation decisions | 0 of 400,000 | 395,240 of 400,000; the rest isn't carried over |
| Diagnostic episodes | 173 of 174 | unchanged |

- **The segment:** one `v15_evaluation` segment, complete, charged 185.3 s, recording 205
  evaluation episodes, 16 workers and below-normal priority.
- **The lock:** released.
- **Outputs:** `outputs/highway-merge-v15/phase-1/run-01` in this worktree (not in Git), with
  `report.md`, `report.json`, `charge.json`, `phase1-log.jsonl` and the results table.

## Phase 2 result (November 26)

**Exit check met:** all four Stage 1 targets met from run 2 on, and every failing gate explained
(gate side or driver side) in the audit below. The expert qualified, so the gates were re-baselined
once (decision 3) and are now frozen.
- **Evaluation only:** no training and no update. All runs are SUMO (libsumo), not physics.
- **Background:** IDM only.
- **Held out:** the held-out rows were not built or run.

**User decision,** relayed by the main Claude session and recorded in the coordination master:
"Yes, approve 5 runs". That allows:
- at most 5 full runs of the rule expert on the 70 development rows;
- at most 725,000 decisions in all;
- clock time charged against the study total, with every run on benchmark rows charged.

Uncharged 1–2-row code checks ran on v14-validation rows only. The settings were as in Phase 1:
16 workers at below-normal priority, the shared lock, and each run's configuration and expected
charge in the master before it ran. Phase 3 has no budget, so Phase 2 ends here.

### What was built

- **The rule expert** (`traffic/v15_rule_expert.py`), Level 1 with the rule Level 2. It drives every
  controlled car through the same normalized action a policy uses, so every environment limit and
  intervention count still applies. It validates only. Each version is kept byte for byte so its run
  can be repeated: v1 in `traffic/v15_rule_expert_v1.py`, v2 in `traffic/v15_rule_expert_v2.py`.
  - **v1:** IDM following with a free-road term, the rate band and ceiling, the stuck rule, the
    curvature cap, MOBIL lane changes, the dilemma rule at ending lanes.
  - **v2,** from run 1's failures:
    - following-bound anticipation;
    - projected leaders through junctions;
    - gap selection at yield entries, judged at the stop line with SUMO's IDM and each mainline
      car's own parameters;
    - a new-leader test for lane changes.
  - **v3,** from run 2's gate audit:
    - lane-change leaders by lane identity;
    - coasting for small gap corrections.
- **The Phase 2 runner** (`experiments/v15_phase2.py`) and one pinned configuration per run
  (`configs/evaluation/v15-phase2-run-0N.json`). Each run is 2 batches with canaries, plus the
  round-16 parity task on development row 24.
- **The gate re-baseline** (`experiments/v15_regrade.py`), below.
- **Tests:** 90 v15 tests pass.

### The runs

Each run: the 70 development rows in 2 batches, 73 episodes, 16 workers at below-normal priority.
In every run both canaries matched on another worker, and the round-16 parity row matched its
Phase 1 record.

| Run | Driver | Round 24's gates (recorded) | Re-baselined | Stage 1 targets | Charge |
|---|---|---|---|---|---|
| 1 | v1 | 50 of 70, 71% (60–81%) | not re-graded (before the re-baseline) | 2 of 4: 1 collision; 2 cut-ins in one-car LA cases | 132,879 decisions, 90.4 s |
| 2 | v2 | 52 of 70, 74% (63–83%) | 59 of 70, 84% (74–91%) | all 4 | 136,365 decisions, 111.6 s |
| 3 | v3 | 54 of 70, 77% (66–85%) | 62 of 70, 89% (79–94%) | all 4 | 136,119 decisions, 104.8 s |

95% Wilson intervals. The fourth target is a pass rate above round 16's 33 of 70.

- **Paired with the round-16 policy's Phase 1 record** (same rows):
  - run 1, recorded: +24.3 points (+12.8 to +34.7);
  - run 2: +27.1 (+13.9 to +38.9) recorded, +32.9 (+20.3 to +44.1) re-baselined;
  - run 3: +30.0 (+16.4 to +42.0) recorded, p = 4.9e-5; +37.1 (+24.2 to +48.6) re-baselined,
    p = 2.2e-7. Re-baselined: both pass 35, v3 only 27, round-16 only 1, neither 7.
- **Run 1** failed per-car checks on 16 rows (25 peak-jerk failures), raw request jerk on 15 and
  merge quality on 12, with 1 collision. That led to v2.
- **Run 2** met every target. Its failing rows were audited (below) and the gates re-baselined.
- **Run 3** fails only rolling merge quality (16 rows recorded, 8 re-baselined). There are no
  per-car or raw request jerk failures. Against v2 on the same rows, v3 also passes rows 4, 43
  and 69 and loses none: 62 against 59 re-baselined (+4.3 points, −1.8 to +11.3, p = 0.25), and
  54 against 52 recorded. That's too few rows to show the difference is real.
  - **Row 4:** during the lane change v2's lowest request was −3.9 m/s². v3's was −1.5 m/s², and
    R1 then grades the merge as passing.
  - **Row 43:** the small gap correction coasts, so the avoidable braking event is gone.
  - **Row 69, a correction to the run 2 audit,** which put it down to a background cut-in:
    - v2 lost its source-lane leader during the lane change. It had no IDM leader at 16.5 s, and
      the environment's bound then took acceleration from −1.9 to −3.3 m/s² (jerk 14.6 m/s³).
    - v3 kept that car as a leader and braked steadily at about −1.6 m/s² (peak jerk 3.0).
- **SUMO's "junction collision" warning** at the top of the run 2 and run 3 logs comes from the
  round-16 parity task on row 24. That task reproduces round-16's Phase 1 record of the row,
  collision included. The expert had no collision in runs 2 or 3.
- **Still failing for v3 after the re-baseline** (merge quality on 8 rows), for the causes the
  audit found:
  - following headway on rows 0, 9, 14 and 24 (driver side, not changed);
  - yield-entry waits on rows 8, 22, 37 and 38, where samples before the stop line are still
    graded by design.

### Gate audit (run 2)

Each failing row was read from each decision's record and the map and scene at the failure.

- **Gate side:**
  - **Acceleration-lane merges** (rows 4, 20, 21, 34, 48, and part of rows 0 and 14):
    - v14's target returns to the road target at once at the merge completion (row 4: 15.9 →
      22.35 m/s; row 20: 6.8 → 24.6 m/s).
    - At the 2.0 m/s² comfort cap a car needs 3–8 s to come within 2 m/s.
    - Round 23's follow gate fixed this for yield entries only.
  - **Ineligible targets count as misses** whatever the car does: 32 of 161 samples for row 0's
    agent_0.
  - **Yield entries** (rows 8, 22, 37, 38, 39, 53, 67): the follow reference keeps rising toward the
    gap planner's target while the car waits for a gap its own test rejects. Once 2 m/s behind, a car
    limited by the same drive ceiling can't catch up.
    - Row 53: the expert held for main_0 (22 m/s), which passed the junction about 0.5 s before our
      car crossed.
    - The reference had restarted 2 s earlier and rose at the 1.32 m/s² drive ceiling.
    - Our car stayed 3.4–3.8 m/s behind for the whole window.
- **Driver side:**
  - **Lane-change leaders** (rows 4 and 14). After the lane switch mid-maneuver, v2 took its
    "target-lane leader" from the lane beyond the target.
    - Row 4 braked 0.8 s for a car alongside at a 0.0 m gap and lost 3 m/s.
    - Row 14 dropped the slow car in its ending source lane. That car then merged into the same lane,
      and the environment's bound clamped at −4.5 m/s² (jerk 42.9 m/s³).
    - Fixed in v3.
  - **Following headway** (rows 0, 9, 14 and 24; row 43's avoidable braking).
    - Behind a slower leader, the gate's target is the speed whose 1.5 s + 2.5 m gap equals the
      current gap. The per-car check needs a 1.5 s minimum headway.
    - So the gates accept gaps from 1.5 s × speed up to about 1.5 s × speed + 5.5 m. The expert keeps
      1.8 s + 4 m.
    - Not changed: run 2's per-car minimum headways are 1.60–1.70 s on 5 cars, so a 1.5 s setting
      would likely fail the headway check. Row 43's gentle braking became coasting in v3.
  - **Row 69:** put down to a background car that cut in about 10 m ahead from the far lane during
    our lane change, and not addressed. **Corrected after run 3:** v2 had also lost its
    source-lane leader there, and v3 passes the row (see [The runs](#the-runs)).

### Gate re-baseline (decision 3), frozen

`experiments/v15_regrade.py` (sha256 `515c3b7c…`) re-grades only the speed match inside rolling merge
quality, offline from the recorded action journals. Nothing is simulated and nothing is charged.
Every other gate and per-car check is round 24's installed gates, unchanged.

- **Parity first.** With no rule, it reproduces:
  - every recorded verdict (271 episodes of run 2 and Phase 1);
  - every step's speed match (69,813 steps);
  - every graded car's window and fraction (141 cars).

  It also rebuilds every follow reference exactly from its recorded inputs. A mismatch stops it.
- **R1, every merge:** a merge without a yield entry is also graded on the follow gate's reference
  band. The reference starts at the car's speed at the window's first eligible decision.
- **R2, restart at the stop line:** a yield entry's reference also restarts from the car's speed when
  it crosses the stop line. Samples before the crossing are still graded.
- **R3, eligible samples only:** the speed-match fraction counts only samples with an eligible
  target. They stay in every other check, and fewer than half the window eligible does not qualify.
- **Not changed:** following behind a slower leader.

| Driver | Recorded | R1 alone | R2 alone | R3 alone | All three |
|---|---|---|---|---|---|
| Expert v2 (run 2) | 52 of 70 | 55 | 55 | 53 | 59 of 70 |
| Expert v3 (run 3) | 54 of 70 | 58 | 57 | 55 | 62 of 70 |
| Round-16 policy (Phase 1) | 33 of 70 | 35 | 33 | 34 | 36 of 70 |
| Round 21's final policy (Phase 1) | 31 of 70 | 31 | 31 | 31 | 31 of 70 |
| Plan follower (Phase 1) | 21 of 60 | 21 | 21 | 21 | 21 of 60 |

- **Rows that changed:**
  - v2: 20, 34 and 48 (R1), 21 (R3), and 39, 53 and 67 (R2);
  - v3: the same, plus 4 (R1);
  - round-16: 35 and 49 (R1), and 21 (R3).
- **Parity** held on all three re-grades: every verdict and step was reproduced, and the largest
  rebuilt-reference difference was 0.0.
- **Outputs:** `outputs/highway-merge-v15/phase-2/rebaseline` (v2) and
  `outputs/highway-merge-v15/phase-2/run-03/rebaseline` (v3), not in Git.

**Frozen:** from here, the v15 benchmark's gates are round 24's installed gates, with rolling merge
quality re-graded by this module at this hash. Every later run reports both verdicts. Changing the
gates again needs you.

### Ledger

| | Before Phase 2 | After |
|---|---|---|
| Study time | 60,381.0 of 64,700 s | 60,687.8 of 64,700 s (4,012.2 s left) |
| Training decisions | 6,722,376 of 6,871,150 | unchanged |
| Phase 2 evaluation decisions | 0 of 725,000 | 405,363 of 725,000; the rest isn't carried over |
| Phase 2 runs | 0 of 5 | 3 of 5; the other 2 aren't carried over |
| Diagnostic episodes | 173 of 174 | unchanged |

- **Segments:** three `v15_evaluation` segments, all complete, charged 90.4 s, 111.6 s and
  104.8 s (306.8 s), recording 219 evaluation episodes.
- **Uncharged:** the 1–2-row code checks on v14-validation rows, the test suite, and the
  re-grades, which simulate nothing.
- **The lock:** released.

### Why Phase 2 stopped at 3 runs

- The expert qualified in run 2.
- v3 fixed the driver-side faults the audit found and lost no row.
- What's left is either graded on purpose by the frozen gates (waits before the stop line) or a
  headway trade-off that would likely fail the 1.5 s headway check.
- More runs would only tune against the 70 development rows, and the held-out rows can't check
  that.

**The qualified expert is v3** (`rule:expert-v3`, module sha256 `9018ba3a…`). Phase 3 has no
budget, so nothing more runs until you approve one.

## Phase 3 proposal (November 26): imitation, for your approval

**Approved on November 26** ("Recommended size", decision 8) and run; see
[Phase 3 result](#phase-3-result-november-26). The proposal is kept below as written.

**Proposal only.** Nothing here has been built or run, and nothing is charged until you approve a
budget. The user asked for it on November 26: "Yes, size it" and "Yes, draft it". All runs would be
SUMO (libsumo), not physics, with IDM background only. The held-out rows are never built or run.

**What Phase 3 does** (Stages 2 and 3): a learned Level 2 learns to copy the rule expert v3's Level 2
choices, first by behaviour cloning and then by DAgger.
- **Level 2's choices:** the gap, commit or wait, a target-speed offset and lane intent.
- **Level 1,** the rule executor, is the same in every run.
- **Exit check:** close to the expert on the 70 development rows, with no new kind of failure.
- **The expert's scores there:** 54 of 70 under round 24's gates and 62 of 70 re-baselined.

### Built first (uncharged)

- **A Level 2 interface.** v3 is split into its executor and a rule Level 2 with explicit outputs.
  The split must reproduce v3's per-decision traces exactly on the uncharged check rows (v14-validation
  rows 0 and 4).
- **Level 2 inputs without privileged state:**
  - today's local fields and V2V messages;
  - the executor's state (committed or not, and the chosen gap).

  The expert keeps reading the environment's own state, but the learner doesn't.
- **Training rows:** round 24's 20 training rows (`training_rows` in
  `configs/training/highway-flow-rl-v14-round24.json`) at new seeds in the training range (9,100,000
  and up). Each seed is checked unused, as Phase 0 did, and none is a development or held-out seed.
- **Perturbations and balance,** as Stage 2 describes:
  - some episodes start slow, stopped or braking near the merge;
  - random speed offsets, and late commits for 0.6 + 0.8u s, with the expert's recovery as the label;
  - samples where a stopped car stays stopped are down-weighted, but every "stopped on a free road,
    so go" label is kept.
- **Split by seed:** 2 of the 10 data seeds are held back. Their error is watched for memorizing, but
  it never chooses a model.
- **The learning pieces:**
  - a plain-PyTorch trainer (CPU, one thread);
  - the learned Level 2 as a pool driver;
  - DAgger mixing: the learner acts with probability 0.7 and the expert labels every state.
- **A ledger category for Phase 3's segments** (training as well as evaluation), passing the ledger's
  totals check.
- **Tests,** and 1–2-row code checks on v14-validation rows only. These are uncharged, as in Phase 2.

### What the estimate is built from

- **The expert's cost:** about 145 decisions per worker-second (run 3: 135,428 decisions in 937.4
  worker-seconds).
- **Pool throughput:** Phase 2 ran about 1,300 decisions per second of clock time at 16 workers
  (136,119 in 104.8 s).
- **A development-row evaluation:** 132,879–136,365 decisions and 90.4–111.6 s in Phase 2's three
  runs.
- **The 20 training rows:**
  - at most 79,200 decisions per seed (controlled cars × horizon steps);
  - round 24's 32 training episodes averaged 1,601 decisions, so about 32,000 per seed;
  - the 12-car 4-lane cruise row alone made 11,299.
- **Data size:** about 6,400 Level 2 samples per seed, at one sample per car every 0.5 s.
- **A fit:** one epoch of 160,000 samples took 1.1 s on one CPU thread. That was a synthetic timing
  of a small network (two layers of 256) on random data. No policy was trained, nothing was
  simulated, and no code check ran.
  - At 30 epochs, that is about 15 s on step 1's ~64,000 samples and about 40 s on the ~190,000
    after 4 DAgger iterations.
  - The estimates allow 35 s per cloning fit and 40 s per DAgger refit, all charged as clock time.
- **Expert labelling adds compute, not simulated decisions.** In a DAgger rollout the expert computes
  its choice on the same simulated step that the learner's choice (or, 30% of the time, its own)
  drives. Only those car-steps are decisions.
- **Not measured:** what that extra compute costs. The estimate assumes a DAgger rollout costs 1.3
  times the expert's cost per decision, and the uncharged check measures it.

### Recommended size and estimated charge

| Step | Work | Episodes | Decisions (estimate) | Clock time (estimate) |
|---|---|---|---|---|
| 1. Data | The expert on the 20 training rows at 10 new seeds, half of the episodes perturbed | 200 | ~320,000 | ~250 s |
| 2. Cloning | 3 fits at different training seeds, each evaluated on the 70 development rows | 210 | ~408,000 | ~420 s |
| 3. DAgger | Up to 4 iterations from the best fit. Each: 5 new seeds of rollouts at 0.7 (100 episodes), a refit on all the data, and an evaluation (70) | up to 680 | ~1,184,000 | ~1,220 s |
| 4. Final check | The final data refit at 2 more training seeds, each evaluated | 140 | ~272,000 | ~290 s |
| Overheads | Worker start-up, canaries and a parity task (expert v3 against its run 3 record) in each of about 14 runs | ~42 | ~20,000 | ~210 s |
| **Total** | | **~1,270** | **~2.2 million** | **~2,400 s** |

### Minimum option

| Step | Work | Episodes | Decisions (estimate) | Clock time (estimate) |
|---|---|---|---|---|
| 1. Data | The expert on the 20 training rows at 5 new seeds, half perturbed, with 1 seed held back | 100 | ~160,000 | ~125 s |
| 2. Cloning | 1 fit, evaluated on the 70 development rows | 70 | ~136,000 | ~140 s |
| 3. DAgger | 3 iterations, the plan's minimum. Each: 3 new seeds of rollouts at 0.7 (60 episodes), a refit and an evaluation (70) | 390 | ~696,000 | ~690 s |
| Overheads | About 8 runs | ~24 | ~12,000 | ~120 s |
| **Total** | | **~580** | **~1.0 million** | **~1,100 s** |

The minimum option has less data and a single training seed, and it has no final check. So it can't
tell a gain from chance: two fits from the same data can drive differently (Stage 2). Phase 4's test
is to beat Phase 3 across seeds, and the recommended size gives it that baseline.

- **Selection:** by closed-loop pass rate on the 70 development rows, never by held-back loss
  (Stage 2). The frozen re-baselined verdicts are reported with the recorded ones.
- **Stop rule:** DAgger stops at whichever comes first:
  - the development pass rate stops rising;
  - 5 iterations are done;
  - the next iteration won't fit the cap.

  The recommended cap pays for 4 iterations. A 5th runs only if the earlier steps came in under their
  estimates.
- **Repeated looks:** every evaluation is another look at the same 70 rows. Only the held-out check
  (Stage 5, which needs your authorization) can show how much of a result carries over.

### Proposed budget

- **Recommended:** at most 2,700,000 decisions and 3,000 s of study time. That is the estimate plus
  about 20% in decisions and 25% in time. It leaves at least 1,012 s of the 4,012.2 s of study time
  left.
- **Minimum:** at most 1,200,000 decisions and 1,400 s, leaving at least 2,612 s.
- **Either way:**
  - it is Phase 3's own allowance, as Phases 1 and 2 had, since v14's training-decision total has
    only 148,774 left;
  - clock time is charged against the study total, as before.
- **The same settings as Phases 1 and 2:**
  - at most 16 workers at below-normal priority, with nothing heavy alongside;
  - the shared lock;
  - each run's resolved configuration and expected charge in the coordination master before it runs.
- **Stop and report if:**
  - the next step's charge won't fit;
  - the lock is held;
  - a canary or parity check mismatches;
  - tests fail;
  - a learned policy shows a new kind of failure: a collision, a free-road stop, or a cut-in or
    emergency in a one-car LA case. First find out whether the learner or the gate is wrong, as in
    Phase 2;
  - the uncharged check measures DAgger's cost high enough that the chosen size no longer fits its
    cap. Then re-size before anything is charged;
  - after step 1, the measured cost per decision puts the remaining steps over the cap;
  - the expert collides or stops on a free road on any training row in step 1. Its labels there
    would be wrong, so I'd find out why before cloning. Step 1 also reports the expert's pass rate
    on the training rows.

**After Phase 3,** about 1,000–1,600 s of study time would be left at the recommended size, or about
2,600–2,900 s at the minimum. Neither can pay for Phase 4 (MAPPO at 3 training seeds), so Phase 4 will
need its own budget and a raise of the study total. Nothing is asked for that now.

## Phase 3 result (November 26)

**Exit check met, by my reading.** The selected learned Level 2 is close to the expert on the 70
development rows, with no new kind of failure.
- **Re-baselined (the frozen gates):** 60 of 70, against the expert's 62. Paired with the expert,
  60 rows both pass, 2 only the expert, and none only the learner: −2.9 points (−9.3 to +2.8),
  McNemar p = 0.5.
- **Recorded (round 24's gates):** 51 of 70, against the expert's 54.
- **"Close"** has no number in the plan, so calling this close is my judgement.

**How it ran:**
- **Training:** only Level 2 was trained, a small network (124 inputs, two layers of 256), by
  supervised imitation of expert v3's Level 2 choices. Level 1, the rule executor, is the same in
  every run. No reward and no PPO.
- **Simulator:** SUMO (libsumo), not physics, with IDM background only.
- **Held out:** the held-out rows were not built or run.

**User decisions,** relayed by the main Claude session and recorded in the coordination master:
- **Decision 8:** "Recommended size", at most 2,700,000 decisions and 3,000 s as Phase 3's own
  allowance.
- **After step 1 stopped at a stop point** (below): "Yes, continue", and "continue with data" typed
  in the side session, keeping all the data. The stop rules were restated with that answer:
  - an expert free-road stop that begins during an injected slow-down is reported, not a stop point;
  - any collision in a DAgger rollout stops Phase 3.

**Settings as in Phases 1 and 2:**
- uncharged 1–2-row code checks on v14-validation rows only;
- 16 workers at below-normal priority and the shared lock;
- each step's resolved configuration and expected charge in the master before it ran.

### What was built

- **`traffic/v15_level2.py`** splits expert v3 into its executor and a rule Level 2 with explicit
  choices:
  - **the gap:** how many stream cars (0–3) to let pass. The executor takes the first gap at or after
    it that passes its safety test, so a choice can only make a car wait longer;
  - **the lane intent,** requested only inside the mask and MOBIL's safety test;
  - **a target-speed offset** from −2 to +2 m/s. The expert's is always 0.

  The split drives the identical trace to `rule:expert-v3`. The file also holds the learned Level 2,
  the perturbations (`p1`), the DAgger mixing and the sampling.
- **The learned Level 2's 124 inputs:** the 121-field observation without own acceleration and own
  previous request, plus the executor's state and two map facts. The previous gap choice is left
  out, because copying it locks a waiting car into waiting.
- **`traffic/v15_training_rows.py`:** round 24's 20 training rows at new seeds,
  9,700,000 + 1000 k + i, disjoint from every earlier seed.
- **`experiments/v15_level2_train.py`:** the plain-PyTorch trainer. CPU, one thread, deterministic,
  AdamW, 30 passes at batch 1024.
- **`experiments/v15_phase3.py`:** the runner, with one pinned configuration per step
  (`configs/evaluation/v15-phase3-*.json`). Changes to `experiments/v15_pool.py` and
  `experiments/v15_ledger.py`.
- **Tests:** 116 v15 tests pass.

### The steps

| Step | Work | Decisions | Clock time | Re-baselined / recorded, of 70 |
|---|---|---|---|---|
| 1. Data | The expert on the 20 training rows at 10 seed sets, half perturbed: 200 episodes, 163,972 samples | 350,758 | 257.3 s | Stop point (below) |
| 2. Cloning | 3 fits, at seeds 1–3 | 412,511 | 258.3 s | 59/49, **60/50**, 59/51 |
| DAgger 1, attempt 1 | Failed before any learner episode (below) | 191 | 8.1 s | — |
| DAgger 1 | Rollouts of cloning seed 2 (100 episodes, learner at 0.7), then a refit at seed 2 | 314,757 | 210.5 s | **60/51** |
| DAgger 2 | Rollouts of DAgger 1's refit, then a refit | 314,984 | 221.0 s | 59/49 |
| DAgger 3 | Rollouts of DAgger 2's refit, then a refit | 314,328 | 228.8 s | 59/48 |
| Final check | DAgger 1's data refit at seeds 4 and 5 | 276,036 | 201.7 s | 60/52, 60/52 |
| **Total** | 505 training and 572 evaluation episodes; 8 fits (255.5 s) | **1,983,565** | **1,385.7 s** | |
| Relabel (added November 26) | DAgger 1's data refit at seed 2, with the gap labelled 0 ([below](#the-relabel-step-november-26)) | 139,117 | 107.6 s | **62/54** |

- **Selected:** DAgger 1's refit, `l2:5aebbf7d8a40d2b9`. Selection was by most re-baselined passes,
  then recorded passes, and never by held-back loss.
- **DAgger stopped after 3 iterations,** under the approved rule: at least 3, then stop at the first
  that isn't above the best so far.
- **The final check:** I read "the final data refit at 2 more training seeds" as a refit on the
  selected configuration's data (step 1 plus DAgger 1), and recorded that in the master before it ran.
  - That configuration scores 60 at all 3 training seeds (2, 4 and 5), with the same 10 failing rows.
    That is Phase 4's baseline across seeds.
  - The seed 4 and 5 refits score 52 recorded against the selected one's 51. They are a seed check,
    not a selection.
- **Checks:** every step's canaries matched, and so did its parity row (expert v3 on development
  row 52, against Phase 2 run 3).

### What it shows

- **The learner loses 2 rows to the expert and gains none.**
  - It fails all 8 of the expert's failing rows, plus rows 52 and 66, both
    `la-nb-pasadena-model-merge-focused-1of16-medium`.
  - Rows 52 and 66 fail at all 3 training seeds, on rolling merge quality.
  - The cause is one over-long gap choice at the first decision; see
    [Diagnosis of rows 52 and 66](#diagnosis-of-rows-52-and-66-uncharged).
- **DAgger added no closed-loop passes here.** The refits scored 60, 59 and 59 after cloning's best
  of 60, while held-back gap accuracy rose from 0.72 to 0.76. Apart from those 2 rows, the remaining
  failures are the expert's own.
- **The gap choice is the hard one to copy.**
  - Held back, lane intent is copied at 99.6% or better and the offset always (the expert's is
    always 0). The gap is copied at 0.71–0.76.
  - On development rows, the learner's gap differs from the rule's at 34–47% of the decisions where
    a gap applies. The closed-loop result is close anyway; why hasn't been checked.
- **No new kind of failure:**
  - on development rows, no learner collided, stopped on a free road, or had a cut-in or emergency
    in a one-car LA case;
  - the learners' cut-ins are on rows 28 and 37, where the expert also cuts in;
  - the 300 DAgger rollout episodes had no collision.

### Diagnosis of rows 52 and 66 (uncharged)

The user asked for it on November 26 ("Yes, diagnose"). It uses the recorded traces and level2 logs
only, so nothing was simulated or charged. Both rows are
`la-nb-pasadena-model-merge-focused-1of16-medium`: one controlled car (`agent_0`) at a yield entry.

- **What fails:** rolling merge quality, which this one car must pass.
  - **Expert:** it merges at once (graded window from 0.2 s) and matches the speed reference at 100%
    of the window.
  - **Learners:** every one of the 8 merges 14–16 s later (windows 14.2–30.1 s and 15.7–31.7 s) and
    matches only 0.869 and 0.727, below the gate's 0.9.
  - The car's other checks pass: no stop in the window, jerk at most 3, no override.
  - On row 38 (the same case at another seed) the expert also waits, and it fails with exactly the
    learners' numbers.
- **The cause is the first decision, at t = 0.1 s.**
  - The rule's label is gap 0 (take the first acceptable gap), and the learner chooses gap 3 (let 3
    stream cars pass).
  - The car then slows to wait. From t = 0.2 s, the rule's own label for that new state is 3 as well.
  - So one early choice commits the car to the long wait.
  - All 8 learners (3 cloning fits, 3 DAgger refits, 2 final refits) drive identical traces on both
    rows. More fits or more DAgger iterations of this kind would not change it.
- **Why only over-long choices matter.**
  - The executor takes the first acceptable gap at or after the chosen one. So any choice at or below
    the rule's label gives the expert's own plan. This is tested on 3,000 random streams
    (`tests/test_v15_level2.py`).
  - In the selected learner's 4,901 gap decisions on the 70 development rows, 2,970 equal the label.
  - 677 are below it; in 670 of those the executor's plan lets the same number of cars pass as the
    rule's.
  - 1,254 (26%), in 19 rows, are above it; only these can change the driving. A first-decision choice
    of 3 against a label of 0 also happens on rows 25, 50, 51 and 64, which still pass.
- **Why the learner chooses 3 there.**
  - On round 24's training row for this case (row 6), the rule's first-decision label is mixed across
    the 25 seeds in the data: 0 in 11 and 3 in 12.
  - The car's speed and distance to the stop line overlap between the two groups (about 36–52 m), so
    the stream cars' positions decide it.
  - The learner gets those training seeds right, but not development seeds 6952004 and 6952005.
  - Not checked: whether its 124 inputs carry enough about the stream cars to tell the cases apart.
    The input layout spans several environment versions.
- **A design fault in the labels** (mine, from the build):
  - The label is the number of cars the rule's plan lets pass, `min(len(waited_for), 3)`. That is the
    largest of the choices that all give the expert's plan, so every error upward changes the driving.
  - The smallest such choice, 0 (no extra car), would give the expert's gap behaviour exactly in every
    state. The gap would then be left for Phase 4 to learn when skipping a gap helps.
- **Ways to fix it** (each needs a charged development-row evaluation, so each needs you):
  1. relabel the gap as 0 and refit on the same data (no new rollouts);
  2. keep the labels but penalise choices above the label more than choices below it;
  3. give Level 2 the stream cars' arrival times as inputs, if they are missing.

  Until then, the learned gap choice can only add waiting compared with the expert.

### The relabel step (November 26)

The user approved it in this side session: "do the relabel fix first then work on the phase 4
budget". Its configuration and expected charge were recorded in the master before it ran. Same
settings as the other steps: SUMO (libsumo), IDM, 16 workers at below-normal priority, the shared
lock, development rows only.

- **What ran:** one refit of the selected learner's data (step 1 plus DAgger 1, 178,746 samples) at
  its fit seed 2, evaluated on the 70 development rows.
  - The gap label is 0 wherever the gap applies; the lane and offset labels are unchanged.
  - Only the gap labels differ from `l2:5aebbf7d8a40d2b9`'s fit, so the two are a matched pair.
  - The recorded sample files are unchanged. `relabel_gap` in `experiments/v15_level2_train.py`
    applies the relabel when they're loaded. Configuration: `configs/evaluation/v15-phase3-relabel.json`
    (sha256 `da83cfef…`).
- **Result:** `l2:bc7c050f6d3da3ce` scores **62 of 70 re-baselined** (79–94%) and **54 recorded**
  (66–85%), the expert's numbers exactly.
  - **Paired with the expert:** 62 rows both pass, none only one, 8 neither; 0.0 points (−4.9 to
    +4.9), p = 1.
  - **Paired with the selected learner:** it also passes rows 52 and 66, and loses none.
  - **Failing rows:** the expert's 8 (0, 8, 9, 14, 22, 24, 37 and 38), all on rolling merge quality.
  - **Events:** no collision, free-road stop or emergency. The cut-ins are on rows 28 and 37, as the
    expert's are.
- **How close it is to the expert:**
  - it drives the expert's identical trace (the same trace hash) on 64 of the 70 rows, where the
    selected learner did on 52;
  - on the other 6 (rows 2, 13, 30, 41, 43 and 69), the first difference is a lane-intent choice, and
    each row's verdict is the expert's;
  - its lane intent differs from the rule's at 0.6% of decisions, and its offset never;
  - held back, it copies lane intent at 99.7% and the gap and offset at 100%.
- **What it means:**
  - The learned gap choice is now always 0, so the car takes the rule's gap every time. The gap still
    differs from the rule's recorded label at 50% of the 4,292 gap decisions, but each of those
    choices is 0 where the rule's count is higher, which gives the same plan. That is by design.
  - Learning when skipping a gap helps is left to Phase 4.
  - **Not checked:** other training seeds. With a constant gap, only the lane and offset choices can
    vary with the seed.
- **Charge:** 139,117 decisions (3,487 of them the canary), 72 evaluation episodes and 107.6 s of
  clock time (34.4 s of it the fit). The estimate was 137,494 decisions and 165.8 s. The canary and
  the parity row matched, and the lock was released.

### The stop point, the fixes and a ledger correction

- **Step 1 stopped at a stop point:** the expert had 18 free-road stops in 17 perturbed training
  episodes, and none in unperturbed ones.
  - All 18 began during an injected slow-down to 0 m/s. In 13, the car pulled away within 0.5 s of
    the slow-down ending.
  - In 5, the expert held the car about 15 m short of the yield entry's stop line for 3.8–9.7 s more,
    waiting for the stream cars it had chosen to let pass.
  - The user chose to continue with all the data, keeping the 361 samples from those waits.
- **The expert on unperturbed training rows:** 50 of 100 under round 24's gates (40–60%), mostly on
  rolling merge quality. No collision and no free-road stop.
- **Fixes during the run,** each committed before the next charged step:
  - a crash in the closing report. It came after step 1's charge had closed, so nothing extra was
    charged;
  - DAgger 1 would have rolled out the last cloning fit, not the best one ("Up to 4 iterations from
    the best fit");
  - DAgger 1's first attempt failed in 8 s, because the workers couldn't find the cloning learner's
    weights. The runner now re-publishes the rollout learner into each run with a hash check. It
    also refuses to dispatch any learned driver whose weights aren't published.
- **Ledger correction:** that attempt's segment had been charged the runner's estimate for a failed
  batch, 184,076 decisions (175,310 expected × 1.05). Only the parity episode had run, 191 decisions.
  - At the main session's request, the segment was corrected openly to 191 decisions, under the
    lock.
  - The original value, the reason and the evidence are in its `corrections` list.
  - Its 8.1 s of clock time stays.

### Ledger

| | Before Phase 3 | After |
|---|---|---|
| Study time | 60,687.8 of 64,700 s (4,012.2 s left) | 62,181.1 of 64,700 s (2,518.9 s left) |
| Training decisions | 6,722,376 of 6,871,150 | unchanged |
| Phase 3 decisions | 0 of 2,700,000 | 2,122,682 (577,318 unused) |
| Phase 3 clock time | 0 of 3,000 s | 1,493.3 s (1,506.7 s unused) |
| Diagnostic episodes | 173 of 174 | unchanged |

- **The "After" column includes the relabel step.** Without it: 62,073.5 s of study time, and
  1,983,565 decisions and 1,385.7 s for Phase 3.
- **Segments:** eight Phase 3 `v15_evaluation` segments, all category preparation. Seven are
  complete, and DAgger 1's first attempt failed (corrected as above). In all: 505 training and 644
  evaluation episodes, and 9 fits (289.9 s).
- **The lock:** released.

### What's next (your decision)

- **Phase 4** (anchored MAPPO on Level 2, at 3 training seeds): see the
  [Phase 4 proposal](#phase-4-proposal-november-26-anchored-rl-on-level-2-for-your-approval) and
  decision 9.
- **The held-out check** (Stage 5) needs your authorization.
- Nothing more runs until you decide.

## Phase 4 proposal (November 26): anchored RL on Level 2, for your approval

**Step A approved on November 26** ("Yes, run step A", decision 9) and run in full. Its probe stopped
at its stop rule after the first batch, and the second batch ran after you approved it. See
[Phase 4 step A result](#phase-4-step-a-result-november-26). Step B was skipped on November 26, and
the +1 offset check ran instead; see [Phase 4 offset check](#phase-4-offset-check-november-26). The
proposal is kept below as written.

**Proposal only.** Nothing here has been built or run, and nothing is charged until you approve it.
The user asked for it on November 26: "do the relabel fix first then work on the phase 4 budget".
All runs would be SUMO (libsumo), not physics, with IDM background only. The held-out rows are never
built or run.

**What Phase 4 does** (Stage 4): MAPPO trains Level 2 only (the gap, lane intent and speed offset).
It starts from the Phase 3 policy and is held close to it. Level 1, the rule executor, is unchanged.
- **Starting point and anchor:** the relabel learner `l2:bc7c050f6d3da3ce`, at 62 of 70 re-baselined
  and 54 recorded, the expert's numbers.
- **Exit check:** beat the Phase 3 policy across 3 training seeds on the 70 development rows, with no
  safety event worse.

### How much room there is

Phase 4 can only gain on the 8 rows that the expert and the starting policy both fail: rows 0, 8, 9,
14, 22, 24, 37 and 38. All 8 fail one gate, rolling merge quality: speed match of at least 0.9 from
8 s before the merge completes to the stable merge.
- **Phase 2's audit** put them down to:
  - following headway on rows 0, 9, 14 and 24 (Level 1's following, not changed);
  - yield-entry waits on rows 8, 22, 37 and 38, where samples before the stop line are graded by
    design.
- **What Level 2 can do there** (from the code, `traffic/v15_level2.py`):
  - **the gap** can only make a car wait longer, never enter earlier, so it can't shorten a
    yield-entry wait;
  - **a speed offset** (−2 to +2 m/s) changes the target speed, but never raises it above the larger
    of the plan's target and the free-road target;
  - **a lane intent** can move a car away from a slow leader, inside the mask and MOBIL's safety
    test.
- **Not checked:** whether any Level 2 choice can lift those rows' speed match to 0.9. If none can,
  Phase 4 can't meet its exit check on these rows, however long it trains. The reward below doesn't
  target speed match directly either.
- **So the proposal starts small:** a probe and a seed check (step A) before the training spend
  (step B).

### Built first (uncharged)

- **For step A:** scripted Level 2 variants as pool drivers: the rule's choices with a constant
  offset of −2, −1, +1 or +2 m/s, or with the gap one car later than the rule's.
- **For step B:**
  - **a stochastic Level 2 pool driver.** It samples the three choices from the network and holds
    them for 0.5 s (5 decisions). For each car it records the inputs, the choices and their
    log-probabilities, the centralized state and the event rewards;
  - **a Level 2 PPO trainer** (plain PyTorch, CPU, one thread, deterministic):
    - the Phase 3 network as the actor;
    - a centralized critic of the existing `MaskedPopulationCritic` design, on the v15 environment's
      state;
    - a KL term toward the Phase 3 policy that decays slowly;
  - **the reward's event terms,** each logged:
    - progress and merges completed;
    - cut-ins, emergencies and collisions;
    - stuck and comfort events;
    - a cooperation term for gaps opened.

    Their weights go in the resolved configuration before anything is charged;
  - **stop rules and screens:** free-road stops, collision regression, a speed-match floor, and the
    checkpoint screens;
  - **a Phase 4 ledger category,** with its own allowance as Phases 1–3 had. v14's training-decision
    total has 148,774 left.
- **Tests,** the uncharged code checks on v14-validation rows 0 and 4, and an uncharged cost check of
  the stochastic driver and one PPO update.

### What the estimate is built from

Measured in Phase 3:
- **One seed set of the 20 training rows:** about 34,800 decisions (step 1: 34,864). At one Level 2
  action per car every 0.5 s, that is about 7,000 Level 2 actions.
- **Training row 17** (the 12-car cruise) makes about 11,200 decisions per episode, a third of each
  seed set, and takes about 72 s on one worker.
  - So no rollout batch finishes in much under 75 s.
  - A batch of 5 seed sets (100 episodes) costs little more clock time than one: DAgger's three
    rollout batches made 173,205–174,237 decisions in 76.7–103.8 s.
- **A development-row evaluation:** 135,630–136,503 decisions in 67.6–98.5 s.
- **The relabel configuration's seed check** would repeat the final check's cost: 276,036 decisions
  and 201.7 s for 2 refits and 2 evaluations.

Not measured:
- **A PPO update with a critic.** The estimate allows 10 s per update on about 35,000 Level 2 actions
  (a cloning fit runs at about 1.1 s per pass over 160,000 samples). The uncharged cost check measures
  it.
- **How many updates** anchored PPO needs to change anything. Nothing here measures that; it is the
  main uncertainty in step B's size.

### Step A: the probe and the seed check (no raise needed)

| Part | Work | Episodes | Decisions (estimate) | Clock time (estimate) |
|---|---|---|---|---|
| Probe | The 5 scripted variants on the 8 failing development rows | 40 | ~84,000 | ~90 s |
| Seed check | The relabel configuration refit at fit seeds 4 and 5, each evaluated on the 70 rows | 140 | ~276,000 | ~200 s |
| **Total** | Two runs, each with its canary and parity row | **~184** | **~360,000** | **~290 s** |

- **What it tells you:**
  - **The probe:** whether any simple Level 2 choice passes any of the 8 failing rows. If none does,
    I'd recommend skipping step B and going to Stage 5. RL could still find a state-dependent choice
    that the constant variants miss, so a negative probe is a strong hint, not proof.
  - **The seed check:** Phase 3's baseline across 3 seeds, which step B's exit check needs.
- **Charge:** the seed check from Phase 3's unused allowance (577,318 decisions and 1,506.7 s). The
  probe as Phase 4's first segment, with its own allowance of at most 100,000 decisions and 150 s.
  Both use study time: about 2,230 s would be left.
- **Repeated looks:** the probe is another look at 8 development rows. It trains nothing.

### Step B: anchored MAPPO, sized after the probe

Each training seed starts from the Phase 3 policy. Each iteration rolls out the current policy on 5
new seed sets of the training rows (100 episodes), then makes one PPO update.

| Size | Work | Episodes | Decisions (estimate) | Clock time (estimate) |
|---|---|---|---|---|
| **Recommended** | 3 training seeds × 12 iterations; each seed evaluated on the 70 rows after iterations 4, 8 and 12 | ~4,280 | ~7.6 million | ~4,500 s |
| **Minimum** | 3 training seeds × 6 iterations; evaluated after iterations 3 and 6 | ~2,250 | ~4.0 million | ~2,400 s |

How the recommended size adds up:
- rollouts: 36 × ~174,000 decisions and ~92 s;
- updates: 36 × ~10 s;
- evaluations: 9 × ~136,500 decisions and ~80 s;
- canaries, parity rows and start-up: ~110,000 decisions and ~100 s.

The minimum size is the same with 18 rollouts, 18 updates and 6 evaluations.

- **Proposed caps** (the estimate plus about 20% in decisions and 25% in time):
  - **Recommended:** at most 9,200,000 decisions and 5,700 s.
  - **Minimum:** at most 4,900,000 decisions and 3,100 s.
- **The study total needs a raise either way.**
  - After step A about 2,230 s would be left.
  - Stage 5 needs some too. Running the held-out rows once costs about 82,000 decisions per driver
    on 42 rows. A physics drive took 55–108 s each in v14, and only 1 of 174 diagnostic episodes is
    left. I'd keep about 600 s for it.
  - So: **recommended**, a raise of about 4,100 s (from 64,700 to about 68,800 s); **minimum**, about
    1,500 s (to about 66,200 s).
- **Selection:** by closed-loop pass rate on the 70 development rows under the frozen re-baselined
  gates, then the recorded gates, never by training reward. A change is accepted only if it beats
  Phase 3 across the 3 seeds, with no safety event worse.
- **Same settings as Phases 1–3:**
  - at most 16 workers at below-normal priority, with nothing heavy alongside;
  - the shared lock;
  - each run's resolved configuration and expected charge in the coordination master before it
    runs.
- **Stop and report if:**
  - the next run won't fit;
  - the lock is held;
  - a canary or parity check mismatches;
  - tests fail;
  - a trained policy collides, stops on a free road, or causes a cut-in or emergency in a one-car LA
    case;
  - the uncharged cost check puts the chosen size over its cap.
- **Why the minimum is weaker:** with 6 iterations per seed, a policy that changes slowly (as the
  anchor intends) may not move off the Phase 3 policy at all. Then step B would spend about 2,400 s to
  confirm the start.

### My recommendation

1. Approve **step A now** (about 360,000 decisions and 290 s, no raise).
2. Decide on step B after the probe. If the probe finds no room, skip step B and go to Stage 5 with
   the Phase 3 policy.

That keeps the large spend behind evidence that it can pay off.

## Phase 4 step A result (November 26)

**User decisions:**
- **Step A:** "Yes, run step A", typed in the Claude side session. The main Claude session also
  relayed "Yes, approve step A".
- **The probe's second batch:** "Yes, run batch 2", relayed by the main Claude session.
- **Settings:** as decision 9 describes. SUMO (libsumo), not physics, with IDM background, 16 workers
  at below-normal priority, under the shared lock.
- **Rows:** development rows only. The held-out rows were not built or run.
- **Nothing was trained by RL.**

**Step A ran in full.**
- **First batch:** it stopped at the probe's stop rule. Scripted variants stopped on a free road on 3
  rows, one of them a one-car LA case. Nothing was retuned.
- **Second batch** (offsets +1 and +2): it ran after you approved it, as a step of its own, with no
  stop-rule event.

### The seed check

- **Ran:** the relabel configuration refit at fit seeds 4 and 5, each evaluated on the 70 development
  rows (`configs/evaluation/v15-phase3-relabel-seeds.json`, sha256 `4d1d3165…`).
- **Result:** both score 62 of 70 re-baselined and 54 recorded: `l2:85ae5076488ee598` at seed 4 and
  `l2:dc004c196a2efbfb` at seed 5.
  - Each fails the expert's 8 rows and no others, with no new kind of failure.
  - Their cut-ins are on rows 28 and 37, as the expert's are.
  - They drive the expert's identical trace on 66 and 65 rows.
- **So the relabel configuration scores 62 (54) at all 3 fit seeds** (2, 4 and 5). That is Phase 4's
  baseline.
- **Charged** from Phase 3's unused allowance:
  - 274,702 decisions (3,487 of them the canary), against 273,997 expected;
  - 142 evaluation episodes;
  - 196.7 s of clock time (73.7 s of it the 2 fits), against 305.8 s expected.

  The canary and the parity row matched.

### The probe

**What ran:** the scripted variants on the 8 development rows that the starting policy
`l2:bc7c050f6d3da3ce` and the expert both fail (0, 8, 9, 14, 22, 24, 37 and 38).
- **Each variant** is the rule's Level 2 choices with one change, for every car and the whole
  episode.
- **Configurations:**
  - `configs/evaluation/v15-phase4-probe.json`: the first batch;
  - `configs/evaluation/v15-phase4-probe-faster.json`: the second.
- **Checks:** in both batches, the canary and the parity row (52) matched.
- **The rows' cases:**
  - rows 0 and 14: the I-110 merge;
  - rows 8 and 22: Pasadena northbound, 8 of 16 cars controlled;
  - rows 9 and 37: Adams southbound, 8 of 16;
  - rows 24 and 38: Pasadena northbound, 1 of 16. These are the probed rows' one-car LA cases.

| Variant | Passes, re-baselined | Passes, recorded | Stop-rule events | Other events |
|---|---|---|---|---|
| Gap +1 (one car later) | row 37 | none | none | none |
| Offset −1 m/s | row 9 | none | a free-road stop on row 37 | 2 stops in traffic on row 8; a cut-in on row 37 (the expert has it too) |
| Offset −2 m/s | none | none | free-road stops on rows 9, 37 and 38 (row 38 is a one-car LA case) | 4 stops in traffic on row 8; a cut-in on row 37; 2 new failure kinds on row 14 (per-car route comfort or control, raw request jerk) |
| Offset +1 m/s | row 37 | row 37 | none | a new failure kind on row 24 (per-car route comfort or control) |
| Offset +2 m/s | row 37 | row 37 | none | the same on row 24 |

No probe episode had a collision or an emergency. Offsets +1 and +2 had no events at all.

**Rows with room** (a variant passes the row under the re-baselined gates): rows 9 and 37.
- **Row 9:** offset −1 lifts all 3 failing cars' speed match to 1.0, with no events. The expert's
  worst car there scores 0.870.
- **Row 37:** three variants pass it.
  - **Gap +1** lifts agent_0's speed match to 0.907, just over the gate's 0.9 (the expert's: 0.894).
  - **Offsets +1 and +2** lift it to 1.0 and pass the recorded gates too. The two drive the same
    trace there.
  - None of the three has an event there. The expert's cut-in on that row doesn't happen.
- **Recorded gates:** only row 37 passes them, with offset +1 or +2.
- **Row 24 with the faster offsets:** the row also fails per-car route comfort or control. agent_0's
  minimum headway falls under that gate's 1.5 s; its other checks weren't looked at.

  | Driver | Minimum headway |
  |---|---|
  | Starting policy | 1.88 s |
  | Offset +1 | 1.35 s |
  | Offset +2 | 1.19 s |
- **Speed match of 0.9 isn't enough on its own.** On rows 14 and 24 a failing car reaches it, and
  both rows still fail rolling merge quality:
  - row 14: 1.0 with offset −1;
  - row 24: 0.938, the expert's own.

  Which other part of that gate fails isn't checked.
- **Speed match of each row's worst failing car**, against the gate's 0.9:

  | Row | Expert v3 | Gap +1 | Offset −1 | Offset −2 | Offset +1 | Offset +2 |
  |---|---|---|---|---|---|---|
  | 0 | 0.323 | 0.323 | 0.285 | 0.087 | 0.175 | 0.202 |
  | 8 | 0.731 | 0.652 | 0.894 | 0.609 | 0.721 | 0.714 |
  | 9 | 0.870 | 0.863 | 1.000 (passes) | 0.714 | 0.863 | 0.863 |
  | 14 | 0.669 | 0.669 | 1.000 | 0.164 | 0.540 | 0.400 |
  | 22 | 0.851 | 0.851 | 0.882 | 0.683 | 0.850 | 0.850 |
  | 24 | 0.938 | 0.775 | 0.919 | 0.484 | 0.913 | 0.981 |
  | 37 | 0.894 | 0.907 (passes) | 0.845 | 0.845 | 1.000 (passes both) | 1.000 (passes both) |
  | 38 | 0.727 | 0.727 | 0.689 | 0.130 | 0.727 | 0.727 |

  - The expert's column is Phase 2 run 3. "Passes" means the row passes under the re-baselined gates;
    "passes both" means the recorded gates too.
  - Offset −1 raises the worst car on 4 rows (8, 9, 14 and 22). Offset −2 lowers it on all 8.
  - Offset +1 raises it only on row 37, and offset +2 on rows 24 and 37.
  - Gap +1 drives the starting policy's identical trace on rows 0, 14, 22 and 38, so the later gap
    changed nothing there. Offsets +1 and +2 do the same on row 38. Why isn't checked; for the
    offsets, the cap at the larger of the plan's and the free-road target would explain it.

**The free-road stops,** from each car's Level 2 trace:

| Variant, row | Car | Stopped for | Where it stopped |
|---|---|---|---|
| Offset −1, row 37 | agent_0 | 6.4 s | 6.7 m short of the yield entry's stop line; waiting for 1 main-road car and 2 controlled cars |
| Offset −2, row 37 | agent_0 | 7.5 s | 7.3 m short, waiting for the same 3; no leader in its lane |
| Offset −2, row 38 | agent_0 | 6.8 s | 11.9 m short, waiting for 3 main-road cars |
| Offset −2, row 9 | agent_6 | 4.9 s | 50.5 m short, 43 m behind a ramp car; waiting for 2 main-road cars |

- **In each,** Level 1's executor set the target to 0 at a yield entry it hadn't committed to. The
  event classifier counts it as a free-road stop because no leader was close ahead.
- **My reading,** not verified beyond these traces: a slower approach reaches the entry later, misses
  the gap the rule takes, and waits at the line.
- **Not checked:** why row 9's car stopped 50 m back.
- **The same wait in Phase 3:** step 1 found it for the expert after an injected slow-down. The
  expert held cars about 15 m short of the stop line for 3.8–9.7 s.
- **The starting policy** has none of these events on these rows; its only event there is the row 37
  cut-in.

**The cut-ins on row 37** (Adams southbound, 8 of 16: not a one-car LA case, so outside the stop
rule):
- **What the metric counts:** a background car braking harder than 3.0 m/s² behind a controlled car,
  within the metric's gap limit. It doesn't record whether a lane change came first.
- **With offsets −1 and −2:** the same event in both. The ramp car `ramp_1` braked at up to
  3.05 m/s², 15.1–12.4 m behind agent_0, from 5.0 to 5.4 s (5 charged decisions).
- **The starting policy's event** has the same pair from 4.9 to 5.7 s: up to 3.21 m/s², 17.5–11.7 m
  behind, 9 charged decisions. So the offsets made it milder, and gap +1 removed it.
- **It comes before the free-road stops** (agent_0 stopped from 8.0 s and 7.1 s), so the stops didn't
  cause it.
- **Not checked:** what agent_0 was doing at 5 s.

### The stop, a crash and a configuration hash correction

- **The stop:** the first batch's free-road stops met the stop rule, so the second batch (offsets +1
  and +2) wasn't dispatched then. Nothing was retuned. The second batch ran later, after your
  approval (below).
- **A crash first:**
  - After the first batch was charged, the runner failed while recording it: `KeyError:
    'training_episodes'`. Phase 3's batch accounting needs that field, and the Phase 4 segment lacked
    it.
  - The batch's 25 results were all written. The segment is recorded as failed with its real counts
    (below), and the lock was released.
  - **The charge is checked.** The ledger had already added the batch when the error came, so only
    the field itself (0 here) was missing. No estimate was charged. The segment's figures equal the
    batch's:
    - 58,001 decisions (54,514 plus the 3,487 canary);
    - 26 evaluation episodes (25 plus the canary);
    - 335.8 worker-seconds and 2,002.5 simulated seconds;
    - 47.7 s of clock time, to the failure.
  - The stop rule would have stopped the run at that point anyway.
  - Fixed, uncharged, in `1f57be7`, with the two changes below. 122 v15 tests pass.
- **Hash correction:** the master's probe entry cites sha256 `44ec066f…`. That was the hash of the
  configuration as resolved, with integer row keys, before it was written.
  - The pinned file holds the keys as strings. Its hash is
    `3c622365fffa03ba8a974d4edd0888d03fbc7c4d37b25c2838c06195d4b7b41a`: the one the run checked and the
    ledger records. The content is the same.
  - The resolve step now prints the pinned file's hash. As for every v15 configuration, it is the
    sha256 of the configuration's canonical JSON (sorted keys), not of the file's bytes.
- **The report** now covers a run that stopped before a batch, and lists the variants not run.

### The second batch (November 26)

- **Approved by you:** "Yes, run batch 2", relayed by the main Claude session.
- **Built first, uncharged:** step `probe-faster` in `experiments/v15_phase4.py` (commit `560699f`;
  124 v15 tests pass). It runs only the pinned probe configuration's second batch, with its own canary
  and parity row. It refuses to run unless:
  - the first batch's results are all there;
  - its Level 2 source files match the first batch's byte for byte;
  - its settings and benchmark are the same.

  The step `probe` now refuses to run again.
- **Configuration:** `configs/evaluation/v15-phase4-probe-faster.json`, sha256
  `2bceef41fbeb5a866a1b0be55373102dd1646d63768823f20e96ec7d452c1b83`. It was recorded in the
  coordination master before it ran.
- **Expected:** 36,309 decisions (the batch's 36,118 plus the parity row's 191). Projected: 40,973.
  That is the expected figure times the first batch's measured 1.0747, plus the 5% margin. It was
  checked against the 41,999 left before dispatch.
- **Charged:** 38,632 decisions (3,485 of them the canary), 18 evaluation episodes, and 55.7 s of
  clock time against 52.9 s expected.
- **Checks:** the canary and the parity row matched. No stop-rule event. The lock was released.

### Ledger

| | Before step A | After |
|---|---|---|
| Study time | 62,181.1 of 64,700 s (2,518.9 s left) | 62,481.2 of 64,700 s (2,218.8 s left) |
| Phase 3 decisions | 2,122,682 of 2,700,000 | 2,397,384 (302,616 unused) |
| Phase 3 clock time | 1,493.3 of 3,000 s | 1,690.0 s (1,310.0 s unused) |
| Phase 4 decisions | none | 96,633 of step A's 100,000 |
| Phase 4 clock time | none | 103.3 of step A's 150 s |

- **Segments:** three `v15_evaluation` segments, all category preparation.
  - **The seed check:** Phase 3, complete.
  - **The probe's first batch:** Phase 4's first segment, failed as above. It charged 58,001
    decisions (3,487 of them the canary), 26 evaluation episodes and 47.7 s; it expected 53,968
    decisions.
  - **The probe's second batch:** Phase 4's second segment, complete. 38,632 decisions, 18
    evaluation episodes and 55.7 s.
- **The lock:** released.

### Step B: my recommendation

- **What the probe says about room:** 2 of the 8 rows, 9 and 37.
  - **Row 37:** a constant +1 offset passes it under both gates, with no events.
  - **Row 9:** only offset −1 passes it, under the re-baselined gates only. Slower approaches cause
    the stop-rule events on 3 rows, row 37 among them.
  - **The others:** no variant passes rows 0, 8, 14, 22, 24 or 38.
- **The best step B could do** on this evidence: a policy that slows down only in row 9's situation
  and takes row 37 the faster way. That scores 64 of 70 re-baselined and 55 recorded. Phase 3's
  policy scores 62 and 54.
- **My recommendation: skip step B.** Two rows don't justify a raise of about 4,000 s. Instead:
  1. **Check the constant +1 offset on all 70 development rows,** with no training.
     - **Why:** the probe ran it only on the 8 failing rows. If it keeps the other 62 and adds row
       37, it scores 63 (55) as a fixed scripted change to Level 2. If it loses rows, only a policy
       that chooses by situation could keep the gain.
     - **The risk:** on row 24 the faster approach cut the headway under 1.5 s, so it may lose rows.
     - **Cost:** about 140,000 decisions and 100 s (Phase 3's 70-row checks: 135,630–136,503
       decisions in 67.6–98.5 s). That's a small new Phase 4 allowance, with no raise; about 2,120 s
       of study time would be left.
     - **Caveat:** it's another look at the development rows, chosen after the probe. Only the
       held-out check can confirm it.
  2. **Then the held-out check (Stage 5):** the Phase 3 policy `l2:bc7c050f6d3da3ce`, plus the +1
     variant if it wins on the 70 rows. It needs your authorization, and about 600 s.
- **The sizes, if you want step B anyway.** Each keeps about 600 s for Stage 5, and each raise grows
  by about 100 s if the 70-row check runs first.

  | Size | Phase 4 allowance | Study total raise |
  |---|---|---|
  | Recommended (3 seeds × 12 iterations) | 9,200,000 decisions and 5,700 s | about 4,080 s (to about 68,780 s) |
  | Minimum (3 seeds × 6 iterations) | 4,900,000 decisions and 3,100 s | about 1,480 s (to about 66,180 s) |

- **Settle one rule before step B.** Its stop rule stops on a trained policy's free-road stop.
  - I'd read "trained policy" as each evaluated checkpoint.
  - The probe shows that slower approaches produce exactly these stops at yield entries, and
    exploration will try slower approaches.
  - So the rule should say whether it also covers the training rollouts.

## Phase 4 offset check (November 26)

- **Approved by you:** "Yes, run the check", typed in this side session. The main Claude session
  recorded the same answer in the coordination master, with step B skipped (decision 9).
- **What it is:** the probe's constant +1 m/s offset (`l2:probe-offset-p1`: the rule's choices, with
  the speed target 1 m/s higher) on every development row, with no training.
- **Settings:** SUMO (libsumo), not physics, with IDM background, 16 workers at below-normal
  priority, under the shared lock. Development rows only; the held-out rows were not built or run.

### Built first (uncharged)

- **Step `offset-check`** in `experiments/v15_phase4.py` (commit `46ea143`; 125 v15 tests pass).
  - It runs the variant on the 62 development rows the probe didn't, with its own canary and the
    parity row.
  - The other 8 rows (0, 8, 9, 14, 22, 24, 37 and 38) come from the probe's second batch, whose
    report it pins by sha256. It refuses to run unless:
    - that batch's segment is complete and its parity matched;
    - its Level 2 sources match byte for byte;
    - its settings and benchmark are the same.
  - The report covers all 70 rows under both gates, against expert v3 and the Phase 3 policy.
- **Checked before the build:** expert v3 and the Phase 3 policy `l2:bc7c050f6d3da3ce` give the same
  verdict on every development row under both gates.
- **Configuration:** `configs/evaluation/v15-phase4-offset-check.json`, sha256
  `cb353180ee1ccddc6bf6a0a73f959493b823eee8c75ccd6eddd70dd4749e52b1`. It was recorded in the
  coordination master before it ran.
- **Its own allowance:** 140,000 decisions and 100 s. That fits inside the main session's
  authorized "at most 160,000 decisions and 150 s", with no raise.
- **Expected:** 121,044 decisions and 88.7 s. The decisions are expert v3's 117,769 on the 62 rows,
  the parity row's 191 and the canary's 3,084. Projected: 127,096 decisions, with the 5% margin.
- **Reading, set before the run:** the variant joins the held-out check only if both hold:
  - it passes every row expert v3 passes, under the re-baselined gates;
  - it has no stop-rule event.

### Result

| | Re-baselined | Recorded |
|---|---|---|
| Phase 3 policy (and expert v3) | 62 of 70 | 54 of 70 |
| The +1 offset | 61 of 70 | 54 of 70 |

- **Gained:** row 37, under both gates. That result comes from the probe's second batch.
- **Lost:** rows 48 and 50 under the re-baselined gates. Row 50 is also lost under the recorded
  gates; row 48 already failed them.
- **Every other row:** the same verdicts as the Phase 3 policy, under both gates.
- **No stop-rule event:** no collision, free-road stop, cut-in or emergency on any row. The canary and
  the parity row matched.
- **By the reading set before the run, the +1 variant doesn't join the held-out check.**

### The two lost rows, traced

Both fail the headway gate (1.5 s): the faster target brings the car too close to the one ahead.

- **Row 50** (LA northbound Pasadena, 8 controlled cars).
  - **What fails:** only agent_0, on headway: 1.42 s, against expert v3's 1.60 s. Every other check
    and every other car is unchanged.
  - **Where:** 3.0 s into the episode on the on-ramp, behind agent_3 (another controlled car). The
    gap was 22.6 m at 15.9 m/s, against expert v3's 24.5 m at 15.3 m/s.
  - **Why:** the offset raises agent_0's target by 1 m/s from the first decision, so it closes on
    agent_3.
- **Row 48** (the 2-lane ramp case `ramp-eval-validation-2-lane-ramp180-ramp-started`, one
  controlled car).
  - **What fails:** headway (0.31 s against 1.64 s), peak jerk (62 m/s³ against 3.0), the stable merge
    and the rolling merge quality.
  - **The merge:** the +1 car changed into the main lane 1.0 s later and 23 m further along than
    expert v3's car: 38.6 s at 288 m, against 37.6 s at 265 m. It was going 18.3 m/s, against 19.5.
  - **After the merge:** its merge slowdown ended 1 s later, so it was about 3 m/s slower than
    expert v3's car.
  - **The brake:** at 41.8 s a background car, main_5, appeared 5.3 m ahead of it in its lane. Within
    one 0.1 s step agent_0 went from +1.7 to −4.5 m/s², and was still at −4.5 at 42.0 s.
  - **main_5's lane change:** SUMO's lane-change log records it moving over from the next lane (reason
    "keepRight"), with agent_0 5.27 m behind; the log stamps it at 43.1 s. In expert v3's run, main_5
    never changed lanes.
  - **Not traced:** why the +1 car merged later.
- **The same pattern as row 24 in the probe** (headway 1.35 s): the faster target closes gaps.
- **Speed match on rows that still pass:** rows 20 and 34 dropped from 1.0 to 0.97 and 0.96, still
  above the 0.9 gate.

### Ledger

| | Before the check | After |
|---|---|---|
| Study time | 62,481.2 of 64,700 s (2,218.8 s left) | 62,535.6 of 64,700 s (2,164.4 s left) |
| Phase 4 decisions | 96,633 | 217,715 (the check: 121,082 of its 140,000) |
| Phase 4 clock time | 103.3 s | 157.8 s (the check: 54.4 of its 100 s) |

- **Charged:** 121,082 decisions (3,084 of them the canary), 64 evaluation episodes and 54.4 s. It
  expected 121,044 decisions and 88.7 s.
- **Segment:** Phase 4's third, complete.
- **The lock:** released.

### Stage 5: my recommendation

- **Go to the held-out check (Stage 5) with the Phase 3 policy `l2:bc7c050f6d3da3ce` alone.**
  - **Why:** no constant Level 2 change has beaten it on the development rows. The probe and this
    check found only row 37, and the +1 offset that takes it loses 2 others.
  - **What it tests:** the policy matches expert v3 row for row, so Stage 5 shows whether the learned
    Level 2 holds up on rows that nothing was chosen on.
- **What Stage 5 needs** (from the plan; it needs your authorization and its own sizing):
  - **The held-out rows, once:** about 82,000 decisions per driver on 42 rows.
  - **Physics drives** of the expert and the policy on the PhysX car on the LA model, each video
    labelled physics or kinematic replay. v14's took 55–108 s each, and only 1 of 174 diagnostic
    episodes is left.
  - **To decide in that sizing:** whether expert v3 also runs the held-out rows, for the paired
    comparison the benchmark reports.
- **Study time left:** 2,164.4 s, about 1,560 s more than the 600 s kept for Stage 5.
- **The +1 variant stays out.** A version that speeds up only in row 37's situation would be a rule
  fitted to one development row.

## Stage 5 proposal (November 26): the held-out check and physics drives, for your approval

**Proposal only.** You asked for it on November 26: "Yes, size it", relayed by the main Claude session.
Nothing here has been built or run, and nothing is charged until you approve it.
- **The held-out rows weren't built or opened for this sizing.** Every estimate comes from the
  development rows and the records of Phases 1–4.
- **Nothing is trained in Stage 5.** The Phase 3 policy `l2:bc7c050f6d3da3ce` is the only candidate.
  The +1 offset stays out, by the offset check's reading.

**What Stage 5 does** (from "Stage 5: physics and held-out evaluation" above):
- **Part 1, SUMO:** the policy drives the 42 reserved held-out rows (the 14 cases at seeds
  6953001–6953003) once.
- **Part 2, physics:** expert v3 and the policy each drive the PhysX car on the LA model, on a
  development row. Each video is labelled physics.

### Part 1: the held-out rows (SUMO)

**Which drivers run the held-out rows:**

| Option | Drivers on the 42 held-out rows | What it can tell you |
|---|---|---|
| **Recommended** | The policy, expert v3 and the round-16 policy | The benchmark's full report: pass rates, and the paired differences from expert v3 and from round 16 on the same rows |
| Middle | The policy and expert v3 | The pass rate, and the paired difference from expert v3 |
| Minimum | The policy alone | Its pass rate with an interval, compared only with its development numbers |

- **Why I recommend all three:**
  - **Expert v3 separates two explanations.** The policy matches expert v3 on every development row.
    A lower held-out pass rate on its own can't tell "the learned Level 2 doesn't carry over" from
    "the held-out seeds are harder". Expert v3 on the same rows tells them apart.
  - **Round 16 answers whether v15 beats v14's best** on rows nothing was chosen on. On the
    development rows, the policy passes 62 against round 16's 36 re-baselined, and 54 against 33
    recorded.
  - **The held-out rows are still looked at once:** all three run in one batch.
- **This needs your yes:** the plan says to run the held-out seeds "once, for the final candidate
  only", and `traffic/v15_benchmark.held_out_rows` says the same. Expert v3 and round 16 are fixed
  baselines, not candidates, and nothing is chosen from their results. Running them there still
  goes beyond that wording.

**Development checks, in the same batch:**
- **The policy on development row 3:** it must reproduce its Phase 3 result exactly. The pool's
  canary re-runs the shortest-horizon task that comes first, so this row goes first, and the canary
  re-runs it on another worker.
- **The parity row 52** (`l2:rule-v3`): as in Phase 4.
- **So every held-out row runs exactly once.** No canary falls on a held-out row.

**How the estimate is built:**
- **Each driver's decisions per case:** the mean over its 5 development seeds, times 3 held-out
  seeds, summed over the 14 cases.
- **The range:** each case's fewest and most decisions across its 5 seeds.

| Driver | Decisions on the 70 development rows | Held-out estimate (42 rows) | Range | Source |
|---|---|---|---|---|
| The policy | 135,439 | 81,263 | 76,341–88,650 | Phase 3, the relabel step |
| Expert v3 | 135,428 | 81,257 | 76,335–88,737 | Phase 2, run 3 |
| Round 16 | 134,530 | 80,718 | 73,569–88,353 | Phase 1 |

- **The development checks:** 6,359 decisions. That is row 3's 3,084 twice (the row and its canary)
  and the parity row's 191.
- **Projected:** the expected decisions plus 5%, as in Phase 4.
- **Clock time:** 15 s to start, plus the decisions at 1,900 per second, plus 10 s. The rate is the
  Phase 3 relabel step's, the slowest of Phase 3's one-batch evaluations; the offset check used it
  too, and its own run averaged about 2,220 per second.
  - No batch is shorter than its longest episode, at 140 decisions per worker-second.
  - That sets the minimum option's estimate: the policy's longest development row, 6,688 decisions
    ÷ 140 ≈ 48 s.

| Option | Episodes | Expected decisions | Projected | Range | Clock time | Proposed cap |
|---|---|---|---|---|---|---|
| **Recommended** | 129 | 249,597 | 262,077 | 232,604–272,099 | ~156 s | 300,000 decisions and 300 s |
| Middle | 87 | 168,879 | 177,323 | 159,035–183,746 | ~114 s | 200,000 decisions and 250 s |
| Minimum | 45 | 87,622 | 92,003 | 82,700–95,009 | ~73 s | 110,000 decisions and 150 s |

- **Episodes:** 42 per driver, plus the 2 development rows and the canary.
- **Each cap** covers the top of the range plus 5%, and the run's time reservation (its clock time
  × 1.6 + 30 s).
- **The charge:** Stage 5's own ledger allowance (phase 5), from the study's time, as Phases 1–4 had.

**Settings, as in Phases 1–4:**
- SUMO (libsumo), not physics, with IDM background only; no Krauss runs.
- At most 16 workers at below-normal priority, with nothing heavy alongside.
- Under the shared lock.
- The resolved configuration and expected charge go in the coordination master before it runs.

**Built first (uncharged):**
- **A Stage 5 runner.** It:
  - builds the held-out rows only when it runs, through `held_out_rows(cfg, authorized=True)` and a
    held-out row set in the pool (today the pool has none, and `held_out_rows` refuses without
    authorization);
  - pins the policy's network hash, expert v3's sources, round 16's hash (`policy:f0b11fa577b33763`),
    the frozen regrade and the benchmark manifest;
  - runs one batch with the development checks first;
  - refuses to run again once any held-out result exists.
- **Its report:**
  - each case (3 rows) and the pool, under the re-baselined gates and then the recorded ones;
  - Wilson intervals, and paired differences with exact McNemar tests (`traffic/v15_stats.py`);
  - events, and a comparison with the development rows.
- **Tests** that never build a held-out row: they check the guard and use development rows.

**Stop rules:**
- **Before dispatch:** stop and ask if the projection doesn't fit the cap, the lock is held, or a
  test fails. The run is never trimmed to fit.
- **A canary or parity mismatch:** the run is reported as failed. Nothing is re-run; you decide what
  comes next.
- **Events on held-out rows** (collisions, free-road stops, cut-ins, emergencies): they are results.
  They're reported, and nothing is retuned.
- **No held-out row is re-run** once its result is written, for any reason.

**Reading, set before the run:**
- **Holds up as well as its teacher:** the 95% interval of the policy's paired difference from
  expert v3 (re-baselined) includes 0. There must also be no collision, and no gate failure or event
  the policy never had on the development rows.
- **Better than v14's best:** the 95% interval of the paired difference from round 16 is above 0.
- **These are the study's final numbers for this policy.** Nothing is tuned on them.
- **The intervals will be wide.** With 42 rows, each row is about 2.4 points. For example, 37 of 42
  gives 75–95% (Wilson), against 79–94% for the policy's 62 of 70.

### Part 2: the physics drives

- **What:** expert v3 and the policy each drive the PhysX Americano on the LA model, with traffic.
  - **The row:** development row 11, the southbound Adams focused merge: 1 controlled car among 16
    cars, seed 6952001. Both pass it in SUMO under both gates.
  - **Why this row:** it is the corridor of v14's two physics drives (`physics-01` and
    `physics-02-round20`, round 19's case at seed 6950004), so the results compare.
- **Physics, not replay:** the controlled car is a PhysX rigid body on the model's deck colliders.
  Background cars are SUMO's, drawn at their SUMO poses. Each video and report says so.
- **Not on held-out rows:** those run once, in SUMO, in part 1.

**Built first (uncharged):**
- **A v15 version of v14's physics adapter.**
  - The v15 environment drives its cars with `setAcceleration` and `changeLane`. Those are the two
    commands v14's adapter intercepts (`environments/highway_flow_rl_v14_physical_env.py`).
  - That adapter is built on the v14 environment. Whether it carries over to v15 unchanged isn't
    checked.
- **The physics side as in v14:**
  - `scripts/physics_drive_server.py`, run in Isaac Sim's Python;
  - the low-level controller and the link;
  - the shadow pose sent back into SUMO.

  The LA model is referenced unchanged.
- **An uncharged development drive without rendering,** in `outputs/test-runs/`, as v14's
  `physics-stage2-01`. It also measures how long a drive takes.

**Expected charge:** 2 drives, one episode each.
- **v14's measured drives:** its two recorded drives were charged 108.2 s (226 decisions, rendered
  at 10 Hz) and 54.9 s.
- **Row 11 is longer:** its SUMO episode has 462 decisions (80.1 s simulated), about twice
  `physics-01`'s. So a drive could take about 110–220 s. That isn't measured; the uncharged drive
  measures it.
- **Proposed cap:** at most 500 s for both drives, checked against the uncharged drive's time before
  the first charged one.
- **How to count the episodes (your decision):** 1 of the 174 diagnostic episodes is left, and two
  drives need 2.
  - **Recommended:** count them as Stage 5 evaluation episodes in Stage 5's own allowance, as Phases
    1–4 count theirs.
  - **Or:** keep the plan's table ("Diagnostic episodes"), and raise the diagnostic episodes by 1, to
    175, as v14 did for its recordings.
  - Their clock time counts in the study total either way.

**What it compares:** for each driver, SUMO against PhysX on row 11, as v14's tables did:
- the gates and interventions;
- peak jerk;
- commanded against PhysX acceleration, and decisions at the torque limit.

There's no pass mark; the plan's exit check is "Recorded and labelled".

**Settings and stop rules:**
- **The drives run alone,** after part 1, never alongside a SUMO run. Isaac Sim uses the GPU.
- **Your own Isaac Sim:** if it's open, I'd ask before starting, and never close it.
- **A collision, or a link or physics failure,** ends that drive. It's reported, and the controller
  isn't retuned.

### Totals and the study time

| | Recommended | Minimum |
|---|---|---|
| Part 1 (SUMO, held-out rows) | At most 300,000 decisions and 300 s (expected ~156 s) | At most 110,000 decisions and 150 s (expected ~73 s) |
| Part 2 (physics, 2 drives) | At most 500 s | At most 500 s |
| **Stage 5 at its caps** | **800 s** | **650 s** |
| **Study time left after, at the caps** | **1,364.4 s** | **1,514.4 s** |

- **It fits.** The study has 2,164.4 s left, so neither option needs a raise of the study total.
- **The order:** part 1 first, then part 2's build and drives.
- **Not part of Stage 5:** Krauss runs, Isaac Lab training, and step B.

### My recommendation

1. **Part 1 with all three drivers,** at most 300,000 decisions and 300 s, after its uncharged build.
2. **Part 2:** its uncharged build and development drive, then the two charged drives, at most 500 s,
   counted in Stage 5's own allowance.

Together they fit the study time left with no raise.

## Stage 5 part 1 (November 27): the held-out check

**Approved** on November 27, 2026: "Approve recommended", typed in the Claude side session, in answer
to "Approve Stage 5 at the recommended size?" (decision 10). So:
- **Part 1:** the policy, expert v3 and round 16 on the 42 held-out rows, once, in one batch, at most
  300,000 decisions and 300 s.
- **Part 2:** the uncharged physics build and development drive, then the two charged drives on
  development row 11, at most 500 s, counted as Stage 5 evaluation episodes.
- **No raise** of the study total.

### Built first (uncharged)

- **The runner:** step `held-out` in `experiments/v15_stage5.py`.
  - **Its configuration pins:**
    - the policy's weights (`l2:bc7c050f6d3da3ce`, from the relabel step) and round 16's
      (`policy:f0b11fa577b33763`, from Phase 1), each checked against its hash;
    - expert v3's source;
    - the frozen regrade (sha256 `515c3b7c…`, unchanged);
    - the benchmark manifest.
  - **It runs one batch:** the policy on development row 3 first (the canary re-runs it), the 126
    held-out tasks, then the parity row 52.
  - **It refuses to run again** once its output holds a result or the ledger has a segment for it.
  - **Its report** covers each case and the pool, under the re-baselined and the recorded gates,
    with Wilson intervals, paired differences with exact McNemar tests, events, and the reading set
    before the run.
- **The held-out rows in the pool:** a new row set, `v15-held-out`.
  - Only a pool given the user's authorization (`WorkerPool(held_out=...)`) builds those rows,
    through `held_out_rows(cfg, authorized=True)`.
  - Without one, a task on them is refused, and every other pool refuses them too.
  - The other row sets and every result key are unchanged; the development check and the parity row
    confirm it in the run.
- **The resolve builds no held-out row.** It takes their horizons from the cases and their decisions
  from the development rows, as in the proposal.
- **Tests:** 8 new, none of which builds a held-out row; the v15 suite passes (123 passed).

**Configuration:** `configs/evaluation/v15-stage5-held-out.json` (canonical-JSON sha256 `d48b2abb…`).
Its numbers are the proposal's:
- **Expected:** 249,597 decisions, 129 evaluation episodes, about 156.3 s.
- **Projected:** 262,077 decisions.
- **Range:** 232,604–272,099 decisions.
- **Reservation:** 280 s.
- **Allowance:** 300,000 decisions and 300 s, out of the 2,164.4 s of study time left.

### Result

**Ran** on November 27, under the shared lock (`outputs/highway-merge-v15/stage-5/held-out`).
- **The run:** one batch of 128 tasks. SUMO (libsumo), IDM, 16 workers at below-normal priority.
- **The checks all matched:**
  - the canary re-ran the development check on another worker;
  - the policy reproduced its Phase 3 record on development row 3;
  - the parity row matched Phase 2 run 3.
- **Charged:** 253,863 decisions (3,084 of them the canary), 129 evaluation episodes, and 110.8 s.
  - Expected: 249,597 decisions and 156.3 s; projected 262,077.
  - Allowance: 300,000 decisions and 300 s.
- **Study time:** 62,646.4 of 64,700 s used (2,053.6 s left). The lock was released.
- **Each held-out row ran once.** None is run again.

**Pass rates on the 42 held-out rows** (95% Wilson intervals):

| Driver | Re-baselined gates | Round 24's recorded gates | On the 70 development rows (re-baselined, recorded) |
|---|---|---|---|
| The policy `l2:bc7c050f6d3da3ce` | 34/42 (81%, 67–90%) | 29/42 (69%, 54–81%) | 62, 54 |
| Expert v3 | 34/42 (81%, 67–90%) | 29/42 (69%, 54–81%) | 62, 54 |
| Round 16 (v14's best) | 25/42 (60%, 45–73%) | 24/42 (57%, 42–71%) | 36, 33 |

**Paired differences** on the same rows:

| Pair | Gates | Difference | 95% interval | Only the first passes | Only the second passes | McNemar p |
|---|---|---|---|---|---|---|
| Policy − expert v3 | Re-baselined | 0 | −6.8 to +6.8 points | 0 | 0 | 1 |
| Policy − expert v3 | Recorded | 0 | −5.5 to +5.5 points | 0 | 0 | 1 |
| Policy − round 16 | Re-baselined | +21.4 points | +6.3 to +35.5 | 10 | 1 | 0.012 |
| Policy − round 16 | Recorded | +11.9 points | −0.8 to +24.0 | 6 | 1 | 0.125 |

- **The policy matches its teacher row for row.**
  - It has the same verdict as expert v3 on every held-out row, under both gates, with the same
    failures.
  - Their traces are identical on 36 of the 42 rows. They differ on rows 2, 13, 16, 20, 27 and
    36, with the same verdicts.
- **Where both fail, re-baselined:**
  - **Rolling merge quality:** rows 0, 6, 14, 20, 22, 28, 36 and 37.
  - **The stable merge, too:** rows 14 and 28.
  - **No driver passes case 0** (the v05 I-110 merge; rows 0, 14 and 28) at any held-out seed. On
    the development rows the policy and expert v3 failed it at 2 of its 5 seeds (rows 0 and 14).
- **Events:**
  - **The policy and expert v3:** one cut-in each, on row 36 (the Pasadena 8-of-16 merge), and
    nothing else: no collision, free-road stop or emergency.
  - **Round 16:**
    - a collision and an emergency on row 37;
    - 11 free-road stops on rows 22, 23 and 24;
    - a cut-in each in the two focused 1-of-16 LA cases (rows 24 and 25).

  These are results; nothing was retuned.
- **The intervals are wide.** Each row is about 2.4 points. The three rows of a case share its road
  and demand and differ only by seed, so they aren't independent, and the pooled intervals are
  likely narrower than that structure justifies.

**Reading, set before the run:**
- **Better than v14's best: yes.** Against round 16, re-baselined, the policy is +21.4 points, and
  the 95% interval (+6.3 to +35.5) lies above 0. Under round 24's recorded gates the interval
  (−0.8 to +24.0) includes 0.
- **Holds up as well as its teacher: no, by the rule as written.** Two of its three conditions hold:
  - the interval against expert v3 includes 0 (the difference is 0);
  - there's no collision and no new kind of event.

  The third doesn't: the policy fails the stable merge on rows 14 and 28, a gate failure it never
  had on the development rows. Expert v3 fails the stable merge on the same two rows, and the two
  drivers' verdicts match on every row.
- **These are the study's final numbers for this policy.** Nothing is tuned on them.

## Stage 5 part 2 (November 27): the physics drives

**Physics, not a replay.** The controlled car is an Isaac Sim PhysX rigid-body vehicle (the v14
Americano, provisional physics) on the LA model's deck colliders, driven by the low-level
controller. The other cars are SUMO's (IDM), drawn at their SUMO poses. The LA model is referenced
unchanged, and nothing was installed into Isaac Sim's Python.

### Built first (uncharged)

- **The runner:** `experiments/v15_stage5_physics.py` (commit `c5eda94`, with 2 new tests; the v15
  suite passes, 125).
  - It reuses v14's physics adapter unchanged: `HighwayFlowRLV14PhysicalEnv`, the link, and
    `scripts/physics_drive_server.py` in Isaac Sim's Python.
  - The v15 pool's worker code drives the episode, so the drivers and the result key are the
    ones of Phases 2–4.
- **Ideal-driver check (not physics):** through the physical environment with exact kinematics, both
  drivers reproduce their SUMO records on row 11: 462 decisions, both gates passed, no events. So the
  adapter carries over.
- **Development drive (PhysX, unrendered):** expert v3 on row 11,
  `outputs/test-runs/v15-stage5-physics-dev-expert-01`.
  - **Time:** 70.8 s in all. Isaac Sim's start took 36 s of it and the drive 19 s.
  - **Result:** the same as the charged drives below.
- **Time check against the 500 s:** fits.
  - The runner takes, per drive, the larger of the development drive's time and v14's rendered
    rate (108.2 s for 226 decisions): 221.2 s each, 442.4 s for both.
  - Before the second drive it checks the larger of that and the first drive's measured time.
- **Configuration:** `configs/evaluation/v15-stage5-physics.json` (canonical-JSON sha256
  `41cf3ea9…`, commit `19aef9f`; CI passed before the run).
  - Development row 11, seed 6952001. Expert v3 first, then the policy. Rendered at 10 Hz.
  - Expected: 924 decisions, 2 evaluation episodes, 442.4 s. Reservation: 500 s.

### Result

**Ran** on November 27, under the shared lock (`outputs/highway-merge-v15/stage-5/physics`).
- **Charged (Stage 5 evaluation episodes):** 924 decisions, 2 episodes, 176.2 s (expert v3 88.4 s,
  the policy 86.8 s).
  - Expected 442.4 s; cap 500 s.
  - It was quicker than estimated because the rendered scene loaded in under 1 s; in v14's
    rendered drive it took 58 s.
- **Rendered:** 462 frames per drive at 10 Hz.
- **Isaac Sim:** no other Isaac Sim was open, and each drive's own Isaac Sim closed when the drive
  ended.
- **No stop-rule event:** no collision, link failure or physics failure.

**SUMO against PhysX on row 11.** Expert v3 and the policy have the same numbers on both sides.

| | SUMO | PhysX |
|---|---|---|
| Recorded gates | pass | **fail:** safety events or missing counts, per-car comfort or control, stable merge, merge quality |
| Re-baselined gates | pass | **fail:** the same four |
| Collisions | 0 | 0 |
| Merge entry / lateral merge | 27.7 s / 27.7 s | 27.7 s / 27.7 s |
| Stable merge | passed | **failed:** 12 of 81 window decisions flagged (14.8%; the gate allows 5%) |
| Interventions | 0% | 4.8% (22 of 462 decisions) |
| Override episodes | 0 | 10 |
| Peak jerk | 3.0 m/s³ | 7.9 m/s³ |

**Physics side:**
- acceleration error: mean 0.014 m/s² (0.013 under the torque limit), at most 0.49;
- 6 decisions at the drive-torque limit, at 27.7–28.0 s and 28.7–28.8 s;
- lateral path error at most 0.12 m;
- no wheel left the deck;
- pitch up to 0.087 rad.

**The two drivers drove identically.**
- The two drives' action logs are byte-identical.
- The policy's Level 2 output (source, target, plan, lane and maneuver) matches expert v3's on all
  462 decisions, as in SUMO, where the two share a trace on this row.
- The unrendered development drive gave the same actions, so rendering doesn't change the result.

**Why the gates fail (read from the physics log and the episode):**
- **The flags:** all 22 are v14's `native_realization_unattributed`: the acceleration PhysX
  realized differs from the command. One also has a comfort-rate clip.
- **Around the merge, 13 of them, at 27.5–29.6 s:**
  - over these decisions the pitch rises to 0.081 rad and falls to −0.057 rad: the climb onto the
    mainline and over its crest;
  - the torque limit is reached on 6 decisions;
  - realized acceleration is off the command by up to 0.49 m/s² (commanded 1.24, realized 1.73 at
    28.7 s).
- **Inside the stable-merge window (27.7–35.7 s):** 12 of those 13. That's 14.8% of the window's
  81 decisions, over the gate's 5%.
- **The other 9:** 2 on the ramp (5.1 and 6.4 s) and 7 at 42.8–43.5 s.
- **The other three gates:** the recorder doesn't save why they fail. They likely count the same
  overrides and jerk (inferred, not read).

**Compared with v14's drives on this corridor:** those were a different seed (6950004) and policy,
so this isn't a matched comparison.
- Round 16 had 33.6% interventions and peak jerk 14.1; round 20 had 25.5% and 5.5.
- Both failed the stable merge too, with flagged decisions on the climb and crest. For round 20,
  v14 inferred this rather than read it.

**What it means:**
- On this row, the SUMO pass doesn't carry over to physics, for either driver. The reason is the
  car's realized acceleration on the grade and crest, which SUMO's car model doesn't have.
- The mismatches are small on average (0.014 m/s²). But the stable-merge gate counts each flagged
  decision as an intervention, and 12 of them fall inside its 8 s window.
- The policy behaves like its teacher here as well: physics changes neither driver's choices, only
  how the car realizes them.

**Video** (outside Git):
`outputs/highway-merge-v15/stage-5/physics/policy/physics-with-graph/physics-with-graph.mp4`.
- The title says PHYSICS, and that the other cars are at SUMO poses.
- Below the 3D view: the speed against the merge target, and the commanded acceleration against
  PhysX's, with ticks at the torque limit.
- The expert's drive is identical, so it wasn't composed separately.

**Ledger:**
- **Phase 5:** 254,787 decisions, 286.9 s and 131 evaluation episodes.
- **Study time:** 62,822.5 of 64,700 s used, 1,877.5 s left.
- The lock was released.

**Exit check ("Recorded and labelled"): met.** Stage 5 is complete.

## Physics gap proposal (November 27): why row 11 fails in physics, for your approval

**Your decision (relayed by the main Claude session):** "Physics gap first", a written proposal only.
**Nothing ran for it.** No simulator was launched.
- **What was read:** Stage 5's saved PhysX drives on row 11 and the LA lane graph.
  - The three drives (the uncharged development drive and the two charged ones) are byte-identical,
    so one drive's log stands for all three.
  - The road slope under the car is computed from the lane graph at the logged chassis positions.
- **Where the analysis is:** scripts and outputs in
  `outputs/highway-merge-v15/stage-5/physics/gap-analysis/` (outside Git).
- **Labels:** "read" means taken from the logs, the report or the code. Everything else is inferred.

### What fails on row 11 (PhysX)

**The 22 flags.** Each one is v14's `native_realization_unattributed`: the realized acceleration
differs from the command by more than 0.05 m/s² over a 0.1 s decision.
- **In speed terms:** 0.05 m/s² is 5 mm/s at the decision's end. The 22 flagged misses are 5–49 mm/s
  (read). The mean miss over all 462 decisions is 0.014 m/s².
- **Times** are decision end times, as in the Stage 5 records.

| Where | Decisions (s) | Flags | Miss (m/s²) | Read from the logs | Cause (inferred) |
|---|---|---|---|---|---|
| Ramp: command ramps | 5.1, 6.4 | 2 | −0.055, +0.056 | Road slope steady (0.030–0.039). The command steps by about 0.25 per decision, and the realized acceleration trails it by 0.03–0.06 through both ramps | Response lag. The feedforward also leaves out the wheels' rotational inertia (1.7% of the mass, read from the asset) |
| Grade break at the junction onto the mainline (a sag) | 27.5 | 1 | −0.325 | Road slope 0.031 → 0.049 in one decision (then 0.066 and 0.076). The chassis drops 1.3 cm. Its pitch follows the road slope to within 0.006 | The body's response to the break. The grade feedforward is not the cause here |
| Just after the sag | 27.9–28.2 | 4 | +0.07 to +0.16 | The force bias rises from 72 to 274 N, learned from the 27.5 miss. Pitch exceeds the road slope by up to 0.012 | The bias, learned from a transient, and the pitch overshoot both add force. The car goes faster than commanded, even at the torque limit |
| The crest | 28.6–29.3 | 6 | −0.14 to +0.49 | Road slope 0.070 → −0.050 in 0.6 s. The chassis rides up to 3.2 cm high. The misses alternate in sign; over 28.6–28.9 they net 2.6 cm/s. 28.7 is the 7.9 m/s³ peak jerk | The body's pitch and heave over the crest |
| Settling after the crest | 29.5, 29.6 | 2 | +0.08, +0.07 | Road slope steady (−0.050). The bias moves from 285 to 225 N | The bias relearning |
| A second crest, at 26.8 m/s | 42.8–43.3, 43.5 | 7 | −0.33 to +0.29 | Road slope 0.049 → 0 in 0.2 s, an implied vertical acceleration of up to 8.9 m/s². The chassis rides 6.4 cm high and its pitch differs from the road slope by 0.021, both the most on the drive. The bias swings from 169 to −64 to 255 N | Heave, the pitch-based feedforward (about 30–70% of each miss, estimated) and the bias |

**Inside the stable-merge window (27.7–35.7 s):** 12 of its 81 decisions are flagged, in the third
to fifth rows of the table.

**What the logs rule out:**
- **No miss was the car running out of drive.** On all 6 torque-limit decisions it went *faster*
  than commanded, by +0.009 to +0.49 m/s² (read).
- **Level 1's drive limit is already in place.** v14's grade-aware ceiling
  (`traffic/road_grade.py`: 0.1 m/s² headroom, a 0.2 s look-ahead) caps every command. It never
  bound on row 11, in SUMO or in PhysX (read).
  - On the climb and the crest, expert v3's requests sat 0.05–0.08 m/s² under that ceiling.
  - Over the crest, the request follows the ceiling as it rises: from 1.04 to 1.71 m/s² in 0.3 s.
- **Measuring speed along the road instead of horizontally** gives 23 flags instead of 22 (read).
- **A rough offline estimate** removes the pitch-versus-road feedforward error from each miss. It
  leaves 25 flags rather than fewer. So feeding forward the road's slope alone probably isn't
  enough. This is a linear estimate, not a physics prediction.

**The other three gates, from the saved report** (read, with the pool's own gate function):
- **Per-car comfort or control:** only the peak jerk fails. It is 7.9 m/s³ at 28.7 s and −5.1 at
  27.5 s, against 5. The interventions, 4.8%, pass the per-car 5%.
- **Safety events:** 2 overspeed decisions, at 42.8 and 42.9 s. The second crest pushes the speed to
  26.83 and 26.85 m/s, against the 26.82 m/s limit. There is no collision, teleport or emergency
  braking.
- **Rolling merge quality:** the environment records a car's merge quality only after a stable merge
  (v04's code). With the stable merge failed, there's none, so the gate fails.
- **The comfort-rate clip at 43.0 s** is a knock-on as well. The clip allows ±0.4 m/s² around the
  measured acceleration, and the crest had pushed that to +0.14.

**In short:** all four gate failures on row 11 come from the two command ramps, three sharp grade
breaks in the road model, and the force bias the controller learns from them.

### The fixes, ranked

**1. Recommended: the low-level controller only (physics side).** Two versions, both fixed in code
before any drive:
- **C1:**
  - grade feedforward from the road model's slope under the axles (the server already loads that
    surface), taken 0.05 s ahead, instead of the chassis pitch;
  - force-bias learning paused while the road slope changes by more than 0.005 in a decision, and
    for 0.5 s after;
  - the wheels' rotational inertia in the feedforward mass (1,830 kg instead of 1,800).
- **C2:** C1 plus a stronger speed correction within each decision.
  - The gain goes from 2 to 6 per second.
  - At 2, a decision removes about 18% of a speed error that arises within it. At 6 it removes
    about 45%.
  - This is the only part that acts on the body's own pitch and heave.
- **What changes:** `traffic/physical_controller.py` and `scripts/physics_drive_server.py`, with tests
  on the existing simple car model. That's about 100–150 lines, built and tested without a
  simulator.
- **No SUMO result can change:** only the physics server uses this controller (read). So no SUMO
  re-check is needed.
- **Expected effect: unknown until driven.** C1 targets the bias, the pitch overshoot and the ramp
  lag. C2 also targets the heave at the breaks. Neither may be enough.

**2. Not recommended now: a Level 1 change.** For example, a limit on how fast the command may follow
a rising drive ceiling over a crest.
- **Why not now:** Level 1 isn't what failed on row 11.
- **The cost:**
  - **The re-check:** it changes the executor of both expert v3 and the policy. So it needs a 70-row
    SUMO re-check of both: about 271,000 decisions, about 120–150 s at 16 workers.
  - **The no-regression rule, set before the re-check runs:**
    - neither driver loses a development row under either gate set (now 62 re-baselined, 54
      recorded);
    - no new event kind;
    - rows where the new limit never acts reproduce their recorded traces exactly.
  - **The physics drives** come on top of that.

**3. Outside this plan: smoothing the road model's vertical profile.** The three breaks are in the LA
lane graph (read). Changing it changes the 3D model, and Level 1's drive limit reads the same lane
graph. That isn't authorized, and it isn't sized here.

### How it would be checked in physics

**Which rows physics can drive:** 10 of the 70 development rows.
- **The constraints (read from the code):** the physics server drives one car per run, and needs an
  LA corridor with a corridor manifest.
- **The 10 rows:** the two focused cases, with 1 controlled car among 16.
  - **Southbound Adams** (case 11): rows 11, 25, 39, 53 and 67. The policy, like expert v3, passes
    all 5 in SUMO re-baselined, and rows 11 and 25 recorded.
  - **Northbound Pasadena** (case 10): rows 10, 24, 38, 52 and 66. The policy passes 10, 52 and 66 in
    SUMO under both gates, and fails 24 and 38.
    - This corridor has never been driven in physics.
- **The other cases can't be driven:**
  - the multi-car LA cases (4 or 8 controlled cars) would need more than one physics car;
  - the other 8 cases' networks have no corridor manifest, so there's no road model to drive on.

**The setup:**
- **The driver:** the policy only. It drove identically to expert v3 on row 11, in SUMO and in
  PhysX.
- **Matched before and after:**
  - Row 11's "before" is the Stage 5 drive, which PhysX reproduced byte-identically 3 times, so it
    isn't re-run.
  - Every other row gets a drive with the current controller as well as one with the new controller.
- **Rendering:** row 11's drives render at 10 Hz, for the video. The others aren't rendered. In
  Stage 5, rendering changed no action or physics value (read).
- **Uncharged before any drive:**
  - the controller code and its tests, with no simulator;
  - the ideal-driver check (the physical environment without physics, a SUMO run) on each new row,
    as in Stage 5.
- **All physics drives are charged.** PhysX repeated row 11 exactly in 3 drives (read), so a
  practice drive on a row would likely be that row's result.

**The options.**
- **Per-drive estimates:** the measured maxima, 88.4 s rendered and 70.8 s unrendered.
  - These are taken whatever the row's length, since Isaac Sim's start (36 s) is about half of a
    drive.
  - Before each drive, the runner checks the larger of its estimate and the longest drive so far
    against what's left of the cap.

| | Minimum | Recommended | Larger |
|---|---|---|---|
| Drives | C1 on row 11 (rendered) | C1 and C2 on row 11 (rendered); then the pick and the current controller on rows 25, 39, 53 and 67 | The recommended drives; then the pick and the current controller on the 5 northbound rows |
| Number of drives | 1 | 10 | 20 |
| Expected time | 88.4 s | 743.2 s | 1,451.2 s |
| Cap | 150 s, 1,000 decisions | 850 s, 6,000 decisions | 1,650 s, 10,000 decisions |
| Study time left after, at the cap | 1,727.5 s | 1,027.5 s | 227.5 s |

**The pick between C1 and C2,** made on row 11 by a rule set now:
1. fewer failed gates, counted over both gate sets;
2. then fewer realization flags;
3. then C1, the smaller change.

**The episodes:**
- **Recommended:** count them as evaluation episodes in their own allowance, as Stage 5's were.
- **Or:** count them as diagnostic episodes. Only 1 of the 174 is left, so the recommended option
  would need a raise of 9 diagnostic episodes, and the larger one 19.
- **No raise of the study total** is needed for any option. Their clock time counts in the study
  total either way.

**Settings and stop rules, set before any drive:**
- **C1 and C2 are frozen** (committed, with hashes in the configuration) before the first drive.
  Nothing is changed after a drive, and the pick isn't revisited after the other rows.
- **A collision, or a link or physics failure,** ends that drive. A collision with C1 or C2 also
  stops the rest of the run. It's reported, and nothing is retuned.
- **Before each drive:** if it doesn't fit what's left of the cap, the run stops and asks. Drives
  are never trimmed.
- **Your own Isaac Sim:** if one the run didn't start is open, the run stops and asks, and never
  closes it.
- **As in Stage 5:**
  - one process at below-normal priority, under the shared lock, with no SUMO run alongside;
  - never on held-out rows;
  - nothing installed into Isaac Sim's Python;
  - the LA model and the car's physics parameters unchanged.

**The reading, set before any drive:**
- **Row 11:** the gap is closed there if the pick passes both gate sets in PhysX. In Stage 5 it
  failed 4 gates under each set.
- **The southbound corridor** (recommended option; 5 rows, so counts and no intervals):
  - **Closed:** the pick passes in PhysX every row the policy passes in SUMO, under each gate set (5
    re-baselined; 2 recorded), with no collision.
  - **Improved:** the pick passes more rows than the current controller under the re-baselined
    gates, loses none under either set, and has no new event kind.
  - **Otherwise: not improved.** The next step would then be your decision: the gate question below,
    or the road model.
- **The northbound rows** (larger option) are read the same way. The policy passes 3 of them in SUMO.
- **Reported per row, not gated:**
  - the flags and the stable-merge window fraction;
  - the peak jerk and overspeed decisions;
  - the acceleration error and torque-limit decisions.

### A separate decision: do realization mismatches count as interventions in physics?

- **The background:** the 0.05 m/s² tolerance comes from SUMO, where commands are carried out
  exactly. In PhysX it is 5 mm/s of speed per decision.
- **Changing it alone doesn't fix row 11:**
  - Without these flags, the stable-merge window would have no interventions.
  - But the same drive would still fail two gates: per-car (peak jerk 7.9) and safety (2 overspeed
    decisions).
- **The choices:**
  - **Keep it** (recommended until C1's result is in);
  - **count only mismatches above a physics tolerance;**
  - **count a cumulative speed error instead.**
- It isn't part of the options above, and it changes no SUMO result.

### My recommendation

1. **The recommended option:** C1 and C2 on row 11, then the pick and the current controller on the
   other 4 southbound rows.
   - At most 850 s and 6,000 decisions, counted as evaluation episodes in its own allowance.
   - No raise.
   - The northbound rows can follow as a separate approval, from the time left.
2. **Keep the gates as they are for now** (decision 12), and decide after C1's result.

## Physics gap result (November 27): C1 and C2 in PhysX

**Your decision (relayed by the main session):** "Yes, as recommended" (decision 11, the recommended
option; decision 12, keep the gates).

**What ran:** the policy `l2:bc7c050f6d3da3ce` drove the PhysX car (physics, not a replay; the other
cars are SUMO's, drawn at their SUMO positions). The 10 drives, in order:
1. C1, then C2, on development row 11, rendered at 10 Hz.
2. The pick between them, by the rule set before the run.
3. On each of southbound rows 25, 39, 53 and 67, unrendered: the current controller, then the pick.

Row 11's "before" is the Stage 5 drive.

**The setup:**
- **The controllers:** C1 and C2 were frozen and committed before the first drive (`219b5b8`). The
  configuration (`d21915c`, `configs/evaluation/v15-physics-gap.json`) was pinned and CI passed
  (run 46069377364) before any drive.
- **The current controller** reproduces its earlier commands byte for byte: a fingerprint test over
  960 ticks matches the hash taken before the change.
- **The ideal-driver checks** (uncharged; SUMO with the ideal driver, not physics) on rows 25, 39,
  53 and 67:
  - They matched SUMO's verdict under both gate sets, and its events.
  - The ideal driver's car ended 1 to 3 decisions later. I haven't investigated why.
- **The runner:** `experiments/v15_physics_gap.py`, ledger phase 6.

### The pick: C2

- **Gates:** C1 and C2 each failed 2 gates per set on row 11 (4 in all), a tie.
- **Flags:** C2 had fewer, 9 to C1's 11, so the rule picked C2.

### Results (PhysX; recorded and re-baselined gates gave the same verdicts on every drive)

| Row | Controller | Failed gates (per set) | Flags | Window interventions (of 81) | Per-car interventions | Peak jerk (m/s³) | Overspeed decisions | Torque-limit decisions | Mean acceleration error (m/s²) |
|---|---|---|---|---|---|---|---|---|---|
| 11 | current (Stage 5 drive) | 4: safety, per-car, stable merge, merge quality | 22 | 12 | 4.8% | 7.90 | 2 | 6 | 0.014 |
| 11 | C1 | 2: stable merge, merge quality | 11 | 9 | 2.4% | 4.47 | 0 | 0 | 0.009 |
| 11 | **C2 (pick)** | 2: stable merge, merge quality | 9 | 7 | 2.0% | 4.75 | 0 | 0 | 0.008 |
| 25 | current | 4 | 38 | 11 | 8.4% | 3.91 | 2 | 4 | 0.015 |
| 25 | C2 | 2: stable merge, merge quality | 16 | 7 | 3.5% | 3.31 | 0 | 0 | 0.009 |
| 39 | current | 4 | 30 | 14 | 6.9% | 5.05 | 2 | 3 | 0.013 |
| 39 | C2 | 2: stable merge, merge quality | 18 | 8 | 4.1% | 4.96 | 0 | 0 | 0.007 |
| 53 | current | 4 | 31 | 13 | 7.2% | 5.38 | 2 | 2 | 0.013 |
| 53 | C2 | 3: per-car (jerk), stable merge, merge quality | 15 | 8 | 3.5% | 5.28 | 0 | 0 | 0.007 |
| 67 | current | 4 | 29 | 12 | 6.5% | 5.80 | 2 | 2 | 0.013 |
| 67 | C2 | 3: per-car (jerk), stable merge, merge quality | 9 | 5 | 2.0% | 5.13 | 0 | 0 | 0.007 |

**The limits:**
- **The stable merge:** at most 5% of the 81 decisions in the 8 s window after the merge may be
  interventions, which is 4.
- **Per-car:** at most 5% interventions over the route, and a peak jerk of at most 5 m/s³.
- **What "current" fails:** the same 4 gates on every row. Safety fails on its 2 overspeed decisions.
  Per-car fails on interventions over 5% on rows 25 to 67, and also on jerk on rows 39 to 67.
- **Not shown in the table:** no drive had a collision, and every drive's server reported the
  planned controller.

### The reading (set before the run)

- **Row 11:** not closed. C2 still fails the stable merge and merge quality.
- **The southbound corridor:** **not improved.**
  - Neither controller passes any of the 5 rows under the re-baselined gates (0 and 0).
  - C2 loses no row and adds no event kind.
  - In SUMO the policy passes all 5 rows re-baselined, and rows 11 and 25 recorded.

### What the drives show (read from the reports and logs; causes are inferred)

- **C2 fixes what the proposal traced to the controller:**
  - **Overspeed:** 10 decisions over the 5 rows become 0.
  - **Torque limit:** 17 decisions become 0.
  - **Flags:** 150 become 67.
  - **Peak jerk:** 7.9 on row 11 becomes 4.75.
  - **Per-car interventions:** under 5% on every row, where the current controller is at 4.8% to
    8.4%.
  - **Second crest:** on row 11, its flags (7 in Stage 5) drop to 1.
- **What still blocks the stable merge:** 5 to 9 interventions in the 81-decision window on every
  row, all realization flags.
  - **Where they fall on row 11 (C2), 8 of the 9 flags:** from the junction sag (27.5 s) to the end
    of the crest (29.2 s); the ninth is at 43.1 s. 7 of the 8 are inside the window.
  - **What happens there:** the car is commanded +1.0 to +1.8 m/s². It mostly gets 0.08 to 0.19 m/s²
    less, once 0.18 m/s² more, with no torque limit reached.
  - **What changes there:** the road model's slope changes by up to 0.027 within a single decision.
  - **Inferred:** the body's pitch and heave over those grade breaks, which no controller-only change
    here removes.
- **Merge quality:** no qualified merge is recorded in any C2 drive (`quality_merges` is empty), as
  in Stage 5, so it fails with the stable merge.
- **Rows 53 and 67:** C2 also fails per-car there, on peak jerk (5.28 and 5.13) alone.

### Charge

- **Phase 6:** 4,458 decisions, 10 evaluation episodes and 462.7 s, of the 850 s and 6,000
  decisions; the configuration expected 4,444 and 743.2 s.
- **Drive times:** 121.4 s for C1 and 89.9 s for C2 (rendered). The unrendered drives took 28.5 to
  33.8 s.
  - C1's drive ran over its 88.4 s estimate; I haven't investigated why.
  - The 70.8 s unrendered estimate came from Stage 5's development drive.
  - The check before each drive held throughout.
- **No raise:** of the study total or of the diagnostic episodes.
- **Study time:** 63,285.2 of 64,700 s used; 1,414.8 s left.
- **Video** (PhysX physics, labelled so in the video, the other cars at their SUMO positions):
  `outputs/highway-merge-v15/physics-gap/row11-c2/physics-with-graph/physics-with-graph.mp4`, outside
  Git.

### Not done

None of these was authorized:
- the northbound rows;
- any Level 1 change;
- the road model, the LA model, or the car's physics parameters;
- gate changes;
- held-out rows;
- installs into Isaac Sim's Python;
- training, any raise, or a merge to main.

The gates are unchanged (decision 12, "keep"). Revisiting them is your decision, now that the
results are in.

## Ramp driving (November 27): the slow merges, for your decision

**Your decision (relayed by the main session):** "Yes, driving first": write up the ramp-driving
problem first, then the physics options. Your words included:
- "looks significantly worse compared to the old versions";
- "are you measuring success by just a few numerical values? or are you also looking at the actual
  movement of the vehicle".

**What this is:** a written proposal only.
- **Nothing ran:** no SUMO, no Isaac Sim, no code or gate change, no charge.
- **Where the numbers come from:**
  - the saved development-row records (IDM background, first attempt) of expert v3 (Phase 2 run 3),
    the policy (Phase 3 relabel) and round 16 (Phase 1 run 1);
  - the row 11 physics drive with C2;
  - the code;
  - offline Python over those logs.
- **No held-out record was used** for any ramp measure. The physics options below read only the
  held-out records' total realization-flag and drive-limit counts.
- **The scripts and their outputs** are in `outputs/highway-merge-v15/ramp-driving/`, outside Git.

### The short answer to your question

- **The pass/fail verdicts are numbers from the gates, and none of them looks at the movement you
  saw.** (Read in the code.)
  - **No gate checks the merge speed against the traffic the car joins.** Merge quality's speed
    match compares the car with the gate's own target speed, within 2 m/s. After a yield entry, that
    target starts at the car's own speed and rises at a set rate.
  - **No gate flags slowing or crawling on an open ramp.** A stop counts only below 0.1 m/s.
    Expert v3's crawls at 0.1–0.7 m/s never counted as stops.
  - **Rule R2 of the re-baselined gates** restarts that target at the car's own speed when it
    crosses the stop line. That turned 3 of the slow merges' recorded failures (rows 39, 53 and 67)
    into passes.
- **I looked at movement for row 11's physics drive, but not across rows until now.**
  - Across rows, the slow merges are common and real.
  - They come from expert v3's merge logic, which the policy copies.
  - How SUMO models the ramp junction makes them worse.
- **The comparison with the old versions wasn't matched.** v14's video was on another seed, with a
  clear ramp (the main session's reading; I didn't find that record to re-check it). On the same
  rows:
  - **With a clear ramp,** the policy merges like round 16, in 4–6 s.
  - **When mainline cars are coming,** round 16 merges about 18 s sooner and about 8 m/s faster.
    But 4 of those 9 rows recorded a SUMO emergency with round 16.
  - **On row 11 itself,** round 16 merged later (31.4 s, against the policy's 27.7 s) and stood still
    for 5.2 s.

### Row 11, checked against the saved logs

Below is the main session's reading of the row 11 drive (PhysX, controller C2). I checked it
against the drive's Level 1 log and trace, and against the policy's SUMO record of the same row.

| The main session's reading | What the logs show |
|---|---|
| From 0 to 3.5 s it brakes from 27 to 10.5 mph on an open ramp | **Confirmed.** 12.11 m/s (27.1 mph) at 0.2 s, 4.68 m/s (10.5 mph) at 3.5 s. The request is at its −3.0 m/s² floor from 1.1 to 2.4 s, with the free term at −3.0. The car decelerates at 2 m/s² or more for 22 decisions from 0.8 s. The leader, ramp_0, is 54–57 m ahead and pulling away (12.1 m/s at the start) |
| Junction hold 5.1 m/s, waiting for main_10, main_7, main_4 and main_1 | **Close.** The hold is 5.22 m/s at 0.2 s, easing to 4.68 m/s by 3.5 s. It waits for those 4 cars from 0.2 s |
| Holds about 4.7 m/s from 3.5 to 13 s | **Confirmed** (4.68–4.75 m/s), apart from the commit below |
| Commits at about 5 s (target 14.35 m/s), up to 13.4 mph | **Commits at 4.8 s** (target 14.57 m/s; 14.35 at 5.0 s). Peak 5.99 m/s (13.4 mph) at 6.5 s |
| Un-commits at about 7 s and brakes | **Un-commits at 6.2 s.** The request turns negative at 6.5 s and reaches −1.68 m/s² at 7.1 s |
| From 13 to 18 s the bound term goes negative behind ramp_0, which stands (≤ 0.2 m/s) at the ramp end from about 10 to 18 s; minimum 1.32 m/s at 18 s | **Confirmed.** ramp_0 is at ≤ 0.2 m/s from 10 to 18 s, at the stop line. The bound term is negative from 12.3 to 17.9 s. Minimum 1.32 m/s (3.0 mph) at 18 s |
| Merges at 27.7 s at 11.6 m/s, while mainline cars do 17–23 m/s | **Merge completion at 27.7 s at 11.8 m/s** (26.4 mph). The 4 mainline cars it waited for passed at 20.8–24.4 m/s (median 21.5). At the merge itself the logs show only ramp cars nearby (ramp_0, 45 m ahead at 18.3 m/s), so 17–23 m/s is neither confirmed nor ruled out |
| 60 mph at about 40 s | **Confirmed.** 26.78 m/s at 40 s; 26.82 m/s (60 mph) at 41 s |
| Not physics: PhysX matched the commands within about 0.01 m/s² | **Within 0.04 m/s² on the ramp** (the largest difference in 226 decisions to 22.7 s; none over 0.05). The policy's SUMO record of row 11 drives the same way: merge at 27.7 s at 11.83 m/s |
| Matched: round 16 in SUMO merged at 31.4 s with 52 stopped decisions, finishing at 49.4 s | **Merge at 31.4 s,** with 52 decisions below 0.1 m/s (5.2 s). Its last record is at 49.7 s (route completed) |
| Unmatched: v14's videos, seed 6950004, clear ramp, merge at about 6 s | **Not re-checked.** No saved v14 episode with that seed turned up in a quick search |

In SUMO only, the policy also committed briefly from 1.9 to 2.2 s.

### 1. How common it is (saved SUMO records, development rows)

- **The cars:** every car that approaches a ramp's yield entry. That's 31 cars on 25 of the 70
  development rows, the same cars for each driver.
- **The policy drives exactly like expert v3** on all 31: the same merge time, merge speed, lowest
  speed and crawl time. So below, "expert v3" also stands for the policy.
- **Four situations,** assigned from expert v3's record (the same cars for each driver):

| Situation | Cars | Rows | What happens |
|---|---|---|---|
| **Clear** | 7 | 36, 50, 51, 52, 64, 65, 66 | Expert v3 merges within 7 s |
| **Stream** | 12 | 8, 9, 22, 24, 37, 38, 39, 53, 67 | Our car waits for mainline cars, with no stopped car ahead |
| **Blocked** | 5 | 9, 10, 11, 23, 25 | A SUMO ramp car ahead stands still for 2 s or more |
| **I-110 merge case** | 7 | 0, 14, 28, 42, 56 | A case with its own network (see section 3) |

| Situation (cars) | Driver | Merged | Time to merge: median (range) | Merge speed (median) | Lowest ramp speed (median) | Time below 5 m/s (median) | Cars that stopped | Rows passed, recorded |
|---|---|---|---|---|---|---|---|---|
| Clear (7) | expert v3 | 7 | 4.5 s (4.4–5.9) | 17.7 m/s | 12.2 m/s | 0 s | 0 | 7 of 7 |
| | round 16 | 7 | 4.3 s (4.2–5.9) | 18.6 m/s | 12.3 m/s | 0 s | 0 | 3 of 7 |
| Stream (12) | expert v3 | 12 | 24.4 s (12.9–35.2) | 10.9 m/s | 0.7 m/s | 16.1 s | 0 | 0 of 9 |
| | round 16 | 10 | 6.2 s (4.4–21.3) | 19.4 m/s | 12.2 m/s | 0 s | 1 | 2 of 9 |
| Blocked (5) | expert v3 | 5 | 26.6 s (24.4–29.8) | 11.8 m/s | 1.2 m/s | 15.6 s | 0 | 4 of 5 |
| | round 16 | 5 | 28.7 s (10.9–31.2) | 14.2 m/s | 0.0 m/s | 17.2 s | 4 | 0 of 5 |
| I-110 (7) | expert v3 | 7 | 35.4 s (26.4–43.1) | 4.2 m/s | 3.5 m/s | 3.9 s | 0 | 3 of 5 |
| | round 16 | 7 | 39.3 s (20.0–49.8) | 2.4 m/s | 0.0 m/s | 5.6 s | 5 | 0 of 5 |

- **How to read the table:**
  - Times start at the car's first record.
  - "Stopped" means below 0.1 m/s, the gates' definition.
  - "Rows passed" counts each row once, under round 24's gates.
- **On the stream rows:** expert v3 crawls (a median of 16.1 s below 5 m/s, or 11 mph). It merges at
  a median of 10.9 m/s (24 mph). Round 16 merges at 19.4 m/s (43 mph).
- **Round 16's quick merges came with problems:**
  - It never merged 2 cars, on rows 8 and 24.
  - Rows 8, 9, 24 and 39 recorded a SUMO native emergency. The failures are read from the records;
    that round 16's merge caused them is inferred.
  - It passed rows 38 and 53, which expert v3 failed.
- **Merge speed against the mainline:** I took the mainline cars that passed the entry in the 10 s
  before our car left the ramp. That covers 15 of expert v3's cars and 16 of round 16's.
  - Expert v3 merged a median of 10.4 m/s (23 mph) below them; round 16, 3.7 m/s (8 mph) below.
  - Which cars count as mainline is inferred from their ids and positions.

### 2. Why expert v3's Level 1 does it

This is read in `traffic/v15_rule_expert.py` (`select_gap`, `entry_options`, `_junction` and
`decide`), unless it's marked inferred.
- **How it judges a gap.** At a yield entry it takes the mainline cars approaching the shared lane,
  in order. It accepts a gap when both of these hold:
  - the car behind the gap would brake at most 2.0 m/s² (SUMO's IDM, with that car's own
    parameters) when our car crosses the stop line, at the speed our car would have there;
  - every car ahead of the gap passed at least 1 s before.
- **The speed it would have there** is a run-up from its current speed at 1.5 m/s².
  - A slow car enters slowly.
  - A mainline car at 21–24 m/s closing on a car entering at 5–14 m/s needs a long gap.
  - (Inferred: this is why most gaps fail.)
- **The early slowing.** When no visible gap is acceptable, it plans to arrive 1 s after the last
  visible car passes. It holds the steady speed that gets it there on time: the distance to the line
  divided by the time.
  - **On row 11 at 0.2 s:** 102 m in 19.6 s, so 5.2 m/s. The free term then asks for its −3.0 m/s²
    floor, wherever the leader is.
  - **It slows at once,** not when it needs to.
  - **Slowing lowers its entry speed,** so the next gaps fail too. (Inferred.)
  - **Its only other option** is a 0.5 m/s crawl, then the run-up. There's nothing in between, such
    as holding a moderate speed and then running up to cross the line at speed.
- **The commit and un-commit flip:**
  - **Commit:** it commits as soon as a gap is acceptable now. Committed, its speed target is the
    larger of the gate's target and its current speed. So it no longer slows, but nothing makes it
    run up at the 1.5 m/s² its plan assumed.
  - **On row 11:** the request may move only 0.3 m/s² from the measured acceleration per decision
    (jerk ≤ 3 m/s³). It rose in those steps to 1.41 m/s² at 5.3 s. Then
    the gate's target restarted at the car's own speed at 5.4 s (the trace marks it "restarted").
    The request fell to 0.35–1.0 m/s².
  - **Then each re-plan slips.** The planned crossing moved from 11.8 s to 12.1 s. The gap in front
    of main_4 shrank from 54.5 m to 45.7 m.
  - **At 6.2 s,** main_4's predicted braking passed the 3.0 m/s² abort limit, so the car
    un-committed and braked.
  - **What's read and what's inferred:** each step is in the logs; the chain from the slow run-up
    to the abort is inferred.
- **It isn't a physics effect:** the same crawl and flip are in the SUMO record.

### 3. How SUMO models the ramp junction

Read in each case's `network.net.xml` and `routes.rou.xml`:
- **Every ramp case** joins a one-lane ramp to mainline lane 0 at a junction of type "priority".
  - **The ramp's connection is minor** ("m": it must yield); the mainline's are major ("M").
  - **The ramp's visibility** is 100 m.
- **There's no added lane.** The road after the junction has as many lanes as the road before it:
  3 in the LA cases and 5 in the I-110 merge case. So there's no acceleration lane in which to match
  speed.
- **The junction lane** is 43.3 m long at Adams, 20.9 m at Pasadena and 2.0 m in the I-110 merge
  case.
- **The car types set no junction-model or impatience parameters,** so SUMO's defaults apply. I
  didn't look the default values up.
- **SUMO's mainline cars ignore a ramp car until it crosses the stop line,** then follow it. This
  comes from a check recorded in expert v3's code: a mainline car braked at 9 m/s² as a ramp car
  entered the junction lane at 4.4 m/s. I didn't re-check it.
- **So the model is a yield junction,** like a side street with a yield sign: a ramp car must find
  its gap at the line. SUMO's own ramp cars do just that. Row 11's ramp_0 stood still at the line
  for 8 s.

**Is it realistic?**
- **Freeway entrances usually have an acceleration lane,** where drivers match speed before they
  merge. This model has none.
- **Not checked:** whether the real Adams, Pasadena and I-110 ramps have one. I read no design
  source either, so "unrealistic" here is an inference, not a checked fact.

**Which rows can be trusted for judging the merge logic:**
- **Clear (7 cars):** yes; the junction barely matters. But they don't test waiting.
- **Stream (12 cars):** yes, for comparing drivers under the same model. They're where expert v3
  differs from round 16. The yield junction limits their realism.
- **Blocked (5 cars, rows 9, 10, 11, 23 and 25):** not on their own. The stopped SUMO ramp car ahead
  decides most of the outcome, and that's the yield model, not our driver. Row 11 is one of these,
  so it's a poor showcase.
- **The I-110 merge case (7 cars):** every driver is slow there, and round 16 stops 5 of the cars. I
  didn't examine it further.

### 4. Movement checks, set now, before any run

**Where the thresholds come from.** Each one reuses a value the project already has, or it's marked
as my choice.

**The close leader,** for M1 and M2. A leader is close if any of these holds:
- it's within the stop audit's 20 m free gap;
- the headway is under 3 s;
- the closing time is under 6 s.

The 3 s and 6 s are twice the per-car gate's minima of 1.5 s headway and 3 s TTC; the factor of 2 is
my choice.

**The four checks:**
- **M1, hard braking on an open road:** 2.0 m/s² or more of deceleration for 3 decisions in a row
  before the merge, with no close leader.
  - 2.0 m/s² is expert v3's own comfort braking (`idm_comfort_m_s2`).
  - The 3 decisions are the gate's avoidable-braking count.
- **M2, crawling on an open ramp:** more than 1 s below 5 m/s before the merge, with no close leader.
  - 5 m/s is the free-road stop rule's minimum target.
  - 1 s is the merge-quality gate's stop limit.
- **M3, merge speed against traffic (report only):** at merge completion, the car's speed is at
  least the nearby mainline speed minus 2 m/s, the gate's speed-match tolerance. It needs mainline
  cars in view, so it applies to about half the cars.
- **M4, accelerate then brake:** on the ramp, the speed rises at least 1 m/s above its lowest point
  so far, then falls at least 1 m/s again. The 1 m/s is my choice.

**How a car fails:** if M1, M2 or M4 flags it, or if it never merges.

**SUMO and physics alike:** the checks need only the car's own trace and its leader. The row 11
physics drive (C2) fails M1 (22 decisions), M2 (6.6 s) and M4 (once).

**Their effect on the saved SUMO records** (post hoc: seen after the fact, not a test):

| Situation (cars) | Driver | M1 | M2 | M4 | Fails any |
|---|---|---|---|---|---|
| Clear (7) | expert v3 | 0 | 0 | 0 | 0 |
| | round 16 | 0 | 0 | 0 | 0 |
| Stream (12) | expert v3 | 11 | 8 | 3 | 11 |
| | round 16 | 2 | 1 | 3 | 5 |
| Blocked (5) | expert v3 | 3 | 4 | 4 | 5 |
| | round 16 | 5 | 4 | 4 | 5 |
| I-110 (7) | expert v3 | 2 | 4 | 7 | 7 |
| | round 16 | 5 | 5 | 6 | 6 |
| **All (31)** | **expert v3 (= the policy)** | **16** | **16** | **14** | **23** |
| | **round 16** | **12** | **10** | **13** | **16** |

- **Round 16's "fails any"** includes its 2 cars that never merged.
- **M3 flags 14 of the 15 expert v3 cars it applies to, and 12 of round 16's 16.**
  - That includes the clear merges with mainline cars in view: here every car leaves the junction
    below traffic speed.
  - So it can't tell good driving from bad in this network. I'd report it, not gate on it.

### 5. Fix options for the merge logic

**A. A Level 1 change: a new expert version, v4.**
- **The changes,** each aimed at one cause above:
  1. **No early slowing.** With no acceptable gap, keep the current speed until holding would need
     2.0 m/s² of braking; then slow. It still stays within the leader, curve and stop-line limits.
  2. **A run-up option.** Hold a moderate speed, then run up at 1.5 m/s² to cross the line as the
     gap arrives, as fast as that allows. Today the only options are the steady speed or a 0.5 m/s
     crawl. Your example, "match traffic speed, then take a gap", is as near as this yield junction
     allows: there's no lane to match speed in before the line.
  3. **A committed car runs up as planned.** Its target is the planned run-up speed, not the larger
     of the gate's target and its current speed. The 3.0 m/s² abort stays.
- **What it changes:**
  - **Both drivers:** Level 1 is the executor of both expert v3 and the policy, so both change, and
    so do their driver ids.
  - **The labels:** the policy's Level 2 labels came from expert v3's choices. If v4 picks gaps
    differently, a relabel and refit may be needed. That's unknown until the re-check.
- **Before any run (uncharged):**
  - the code and its tests;
  - an offline replay of v4's gap choice over the saved stream rows. `select_gap` needs only
    recorded inputs; closed-loop effects need SUMO.
- **The re-check:** a 70-row SUMO run of expert v4, and of the policy on v4. About 271,000
  decisions and 120–150 s at 16 workers, the plan's estimate for a Level 1 change.
- **The no-regression rule, set now:**
  - neither driver loses a development row it passes today, under either gate set (54 recorded,
    62 re-baselined);
  - no new event kind, in particular no native emergency and no cut-in;
  - fewer cars fail the movement checks than today's 23 of 31, and none of the 7 clear cars does;
  - rows where the change never acts reproduce their traces exactly.
- **If a relabel and refit are needed:** about 140 s for a fit and its 70-row check (the plan's
  cloning estimate), plus a labelling run of about 100 s (my estimate).
- **The held-out check can't follow.** Held-out rows are never re-run, so Stage 5's held-out result
  wouldn't carry over to v4. Confirming v4 would need new held-out rows (your decision): about 600 s
  at Stage 5's size.
- **Cost:**
  - about 150 s for the re-check alone;
  - about 900–1,000 s with a refit and new held-out rows, of the 1,414.8 s left.
  - No raise, but little would be left for the physics drives.

**B. Add acceleration lanes to the SUMO ramp junctions.**
- **It's the realistic fix,** if the real ramps have them (not checked).
- **What it changes:** every ramp row's SUMO result, for every driver and baseline.
- **The 3D model would have to match.** You said not to modify it, so this is your decision only.
- **The held-out comparison couldn't follow.**
- **Cost:** unknown. A network rebuild, plus at least 3 baselines re-run at about 100 s each.

**C. Report the movement checks, and change no driver.**
- **What it changes:** the reports. Every SUMO and physics report adds M1 to M4 for each car.
- **Cost:** 0 s.

### My recommendation for the ramp driving

1. **Adopt M1, M2 and M4 now, as reported checks,** with M3 reported alongside (decision 13).
   - Use them in the no-regression rule of any Level 1 change.
   - **Making them gates for every driver is your decision.** I'd decide that after option A's
     re-check; the thresholds are already fixed above.
2. **Take option A in two steps** (decision 14):
   - **First, uncharged:** write v4, its tests and the offline replay.
   - **Then, with your go-ahead:** the 70-row SUMO re-check (about 150 s, no raise), under the rule
     above.
   - **Not option B now.**
3. **Report the four situations separately, and compare drivers only on matched rows** (decision 15).
   - Don't judge the merge logic on the blocked rows alone.
   - Use a stream row, not row 11, for any future matched video.
   - Label every video physics or replay.
4. **Hold the physics test drives** (decision 16) until decision 14 is settled. A Level 1 change
   would change what they test. C2 as the default (decision 17) doesn't depend on it.

## Physics gap options (November 27): what next, for your decision

**Your decision (relayed by the main session):** "Write up the options", answering "What next for the
physics gap?" The other choice offered was "Stop here".

**Order:** this comes after [Ramp driving](#ramp-driving-november-27-the-slow-merges-for-your-decision), as you asked ("Yes, driving first"). Its
decisions are now 16 to 20.

**What this is:** a written proposal only.
- **Nothing ran:** no Isaac Sim, no SUMO, no charge.
- **Where the numbers come from:** reading code, configurations and the saved Stage 5 and physics-gap
  outputs, and offline Python over those logs.
- **The scripts and their outputs** are in `outputs/highway-merge-v15/physics-gap/options-analysis/`,
  outside Git.

### Where things stand

With C2, in PhysX (physics, not a replay), no southbound row passes either gate set. Two gates fail:
- **The stable merge, on every row.** 5 to 8 of the 81 decisions after the merge are realization
  flags; the limit is 4. Merge quality is scored only after a stable merge, so it fails too.
- **Per-car jerk, on rows 53 and 67:** 5.28 and 5.13, against a limit of 5.

Two findings from the offline analysis:
- **Where the flags fall.** 44% to 67% of each drive's flags (5 to 8 per drive) fall on the 2.4% to
  3.2% of decisions where the road slope changes by more than 0.005 within the decision.
  - In the stable-merge windows, these are 5 of the 7 flags on row 11, 4 of 7 on row 25, 5 of 8 on
    row 39, 4 of 8 on row 53 and 3 of 5 on row 67.
  - The rest fall where the slope barely changes.
- **The jerk on rows 53 and 67 isn't at the merge.**
  - **Where it is:** both peaks are at 0.2 s, the episode's second decision. The car starts on the
    ramp's 4.7% grade there (read from the physics log).
  - **What happens:** the command is −0.3 m/s². The gate's measure of what the car did is −0.53
    and −0.51 m/s²; the physics log's own measure is −0.39 and −0.38. In SUMO the same decision
    realizes −0.3 exactly (jerk 3.0).
  - **Not explained yet:** why the two measures differ at the first decision isn't investigated.
    None of the five options below targets it.

### What the saved SUMO records show (read offline)

- **The policy and expert v3 have no realization flags and no drive-limited decisions** in any
  saved SUMO record.
  - **The records:** the policy's 70 development episodes (Phase 3's relabel step); the policy's 43
    and expert v3's 42 held-out episodes (Stage 5 part 1).
  - **How exactly SUMO carries out the commands:** on the 5 southbound rows, the policy's commands
    are realized to within 2×10⁻¹⁴ m/s².
- **The older baselines do have them** (Phase 1's records):
  - **Round 16's policy:** on the development rows, 314 flags and 992 drive-limited decisions; in the
    held-out records, 209 flags and 428 drive-limited decisions.
  - **Round 21's final policy:** on the development rows, 984 flags and 872 drive-limited decisions.
  - **What these flags are:** SUMO's own logic overriding a command, by 0.05 to 4.3 m/s² (median
    0.19).
- **Level 1's drive ceiling does more than clip** (read in the code). It also sets:
  - how fast the policy's yield-entry target rises, which is one of the policy's inputs;
  - the re-baselined gate's reference band.

### Option 1: the gate (decision 12, revisited)

**What it would change:** which physics decisions count as realization interventions. The rule would
apply in the physics environment only. That count feeds three gates:
- per-car interventions;
- the stable-merge window;
- merge quality's override fraction.

**The case, on grounds that don't depend on these results:**
- **Where 0.05 m/s² comes from:** v05 built it for SUMO, to catch SUMO's own logic overriding a
  command. SUMO otherwise carries out a command exactly, so in SUMO it is a "was this overridden?"
  test with a wide margin.
- **What it means physically in PhysX:** 5 mm/s of speed over one 0.1 s decision, and 0.25 mm of
  distance.
- **Measurement resolution:** on calm decisions in the 10 physics-gap drives, the car's own mismatch
  is mostly far below the threshold.
  - **"Calm" means:** a steady command, the slope barely changing (under 0.001 in that decision and
    the one before), over 3 m/s, and no torque limit.
  - **The spread:** the median is 0.002 to 0.003 m/s², and the 99th percentile is 0.017 to 0.040.
  - **The worst case:** the largest calm mismatch exceeds 0.05 on 4 of the 10 drives (0.051 to
    0.068).
  - **So** the current threshold sits at the edge of the car's own noise.
- **What a real car does:** Liu et al. ran a field experiment with two vehicles carrying
  developable automated-driving systems: "Experimental study and modeling of the lower-level
  controller of automated vehicle" ([arXiv:2210.17160](https://arxiv.org/abs/2210.17160), 2022).
  - **What they measured:** their Table II gives a delay between the commanded and actual
    acceleration of 0.78 to 1.08 s, depending on the model fitted.
  - **Inferred:** a real car's acceleration trails its command by most of a second. A 0.05 m/s² test
    every 0.1 s would flag most of a real car's changing decisions.
  - **No published tolerance found:** I found none for this kind of intervention. ISO 15622 (adaptive
    cruise control) is paywalled and I haven't read it, so nothing here comes from it.
- **For a cumulative speed error:** the rest of the system works on speed and position.
  - **Why a short mismatch matters little:** the policy re-plans every 0.1 s from the measured
    speed, so a mismatch that doesn't build up changes little.
  - **The window has an anchor:** a trailing 1 s window is on the scale of a real car's response
    (Liu et al.).
  - **The threshold doesn't:** I found no independent anchor for it. The only speed tolerance in the
    gates, merge quality's ±2 m/s band, is far looser.
  - **The definition used below:** the sum, over the last 10 decisions, of (realized − commanded) ×
    0.1 s.
- **A third definition, mine** (not one of decision 12's two choices): a physics decision counts as
  a realization intervention only if it differs from the command by more than 0.05 m/s² and the
  low-level controller hit its drive or brake torque limit in that decision.
  - **Why:** it is the physics analogue of SUMO overriding a command: the car, not the policy,
    decided the motion.
  - **What it avoids:** it has no number to choose.
  - **What it misses:** a controller that tracks badly within its limits. Only per-car jerk and
    merge quality's speed match would catch that.

**What each rule would do on the saved drives.** This is post hoc, read after seeing the results: a
recount of the saved traces, with nothing re-run.
- **The recount is exact at today's rule.** At 0.05 m/s² it reproduces every recorded flag, override,
  window count and both verdicts on all 11 drives.
- **The C2 drives:**

| Rule (physics only) | Window flags on rows 11 / 25 / 39 / 53 / 67 | Rows passing, recorded gates | Rows passing, re-baselined gates |
|---|---|---|---|
| 0.05 m/s² (today) | 7 / 7 / 8 / 8 / 5 | none | none |
| 0.10 m/s² | 6 / 3 / 3 / 3 / 2 | 25 | 25, 39 |
| 0.15 m/s² | 2 / 0 / 1 / 1 / 1 | 11, 25 | 11, 25, 39 |
| 0.20 to 0.30 m/s², or no realization flags | 0 or 1 | 11, 25 | 11, 25, 39 |
| Speed error over 1 s above 0.05 m/s | 5 / 6 / 6 / 6 / 6 | none | none |
| Speed error over 1 s above 0.10 to 0.50 m/s | 0 | 11, 25 | 11, 25, 39 |
| Torque limit (mine) | 0: C2 never hit its torque limit | 11, 25 | 11, 25, 39 |

- **For comparison, in SUMO** the policy passes rows 11 and 25 under the recorded gates and all 5
  under the re-baselined gates.
- **Where the verdicts switch:**
  - row 11 between 0.10 and 0.15 m/s²;
  - rows 25 to 67 between 0.05 and 0.10 m/s²;
  - the speed error between 0.05 and 0.10 m/s.
  - So any value chosen after reading this table is fitted to these 5 rows.
- **Rows that still fail:**
  - **Jerk:** rows 53 and 67 fail per-car on jerk under every rule, at the first-decision transient
    above.
  - **Recorded merge quality:** rows 39, 53 and 67 fail it, as they do in SUMO. Its speed-match
    fraction there is 0, 0 and 0.24.
- **Merge quality would then be scored** wherever the stable merge passes.
  - **Under the re-baselined gates,** every C2 row then qualifies.
  - **Inside the quality window:** peak jerk is 3.1 to 4.75, the override fraction is at most 0.037,
    and the speed match is at least 0.925.
- **The current controller** never passes a row under any rule: safety always fails on its overspeed
  decisions.

**Can a SUMO result change?**
- **In physics only:** no.
- **In SUMO too:** nothing would change for the policy or expert v3, which have no flags there. It
  would change the older baselines' records. For example, a 0.15 tolerance drops 570 of Phase 1's
  1,302 flags, and 66 of the 209 held-out flags.

**Cost:**
- **The recount:** nothing.
- **The rule:** a code change in the physics environment, with tests and CI.
- **A test on rows that didn't choose it:** the 5 northbound rows (10, 24, 38, 52 and 66), never
  driven in physics, with C2, unrendered.
  - **Time:** by the plan's per-drive rule, 5 × 70.8 s = 354 s. The unrendered drives here took 28.5
    to 33.8 s each.
  - **Cap:** 400 s and 3,000 decisions. The policy took 1,499 decisions on these rows in SUMO.
  - **Left after the cap:** at least 1,014.8 s of study time.
  - **Before the drives:** uncharged ideal-driver checks (SUMO) on those rows.

**Risk of fitting to these 5 rows:**
- **High** for a tolerance or speed-error threshold picked now.
- **Lower** for the torque-limit rule, which has no number to pick. But the choice to change the gate
  at all comes after seeing the failures.
- **What would make it a result:** the northbound drives, with the reading set before them.

**Raise:** none.

### Option 2: the road data's vertical profile

**What it is (read):** the LA model's README says "Bridge heights are engineered approximations, not
surveyed values."
- **How the build makes them:**
  - the stacking from OpenStreetMap;
  - design grades of 5% on mainlines and 6.5% on ramps;
  - a grade-limited envelope;
  - vertical-curve smoothing.
- **What is left at the merge:** on row 11, the lane graph's slope still changes by up to 0.027
  within one decision. That is about 1.3 m of travel at 12.8 m/s.

**What smoothing would change (read):**
- the deck meshes the car drives on, which are built lane-first from the same profiles;
- `lanegraph.json`;
- the road slope the physics server gives C1 and C2;
- Level 1's drive ceiling (`traffic/road_grade.py`, which reads the lane graph). It bounds the
  command, sets the rise of the policy's yield-entry target and sets the re-baselined reference band;
- the SUMO corridors, which are cut from `lanegraph.json` and whose manifests pin its sha.

**Smoothing the lane graph alone** would leave the decks unchanged (inferred). The road the car drives
on and the road the controller and Level 1 read would then disagree.

**Does it count as modifying the 3D model?** Yes.
- **Why:** the decks are the model's meshes, and `highway_usd/la_10_110/` is the model package.
- **So it's your decision only:** you said not to modify the 3D model.
- **The build scripts aren't here:** the ones the README names (`build_all.py`, `tools/network.py`)
  aren't in this repository.

**Can a SUMO result change?** Yes.
- **Why:** the drive ceiling never clipped the policy or expert v3 in SUMO. But it sets the policy's
  target and the re-baselined band near the breaks, so the policy's SUMO actions can change
  (inferred from the code).
- **Formally:** every LA SUMO result would then be on a different road.

**The re-checks it needs:**
1. **The model:** rebuild it, and rerun its checks (clearance, obstacles, Isaac's spawn check).
2. **The corridors:** regenerate them and their manifests.
3. **SUMO:** rerun the development evaluation of the policy and expert v3.
   - Phase 3's relabel step took 107.6 s for the policy.
   - Each of Phase 2's three expert runs took 90.4 to 111.6 s.
4. **Physics:** rerun the southbound physics drives.
5. **What can't follow:** the held-out comparison, since held-out rows are never re-run. It would
   stay on the old road.

**Post hoc, an optimistic bound:**
- **The assumption:** smoothing removes exactly the flags at slope changes, and nothing else.
- **The windows would then hold** 2, 3, 3, 4 and 2 flags, so the stable merge would pass on every
  southbound row.
- **Rows 53 and 67 would still fail on jerk.**
- **Why it's optimistic:** the car still has to follow the smoothed grade change, and the other
  flags stay.

**Cost:** about 215 s of SUMO and up to 354 s of physics (estimated), plus the rebuild. That is
about 570 s of the 1,414.8 s, with no raise.

**Risk of fitting:** medium. How much to smooth would be chosen with these rows in view.
- **An independent rule:** a design standard would be one. The Caltrans Highway Design Manual sets
  minimum vertical-curve lengths.
- **But no numbers here:** I couldn't extract that manual's text, so I give none.

### Option 3: the car's physics parameters

**What's provisional (read):**
- **The package says so:** `vehicles/sim_ready/americano-i7-ev/v14/model.json` has
  `provisional_physics: true`, and its scope says "Provisional dynamics".
- **The mass:** 1,800 kg, which you approved as provisional.
- **The dimensions:** they come from the mesh scale, not from a manufacturer. I haven't found that
  this model matches a production car.
- **The values** (in `assets/vehicle-physics.usda` unless noted):
  - **Suspension, per wheel:** spring 45,000 N/m, damper 4,500 N·s/m and 0.2 m of travel.
  - **Tires:**
    - longitudinal stiffness 5,000;
    - lateral stiffness graphs (2, 76,500) front and (2, 112,500) rear;
    - friction-vs-slip flat at 1;
    - the friction table named `WinterTireFrictionTable` (0.75).
  - **Wheels:** moment of inertia 1.225 kg·m² and damping 0.25.
  - **Drive and brakes** (from the drive-limit settings): 700 N·m per front wheel, and 1,500 N·m of
    brake per wheel.
- **The v06 documentation:** the other values come from NVIDIA's installed vehicle example. Tire
  response, steering and braking need reference data and measured acceptance tests.

**What could change the flags:** the suspension and the mass set how the body pitches and heaves over
a grade break.
- **My estimate (not from a source):** a ride frequency near 1.6 Hz and a damping ratio near 0.5,
  taking a quarter of 1,800 kg per corner.

**What it would take to justify the values from a real vehicle, rather than tuning them to pass:**
1. Pick a real reference vehicle of about this size.
2. Take its mass, weight distribution and centre-of-gravity height from published specifications.
3. Take suspension rates and damping from published or measured data, and tire properties from tire
   test data.
4. Check the car against measured behaviour, as the v06 audit asked: acceleration, braking and
   step-steer tests.
5. Only then re-drive. Values changed until these rows pass would be tuning.

**Can a SUMO result change?** Only through Level 1's drive limit.
- **What the limit uses:** the car's mass, drive torque, wheel radius and inertia. These are recorded
  in every SUMO report's drive-limit settings.
- **Suspension and tires:** changing them alone touches no SUMO result.

**Cost:**
- **Sourcing the data:** uncharged.
- **The re-drives:** up to 354 s for the 5 southbound rows.
- **The earlier results:** every earlier physics result would then be an old-car result.

**Risk of fitting:** high if the values are tuned; low if they're sourced before any drive.

**Raise:** none.

### Option 4: C2 as the default physics controller

**What it would change:** the physics server's `--controller` default, from `stage5` to `c2`.
- **The change:** one line of code.
- **The fingerprint test** keeps pinning `stage5`'s commands when it's named.
- **Earlier physics results** keep the label of the controller they used.

**Can a SUMO result change?** No; the controller runs only in physics.

**Cost:** none in study time; a code change and CI.

**Risk of fitting:** low to moderate.
- **How it was picked:** on row 11, by a rule set before the drives.
- **On the 4 other rows** it improved every reported measure:
  - failed gates and flags;
  - window and per-car interventions;
  - overspeed and torque-limit decisions.
- **Not yet tested:** the northbound rows.

**Raise:** none.

**What it doesn't do:** close the gap by itself.

### Option 5: stop here

**What it would change:**
- **The SUMO results stand:** Phase 3, and Stage 5's held-out check, where the policy has the same
  verdicts as expert v3.
- **The physics gap is recorded as a known limit.** In PhysX, with C2, none of the 5 southbound rows
  passes the gates as they stand.
- **What's left:**
  - realization flags at the merge, 44% to 67% of them at grade breaks;
  - a first-decision jerk on rows 53 and 67.

**Can a SUMO result change?** No.

**Cost:** none; the 1,414.8 s stay unused.

**Risk of fitting:** none.

**Raise:** none.

### The options side by side

| Option | What changes | SUMO results can change | Study time, of 1,414.8 s | Risk of fitting to these 5 rows | Raise |
|---|---|---|---|---|---|
| 1. A physics-only realization rule | What counts as a realization intervention in physics | No | 0 to recount; about 354 s (cap 400 s) for a northbound test | High for a number picked now; lower for the torque-limit rule | No |
| 2. Smooth the road profile | The 3D model, the lane graph, the drive ceiling and the corridors | Yes | About 570 s plus a rebuild; the held-out check can't follow | Medium | No (estimate) |
| 3. The car's parameters | The physics car | Only through the drive limit (mass, torque) | Up to 354 s of re-drives | High if tuned; low if sourced first | No |
| 4. C2 as the default | The server's default controller | No | 0 | Low to moderate | No |
| 5. Stop here | Nothing; the gap is recorded as a known limit | No | 0 | None | No |

### My recommendation for the physics gap

1. **Make C2 the default** (option 4; decision 17). It improved every row on every reported measure,
   and it changes no SUMO result.
2. **For the gate, test the torque-limit rule in physics only** (option 1; decision 16).
   - **Why this rule:** it has no number to fit.
   - **When:** chosen now, before any further drive. Driven only after decision 14 (the merge
     logic), since a Level 1 change would change what the drives test.
   - **Where:** on the 5 northbound rows.
   - **What gets reported:** the verdicts under today's rule alongside it. The southbound numbers
     above stay post hoc.
   - **The reading, set now:** the rule is confirmed if, with C2 in PhysX, the policy passes every
     northbound row it passes in SUMO (10, 52 and 66) under both gate sets, with no collision.
     - Otherwise it is not confirmed, and the failure is reported as it is. That includes a failure
       for another reason, such as a first-decision jerk.
   - **If you'd rather not change the gate:** stop here (option 5), with option 4.
3. **Not options 2 or 3 now** (decisions 18 and 19).
   - **Option 2** modifies the 3D model, which you said not to do, and the held-out comparison
     couldn't follow it.
   - **Option 3** has no real-vehicle source yet.
4. **Take an offline look at the first-decision jerk on rows 53 and 67** (decision 20): reading code
   and logs only, uncharged.

### The whole recommendation, in order

1. **The ramp driving first:**
   - decision 13, the movement checks;
   - decision 14, option A in two steps;
   - decision 15, reporting by situation on matched rows.
2. **Then the physics gap:**
   - decision 17, C2 as the default, now;
   - decision 16, the torque-limit test drives, after decision 14;
   - decisions 18 and 19, no change to the road model or the car;
   - decision 20, the offline look at the first-decision jerk.

## Decisions for you

Decisions 1–7 were decided on November 25, 2026, relayed by the main Claude session and recorded
in the coordination master. Phase 0 started on "tell the other session to start phase 0";
decision 5 was "use clock time"; decisions 1–3, 6 and 7 were "Yes, all five", accepting the
main session's recommendations. Decision 8 was decided on November 26: "Recommended size",
relayed by the main Claude session.

1. **The two-level controller.** Does a rule executor with a learned maneuver policy fit your
   "one controller, policy only" direction?
   **Decided: yes.** A rule executor acts on every decision; a learned maneuver policy acts
   every 0.5–1 s.
2. **The stuck rule:** pull away automatically, or only count it?
   **Decided: pull away automatically.** Every such stop still counts as an event in the
   metrics and gates. The pull-away is part of the Phase 2 executor; its time threshold isn't
   set yet.
3. **Re-baselining the gates** on the 70 rows and the qualified expert.
   **Decided: once,** after the rule expert qualifies (Phase 2), on the 70 development rows.
   When the expert fails a gate, check whether the gate or the driver is wrong. The gates are
   frozen before any learning, and the held-out seeds stay untouched.
   **Done on November 26** after run 2, and frozen: `experiments/v15_regrade.py` (sha256
   `515c3b7c…`); see [Phase 2 result](#phase-2-result-november-26).
4. **Installing libsumo** into `.venv-traffic` (never into Isaac's Python).
   **Done in Phase 0:** libsumo 1.27.1, with sumo-data 1.27.1 pinned, in `.venv-traffic` only.
5. **Ledger treatment** of parallel runs.
   **Decided: clock time.** Each run charges its elapsed clock time against the time budget,
   however many SUMO workers it uses. The decision budget still counts every decision across
   all workers. Every run records its clock time, the summed time across workers and the
   worker count.
6. **Budgets** for Phases 1–4.
   **Phase 1 approved:** about 210 evaluation episodes (the round-16 policy, round 21's final
   policy and the plan follower, each on the 70 rows), charged by clock time. It starts only
   after Phase 0 passes its exit check, and once the Phase 0 result, Phase 1's resolved
   configuration and its expected charge are in the coordination master.
   **Phase 2 approved on November 26:** "Yes, approve 5 runs", at most 725,000 decisions,
   charged by clock time. 3 runs were used. The Phase 3–4 budgets still need you; they will be
   sized from Phase 0's throughput.
7. **v14's open question** (diagnose the southbound stops, or stop there): close it in favour
   of this plan?
   **Decided: closed in favour of v15.** No round 25 and no further v14 diagnosis.
8. **Phase 3's budget** (proposed on November 26; see
   [Phase 3 proposal](#phase-3-proposal-november-26-imitation-for-your-approval)).
   - **Recommended:** at most 2,700,000 decisions and 3,000 s of the 4,012.2 s of study time left
     (estimate: about 1,270 episodes, 2.2 million decisions and 2,400 s).
   - **Minimum:** at most 1,200,000 decisions and 1,400 s (estimate: about 580 episodes,
     1.0 million decisions and 1,100 s), with a single training seed.
   - Either is Phase 3's own allowance, with the settings of Phases 1 and 2.

   **Decided: recommended,** on November 26 ("Recommended size", relayed by the main Claude
   session). Phase 3 used 1,983,565 decisions and 1,385.7 s; see
   [Phase 3 result](#phase-3-result-november-26).
   The user then approved the relabel step on November 26 ("do the relabel fix first then work on
   the phase 4 budget"). With it, Phase 3 used 2,122,682 decisions and 1,493.3 s.
9. **Phase 4** (proposed on November 26; see
   [Phase 4 proposal](#phase-4-proposal-november-26-anchored-rl-on-level-2-for-your-approval)).
   - **Step A:** the probe (5 scripted Level 2 variants on the 8 failing development rows) and the
     relabel configuration's seed check: about 360,000 decisions and 290 s, with no raise.
   - **Step B:** anchored MAPPO, sized after the probe.
     - **Recommended:** at most 9,200,000 decisions and 5,700 s, with the study total raised by
       about 4,100 s.
     - **Minimum:** at most 4,900,000 decisions and 3,100 s, with a raise of about 1,500 s.
   - **Or skip Phase 4** and go to Stage 5 (it needs your authorization, and diagnostic episodes).

   **Step A decided: yes,** on November 26 ("Yes, run step A"). The probe stopped at its stop rule
   after its first batch. Its second batch then ran, after "Yes, run batch 2"; see
   [Phase 4 step A result](#phase-4-step-a-result-november-26).
   - **Step B decided: skipped,** on November 26 ("Yes, run the check", also recorded by the main
     Claude session). The +1 offset check ran instead. It scores 61 of 70 re-baselined and 54
     recorded, so it doesn't join Stage 5; see
     [Phase 4 offset check](#phase-4-offset-check-november-26).
   - **Next:** Stage 5 with the Phase 3 policy, which needs your authorization; see decision 10.
10. **Stage 5** (proposed on November 26; see [Stage 5 proposal](#stage-5-proposal-november-26-the-held-out-check-and-physics-drives-for-your-approval)).
    - **Part 1, the held-out rows in SUMO:**
      - **Recommended:** the policy, expert v3 and round 16, at most 300,000 decisions and 300 s.
      - **Middle:** the policy and expert v3, at most 200,000 decisions and 250 s.
      - **Minimum:** the policy alone, at most 110,000 decisions and 150 s.
    - **Part 2, the physics drives:** expert v3 and the policy on development row 11.
      - First an uncharged build and development drive, then two charged drives (at most 500 s).
      - The episodes are counted as Stage 5 evaluation episodes (recommended), or as diagnostic
        episodes with a raise of 1.
    - **No raise of the study total** is needed for any option.

    **Decided: recommended,** on November 27, 2026 ("Approve recommended", typed in the Claude side
    session). Part 1 runs all three drivers, at most 300,000 decisions and 300 s. Part 2's drives are
    counted as Stage 5 evaluation episodes, at most 500 s. See
    [Stage 5 part 1](#stage-5-part-1-november-27-the-held-out-check).
11. **The physics gap** (proposed on November 27; see
    [Physics gap proposal](#physics-gap-proposal-november-27-why-row-11-fails-in-physics-for-your-approval)).
    - **Minimum:** C1 on row 11 only, at most 150 s.
    - **Recommended:** C1 and C2 on row 11, then the pick and the current controller on the other 4
      southbound rows, at most 850 s and 6,000 decisions.
    - **Larger:** the recommended drives, then the 5 northbound rows, at most 1,650 s and 10,000
      decisions.
    - **The episodes:** counted as evaluation episodes in their own allowance (recommended), or as
      diagnostic episodes with a raise of 9 (recommended option) or 19 (larger).
    - **No raise of the study total** is needed for any option.

    **Decided: recommended,** on November 27, 2026 ("Yes, as recommended", relayed by the main
    session). It ran: C2 was picked, and the gap was not closed. See
    [Physics gap result](#physics-gap-result-november-27-c1-and-c2-in-physx).
12. **Realization mismatches in physics:** do they count as interventions?
    - **Keep as now** (recommended until C1's result is in);
    - **or count only mismatches above a physics tolerance;**
    - **or count a cumulative speed error.**
    - Changing this alone leaves row 11 failing two gates (peak jerk and overspeed).

    **Decided: keep,** on November 27, 2026, to be revisited with you after the results.
    - **What has changed since:** with C2, row 11's peak jerk (4.75) and overspeed (0) now pass.
    - **What's left:** every southbound row fails only on the stable merge, merge quality, and, on
      rows 53 and 67, per-car jerk.

    **Revisited** in [Physics gap options](#physics-gap-options-november-27-what-next-for-your-decision): decision 16.

Decisions 13 to 20 were proposed on November 27, 2026. Nothing in them has run.
- **Decisions 13 to 15** are the ramp driving (see [Ramp driving](#ramp-driving-november-27-the-slow-merges-for-your-decision)).
- **Decisions 16 to 20** are the physics gap (see [Physics gap options](#physics-gap-options-november-27-what-next-for-your-decision)).

13. **Movement checks:** adopt M1, M2 and M4 as reported checks, with M3 alongside, and use them in
    the no-regression rule of any Level 1 change (recommended).
    - **Or also make them gates now,** with the thresholds set in the proposal.
    - **Or neither.**
14. **The merge logic:**
    - **Option A in two steps** (recommended):
      - **first, uncharged:** expert v4's code, its tests and an offline replay;
      - **then, with your go-ahead:** a 70-row SUMO re-check of expert v4 and the policy (about
        271,000 decisions and 150 s, no raise), under the no-regression rule set in the proposal.
    - **Or option B:** acceleration lanes in the SUMO network. It would modify the 3D model, so it's
      your decision only.
    - **Or option C:** report the checks and change no driver.
    - **New held-out rows** for a changed driver would be a separate decision (about 600 s).
15. **The ramp rows in reports** (recommended):
    - report the four situations separately;
    - compare drivers only on matched rows;
    - don't judge the merge logic on the blocked rows alone;
    - use a stream row, not row 11, for any future matched video.

    Or keep today's reporting.
16. **The physics gate:** what counts as a realization intervention in physics?
    - **The torque-limit rule, tested on the 5 northbound rows** (recommended).
      - **The drives:** C2, unrendered, at most 400 s and 3,000 decisions, counted as evaluation
        episodes in their own allowance.
      - **The reading:** as set in the proposal.
      - **First, uncharged:** the ideal-driver checks.
      - **Code changes:** the rule and the runner, with CI before any drive.
    - **Or a tolerance or trailing 1 s speed-error threshold,** with the value named by you before any
      drive, and tested the same way.
    - **Or keep the gate and stop here** (option 5).
    - No choice needs a raise.
17. **C2 as the default physics controller:** yes (recommended) or no.
18. **The road profile:** leave the 3D model as it is (recommended), or ask me for a smoothing
    proposal, which would modify the model.
19. **The car's physics parameters:** keep them provisional and labelled so (recommended), or ask me
    for a proposal to source them from a real reference vehicle.
20. **The first-decision jerk on rows 53 and 67:** an offline look, reading code and logs only and
    uncharged (recommended), or leave it.

## What carries over from v14

- The LA model, unchanged.
- The drive limit and the yield plan's crossing check, which become part of the executor.
- The free-road stop rule and stop counts.
- Multi-seed validation.
- The cut-in measurement.
- The round-16 policy and round 21's final policy as baselines.
- The settling-point screen.
- Drive saturation: whether a hill-limited request counts as an intervention stays your
  decision.

## What stops

- Reward or gate patches after each failed round.
- Choosing results on one seed per case, or one training run.
- PPO warm-started from round 16 as the default way forward.
- Building another version on top of the v14 environment.
