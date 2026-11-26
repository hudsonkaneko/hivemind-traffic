# Highway merge plan v15: rules first, learning on top

Proposed by Claude on November 25, 2026, after v14 round 24. Every phase that charges the
shared ledger needs the user's approval of its resolved configuration first, as before.

**Status (November 26, 2026):** the user decided all seven questions at the end of this plan.
- **Phase 0** (tools only, no training, nothing charged) passed its exit check; see
  [Phase 0 result](#phase-0-result-november-25).
- **Phase 1** (baselines, evaluation only, charged) ran and met its exit check; see
  [Phase 1 result](#phase-1-result-november-25).
- **Phase 2** (the rule expert, evaluation only, charged; the user approved up to 5 runs) met its
  exit check in 3 runs; see [Phase 2 result](#phase-2-result-november-26).
  - The rule expert v3 passes 54 of 70 development rows under round 24's gates and 62 of 70 under
    the gate re-baseline, now frozen. Round 16 passes 33 and 36.
- **Phase 3** has no budget yet. Nothing has been trained under this plan.

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

## Decisions for you

All seven were decided on November 25, 2026, relayed by the main Claude session and recorded
in the coordination master. Phase 0 started on "tell the other session to start phase 0";
decision 5 was "use clock time"; decisions 1–3, 6 and 7 were "Yes, all five", accepting the
main session's recommendations.

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
