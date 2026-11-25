# Highway merge plan v15: rules first, learning on top

Proposed by Claude on November 25, 2026, after v14 round 24. **This is a proposal only.
Nothing in it is built, trained or authorized.** Every phase that charges the shared ledger
needs the user's approval of its resolved configuration first, as before.

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

## Decisions for you

1. **The two-level controller.** Does a rule executor with a learned maneuver policy fit your
   "one controller, policy only" direction?
2. **The stuck rule:** pull away automatically, or only count it?
3. **Re-baselining the gates** on the 70 rows and the qualified expert.
4. **Installing libsumo** into `.venv-traffic` (never into Isaac's Python).
5. **Ledger treatment** of parallel runs.
6. **Budgets** for Phases 1–4.
7. **v14's open question** (diagnose the southbound stops, or stop there): close it in favour
   of this plan?

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
