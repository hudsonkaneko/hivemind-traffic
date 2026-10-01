# Two-car live lidar: work summary

Updated September 30, 2026. Work and evidence are in the canonical `highwaysim` repository.

## Where we ended up

We now have a live two-car demonstration in which each car has its own RTX lidar, its own local object tracker, and its own scripted controller. The cars can exchange lane-change intentions so one can yield while the other passes a stationary obstacle and returns to its original lane.

**The demonstration works, but the full stability milestone is not finished.** Eight of nine cases in the latest suite passed all their per-run gates. The two-minute case remained safe and completed the maneuver, but missed the strict timing gate. Repeated fresh runs also did not produce identical trajectories. These are recorded limitations, not hidden successes.

## What changed and why

| Change | Purpose and technology |
| --- | --- |
| Two independent lidar streams | Isaac Sim/RTX produces a separate timestamped point cloud for each car. Ownership, sensor positions and actual returns on the neighboring car are checked to catch mixed-up or blind sensors. |
| Local moving-object tracking | NumPy/Python groups nearby returns into boxes, associates them across scans, estimates velocity and expires missing tracks. Each car has its own bounded tracker; track numbers are not simulator vehicle identities. |
| Batched vehicle control | TraCI submits both cars' actions before advancing SUMO exactly once per 0.1-second traffic step. This avoids giving one car an extra simulation step. |
| Cooperative scripted behavior | The passing car requests a gap. The other car can slow down in response, but messages never override local lidar clearance checks. This is a readable baseline, not a learned driving policy. |
| Explicit V2V protocol | Messages carry version, episode, sender, sequence, timestamp and expiry. Tests cover stale, duplicate, out-of-order and invalid messages. Comparisons include no messages, ideal delivery, and seeded delay/drop. |
| Live timing and evidence | The runner uses 10 Hz traffic/control and 30 Hz rendering. It records raw scans, decisions, message delivery, clearance, deadlines, memory samples and source/configuration hashes. |
| Independent replay audit | A separate auditor reloads the raw evidence and reconstructs filtering, tracking, decisions and message delivery. It checks that the recorded behavior follows from the recorded inputs. |
| Viewer and portable launch | The GUI distinguishes the blue and yellow cars and shows their phases, speeds and message counts. Commands locate the repository and external runtimes without hard-coded usernames. |

SUMO still owns vehicle motion; TraCI is the interface used to control it. Isaac supplies the 3D scene and simulated lidar. Rendering interpolates SUMO's states rather than running a second vehicle-motion solver. Simulator truth is used afterward for evaluation, not as hidden perception input.

**PettingZoo, PPO training, low-level steering physics and ovrtx were not added or changed in this milestone.** Those remain separate parts of the roadmap.

## What happened during development

1. **A prediction bug was caught in review.** Checking clearance at only 0, 2 and 5 seconds could miss a fast car crossing between those times. The check now considers continuous relative-motion intervals, with regression tests for rear and lateral approaches.
2. **The first live run produced no sensor packets.** The custom writer's initialization bypassed the code that establishes its annotators. Explicit initialization fixed this; tests now exercise the actual writer with two separate destinations.
3. **The next test revealed a blind spot caused by our own geometry.** The 1.0 m sensor mount was below the 1.6 m car roof and could not see the car behind. Moving the sensors to 1.8 m restored those returns. A new evaluation gate checks that both cars actually see each other at the start.
4. **A correct scan was being rejected.** Some completed scans had valid hits spanning only about 65 ms because other directions hit nothing. The old 90 ms minimum-hit-span rule was wrong for that situation. The fleet now uses native `scanComplete` metadata, explicit accumulation and timestamp bounds.
5. **Garbage collection caused a measured timing pause.** One 192 ms full-generation collection produced four late control ticks in a pilot. Cyclic collection is now deferred only inside the bounded run and its prior state restored afterward. This improves latency but is not an indefinite memory-management solution.
6. **The final audit exposed startup-dependent divergence.** Preparing garbage collection after warm-up aged the first packet to about 254 ms in one run and 244 ms in another. With a 250 ms limit, one car braked immediately in only the first run. Refreshing sensors after preparation is the identified next fix; it has not yet been integrated and retested.
7. **Longer testing exposed additional stalls and GUI memory growth.** The two-minute run had gaps not accounted for by the main render/control profile. Repeated synchronous telemetry-file writes are a hypothesis, not a proven sole cause. Bounded background logging is a planned follow-up, not an implemented fix. GUI memory growth remains unresolved.

The work used parallel GPT-6.1-sol agents for tracking, communications, runtime investigation and audit support. GPU simulations were run serially. NVIDIA documentation and installed runtime source were checked when sensor and lifecycle behavior differed from expectations. Failed trials and their logs were retained.

## What the latest tests showed

Latest full CPU test run: **139 passed**. All nine simulation evidence packages passed the independent integrity/replay audit; that does not mean all nine met the behavioral and timing requirements.

| Test | Result |
| --- | --- |
| No communication, 45 seconds | Passed; maneuver completed in 28.2 seconds. |
| Ideal communication, 45 seconds | Passed; completed in 19.7 seconds. |
| Delayed/dropped communication, 45 seconds | Passed; completed in 20.0 seconds, with 197 of 900 transmissions dropped. |
| Repeated ideal run | Passed its individual gates, but its trajectory was not identical to the first run. |
| Additional seed | Passed; completed in 19.5 seconds. |
| Sensor dropout | Passed the braking/no-new-lane-request test. Completing the pass was not its objective. |
| GUI, point drawing off/on, 60 seconds each | Both passed safety and timing gates. |
| Headless, 120 seconds | Safe and maneuver completed; failed timing with 28 control ticks more than 100 ms late. |

The communication comparison is promising **for this fixture**, not statistical proof that V2V always improves driving. All cases averaged approximately one simulated second per wall-clock second, but the long run demonstrates why an average alone is insufficient.

The same-seed repeat differed by up to about 0.55 m for the passing car and 0.84 m for the other car. Its phase/request sequence also differed. SUMO and message delivery are seeded; a controllable seed for native RTX noise has not been established. Startup packet age is a demonstrated source of divergence, and repeatability must be tested again after fixing it.

## Memory findings and remaining work

The 60-second GUI runs increased process RSS by approximately **400 MiB without point drawing** and **451 MiB with it**. The two-minute headless run increased RSS by about 43 MiB. RSS covers the entire application; these measurements do not isolate a leak or prove indefinite stability. Point drawing is therefore not the sole explanation. Inspecting the camera helper also did not support the proposed ordinary undo-history explanation.

Next actions, still pending:

- Refresh stationary sensor frames after startup preparation, reset the sensor epoch, then start the measured loop.
- Implement and test bounded background telemetry writing; retain errors rather than silently dropping evidence.
- Compare fixed and following cameras to narrow the GUI memory-growth cause.
- Repeat the matched suite and long-run timing checks after those changes.

Broader safety work also remains: a healthy scan does not prove an occluded area is empty, and refusing new lane requests does not cancel a lane change SUMO already accepted. This is a known-map, two-car, static-obstacle demonstration—not general autonomous driving or hard-real-time certification.

## Run the current demonstration

From the repository root, with the external Isaac and SUMO runtimes configured:

```text
python scripts/demo_cooperative_lidar.py --comm ideal --seconds 45
python scripts/demo_cooperative_lidar.py --comm ideal --points --seconds 45
python scripts/demo_cooperative_lidar.py --headless --comm none --seconds 45
```

These are live simulations, not recorded playback. The viewer closes when the requested duration finishes. Compatible RTX hardware and runtime versions are still required.

For deeper reference: [implementation and learning guide](cooperative-lidar.md), [validation plan](cooperative-lidar-plan.md), [detailed results](cooperative-lidar-results.md), and [machine-readable evidence summary](cooperative-lidar-results.json). The latest suite is `outputs/cooperative_lidar/suite-20260930T092530Z-6b36ad40ff3e.json`; raw runs remain under `outputs/cooperative_lidar/` and logs under `logs/`.
