# Two-car lidar and explicit cooperation

This milestone extends the single-car real-time demo. It is a scripted sensing-and-control experiment, not a trained PPO policy. See [the validation plan](cooperative-lidar-plan.md) and the separate results report for what has actually passed.

Start with the [work summary](cooperative-lidar-summary.md) for completed changes, the development issues, measured outcomes, and still-pending fixes.

## Who does what

| Component | Responsibility | Information it must not supply to the controller |
| --- | --- | --- |
| SUMO / TraCI | Move both cars; accept a complete action batch before one traffic tick | Hidden peer or obstacle truth as perception |
| Isaac RTX lidar | Produce an independent timestamped point cloud for each car | Labels or exact object locations as a substitute for sensing |
| Local tracker | Cluster that car's returns, estimate motion, expire missing tracks | Other car's private tracks or simulator vehicle IDs |
| Scripted controller | Choose speed and request a lane using local clearance | Permission to merge based only on a message |
| V2V message bus | Deliver intentions with episode, sender, sequence and expiry | Guaranteed delivery or a shared omniscient planner |
| Evaluator | Measure actual collisions, separation, completion and timing | Feedback into driving decisions |

Both controllers decide from the same traffic tick. All receive messages before either publishes its next intention; this prevents processing order from giving one car same-tick information that the other cannot have. Messages may reduce cooperation latency, but local sensing remains the safety check.

## Concepts worth being able to explain

**Tracking is not identification.** Nearby lidar returns are clustered into local bounding boxes. Matching boxes across time gives an estimated velocity. A local track number is not the true SUMO vehicle ID. Partial surfaces, occlusion and changing viewpoints can change those estimates.

**Bounded memory prevents ghost obstacles.** A missing track survives briefly with a predicted location and increasing uncertainty, then expires. A maneuver may retain the observed end of the static obstruction while passing it; this is a local maneuver landmark, not an accumulating map of every return.

**Freshness is part of the data contract.** A scan has per-ray times, a coordinate frame, a sensor owner and an acquisition interval. Track `last_seen` is the last observation time; `state_time` is the time to which its returned box has already been predicted. Confusing these would predict motion twice. Track `age` instead measures time since its creation.

**Communication is advisory.** An intention says, in effect, “I would like to enter your lane.” The other car can yield, but the requesting car still waits for its own lidar-based clearance. Expired, foreign-episode, duplicate and out-of-order messages are rejected. This is a simulation protocol, not an authenticated radio network.

**Real-time factor is not a safety certification.** One simulated second per wall-clock second is the target. We also measure late control deadlines: a good average can hide individual pauses. Initialization is outside the timed loop. Desktop tests cannot establish hard-real-time guarantees.

## Demo commands

Run from the repository root after its Python environment and external SUMO/Isaac runtimes are configured. These commands contain no user-specific paths. Isaac RTX lidar requires a compatible NVIDIA RTX machine; portable paths do not make the hardware dependency disappear.

The tested runtime is Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl`. The launcher itself uses standard-library Python; CPU tests/audits use the dependencies in `experiments/live-lidar-requirements.txt`, and the simulation runs in Isaac's Python with SUMO's bundled TraCI tools. Configure `ISAAC_SIM_PATH` and `SUMO_BINARY` when automatic discovery does not match your installation. A different Isaac API release still needs compatibility testing.

```text
python scripts/demo_cooperative_lidar.py --check
python scripts/demo_cooperative_lidar.py --comm ideal --seconds 45
python scripts/demo_cooperative_lidar.py --comm none --seconds 45
python scripts/demo_cooperative_lidar.py --comm degraded --seed 42 --seconds 45
```

Use `--headless` for measurements without a viewer and `--points` to request lidar point visualization. The launcher locates the configured external Isaac runtime; it does not import Isaac into the repository's ordinary Python environment.

```text
python scripts/demo_cooperative_lidar.py --headless --comm ideal --dropout-step 20 --seconds 25
python experiments/run_cooperative_suite.py
```

The dropout case deliberately withholds one car's scans starting at control tick 20. This is a fail-safe braking test, not a successful obstacle-passing demonstration. Each run writes its configuration, raw scans, control records and results to a unique directory under `outputs/cooperative_lidar/`. Suite logs belong in `logs/`.

## Runtime choices and limitations

The first two-sensor smoke test exposed an initialization mistake: overriding the Replicator writer's `initialize` method skipped the base behavior that calls `__init__`. The per-sensor owner was assigned, but its annotator list stayed empty, so neither stream produced packets. The fix initializes the writer layout/annotators explicitly before binding each sensor's private destination. This follows [NVIDIA's custom-writer lifecycle](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/custom_writer.html); the installed runtime source was checked as well. The failed run is retained, not counted as a passing demo.

A second smoke test exposed self-occlusion: the earlier sensor at 1.0 m sat below the 1.6 m body roof and did not see the car behind. Both sensors now mount at 1.8 m. An evaluation-only gate checks that each stream actually contains returns near the other vehicle during the opening second. This truth-based diagnostic is calculated after both driving decisions and never supplies observations to the controllers.

The no-communication trial then exposed a scan-validity error. A complete scan returned hits spanning only 65 ms because some firing directions hit nothing. Requiring successful returns to span at least 90 ms incorrectly declared that sensor unhealthy. The fleet path now requires native `scanComplete` metadata with `accumulateOutputs=True`, validates ray offsets against the configured 100 ms period, and uses the scan-origin timestamp for a conservative age limit. It does not infer complete angular visibility from those returns. The older one-car caller keeps its existing behavior. See [NVIDIA's scan accumulation and firing-period definitions](https://docs.omniverse.nvidia.com/kit/docs/omni.sensors.nv.lidar/3.0.0/lidar_extension.html).

The first 45-second roof-sensor pilot finished safely but failed the timing gate: one 192 ms full-generation garbage collection caused four late control ticks. The experimental freeze-established mode correctly declined to unfreeze objects already owned by the runtime, so it effectively ran normal collection. The default therefore defers cyclic collection only inside the bounded loop and restores the previous state afterward. This is a latency/memory tradeoff, not a permanent memory-leak fix. The result report includes memory measurements and retains this failed timing trial.

The demo uses 10 Hz traffic/control and 30 Hz interpolated rendering, with a reduced-density rotary lidar on each car. SUMO remains the only motion authority. Rendering interpolates its two successive states; it does not independently integrate vehicle physics.

The earlier one-car implementation saw GUI process-memory growth. The new runner adds low-frequency memory samples, bounded tracking/messages, optional point drawing and scoped garbage-collection experiments. A short run or a flatter memory curve does not prove that a native leak is fixed. Compare GUI point drawing on and off, repeated launches, and longer bounded runs before making that claim.

NVIDIA documents that each additional sensor and Motion BVH increase resource use. Motion BVH models motion during ray acquisition; the output compensation setting controls the resulting coordinates. They are different settings. This fixture retains the previously validated WORLD/NONCOMPENSATED mode and records its configuration. See [NVIDIA RTX sensor performance guidance](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx.html) and [lidar motion/output conventions](https://docs.omniverse.nvidia.com/kit/docs/omni.sensors.nv.lidar/latest/lidar_extension.html).

The experiment uses a known two-lane map and simple box-shaped vehicles/obstacle. It does not yet establish general road understanding, realistic steering dynamics, learned V2V, communication security, or scalable fleet performance. Those remain separate milestones.

## Reproduce and audit

The independent auditor reloads raw point clouds and recreates each local tracker, controller, message bus and recipient inbox. It checks their decisions against the recorded actions, validates file/source hashes and sensor ownership, and recomputes geometric clearance from the recorded poses. It refuses to silently replay with changed controller/perception source. Wall-clock receipt age and deadline timestamps remain recorded measurements rather than independently observable sensor facts.

```text
python scripts/audit_cooperative_lidar.py outputs/cooperative_lidar/RUN_DIRECTORY
python experiments/report_cooperative_results.py outputs/cooperative_lidar/suite-SUITE_ID.json
```

Replace the uppercase placeholders with a run or suite path printed by the launcher. Do not add files inside an already-hashed run directory; keep notes and audit reports outside it. The results report should include failed attempts as well as passing runs.

The clearance check intersects continuous constant-velocity intervals over five seconds; checking only a few sampled future times can miss a fast vehicle crossing between them. This is still a prediction assumption, not a guarantee against acceleration or an unexpected maneuver. A healthy scan is not proof that an occluded area is empty. Track expiration, static-object classification from partial surfaces, and interruption of an already-active SUMO lane change remain limits outside this fixture. A sensor dropout prevents new requests and reduces speed, but does not by itself cancel a lane change already in progress. The fault test must state when the dropout occurs.
