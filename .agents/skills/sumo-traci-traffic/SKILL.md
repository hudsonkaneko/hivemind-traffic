---
name: sumo-traci-traffic
description: Build and diagnose deterministic SUMO traffic scenarios controlled from Python through TraCI for autonomous-vehicle coordination projects. Use for scenario setup, live vehicle control, traffic baselines, telemetry, replay, or preparing a SUMO backend for reinforcement learning; do not use for defining the multi-agent learning API or high-fidelity vehicle physics.
---

# SUMO and TraCI Traffic Control

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

## Outcome

Produce a reproducible, headless SUMO scenario with a Python controller that advances exactly one known simulation interval per step, controls selected vehicles, records baseline metrics, and can be wrapped by a learning environment. SUMO is the authority for roads, traffic lights, routes, collisions, and vehicle motion; downstream tools consume its state rather than independently moving the same vehicles.

## Start With Read-Only Checks

Before editing or running anything:

- Inspect the repository layout and locate the `.sumocfg`, `.net.xml`, route, additional, and output files actually referenced by the configuration.
- Record `sumo --version`, the Python version, and `traci.__file__`; a different TraCI package than the installed SUMO release is a common source of failures.
- Inspect configured step length, seed, begin/end times, teleport behavior, collision policy, and existing output paths.
- Identify which vehicles are policy-controlled and how their SUMO IDs are assigned. Do not assume a vehicle exists before departure or after arrival.
- Preserve source scenarios and prior results. Use a separate run directory or explicit new output names when execution would overwrite files.

For a review or diagnosis request, remain read-only. Do not install software, modify system environment variables, open a GUI, overwrite inputs, start a long batch run, or publish data unless the user authorized that action.

## Workflow

1. **Define the experiment contract.** Fix the network, demand/routes, controlled vehicle IDs, simulation step length, control interval, seed, episode horizon, and success metrics. Create at least one scripted or normal-traffic baseline before adding learning.
2. **Create a lifecycle-safe controller.** Build the SUMO command as an argument list; implement `reset`, `step`, and idempotent `close`; and release the TraCI connection in `finally`. Use headless `sumo` for repeatable runs and `sumo-gui` only for an explicitly requested visual diagnosis.
3. **Use one clear step order.** On an environment step, validate currently active vehicle IDs, apply the complete action batch, call `simulationStep()` once, then read the resulting state, events, and metrics. Log both simulation time and episode step.
4. **Subscribe to recurring state.** Prefer vehicle subscriptions and context subscriptions for repeated ego and neighbor variables. Maintain a stable policy-agent-to-SUMO-ID map while separately tracking departed, active, arrived, and removed vehicles.
5. **Translate tactical actions.** Map actions such as desired speed, acceleration, lane change, or route choice to a small adapter. Clamp invalid values and report rejected commands. Keep requested actions and realized motion in the log.
6. **Measure behavior.** Record throughput, travel time, time loss, waiting/queue time, stops, collisions, emergency braking, route completion, and control interventions at consistent intervals. Save the configuration, seed, controller version, and metric schema with each run.
7. **Prove replayability.** Run a short scenario twice with the same seed and scripted actions, then compare vehicle state and metric traces. Exercise at least one different seed to ensure the scenario is not accidentally hard-coded.
8. **Optimize only after correctness.** Use subscriptions before redesigning the system. Consider `libsumo` for faster headless training only after TraCI tests pass; keep the adapter boundary so the backend can change without changing the environment contract.

## Non-Obvious Constraints

- A TraCI command is not automatically the realized vehicle behavior. SUMO car-following, lane-change, and safety logic may limit it. Do not disable `speedMode` or `laneChangeMode` protections unless that is an explicit, documented experiment; log requested and realized values.
- Do not let multiple threads call `simulationStep()` on one connection. For parallel training, prefer isolated processes with unique connection labels/ports and unique output paths. `libsumo` parallelism also requires separate processes.
- Never query a cached vehicle ID without checking the current active set. Departures and arrivals change that set during an episode.
- Keep training independent of wall-clock time and GUI frame rate. The SUMO simulation clock is authoritative.
- Coordinate transformations, OpenUSD writes, and rendering belong downstream; they must not feed poses back into SUMO unless the experiment explicitly defines a single synchronization owner.

## Acceptance Criteria

- SUMO starts, resets, steps, and closes cleanly from Python without orphaned processes.
- A scripted controller completes the target scenario and produces a baseline metrics file.
- One controller step produces exactly one documented SUMO advance and no stale-ID exceptions.
- Identical seed, configuration, and action trace reproduce the same relevant state and metric trace within stated numeric tolerances.
- The run log identifies versions, seed, scenario, controlled vehicles, requested actions, realized states, termination reason, and output files.
- Automated tests cover reset/close, agent-ID lifecycle, action translation, one-step timing, deterministic replay, and failure cleanup.

## Troubleshooting

- **Connection refused or immediate exit:** run the generated SUMO command directly, inspect SUMO stderr, configuration paths, port/label reuse, and whether an earlier connection was closed.
- **Wrong or missing TraCI API:** print `traci.__file__` and compare the imported package with the intended SUMO installation before changing `PATH` or `SUMO_HOME`.
- **Unknown vehicle ID:** reconcile departed/arrived lists and rebuild the active set after each simulation step; do not treat `possible_agents` as active vehicles.
- **Actions appear ignored:** inspect speed and lane-change modes, car-following constraints, lane availability, action duration, and the realized-state log.
- **Runs differ with the same seed:** seed every route/scenario generator, keep step and action ordering stable, and remove wall-clock or unordered-collection dependencies.
- **Training is slow:** turn off the GUI and rendering, batch recurring reads with subscriptions, reduce logging frequency, then evaluate `libsumo` behind the tested adapter.

## Official References

- [SUMO documentation](https://sumo.dlr.de/docs/)
- [TraCI overview](https://sumo.dlr.de/docs/TraCI/)
- [Python TraCI interface](https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html)
- [Object subscriptions](https://sumo.dlr.de/docs/TraCI/Object_Variable_Subscription.html)
- [Context subscriptions](https://sumo.dlr.de/docs/TraCI/Object_Context_Subscription.html)
- [libsumo and its limitations](https://sumo.dlr.de/docs/Libsumo.html)
- [Official SUMO downloads](https://sumo.dlr.de/docs/Downloads.php)
