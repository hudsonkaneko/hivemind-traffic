# Hivemind Traffic implementation roadmap

## Committed architecture

Isaac Sim with RTX lidar is a required project stage, not an optional visualization
extra. SUMO remains the authority for scalable traffic behavior and early policy
training. Isaac Sim provides the 3D scene, lidar returns, and later physical
validation for selected vehicles.

No vehicle may have two pose authorities during the same interval:

- In playback/sensor mode, SUMO owns every vehicle pose and Isaac Sim renders lidar.
- In later hybrid mode, Isaac Lab owns physics for selected ego vehicles while SUMO
  owns background traffic through an explicit synchronization boundary.

## Timeline and gates

### 1. Deterministic SUMO foundation — complete

- Single controlled vehicle and two-lane highway.
- Lifecycle-safe TraCI backend.
- Scripted baseline, telemetry, manifests, and deterministic replay tests.

Exit evidence: `agent_0` completes the route without collisions and repeated seed-42
runs match exactly.

### 2. PettingZoo traffic environment — complete

- Wrap the TraCI backend as a `ParallelEnv`.
- Begin with one agent, then expand to two simultaneously acting vehicles.
- Define invariant observations and actions, separate termination from truncation,
  and pass PettingZoo API and seed tests.
- Keep observations state-based; do not introduce lidar into training yet.

Exit evidence: the curved two-agent scenario passes PettingZoo API and seed tests,
fixed-action replay is deterministic, and both controlled agents complete the route
alongside three SUMO-controlled background vehicles.

### 3. Coordination baselines and learning — baselines complete, training next

- Establish no-learning and independent-control baselines.
- Train shared PPO/IPPO before communication.
- Add hive-mind messages and compare no, ideal, and degraded communication.
- Evaluate on held-out, matched traffic seeds.

Exit gate: a measurable policy checkpoint with documented observation, action,
normalization, safety, and communication contracts.

Current evidence: the deterministic scripted policy completes both routes without
collisions, while the matched seeded-random policy times out both agents and incurs
substantially more safety-shield overrides. PPO/IPPO training has not started.

### 4. OpenUSD bridge and replay

- Implement the snapshot contract in `usd/TRAFFIC_SNAPSHOT.md`.
- Validate position, heading, units, timing, and stable vehicle prim identities.
- Export a deterministic replay layer before attempting live streaming.
- Keep static road/assets separate from dynamic transforms.

Exit gate: the recorded SUMO baseline can be scrubbed in an OpenUSD stage and its
first and last poses match the source telemetry.

### 5. Isaac Sim RTX lidar playback — required

- Select and pin one Isaac Sim version, compatible Python runtime, NVIDIA driver,
  GPU requirements, and lidar implementation.
- Prove a minimal scene can render one timestamped lidar frame before connecting
  traffic.
- Load the OpenUSD road and vehicle assets, mount lidar to the ego vehicle, and
  replay SUMO-owned poses.
- Record sensor pose, calibration, scan cadence, frame number, simulation timestamp,
  and source scenario for every output.
- Keep lidar cadence independent from the SUMO control interval when needed, using
  documented interpolation.

Exit gate: the same recorded traffic replay produces timestamped lidar output with
stable calibration and no unresolved USD assets.

### 6. Sensor-aware and physics validation

- Decide whether lidar is used only for evaluation/perception or becomes a policy
  observation. Do not change the policy input silently.
- If perception is required, define a fixed-size representation such as tracked
  objects, occupancy features, or a bounded point-cloud encoder.
- Validate representative scenarios in Isaac Lab with lidar noise, occlusion,
  lighting, friction, actuator delay, and communication degradation.
- Introduce hybrid physics only after playback/sensor mode is stable.

Exit gate: matched evaluation against SUMO baselines, with safety, completion,
smoothness, sensing cost, and synchronization drift reported.

## Immediate implementation order

1. Build the PettingZoo wrapper around the tested TraCI backend.
2. Define and unit-test the traffic snapshot and coordinate conversion contract.
3. Verify the target Isaac Sim installation and run a one-frame RTX lidar smoke test.
4. Export the single-vehicle SUMO baseline as the first OpenUSD replay fixture.
