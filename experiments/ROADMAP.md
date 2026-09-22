# Hivemind Traffic implementation roadmap

## Committed architecture

Isaac Sim with RTX lidar is a required project stage, not an optional visualization
extra. SUMO remains the authority for scalable traffic behavior and early policy
training. Isaac Sim provides the 3D scene, lidar returns, and later physical
validation for selected vehicles.

User decision recorded September 22, 2026: ovstage + ovrtx is also a required
target for rendering and sensor workflows. Isaac Sim remains the interactive
lidar/validation runtime. Both consume the same portable USD scene and recorded
SUMO motion; each has its own runtime and sensor adapter. Using RTX inside Isaac
Sim does not constitute integrating the standalone ovrtx library.

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

### 3. Coordination baselines and learning — no-communication baseline complete

- Establish no-learning and independent-control baselines.
- Train shared PPO/IPPO before communication.
- Add hive-mind messages and compare no, ideal, and degraded communication.
- Evaluate on held-out, matched traffic seeds.

Exit gate: a measurable policy checkpoint with documented observation, action,
normalization, safety, and communication contracts.

Current evidence: the deterministic scripted policy completes both routes without
collisions, while the matched seeded-random policy times out both agents. The
refined shared PPO/no-communication policy completes both agents on all three held-
out seeds with no collisions. It averages 37 safety overrides, versus 0 for the
scripted policy and 498 for random, and finishes in about 245 steps versus 269 for
scripted. Communication experiments may now start, while override rate remains an
explicit metric to improve rather than being hidden by the safety shield.

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

### 6. ovstage + ovrtx rendering — fixed-timestamp RGB milestone verified

- Check and pin compatible runtime, Python, driver, and GPU versions in a separate
  environment before integrating traffic. Start with NVIDIA's single-PNG example.
- Separate reusable scene export from Isaac Sim startup. Keep the portable root
  free of Isaac-specific lidar prims, runtime writers, and UI configuration.
- Load the existing road, referenced vehicle assets, and recorded motion through
  ovstage; use ovrtx for rendering. Validate matching poses at selected timestamps.
- Add an ovrtx sensor adapter after the image/replay test. Verify actual lidar
  configuration support and timestamps; do not assume Isaac sensor prims or
  annotators transfer unchanged.

Exit gate: timestamped images from the same replay without running Isaac Sim or
SUMO, followed by verified sensor output with a recorded configuration.

Current evidence: separate Python environment, NVIDIA minimal image, and traffic
images at 2 and 15 seconds through ovstage + ovrtx. Shared export checks first/last
positions and headings for all five vehicles. Rendering preserves the shared USD
layers. Interactive ovrtx playback and ovrtx lidar remain future work.

### 7. Sensor-aware and physics validation

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

1. Preserve the working Isaac replay/lidar milestone: five recorded vehicles,
   endpoint position checks, moving sensor returns, and tested GUI point display.
   Geometric and timing validation of moving scans remains outstanding.
2. Shared USD export and isolated Isaac sensor/UI authoring are implemented.
3. Minimal ovstage + ovrtx PNG rendering and shared traffic snapshots are verified.
4. Controlled straight-motion lidar validation passes. Curved traffic replay
   checks are now implemented; timing/coverage pass in the first full run, but
   strict range/identity acceptance fails. Resolve the dynamic identity-map
   limitation and investigate silhouette returns before using lidar as policy
   observations. See `documentation/replay-lidar-validation.md` for evidence.
   Five short diagnostic captures now reject simple identity-delta, visibility
   preflight, and uninstancing fixes; disabling azimuth noise alone does not remove
   range outliers. The offline audit preserves confusion matrices and boundary
   distances. See `documentation/lidar-diagnostics.md`. Next isolate renderer ID
   lifecycle and return semantics before repeating full-route acceptance.
   Improve shared road/vehicle assets without breaking the validation fixtures.
5. Resume communication experiments against the frozen no-communication baseline.

Reference: https://github.com/NVIDIA-Omniverse/ovrtx (official setup and ovstage
integration guidance, checked September 22, 2026).
