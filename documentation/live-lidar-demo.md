# Live lidar driving: first closed-loop milestone

This demo connects live traffic state to an RTX lidar sensor, then uses the measured
clearance to choose the next speed command. It is not a recording being replayed.
The first mode stops one car before a stationary obstacle. The second follows a
scripted lead car and stops when that car brakes.

Verified outcomes and retained failures: [validation results](live-lidar-results.md).

For the newer pass-and-return maneuver, see [stationary obstacle avoidance](lidar-avoidance.md).

## Launch

Run these single-line commands from the repository root. They work in PowerShell
and Command Prompt when `python` is available:

```text
python scripts/demo_live_lidar.py --check
python scripts/demo_live_lidar.py
python scripts/demo_live_lidar.py --mode follow --speed 6 --gap 30
```

On this machine, the project root is `C:\Users\hudso\Documents\highwaysim`.
If `python` is not on PATH, use `.venv\Scripts\python.exe` instead. The launcher
uses standard-library Python only and starts Isaac's own Python for the simulation.
The window closes after 25 simulated traffic seconds. The console prints the
output directory and pass/fail result. Add `--headless` for no visible window.

Another machine needs compatible NVIDIA hardware, Isaac Sim, and SUMO installed.
The code has no hard-coded username or checkout path. It checks `SUMO_BINARY`,
PATH, `SUMO_HOME`, and standard Windows SUMO locations. Set `ISAAC_SIM_PATH` to
the Isaac installation if it is not at the Windows fallback `C:\isaacsim`.
Runtime-version compatibility is still necessary; portable paths do not make
different Isaac releases automatically interchangeable.

## Six milestones and their boundaries

| Milestone | Implementation / evidence |
|---|---|
| 1. Validate sensor geometry | Compare raw-scan forward distance with the known obstacle/lead face, after the control decision; maximum error gate 0.25 m |
| 2. Live synchronization | Freeze each SUMO snapshot while USD is updated and a fresh RTX scan settles; advance SUMO exactly once by 0.1 s |
| 3. Usable lidar observations | Filter valid finite returns to a central forward corridor; reject ground and self returns; check scan health and timestamps |
| 4. Closed-loop control | Bounded deterministic speed/braking controller using only lidar clearance and ego speed |
| 5. One-car stopping | Predeclared speed/gap pairs 4 m/s–20 m, 8–40, and 12–65; obstacle exists only in USD |
| 6. Two-car following | Scripted lead travels at 4 m/s, requests braking at traffic time 8 s; follower measures its rear face with lidar |

## How the components interact

```text
SUMO vehicle state → USD transforms → RTX lidar scan
        ↑                                  ↓
  TraCI speed command ← controller ← forward clearance
```

**SUMO** owns positions, road-following, and traffic safety rules. **TraCI** is the
Python command/query connection to SUMO. **OpenUSD** represents road and vehicle
geometry. **Isaac Sim / RTX** renders the sensor returns. The **controller** converts
distance into target speed. **PPO and PettingZoo are not used in this demo**: first
we establish a measurable sensor-control baseline before adding learning.

The controller has no access to target IDs, semantic labels, obstacle coordinates,
lead-car state, or an omniscient scene map. Truth data is computed afterward for
evaluation. Ego speed is supplied by SUMO as simulated vehicle odometry. This is
therefore lidar-based obstacle response, not a claim that every input is lidar.

SUMO retains default speed safety (`speedMode=31`). Discretionary lane changing
is disabled while collision-avoidance protections remain. Every requested and
realized ego speed is logged; a deviation above 0.15 m/s fails the no-intervention
gate. The static obstacle is absent from SUMO, so SUMO cannot brake for it itself.

## Sensor and controller choices

- Sensor: `Example_Rotary`, front bumper, 1 m above ground, metres and Z-up.
- Straight-road mapping: SUMO x/y equals USD x/y; SUMO heading must be 90 degrees.
- Corridor: 0.5–80 m ahead, within 0.85 m of centre, world height 0.3–1.5 m.
- Health: at least 1,000 samples and 20 valid environmental returns. Healthy scans
  without corridor returns report 80 m. This is a fixture-specific heuristic,
  not guaranteed detection of all possible obstacles.
- Desired speed is the minimum of cruise speed, a stopping-distance bound, and a
  distance/time-gap bound. A 5 m stopping margin is used. Acceleration is limited
  to 2 m/s² and deceleration to 3 m/s².
- A deliberately stale observation commands braking. Acquisition timeout freezes
  SUMO and ends the run as failed; the car is not advanced using an old scan.

## Timing: live, but not real time

This first implementation uses **sample-and-hold**: SUMO remains frozen during
each sensor acquisition. RTX's scan clock advances separately from SUMO's traffic
clock. Both timestamps are saved; they must not be treated as the same time base.
Vehicle poses are held through settling and scan collection to avoid mixing old
and new poses in a rotary scan. It is intentionally slower than wall-clock real
time. It does not validate motion distortion or continuously moving sensor scans.

## Reproduce and audit

For unit tests and offline analysis, create a local virtual environment once:

```text
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r experiments/live-lidar-requirements.txt
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe experiments/run_live_lidar_suite.py --gui-last
```

The seven-case GPU suite covers three initial speeds, a repeated following case,
a second SUMO seed, and stale-observation fault injection. This simple fixture
has no randomized demand, so changing the seed is a reproducibility check, not
evidence of generalization. See `live-lidar-plan.md` for the predeclared gates.

Each run writes a unique directory under `outputs/live_lidar/` containing raw
compressed scans, telemetry, a USD scene, configuration/version/source manifest,
source copies, a working-tree diff, hashes, and a summary. GUI runs also attempt
a viewport capture. Raw evidence can occupy hundreds of MB per run and stays
out of Git. Logs belong under `logs/`.

```text
.venv\Scripts\python.exe scripts/analyze_live_lidar.py outputs/live_lidar/YOUR_RUN_ID
```

The audit recomputes clearance and controller decisions from the saved scans,
checks clock order and state continuity, verifies file hashes, and regenerates
acceptance metrics. `YOUR_RUN_ID` must be replaced with the actual printed run ID.

## What this does not establish

This is a small straight-road sensor/control demonstration, not a road-ready
autonomous-driving system. It does not implement lidar lane detection, steering,
PhysX vehicle dynamics, overtaking, two independent lidar agents, learned perception,
communication, or PPO training. The lead car is scripted. Earlier label-registration
and curved-replay issues are not declared fixed by these tests. The ovrtx pipeline
is not changed; this milestone uses Isaac's Kit runtime.

Vehicles are simple 5 m box proxies rather than detailed car assets. They have no
wheel or steering physics in this fixture. The sensor is mounted at the front
bumper to make distance evaluation unambiguous.

The green point-cloud overlay shows raw returns, including self/road hits.
The controller's forward corridor is filtered separately, so seeing green points
on the ego vehicle is not evidence that it is braking for its own body.

The native sensor profile has angular/range measurement error settings. A stricter
1 mm repeated-clearance check failed at 1.46 mm even though both following runs
met every driving gate. This failure is retained, not relabelled a pass.
An explicitly separate `--ideal-sensor` diagnostic disables configured angular
standard deviations and range accuracy noise. It does not make the native noisy
profile deterministic or validate robustness to other noise distributions.

NVIDIA documents the angular error and range-accuracy fields in its
[lidar configuration reference](https://docs.isaacsim.omniverse.nvidia.com/2023.1.1/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar/lidar_config.html).
The actual values used here are captured from the installed runtime's USD sensor
attributes in each manifest; the reference explains the fields, not this experiment's result.

```text
.venv\Scripts\python.exe experiments/run_live_lidar_suite.py --diagnostics --gui-last
.venv\Scripts\python.exe scripts/report_live_lidar.py outputs/live_lidar/suite-YOUR_TIMESTAMP.json
```

The diagnostic batch repeats the ordinary stop and runs two ideal-sensor following
cases. The report re-audits every scan and compares repeated trajectories. The
original suite's overall report intentionally fails when its stricter repeat
gate fails, even if all individual driving cases pass.

Next development should tackle continuous-motion synchronization and robust
perception, then physical steering/dynamics or policy training against a fixed
observation contract. Keep those separate from proving this small baseline works.

## Troubleshooting and references

An exploratory run warned that multi-tick lidar needs motion BVH. The application
now enables `enable_motion_bvh=True`, following NVIDIA's
[RTX sensor instructions](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx.html).
GUI point-cloud registration enables the RTX nodes extension before constructing
the sensor. Runtime dependencies are read/executed, never patched.

SUMO command/safety meanings are documented in
[Change Vehicle State](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html).
Raw lidar coordinate/timestamp semantics are described in NVIDIA's
[GenericModelOutput reference](https://docs.isaacsim.omniverse.nvidia.com/latest/py/docs/source/generic_model_output/generic_model_output.html).

An unrestricted test discovery initially spent time scanning non-test project
directories. `pytest.ini` now restricts collection to `tests/` and keeps temporary
test artifacts in `outputs/`, using pytest's documented
[discovery configuration](https://pytest.org/en/stable/example/pythoncollection.html).

## Workspace and documentation provenance

The latest workspace instructions select `Documents/highwaysim`; earlier work
used `Documents/hivemind-traffic`. The older batch was stopped and its files left
intact. Only the new demo's source was brought into this canonical workspace;
existing waypoint and migration work was preserved. All new runs are created here.
This local document is the learning reference for the change. The Google Doc has
not been edited because the current instructions restrict documentation writes
to this project folder.
