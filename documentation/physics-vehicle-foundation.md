# Physics-driven vehicle foundation

Updated: October 3, 2026. **Verified low-speed compatibility spike; full migration
still in progress.** Existing SUMO-motion demos and Leatherback results are unchanged.

## What changed and why

A separate one-car experiment now runs **without SUMO**. An explicit scripted
driver issues steering, throttle and brake requests; an adapter converts them
to road-wheel angles and wheel torques; Isaac/PhysX determines the resulting
motion. No per-step chassis pose or velocity command is used to move the car.

This tests the missing link between a driver decision and physical actuation
before adding road following, LiDAR, communication or a larger fleet. It is not
PPO training, a perception controller, or the finished hybrid traffic experiment.

| Layer | Implementation | Responsibility |
| --- | --- | --- |
| Driver command | `traffic/driver_control.py` | Identity, sequence, expiry, finite values, limits and fallback |
| Vehicle adapter | `traffic/physx_vehicle.py` | Ackermann wheel angles and per-wheel torque writes; physical state readback |
| Scripted test driver | `scripts/probe_physics_vehicle.py` | Speed-error feedback, fixed left turn, coast, brake and command dropout phases |
| Bounded launcher | `experiments/verify_physics_vehicle.py` | Runtime discovery, unique run, deadline, GPU headroom, source stability |
| Test configuration | `experiments/physics-vehicle-smoke.json` | Rates, torques, phase durations and predeclared acceptance thresholds |
| Evidence export | `experiments/report_physics_vehicle.py` | Verify captured hashes and generate a small shareable result |

## Control contract

Commands identify `episode_id`, `vehicle_id`, increasing `sequence`, `issued_tick`
and exclusive `expires_tick`. Validity is `issued_tick <= tick < expires_tick`.
This runner uses 120 Hz physics, 60 Hz new commands, and a 12-tick (100 ms)
command lifetime. No new delivery holds the last valid command until expiry;
an exact duplicate does not renew its lifetime. Wrong identity, malformed,
future, expired or replayed commands clear the hold and enter fallback.

- Steering: bicycle-equivalent road-wheel angle in radians; positive turns left.
  The default limit is ±0.5 rad with 0.5 rad/s slew limiting. The smoke turn asks
  for 0.2 rad. These are low-speed limits, not speed-adaptive highway limits.
- Throttle/brake: unitless requests in [0, 1]. Any brake request suppresses drive.
  Forward-only drive maps to at most 700 N·m on each front wheel; brakes map to
  at most 1,500 N·m on each of four wheels. Brake torque is not a guaranteed
  deceleration: traction and contact determine what actually happens.
- Fallback: zero drive, full configured brake, steering rate-limited toward zero.
  A broken local clock raises an error and the runner must stop, not continue
  applying a stale action. The local runner chooses the TTL; an externally
  supplied command transport still needs an enforced maximum lifetime/age.
- Coordinates: right-handed X forward, Y left, Z up, meters and simulated seconds.
  PhysX readback quaternions are **xyzw**. Position refers to the chassis prim
  origin, with center of mass 0.25 m below it—not SUMO's front bumper.

The existing SUMO state contract v1 has **not** been silently reinterpreted.
A versioned cross-backend state contract and conversion tests remain necessary.

## Vehicle and dependencies

The installed `omni.physx.vehicle` Factory sample authors a box chassis with four
wheels, tire friction and raycast suspension. It is reused by import, not copied
into the repository or modified in the runtime installation. Its source hash and
exported initial USD are saved with the run. This sample helper is a version-sensitive
prototype dependency; replace or pin it before making a reusable production asset.

Measured/authored setup: 1,800 kg; collision box 4.8 × 1.8 × 1.4 m; wheelbase
3.2 m; track 1.6 m; wheel radius 0.35 m. Inertia uses the sample's separate
4.8 × 1.8 × 1.0 m mass box. Gravity is explicitly 9.81 m/s². Each suspension
uses 45,000 N/m spring stiffness, 4,500 N·s/m damping and 0.2 m travel. Tire/tarmac
friction is 0.75; rigid contact static/dynamic friction is 0.9/0.7.

The collision ground is infinite, but the sample's visible ground spans only
30 × 30 m. The probe is headless and does not render a traffic demo. These values
are an exploratory full-size surrogate, not a measured real car or certified
highway model. The 0.32 m-wheelbase Leatherback gains/policy were not scaled up.

Validated installation: Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl`, bundled
Python 3.12.13, RTX 5070, driver 610.74. No dependencies were installed and no
external runtime files were changed. CPU tests use the isolated traffic Python.

NVIDIA's [vehicle dynamics guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/vehicles/vehicles.html)
describes the separate tire, suspension and drive components. The actual installed
helper and APIs, rather than assumptions from another release, determined this probe.

## Verification and retained attempts

The [generated result](physics-vehicle-results.json) verifies every recorded artifact
hash and includes exact source hashes, resolved configuration and runtime metadata.
Both runs below remain local; none was overwritten or excluded from the record.

| Run | Result and interpretation |
| --- | --- |
| `20261003T092506Z-aa86d952` | Initial two fresh-stage episodes passed; observed USD stage reference-count warning |
| `20261003T092703Z-7806d7c0` | Two episodes passed after releasing Python vehicle/stage references before rebuild; warning persists, so repeated-reset memory health is not closed |

Each run contains two 28-s simulated episodes: settle, straight, brake, hold,
turn, coast, then command dropout. The **second run** is the published source
checkpoint. Results in its two episodes:

| Check | Gate | Observed |
| --- | --- | --- |
| Straight speed | 2.5–3.5 m/s, positive X progress ≥15 m | 2.99127 m/s; passed |
| Straight lateral drift | ≤0.15 m | <0.000001 m |
| Brake | Entry speed ≥2.5 m/s, path to ≤0.05 m/s within 4 m | Entry 2.99127 m/s; 0.67187 m |
| Stationary hold | ≤0.05 m drift and ≤0.05 m/s | 0.000157 m drift; passed |
| Left turn | Yaw increase ≥0.3 rad | 1.33487 rad |
| Stability/contact | Upright axis Z ≥0.95, root height 0.5–1.5 m, all wheels grounded ≥99% after settling | Upright minimum 0.999931; 100% grounded; passed |
| Command expiry | Final speed ≤0.05 m/s with zero throttle/full brake | 0.000226 m/s; passed |
| Same-version repeat | Position/speed differences ≤0.001 m and ≤0.001 m/s | Both maximum differences 0 in this fixture |

All **331 automated tests passed** at this checkpoint, including 77 new command
tests and 10 new acceptance-gate tests. Equal traces here are not a promise of
bitwise RTX/PhysX reproducibility across hardware, versions or different scenes.

The second run took about 31.97 s through the measured probe path including
startup; this is not a rendered real-time benchmark. Sampled whole-GPU peak was
4,070 MiB including desktop applications; process peak working set was about
5.08 GiB. No full-fleet memory limit can be inferred from these short runs.
The USD reference-count warning remains a tracked reset/lifetime investigation,
even though both stages passed motion gates and the process exited normally.

## Portable reproduction

Start in the repository root with the traffic environment activated. Configure
`isaac_root` in ignored `hivemind.local.json`, or `ISAAC_SIM_PATH`, as described in
[portable demos](portable-demos.md). Do not install traffic packages in Isaac Python.

```text
python -m experiments.verify_physics_vehicle --check
python -m experiments.verify_physics_vehicle
python -m json.tool documentation/physics-vehicle-results.json
python -m pytest -q
```

`--check` resolves settings only. The second command runs **headlessly**, exits
nonzero on failure, and prints a new `outputs/physics_vehicle/<run-id>` location.
It has a 180-s process deadline and stops if sampled whole-GPU memory reaches 90%.
Do not run competing Isaac/GPU experiments during a measurement. Raw evidence is
ignored by Git and is not included in a clone. To inspect a fresh local run:

```text
python -m experiments.report_physics_vehicle outputs/physics_vehicle/YOUR_RUN_ID
```

Replace `YOUR_RUN_ID` with the printed folder. The exported source/configuration
and working-tree record explain that these runs were made during development
from baseline commit `76c9c19`, not from an artificially clean checkout.

For the existing visible traffic demo (still SUMO movement), use:

```text
python scripts/demo_cooperative_lidar.py
```

## What remains before advancing

1. Close repeated reset/lifetime validation, including the USD warning and ten
   resets; exercise 1 and 6 m/s, right turns, turn-radius tracking, coast behavior,
   slope/friction changes and longer stationary holds with explicit gates.
2. Test a one-environment Isaac Lab action/reset adapter. Native PhysX vehicles
   are not Leatherback articulations; shared stepping APIs alone do not prove
   training, vectorized reset or cloning support. Do not begin training yet.
3. Add versioned cross-backend state conversion, shared lanes, path tracking and
   speed-dependent steering limits before physical LiDAR and obstacle avoidance.

Then proceed through the [hybrid roadmap](hybrid-roadmap.md). The two-way SUMO
bridge, mixed ratios, communication, physical-car sensors and learned coordination
are still planned. Straight open-loop stability is not lane-keeping validation.

The Isaac validation skill drove the single-authority boundary and scripted-first
test; the reproducible-experiment skill drove predeclared gates, retained attempts,
source snapshots and explicit limits on performance claims.
