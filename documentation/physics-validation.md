# Physical-car validation and Isaac Lab compatibility

Updated: October 3, 2026 (America/Los_Angeles).

This milestone extends the [first physical-car fixture](physics-vehicle-foundation.md).
The working SUMO/LiDAR demonstrations and their historical results are preserved.
The native PhysX vehicle remains an uncalibrated low-speed sample, not a validated
highway car. No dependencies were installed or external runtime files modified.

## Implementation and purpose

- [Lifecycle helper](../traffic/physics_session.py): explicit stage creation,
  attachment, fixed stepping, detach, context close and targeted cache cleanup.
  Kit updates occur outside active manual physics stepping. Each run records
  lifecycle events, not just whether a car reached its endpoint.
- [Reset fixture](../scripts/probe_vehicle_resets.py): ten fresh-stage episodes,
  recording trajectories before discarding them and sampling process memory.
  This avoids mistaking intentionally retained telemetry for an engine leak.
- [Dynamics assessor](../traffic/dynamics_validation.py): braking at 1/3/6 m/s,
  five-second holds, left/right steady turns, and unpowered coasting. The turn
  comparison uses the rear axle's path length divided by unwrapped yaw change,
  against the bicycle reference `wheelbase / tan(steering)`.
- [Lab environment](../environments/physics_vehicle_lab.py): a real Isaac Lab
  `DirectRLEnv` with one native vehicle. A `RigidObject` provides chassis reset
  access; it is **not** an articulated robot. Wheel torques and steering still
  command movement. Pose/velocity writes are restricted to episode reset, which
  also clears native wheel/suspension state.
- [Portable supervisor](../experiments/verify_vehicle_stage.py): preflight,
  process deadlines, sampled 90% whole-GPU-memory stop, unique evidence folders,
  captured source/configuration, failure preservation and post-run source checks.
- Lazy environment imports keep the Isaac-only task from importing SUMO or
  PettingZoo. Existing traffic environment exports remain available.

## Control and observation boundaries

Physics runs at 120 Hz and commands at 60 Hz. Actions are steering radians
(positive left, limited to +/-0.5), throttle and brake in `[0,1]`. Brake takes
priority over drive. The existing command gate limits steering slew and applies
fallback braking after the 12-tick/100 ms command lifetime expires.

Lab returns eleven **simulator-state** values: position XYZ in meters, yaw in
radians, velocity XYZ in m/s, upright-Z, applied steering, throttle and brake.
These are privileged state, not LiDAR observations. The probe returns zero reward
deliberately: it checks the environment interface and does not define an RL
objective or train a policy. One environment passing says nothing about cloning,
vectorized training, multi-agent collisions or sensor synchronization.

## Reset warning investigation

The original warning appeared when replacing a USD stage. Inspection found that
NVIDIA's authoring wizard can retain a stage reference. NVIDIA documents the
[vehicle authoring extension separately from the core physics runtime](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/ux/source/omni.physx.vehicle/docs/index.html).
That was a plausible hypothesis, **not a confirmed sole cause**.

The tools-enabled two-episode control reproduced one warning. Disabling the
authoring extension still produced one warning across ten episodes. Processing a
stopped application update after opening the stage also failed to remove it.
The exact remaining reference owner is unresolved. No vendor/private state was
patched and warning output was not suppressed.

All ten reset episodes produced identical final positions and speeds. Every
post-close stage cache was empty. Post-warmup peak RSS above the second-episode
baseline was 19.00 MiB in the first ten-reset attempt and 0.00 MiB in the second,
within the predeclared provisional 256 MiB budget. Zero means the sampled RSS
never exceeded that baseline, **not zero allocations or proven leak freedom**.

The reset supervisor therefore deliberately reports **failed overall**, even
though physical/reset/cache/memory subchecks passed: the zero-reference-warning
gate remains unmet. Both attempts remain recorded. Disabling authoring also
emits a plugin-interface-pointer warning; it is not presented as a fix. This
helper is a diagnostic headless path, not a modification of the existing GUI demo.

## Verified Isaac Lab subset

Run `20261004T001311Z-59439425` passed three eight-second drive/brake episodes.
Each advanced about 9.46239 m during its drive phase and ended at approximately
0.00000481 m/s. Recorded position and speed traces matched exactly; tolerances
were 0.001 m and 0.001 m/s. Initial plus three automatic resets returned position,
linear speed and wheel rotation speed to the checked starting values.

The environment supplied finite `[1,11]` observations and accepted `[1,3]` actions;
episode timing and reset counts were checked. The run did not emit the USD
reference-count warning. Other runtime warnings are retained in its log,
including the installed TGS solver's velocity-noise warning. Two startup errors
from property/material UI extensions report missing `OmniPlaybackAPI` schema
handlers. They did not prevent the tested reset/step behavior, but runtime
startup/teardown cleanliness is not validated. The supervisor's declared Lab
gate does not require an error-free log; this pass is limited to its interface
and motion checks, not every runtime subsystem.

At the terminal step, recorded physical state is pre-reset while the returned
Lab observation is post-reset. Future training must explicitly handle terminal
observations instead of pairing that next-episode observation with the old
transition. Angular velocity and full suspension state were reset through APIs
but not independently recorded; only the specified chassis/wheel values were
checked. Generic vehicle metadata still labels general Lab validation false;
this narrow compatibility result does not validate every vehicle/Lab operation.

Recorded dependencies: Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl`,
Python 3.12.13, PyTorch `2.10.0+cu128`, clean installed Isaac Lab checkout
`0603cb1710dcda13087665f63abf9ac483b63c05`. The native vehicle model and Factory
source hash remain captured per run. No SUMO, LiDAR or training was involved.

## Dynamics measurements

Run `20261004T001358Z-d7d5d04e` completed all twelve cases (six scenarios, two
repeats), totaling 288 simulated seconds. Every dynamics and trajectory-repeat
subcheck passed. Overall status is **failed** only because the retained
scene-reference warning violates the separate zero-warning gate.

| Test | Measured response | Frozen acceptance |
| --- | --- | --- |
| Brake from 0.99709 m/s | 0.07536 m to speed <=0.05 m/s | <=1 m |
| Brake from 2.99127 m/s | 0.67187 m | <=4 m |
| Brake from 5.98243 m/s | 2.67475 m | <=8 m |
| Left and right turns, 3 m/s target, +/-0.2 rad | Rear-axle radius about 15.97414 m; 1.1912% error from bicycle reference | <=10% error; correct direction; >=1 rad yaw |
| Post-braking stationary holds | Five seconds each; all drift/speed checks passed | Drift <=0.05 m and speed <=0.05 m/s |
| Four-second unpowered coast | Speed fell from 2.99127 to 2.93892 m/s | Zero drive/brake/steering commands; finite stable state |

All post-settling samples had four grounded wheels. Combined tilt stayed within
the predeclared five-degree limit (measured maximum 0.727 degrees). Maximum hold
drift was about 1.055 mm. Aligned recorded positions, speeds and yaw
matched exactly between equivalent repeats on this setup (0.001-unit tolerance).
This is sample-model consistency, not real-vehicle calibration or universal
cross-machine determinism. Coasting does not validate real aerodynamic drag.

## Evidence, resources and retained failures

[Generated compact evidence](physics-validation-results.json) verifies artifact
and captured-source hashes for all five attempts, including failures:

| Attempt | Outcome |
| --- | --- |
| `vehicle_reset_control/20261004T000718Z-a18344b9` | Two-episode tools-enabled diagnostic passed its subchecks; one warning, explicitly exempt only in this control |
| `vehicle_resets/20261004T000757Z-f4c03ae5` | Ten reset subchecks passed; overall failed because disabling authoring did not eliminate warning |
| `vehicle_resets/20261004T001205Z-dbc29de5` | Ten reset subchecks passed; overall failed because event-draining did not eliminate warning |
| `vehicle_lab/20261004T001311Z-59439425` | Three Lab episodes passed; zero USD reference warnings |
| `vehicle_dynamics/20261004T001358Z-d7d5d04e` | Twelve dynamics cases passed; overall failed on one USD reference warning |

Raw artifacts remain under local ignored `outputs/`; GitHub contains the compact
summary and reproduction source, **not** the raw traces. Baseline commit was
`757961b` with captured uncommitted source snapshots; no captured source changed
during any run. The original physics-foundation result was not overwritten.

Lab took about 17.63 s total process time, with a roughly 2.37 GiB peak process
working set and 4,676 MiB sampled whole-GPU peak. Dynamics took about 167.26 s,
with a roughly 5.10 GiB peak working set and 5,696 MiB whole-GPU peak. The reset
attempts ranged up to 7,316 MiB whole-GPU use. Desktop and other application GPU
use is included; these are unpaced, headless tests, not sensor/GUI capacity or
real-time fleet guarantees. No 90% memory guard or process deadline was hit.

All 434 CPU tests passed at this checkpoint. They test software contracts and
assessors; the actual simulator evidence above is separate. Independent agents
recomputed all 34,560 dynamics ticks and checked 43 dynamics artifact hashes plus
10 source hashes, and checked 23 Lab artifact hashes plus 12 source hashes.

## Reproduce the numerical demonstrations

From the repository root, activate the configured traffic environment; configure
`isaac_root` in ignored `hivemind.local.json` or set `ISAAC_SIM_PATH`. The launcher
uses Isaac's separate Python. Paste each command as one line.

```text
python -m experiments.verify_vehicle_stage --stage lab --check
python -m experiments.verify_vehicle_stage --stage lab
python -m experiments.verify_vehicle_stage --stage dynamics
python -m experiments.verify_vehicle_stage --stage resets
```

These are **headless numerical tests**, not new GUI demonstrations. `--check`
validates the selected configuration and resolves the runtime but does not prove
simulation success. Resets and dynamics currently exit nonzero because of the retained warning
gate. Inspect `summary.json`, subchecks and `runtime.log` rather than calling that
an unexplained crash. Run only one Isaac job at a time on this workstation.

The tools-enabled diagnostic is also available as `--stage reset-control`; its
warning is expected and exempted only for that explicit diagnostic control.
Never treat that exemption as completion of the zero-warning gate.

## What follows

Next build shared straight/curved lane geometry and a scripted path-following
controller, first using declared simulator state. Separate behavior decisions,
planned path/speed and low-level steering/braking. Then mount physical-car LiDAR
and verify acquisition time, pose and frame alignment before using perception
for control. Keep the scene-reference investigation open alongside that work.

Training, the SUMO shadow bridge, two physical cars, mixed-driver ratios,
communication-failure comparisons, and GUI/endurance performance remain later
roadmap gates. None is implied by successful low-speed wheel control.
