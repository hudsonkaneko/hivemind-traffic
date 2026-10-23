# CIRCLES physical-car smoke

October 22, 2026. **Basic longitudinal actuation works; reliable following does
not yet.** Seven public ONNX exports drove the existing native PhysX car through
a small pedal adapter. Six avoided contact but timed out well short of the
required stopping window. The faster interpolated export hit the obstacle.
Strict stopping-task success was **0/7**, distinct from **6/7** basic actuation
smoke passes. No training ran and the default showcase was not changed.

## Scope and measured results

One existing `LoopHazardLabEnv`, radius-500 m route, seed 31001, unperturbed reset,
stationary native 1 m obstacle, 35 simulated seconds per case, two seconds of
initial brake settling. Each model received the same reset and obstacle location.
This isolated physical-car fixture contains no background traffic or moving
leader. Inputs use simulator state, not LiDAR. Existing `PathFollower` supplies
steering; only its steering component is submitted during driving.

| Export alias | Distance (m) | Peak speed (m/s) | Final speed (m/s) | Native contacts | Outcome |
| --- | ---: | ---: | ---: | ---: | --- |
| cra | 9.143 | 0.590 | 0.071 | 0 | Timed out, about 13.37 m remaining gap |
| rl_decel3 | 9.143 | 0.590 | 0.071 | 0 | Same conservative behavior |
| rl_ttc4 | 9.143 | 0.590 | 0.071 | 0 | Same conservative behavior |
| rll_ttc7 | 9.143 | 0.590 | 0.071 | 0 | Same conservative behavior |
| rll_ttc6p5 | 9.144 | 0.590 | 0.071 | 0 | Same conservative behavior |
| cra_interp | 22.509 | 2.989 | 0.002 | 2 | Rigid contact with obstacle |
| max80mph | 11.733 | 0.768 | 0.113 | 0 | Timed out, about 10.79 m remaining gap |

All seven issued both positive throttle and brake requests after settling. Lane
error stayed below 0.006 m under the existing steering controller. Small final
speed is not proof of a correct policy stop: the conservative models stop/crawl
far from the target and vehicle drag interacts with the simple pedal mapping.
The contact case is retained, not removed from the totals. No safety controller
overrode obstacle approach; the existing native contact/physics guards still
terminate failures and brake after a latched failure.

Predeclared basic actuation pass: progress >5 m, peak speed >0.5 m/s, positive
throttle, zero rigid contacts/physics-guard failures, and lateral error <0.5 m.
This deliberately limited gate is not the task's stopping or autonomy gate.

## Model contract and adapter

Source: [CIRCLES algos-stack](https://github.com/CIRCLES-consortium/algos-stack/tree/3b3ee3b91afdae93075c8b90334d65748cd9ff74),
MIT license, pinned revision `3b3ee3b91afdae93075c8b90334d65748cd9ff74`.
Exact paths are in `scripts/fetch_circles_controllers.py`; individual model SHA-256
values and compact results are in [the evidence record](evidence/circles-control-v01.json).
Weights, upstream license, README, wrapper source and graph dumps remain local
under ignored `outputs/circles_control/models/<revision>/`.

- Input float32 `[1,3]`: current ego forward speed (m/s), measured obstacle forward
  speed (zero here), and chassis-frame bumper gap (m). No normalization. Gap uses
  the measured obstacle's projected footprint and the ego's 2.4 m half-length.
- Output: one scalar, interpreted as acceleration (m/s²). **Only CRA explicitly
  documents that contract**, calling it a Time-Headway Follower-Stopper. Other
  exports share the three-input wrapper and compatible graphs, but their use here
  is an experimental acceleration interpretation, not a verified deployment API.
- CRA matched the upstream examples: 0.007675, -2.115352 and 1.500000 m/s².
  All seven passed ONNX structural checks and finite-output spot checks.
- Inference 20 Hz, controls 60 Hz, physics 120 Hz. Desired speed is
  `clamp(speed + 0.6 * output, 0, 3)`; throttle is `clamp(0.7 * speed_error, 0, 1)`;
  brake is `clamp(-0.35 * speed_error, 0, 1)`. Steering remains local. These gains
  are an initial integration choice, not calibrated acceleration tracking.

These are inference exports, not resumable RSL-RL checkpoints. File names are
not proof that every export is RL or the exact final CIRCLES field-test model.

## Reproduce and evidence

From the repository root, in the existing traffic Python environment:

```text
python scripts/fetch_circles_controllers.py
python -m pip install --target runtimes/circles_onnx312 --python-version 3.12 --no-deps --only-binary=:all: onnxruntime==1.23.2
python -m unittest discover -s tests -p test_circles_control.py -v
python experiments/probe_circles_control.py --max-seconds 420
```

The current native runtime uses Python 3.12; the ONNX Runtime wheel is isolated
inside this checkout. No packages were installed into external Isaac runtimes.
An additional local Python 3.11 ONNX 1.19.1 / ONNX Runtime 1.23.2 inspection target
was used for graph validation; it is not needed for the native rerun command.
The existing configured Isaac launcher and vehicle assets are prerequisites.
Models are downloaded explicitly; the test itself performs no network access.

Confirmed package: `outputs/circles_control/v01-20261023T021853Z-3dd776cb`;
runtime logs: `logs/circles_control/v01-20261023T021853Z-3dd776cb/runtime.log`.
Seven episodes completed with unchanged source/model identities, successful
contact/viewport cleanup, exit 0, no remaining Kit process and no unexpected
runtime errors. Two known startup schema errors are recorded, not hidden.
Total process time: 76.265 seconds. Two targeted adapter tests passed.

Earlier package `v01-20261023T021603Z-1e35effc` is preserved. Its seven physical
outcomes matched, but its strict harness failed because concatenating separate
stdout/stderr logs placed startup errors after readiness. The corrected runner
merges streams during execution and uses unbuffered output. The complete repeat
passed the evidence check; the failed original is not relabeled as passed.
An initial exact floating-point assertion was also corrected to a tolerance;
the adapter arithmetic did not change.

No rendered screenshots were captured: this was a headless control smoke with
unchanged scene appearance. Raw transitions, measured native states, requests,
applied controls, contacts, GPU samples and source snapshots are retained locally.

Next work would calibrate acceleration tracking and confirm each model's contract,
then test a moving leader and full background traffic. This result does not
establish merging, highway-speed behavior, sensing, or random-object avoidance.
The Google engineering guide's lane-task tab and its Markdown mirror record the
same limited result; native content was read back, not visually paginated.
