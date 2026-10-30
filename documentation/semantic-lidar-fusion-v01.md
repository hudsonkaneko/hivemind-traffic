# Semantic mask and LiDAR fusion v01

October 30, 2026. Scope: implement calibrated semantic-mask projection onto
actual RTX LiDAR returns, validate static captures before moving traffic, and
retain screenshots and metrics. No model optimization or driving-policy change
is authorized or performed by this work.

## Measurement contract

The prepared v14 vehicle retains its front pinhole camera and five RTX scanners.
CameraFront is a 640×360 perspective camera with a 120-degree horizontal field
of view, not a fisheye projection. Intrinsics are read from the live USD camera
attributes. The USD camera-to-world matrix uses row vectors, -Z forward and +Y
up; world coordinates use metres and Z-up. LiDAR XYZ is already in WORLD space
and must not receive a second sensor-to-world transform.

The pure NumPy fusion module projects each successful finite return into the
image at its recorded capture pose and samples the integer semantic mask.
It rejects invalid returns, future/stale timestamps, points outside the clipping
range or image, sparse LiDAR depth occlusion, ambiguous semantic boundaries,
unknown labels and low supplied confidence. Rejected points remain explicitly
unassigned (-1), separate from valid background class 0. Every input point
retains its projected position, selected class, time difference and reason.

Current masks are native simulator ground truth. The implementation accepts
predicted class masks/confidence in the same calibrated interface, but no
segmentation model has been trained or validated here. RTX object IDs are
resolved to semantic ancestor labels in a separate evaluation writer. They
never determine fused labels. Rendered camera depth likewise stays evaluation
only and measures remaining camera-visibility errors.

## Protocol and acceptance

[`semantic_lidar_fusion_v01.json`](../configs/perception/semantic_lidar_fusion_v01.json)
declares eight frames per fixture, captured at 10 Hz after warm-up. A parked,
braked PhysX ego vehicle observes reusable scripted traffic and a stationary
obstacle. The static fixture must pass before 2 m/s and 10 m/s traffic cases
are attempted. Ego motion is not commanded; this is a sensing fixture, not
physical policy validation. Each capture runs in a fresh isolated Isaac process
with a 180-second wall budget and writes only inside the project.

Fusion uses a 50 ms maximum timestamp difference, a 2-pixel sparse depth cell,
0.15 m depth tolerance and 2-pixel semantic boundary margin. Future returns
more than 10 microseconds beyond image time are rejected. This is time gating,
not moving-object deskewing. The native camera uses a synchronous render-barrier
caller timestamp; its annotator does not guarantee an independent acquisition
clock. Moving tests must show changing RGB/mask buffers and increasing LiDAR
timestamps.

Declared gates: assigned-point class accuracy at least 97% static / 95% moving,
vehicle and obstacle accuracy at least 90% with at least 20 evaluated points
per class, at least four frames containing assigned vehicle returns, at least
20% coverage of all successful finite in-image points before rejection, and
at least 98% independent oracle coverage. Rejected points and class-specific
coverage are reported to prevent high accuracy from hiding low assignment.
Rendered-depth occlusion errors are reported separately. Negative controls
check stale timestamps and incorrect camera calibration.

Moving acceptance additionally requires each of the two traffic actors to have
at least 20 assigned returns across four frames, at least 90% correctly labeled
vehicle returns, and nonzero displacement from its captured composed USD pose.
Evaluation-only object IDs identify each actor by prim ancestry. This is an
independent support check, not learned tracking or a complete object-recall test.
The stale-time control is a required rejection gate; the incorrect-calibration
and rendered-depth comparisons are diagnostics rather than pass/fail gates.

## Reproducibility and artifacts

Run from the project root in the traffic environment:

```powershell
.\.venv-traffic\Scripts\python.exe -m experiments.verify_semantic_lidar_fusion --phase static
.\.venv-traffic\Scripts\python.exe -m experiments.verify_semantic_lidar_fusion --phase all
```

The launcher uses the configured external Isaac runtime and refuses to overlap
an active Isaac/Kit process. Prepared v14 assets are a local prerequisite; they
are not embedded in this feature's Git changes. The original sensor preparation
work, old captures and trained MAPPO actors remain intact. New code does not
import the uncommitted dataset-preparation tools.

Each unique `outputs/semantic-lidar-fusion/v01/` run saves source/config hashes,
logs, actual RGB and semantic images, evaluation-only depth, all raw returns and
object IDs, per-ray timestamps, per-frame camera calibration/transforms,
fused point clouds, rejection/coverage/accuracy metrics and rendered evidence.
Scene assets remain referenced; traffic motion is authored in a separate live
layer. Saved initial scenes retain composition. Runtime-owned `/Render` and
sensor graph namespaces are exceptions, not authored asset hierarchy changes.

Sparse LiDAR depth cannot resolve foreground visible only to the camera.
Moving-object deskew, inferred poses, camera model uncertainty, weather,
perception errors and integration into the MAPPO observation adapter remain
separate work. A clean simulator label projection does not establish those
capabilities or authorize training.

## Validation status

The initial implementation checkpoint passes 41 focused offline tests. Native
attempt `20261030T221925Z-f48d4323` stopped before physics because it incorrectly
waited for a UI viewport in headless mode. That dependency was removed; sensor
readiness now depends on actual image and fresh LiDAR packets. The failed
package and log are preserved. Its saved scene independently reopened with
183 prims, 15 dependency layers, no composition errors and no missing assets.
Static and moving capture acceptance is pending the corrected native run.
