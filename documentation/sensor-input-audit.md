# Sensor input audit — first perception milestone

Audit date: October 25, 2026 (America/Los_Angeles). Baseline commit:
`6b14eb0ccfea1c7970ce920ccc1a5422303a635b`.

This is a source-code audit of the existing actor inputs and sensor/control
boundaries. It does not certify that the new camera, localization, or perception
pipeline has passed native runtime validation. Implementation evidence belongs
in the accompanying milestone report. No new policy training is authorized by
this audit, and existing checkpoint contracts must remain unchanged.

A real vehicle can estimate many of these quantities. The privileged shortcut is
reading exact simulator state, perfect identities, or evaluator bookkeeping.
Simulator truth remains appropriate for generating simulated measurements,
training labels, rewards, evaluation and emergency experiment termination; it
must not silently reach a controller marketed as sensor-driven.

## SUMO MAPPO: 27 actor features

Sources: [feature contract and observation/filter builders](../environments/sumo_mappo_env.py)
(`observation_features`, `_observations`, `_neighbor`, `_safety_filter`, `_state_for`)
and [TraCI backend](../traffic/mappo_sumo.py). The backend subscribes to exact
speed, acceleration, lane index/position, distance, length and allowed speed for
all active cars. Neighbor selection uses those values with a 120 m range gate,
not visibility, sensor occlusion, detection probability or measured confidence.
Indices below are zero-based.

| Index | Exact feature(s) | Current source | Migration decision |
| --- | --- | --- | --- |
| 0 | `speed_over_cap` | TraCI speed / 26.8224 | Replace source with estimated ego speed; retain semantic quantity. |
| 1 | `acceleration_over_4p5` | TraCI acceleration / 4.5 | Replace with IMU/odometry estimate with bias, noise and timestamps. |
| 2 | `lane_index_over_2` | Exact lane index / 2 | Derive from map localization and lane perception; include uncertainty. |
| 3 | `remaining_route_fraction` | 1 − exact lane position / 5000 | Derive from localization and route; navigation information is allowed. |
| 4 | `speed_limit_over_cap` | Exact allowed speed / 26.8224 | Keep an explicit map/sign-derived speed-limit input. |
| 5 | `previous_requested_acceleration_over_4p5` | Controller's last request / 4.5 | Keep command history; do not present it as measured acceleration. |
| 6–8 | `own_front_gap_over_120`, `own_front_relative_speed_over_cap`, `own_front_present` | Exact neighbor bumper gap, relative speed, existence | Replace with estimated tracks and visibility/validity. |
| 9–11 | `own_rear_gap_over_120`, `own_rear_relative_speed_over_cap`, `own_rear_present` | Same, own lane behind | Requires rear coverage; front camera alone cannot supply this. |
| 12–14 | `left_front_gap_over_120`, `left_front_relative_speed_over_cap`, `left_front_present` | Same, left lane ahead | Replace with tracks and lane association. |
| 15–17 | `left_rear_gap_over_120`, `left_rear_relative_speed_over_cap`, `left_rear_present` | Same, left lane behind | Requires side/rear coverage. |
| 18–20 | `right_front_gap_over_120`, `right_front_relative_speed_over_cap`, `right_front_present` | Same, right lane ahead | Replace with tracks and lane association. |
| 21–23 | `right_rear_gap_over_120`, `right_rear_relative_speed_over_cap`, `right_rear_present` | Same, right lane behind | Requires side/rear coverage. |
| 24–25 | `left_lane_exists`, `right_lane_exists` | Exact lane index within fixed three-lane road | Derive from map/perception, preserve explicit unknown status. |
| 26 | `elapsed_fraction` | Episode step / horizon | Remove from a new deployed sensor actor; keep evaluator-side. |

The neighbor `present=0` currently means the simulator found no neighbor within
range. A sensor miss is not equivalent to a clear road. Do not zero-fill an
unobserved rear sector and claim compatibility with this contract.

The centralized critic receives 112 values: four stable slots of 27 actor
features plus an alive flag. The critic network also appends a four-element
agent one-hot. These joint observations, alive masks and agent conditioning are
training-only; the exported local actor must not acquire this critic pathway.

Safety leakage is separate from actor leakage: `_safety_filter` uses exact ego
speed and `_neighbor` gap, and the backend retains SUMO `speedMode=31` safety
checks (`laneChangeMode=512`). An actor fed noisy observations can still appear
collision-free because simulator-assisted guards rescue it. Report requested,
filtered and realized actions separately; measure interventions. Do not disable
existing guards merely to relabel this baseline as sensor-only.

## Physical lane task: 22 features

Source: [lane contract](../environments/lane_following_contract.py), especially
`OBSERVATION_NAMES`, `_relative` and `_observation`; the native adapter is
[lane_following_lab.py](../environments/lane_following_lab.py). It explicitly
labels observations `privileged_simulator_state_and_known_map`. These values
are physical units (except trigonometric/flag/fraction fields), not SUMO's
normalized vector.

| Index | Exact feature(s) | Current source | Migration decision |
| --- | --- | --- | --- |
| 0–2 | `progress_m`, `remaining_m`, `lateral_error_m` | Exact body position projected onto known route | Replace with estimated localization/map projection and lane perception. |
| 3–4 | `sin_heading_error`, `cos_heading_error` | Exact body yaw minus route tangent | Use estimated heading and perceived/map lane heading. |
| 5–7 | `forward_speed_m_s`, `lateral_speed_m_s`, `planar_speed_m_s` | Native velocity projected onto route tangent / its norm | Replace with motion estimates; distinguish route frame from body frame. |
| 8–10 | `vertical_speed_m_s`, `height_m`, `upright_z` | Exact native velocity, world height and orientation | Estimate only if needed by the actor; retain exact versions for physics validation. Absolute world height is not a direct IMU reading. |
| 11–13 | `curvature_rad_m`, `curvature_3m_ahead_rad_m`, `curvature_6m_ahead_rad_m` | Known route queried at exact progress | Keep map geometry, but query using estimated localization; add perception validation. |
| 14 | `target_cruise_speed_m_s` | Configured desired speed | Keep explicit planning request. |
| 15 | `target_speed_m_s` | Analytic remaining-distance braking target; forced zero during settling | Recompute in planner from estimated location and explicit destination; separate startup interlock from actor state. |
| 16–18 | `applied_steering_rad`, `applied_throttle`, `applied_brake` | Applied control object | Keep command feedback explicitly labeled; this alone is not a noisy steering/wheel sensor. |
| 19–21 | `elapsed_fraction`, `settling`, `hold_fraction` | Episode clock, startup phase and success counter | Evaluator/controller lifecycle only; remove from new deployed actor. |

The task's physics guards additionally use contacts, support loss, exact tilt,
height, lane footprint and progress. These are experiment-stop/evaluation inputs,
not evidence of onboard perception. Preserve guard reports and distinguish a
simulation abort from a sensor-based braking action.

## Hazard and full-loop tasks: 42 features

[loop_hazard_contract.py](../environments/loop_hazard_contract.py) adds two
10-field slots to the lane22 vector. For slot `k` in `{0,1}`, offset `22+10*k`:

| Offset within slot | Field | Migration decision |
| --- | --- | --- |
| 0 | `valid` | Replace exact-object existence with track validity; unknown coverage remains explicit. |
| 1–2 | `forward_m`, `lateral_m` | Estimate object position in ego chassis frame. |
| 3–4 | `relative_forward_m_s`, `relative_lateral_m_s` | Estimate relative velocity from tracking/ego motion. |
| 5–6 | `half_length_m`, `half_width_m` | Estimate visible extent with uncertainty; do not use exact hidden full box. |
| 7–8 | `sin_relative_heading`, `cos_relative_heading` | Estimate orientation only when observable; represent uncertainty. |
| 9 | `age_s` | Keep actual acquisition/track age, without re-stamping held data as fresh. |

The `MeasuredObject` name does not establish a sensor origin.
[loop_hazard_lab.py](../environments/loop_hazard_lab.py) `_refresh_objects` reads
native hazard `measured_frames`; [full_loop_hazard_lab.py](../environments/full_loop_hazard_lab.py)
reads native background poses/velocities and wraps them as `MeasuredObject`.
These are privileged native measurements, not image/LiDAR detections.

`hazard_observation` chooses the nearest two by squared chassis distance with
object ID as tie-breaker. IDs are evidence, not explicit vector elements, but
exact identities influence slot ordering. Stale/future objects make the frame
unhealthy. Preserve that conservative handling with sensor tracks.

**Hidden assistance at index 15:** `safe_target_speed` computes a swept corridor
from exact object geometry and velocity. `HazardLaneTask._relative` inserts this
braking target into the base22 vector. Replacing only the 20 object fields would
leave a privileged planner signal. Static-stage remaining distance also depends
on an exact obstacle-derived stop station. Both require migration.

[FullLoopHazardTask](../environments/full_loop_hazard_contract.py) adds background
objects to actor selection and braking targets. Nearer traffic can mask a hazard
from the two slots; existing masked-hazard evidence must remain.
[ContinuousFullLoopHazardTask](../environments/continuous_loop_hazard_contract.py)
retains 42 fields but restarts index19's elapsed fraction for rolling 60 m windows;
this is another evaluator-dependent feature. Existing continuation/reset
wrappers do not turn this policy into sensor perception.

## Other old contracts retained as baselines

- [physics_vehicle_contract.py](../environments/physics_vehicle_contract.py)
  defines an eleven-value compatibility probe: exact `x_m,y_m,z_m,yaw_rad,
  vx_m_s,vy_m_s,vz_m_s,upright_z`, then three applied controls. It has zero reward
  and is explicitly an interface/reset check. Keep it as such; it is not a
  sensor-trained policy.
- [highway_parallel_env.py](../environments/highway_parallel_env.py) retains a
  legacy 16-value SUMO actor: speed/27, lane index, distance/route length, twelve
  exact front/rear neighbor fields across two lanes, and lane-change cooldown.
  Migrate the measurements by the same rules as MAPPO; cooldown is legitimate
  controller history. Do not confuse this checkpoint with the new27 contract.

## Existing LiDAR capabilities and limits

| Module | Reusable capability | Boundary to review |
| --- | --- | --- |
| [physical_lidar.py](../traffic/physical_lidar.py) | Actual RTX packet capture, success flags, per-ray offsets, scan completeness, acquisition/delivery times; rejects invalid packets | WORLD/NONCOMPENSATED points transformed with exact chassis position/quaternion in `world_to_chassis`. Metadata declares `odometry_source=simulator_chassis_pose`. Static-obstacle assumption; no complete moving-scene deskew. |
| [lidar_braking.py](../traffic/lidar_braking.py) | Label-free corridor clearance, age checks, episode identity checks, bounded braking/fail-safe | Caller supplies ego speed and target; audit their provenance too. Fixed height/corridor filtering does not cover all low/overhanging hazards. |
| [lidar_tracking.py](../traffic/lidar_tracking.py) | Geometric clustering/association and local track IDs, stale-track growth and motion uncertainty | World-XY input needs an explicitly justified transform. Visible surfaces bias extent/center; velocities are estimates, not guaranteed identities or accuracy. |
| [lidar_control.py](../traffic/lidar_control.py) | Label-free forward clearance and longitudinal speed target | Requires independently justified ego speed; flat-road ROI and healthy-scan assumptions are limited. |
| [lidar_avoidance.py](../traffic/lidar_avoidance.py) | Bounded static fixture avoidance | Persistent occupancy is explicitly valid only for that static fixture. |
| [realtime_lidar.py](../traffic/realtime_lidar.py) | SUMO/display/RTX synchronization and post-control ground-truth alignment audit | Interpolated display poses do not prove physical-car sensing. Keep evaluator alignment truth outside control. |

Generating sensor measurements from simulation necessarily uses scene geometry
and physics internally. That is different from passing truth into the estimator
or planner. Prefer calibrated sensor/body-frame packets; if a runtime only
provides world points, declare the acquisition transform and distinguish a
coordinate conversion from using a perfect vehicle trajectory for localization.
Do not claim that injecting noise into a full world pose implements an IMU.

## First milestone gates and deferred work

1. Keep legacy observations/checkpoints frozen. Introduce a separate versioned
   sensor/perception interface with units, frame, calibration, acquisition and
   delivery timestamps, validity, provenance and uncertainty.
2. Camera RGB is a measurement. Perfect depth/segmentation, simulator object IDs
   and exact object boxes are generation/evaluation labels unless explicitly
   modeling a declared depth sensor. A segmentation-colored debug overlay is
   not a detector.
3. Start forward-only scripted driving. Missing rear/side coverage must not be
   silently manufactured to satisfy the old27/42 vectors. Use conservative
   stale/unknown handling and log its interventions.
4. Validate ego-motion measurements/estimates, mounting transforms, reset
   isolation and timestamps before dynamic tracking claims. Document remaining
   truth-assisted runtime transforms honestly.
5. Audit the complete path through planner and emergency control, not only the
   neural actor. Replay identical sensor packets while changing evaluator-only
   truth; control output should remain unchanged. Test stale, missing, malformed,
   out-of-order and previous-episode packets.
6. Native evidence must show real captured camera images and LiDAR packets plus
   the actual scripted response. Synthetic CPU fixtures prove contracts, not
   rendering, sensor fidelity, highway safety or trained driving behavior.
7. Cosmos augmentation, surround coverage, radar, GNSS fusion and policy
   retraining remain later milestones. Review a new training contract before
   training; matching vector length alone does not preserve checkpoint meaning.
