# Live label-free lidar milestone contract

SUMO alone owns vehicle poses and road-following. RTX measures the scene;
the follower controller consumes only forward lidar clearance and ego speed.
Obstacle/lead truth is evaluation-only. No PPO, object labels, or physical steering.
SUMO speed safety remains enabled (31); all requested/realized speeds are logged.
A stationary obstacle exists only in USD, so SUMO cannot independently brake for it.

First implementation uses lockstep sample-and-hold: freeze a SUMO snapshot,
update USD, obtain a fresh complete RTX scan after renderer settling, compute
the next command, advance SUMO once by 0.1 seconds. Sensor acquisition clock
and traffic clock are distinct and both recorded. This is live feedback, not
replay or continuous-motion lidar; wall-clock real time is not promised.

Predeclared acceptance: no collision/obstacle penetration, minimum ground-truth
gap at least 2 m, final speed below 0.2 m/s, progress at least 5 m, no sensor
timeouts, maximum checked clearance error 0.25 m, no unexplained SUMO speed
intervention above 0.15 m/s. Stationary tests: speed/gap 4/20, 8/40, 12/65;
following: 6/30 with lead at 4 m/s braking at traffic time 8 seconds. Run each
for 25 traffic seconds, repeat the default stop/follow cases, and test another
seed. Retain failures. Fault injection must command braking on absent/stale
scans. No runtime modifications. Start with a short exploratory smoke capture.

Repeated following runs must agree within 0.001 m for ego position, lidar clearance,
and gap, and 0.001 m/s for speed (declared before the first following case).
Raw sensor timestamps need not be identical because sensor and traffic clocks
are separate. Compare the state/control trajectory at each traffic step.

Diagnostic amendment after the first repeated native-profile pair: the maximum
clearance difference was 0.00146484375 m, failing the original 0.001 m gate.
The original failure remains reported. The profile has azimuthErrorStd=0.015 deg
and rangeAccuracyM=0.02 m. Run two additional 25 s following cases with an explicit
`--ideal-sensor` flag setting angular standard deviations and range accuracy to
zero. Keep the original 0.001 tolerance for this separate diagnostic; do not
replace the original noisy-profile runs or silently relax the gate.

Sensor mount is the ego front bumper at z=1 m, world Z up, metres, +X road.
SUMO front-bumper x/y maps directly to USD x/y; yaw=90-angle. The central
forward corridor excludes self geometry behind the bumper and ground below
0.3 m. Zero obstacle returns means clear only with a complete healthy scan;
invalid or missing scans trigger braking. This is not a general perception model.

Reference: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html
Reference: https://docs.isaacsim.omniverse.nvidia.com/latest/py/docs/source/generic_model_output/generic_model_output.html
