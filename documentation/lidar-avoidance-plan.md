# Static obstacle avoidance contract

Implement `--mode avoid` without changing stop/follow behavior. SUMO owns all
motion and executes 5-second continuous lane changes. USD follows SUMO position
AND heading. RTX scans are acquired with poses held, as in the existing demo.

The planner consumes raw label-free lidar, ego odometry, and two known road lane
centres. Ground and self returns are excluded in sensor coordinates, then points
are transformed into road coordinates. A persistent 25 cm occupancy grid retains
STATIC obstacles through occlusion. It must not be used for moving traffic.
No obstacle position, dimensions, or IDs enter the planner from simulator truth.

Detect the blocking obstacle; require the neighboring lane's observed map clear
from 30 m behind to 45 m ahead; move left; pass; return after the observed obstacle
is at least 12 m behind the front bumper. Keep forward braking active and cap
maneuver cruise speed at 6 m/s. No fresh scan means no new lane command and braking;
acquisition failure freezes traffic. SUMO speed safety 31 and lane safety 512 remain.

Predeclared GPU checks: free lane at speed/gap 6/40 twice, 8/55 with seed 43,
blocked neighboring lane at 6/40, and a GUI free-lane run. Each lasts 25 traffic
seconds. Free-lane runs must pass the obstacle and return to lane 0. Blocked runs
must stay in lane 0 and stop. Every run requires zero collisions, conservative
oriented-body AABB separation above 0.3 m from every static box, lateral change
at most 0.15 m per 0.1 s step, and no ego speed intervention above 0.15 m/s.
Also require the oriented vehicle footprint to stay within the 6.4 m road width.
Compare repeated control/pose traces within 0.01 m and 0.01 m/s; noisy raw scans
are not expected to match exactly. Preserve failures. Independently re-audit scans.

The road is known, straight, and has two lanes. Empty measured space is not a
general proof of visibility; this fixture does not handle moving traffic,
unknown obstacles hidden behind others, pedestrians, road perception, or physics
steering. This is scripted avoidance, not PPO.

Reference: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html
Reference: https://sumo.dlr.de/docs/Simulation/Output/Lanechange.html
