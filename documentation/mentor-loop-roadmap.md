# Mentor-guided continuous-loop branch

This is a branch-off of the existing project roadmap, not a replacement.
The SUMO demos, hybrid/mixed-traffic plan, previous physical experiments and
their evidence remain intact. Workspace: the canonical `highwaysim` checkout.

## Ordered milestones

| Step | Work | Exit gate / current scope |
| --- | --- | --- |
| 1 | Preserve V02 and package a referenced environment | Source hashes unchanged; portable references, visual payload, required road collider, one physics scene |
| 2 | Add continuous known-map loop following | CPU tests: periodic projection/lookahead, true lap distance, identity/time guards and explicit braking |
| 3 | Integrate one physical car | Fixed 120 Hz PhysX, 60 Hz steering/torque commands; 3 m/s, 6 m/s, then 35 mph short runs |
| 4 | Qualify continuous driving and preview | Measured lane/footprint/support/speed/braking gates, command dropout, full lap, repeated fresh runs; follow/overview cameras |
| 5 | Prepare an Isaac Lab task | Same physical/control contract, bounded episode resets, observations/actions/rewards design; no automatic training launch |
| 6 | Qualify parallel environment copies | Start 1, then 4, then measured increases; collision isolation, reset independence and resource limits. Do not assume the entire 1.15 km stage should be cloned |
| 7 | Agree on first training run with the user | Review observations, actions, rewards, algorithm, budget and held-out evaluation together before training |
| 8 | Train progressively | Speed/lane keeping, spacing, more radii, then disturbances/traffic. Preserve scripted baselines |
| 9 | Introduce sensor-derived observations | LiDAR timing/frames/failures; explicitly separate sensor input from privileged simulator state |
| 10 | Add coordination and matched mixed traffic | Independent versus communicating AVs, human variation, 0/25/50/75/100% AV ratios, matched demand/seeds and fairness/safety/efficiency metrics |
| 11 | Rejoin main integration and demo goals | Vehicles, overview/follow/path overlay, teammate environment integration; measured performance and reusable assets |

## Current implementation checkpoint

Steps 1–4 passed their bounded one-car scope in `codex/mentor-loop-driving`:
portable composition, periodic control, increasing-speed physical checks,
command dropout, a real-time short GUI preview, two exactly matching full-lap
fresh runs and a three-lap / 9.93 km run. No same-process reset-soak, traffic,
LiDAR or learned-control qualification is implied.
See [continuous highway loop](continuous-highway-loop.md) for exact commands,
frozen gates and evidence. A static road validation is not a vehicle-driving
pass; a seam crossing is not a full lap; a fresh-process repeat is not a
same-process reset soak. Those distinctions remain explicit in status reports.

No Isaac Lab training, dependency installation, detailed vehicle-asset migration,
or main-roadmap rewrite is part of this checkpoint.

**Next implementation:** step 5, the Isaac Lab task/episode-reset plumbing and a
scripted baseline using the same physical/control contract. Validate a single
environment before any clone scaling. Do not launch policy training: step 7
requires direct user review of observations, actions, rewards, budget and evaluation.
