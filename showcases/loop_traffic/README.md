# Loop traffic showcase

Separate, same-day presentation branch; the research roadmap and SUMO demos
remain available. Isaac Lab task/training work is postponed for this showcase.

## Version folders

These are deliverable milestones, not copies of the repository. Exact code is
tracked by Git; unique raw runs live under ignored `outputs/loop_showcase/`.
The requested prepared vehicle is local-only and is never uploaded to GitHub.

| Version | Intended change | Acceptance before calling it ready |
| --- | --- | --- |
| v01 | Prepared physics car plus referenced, slower block traffic | Loaded dependencies, stable wheel support, explicit movement ownership, source preservation, real moving colliders |
| v02 | Object-aware scripted passing | At least 3 distinct complete passes and 2 completed lane changes in 90 s, no rigid contact or overlap, road containment and minimum 0.6 m planar body clearance |
| v03 | Presentation and reliability | Follow/overview/adapting-path display; paced GUI within measured 1x tolerance; repeat test, blocked-lane and command-loss braking, longer circulation |

## Scope

Main car: native PhysX, steering and wheel torques. Background: deterministic
kinematic PhysX targets (nonreactive, infinite effective mass), not human models.
Controller: known map and privileged, timestamped object tracks; not LiDAR
perception, PPO or hivemind intelligence. Obstacles are not disabled to allow a
pass. Stop or follow if a neighboring lane is unavailable.

See [technical record](../../documentation/loop-traffic-showcase.md). Open a
saved `scene/world.usda` only to inspect the initial assembly; use the Python
launcher for a live, controlled simulation.
