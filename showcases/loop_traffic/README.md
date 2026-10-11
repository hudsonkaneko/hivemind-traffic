# Loop traffic showcase

Separate, same-day presentation branch; the research roadmap and SUMO demos
remain available. Isaac Lab task/training work is postponed for this showcase.

**Performance option:** [v06](v06/README.md) keeps the same 80-car physical
scenario with shared reads, less redundant display work and an explicit light
evidence profile. Two full GUI drives repeated exactly and averaged 1x, but
still missed strict lag/frame-interval limits. See the
[measured performance record](../../documentation/showcase-performance.md).
Run `python scripts/demo_loop_showcase.py --version v06`; v04 remains default.

**Denser option:** v05 adds 80 slower block cars with alternating passing gaps.
Run `python scripts/demo_loop_showcase.py --version v05`. It is not real-time
qualified; see [dense-preset evidence](../../documentation/dense-loop-showcase.md)
for the separate physical, visual, repeatability and timing results. The default
remains v04 so an existing real-time demo command is not silently made slower.

**Ready to demonstrate:** v04 passed a full, real-time GUI circuit with the
prepared Americano: twelve overtakes, five lane changes, no contacts and a clean
stop/shutdown. Run `python scripts/demo_loop_showcase.py --version v04` from the
configured traffic environment at the repository root. See v04 for measured
evidence, repeat and failure tests; the supplied vehicle remains a local prerequisite.

## Version folders

These are deliverable milestones, not copies of the repository. Exact code is
tracked by Git; unique raw runs live under ignored `outputs/loop_showcase/`.
The requested prepared vehicle is local-only and is never uploaded to GitHub.

| Version | Intended change | Acceptance before calling it ready |
| --- | --- | --- |
| v01 | Prepared physics car plus referenced, slower block traffic | Loaded dependencies, stable wheel support, explicit movement ownership, source preservation, real moving colliders |
| v02 | Object-aware scripted passing | At least 3 distinct complete passes and 2 completed lane changes in 90 s, no rigid contact or overlap, road containment and minimum 0.6 m planar body clearance |
| v03 | Presentation and reliability | Follow/overview/adapting-path display; paced GUI within measured 1x tolerance; repeat test, blocked-lane and command-loss braking, longer circulation |
| v04 | Correct native-to-visible background synchronization | Render-only referenced proxies mirror measured physics poses; check every displayed frame; repeat and paced GUI qualification on the corrected view |
| v05 | 80-car staggered traffic and explicit fleet rundown | At least 6 passes/4 changes in 90 s, original clearance/contact gates, synchronized visuals, repeated physical result and stopped background fleet; timing reported separately |
| v06 | Profiled, lighter presentation of the same 80-car scenario | Same physical motion/safety, strict same-source repeat, explicitly sampled versus full audits; smooth real-time remains a separate open gate |

**Visual correction:** early v01–v03 runs proved physical obstacle motion, not
visible obstacle motion. Screenshot review found static block visuals despite
moving native colliders. Those old scenes remain as evidence; v04 addresses
that defect and remains the lighter real-time presentation option.

## Scope

Main car: native PhysX, steering and wheel torques. Background: deterministic
kinematic PhysX targets (nonreactive, infinite effective mass), not human models.
Controller: known map and privileged, timestamped object tracks; not LiDAR
perception, PPO or hivemind intelligence. Obstacles are not disabled to allow a
pass. Stop or follow if a neighboring lane is unavailable.

See [technical record](../../documentation/loop-traffic-showcase.md). Open a
saved `scene/world.usda` only to inspect the initial assembly; use the Python
launcher for a live, controlled simulation.
