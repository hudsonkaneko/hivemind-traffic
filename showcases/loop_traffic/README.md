# Loop traffic showcase

Separate, same-day presentation branch; the research roadmap and SUMO demos
remain available. Isaac Lab task/training work is postponed for this showcase.

**Responsive option:** [v07](v07/README.md) adds shared immutable fleet visuals,
an optional smaller single-viewport Kit app and explicit native-frame/UI timings.
The regular editor UI is now the v07 default. The original minimal-app
30-Hz GUI runs held real-time speed and repeated exactly, but rare hitches remain;
60 FPS is experimental. See the
[responsive-showcase evidence](../../documentation/showcase-responsiveness.md)
for longer-run results and limitations. Run
`python scripts/demo_loop_showcase.py --version v07`; the default stays v04.

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
| v07 | Immutable visual instancing, small Kit app and native-frame/UI timing | Same physics and colliders, matched repeats, physical failure controls, and separate strict 30/60-Hz display gates; see measured evidence |
| v08 | Native sidebar, live display controls and supervised next-run settings | Native callback/disabled-control checks, safe stop/restart and preserved evidence |
| v09 | Live density, speed targets and following gap using a fixed pool | Repeated same-process count changes, native identity/visibility, physical safety and clean process gates; see [live controls](../../documentation/showcase-live-controls.md) |

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
