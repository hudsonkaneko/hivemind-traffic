# Highway Sim documentation

Updated: October 4, 2026. The only active local workspace is
`C:\Users\hudso\Documents\highwaysim`; its remote is
`hudsonkaneko/hivemind-traffic`. See [locations](locations.md) and the
[consolidation record](consolidation-2026-10-02.md).

## Start here

The working traffic demonstrations currently use **SUMO-owned movement** with
Isaac Sim rendering, RTX LiDAR, scripted obstacle avoidance, and two-car V2V.
A separate **Leatherback waypoint prototype** already uses Isaac physics and
PPO, but its RC-scale vehicle is not a validated highway vehicle. Neither the
capacity probes nor that prototype establish a physics-driven mixed fleet.

The active development direction is a hybrid experiment: **Isaac physics owns
research cars; SUMO owns economical background traffic**. The one-car physics
foundation is in progress. Later phases keep the existing sensing, communication,
mixed-traffic, statistics, and visualization goals.

| Read this | Purpose |
| --- | --- |
| [Active 20-step roadmap](hybrid-roadmap.md) | Development order, acceptance gates, and what is not validated yet |
| [Physics-vehicle foundation](physics-vehicle-foundation.md) | First low-speed physical-car fixture, verified subset, and remaining gates |
| [Dynamics, resets and Isaac Lab](physics-validation.md) | Expanded physical checks, real Lab reset/step compatibility, and retained warning investigation |
| [Scripted lane following](lane-following.md) | Straight/curved 100 m physical routes, driver layers, measured tracking/stop/contact checks, and future user-led training setup |
| [Visible physical car + LiDAR](physics-lidar-view.md) | Curved road, overview/follow controls, path overlays, sensor timing and scripted emergency braking |
| [Wheel orientation correction](wheel-orientation.md) | Explicit axle basis, visual/physical regression checks and unchanged driving evidence |
| [Adaptive obstacle bypass](obstacle-bypass.md) | Scripted one-car LiDAR detour, cyan driver-path preview, blocked-road and sensor-loss checks |
| [35 mph physical profile](physical-35mph.md) | Extended-road speed target, longer stopping/turning geometry, denser LiDAR, validation status and preserved low-speed mode |
| [Hybrid motion-authority decision](decisions/001-hybrid-motion-authority.md) | Who moves each vehicle, responsibilities transferred from SUMO, and comparison limits |
| [Change log](CHANGELOG.md) | Meaningful changes, reasons, and evidence pointers |
| [Contribution and documentation workflow](workflow.md) | Branches, tests, commits, GitHub review, and learning-document updates |
| [Iteration template](templates/iteration.md) | Purpose, implementation, commands, evidence, failures, and next gate |
| [Portable demonstrations](portable-demos.md) | Launch commands and environment configuration |
| [Learning guide](LEARNING_GUIDE.md) | Technology explanations and the earlier implementation narrative |
| [Google guide sync](learning-guide-sync.md) | Latest native notebook update and verification scope |

## Existing traffic, sensing, and evidence

| Read this | Scope |
| --- | --- |
| [Cooperative LiDAR walkthrough](cooperative-lidar.md) | Two-car scripted sensing and V2V fixture; SUMO still owns motion |
| [Cooperative results](cooperative-lidar-results.md) | Retained passes, failures, repeat differences, and GUI memory measurements |
| [Scaling foundations](scaling-foundations.md) | State contracts, seeded fleet assignments, probes, and startup refresh |
| [Measured capacity](capacity-results.md) | SUMO and RTX workload measurements; not a physical-fleet capacity guarantee |
| [State contract v1](state-contract-v1.md) | Existing SUMO front-bumper, identity, and simulation-time rules |
| [Live LiDAR demo](live-lidar-demo.md) | Single-car feedback demonstration |
| [LiDAR avoidance](lidar-avoidance.md) | Scripted obstacle-passing controller |
| [LiDAR map-source investigation](lidar-map-source-study.md) | Identity-label investigation and evidence |
| [ovrtx replay](ovrtx-replay.md) | Separate renderer milestone; not an Isaac vehicle solver |

The [earlier implementation roadmap](../experiments/ROADMAP.md) remains a
historical record of the SUMO/replay migration. Its old immediate-next-step
sections are not the current work queue; use the active roadmap above.

## Preserved Leatherback prototype

| Read this | Scope |
| --- | --- |
| [Vehicle provenance](vehicle.md) | RC-scale asset, measured dimensions, joints, and limitations |
| [Methods](methods.md) | Original waypoint physics, observations, actions, PPO, and evaluation |
| [Parameters](parameters.md) | Original prototype values and units |
| [Historical run records](runs/) | Existing September 11 source snapshots and manifests |
| [Experimental runtime migration](migration-2026-09-21.md) | Earlier Isaac/ovrtx runtime investigation, not a completed replacement |

Historical records and paths describe the run that produced them. Do not rewrite
them to imply the new architecture or current dependency versions. Generated new
evidence belongs in unique `outputs/` run directories; keep reviewed summaries
here. Human-written dates use America/Los_Angeles; machine records retain their
explicit time zone.

## Demonstrate what exists

From the repository root in the configured traffic Python environment:

```text
python -m hivemind demo sumo --check
python scripts/demo_cooperative_lidar.py --check
python scripts/demo_cooperative_lidar.py
```

`--check` resolves the launch configuration; it does not validate sensing or
driving. The final command launches the existing SUMO-motion cooperative demo,
not the new physical-car foundation. See the [project README](../README.md) for
the separate Leatherback demonstrations and their dependencies.
