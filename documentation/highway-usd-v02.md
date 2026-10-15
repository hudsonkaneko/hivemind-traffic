# Outer collector highway — October 6, 2026

[V02](../highway_usd/_v02/README.md) replaces the tight v01 petals with a fixed
outer collector ring and four gradual outbound/four gradual return connectors.
V01 remains unchanged, verified against 22 local file hashes.

The main lane radii are 500 / 503.7 / 507.4 / 511.1 m, auxiliary 514.8 m and
collector 535 m. All travel is counterclockwise, Z-up and meter-based. Each
connector occupies 42 degrees of its own 45-degree sector, with quintic radial
interpolation matching the ring's tangent and curvature at both ends.

The independently calculated minimum connector radius is 379.386 m (v01:
23.366 m). The geometric lateral acceleration estimate at 65 mph is 0.227 g
on the tightest connector and about 0.172 g on the innermost main lane. That
lane takes 108.12 seconds per lap at 65 mph. These are unbanked geometric
estimates, not a physical vehicle's validated speed envelope.

`route_policy.py` provides executable seeded planning: choose whether to exit,
then rejoin at one of the next four return opportunities. Requested speed and
release-time variations are saved with the seed. Roads do not move during a
run. The planner does not grant merge priority or implement vehicle control,
lane changes, yielding, collision avoidance or SUMO network integration.
Return links leave the collector on the left; inner-highway exits remain on
the right. No connector paths cross.

## Evidence

- All 28 installed USD validators passed: 143 prims, 191 valid relationship
  targets, 14 lane paths, 28 edges, 24 spawn anchors and watertight pavement.
- All 24 auxiliary/collector/connector edges form a strongly connected graph;
  the four main-lane edges are closed loops. Every route can continue.
- 1,280 seeded trips across 20 seeds returned within one collector lap; four
  targeted route-policy tests passed. Independent waypoint curvature agrees
  with the analytic check. Render/collision footprint discrepancy is 0.03321 m².
- USDA, navigation, parameters and route examples reproduce byte-for-byte in
  `outputs/highway_usd/v02-repro-20261006-01`.
- All 47 stationary sphere contacts passed over three simulated seconds at
  120 Hz in `outputs/highway_usd/v02-smoke-20261006-01`. The strict overall
  smoke result is failed due to an unexpected stage reference count during
  close. This reproduces the warning class retained from v01; its owner remains
  unresolved. No moving-car or runtime lifecycle acceptance is claimed.
- Three actual USD/RTX screenshots and a four-choice route diagram are saved
  in the version folder. Source and screenshot hashes are recorded.

The raw smoke log stays in the ignored local evidence directory. Compact
validation, smoke and reproducibility reports are committed beside the asset.
Screenshot capture was separate from contact stepping; no performance claim
is made with another existing simulator session open. Existing simulator
sessions and unrelated project edits were not modified.

V03 should add a physical driver using these routes, yielding and lane-change
behavior, moving-car checks at increasing speeds, and shared SUMO/Isaac map
integration. No policy training or Google guide synchronization occurred.
