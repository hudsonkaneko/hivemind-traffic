# Changelog

## _v02 — 2026-10-06

- Enlarged the circular highway so MainLane1 has a 500 m centerline radius; set the auxiliary centerline radius to 514.8 m and added a separated 535 m outer collector.
- Added four 42° quintic smootherstep exits and four reentry connectors in alternating sectors. The CCW return connectors leave the collector on its left and rejoin the auxiliary ring without crossing paved road.
- Added a 14-lane, 28-edge directed navigation graph, junction metadata, 24 surface spawn anchors, and repeatable route plans with one to four reentry opportunities.
- Independent validation passed: 28 USD validators, 1,280 seeded route plans, and both connector directions at 379.386 m minimum curvature radius and 0.2269g theoretical unbanked lateral load at 65 mph. These are static geometry and planning checks, not vehicle handling results.
- Three rendered screenshots (top, angled, and east diverge) were captured and recorded with hashes. All 47 contact assertions passed, but the smoke supervisor rejected its run because of an `Unexpected reference count` warning during stage close; a clean bounded smoke pass remains unresolved.
- Next: validate a moving vehicle and add lane-change, yield, and merge handling. The 65 mph design figure is not a safe-speed claim.
