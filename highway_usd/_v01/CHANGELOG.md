# Changelog

## _v01 — 2026-10-06

- Generated a static, Z-up four-lane circular highway with a continuous outer auxiliary lane and four same-level radial `sin⁴` ramp petals that return downstream.
- Added a single union pavement collider, separate analytic box ground, road markings, USD lane and edge guide geometry, eight merge/diverge zones, and twenty surface spawn anchors.
- Exported the navigation sidecar, parameters, top-down preview, and SHA-256 manifest. Manifest records USD 0.26.8 and Shapely 2.1.2.
- Corrected degenerate paint triangles by snapping mesh footprints to 1 mm and triangulating the float32 coordinates USD will store. Replaced the first draft's large-triangle ground mesh with a box after a PhysX cooking warning.
- Static validation passed: all 28 USD validators, watertight collider, exact route endpoint joins, strong auxiliary/ramp connectivity, and three corruption-regression tests. Final USDA/navigation/parameters reproduce byte-for-byte.
- Both retained Isaac smoke runs passed all 26 sphere-contact checks; the strict overall smoke result remains failed because of the unresolved stage-close reference-count warning. Run 02 removes the ground cooking warning. This does not validate vehicle driving or traffic behavior.
- Added actual USD/RTX top, angled and diverge-detail screenshots, a reusable capture script, and camera/source/image manifests. Keep the same views in every later version.
- v02 priorities: validate ramp curvature and speeds with the physical vehicle; add lane-change/yield routing; widen acceleration/deceleration areas and add shoulders; integrate the shared map.
