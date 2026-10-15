# Four-leaf highway environment — October 6, 2026

The new [version folder](../highway_usd/_v01/README.md) implements the user's
`Documents/highwaysim/highway_usd/_v01` layout. This is a separate scene family
from the preserved three-lane circle in `scenes/highway/v1`.

The draft has four circular main lanes and a continuous outer auxiliary lane,
all counterclockwise so the outside is the driver's right. Four symmetric
same-level return petals branch from and rejoin that auxiliary lane. Z-up,
meter units, +X east / +Y north and local vehicle +X forward are explicit.
Navigation consists of ordered guide curves and a directed JSON route graph.
It supplies routing data, not a controller or SUMO adapter.

Static acceptance passed: all 28 installed USD validators, 108 prims, 129
valid relationship targets, 16 route edges without dead ends, four exact ramp
reconnections, strong auxiliary/ramp connectivity, four closed main lanes,
20 spawn anchors and a watertight pavement collider. Three corrupted-scene
fixtures were rejected. The minimum sampled ramp radius is 23.37 m; speeds
remain provisional. The version README documents tolerances and reproduction.

Two preserved Isaac runs at `outputs/highway_usd/v01-smoke-20261006-01` and
`v01-smoke-20261006-02` passed 26 stationary sphere drop checks each at 120 Hz
for three simulated seconds. Run 02 uses the final ground box and removes the
first run's large-triangle cooking warning. Both strict overall smoke gates
remain failed due to an unexpected stage reference count during close;
Fabric/renderer warnings also remain logged. These contact checks do not close
the existing runtime lifecycle or physical-vehicle acceptance gates.

Actual USD/RTX screenshots (top, angled and exit detail) live beside the scene
in `_v01/screenshots`, with a reproducible camera layer and hashes. Future
versions must retain the same views. The labelled `overview.png` is a separate
navigation diagram, not a simulator screenshot.

V02 should validate physical-car curvature/speeds, define merge/yield and
lane-change behavior, add shoulders and acceleration/deceleration geometry,
and connect the shared SUMO/Isaac map. No runtime dependency installation,
existing controller changes or policy training occurred. Offline authoring uses
a separate ignored `.venv` inside the version folder. Google guide sync was
not performed in this local-asset task.
