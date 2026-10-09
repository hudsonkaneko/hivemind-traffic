# v04 — synchronized live presentation

Status: short physical/visual integration verified; longer presentation pending.
Correct the v03 gap between native kinematic collision
poses and rendered USD transforms. Each background gets a nonphysical,
referenced `RenderProxy`; its display pose is copied from measured native state
in the dynamic layer before each render. The hidden kinematic chassis retains
collision ownership and never receives display pose commands.

Predeclared new gate: every displayed proxy must match its native body to
0.002 m / 0.0001 rad, including after rendering. Verify the visible side-by-side
pass with screenshots. Repeat physical acceptance, fresh-process comparison,
and the default four-minute paced GUI preview on this corrected version.

The source vehicle remains local-only and unchanged. No training or dependency
installation. Previous versions and failed visual evidence remain preserved.

Initial proof: `outputs/loop_showcase/20261009T175702Z-57369b07` passed a
25-second drive plus settling/braking (37 simulated seconds). One completed
pass, one lane change, 1.6908 m minimum body clearance, no contacts, and 740
rendered-frame synchronization checks passed. The t=20 traffic-overhead image
now visibly shows the side-by-side pass. This unpaced run is not a real-time claim.
