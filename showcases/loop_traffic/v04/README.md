# v04 — synchronized live presentation

Status: verified — full physical/visual/real-time GUI presentation, fresh repeat,
blocked-lane following, command-loss braking and contact positive control.
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

Full GUI proof after precision/runtime corrections:
`outputs/loop_showcase/20261009T180436Z-70fff812`, source `fa8eff9`.
252 simulated seconds / 252.0003424 live wall seconds, 12 distinct passes,
5 completed lane changes, 3,698.0942 m traveled, minimum clearance 1.5808 m,
zero contacts, 18.0196 m stopping distance and 7.7083 s stopped hold. All 5,040
visible-frame checks passed; source files were unchanged and stage/child process
closed cleanly. RTX 5070 whole-desktop GPU peak was 2,722 MiB. Actual follow
captures show the requested Americano and synchronized moving block cars.

```text
python scripts/demo_loop_showcase.py --version v04
```

Use the configured traffic environment from the canonical repository root.
The default drives for four minutes, brakes, then closes. Camera and pause/path
controls are in the showcase window. This is scripted ground-truth-object
driving, not sensor perception or RL. The main research roadmap is preserved.

Fresh-process repeat: `20261009T180927Z-9edacf1b` and
`20261009T181057Z-d2fbb54a`, each 110 seconds of driving. Comparison
`outputs/loop_showcase_repeatability/20261009T181248Z-29b478bb` passed: 14,640
aligned physics samples with zero position/speed difference, nine passes,
three changes, and all 2,440 displayed-frame checks in each run. This does not
claim cross-platform determinism or same-process reset repeatability.

Corrected-version fault checks also passed: contact positive control
`20261009T181228Z-6e5d47fc`, blocked-lane following
`20261009T181244Z-daf21212`, command-loss braking `20261009T181325Z-197921ce`.
Exact gates and limitations are in the [technical record](../../../documentation/loop-traffic-showcase.md).
