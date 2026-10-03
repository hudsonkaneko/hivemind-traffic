# Isaac Sim 7 / OVRTX migration

Started: 2026-09-21. Status: libraries built, physics and rendering smoke tests passed; vehicle migration blocked by non-finite articulation state. This is NOT a completed migration.

## Recovery baseline

Before any project edits, the complete canonical project was duplicated to
`C:\Users\hudso\Documents\highwaysim-backup-20260921-161242`.
All 229 files, including hidden Git metadata, were verified with SHA-256 against
the original. This is an independent copy, not hard links. External Isaac Sim
and Isaac Lab installations are not included in the project-folder backup.

Existing `demo.cmd`, `run.ps1`, vehicle code, trained policies, and original
external runtimes are retained. The backup was explicitly requested by the user.

## Experimental runtime

- Source: https://github.com/isaac-sim/IsaacSim/tree/v7.0.0a1
- Revision: `2469084bc328710207c6bc4ede32209082a9286c`
- Project-local checkout: `runtimes/IsaacSim7`
- Build log: `logs/isaacsim7-build.log`
- Repeatable build entry point: `scripts/build_isaac7.ps1`
- GPU checked: RTX 5070, 12227 MiB VRAM, driver 610.74.

The alpha provides no prebuilt library wheels. Its documented Windows source
build bootstraps the pinned toolchain. The existing Isaac Lab beta task cannot
be assumed compatible with the new libraries.

## Required acceptance checks

1. Build and import the standalone Isaac Sim libraries.
2. Render a camera with OVRTX in an isolated environment.
3. Validate shared scene updates between physics and rendering.
4. Port the Leatherback vehicle, including suspension and Ackermann control.
5. Preserve the 9 observations / 2 actions and evaluate the existing PPO policy.
6. Compare driving results and retain interactive inspection before switching defaults.

Sources: https://docs.isaacsim.omniverse.nvidia.com/7.0.0a1/installation.html
and https://github.com/NVIDIA-Omniverse/ovrtx .

## Validated results

- Built all 13 Isaac Sim 7.0.0a1 wheels on Windows; build returned exit code 0.
- Installed the physics, foundation, common, and OV SIM libraries and dependencies in `.venv-isaac7`.
- Physics test: cube fell from 2.0 m to 1.8501247168 m in ten steps, exit code 0.
- OVRTX 0.5.0.377615 rendered a reference image alone, but failed when combined with Isaac Sim 7: OmniClient 2.74 versus the physics runtime's exact 2.72.3 requirement. Both import orders were tested.
- OVRTX 0.4.1.364340 rendered the reference scene with the Isaac Sim OvPhysX backend loaded in the same process, exit code 0. The output image was visually inspected. This validates coexistence, not synchronized physics/render updates.
- In combined mode, import Isaac Sim physics before OVRTX/OVStage. Isaac Sim selects its bundled OVStage Python/runtime rather than the separately installed ovstage distribution. The renderer still emits missing-schema warnings; camera functionality beyond this reference scene is unverified.
- `outputs/migration/ovrtx-smoke/frame-0.png` is a reference robot scene, NOT the migrated car.

## Vehicle blocker

The stock Leatherback loads and exposes its joints, with finite initial masses,
inertias, and poses. However, simulated transforms become non-finite during the
first few steps. Tested authored joint poses, disabled self collisions and the
collision group, solver iteration overrides, 1/120 s and 0.001 s steps, and
bounded wheel/steering gains. The 0.001 s test failed at step 3. Exact cause is
not established; this is not evidence that every vehicle is unsupported.

Initial imported wheel damping was approximately 5.73e9 and steering stiffness
5.73e4. A diagnostic with wheel damping 100 and steering stiffness/damping
100/10 still failed. These diagnostics did not modify the original asset or task.

The existing PPO policy, training loop, circular-lane demo, and interactive viewer
have NOT been migrated or validated on the new runtime. Their original files and
launchers remain unchanged. Do not use these experimental checks as a trained
vehicle demo.

## Reproduce

From the project root in PowerShell:

```powershell
.\migration.ps1 -Check physics
.\migration.ps1 -Check render
.\migration.ps1 -Check vehicle  # Currently expected to fail; diagnostic only.
```

Logs are under `logs/isaacsim7-*` and `logs/ovrtx-*`. Environment package versions
are captured in `documentation/migration-requirements.txt`. Native build tools
also populated their standard shared caches outside the project (Packman, Pixi,
pip, NVIDIA shader/asset caches); project sources and the environment are local.

Next required work is a minimal reproduction of the articulation failure,
followed by a stable car physics port and synchronized camera updates. Do not
switch the default launcher until those and policy evaluation pass.
