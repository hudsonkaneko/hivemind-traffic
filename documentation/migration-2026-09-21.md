# Isaac Sim 7 / OVRTX migration

Started: 2026-09-21. Status: dependency build and compatibility validation in progress; not yet a working migrated vehicle.

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
