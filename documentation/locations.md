# Project locations

Updated: 2026-10-02

## Canonical project root

All Highway Sim project work belongs under:

`C:\Users\hudso\Documents\highwaysim`

This directory contains the source scripts, launchers, assets, documentation, logs, training outputs, and Git metadata.

## External runtime dependencies

These installations remain outside the project because they are large, independently managed runtimes:

| Component | Location | Role |
| --- | --- | --- |
| NVIDIA Isaac Sim | `C:\isaacsim` | Simulator, physics, rendering, and bundled Python runtime |
| NVIDIA Isaac Lab | `C:\Users\hudso\Documents\IsaacLab` | Reinforcement-learning framework and launcher |

The Highway Sim launch scripts refer to these locations. They are dependencies, not additional project roots.

## Legacy compatibility path

The old `C:\Users\hudso\Documents\ChatGPT\Highway Sim` compatibility entries
and `C:\Users\hudso\Documents\hivemind-traffic` checkout have been retired into
`C:\Users\hudso\Documents\expired_highwaysim`. Do not use those paths to run or
edit the project. The empty legacy directory may remain while an application
holds it open. Open new tasks and terminals directly in the canonical root.

The canonical checkout's remote remains
`https://github.com/hudsonkaneko/hivemind-traffic.git`.
See [the consolidation record](consolidation-2026-10-02.md).

## Folder ownership

| Folder | Contents |
| --- | --- |
| `scripts\` | Simulation environment, training, evaluation, and run-record code |
| `assets\` | Project-owned assets and asset metadata |
| `documentation\` | Design notes, provenance, parameters, methods, and iteration records |
| `outputs\` | Policies, checkpoints, evaluation results, and captured media |
| `logs\` | Installation, simulation, training, and validation logs |
| `isaacsim-research\` | Research notes and supporting experiments |
