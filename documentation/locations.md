# Project locations

Updated: 2026-09-18

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

`C:\Users\hudso\Documents\ChatGPT\Highway Sim` remains only so older commands and saved references continue to work. Its project entries point to the canonical project. New Codex tasks and terminal sessions should be opened directly in `C:\Users\hudso\Documents\highwaysim`.

## Folder ownership

| Folder | Contents |
| --- | --- |
| `scripts\` | Simulation environment, training, evaluation, and run-record code |
| `assets\` | Project-owned assets and asset metadata |
| `documentation\` | Design notes, provenance, parameters, methods, and iteration records |
| `outputs\` | Policies, checkpoints, evaluation results, and captured media |
| `logs\` | Installation, simulation, training, and validation logs |
| `isaacsim-research\` | Research notes and supporting experiments |

