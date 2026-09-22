# Hivemind Traffic project scope

Use this repository, `C:\Users\hudso\Documents\hivemind-traffic`, for all work on
this project. Do not use `highwaysim` or its legacy compatibility path for new
project files. Isaac Sim and Isaac Lab installations are external dependencies.

Keep project paths relative to the repository root. Store generated replay and
lidar artifacts in `outputs/`, runtime logs in `logs/`, and implementation notes
in `documentation/`. Preserve the existing directory boundaries for SUMO,
PettingZoo, policies, OpenUSD, visualization, experiments, and tests.

Project direction: both Isaac Sim with RTX lidar and ovstage + ovrtx are intended
targets. Share portable USD assets, motion recordings, and timestamp conventions
between them. Keep renderer-specific sensors and UI in separate adapters/layers.
Commit and push each tested, meaningful change; exclude generated scans and logs.
