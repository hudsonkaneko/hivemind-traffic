# Lidar and 3D replay transfer

Moved the work created during the lidar and 3D replay tasks from
`C:\Users\hudso\Documents\highwaysim` into this repository.

All 25 files were copied, compared with SHA-256, and matched before the original
files were removed. No destination files existed before the transfer. Zero
original files from that transfer remained afterward. Unrelated Highway Sim
files were left untouched; empty source directories may remain.

Transferred: three Python scripts, two documentation files, three logs, six
standalone lidar output files, and eleven recorded-traffic/replay output files.
Historical logs retain the original run paths as evidence of where those runs
occurred. Scripts resolve their project root from their own location, so new runs
use this repository. USD replay layer references are relative.

Generated outputs and logs are ignored by Git; source code and documentation can
be version controlled. This repository's AGENTS.md records the correct root.

Post-transfer verification: `scripts/traffic_replay_3d.py` ran successfully from
this repository and exited with code 0. It rebuilt the composed stage, passed all
10 vehicle endpoint checks, and generated 40 moving lidar frames. The new log is
`logs/traffic-replay-transfer-check.log`; the refreshed report and replay are in
`outputs/sumo_replay/`. This verification refreshed generated replay artifacts
after their initial byte-for-byte transfer verification.
