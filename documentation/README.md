# Highway Sim documentation

Created: 2026-09-11. Project: `C:\Users\hudso\Documents\highwaysim`.

Current file and dependency locations are documented in [locations.md](locations.md).
Dates in the human-written history use America/Los_Angeles; automatic records
include both UTC and the machine's local UTC offset.

This folder explains what exists, how it works, and what changed. The current
milestone is a single-car waypoint task, before highway traffic or coordination.

| Read this | What it contains |
|---|---|
| [Vehicle provenance](vehicle.md) | Where the car came from, its dimensions, and local modifications |
| [How the simulation and training work](methods.md) | Physics, observations, actions, reward, PPO, and evaluation |
| [Parameter reference](parameters.md) | Concrete values, units, and where to change them |
| [Dated change log](CHANGELOG.md) | Project changes and reasons |
| [Iteration 001: initial driving sandbox](iterations/2026-09-11-initial-sandbox.md) | Attempts, failures, fixes, training lineage, and results |
| [Run records](runs/README.md) | Automatic records for future executions |
| [Documentation workflow](workflow.md) | How to keep this folder accurate |
| [Iteration template](templates/iteration.md) | A reusable record for the next experiment |

The initial iteration was reconstructed from the saved files and the setup
conversation. Its [evidence archive](iterations/2026-09-11-evidence/) preserves
the available logs, configurations, and summaries. It does not pretend to be a
complete historical source snapshot. Future runs record their code at startup.

The broader [AV research notebook](../isaacsim-research/research_notes.md) covers
possible future technology choices. Those proposals are not all implemented in
this simulator. Use these implementation documents to understand what runs now.

Start the saved demo from Command Prompt:

```bat
cd /d C:\Users\hudso\Documents\highwaysim
demo.cmd
```

See the [project README](../README.md) for training and evaluation commands.
