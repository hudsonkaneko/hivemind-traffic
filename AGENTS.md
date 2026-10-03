# Highway Sim workspace scope

Treat `C:\Users\hudso\Documents\highwaysim` as the canonical root for this project.

- Create and modify project code, assets, documentation, logs, checkpoints, and generated outputs only inside this folder.
- Do not use `C:\Users\hudso\Documents\ChatGPT\Highway Sim` as a project root. It is a legacy compatibility path only.
- `C:\isaacsim` and `C:\Users\hudso\Documents\IsaacLab` are external runtime dependencies. Read or execute them when needed, but do not relocate or modify them as part of ordinary Highway Sim work.
- Record meaningful implementation, training, asset, and parameter changes in `documentation/`.
- Put trained policies and evaluation artifacts in `outputs/`, and runtime logs in `logs/`.

## Consolidated workspace (2026-10-02)

- This local folder is the checkout of `https://github.com/hudsonkaneko/hivemind-traffic`.
- `C:\Users\hudso\Documents\expired_highwaysim` contains recovery archives only. Do not run, edit, or publish from those copies.
- The former `Documents\hivemind-traffic` and `Documents\ChatGPT\Highway Sim` locations are retired. Do not recreate competing workspaces there.
- Vehicle preparation belongs in `vehicles/sim_ready/<vehicle-name>/source` and sequential `v01`, `v02`, etc. folders. Keep version-specific documentation, evidence, and logs in their normal top-level directories.
- Preserve historical manifests as evidence; their old absolute paths describe where those runs happened, not the current workspace.
- Keep SUMO/PettingZoo/PPO and Isaac runtime dependencies isolated. Use the portable launcher and documented environments; never install the traffic requirements into Isaac's Python.
- Commit and push tested meaningful changes. Do not commit local configuration, installed runtimes, generated scans, or checkpoints accidentally.
