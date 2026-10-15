# Highway Sim workspace scope

Treat `C:\Users\hudso\Documents\highwaysim` as the canonical root for this project.

**ONLY use this directory as the active project workspace.** Before any project
command or edit, verify the working directory resolves to this root or one of
its children. If a chat opens in a legacy folder, explicitly select this root
instead; do not infer the working folder from the GitHub repository name.
This rule supersedes older project-location notes. Do not read recovery archives
as current implementation or use them for new work; consult them only for an
explicit recovery/audit task. User-provided source assets may be read/copied from
elsewhere, but all prepared assets and project outputs belong under this root.

- Create and modify project code, assets, documentation, logs, checkpoints, and generated outputs only inside this folder.
- Do not use `C:\Users\hudso\Documents\ChatGPT\Highway Sim` as a project root. It is retired, not an active compatibility workspace.
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

## Development continuity

- Follow `documentation/workflow.md`: feature branches, code/tests/documentation
  together, tested GitHub checkpoints, and pull-request review before merging.
- The active plan is `documentation/hybrid-roadmap.md`. Older roadmap text is
  historical; preserve experiment evidence rather than rewriting its meaning.
- Update `documentation/CHANGELOG.md` and the relevant evidence/learning note for
  each meaningful milestone. Keep the Google engineering guide aligned when
  authorized and connected; report sync status honestly.
- Preserve the working SUMO-motion demos. The new physical-car foundation is
  separate: `documentation/physics-vehicle-foundation.md` states what is verified
  and which reset, sensing, training and hybrid-integration gates remain open.
- User preference (2026-10-04): involve the user directly when reaching Isaac Lab
  policy training. Explain and agree on observations, actions, rewards, algorithm,
  training budget and evaluation before launching training. Scripted controllers,
  environment plumbing and bounded validation may proceed now; do not interpret
  that authorization as permission to begin unattended training.

## Required OpenUSD authoring discipline

User requirement (2026-10-06): always maintain a clean, clearly named scene
hierarchy and standard OpenUSD/Omniverse composition workflows, including during
prototyping. Follow `usd/SCENE_STRUCTURE.md` before creating or changing stages,
asset generators, importers, exporters, vehicle packages, or viewer scenes.

- Separate Environment, Vehicles, Lighting, Cameras, Physics and Debug concerns.
  Use stable vehicle identities and descriptive component names; avoid anonymous
  mesh dumps, incidental creation-order names, and scattered hardcoded prim paths.
- Reference reusable vehicle/environment/prop assets. Use payloads for heavy,
  selectively loadable content, not indiscriminately on every small prim.
- Keep asset geometry/materials, physics/sensor configuration, scene assembly,
  and live/replay/debug opinions in appropriate separate layers. Keep published
  sources immutable during simulation. Preserve composition in authored stages;
  flattened diagnostic snapshots are explicitly labeled exceptions.
- Use valid defaultPrim, units/axes, model kinds and portable asset paths. Do not
  blindly instance mutable physics, wheel, joint or sensor hierarchies. Required
  collision and sensing content must stay loaded while a simulation is active.
- Existing prim paths are integration contracts: migrate relationships, runtime
  handles, sensor mounts, ID mappings and tests together. Do not rearrange prims
  in a running physics stage or rewrite historical results for cosmetic cleanup.
- Inspect the composed hierarchy AND layer/composition structure before calling
  a scene finished; add appropriate structure/regression checks with each scene
  implementation. Document runtime-owned namespace exceptions explicitly.
