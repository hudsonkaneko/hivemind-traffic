# Workspace consolidation — October 2, 2026

## One working folder, one GitHub repository

- Active: `C:\Users\hudso\Documents\highwaysim`
- GitHub: `https://github.com/hudsonkaneko/hivemind-traffic`
- Recovery archive: `C:\Users\hudso\Documents\expired_highwaysim`

The local folder and remote repository intentionally have different names.
The user's follow-up explicitly requires ONLY this active directory. `AGENTS.md`
now requires checking the working root before project commands/edits and forbids
using archived implementations for ordinary work. External source assets can be
read/copied into the project; this does not make their source folders workspaces.
Do not create another working copy at the old `Documents/hivemind-traffic` path.
Open this project's chats and terminals in the active folder, not the retired
`Documents/ChatGPT/Highway Sim` compatibility directory.

## Why consolidation was necessary

The old traffic checkout ended at `765d912` on the original main history.
The newer live-sensor branch ended at `1f017d9`, but its first implementation
commit branched directly from scaffold commit `a308dbe`. The newer directory
therefore contained newer demonstrations without the earlier PettingZoo/PPO,
launcher, replay code, and generated results in its working tree. Older commit
objects remained in Git, which is different from having an integrated checkout.

## Recovery copies

`expired_highwaysim` contains:

- `highwaysim/`: the previous canonical workspace, including its uncommitted work.
- `hivemind-traffic/`: the older checkout, its uncommitted launcher changes,
  original environments, PPO checkpoints, and recorded sensor evidence.
- `highwaysim-backup-20260921-161242/`: the existing September 21 backup.
- `legacy-chatgpt-highway-sim/`: the archived compatibility entries. Directory
  junctions now point to archived content, not the active workspace.
- `highwaysim-move-remnants/`: retained partial source from a Windows Move-Item
  interruption in the runtime tree. Its remaining files were copied into the
  complete archived highwaysim before creating the active workspace. Retained
  conservatively; not a separate active project.

The original legacy directory was held open by this chat/application. Its
contents were archived; the empty directory shell may remain until released.
No recovery folder was deleted. Existing external runtime/cache references are
still dependencies, not self-contained runtime backups. Seven runtime junctions
were reconstructed in both the active and archived canonical directories.

## Reconciliation policy

1. Preserve the previous canonical workspace as the active baseline.
2. Restore all missing paths from the older traffic checkout, without replacing
   the newer live-control code with an older draft.
3. Preserve every overlapping older source in the recovery archive and in the
   appropriate Git history. The hash reports distinguish identical, different,
   and missing files rather than treating every difference as data loss.
4. Keep historical recordings/manifests unchanged, including their original
   absolute paths and source hashes. These describe historical runs.
5. Preserve the September 21 backup separately; do not overwrite newer working
   files with that older snapshot.
6. Join both committed histories and publish the consolidated tree to GitHub.

The union restored 5,095 previously absent paths (mostly sensor evidence),
including the older PettingZoo environment, shared PPO, launcher, SUMO scenario
definitions, replay tools, documentation, and trained checkpoints. Twenty older
paths differed at the start; newer versions stayed active. The manifest records
additional intentional consolidation edits as differences too.

The root README now points to both generations of demonstrations. The lightweight
`traffic` package lazily exports its older SUMO backend so sensor utilities do
not require an early TraCI import. The portable launcher recognizes the separate
traffic environment before falling back to `.venv`.

## Environments and portable commands

- `.venv`: retained newer CPU-test environment; no Isaac packages added.
- `.venv-traffic`: preserved original traffic/PettingZoo/PPO environment. Its
  Python environment scripts were regenerated for the new location.
- `.venv-ovrtx`: preserved renderer environment, similarly relocated.
- `.venv-isaac7` and `runtimes/`: retained experimental runtime; still not a
  validated vehicle-physics migration.
- `C:\isaacsim` / `C:\Users\hudso\Documents\IsaacLab`: unchanged external runtimes.

Use `python -m pip` rather than old copied pip.exe entry points. The ignored
`hivemind.local.json` selects relative traffic/renderer Python paths and the
preserved PPO checkpoint. No credentials or private configuration are published.

From the repository root, with the appropriate environment selected:

```text
python -m hivemind demo sumo --check
python -m hivemind demo trained --check
python -m hivemind demo ovrtx --check
python scripts/demo_cooperative_lidar.py --check
```

Remove `--check` to launch. These are separate historical/current demos, not a
claim that the live LiDAR controller is the older trained PPO policy.

The local combined test command is:

```text
.venv-traffic\Scripts\python.exe -m pytest -q
```

New clones must install the documented dependencies; Python environments,
large scans, checkpoints, and outputs remain local/ignored. GitHub consistency
means matching tracked source, documentation, configuration templates, and tests,
not uploading installed runtimes or gigabytes of recorded sensor data.

## Verification evidence

- Copy logs: `logs/consolidation/`; canonical copy and older union reported zero
  failed files. Runtime and traffic environments are kept separate.
- SHA-256 reconciliation reports: `outputs/consolidation/*.json`, generated by
  `scripts/audit_workspace_consolidation.py`. Excludes Git internals, installed
  environments, runtime build trees, Python caches, and one-off move scripts.
- Short scripted and trained SUMO rollouts: `outputs/consolidation/smoke-*`.
- Launcher checks validate configuration/dependency discovery, not GPU behavior.
- A fresh bounded cooperative LiDAR check is recorded separately from historical
  long-duration performance evidence.

### Final local verification

| Archived source | Compared files | Byte-identical | Intentionally different | Missing |
| --- | ---: | ---: | ---: | ---: |
| Previous highwaysim | 26,366 | 26,360 | 6 | 0 |
| Older hivemind-traffic | 5,116 | 5,092 | 24 | 0 |
| September 21 backup | 119 | 114 | 5 | 0 |

All 31,601 comparisons have a corresponding active file. Differences are retained
in the archive, not discarded. In the previous canonical workspace, the six
content changes are the ignore rules, workspace instructions, root README,
documentation index, location guide, and pytest temporary directory setting.
Older checkout differences include the newer controllers and consolidated
launcher configuration. Historical source snapshot line endings were restored
exactly and protected with `.gitattributes` after Git checkout normalization.

- **218 tests passed** in the combined traffic environment after consolidation.
- Scripted and trained-PPO SUMO rollouts each ran for a 50-step budget.
- SUMO, trained PPO, OVRTX, and cooperative launcher setup checks passed.
- Fresh cooperative run: `20261003T003430Z-ideal-e3ccac` (UTC ID; October 2 local).
  Completed 250 steps / 25 simulated seconds, real-time factor 0.999957,
  zero delays over 100 ms, safe obstacle passage, ego complete and peer cruise.
  The independent raw-evidence audit passed. This is a bounded headless check,
  not a new long-duration or GUI performance certification.
- An orphaned console remained after Isaac logged shutdown; its console was
  closed separately. Behavior and evidence passed, but a clean wrapper exit
  was not used as the proof of success.
- Both `765d912` (old main) and `1f017d9` (live LiDAR) are ancestors of the
  consolidation merge, `a6f081e`. Its tree deliberately retains the manually
  reconciled union, rather than applying the old branch's scaffold-era deletions.
- Historical pytest fixtures were preserved; new tests use
  `outputs/pytest-current` instead of overwriting `outputs/pytest-live-lidar`.

Previous repeatability/timing/memory limitations remain. Full physics, OVRTX
rendering, and every historical training run were not rerun in this operation.

## Documentation synchronization

This local consolidation record is authoritative for the folder migration.
The external Google Docs learning guide has not been edited in this operation;
its next update should link this record and replace active-workspace commands
that use the retired local folder names. Do not rewrite historical run evidence.
