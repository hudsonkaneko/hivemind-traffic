# Highway Sim assistant coordination

Follow `AGENTS.md` for project scope and user preferences. Before training,
evaluation or overlapping changes, read `TRAINING_COORDINATION.md` and its shared
plan. Preserve any active run and unrelated working files. Record your actual
branch/run and acknowledge the handoff before taking a shared task. This file
does not authorize training or override user instructions.

## Single shared coordination source

The one editable coordination document is `TRAINING_COORDINATION.md` in the
parent directory of Git's common directory. Resolve
`git rev-parse --path-format=absolute --git-common-dir`, take its parent, and
read/update that parent's `TRAINING_COORDINATION.md`. On this workstation it is
`C:\Users\hudso\Documents\highwaysim\TRAINING_COORDINATION.md`.

Read the latest full master before adopting related work; update that same file
at task milestones. Re-read immediately before scoped edits and reconcile any
new entries instead of overwriting them. Follow its branch/worktree ownership
and completion/failure-only handoff. `documentation/shared-training-plan.md` is
only a publication snapshot and must not be independently edited. Refresh the
published snapshot from the master through the assigned documentation worktree.
No background LLM watcher or training is authorized by these pointers.
