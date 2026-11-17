# Shared assistant coordination pointer

The one editable coordination document is `TRAINING_COORDINATION.md` in the
parent directory of Git's common directory. Resolve
`git rev-parse --path-format=absolute --git-common-dir`, take its parent, and
read/update that parent's `TRAINING_COORDINATION.md`. On this workstation it is
`C:\Users\hudso\Documents\highwaysim\TRAINING_COORDINATION.md`.

Read the latest full master before related work, update it at task milestones,
and re-read immediately before scoped edits to avoid overwriting another
assistant's changes. Follow its ownership and completion/failure handoff.
`documentation/shared-training-plan.md` is a publication snapshot, never an
independent editable plan. Do not create a background LLM watcher. This pointer
does not authorize training or interrupt an active run.
