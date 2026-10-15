# Development, GitHub, and documentation workflow

Updated: October 3, 2026.

## One working copy, one history

Use only the canonical checkout identified in [locations](locations.md).
Git records source versions; do not make `highwaysim-v2` or similar competing
project copies. The archived folders are recovery material, not active branches.
Sequential vehicle asset folders under `vehicles/sim_ready/<vehicle>/v01`, etc.
remain useful because they identify prepared deliverables and their evidence;
they do not replace Git history.

Keep `main` as the reviewed, usable baseline. Develop a coherent milestone on a
named feature branch, make small meaningful commits, and push tested checkpoints.
Git commits are local snapshots; a push is what synchronizes those commits with
GitHub. An uncommitted file is not backed up by pushing another commit.

## For each change

1. Inspect the branch, remote, and working tree. Preserve unrelated user changes.
   Fetch remote updates before deciding whether a merge is needed; never force
   push or silently reset a dirty checkout.
2. Identify the roadmap step, intended behavior, and acceptance rule before
   running the experiment. Keep the old demo and its evidence available.
   For USD scene/asset changes, apply [scene conventions](../usd/SCENE_STRUCTURE.md),
   inspect both hierarchy and composition, and include path/asset-resolution and
   physics/sensor regression checks appropriate to the change.
3. Implement the smallest useful change with targeted tests. Separate runtime
   environments: traffic packages must not be installed into Isaac's Python.
4. Run CPU/schema tests first, then a bounded simulator smoke test. Run GPU tests
   serially on this workstation so competing jobs do not corrupt timing claims.
5. Save a unique evidence package with resolved settings, source hashes, Git
   commit and dirty state, runtime versions, seeds, metrics, logs, and failures.
   Never reuse a run directory to make a failed attempt disappear.
6. Update the relevant technical note, the [change log](CHANGELOG.md), and active
   roadmap status. Use the [iteration template](templates/iteration.md) for a
   substantial experiment. State what is tested, provisional, blocked, or pending.
7. Inspect the diff, stage only intended files, commit code/tests/docs together,
   and push the feature branch. Do not publish secrets, local runtime paths in
   configuration, installed dependencies, or generated datasets accidentally.
8. Open a pull request with purpose, test commands, actual outcomes, limitations,
   and evidence IDs. Review before merging. A written plan or passing unit test
   alone does not prove a simulator acceptance gate passed.

Portable starting checks, run from the repository root in the configured traffic
environment:

```text
git status --short --branch
git remote -v
python -m pytest -q
git diff --check
```

Check the current milestone's instructions before running any simulation; do not
assume that the traffic Python can import Isaac APIs. See [portable demos](portable-demos.md).

## What belongs in Git

| Commit and review | Keep out of normal Git history |
| --- | --- |
| Source, tests, small reproducible configurations, road fixtures | Virtual environments and installed runtimes |
| Documentation, decisions, evidence summaries, metric definitions | Machine-specific `hivemind.local.json`, credentials, secrets |
| Compact derived result tables with run IDs and source hashes | Bulk LiDAR scans, checkpoints, videos, runtime logs |
| Small, intentionally licensed and reviewed assets | Unreviewed vendor downloads or large binary asset trees |

New raw results go in `outputs/` and logs in `logs/`, both ignored by Git. An
ignored local result is **not** stored on GitHub: retain it locally and arrange an
explicit backup or approved artifact release when sharing is needed. Record an
artifact's location, hash, license, and retrieval requirements; do not claim that
a clone includes it. Preserve already tracked historical evidence.

## Documentation roles

- **Repository:** authoritative implementation, current roadmap, definitions,
  commands, verification status, and compact evidence summaries.
- **Git commits and pull requests:** exact code history and review discussion.
- **Google learning document:** readable explanation of each milestone, purpose,
  technology, commands, and lessons. It mirrors reviewed facts, not a separate
  source of implementation truth.

For each meaningful learning-document update, preserve existing tabs and styles;
add or update the relevant milestone tab with command blocks, key terms, test
outcomes, limitations, and a source commit or pull-request link. Do not overwrite
historical results with new results. Verify the native document after the edit
and record the sync outcome. If access or formatting verification fails, report
that the repository is updated but the document is pending; do not claim success.

## Status vocabulary

**Planned** means no implementation claim. **In progress** means work exists but
the exit gate is not satisfied. **Verified** means a named, reproducible evidence
package passed the declared gate on the recorded setup. **Blocked** names a
specific unmet prerequisite. Record numerical tolerances and scope: a 3 m/s
single-car pass is not highway-speed, multi-agent, or real-world validation.
