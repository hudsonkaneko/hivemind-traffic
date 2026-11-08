# Branch naming cleanup

November 8, 2026. The user requested removal of the `codex/` prefix, then explicitly
approved replacing affected open PRs. All 62 repository branches were renamed
without the prefix, with identical commit IDs verified at the time of rename.
All matching local branch names and upstreams were updated. No prefixed local or
remote branches remain in this repository. The default branch remains `main`
at its original commit; no branch was merged or its work discarded.

GitHub closes an open PR when its head branch is renamed. After saving full PR
metadata, 48 affected PRs were replaced, preserving titles, descriptions, draft
status and base/head commit relationships. Each replacement links to its original
PR, where comments and reviews remain available. Those historical reviews are not
transferred as approvals of the replacements. Labels, assignees and pending review
requests were empty on the migrated PRs. No review requests or comments were sent.

One of the original 49 open PRs, #16, belongs to a teammate's fork and remains
open unchanged. The initial inventory counted all 49 open PRs; the full metadata
check established that only 48 needed replacement. There are still 49 open PRs.

The active branch is `mappo-flow-rl-v02`, and
[PR #108](https://github.com/hudsonkaneko/hivemind-traffic/pull/108) replaces #60.
[Complete branch and PR mapping](evidence/branch-naming-cleanup/migration.json)
contains every original/new name, preserved commit and replacement PR URL.
Full before/after API snapshots are retained in `outputs/branch-prefix-removal-01/`.

AGENTS.md and the workflow guide record descriptive names without an agent prefix.
The current branch's push-trigger pattern changes from `main`/`codex/**` to all
branch names, retaining exactly the same test jobs. Older branches preserve their
historical workflow definitions and documentation; this rename does not rewrite
historical code to make old checks pass. Replacement PRs can start new CI runs.

Separately, [the missing-checkpoint test repair](ci-clean-clone-checkpoint-fixture-fix.md)
passed both GitHub push and PR CI at commit e9b7186: 4,105 tests passed in the PR
full suite. Later branch/workflow metadata commits have their own CI status.

[GitHub's rename behavior](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-branches-in-your-repository/renaming-a-branch)
explains the automatic PR closure and URL redirection. Collaborators should fetch
and update local branch/upstream names. No training or policy changes occurred.
