# Main integration checkpoint — October 14, 2026

The user requested merging all committed project work into `main`. This checkpoint
joins the published feature histories without changing simulation behavior or
publishing private local assets. [PR 10](https://github.com/hudsonkaneko/hivemind-traffic/pull/10)
records the final GitHub merge outcome; the earlier PRs retain their scoped reviews.

## Scope and review

- Reviewed the ten open PR descriptions, validation evidence, limitations, branch
  ancestry and checks. GitHub reported no status-check runs or review decisions;
  local regression results below are not GitHub CI or an independent approval.
- The showcase head `9d572d2` already contains the physics, demo, scene-convention,
  physical-scene, 35 mph, continuous-loop, sidebar and skill milestones.
- Joined the separate collector-v02 head `aab4cf1` with a merge commit rather than
  squashing away its provenance. Its road/code assets were already byte-identical
  in the showcase branch. Only changelog and roadmap history conflicted; both
  sides' public entries were retained. No simulation source or assets changed.
- Keep original feature branches and commits. Preserve existing historical
  experiments, reports, failed runs, local-only vehicle files and installed skills.
- Uncommitted documentation was recoverably stashed before resolving conflicts.
  Its private vehicle notes are excluded from GitHub and restored locally after
  promotion. Public collector entries belong to the merged tracked history.

## Fresh checks on the integrated source tree

| Check | Result |
| --- | --- |
| Full traffic-Python pytest suite | 1,599 passed, 180 optional/runtime skips, 34 subtests passed |
| USD fleet/pool/view/loop-scene unittest suites | 134 passed using existing USD 26.8 environment |
| Collector-v02 route unittest suite | 4 passed |
| v09 showcase launcher `--check` | Passed configuration/runtime/vehicle availability preflight |
| Merge index / whitespace | No unresolved entries; `git diff --cached --check` passed |

The full suite used a unique temporary directory:

```text
python -m pytest -q --basetemp=outputs/pytest-main-20261014 --junitxml=outputs/main-integration-20261014-tests.xml
python -m unittest discover -s highway_usd/_v02 -p test_routes.py -v
python scripts/demo_loop_showcase.py --version v09 --check
```

The USD environment's built-in unittest runner exercised `test_showcase_fleet`,
`test_live_showcase_fleet`, `test_loop_showcase_view`, `test_highway_loop_view`
and `test_highway_loop_scene`. It remains isolated from traffic Python.

Initial command failures were test setup, not masked code failures: a nested
pytest temporary directory lacked its parent (215 setup errors), and an extra
pytest-style module was initially passed to the USD environment without pytest
installed (one import error). Corrected invocation paths/suite selection passed;
no dependencies or tests were changed. The pytest-style module remains covered
by the full general suite, with USD-only cases explicitly skipped there.

## Acceptance is not expanded by merging

No fresh Isaac GUI/physics benchmark or policy training ran for this Git-only
integration. Existing runtime evidence remains scoped to its recorded source and
configuration. In particular, strict smoothness/60 FPS, standalone 35 mph bypass
qualification, shared-map hybrid traffic, learned coordination and outstanding
scene-lifecycle checks are not made complete by changing the default branch.
Private assets/checkpoints/runtimes are still separate clone prerequisites.
