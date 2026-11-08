# Open-PR clean-clone failure audit

November 8, 2026. Recreating historical PRs during branch renaming retriggered
checks on their preserved old code. The earlier green result for repair e9b7186
applied to the active MAPPO branch, not all historical branches. The migration
exposed existing failures rather than invalidating that specific green run.

The first complete PR audit found 11 PRs with failure checks, 21 pending, 16
passing and one fork PR without checks. Ten branches shared the same missing
local checkpoint dependency in the smooth initialization unit test. PR #79's
semantic/LiDAR tests instead failed collection because PIL was not installed.
All ten failure logs were inspected, not inferred from matching PR titles.

## Repairs

- Backport the real synthetic checkpoint/export fixture to all ten affected
  branches. Preserve validation, checksums, loading, parity, actor-copy/fresh-critic
  and no-optimizer-step tests; add corruption assertions. No test is disabled.
- Add Pillow==11.3.0 to the semantic fusion branch's CI requirements, matching
  its successor's existing pinned dependency. No dependency was installed into
  Isaac. The isolated traffic Python received Pillow for local verification.
- Each repair is a new fast-forward commit touching only its test/dependency
  file and a concise documentation note. No policy weights, observations,
  rewards, simulator parameters or training configuration change.

The affected test, runner, contract and model files had identical Git blob IDs
across the ten checkpoint failures. The revised 67-test module passed on the
oldest affected checkout (18.77 s), in addition to the previously verified latest
branch. The missing Pillow failure was reproduced locally; after installing the
pinned dependency, seven semantic evaluation tests passed (0.30 s), and the
entire semantic branch collected 3,169 tests without errors (4.86 s).

Both failure and successful local logs are retained under `logs/ci-pr-audit-*`;
JUnit and GitHub snapshots are under `outputs/ci-pr-audit-01/`. These local
checks do not replace each branch's full GitHub run. Verify the exact current
head and pending/failure/success status before reporting a PR green.

| PR | Branch | Repair commit |
| --- | --- | --- |
| [#79](https://github.com/hudsonkaneko/hivemind-traffic/pull/79) | semantic-lidar-fusion | `473af314500b60ede6c885d36aef6baa55120edf` |
| [#98](https://github.com/hudsonkaneko/hivemind-traffic/pull/98) | highway-smooth-preparation-v01 | `a81f0b2eef93915e850bffe42534abf0d9b714af` |
| [#99](https://github.com/hudsonkaneko/hivemind-traffic/pull/99) | highway-training-rethink-v01 | `3f82d5c06adf308cca2c1a4914186e7036e6aecb` |
| [#100](https://github.com/hudsonkaneko/hivemind-traffic/pull/100) | highway-flow-preparation-v01 | `e6504b60798e463c3eaddf4990102771a393a1af` |
| [#101](https://github.com/hudsonkaneko/hivemind-traffic/pull/101) | gpudrive-i110-pretrained-v01 | `2a3a86aa44a2ab193ddb492f33e860d892c43a52` |
| [#102](https://github.com/hudsonkaneko/hivemind-traffic/pull/102) | gpudrive-i110-shielded-v02 | `e2e296477f103bed43edcece36cc3904113caecd` |
| [#103](https://github.com/hudsonkaneko/hivemind-traffic/pull/103) | gpudrive-continuous-v03 | `9b99da4079591b77d8da1d9381e8ba2c635f058a` |
| [#104](https://github.com/hudsonkaneko/hivemind-traffic/pull/104) | gpudrive-route-v04 | `bc761c765db97df8adbe5c029a5f213d8123066e` |
| [#105](https://github.com/hudsonkaneko/hivemind-traffic/pull/105) | gpudrive-smooth-v05 | `1963e83d316633b101a71e4c17e541374aab2fef` |
| [#106](https://github.com/hudsonkaneko/hivemind-traffic/pull/106) | mappo-gpudrive-plan-v01 | `bbadbf0d91ce4b7e594316d4e3a2a2ab8caf103c` |
| [#107](https://github.com/hudsonkaneko/hivemind-traffic/pull/107) | mappo-flow-rl-v01 | `6692a76b26587c0dc1d285639de63b52a1e23f1e` |

[Inspected failure evidence](evidence/ci-open-pr-audit-2026-11-08/failure-evidence.json) |
[Exact backport/file inventory](evidence/ci-open-pr-audit-2026-11-08/backports.json).

Validation scope is all 48 repository-owned replacement PRs. Teammate fork
PR #16 has no CI checks and is reported separately, not called passing.
Old failed runs/emails remain historical records; only a completed successful
check on the current head establishes that a repair passed.
