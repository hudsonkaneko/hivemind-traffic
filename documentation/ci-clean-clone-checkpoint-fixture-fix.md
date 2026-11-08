# PR 60 clean-clone test repair

November 8, 2026. Test-only maintenance; no policy or training changes.

The [failed GitHub run](https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/42499248164)
on commit `a6803b22eaf81191fc538a824d223724f07050e9` passed dependency setup and
the real SUMO smoke tests. The full suite recorded 4,102 passes, 237 skips,
162 passing subtests and one failure. The smooth-training initialization test
required an ignored local `outputs/highway-environment/v05/.../last.pt` file
that is absent from a clean checkout. The test predates PR 60; documentation
commits also rerun the full suite and expose that same missing dependency.

The test now creates a deterministic untrained checkpoint and matching exported
actor in an isolated temporary directory. Only test fixture identities are
substituted; production configuration validation, checksums, real model loading
and export parity remain active. It additionally verifies actor import, fresh
critic initialization and rejection of corrupted actor/checkpoint files. The
module creates its ignored output parent so it also works independently of
other tests. Optimizer steps remain prohibited.

Validation: 67 tests passed locally (18.78 s) and in a clean checkout with no
local training checkpoint (18.37 s). Both report seven Torch deprecation warnings.
Logs: `logs/ci-pr60-fixture-fix-02.log` and `logs/ci-pr60-clean-clone-01.log`;
JUnit reports: `outputs/ci-pr60-fixture-fix-02/tests.xml` and
`outputs/ci-pr60-clean-clone-evidence-01/tests.xml`.
The first local invocation had seven setup errors because its custom temporary
parent was missing; that harness error and log are retained separately.

GitHub verified repair commit `e9b718638c9377b6efe71f7a5d432683bdcf6e6c`:
[PR run](https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/42527724702)
and [push run](https://github.com/hudsonkaneko/hivemind-traffic/actions/runs/42527721537)
both succeeded. The PR run reports 4,105 passed, 237 skipped, 162 passing subtests
and 65 deprecation warnings, with no failures (382.81 s for the full CPU suite).
This result applies to that exact repair commit; later metadata or workflow edits
have their own CI status.
