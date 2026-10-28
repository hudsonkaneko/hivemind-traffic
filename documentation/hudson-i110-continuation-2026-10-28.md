# Hudson actor and I-110 continuation handoff

28 October 2026 · Codex continuation agent for Sam · local work, not published.

## Question and authorized next work

Sam forwarded Hudson's checkpoint confirmation and asked a new agent to continue
the existing work and add it to the handoff. The current task is to make the
original frozen actor reproducible, then define a defensible I-110 comparison.
Implementation of an inference/evaluation harness and bounded engineering checks
is authorized. Training, shared Google Doc writes, teammate messages and publishing
unreviewed local commits remain outside this tranche.

The active checkout is `research/digital-twins/repository-review/source` below the
research root, branch `codex/sam-visual-twin`. Start from local `b6f6837` plus this
dated continuation commit. The branch already had six unpublished commits; do not
reset it, silently replace it with upstream, or assume a GitHub clone has raw runs.

## Hudson's confirmation: what is and is not established

The user-forwarded maintainer message confirms that the frozen actor still exists
on Hudson's machine; no retraining is needed. It has not been delivered here.
The public GitHub release API returned zero releases on 28 October. The linked
published evidence was fetched and is retained in
`outputs/hudson-continuation-20261028/published-latest.json`.
The complete 733-entry referenced branch tree contains no checkpoint extension;
all 70 listed Actions artifacts are named `windows-py311-cpu-sumo`. This session
inspected artifact metadata, not those archive contents. No checkpoint exists in
the local outputs search. This establishes the inspected scope, not every private
file or branch on Hudson's machine.

| Item | Identity / meaning |
|---|---|
| Run | `train-20261026T190003Z-9d133a8c` |
| Artifact | `outputs/sumo_mappo/v01/train-20261026T190003Z-9d133a8c/policy.pt` |
| SHA256 | `6ec2167aa2c769ec41c9221b77382fe74658430ce852703ce32568d089e591fd` |
| Source commit | `c06e45eaa79698eda85d01a65fa6ae5a6811b6d7` |
| Report commit | `d12c168c72c3a7441fb274af3f483de1609dac19` |
| Original environment | Three lanes, 5 km, 0.1 s step, 600-step episode, exact SUMO state |
| Actor contract | float32 `[N,27]` → normalized acceleration `[N,1]`, in `[-1,1]` |
| Physical action | `-4.5 + (action + 1) * 3.25` m/s² |
| Controls absent | No steering or communication inputs; external safety filters still required |

Hudson reports Python 3.11.9, Torch 2.14.0+cpu, NumPy 2.4.6, Gymnasium 1.3.0,
PettingZoo 1.27.0 and SUMO/TraCI 1.27.1. Sam's runtime has Python 3.11.16 and
Torch 2.14.0+cu132; CPU inference does not make those builds identical. Record
actual versions and numerical differences rather than claiming bitwise parity.

Published aggregate evidence reports 20/20 nominal held-out episodes, no measured
collisions/overspeed/external safety interventions, and mean progress 1,445.45 m.
Nominal episode endings at 60 seconds are **not completion of the 5 km route**.
Zero external override fraction does not prove SUMO internals never intervened.
These remain reported results until the actor is independently evaluated here.
The original native comparison is not automatically Sam's newly tuned ACC.

Hudson confirms no newer accepted MAPPO checkpoint; the communication/wave setup
is implemented but untrained. The older shared-document 42-observation physical
PPO report refers to a different actor and must not be merged into this contract.

## Acquisition and reproduction sequence

1. Obtain the exact `policy.pt` as a repository release asset or an explicitly
   supplied local file. Also request original `held-out-learned/evaluation.json`
   (20 per-seed records), the resolved manifest/configuration and, if available,
   action journals. Optimizer checkpoints are unnecessary for frozen inference.
2. Hash bytes before loading and reconstruct the 14 recorded source files from
   the pinned Git commit, including verified CRLF restoration where necessary.
   Verify the normalized v02 configuration digest. Wrong/missing files must stop
   actor evaluation; never substitute random weights or override the expected hash.
3. Run the original three-lane environment on preflight seed 520001 twice with
   the actor frozen, preserving original safety filters, feature order/scaling,
   deterministic CPU execution and one Torch thread. Require finite contract-valid
   inputs/actions, clean lifecycle and repeat metrics; retain the action journals.
4. Run the fixed original test seeds 410001–410020 without tuning on them. Compare
   per-seed outputs with Hudson's actual evaluation file when supplied. Keep
   threshold acceptance and numerical reproduction as separate results.
5. Only then review the I-110 transfer protocol below. A successful reproduction
   is not I-110 acceptance, evidence of traffic improvement or authorization to train.

`scripts/evaluate_hudson_actor.py` provides explicit check/native-check/evaluate
modes. It reconstructs an isolated source bundle and calls the original v02
`evaluate()` function, using a frozen actor shim instead of constructing an
actor/critic or optimizer. The v01 recorder is not used as an exact v02 evaluator.

```text
python -m scripts.evaluate_hudson_actor --output outputs/hudson-identity-new --checkpoint PATH_TO_POLICY
python -m scripts.evaluate_hudson_actor --mode native-check --output outputs/hudson-native-scaffold-new
python -m scripts.evaluate_hudson_actor --mode evaluate --suite smoke --checkpoint PATH_TO_POLICY --output outputs/hudson-smoke-new
python -m scripts.evaluate_hudson_actor --mode evaluate --suite held-out --wall-seconds 1800 --checkpoint PATH_TO_POLICY --output outputs/hudson-held-out-new
```

Use the research root's `.venv/Scripts/python.exe`, not Isaac Python. Every output
directory must be new. Default `check` never deserializes the actor. Native-check
uses only development seeds and verifies plumbing, not learned driving. The
original evaluation reports progress/reward over a fixed horizon. Acceptance uses
collision, overspeed, progress, intervention and nominal-end thresholds; the
separate I-110 runner retains whole-cohort boundary/drain accounting.
Actor threshold failure produces `evaluated_acceptance_failed` and a nonzero exit.
Process completion alone is not acceptance; even threshold acceptance does not
establish numerical reproduction against the unavailable per-seed reference.

## I-110 transfer and all remaining work

| Track | Verified starting point | Next bounded action and acceptance boundary |
|---|---|---|
| Original actor | All 14 sources and v02 config recovered; weights absent here | Exact bytes, original-fixture smoke/repeat, then fixed 20-seed evaluation; no claim before actual actor runs |
| I-110 policy interface | 30-feature `i110_graph_v1`, connected-path neighbors, stable lanes and masks | Write an explicit old/new feature and information-access comparison; do not pad/truncate 30 to 27. Agree whether a restricted supported layout is meaningful or new training is required |
| Ordinary ACC | Seven smooth runs passed the disclosed 2.05 m/s³ engineering jerk bound | Predeclare cut-in, hard-braking and ramp-conflict fixtures; keep safety authority, log requested/realized controls and interventions; a safety stop may override comfort |
| Road fidelity | Imagery-refined MLK–Vernon geometry and bounded surrogate tire support | Independent lane/taper/current-epoch review and uncertainty register; preserve source epochs and bridge-occlusion assumptions |
| Bottleneck layer | Separate construction, disabled-car and police-break viewer presets; separate SUMO leader-speed event | Define matching SUMO event adapters with stable event IDs, onset/duration/extent and requested/realized effects; do not infer traffic effects from USD visibility |
| Traffic calibration | Synthetic demand and complete boundary accounting | Optional PeMS station metadata/health and bounded offline exports; PeMS is not a runtime dependency; no credentials in chat |
| Matched comparison | Original policy and I-110 ACC currently have different domains | Freeze shared network/demand/seed/horizon, information access and safety accounting before comparing; report every external request including ramp demand and unfinished trips |
| USD/physics integration | Refined road and scripted surrogate contact, separate traffic traces | After traffic acceptance, map recorded states into existing layered scene; selected physical checks remain bounded, not a city-scale solver replacement |
| Research validity | Approved design and open protocol map preserved | P01–P05/P07 and remaining complete protocol fields remain open; no communication benefit, empirical site admission or team-wide adoption inferred |
| Organization | Installed evidence skill and catalog, historical gaps visible | Link each new result/graphic to trigger, exact inputs/generator and limits; retain failures, refresh catalog and local continuity every session |

## Resume prompt for the next agent

Read PROJECT_REFERENCE.md, both AGENTS.md scopes, the research-custodian and
organize-av-evidence skills, and this handoff. Use the existing checkout/branch
without resetting local commits. Check working-tree state first. Treat Hudson's
28 October confirmation as maintainer-reported evidence; verify any supplied
actor hash before loading. Continue the frozen v02 reproduction sequence above.
If weights are still absent, complete independent ACC stress-test/event-adapter
design and graph-transfer tests; do not replace the missing actor with a fabricated
result. Keep the 27-feature original and 30-feature I-110 contracts separate.
No training or publication is authorized by this resume prompt. Preserve the
shared Doc hold, local result paths and independent factual-audit boundaries.
At close, record actual tests/results and pending gates, update the evidence
registry and local handoff, and leave a coherent tested local checkpoint.

## Session evidence

Ten targeted tests pass: missing/wrong actor rejection before a child launch,
hash-exact source/config reconstruction, unchanged output protection, held-out seed
protection for native checks, and observation/action shape, dtype and finite bounds.
The initial two failures were caused by a test mock blocking the required read-only
Git commands along with the evaluation child; corrected tests and the first failed
XML are both retained.

The native scaffold ran two 600-step episodes at development seed 520001 with
identical episode metrics and clean process exits. It recorded zero collisions,
overspeed and external safety overrides, mean controlled progress 1,593.039 m,
and passed the original native threshold check. All 14 source hashes remained
unchanged. This verifies the original environment/evaluator plumbing only; **no
learned actor was loaded or evaluated**. A separate missing-file CLI invocation
returned `blocked_missing_or_wrong_checkpoint` without loading a model.

The first scaffold failed because the original backend requires output inside its
own root. The wrapper now uses `source/outputs/` within the isolated bundle and
preserves the original code byte-for-byte. `native-scaffold-v1` remains as failed
evidence; `native-scaffold-v2` contains the successful repeat, logs, actions,
runtime, source snapshots and hashes. No SUMO process remained after verification.

See [compact evidence](evidence/hudson-continuation-20261028.json),
[the refreshed local catalog](../outputs/evidence-catalog/20261028-hudson/index.html),
and the raw `outputs/hudson-continuation-20261028/` package. No new figure was needed
for this plumbing result; the catalog links the result records to this trigger.

Parent successfully read the live master and Sam refresher on 28 October; its last
modification remains 22 October 2026. Current direct Sam decisions supersede its
historical exiD next step. Shared Google Doc writes remain held.


## Later same-day availability recheck

28 October 2026 fresh public GitHub check: zero releases, no model/checkpoint file in all 32 current branch-head trees (including the newly fetched MAPPO 3D replay branch), and 70 CI-named Actions artifacts. Archive contents/private drafts were not inspected. Exact policy.pt remains unavailable here; no model loaded or training. Next: obtain policy.pt and original per-seed evaluation.json from Hudson.

Evidence: `outputs/github-policy-check-20261028-a/result-v2.json` and retained raw API metadata. The initial summary had a PowerShell array-enumeration error; it is superseded by the individually checked branch-tree result.
