# Checkpoint and curriculum audit — read-only

This audit read existing local weights, contracts, configurations, results and source history. It ran no simulation, teacher rollout or optimizer, and edited no project source. CPU inference used twelve hand-constructed empty-cruise inputs per cataloged actor; these are numerical probes, not driving tests. The companion JSON records exact actor/configuration/checkpoint paths and full SHA-256 values, tensor shapes/statistics, and 70 local run weight files. Seven selected full checkpoints received numerical inspection; hashing the remaining files does not imply behavioral testing.

The weights are intact. The later history shows repeated regressions and a tradeoff between task completion and intervention rates. It does not support replacing the accepted old fixture with the newest actor or treating the latest scripted reference as a successful learned policy.

## Which baseline to keep

- **MAPPO v04r2:** preserve the accepted baseline for its original straight three-lane, four-car following/passing fixture. Its 81-field semantics are incompatible with the current 99-field environment. Old acceptance does not certify modern merge, population or comfort gates.
- **Stage 1 v03 phase 0:** preserve the strongest tested parent for intervention rates in the matched population comparisons. It is an unaccepted 93-field research baseline, not a universal winner.
- **Environment v05 `fd639…`:** preserve the existing 99-field merge-progress candidate. It is unaccepted: learning improved stable merges but worsened pooled interventions in every task family. Compare it against the exact migrated v05 initial phase-0 actor under the same corrected/new contract **before BC or optimization**. Existing results do not determine a universal winner.

The practical existing 99-field candidate is v05 final, but choosing it unconditionally would discard evidence that its initial actor had fewer interventions. Choosing phase 0 from old unmatched 93-field numbers would be equally unjustified. Any extra observation fields require explicit, identical migrations for both candidates and a fresh matched comparison.

## Inventory and compatibility

| Version | Actor / critic contract | Selected actor SHA-256 prefix | Status |
| --- | --- | --- | --- |
| MAPPO v04r2 | 81 = 33 local + 48 message fields; critic 328, four cars | `4cf4898df997a444` | Accepted only in original straight-road fixture |
| Stage 1 v02 | 93 = 45 local + 48 messages; critic 376, four cars | `07ef91a2e7591bec` | Experimental, eight of twelve validation cases fail |
| Stage 1 v03 final | Same 93/376 contract | `a6d59d3acf58fcca` | Experimental, five of twelve cases fail |
| Stage 1 v03 phase 0 | Same 93/376 contract | `8526f6134b7eae90` | Retained comparison/initialization reference, not accepted |
| Population v01 | 93-field actor; masked critic 752, eight slots | `8d2e8793072e8a57` | Regressed, fifteen of sixteen cases fail |
| Population v02 | Same population contract | `680d50cebf5f943c` | Regressed, twelve of sixteen cases fail |
| Environment v05 | 99 = 45 local + six own-motion + 48 messages; critic 2400, 24 slots | `fd639c39f0c51425` | Rejected, nineteen of twenty-four cases fail |

Each actor is `policies/versions/<version>/policy.pt`; full exact hashes and selected full-resume paths are in the companion JSON. All seven exported actors exactly match all ten corresponding actor tensors in their selected full checkpoints. All inspected model tensors are finite.

Input width is not semantic compatibility. The original 81-field MAPPO uses elapsed time/cooperative-leader/gap fields where later graph contracts place lane count/end distance/merge state. The earlier I-110 graph pilot also has 81 inputs but is not the original contract. Its files are inventoried; its internals were not part of the seven cataloged numerical inspections. Its run summary's `passed=true` records execution completion, not accepted driving quality.

Population v03 and v04 are **preparation versions**, not separately trained actors. Both configurations have `training_authorized=false`; each retains one identical `frozen-parent-contract.pt` preflight export, full SHA `b6605cccb2ac0c24ebbe7dd44f46140e22f43083c06596867ef8e92815760947`. V03 revises guard/entry target semantics; v04 adds near-limit loss and a speed curriculum. Neither is the original accepted MAPPO v04r2.

V05 explicitly inserts six zero actor-input columns into the phase-0 actor and initializes a fresh critic/optimizer. Continuous lane changes, 24 slots and eight-second stable merges alter the task. Ordinary resume across these critic/semantic changes would be invalid. Population-v02's actual run is under `outputs/highway-population/v01/train-20261105T045039Z-a6db8f1b`; directory names alone are misleading.

The current smooth reference is scripted v4. V5 adjacent preview was rejected and v4 restored. There is no newly trained smooth actor. Four of six v4 fixtures still fail peak-jerk comfort, so it is not a validated strong-imitation target.

## What the weights show and do not show

The shared architecture uses a 16-unit message encoder, two 128-unit actor layers, and a four-row head: one acceleration mean and three lane logits. The first dense layer has 50, 62 or 68 inputs after message pooling for the 81-, 93- or 99-field contracts. Gaussian log standard deviation is separate from the deterministic export. Fields have defined scaling, not a learned running observation normalizer.

Frozen TorchScript exports inline constants and therefore have empty `state_dict()` results; this is expected. The original MAPPO checkpoint loaded with `weights_only=True`. Six later trusted, locally generated full resumes required ordinary loading for saved NumPy RNG/container metadata. No downloaded checkpoint was loaded.

Saved log standard deviations are: v04r2 −2.50910; Stage 1 v02 −2.08614; Stage 1 v03 final −2.07726; phase 0 −1.99893; population v01 −1.99973; population v02 −1.99322; environment v05 −2.04325. Actor norms remain around 21. Parameter changes of L2 0.46–1.01 from phase 0 are not quality metrics and do not prove forgetting.

None of the twelve narrow synthetic probes per actor produced normalized acceleration magnitude at least 0.95. This rules out blanket head saturation on that grid only. It is not an activation study in traffic. V05 tapers near the speed limit on the grid, consistent with saved near-limit improvement; real following/merge-context errors remain.

## Comparable result history

Percentages are pooled raw interventions **within each named protocol**. Protocols differ in roads, traffic, definitions and denominators; they cannot be ranked as one league table.

| Matched protocol | Earlier / parent | Later candidate | Interpretation |
| --- | --- | --- | --- |
| Original MAPPO | Selected v04r2: 3.175% on fifty held-out episodes | Final training update: 9.2708% validation | Keep selected update 175 rather than latest. Held-out and final validation are distinct sets. |
| Stage 1 v02 fixed twelve cases | Phase 2: 12.47% | Selected final: 18.1475%, eight failures | Selection was not minimum pooled interventions. |
| Stage 1 v03 fixed twelve cases | Phase 0: 11.34%, worst 32.63%, six failures | Final: 14.9687%, worst 51.4688%, five failures | Ranking favored one fewer failed case despite worse pooled/worst rates. |
| Population fixed sixteen cases | Phase-0 parent: 20.6393%, worst 44.875%, fifteen failures | V01: 31.7013%, worst 75.125%, fifteen failures | Same-protocol regression. |
| Population fixed sixteen cases | Same phase-0 parent | V02: 29.9716%, worst 86.75%, twelve failures | Acceleration imitation weight fifty did not solve the regression. |
| Environment v05 fixed twenty-four cases | Migrated parent: 13.3749%, two of nine stable merges, 55 passes | Final: 19.8287%, eight of nine stable merges, 59 passes | Task-progress/intervention tradeoff; no held-out promotion. |

Original v04r2 completed 74 passes, at least one per held-out episode, with zero collisions, overspeed or required-gap deficits and minimum headway 1.638 s. Its recorded peak jerk was **42.01 m/s³**: original acceptance does not establish the current comfort criterion. Both Stage 1 finals completed all six merge opportunities and twelve passes despite failing intervention gates.

V05 per-task interventions changed: cruise **11.1562→13.6870%**, following **15.0162→26.2364%**, passing **15.3459→22.9498%**, merge **12.0043→16.5314%**. Conversely, near-limit raw-request RMSE improved **0.22746→0.11733 m/s²**, positive requests at the legal limit fell **37.06→0.85%**, and near-ceiling excess requests fell **853→3** on their recorded respective denominators. This is not blanket disappearance of cruise behavior.

Those v05 evaluations used the original I-110 junction defect. Correcting false lane conflicts with unchanged final weights improved the one retained I-110 merge from 30.53% to **244/2864 = 8.5196%** and restored stable completion. This is an environment repair, not learning. There is no full twenty-four-case corrected-network initial/final comparison.

## Actual objectives and progression

Accepted MAPPO v04r2 used actor LR 3e-5, critic LR 3e-4, imitation coefficient 3, PPO lane-loss weight 2, entropy 0.0003, and sixty initial BC epochs. Signed rewards included progress +1, headway −0.5, jerk −0.02, intervention −1, cruise excess −2, passing +1 and lane change −0.5. These coefficients belong to the original formulas; copying them does not preserve effective strength in a newer environment.

Later Stage 1/population/v05 configurations generally use actor LR 1e-4, critic LR 3e-4, four PPO epochs, minibatch 256, clip 0.2, target KL 0.02, entropy 0.001 and global gradient cap 0.5. Stage 1 v03 used twelve BC epochs with acceleration weight four, followed by teacher coefficient 0.05→0.01 over 400k decisions. Population used coefficient 0.04, lane weight ten and acceleration weight one; v02 raised acceleration weight to fifty. V05 returned it to one and added near-limit coefficient 0.1 within 1 m/s of the limit.

Common later reward magnitudes are progress 1, merge 5, passing 2, arrival 2, collision 20, intervention 0.2, jerk 0.01, lane change 0.1, feasible-acceleration alignment 0.25, unsafe request 0.5 and cooperation 0.1. V05 adds stable merge 2 and reversal/oscillation 0.5. Signs, normalization and event frequency depend on the matching implementation. Exact configurations and hashes are retained in the JSON.

Stage 1 v03 progressed through cruise/follow foundation, passing, easy merge and dense merge. Validation after BC was 29.02% interventions, falling to 11.34% at phase 0 and rising later. Training-label RMSE improved 0.20134→0.14399 without guaranteeing improved closed-loop behavior. Later selection reduced failure count while sacrificing pooled and worst-case rates.

V05's 249,999 decisions comprise 4,096 critic-only warmup and 245,903 actor-training decisions. It ran 2,025 PPO optimizer steps; 512.048 seconds from two zero-update failures remained in the 2,455.032-second charged total. The adaptive curriculum **never left grade 0**. Collected exposure was cruise 109,909 decisions/21 episodes, following 102,623/19, passing 9,467/6 and merge 28,000/4, including warmup. There was **no I-110 merge or passing training exposure**, only two cruise and two following episodes there. Of the final 32 admitted reports, cruise passed 0/13, following 4/13, passing 1/4 and merge 0/2.

Cruise therefore had substantial exposure. Its regression cannot be explained solely by omission. Plausible contributors are shared-actor cross-task gradients, changed observation/guard semantics, imperfect teacher labels, reward tradeoffs and admission/selection rules. The history does not isolate their causal contributions.

The saved v05 objective audit records global clipping on all 2,025 PPO steps, median gradient multiplier 0.12056. On recorded first minibatches, acceleration-imitation gradient magnitude was median 0.01141 of PPO and near-limit gradient 0.08154. Critic norm exceeded actor norm on 360/2025 steps: the critic did not always dominate. Mean weighted reward per decision was progress +0.61330, jerk −0.02790, acceleration alignment −0.03629 and intervention −0.03747. Effective contribution and gradient balance matter more than the printed coefficient alone.

## Actionable next plan — not a launch

1. **Repair the task model first.** Resolve observed native junction/route utility and leader-information limitations with bounded non-learning checks. Preserve native safety, raw override accounting and historical evidence. Version observation changes explicitly.
2. **Compare initializations before BC.** On the final corrected contract, compare the existing v05 final actor against the exact migrated v05 initial phase-0 actor using identical physical demand and per-task progress, intervention, retention and comfort gates. Use final v05 as an available candidate, not a predeclared winner. Keep accepted 81-field v04r2 separate.
3. **Validate the teacher and reward scale.** Low overrides alone are insufficient: the old teacher reached peak jerk 65 m/s³, while smooth v4 retains four comfort failures. Do not globally amplify imitation of these failures. Any target eligibility rule must be explicit and must not remove hard-braking cases from evaluation or change override counts.
4. **Use a balanced short pilot with retention gates.** Specify actual decision quotas, foundational cruise/follow exposure every stage, meaningful I-110 merge exposure and advancement rules that cannot silently prevent coverage forever. Keep latest resumable and best eligible checkpoints separate; reject regressions rather than equating more updates with improvement.
5. **Make attribution honest.** Compare a declared preparation/training bundle against a matched frozen baseline. Attribute improvement to the bundle unless later one-factor ablations isolate actor LR, teacher, reward or retention changes. Anchor-policy distillation/KL may use training-only observations as an explicitly separate loss; PPO ratios use fresh on-policy data. Validation trajectories remain evaluation-only.
6. **Obtain approval for the resolved run.** Specify the contract, baseline decision, target eligibility, objectives, quotas, stopping rules and acceptance gates before requesting a budget. This audit starts no training or simulation.

## Missing evidence

No accepted broad 93- or 99-field policy exists. No full corrected-network initial/final v05 comparison or new held-out promotion exists. Single-seed histories do not establish robust optimizer effects, and there is no causal communication-benefit result. Current own-motion observations do not directly expose neighboring lane-change intent; failure of the adjacent-preview heuristic is not proof that anticipation is impossible. Weight norms and twelve synthetic inputs cannot diagnose real traffic from appearance. Historical headway, emergency-warning counts, override denominators and merge definitions must retain their original scope.

Exact source hashes, current Git identity, all run-file hashes, selected-checkpoint statistics, configuration values and links to unchanged raw evidence are in `checkpoint-audit.json`.
