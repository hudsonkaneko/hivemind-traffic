# MAPPO v04r1: imitation-assisted passing refinement

October 29, 2026. The user authorized correction and retraining after the initial
v04 pilot failed acceptance. This is a new, explicitly imitation-assisted MAPPO
experiment. The original v04 configuration, failed run and accepted v02 actor
remain intact. Results will be reviewed after completion or failure, without
agent monitoring during optimization.

## Problem and change

Initial v04 selected update 175: 7.7275% held-out safety interventions and no
passes. Later validation checkpoints learned passing but had 21–31%
interventions. Many requests accelerated into the speed ceiling, SUMO's
car-following limits or the explicit stopping-distance bound. The scripted
reference supports passing with low intervention rates. A reward-only ranking
among failed policies also favored following over the intended passing task.

The refinement learns from the existing conservative scripted controller on
training seeds, then continues MAPPO while receiving teacher labels at the
states the learning policy actually visits. These labels are auxiliary training
targets. The exported actor is still a neural network receiving the same 81
local/message features and choosing normalized acceleration and lane delta;
deployment does not call the teacher or replace the actor with scripted actions.
The critic remains centralized and training-only. This is not pure from-scratch
RL, and imitation does not establish a communication benefit.

The environment, physical limits, communication, rewards, intervention counting
and acceptance gates are unchanged. No shorter braking distance or suppression
of override reporting is used to improve the score. SUMO still owns motion and
lane centering; this does not train physical steering or sensor perception.

## Declared configuration

Config: [`sumo_mappo_v4_refine.json`](../experiments/configs/sumo_mappo_v4_refine.json).

| Setting | Value |
| --- | --- |
| Total new training budget | 1,000,000 agent transitions, including demonstrations; 7,200 wall seconds including evaluation |
| Demonstrations | 20 complete scripted training episodes, up to 48,000 agent transitions |
| Initial supervised fitting | 60 epochs, minibatch 512, learning rate 0.0003, actor only |
| Supervised loss | Normalized acceleration mean squared error plus masked lane cross-entropy |
| Rare lane-change label weight | 10 versus 1 for stay |
| PPO auxiliary supervision | Same loss, coefficient 1, current-state teacher labels; actor actions actually execute |
| Initial Gaussian log standard deviation | −2 after supervised fitting |
| PPO actor / critic learning rate | 0.00003 / 0.0003 |
| Entropy coefficient | 0.001 |
| Workers / rollout / PPO epochs | 4 CPU / 256 steps per worker / 4 |
| Training RNG / episode seeds | 7104 / 2000000–2099999 |
| Validation / new test / preflight seeds | 2110001–2110010 / 2210001–2210050 / 2310001–2310002 |
| Validation frequency | After supervised fitting, every 25 PPO updates, and the last update |

The initial 20 training seeds supply demonstrations; PPO begins at the next
unused seed. Demonstration transitions count inside the one-million total, so
normal 600-step episodes leave 952,000 PPO transitions. Warm-start optimizer
steps are reported separately and included in total optimization counts.
Full resumable checkpoints record completion of imitation so recovery does not
repeat it or grant a new transition budget. Runtime failures are retained.

Checkpoint ranking is declared before execution: accepted first, then fewer
collisions, fewer overspeed steps, more nominal episodes, passing/frequency
targets met, fewer interventions, then reward. Original configurations retain
their earlier selection rule. This ranking changes selection, not acceptance.
The supervised-only snapshot remains a validation candidate if PPO regresses;
its provenance must be disclosed if it is selected.

Acceptance still requires zero collisions and overspeed, all nominal endings,
mean per-car progress at least 600 m, interventions at most 5%, at least one
actual pass in every evaluation episode and at most six lane changes per
vehicle per minute. Test seeds are new because the previous held-out results
have already informed the refinement. No seed is excluded after seeing results.

All settings, source/runtime hashes, checkpoints, teacher labels, action
journals, per-seed evaluations and optimization metrics are retained under a
unique `outputs/sumo_mappo/v04/` run. Compact evidence belongs in `documentation/`.
No claim of successful learning is made before evaluation.

## Completed retraining result

Run **train-20261029T230444Z-1a5f80b7** completed without runtime errors in
**1,410.844 seconds (23 min 31 s)**. The agent waited without monitoring or
analyzing progress. Sources stayed unchanged. The full budget was consumed:
48,000 demonstration transitions plus 952,000 on-policy transitions, 5,640
supervised optimizer steps and 14,537 PPO optimizer steps over 233 PPO updates.
No recovery or resume was needed. No extra training was launched.

The frozen actor now passes slower cars in all 50 fresh held-out episodes and
meets the aggregate 5% intervention ceiling. **Full acceptance still fails**
because one episode exceeds the per-car lane-change frequency limit.

| Held-out result, 50 episodes each | Refined neural actor | Scripted reference |
| --- | ---: | ---: |
| Safety interventions | 4.9583% | 0.1117% |
| Slower-car passes | 82 | 80 |
| Episodes with at least one pass | 50 / 50 | 50 / 50 |
| Applied lane changes | 331 | 171 |
| Maximum changes / car / minute | 8 | 1 |
| Collisions / overspeed / required-gap deficits | 0 / 0 / 0 | 0 / 0 / 0 |
| Mean per-car progress | 1,449.63 m | 1,488.69 m |
| Minimum measured headway | 1.614 s | 1.630 s |
| Mean cooperative-eligible headway | 2.665 s | 2.043 s |
| Full acceptance | **No** | Yes |

The frequency failure is seed **2210006**: `agent_2` made eight lane changes
in 60 seconds; the episode still completed two passes. The remaining 49
episodes meet the frequency gate. The aggregate intervention result is close
to its limit and does not imply every individual episode is below 5%.
The original v04 held-out set differed, so the previous 7.7275% versus current
4.9583% is a descriptive comparison, not a paired same-seed improvement estimate.

The declared validation rule selected **update 225 (969,600 total transitions)**.
Its ten validation episodes had 6.6625% interventions, 18 passes and at most
five changes/car/minute. The final update 233 had 8.2875% interventions, 15
passes and at most four changes/car/minute. Neither passed every validation
gate. Selection did not use held-out results. The supervised-only update-0
candidate had 9.692% validation interventions; the selected actor includes
actual PPO updates, not only demonstration fitting. Independent post-run review
confirmed no skipped optimizer updates, teacher at inference or scripted fallback.

The initial supervised loss fell from 0.797593 to 0.002545; this measures fitting
the teacher labels, not independent driving acceptance. The unchanged stopping
bound still takes precedence over the desired 0.8 s cooperative headway. No
short-gap advantage, causal communication benefit or 3D/physical capability is
established. Accepted v02 remains available and is not superseded.

Source commit: `584dd9760d91560f019340b14f85a25acc7db4b8`. Exported actor SHA256:
`6695282f8128168316feec7e26692d1644e6815deaa458ad625555bbc025ce52`.
Full local policy, best/latest resumable checkpoints, demonstration dataset,
action journals, per-seed evaluations and manifests remain under
`outputs/sumo_mappo/v04/train-20261029T230444Z-1a5f80b7/`.

Versioned review files: [summary and checkpoint provenance](evidence/sumo-mappo-v4-refinement/summary.json),
[exact learned evaluation](evidence/sumo-mappo-v4-refinement/held-out-learned-evaluation.json),
[all-controller per-seed metrics](evidence/sumo-mappo-v4-refinement/held-out-metrics.csv),
and [optimization curves](evidence/sumo-mappo-v4-refinement/training-updates.csv).
The next remaining behavior is unnecessary lane switching. Any further training
should use a declared refinement and fresh test set rather than tune against
seed 2210006 or silently extend this exhausted budget.

Configuration SHA256: `b092e1ceb14b6f42d2f5a3b3ba8324d9a3cac2b4ca19562221158a2142d14018`.

167 targeted tests passed. Independent review confirmed that teacher labels
are sampled before the action, only the neural action executes during PPO,
and demonstration transitions and resumed optimizer totals remain accounted.
The inference and safety paths are unchanged. Initial demonstration collection
or supervised-fitting failures retain evidence but do not have a completed
warm-start checkpoint; normal PPO/evaluation failures retain full checkpoints.

Native no-learning preflight `preflight-20261029T230312Z-a42cad2f` passed in 46.188 s: unchanged weights/sources, deterministic scripted repeat and actor export parity. An earlier preflight passed behavior checks but failed source-integrity because the resume-accounting correction overlapped it; both attempts are retained. See [setup evidence](evidence/sumo-mappo-v4-refinement-setup.json).
