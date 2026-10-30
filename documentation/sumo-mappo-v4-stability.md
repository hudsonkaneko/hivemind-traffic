# MAPPO v04r2: learned lane stability

October 29, 2026. The user authorized correcting unnecessary lane changes and
retraining, with no agent monitoring while training runs. This new experiment
keeps the original v04/v04r1 evidence intact. Accepted v02 is preserved.

## Hypothesis and unchanged gates

V04r1 passes slower cars in all 50 fresh test episodes and meets the aggregate
5% intervention gate, but one episode exceeds six changes per car per minute.
The training/validation record shows excessive switching emerging during PPO:
the supervised-only candidate passes on every validation episode with at most
one lane change per car per minute, while later candidates reach 7–12.

This refinement tests stronger guidance against unnecessary switching and a
larger action cost. It does not add an inference hard cap or alter lane-action
masks to force a passing frequency score. Cooldown remains five seconds,
the observation/action contract stays 81 local/message fields to acceleration
plus lane delta, and all braking, clearance, TTC and communication assumptions
remain unchanged. The exported neural actor runs without the training teacher.

Acceptance remains zero collisions and overspeed, all nominal endings, mean
progress at least 600 m, aggregate interventions at most 5%, at least one slower
car passed in every episode, and at most six lane changes per car per minute.
Passing these simulator checks would not establish physical steering, sensor
perception, achieved 0.8 s following or a communication benefit.

## Declared refinement

Config: [`sumo_mappo_v4_stable.json`](../experiments/configs/sumo_mappo_v4_stable.json).

| Setting | v04r1 | v04r2 |
| --- | ---: | ---: |
| Applied lane-change reward weight | −0.05 | −0.5 |
| Auxiliary imitation coefficient during PPO | 1 | 3 |
| Lane-change label weight during initial imitation | 10 | 10 |
| Lane-change label weight during PPO | 10 | 2 |
| Initial Gaussian log standard deviation after imitation | −2 | −2.5 |
| PPO entropy coefficient | 0.001 | 0.0003 |

The initial rare-action weighting remains because the prior demonstration set
contained only 78 lane-change labels among 48,000 examples. The separate PPO
weight gives greater relative importance to staying while still supervising
passing; stronger acceleration guidance also targets the narrow intervention
margin. This is imitation-assisted MAPPO, not from-scratch RL. The stage-specific
weight is optional and preserves earlier configurations' behavior when omitted.

The new budget is one million agent transitions, including 20 complete scripted
episodes (48,000 transitions) and 952,000 PPO transitions, with a two-hour wall
cap including evaluation. Initial imitation uses 60 epochs, minibatch 512 and
learning rate 0.0003; PPO uses four CPU workers, actor rate 0.00003 and critic
rate 0.0003. All other optimizer, scenario and selection settings match v04r1.

Training RNG is 8104; training episode seeds are 3000000–3099999, validation
3110001–3110010, held-out tests 3210001–3210050, and preflight 3310001–3310002.
The previous failure seed is not used as the new test criterion. Checkpoint
ranking remains accepted, collision/overspeed/nominal gates, passing/frequency
targets, intervention rate, then reward. Validation includes the initial
imitation actor, each 25 PPO updates and the final update. Selection never uses
held-out results. New test seeds mean cross-run comparisons are descriptive,
not matched-seed effect estimates.

Save full checkpoints, exact configuration/source/runtime identity, training
and validation curves, demonstration records, per-seed evaluations and action
journals under a unique `outputs/sumo_mappo/v04/` run. Recoverable failures retain
evidence and resume completed checkpoints within the remaining budget. Results
are reviewed only after process completion or failure.

## Completed result: accepted for the declared SUMO task

Run **train-20261029T235954Z-cc89b964** completed without runtime errors in
**1,291.031 seconds (21 min 31 s)**. The agent waited without monitoring or
analyzing training. Sources stayed unchanged; no recovery or extra training was
needed. The full one-million-transition budget comprised 48,000 demonstrations
and 952,000 on-policy transitions. There were 233 PPO updates, 13,878 PPO
optimizer steps and 5,640 supervised optimizer steps; no PPO update was skipped.

The selected frozen neural actor passes all declared safety/progress/passing
and lane-frequency acceptance gates on the 50 fresh held-out scenarios.

| Held-out result, 50 episodes each | Selected neural actor | Scripted reference |
| --- | ---: | ---: |
| Accepted | **Yes** | Yes |
| Aggregate safety intervention fraction | 3.175% | 0.2367% |
| Actual slower-car passes | 74 | 84 |
| Episodes with at least one pass | 50 / 50 | 50 / 50 |
| Applied lane changes | 212 | 176 |
| Maximum changes / car / minute | 2 | 2 |
| Collisions / overspeed / required-gap deficits | 0 / 0 / 0 | 0 / 0 / 0 |
| Nominal episode endings | 50 / 50 | 50 / 50 |
| Mean per-car progress | 1,446.91 m | 1,487.60 m |
| Minimum measured headway | 1.638 s | 1.625 s |
| Mean cooperative-eligible headway | 3.086 s | 2.048 s |

The intervention ceiling is an aggregate gate, not a claim that every episode
individually has fewer than 5% overrides. Safety filters remain active. The
scripted reference still has fewer interventions and higher average progress;
passing acceptance is not evidence the learned actor outperforms that baseline.
Cross-refinement test seeds differ, so reduced intervention/switching figures
are descriptive rather than paired causal improvement estimates.

**Use `policy.pt` or `checkpoints/best.pt`, selected update 175 at 764,800 total
transitions.** Its ten validation episodes also passed: 1.9708% interventions,
14 passes with at least one in every episode, and at most two changes/car/minute.
The final update 233 regressed to 9.2708% validation interventions; its
`checkpoints/latest.pt` is retained for recovery/history and is not the accepted
deployment actor. Checkpoint selection used validation only, before held-out
evaluation. Independent review verified the exact selection maximum and all
50 declared test seeds without exclusions. Frozen export and best checkpoint
matched exactly on 960 saved observation samples (zero maximum action error).

Source commit: `bd59d2f008c4c59615276c759377562fc339e826`. Accepted actor SHA256:
`4cf4898df997a444624552515ea987e7e994b1ca40bb002cf91f0647be8a7245`.
Full local artifacts: `outputs/sumo_mappo/v04/train-20261029T235954Z-cc89b964/`.
This includes the actor, best/latest checkpoints, demonstrations, journals,
all update/episode metrics, source/runtime manifests and exact configuration.

Versioned review evidence: [summary/provenance](evidence/sumo-mappo-v4-stability/summary.json),
[exact learned evaluation](evidence/sumo-mappo-v4-stability/held-out-learned-evaluation.json),
[per-seed controller metrics](evidence/sumo-mappo-v4-stability/held-out-metrics.csv),
and [optimization curves](evidence/sumo-mappo-v4-stability/training-updates.csv).
No model weights or bulk journals were committed to Git.

This is an accepted **v04 SUMO passing/following actor under the declared
fixtures**, with no teacher or scripted fallback at inference. It does not
replace the different-contract v02 actor in existing integrations automatically.
Physical steering, cameras/LiDAR as policy inputs, 3D integration, much closer
following and causal communication benefits remain separate validation tasks.

Configuration SHA256: `8d926dc86f53473ef64fcb5f7f36bfec5801554fe26e1773ecfb803c882485e9`.
Independent code review confirmed that the stage-specific weight switches only
the training objective, including resume; the actor inference and environment
guards are unchanged. Missing configuration preserves the earlier behavior.

178 targeted tests passed (15 existing TorchScript deprecation warnings). Native
no-learning preflight `preflight-20261029T235834Z-c343b674` passed in 54.921 s with unchanged weights/sources, exact scripted repeat, export parity and all passing/safety setup checks. [Setup evidence](evidence/sumo-mappo-v4-stability-setup.json).
