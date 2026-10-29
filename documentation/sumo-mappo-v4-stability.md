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

Configuration SHA256: `8d926dc86f53473ef64fcb5f7f36bfec5801554fe26e1773ecfb803c882485e9`.
Independent code review confirmed that the stage-specific weight switches only
the training objective, including resume; the actor inference and environment
guards are unchanged. Missing configuration preserves the earlier behavior.

178 targeted tests passed (15 existing TorchScript deprecation warnings). Native
no-learning preflight `preflight-20261029T235834Z-c343b674` passed in 54.921 s with unchanged weights/sources, exact scripted repeat, export parity and all passing/safety setup checks. [Setup evidence](evidence/sumo-mappo-v4-stability-setup.json).
