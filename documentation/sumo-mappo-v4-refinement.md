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

Configuration SHA256: `b092e1ceb14b6f42d2f5a3b3ba8324d9a3cac2b4ca19562221158a2142d14018`.

167 targeted tests passed. Independent review confirmed that teacher labels
are sampled before the action, only the neural action executes during PPO,
and demonstration transitions and resumed optimizer totals remain accounted.
The inference and safety paths are unchanged. Initial demonstration collection
or supervised-fitting failures retain evidence but do not have a completed
warm-start checkpoint; normal PPO/evaluation failures retain full checkpoints.

Native no-learning preflight `preflight-20261029T230312Z-a42cad2f` passed in 46.188 s: unchanged weights/sources, deterministic scripted repeat and actor export parity. An earlier preflight passed behavior checks but failed source-integrity because the resume-accounting correction overlapped it; both attempts are retained. See [setup evidence](evidence/sumo-mappo-v4-refinement-setup.json).
