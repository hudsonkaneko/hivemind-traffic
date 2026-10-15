---
name: train-hivemind-marl
description: Train and evaluate coordinated autonomous-vehicle policies with shared PPO or MAPPO, explicit communications, safety shielding, and defensible baselines. Use after a SUMO/TraCI multi-agent environment exists; use a separate experiment skill to package reproducibility evidence.
---

# Train Hive-Mind MARL

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

Build a fleet policy that coordinates vehicles without depending on information unavailable at deployment. Keep SUMO authoritative for traffic motion and expose simultaneous agents through a PettingZoo `ParallelEnv`.

## Establish the contract

Before training, inspect or define:

- Stable agent IDs, observation/action spaces, termination rules, control interval, reward terms, and action masks.
- `reset(seed=...)` and `step(actions)` behavior. Run PettingZoo's `parallel_api_test` and `parallel_seed_test`.
- Per-agent observations containing only ego/local sensing plus messages actually delivered by the communications model.
- A separately named global state for a centralized critic. Never feed it to deployed actors.
- A safety shield between policy output and TraCI. Log both the proposed and applied action.

Do not begin expensive training until a random policy and a scripted controller can finish an episode without API, NaN, or lifecycle errors.

## Build policies in this order

1. Measure a no-control or SUMO-native baseline.
2. Implement a deterministic scripted coordination baseline.
3. Train independent PPO/IPPO without communications.
4. Train a shared-parameter actor: every equivalent vehicle uses the same network and contributes experience to it. Add an agent-role input only when roles genuinely differ.
5. Add centralized training with decentralized execution (CTDE): use a joint/global state only in the value function during training, while each actor consumes its local observation and received messages. This is MAPPO.
6. Add learned or structured communications only after the no-communication policy is stable.

Prefer a shared actor for homogeneous fleet vehicles. If agents have different action spaces or physical roles, use role-specific heads or policies and document the reason.

## Model the hive-mind channel

Treat communication as part of the environment, not invisible state sharing. Specify:

- Message contents or learned embedding size.
- Sender/receiver topology: none, nearest neighbors, range limited, or fleet wide.
- Update rate, latency, packet loss, range, and bandwidth budget.
- Aggregation rule and behavior when no message arrives.

Run at least these ablations with the same scenarios and evaluation seeds:

- No communication.
- Local or nearest-neighbor communication.
- Full communication.
- The chosen channel under latency and packet loss.

Log messages or compact message summaries, delivered bytes, drops, and delay. Do not expose future SUMO state or simulator-only variables through a message.

## Keep safety outside the learned policy

The shield must enforce hard constraints before applying actions: legal speed bounds, feasible acceleration/braking, valid lane changes with adequate gaps, and emergency collision avoidance. Unit-test boundary cases. Count overrides and their causes; a policy that depends on frequent overrides has not learned safe behavior.

Keep reward terms interpretable. Typical components are route progress, team throughput, safe headway, successful formation, smooth acceleration, collisions, congestion, and communication cost. Log every component separately and check for reward hacking.

## Train and evaluate

- Use PPO for the shared decentralized baseline; use MAPPO when the centralized critic is justified. Keep algorithm, policy sharing, and critic visibility independently configurable.
- Normalize observations from training data only. Freeze normalization statistics for evaluation.
- Save latest and best checkpoints with actor, critic, optimizer, normalization state, training step, and resolved configuration.
- Separate training, validation, and held-out test scenarios or seed sets. Tune on validation, never on test.
- Evaluate without exploration using matched seeds and identical demand, horizon, warm-up, safety rules, and communications conditions.
- Report mean and variation across seeds for success, collisions, near-collisions or minimum TTC, throughput, travel time/delay, queue length, stops, comfort/jerk, shield overrides, and communication cost.
- Include learning curves and comparison tables for the scripted, no-communication, local-communication, and selected full system. Report failed runs and exclusions.

Isaac Lab is optional. Use its `skrl` MAPPO workflow only when detailed vehicle physics, GPU-parallel environments, or RTX sensor observations are part of the question; it does not replace the SUMO traffic experiment.

## Acceptance criteria

- PettingZoo parallel API and seed tests pass, and observation/action shapes remain stable as agents enter or leave.
- One shared actor controls all equivalent fleet agents; evaluation does not require the centralized critic or privileged state.
- The safety shield passes boundary tests, and every override is measurable.
- Scripted, IPPO/no-communication, and MAPPO/communication variants are evaluated on matched held-out seeds.
- Communication ablations include no, local, full, and impaired channels.
- A saved checkpoint can replay a held-out episode and regenerate the reported metrics.
- The conclusion states both coordination gains and costs, including safety interventions and message traffic.

## Authoritative references

- [PettingZoo Parallel API](https://pettingzoo.farama.org/main/api/parallel/)
- [PettingZoo environment tests](https://pettingzoo.farama.org/main/content/environment_tests/)
- [TorchRL multi-agent PPO tutorial](https://docs.pytorch.org/rl/main/tutorials/multiagent_ppo.html)
- [TorchRL MAPPO and IPPO objectives](https://docs.pytorch.org/rl/main/reference/objectives_multiagent.html)
- [Isaac Lab reinforcement-learning workflows](https://isaac-sim.github.io/IsaacLab/develop/source/concepts/reinforcement_learning.html)
