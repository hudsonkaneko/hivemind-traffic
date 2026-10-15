---
name: pettingzoo-multiagent-traffic
description: Wrap a simultaneous-action traffic simulator such as SUMO/TraCI as a deterministic PettingZoo ParallelEnv for multi-agent reinforcement learning. Use for agent lifecycles, spaces, observations, actions, rewards, seeding, API compliance, or environment tests; do not use to choose or train the learning algorithm itself.
---

# PettingZoo Multi-Agent Traffic Environment

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

## Outcome

Produce a tested PettingZoo `ParallelEnv` in which all active autonomous vehicles act once per traffic-control interval, one action batch advances the simulator once, and the returned observations, rewards, termination flags, truncation flags, and diagnostics are reproducible. Keep the simulator behind a small backend interface so API tests can run against a fake backend and integration tests can run against SUMO.

## Start With Read-Only Checks

Before editing:

- Inspect the traffic backend's `reset`, `step`, state, event, and `close` contract and identify which component owns the simulation clock.
- Record installed Python, PettingZoo, Gymnasium, NumPy, and SUMO/TraCI versions. Inspect existing environment and tests before selecting APIs from examples online.
- Confirm agents truly act simultaneously. Use `ParallelEnv` for one joint action per traffic tick; use the AEC API only when actions are genuinely sequential and ordering is part of the problem.
- List the maximum agent population, dynamic entry/exit behavior, observation sources, action semantics, reward terms, episode limits, and safety overrides.

For review or diagnosis, do not modify files. Do not install packages, change the simulator configuration, launch training, delete checkpoints, upload results, or weaken safety constraints without the user's authorization.

## Workflow

1. **Specify the environment contract.** Define stable agent names, `possible_agents`, active `agents`, control interval, observation/action schemas, reward terms, termination conditions, truncation conditions, and the global state used by a centralized critic.
2. **Define invariant spaces.** Implement `observation_space(agent)` and `action_space(agent)` with Gymnasium spaces whose shapes, bounds, and dtypes match actual values. Cache them only when they are invariant for that agent. `possible_agents` and spaces must not change during play.
3. **Implement deterministic reset.** Seed the environment RNG, scenario generation, SUMO, and any action/noise process from the supplied seed. Reset the backend before constructing the active-agent list. Return observation and info dictionaries keyed by the newly active agents.
4. **Implement one atomic step.** Snapshot the agents active at the start; validate an action for each; translate and safety-filter the full batch; advance the traffic backend exactly once; compute transition rewards and events; construct the five required dictionaries; and update `agents` at one defined lifecycle boundary before returning.
5. **Separate local observation from global state.** Give each actor only permitted ego, neighbor, signal, route, and message information. Implement `state()` separately for centralized-training/decentralized-execution methods. Pad variable neighbor/message sets and include masks rather than changing tensor shapes.
6. **Make communication explicit.** Represent hive-mind messages, delay, range, dropout, bandwidth, and missing-message masks in the environment configuration and logs. Include a no-communication setting for comparison.
7. **Design auditable rewards.** Keep each reward term separately logged before summing. Include progress or throughput plus safety and smoothness terms, and state whether rewards are individual, shared, or mixed. Apply hard safety rules in an action shield rather than relying only on collision penalties.
8. **Test in layers.** Unit-test spaces, lifecycle changes, action translation, reward terms, and termination logic against a fake backend. Then run PettingZoo compliance and seed tests, followed by a short SUMO integration rollout using random and scripted policies.

## Non-Obvious Constraints

- `possible_agents` is the immutable universe of agent IDs; `agents` is the live set and may change. Do not use one as a substitute for the other.
- Keep keys consistent across actions and returned dictionaries for each transition, including agents that terminate on that transition. Add newly live agents once, remove completed agents once, and never revive a finished agent ID.
- A route completion or collision may be a termination; an episode time limit is a truncation. Do not collapse both into one flag.
- One PettingZoo `step()` must not contain an accidental extra SUMO step. Rewards and next observations must describe the same transition.
- Do not expose global or future information to decentralized actors. Global state may be available to a centralized critic through `state()` without appearing in local observations.
- Log both requested and safety-filtered actions. An intervention must not silently change the learning signal.
- `close()` must be idempotent and release subprocesses/connections even after reset or step fails.

## Acceptance Criteria

- `parallel_api_test` passes for a meaningful number of cycles.
- `parallel_seed_test` passes, and two fixed-action rollouts with the same seed produce matching observations, rewards, lifecycle events, and completion reason within stated tolerances.
- Every emitted observation is contained by its declared space; every accepted action is validated against its action space; no NaN or infinite values reach the learner.
- Random and scripted rollouts reach clean termination or truncation without stale agents, missing dictionary keys, extra simulator steps, or leaked SUMO processes.
- Unit tests run without SUMO through a fake backend, and at least one integration test exercises a small real SUMO scenario.
- Per-step diagnostics expose reward components, requested/executed actions, safety interventions, active agents, simulator time, and termination/truncation causes.

## Troubleshooting

- **API test reports inconsistent keys:** capture the live-agent set at step entry, build all return dictionaries from that transition set, and update `agents` at one explicit point.
- **Observation is outside its space:** inspect dtype, bounds, padding, masks, and normalized values; use `space.contains(value)` in focused tests.
- **Seed test fails:** route all randomness through seeded generators and propagate the seed into SUMO and scenario generation; avoid global `random` or wall-clock values.
- **Vehicle disappears mid-step:** treat SUMO departures and arrivals as lifecycle events, not exceptions; preserve the terminal transition for an agent that leaves.
- **Reward learning is unstable:** log unweighted components, check scale and sign, compare against a no-communication baseline, and verify reward and observation use the same post-step state.
- **Reset or test hangs:** ensure failed paths call the backend's idempotent `close`, and do not share one TraCI connection across concurrent environments.

## Official References

- [PettingZoo Parallel API](https://pettingzoo.farama.org/api/parallel/)
- [Custom environment tutorial](https://pettingzoo.farama.org/tutorials/custom_environment/)
- [Testing PettingZoo environments](https://pettingzoo.farama.org/content/environment_tests/)
- [Gymnasium spaces](https://gymnasium.farama.org/api/spaces/)
- [SUMO TraCI documentation](https://sumo.dlr.de/docs/TraCI/)
