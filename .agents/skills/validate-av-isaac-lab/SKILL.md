---
name: validate-av-isaac-lab
description: Validate selected autonomous-vehicle coordination scenarios in Isaac Lab with vehicle physics, multi-agent control, RTX sensors, and domain randomization. Use after a SUMO/TraCI policy has a measurable baseline; do not replace city-scale SUMO traffic simulation with Isaac Lab.
---

# Validate AV Policies in Isaac Lab

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

Treat Isaac Lab as an optional second-stage validation environment. It runs on Isaac Sim, so selecting this skill also introduces Isaac Sim installation, GPU, and version-compatibility requirements. Use it only when the research question needs physical actuation, contact dynamics, perception sensors, ROS 2 integration, or robustness testing beyond SUMO's traffic model.

## Gate the work

Do not begin until the team has:

- A deterministic SUMO/TraCI scenario and scripted baseline.
- A trained policy checkpoint plus observation, action, and normalization definitions.
- Evaluation metrics across multiple traffic seeds.
- One or a few representative validation scenarios. Do not reproduce the entire city or training corpus in Isaac Lab.

## Assign one physical authority

Document ownership for every simulated object before implementation:

- **Playback/sensor mode:** SUMO owns all vehicle poses; Isaac Sim receives kinematic transforms and renders sensors. Do not claim this validates low-level vehicle dynamics.
- **Hybrid dynamics mode:** Isaac Lab owns physics and actuation for the evaluated fleet or ego vehicles; SUMO owns background traffic. Exchange timestamped commands and state across an explicit synchronization boundary.

Never allow SUMO and Isaac Lab to write the same vehicle pose during the same interval. Define physics timestep, policy timestep, synchronization cadence, lag handling, and reset behavior.

## Build the multi-agent task

Use `DirectMARLEnv` when each vehicle is an agent. It follows the shape of the PettingZoo Parallel API while remaining a vectorized Isaac Lab environment.

1. Import the selected road and vehicle assets as OpenUSD and validate scale, axes, collisions, mass, wheels, steering, drivetrain, and sensor frames.
2. Define per-agent observations, actions, rewards, terminations, and truncations. Start with state observations and throttle/brake/steering; add RTX pixels only if the policy genuinely consumes them.
3. Expose fleet-wide state through `state_space` for centralized training while keeping deployable actors limited to local observations plus permitted messages.
4. Reuse the high-level SUMO policy where possible: it chooses tactics such as target lane or speed, while an Isaac Lab policy executes steering, throttle, and braking.
5. Run many compact environment copies headlessly. Enable rendering only for sensor observations, debugging, or final evidence.

## Train and compare

- Use `skrl` MAPPO for centralized-training/decentralized-execution experiments; include IPPO or an equivalent no-centralized-critic baseline.
- Pin exact Isaac Lab, Isaac Sim, trainer, driver, and asset versions.
- Seed every run and save configuration, normalization statistics, checkpoints, and evaluation traces.
- Randomize only parameters tied to a hypothesis: road friction, mass, actuator delay, sensor noise, lighting/weather, communication latency, packet loss, or initial traffic arrangement.
- Compare no communication, ideal communication, and degraded communication when evaluating the hive-mind claim.

## Deliverables

- Authority and synchronization specification.
- One `DirectMARLEnv` task with a small deterministic smoke-test configuration.
- Baseline and MAPPO/IPPO training configurations and checkpoints.
- Domain-randomization matrix, evaluation results, and replay/video evidence.
- A limitations note stating what Isaac Lab did and did not validate.

## Acceptance tests

- A seeded reset reproduces the same initial state, and the task runs headlessly for a full episode without NaNs or invalid contacts.
- Agent observation/action spaces, rewards, terminations, and centralized state have documented shapes and pass a one-step contract test.
- Only the declared authority updates each evaluated vehicle pose; synchronization drift is measured and stays within the declared tolerance.
- A scripted controller can complete the simplest selected scenario before RL training starts.
- The trained policy is evaluated on held-out seeds and compared with the SUMO policy, scripted baseline, and communication ablations using identical metrics.
- Domain-randomized evaluation reports safety events, completion rate, progress/travel time, smoothness, and performance cost; one successful video is not sufficient evidence.

## Official references

- [Isaac Lab documentation](https://isaac-sim.github.io/IsaacLab/main/)
- [DirectMARLEnv API](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.envs.html#isaaclab.envs.DirectMARLEnv)
- [Isaac Lab reinforcement-learning integrations](https://isaac-sim.github.io/IsaacLab/develop/source/concepts/reinforcement_learning.html)
- [Available Isaac Lab environments and trainer support](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments)
- [NVIDIA Isaac Sim overview](https://developer.nvidia.com/isaac/sim)
