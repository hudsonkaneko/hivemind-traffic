---
name: run-reproducible-av-experiments
description: Plan, run, compare, and report reproducible autonomous-vehicle simulation and reinforcement-learning experiments. Use when results need traceable configurations, seeds, manifests, checkpoints, matched comparisons, metrics, and rerunnable evidence.
---

# Run Reproducible AV Experiments

## Highway Sim repository scope

Read the repository's `AGENTS.md` and `documentation/project-skills.md` before
applying this workflow. The active roadmap determines the selected backend:
SUMO authority statements below apply to SUMO-backed experiments, not to native
PhysX-controlled vehicles. Preserve one motion owner per vehicle. Repo-local
skills do not authorize training, dependency installation, or publication of
local-only vehicle assets; follow the current task and project authorization.

Produce an evidence package from which another student can identify exactly what ran, replay a checkpoint, and reproduce every table. Never overwrite an existing run.

## Define the study before running it

Record the research question, hypothesis, primary metric, safety guardrails, variants, baseline, training budget, evaluation seed set, and pass/fail rule. Separate exploratory runs from the final comparison. Do not choose or remove seeds after seeing results.

Use one resolved configuration per run. It should include:

- Scenario/network/demand paths and content hashes; SUMO step length, horizon, warm-up, and version.
- Training and evaluation seeds for SUMO, the environment, Python, NumPy, and PyTorch.
- Observation, action, reward, termination, safety-shield, and communications settings.
- Algorithm, model architecture, optimizer, hyperparameters, rollout size, update count, and training budget.
- Evaluation episodes, policy mode, checkpoint selection rule, and metric definitions.

Prefer configuration files over command-line-only parameters. Save the fully resolved configuration after defaults and overrides are applied.

## Create an immutable run package

Use a unique run ID containing a study name plus a timestamp or configuration hash:

```text
runs/<study>/<run-id>/
  manifest.json
  resolved-config.yaml
  metrics.csv
  episode-metrics.csv
  logs/
  checkpoints/
    best.pt
    latest.pt
  artifacts/
```

The manifest must capture start/end time, status, exact command, current Git commit, whether the tree was dirty, dependency lockfile or package versions, Python/SUMO/CUDA/driver versions, OS, CPU/GPU, input hashes, seed set, and output hashes. If the tree is dirty, save the diff or its hash; do not label the run clean.

Save checkpoints atomically. Include actor, critic, optimizer, learning-rate scheduler when used, observation-normalization state, global step, RNG states where practical, and resolved configuration. Keep `best` selection tied to the predeclared validation metric; never select it using test results.

## Execute in stages

1. Run configuration/schema checks and unit tests.
2. Run a short smoke test and verify finite observations, actions, rewards, losses, and metrics.
3. Run one deterministic replay twice with the same seed; investigate differences before scaling.
4. Launch the declared training seed set. Record crashes and interrupted runs rather than silently replacing them.
5. Evaluate frozen checkpoints on held-out, matched seeds with exploration disabled.
6. Aggregate results from raw episode records, then generate the report. Do not type summary values manually.

SUMO is deterministic by default for a fixed version, inputs, and seed, but cross-version, platform, GPU, or threaded-library results may differ. Pin versions and state any deterministic settings and remaining limitations rather than claiming universal bitwise reproducibility.

## Metrics and fair comparisons

At minimum collect:

- Traffic: throughput, completed trips, travel time, delay, queue length, stops, and route progress.
- Safety: collisions, minimum time-to-collision or near-collision count, unsafe gaps, and shield overrides.
- Control: acceleration, jerk, lane changes, formation success, and task completion.
- Learning: total and component rewards, policy/value losses, entropy, and episode length.
- Communication: messages/bytes sent and delivered, drops, latency, and active neighbors.
- System: simulation steps per second, wall time, CPU/GPU use, and peak memory when material.

Compare variants with identical scenario inputs, horizons, warm-up, safety constraints, checkpoints rules, and evaluation seeds. Use paired per-seed differences when the same seed is shared. Report sample count, mean, dispersion, and confidence intervals when appropriate; preserve raw values so readers can recompute them.

The core comparison set should include SUMO/no learned control, a scripted policy, PPO or IPPO without communication, the selected MAPPO/shared policy, and communication ablations. Change one factor at a time for an ablation.

## Report the evidence

Create a concise report containing:

- Question, hypothesis, environment, variants, and evaluation protocol.
- Version/manifest summary and the exact reproduce commands.
- Tables and plots linked to raw metrics and run IDs.
- Primary result with uncertainty, safety results, communications cost, and computational cost.
- Failed/excluded runs with reasons, limitations, and the next experiment.

Do not claim an improvement from training reward alone. A conclusion requires held-out evaluation against the declared baseline.

## Acceptance criteria

- Every result maps to an immutable run ID, resolved configuration, manifest, raw metrics, and checkpoint or explicit no-checkpoint reason.
- Repeating a same-version smoke run with the same seed reproduces its trajectory or stays within a documented numeric tolerance.
- Final variants use the predeclared matched seed set, with no hidden exclusions.
- A checkpoint replay regenerates its episode metrics without the training process.
- Summary tables and plots are generated from stored raw results and include uncertainty and sample count.
- A new student can use the recorded command and pinned environment to reproduce at least one representative run.

## Authoritative references

- [SUMO randomness and reproducibility](https://sumo.dlr.de/docs/Simulation/Randomness.html)
- [SUMO TraCI documentation](https://sumo.dlr.de/docs/TraCI/)
- [PettingZoo environment and seed tests](https://pettingzoo.farama.org/main/content/environment_tests/)
- [PyTorch reproducibility notes](https://docs.pytorch.org/docs/stable/notes/randomness.html)
- [PyTorch model and checkpoint guidance](https://docs.pytorch.org/tutorials/beginner/saving_loading_models.html)
