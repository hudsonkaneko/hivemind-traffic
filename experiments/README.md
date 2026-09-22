# Experiments

Training and evaluation configurations belong here.

The project implementation sequence and validation gates are recorded in
[`ROADMAP.md`](ROADMAP.md).

Run and watch the scripted PettingZoo baseline:

```powershell
.\.venv\Scripts\python.exe -m experiments.rollout --policy scripted --gui
```

Run matched headless baselines:

```powershell
.\.venv\Scripts\python.exe -m experiments.rollout --policy scripted --seed 42
.\.venv\Scripts\python.exe -m experiments.rollout --policy random --seed 42
```

Train the shared no-communication PPO baseline:

```powershell
.\.venv\Scripts\python.exe -m experiments.train_ppo
```

Replay a saved checkpoint visibly:

```powershell
.\.venv\Scripts\python.exe -m experiments.rollout --policy trained --checkpoint <path-to-best.pt> --gui
```

The default configuration is `configs/ppo_baseline.json`. Training, validation,
and held-out test seeds are declared before execution. Checkpoints include the
shared actor, local critic, optimizer, observation-normalization state, RNG states,
configuration, and transition count.

