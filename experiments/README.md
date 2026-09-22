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

