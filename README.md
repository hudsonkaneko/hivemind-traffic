# Hivemind Traffic

Repository scaffold for a multi-agent highway traffic simulation project.

```text
hivemind-traffic/
├── scenarios/       # SUMO networks, routes, and configurations
├── traffic/         # TraCI runner and vehicle control
├── environments/    # PettingZoo environment
├── policies/        # PPO networks and communication models
├── usd/             # OpenUSD assets, stages, and exporters
├── visualization/   # Kit extensions or ovrtx application
├── experiments/     # Training and evaluation configurations
├── tests/
└── results/
```

## Development setup

The initial toolchain is pinned to Python 3.11 and SUMO 1.27.1. On Windows,
install SUMO, create the local environment, and install the Python dependencies:

```powershell
winget install --exact --id EclipseFoundation.SUMO --version 1.27.1
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

The SUMO installer normally configures `SUMO_HOME` and adds its `bin` directory
to new terminal sessions. If it does not, set `SUMO_HOME` to the SUMO installation
directory and add `$env:SUMO_HOME\bin` to `PATH` for that session.

Verify the installation:

```powershell
sumo --version
.\.venv\Scripts\python.exe -c "import traci, pettingzoo; print(traci.__file__); print(pettingzoo.__version__)"
```

