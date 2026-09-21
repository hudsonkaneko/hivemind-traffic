# Repository boundaries

On 2026-09-21, the project adopted the `hivemind-traffic` repository layout:

- `scenarios/`: SUMO networks, routes, demand, and configurations.
- `traffic/`: TraCI runners and vehicle-control integration.
- `environments/`: PettingZoo environments and wrappers.
- `policies/`: PPO networks and communication models.
- `usd/`: OpenUSD assets, stages, and exporters.
- `visualization/`: Kit extensions and ovrtx application code.
- `experiments/`: training and evaluation configurations.
- `tests/`: automated verification.
- `results/`: evaluation summaries and publication-ready outputs.

Existing prototype material remains in place for history and migration. New work should use the boundaries above. Runtime logs remain in `logs/`, while trained policies and generated evaluation artifacts remain in `outputs/`, as required by the workspace conventions.
