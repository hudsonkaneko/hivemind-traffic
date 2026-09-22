# Environments

PettingZoo environments and wrappers belong here.

`highway_parallel_env.py` exposes the curved two-agent SUMO scenario through the
PettingZoo Parallel API. Each active autonomous vehicle submits one tactical action
per control interval, after which the backend advances SUMO exactly once.

Current action: `[speed, lane]`, where each component has three discrete choices.
Current local observation: ego speed, lane, route progress, and padded nearest-front
and nearest-rear gap/relative-speed features with presence masks. `state()` exposes
the fixed-size fleet state for a future centralized critic.

