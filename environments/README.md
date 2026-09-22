# Environments

PettingZoo environments and wrappers belong here.

`highway_parallel_env.py` exposes the curved two-agent SUMO scenario through the
PettingZoo Parallel API. Each active autonomous vehicle submits one tactical action
per control interval, after which the backend advances SUMO exactly once.

Current action: `[speed, lane]`, where each component has three discrete choices.
Current local observation: 16 values containing ego speed, lane, route progress,
front/rear gap and relative-speed features with presence masks for each of the two
lanes, plus the remaining lane-change cooldown. Dynamic action masks reject road
boundaries, occupied target gaps, and repeated lane changes during the cooldown.
`state()` exposes the fixed-size fleet state for a future centralized critic.

