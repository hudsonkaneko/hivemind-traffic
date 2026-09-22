# Policies

PPO networks and communication models belong here.

`baselines.py` defines the required non-learning comparisons:

- `ScriptedPolicy`: deterministic local traffic rules with no communication.
- `RandomPolicy`: seeded random actions as a weak baseline.

Both produce proposed actions. The environment's safety shield may revise unsafe
speed or lane requests before TraCI receives them, and every override is logged.

`ppo.py` defines the shared local-observation actor-critic, action masks,
training-only observation normalization, and deterministic checkpoint loader. The
same neural network controls every equivalent vehicle; no communication or global
critic state is used in this baseline.

