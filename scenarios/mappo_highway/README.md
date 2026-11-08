# SUMO MAPPO v01 highway fixture

This separate 5 km, three-lane straight road is an initial car-following and
cooperative-speed training fixture. It does not contain a merge, sensors, lane
changes or Omniverse physics. Its speed ceiling is 26.8224 m/s (60 mph).

`traffic/mappo_sumo.py` writes seeded episode demand and configuration into each
run's output folder. Four controlled IDs (`agent_0` through `agent_3`) begin among
SUMO-native background vehicles; further background vehicles enter over time.
All vehicles keep their lane. SUMO speed safety mode 31 and lane-change mode 512
retain collision and safety-gap checks. The additional local filter and any
SUMO adjustment are exposed in transition diagnostics.

The checked-in network was generated with SUMO netconvert 1.27.1:

```text
netconvert --node-files nodes.nod.xml --edge-files edges.edg.xml --output-file network.net.xml --no-turnarounds true --precision 4
```

No training is launched by constructing this fixture or running its tests.
