# Curved two-agent highway

An S-shaped, two-lane route with two PettingZoo-controlled vehicles and three
SUMO-controlled background vehicles. It is the first multi-agent environment
fixture; the straight single-vehicle scenario remains the backend regression test.

Generate the network after editing its sources:

```powershell
netconvert --node-files nodes.nod.xml --edge-files edges.edg.xml --output-file network.net.xml
```

View the uncontrolled scenario with:

```powershell
sumo-gui -c scenarios\curved_two_agent\scenario.sumocfg
```

