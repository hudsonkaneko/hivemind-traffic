# Single-vehicle highway

Deterministic two-lane, 800 m highway used to validate the TraCI backend before
introducing PettingZoo or learning policies.

`agent_0` departs in lane 0 at 8 m/s and follows `highway_0 highway_1`. Generate
the committed network after editing its source files with:

```powershell
netconvert --node-files nodes.nod.xml --edge-files edges.edg.xml --output-file network.net.xml
```

Run the scripted baseline from the repository root:

```powershell
.\.venv\Scripts\python.exe -m traffic.scripted_baseline
```

