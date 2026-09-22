# Visualization

Kit extensions and ovrtx application code belong here.

Both Isaac Sim and standalone ovstage + ovrtx are planned targets. They must share
the same USD road/vehicle assets, recorded poses, units, IDs, and timing contract.
Runtime-specific lidar configuration and UI belong in separate adapters/layers.
The existing Isaac replay is a working prototype; ovrtx integration remains to
be implemented. First gate: a minimal PNG render, then the shared traffic replay.

