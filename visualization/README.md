# Visualization

Kit extensions and ovrtx application code belong here.

Both Isaac Sim and standalone ovstage + ovrtx are planned targets. They must share
the same USD road/vehicle assets, recorded poses, units, IDs, and timing contract.
Runtime-specific lidar configuration and UI belong in separate adapters/layers.
The Isaac replay and standalone ovrtx fixed-timestamp RGB rendering now use the
same exporter. See `documentation/ovrtx-replay.md` for setup and render commands.
Interactive ovrtx playback and ovrtx lidar are not yet implemented.

