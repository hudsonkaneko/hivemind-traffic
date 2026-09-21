# Ground-truth circular lane follower v1

Added `scripts/follow_circular_lane.py` as the baseline controller for the v1
highway. It loads `scenes/highway/v1/highway_v1.usda`, spawns the stock NVIDIA
Leatherback, and follows the selected circular lane clockwise using pure pursuit.

The controller uses the simulated vehicle pose and the exact lane radius. It
does not inspect rendered white lines yet. This deliberately separates vehicle
control and scene validation from the later camera-perception problem.

The default run targets the middle lane at 3 m/s for 4,500 physics steps (75
simulated seconds), which is long enough for approximately one lap. Results are
written to `outputs/lane_follow_v1/summary.json`, with a half-second sampled
trajectory in `outputs/lane_follow_v1/trace.json`.

Run it from the project root with:

```powershell
.\run.ps1 -Mode lane -Steps 4500 -Gui
```

The Python entry point also accepts `--lane`, `--speed`, and `--lookahead` for
controller experiments.

## Verified result

The default 4,500-step headless run completed 1.184 clockwise laps without
leaving the road. RMS lane-center error was 0.061 m, maximum absolute lane error
was 0.065 m, and RMS heading error was 0.353 degrees. The saved
`outputs/lane_follow_v1/summary.json` records the full result.
