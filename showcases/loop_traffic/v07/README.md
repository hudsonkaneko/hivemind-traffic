# v07 — responsive showcase

Opt-in 80-car presentation with shared visual geometry and the regular Isaac Sim
editor UI by default. The smaller Kit app remains an explicit option.
Two 252-second minimal-app GUI runs maintained 30-Hz updates and average real-time speed,
with exact physical repeatability. Rare 126–128 ms hitches keep the strict
smoothness gate open. Sixty FPS remains experimental (about 50 completion
notifications/s and 0.83x real time in the short test). Preserves v06
and all prior evidence. See [study and results](../../../documentation/showcase-responsiveness.md).

Use the configured traffic Python from repository root:

```text
python scripts/demo_loop_showcase.py --version v07
python scripts/demo_loop_showcase.py --version v07 --ui-profile showcase
python scripts/demo_loop_showcase.py --version v07 --evidence full --drive-seconds 90
```

The default v07 profile is 30 Hz presentation, 8 CPU workers, 960x600 rendering,
5 Hz statistics, and no frame generation. Physics/safety stay at 120 Hz. Follow,
traffic and overview controls remain available alongside the regular editor's
panels. Shared visual instances, cached updates and lighter evidence remain
enabled. Use `--ui-profile showcase` only for the optional minimal interface.
Historic minimal-app performance results do not automatically qualify the editor.
The fresh 90-second editor drive passed physical/scene/process checks at 0.9513x
real time (102 simulated seconds / 107.225 wall seconds); strict display timing
remains open. The regular interface is retained as requested.
Renderer completions are notifications, not independently verified unique
monitor frames. Actual hover/click latency and long-term memory stability are
not qualified by these tests.

Diagnostics (not smooth-display qualification):

```text
python scripts/demo_loop_showcase.py --version v07 --render-hz 60 --drive-seconds 15
python scripts/demo_loop_showcase.py --version v07 --profile-render --drive-seconds 15
```

Presentation buffers records and samples optional native/display audits; use
full mode for every-tick/every-display agreement audits and durable checkpoints.
Neither mode reduces the contact/clearance/road guards or commanded physics ticks.

The same separately supplied Americano v07 and Isaac Sim are required. No asset
publication, new training or LiDAR driving is implied. Default remains v04 when
no version is supplied.
