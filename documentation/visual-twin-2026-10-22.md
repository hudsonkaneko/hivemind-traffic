# 22 October 2026 — Supplied-car replay verification

Status: bounded rendering and replay-state checks verified. Manual GUI clicking,
steady-state performance and vehicle physics are not verified. This completes
the render portion of stages A–B in [Sam's scoped plan](sam-development-plan.md).
Branch: `codex/sam-visual-twin`; base `edb7125`, initial implementation `f825538`.
The successful run used that commit with the capture/runtime fixes in this PR
uncommitted. A subsequent readiness log line does not change replay behavior.

## Purpose and acceptance

Show the supplied car using recorded SUMO poses with explicit units, axis and
front-bumper anchoring, reusable visual references and immutable saved layers.
SUMO owns motion. UI speed/position come from recorded state, not RTX sensors.
The run must resolve active assets, match source poses/headings within 1e-6,
preserve lifecycle and saved layers, produce two inspectable camera PNGs, pass
control/state checks and exit successfully. No traffic benefit is being tested.

## Environment and reproduction

Windows 11 Enterprise, Intel i9-7900X, approximately 64 GB RAM, RTX A6000
(48 GB), NVIDIA driver 582.16. Isolated Python 3.12.14, Isaac Sim 6.0.1.0,
Torch 2.11.0+cu128, torchvision 0.26.0+cu128, torchaudio 2.11.0+cu128.
`pip check` passed; a small CUDA tensor operation passed on the A6000.
The existing Python 3.11 traffic environment was kept separate.

From the checkout, using the appropriate environment's Python:

```text
python -m pytest tests/test_visual_twin.py -q
python scripts/check_visual_twin.py outputs/twin-20261022-a/scene-v02/world.usda --report outputs/twin-20261022-a/composed-checks-v02.json
python scripts/view_visual_twin.py --scene outputs/twin-20261022-a/scene-v02/world.usda --headless --frames 360 --check-controls --evidence outputs/twin-20261022-a/render-attempt06
```

These are the executed arguments; select a fresh output/report name to repeat.
Handle NVIDIA's license prompt before unattended execution. Use the portable
launcher in [the usage guide](visual-twin.md) for the interactive GUI.

## Evidence and outcomes

All raw artifacts below are ignored local files, not supplied by a Git clone.
The private geometry and rendered images were not uploaded.

| Check | Observed result |
|---|---|
| Public-fixture tests | 3 passed; poses, lifecycle, immutability, invalid times, overwrite refusal, heading wrap |
| Full recording composition | 1,350 vehicle/frame checks; maximum position error 0 m; 30 vehicle mesh instances bound; no active unresolved assets |
| RTX bounded run | 360 updates; `follow.png` and `overview.png` written and visually inspected |
| Control/state checks | play, pause, seek, reset, overview, selected-car follow, playback advancement all true |
| Replay advancement | 5.0000 to 8.5352 seconds during the bounded check |
| Saved scene layers | hashes unchanged |
| Shutdown | process exit 0, shutdown log present |

Evidence root: `outputs/twin-20261022-a/`. Main records:
`manifest.json`, `composed-checks-v02.json`, `render-attempt06/runtime.json`,
`isaac-environment.lock.txt`; log: `logs/twin-render-attempt06.log`.
Source recording and network are copied unchanged from baseline seed 2001.
The mean app-update duration includes warm-up and is not an FPS benchmark.
Camera/selection checks exercise state and callbacks; they are not manual
mouse interaction. Road strips are schematic, including visible segment seams.

## Retained failures and repairs

- Initial Python installation needed an explicit local managed-runtime path.
- Isaac's first launch stopped at its license prompt; Sam then explicitly
  accepted the agreement and authorized the bounded launch.
- Attempt 02 failed loading a native DLL from the deeply nested environment.
  Moving the newly installed runtime to a short user-writable path fixed it.
  A prior move to a drive-root location was denied; no target was created there.
- Attempt 03 had no completed viewport images, a deferred seek check failed,
  and native shutdown asserted. Viewport rendering was enabled in headless mode,
  the timeline was updated before checking seek, and capture shutdown was repaired.
- Attempt 04 produced its first image after shader warm-up but no final report.
  Attempt 05 identified a renderer callback/file-write race. Both are failed
  attempts even though the old fast shutdown returned exit 0. The viewer now
  waits for the complete PNG and explicitly preserves nonzero failure status.
- Attempt 06 passed the declared bounded checks. No failed record was replaced.

The original archive still contains missing inactive texture references.
Explicit fallback materials resolve the active selected-car scene; this does
not restore the original textures or establish redistribution rights.

## Next gate and document synchronization

The synthetic replay is usable engineering scaffolding. Next work is site/data
qualification and Esri alignment alongside complete requested/waiting/active/
completed/unfinished accounting. The baseline still ends with three background
trips active. A calibrated empirical merge, learned communicating policy,
calibrated dynamics and bounded physics-response validation remain separate.

The live master and Sam refresher were read this session. The shared Google Doc
write hold remains; a local attributed update is prepared in the research base.
This milestone does not establish team-wide adoption or teammate capacity.
