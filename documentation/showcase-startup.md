# Showcase launch and window readiness

October 16, 2026 (Pacific). Implementation and CPU fault-injection tests are
complete. Native repeat qualification must be recorded separately; these tests
do not establish that the original intermittent startup fault is fixed.

## Retained failure and interpretation

The clean-install run `20261015T180043Z-2210d3c9` reported a renderer swapchain
creation request with width **120** and height **0**, followed by failed graphics
initialization. Its final sidebar record was `docked: false`. The first renderer
failure occurred during `SimulationApp` construction, around extension startup,
before project sidebar construction and before the driving clock. Later resize
errors remained in the runtime log. The launch arguments already requested a
1440 by 900 application window and a 960 by 600 render resolution.

The unchanged-source retry `20261015T220110Z-c47ce2fb` passed simulation and
viewer gates. That establishes an intermittent failure, not its cause. The
retained logs do not establish whether desktop minimization, display/DPI state,
saved settings, window initialization, or another cause produced the zero
height. No external runtime or persistent user layout was edited to pursue a
speculative cause.

Raw source evidence remains in the disposable clean-install checkout described
in [clean-clone readiness](clean-clone.md), under those original run IDs. It has
not been rewritten, copied into source control, or reclassified as a pass.

## Behavior and boundaries

`visualization/showcase_startup.py` separates three observable quantities:

- Native application window extent, measured by the installed window API.
- Viewport image resolution, plus computed UI frame extent and visibility for
  GUI runs. A configured image resolution alone cannot prove a visible window.
- The actual sidebar `docked` property, rather than assuming a dock request
  completed synchronously.

The launch helper records `SimulationApp`'s existing 1440 by 900 window defaults
explicitly, preserves independently configured render dimensions, and passes
`/app/window/saveSizeOnExit=false` for this process. It does not change
`/persistent/` settings, fullscreen, monitor selection, DPI, or saved layouts.
Making the existing defaults explicit is evidence hygiene, not a claimed fix
for the 120 by 0 startup condition.

After normal initial render warm-up, the readiness helper services native
lifecycle frames with `RenderedPhysicsSession.render`. That method verifies
zero advancement of the native physics counter and simulation clock. The helper
does not call `SimulationApp.update`, start/stop the timeline, or step physics.

Readiness requires **three consecutive** observations with positive native and
visible viewport dimensions and, for a visible sidebar, confirmed docking. A
transient valid frame followed by an invalid frame resets this count. Headless
runs require a valid viewport resolution but deliberately skip native-window,
visibility, and docking requirements. A hidden sidebar is not required to dock.

An invalid native extent observed for three updates permits **one** native
restore/resize request to the configured window dimensions. This is an attempted
repair, never proof of readiness. The subsequent observations must pass. The
window-size save option prevents that repair from persisting its size on exit.
No healthy window is resized. Docking retries occur at most once every four
lifecycle frames and only after the native window extent is valid. The current
sidebar owns its docking target choice; missing targets cannot be treated as a
successful dock.

The default limits are **120 lifecycle updates or 15 wall seconds**, whichever
is reached first. Wall checks include completed native pump and sampling costs;
a late successful-looking frame is rejected. Closing the application or a
native API error fails readiness immediately. A native call that never returns
still requires the supervisor's existing process timeout.

The helper returns JSON-safe evidence or raises `ShowcaseStartupError` with the
same `.evidence` payload. The runner retains it as `startup-readiness.json` on
both success and failure, including observations, actions, elapsed time, and the
precise terminal reason. Call-site failures also remain in the normal run result.

**A readiness pass does not erase earlier renderer errors.** The existing
supervisor still rejects runtime errors, including an earlier failed swapchain,
even if the window later becomes usable. Positive extents also do not prove
image correctness, renderer presentation, mouse latency, or display cadence.
Viewport capture and existing physical/scene/process checks remain necessary.

## Installed API evidence

Read-only inspection used the installed Isaac Sim 6.0.1-rc.7 runtime:

| Component | Source/API checked |
| --- | --- |
| `isaacsim.simulation_app` | `simulation_app.py`: separate `width`/`height` and `window_width`/`window_height`, forwarding `extra_args` |
| `omni.appwindow` 1.2.2 | `docs/SETTINGS.md`: `app.window.saveSizeOnExit`; `_appwindow.pyi`: `get_default_app_window`, `get_width`, `get_height`, `restore_window`, `resize` |
| `omni.kit.viewport.utility` 2.0.1 | `get_active_viewport_window`, returning the current `ViewportWindow` |
| `omni.kit.viewport.window` 109.0.2 | `ViewportWindow` derives from `omni.ui.Window` |
| `omni.ui` 3.2.0 | Frame computed extents; window `visible`, `docked`, `selected_in_dock`; native `dock_in` |
| `omni.kit.widget.viewport` 109.2.0 | `ViewportAPI.resolution`, independent of native window dimensions |

These are installed public APIs, not filesystem modifications or calls into a
running simulator during investigation. The adapter imports Isaac modules only
when called inside an initialized application, so CPU validation remains usable
on a clean checkout without Isaac installed.

## Verification and acceptance

From the repository root, using the existing traffic environment:

```text
python -m pytest tests/test_showcase_startup.py -q
```

Initial CPU result: **17 passed, 20 subtests passed**. Fault injection covers
120 by 0 extents, delayed docking, a successful request with no resulting state
change, unstable layout, headless and hidden-panel behavior, elapsed-time and
update limits, user closure, and native API exceptions. The fake native adapter
checks public property use. No simulation or training is launched by this suite.

Native qualification should use repeated fresh processes with the same public
vehicle, seed, configuration, and source hashes. Every repeat must retain:
valid startup-readiness evidence, a readable viewport capture, confirmed visible
sidebar docking where required, all existing safety/scene/process gates, no
runtime errors, and clean shutdown. Report failed attempts alongside successes.
The initial failure occurred in application construction, so even repeated
success only narrows the uncertainty; the historical cause remains unconfirmed
unless a controlled reproduction establishes it.

The regular editor remains the default. Minimal-editor comparisons must also
have a real supported docking target if their sidebar is visible. Performance
measurements exclude this startup readiness period and retain the existing
fixed physics/control settings and safety tolerances.
