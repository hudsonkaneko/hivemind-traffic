"""Bounded window/layout readiness; importable without Isaac Sim.

Call the native adapter only after SimulationApp is constructed. Once physics
has started, ``pump`` must be RenderedPhysicsSession.render, never app.update.
Readiness measures UI extents/docking, not successful RTX presentation or FPS.
"""
from __future__ import annotations

import math
import time


class ShowcaseStartupError(RuntimeError):
    """A bounded readiness failure with JSON-serializable retained evidence."""

    def __init__(self, reason, evidence):
        super().__init__(f'Showcase UI startup failed: {reason}')
        self.evidence = evidence


def window_launch_config(graphics, *, gui):
    """Copy graphics options; do not persist a startup resize to user settings.

    The dimensions are SimulationApp's existing defaults, made explicit in the
    evidence. No persistent setting keys, fullscreen, DPI or layout are changed.
    """
    if type(gui) is not bool:
        raise ValueError('gui must be boolean')
    result = dict(graphics)
    result['extra_args'] = list(graphics.get('extra_args', ()))
    if gui:
        for key, default in (('window_width', 1440), ('window_height', 900)):
            value = result.setdefault(key, default)
            if type(value) is not int or value <= 0:
                raise ValueError(f'{key} must be a positive integer')
        # Append last so an inherited value cannot cause the repair to persist.
        result['extra_args'].append('--/app/window/saveSizeOnExit=false')
    return result


def _extent(value):
    """Normalize dimensions for JSON; invalid/unavailable dimensions stay null."""
    try:
        if len(value) != 2:
            return None
        result = [float(dimension) for dimension in value]
        if not all(math.isfinite(dimension) for dimension in result):
            return None
        return result
    except (TypeError, ValueError, OverflowError):
        return None


def _valid_extent(value):
    return value is not None and all(dimension > 0 for dimension in value)


def wait_for_showcase_ready(*, pump, sample, is_running, request_dock=None,
                            repair_window=None, gui=True, require_sidebar=False,
                            timeout_s=15., max_updates=120, stable_updates=3,
                            retry_every_updates=4, clock=time.perf_counter):
    """Pump lifecycle frames and require consecutive valid observed layouts.

    ``sample`` returns native_window_extent, viewport_resolution,
    viewport_window_extent, viewport_visible, and sidebar_docked. GUI-only
    fields are deliberately ignored in headless mode. Repair is attempted once,
    after three observed invalid native extents; docking retries are spaced by
    lifecycle updates. Both frame and wall bounds apply, including pump cost.
    A native call blocked inside pump still needs the supervisor's hard timeout.
    """
    if type(gui) is not bool or type(require_sidebar) is not bool:
        raise ValueError('gui and require_sidebar must be boolean')
    for name, value in (('max_updates', max_updates), ('stable_updates', stable_updates),
                        ('retry_every_updates', retry_every_updates)):
        if type(value) is not int or value <= 0:
            raise ValueError(f'{name} must be a positive integer')
    if stable_updates > max_updates:
        raise ValueError('stable_updates cannot exceed max_updates')
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('timeout_s must be finite and positive')
    started = clock()
    evidence = dict(schema_version=1, passed=False, gui=gui,
                    sidebar_required=gui and require_sidebar,
                    timeout_s=timeout_s, max_updates=max_updates,
                    stable_updates_required=stable_updates, updates=0,
                    observations=[], actions=[],
                    scope='Startup UI extents and docking; not renderer/FPS qualification')
    stable = invalid_native = 0
    repaired = False
    last_dock = -retry_every_updates

    def fail(reason, error=None):
        evidence.update(reason=reason, elapsed_s=clock() - started,
                        stable_updates_observed=stable)
        failure = ShowcaseStartupError(reason, evidence)
        if error is not None:
            raise failure from error
        raise failure

    for update in range(1, max_updates + 1):
        if not is_running():
            fail('application_closed')
        if clock() - started >= timeout_s:
            fail('wall_timeout')
        try:
            pump()
        except Exception as error:
            evidence['actions'].append(dict(update=update, action='pump_error',
                                            error=f'{type(error).__name__}: {error}'))
            fail('lifecycle_update_failed', error)
        evidence['updates'] = update
        # Never accept a successful-looking frame that exceeded the deadline.
        if clock() - started >= timeout_s:
            fail('wall_timeout')
        if not is_running():
            fail('application_closed')
        try:
            raw = sample()
            row = {key: _extent(raw.get(key)) for key in (
                'native_window_extent', 'viewport_resolution', 'viewport_window_extent')}
            row.update(update=update, elapsed_s=clock() - started,
                       viewport_visible=raw.get('viewport_visible') is True,
                       sidebar_docked=raw.get('sidebar_docked') is True)
        except Exception as error:
            evidence['actions'].append(dict(update=update, action='sample_error',
                                            error=f'{type(error).__name__}: {error}'))
            fail('readiness_sample_failed', error)
        reasons = []
        if not _valid_extent(row['viewport_resolution']):
            reasons.append('invalid_viewport_resolution')
        if gui:
            native_ok = _valid_extent(row['native_window_extent'])
            invalid_native = 0 if native_ok else invalid_native + 1
            if not native_ok:
                reasons.append('invalid_native_window_extent')
            if not _valid_extent(row['viewport_window_extent']) or not row['viewport_visible']:
                reasons.append('viewport_not_visible_or_invalid_extent')
            if require_sidebar and not row['sidebar_docked']:
                reasons.append('sidebar_not_docked')
        row['pending'] = reasons
        evidence['observations'].append(row)
        stable = 0 if reasons else stable + 1
        if clock() - started >= timeout_s:
            fail('wall_timeout')
        if stable >= stable_updates:
            evidence.update(passed=True, reason='ready', elapsed_s=clock() - started,
                            stable_updates_observed=stable)
            return evidence
        if gui and invalid_native >= 3 and repair_window is not None and not repaired:
            repaired = True
            try:
                repair_window()
                evidence['actions'].append(dict(update=update, action='resize_invalid_window'))
            except Exception as error:
                evidence['actions'].append(dict(update=update, action='resize_error',
                                                error=f'{type(error).__name__}: {error}'))
                fail('window_repair_failed', error)
        if (gui and require_sidebar and not row['sidebar_docked'] and request_dock is not None
                and _valid_extent(row['native_window_extent'])
                and update - last_dock >= retry_every_updates):
            last_dock = update
            try:
                request_dock()
                evidence['actions'].append(dict(update=update, action='request_dock'))
            except Exception as error:
                evidence['actions'].append(dict(update=update, action='dock_error',
                                                error=f'{type(error).__name__}: {error}'))
                fail('sidebar_dock_failed', error)
    fail('update_limit')


def wait_for_native_showcase_ready(*, pump, is_running, viewport, sidebar=None,
                                  gui=True, window_size=(1440, 900), **limits):
    """Adapter to installed Kit public APIs; no native imports at module import.

    ``window_launch_config`` must be applied before SimulationApp so any native
    repair cannot save its size on exit. No settings/layout files are written.
    """
    import omni.appwindow
    from omni.kit.viewport.utility import get_active_viewport_window

    if _extent(window_size) is None or not _valid_extent(_extent(window_size)):
        raise ValueError('window_size must contain two finite positive dimensions')
    required = gui and sidebar is not None and bool(sidebar.window.visible)

    def sample():
        native = omni.appwindow.get_default_app_window() if gui else None
        visible = get_active_viewport_window() if gui else None
        frame = visible.frame if visible is not None else None
        return dict(
            native_window_extent=(native.get_width(), native.get_height()) if native else None,
            viewport_resolution=viewport.resolution,
            viewport_window_extent=(frame.computed_width, frame.computed_height) if frame else None,
            viewport_visible=(bool(visible.visible) and
                              (not visible.docked or bool(visible.selected_in_dock))) if visible else False,
            sidebar_docked=bool(sidebar.docked) if required else False)

    def repair_window():
        native = omni.appwindow.get_default_app_window()
        if native is None:
            raise RuntimeError('Native application window unavailable')
        native.restore_window()
        native.resize(int(window_size[0]), int(window_size[1]))

    return wait_for_showcase_ready(
        pump=pump, sample=sample, is_running=is_running, gui=gui,
        require_sidebar=required, request_dock=sidebar.dock if required else None,
        repair_window=repair_window if gui else None, **limits)
