"""Bounded, opt-in native render diagnostics; importing does not load Kit.

The installed Kit profiler supplies inclusive CPU/GPU zones in milliseconds.
These diagnostics add overhead and are not performance-qualification evidence.
The owner calls ``after_render`` after its normal, clock-checked app update and
``close`` in its cleanup path. This helper never pumps Kit or steps physics.
"""
from __future__ import annotations

from copy import deepcopy
import math


_GPU_ENABLED = '/profiler/enabled'
_CPU_SAVE = '/plugins/carb.profiler-cpu.plugin/saveProfile'
_CAPTURE_MASK = 0xFFFFFFFFFFFFFFFF


def _load_runtime(viewport):
    """Use native interfaces, without loading the optional profiler window."""
    import carb.profiler
    import carb.settings
    import omni.hydra.engine.stats
    import omni.kit.app

    return dict(
        app=omni.kit.app.get_app(),
        settings=carb.settings.get_settings(),
        cpu=carb.profiler.acquire_profiler_interface(plugin_name='carb.profiler-mux.plugin'),
        monitor=carb.profiler.acquire_profile_monitor_interface(),
        gpu=omni.hydra.engine.stats.HydraEngineStats(
            viewport.usd_context_name, viewport.hydra_engine),
    )


def _nodes(raw_nodes, minimum_ms, *, gpu=False):
    """Retain original ordering/indentation, including indices before filtering."""
    retained = []
    raw_count = 0
    root_ms = 0.
    for index, node in enumerate(raw_nodes):
        raw_count += 1
        duration = float(node['duration'])
        indent = node['indent']
        if (not math.isfinite(duration) or duration < 0
                or type(indent) is not int or indent < 0):
            raise ValueError('Native profiler returned an invalid duration or indentation')
        if indent == 0:
            root_ms += duration
        if duration < minimum_ms:
            continue
        name = node['name']
        if name is not None and not isinstance(name, str):
            raise ValueError('Native profiler returned an invalid zone name')
        if not name:
            name = f"query-{node['query']}" if gpu and 'query' in node else '(unnamed)'
        retained.append(dict(name=name, duration_ms=duration, indent=indent, raw_index=index))
    return dict(raw_node_count=raw_count, retained_node_count=len(retained),
                root_total_ms=root_ms, nodes=retained)


class RenderProfile:
    """Capture at most the next 60 native app updates, then restore settings.

    CPU and GPU snapshots may describe different completed native frames; their
    API does not supply a common frame identifier here. App update numbers mark
    when data was read, not a claim of CPU/GPU frame alignment. Empty snapshots
    remain explicit missing evidence, including when a backend is unavailable.
    """

    def __init__(self, viewport, *, max_updates=60, minimum_ms=.1):
        if type(max_updates) is not int or not 1 <= max_updates <= 60:
            raise ValueError('Render profiling must be bounded to 1..60 native app updates')
        if (type(minimum_ms) not in (int, float) or not math.isfinite(minimum_ms)
                or minimum_ms < .1):
            raise ValueError('Render-profile minimum duration must be at least 0.1 ms')
        self.max_updates = max_updates
        self.minimum_ms = float(minimum_ms)
        self._runtime = None
        self._settings_before = {}
        self._settings_pending = set()
        self._mask_before = None
        self._mask_pending = False
        self._samples = []
        self._error = None
        self._cleanup_errors = []
        self._closed = False
        self._window_exhausted = False
        self._start_update = None
        self._last_update = None
        try:
            runtime = self._runtime = _load_runtime(viewport)
            if any(runtime.get(name) is None for name in ('app', 'settings', 'cpu', 'monitor', 'gpu')):
                raise RuntimeError('A required native profiler interface is unavailable')
            self._start_update = self._last_update = self._update_number()
            self._mask_before = runtime['cpu'].get_capture_mask()
            for name in (_GPU_ENABLED, _CPU_SAVE):
                self._settings_before[name] = runtime['settings'].get(name)
            # Mark each write pending before attempting it: a failing native
            # setter may already have changed the value.
            self._settings_pending.add(_CPU_SAVE)
            runtime['settings'].set_bool(_CPU_SAVE, False)
            self._settings_pending.add(_GPU_ENABLED)
            runtime['settings'].set_bool(_GPU_ENABLED, True)
            self._mask_pending = True
            runtime['cpu'].set_capture_mask(_CAPTURE_MASK)
        except Exception as error:
            self._fail('setup', error)

    def _update_number(self):
        value = self._runtime['app'].get_update_number()
        if type(value) is not int or value < 0:
            raise ValueError('Kit returned an invalid app update number')
        return value

    def _fail(self, operation, cause):
        self._error = f'{type(cause).__name__}: {cause}'
        cleanup_error = None
        try:
            self.close()
        except Exception as error:
            cleanup_error = error
        message = f'Native render profiler {operation} failed: {self._error}'
        if operation == 'setup':
            message += '; enable omni.hydra.engine.stats and verify native CPU profiler availability'
        if cleanup_error is not None:
            message += f'; restoration also failed: {cleanup_error}'
        raise RuntimeError(message) from cause

    def after_render(self):
        """Read one completed update without rendering; return whether sampled."""
        if self._error is not None:
            raise RuntimeError('Native render profiler previously failed: ' + self._error)
        if self._closed:
            return False
        try:
            update = self._update_number()
            if update < self._last_update:
                raise RuntimeError('Kit app update counter moved backwards')
            if update == self._last_update:
                return False
            self._last_update = update
            elapsed = update - self._start_update
            if elapsed > self.max_updates:
                self._window_exhausted = True
                self.close()
                return False
            events = self._runtime['monitor'].get_last_profile_events()
            if events is None:
                raise RuntimeError('Native CPU profiler returned no snapshot object')
            thread = events.get_main_thread_id()
            cpu = _nodes(events.get_profile_events(thread), self.minimum_ms)
            devices = self._runtime['gpu'].get_gpu_profiler_result()
            gpu = [dict(device_index=index, **_nodes(nodes, self.minimum_ms, gpu=True))
                   for index, nodes in enumerate(devices)]
            self._samples.append(dict(app_update_number=update, cpu=cpu, gpu=gpu))
            if elapsed >= self.max_updates:
                self._window_exhausted = True
                self.close()
            return True
        except Exception as error:
            self._fail('capture', error)

    def close(self):
        """Restore all changed state, trying every restoration even after failure.

        A failed restoration remains pending so the owner's cleanup can retry.
        Failures raise rather than claiming a successful diagnostic or cleanup.
        """
        if self._closed:
            return
        errors = []
        if self._runtime is not None:
            if self._mask_pending:
                try:
                    self._runtime['cpu'].set_capture_mask(self._mask_before)
                    self._mask_pending = False
                except Exception as error:
                    errors.append(f'CPU capture mask: {type(error).__name__}: {error}')
            for name in sorted(self._settings_pending):
                try:
                    previous = self._settings_before[name]
                    if previous is None:
                        self._runtime['settings'].destroy_item(name)
                    else:
                        self._runtime['settings'].set(name, previous)
                    self._settings_pending.remove(name)
                except Exception as error:
                    errors.append(f'{name}: {type(error).__name__}: {error}')
        if errors:
            self._cleanup_errors.extend(errors)
            raise RuntimeError('Native render profiler restoration failed: ' + '; '.join(errors))
        self._closed = True
        # Avoid retaining Hydra context/engine handles through stage shutdown.
        self._runtime = None

    def summary(self):
        """Return bounded scalar data; empty results cannot masquerade as a pass."""
        cpu_available = any(row['cpu']['raw_node_count'] for row in self._samples)
        gpu_available = any(device['raw_node_count'] for row in self._samples for device in row['gpu'])
        if self._error or self._cleanup_errors:
            status = 'failed'
        elif not self._samples:
            status = 'no_samples'
        elif not cpu_available or not gpu_available:
            status = 'missing_native_data'
        elif not self._window_exhausted:
            status = 'partial_window'
        else:
            status = 'captured'
        return dict(schema_version=1, enabled=True, diagnostic_status=status,
                    performance_qualification_eligible=False,
                    max_native_updates=self.max_updates, minimum_duration_ms=self.minimum_ms,
                    app_update_start=self._start_update, app_update_last=self._last_update,
                    sampled_updates=len(self._samples), window_exhausted=self._window_exhausted,
                    cpu_data_available=bool(cpu_available), gpu_data_available=bool(gpu_available),
                    settings_restored=self._closed, error=self._error,
                    cleanup_errors=list(self._cleanup_errors), samples=deepcopy(self._samples),
                    timing_scope='Native inclusive CPU/GPU zones; nested durations overlap. '
                                 'root_total_ms sums only indent-zero nodes before filtering.',
                    hierarchy='Native ordering and indentation retained; raw_index precedes threshold filtering.',
                    frame_alignment='Read after recorded app update; native CPU/GPU snapshot frames may differ.',
                    evidence_limit='Instrumented diagnostic only; empty native data is not a successful diagnosis.')

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
