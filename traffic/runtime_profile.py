"""Low-overhead wall-clock stage profiling for bounded runtime loops."""
from contextlib import contextmanager
import math
import time


class RuntimeProfiler:
    """Collect exclusive stage timings and a residual for each outer iteration."""

    def __init__(self, clock=time.perf_counter_ns):
        self.clock = clock
        self.samples_ns = {}
        self.iteration_samples_ns = []
        self.loop_wall_ns = 0
        self.iterations = 0
        self._iteration_start_ns = None
        self._iteration_stage_total_ns = 0
        self._active_stage = None

    @property
    def iteration_active(self):
        return self._iteration_start_ns is not None

    @contextmanager
    def measure(self, stage):
        if not stage or stage == 'loop_other':
            raise ValueError('Stage name must be nonempty and not reserved')
        if self._iteration_start_ns is None:
            raise RuntimeError('Stage timing requires an active iteration')
        if self._active_stage is not None:
            raise RuntimeError('Stage timings cannot be nested')
        start = self.clock()
        self._active_stage = stage
        try:
            yield
        finally:
            elapsed = max(0, self.clock() - start)
            self.samples_ns.setdefault(stage, []).append(elapsed)
            self._iteration_stage_total_ns += elapsed
            self._active_stage = None

    def begin_iteration(self):
        if self._iteration_start_ns is not None:
            raise RuntimeError('Previous iteration has not been finished')
        self._iteration_start_ns = self.clock()
        self._iteration_stage_total_ns = 0

    def finish_iteration(self):
        if self._iteration_start_ns is None:
            raise RuntimeError('No iteration is active')
        if self._active_stage is not None:
            raise RuntimeError('Cannot finish an iteration inside a stage timing scope')
        elapsed = max(0, self.clock() - self._iteration_start_ns)
        other = max(0, elapsed - self._iteration_stage_total_ns)
        self.samples_ns.setdefault('loop_other', []).append(other)
        self.iteration_samples_ns.append(elapsed)
        self.loop_wall_ns += elapsed
        self.iterations += 1
        self._iteration_start_ns = None
        self._iteration_stage_total_ns = 0

    def calibrate_overhead(self, samples=200):
        """Estimate instrumentation overhead using a separate empty profiler."""
        if type(samples) is not int or samples < 1:
            raise ValueError('Positive calibration sample count required')
        calibration = RuntimeProfiler(clock=self.clock)
        started = self.clock()
        for _ in range(samples):
            calibration.begin_iteration()
            with calibration.measure('_calibration'):
                pass
            calibration.finish_iteration()
        wall = max(0, self.clock() - started)
        scope_overhead = sum(calibration.samples_ns.get('loop_other', ()))
        per_scope = scope_overhead / samples
        measured_scope_count = sum(len(values) for name, values in self.samples_ns.items()
                                   if name != 'loop_other')
        return dict(calibration_samples=samples,
                    calibration_scope_overhead_ns=scope_overhead,
                    calibration_overhead_per_scope_ns=per_scope,
                    estimated_loop_scope_overhead_ns=per_scope * measured_scope_count,
                    estimated_loop_scope_count=measured_scope_count,
                    calibration_wall_s=wall / 1e9,
                    method='separate profiler with empty stage scopes; approximate')

    def estimate_overhead_for_run(self, calibration):
        """Scale isolated per-scope calibration by this run's measured scopes."""
        report = dict(calibration)
        scope_count = sum(len(values) for name, values in self.samples_ns.items()
                          if name != 'loop_other')
        per_scope = report['calibration_overhead_per_scope_ns']
        report['estimated_loop_scope_count'] = scope_count
        report['estimated_loop_scope_overhead_ns'] = per_scope * scope_count
        return report

    def summary(self, *, simulation_time_s=None):
        stages = {}
        total_stage_ns = 0
        for name, values in self.samples_ns.items():
            ordered = sorted(values)
            total = sum(values)
            total_stage_ns += total
            stages[name] = dict(
                count=len(values), total_s=total / 1e9,
                p50_ms=_percentile(ordered, .50) / 1e6,
                p95_ms=_percentile(ordered, .95) / 1e6,
                max_ms=(ordered[-1] / 1e6 if ordered else 0.0))
        wall_s = self.loop_wall_ns / 1e9
        iteration_ordered = sorted(self.iteration_samples_ns)
        return dict(clock='time.perf_counter_ns', iterations=self.iterations,
                    loop_wall_s=wall_s, simulation_time_s=simulation_time_s,
                    iteration_timing=dict(count=len(iteration_ordered), total_s=wall_s,
                        p50_ms=_percentile(iteration_ordered, .50) / 1e6,
                        p95_ms=_percentile(iteration_ordered, .95) / 1e6,
                        max_ms=(iteration_ordered[-1] / 1e6 if iteration_ordered else 0.0)),
                    wall_to_sim_ratio=(wall_s / simulation_time_s
                                       if simulation_time_s and simulation_time_s > 0 else None),
                    stage_coverage_percent=(100 * total_stage_ns / self.loop_wall_ns
                                            if self.loop_wall_ns else 0.0),
                    non_overlapping_stages=True, stages=stages)


def _percentile(ordered, fraction):
    if not ordered:
        return 0
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]
