"""Named sequential phase timing and explicit optional-audit cadence.

No physics, collision, controller or command-expiry checks are decimated here.
Presentation retains every safety/trajectory sample in memory and writes it on
normal Python exit; unlike full evidence it has no durable per-second recovery.
"""
from traffic.runtime_profile import RuntimeProfiler


class SequentialProfiler(RuntimeProfiler):
    """Use the existing summary format without nested instrumentation scopes."""
    def measure(self, stage):
        raise RuntimeError('Sequential profiling uses mark only; scopes would double-count time')

    def calibrate_overhead(self, samples=200):
        """Approximate the actual mark path, including amortized loop bookkeeping."""
        if type(samples) is not int or samples < 1:
            raise ValueError('Positive calibration sample count required')
        calibration = SequentialProfiler(clock=self.clock)
        marks_per_iteration = 14
        started = self.clock()
        for _ in range(samples):
            calibration.begin_iteration()
            for phase in range(marks_per_iteration):
                calibration.mark(str(phase))
            calibration.finish_iteration()
        wall = max(0, self.clock() - started)
        count = samples * marks_per_iteration
        return dict(calibration_samples=samples,calibration_scope_overhead_ns=wall,
            calibration_overhead_per_scope_ns=wall/count,
            estimated_loop_scope_overhead_ns=0.,estimated_loop_scope_count=0,
            calibration_wall_s=wall/1e9,
            method='separate sequential profiler with 14 empty marks/iteration; includes loop bookkeeping; approximate')

    def begin_iteration(self):
        super().begin_iteration()
        self._phase_start = self._iteration_start_ns

    def mark(self, stage):
        if not self.iteration_active or self._active_stage is not None:
            raise RuntimeError('Sequential mark requires an active unscoped iteration')
        if not stage or stage == 'loop_other':
            raise ValueError('Nonempty, nonreserved phase required')
        now = self.clock()
        elapsed = max(0, now - self._phase_start)
        self.samples_ns.setdefault(stage, []).append(elapsed)
        self._iteration_stage_total_ns += elapsed
        self._phase_start = now


def audit_schedule(config, tick):
    """tick is the completed physics tick (1-based), never a wall-clock sample."""
    if type(tick) is not int or tick < 1:
        raise ValueError('A positive completed physics tick is required')
    presentation = config.get('evidence_profile') == 'presentation'
    hz = config['physics_hz']
    display = tick % (hz // config['render_hz']) == 0
    return dict(native=(not presentation or tick == 1 or tick % hz == 0),
                visual=display and (not presentation or tick == hz // config['render_hz'] or tick % hz == 0),
                objects=tick % (hz if presentation else hz // config['planner_hz']) == 0,
                checkpoint=not presentation and tick % hz == 0)
