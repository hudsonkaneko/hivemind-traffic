"""Process-local preview quality profiles and measured soft real-time pacing.

Never changes simulation dt, sensor cadence, or skips physics steps. A slow
machine falls behind and reports it rather than silently changing the experiment.
"""
import math
import time


def rendering_profile(real_time=False):
    if type(real_time) is not bool:
        raise ValueError('Expected boolean real-time setting')
    if real_time:
        return dict(renderer='MinimalRendering', minimal_shading_mode=2,
                    width=960, height=600, anti_aliasing=2, limit_cpu_threads=8,
                    extra_args=['--/app/vsync=false'])
    return dict(renderer='RealTimePathTracing', minimal_shading_mode=0,
                width=1280, height=800, anti_aliasing=3)


class PreviewClock:
    """Track completed frames against a 1x wall clock, excluding user pauses."""

    def __init__(self, *, paced, clock=time.perf_counter, sleep=time.sleep):
        self.clock, self.sleep, self.paced = clock, sleep, paced
        self.started = clock()
        self.frames = []

    def finish_frame(self, sim_time_s, paused_wall_s=0.):
        elapsed = self.clock() - self.started - paused_wall_s
        if self.paced and elapsed < sim_time_s:
            self.sleep(sim_time_s - elapsed)
        elapsed = self.clock() - self.started - paused_wall_s
        previous_wall = self.frames[-1]['active_wall_s'] if self.frames else 0.
        row = dict(sim_time_s=sim_time_s, active_wall_s=elapsed,
                   lag_s=max(0., elapsed-sim_time_s),
                   frame_interval_s=elapsed-previous_wall,
                   rtf=sim_time_s/elapsed if elapsed > 0 else None)
        self.frames.append(row)
        return row

    def summary(self, *, expected_simulation_time_s=None, expected_frames=None):
        lags = sorted(frame['lag_s'] for frame in self.frames)
        last = self.frames[-1] if self.frames else {}
        rtf = last.get('rtf')
        p95 = lags[max(0, math.ceil(.95*len(lags))-1)] if lags else None
        maximum = max(lags) if lags else None
        intervals = sorted(frame['frame_interval_s'] for frame in self.frames)
        # Whole episode, including settle/capture/checkpoint work, but excluding
        # initialization and explicit pauses. Soft timing, not hard deadlines.
        complete = ((expected_frames is None or len(self.frames) == expected_frames)
                    and (expected_simulation_time_s is None
                         or math.isclose(last.get('sim_time_s', 0.), expected_simulation_time_s,
                                         rel_tol=0., abs_tol=1e-9)))
        passed = bool(complete and self.paced and rtf is not None and .99 <= rtf <= 1.01
                      and p95 <= .10 and maximum <= .50)
        return dict(paced=self.paced, completed_frames=len(self.frames),
                    simulation_time_s=last.get('sim_time_s', 0.),
                    active_wall_s=last.get('active_wall_s', 0.), rtf=rtf,
                    p95_lag_s=p95, max_lag_s=maximum, soft_realtime_passed=passed,
                    expected_simulation_time_s=expected_simulation_time_s,
                    expected_frames=expected_frames,
                    expected_window_complete=(complete if expected_simulation_time_s is not None
                                              or expected_frames is not None else None),
                    p95_frame_interval_s=(intervals[math.ceil(.95*len(intervals))-1] if intervals else None),
                    max_frame_interval_s=(max(intervals) if intervals else None),
                    acceptance=dict(rtf_min=.99, rtf_max=1.01,
                                    p95_lag_max_s=.10, max_lag_max_s=.50),
                    scope='Live loop including settle and recording; excludes startup and explicit pauses')
