"""Low-frequency instrumentation for bounded desktop runtime experiments."""
from collections import deque
import gc
import math
import time


def refresh_sensor_packets(update, packets, *, frames=6, clock=time.perf_counter):
    """Refresh stationary scans after expensive setup, before setting epochs.

    Does not advance SUMO or relax the normal runtime sensor freshness check.
    """
    if type(frames) is not int or frames < 1 or not packets:
        raise ValueError('Positive frame count and sensor packet sinks required')
    previous = {vid: packet.get('timestamp', -1) for vid, packet in packets.items()}
    for _ in range(frames):
        update()
    now = clock()
    ages = {}
    for vid, packet in packets.items():
        age = now - packet.get('received_wall', -math.inf)
        if packet.get('timestamp', -1) <= previous[vid] or not -.001 <= age <= .25:
            raise RuntimeError('Sensor failed post-setup refresh: ' + vid)
        ages[vid] = age
    return dict(frames=frames, receipt_ages_s=ages)


class MemorySampler:
    """Sample process RSS at simulation-time intervals without collecting objects."""

    def __init__(self, interval_seconds=10., max_samples=32, process=None):
        if not math.isfinite(interval_seconds) or interval_seconds <= 0 or max_samples < 2:
            raise ValueError('Positive finite interval and at least two samples required')
        self.interval_seconds = float(interval_seconds)
        self.samples = deque(maxlen=max_samples)
        self._next = None
        self._first = None
        self._peak = None
        self.error = None
        if process is None:
            try:
                import psutil
                process = psutil.Process()
            except ImportError:
                self.error = 'psutil unavailable'
        self.process = process

    def sample(self, simulation_time, force=False):
        simulation_time = float(simulation_time)
        if not math.isfinite(simulation_time):
            raise ValueError('Finite simulation time required')
        if self.process is None or (not force and self._next is not None and simulation_time < self._next):
            return None
        started = time.perf_counter()
        try:
            info = self.process.memory_info()
        except (OSError, RuntimeError) as exc:
            self.error = repr(exc)
            return None
        row = dict(simulation_time=simulation_time, wall_time=started, rss_bytes=int(info.rss),
                   sampling_seconds=time.perf_counter()-started)
        if hasattr(info, 'private'):
            row['private_bytes'] = int(info.private)
        self.samples.append(row)
        self._first = row if self._first is None else self._first
        self._peak = max(self._peak or 0, row['rss_bytes'])
        self._next = simulation_time + self.interval_seconds
        return row

    def summary(self):
        rows = list(self.samples)
        result = dict(available=bool(rows), error=self.error, samples=rows,
                      rss_start_bytes=None, rss_end_bytes=None, rss_delta_bytes=None,
                      rss_peak_bytes=self._peak, last_half_slope_bytes_per_second=None)
        if not rows:
            return result
        first, last = self._first, rows[-1]
        result.update(rss_start_bytes=first['rss_bytes'], rss_end_bytes=last['rss_bytes'],
                      rss_delta_bytes=last['rss_bytes']-first['rss_bytes'])
        midpoint=(first['simulation_time']+last['simulation_time'])/2
        tail=[r for r in rows if r['simulation_time'] >= midpoint]
        if len(tail) >= 2:
            xs=[r['simulation_time'] for r in tail]; ys=[r['rss_bytes'] for r in tail]
            xmean=sum(xs)/len(xs); ymean=sum(ys)/len(ys)
            denominator=sum((x-xmean)**2 for x in xs)
            if denominator:
                result['last_half_slope_bytes_per_second']=sum((x-xmean)*(y-ymean) for x,y in zip(xs,ys))/denominator
        return result


class ScopedGC:
    """Restore GC state after a bounded run; never unfreeze somebody else's heap."""

    def __init__(self, mode='normal', max_events=4096):
        if mode not in ('normal', 'deferred', 'freeze-established') or max_events < 1:
            raise ValueError('Invalid GC mode or event limit')
        self.mode=mode; self.effective_mode=mode
        self.events=deque(maxlen=max_events); self._active={}
        self.total_seconds=0.; self.collection_count=0; self._owns_freeze=False
        self.note=None

    def _observe(self, phase, info):
        generation=info['generation']; now=time.perf_counter()
        if phase=='start':
            self._active[generation]=now
        elif generation in self._active:
            start=self._active.pop(generation); duration=now-start
            self.events.append(dict(start=start, end=now, generation=generation, duration_seconds=duration))
            self.total_seconds+=duration; self.collection_count+=1

    def __enter__(self):
        self._enabled=gc.isenabled()
        gc.callbacks.append(self._observe)
        try:
            if self.mode=='deferred':
                gc.collect(); gc.disable()
            elif self.mode=='freeze-established':
                if gc.get_freeze_count():
                    self.effective_mode='normal'
                    self.note='Existing frozen objects preserved; freezing skipped'
                else:
                    gc.collect(); gc.freeze(); self._owns_freeze=True
                    gc.enable()
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, exc_type, exc, traceback):
        if self._observe in gc.callbacks:
            gc.callbacks.remove(self._observe)
        if self._owns_freeze:
            gc.unfreeze(); self._owns_freeze=False
        if self._enabled:
            gc.enable()
        else:
            gc.disable()

    def summary(self):
        return dict(requested_mode=self.mode, effective_mode=self.effective_mode, note=self.note,
                    collection_count=self.collection_count, total_collection_seconds=self.total_seconds,
                    events=list(self.events))
