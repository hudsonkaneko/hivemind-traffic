"""Crash-safe, immutable per-second chunks for trajectory and planner evidence."""
import json
import os
from pathlib import Path
import tempfile


CHUNK_FORMAT = 'hivemind-traffic-evidence-chunk-v1'
CHECKPOINT_STRIDE_TICKS = 120


class EvidenceChunkWriter:
    """Write append-only trajectory and planner checkpoints to a fresh directory."""

    def __init__(self, output_dir):
        self.directory = Path(output_dir) / 'checkpoints'
        self.directory.mkdir(parents=False, exist_ok=False)
        _sync_directory(self.directory.parent)
        self._row_count = 0
        self._planner_count = 0
        self._last_tick = 0
        self._chunk_paths = []

    def write_checkpoint(self, rows, planner):
        """Atomically publish only records not included in the prior checkpoint."""
        if len(rows) <= self._row_count:
            raise ValueError('Checkpoint must contain new trajectory rows')
        if len(planner) < self._planner_count:
            raise ValueError('Planner records cannot be removed between checkpoints')
        new_rows = rows[self._row_count:]
        new_planner = planner[self._planner_count:]
        first_tick = _tick(new_rows[0])
        last_tick = _tick(new_rows[-1])
        if first_tick != self._last_tick + 1:
            raise ValueError('Trajectory checkpoints must be contiguous')
        if any(_tick(row) != first_tick + index for index, row in enumerate(new_rows)):
            raise ValueError('Trajectory rows must be contiguous and ordered')
        if last_tick - self._last_tick != CHECKPOINT_STRIDE_TICKS:
            raise ValueError('Checkpoint must cover exactly one simulation second')
        previous_plan_tick = -1
        if self._planner_count:
            previous_plan_tick = _tick(planner[self._planner_count - 1])
        plan_ticks = [_tick(record) for record in new_planner]
        if (any(tick <= previous_plan_tick or tick < self._last_tick or tick > last_tick for tick in plan_ticks)
                or plan_ticks != sorted(set(plan_ticks))):
            raise ValueError('Planner checkpoints must be ordered and fall within saved trajectory')

        chunk_index = len(self._chunk_paths) + 1
        target = self.directory / f'checkpoint-{chunk_index:05d}.json'
        payload = dict(format=CHUNK_FORMAT, chunk_index=chunk_index,
                       checkpoint_stride_ticks=CHECKPOINT_STRIDE_TICKS,
                       first_tick=first_tick, last_tick=last_tick,
                       trajectory=new_rows, planner=new_planner)
        temporary = None
        try:
            descriptor, temporary = tempfile.mkstemp(prefix='.checkpoint-', suffix='.tmp', dir=self.directory)
            with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as stream:
                json.dump(payload, stream, indent=2, allow_nan=False)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            # Hard-link publication is atomic and fails if the immutable name exists.
            os.link(temporary, target)
            self._row_count = len(rows)
            self._planner_count = len(planner)
            self._last_tick = last_tick
            self._chunk_paths.append(target)
            _sync_directory(self.directory)
        finally:
            if temporary is not None:
                try:
                    os.unlink(temporary)
                except FileNotFoundError:
                    pass

        return target

    def summary(self):
        return dict(format=CHUNK_FORMAT, directory=self.directory.name,
                    checkpoint_stride_ticks=CHECKPOINT_STRIDE_TICKS,
                    chunks=[path.name for path in self._chunk_paths],
                    saved_rows=self._row_count, saved_planner_records=self._planner_count,
                    last_tick=self._last_tick, durable_through_tick=self._last_tick)


def _tick(record):
    tick = record.get('tick') if isinstance(record, dict) else None
    if type(tick) is not int or tick < 0:
        raise ValueError('Evidence records require a nonnegative integer tick')
    return tick


def _sync_directory(directory):
    if os.name == 'nt':
        return
    descriptor = os.open(directory, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
