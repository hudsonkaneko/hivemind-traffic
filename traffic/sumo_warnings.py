"""Native SUMO emergency events, distinct from sampled vehicle acceleration."""
from __future__ import annotations

import hashlib
import re
from pathlib import Path


def parse_native_emergencies(text: str) -> dict:
    """Retain all warning lines; deduplicate within one episode log only."""
    events = {}
    lines = [line for line in text.splitlines() if any(
        phrase in line.lower() for phrase in ('emergency braking', 'emergency stop'))]
    for line in lines:
        vehicle = re.search(r"[Vv]ehicle ['\"]([^'\"]+)['\"]", line)
        timestamp = re.search(r'\btime\s*[=:]\s*([0-9]+(?:\.[0-9]+)?)', line)
        deceleration = re.search(r'\bdecel=([-+]?[0-9]+(?:\.[0-9]+)?)', line)
        kind = ('lane_end_emergency_stop' if 'emergency stop at the end of lane' in line.lower()
                else 'emergency_stop' if 'emergency stop' in line.lower() else 'emergency_braking')
        identity = vehicle.group(1) if vehicle else None
        at = float(timestamp.group(1)) if timestamp else None
        # Unknown warning formats remain distinct; never silently count them as zero.
        key = (identity, at, kind) if identity is not None and at is not None else ('unparsed', line)
        events[key] = dict(vehicle_id=identity, time_s=at, kind=kind, line=line,
            controlled=identity.startswith('agent_') if identity is not None else None,
            native_reported_decel=float(deceleration.group(1)) if deceleration else None)
    rows = list(events.values())
    return dict(warning_lines=len(lines), unique_events=len(rows),
        controlled_events=sum(e['controlled'] is True for e in rows),
        background_events=sum(e['controlled'] is False for e in rows),
        unclassified_events=sum(e['controlled'] is None for e in rows), events=rows)


def native_emergency_report(path: str | Path) -> dict:
    """Read only after the SUMO connection closes, so buffered logs are flushed."""
    path = Path(path)
    if not path.is_file():
        return dict(available=False, unique_events=None, reason='Native SUMO log missing')
    raw = path.read_bytes()
    return dict(available=True, log_sha256=hashlib.sha256(raw).hexdigest(),
        metric='Native warning events per episode; not sampled acceleration threshold crossings',
        **parse_native_emergencies(raw.decode('utf-8', errors='replace')))
