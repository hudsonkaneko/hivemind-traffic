"""Boundary-inclusive accounting for an explicit, fixed cohort of requested trips."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class TripRequest:
    vehicle_id: str
    requested_at: float
    role: str = 'human_proxy'
    origin: str = 'unspecified'
    initial: bool = False


def read_requests(routes: Path, controlled_ids=()) -> list[TripRequest]:
    """Require expanded numeric departures; never silently omit flows or trips."""
    root = ET.parse(routes).getroot()
    unsupported = [p.tag for p in root if p.tag not in {'vehicle', 'route', 'vType'}]
    if unsupported:
        raise ValueError(f'Expand demand to explicit vehicles first: {unsupported}')
    requests = []
    for vehicle in root.findall('vehicle'):
        try:
            depart = float(vehicle.attrib['depart'])
        except (KeyError, ValueError) as exc:
            raise ValueError('Every vehicle needs a numeric scheduled departure') from exc
        params = {p.attrib['key']: p.attrib['value'] for p in vehicle.findall('param')}
        vid = vehicle.attrib['id']
        requests.append(TripRequest(vid, depart, 'controlled' if vid in controlled_ids else 'human_proxy',
                                    params.get('research.origin', 'unspecified')))
    if not requests:
        raise ValueError('Demand must contain at least one explicit vehicle')
    return requests


class TripLedger:
    """Count failed/removed trips as unserved through H, never as quick successes.

    Departures use native departure times. Arrivals use the event observation
    boundary (up to one SUMO step later than native tripinfo arrival). Initial
    vehicles accrue time from the observation boundary, not unobserved history.
    """

    def __init__(self, requests: list[TripRequest]):
        self.time = 0.
        self.trips = {}
        for request in requests:
            if request.vehicle_id in self.trips:
                raise ValueError('Duplicate vehicle ID in fixed cohort')
            if not math.isfinite(request.requested_at) or request.requested_at < 0:
                raise ValueError('Request times must be finite and nonnegative')
            if request.initial and request.requested_at != 0:
                raise ValueError('Initial cohort must start at observation time zero')
            self.trips[request.vehicle_id] = dict(**asdict(request),
                state='active' if request.initial else 'waiting',
                departed_at=0. if request.initial else None, arrived_at=None,
                removed_at=None, removal_reason=None, had_teleport=False)

    def observe(self, time: float, *, departed=None, arrived=(), active=(),
                teleport_started=(), teleport_ended=()):
        if not math.isfinite(time) or time <= self.time:
            raise ValueError('Observation times must strictly increase')
        departed = departed or {}
        ids = set(departed) | set(arrived) | set(active) | set(teleport_started) | set(teleport_ended)
        if ids - self.trips.keys():
            raise ValueError(f'Observed IDs outside declared cohort: {sorted(ids-self.trips.keys())}')
        for vid, stamp in departed.items():
            trip = self.trips[vid]
            if trip['departed_at'] is not None or not trip['requested_at'] <= stamp <= time:
                raise ValueError(f'Invalid/duplicate departure: {vid}')
            trip.update(departed_at=float(stamp), state='active')
        for vid in teleport_started:
            trip = self.trips[vid]
            if trip['state'] != 'active':
                raise ValueError(f'Teleport before active trip: {vid}')
            trip.update(state='teleporting', had_teleport=True)
        for vid in teleport_ended:
            trip = self.trips[vid]
            if trip['state'] != 'teleporting':
                raise ValueError(f'Teleport end without start: {vid}')
            trip['state'] = 'active'
        for vid in arrived:
            trip = self.trips[vid]
            if trip['state'] not in {'active', 'teleporting'}:
                raise ValueError(f'Arrival without active trip: {vid}')
            if trip['had_teleport']:
                trip.update(state='removed', removed_at=time, removal_reason='arrival_after_teleport')
            else:
                trip.update(state='completed', arrived_at=time)
        for vid in active:
            if self.trips[vid]['state'] not in {'active', 'teleporting'}:
                raise ValueError(f'Active ID without valid lifecycle: {vid}')
        for vid, trip in self.trips.items():
            if trip['state'] == 'active' and vid not in active:
                trip.update(state='removed', removed_at=time, removal_reason='unannounced_disappearance')
        self.time = float(time)

    def snapshot(self):
        rows = []
        for trip in self.trips.values():
            row = dict(trip)
            due = trip['requested_at'] <= self.time
            row['state'] = trip['state'] if due else 'scheduled'
            endpoint = trip['arrived_at'] if trip['state'] == 'completed' else self.time
            row['boundary_time_s'] = max(0., endpoint-trip['requested_at']) if due else 0.
            departure = trip['departed_at'] if trip['departed_at'] is not None else self.time
            row['external_wait_s'] = max(0., min(self.time, departure)-trip['requested_at']) if due else 0.
            rows.append(row)

        def summarize(items):
            counts = {key:sum(t['state']==key for t in items) for key in
                      ('scheduled','waiting','active','teleporting','completed','removed')}
            requested = len(items)-counts['scheduled']
            return dict(**counts,requested=requested,initial=sum(t['initial'] for t in items),
                unfinished=requested-counts['completed'],
                boundary_time_s=sum(t['boundary_time_s'] for t in items),
                external_wait_s=sum(t['external_wait_s'] for t in items),
                teleported=sum(t['had_teleport'] for t in items))
        groups = {}
        for field in ('role','origin'):
            for value in sorted({t[field] for t in rows}):
                groups[f'{field}:{value}'] = summarize([t for t in rows if t[field]==value])
        total = summarize(rows)
        assert total['requested'] == sum(total[k] for k in ('waiting','active','teleporting','completed','removed'))
        return dict(time=self.time,totals=total,groups=groups,trips=rows,
            metric='Sum from requested departure to observed successful arrival or horizon; failed/removed trips accrue through horizon',
            initial_cohort='Time before observation boundary is excluded',
            comparison_valid=total['removed']==0 and total['teleported']==0)
