import json
import math
import xml.etree.ElementTree as ET

import pytest

from traffic.highway_merge_context import Stage1LaneGraph
from traffic.highway_v05_demand import (
    CAP, DIFFICULTIES, MAIN_ROUTE, PROFILES, _safe_gap, demand_report, validate_initial_spawns, write_demand,
)


@pytest.fixture
def graph(tmp_path):
    root = ET.Element('net')
    for edge, count, length in [(edge, 5 if i < 3 else 4, 400.) for i, edge in enumerate(MAIN_ROUTE)] + [('mlk_on', 1, 350.)]:
        element = ET.SubElement(root, 'edge', id=edge)
        for index in range(count):
            ET.SubElement(element, 'lane', id=f'{edge}_{index}', index=str(index), length=str(length), speed=str(CAP))
    for i, (source, target) in enumerate(zip(MAIN_ROUTE, MAIN_ROUTE[1:])):
        for lane in range(1 if i == 2 else 0, 5 if i < 3 else 4):
            ET.SubElement(root, 'connection', {'from': source, 'to': target, 'fromLane': str(lane), 'toLane': str(lane - int(i == 2))})
    ET.SubElement(root, 'connection', {'from': 'mlk_on', 'to': 'sb2', 'fromLane': '0', 'toLane': '0'})
    path = tmp_path / 'network.xml'
    ET.ElementTree(root).write(path)
    return Stage1LaneGraph(path, 4)


def physical(rows):
    return [{key: value for key, value in row.items() if key not in ('vehicle_id', 'controlled')} for row in rows]


@pytest.mark.parametrize('total', [8, 16, 24])
@pytest.mark.parametrize('fraction', [0., .25, .5, 1.])
def test_complete_initial_population_and_stable_controller_ids(graph, tmp_path, total, fraction):
    count = round(total * fraction)
    path = tmp_path / 'routes.xml'
    rows = write_demand(path, 9, graph, 'merge', count, total)
    native = ET.parse(path).findall('vehicle')
    assert len(rows) == len(native) == total
    assert all(row['initial_cohort'] and row['depart'] == 0. for row in rows)
    assert {row['vehicle_id'] for row in rows if row['controlled']} == {f'agent_{i}' for i in range(count)}
    assert sum(vehicle.get('type') == 'controlled' for vehicle in native) == count
    assert 1 <= sum(row['route'] == 'ramp' for row in rows) <= 4
    if count:
        assert next(row for row in rows if row['vehicle_id'] == 'agent_0')['route'] == 'ramp'
    assert json.loads(json.dumps(rows)) == rows


@pytest.mark.parametrize('mode', ['initial', 'sustained'])
def test_seed_and_assignment_are_independent_and_nested(graph, tmp_path, mode):
    def demand(count=4, assignment_seed=99, seed=123):
        return write_demand(tmp_path / f'{count}-{assignment_seed}-{seed}.xml', seed, graph, 'merge', count, 24,
                            traffic_mode=mode, assignment_seed=assignment_seed)
    low, high = demand(), demand(20)
    assert physical(low) == physical(high)
    assert low == demand()
    low_ids = {row['slot']: row['vehicle_id'] for row in low if row['controlled']}
    high_ids = {row['slot']: row['vehicle_id'] for row in high if row['controlled']}
    assert low_ids.items() <= high_ids.items()
    other_assignment = demand(assignment_seed=101)
    assert physical(low) == physical(other_assignment)
    assert low != other_assignment
    assert physical(low) != physical(demand(seed=124))


@pytest.mark.parametrize('task', ['following', 'passing'])
def test_native_slow_leader_is_counted_and_never_assigned(graph, tmp_path, task):
    rows = write_demand(tmp_path / 'routes.xml', 42, graph, task, 7, 8, speed_limit_m_s=12.)
    leader = next(row for row in rows if row['slot'] == 'slow_leader')
    assert not leader['controlled'] and leader['vehicle_id'] == 'slow_leader'
    assert len(rows) == 8 and sum(row['controlled'] for row in rows) == 7
    assert leader['speed'] == pytest.approx(7.2 if task == 'passing' else 9.6)
    with pytest.raises(ValueError, match='native slow leader'):
        write_demand(tmp_path / 'bad.xml', 42, graph, task, 8, 8)


def test_sustained_arrivals_and_fraction_dilution_are_explicit(graph, tmp_path):
    rows = write_demand(tmp_path / 'routes.xml', 88, graph, 'merge', 8, 16,
                        difficulty=2, traffic_mode='sustained', stabilization_seconds=5., horizon_seconds=30.)
    initial = [row for row in rows if row['initial_cohort']]
    later = [row for row in rows if not row['initial_cohort']]
    assert len(initial) == 16 and later
    assert all(not row['controlled'] and row['edge'] == 0 and row['position'] == 10. for row in later)
    assert all(8. <= row['depart'] < 35. for row in later)
    assert len({row['depart'] for row in later}) == len(later)
    assert [row['depart'] for row in rows] == sorted(row['depart'] for row in rows)
    report = demand_report(rows)
    assert report['requested_av_fraction'] == .5
    assert report['scheduled_all_departure_av_fraction'] == 8 / len(rows) < .5
    assert report['scheduled_fraction_dilution'] > 0.
    assert report['scheduled_later_native_count'] == len(later)
    assert report['traffic_mode'] == 'sustained' and report['difficulty'] == 2
    assert report['physical_seed'] == report['assignment_seed'] == 88
    with pytest.raises(ValueError, match='100% AV'):
        write_demand(tmp_path / 'bad.xml', 88, graph, 'merge', 16, 16, traffic_mode='sustained')


def test_initial_demand_is_unchanged_by_later_arrivals(graph, tmp_path):
    initial = write_demand(tmp_path / 'initial.xml', 8, graph, 'merge', 8, 24)
    sustained = write_demand(tmp_path / 'sustained.xml', 8, graph, 'merge', 8, 24, traffic_mode='sustained')
    assert initial == [row for row in sustained if row['initial_cohort']]


def test_exact_failed_seed_departure_matches_native_milliseconds_and_strict_ledger(graph, tmp_path):
    from traffic.trip_accounting import TripLedger, TripRequest, read_requests
    # Failed validation seed6900011 authored this value. SUMO 1.27.1 converts
    # it to8200ms, so the strict ledger correctly rejects the old precision.
    raw_departure = 8.200348149208052
    old = TripLedger([TripRequest('inflow_2', raw_departure)])
    with pytest.raises(ValueError, match='Invalid/duplicate departure'):
        old.observe(8.3, departed={'inflow_2': 8.2}, active=['inflow_2'])
    path = tmp_path / 'routes.xml'
    rows = write_demand(path, 6900011, graph, 'merge', 12, 24, speed_limit_m_s=22.352,
                        difficulty=1, traffic_mode='sustained', stabilization_seconds=0., horizon_seconds=80.)
    requests = read_requests(path)
    row = next(row for row in rows if row['slot'] == 'inflow_2')
    request = next(request for request in requests if request.vehicle_id == 'inflow_2')
    assert row['depart'] == request.requested_at == 8.2
    assert abs(row['depart'] - raw_departure) < .0005
    ledger = TripLedger([request])
    ledger.observe(8.3, departed={'inflow_2': 8.2}, active=['inflow_2'])
    assert ledger.snapshot()['trips'][0]['state'] == 'active'
    assert len(rows) == 52  # No arrival dropped to repair the failure.
    assert all(request.requested_at == next(row['depart'] for row in rows if row['vehicle_id'] == request.vehicle_id)
               for request in requests)
    assert all(row['depart'] == round(row['depart'] * 1000.) / 1000. for row in rows)


@pytest.mark.parametrize('difficulty', [0, 1, 2])
def test_sustained_requests_survive_strict_native_time_accounting(graph, tmp_path, difficulty):
    from traffic.trip_accounting import TripLedger, read_requests
    for seed in (88, 6900008, 6900009, 6900010, 6900011):
        path = tmp_path / f'{seed}.xml'
        rows = write_demand(path, seed, graph, 'merge', 8, 24, difficulty=difficulty,
                            traffic_mode='sustained', stabilization_seconds=.1234, horizon_seconds=80.)
        ledger = TripLedger(read_requests(path))
        # Native SUMO parses integer milliseconds then inserts at a 100ms step.
        # Account every requested vehicle, with no dropped or ignored IDs.
        events = {}
        for row in rows:
            native_ms = int(row['depart'] * 1000. + .5)
            insertion_ms = ((native_ms + 99) // 100) * 100
            events.setdefault(insertion_ms, []).append(row['vehicle_id'])
        active = []
        for stamp_ms, ids in sorted(events.items()):
            active.extend(ids)
            ledger.observe((stamp_ms + 100) / 1000., departed={vid: stamp_ms / 1000. for vid in ids}, active=active)
        assert ledger.snapshot()['totals']['active'] == len(rows)
        assert ledger.snapshot()['totals']['removed'] == 0


@pytest.mark.parametrize('difficulty', [0, 1, 2])
@pytest.mark.parametrize('ceiling', [6., 12., 18., CAP])
def test_static_clearance_and_native_limits(graph, tmp_path, difficulty, ceiling):
    graph.speed_limits['mlk_on_0'] = 14.
    rows = write_demand(tmp_path / 'routes.xml', 48, graph, 'merge', 12, 24,
                        speed_limit_m_s=ceiling, difficulty=difficulty)
    validate_initial_spawns(graph, rows, ceiling=ceiling)
    assert all(math.isfinite(row['speed']) and row['speed'] <= min(ceiling, graph.speed_limits[row['lane_id']]) for row in rows)
    types = {element.get('id'): element.attrib for element in ET.parse(tmp_path / 'routes.xml').findall('vType')}
    assert types['controlled']['tau'] == '1.5' and types['controlled']['accel'] == '2'
    assert types['controlled']['decel'] == '4.5' and types['controlled']['carFollowModel'] == 'ACC'
    assert len({types[name]['tau'] for name in PROFILES}) == 3
    assert all(1.5 <= float(types[name]['tau']) <= 2. and float(types[name]['speedFactor']) <= 1. for name in PROFILES)


def test_speed_spacing_ramp_counts_and_profiles_vary_with_seed(graph, tmp_path):
    rows = [write_demand(tmp_path / f'{seed}.xml', seed, graph, 'merge', 8, 24, difficulty=2) for seed in range(20)]
    ramp_counts = {sum(row['route'] == 'ramp' for row in demand) for demand in rows}
    assert ramp_counts == {1, 2, 3, 4}
    assert any(row['speed'] > .85 * CAP for demand in rows for row in demand if row['route'] == 'main')
    assert len({row['position'] for demand in rows for row in demand}) > 100
    assert {row['driver_profile'] for demand in rows for row in demand} == set(PROFILES)


def test_declared_difficulty_increases_density_and_speed_spread(graph, tmp_path):
    reports = []
    spans = []
    for difficulty in range(3):
        rows = write_demand(tmp_path / f'{difficulty}.xml', 8, graph, 'cruise', 8, 24, difficulty=difficulty)
        speeds = [row['speed'] for row in rows]
        reports.append(max(speeds) - min(speeds))
        spans.append(max(row['edge'] * 400. + row['position'] for row in rows))
    assert reports[0] < reports[1] < reports[2]
    assert spans[0] > spans[1] > spans[2]
    assert DIFFICULTIES[0]['arrival_interval_s'] > DIFFICULTIES[2]['arrival_interval_s']


def test_failed_following_insertion_gap_retains_speed_and_safety():
    from traffic.highway_stage1_teacher import acc_response_ceiling
    # Seed6900021 had collision-safe spacing but ACC's stationary insertion
    # speed was below the explicitly requested20.2135m/s; native insertion queued.
    rear_speed, front_speed = 20.21351833671351, 16.
    old_bumper_gap = 307.44022254371237 - 235.40888435683965 - 4.8
    assert acc_response_ceiling(rear_speed, 24.5872, (old_bumper_gap, front_speed)) < 0.
    new_bumper_gap = _safe_gap(rear_speed, front_speed)
    assert new_bumper_gap > old_bumper_gap
    assert acc_response_ceiling(rear_speed, 24.5872, (new_bumper_gap, front_speed)) >= -1e-12
    for rear, front in ((6., 3.), (12., 5.), (18., 16.), (24., 10.), (24., 24.)):
        gap = _safe_gap(rear, front)
        assert gap >= 2.5 + 2. * rear + max(0., rear ** 2 - front ** 2) / 9.
        assert acc_response_ceiling(rear, CAP, (gap, front)) >= -1e-12


@pytest.mark.parametrize('difficulty', [0, 1, 2])
def test_all_initial_followers_have_nonnegative_acc_insertion_response(graph, tmp_path, difficulty):
    from traffic.highway_stage1_teacher import acc_response_ceiling
    rows = write_demand(tmp_path / 'routes.xml', 6900021, graph, 'following', 18, 24,
                        speed_limit_m_s=CAP, difficulty=difficulty, traffic_mode='sustained')
    initial = [row for row in rows if row['initial_cohort']]
    assert len(initial) == 24 and sum(row['controlled'] for row in initial) == 18
    for rear in initial:
        distances = graph.distances(rear['lane_id'], rear['position'], MAIN_ROUTE)
        fronts = [(distances[front['lane_id']] + front['position'] - 4.8, front)
                  for front in initial if front is not rear and front['lane_id'] in distances
                  and distances[front['lane_id']] + front['position'] > 0]
        if fronts:
            gap, front = min(fronts, key=lambda item: item[0])
            assert acc_response_ceiling(rear['speed'], CAP, (gap, front['speed'])) >= -1e-12


def test_short_ramp_reduces_ramp_count_without_losing_cohort(graph, tmp_path):
    graph.lanes['mlk_on_0']['length'] = 90.
    rows = write_demand(tmp_path / 'routes.xml', 9, graph, 'merge', 8, 24, speed_limit_m_s=12.)
    assert len(rows) == 24
    assert 1 <= sum(row['route'] == 'ramp' for row in rows) <= 2
    graph.lanes['mlk_on_0']['length'] = 21.
    with pytest.raises(ValueError, match='Ramp too short'):
        write_demand(tmp_path / 'bad.xml', 9, graph, 'merge', 8, 24)
    assert not (tmp_path / 'bad.xml').exists()


def test_invalid_spawn_fails_without_writing_partial_demand(graph, tmp_path):
    for row in graph.lanes.values():
        if row['edge'].startswith('sb'):
            row['length'] = 25.
    with pytest.raises(ValueError, match='exceeds available route'):
        write_demand(tmp_path / 'bad.xml', 99, graph, 'cruise', 8, 24)
    assert not (tmp_path / 'bad.xml').exists()


def test_clearance_validator_catches_duplicate_position_and_bad_speed(graph, tmp_path):
    rows = write_demand(tmp_path / 'routes.xml', 9, graph, 'cruise', 8, 24)
    a, b = rows[0], next(row for row in rows[1:] if row['lane_id'] == rows[0]['lane_id'])
    b['position'] = a['position']
    with pytest.raises(ValueError, match='Unsafe initial spawn gap'):
        validate_initial_spawns(graph, rows)
    a['speed'] = CAP + 1.
    with pytest.raises(ValueError, match='Invalid spawn speed'):
        validate_initial_spawns(graph, rows)


@pytest.mark.parametrize('options', [dict(controlled_count=25), dict(controlled_count=True),
    dict(total_vehicles=12), dict(difficulty=3), dict(difficulty=.5), dict(speed_limit_m_s=float('nan')),
    dict(speed_limit_m_s=30.), dict(traffic_mode='mystery'), dict(assignment_seed=-1),
    dict(horizon_seconds=0.), dict(stabilization_seconds=-1.), dict(traffic_mode='sustained', horizon_seconds=2.)])
def test_invalid_scenario_configuration_is_rejected(graph, tmp_path, options):
    kwargs = dict(controlled_count=8, total_vehicles=24)
    kwargs.update(options)
    with pytest.raises(ValueError):
        write_demand(tmp_path / 'bad.xml', 9, graph, 'merge', **kwargs)
    assert not (tmp_path / 'bad.xml').exists()


def test_optional_ramp_distance_preserves_legacy_geometry_and_varies_only_ramp():
    from scripts.build_highway_curriculum_v2 import source_geometry
    from scripts.build_highway_environment_v05_bank import specifications
    options = dict(lanes=4, length=2200., bend=20., speed_mph=60.)
    legacy = source_geometry(**options)
    explicit_default = source_geometry(**options, ramp_upstream_distance=160.)
    assert [ET.tostring(element) for element in legacy[:3]] == [ET.tostring(element) for element in explicit_default[:3]]
    assert legacy[3] == explicit_default[3]
    short = source_geometry(**options, ramp_upstream_distance=120.)
    long = source_geometry(**options, ramp_upstream_distance=240.)
    assert [ET.tostring(element) for element in short[1:3]] == [ET.tostring(element) for element in long[1:3]]
    assert [ET.tostring(node) for node in short[0] if node.get('id') != 'ramp_start'] == [
        ET.tostring(node) for node in long[0] if node.get('id') != 'ramp_start']
    assert short[3]['ramp_start_xy'] != long[3]['ramp_start_xy']
    specs = specifications()
    assert len(specs) == 6
    assert {row['ramp_upstream_distance'] for row in specs if row['split'] == 'train'} == {120., 240.}
    assert {row['ramp_upstream_distance'] for row in specs if row['split'] == 'validation'} == {180.}
    assert all(row['split'] != 'test' for row in specs)
    for invalid in (float('nan'), 0., -1., 500., True):
        with pytest.raises(ValueError, match='Ramp upstream distance'):
            source_geometry(**options, ramp_upstream_distance=invalid)


@pytest.mark.parametrize('speed,remaining', [(9.70,26.),(10.,100.),(7.,25.),(20.,30.)])
def test_ramp_departure_can_stop_for_yield_before_assignment(speed,remaining):
    from traffic.highway_v05_demand import _ramp_insertion_speed
    actual=_ramp_insertion_speed(speed,remaining)
    assert 0. <= actual <= speed
    assert 2.1*actual+actual**2/9.+2.5 <= remaining+1e-9
    if remaining>=100.: assert actual==speed
