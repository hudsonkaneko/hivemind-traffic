"""CPU tests for reporting classification and copying expiring native buffers."""
from types import SimpleNamespace

import pytest

from traffic.vehicle_contacts import classify_vehicle_contacts, VehicleContactMonitor


def header(**changes):
    values = dict(type=0, stage_id=42, actor0='/World/Car_0', actor1='/World/Barrier',
                  collider0='/World/Car_0/ChassisCollision', collider1='/World/Barrier',
                  contact_data_offset=0, num_contact_data=1)
    values.update(changes)
    return SimpleNamespace(**values)


def point():
    return SimpleNamespace(position=[1., 2., 3.], normal=[-1., 0., 0.],
                           impulse=[4., 0., 0.], separation=-.001)


def classify(headers, data=None, tick=1):
    return classify_vehicle_contacts(headers, [point()] if data is None else data,
                                     vehicle_path='/World/Car_0', stage_id=42, tick=tick,
                                     path_converter=str, event_type_names={0:'found', 2:'persist'})


def test_vehicle_contact_copies_json_primitives():
    native = point()
    event = classify([header()], [native])[0]
    native.position[0] = 999
    assert event['contacts'][0]['position'][0] == 1.
    assert event['contacts'][0]['separation_m'] == -.001
    assert event['event_type'] == 'found'
    assert event['collider1'] == '/World/Barrier'


def test_persist_included_lost_and_other_stage_ignored():
    result = classify([header(type=2), header(type=1), header(stage_id=43)])
    assert [e['event_type'] for e in result] == ['persist']


def test_chassis_ground_is_not_whitelisted_as_tire_support():
    result = classify([header(actor1='', collider1='/World/GroundPlane/CollisionPlane')])
    assert len(result) == 1


def test_actor_order_and_collider_membership_are_symmetric():
    assert classify([header(actor0='', actor1='/World/Car_0', collider0='/World/Barrier',
                            collider1='/World/Car_0/ChassisCollision')])
    assert classify([header(actor0='', actor1='', collider0='/World/Barrier',
                            collider1='/World/Car_0/Wheel0/Collision')])


def test_path_prefix_does_not_match_another_vehicle():
    assert not classify([header(actor0='/World/Car_01', collider0='/World/Car_01/ChassisCollision')])


def test_invalid_buffer_range_fails_instead_of_hiding_contact():
    with pytest.raises(RuntimeError, match='range'):
        classify([header(num_contact_data=2)])


def test_nonfinite_contact_fails():
    native = point()
    native.separation = float('nan')
    with pytest.raises(RuntimeError, match='non-finite'):
        classify([header()], [native])


def test_contact_without_points_still_counts():
    assert classify([header(num_contact_data=0)], [])[0]['contact_count'] == 0


def test_tick_validation():
    for tick in (-1, True, 1.5):
        with pytest.raises(ValueError):
            classify([header()], tick=tick)


def test_sample_preserves_copies_and_rejects_duplicate_tick():
    monitor = VehicleContactMonitor.__new__(VehicleContactMonitor)
    monitor.vehicle_path = '/World/Car_0'
    monitor.stage_id = 42
    monitor.events = []
    monitor._last_tick = None
    monitor._path_converter = str
    monitor._event_type_names = {0:'found', 2:'persist'}
    native = point()
    sim = SimpleNamespace(get_contact_report=lambda: ([header()], [native]))
    events = monitor.sample(sim, 1)
    native.position[0] = 999
    assert events[0]['contacts'][0]['position'][0] == 1.
    assert monitor.events == events
    with pytest.raises(ValueError):
        monitor.sample(sim, 1)
