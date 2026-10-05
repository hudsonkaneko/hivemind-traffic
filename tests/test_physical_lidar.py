import math
import numpy as np
import pytest

from traffic.physical_lidar import (
    SENSOR_RATE_HZ, _configure_rotary_profile, packet_to_scan,
    rotation_matrix, world_to_chassis,
)


def test_world_to_chassis_uses_center_and_full_rotation():
    state = dict(position_m=[10,20,1], quaternion_xyzw=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)])
    assert np.allclose(world_to_chassis([[10,25,2]], state), [[5,0,1]])
    state['quaternion_xyzw'] = [math.sin(.1),0,0,math.cos(.1)]
    local = np.array([[5,1,-.5]])
    world = local@rotation_matrix(state['quaternion_xyzw']).T+state['position_m']
    assert np.allclose(world_to_chassis(world, state), local)


@pytest.mark.parametrize('q', [[0,0,0,0], [0,0,math.nan,1], [1,2,3]])
def test_reject_bad_rotation(q):
    with pytest.raises(ValueError):
        rotation_matrix(q)


@pytest.fixture
def packet():
    return dict(episode_id='ep', vehicle_id='ego', timestamp_ns=1_000_000_000,
                scan_complete=1, delivery_tick=126,
                xyz=np.tile([10.0, 3.0, 1.0], (21, 1)),
                flags=np.full(21, 64, dtype=np.uint32),
                offset_ns=np.linspace(0, 40_000_000, 21, dtype=np.int64))


def convert(packet, **kwargs):
    args = dict(episode_id='ep', vehicle_id='ego', tick=128)
    args.update(kwargs)
    state = dict(position_m=[0, 0, 1], quaternion_xyzw=[0, 0, 0, 1])
    return packet_to_scan(packet, state, **args)


def test_valid_packet_preserves_original_delivery_and_bounds(packet):
    scan = convert(packet)
    assert scan.acquisition_start_tick == 120
    assert scan.acquisition_end_tick == 125  # Outwards from 124.8.
    assert scan.delivery_tick == 126
    assert scan.coordinate_reference_tick == 128
    assert np.allclose(scan.points, np.tile([10, 3, 0], (21, 1)))


@pytest.mark.parametrize('nonfinite', [math.nan, math.inf, -math.inf])
def test_nonfinite_successful_hit_rejects_whole_packet_and_brakes(packet, nonfinite):
    from traffic.lidar_braking import LidarEmergencyBrake
    # Twenty otherwise-clear returns remain: merely dropping this potential
    # obstacle used to admit a clear cloud rather than reject the unknown hit.
    packet['xyz'][0] = [4.0, 0.0, nonfinite]
    scan = convert(packet)
    assert scan is None
    decision = LidarEmergencyBrake('ep', 'ego').evaluate(scan, tick=128, speed_m_s=3)
    assert decision.status == 'stale_invalid' and decision.brake_override == 1
    assert decision.target_speed_m_s == 0


def test_nonfinite_miss_is_not_mistaken_for_corrupt_success(packet):
    packet['flags'][0] = 0
    packet['xyz'][0] = [math.nan, math.inf, -math.inf]
    scan = convert(packet)
    assert scan is not None and len(scan.points) == 20


@pytest.mark.parametrize('key, value', [
    ('episode_id', 'other'), ('vehicle_id', 'other'),
    ('scan_complete', 0), ('scan_complete', 2), ('scan_complete', True),
    ('scan_complete', 1.0), ('timestamp_ns', math.nan),
    ('timestamp_ns', -1), ('timestamp_ns', True),
    ('timestamp_ns', 2**63), ('delivery_tick', 124),
    ('delivery_tick', 129), ('delivery_tick', True), ('delivery_tick', 126.0),
])
def test_reject_invalid_packet_identity_completion_or_timestamps(packet, key, value):
    packet[key] = value
    assert convert(packet) is None


@pytest.mark.parametrize('missing', ['timestamp_ns', 'xyz', 'flags', 'offset_ns',
                                     'scan_complete', 'delivery_tick', 'episode_id', 'vehicle_id'])
def test_missing_packet_field_fails_safe(packet, missing):
    del packet[missing]
    assert convert(packet) is None


@pytest.mark.parametrize('kind', ['negative', 'fractional', 'nan', 'too_long', 'overflow'])
def test_invalid_per_ray_timing_is_rejected(packet, kind):
    if kind == 'negative':
        packet['offset_ns'][0] = -1
    elif kind == 'fractional':
        packet['offset_ns'] = packet['offset_ns'].astype(float) + .5
    elif kind == 'nan':
        packet['offset_ns'] = packet['offset_ns'].astype(float)
        packet['offset_ns'][0] = math.nan
    elif kind == 'too_long':
        packet['offset_ns'][-1] = 70_000_000
    else:
        packet['offset_ns'] = np.full(21, 2**64-1, dtype=np.uint64)
    assert convert(packet) is None


@pytest.mark.parametrize('key, value', [
    ('flags', np.full(21, -1)), ('flags', np.full(21, 2**32)),
    ('flags', np.full(21, 64.5)), ('flags', np.full(20, 64)),
    ('flags', np.full(21, True)), ('xyz', np.zeros((21, 2))),
    ('xyz', np.zeros((0, 3))), ('offset_ns', np.zeros((21, 1), dtype=int)),
])
def test_bad_shapes_or_flag_encodings_are_rejected(packet, key, value):
    packet[key] = value
    assert convert(packet) is None


def test_acquisition_rounds_outwards_and_not_to_delivery(packet):
    packet['timestamp_ns'] = 1_003_000_000  # Physics tick 120.36.
    packet['offset_ns'] = np.linspace(0, 17_000_000, 21, dtype=np.int64)
    scan = convert(packet)
    assert scan.acquisition_start_tick == 120
    assert scan.acquisition_end_tick == 123  # Latest sample tick122.4 rounds up.
    assert scan.delivery_tick == 126


def test_epoch_subtraction_and_pre_epoch_rejection(packet):
    assert convert(packet, epoch_s=1).acquisition_start_tick == 0
    assert convert(packet, epoch_s=1.001) is None
    assert convert(packet, epoch_s=math.nan) is None
    assert convert(packet, epoch_s=True) is None


def test_duplicate_delivery_cannot_rejuvenate_acquisition_age(packet):
    from traffic.lidar_braking import LidarBrakeConfig, LidarEmergencyBrake
    assert LidarBrakeConfig().max_scan_age_ticks == 24  # 0.2s remains unchanged.
    brake = LidarEmergencyBrake('ep', 'ego')
    first = brake.evaluate(convert(packet), tick=128, speed_m_s=3)
    assert first.status == 'clear'
    late_scan = convert(packet, tick=145)
    assert late_scan.delivery_tick == 126 and late_scan.acquisition_start_tick == 120
    later = brake.evaluate(late_scan, tick=145, speed_m_s=3)
    assert later.status == 'stale_invalid' and later.reason == 'stale_scan'


def test_trace_labels_never_enter_brake_input(packet):
    plain = convert(packet)
    packet.update(object_ids=['obstacle'], semantic_labels={'obstacle': 'ignore'},
                  trace_labels={'nearest_gap_m': 9999}, obstacle_position=[9999, 0, 0])
    labeled = convert(packet)
    assert np.array_equal(labeled.points, plain.points)
    assert labeled.acquisition_start_tick == plain.acquisition_start_tick
    assert labeled.delivery_tick == plain.delivery_tick


def test_rate_profile_sets_and_reads_back_scan_and_tick_independently():
    class Attr:
        def __init__(self, name, value=None):
            self.name, self.value = name, value
        def GetName(self):
            return self.name
        def Get(self):
            return self.value
        def Set(self, value):
            self.value = value
            return True

    class Prim:
        def __init__(self):
            self.attrs = {'omni:sensor:tickRate': Attr('omni:sensor:tickRate', 10.0),
                          'omni:sensor:Core:scanRateBaseHz': Attr('omni:sensor:Core:scanRateBaseHz', 10),
                          'omni:sensor:Core:emitterState:s001:azimuthDeg':
                              Attr('omni:sensor:Core:emitterState:s001:azimuthDeg', list(range(128)))}
        def GetAttributes(self):
            return list(self.attrs.values())
        def GetAttribute(self, name):
            return self.attrs.setdefault(name, Attr(name))

    prim = Prim()
    actual = _configure_rotary_profile(prim)
    assert actual == dict(sensor_tick=20.0, rotary_scan=20.0, pattern_firing=7200.0)
    assert actual['sensor_tick'] == SENSOR_RATE_HZ
    assert len(prim.GetAttribute('omni:sensor:Core:emitterState:s001:azimuthDeg').Get()) == 32


def test_actual_constructor_enables_point_writer_before_sensor_snapshot_and_cleans_up(monkeypatch):
    """Reproduce the installed alias-snapshot behavior without importing Isaac.

    Installed _SensorRuntime copies WRITER_SPEC in its constructor. Registering
    the point writer afterward cannot repair that instance. Exercise the real
    project constructor, not a copied sequence or source-text ordering assertion.
    """
    import sys
    from types import ModuleType, SimpleNamespace
    from traffic.physical_lidar import PhysicalLidar

    events, shared_aliases, writer_registry = [], {}, {}

    class Attr:
        def __init__(self, name):
            self.name, self.value = name, None
        def GetName(self):
            return self.name
        def Get(self):
            return self.value
        def Set(self, value):
            self.value = value
            return True

    class Prim:
        def __init__(self):
            self.attrs = {}
        def GetAttributes(self):
            return list(self.attrs.values())
        def GetAttribute(self, name):
            return self.attrs.setdefault(name, Attr(name))

    prim = Prim()

    def enable_extension(name):
        events.append(('enable', name))
        assert name == 'isaacsim.sensors.rtx.nodes'
        shared_aliases['draw-point-cloud'] = 'RtxSensorDebugDrawPointCloud'

    class FakeLidar:
        @staticmethod
        def create(path, **kwargs):
            events.append(('create', path))
            assert kwargs['tick_rate'] == 20
            return SimpleNamespace(prims=[prim])

    class FakeLidarSensor:
        def __init__(self, lidar, *, annotators):
            events.append(('construct_sensor',))
            assert lidar.prims[0] is prim and annotators == []
            # Match installed _SensorRuntime: copy, not a live shared mapping.
            self.writer_specs = dict(shared_aliases)
            self.attached = {}

        def attach_writer(self, name, **kwargs):
            events.append(('attach', name))
            if name == 'draw-point-cloud':
                assert self.writer_specs.get(name) == 'RtxSensorDebugDrawPointCloud', (
                    'Point writer extension must be enabled before sensor construction')
                assert kwargs['size'] > 0 and len(kwargs['color']) == 4
                assert kwargs['doTransform'] is False  # WORLD returns must not transform twice.
                self.attached[name] = object()
            else:
                assert name in writer_registry
                self.attached[name] = writer_registry[name]()

        def detach_writer(self, name):
            events.append(('detach', name))
            assert name in self.attached
            del self.attached[name]

    def module(name, **attributes):
        result = ModuleType(name)
        result.__path__ = []
        result.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, result)
        if '.' in name:
            parent, child = name.rsplit('.', 1)
            setattr(sys.modules[parent], child, result)
        return result

    module('omni')
    module('omni.replicator')
    module('omni.replicator.core', Writer=type('Writer', (), {}),
           WriterRegistry=SimpleNamespace(register=lambda cls: writer_registry.__setitem__(cls.__name__, cls)),
           annotators=SimpleNamespace(get=lambda name: ('annotator', name)))
    for name in ('isaacsim', 'isaacsim.sensors', 'isaacsim.sensors.experimental'):
        module(name)
    module('isaacsim.sensors.experimental.rtx', Lidar=FakeLidar, LidarSensor=FakeLidarSensor,
           parse_generic_model_output_data=lambda data: data)
    for name in ('isaacsim.core', 'isaacsim.core.experimental', 'isaacsim.core.experimental.utils'):
        module(name)
    module('isaacsim.core.experimental.utils.app', enable_extension=enable_extension)
    module('isaacsim.core.rendering_manager',
           RenderingManager=SimpleNamespace(get_dt=lambda: 1 / 30))

    lidar = PhysicalLidar('/World/Vehicle', episode_id='ep', vehicle_id='ego',
                          tick_source=lambda: 12, points=True)
    sensor = lidar.sensor
    assert events.index(('enable', 'isaacsim.sensors.rtx.nodes')) < events.index(('construct_sensor',))
    assert lidar.points_enabled is True
    assert set(sensor.attached) == {'PhysicsLidarCapture', 'draw-point-cloud'}
    assert lidar.metadata['frequencies_hz'] == {
        'sensor_tick': 20.0, 'rotary_scan': 20.0, 'pattern_firing': 7200.0, 'render': 30.0}
    assert lidar.metadata['trace_labels_used'] is False
    assert lidar.metadata['debug_draw_do_transform'] is False

    lidar.set_points(False)
    lidar.set_points(False)  # Idempotent toggle: no second detach.
    assert lidar.points_enabled is False
    assert set(sensor.attached) == {'PhysicsLidarCapture'}
    lidar.set_points(True)
    assert lidar.points_enabled is True
    assert set(sensor.attached) == {'PhysicsLidarCapture', 'draw-point-cloud'}
    lidar.close()
    assert lidar.sensor is None and lidar.lidar is None
    assert lidar.points_enabled is False and sensor.attached == {}
    assert [(action, name) for action, *rest in events
            if action in ('attach', 'detach') for name in rest] == [
        ('attach', 'PhysicsLidarCapture'), ('attach', 'draw-point-cloud'),
        ('detach', 'draw-point-cloud'), ('attach', 'draw-point-cloud'),
        ('detach', 'draw-point-cloud'), ('detach', 'PhysicsLidarCapture'),
    ]
