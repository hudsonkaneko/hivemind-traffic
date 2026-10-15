"""RTX capture for one physics-owned car; labels never enter the braking input.

Runtime imports are lazy. WORLD/noncompensated returns are transformed into the
current chassis frame using odometry. This assumes static obstacles; it is not
motion compensation or tracking for moving actors. Raw per-ray times are retained.
"""
import math
import re
from numbers import Integral
import numpy as np
from traffic.speed_profiles import speed_limit


SENSOR_RATE_HZ = 20
MAX_SCAN_SPAN_S = 1.0 / SENSOR_RATE_HZ + 0.01
MAX_PACKET_ELEMENTS = 500_000


def _integer(value):
    return isinstance(value, Integral) and not isinstance(value, (bool, np.bool_))


def rotation_matrix(quaternion_xyzw):
    q = np.asarray(quaternion_xyzw, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Finite nonzero xyzw quaternion required')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def world_to_chassis(points, state):
    points = np.asarray(points, dtype=float)
    position = np.asarray(state['position_m'], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('Nx3 world points and finite XYZ chassis position required')
    return (points-position) @ rotation_matrix(state['quaternion_xyzw'])


def packet_to_scan(packet, state, *, episode_id, vehicle_id, tick, epoch_s=0.0, physics_hz=120):
    """Reject malformed packets; only successful, finite hits become brake inputs.

    Integer acquisition bounds round outwards (at most one tick conservative).
    Delivery time is captured by the writer, not re-stamped when consumed.
    A successful-hit flag with a NaN/Inf coordinate invalidates the WHOLE packet:
    filtering that hit alone could turn an unknown obstacle into a clear road.
    No-return rows may have nonfinite XYZ and are excluded using their flags.
    Packet age is not renewed or relaxed here; the brake gate retains its 0.2 s
    oldest-return freshness limit.
    """
    from traffic.lidar_braking import LidarScan
    if (not isinstance(packet, dict) or not _integer(tick) or tick < 0
            or not _integer(physics_hz) or physics_hz <= 0
            or isinstance(epoch_s, (bool, np.bool_))):
        return None
    try:
        if not math.isfinite(epoch_s):
            return None
        xyz = np.asarray(packet['xyz'], dtype=float)
        flags, offsets = (np.asarray(packet[name]) for name in ('flags', 'offset_ns'))
        stamp, complete, delivery = (packet[name] for name in
                                     ('timestamp_ns', 'scan_complete', 'delivery_tick'))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not 0 < len(xyz) <= MAX_PACKET_ELEMENTS
            or flags.shape != (len(xyz),) or offsets.shape != (len(xyz),)
            or flags.dtype.kind not in 'iu' or offsets.dtype.kind not in 'iu'
            or not _integer(stamp) or not 0 <= stamp <= np.iinfo(np.int64).max
            or not _integer(complete) or complete != 1
            or not _integer(delivery) or not 0 <= delivery <= tick
            or packet.get('episode_id') != episode_id or packet.get('vehicle_id') != vehicle_id
            or not isinstance(episode_id, str) or not episode_id.strip()
            or not isinstance(vehicle_id, str) or not vehicle_id.strip()):
        return None
    if (np.any(flags < 0) or np.any(flags > np.iinfo(np.uint32).max)
            or np.any(offsets < 0) or int(offsets.max()) > np.iinfo(np.int64).max-int(stamp)):
        return None
    times = (int(stamp)+offsets.astype(np.int64))*1e-9-float(epoch_s)
    if (not np.isfinite(times).all() or times.min() < 0
            or times.max()-times.min() > MAX_SCAN_SPAN_S):
        return None
    successful = (flags.astype(np.uint32)&64) != 0
    if successful.sum() < 20 or not np.isfinite(xyz[successful]).all():
        return None
    start = math.floor(float(times.min())*physics_hz+1e-6)
    end = math.ceil(float(times.max())*physics_hz-1e-6)
    if not 0 <= start <= end <= delivery <= tick:
        return None
    try:
        points = world_to_chassis(xyz[successful], state)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if not np.isfinite(points).all():
        return None
    return LidarScan(episode_id=episode_id, vehicle_id=vehicle_id,
        acquisition_start_tick=start, acquisition_end_tick=end,
        delivery_tick=int(delivery), coordinate_reference_tick=int(tick), points=points)


def _configure_rotary_profile(prim, speed_profile='low-speed'):
    """Author and read back both rates: installed tick_rate does NOT set scan rate."""
    speed_limit(speed_profile)
    for attr in prim.GetAttributes():
        if attr.GetName().startswith('omni:sensor:Core:emitterState:s001:'):
            value = attr.Get()
            if hasattr(value, '__len__') and len(value) == 128:
                if not attr.Set(value[:32]):
                    raise RuntimeError('Cannot configure RTX emitter array '+attr.GetName())
    values = {
        'omni:sensor:tickRate': float(SENSOR_RATE_HZ),
        'omni:sensor:Core:scanRateBaseHz': SENSOR_RATE_HZ,
        'omni:sensor:Core:numberOfEmitters': 32,
        'omni:sensor:Core:patternFiringRateHz': 72000.0 if speed_profile == '35mph' else 7200.0,
        'omni:sensor:Core:elementsCoordsType': 'CARTESIAN',
        'omni:sensor:Core:outputFrameOfReference': 'WORLD',
        'omni:sensor:Core:outputMotionCompensationState': 'NONCOMPENSATED',
    }
    actual = {}
    for name, value in values.items():
        attr = prim.GetAttribute(name)
        if not attr.Set(value) or attr.Get() != value:
            raise RuntimeError('Cannot configure/read back RTX '+name)
        actual[name] = attr.Get()
    return dict(sensor_tick=float(actual['omni:sensor:tickRate']),
                rotary_scan=float(actual['omni:sensor:Core:scanRateBaseHz']),
                pattern_firing=float(actual['omni:sensor:Core:patternFiringRateHz']))


class PhysicalLidar:
    """Bounded latest-packet sink and switchable RTX debug writer."""
    def __init__(self, vehicle_path, *, episode_id, vehicle_id, tick_source, points=False,
                 sensor_path=None, speed_profile='low-speed'):
        import omni.replicator.core as rep
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
        from isaacsim.core.experimental.utils.app import enable_extension
        from isaacsim.core.rendering_manager import RenderingManager
        self.latest, self.errors, self.packet_count, self.points_enabled = None, [], 0, False
        self.episode_id, self.vehicle_id = episode_id, vehicle_id
        self.mount_translation_m = [0.0, 0.0, 1.2]
        self.path = sensor_path or vehicle_path+'/Lidar'
        if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', self.path) or not self.path.startswith(vehicle_path+'/'):
            raise ValueError('LiDAR must be a descendant of the actual chassis rigid body')
        owner = self

        class PhysicsLidarCapture(rep.Writer):
            def __init__(self):
                self.data_structure = 'renderProduct'
                self.annotators = [rep.annotators.get('GenericModelOutput')]

            def write(self, data):
                try:
                    for product in data.get('renderProducts', {}).values():
                        raw = product.get('GenericModelOutput')
                        if isinstance(raw, dict):
                            raw = raw.get('data')
                        if raw is None:
                            continue
                        gmo = parse_generic_model_output_data(raw)
                        if not gmo.numElements:
                            continue
                        modes = [str(getattr(gmo, k)).split('.')[-1]
                                 for k in ('elementsCoordsType','frameOfReference','motionCompensationState')]
                        if modes != ['CARTESIAN','WORLD','NONCOMPENSATED']:
                            raise RuntimeError('Unexpected RTX frame contract: '+repr(modes))
                        stamp = int(gmo.timestampNs)
                        if owner.latest is not None and stamp < owner.latest['timestamp_ns']:
                            raise RuntimeError('RTX acquisition timestamp went backwards')
                        owner.latest = dict(episode_id=episode_id, vehicle_id=vehicle_id,
                            timestamp_ns=stamp, scan_complete=int(gmo.scanComplete),
                            delivery_tick=int(tick_source()), xyz=np.column_stack((gmo.x,gmo.y,gmo.z)).copy(),
                            flags=np.asarray(gmo.flags).copy(), offset_ns=np.asarray(gmo.timeOffsetNs).copy(),
                            frame_end_ns=int(gmo.frameEnd.timestampNs),
                            sensor_position_world_m=np.asarray(gmo.frameEnd.posM).copy())
                        owner.packet_count += 1
                except Exception as error:
                    if len(owner.errors) < 8:
                        owner.errors.append(repr(error))

        rep.WriterRegistry.register(PhysicsLidarCapture)
        self.lidar = Lidar.create(self.path, config='Example_Rotary', translations=np.array(self.mount_translation_m),
                                 aux_output_level='FULL', accumulate_outputs=True, tick_rate=SENSOR_RATE_HZ)
        prim = self.lidar.prims[0]
        frequencies = _configure_rotary_profile(prim, speed_profile)
        render_dt = float(RenderingManager.get_dt())
        if not math.isfinite(render_dt) or render_dt <= 0:
            raise RuntimeError('Rendering cadence must be configured before the physical LiDAR')
        frequencies['render'] = 1.0 / render_dt
        self.metadata = dict(path=self.path, mount_translation_m=self.mount_translation_m, speed_profile=speed_profile,
            frame='CARTESIAN WORLD NONCOMPENSATED', chassis_quaternion='xyzw',
            labels_used=False, trace_labels_used=False, obstacle_ground_truth_used=False,
            odometry_source='simulator_chassis_pose', frequencies_hz=frequencies,
            frequency_note='Read-back configured rates; delivered packet cadence is measured separately',
            successful_hit_nonfinite_policy='reject_entire_packet',
            debug_draw_do_transform=False,
            attributes={a.GetName():str(a.Get()) for a in prim.GetAttributes()
                                          if a.GetName().startswith('omni:sensor')})
        # SensorBase snapshots writer aliases at construction: loading this
        # extension afterward leaves 'draw-point-cloud' unresolved on this sensor.
        enable_extension('isaacsim.sensors.rtx.nodes')
        self.sensor = LidarSensor(self.lidar, annotators=[])
        self.sensor.attach_writer('PhysicsLidarCapture')
        self.set_points(points)

    def set_points(self, enabled):
        enabled = bool(enabled)
        if enabled == self.points_enabled:
            return
        if enabled:
            # XYZ is already WORLD: applying the sensor-to-world matrix again
            # displaces/rotates the visual cloud even though braking is correct.
            self.sensor.attach_writer('draw-point-cloud', size=.035,
                                      color=[0.1,1.0,.55,1.0], doTransform=False)
        else:
            self.sensor.detach_writer('draw-point-cloud')
        self.points_enabled = enabled

    def close(self):
        self.set_points(False)
        self.sensor.detach_writer('PhysicsLidarCapture')
        self.sensor = self.lidar = None
