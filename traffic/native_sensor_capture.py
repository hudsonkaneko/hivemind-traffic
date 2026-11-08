"""Native camera capture and explicit simulator-to-sensor measurement boundary.

No detector receives depth, labels, exact world pose or simulator object IDs.
WORLD RTX points are converted into a chassis measurement here, using simulation
pose solely to express simulated ray returns in the receiving sensor's frame.
This legacy scan is noncompensated; moving-object/high-speed accuracy is unproven.
"""
import math
from numbers import Integral, Real
import numpy as np
from traffic.physical_lidar import world_to_chassis
from traffic.sensor_pipeline import CameraFrame, LidarFrame, SensorHeader


def _finite(value):
    try:
        return isinstance(value,Real) and not isinstance(value,(bool,np.bool_)) and math.isfinite(value)
    except (TypeError,ValueError,OverflowError):
        return False


class NativeCamera:
    def __init__(self, path, resolution=(1280,720), episode_id='sensor-v01', *,
                 sensor_id='front_camera', frame_id='camera_optical', calibration_id='front-camera-v1'):
        if len(resolution) != 2 or any(not isinstance(v,Integral) or isinstance(v,bool) or v < 32 for v in resolution):
            raise ValueError('Resolution must be (width,height), each at least 32')
        from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
        self.camera = RtxCamera(path, reset_xform_op_properties=False)
        self.sensor = CameraSensor(self.camera, resolution=(resolution[1],resolution[0]),
                                   annotators=['rgb','distance_to_image_plane'])
        self.path = path
        self.resolution = tuple(resolution)
        self.episode_id = episode_id
        self.sensor_id, self.frame_id, self.calibration_id = sensor_id, frame_id, calibration_id
        self._last_time_s = -1.

    def read(self, time_s):
        """Caller must render synchronously at time_s, without advancing physics.

        Native metadata provides no guaranteed acquisition clock. Monotonic caller
        stamps detect repeated calls, not a renderer returning held image buffers.
        Depth is returned separately for evaluation only; CameraFrame contains RGB.
        """
        if not _finite(time_s) or time_s < 0 or time_s <= self._last_time_s:
            raise ValueError('Camera render time must be finite and strictly increasing')
        rgb, _ = self.sensor.get_data('rgb')
        depth, _ = self.sensor.get_data('distance_to_image_plane')
        if rgb is None or depth is None:
            raise RuntimeError(f'Camera data unavailable: rgb={rgb is not None}, depth={depth is not None}')
        rgb = rgb.numpy().copy()
        depth = depth.numpy().copy()
        if (rgb.ndim != 3 or rgb.shape[2] not in (3,4) or rgb.dtype != np.uint8
                or rgb.shape[:2] != (self.resolution[1],self.resolution[0])):
            raise RuntimeError('Camera did not deliver an RGB image')
        if depth.ndim == 3 and depth.shape[2] == 1:
            depth = depth[:,:,0]
        if depth.shape != rgb.shape[:2] or not np.issubdtype(depth.dtype,np.number):
            raise RuntimeError('Camera depth shape/type mismatch')
        # Infinite depth represents no return; retain it as an evaluation label.
        if np.isnan(depth).any() or np.any(depth < 0):
            raise RuntimeError('Camera depth contains invalid negative/NaN values')
        # Synchronous render barrier: no physics advances between render and read.
        frame = CameraFrame(SensorHeader(self.sensor_id,self.frame_id,time_s,time_s,
                                        self.calibration_id,episode_id=self.episode_id),rgb[:,:,:3])
        self._last_time_s = time_s
        return frame, depth

    def close(self):
        if self.sensor is not None:
            self.sensor.detach_annotators(['rgb','distance_to_image_plane'])
        self.sensor = self.camera = None


def lidar_measurement(packet, acquisition_state, *, now_s, epoch_s, noise_rng,
                      episode_id='sensor-v01', vehicle_id='ego', delivery_s=None):
    """Render truth is confined here; downstream receives only local XYZ and time.

Require finite successful returns and a complete scan. .01m independent XYZ
noise and 1% dropout are prototype settings, not a calibrated LiDAR model.
"""
    invalid = LidarFrame(SensorHeader('front_lidar','chassis',now_s,now_s,
                                     'front-lidar-v1',False,episode_id),np.empty((0,3)))
    if (not isinstance(packet,dict)
            or not isinstance(packet.get('scan_complete'),Integral)
            or isinstance(packet.get('scan_complete'),(bool,np.bool_))
            or packet.get('scan_complete') != 1
            or not isinstance(episode_id,str) or not episode_id.strip()
            or not isinstance(vehicle_id,str) or not vehicle_id.strip()
            or packet.get('episode_id') != episode_id or packet.get('vehicle_id') != vehicle_id
            or not all(_finite(v) for v in (now_s,epoch_s))):
        return invalid
    try:
        xyz = np.asarray(packet['xyz'],dtype=float)
        flags = np.asarray(packet['flags'])
        offsets = np.asarray(packet['offset_ns'])
        stamp = packet['timestamp_ns']
    except (KeyError,TypeError,ValueError,OverflowError):
        return invalid
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not 20 <= len(xyz) <= 500000
            or flags.shape != (len(xyz),) or offsets.shape != (len(xyz),)
            or flags.dtype.kind not in 'iu' or offsets.dtype.kind not in 'iu'
            or np.any(flags<0) or np.any(flags>np.iinfo(np.uint32).max)
            or np.any(offsets<0) or not isinstance(stamp,Integral) or isinstance(stamp,bool)
            or stamp < 0 or stamp > np.iinfo(np.int64).max
            or int(offsets.max(initial=0)) > np.iinfo(np.int64).max-int(stamp)):
        return invalid
    hits = (flags.astype(np.uint32)&64) != 0
    if hits.sum() < 20 or not np.isfinite(xyz[hits]).all():
        return invalid
    times = (int(stamp)+offsets.astype(np.int64))*1e-9-epoch_s
    acquisition,end = float(times.min()),float(times.max())
    delivery = now_s if delivery_s is None else delivery_s
    if (not _finite(delivery) or not 0 <= acquisition <= end <= delivery+1e-5
            or delivery > now_s or now_s-acquisition > .2 or end-acquisition > .06):
        return invalid
    try:
        points = world_to_chassis(xyz[hits], acquisition_state)
    except (KeyError,TypeError,ValueError,OverflowError):
        return invalid
    points = points[noise_rng.random(len(points)) >= .01]
    points = points + noise_rng.normal(0,.01,points.shape)
    if len(points)<20 or not np.isfinite(points).all():
        return invalid
    return LidarFrame(SensorHeader('front_lidar','chassis',acquisition,delivery,
                                  'front-lidar-v1',episode_id=episode_id),points)
