"""Native RTX capture with object-ID labels reserved for independent evaluation.

The existing driving LiDAR path stays unchanged. This writer records a second,
evaluation-only view of the same sensor output; no class IDs enter braking or
the camera-mask fusion inputs.
"""
import uuid

import numpy as np

from traffic.physical_lidar import PhysicalLidar


class SemanticLidarCapture:
    def __init__(self, vehicle_path, *, class_for_path, **kwargs):
        import omni.replicator.core as rep
        from isaacsim.sensors.experimental.rtx import (
            parse_generic_model_output_data, parse_object_ids, parse_stable_id_map_data,
        )
        self.base = PhysicalLidar(vehicle_path, **kwargs)
        self.latest = None
        self.errors = []
        self.packet_count = 0
        self.object_map = {}
        self.path = self.base.path
        self.metadata = dict(self.base.metadata,
            evaluation_oracle='RTX stable object ID -> USD semantic ancestor class',
            oracle_used_by_fusion=False)
        owner = self
        label_cache = {}

        class SemanticEvaluationWriter(rep.Writer):
            def __init__(self):
                self.data_structure = 'renderProduct'
                self.annotators = [rep.annotators.get('GenericModelOutput'), rep.annotators.get('StableIdMap')]

            def write(self, data):
                try:
                    for product in data.get('renderProducts', {}).values():
                        raw = product.get('GenericModelOutput')
                        if isinstance(raw, dict): raw = raw.get('data')
                        if raw is None: continue
                        gmo = parse_generic_model_output_data(raw)
                        if not gmo.numElements: continue
                        modes = [str(getattr(gmo, key)).split('.')[-1] for key in
                                 ('elementsCoordsType', 'frameOfReference', 'motionCompensationState')]
                        if modes != ['CARTESIAN', 'WORLD', 'NONCOMPENSATED']:
                            raise RuntimeError('Unexpected semantic LiDAR frame contract: '+repr(modes))
                        raw_map = product.get('StableIdMap')
                        if isinstance(raw_map, dict): raw_map = raw_map.get('data')
                        if raw_map is not None:
                            owner.object_map.update(parse_stable_id_map_data(raw_map))
                        if not owner.object_map: continue  # Annotator warmup, not a labeled packet.
                        ids = parse_object_ids(gmo.objId)
                        xyz = np.column_stack((gmo.x, gmo.y, gmo.z)).copy()
                        flags = np.asarray(gmo.flags).copy()
                        if len(ids) != len(xyz): raise RuntimeError('Object ID/point count mismatch')
                        labels = np.full(len(xyz), -1, dtype=np.int16)
                        for index in np.flatnonzero((flags.astype(np.uint32) & 64) != 0):
                            key = ids[index]
                            path = owner.object_map.get(key)
                            if path is None: continue
                            path = str(path)
                            if path not in label_cache: label_cache[path] = int(class_for_path(path))
                            labels[index] = label_cache[path]
                        stamp = int(gmo.timestampNs)
                        offsets = np.asarray(gmo.timeOffsetNs).copy()
                        if (xyz.shape != (int(gmo.numElements), 3) or flags.shape != (len(xyz),)
                                or offsets.shape != (len(xyz),) or stamp < 0):
                            raise RuntimeError('Malformed native LiDAR packet arrays/timestamp')
                        if owner.latest is not None and stamp < owner.latest['timestamp_ns']:
                            raise RuntimeError('Native LiDAR packet timestamp moved backwards')
                        owner.latest = dict(xyz_world=xyz, flags=flags,
                            timestamp_s=(stamp+offsets.astype(np.int64))*1e-9,
                            oracle_class=labels,
                            timestamp_ns=stamp, offset_ns=offsets,
                            frame_end_ns=int(gmo.frameEnd.timestampNs), scan_complete=int(gmo.scanComplete),
                            delivery_tick=int(kwargs['tick_source']()),
                            sensor_position_world_m=np.asarray(gmo.frameEnd.posM).copy(),
                            object_id_bytes=np.ascontiguousarray(gmo.objId).view(np.uint8).reshape(-1,16).copy())
                        owner.packet_count += 1
                except Exception as error:
                    if len(owner.errors) < 8: owner.errors.append(repr(error))

        self.writer_name = 'SemanticEvaluationCapture_'+uuid.uuid4().hex
        SemanticEvaluationWriter.__name__ = self.writer_name
        rep.WriterRegistry.register(SemanticEvaluationWriter)
        self.base.sensor.attach_writer(self.writer_name)

    def close(self):
        if self.base is not None:
            self.base.sensor.detach_writer(self.writer_name)
            self.base.close()
            self.base = None
