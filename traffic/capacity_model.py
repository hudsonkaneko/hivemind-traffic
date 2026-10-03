"""Arithmetic workload estimates, not a GPU performance predictor."""
import math


def workload(*, vehicles, lidar_vehicles, av_vehicles, control_hz=10., emitters=32,
             firing_hz=7200., message_hz=10., max_neighbors=None,
             bytes_per_point=20, bytes_per_message=300):
    for name,value in [('vehicles',vehicles),('lidar_vehicles',lidar_vehicles),('av_vehicles',av_vehicles),('emitters',emitters),('bytes_per_point',bytes_per_point),('bytes_per_message',bytes_per_message)]:
        if type(value) is not int or value < 0:raise ValueError(name+' must be a nonnegative integer')
    if vehicles < 1 or max(lidar_vehicles,av_vehicles)>vehicles or min(emitters,bytes_per_point,bytes_per_message)<1:
        raise ValueError('Invalid population or payload dimensions')
    for rate in (control_hz,firing_hz,message_hz):
        if type(rate) not in (float,int) or not math.isfinite(rate) or rate<=0:
            raise ValueError('Rates must be positive finite numbers')
    if max_neighbors is not None and (type(max_neighbors) is not int or max_neighbors<0):
        raise ValueError('Invalid neighbor limit')
    neighbors=max(av_vehicles-1,0)
    if max_neighbors is not None:neighbors=min(neighbors,max_neighbors)
    rays=lidar_vehicles*emitters*firing_hz
    deliveries=av_vehicles*neighbors*message_hz
    return dict(control_commands_per_second=av_vehicles*control_hz,
        nominal_lidar_sample_opportunities_per_second=rays,
        full_return_payload_MB_per_second=rays*bytes_per_point/1e6,
        full_return_payload_GB_per_hour=rays*bytes_per_point*3600/1e9,
        message_publications_per_second=av_vehicles*message_hz,
        logical_receiver_deliveries_per_second=deliveries,
        logical_receiver_payload_MB_per_second=deliveries*bytes_per_message/1e6,
        assumptions=dict(one_sample_per_emitter_per_firing=True,full_returns=True,
            point_bytes=bytes_per_point,message_bytes=bytes_per_message,
            caveat='Not measured ray hits, compressed disk use, physical network bandwidth or GPU load'))
