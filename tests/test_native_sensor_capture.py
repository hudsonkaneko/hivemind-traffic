from types import SimpleNamespace
import sys
import numpy as np
import pytest
from traffic.native_sensor_capture import NativeCamera, lidar_measurement


class Array:
    def __init__(self,data): self.data=data
    def numpy(self): return self.data


class MockSensor:
    def __init__(self,camera,*,resolution,annotators):
        self.resolution=resolution
        self.rgb=Array(np.zeros((*resolution,4),np.uint8))
        self.depth=Array(np.ones((*resolution,1),np.float32))
        self.detached=False
    def get_data(self,name):
        return (self.rgb if name=='rgb' else self.depth),{}
    def detach_annotators(self,names): self.detached=True


@pytest.fixture
def camera(monkeypatch):
    def wrap(path, *, reset_xform_op_properties):
        assert reset_xform_op_properties is False
        return path
    monkeypatch.setitem(sys.modules,'isaacsim.sensors.experimental.rtx',SimpleNamespace(RtxCamera=wrap,CameraSensor=MockSensor))
    return NativeCamera('/World/Car/Camera',(64,32))


def test_camera_resolution_rgb_only_and_depth_separate(camera):
    assert camera.sensor.resolution==(32,64)
    frame,depth=camera.read(.1)
    assert frame.rgb.shape==(32,64,3) and depth.shape==(32,64)
    assert not hasattr(frame,'depth')
    camera.sensor.rgb.data[:]=255
    assert frame.rgb.max()==0
    with pytest.raises(ValueError): camera.read(.1)
    sensor=camera.sensor
    camera.close(); camera.close()
    assert sensor.detached


def test_multiple_cameras_keep_distinct_sensor_and_calibration_identity(camera):
    rear = NativeCamera('/World/Car/Rear', (64,32), episode_id='rig-test',
                        sensor_id='rear', frame_id='rear_optical', calibration_id='rig/rear')
    front_frame, _ = camera.read(.1)
    rear_frame, _ = rear.read(.1)
    assert rear_frame.header.sensor_id == 'rear'
    assert rear_frame.header.frame_id == 'rear_optical'
    assert rear_frame.header.calibration_id == 'rig/rear'
    assert rear_frame.header.episode_id == 'rig-test'
    assert front_frame.header.sensor_id != rear_frame.header.sensor_id
    rear.close()


@pytest.mark.parametrize('kind',['missing','shape','nan_depth','negative_depth'])
def test_bad_camera_payload_rejected(camera,kind):
    if kind=='missing': camera.sensor.rgb=None
    elif kind=='shape': camera.sensor.rgb=Array(np.zeros((64,32,3),np.uint8))
    elif kind=='nan_depth': camera.sensor.depth.data[0,0]=np.nan
    else: camera.sensor.depth.data[0,0]=-1
    with pytest.raises(RuntimeError): camera.read(.1)


def packet():
    return dict(scan_complete=1,episode_id='sensor-v01',vehicle_id='ego',xyz=np.tile([11.,2.,3.],(100,1)),
        flags=np.full(100,64,np.uint32),offset_ns=np.arange(100,dtype=np.int64)*1000,timestamp_ns=100000000)


def measure(p,**kwargs):
    return lidar_measurement(p,dict(position_m=[1,2,3],quaternion_xyzw=[0,0,0,1]),
        now_s=.12,epoch_s=0,noise_rng=np.random.default_rng(1),**kwargs)


def test_world_conversion_noise_delivery_and_no_groundtruth_identity():
    frame=measure(packet(),delivery_s=.11)
    assert frame.header.valid and frame.header.delivery_s==.11
    assert frame.header.acquisition_s==pytest.approx(.1)
    assert np.mean(frame.points_chassis[:,0])==pytest.approx(10,abs=.01)
    assert np.std(frame.points_chassis[:,0])>0
    assert not hasattr(frame,'object_ids')
    assert np.array_equal(frame.points_chassis,measure(packet(),delivery_s=.11).points_chassis)


@pytest.mark.parametrize('kind',['missing','negative_offsets','float_flags','float_stamp','incomplete','bad_hits','short','future','huge_span'])
def test_invalid_lidar_fail_closed(kind):
    p=packet()
    if kind=='missing': del p['xyz']
    elif kind=='negative_offsets': p['offset_ns'][0]=-1
    elif kind=='float_flags': p['flags']=p['flags'].astype(float)
    elif kind=='float_stamp': p['timestamp_ns']=1.5
    elif kind=='incomplete': p['scan_complete']=0
    elif kind=='bad_hits': p['xyz'][0,0]=np.nan
    elif kind=='short': p['flags'][:90]=0
    elif kind=='future': p['timestamp_ns']=200000000
    else: p['offset_ns'][-1]=100000000
    result=measure(p)
    assert not result.header.valid and len(result.points_chassis)==0


def test_nonfinite_nonreturn_points_permitted():
    p=packet(); p['flags'][0]=0; p['xyz'][0]=np.nan
    assert measure(p).header.valid


@pytest.mark.parametrize('field,value',[
    ('episode_id','old'),('vehicle_id','other'),('episode_id',None),
    ('scan_complete',True),('scan_complete',1.0),('scan_complete',np.bool_(True))])
def test_raw_identity_and_complete_type_fail_closed(field,value):
    p=packet(); p[field]=value
    assert not measure(p).header.valid


@pytest.mark.parametrize('field',['now_s','epoch_s','delivery_s'])
@pytest.mark.parametrize('value',['invalid',None,True,float('inf'),10**400])
def test_invalid_boundary_times_do_not_throw(field,value):
    kwargs=dict(now_s=.12,epoch_s=0,delivery_s=.12,noise_rng=np.random.default_rng(1))
    kwargs[field]=value
    result=lidar_measurement(packet(),dict(position_m=[1,2,3],quaternion_xyzw=[0,0,0,1]),**kwargs)
    # None delivery explicitly requests synchronous now, per API.
    assert result.header.valid == (field=='delivery_s' and value is None)


def test_explicit_alternate_vehicle_episode_supported():
    p=packet(); p.update(episode_id='new',vehicle_id='car-2')
    assert measure(p,episode_id='new',vehicle_id='car-2').header.valid


@pytest.mark.parametrize('value',['invalid',None,True,float('inf'),10**400])
def test_invalid_camera_time_is_value_error(camera,value):
    with pytest.raises(ValueError): camera.read(value)
