import numpy as np
from scripts.replay_lidar_geometry import ReplayReference, rotate, summarize


def reference():
    def car(name,x,y,heading=90):
        return dict(id=name,x=x,y=y,heading=heading,length=4,width=2)
    data=dict(frames=[dict(time=.2,vehicles=[car('ego',0,0),car('target',10,0)]),
                      dict(time=1.2,vehicles=[car('ego',0,0),car('target',10,0,0)]),
                      dict(time=2.2,vehicles=[car('ego',0,0)])])
    config=dict(ego_id='ego',sensor_mount_m=[0,0,.65],sensor_to_usd_offset_s=0,
                body_center_height_m=.65,body_height_m=1.1)
    return ReplayReference(data,config)


def test_rotation_and_mount_frame():
    np.testing.assert_allclose(rotate(np.array([[1,0,0]]),np.array([np.pi/2])),[[0,1,0]],atol=1e-12)


def test_source_time_origin_and_visibility():
    ref=reference()
    _,yaw,visible=ref.pose('target',np.array([0,.5,1.99,2]))
    np.testing.assert_allclose(yaw[:2],[0,np.pi/4])
    assert visible.tolist()==[True,True,True,False]


def test_rotated_box_and_lifecycle():
    ref=reference()
    # Start just outside ego's front bumper, so ego is not intersected.
    ref.config['sensor_mount_m']=[.01,0,.65]
    for time,expected in [(0,5.99),(1,8.99),(2,np.inf)]:
        scan=dict(timestamp_ns=round(time*1e9),offset_ns=np.array([0]),
                  azimuth_deg=np.array([0]),elevation_deg=np.array([0]))
        distance,labels=ref.expected(scan)
        np.testing.assert_allclose(distance,[expected],atol=1e-9)
        assert labels[0]==(1 if time<2 else -1)


def test_nearest_vehicle_occludes_farther():
    ref=reference()
    ref.config['sensor_mount_m']=[.01,0,.65]
    # Reuse trajectory with an extra nearer box.
    ref.ids.append('near')
    ref.tracks['near']=dict(ref.tracks['target'],x=np.array([6.,6.]))
    scan=dict(timestamp_ns=0,offset_ns=np.array([0]),azimuth_deg=np.array([0]),elevation_deg=np.array([0]))
    distance,labels=ref.expected(scan)
    assert abs(distance[0]-1.99)<1e-9
    assert labels[0]==2


def test_unknown_identity_is_not_accepted_as_expected_vehicle():
    ref=reference()
    ref.config['sensor_mount_m']=[.01,0,.65]
    scan=dict(timestamp_ns=0,offset_ns=np.array([0]),azimuth_deg=np.array([0]),elevation_deg=np.array([0]),
              range_m=np.array([5.99]),flags=np.array([64]),object_label=np.array([-1]))
    metrics,_=ref.analyze(scan)
    assert metrics['identity_checks']==1
    assert metrics['identity_matches']==0
    assert metrics['p95_m']<1e-9


def test_empty_replay_cannot_pass():
    config=dict(warmup_scans=3,min_evaluated_scans=470,min_vehicle_rays=1000,
                min_other_vehicles=4,min_identity_agreement=.99,scan_period_s=.1,pose_tolerance_m=.05)
    result=summarize([],config)
    assert not result['passed']
    assert result['max_pose_error_m'] is None
    assert result['max_cadence_error_s'] is None
