from dataclasses import replace
import numpy as np
import pytest
from traffic.sensor_pipeline import (SensorHeader, CameraFrame, LidarFrame,
    LocalizationFix, EgoMeasurement, EgoEstimate, EgoEstimator, SeededSensorModel,
    SensorPipeline, detect_lane_rgb)


def packets(t=.1, obstacle_x=20.):
    rgb = np.zeros((100,200,3),np.uint8)
    rgb[55:90,45:49] = 220
    rgb[55:90,151:155] = 220
    cloud = np.column_stack((np.full(30,obstacle_x),np.linspace(-.3,.3,30),np.zeros(30)))
    camera = CameraFrame(SensorHeader('camera','camera_optical',t,t,'front-camera-v1'),rgb)
    lidar = LidarFrame(SensorHeader('lidar','chassis',t,t,'front-lidar-v1'),cloud)
    ego = EgoEstimate(t,0.,0.,0.,3.,.15)
    return camera,lidar,ego


def test_seeded_measurement_reproducibility_and_nonzero_noise():
    a,b = SeededSensorModel(5),SeededSensorModel(5)
    assert a.localization(0,0,0) == b.localization(0,0,0)
    ma,mb = a.telemetry(3,0,.1),b.telemetry(3,0,.1)
    assert ma == mb
    assert ma.wheel_speed_m_s != 3 and ma.yaw_rate_rad_s != 0


def test_dead_reckoning_consumes_measurements_and_grows_uncertainty():
    estimator = EgoEstimator(localization_calibration_id='loc-v1',telemetry_calibration_id='odo-v1')
    estimator.initialize(LocalizationFix(SensorHeader('loc','estimated_map',0,0,'loc-v1'),4,5,0))
    for i in range(1,11):
        result = estimator.update(EgoMeasurement(SensorHeader('odo','chassis',i*.1,i*.1,'odo-v1'),2,0))
    assert result.x_m == pytest.approx(6)
    assert result.y_m == 5
    assert result.position_uncertainty_m > .15
    with pytest.raises(ValueError):
        estimator.update(EgoMeasurement(SensorHeader('odo','chassis',1,1,'odo-v1'),2,0))


def test_lane_uses_pixels_and_rejects_bland_or_glare():
    camera,_,_ = packets()
    lane = detect_lane_rgb(camera.rgb)
    assert lane.valid and abs(lane.center_offset_pixels)<2
    assert not detect_lane_rgb(np.zeros((100,200,3),np.uint8)).valid
    assert not detect_lane_rgb(np.full((100,200,3),255,np.uint8)).valid


def test_forward_stop_and_geometric_track():
    camera,lidar,ego = packets(obstacle_x=5)
    result = SensorPipeline().process(camera,lidar,ego,.1)
    assert result.valid and result.stop_required
    assert result.nearest_forward_gap_m == pytest.approx(2.4)
    assert len(result.tracks)==1
    assert result.tracks[0].id == 1


def test_clear_forward_far_obstacle_and_no_steering_claim():
    result = SensorPipeline().process(*packets(),.1)
    assert result.valid and not result.stop_required
    assert result.nearest_forward_gap_m == pytest.approx(17.4)


@pytest.mark.parametrize('kind',['stale','future','wrong_frame','wrong_calibration','nan','empty','malformed','invalid'])
def test_lidar_invalid_fails_closed(kind):
    camera,lidar,ego = packets()
    if kind=='stale':
        lidar=replace(lidar,header=replace(lidar.header,acquisition_s=-1))
    elif kind=='future':
        lidar=replace(lidar,header=replace(lidar.header,delivery_s=2))
    elif kind=='wrong_frame':
        lidar=replace(lidar,header=replace(lidar.header,frame_id='world'))
    elif kind=='wrong_calibration':
        lidar=replace(lidar,header=replace(lidar.header,calibration_id='unknown'))
    elif kind=='invalid':
        lidar=replace(lidar,header=replace(lidar.header,valid=False))
    elif kind=='nan':
        lidar.points_chassis[0,0]=np.nan
    elif kind=='empty':
        lidar=replace(lidar,points_chassis=np.empty((0,3)))
    else:
        lidar=replace(lidar,points_chassis='bad')
    result=SensorPipeline().process(camera,lidar,ego,.1)
    assert not result.valid and result.stop_required and not result.tracks


def test_duplicate_scan_does_not_refresh_or_retrack():
    pipeline=SensorPipeline()
    camera,lidar,ego=packets()
    assert pipeline.process(camera,lidar,ego,.1).valid
    result=pipeline.process(camera,lidar,ego,.11)
    assert not result.valid and result.stop_required


def test_estimated_pose_transforms_tracks_no_truth_ids():
    camera,lidar,ego=packets()
    ego=replace(ego,x_m=10,y_m=30,yaw_rad=np.pi/2)
    result=SensorPipeline().process(camera,lidar,ego,.1)
    assert result.tracks[0].x == pytest.approx(10)
    assert result.tracks[0].y == pytest.approx(50)


def test_lidar_ego_timestamp_alignment_required():
    camera,lidar,ego=packets()
    result=SensorPipeline().process(camera,lidar,replace(ego,timestamp_s=.16),.16)
    assert result.stop_required and not result.valid


def test_missing_camera_evidence_stops_even_with_lidar():
    camera,lidar,ego=packets()
    camera.rgb[:]=0
    result=SensorPipeline().process(camera,lidar,ego,.1)
    assert result.stop_required and 'lane_unknown' in result.reasons


def test_stale_ego_never_updates_tracker():
    camera,lidar,ego=packets(.4)
    result=SensorPipeline().process(camera,lidar,replace(ego,timestamp_s=.1),.4)
    assert result.stop_required and not result.tracks


def test_header_bad_time_fails_closed():
    assert not SensorHeader('c','camera_optical','oops',0,'v1').fresh(.1)


def test_configured_vehicle_envelope_controls_gap_and_corridor():
    camera,lidar,ego=packets(obstacle_x=5)
    lidar.points_chassis[:,1]=1.3
    narrow=SensorPipeline(ego_half_length_m=2.591,corridor_half_width_m=1.2).process(camera,lidar,ego,.1)
    wide=SensorPipeline(ego_half_length_m=2.591,corridor_half_width_m=1.4).process(camera,lidar,ego,.1)
    assert narrow.nearest_forward_gap_m is None
    assert wide.nearest_forward_gap_m == pytest.approx(2.409)
    assert wide.stop_required
    with pytest.raises(ValueError):
        SensorPipeline(ego_half_length_m=-1)


@pytest.mark.parametrize('sensor',['camera','lidar'])
def test_cross_episode_packets_fail_closed(sensor):
    camera,lidar,ego=packets()
    if sensor=='camera':
        camera=replace(camera,header=replace(camera.header,episode_id='old'))
    else:
        lidar=replace(lidar,header=replace(lidar.header,episode_id='old'))
    result=SensorPipeline().process(camera,lidar,ego,.1)
    assert not result.valid and result.stop_required


def test_held_camera_timestamp_fails_with_fresh_lidar():
    camera,lidar,ego=packets()
    pipeline=SensorPipeline()
    assert pipeline.process(camera,lidar,ego,.1).valid
    _,lidar,ego=packets(.2)
    result=pipeline.process(camera,lidar,ego,.2)
    assert not result.valid and result.stop_required


def test_ego_calibration_and_uncertainty_checks():
    estimator=EgoEstimator()
    fix=SeededSensorModel(1).localization(0,0,0)
    with pytest.raises(ValueError):
        estimator.initialize(replace(fix,header=replace(fix.header,calibration_id='wrong')))
    estimator.initialize(fix)
    measure=SeededSensorModel(1).telemetry(0,0,.1)
    with pytest.raises(ValueError):
        estimator.update(replace(measure,header=replace(measure.header,calibration_id='wrong')))
    camera,lidar,ego=packets()
    assert SensorPipeline().process(camera,lidar,replace(ego,position_uncertainty_m=np.nan),.1).stop_required
