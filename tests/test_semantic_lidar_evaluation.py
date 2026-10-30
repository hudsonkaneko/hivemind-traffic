"""Offline fusion evaluation with synthetic geometry, never Isaac or training."""
import json

import numpy as np
import pytest
from PIL import Image

from traffic.semantic_lidar_evaluation import (aggregate_metrics, evaluate_capture, measure_fusion,
    measure_actor_support, aggregate_actor_support)
from traffic.semantic_lidar_fusion import fuse_semantic_lidar


def fixture_data():
    mask=np.ones((48,64),dtype=np.int32)
    mask[:,16:32]=3;mask[:,32:48]=5
    u,v=np.meshgrid(np.arange(18.5,46,2),np.arange(4.5,44,2))
    u=u.ravel();v=v.ravel();depth=np.full(len(u),10.)
    xyz=np.column_stack(((u-32)*depth/32,-(v-24)*depth/32,-depth))
    calibration=dict(model='pinhole',width=64,height=48,fx=32,fy=32,cx=32,cy=24)
    flags=np.full(len(xyz),64,dtype=np.uint32);times=np.full(len(xyz),.97)
    oracle=mask[np.floor(v).astype(int),np.floor(u).astype(int)]
    return xyz,mask,calibration,flags,times,oracle


def fuse(data, matrix=None, image_time=1.):
    xyz,mask,calibration,flags,times,_=data
    return fuse_semantic_lidar(xyz,mask,np.eye(4) if matrix is None else matrix,calibration,
        flags=flags,point_times_s=times,image_time_s=image_time,boundary_margin_px=1)


def test_independent_oracle_cannot_modify_fused_predictions():
    data=fixture_data();result=fuse(data);before=result['labels'].copy()
    good=measure_fusion(result,data[-1]);wrong=measure_fusion(result,np.ones(len(data[0]),dtype=np.int32))
    assert good['accuracy']==1 and wrong['accuracy']==0
    np.testing.assert_array_equal(result['labels'],before)
    assert good['roi_coverage']<1  # Semantic edge rejection remains in denominator.
    assert aggregate_metrics([good],dict(minimum_vehicle_frames=1))['passed']
    assert not aggregate_metrics([wrong],dict(minimum_vehicle_frames=1))['passed']


def test_wrong_camera_time_and_extrinsic_are_detected():
    data=fixture_data();ordinary=measure_fusion(fuse(data),data[-1])
    stale=fuse(data,image_time=1.25)
    assert stale['accepted'].sum()==0
    assert measure_fusion(stale,data[-1])['roi_points']==ordinary['roi_points']
    wrong=np.eye(4);wrong[3,0]=3.
    metrics=measure_fusion(fuse(data,matrix=wrong),data[-1])
    assert metrics['accuracy']<.9
    assert not aggregate_metrics([metrics],dict(minimum_vehicle_frames=1))['passed']


def test_unmapped_oracle_fails_without_silently_counting_as_background():
    data=fixture_data();metrics=measure_fusion(fuse(data),np.full(len(data[0]),-1,dtype=np.int32))
    assert metrics['accuracy'] is None and metrics['oracle_roi_coverage']==0
    result=aggregate_metrics([metrics],dict(minimum_vehicle_frames=1))
    assert not result['passed'] and not result['checks']['oracle_coverage']


def test_depth_is_evaluation_only_and_exposes_unseen_camera_occluders():
    data=fixture_data();fused=fuse(data)
    result=measure_fusion(fused,data[-1],depth_evaluation=np.full((48,64),5.))
    assert result['accuracy']==1
    assert result['depth_evaluation_only']['behind_surface_fraction']==1
    assert fused['accepted'].any()  # No oracle depth is fed back into fusion.


def write_capture(tmp_path):
    xyz,mask,calibration,flags,times,oracle=fixture_data()
    Image.fromarray(np.full((48,64,3),120,dtype=np.uint8)).save(tmp_path/'rgb.png')
    np.save(tmp_path/'mask.npy',mask);np.save(tmp_path/'depth.npy',np.full((48,64),10.))
    np.savez_compressed(tmp_path/'lidar.npz',xyz_world=xyz,flags=flags,point_times_s=times,oracle_class=oracle)
    (tmp_path/'labels.json').write_text(json.dumps(dict(calibration=calibration,camera_to_world=np.eye(4).tolist(),image_time_s=1.)))
    frame=dict(rgb_path='rgb.png',semantic_path='mask.npy',lidar_path='lidar.npz',labels_path='labels.json',depth_path='depth.npy')
    (tmp_path/'capture-result.json').write_text(json.dumps(dict(passed=True,scenario='synthetic-test',frames=[frame])))


def test_capture_evaluation_retains_every_return_and_never_overwrites(tmp_path):
    write_capture(tmp_path)
    result=evaluate_capture(tmp_path,dict(boundary_margin_px=1),dict(minimum_vehicle_frames=1))
    assert result['passed'] and result['negative_controls']['stale_camera_plus_250ms']['passed']
    assert not result['inference_uses_oracle'] and result['training'] is False
    saved=np.load(tmp_path/'fusion/frame-0000-fused.npz')
    assert len(saved['labels'])==len(fixture_data()[0]) and 'oracle_class_evaluation_only' in saved
    assert (tmp_path/'fusion/frame-0000-fusion.png').is_file()
    with pytest.raises(FileExistsError):
        evaluate_capture(tmp_path,{},dict(minimum_vehicle_frames=1))


def test_capture_path_traversal_and_partial_failure_are_retained(tmp_path):
    write_capture(tmp_path)
    report=json.loads((tmp_path/'capture-result.json').read_text());report['frames'][0]['rgb_path']='../elsewhere.png'
    (tmp_path/'capture-result.json').write_text(json.dumps(report))
    with pytest.raises(ValueError,match='escaping'):
        evaluate_capture(tmp_path,{},dict(minimum_vehicle_frames=1))
    result=json.loads((tmp_path/'fusion/evaluation.json').read_text())
    assert not result['passed'] and 'error' in result


def test_per_actor_id_mapping_is_independent_and_both_moving_actors_required():
    data=fixture_data();fused=fuse(data);before=fused['labels'].copy()
    count=len(before);ids=np.tile(np.frombuffer((2**100+7).to_bytes(16,'little'),dtype=np.uint8),(count,1))
    selected=np.flatnonzero(fused['accepted'] & (before==3));half=len(selected)//2
    ids[selected[half:]]=np.frombuffer((2**90+9).to_bytes(16,'little'),dtype=np.uint8)
    mapping={str(2**100+7):dict(path='/World/lead/Body'),str(2**90+9):dict(path='/World/adjacent/Body')}
    # Obstacle returns share the first ID in this fixture; make their identity explicitly unrelated.
    ids[before!=3]=np.frombuffer((99).to_bytes(16,'little'),dtype=np.uint8)
    rows=[measure_actor_support(fused,ids,mapping,{'/World/lead':[i,0,0],'/World/adjacent':[i,3,0]}) for i in range(4)]
    assert aggregate_actor_support(rows,moving=True)['passed']
    np.testing.assert_array_equal(fused['labels'],before)
    assert not aggregate_actor_support(rows[:3],moving=True)['passed']
    stationary=[rows[0]]*4
    assert not aggregate_actor_support(stationary,moving=True)['passed']
    assert aggregate_actor_support(stationary,moving=False)['passed']
    mapping.pop(str(2**90+9))
    missing=[measure_actor_support(fused,ids,mapping,{'/World/lead':[i,0,0],'/World/adjacent':[i,3,0]}) for i in range(4)]
    assert not aggregate_actor_support(missing,moving=True)['passed']
