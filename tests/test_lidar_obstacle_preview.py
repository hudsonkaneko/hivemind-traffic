import numpy as np
import pytest
from traffic.lidar_obstacle_preview import (compute_preview,estimate_ground,evaluate_preview,render_overlay)


def frame():
    x,y=np.meshgrid(np.linspace(-4,4,30),np.linspace(-4,4,30))
    floor=np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])
    obj=np.array([[.1,.1,1.],[.3,.3,1.],[.5,.5,1.],[.6,.6,.1]])
    xyz=np.vstack([floor,obj]); matrix=np.eye(4);matrix[3,2]=5
    return dict(xyz_world=xyz,flags=np.full(len(xyz),64,dtype=np.uint8),point_times_s=np.full(len(xyz),1.),image_time_s=1.,camera_to_world=matrix,
        calibration=dict(model='pinhole',width=100,height=100,fx=40.,fy=40.,cx=50.,cy=50.,near_m=.1,far_m=100.))


def test_plane_is_estimated_at_arbitrary_height_and_slope():
    f=frame();xyz=f['xyz_world'].copy();xyz[:,2]+=7+.1*xyz[:,0]
    plane=estimate_ground(xyz)
    assert plane['valid'] and plane['normal'][2]>.99
    assert np.allclose(np.asarray(plane['normal'])@[0,0,7]+plane['offset_m'],0,atol=1e-8)
    assert plane==estimate_ground(xyz)


def test_sparse_occupancy_does_not_depend_on_rgb_label_or_oracle():
    f=frame();a=compute_preview(np.ones((100,100),dtype=np.uint8),f)
    f['oracle_class']=np.full(len(f['xyz_world']),999)
    b=compute_preview(np.full((100,100),3,dtype=np.uint8),f)
    assert np.array_equal(a['occupied'],b['occupied'])
    assert a['occupied'][-4:-1].all() and not a['occupied'][-1]
    assert not a['occupied'][:-4].any()
    assert (a['fused_occupied_labels'][a['occupied']]==5).all()
    assert (b['fused_occupied_labels'][b['occupied']]==3).all()


@pytest.mark.parametrize('offset',[.051,-.001])
def test_stale_and_future_points_cannot_be_occupied(offset):
    f=frame();f['point_times_s']-=offset
    r=compute_preview(np.ones((100,100),dtype=np.uint8),f)
    assert not r['plane']['valid'] and not r['occupied'].any()


def test_failed_plane_abstains():
    f=frame();f['xyz_world']=f['xyz_world'][:10];f['flags']=f['flags'][:10];f['point_times_s']=f['point_times_s'][:10]
    r=compute_preview(np.ones((100,100),dtype=np.uint8),f)
    assert not r['plane']['valid']; assert not r['occupied'].any()
    assert np.all(r['fused_occupied_labels']==-1)


def test_exact_pixels_overlay_and_evaluation():
    f=frame();r=compute_preview(np.ones((100,100),dtype=np.uint8),f)
    image=np.zeros((100,100,3),dtype=np.uint8);overlay=render_overlay(image,r)
    assert not image.any()
    assert np.count_nonzero(overlay.any(axis=2))==r['metrics']['occupied_pixels']
    gt=np.ones(len(f['xyz_world']),dtype=np.int16);gt[-4:]=5
    m=evaluate_preview(r,gt)
    assert m['precision']==1 and m['per_class']['generic_obstacle']['occupied_points']==3
    assert m['per_class']['pedestrian']['visible_timely_recall'] is None


def test_invalid_config_and_calibration_rejected():
    f=frame();mask=np.ones((100,100),dtype=np.uint8)
    with pytest.raises(ValueError):compute_preview(mask,f,dict(plane_trials=100000))
    with pytest.raises(ValueError):compute_preview(mask,f,dict(minimum_height_m=-1))
    f['camera_to_world'][0,0]=2
    with pytest.raises(ValueError):compute_preview(mask,f)


def test_fov_and_sparse_occlusion_exclude_hidden_points():
    f=frame();f['xyz_world'][-4:]=[[0,0,1],[0,0,.5],[100,100,1],[.5,.5,1]]
    r=compute_preview(np.ones((100,100),dtype=np.uint8),f)
    assert r['occupied'][-4]
    assert not r['occupied'][-3] and not r['occupied'][-2]
    assert r['metrics']['geometry_rejection_counts']['sparse_occlusion']>0
