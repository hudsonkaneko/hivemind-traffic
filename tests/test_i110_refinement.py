import json
from pathlib import Path
import numpy as np
import pytest

pytest.importorskip('scipy')
from scripts.refine_i110_geometry import union_support,fit_cross_sections


def test_shared_support_preserves_plane_and_does_not_bridge_gore():
    x,y=np.meshgrid(np.arange(-10,11,.25),np.arange(-1,21,.25))
    ground=np.column_stack([x.ravel(),y.ravel(),(4+.02*x+.03*y).ravel()])
    west=fit_cross_sections([[-4,0],[-4,20]],ground,np.array([[0,4],[20,4]]))
    east=fit_cross_sections([[4,0],[4,20]],ground,np.array([[0,4],[20,4]]))
    support,diag=union_support({'west':west,'east':east},
        {'west':[[20,-6,-2],[0,-6,-2]],'east':[[20,2,6],[0,2,6]]})
    v=np.array(support['points']);f=np.array(support['faces'])
    assert v[:,2]==pytest.approx(4+.02*v[:,0]+.03*v[:,1],abs=1e-9)
    assert (abs(v[:,0])>=2).all()
    assert np.all(np.ptp(v[f,0],axis=1)<=.5)
    tri=v[f];assert np.all(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])[:,2]>0)
    assert diag['overlap_grid_points']==0


def test_overlap_becomes_one_surface_with_unique_vertices():
    x,y=np.meshgrid(np.arange(-10,11,.25),np.arange(-1,21,.25))
    ground=np.column_stack([x.ravel(),y.ravel(),(4+.02*x).ravel()])
    a=fit_cross_sections([[-1,0],[-1,20]],ground,np.array([[0,6],[20,6]]))
    b=fit_cross_sections([[1,0],[1,20]],ground,np.array([[0,6],[20,6]]))
    support,diag=union_support({'a':a,'b':b},{'a':[[20,-4,2],[0,-4,2]],'b':[[20,-2,4],[0,-2,4]]})
    v=np.array(support['points'])
    assert len(np.unique(v[:,:2],axis=0))==len(v)
    assert diag['overlap_grid_points']>0
    assert diag['preblend_max_height_difference_m']<1e-9


def test_annotation_separates_measurement_from_interpretation():
    p=Path(__file__).resolve().parents[1]/'scenarios/i110/mlk-vernon-image-trace.json'
    trace=json.loads(p.read_text())
    assert trace['source_native_gsd_m']==.6
    assert 'NOT surveyed' in trace['accuracy']
    for points in trace['roads'].values():
        a=np.array(points)
        assert np.all(np.diff(a[:,0])<0)
        assert np.all(a[:,2]>a[:,1])


def test_viewer_uses_semantic_right_lane_not_shoulder():
    pytest.importorskip('pxr')
    from usd.highway_event_layer import right_lane_offset
    route=dict(station_m=[0,100],widths_m=[24,24],right_lane_offsets_m=[-7,-3])
    assert right_lane_offset(route,50)==pytest.approx(-5)


def test_paint_follows_actual_triangle_height_and_rejects_holes():
    pytest.importorskip('pxr')
    from scripts.build_i110_twin import support_sampler
    data=dict(points=[[0,0,4],[1,0,4.1],[1,1,4.3],[0,1,4.2]],faces=[[0,1,2],[0,2,3]])
    sample=support_sampler(data,spacing=1)
    assert sample(.25,.75)==pytest.approx(4.175)
    with pytest.raises(ValueError):sample(1.5,.5)
