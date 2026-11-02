import numpy as np
import pytest
pytest.importorskip('scipy')
pytest.importorskip('pyproj')
from scripts.prepare_i110_geometry import fill_short_gaps,ribbon,sample_path


def test_occlusion_interpolation_is_bounded_and_never_extrapolates():
    s=np.array([0,10,20,30,40,50,60,70])
    v=np.array([1,2,np.nan,np.nan,np.nan,6,7,8])
    z,mask=fill_short_gaps(s,v,max_gap=50)
    assert z==pytest.approx(np.arange(1,9))
    assert mask.tolist()==[False,False,True,True,True,False,False,False]
    with pytest.raises(ValueError):fill_short_gaps(s,v,max_gap=35)
    with pytest.raises(ValueError):fill_short_gaps(s,np.r_[np.nan,v[1:]])


def test_road_fit_retains_grade_crossfall_and_upward_triangles():
    x,y=np.meshgrid(np.arange(-8,8.01,.3),np.arange(-1,31.01,.3))
    ground=np.column_stack([x.ravel(),y.ravel(),(4+.02*x+.05*y).ravel()])
    m=ribbon([[0,0],[0,30]],ground,np.array([[0,12],[30,12]]))
    v=np.array(m['points']);assert v[:,2]==pytest.approx(4+.02*v[:,0]+.05*v[:,1],abs=1e-8)
    triangles=v[np.array(m['faces'])]
    assert (np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])[:,2]>0).all()
    assert not np.any(m['interpolated'])


def test_coordinate_origin_and_cardinal_direction():
    from pyproj import Transformer
    t=Transformer.from_crs(4326,6340,always_xy=True)
    anchor=t.transform(-118.2812851,34.0109392)
    assert anchor==pytest.approx((381687.546870016,3764108.929151129),abs=.02)
    second=t.transform(-118.2812689,34.0035972)
    assert np.subtract(second,anchor)==pytest.approx((-8.687,-814.16),abs=.1)
    s,p,n=sample_path([[0,0],[0,-30]])
    assert n[0]==pytest.approx([1,0])


def test_route_pose_uses_crossfall_and_heading():
    pytest.importorskip('pxr')
    from usd.highway_event_layer import route_pose
    route=dict(station_m=[0,100],center_xyz=[[0,0,10],[0,-100,5]],widths_m=[18,18],
               points=[[x,y,z+.02*x] for y,z in [(0,10),(-100,5)] for x in (-9,-6,-3,0,3,6,9)])
    pos,angles=route_pose(route,50,3)
    assert pos==pytest.approx([3,-50,7.56]);assert angles[2]==pytest.approx(-90)
    with pytest.raises(ValueError):route_pose(route,50,20)
