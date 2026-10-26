import numpy as np
import pytest
from usd.body_camera_rig import exterior_anchor


def test_nearest_exterior_surface_not_back_panel():
    panel=np.array([[[2,-1,-1],[2,1,-1],[2,1,1]],[[2,-1,-1],[2,1,1],[2,-1,1]]],dtype=float)
    inner=panel.copy();inner[:,:,0]=1
    assert np.allclose(exterior_anchor(np.concatenate([inner,panel]),0,1,(0,0,.2)),[2,0,.2])
    assert np.allclose(exterior_anchor(np.concatenate([inner,panel]),0,-1,(0,0,.2)),[1,0,.2])


def test_missing_surface_fails_instead_of_floating_mount():
    triangle=np.array([[[2,0,0],[2,1,0],[2,0,1]]],dtype=float)
    with pytest.raises(ValueError,match='No exterior'):
        exterior_anchor(triangle,0,1,(0,5,5))
