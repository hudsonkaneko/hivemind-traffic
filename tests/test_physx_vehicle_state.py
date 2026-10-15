"""Optional duplicate-publication avoidance must leave native reads intact."""
import sys
from types import ModuleType,SimpleNamespace
import pytest
from traffic.physx_vehicle import PhysxVehicle


@pytest.mark.parametrize('paths',[[],['/other'],['/ego','/other']])
def test_native_state_rejects_wrong_identity(paths):
    car=PhysxVehicle.__new__(PhysxVehicle)
    car.path='/ego'
    with pytest.raises(RuntimeError,match='identity mismatch'):
        car.bind_native_state(SimpleNamespace(create_rigid_body_view=lambda path:SimpleNamespace(prim_paths=paths)))
    assert getattr(car,'_native_state_view',None) is None


@pytest.mark.parametrize('refresh',[True,False])
def test_refresh_control_preserves_native_state(monkeypatch,refresh):
    binding=ModuleType('omni.physx.bindings._physx')
    binding.VEHICLE_WHEEL_STATE_IS_ON_GROUND=0
    monkeypatch.setitem(sys.modules,binding.__name__,binding)
    calls=[]
    car=PhysxVehicle.__new__(PhysxVehicle)
    car.path='/ego';car.wheel_paths=['fl','fr','rl','rr']
    car.physx=SimpleNamespace(get_rigidbody_transformation=lambda p:dict(ret_val=True,position=[5,6,1],rotation=[0,0,0,1]),
        update_transformations=lambda *args:calls.append(args),get_wheel_state=lambda p:[True])
    car.body=SimpleNamespace(GetVelocityAttr=lambda:SimpleNamespace(Get=lambda:[3,4,0]))
    state=car.state(refresh_usd=refresh)
    assert state['speed_m_s']==5
    assert state['position_m']==[5,6,1]
    assert state['wheel_on_ground']==[True]*4
    assert calls==([(False,True,True,False)] if refresh else [])
    import numpy as np
    view=SimpleNamespace(prim_paths=['/ego'],get_velocities=lambda:np.array([[6.,8.,0.,0.,0.,0.]]))
    car.bind_native_state(SimpleNamespace(create_rigid_body_view=lambda path:view))
    assert car.state(refresh_usd=False)['speed_m_s']==10
    with pytest.raises(RuntimeError):car.bind_native_state(None)
    view.get_velocities=lambda:np.zeros((2,6))
    with pytest.raises(RuntimeError):car.state(refresh_usd=False)
    for bad in (None,0,1,'false'):
        with pytest.raises(ValueError):car.state(refresh_usd=bad)
