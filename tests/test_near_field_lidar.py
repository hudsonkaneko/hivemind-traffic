"""Profile readback and geometric reach; native validation checks occlusion."""
import math

import numpy as np
import pytest

from traffic.physical_lidar import (
    NEAR_FIELD_ELEVATIONS_DEG, NEAR_FIELD_SCAN_PROFILE, _configure_rotary_profile,
)

PREFIX = 'omni:sensor:Core:emitterState:s001:'


class Attribute:
    def __init__(self, name, value=None):
        self.name, self.value = name, value

    def GetName(self):
        return self.name

    def Get(self):
        return self.value

    def Set(self, value):
        self.value = value
        return True


class Prim:
    def __init__(self):
        arrays = dict(azimuthDeg=[-3.0]*128, elevationDeg=list(np.linspace(-15, 10, 32))*4,
                      channelId=list(range(1, 129)), fireTimeNs=[3500*(i//16) for i in range(128)],
                      distanceCorrectionM=[0.0]*128, bank=[])
        self.attrs = {PREFIX+k: Attribute(PREFIX+k, v) for k, v in arrays.items()}

    def GetAttributes(self):
        return list(self.attrs.values())

    def GetAttribute(self, name):
        return self.attrs.setdefault(name, Attribute(name))


@pytest.mark.parametrize('speed_profile, firing', [('low-speed', 7200.0), ('35mph', 72000.0)])
def test_profile_preserves_rates_and_aligns_all_emitter_arrays(speed_profile, firing):
    prim = Prim()
    rates = _configure_rotary_profile(prim, speed_profile, scan_profile=NEAR_FIELD_SCAN_PROFILE)
    assert rates == dict(sensor_tick=20.0, rotary_scan=20.0, pattern_firing=firing)
    assert prim.GetAttribute('omni:sensor:Core:numberOfEmitters').Get() == 64
    for attr in prim.GetAttributes():
        if attr.GetName().startswith(PREFIX):
            assert len(attr.Get()) in (0, 64)
    assert np.allclose(prim.GetAttribute(PREFIX+'elevationDeg').Get(), NEAR_FIELD_ELEVATIONS_DEG)
    assert prim.GetAttribute(PREFIX+'fireTimeNs').Get() == [3500*(i//16) for i in range(64)]


def test_default_retains_legacy_optics_and_emitter_count():
    prim = Prim()
    elevation = prim.GetAttribute(PREFIX+'elevationDeg').Get()[:32]
    _configure_rotary_profile(prim)
    assert prim.GetAttribute(PREFIX+'elevationDeg').Get() == elevation
    assert prim.GetAttribute(PREFIX+'azimuthDeg').Get() == [-3.0]*32
    assert prim.GetAttribute('omni:sensor:Core:numberOfEmitters').Get() == 32


@pytest.mark.parametrize('distance', [2.0, 4.0, 6.0])
def test_rays_intersect_thirty_cm_target_at_lateral_range_without_occlusion(distance):
    # Roof sensor at 2.45m over level ground. This intentionally does not model
    # car geometry: real body/mount occlusion is tested with native RTX returns.
    heights = 2.45+distance*np.tan(np.radians(NEAR_FIELD_ELEVATIONS_DEG))
    assert np.any((heights >= 0.0) & (heights <= .3))
    # Previous -15deg minimum cannot reach any of these targets.
    assert 2.45+distance*math.tan(math.radians(-15)) > .3


def test_bad_profile_and_misaligned_array_fail_closed():
    with pytest.raises(ValueError, match='Unknown'):
        _configure_rotary_profile(Prim(), scan_profile='typo')
    prim = Prim()
    prim.GetAttribute(PREFIX+'fireTimeNs').Set([0]*17)
    with pytest.raises(RuntimeError, match='Misaligned'):
        _configure_rotary_profile(prim, scan_profile=NEAR_FIELD_SCAN_PROFILE)


def test_failed_elevation_readback_is_rejected():
    prim = Prim()
    attr = prim.GetAttribute(PREFIX+'elevationDeg')
    attr.Set = lambda value: True  # Simulate a backend silently ignoring authoring.
    with pytest.raises(RuntimeError, match='Cannot configure/read back'):
        _configure_rotary_profile(prim, scan_profile=NEAR_FIELD_SCAN_PROFILE)
