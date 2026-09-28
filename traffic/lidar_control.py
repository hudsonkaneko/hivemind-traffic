"""Label-free forward clearance and bounded longitudinal control (no scene IDs)."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Clearance:
    distance: float
    healthy: bool
    points: int


def forward_clearance(azimuth, elevation, ranges, flags):
    arrays=[np.asarray(a) for a in (azimuth,elevation,ranges,flags)]
    if len({a.shape for a in arrays})!=1 or arrays[0].ndim!=1:
        raise ValueError('Lidar arrays must be aligned one-dimensional arrays')
    az,el,r,f=arrays
    valid=np.isfinite(az)&np.isfinite(el)&np.isfinite(r)&(r>0)&((f&64)!=0)
    # A complete scan with environmental returns is required, even on an empty road.
    healthy=len(r)>=1000 and int(valid.sum())>=20
    a,e=np.radians(az),np.radians(el)
    x=r*np.cos(e)*np.cos(a); y=r*np.cos(e)*np.sin(a); z=1.+r*np.sin(e)
    roi=valid&(x>.5)&(x<80)&(np.abs(y)<.85)&(z>.3)&(z<1.5)
    # Conservative nearest return, without labels or simulator obstacle state.
    return Clearance(float(x[roi].min()) if roi.any() else 80.,healthy,int(roi.sum()))


def target_speed(clearance, speed, *, cruise=8., dt=.1, fresh=True):
    if not math.isfinite(speed) or speed<0 or not math.isfinite(cruise) or cruise<0 or not math.isfinite(dt) or dt<=0:
        raise ValueError('Invalid speed/control interval')
    if not fresh or not clearance.healthy or not math.isfinite(clearance.distance):
        return max(0.,speed-3.*dt), 'sensor-failsafe'
    remaining=max(0.,clearance.distance-5.)
    desired=min(cruise,math.sqrt(2.*2.5*remaining),remaining/1.5)
    if desired<.15: desired=0.
    requested=max(0.,min(speed+2.*dt,max(speed-3.*dt,desired)))
    return requested, 'lidar-control'
