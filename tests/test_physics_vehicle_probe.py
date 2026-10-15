"""CPU tests for acceptance gates; not substitutes for physical runtime tests."""
from copy import deepcopy
import json
from pathlib import Path

import pytest
from scripts.probe_physics_vehicle import assess


@pytest.fixture
def example():
    config = json.loads((Path(__file__).parents[1]/'experiments/physics-vehicle-smoke.json').read_text())
    rows, bounds = [], {}
    for phase in ('settle', 'straight', 'brake', 'hold', 'turn', 'coast', 'dropout'):
        start = len(rows)
        for i in range(2):
            speed = 3.0 if phase in ('straight','turn','coast') else 0.0
            x = (20.0*i if phase == 'straight' else 21.0)
            yaw = 0.5*i if phase == 'turn' else 0.0
            rows.append(dict(tick=len(rows)+1, phase=phase, position_m=[x,0,1],
                             velocity_m_s=[speed,0,0], speed_m_s=speed,
                             yaw_rad=yaw, upright_z=1.0, wheel_on_ground=[True]*4,
                             control=dict(throttle=0.0,brake=1.0)))
        bounds[phase] = start,len(rows)
    return rows,bounds,config['acceptance']


def test_acceptance_positive_fixture(example):
    assert assess(*example)['passed']


@pytest.mark.parametrize('failure,gate', [
    ('backwards','forward'), ('no_braking','braking'), ('slow_entry','braking'),
    ('turn_wrong_way','turn_left'), ('hovering','height'), ('airborne','grounded'),
    ('stale_throttle','expiry_stop'), ('rolling','hold'), ('drift','straight_drift'),
])
def test_rejects_bad_trajectory(example, failure, gate):
    rows,bounds,limits = deepcopy(example)
    start,end = bounds['straight']
    if failure == 'backwards':
        rows[end-1]['velocity_m_s'][0] = -3
    elif failure == 'no_braking':
        rows[bounds['brake'][1]-1]['speed_m_s'] = 1
    elif failure == 'slow_entry':
        rows[end-1]['speed_m_s'] = 1
    elif failure == 'turn_wrong_way':
        rows[bounds['turn'][1]-1]['yaw_rad'] = -0.5
    elif failure == 'hovering':
        rows[end-1]['position_m'][2] = 5
    elif failure == 'airborne':
        rows[end-1]['wheel_on_ground'][0] = False
    elif failure == 'stale_throttle':
        rows[-1]['control']['throttle'] = 1
    elif failure == 'rolling':
        rows[bounds['hold'][1]-1]['speed_m_s'] = 0.2
    elif failure == 'drift':
        rows[end-1]['position_m'][1] = 1
    result = assess(rows,bounds,limits)
    assert not result['gates'][gate]
    assert not result['passed']
