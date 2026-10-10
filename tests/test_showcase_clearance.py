import math
import random

import pytest

from traffic.bypass_validation import OrientedBox,box_clearance
from traffic.showcase_clearance import nearest_clearance


def exhaustive(ego,boxes):
    pairs=[box_clearance(ego,box) for box in boxes]
    return min(p['clearance_m'] for p in pairs),any(p['overlap'] for p in pairs)


def test_pruning_matches_full_geometry_for_seeded_rotated_boxes():
    rng=random.Random(101)
    for _ in range(150):
        ego=OrientedBox(rng.uniform(-500,500),rng.uniform(-500,500),rng.uniform(-math.pi,math.pi),5.1,2.22)
        boxes=[OrientedBox(ego.x_m+rng.uniform(-200,200),ego.y_m+rng.uniform(-20,20),
            rng.uniform(-math.pi,math.pi),rng.uniform(1,8),rng.uniform(.5,2.5)) for _ in range(80)]
        actual=nearest_clearance(ego,boxes);distance,overlap=exhaustive(ego,boxes)
        assert actual['clearance_m']==pytest.approx(distance,abs=1e-10)
        assert actual['overlap']==overlap


def test_touching_and_overlapping_boxes_cannot_be_pruned():
    ego=OrientedBox(0,0,0,5,2)
    for x in (0.,4.999999999,5.,5.000000001):
        boxes=[OrientedBox(1000,0,0),OrientedBox(x,0,0,5,2)]
        actual=nearest_clearance(ego,boxes);distance,overlap=exhaustive(ego,boxes)
        assert (actual['clearance_m'],actual['overlap'])==(distance,overlap)


def test_far_objects_are_pruned_but_nearest_distance_remains_exact():
    ego=OrientedBox(0,0,0)
    boxes=[OrientedBox(10+i*20,0,0) for i in range(80)]
    actual=nearest_clearance(ego,boxes)
    assert actual['clearance_m']==pytest.approx(5.2)
    assert actual['exact_pairs']==1
    assert actual['obstacle_count']==80
    with pytest.raises(ValueError):nearest_clearance(ego,[])
