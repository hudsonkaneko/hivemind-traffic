"""Exact nearest footprint clearance with conservative spatial pruning.

Circumscribed circles give a LOWER bound on polygon distance. A box is skipped
only if this bound already exceeds the closest exact distance found. This is
not a sensor range, decimation, or relaxation of safety checks.
"""
import math

from traffic.bypass_validation import box_clearance


def nearest_clearance(ego, obstacles):
    if not obstacles:
        raise ValueError('A nonempty obstacle frame is required')
    ego_radius=math.hypot(ego.length_m,ego.width_m)/2
    candidates=[]
    for index,box in enumerate(obstacles):
        distance=math.hypot(box.x_m-ego.x_m,box.y_m-ego.y_m)
        lower=distance-ego_radius-math.hypot(box.length_m,box.width_m)/2
        candidates.append((lower,index,box))
    best=math.inf;overlap=False;evaluated=0
    for lower,_,box in sorted(candidates,key=lambda item:item[:2]):
        if lower>best+1e-9:
            break
        exact=box_clearance(ego,box);evaluated+=1
        best=min(best,exact['clearance_m']);overlap=overlap or exact['overlap']
    return dict(clearance_m=best,overlap=overlap,exact_pairs=evaluated,
                obstacle_count=len(obstacles))
