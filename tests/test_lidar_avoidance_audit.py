import hashlib
import json
import numpy as np
import pytest
from scripts.audit_lidar_avoidance import audit
from traffic.lidar_avoidance import AvoidancePlanner, road_points, body_bounds, rectangle_gap, summarize


def rehash(path):
    manifest=json.loads((path/'manifest.json').read_text())
    manifest['hashes']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in path.iterdir() if f.name!='manifest.json'}
    (path/'manifest.json').write_text(json.dumps(manifest))


def make_run(path):
    scan=dict(az=np.zeros(2000),el=np.zeros(2000),ranges=np.full(2000,40.),
              flags=np.full(2000,64),timestamp=200_000_000,offset=np.zeros(2000,dtype=int))
    np.savez_compressed(path/'scan_0000.npz',**scan)
    ego=dict(x=20.,y=-4.8,angle=90.,speed=6.)
    after=dict(x=20.6,y=-4.736,angle=89.3,speed=6.)
    points,healthy=road_points(scan['az'],scan['el'],scan['ranges'],scan['flags'],ego)
    d=AvoidancePlanner().step(points,healthy,ego)
    a,b=body_bounds(ego),body_bounds(after)
    swept=(min(a[0],b[0])-.02,max(a[1],b[1])+.02,min(a[2],b[2])-.02,max(a[3],b[3])+.02)
    obstacle=(60.,64.,-5.8,-3.8)
    row=dict(step=0,ego=ego,ego_after=after,sensor_time=.2,sensor_epoch=0.,traffic_time=.1,
             requested_speed=d.speed,realized_speed=6.,phase=d.phase,reason=d.reason,
             lane_request=d.lane_request,adjacent_clear=d.adjacent_clear,healthy=d.healthy,
             lidar_distance=d.clearance,obstacle_gap=rectangle_gap(swept,obstacle),
             obstacle_end_x=64.,collisions=[])
    (path/'telemetry.json').write_text(json.dumps([row]))
    (path/'summary.json').write_text(json.dumps(summarize([row])))
    manifest=dict(config=dict(mode='avoid',speed=6.,fault_step=-1,blocked_lane=False,seconds=.1),
                  evaluation_obstacles=[obstacle],hashes={})
    (path/'manifest.json').write_text(json.dumps(manifest));rehash(path)


def test_incomplete_maneuver_is_verified_but_failed(tmp_path):
    make_run(tmp_path);result=audit(tmp_path)
    assert result['verified'] and not result['summary']['passed']


@pytest.mark.parametrize('key,value,match', [
    ('lane_request',0,'Maneuver mismatch'),('requested_speed',12.,'Command mismatch'),
    ('obstacle_gap',100.,'Geometric clearance mismatch'),('sensor_epoch',.2,'Stale scan'),
])
def test_changed_decision_or_safety_metric_rejected(tmp_path,key,value,match):
    make_run(tmp_path)
    rows=json.loads((tmp_path/'telemetry.json').read_text());rows[0][key]=value
    (tmp_path/'telemetry.json').write_text(json.dumps(rows));rehash(tmp_path)
    with pytest.raises(ValueError,match=match):audit(tmp_path)
