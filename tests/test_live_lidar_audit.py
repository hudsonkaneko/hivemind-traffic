import hashlib
import json
import numpy as np
import pytest
from scripts.analyze_live_lidar import evaluate


def make_run(path):
    np.savez_compressed(path/'scan_0000.npz',timestamp=200_000_000,
                        az=np.zeros(2000),el=np.zeros(2000),ranges=np.full(2000,6.),
                        flags=np.full(2000,64),offset=np.zeros(2000,dtype=int))
    row=dict(step=0,traffic_time=.1,sensor_time=.2,sensor_epoch=0.,
             lidar_distance=6.,points=2000,healthy=True,speed=0.,
             requested_speed=.2,realized_speed=.2,reason='lidar-control',
             true_gap=6.,post_gap=5.98,error_m=0.,ego_x=20.,collisions=[])
    summary=dict(passed=False,steps=1,min_gap_m=5.98,max_clearance_error_m=0.,
                 final_speed=.2,progress_m=.02,speed_interventions=0,sensor_fault_actions=0)
    (path/'telemetry.json').write_text(json.dumps([row]))
    (path/'summary.json').write_text(json.dumps(summary))
    (path/'manifest.json').write_text(json.dumps(dict(config=dict(speed=8.,fault_step=-1,seconds=.1),hashes={})))
    rehash(path)


def rehash(path):
    manifest=json.loads((path/'manifest.json').read_text())
    manifest['hashes']={f.name:hashlib.sha256(f.read_bytes()).hexdigest()
                        for f in path.iterdir() if f.name!='manifest.json'}
    (path/'manifest.json').write_text(json.dumps(manifest))


def test_audit_recomputes_valid_failed_run(tmp_path):
    make_run(tmp_path)
    result=evaluate(tmp_path)
    assert result['verified'] and not result['summary']['passed']


def test_changed_artifact_rejected(tmp_path):
    make_run(tmp_path)
    (tmp_path/'summary.json').write_text('{}')
    with pytest.raises(ValueError,match='Changed artifact'):
        evaluate(tmp_path)


@pytest.mark.parametrize('field,value,match', [
    ('lidar_distance',7.,'Clearance mismatch'),
    ('requested_speed',8.,'Control mismatch'),
    ('sensor_time',.1,'timestamp mismatch'),
    ('sensor_epoch',.15,'predates'),
    ('error_m',1.,'Distance error'),
])
def test_inconsistent_telemetry_rejected_even_with_new_hash(tmp_path,field,value,match):
    make_run(tmp_path)
    rows=json.loads((tmp_path/'telemetry.json').read_text())
    rows[0][field]=value
    (tmp_path/'telemetry.json').write_text(json.dumps(rows));rehash(tmp_path)
    with pytest.raises(ValueError,match=match):evaluate(tmp_path)


def test_inconsistent_summary_rejected(tmp_path):
    make_run(tmp_path)
    summary=json.loads((tmp_path/'summary.json').read_text());summary['progress_m']=10.
    (tmp_path/'summary.json').write_text(json.dumps(summary));rehash(tmp_path)
    with pytest.raises(ValueError,match='Summary mismatch'):evaluate(tmp_path)
