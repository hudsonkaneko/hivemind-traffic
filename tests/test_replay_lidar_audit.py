import hashlib
import json
import numpy as np
import pytest
from scripts.audit_replay_lidar import audit


def test_audit_keeps_unknown_ids_and_checks_hashes(tmp_path):
    vehicle = dict(id='ego',x=10,y=0,heading=90,length=4,width=2)
    (tmp_path/'recording.json').write_text(json.dumps({'frames': [{'time': 0, 'vehicles': [vehicle]}]}))
    config = dict(ego_id='ego',sensor_mount_m=[-10,0,.65],sensor_to_usd_offset_s=0,
                  body_center_height_m=.65,body_height_m=1.1,warmup_scans=0)
    (tmp_path/'resolved-config.json').write_text(json.dumps(config))
    (tmp_path/'scan-metrics.json').write_text(json.dumps([dict(index=0)]))
    (tmp_path/'summary.json').write_text(json.dumps(dict(passed=False)))
    np.savez(tmp_path/'scan_0000.npz',timestamp_ns=0,offset_ns=np.array([0]),azimuth_deg=np.array([0]),
             elevation_deg=np.array([0]),range_m=np.array([6.]),flags=np.array([64]),object_label=np.array([-1]))
    hashes = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in tmp_path.iterdir()}
    (tmp_path/'manifest.json').write_text(json.dumps(dict(run_id='test',recording_sha256='test',command=['test'],output_hashes=hashes)))
    result = audit(tmp_path)
    assert result['confusion_rows_expected_columns_observed'] == [[0,0],[1,0]]
    assert result['outlier_count'] == 0
    assert result['capture_summary']['passed'] is False
    assert all(hashlib.sha256((tmp_path/name).read_bytes()).hexdigest()==value for name,value in hashes.items())
    (tmp_path/'summary.json').write_text('{}')
    with pytest.raises(ValueError,match='Artifact hash mismatch'):
        audit(tmp_path)
