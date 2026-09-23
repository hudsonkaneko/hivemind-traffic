import json
from pathlib import Path
import numpy as np
from scripts.lidar_lifecycle_analysis import analyze,summarize

CONFIG=json.loads((Path(__file__).resolve().parents[1]/'experiments/configs/lidar_lifecycle.json').read_text())


def scan(time,label=True):
    return dict(timestamp_ns=round((time+CONFIG['sensor_to_usd_offset_s'])*1e9),azimuth_deg=np.array([0.]),
        elevation_deg=np.array([0.]),range_m=np.array([10.]),flags=np.array([64]),
        target_label=np.array([label]),echo_id=np.array([0]))


def test_lifecycle_phase_guard_and_geometry():
    assert analyze(scan(.95),CONFIG,'lifecycle')['phase']=='transition_excluded'
    assert analyze(scan(1.3),CONFIG,'lifecycle')['phase']=='visible_first'
    assert analyze(scan(2.3),CONFIG,'lifecycle')['phase']=='hidden_again'
    assert analyze(scan(3.3),CONFIG,'lifecycle')['max_m']==0


def test_unknown_identity_remains_failure():
    row=analyze(scan(1.3,False),CONFIG,'lifecycle')
    assert row['identity_matches']==0 and row['max_m']==0
    assert not summarize([row],CONFIG,'lifecycle')['passed']


def test_empty_run_fails():
    assert not summarize([],CONFIG,'always-visible')['passed']


def test_valid_control_and_wrong_identity_gates():
    rows=[dict(analyze(scan(.3+i*.1),CONFIG,'always-visible'),hits=100,identity_matches=100) for i in range(4)]
    assert summarize(rows,CONFIG,'always-visible')['passed']
    rows[0]['identity_matches']=0
    assert not summarize(rows,CONFIG,'always-visible')['passed']


def test_hidden_target_returns_are_not_accepted():
    rows=[analyze(scan(.3+i*.1),CONFIG,'lifecycle') for i in range(4)]
    result=summarize(rows,CONFIG,'lifecycle')
    assert not result['phases']['hidden_initial']['geometry_passed']
