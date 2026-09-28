import json
import pytest
import scripts.report_live_lidar as module


@pytest.mark.parametrize('difference,expected', [(0.,True),(.0005,True),(.0015,False)])
def test_repeat_gate_not_relaxed(tmp_path,monkeypatch,difference,expected):
    runs=[]
    for i in range(2):
        path=tmp_path/str(i);path.mkdir()
        (path/'telemetry.json').write_text(json.dumps([
            dict(ego_x=20.,speed=6.,lidar_distance=30.+i*difference,post_gap=30.)]))
        runs.append(dict(run=str(i),config=dict(mode='follow',fault_step=-1),summary=dict(passed=True)))
    suite=tmp_path/'suite.json';suite.write_text(json.dumps(dict(runs=runs,diagnostics=True)))
    monkeypatch.setattr(module,'evaluate',lambda p:runs[int(p.name)])
    result=module.report(suite)
    assert result['repeat_passed'] is expected
    assert result['passed'] is expected
    assert result['fault_injection_passed'] is None


def test_native_suite_requires_fault_evidence(tmp_path,monkeypatch):
    runs=[]
    for i in range(2):
        path=tmp_path/str(i);path.mkdir()
        (path/'telemetry.json').write_text(json.dumps([
            dict(ego_x=20.,speed=6.,lidar_distance=30.,post_gap=30.)]))
        runs.append(dict(run=str(i),config=dict(mode='follow',fault_step=-1),summary=dict(passed=True)))
    suite=tmp_path/'suite.json';suite.write_text(json.dumps(dict(runs=runs)))
    monkeypatch.setattr(module,'evaluate',lambda p:runs[int(p.name)])
    result=module.report(suite)
    assert result['repeat_passed']
    assert not result['fault_injection_passed'] and not result['passed']
