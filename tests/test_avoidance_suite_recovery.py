import json
from types import SimpleNamespace
import experiments.run_lidar_avoidance_suite as suite


def test_native_crash_retains_prior_successes(tmp_path,monkeypatch):
    (tmp_path/'logs').mkdir();(tmp_path/'outputs/live_lidar').mkdir(parents=True)
    monkeypatch.setattr(suite,'ROOT',tmp_path)
    monkeypatch.setattr(suite.sys,'argv',['suite'])
    calls=[]
    def fake_run(cmd,*,cwd,stdout,stderr,timeout):
        i=len(calls);calls.append(cmd)
        if i==3:
            stdout.write('Native access violation\n');return SimpleNamespace(returncode=1)
        path=tmp_path/'outputs/live_lidar'/f'run-{i}';path.mkdir()
        (path/'telemetry.json').write_text(json.dumps([
            dict(ego_after=dict(x=20,y=-4.8,speed=6),phase='outbound',lane_request=1)]))
        stdout.write('LIVE_RESULT='+json.dumps(dict(output=str(path)))+'\n')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(suite.subprocess,'run',fake_run)
    monkeypatch.setattr(suite,'audit',lambda path:dict(run=path.name,verified=True,summary=dict(passed=True)))
    assert suite.main()==1
    reports=[p for p in (tmp_path/'outputs/live_lidar').glob('avoid-suite-*.json') if 'partial' not in p.name]
    report=json.loads(reports[0].read_text())
    assert len(report['runs'])==4 and not report['passed']
    assert all(r['summary']['passed'] for r in report['runs'][:3])
    assert not report['runs'][3]['verified'] and report['repeat_passed']
    assert len(list((tmp_path/'outputs/live_lidar').glob('*partial*.json')))==4
