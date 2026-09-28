"""Small real-SUMO contract tests; no renderer or GPU required."""
import sys
from pathlib import Path
import pytest
from traffic.live_runtime import ROOT, find_sumo

BINARY = find_sumo()
sys.path.append(str(BINARY.parent.parent/'tools'))
from traffic.live_lidar_sumo import LiveTraffic


def trace(seed):
    traffic = LiveTraffic(BINARY, ROOT/'scenarios/live_lidar/scenario.sumocfg',
                          mode='follow', speed=6, gap=30, seed=seed)
    try:
        first = traffic.snapshot()
        assert set(first['vehicles']) == {'ego', 'lead'}
        assert first['vehicles']['ego']['x'] == pytest.approx(20)
        assert first['vehicles']['lead']['x'] == pytest.approx(55)
        assert first['vehicles']['ego']['y'] == pytest.approx(-4.8)
        assert first['vehicles']['ego']['angle'] == pytest.approx(90)
        assert traffic.connection.vehicle.getSpeedMode('ego') == 31
        rows = [first]
        for i in range(100):
            state = traffic.step(max(0, 6-i*.1))
            assert state['time']-rows[-1]['time'] == pytest.approx(.1)
            assert not state['collisions']
            rows.append(state)
        assert rows[-1]['vehicles']['lead']['speed'] == 0
        return rows
    finally:
        traffic.close()
        traffic.close()
        assert traffic.connection is None


def test_same_seed_exact_trace_and_other_seed():
    assert trace(42) == trace(42)
    # No randomized demand in this minimal fixture; seed changes should still work.
    assert trace(43) == trace(42)


def test_close_on_setup_failure(monkeypatch):
    import traffic.live_lidar_sumo as module
    closed = []
    class BrokenConnection:
        @property
        def route(self):
            raise RuntimeError('injected setup error')
        def close(self):
            closed.append(True)
    monkeypatch.setattr(module.traci, 'start', lambda *a, **kw: None)
    monkeypatch.setattr(module.traci, 'getConnection', lambda *a: BrokenConnection())
    with pytest.raises(RuntimeError, match='injected'):
        LiveTraffic(BINARY, Path('unused'), mode='stop', speed=8, gap=40, seed=42)
    assert closed == [True]


def test_continuous_lane_change_timing_and_safety():
    traffic=LiveTraffic(BINARY,ROOT/'scenarios/live_lidar/scenario.sumocfg',mode='avoid',speed=6,gap=40,seed=42)
    try:
        previous=traffic.snapshot()['vehicles']['ego']
        positions=[]
        for i in range(65):
            current=traffic.step(6,1 if i==0 else None)['vehicles']['ego']
            assert abs(current['y']-previous['y'])<=.15
            positions.append(current);previous=current
        assert positions[-1]['y']==pytest.approx(-1.6)
        assert any(abs(p['angle']-90)>.1 for p in positions)
        assert traffic.connection.vehicle.getSpeedMode('ego')==31
        assert traffic.connection.vehicle.getLaneChangeMode('ego')==512
    finally:traffic.close()
