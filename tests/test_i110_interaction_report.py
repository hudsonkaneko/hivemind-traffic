import json
import pytest
from scripts.report_i110_interactions import physical_difference,trace_diagnostics


def test_physical_equivalence_does_not_confuse_equal_counts_with_equal_values():
    a=[dict(time=.1,states={'ego':dict(speed=2.,acceleration=1.,x=0.,y=1.,lane_id='sb0_0',road_id='sb0')})]
    b=json.loads(json.dumps(a))
    assert physical_difference(a,b)['physical_match_within_1e_minus_9']
    b[0]['states']['ego']['lane_id']='sb0_1'
    assert not physical_difference(a,b)['physical_match_within_1e_minus_9']
    b=json.loads(json.dumps(a));b[0]['states']['ego']['speed']+=.1
    assert physical_difference(a,b)['max_abs_differences']['speed']==pytest.approx(.1)


def test_peak_diagnostic_uses_realized_acceleration_not_command(tmp_path):
    frames=[]
    for t,a in [(.1,1.5),(.2,-1.8)]:
        frames.append(dict(time=t,states={'agent_1':dict(speed=17.,acceleration=a,
            filtered_acceleration=1.3,x=0.,y=t,lane_id='sb0_1',road_id='sb0')}))
    p=tmp_path/'trace.jsonl';p.write_text(''.join(json.dumps(f)+'\n' for f in frames))
    d,_=trace_diagnostics(p)
    assert d['peak']['jerk_m_s3']==pytest.approx(-33)
    assert d['peak']['time_s']==.2 and d['peak']['after']['filtered_acceleration']==1.3
