import copy
import json

import pytest

from traffic.i110_anticipation import headway_signal, prior_warning
from scripts.report_i110_anticipation import verify_run


def observation():
    return dict(schema='i110_graph_v1', adjacent_context_known=True,
                neighbors={lane:dict(front=None,rear=None) for lane in ('own','left','right')})


def test_margin_units_and_closing_direction():
    obs=observation()
    obs['neighbors']['own']['front']=dict(id='car',bumper_gap_m=40.,relative_speed_m_s=-5.)
    signal=headway_signal(20.,obs,headway_s=1.5)
    assert signal['candidates'][0]['required_gap_m']==37.5
    assert not signal['warning']
    assert headway_signal(20.,obs,headway_s=2.)['warning']
    obs['neighbors']['own']['front']['relative_speed_m_s']=5.
    assert headway_signal(20.,obs,headway_s=1.5)['candidates'][0]['required_gap_m']==32.5


def test_adjacent_unknown_not_clear_and_does_not_hide_own_hazard():
    obs=observation();obs['adjacent_context_known']=False
    assert headway_signal(10.,obs,headway_s=1.5,adjacent=True)['clear'] is None
    obs['neighbors']['own']['front']=dict(id='car',bumper_gap_m=1.,relative_speed_m_s=0.)
    assert headway_signal(10.,obs,headway_s=1.5,adjacent=True)['warning']


def test_adjacent_presence_is_hypothetical_and_no_schedule_or_identity_privilege():
    obs=observation();obs['neighbors']['left']['front']=dict(id='arbitrary',bumper_gap_m=0.,relative_speed_m_s=0.)
    original=copy.deepcopy(obs)
    a=headway_signal(15.,obs,headway_s=1.5,adjacent=True)
    assert a['warning'] and not headway_signal(15.,obs,headway_s=1.5)['warning']
    assert obs==original
    obs['future_event_schedule']={'cutin_at':1}
    obs['future_jerk']=1000
    assert headway_signal(15.,obs,headway_s=1.5,adjacent=True)==a
    obs['neighbors']['left']['front']['id']='unrelated'
    b=headway_signal(15.,obs,headway_s=1.5,adjacent=True)
    assert b['warning']==a['warning'] and b['candidates'][0]['margin_m']==a['candidates'][0]['margin_m']


def test_scoring_excludes_current_future_wrong_target_and_wrong_ego():
    def row(t,ego='ego',target='target'):
        return dict(time_s=t,vehicle=ego,signal=dict(candidates=[dict(vehicle=target,warning=True)]))
    rows=[row(0),row(1),row(2),row(3),row(4),row(2.9,'other'),row(2.8,target='other')]
    assert prior_warning(rows,'ego',3,'target',2)==dict(target='target',prior_warning_samples=2,earliest_lead_s=2,latest_lead_s=1)


@pytest.mark.parametrize('speed',[-1,float('nan'),float('inf')])
def test_invalid_speed_rejected(speed):
    with pytest.raises(ValueError):headway_signal(speed,observation(),headway_s=1.5)


def test_changed_trace_rejected(tmp_path):
    (tmp_path/'trace.jsonl').write_text('changed')
    (tmp_path/'manifest.json').write_text(json.dumps(dict(status='completed',inputs={},outputs={'trace.jsonl':'wrong'},source_sha256={})))
    with pytest.raises(ValueError,match='bytes changed'):verify_run(tmp_path)
