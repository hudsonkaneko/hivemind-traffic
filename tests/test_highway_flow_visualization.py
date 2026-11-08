"""Offline derived recording contract; no simulator, renderer or trainer imports."""
from copy import deepcopy
import json
from pathlib import Path
import pytest
from scripts import visualize_highway_flow as visual


def recording(tmp_path,name,*,absent=False):
    directory=tmp_path/name;directory.mkdir()
    (directory/'network.net.xml').write_text('<net/>')
    frames=[]
    for i in range(31):
        vehicle=dict(id='agent_0',x=i,y=0,z=0,heading=90,speed=10.,acceleration=0.,distance_m=float(i),
            legal_speed_m_s=18.,lane_id='lane_0')
        frames.append(dict(time=round(.1+i*.1,1),vehicles=[] if absent else [vehicle]))
    data=dict(schema='test_original',source_timestep=.1,network_sha256=visual.sha(directory/'network.net.xml'),
        seed=7100000,selected_case=dict(seed=7100000),source_sha256={'frozen.py':'saved-hash'},frames=frames)
    (directory/'recording.json').write_text(json.dumps(data))
    (directory/'evaluation.json').write_text(json.dumps(dict(primary_horizon_complete=True,accounting_comparison_valid=True,
        intervention_fraction=.01,accounting=dict(trips=[]))))
    return directory,data


def test_exact_subset_full_metrics_and_absence_without_interpolation(tmp_path,monkeypatch):
    monkeypatch.setattr(visual,'ROOT',tmp_path)
    left,data=recording(tmp_path,'left');right,_=recording(tmp_path,'right')
    output=tmp_path/'outputs'/'compare'
    result=visual.compare(left,right,output)
    assert not result['simulation_run'] and not result['accepted']
    for side in ('left','right'):
        derived=json.loads((output/(side+'-5fps')/'recording.json').read_text())
        assert derived['frames']==data['frames'][::2]
        assert derived['source_sha256']==data['source_sha256']
        assert derived['source_timestep']==.1 and derived['effective_frame_interval_s']==.2
        assert derived['temporal_subset']['source_frame_indices']==list(range(0,31,2))
        assert derived['temporal_subset']['interpolation'] is False
        assert (output/(side+'-5fps')/'network.net.xml').read_bytes()==(left/'network.net.xml').read_bytes()
        assert (output/(side+'-5fps')/'evaluation.json').read_bytes()==(left/'evaluation.json').read_bytes()
    present=[r for r in result['summary'] if r['agent']=='agent_0']
    assert all(r['samples']==31 for r in present)
    assert all(not r['observed'] and r['samples']==0 for r in result['summary'] if r['agent']=='agent_4')
    assert 'not present' in (output/'agent_4-full-route.svg').read_text()
    with pytest.raises(ValueError,match='fresh'):visual.compare(left,right,output)
    with pytest.raises(ValueError,match='original recording'):
        visual.load_recording(output/'left-5fps')


def test_mismatched_cases_and_missing_native_frames_fail(tmp_path,monkeypatch):
    monkeypatch.setattr(visual,'ROOT',tmp_path)
    left,_=recording(tmp_path,'left');right,data=recording(tmp_path,'right')
    data['seed']+=1;(right/'recording.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match='same explicit'):visual.compare(left,right,tmp_path/'outputs'/'compare')
    data['seed']-=1;data['frames'].pop(3);(right/'recording.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match='contiguous'):visual.load_recording(right)


def test_missing_car_spans_are_not_bridged_or_fabricated(tmp_path,monkeypatch):
    monkeypatch.setattr(visual,'ROOT',tmp_path)
    left,data=recording(tmp_path,'left')
    data['frames'][10]['vehicles']=[]
    for f in data['frames']:
        for v in f['vehicles']:v['speed']=0.
    (left/'recording.json').write_text(json.dumps(data))
    record=visual.load_recording(left);summary=visual.summarize_agent(record,'agent_0','source')
    assert summary['samples']==30 and summary['missing_spans']==1
    assert summary['measured_intervals']==28
    assert summary['longest_observed_stop_span_s']==pytest.approx(1.9)
    assert summary['stopped_measured_exposure_s']==pytest.approx(2.8)
