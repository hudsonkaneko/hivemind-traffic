"""I-110 graph contracts, checkpoint identity and native ACC smoke coverage."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from scripts.audit_hudson_checkpoint import checkpoint_identity, recover_source_bytes
from scripts.build_i110_sumo import PROFILE, COUNTS, build, main_shape
from traffic.i110_baseline import CONTROLLED, prepare_demand, run
from traffic.trip_accounting import read_requests


def test_checkpoint_missing_and_wrong_identity_are_not_ready(tmp_path):
    p=tmp_path/'policy.pt'; expected=hashlib.sha256(b'accepted').hexdigest()
    assert not checkpoint_identity(p,expected)['present']
    p.write_bytes(b'random actor')
    assert not checkpoint_identity(p,expected)['matches_published_actor']
    p.write_bytes(b'accepted')
    assert checkpoint_identity(p,expected)['matches_published_actor']


def test_source_hash_reconstruction_only_accepts_exact_recorded_line_endings():
    blob=b'first\nsecond\n'; recorded=b'first\r\nsecond\r\n'
    result,method=recover_source_bytes(blob,hashlib.sha256(recorded).hexdigest())
    assert result==recorded
    assert method=='git_LF_to_recorded_CRLF'
    assert recover_source_bytes(blob,hashlib.sha256(b'different').hexdigest())[1]=='unresolved_content_mismatch'


def test_profile_preserves_local_southbound_direction_and_lane_order():
    d=json.loads(PROFILE.read_text()); main=d['roads']['Southbound']
    assert d['coordinates']['horizontal_crs']=='EPSG:6340'
    assert np.all(np.diff(main['station_m'])>0)
    assert np.all(np.diff(np.asarray(main['center_xyz'])[:,1])<0)
    right=main_shape(main,280,455,0,5); left=main_shape(main,280,455,4,5)
    assert np.all(right[:,0]<left[:,0])
    assert np.allclose(np.linalg.norm(left[:,:2]-right[:,:2],axis=1),14.8,atol=.01)


def test_demand_is_matched_explicit_and_boundary_complete(tmp_path):
    a=tmp_path/'a';b=tmp_path/'b';c=tmp_path/'c'
    for p in (a,b,c):p.mkdir()
    prepare_demand(a,730101);prepare_demand(b,730101);prepare_demand(c,730102)
    assert (a/'routes.rou.xml').read_bytes()==(b/'routes.rou.xml').read_bytes()
    assert (a/'routes.rou.xml').read_bytes()!=(c/'routes.rou.xml').read_bytes()
    requests=read_requests(a/'routes.rou.xml',CONTROLLED)
    assert len(requests)==44
    assert sum(r.role=='controlled' for r in requests)==4
    assert sum(r.origin=='mlk_ramp' for r in requests)==8
    assert max(r.requested_at for r in requests)<40
    xml=ET.parse(a/'routes.rou.xml')
    assert xml.find("./vType[@id='acc']").get('carFollowModel')=='ACC'


@pytest.fixture(scope='module')
def network(tmp_path_factory):
    output=tmp_path_factory.mktemp('i110')/'network'
    build(output)
    return output


def test_compiled_graph_has_correct_lane_order_connections_and_no_sideways_drop(network):
    plain=ET.parse(network/'connections.con.xml').getroot()
    assert plain.find("./connection[@from='sb0'][@fromLane='0']").get('toLane')=='1'
    assert plain.find("./connection[@from='sb3'][@fromLane='0']") is None
    assert plain.find("./connection[@from='sb3'][@fromLane='1']").get('toLane')=='0'
    assert plain.find("./connection[@from='mlk_on']").get('to')=='sb2'
    assert plain.find("./connection[@to='vernon_off']").get('from')=='sb2'
    compiled=ET.parse(network/'network.net.xml').getroot()
    assert compiled.find('location').get('netOffset')=='0.00,0.00'
    for i,count in enumerate(COUNTS):
        lanes=compiled.find(f"./edge[@id='sb{i}']").findall('lane')
        assert len(lanes)==count
        assert all(float(l.get('length'))>0 for l in lanes)
    for lane in compiled.findall('./edge/lane'):
        points=np.array([[float(v) for v in xyz.split(',')] for xyz in lane.get('shape').split()])
        assert np.all(np.diff(points[:,1])<=.02),lane.get('id')
    with pytest.raises(FileExistsError):build(network)


def test_native_acc_completes_cohort_and_closes(network,tmp_path):
    metrics=run(network/'network.net.xml',tmp_path/'run',horizon=40,drain=90)
    assert metrics['passed']
    assert metrics['primary']['totals']['requested']==44
    assert metrics['drain']['totals']['completed']==44
    assert metrics['native_tripinfo_crosscheck']
    assert not metrics['collisions']
    assert metrics['controlled_lane_pass']
    manifest=json.loads((tmp_path/'run/manifest.json').read_text())
    assert manifest['status']=='completed'
    assert manifest['training'] is False
    frames=[json.loads(row) for row in (tmp_path/'run/trace.jsonl').read_text().splitlines()]
    assert np.allclose(np.diff([f['time'] for f in frames]),.1)
    for frame in frames:
        assert all(v in frame['states'] for v in frame['departed'])
    with pytest.raises(FileExistsError):run(network/'network.net.xml',tmp_path/'run')


def test_failure_cleanup_retains_failed_manifest(network,tmp_path):
    output=tmp_path/'timeout'
    with pytest.raises(TimeoutError):run(network/'network.net.xml',output,wall_budget=1e-12)
    assert json.loads((output/'manifest.json').read_text())['status']=='failed'
    # A subsequent fresh connection proves the owned TraCI child was closed.
    from traffic.sumo_backend import SumoBackend
    with SumoBackend(output/'scenario.sumocfg',step_length=.1) as backend:
        result=backend.reset(730101)
        assert result.simulation_time==pytest.approx(.1)


@pytest.mark.parametrize('kwargs',[{'horizon':39},{'horizon':float('nan')},{'drain':-.1},{'seed':-1}])
def test_invalid_run_contract_fails_before_outputs(network,tmp_path,kwargs):
    with pytest.raises(ValueError):run(network/'network.net.xml',tmp_path/'invalid',**kwargs)
    assert not (tmp_path/'invalid').exists()
