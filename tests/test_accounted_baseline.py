from pathlib import Path
import json
import xml.etree.ElementTree as ET
import pytest

from traffic.accounted_baseline import run

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'scenarios/accounting_merge/scenario.sumocfg'


def test_runner_preserves_unfinished_primary_then_drains_every_group(tmp_path):
    result=run(CONFIG,tmp_path/'run',horizon=40,drain=90,controller='native')
    primary=result['primary']['totals']; final=result['drain']['totals']
    assert primary['requested']==12 and primary['completed']==2 and primary['unfinished']==10
    assert primary['external_wait_s']>0
    assert result['primary']['groups']['origin:ramp']['unfinished']>0
    assert final['completed']==12 and final['unfinished']==0
    assert result['native_tripinfo_crosscheck'] and result['comparison_valid']
    frames=json.loads((tmp_path/'run/recording.json').read_text())['frames']
    assert any(f['time']>40 for f in frames)
    assert any(f['accounting']['waiting']>0 for f in frames)
    assert any(f['time']<35 and f['accounting']['scheduled']==2 for f in frames)


def test_undeparted_requests_survive_stop_and_native_crosscheck(tmp_path):
    routes=ET.parse(CONFIG.parent/'routes.rou.xml')
    for v in list(routes.getroot().findall('vehicle')):
        if float(v.attrib['depart'])>0:routes.getroot().remove(v)
    routes.write(tmp_path/'routes.xml')
    config=ET.parse(CONFIG)
    config.find('./input/net-file').set('value',str(CONFIG.parent/'network.net.xml'))
    config.find('./input/route-files').set('value',str(tmp_path/'routes.xml'))
    config.write(tmp_path/'scenario.sumocfg')
    result=run(tmp_path/'scenario.sumocfg',tmp_path/'run',horizon=.2,drain=0,controller='native')
    total=result['primary']['totals']
    assert total['requested']==10 and total['completed']==0
    assert total['waiting']>0 and total['unfinished']==10
    assert total['boundary_time_s']==pytest.approx(2)
    assert result['drain_exhausted'] and result['native_tripinfo_crosscheck']
