"""LA I-10/I-110 corridors: plain-XML layout from the model's lane graph, and the compiled checks."""
import json
import shutil
import uuid
from pathlib import Path

import pytest

from scripts import build_la_10_110_corridors as corridors

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def lanes():
    return corridors.load_lanes()[0]


@pytest.mark.parametrize('ident', sorted(corridors.SITES))
def test_plain_xml_has_the_environment_layout(lanes, ident):
    nodes, edges, connections, layout, *_ = corridors.plain_xml(lanes, ident)
    n = corridors.SITES[ident]['through_lanes']
    counts = {e.get('id'): int(e.get('numLanes')) for e in edges.findall('edge')}
    assert counts == {'sb1': n, 'sb2': n + 1, 'sb3': n + 1, 'sb4': n, 'sb5': n, 'mlk_on': 1}
    links = sorted((c.get('from'), int(c.get('fromLane')), c.get('to'), int(c.get('toLane')))
                   for c in connections.findall('connection'))
    assert ('mlk_on', 0, 'sb2', 0) in links
    assert [link for link in links if link[0] == 'sb3'] == [('sb3', i, 'sb4', i - 1) for i in range(1, n + 1)]
    assert layout['auxiliary_length_m'] == corridors.SITES[ident]['auxiliary_length_m'] == 250.
    assert 700. < layout['corridor_length_m'] < 1000.


def test_sites_are_real_lane_graph_ways(lanes):
    ways = {lane['way'] for lane in lanes.values()}
    for site in corridors.SITES.values():
        assert set(site['upstream_ways']) | set(site['downstream_ways']) | {site['merge_way']} <= ways
        assert all(lane in lanes for lane in site['ramp_lanes'])
        last = lanes[site['ramp_lanes'][-1]]          # the ramp proper (earlier pieces may be the arterial)
        assert last['road'] == 'motorway_link' and last['count'] == 1


def test_compiled_corridors_pass_their_checks_and_match_the_contract():
    try:
        from traffic.sumo_backend import find_sumo_binary
        find_sumo_binary()
    except Exception:
        pytest.skip('SUMO netconvert is not installed')
    from traffic import highway_flow_rl_v12_contract as contract
    output = ROOT / 'outputs' / 'test-runs' / f'la-corridors-{uuid.uuid4().hex[:8]}'
    try:
        for ident, site in contract.LA_CORRIDORS.items():
            manifest = corridors.build(output / ident, ident)
            assert manifest['compiled_checks']['passed']
            assert manifest['compiled_checks']['max_through_lane_deviation_m'] < corridors.MAX_LANE_DEVIATION_M
            assert manifest['compiled_checks']['ramp_crossings'] == manifest['compiled_checks']['ramp_foes'] == []
            assert manifest['through_lanes'] == site['through_lanes']
            assert 'ODbL' in manifest['attribution'] and manifest['model'] == contract.MODEL
            json.dumps(manifest)
    finally:
        shutil.rmtree(output, ignore_errors=True)
