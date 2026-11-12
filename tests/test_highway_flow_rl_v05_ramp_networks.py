import math
import xml.etree.ElementTree as ET

import pytest

from scripts.build_highway_flow_rl_v05_ramp_networks import ramp_started_geometry, GORE_SETBACK_M
from scripts.build_highway_environment_v05_bank import specifications

OPTIONS = dict(lanes=2, length=2000, bend=0, speed_mph=50, ramp_upstream_distance=120.)


def test_aux_lane_starts_at_ramp():
    _, edges, connections, _ = ramp_started_geometry(**OPTIONS)
    lanes = {e.get('id'): int(e.get('numLanes')) for e in edges}
    assert lanes['sb1'] == 2 and lanes['sb2'] == 3 and lanes['sb3'] == 3 and lanes['sb4'] == 2
    into_sb2 = sorted((c.get('from'), int(c.get('fromLane')), int(c.get('toLane')))
                      for c in connections if c.get('to') == 'sb2')
    assert into_sb2 == [('mlk_on', 0, 0), ('sb1', 0, 1), ('sb1', 1, 2)]


def test_ramp_final_segment_runs_parallel_to_road():
    _, edges, _, placement = ramp_started_geometry(**OPTIONS)
    points = [tuple(map(float, p.split(','))) for p in edges.find("edge[@id='mlk_on']").get('shape').split()]
    (ax, ay), (bx, by) = points[-2], points[-1]
    tx, ty = placement['road_tangent']
    length = math.hypot(bx - ax, by - ay)
    assert abs(((bx - ax) * tx + (by - ay) * ty) / length - 1) < 1e-6
    assert placement['parallel_approach_m'] == pytest.approx(min(60., .6 * 120.) - GORE_SETBACK_M)


@pytest.mark.parametrize('spec', specifications())
def test_every_bank_layout_builds_source_geometry(spec):
    options = {k: v for k, v in spec.items() if k != 'split'}
    _, edges, connections, _ = ramp_started_geometry(**options)
    assert int(edges.find("edge[@id='sb1']").get('numLanes')) == options['lanes']
    assert [c.get('from') for c in connections if c.get('to') == 'sb2' and c.get('toLane') == '0'] == ['mlk_on']


def test_shifted_routes_moves_only_mainline(tmp_path):
    pytest.importorskip('traci')
    from scripts.replay_highway_flow_rl_v05_native_braking import shifted_routes
    source = tmp_path / 'in.rou.xml'
    source.write_text('<routes><vehicle id="m" route="main" departLane="2"/>'
                      '<vehicle id="r" route="ramp" departLane="0"/></routes>')
    out = ET.parse(shifted_routes(source, tmp_path / 'out.rou.xml', -1)).getroot()
    assert [(v.get('id'), v.get('departLane')) for v in out] == [('m', '1'), ('r', '0')]
