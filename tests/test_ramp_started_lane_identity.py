"""The lane graph must treat ramp-started networks without code changes.

Lane identity is derived from each edge's lane count minus the through-lane
count, so a mainline edge with no auxiliary lane maps every lane to a through
index, and the auxiliary lane appears only where the ramp feeds it.
"""
from traffic.i110_policy_interface import LaneGraph

NETWORK = """<net>
  <edge id="mlk_on" from="r" to="n1"><lane id="mlk_on_0" index="0" length="130"/></edge>
  <edge id="sb1" from="n0" to="n1"><lane id="sb1_0" index="0" length="380"/><lane id="sb1_1" index="1" length="380"/></edge>
  <edge id="sb2" from="n1" to="n2"><lane id="sb2_0" index="0" length="410"/><lane id="sb2_1" index="1" length="410"/><lane id="sb2_2" index="2" length="410"/></edge>
  <connection from="sb1" to="sb2" fromLane="0" toLane="1"/>
  <connection from="sb1" to="sb2" fromLane="1" toLane="2"/>
  <connection from="mlk_on" to="sb2" fromLane="0" toLane="0"/>
</net>"""


def test_ramp_started_identities(tmp_path):
    path = tmp_path / 'network.net.xml'
    path.write_text(NETWORK)
    graph = LaneGraph(path, through_lanes=2)
    assert graph.identity('sb1_0') == dict(kind='through', index_from_right=0)
    assert graph.identity('sb1_1') == dict(kind='through', index_from_right=1)
    assert graph.identity('sb2_0') == dict(kind='auxiliary')
    assert graph.identity('sb2_1') == dict(kind='through', index_from_right=0)
    assert graph.identity('mlk_on_0') == dict(kind='ramp', edge='mlk_on')
    assert graph.successors['sb1_0'] == {'sb2_1'} and graph.successors['mlk_on_0'] == {'sb2_0'}
