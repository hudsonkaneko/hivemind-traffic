"""Small topology fixtures; no simulator, policy inference or rendering."""
from pathlib import Path
import tempfile
import unittest

from traffic.gpudrive_merge_audit import audit_merge


NETWORK = '''<net>
<edge id="mlk_on"><lane id="mlk_on_0" index="0" length="100" shape="0,0 0,100"/></edge>
<edge id="sb1"><lane id="sb1_0" index="0" length="100" shape="4,0 4,100"/></edge>
<edge id=":j" function="internal"><lane id=":j_0" index="0" length="2" shape="0,100 0,102"/></edge>
<edge id=":k" function="internal"><lane id=":k_0" index="0" length="2" shape="0,102 0,104"/></edge>
<edge id="sb2"><lane id="sb2_0" index="0" length="100" shape="0,104 0,204"/>
<lane id="sb2_1" index="1" length="100" shape="4,104 4,204"/></edge>
<connection from="mlk_on" to="sb2" fromLane="0" toLane="0" via=":j_0"/>
<connection from=":j" to="sb2" fromLane="0" toLane="0" via=":k_0"/>
<connection from=":k" to="sb2" fromLane="0" toLane="0"/>
<connection from="sb1" to="sb2" fromLane="0" toLane="0"/>
</net>'''


class MergeAuditTests(unittest.TestCase):
    def audit(self, lane_ids):
        frames = [dict(time=round(i*.1, 2), vehicles=[dict(
            id='agent_0', lane_id=lane, x=3., y=70.)]) for i, lane in enumerate(lane_ids)]
        output_root = Path(__file__).resolve().parents[1]/'outputs'
        with tempfile.TemporaryDirectory(dir=output_root) as temporary:
            network = Path(temporary)/'network.net.xml'
            network.write_text(NETWORK, encoding='utf-8')
            return audit_merge(dict(frames=frames), network)

    def test_early_cross_road_is_not_a_connected_ramp_exit(self):
        result = self.audit(['mlk_on_0', 'sb1_0', 'sb2_0'])
        self.assertEqual(result['unconnected_cross_road_transition_count'], 1)
        exit_event = result['first_ramp_exit']
        self.assertEqual(exit_event['classification'], 'unconnected_cross_road')
        projection = exit_event['source_ramp_projection']
        self.assertEqual(projection['projected_position_m'], 70.)
        self.assertEqual(projection['projected_distance_to_ramp_end_m'], 30.)
        self.assertEqual(projection['distance_to_ramp_centerline_m'], 3.)
        self.assertFalse(result['legitimate_merge_verified'])

    def test_internal_connector_chain_can_be_observed_or_skipped(self):
        for lanes in (['mlk_on_0', ':j_0', ':k_0', 'sb2_0'], ['mlk_on_0', 'sb2_0']):
            with self.subTest(lanes=lanes):
                result = self.audit(lanes)
                self.assertEqual(result['unconnected_cross_road_transition_count'], 0)
                self.assertTrue(all(t['classification'] == 'graph_connected'
                                    for t in result['mapped_lane_transitions']))
                self.assertFalse(result['legitimate_merge_verified'])

    def test_same_edge_change_does_not_verify_lane_marking_permission(self):
        result = self.audit(['sb2_0', 'sb2_1'])
        self.assertEqual(result['mapped_lane_transitions'][0]['classification'], 'same_edge_lane_change')
        self.assertFalse(result['lane_marking_legality_verified'])

    def test_unknown_mapping_is_not_counted_as_proven_cross_road(self):
        result = self.audit(['mlk_on_0', '', 'sb1_0'])
        self.assertEqual(result['unknown_mapping_transition_count'], 2)
        self.assertEqual(result['unconnected_cross_road_transition_count'], 0)

    def test_reverse_direction_is_not_inferred_from_forward_links(self):
        result = self.audit(['sb2_0', 'mlk_on_0'])
        self.assertEqual(result['unconnected_cross_road_transition_count'], 1)


if __name__ == '__main__':
    unittest.main()
