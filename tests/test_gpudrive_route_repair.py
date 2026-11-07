"""Destination preservation after external placement, without running SUMO."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from traffic.gpudrive_continuous_safety import restore_destination_route


class DestinationRouteRepairTests(unittest.TestCase):
    destination = 'sb5'
    repaired_route = ('sb1', 'sb2', 'sb3', 'sb4', 'sb5')

    def ego(self, **changes):
        return dict(dict(id='agent_0', edge_id='sb1', route=('sb1',),
                         x=12.3, y=-4.5, yaw=.43, speed=8.7), **changes)

    def connection(self, edges=None, readback=None):
        edges = self.repaired_route if edges is None else edges
        vehicle = Mock(spec=['getTypeID', 'setRoute', 'getRoute'])
        vehicle.getTypeID.return_value = 'ego_actual_type'
        vehicle.getRoute.return_value = edges if readback is None else readback
        simulation = Mock(spec=['findRoute'])
        simulation.findRoute.return_value = SimpleNamespace(edges=edges)
        return SimpleNamespace(vehicle=vehicle, simulation=simulation)

    def test_upstream_repair_preserves_pose_and_uses_actual_vehicle_type(self):
        ego = self.ego()
        conn = self.connection()
        repaired, event = restore_destination_route(conn, ego, self.destination)

        self.assertEqual(tuple(repaired['route']), self.repaired_route)
        for key in ('id', 'edge_id', 'x', 'y', 'yaw', 'speed'):
            self.assertEqual(repaired[key], ego[key])
        self.assertIsNotNone(event)
        conn.vehicle.getTypeID.assert_called_once_with('agent_0')
        conn.simulation.findRoute.assert_called_once_with(
            'sb1', 'sb5', vType='ego_actual_type')
        conn.vehicle.setRoute.assert_called_once()
        identity, route = conn.vehicle.setRoute.call_args.args
        self.assertEqual(identity, 'agent_0')
        self.assertEqual(tuple(route), self.repaired_route)
        conn.vehicle.getRoute.assert_called_once_with('agent_0')

    def test_valid_normal_or_internal_route_is_unchanged(self):
        for edge in ('sb2', ':junction_0'):
            with self.subTest(edge=edge):
                ego = self.ego(edge_id=edge, route=self.repaired_route)
                conn = self.connection()
                repaired, event = restore_destination_route(conn, ego, self.destination)
                self.assertEqual(repaired, ego)
                self.assertIsNone(event)
                self.assertEqual(conn.vehicle.mock_calls, [])
                self.assertEqual(conn.simulation.mock_calls, [])

    def test_destination_alone_does_not_validate_unrelated_current_edge(self):
        conn = self.connection()
        ego = self.ego(route=('mlk_on', 'sb2', 'sb3', 'sb4', 'sb5'))
        repaired, event = restore_destination_route(conn, ego, self.destination)
        self.assertEqual(tuple(repaired['route']), self.repaired_route)
        self.assertIsNotNone(event)
        conn.vehicle.setRoute.assert_called_once()

    def test_unreachable_route_is_rejected_before_mutation(self):
        conn = self.connection(edges=())
        with self.assertRaises(ValueError):
            restore_destination_route(conn, self.ego(), self.destination)
        conn.vehicle.setRoute.assert_not_called()

    def test_wrong_route_endpoints_are_rejected_before_mutation(self):
        for route in (('sb2', 'sb3', 'sb4', 'sb5'), ('sb1', 'sb2')):
            with self.subTest(route=route):
                conn = self.connection(edges=route)
                with self.assertRaises(ValueError):
                    restore_destination_route(conn, self.ego(), self.destination)
                conn.vehicle.setRoute.assert_not_called()

    def test_truncated_internal_or_unmapped_route_is_rejected(self):
        for edge, route in ((':junction_0', ('sb1',)), ('', ('sb1',)),
                            (':junction_0', ()), ('', ())):
            with self.subTest(edge=edge, route=route):
                conn = self.connection()
                with self.assertRaises(ValueError):
                    restore_destination_route(
                        conn, self.ego(edge_id=edge, route=route), self.destination)
                self.assertEqual(conn.vehicle.mock_calls, [])
                self.assertEqual(conn.simulation.mock_calls, [])

    def test_changed_route_readback_is_rejected_even_with_same_destination(self):
        for readback in (('sb1',), ('sb1', 'sb3', 'sb4', 'sb5')):
            with self.subTest(readback=readback):
                conn = self.connection(readback=readback)
                with self.assertRaises(ValueError):
                    restore_destination_route(conn, self.ego(), self.destination)
                conn.vehicle.setRoute.assert_called_once()
                conn.vehicle.getRoute.assert_called_once_with('agent_0')


if __name__ == '__main__':
    unittest.main()
