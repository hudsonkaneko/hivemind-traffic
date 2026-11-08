"""Replay coordinate, clip and layered-scene regression checks; no simulator."""
from pathlib import Path
import tempfile
import unittest

from scripts.render_i110_merge_replay import (
    author_scene, elevation, road_quads, select_frames, vehicle_path, screenshot_samples,
)


class MergeGeometryTests(unittest.TestCase):
    def setUp(self):
        self.lanes = {'road_0': {'width': 3.2, 'points': [(0., 0., 2.), (0., 20., 6.)]}}

    def test_elevation_uses_lane_projection_or_recorded_z(self):
        self.assertAlmostEqual(elevation({'x': 1., 'y': 10., 'lane_id': 'road_0'}, self.lanes), 4.)
        self.assertEqual(elevation({'x': 1., 'y': 10., 'z': 9.}, self.lanes), 9.)

    def test_road_vertices_keep_elevation_and_width(self):
        quads = road_quads(self.lanes['road_0'], 0., 3.2)
        self.assertEqual(quads[0], [(-1.6, 0., 2.), (1.6, 0., 2.), (1.6, 20., 6.), (-1.6, 20., 6.)])
        dashes = road_quads(self.lanes['road_0'], 1.6, .1, True)
        self.assertEqual(len(dashes), 3)
        self.assertAlmostEqual(dashes[0][-1][1], 3.)

    def test_window_and_budget_preserve_source_timestamps(self):
        data = {'frames': [{'time': n/10, 'vehicles': []} for n in range(400)],
                'clip_start_time': 3., 'clip_end_time': 4.}
        self.assertEqual(len(select_frames(data)), 11)
        with self.assertRaises(ValueError): select_frames(data, 0., 39.)
        with self.assertRaises(ValueError): select_frames(data, 50., 51.)

    def test_identifiers_are_deterministic_legal_and_distinct(self):
        self.assertEqual(vehicle_path('agent_2'), '/World/Vehicles/vehicle_6167656e745f32')
        self.assertNotEqual(vehicle_path('vehicle-1'), vehicle_path('vehicle_1'))

    def test_merge_screenshots_use_event_time_not_midpoint(self):
        frames = [{'time': n/10} for n in range(1, 111)]
        samples = dict(screenshot_samples(frames, [{'time':3.0}, {'time':9.7}]))
        self.assertEqual(frames[samples['merge']]['time'], 3.0)
        self.assertEqual(frames[samples['merge-2']]['time'], 9.7)
        self.assertIn('midpoint', dict(screenshot_samples(frames, [])))

    def test_composed_scene_retains_pose_and_reference(self):
        try:
            from pxr import Usd, UsdGeom
        except ImportError:
            self.skipTest('USD runtime unavailable; execute this test in the existing ovrtx environment')
        vehicle = dict(id='agent_2', x=0., y=10., heading=0., length=4.8, width=1.8, lane_id='road_0')
        frames = [{'time': 3., 'vehicles': [vehicle]}, {'time': 3.1, 'vehicles': []}]
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]/'outputs') as directory:
            folder = Path(directory)
            author_scene(folder, frames, self.lanes, 'agent_2', {'agent_2'})
            stage = Usd.Stage.Open(str(folder/'world.usda'))
            self.assertFalse(stage.GetCompositionErrors())
            self.assertEqual(stage.GetDefaultPrim().GetPath().pathString, '/World')
            self.assertEqual(len(stage.GetRootLayer().subLayerPaths), 3)
            prim = stage.GetPrimAtPath(vehicle_path('agent_2'))
            self.assertEqual(tuple(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.).ExtractTranslation()), (0., 10., 4.))
            self.assertEqual(UsdGeom.Imageable(prim).GetVisibilityAttr().Get(1.), 'invisible')
            model = stage.GetPrimAtPath(vehicle_path('agent_2')+'/Model')
            self.assertTrue(model.HasAuthoredReferences())
            color = UsdGeom.Gprim(stage.GetPrimAtPath(str(model.GetPath())+'/Body')).GetDisplayColorAttr().Get()[0]
            for actual, expected in zip(color, (1., .36, .035)):
                self.assertAlmostEqual(actual, expected, places=6)


if __name__ == '__main__':
    unittest.main()
