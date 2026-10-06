"""Regression checks: the validator must reject plausible corrupted assets."""
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from pxr import Usd, UsdGeom, UsdPhysics
from validate_highway import validate

HERE = Path(__file__).resolve().parent
OUTPUTS = HERE.parents[1]/"outputs"/"highway_usd"


class ValidationRejectsCorruption(unittest.TestCase):
    def setUp(self):
        OUTPUTS.mkdir(parents=True,exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="validator-test-",dir=OUTPUTS)
        self.directory = Path(self.temp.name).resolve()
        assert self.directory.is_relative_to(OUTPUTS.resolve())
        for name in ["navigation.json","manifest.json","parameters.json","highway_v01.usda","generate_highway.py"]:
            shutil.copy2(HERE/name,self.directory/name)
        self.network=json.loads((self.directory/"navigation.json").read_text())

    def tearDown(self):
        self.temp.cleanup()

    def save_navigation(self):
        (self.directory/"navigation.json").write_text(json.dumps(self.network))

    def test_dead_end_is_rejected(self):
        self.network["edges"]["RampEast"]["successors"]=[]
        self.save_navigation()
        with self.assertRaisesRegex(AssertionError,"Dead-end edge RampEast"):
            validate(self.directory)

    def test_broken_reconnection_is_rejected(self):
        # Keep the USD and JSON versions in agreement: rejection must be about
        # connectivity, rather than only sidecar consistency or manifest hashes.
        stage=Usd.Stage.Open(str(self.directory/"highway_v01.usda"))
        for item in [self.network["lanes"]["RampEast"], self.network["edges"]["RampEast"]]:
            item["points"][-1][0]+=.25
            UsdGeom.BasisCurves(stage.GetPrimAtPath(item["usd_path"])).GetPointsAttr().Set(item["points"])
        stage.GetRootLayer().Save()
        stage=None
        self.save_navigation()
        with self.assertRaisesRegex(AssertionError,"Open join RampEast"):
            validate(self.directory)

    def test_missing_road_collision_is_rejected(self):
        stage=Usd.Stage.Open(str(self.directory/"highway_v01.usda"))
        stage.GetPrimAtPath("/World/Physics/RoadCollider").RemoveAPI(UsdPhysics.CollisionAPI)
        stage.GetRootLayer().Save()
        stage=None
        with self.assertRaisesRegex(AssertionError,"No road collider"):
            validate(self.directory)


if __name__ == "__main__":
    unittest.main(verbosity=2)
