import ast
import unittest
from pathlib import Path

ROOT=Path("/workspace/simulation/isaac")
class IsaacSceneContract(unittest.TestCase):
    def test_assets_and_exporter_syntax(self):
        scene=(ROOT/"worlds/cooperative_reference.usda").read_text()
        self.assertTrue(scene.startswith("#usda 1.0"))
        for name in ("Rover","Drone","MovingObstacle"):
            self.assertIn('def Xform "'+name+'"',scene)
        ast.parse((ROOT/"closed_loop/export_scene.py").read_text())

    def test_exporter_is_not_actuation(self):
        source=(ROOT/"closed_loop/export_scene.py").read_text()
        self.assertIn("isaac_usd_scripted_ground_truth",source)
