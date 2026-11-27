import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
class ClosedLoopAssets(unittest.TestCase):
    def test_assets(self):
        p=Path("/workspace/simulation/gazebo")
        ET.parse(p/"worlds/closed_loop_reference.sdf")
        ast.parse((p/"closed_loop_probe.py").read_text())
        self.assertIn("/autonomymind_gazebo_only/cmd_vel",(p/"closed_loop_bridge.yaml").read_text())
        self.assertNotIn("ROS_TO_GZ",(p/"mobile_bridge.yaml").read_text())
