import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
class GazeboMobileAssets(unittest.TestCase):
    def test_mobile_world_contract(self):
        root=Path("/workspace/simulation/gazebo")
        world=ET.parse(root/"worlds/mobile_reference.sdf").getroot()
        rover=world.find(".//model[@name='rover']")
        self.assertEqual(len(rover.findall("joint")),2)
        self.assertEqual({s.attrib["type"] for s in rover.findall(".//sensor")},{"imu","gpu_lidar","camera"})
        ast.parse((root/"sensor_probe.py").read_text())
        self.assertNotIn("ROS_TO_GZ",(root/"mobile_bridge.yaml").read_text())
