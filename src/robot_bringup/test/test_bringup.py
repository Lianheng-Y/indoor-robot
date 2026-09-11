import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]


class BringupIntegrationTest(unittest.TestCase):
    def test_three_point_patrol_configuration(self):
        config = yaml.safe_load((ROOT / "src/robot_bringup/config/robot.yaml").read_text())
        task = config["task_manager"]["ros__parameters"]
        self.assertEqual(len(task["waypoint_x"]), 3)
        self.assertEqual(len(task["waypoint_y"]), 3)

    def test_tf_tree_has_one_parent_per_child(self):
        root = ET.parse(ROOT / "src/robot_description/urdf/robot.urdf").getroot()
        parents = {}
        for joint in root.findall("joint"):
            child = joint.find("child").attrib["link"]
            parent = joint.find("parent").attrib["link"]
            self.assertNotIn(child, parents, f"multiple parents for {child}")
            parents[child] = parent
        self.assertEqual(parents["base_link"], "base_footprint")
        self.assertEqual(parents["lidar_link"], "base_link")
        launch = (ROOT / "src/robot_bringup/launch/simulation.launch.py").read_text()
        self.assertEqual(launch.count('"--frame-id", "map"'), 1)
        self.assertIn('"--child-frame-id", "odom"', launch)

    def test_complete_patrol_launch_contains_task_nodes(self):
        launch = (ROOT / "src/robot_bringup/launch/simulation.launch.py").read_text()
        self.assertIn('executable="point_controller_node"', launch)
        self.assertIn('executable="task_manager_node"', launch)


if __name__ == "__main__":
    unittest.main()
