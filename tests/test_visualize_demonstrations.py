import unittest
import sys
from pathlib import Path
import rclpy

pkg_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(pkg_dir / "scripts"))

from visualize_demonstrations import DemonstrationVisualizer


class TestDemonstrationsVisualizer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def test_visualizer_initialization_and_topics(self):
        """Visualizer node must load demonstration bags, identify ref demon, and build valid PoseArrays and MarkerArrays."""
        demons_dir = Path("/home/zizo/the_folder/Trajectory_Data_Collection/Demons/franka_pick_cube")
        if not demons_dir.exists():
            self.skipTest("franka_pick_cube demonstrations not found.")

        node = DemonstrationVisualizer(task_name="franka_pick_cube", publish_rate=1.0)
        try:
            # Check reference demon
            self.assertEqual(node.demon_data["ref_num"], "demon_3")
            self.assertEqual(node.demon_data["ref_nbpoints"], 24)
            self.assertEqual(len(node.demon_keys), 50)

            # Check PoseArrays
            self.assertGreater(len(node.raw_pose_array_msg.poses), 0)
            self.assertEqual(len(node.ref_pose_array_msg.poses), 24)

            # Check MarkerArrays
            self.assertGreater(len(node.raw_markers_msg.markers), 0)
            self.assertGreater(len(node.aligned_markers_msg.markers), 0)
            self.assertGreater(len(node.ref_markers_msg.markers), 0)

            # Ensure all aligned demonstrations have uniform length (24)
            for d_id, data in node.demon_data["aligned"].items():
                self.assertEqual(len(data["poses"]), 24)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
