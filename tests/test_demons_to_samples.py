import unittest
from geometry_msgs.msg import Pose, PoseArray
import numpy as np


class TestPoseExtractionDeduplication(unittest.TestCase):
    def test_deduplication_of_messages(self):
        """When multiple PoseArray messages exist in a stream, only the target message should be extracted."""
        # Message 1 (older, 5 poses)
        msg1 = PoseArray()
        for i in range(5):
            p = Pose()
            p.position.x = float(i)
            msg1.poses.append(p)

        # Message 2 (latest, 10 poses)
        msg2 = PoseArray()
        for i in range(10):
            p = Pose()
            p.position.x = float(i * 2)
            msg2.poses.append(p)

        posearray_msgs = [msg1, msg2]

        target_msg = posearray_msgs[-1]
        dim = 4
        num_poses = len(target_msg.poses)
        data_points = np.zeros((dim, num_poses))
        for pose_count, pose in enumerate(target_msg.poses):
            data_points[:, pose_count] = [pose_count, pose.position.x, pose.position.y, pose.position.z]

        # Verify no duplicate or concatenated points
        self.assertEqual(data_points.shape, (4, 10))
        self.assertEqual(data_points[0, -1], 9)  # Monotonic index
        self.assertEqual(data_points[1, -1], 18.0)  # Correct value from msg2


if __name__ == "__main__":
    unittest.main()
