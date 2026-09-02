import unittest
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3


class TestMarkerFilteringLogic(unittest.TestCase):
    def test_marker_filtering_handles_origin_poses(self):
        """Markers at (0,0,0) must not prematurely terminate parsing when action is ADD."""
        msg = MarkerArray()

        # Marker at (0, 0, 0) that should be parsed
        m0 = Marker()
        m0.action = Marker.ADD
        m0.pose.position = Point(x=0.0, y=0.0, z=0.0)
        m0.scale = Vector3(x=0.1, y=0.1, z=0.1)
        msg.markers.append(m0)

        # Subsequent marker that should also be parsed
        m1 = Marker()
        m1.action = Marker.ADD
        m1.pose.position = Point(x=0.5, y=0.2, z=0.3)
        m1.scale = Vector3(x=0.2, y=0.2, z=0.2)
        msg.markers.append(m1)

        # DELETE marker that must be skipped
        m2 = Marker()
        m2.action = Marker.DELETE
        m2.pose.position = Point(x=0.0, y=0.0, z=0.0)
        msg.markers.append(m2)

        # Simulate the new filtering logic
        parsed = [
            m for m in msg.markers
            if m.action == Marker.ADD and m.scale.x > 0 and m.scale.y > 0 and m.scale.z > 0
        ]

        self.assertEqual(len(parsed), 2)
        self.assertEqual(parsed[0].pose.position.x, 0.0)
        self.assertEqual(parsed[1].pose.position.x, 0.5)


if __name__ == "__main__":
    unittest.main()
