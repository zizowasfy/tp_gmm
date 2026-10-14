import json
import unittest
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from tp_gmm_sampling.corridors import contains, connect_reference


class SupportTest(unittest.TestCase):
    def test_rotated_ellipsoid_excludes_box_corner_and_zero_weight_component(self):
        angle = .63
        axes = np.array([[np.cos(angle),-np.sin(angle),0], [np.sin(angle),np.cos(angle),0], [0,0,1]])
        component = dict(weight=1., mean=[.4,.1,.5], eigenvalues=[.0004,.0009,.0016], axes=axes.ravel().tolist())
        support = dict(geometry='gmm_ellipsoids', cutoff=3., components=[component,
            dict(component,weight=0.,mean=[10.,10.,10.])])
        local = np.array([[0,0,0], [.06,0,0], [.05,.08,0]])
        points = local@axes.T+component['mean']
        self.assertEqual(contains(json.dumps(support),np.vstack([points,[10.,10.,10.]])).tolist(), [True,True,False,False])

    def test_reference_connections_preserve_interior_and_do_not_mutate_input(self):
        def point(x):
            p=Pose();p.position.x=x;p.orientation.w=1.;return p
        original=PoseArray(poses=[point(.1),point(.2)])
        connected=connect_reference(original,point(0.),point(.3))
        self.assertEqual([p.position.x for p in connected.poses],[0.,.1,.2,.3])
        self.assertEqual([p.position.x for p in original.poses],[.1,.2])
        self.assertEqual(len(connect_reference(original,point(.1),point(.2)).poses),2)


if __name__=='__main__':unittest.main()
