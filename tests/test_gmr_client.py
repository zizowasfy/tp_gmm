import unittest
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from tp_gmm.msg import GaussianMixture
from tp_gmm_sampling import SamplingClient
from tp_gmm_sampling.reference_baseline import resample_reference


class GMRClientTest(unittest.TestCase):
    def setUp(self):
        self.client=object.__new__(SamplingClient)
        self.client.prepare_client=object()
        self.client.call=lambda client,request: request
        self.model=GaussianMixture()
        self.path=PoseArray();self.path.header.frame_id='world'
        for x in [0.,.001,.001,.1]:
            p=Pose();p.position.x=x;self.path.poses.append(p)

    def test_request_copies_reference_and_preserves_independent_flags(self):
        request=self.client.prepare(self.model,'arm','tool',Quaternion(w=1.),'cartesian_ik',
            proposal='gmr_path',reference_path=self.path,visualize=False,visualize_paths=True,gmr_stddev=.015)
        self.path.poses[0].position.x=10.
        self.assertEqual(request.reference_path.poses[0].position.x,0.)
        self.assertFalse(request.visualize);self.assertTrue(request.visualize_path)
        self.assertEqual(request.gmr_stddev,.015)
        self.assertEqual(request.cutoff,2.)

    def test_unsupported_mapping_or_missing_path_is_explicit(self):
        for mode,path in [('joint_projected',self.path),('uniform',self.path),('cartesian_ik',None)]:
            with self.assertRaises(ValueError):
                self.client.prepare(self.model,'arm','tool',Quaternion(w=1.),mode,proposal='gmr_path',reference_path=path)

    def test_session_releases_on_error(self):
        self.client.prepare=lambda *a,**kw:SimpleNamespace(request_id='one')
        released=[];self.client.release=released.append
        with self.assertRaises(RuntimeError):
            with self.client.session(): raise RuntimeError('caller failed')
        self.assertEqual(released,['one'])

    def test_baseline_arc_resampling(self):
        points=resample_reference(self.path,.01)
        self.assertTrue(np.allclose(points[:,0],np.linspace(0,.1,11)))
        self.assertTrue(np.isfinite(points).all())
        bad=deepcopy(self.path);bad.poses[0].position.x=float('nan')
        with self.assertRaises(ValueError): resample_reference(bad)

if __name__=='__main__': unittest.main()
