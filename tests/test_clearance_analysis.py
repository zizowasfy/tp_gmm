import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from clearance_analysis import (cylinder_distance, curve_metrics, clearances, random_environment,
                                DEFAULT_ENVIRONMENT, validate_environment, policy_metrics, monotonic_summary, summarize)


class ClearanceAnalysisTest(unittest.TestCase):
    def test_finite_cylinder_sides_caps_and_corner(self):
        points=[[2.,0,0],[0,0,2],[2,0,2],[0,0,0],[1,0,0],[0,0,1]]
        np.testing.assert_allclose(cylinder_distance(points,[0,0,0],1,2),[1,1,np.sqrt(2),-1,0,0])
        shifted=np.array(points)+[3,4,5]
        np.testing.assert_allclose(cylinder_distance(shifted,[3,4,5],1,2),[1,1,np.sqrt(2),-1,0,0])

    def test_segment_crossing_detected_between_safe_endpoints(self):
        case=dict(obstacle=[0,0,0],radius=1.,height=2.,start=[-2,0,0],goal=[2,0,0])
        metric=curve_metrics([case['start'],case['goal']],case,step=.01)
        self.assertAlmostEqual(metric['min_clearance_m'],-1)
        self.assertTrue(metric['intersecting'])
        self.assertAlmostEqual(metric['path_length_m'],4.)
        self.assertEqual(metric['endpoint_goal_error_m'],0.)

    def test_design_reproducibility_and_levels(self):
        self.assertEqual(clearances(),[i/10 for i in range(1,11)])
        self.assertEqual(len(clearances(True)),11)
        validate_environment(DEFAULT_ENVIRONMENT)
        self.assertEqual(random_environment(np.random.default_rng(8),DEFAULT_ENVIRONMENT,0),
                         random_environment(np.random.default_rng(8),DEFAULT_ENVIRONMENT,0))

    def test_training_metric_is_3d_and_excludes_endpoints(self):
        model={'gaussians':[{'means':[0,0,0]},{'means':[0,0,1]},{'means':[0,0,0]}]}
        metric=policy_metrics(model,model,np.zeros(3),.1,.3)
        self.assertAlmostEqual(metric['intermediate_mean_min_margin_m'],.9)
        self.assertTrue(metric['training_clearance_satisfied'])
        self.assertEqual(metric['mean_shift_rms_m'],0)

    def test_missing_levels_and_failures_not_hidden(self):
        rows=[dict(trial=0,clearance=.1,sampler='cartesian_ik',success=True,planned_min_clearance_m=.1),
              dict(trial=0,clearance=.2,sampler='cartesian_ik',success=False),
              dict(trial=0,clearance=.3,sampler='cartesian_ik',success=True,planned_min_clearance_m=.3)]
        self.assertEqual(monotonic_summary(rows,'planned_min_clearance_m')['adjacent_pairs'],0)
        summary=summarize(rows)['cartesian_ik']['levels']
        self.assertEqual(summary[1]['success_rate'],0)
        self.assertIsNone(summary[1]['planned_min_clearance_m_median'])
        self.assertEqual(summary[1]['planned_min_clearance_m_n'],0)


if __name__=='__main__':
    unittest.main()
