import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_sampling_approaches import summarize

class ReportingTest(unittest.TestCase):
    def test_failed_and_warmup_trials_do_not_bias_success_metrics(self):
        rows = [dict(mode='cartesian_ik', success=True, action_wall_s=1., sampling={'online_ik_s':0., 'attempts':10., 'valid_samples':5.}),
                dict(mode='cartesian_ik', success=False, action_wall_s=9., sampling={'online_ik_s':2., 'attempts':10., 'valid_samples':0.}),
                dict(mode='cartesian_ik', success=True, warmup=True, action_wall_s=100., sampling={})]
        summary = summarize(rows, ['cartesian_ik'])['cartesian_ik']
        self.assertEqual(summary['trials'], 2)
        self.assertEqual(summary['success_rate'], 0.5)
        self.assertEqual(summary['action_wall_s_all_median'], 5.)
        self.assertEqual(summary['action_wall_s_successful_median'], 1.)
        self.assertEqual(summary['online_ik_s_median'], 1.)
        self.assertEqual(summary['valid_per_attempt'], 0.25)

    def test_no_samples_report_unavailable_efficiency(self):
        summary = summarize([dict(mode='uniform', success=False)], ['uniform'])['uniform']
        self.assertIsNone(summary['valid_per_attempt'])
        self.assertIsNone(summary['min_world_clearance_m_median'])
        self.assertEqual(summary['inactive_sampler_trials'], 1)
