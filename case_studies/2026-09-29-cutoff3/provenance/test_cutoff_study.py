"""The explicit cutoff must reach strict checks without enabling mixed proposals."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_sampling_study import PURE,MODES,pure_settings,assert_pure
from replay_sampling_study import check_replay


class CutoffStudy(unittest.TestCase):
    def test_only_covariance_cutoff_changes(self):
        changed=pure_settings(3.)
        self.assertEqual(changed | {'cutoff':2.},PURE)
        self.assertEqual(PURE['cutoff'],2.)
        for bad in (float('nan'),float('inf'),.99,6.01):
            with self.assertRaises(ValueError):pure_settings(bad)

    def test_telemetry_must_match_requested_cutoff_and_purity(self):
        stats=dict(proposal='gmm',component_cutoffs=[3.,3.])
        for mode in MODES[:2]:
            assert_pure(mode,stats,3.)
            for changed in (stats | {'component_cutoffs':[2.,3.]},stats | {'component_cutoffs':[]},
                            stats | {'uniform_attempts':1},stats | {'missing_anchor_fallbacks':1},
                            stats | {'gmr_attempts':1}):
                with self.assertRaises(RuntimeError):assert_pure(mode,changed,3.)
        with self.assertRaises(RuntimeError):assert_pure('joint_projected',stats | {'online_ik_calls':1},3.)
        with self.assertRaises(RuntimeError):assert_pure('cartesian_ik',stats | {'projected_attempts':1},3.)
        assert_pure('ompl_uniform',{},3.)
        with self.assertRaises(AssertionError):assert_pure('ompl_uniform',stats,3.)

    def test_source_cutoff_is_explicit_but_other_controls_stay_strict(self):
        source=dict(status='complete',settings=pure_settings(3.),modes=MODES,runtime={'strict':True},
                    design={'repeats':1},environments=[{'id':0}],
                    schedule=[dict(environment=0,repeat=0,mode=m) for m in MODES])
        check_replay(source,source['runtime'])
        for change in ({'uniform_fraction':.1},{'cartesian_fraction':.1},{'covariance_floor':1e-4}):
            with self.assertRaises(ValueError):check_replay(source | {'settings':source['settings'] | change},source['runtime'])


if __name__=='__main__':unittest.main()
