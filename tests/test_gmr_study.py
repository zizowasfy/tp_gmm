import unittest
from copy import deepcopy
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_gmr_study import assert_proposal_pure, settings, MODES


class PurityTest(unittest.TestCase):
    def stats(self, mode):
        return settings(mode) | dict(mode='cartesian_ik', component_cutoffs=[3.]*5,
            attempts=0, gmm_attempts=0, gmr_attempts=0, uniform_attempts=0)

    def test_sources_are_exclusive_and_hybrid_accounts_for_both(self):
        for mode,gmm,gmr in [('gmm',10,0),('gmr_30mm',0,10),('hybrid_30mm',2,8),('reference_ik',0,0)]:
            stats=self.stats(mode) | dict(attempts=gmm+gmr,gmm_attempts=gmm,gmr_attempts=gmr)
            assert_proposal_pure(mode,stats)
        for mode,stats in [('gmr_30mm',dict(attempts=1,gmm_attempts=1)),
                           ('gmm',dict(attempts=1,gmr_attempts=1)),
                           ('reference_ik',dict(attempts=1,gmm_attempts=1))]:
            with self.assertRaises(RuntimeError): assert_proposal_pure(mode,self.stats(mode)|stats)

    def test_missing_or_contaminated_telemetry_is_fatal(self):
        for changes in [dict(uniform_attempts=1),dict(projected_attempts=1),dict(missing_anchor_fallbacks=1),
                        dict(attempts=1),dict(gmr_stddev=.015),dict(gmr_cutoff=2.),dict(component_cutoffs=[2.]),
                        dict(uniform_fraction=.1),dict(mode='joint_projected')]:
            with self.assertRaises(RuntimeError): assert_proposal_pure('hybrid_30mm',self.stats('hybrid_30mm')|changes)
        bad=self.stats('gmm');del bad['gmm_attempts']
        with self.assertRaises(RuntimeError):assert_proposal_pure('gmm',bad)

if __name__=='__main__': unittest.main()
