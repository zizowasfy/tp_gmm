"""Statistical safeguards: environment resampling, pairing and familywise adjustment."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_sampling_study import bootstrap_indices, holm, sign_pvalue

class StatisticsTests(unittest.TestCase):
    def test_holm_preserves_input_order_and_monotonicity(self):
        np.testing.assert_allclose(holm([.04,.01,.03]),[.06,.03,.06])
        np.testing.assert_allclose(holm([.8,.9]),[1.,1.])

    def test_exact_two_sided_cluster_sign_test(self):
        self.assertEqual(sign_pvalue(np.ones(4),np.random.default_rng(1)),.125)
        self.assertEqual(sign_pvalue(np.zeros(4),np.random.default_rng(1)),1.)
        self.assertEqual(sign_pvalue(np.r_[np.ones(4),np.zeros(56)],np.random.default_rng(1)),.125)

    def test_directional_confirmation_test(self):
        self.assertEqual(sign_pvalue(np.ones(4),np.random.default_rng(1),alternative='greater'),.0625)
        self.assertEqual(sign_pvalue(-np.ones(4),np.random.default_rng(1),alternative='greater'),1.)

    def test_stratified_clusters_retain_paired_methods(self):
        indices=bootstrap_indices(['a','a','b','b'],np.random.default_rng(1),200)
        self.assertEqual(indices.shape,(200,4))
        self.assertTrue((indices[:,:2]<2).all())
        self.assertTrue((indices[:,2:]>=2).all())
        paired=np.stack([np.arange(4),np.arange(4)+10],axis=1)
        np.testing.assert_allclose((paired[indices,:,][...,1]-paired[indices,:,][...,0]),10)

if __name__=='__main__': unittest.main()
