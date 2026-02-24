import unittest
import numpy as np
import sys
import os
import math

# Add include to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'include')))

from gaussPDFfast import gaussPDFfast

class TestGaussPDF(unittest.TestCase):
    def test_gaussPDFfast_correctness(self):
        """Test that the optimized gaussPDFfast produces correct results compared to a reference implementation."""

        # Reference implementation (the original slow one)
        def gaussPDFfast_ref(Data, Mu, invSigma, detSigma):
            realmin = sys.float_info[3]
            nbVar, nbData = np.shape(Data)
            Data = np.transpose(Data) - np.tile(np.transpose(Mu), (nbData, 1))
            prob = np.sum(np.dot(Data, invSigma)*Data, 1)
            prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
            return prob

        nbVar = 3
        nbData = 100

        np.random.seed(42)
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar, 1)
        Sigma = np.eye(nbVar) + np.random.rand(nbVar, nbVar) * 0.1
        Sigma = Sigma @ Sigma.T # Make positive definite
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        expected = gaussPDFfast_ref(Data, Mu, invSigma, detSigma)
        actual = gaussPDFfast(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(actual, expected, rtol=1e-10, err_msg="Optimized gaussPDFfast results match reference")

    def test_broadcasting_shapes(self):
        """Test that it handles different shapes of Mu correctly (e.g., (nbVar,) vs (nbVar, 1))."""
        nbVar = 3
        nbData = 50
        Data = np.random.rand(nbVar, nbData)

        Mu_col = np.random.rand(nbVar, 1)
        Mu_flat = Mu_col.flatten()

        Sigma = np.eye(nbVar)
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        res_col = gaussPDFfast(Data, Mu_col, invSigma, detSigma)
        res_flat = gaussPDFfast(Data, Mu_flat, invSigma, detSigma)

        np.testing.assert_allclose(res_col, res_flat, err_msg="Should handle both (N,1) and (N,) shapes for Mu")

    def test_single_dimension(self):
        """Test with nbVar=1 (scalar case)."""
        nbVar = 1
        nbData = 50
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar, 1)
        Sigma = np.eye(nbVar) * 2.0
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        res = gaussPDFfast(Data, Mu, invSigma, detSigma)
        self.assertEqual(res.shape, (nbData,))

if __name__ == '__main__':
    unittest.main()
