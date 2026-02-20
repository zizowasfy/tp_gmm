
import unittest
import numpy as np
import sys
import os
import math

# Add include/ to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../include')))

# Import the function to be tested
from gaussPDFfast import gaussPDFfast

# Original implementation for reference
def gaussPDFfast_legacy(Data, Mu, invSigma, detSigma):
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)
    Data = np.transpose(Data) - np.tile(np.transpose(Mu), (nbData, 1))
    prob = np.sum(np.dot(Data, invSigma)*Data, 1)
    prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
    return prob

class TestGaussPDFFast(unittest.TestCase):
    def setUp(self):
        np.random.seed(42)

    def test_correctness_small_dim(self):
        nbVar = 2
        nbData = 100
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar)
        Sigma = np.eye(nbVar) + np.random.rand(nbVar, nbVar) * 0.1
        Sigma = np.dot(Sigma, Sigma.T)
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        expected = gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)
        actual = gaussPDFfast(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(actual, expected, rtol=1e-5)

    def test_correctness_large_dim(self):
        nbVar = 20
        nbData = 50
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar)
        Sigma = np.eye(nbVar) + np.random.rand(nbVar, nbVar) * 0.1
        Sigma = np.dot(Sigma, Sigma.T)
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        expected = gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)
        actual = gaussPDFfast(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(actual, expected, rtol=1e-5)

    def test_correctness_single_data_point(self):
        nbVar = 3
        nbData = 1
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar)
        Sigma = np.eye(nbVar)
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        expected = gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)
        actual = gaussPDFfast(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(actual, expected, rtol=1e-5)

    def test_correctness_1D(self):
        nbVar = 1
        nbData = 100
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar)
        Sigma = np.eye(nbVar) * 2.0
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        expected = gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)
        actual = gaussPDFfast(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(actual, expected, rtol=1e-5)

    def test_correctness_Mu_shape_variants(self):
        # Case where Mu is (nbVar, 1) instead of (nbVar,)
        nbVar = 3
        nbData = 10
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar, 1) # 2D column vector
        Sigma = np.eye(nbVar)
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        # Original legacy code expects Mu to be handled via transpose and tile.
        # But let's see if legacy handles (nbVar, 1) correctly.
        # Legacy: Mu.T is (1, nbVar). Tile -> (nbData, nbVar).
        # Data.T is (nbData, nbVar). Correct.

        expected = gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)
        actual = gaussPDFfast(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(actual, expected, rtol=1e-5)

if __name__ == '__main__':
    unittest.main()
