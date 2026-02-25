
import unittest
import numpy as np
import time
import sys
import os
import math

# Add include directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../include'))

from gaussPDFfast import gaussPDFfast

def gaussPDFfast_legacy(Data, Mu, invSigma, detSigma):
    """
    Original implementation using np.tile and transpose.
    """
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)
    Data = np.transpose(Data) - np.tile(np.transpose(Mu), (nbData, 1))
    prob = np.sum(np.dot(Data, invSigma)*Data, 1)
    prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
    return prob

class TestGaussPDF(unittest.TestCase):
    def test_correctness(self):
        """Verify that the optimized implementation matches the legacy behavior."""
        nbVar = 4
        nbData = 100
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar, 1)
        Sigma = np.random.rand(nbVar, nbVar)
        Sigma = np.dot(Sigma, Sigma.transpose()) # Make it positive definite
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        res_new = gaussPDFfast(Data, Mu, invSigma, detSigma)
        res_legacy = gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(res_new, res_legacy, rtol=1e-5)

    def test_single_point(self):
        """Verify the (1,1) edge case used in reproduction_DSGMR.py."""
        nbVar = 1
        nbData = 1
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar, 1)
        Sigma = np.random.rand(nbVar, nbVar)
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        res_new = gaussPDFfast(Data, Mu, invSigma, detSigma)
        res_legacy = gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)

        np.testing.assert_allclose(res_new, res_legacy, rtol=1e-5)

    def test_performance(self):
        """Benchmark the performance improvement."""
        nbVar = 20 # Robotics often use 6-7, but let's test a bit larger
        nbData = 10000 # Large dataset
        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar, 1)
        Sigma = np.random.rand(nbVar, nbVar)
        Sigma = np.dot(Sigma, Sigma.transpose())
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        # Warmup
        gaussPDFfast(Data, Mu, invSigma, detSigma)
        gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)

        start = time.time()
        for _ in range(100):
            gaussPDFfast_legacy(Data, Mu, invSigma, detSigma)
        end = time.time()
        time_legacy = end - start

        start = time.time()
        for _ in range(100):
            gaussPDFfast(Data, Mu, invSigma, detSigma)
        end = time.time()
        time_new = end - start

        print(f"\nLegacy time: {time_legacy:.4f}s")
        print(f"New time: {time_new:.4f}s")
        if time_new > 0:
            print(f"Speedup: {time_legacy / time_new:.2f}x")

        # We expect the new implementation to be faster or at least not significantly slower
        # Note: for very small N, overhead might dominate, but for typical use cases it should be faster.
        self.assertLess(time_new, time_legacy)

if __name__ == '__main__':
    unittest.main()
