import unittest
import numpy as np
import sys
import os
import math
import time

# Add include to path to import gaussPDFfast
include_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../include'))
sys.path.append(include_dir)

from gaussPDFfast import gaussPDFfast

# Reference implementation (slow but correct)
def reference_gaussPDFfast(Data, Mu, invSigma, detSigma):
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)
    # Original slow logic: transpose -> tile -> transpose -> sub -> dot -> mul -> sum
    Data = np.transpose(Data) - np.tile(np.transpose(Mu), (nbData, 1))
    prob = np.sum(np.dot(Data, invSigma)*Data, 1)
    prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
    return prob

class TestGaussPDF(unittest.TestCase):
    def setUp(self):
        self.nbVar = 3
        self.nbData = 1000
        self.Data = np.random.rand(self.nbVar, self.nbData)
        self.Mu = np.random.rand(self.nbVar)
        self.Sigma = np.eye(self.nbVar)
        self.invSigma = np.linalg.inv(self.Sigma)
        self.detSigma = np.linalg.det(self.Sigma)

    def test_correctness_standard(self):
        expected = reference_gaussPDFfast(self.Data, self.Mu, self.invSigma, self.detSigma)
        actual = gaussPDFfast(self.Data, self.Mu, self.invSigma, self.detSigma)
        np.testing.assert_allclose(actual, expected, err_msg="Output mismatch for standard input")

    def test_correctness_nbData_1(self):
        Data = np.random.rand(3, 1)
        Mu = np.random.rand(3)
        expected = reference_gaussPDFfast(Data, Mu, self.invSigma, self.detSigma)
        actual = gaussPDFfast(Data, Mu, self.invSigma, self.detSigma)
        np.testing.assert_allclose(actual, expected, err_msg="Output mismatch for nbData=1")

    def test_correctness_nbVar_1(self):
        Data = np.random.rand(1, 10)
        Mu = np.random.rand(1)
        Sigma = np.eye(1)
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)
        expected = reference_gaussPDFfast(Data, Mu, invSigma, detSigma)
        actual = gaussPDFfast(Data, Mu, invSigma, detSigma)
        np.testing.assert_allclose(actual, expected, err_msg="Output mismatch for nbVar=1")

    def test_performance(self):
        loops = 1000
        start = time.time()
        for _ in range(loops):
            reference_gaussPDFfast(self.Data, self.Mu, self.invSigma, self.detSigma)
        ref_time = time.time() - start

        start = time.time()
        for _ in range(loops):
            gaussPDFfast(self.Data, self.Mu, self.invSigma, self.detSigma)
        opt_time = time.time() - start

        print(f"\nReference Time: {ref_time:.4f}s")
        print(f"Current Time:   {opt_time:.4f}s")
        if opt_time < ref_time:
             print(f"Speedup: {ref_time/opt_time:.2f}x")
        else:
             print(f"Slowdown: {opt_time/ref_time:.2f}x")

if __name__ == '__main__':
    unittest.main()
