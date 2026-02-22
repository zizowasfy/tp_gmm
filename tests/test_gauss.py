import sys
import os
import time
import numpy as np
import math

# Add include to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../include'))

try:
    from gaussPDFfast import gaussPDFfast
except ImportError:
    print("Error: Could not import gaussPDFfast")
    sys.exit(1)

def ref_gaussPDFfast(Data, Mu, invSigma, detSigma):
    # Reference implementation (original)
    realmin = sys.float_info[3]
    nbVar, nbData = np.shape(Data)
    Data = np.transpose(Data) - np.tile(np.transpose(Mu), (nbData, 1))
    prob = np.sum(np.dot(Data, invSigma)*Data, 1)
    prob = np.exp(-0.5*prob)/np.sqrt((np.power((2*math.pi), nbVar))*(detSigma+realmin))
    return prob

def run_tests():
    np.random.seed(42)

    # Test cases: (nbVar, nbData)
    test_cases = [
        (1, 1),
        (2, 10),
        (3, 100),
        (4, 1000),
        (10, 5000)
    ]

    for nbVar, nbData in test_cases:
        print(f"Testing nbVar={nbVar}, nbData={nbData}")

        Data = np.random.rand(nbVar, nbData)
        Mu = np.random.rand(nbVar) # 1D array
        Sigma = np.eye(nbVar) + np.random.rand(nbVar, nbVar) * 0.1
        Sigma = Sigma @ Sigma.T
        invSigma = np.linalg.inv(Sigma)
        detSigma = np.linalg.det(Sigma)

        # Original (Reference)
        start = time.time()
        res_ref = ref_gaussPDFfast(Data, Mu, invSigma, detSigma)
        ref_time = time.time() - start

        # New (Imported)
        start = time.time()
        res_new = gaussPDFfast(Data, Mu, invSigma, detSigma)
        new_time = time.time() - start

        # Verify correctness
        if not np.allclose(res_ref, res_new):
            print(f"FAILED: Mismatch for nbVar={nbVar}, nbData={nbData}")
            print("Ref:", res_ref[:5])
            print("New:", res_new[:5])
            sys.exit(1)

        print(f"PASSED. Ref Time: {ref_time:.6f}s, New Time: {new_time:.6f}s")

    print("\nAll tests passed!")

if __name__ == "__main__":
    run_tests()
