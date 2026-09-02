import unittest
import sys
from pathlib import Path
import numpy as np

pkg_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(pkg_dir / "include"))

from modelClass import model
from refClass import ref
from pClass import p
from computeResultingGaussians import computeResultingGaussians


class TestResultingGaussians(unittest.TestCase):
    def setUp(self):
        self.nb_var = 4
        self.nb_states = 3
        self.nb_frames = 2

        # Create model with random symmetric positive definite covariances
        reflist = []
        for m in range(self.nb_frames):
            zmu = np.random.randn(self.nb_var, self.nb_states)
            zsigma = np.zeros((self.nb_var, self.nb_var, self.nb_states))
            for i in range(self.nb_states):
                rand_mat = np.random.randn(self.nb_var, self.nb_var)
                cov = np.dot(rand_mat, rand_mat.T) + np.eye(self.nb_var) * 0.1
                zsigma[:, :, i] = cov
            reflist.append(ref(zmu, zsigma))

        self.test_model = model(self.nb_states, self.nb_frames, self.nb_var, reflist, [1/3]*3, 150, 20, 0.05)

    def test_symmetry_and_positive_definiteness(self):
        """Resulting Gaussians must have symmetric positive-definite covariances."""
        nb_data = 10
        pp = np.empty((self.nb_frames, nb_data), dtype=object)

        for m in range(self.nb_frames):
            # Non-orthogonal matrix to test A Sigma A^T vs A Sigma invA
            A = np.eye(self.nb_var)
            A[1, 2] = 0.3 * (m + 1)
            A[2, 1] = 0.1 * (m + 1)
            b = np.array([[0.0], [0.1 * m], [0.2 * m], [0.3 * m]])
            invA = np.linalg.inv(A)

            for t in range(nb_data):
                pp[m, t] = p(A, b, invA, self.nb_states)

        prodResList, updated_pp = computeResultingGaussians(self.test_model, pp)

        self.assertEqual(len(prodResList), nb_data)

        for t in range(nb_data):
            for i in range(self.nb_states):
                # Check frame covariances
                for m in range(self.nb_frames):
                    frame_cov = updated_pp[m, t].Sigma[:, :, i]
                    self.assertTrue(np.allclose(frame_cov, frame_cov.T, atol=1e-7), f"Frame {m} cov not symmetric at t={t}, state={i}")
                    eigvals = np.linalg.eigvalsh(frame_cov)
                    self.assertTrue(np.all(eigvals > 0), f"Frame {m} cov not positive definite at t={t}")

                # Check product covariance
                prod_cov = prodResList[t].Sigma[:, :, i]
                self.assertTrue(np.allclose(prod_cov, prod_cov.T, atol=1e-7), f"Product cov not symmetric at t={t}, state={i}")
                prod_eigvals = np.linalg.eigvalsh(prod_cov)
                self.assertTrue(np.all(prod_eigvals > 0), f"Product cov not positive definite at t={t}")
                self.assertGreater(prodResList[t].detSigma[i], 0.0)


if __name__ == "__main__":
    unittest.main()
