import unittest
import sys
from pathlib import Path
import numpy as np

pkg_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(pkg_dir / "include"))

from sClass import s
from pClass import p
from modelClass import model
from init_proposedPGMM_timeBased import init_proposedPGMM_timeBased


class TestInitPGMM(unittest.TestCase):
    def test_all_points_accounted_for_including_goal(self):
        """All demonstration data points, including the final time point, must be partitioned into states."""
        nb_samples = 3
        nb_data = 31
        nb_frames = 2
        nb_var = 4
        nb_states = 5

        slist = []
        for j in range(nb_samples):
            t = np.linspace(0, 30, nb_data)
            x = np.cos(t)
            y = np.sin(t)
            z = t * 0.01
            data = np.vstack([t, x, y, z])

            pmat = np.empty((nb_frames, nb_data), dtype=object)
            for m in range(nb_frames):
                A = np.eye(nb_var)
                b = np.zeros((nb_var, 1))
                invA = np.linalg.inv(A)
                for k in range(nb_data):
                    pmat[m, k] = p(A, b, invA, nb_states)
            slist.append(s(pmat, data, nb_data, nb_states))

        init_model = model(nb_states, nb_frames, nb_var, None, None, 150, 20, 0.05)
        res_model = init_proposedPGMM_timeBased(slist, init_model)

        self.assertEqual(len(res_model.ref), nb_frames)
        self.assertAlmostEqual(sum(res_model.Priors), 1.0, places=6)

        for m in range(nb_frames):
            self.assertEqual(res_model.ref[m].ZMu.shape, (nb_var, nb_states))
            self.assertEqual(res_model.ref[m].ZSigma.shape, (nb_var, nb_var, nb_states))
            for i in range(nb_states):
                cov = res_model.ref[m].ZSigma[:, :, i]
                self.assertTrue(np.allclose(cov, cov.T), f"Covariance not symmetric for frame {m}, state {i}")
                eigvals = np.linalg.eigvalsh(cov)
                self.assertTrue(np.all(eigvals > 0), f"Covariance not positive-definite for frame {m}, state {i}")


if __name__ == "__main__":
    unittest.main()
