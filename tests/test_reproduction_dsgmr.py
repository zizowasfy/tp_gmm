import unittest
import sys
from pathlib import Path
import numpy as np

pkg_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(pkg_dir / "include"))

from modelClass import model
from refClass import ref
from pClass import p
from reproduction_DSGMR import reproduction_DSGMR, recompute_DSGMR


class TestReproductionDSGMR(unittest.TestCase):
    def setUp(self):
        self.nb_var = 4
        self.nb_states = 3
        self.nb_frames = 2

        reflist = []
        for m in range(self.nb_frames):
            zmu = np.zeros((self.nb_var, self.nb_states))
            zmu[0, :] = [0.0, 15.0, 30.0]
            zmu[1, :] = [0.1 * m, 0.2, 0.3 * m]
            zmu[2, :] = [0.3, 0.2 * m, 0.1]
            zmu[3, :] = [0.2, 0.3, 0.4]

            zsigma = np.zeros((self.nb_var, self.nb_var, self.nb_states))
            for i in range(self.nb_states):
                zsigma[:, :, i] = np.diag([1.0, 0.05, 0.05, 0.05])
            reflist.append(ref(zmu, zsigma))

        self.test_model = model(self.nb_states, self.nb_frames, self.nb_var, reflist, [1/3]*3, 150, 20, 0.05)

    def test_single_frame_step_no_index_error(self):
        """When len(prodRes) == 1 (pp has shape 1 along axis 1), reproduction must not raise IndexError."""
        data_in = np.array([0.0])
        curr_pos = np.array([0.1, 0.2, 0.3])

        pp = np.empty((self.nb_frames, 1), dtype=object)
        for m in range(self.nb_frames):
            A = np.eye(self.nb_var)
            b = np.zeros((self.nb_var, 1))
            invA = np.linalg.inv(A)
            pp[m, 0] = p(A, b, invA, self.nb_states)

        result = reproduction_DSGMR(data_in, self.test_model, pp, curr_pos)
        self.assertIsNotNone(result)
        self.assertEqual(result.Data.shape, (4, 1))
        self.assertFalse(np.isnan(result.Data).any())

    def test_multi_step_trajectory_generation(self):
        """Reproduction generates smooth Cartesian trajectory without NaNs."""
        nb_data = 20
        data_in = np.linspace(0, 30, nb_data)
        curr_pos = np.array([0.1, 0.3, 0.2])

        pp = np.empty((self.nb_frames, nb_data), dtype=object)
        for m in range(self.nb_frames):
            A = np.eye(self.nb_var)
            b = np.zeros((self.nb_var, 1))
            invA = np.linalg.inv(A)
            for t in range(nb_data):
                pp[m, t] = p(A, b, invA, self.nb_states)

        rnew = reproduction_DSGMR(data_in, self.test_model, pp, curr_pos)
        self.assertEqual(rnew.Data.shape, (4, nb_data))
        self.assertFalse(np.isnan(rnew.Data).any())

        # Test deformation recompute
        rnew.Mu[1:4, :, :] += 0.05
        r_recomputed = recompute_DSGMR(rnew, self.test_model, curr_pos)
        self.assertFalse(np.isnan(r_recomputed.Data).any())


if __name__ == "__main__":
    unittest.main()
