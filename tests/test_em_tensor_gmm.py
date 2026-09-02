import unittest
import sys
from pathlib import Path
import numpy as np
from copy import deepcopy

# Add include to sys.path
pkg_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(pkg_dir / "include"))

from sClass import s
from pClass import p
from modelClass import model
from refClass import ref
from init_proposedPGMM_timeBased import init_proposedPGMM_timeBased
from EM_tensorGMM import EM_tensorGMM


def create_synthetic_demonstrations(nb_samples=3, nb_data=25, nb_frames=2, nb_var=4, nb_states=3):
    slist = []
    for j in range(nb_samples):
        # Time row: 0, 1, 2, ...
        t = np.linspace(0, nb_data - 1, nb_data)
        # 3D trajectory
        x = np.sin(t / 5.0) + j * 0.05
        y = np.cos(t / 5.0) - j * 0.05
        z = t * 0.02 + 0.1
        data = np.vstack([t, x, y, z])

        pmat = np.empty((nb_frames, nb_data), dtype=object)
        for m in range(nb_frames):
            A = np.eye(nb_var)
            b = np.zeros((nb_var, 1))
            b[1:] = np.array([[0.1 * m], [0.2 * m], [0.05 * m]])
            invA = np.linalg.inv(A)
            for k in range(nb_data):
                pmat[m, k] = p(A, b, invA, nb_states)
        slist.append(s(pmat, data, nb_data, nb_states))

    init_model = model(nb_states, nb_frames, nb_var, None, None, 150, 20, 0.05)
    init_model = init_proposedPGMM_timeBased(slist, init_model)
    return slist, init_model


class TestEMTensorGMM(unittest.TestCase):
    def test_single_demonstration_no_crash(self):
        """EM should not crash when given fewer demonstrations than frames (e.g. 1 sample, 2 frames)."""
        slist, init_model = create_synthetic_demonstrations(nb_samples=1, nb_frames=2)
        trained_model = EM_tensorGMM(slist, deepcopy(init_model))
        self.assertIsNotNone(trained_model)
        self.assertEqual(len(trained_model.ref), 2)

    def test_parameters_updated(self):
        """EM must update ZMu and ZSigma away from the initial heuristic values."""
        slist, init_model = create_synthetic_demonstrations(nb_samples=4, nb_frames=2)
        initial_zmu = [ref_m.ZMu.copy() for ref_m in init_model.ref]
        trained_model = EM_tensorGMM(slist, deepcopy(init_model))

        for m in range(trained_model.nbFrames):
            diff = np.max(np.abs(trained_model.ref[m].ZMu - initial_zmu[m]))
            self.assertGreater(diff, 1e-4, f"Frame {m} ZMu was not updated by EM!")

    def test_priors_normalized(self):
        """Priors must sum to 1.0 after EM."""
        slist, init_model = create_synthetic_demonstrations(nb_samples=3, nb_frames=2)
        trained_model = EM_tensorGMM(slist, deepcopy(init_model))
        self.assertAlmostEqual(sum(trained_model.Priors), 1.0, places=5)


if __name__ == "__main__":
    unittest.main()
