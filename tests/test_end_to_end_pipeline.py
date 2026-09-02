import unittest
import sys
from pathlib import Path
import numpy as np
import torch
from copy import deepcopy

pkg_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(pkg_dir / "include"))
sys.path.append(str(pkg_dir / "scripts"))

from demons_to_samples import process_demonstrations
from TPGMM_GMR import TPGMM_GMR
from standalone_policy import TPGMMDeformationPolicy


class TestEndToEndPipeline(unittest.TestCase):
    def test_full_pipeline_pick(self):
        """End-to-end test: process demonstrations -> fit EM -> reproduce -> RL deform -> recompute."""
        demons_dir = Path("/home/zizo/the_folder/Trajectory_Data_Collection/Demons/pick")
        if not demons_dir.exists():
            self.skipTest("Demonstrations directory for 'pick' not found.")

        # Step 1: Preprocess demonstrations
        ref_demon, slist = process_demonstrations("pick", nbFrames=2, nbStates=5, nbVar=4)
        self.assertGreater(len(slist), 0)

        # Step 2: Fit model with EM
        tpgmm = TPGMM_GMR(nbStates=5, nbFrames=2, nbVar=4)
        tpgmm.fit(slist)
        self.assertAlmostEqual(sum(tpgmm.model.Priors), 1.0, places=5)

        # Step 3: Reproduce trajectory
        start_pos = np.array([0.35, -0.15, 0.45])
        goal_pos = np.array([0.55, 0.20, 0.15])

        ref_idx = ref_demon["demons_nums"].index(ref_demon["ref"])
        newP = deepcopy(tpgmm.s[ref_idx].p)

        b0 = np.array([[0], [start_pos[0]], [start_pos[1]], [start_pos[2]]])
        A0 = np.eye(4)
        newP[0, 0].b = b0
        newP[0, 0].A = A0
        newP[0, 0].invA = np.linalg.inv(A0)

        b1 = np.array([[0], [goal_pos[0]], [goal_pos[1]], [goal_pos[2]]])
        A1 = np.eye(4)
        newP[1, 0].b = b1
        newP[1, 0].A = A1
        newP[1, 0].invA = np.linalg.inv(A1)

        newPP = np.tile(newP[:, 0][:, np.newaxis], newP.shape[1])
        rnew = tpgmm.reproduce(newPP, start_pos)

        self.assertEqual(rnew.Data.shape[0], 4)
        self.assertFalse(np.isnan(rnew.Data).any())

        # Step 4: Deform GMM via RL Policy
        ckpt_path = Path(
            "/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-26_21-56-10_ppo_torch_nocovariance/checkpoints/best_agent.pt"
        )
        if not ckpt_path.exists():
            return

        policy = TPGMMDeformationPolicy.load_from_skrl_checkpoint(str(ckpt_path))
        policy.eval()

        original_mu = torch.zeros((1, 5, 3))
        for k in range(5):
            original_mu[0, k, 0] = rnew.Mu[1, k, -1]
            original_mu[0, k, 1] = rnew.Mu[2, k, -1]
            original_mu[0, k, 2] = rnew.Mu[3, k, -1]

        gmm_features = original_mu.reshape(1, -1)
        sp_tensor = torch.tensor([[start_pos[0], start_pos[1], start_pos[2], 1.0, 0.0, 0.0, 0.0]], dtype=torch.float32)
        gp_tensor = torch.tensor([[goal_pos[0], goal_pos[1], goal_pos[2], 1.0, 0.0, 0.0, 0.0]], dtype=torch.float32)
        op_tensor = torch.tensor([[0.45, 0.02, 0.25]], dtype=torch.float32)
        orad = torch.tensor([[0.05]], dtype=torch.float32)
        clearance = torch.tensor([[0.8]], dtype=torch.float32)

        obs = torch.cat((gmm_features, sp_tensor, gp_tensor, op_tensor, orad, clearance), dim=-1)
        self.assertEqual(obs.shape[-1], 34)

        with torch.no_grad():
            action = policy(obs)

        action_scale = 0.15
        action_deltas = action.view(1, 5, 3) * action_scale
        deformed_mu = original_mu + action_deltas

        for k in range(5):
            rnew.Mu[1, k, :] = deformed_mu[0, k, 0].item()
            rnew.Mu[2, k, :] = deformed_mu[0, k, 1].item()
            rnew.Mu[3, k, :] = deformed_mu[0, k, 2].item()

        rdeformed = tpgmm.recompute_trajectory(rnew, start_pos)
        self.assertEqual(rdeformed.Data.shape, rnew.Data.shape)
        self.assertFalse(np.isnan(rdeformed.Data).any())


if __name__ == "__main__":
    unittest.main()
