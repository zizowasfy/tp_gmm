import unittest
import sys
import os
import pickle
from pathlib import Path
import numpy as np
import torch

pkg_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(pkg_dir / "scripts"))

from standalone_policy import TPGMMDeformationPolicy


class TestPolicySync(unittest.TestCase):
    def setUp(self):
        self.recorded_path = Path("/home/zizo/the_folder/Reach_direct/recorded_experiments.pkl")
        self.ckpt_path = Path(
            "/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-08-26_21-56-10_ppo_torch_nocovariance/checkpoints/best_agent.pt"
        )
        self.latest_ckpt_path = Path(
            "/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-01_01-05-26_ppo_torch/checkpoints/best_agent.pt"
        )

    def test_default_architecture_dims(self):
        """Default policy architecture must use obs_dim=34 and action_dim=15."""
        model = TPGMMDeformationPolicy()
        self.assertEqual(model.running_mean.shape[0], 34)
        self.assertEqual(model.net[0].in_features, 34)
        self.assertEqual(model.mean_layer.out_features, 15)

    def test_checkpoint_load_and_obs_dim(self):
        """Checkpoint loading should load 34-dim weights and preprocess correctly."""
        if not self.ckpt_path.exists():
            self.skipTest(f"Checkpoint not found: {self.ckpt_path}")
        policy = TPGMMDeformationPolicy.load_from_skrl_checkpoint(str(self.ckpt_path))
        self.assertEqual(policy.net[0].in_features, 34)

        dummy = torch.randn(1, 34)
        with torch.no_grad():
            out = policy(dummy)
        self.assertEqual(out.shape, (1, 15))

    def test_match_recorded_experiments_action_and_scale(self):
        """Policy output and action_scale=0.15 must match ground truth recorded experiments."""
        if not self.recorded_path.exists() or not self.ckpt_path.exists():
            self.skipTest("Recorded data or checkpoint not available.")

        with open(self.recorded_path, "rb") as f:
            episodes = pickle.load(f)

        policy = TPGMMDeformationPolicy.load_from_skrl_checkpoint(str(self.ckpt_path))

        step0 = episodes[0]["steps"][0]
        rec_obs = step0["observation"]
        rec_act = step0["action"]
        rec_def_mu = step0["deformed_mu"]
        rec_orig_mu = step0["original_mu"]

        obs_tensor = torch.tensor(rec_obs, dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            pred_act = policy(obs_tensor).squeeze(0).cpu().numpy()

        action_diff = np.linalg.norm(pred_act - rec_act)
        self.assertLess(action_diff, 1e-5, f"Action diff too large: {action_diff}")

        action_scale = 0.15
        pred_def_mu = rec_orig_mu + pred_act.reshape(-1, 3) * action_scale
        mu_diff = np.linalg.norm(pred_def_mu - rec_def_mu)
        self.assertLess(mu_diff, 1e-5, f"Deformed Mu diff with scale 0.15 too large: {mu_diff}")


if __name__ == "__main__":
    unittest.main()
