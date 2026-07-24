import torch
import torch.nn as nn
import os

class TPGMMDeformationPolicy(nn.Module):
    def __init__(self, obs_dim=48, action_dim=15):
        super().__init__()
        # State preprocessor parameters (RunningStandardScaler from skrl)
        self.register_buffer("running_mean", torch.zeros(obs_dim))
        self.register_buffer("running_variance", torch.ones(obs_dim))
        
        # Policy MLP (defaults in skrl_ppo_cfg.yaml: [32, 32] with ELU)
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 32),
            nn.ELU(),
            nn.Linear(32, 32),
            nn.ELU()
        )
        # Final output layer for the mean actions
        self.mean_layer = nn.Linear(32, action_dim)
        
    def forward(self, obs):
        """
        Forward pass for inference. 
        Takes unnormalized observations and outputs the deterministic actions.
        obs: [batch_size, 48] or [48]
        """
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
            
        # Apply standardization
        std = torch.sqrt(self.running_variance + 1e-8)
        obs_scaled = (obs - self.running_mean) / std
        
        # Pass through MLP
        features = self.net(obs_scaled)
        
        # Output deterministic action (mean of the Gaussian)
        actions = self.mean_layer(features)
        
        # The environment uses an action scale to compute offsets:
        # action_deltas = actions * action_scale
        # In tpgmm_deformation_env_cfg.py, action_scale is 0.1
        # It's up to the caller to apply the scale or we can apply it here:
        return actions

    @classmethod
    def load_from_skrl_checkpoint(cls, ckpt_path, obs_dim=48, action_dim=15):
        """
        Loads the trained weights from an skrl checkpoint.
        """
        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")
            
        checkpoint = torch.load(ckpt_path, map_location="cpu")
        
        model = cls(obs_dim, action_dim)
        
        # 1. Load Preprocessor
        if 'state_preprocessor' in checkpoint:
            model.running_mean.copy_(checkpoint['state_preprocessor']['running_mean'])
            model.running_variance.copy_(checkpoint['state_preprocessor']['running_variance'])
            
        # 2. Load Policy Network Weights
        policy_dict = checkpoint['policy']
        
        # Map skrl keys to our standalone network
        # net_container.0 -> net[0] (Linear 48->32)
        # net_container.2 -> net[2] (Linear 32->32)
        # policy_layer -> mean_layer (Linear 32->15)
        
        state_dict = {
            'net.0.weight': policy_dict['net_container.0.weight'],
            'net.0.bias': policy_dict['net_container.0.bias'],
            'net.2.weight': policy_dict['net_container.2.weight'],
            'net.2.bias': policy_dict['net_container.2.bias'],
            'mean_layer.weight': policy_dict['policy_layer.weight'],
            'mean_layer.bias': policy_dict['policy_layer.bias']
        }
        
        model.load_state_dict(state_dict, strict=False)
        model.eval()
        return model

# Example usage/test 
# conda run -n env_isaaclab python tp_gmm/scripts/standalone_policy.py /home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-06-04_16-49-13_ppo_torch_envs=32/checkpoints/agent_4800.pt
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        ckpt = sys.argv[1]
        policy = TPGMMDeformationPolicy.load_from_skrl_checkpoint(ckpt)
        print("Successfully loaded standalone policy!")
        
        # Dummy test
        dummy_obs = torch.randn(1, 48)
        with torch.no_grad():
            action = policy(dummy_obs)
        print("Dummy observation output shape:", action.shape)
        
