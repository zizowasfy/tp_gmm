import torch
import torch.nn as nn
import os

class TPGMMDeformationPolicy(nn.Module):
    def __init__(self, obs_dim=34, action_dim=15):
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
        obs: [batch_size, obs_dim] or [obs_dim]
        """
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
            
        # Apply standardization
        # Match SKRL's RunningStandardScaler denominator: sqrt(variance) + epsilon
        std = torch.sqrt(self.running_variance) + 1e-8
        obs_scaled = (obs - self.running_mean) / std
        
        # SKRL RunningStandardScaler defaults to a clip_threshold of 5.0
        # By default, the RunningStandardScaler in skrl clips normalized observations
        # to the range [-5.0, 5.0] using its clip_threshold default parameter.
        # The standalone_policy.py was not applying this clamp, meaning extreme
        # observation values might have been passed into the MLP differently than
        # they were in Isaac Lab. Updated standalone_policy.py to include
        # torch.clamp(obs_scaled, -5.0, 5.0) to match SKRL's default preprocessor behavior.
        obs_scaled = torch.clamp(obs_scaled, -5.0, 5.0)
        
        # Pass through MLP
        features = self.net(obs_scaled)
        
        # Output deterministic action (mean of the Gaussian)
        actions = self.mean_layer(features)
        
        # The environment uses an action scale to compute offsets:
        # action_deltas = actions * action_scale
        # In tpgmm_deformation_env_cfg.py, action_scale is 0.15
        # It's up to the caller to apply the scale or we can apply it here:
        return actions

    @classmethod
    def load_from_skrl_checkpoint(cls, ckpt_path, obs_dim=None, action_dim=15):
        """
        Loads the trained weights from an skrl checkpoint.
        """
        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")
            
        checkpoint = torch.load(ckpt_path, map_location="cpu")
        
        # Automatically detect observation dimension if not specified
        if obs_dim is None:
            if 'state_preprocessor' in checkpoint and checkpoint['state_preprocessor'] and 'running_mean' in checkpoint['state_preprocessor']:
                obs_dim = checkpoint['state_preprocessor']['running_mean'].shape[0]
            elif 'policy' in checkpoint:
                policy_dict = checkpoint['policy']
                net_key = 'net_container.0.weight' if 'net_container.0.weight' in policy_dict else 'net.0.weight'
                obs_dim = policy_dict[net_key].shape[1]
            else:
                obs_dim = 49
        
        model = cls(obs_dim, action_dim)
        
        # 1. Load Preprocessor
        if 'state_preprocessor' in checkpoint and checkpoint['state_preprocessor']:
            model.running_mean.copy_(checkpoint['state_preprocessor']['running_mean'])
            model.running_variance.copy_(checkpoint['state_preprocessor']['running_variance'])
            
        # 2. Load Policy Network Weights
        policy_dict = checkpoint['policy']
        
        # Map skrl keys to our standalone network
        # Find the keys dynamically to avoid KeyErrors
        net_prefix = 'net_container' if 'net_container.0.weight' in policy_dict else 'net'
        mean_prefix = 'policy_layer' if 'policy_layer.weight' in policy_dict else 'mean_layer'
        
        state_dict = {
            'net.0.weight': policy_dict[f'{net_prefix}.0.weight'],
            'net.0.bias': policy_dict[f'{net_prefix}.0.bias'],
            'net.2.weight': policy_dict[f'{net_prefix}.2.weight'],
            'net.2.bias': policy_dict[f'{net_prefix}.2.bias'],
            'mean_layer.weight': policy_dict[f'{mean_prefix}.weight'],
            'mean_layer.bias': policy_dict[f'{mean_prefix}.bias']
        }
        
        if 'state_preprocessor' in checkpoint and checkpoint['state_preprocessor']:
            state_dict['running_mean'] = checkpoint['state_preprocessor']['running_mean']
            state_dict['running_variance'] = checkpoint['state_preprocessor']['running_variance']
        else:
            state_dict['running_mean'] = model.running_mean
            state_dict['running_variance'] = model.running_variance
        
        model.load_state_dict(state_dict, strict=True)
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
        dummy_obs = torch.randn(1, policy.running_mean.shape[0])
        with torch.no_grad():
            action = policy(dummy_obs)
        print("Dummy observation output shape:", action.shape)
        
