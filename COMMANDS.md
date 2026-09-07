# TP-GMM & MoveIt 2 Pipeline Execution & Testing Guide

This document provides a comprehensive command reference for building, launching, benchmarking, and testing all components of the Task-Parameterized Gaussian Mixture Model (TP-GMM) motion planning pipeline, RL deformation policy, and MoveIt 2 constraint samplers.

---

## Table of Contents
1. [Workspace Setup & Build Commands](#1-workspace-setup--build-commands)
2. [Robot Simulation & MoveIt Bringup](#2-robot-simulation--moveit-bringup)
3. [TP-GMM Core Nodes & Services](#3-tp-gmm-core-nodes--services)
4. [Demonstration Data Collection & Visualization](#4-demonstration-data-collection--visualization)
5. [Benchmarking Sampling Approaches](#5-benchmarking-sampling-approaches)
6. [Automated Gazebo Experiment Runner](#6-automated-gazebo-experiment-runner)
7. [Policy Validation](#7-policy-validation)
8. [Automated Test Suite (pytest & colcon)](#8-automated-test-suite-pytest--colcon)
9. [ROS 2 Topic & Service Debugging Cheat Sheet](#9-ros-2-topic--service-debugging-cheat-sheet)

---

## 1. Workspace Setup & Build Commands

Always source your ROS 2 distribution underlay and the workspace overlay before running commands:

```bash
# Source ROS 2 Jazzy underlay
source /opt/ros/jazzy/setup.bash

# Navigate to workspace base directory
cd /home/zizo/the_folder/ws_moveit

# Source workspace install overlay
source install/setup.bash
```

### Build Commands (`colcon`)

```bash
# Build only the tp_gmm package with symlink install (fast iteration)
colcon build --packages-select tp_gmm --symlink-install

# Build tp_gmm and MoveIt core/planning context packages
colcon build --packages-select tp_gmm moveit_planners_ompl moveit_core --symlink-install

# Build all packages in the workspace
colcon build --symlink-install
```

---

## 2. Robot Simulation & MoveIt Bringup

Run the robot driver or simulation in a dedicated terminal before starting planning nodes:

### Franka Emika Panda in Gazebo Sim (Full Physics & Warehouse World)
This is the primary Gazebo simulation launch file needed by `run_experiments.py` and simulation benchmarks. It starts Gazebo Sim (`ros_gz_sim`) with the `warehouse.world`, spawns the Franka Panda robot arm, starts Gazebo ros2_control hardware controllers, and runs MoveIt's `move_group` node with RViz:

```bash
# Launch Panda in Gazebo Sim (Warehouse world + ros2_control + MoveIt + RViz)
ros2 launch moveit_resources_panda_moveit_config gazebo_sim.launch.py

# Optional: Launch with a custom Gazebo world file
ros2 launch moveit_resources_panda_moveit_config gazebo_sim.launch.py \
  world:=/path/to/custom.world
```

### Franka Emika Panda Mock Hardware (Fast Planning without Gazebo physics)
```bash
# Launch Panda with mock hardware controllers and RViz
ros2 launch moveit_resources_panda_moveit_config demo.launch.py

# Alternatively, launch through moveit2_tutorials
ros2 launch moveit2_tutorials demo.launch.py
```

### Kinova Gen 3 (7-DOF)
```bash
# Bringup Kinova Gen 3 with mock hardware in RViz
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=192.168.1.10 \
  use_fake_hardware:=true

# Bringup physical Kinova Gen 3 arm
ros2 launch kortex_bringup gen3.launch.py \
  robot_ip:=192.168.1.10
```

---

## 3. TP-GMM Core Nodes & Services

### Launch Full TP-GMM & RViz Pipeline
Launches `tp_gmm_node.py`, GMM RViz visualizer converters, and MoveIt constraint bridge nodes:

```bash
# Default launch (task: Rbolts, subtask: action)
ros2 launch tp_gmm lfd_launch.py

# Launch with custom task name (e.g. franka_pick_cube or pick)
ros2 launch tp_gmm lfd_launch.py task:=franka_pick_cube subtask:=action
```

### Run Core Node Standalone
```bash
ros2 run tp_gmm tp_gmm_node.py
```

### Service Calls

#### A. Train / Fit TP-GMM Model (`/StartTPGMM_service`)
Extracts raw demonstrations from rosbags, executes Dynamic Time Warping (DTW) alignment, and fits the task-parameterized GMM using Expectation-Maximization:

```bash
# Fit model for franka_pick_cube
ros2 service call /StartTPGMM_service tp_gmm/srv/StartTPGMM "{task_name: 'franka_pick_cube'}"

# Fit model for legacy 'pick' task
ros2 service call /StartTPGMM_service tp_gmm/srv/StartTPGMM "{task_name: 'pick'}"
```
*Trained models are saved directly to `tasks/<task_name>/TPGMM_model.pkl`.*

#### B. Reproduce Trajectory (`/ReproduceTPGMM_service`)
Adapts the learned model to new start and goal frame configurations and outputs the nominal GMM corridor:

```bash
ros2 service call /ReproduceTPGMM_service tp_gmm/srv/ReproduceTPGMM "{
  task_name: 'franka_pick_cube',
  start_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.35, y: -0.15, z: 0.45}, orientation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}
  },
  goal_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.55, y: 0.20, z: 0.15}, orientation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}
  }
}"
```

#### C. Deform GMM around Obstacle via RL Policy (`/DeformTPGMM_service`)
Deforms the GMM Gaussians away from an obstacle using the trained SKRL PPO neural policy:

```bash
ros2 service call /DeformTPGMM_service tp_gmm/srv/DeformTPGMM "{
  task_name: 'franka_pick_cube',
  start_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.35, y: -0.15, z: 0.45}, orientation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}
  },
  goal_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.55, y: 0.20, z: 0.15}, orientation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}
  },
  obstacle_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.45, y: 0.02, z: 0.25}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}
  },
  obstacle_radius: 0.05,
  desired_clearance: 0.8
}"
```

---

## 4. Demonstration Data Collection & Visualization

### Collect Demonstrations
Records end-effector Cartesian trajectories and frame positions into ROS 2 bags:

```bash
# Collect Franka Panda demonstrations
ros2 run tp_gmm collect_trajectories.py

# Collect Kinova Gen 3 demonstrations
ros2 run tp_gmm collect_trajectories_gen3_ros2.py

# Using the launch file
ros2 launch tp_gmm collect_demons_launch.py
```

### Visualize Demonstrations in RViz
Publishes demonstration trajectories before and after DTW alignment, with reference trajectory highlighting:

```bash
# Visualize all demonstrations for franka_pick_cube
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube

# Inspect a specific demonstration (e.g. demon_3)
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube --demon demon_3

# Automatically cycle through individual demonstrations every 2.0 seconds
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube --cycle 2.0

# Specify custom publishing rate and frame ID
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube --rate 2.0 --frame-id panda_link0
```

---

## 5. Benchmarking Sampling Approaches

The script `compare_sampling_approaches.py` evaluates sampling strategies under identical test conditions:
1. `default_unconstrained`: Default OMPL uniform workspace sampler (no constraints).
2. `uniform_box`: Baseline bounding box corridor sampler with standard IK.
3. `cartesian_ik`: Cartesian GMM sampling with warm-started IK.
4. `joint_projected`: Direct joint-space GMM projection with zero runtime IK.

### Command Variations:

```bash
# 1. Standard benchmark across all 4 modes (5 trials, 0.5 clearance factor)
ros2 run tp_gmm compare_sampling_approaches.py --trials 5 --clearance 0.5

# 2. Step-by-Step Interactive Mode (pauses after each mode plan until [Enter] is pressed)
# Allows inspecting each plan and its visual samples in RViz without rushing
ros2 run tp_gmm compare_sampling_approaches.py --trials 3 --step

# 3. Trajectory Replay Mode (replays all planned mode trajectories sequentially at the end of each trial)
ros2 run tp_gmm compare_sampling_approaches.py --trials 3 --replay --delay 2.5

# 4. Compare specific modes only (e.g. Joint Projected vs Default Unconstrained)
ros2 run tp_gmm compare_sampling_approaches.py --trials 10 \
  --modes joint_projected default_unconstrained

# 5. Fast benchmark without RViz delay
ros2 run tp_gmm compare_sampling_approaches.py --trials 20 --delay 0.0
```

### Benchmark Arguments:
| Flag | Short | Default | Description |
|---|---|---|---|
| `--trials` | `-n` | `5` | Number of randomized benchmark test scenarios |
| `--clearance` | `-c` | `0.5` | Obstacle clearance scale factor $[0.0 - 1.0]$ |
| `--modes` | | all 4 | Subset of modes to test (`default_unconstrained`, `uniform_box`, `cartesian_ik`, `joint_projected`) |
| `--delay` | `-d` | `2.0` | Seconds to pause and view each planned path in RViz |
| `--step` | | `False` | Interactive pause after each mode until [Enter] is pressed |
| `--replay` | | `False` | Sequential replay animation of all successful modes per trial |

*Results and statistical summaries are saved to `tp_gmm/data/sampling_comparison_results.json`.*

---

## 6. Automated Gazebo Experiment Runner

The script `run_experiments.py` runs full pick-and-place experiments in Gazebo Sim (resetting object positions, applying GMM deformation, planning around the obstacle, and grasping).

> **Prerequisite**: Start the Gazebo environment in a separate terminal first:
> ```bash
> ros2 launch moveit_resources_panda_moveit_config gazebo_sim.launch.py
> ```

```bash
# Automated batch run with Franka Panda (10 trials, Joint Projected mode)
ros2 run tp_gmm run_experiments.py --trials 10 --task franka_pick_cube --sampling-mode joint_projected --clearance 0.5

# Run in manual mode (press [Enter] between trials)
ros2 run tp_gmm run_experiments.py --trials 5 --manual --sampling-mode cartesian_ik

# Test with Default Unconstrained OMPL sampler (no corridor constraints)
ros2 run tp_gmm run_experiments.py --trials 10 --sampling-mode default_unconstrained

# Test with Uniform Box baseline sampler
ros2 run tp_gmm run_experiments.py --trials 10 --sampling-mode uniform_box
```

### Experiment Arguments:
| Flag | Short | Default | Description |
|---|---|---|---|
| `--trials` | `-n` | `10` | Total number of experiment trials |
| `--task` | `-t` | `franka_pick_cube` | Task directory name |
| `--sampling-mode` | `-s` | `cartesian_ik` | Sampling approach (`cartesian_ik`, `joint_projected`, `uniform_box`, `default_unconstrained`) |
| `--clearance` | `-c` | `0.5` | Target obstacle clearance factor |
| `--auto` | | `True` | Run trials automatically without pausing |
| `--manual` | | | Pause and prompt before each trial |

---

## 7. Policy Validation

Validates the standalone PyTorch deformation network policy against recorded IsaacLab RL experiment datasets:

```bash
# Validate policy for Franka Panda
ros2 run tp_gmm validate_policy.py --robot franka_panda --task franka_pick_cube

# Validate policy for Kinova Gen 3
ros2 run tp_gmm validate_policy.py --robot kinova_gen3 --task pick

# Validate policy for UR10
ros2 run tp_gmm validate_policy.py --robot ur10 --task pick
```

---

## 8. Automated Test Suite (pytest & colcon)

### Run Entire Test Suite via `pytest`
```bash
# Run all tests in tp_gmm/tests
cd /home/zizo/the_folder/ws_moveit/src/tp_gmm
pytest tests/ -v
```

### Run Individual Test Files
```bash
# 1. End-to-End Pipeline test (extract -> EM fit -> reproduction -> RL deform -> recompute)
pytest tests/test_end_to_end_pipeline.py -v

# 2. Neural Policy Checkpoint & IsaacLab Sync test
pytest tests/test_policy_sync.py -v

# 3. Expectation-Maximization Tensor GMM test
pytest tests/test_em_tensor_gmm.py -v

# 4. Trajectory Dynamic Time Warping & Sample Processing test
pytest tests/test_demons_to_samples.py -v

# 5. Gaussian Mixture Regression (DSGMR) reproduction test
pytest tests/test_reproduction_dsgmr.py -v

# 6. GMM initial parameter clustering test
pytest tests/test_init_pgmm.py -v

# 7. RViz SolidPrimitive Marker filtering test
pytest tests/test_marker_filtering.py -v

# 8. Demonstration Visualizer node test
pytest tests/test_visualize_demonstrations.py -v
```

### Run Tests via `colcon test`
```bash
cd /home/zizo/the_folder/ws_moveit
colcon test --packages-select tp_gmm --event-handlers console_direct+
colcon test-result --verbose
```

---

## 9. ROS 2 Topic & Service Debugging Cheat Sheet

### Sampling & Planning Visualization Topics
| Topic | Type | Description |
|---|---|---|
| `/gmm_sampling_visualization` | `visualization_msgs/msg/Marker` | Real-time sample spheres (Cyan/Green: Accepted, Magenta/Red: Rejected) |
| `/planning_sampling_stats` | `std_msgs/msg/String` | Real-time JSON stats: `samples_drawn`, `samples_accepted`, `sampling_rate` |
| `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | Trajectory animation displayed in RViz |
| `/gmm/cartesian_space` | `tp_gmm/msg/GaussianMixture` | Original reproduced GMM Gaussians |
| `/gmm/deformed_cartesian_space` | `tp_gmm/msg/GaussianMixture` | RL policy deformed GMM Gaussians |
| `/gmm_moveit` | `moveit_msgs/msg/BoundingVolume` | MoveIt constraint volume for nominal corridor |
| `/deformed_gmm_moveit` | `moveit_msgs/msg/BoundingVolume` | MoveIt constraint volume for deformed corridor |

### Useful Monitoring CLI Commands
```bash
# Echo sampling statistics in real-time
ros2 topic echo /planning_sampling_stats

# Echo deformed GMM Gaussians
ros2 topic echo /gmm/deformed_cartesian_space

# Check GMM constraint sampler allocator logs
ros2 topic echo /rosout | grep -E "(GMMConstraintSampler|Allocating specialized)"

# Verify active services
ros2 service list | grep -E "(TPGMM|planning)"
```
