# TP-GMM & MoveIt 2 — Codex Branch Command Guide

Command reference for **`sampling-approaches-cod`**: building, running Cartesian + IK and joint-projected GMM sampling, inspecting samples in RViz, comparing approaches, sweeping RL clearance, and testing the pipeline.

The changed repositories are `tp_gmm`, `moveit2`, and `moveit_resources`. The validated robot is the fixed-base Franka Panda (`panda_arm` / `panda_hand`). For implementation details and mathematics, see [SAMPLING.md](/home/zizo/the_folder/ws_moveit/src/tp_gmm/SAMPLING.md). The separate [COMMANDS-gem.md](/home/zizo/the_folder/ws_moveit/src/COMMANDS-gem.md) describes the Gemini implementation.

## Table of Contents

1. [Workspace Setup & Build](#1-workspace-setup--build)
2. [Plan-Only Panda Laboratory](#2-plan-only-panda-laboratory)
3. [RViz Sampling Inspection](#3-rviz-sampling-inspection)
4. [Paired Sampling Benchmark](#4-paired-sampling-benchmark)
5. [Sampling Configuration & Live Switching](#5-sampling-configuration--live-switching)
6. [Gazebo Experiment Pipeline](#6-gazebo-experiment-pipeline)
7. [TP-GMM Nodes & Service Calls](#7-tp-gmm-nodes--service-calls)
8. [Demonstrations & Policy Validation](#8-demonstrations--policy-validation)
9. [Automated Tests](#9-automated-tests)
10. [ROS Diagnostics & Troubleshooting](#10-ros-diagnostics--troubleshooting)
11. [RL Clearance Generalization Sweeps](#11-rl-clearance-generalization-sweeps)
12. [GMR Reference-Path Proposals](#gmr-reference-path-proposals-cartesian-ik)
13. [Strict Pure-Sampler Case Study](#strict-pure-sampler-statistical-case-study)
14. [Clearance-Only Replay](#clearance-only-replay-05--07)
15. [Training Goal Height and Lower Table](#training-goal-height-range-with-a-separate-lower-table-scene)
16. [Covariance Cutoff 3.0 on the Original Scene](#covariance-cutoff-30-on-the-original-scene)
17. [Journal Sampler Comparison](#journal-sampler-comparison-at-fixed-cutoff-30)

---

## 1. Workspace Setup & Build

### Environment — run in every terminal

All subsequent commands assume the workspace root as the working directory.

```bash
cd /home/zizo/the_folder/ws_moveit
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Use the same domain in every terminal for this experiment.
# 79 is the isolated domain used for the Codex validation runs.
export ROS_DOMAIN_ID=79

# Current policy retrained for the GMM with regularization 0.005.
export TPGMM_POLICY_CKPT=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt
```

Choose one robot bringup per ROS domain: either the plan-only laboratory or the Gazebo/mock-hardware workflow. They provide overlapping node and service names.

### Check / select the Codex branches

```bash
for repo in tp_gmm moveit2 moveit_resources; do
  git -C "src/$repo" status --short --branch
done

# If switching from another implementation, first save any outstanding edits.
git -C src/tp_gmm switch sampling-approaches-cod
git -C src/moveit2 switch sampling-approaches-cod
git -C src/moveit_resources switch sampling-approaches-cod
```

### Build the implementation and its integration packages

```bash
colcon build \
  --packages-select moveit_core moveit_ros_planning moveit_planners_ompl tp_gmm moveit_resources_panda_moveit_config \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For subsequent changes confined to `tp_gmm`:

```bash
colcon build --packages-select tp_gmm --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Restart MoveIt and TP-GMM after switching branches and rebuilding. The branches share `build/` and `install/`; switching Git branches alone does not change the installed sampler library or service definitions.

---

## 2. Plan-Only Panda Laboratory

This is the quickest setup for comparing samplers. It starts MoveIt, a Panda robot-state publisher, a joint-state publisher, TP-GMM/RL, and RViz. It has **no robot controllers and disables trajectory execution**.

### Terminal 1 — bringup

After the environment setup in Section 1:

```bash
ros2 launch tp_gmm sampling_demo.launch.py \
  policy_ckpt_path:="$TPGMM_POLICY_CKPT"
```

For a headless timing run:

```bash
ros2 launch tp_gmm sampling_demo.launch.py \
  rviz:=false \
  policy_ckpt_path:="$TPGMM_POLICY_CKPT"
```

If a TP-GMM node is already running on this domain, disable the laboratory's copy:

```bash
ros2 launch tp_gmm sampling_demo.launch.py lfd:=false
```

| Launch argument | Default | Meaning |
|---|---|---|
| `rviz` | `true` | Start RViz with `sampling.rviz` |
| `lfd` | `true` | Start `tp_gmm_node.py` |
| `policy_ckpt_path` | empty | RL checkpoint for the laboratory's TP-GMM node |

An empty/missing checkpoint leaves the policy unloaded. The benchmark requires a loaded policy unless `--allow-no-policy` is explicitly supplied for an undeformed-model smoke test.

### Terminal 2 — quick visual run

After the same environment setup:

```bash
ros2 run tp_gmm compare_sampling_approaches.py \
  --repeats 1 --warmup 0 --planning-time 3 --visualize \
  --output sampling_results/cod-visual
```

This runs three cases across three modes: nine plan-only requests. No Gazebo instance is needed.

---

## 3. RViz Sampling Inspection

The supplied RViz configuration enables these two MarkerArray displays:

| Display topic | Content |
|---|---|
| `/gmm_sampling/markers` | Request-specific proposals, valid/rejected FK samples, and GMM support ellipsoids |
| `/gmm_sampling/paths` | Latest returned end-effector path for each sampling mode |

The default region uses `corridor_mode: covariance` and `cutoff: 2.0`. Both GMM converters and `/gmm_sampling/markers` show ellipsoids with full diameters `4 × sqrt(covariance eigenvalue)`. Component weights affect sampling frequency, not marker size. Both sampling approaches and the uniform baseline use matching enclosing box constraints.

The retrained model's covariance is read directly from its GaussianMixture messages. The training regularization (`diagRegularizationFactor = 0.005`) is already incorporated in the model; do not apply it again as a visualization scale or covariance floor. The numerical covariance floor remains `1e-8` m².

Restart the TP-GMM/converter launch and runner after sourcing `install/setup.bash`. These are now the launch defaults, so this explicit command is optional:

```bash
ros2 launch tp_gmm lfd_launch.py gmm_legacy_weighted_scale:=false gmm_cutoff:=2.0
```

The runner defaults and `config/sampling.json` use the same covariance/2.0 settings. An explicit `--sampler-config` overrides those defaults. The execution launch uses its `policy_ckpt_path` entry; pass the intended checkpoint explicitly for standalone or laboratory launches.

For custom historical scaling, match JSON `corridor_scale` with launch argument `gmm_corridor_scale`; match `covariance_floor` with `gmm_covariance_floor` in either mode. Leave converter `normalize` false.

The hard path region is a union of enclosing boxes, as in the old pipeline. Box corners and rejected red FK samples can be outside ellipsoids. Cyan proposals stay within their component ellipsoid. Isolate the current request namespace when comparing with the latest deformed model.

### Sample colors

| Color | Meaning |
|---|---|
| Cyan | Cartesian proposals; in projected mode these are the linear task-space draws before FK |
| Green | Valid FK samples returned by the custom GMM sampler |
| Red | FK samples rejected by projection/validity checks |
| Yellow | Valid samples from the standard constrained-sampler fallback |
| Translucent purple | Gaussian cutoff ellipsoids from the exact sampling covariance |

Path colors are green for Cartesian + IK, blue for projection, and orange for the uniform baseline. A returned path that fails the interpolated path audit is red.

Expand the sample display's namespaces to isolate an individual request and stage. Each stage retains up to 1,000 points and publishes at 10 Hz. Samples remain visible for 30 seconds after request release; fast plans also publish a final snapshot. Rerun the visual command to refresh them. Paths remain until replaced for their mode.

For an existing RViz instance, add the two MarkerArray topics and use **Transient Local** durability / **Reliable** reliability. The config file is `src/moveit_resources/panda_moveit_config/launch/sampling.rviz`.

These clouds show sampler proposals and returned states, not every RRT tree vertex. Use `distribution_checks` in `results.json` for per-component mean/covariance checks; visual containment alone does not establish distribution fidelity. Projected FK and validity-conditioned samples need not follow the original Cartesian GMM exactly.

---

## 4. Paired Sampling Benchmark

Requires either the laboratory from Section 2 or a running MoveIt + TP-GMM setup. The benchmark itself **only plans**. It applies temporary benchmark collision objects, restores those objects afterward, and does not execute robot trajectories or move Gazebo entities. Unrelated existing scene objects remain and can influence results.

### Supported modes

| Mode | Proposal |
|---|---|
| `cartesian_ik` | Truncated Cartesian GMM draws followed by randomized-seed IK |
| `joint_projected` | Local joint-space projection around verified IK anchors, with configurable exploration/rescue |
| `uniform` | MoveIt's standard IK constraint sampler inside the same corridor |

### Commands

```bash
# Default comparison: 3 cases × 3 modes × 3 measured repeats = 27 measured plans.
# One warmup pair per case adds 9 requests excluded from the summaries.
ros2 run tp_gmm compare_sampling_approaches.py \
  --repeats 3 --warmup 1 --planning-time 3 \
  --output sampling_results/cod-comparison

# Compare just the two GMM approaches.
ros2 run tp_gmm compare_sampling_approaches.py \
  --modes cartesian_ik joint_projected \
  --repeats 10 --warmup 1 --seed 42 --planning-time 3 \
  --output sampling_results/cod-two-approaches

# Visual inspection; timing includes sample publication overhead.
ros2 run tp_gmm compare_sampling_approaches.py \
  --repeats 1 --warmup 0 --visualize \
  --output sampling_results/cod-visual

# Explicit case file and sampler configuration.
ros2 run tp_gmm compare_sampling_approaches.py \
  --cases src/tp_gmm/config/comparison_cases.json \
  --sampler-config src/tp_gmm/config/sampling.json \
  --repeats 3 --warmup 1 \
  --output sampling_results/cod-configured

# Disable explicit exploration fractions; strict MoveIt wrapper requires the rebuilt library.
ros2 run tp_gmm compare_sampling_approaches.py \
  --modes cartesian_ik joint_projected \
  --sampler-config src/tp_gmm/config/sampling_pure.json \
  --repeats 3 --warmup 1 \
  --output sampling_results/cod-pure
```

### Benchmark arguments

| Flag | Default | Meaning |
|---|---|---|
| `--cases` | installed `config/comparison_cases.json` | JSON test-case list |
| `--modes` | all three modes | Modes to compare |
| `--repeats` | `3` | Measured repetitions per case |
| `--warmup` | `1` | Warmup repetitions per case, excluded from summaries |
| `--planning-time` | `3.0` | Allowed planning time per request, seconds |
| `--seed` | `42` | Custom sampler / trial-order seed |
| `--task` | `franka_pick_cube` | TP-GMM task |
| `--ref-robot` | `native` | Demonstration-frame adjustment; `ur10` for legacy UR10 demonstrations |
| `--clearance` | `0.5` | Policy desired-clearance factor |
| `--sampler-config` | built-in defaults | JSON sampling-option overrides |
| `--visualize` | off | Publish sample and comparison-path markers |
| `--allow-no-policy` | off | Explicitly permit an undeformed-model smoke test |
| `--output` | timestamped `sampling_results/` directory | Output directory |

Each pair reuses the same complete start state, joint goal, scene, and returned GMM. Mode order is shuffled. Both endpoints must satisfy the corridor, and success requires MoveIt success plus the interpolated path audit. Custom seeds do not control all internal OMPL/default-sampler/IK randomness.

**Differences from the Gemini command interface:** the Codex benchmark uses `--repeats` rather than `--trials`, and the baseline is named `uniform`. `uniform_box`, `default_unconstrained`, `--step`, `--replay`, and `--delay` are not supported by this benchmark.

### Results and timing breakdown

| File | Content |
|---|---|
| `results.json` | Exact models, trajectories, request metrics, distribution checks, and aggregate statistics |
| `trials.csv` | Trial outcomes, pipeline/sampler timings, path length, clearance, efficiency, and audit counters |
| `summary.md` | Success, median/p95 action times, and median stage breakdown |
| `comparison.png`, `comparison.pdf` | Six-panel comparison figure |

```bash
cat sampling_results/cod-comparison/summary.md

# Regenerate the figure from saved results; no running ROS nodes required.
python3 src/tp_gmm/scripts/plot_sampling_comparison.py \
  sampling_results/cod-comparison/results.json
```

The stage breakdown includes TP-GMM reproduction, RL deformation, sampler setup, sampling, online IK, anchor IK, Jacobian setup, and FK mapping. IK/FK/validity sub-timers are nested inside setup/sampling; do not add them to their parent totals. Clearance is a sampled whole-robot distance to padded world geometry, not a continuous clearance guarantee.

---

## 5. Sampling Configuration & Live Switching

### Configure before an executing experiment

```bash
ros2 run tp_gmm run_experiments.py \
  --trials 5 --sampling-mode joint_projected \
  --sampler-config src/tp_gmm/config/sampling.json
```

This command requires the execution-capable bringup in Section 6.

### Switch modes while the runner is active — another terminal

```bash
ros2 param get /gazebo_experiment_runner sampling_mode
ros2 param set /gazebo_experiment_runner sampling_mode joint_projected
ros2 param set /gazebo_experiment_runner sampling_mode cartesian_ik
ros2 param set /gazebo_experiment_runner sampling_mode uniform
```

The next prepared plan uses the new mode; in-flight plans retain their immutable settings. The runner must be spinning callbacks to acknowledge the parameter change, so a manual input prompt can delay it. Benchmark mode selection uses `--modes` rather than this live parameter.

### JSON sampling options

Edit a copy of `src/tp_gmm/config/sampling.json` and pass it using `--sampler-config`. Omitted keys retain defaults.

| Key | Default | Meaning |
|---|---:|---|
| `corridor_mode` | `covariance` | Covariance cutoff region; `legacy_weighted` restores historical weighted sizing |
| `corridor_scale` | `10.0` | Historical diameter multiplier: scale × original weight × standard deviation |
| `cutoff` | `2.0` | Cartesian rejection radius when `corridor_mode` is `covariance` |
| `covariance_floor` | `1e-8` | Spatial covariance eigenvalue floor, m² |
| `uniform_fraction` | `0.1` | Standard constrained-sampler exploration probability |
| `cartesian_fraction` | `0.1` | Cartesian IK fraction of remaining projected-mode proposals |
| `ik_timeout` | `0.005` | Per-call IK time budget, seconds |
| `branches` | `3` | Maximum distinct anchors per component |
| `anchor_attempts` | `16` | Maximum nominal IK attempts per component |
| `nullspace_stddev` | `0.08` | Position-null-space noise scale |
| `max_joint_delta` | `0.6` | Maximum joint displacement from an anchor |
| `linearization_tolerance` | `0.01` | Maximum FK linearization residual, metres |

`sampling_pure.json` sets `uniform_fraction` and `cartesian_fraction` to zero. Projected-only sampling rejects unanchored components without Cartesian rescue. The rebuilt MoveIt wrapper honors `allow_constraint_sampler_fallback: false` and times out without default uniform draws. Verify the installed behavior with `test_strict_sampling_runtime.py`; parameters and plugin counters alone cannot prove that the wrapper patch is loaded. See the strict statistical study commands below.

---

## 6. Gazebo Experiment Pipeline

Use this workflow when you want actual simulated trajectory execution. Stop the plan-only laboratory on the same ROS domain first.

### Terminal 1 — Gazebo + MoveIt + RViz

```bash
ros2 launch moveit_resources_panda_moveit_config gazebo_sim.launch.py
```

This loads the Panda warehouse world, robot/controllers, MoveIt, and the sampling RViz configuration. `world:=/path/to/custom.world` is available for a custom world; the experiment runner assumes the warehouse world service and its named entities.

For mock controllers without Gazebo physics, use `ros2 launch moveit_resources_panda_moveit_config demo.launch.py`. The Gazebo experiment runner still expects Gazebo entity services, so use the actual Gazebo launch for that runner.

### Terminal 2 — TP-GMM + legacy model visualizers

```bash
ros2 launch tp_gmm lfd_launch.py
```

The launch file selects the checkpoint in its `policy_ckpt_path` entry; check this when switching between policies. Its `task` and `subtask` arguments are declared but do not select the task passed to the model service; use the runner's `--task` argument. The Codex sampler receives models from service responses and does not depend on the legacy marker-to-box bridge.

To choose a checkpoint without those extra visualization nodes, run this instead:

```bash
ros2 run tp_gmm tp_gmm_node.py --ros-args \
  -p policy_ckpt_path:="$TPGMM_POLICY_CKPT"
```

Run one TP-GMM service provider on the domain.

### Terminal 3 — execute experiment trials

```bash
# Cartesian + IK, with live sample markers enabled.
ros2 run tp_gmm run_experiments.py \
  --trials 5 --task franka_pick_cube --sampling-mode cartesian_ik --clearance 0.5

# Joint projection.
ros2 run tp_gmm run_experiments.py \
  --trials 5 --sampling-mode joint_projected

# Standard constrained-sampler baseline.
ros2 run tp_gmm run_experiments.py \
  --trials 5 --sampling-mode uniform

# Pause before each trial.
ros2 run tp_gmm run_experiments.py \
  --trials 5 --manual --sampling-mode cartesian_ik

# Disable sample publication for an executing run.
ros2 run tp_gmm run_experiments.py \
  --trials 5 --sampling-mode joint_projected --no-sample-viz
```

This runner randomizes the environment, moves to a start state, calls TP-GMM/RL, and executes the constrained approach to a pre-grasp pose. The final descend/grasp/lift block is currently commented out in the implementation. Use the paired benchmark for controlled timing comparisons and structured result files.

| Runner flag | Default |
|---|---|
| `--trials`, `-n` | `10` |
| `--task`, `-t` | `franka_pick_cube` |
| `--robot`, `-r` | `franka_panda` |
| `--ref-robot` | `auto` |
| `--clearance`, `-c` | `0.5` |
| `--sampling-mode` | `cartesian_ik` |
| `--auto` / `--manual` | automatic |
| `--no-sample-viz` | visualization enabled |
| `--sampler-config` | built-in defaults |

---

## 7. TP-GMM Nodes & Service Calls

These examples use the actual Codex service fields. The benchmark normally constructs these requests for you.

### Start only the model node

```bash
ros2 run tp_gmm tp_gmm_node.py --ros-args \
  -p policy_ckpt_path:="$TPGMM_POLICY_CKPT"
```

### Inspect the service definitions

```bash
ros2 interface show tp_gmm/srv/StartTPGMM
ros2 interface show tp_gmm/srv/ReproduceTPGMM
ros2 interface show tp_gmm/srv/DeformTPGMM
ros2 interface show tp_gmm/srv/PrepareSampling
ros2 interface show tp_gmm/srv/SamplingReport
```

### Train a task model

This fits and saves the model in the task directory; it is not needed when using the existing trained model.

```bash
ros2 service call /StartTPGMM_service tp_gmm/srv/StartTPGMM \
  "{task_name: 'franka_pick_cube'}"
```

### Reproduce at new start and goal poses

```bash
ros2 service call /ReproduceTPGMM_service tp_gmm/srv/ReproduceTPGMM "{
  task_name: 'franka_pick_cube',
  frame_id: 'panda_link0',
  start_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.30, y: -0.12, z: 0.62}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}
  },
  goal_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.58, y: 0.12, z: 0.50}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}
  }
}"
```

The reproduction service has an empty response; inspect the published GMM/trajectory topics. The deformation service below returns both GMMs directly.

### Reproduce and deform around an obstacle

The original-demo and deformed task poses are separate required fields. They coincide in this native Panda example. The obstacle pose is its top position, matching the experiment convention.

```bash
ros2 service call /DeformTPGMM_service tp_gmm/srv/DeformTPGMM "{
  task_name: 'franka_pick_cube',
  frame_id: 'panda_link0',
  tpgmm_start_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.30, y: -0.12, z: 0.62}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}
  },
  tpgmm_goal_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.58, y: 0.12, z: 0.50}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}
  },
  deformed_tpgmm_start_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.30, y: -0.12, z: 0.62}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}
  },
  deformed_tpgmm_goal_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.58, y: 0.12, z: 0.50}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}
  },
  obstacle_pose: {
    header: {frame_id: 'panda_link0'},
    pose: {position: {x: 0.44, y: 0.0, z: 0.61}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  },
  obstacle_radius: 0.04,
  desired_clearance: 0.5
}"
```

The response contains `original_gmm`, `deformed_gmm`, `policy_applied`, and the model-load/reproduction/RL/regression/total stage timings. This call alone does not add a collision object or submit a motion-planning request.

The runners use `sampling_client.py` to pass the returned model to `/gmm_sampling/prepare`, then use the acknowledged constraint name and region in the plan. After the action result, `/gmm_sampling/report` retrieves metrics and releases the request. There is no latest-model topic or live statistics topic to select the Codex sampler.

---

## 8. Demonstrations & Policy Validation

### Collect demonstrations — execution-capable setup required

```bash
ros2 run tp_gmm collect_trajectories.py \
  --robot franka_panda --task franka_pick_cube --num-demons 5

# Automatically save valid demonstrations.
ros2 run tp_gmm collect_trajectories.py \
  --robot franka_panda --task franka_pick_cube --num-demons 5 --auto

# Launch-file alternative for the forwarded task/robot/count arguments.
ros2 launch tp_gmm collect_demons_launch.py \
  task:=franka_pick_cube robot:=franka_panda num_demons:=5
```

For collector options such as `--no-gazebo`, `--save-dir`, or `--auto`, use the script directly; the current collection launch does not forward all of its declared arguments.

### Visualize recorded demonstrations

```bash
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube --demon demon_3
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube --cycle 2.0
ros2 run tp_gmm visualize_demonstrations.py --task franka_pick_cube --rate 2.0 --frame-id panda_link0
```

### Legacy interactive policy validation

```bash
ros2 run tp_gmm validate_policy.py --robot franka_panda --task franka_pick_cube \
  --ros-args -p policy_ckpt_path:="$TPGMM_POLICY_CKPT"
```

This is the existing interactive policy-validation workflow and can prompt to execute trajectories. It has not been migrated to the Codex request-scoped sampler comparison; use `compare_sampling_approaches.py` to compare the new sampling implementations.

---

## 9. Automated Tests

### Compiled sampling mathematics

```bash
ctest --test-dir build/tp_gmm -R sampling_math --output-on-failure
```

Covers covariance factorization/symmetrization, rejection sampling, Mahalanobis support, and SVD covariance/null-space identities.

### Python package tests

```bash
ROS_LOG_DIR=/tmp/tpgmm-tests python3 -m unittest discover -s src/tp_gmm/tests

# Focused benchmark-reporting checks.
python3 -m unittest discover -s src/tp_gmm/tests -p test_sampling_reporting.py
```

The existing suite includes ROS-node and data/policy tests, so it needs the sourced workspace, local ROS sockets, and the existing demonstration/checkpoint files. Some legacy tests can regenerate task artifacts.

### Protocol integration — laboratory must be running on the same domain

```bash
python3 src/tp_gmm/tests/test_sampling_protocol.py
```

Checks malformed models, transactional registration, independent request IDs, release behavior, and transient-local marker delivery.

### Actual planner sample-stream integration

```bash
python3 src/tp_gmm/tests/test_sampling_visual_stream.py \
  --output sampling_results/cod-visual-check
```

Runs nine plan-only requests, checks captured raw proposals against their matching ellipsoids, and verifies valid-FK samples and all three path displays. It writes `visual_stream_checks.json` and the normal comparison outputs.

### CMake-registered tests through colcon

```bash
colcon test --packages-select tp_gmm --event-handlers console_direct+
colcon test-result --verbose
```

The new mathematical test is registered with CMake. Run the Python and integration commands above separately; `colcon test` does not automatically run those scripts.

---

## 10. ROS Diagnostics & Troubleshooting

### Active interfaces

| Topic / service | Type | Purpose |
|---|---|---|
| `/gmm_sampling/markers` | `visualization_msgs/msg/MarkerArray` | Proposal/FK clouds and exact covariance support |
| `/gmm_sampling/paths` | `visualization_msgs/msg/MarkerArray` | Per-mode comparison paths |
| `/gmm_sampling/prepare` | `tp_gmm/srv/PrepareSampling` | Register a validated model and immutable request settings |
| `/gmm_sampling/report` | `tp_gmm/srv/SamplingReport` | Retrieve request metrics and optionally release the request |
| `/gmm/cartesian_space` | `tp_gmm/msg/GaussianMixture` | Original reproduced model |
| `/gmm/deformed_cartesian_space` | `tp_gmm/msg/GaussianMixture` | Deformed model |
| `/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` | MoveIt trajectory visualization |
| `/DeformTPGMM_service` | `tp_gmm/srv/DeformTPGMM` | Reproduction + RL deformation and timing |

```bash
# Confirm the installed package and ROS domain.
ros2 pkg prefix tp_gmm
printenv ROS_DOMAIN_ID

# Verify the sampler plugin is configured and exposes its services.
ros2 param get /move_group constraint_samplers
ros2 service list | rg 'gmm_sampling|TPGMM'
ros2 service type /gmm_sampling/prepare

# Check the configured edge-checking resolution.
ros2 param get /move_group ompl.panda_arm.longest_valid_segment_fraction

# Inspect marker publishers and receive a retained marker snapshot.
ros2 topic info /gmm_sampling/markers --verbose
ros2 topic echo /gmm_sampling/markers --once --qos-durability transient_local
ros2 topic info /gmm_sampling/paths --verbose

# Inspect the next deformed model publication.
ros2 topic echo /gmm/deformed_cartesian_space --once

# Monitor sampler allocation / planning messages.
ros2 topic echo /rosout | rg 'GMM|TP-GMM|Allocating specialized|Exception caught'
```

| Symptom | Check / action |
|---|---|
| Sampling service unavailable | Check the ROS domain, sourced overlay, plugin parameter, and MoveIt logs; restart MoveIt after rebuilding |
| Benchmark says the RL policy was not applied | Supply the checkpoint when starting the TP-GMM node; use `--allow-no-policy` only for an intentional undeformed test |
| No sample points in RViz | Benchmark needs `--visualize`; executing runner must omit `--no-sample-viz`; points expire after release |
| Old Gemini topic/flag missing | Use `/gmm_sampling/markers`, the report service, `--repeats`, and the `uniform` mode from this guide |
| Start/goal invalid in corridor | Inspect the matching model and test poses; the benchmark records the failure rather than dropping the corridor |
| Projection accepts few or no targets | Inspect trust-region/support rejections, anchors, online-IK rescue and failed calls; keep the mixed profile for the default comparison |
| MoveIt success but benchmark failure | Inspect `failure_stage`, `planner_success`, `path_audit_passed`, and `path_invalid_samples`; the post-plan audit can reject a returned path |

Stop each launched process with **Ctrl+C** in its terminal before changing bringup workflows or rebuilding a running sampler.


## 11. RL Clearance Generalization Sweeps

See [CLEARANCE_ANALYSIS.md](CLEARANCE_ANALYSIS.md) for the experiment design and metric definitions. Each trial fixes the environment and endpoint joint states across **ten clearance values, 0.1–1.0**. The experiment is plan-only; it changes MoveIt's planning scene and restores its own objects on exit. It does not execute the robot or reposition Gazebo entities.

### Build and start the model service

The deformation service now returns request-matched original/deformed DSGMR trajectories and policy provenance. Rebuild once and restart the TP-GMM service before using the sweep:

```bash
colcon build --packages-select tp_gmm --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Use the existing Gazebo/MoveIt pipeline on its ROS domain, or start an isolated laboratory. For the latter, run the following in the bringup terminal after the workspace setup in Section 1:

```bash
export ROS_DOMAIN_ID=81
ros2 launch tp_gmm sampling_demo.launch.py rviz:=false \
  policy_ckpt_path:="$TPGMM_POLICY_CKPT"
```

Set `ROS_DOMAIN_ID=81` in the runner terminal too. The checkpoint is loaded by the TP-GMM node; the sweep records the actual loaded path/hash. Choose the checkpoint matching the prior model being evaluated.

### Run one sampler or a paired comparison

```bash
# Five environments × ten levels = 50 plans.
ros2 run tp_gmm run_clearance_sweep.py --trials 5 \
  --sampler cartesian_ik --seed 42 --output clearance_results/cartesian

# Five environments × ten levels × three samplers = 150 plans.
ros2 run tp_gmm run_clearance_sweep.py --trials 5 \
  --samplers cartesian_ik joint_projected ompl_uniform \
  --planning-time 5 --seed 42 --output clearance_results/paired

# Include 0.0 for eleven levels and use the pure sampling profile.
ros2 run tp_gmm run_clearance_sweep.py --trials 5 \
  --sampler joint_projected --include-zero \
  --sampler-config src/tp_gmm/config/sampling_pure.json \
  --output clearance_results/projected_with_zero
```

`ompl_uniform` sends empty path constraints and uses the full bounded joint space. It differs from the older `uniform` mode, which samples inside the GMM corridor. It is not uniform in Cartesian volume. Its planning request is independent of the requested clearance; any baseline trend reflects stochastic planning.

### Replay environments and regenerate figures

```bash
ros2 run tp_gmm run_clearance_sweep.py --trials 5 \
  --sampler joint_projected \
  --environments clearance_results/cartesian/environments.json \
  --output clearance_results/replay_projected

# Offline plotting; ROS does not need to be running.
python3 src/tp_gmm/scripts/plot_clearance_analysis.py \
  clearance_results/paired/results.json --max-examples 10
```

Replaying preserves poses but solves endpoint IK again. Use one `--samplers` invocation to share exactly the same endpoint joint states and deformation response across modes.

### Configuration and outputs

| Option | Default | Purpose |
|---|---|---|
| `--trials` | 5 | Number of random endpoint-valid environments |
| `--include-zero` | off | Add clearance 0.0 to the ten default levels |
| `--environment-config` | built-in ranges | Override ranges from `config/clearance_environment.json` |
| `--max-environment-attempts` | 40 | Bound endpoint-IK environment selection attempts |
| `--sampler-config` | sampler defaults | JSON sampler configuration; defaults to covariance mode, cutoff 2.0 |
| `--max-clearance-margin` | 0.30 m | Interpret normalized clearance using the training scale |
| `--base-buffer` | 0.08 m | Training reward's additional physical buffer |
| `--obstacle-reference` | `top` | Policy reference point; use `center` when that matches training/deployment |
| `--joint-step` | 0.02 | Joint-vector spacing for FK and validity audits |
| `--curve-step` | 0.002 m | Polyline spacing for obstacle-distance analysis |
| `--trim-fraction` | 0.1 | Additional interior metric excludes each endpoint's arc-length fraction |
| `--visualize` | off | Enable custom sampler markers; figures are always generated |

Clearance input is normalized, not metres. The training metric uses intermediate Gaussian means' 3-D distance to the policy reference point, minus radius, with target `base_buffer + clearance × max_clearance_margin`. Actual trajectory clearance uses signed distance to the finite cylinder surface. It measures trajectory points/end-effector origin, not the whole robot surface.

Each output directory contains `results.json`, scalar `trials.csv`, per-point `trajectory_points.csv`, replayable `environments.json`, exact model pairs under `models/`, and `summary.md`. PNG/PDF figures under `figures/` show clearance response, success/time/path length, per-environment behavior, endpoint errors, 3-D overlays, and clearance along each curve. Failures remain in success rates; invalid paths are excluded from successful-path statistics. Use a fresh output directory for each run.

### Clearance regression tests

```bash
python3 -m unittest discover -s src/tp_gmm/tests -p 'test_clearance*.py'
```

These cover cylinder geometry, between-vertex crossings, normalized sweep levels, random-design reproducibility, training-reference metrics, missing/failed levels, and the plan-only unconstrained OMPL request contract.


## GMR reference-path proposals (Cartesian IK)

See [GMR_SAMPLING.md](GMR_SAMPLING.md) for the distribution, modular API and interpretation. Rebuild after switching branches because `PrepareSampling` gained reference/proposal fields.

```bash
# Existing Gazebo execution pipeline (executes motion):
ros2 run tp_gmm run_experiments.py --sampling-mode cartesian_ik \
  --sampler-config src/tp_gmm/config/sampling_gmr.json
# 80% GMR / 20% GMM proposals, with the same hard corridor:
ros2 run tp_gmm run_experiments.py --sampling-mode cartesian_ik \
  --sampler-config src/tp_gmm/config/sampling_gmr_hybrid.json
# Independent visualization switches: --no-sample-viz --no-path-viz

# Separate controller-free environment, terminal 1 (no Gazebo required):
export ROS_DOMAIN_ID=81
ros2 launch tp_gmm sampling_demo.launch.py rviz:=true \
  policy_ckpt_path:=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt
# Terminal 2, same sourced workspace and isolated domain:
export ROS_DOMAIN_ID=81
ros2 run tp_gmm compare_gmr_proposals.py \
  --gmr-stddevs 0.005 0.015 0.03 --gmr-fraction 0.8 \
  --repeats 5 --warmup 1 --planning-time 3 --output gmr_results/my_run
```

The paired comparison tests the same start/goal, deformed model and reference with GMM, GMR, hybrid and direct-reference IK. It changes its own MoveIt collision objects and restores them at exit, never executing motion. Reports and plots include reference deviation, latency, acceptance, path lengths and clearance. The clearance sweep also accepts these presets through `--sampler-config` with `--samplers cartesian_ik`. A nonzero `uniform_fraction` adds the existing constrained fallback; the presets and paired GMR comparison use zero.


## Strict pure-sampler statistical case study

Protocol and interpretation: [SAMPLING_STUDY.md](SAMPLING_STUDY.md). All three repositories must be on `sampling-approaches-cod`. Rebuild and restart; the installed MoveIt library must include the strict wrapper patch.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --packages-select moveit_planners_ompl tp_gmm moveit_resources_panda_moveit_config \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=81
ros2 launch tp_gmm sampling_demo.launch.py rviz:=false \
  policy_ckpt_path:=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt
```

In another sourced terminal, using the same isolated domain:

```bash
export ROS_DOMAIN_ID=81
python3 src/tp_gmm/tests/test_strict_sampling_runtime.py
ros2 run tp_gmm run_sampling_study.py \
  --environments-per-stratum 10 --repeats 10 --planning-time 3 \
  --seed 20260926 --output sampling_results/my-pure-study
ros2 run tp_gmm analyze_sampling_study.py sampling_results/my-pure-study
```

This runs 60 environments × 10 repeats × three methods (1,800 plans), with cutoff 2.0, zero explicit mixture fractions and strict no-default-fallback behavior. It never executes trajectories. `--design-only` saves all environments/models before planning; `--resume` requires the same design arguments and resumes only missing trials. Use a new directory for a new independent study. Do not pool interrupted/restarted duplicate trials or keep adding runs until a p-value crosses a threshold.

Output includes frozen inputs, append-only `results.jsonl`, `trials.csv`, paired cluster statistics, PDF/PNG figures and `summary.md`. Unrestricted OMPL has empty GMM path constraints and samples joint coordinates; it is not uniform in Cartesian volume. The automated runner refuses a server that does not advertise strict sampling and joint-state-space configuration. The synthetic regression checks that the installed implementation actually enforces the advertised flag.

The completed case study also uses an independent path-quality confirmation phase. Its directional hypotheses are specified in `SAMPLING_STUDY.md`; report its results separately from the initial success/PAR2 family:

```bash
ros2 run tp_gmm run_sampling_study.py \
  --study-phase path_quality_confirmation \
  --environments-per-stratum 10 --repeats 5 --planning-time 3 \
  --seed 20260927 --output sampling_results/my-path-confirmation
ros2 run tp_gmm analyze_sampling_study.py sampling_results/my-path-confirmation
```


## Clearance-only replay (0.5 / 0.7)

See [CLEARANCE_REPLAY.md](CLEARANCE_REPLAY.md). This reuses the original saved environments and exact joint endpoints; it changes only requested RL clearance. The original 0.2 group maps to 0.5 and the 0.8 group maps to 0.7, so only the latter is a reduction. Keep the original goal-height range, checkpoint, cutoff 2.0, pure samplers and strict wrapper. Run from the workspace root against the same isolated planning laboratory:

```bash
export ROS_DOMAIN_ID=81
python3 src/tp_gmm/tests/test_strict_sampling_runtime.py
python3 src/tp_gmm/scripts/replay_sampling_study.py \
  --source sampling_results/2026-09-26-pure-study \
  --output sampling_results/2026-09-28-clearance-primary --clearances 0.5 0.7
python3 src/tp_gmm/scripts/replay_sampling_study.py \
  --source sampling_results/2026-09-26-path-confirmation \
  --output sampling_results/2026-09-28-clearance-confirmation --clearances 0.5 0.7
python3 src/tp_gmm/scripts/analyze_sampling_study.py sampling_results/2026-09-28-clearance-primary
python3 src/tp_gmm/scripts/analyze_sampling_study.py sampling_results/2026-09-28-clearance-confirmation
python3 src/tp_gmm/scripts/compare_clearance_replays.py \
  --before sampling_results/2026-09-26-pure-study sampling_results/2026-09-26-path-confirmation \
  --after sampling_results/2026-09-28-clearance-primary sampling_results/2026-09-28-clearance-confirmation \
  --output sampling_results/2026-09-28-clearance-comparison
```

Use `--resume` only with identical source and clearance settings. A single `--clearances 0.6` applies 0.6 to every source case. Separate runs at additional levels must retain source-environment identities; they do not create new independent environments.

The replay verifies MoveIt's live collision geometry against every frozen scene before planning. A successful scene-update service response alone is insufficient on this MoveIt build. The first attempt was invalidated and preserved separately after this discrepancy was reproduced; use only the verified restart described in [the protocol amendment](CLEARANCE_REPLAY.md#scene-restoration-amendment-before-the-retained-replay).

Completed run: [clearance replay report, plots and data](case_studies/2026-09-28-clearance-replay/README.md).

## Training goal-height range with a separate lower-table scene

See [GOAL_HEIGHT_STUDY.md](GOAL_HEIGHT_STUDY.md). The control retains the exact original scene, table surface at z=0.25 m and goals at z=0.45–0.54 m. A separate cloned scene maps goal heights to the checkpoint's saved z=0.10–0.30 m training range and lowers the identical table to a surface at z=−0.02 m. Table dimensions, horizontal position, cylinder poses, starts, goal x/y, policy, clearances 0.5/0.7 and strict cutoff-2.0 sampling stay fixed. These are plan-only MoveIt scenes; Gazebo entities are not moved.

Use the isolated controller-free launch from the pure-study instructions above. Run cohorts sequentially because they share the active planning scene:

```bash
export ROS_DOMAIN_ID=81
python3 src/tp_gmm/scripts/run_goal_height_study.py \
  --source sampling_results/2026-09-28-clearance-primary \
  --output sampling_results/2026-09-28-height-primary \
  --goal-z-range 0.1 0.3 --table-top-z -0.02
python3 src/tp_gmm/scripts/run_goal_height_study.py \
  --source sampling_results/2026-09-28-clearance-confirmation \
  --output sampling_results/2026-09-28-height-confirmation \
  --goal-z-range 0.1 0.3 --table-top-z -0.02

# Analyze after all timed measurements have finished.
for cohort in primary confirmation; do
  for arm in control training; do
    python3 src/tp_gmm/scripts/analyze_sampling_study.py \
      "sampling_results/2026-09-28-height-${cohort}/${arm}"
  done
done
python3 src/tp_gmm/scripts/analyze_goal_height_study.py \
  sampling_results/2026-09-28-height-primary \
  sampling_results/2026-09-28-height-confirmation \
  --output sampling_results/2026-09-28-height-comparison
```

`--table-top-z` affects only the training scene. `--design-only` freezes scenes/models/endpoints without measured plans; `--resume` continues an identical frozen design. Keep separate output directories; source results are never overwritten. The total is 5,400 request outcomes, including endpoint-solve failures recorded without invoking planning. No uniform fallback or missing-anchor Cartesian rescue is allowed in either custom sampler.

The paired report retains all endpoint failures and also reports the endpoint-feasible subset. Its primary inference weights source environments equally, preserves repeat clusters and corrects the three sampler comparisons. Because both goal height and table height change, this checks the requested scene arrangement and cannot isolate goal-height mismatch as the sole cause. Reused cohorts are sensitivity evidence, not fresh independent confirmation.

Completed run: [goal-height and lower-table report, figures and raw data](case_studies/2026-09-28-goal-height/README.md).

## Covariance cutoff 3.0 on the original scene

See [CUTOFF_STUDY.md](CUTOFF_STUDY.md). Reuse the original 120 environments, exact joint endpoints, table surface at z=0.25 m, goal heights z=0.45–0.54 m and requested RL clearances **0.2 / 0.8**. Only the covariance cutoff changes from 2.0 to 3.0. This widens both truncated proposals and the hard covariance-derived corridor; it does not change the covariance matrices. The unrestricted OMPL baseline has no GMM path constraints.

In a separate terminal, launch the plan-only laboratory with the original checkpoint:

```bash
export ROS_DOMAIN_ID=81
ros2 launch tp_gmm sampling_demo.launch.py rviz:=false lfd:=true \
  policy_ckpt_path:="$TPGMM_POLICY_CKPT"
```

Run the two cohorts sequentially from a sourced workspace terminal in the same domain. Use fresh output paths when repeating this completed run:

```bash
export ROS_DOMAIN_ID=81
python3 src/tp_gmm/tests/test_strict_sampling_runtime.py --cutoff 3.0
python3 src/tp_gmm/scripts/replay_sampling_study.py \
  --source sampling_results/2026-09-26-pure-study \
  --output sampling_results/2026-09-29-cutoff3-primary \
  --clearances 0.2 0.8 --cutoff 3.0
python3 src/tp_gmm/scripts/replay_sampling_study.py \
  --source sampling_results/2026-09-26-path-confirmation \
  --output sampling_results/2026-09-29-cutoff3-confirmation \
  --clearances 0.2 0.8 --cutoff 3.0

# Analyze after both timed runs finish.
python3 src/tp_gmm/scripts/analyze_sampling_study.py sampling_results/2026-09-29-cutoff3-primary
python3 src/tp_gmm/scripts/analyze_sampling_study.py sampling_results/2026-09-29-cutoff3-confirmation
python3 src/tp_gmm/scripts/compare_cutoff_replays.py \
  --before sampling_results/2026-09-26-pure-study sampling_results/2026-09-26-path-confirmation \
  --after sampling_results/2026-09-29-cutoff3-primary sampling_results/2026-09-29-cutoff3-confirmation \
  --output sampling_results/2026-09-29-cutoff3-comparison
```

`--design-only` freezes the environments before planning; `--resume` requires identical controls. Omitting `--cutoff` inherits the source value. Purity checks require the recorded cutoff and forbid uniform fallback, missing-anchor rescue and online IK in joint projection. The cutoff comparison verifies unchanged scenes, endpoints, both GMMs and both reference trajectories, then reports paired environment-cluster intervals and Holm-corrected success tests. Reused environments provide sensitivity evidence, not another independent confirmation.

Results: [cutoff-3.0 report, plots and data](case_studies/2026-09-29-cutoff3/README.md).

## Journal sampler comparison at fixed cutoff 3.0

The [standalone sampler report](case_studies/2026-09-29-sampler-comparison/README.md) and [manuscript section](case_studies/2026-09-29-sampler-comparison/manuscript.md) compare **Cartesian + IK, joint projection and unrestricted OMPL** using only the completed cutoff-3.0 cohorts. The earlier cutoff investigation remains a separate diagnostic study. This step analyzes saved outcomes; it does not run ROS planning or modify scenes.

```bash
python3 src/tp_gmm/scripts/summarize_sampler_case_study.py \
  sampling_results/2026-09-29-cutoff3-primary \
  sampling_results/2026-09-29-cutoff3-confirmation \
  --output sampling_results/my-sampler-comparison
```

Use a fresh output directory. The analyzer rejects inconsistent configurations, duplicate environments, changed input hashes, failed scene checks and impure sampling. It gives each environment equal weight despite different repeat counts, retains failed outcomes, and compares path quality on matched successful requests. All 24 two-sided method/outcome tests share one Holm correction; this combined analysis is retrospective. Exported CSV tables, JSON statistics and PDF/PNG figures cover success, PAR2, planning action and attributed pipeline time, path quality, clearance, sampler costs and model preparation. Pipeline time attributes the shared per-environment deformation measurement to each GMM request; it excludes common endpoint IK and the offline dense path audit. Stage medians and raw counts are separately labelled descriptive summaries.

## Extensive strict GMR comparison (30 mm only)

[Protocol and design](GMR_STUDY.md). Four methods: **GMM**, **GMR 30 mm**, **Hybrid 30 mm** (80% GMR / 20% GMM), and **reference_ik**. This experiment fixes GMM and GMR cutoffs to **3.0**, clearance to **0.7**, and both uniform and cross-mapping fallback fractions to zero. It also verifies MoveIt's outer fallback parameter is disabled. The three sampling proposals use Cartesian + IK mapping; reference IK follows the reproduced path directly. No unrestricted OMPL arm is included in this experiment.

Launch in a dedicated terminal from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
ros2 launch tp_gmm sampling_demo.launch.py rviz:=false \
  policy_ckpt_path:=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt
```

In a second sourced terminal with the same ROS domain and thread limits:

```bash
python3 src/tp_gmm/tests/test_strict_sampling_runtime.py --cutoff 3.0
python3 src/tp_gmm/tests/test_gmr_strict_runtime.py
python3 src/tp_gmm/scripts/run_gmr_study.py \
  --output gmr_results/my-extensive-gmr-study \
  --environments-per-layout 40 --repeats 5 --legacy-repeats 10 \
  --planning-time 3 --seed 20261005

# Run analysis after all timed measurements finish.
python3 src/tp_gmm/scripts/analyze_gmr_study.py gmr_results/my-extensive-gmr-study
```

The design has 120 new randomized environments × 5 repeats × 4 methods, plus the 3 original pilot scenes × 10 repeats × 4 methods: **2,520 measured outcomes**, with 12 excluded warmups. New randomized scenes and original scenes are analyzed separately. All methods share frozen exact endpoints, scene, deformation and reference in each environment. Endpoint failures remain in every method's denominator. The reference baseline now exports its geometric joint path for the same dense offline auditor used for RRT outputs.

`--design-only` freezes inputs without running measured plans. `--resume` requires the same output path, arguments, runtime configuration and source/binary hashes. Use a fresh output path for a new study. Infrastructure/purity failures stop the run rather than being hidden as planning failures. The analyzer produces environment-cluster confidence intervals, Holm-adjusted paired comparisons, CSV tables and PDF/PNG figures. Results are stored in `gmr_results/2026-10-05-extensive`; start with `analysis/report.md`. The historical `2026-09-24-pilot` remains separate.

Completed findings: [strict GMR comparison](case_studies/2026-10-05-gmr/FINDINGS.md). Each RRT proposal succeeded on all 565 randomized requests with solved endpoints (565/600 overall); reference IK succeeded on 405/600. GMR improved reference adherence by 17.2 mm versus GMM (95% CI 13.4–21.0 mm), without a demonstrated speed, path-length or clearance advantage. The [repository archive](case_studies/2026-10-05-gmr/README.md) includes compressed raw outcomes and frozen inputs, paired statistics, vector figures and integrity checks, plus the separate historical pilot.
