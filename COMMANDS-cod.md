# TP-GMM & MoveIt 2 — Codex Branch Command Guide

Command reference for **`sampling-approaches-cod`**: building, running Cartesian + IK and joint-projected GMM sampling, inspecting samples in RViz, comparing approaches, and testing the pipeline.

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

# Existing checkpoint used for the Codex validation runs.
export TPGMM_POLICY_CKPT=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-03_20-31-51_ppo_torch/checkpoints/best_agent.pt
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

The default region matches `/deformed_gmm_rviz_converter_output`: diameter = `10 × component weight × sqrt(covariance eigenvalue)`, using the same covariance floor. Both sampling approaches and the uniform baseline use the matching hard box corridor. For five equal weights this restores the old 1σ radius, rather than the initial Cod implementation's 3σ radius. The learned covariance itself is preserved.

After updating this version, stop the runner and restart **Gazebo/MoveIt and the TP-GMM launch**, sourcing `install/setup.bash` in each terminal. The preparation service schema and loaded sampler library changed; restarting only RViz does not apply the fix.

To explicitly use the wider covariance region, put `"corridor_mode": "covariance", "cutoff": 3.0` in the JSON passed to `--sampler-config`, and match the converter:

```bash
ros2 launch tp_gmm lfd_launch.py gmm_legacy_weighted_scale:=false gmm_cutoff:=3.0
```

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

# Disable the two explicit exploration fractions; see the fallback caveat in Section 5.
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
| `corridor_mode` | `legacy_weighted` | Historical confined region; `covariance` enables cutoff-based region |
| `corridor_scale` | `10.0` | Historical diameter multiplier: scale × original weight × standard deviation |
| `cutoff` | `3.0` | Cartesian rejection radius when `corridor_mode` is `covariance` |
| `covariance_floor` | `1e-8` | Spatial covariance eigenvalue floor, m² |
| `uniform_fraction` | `0.1` | Standard constrained-sampler exploration probability |
| `cartesian_fraction` | `0.1` | Cartesian IK fraction of remaining projected-mode proposals |
| `ik_timeout` | `0.005` | Per-call IK time budget, seconds |
| `branches` | `3` | Maximum distinct anchors per component |
| `anchor_attempts` | `16` | Maximum nominal IK attempts per component |
| `nullspace_stddev` | `0.08` | Position-null-space noise scale |
| `max_joint_delta` | `0.6` | Maximum joint displacement from an anchor |
| `linearization_tolerance` | `0.01` | Maximum FK linearization residual, metres |

`sampling_pure.json` sets `uniform_fraction` and `cartesian_fraction` to zero. Unanchored components can still use Cartesian IK rescue, and OMPL can fall back after repeated sampler failures. Inspect `online_ik_calls`, `missing_anchor_fallbacks`, `projected_valid`, and `failed_calls` before attributing a successful plan entirely to projection. A successful plan with zero accepted projected targets is not evidence of a successful projection-only search.

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

The launch file currently embeds the 3 September checkpoint used above. Its `task` and `subtask` arguments are declared but do not select the task passed to the model service; use the runner's `--task` argument. The Codex sampler receives models from service responses and does not depend on the legacy marker-to-box bridge.

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
