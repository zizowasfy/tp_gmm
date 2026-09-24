# GMM sampling runtime on gazebo_exps

This branch contains the Cartesian + IK and local joint-projected samplers as reusable MoveIt constraint-sampler modules. The comparison/sweep runners, plotting/report-generation files and experimental model archives remain on `sampling-approaches-cod`.

The implementation uses a selective runtime port, not a full merge. Source: `tp_gmm` commit `6065612` on `sampling-approaches-cod`. The isolated MoveIt allocator fix is cherry-picked with provenance. Panda plugin activation and runtime RViz configuration are ported separately. The source branches are preserved.

## Runtime modules

| Module | Responsibility |
|---|---|
| `include/tp_gmm/sampling_math.hpp` | Covariance canonicalization, truncated Gaussian draws, SVD projection |
| `src/sampling/cartesian_ik.cpp` | Draw a Cartesian GMM target and solve randomized-seed IK |
| `src/sampling/joint_projected.cpp` | Build diverse IK anchors, project task covariance through the Jacobian, reject nonlinear FK drift |
| `src/sampling/sampler.cpp` | Shared frame transforms, validity checks, exploration, counters and sample buffering |
| `src/sampling/request_registry.cpp` | Validate immutable requests, allocate the plugin, report/release bounded sessions |
| `src/sampling/visualization.cpp` | Optional request-matched sample clouds and covariance ellipsoids |
| `python/tp_gmm_sampling/path_visualizer.py` | Optional successful planned end-effector paths using MoveIt FK |
| `python/tp_gmm_sampling` | Importable Python client independent of any task/experiment runner |

The plugin accepts a `GaussianMixture`, group, link, fixed model frame and proposal orientation. It has no battery, object-name, task-name, training-policy or RRTConnect dependency. The planner remains selected by the caller's normal MoveIt request. The existing Panda Gazebo bringup is the validation adapter, not the future benchmark environment.

The original `gazebo_exps` trained model, training parameters and default RL checkpoint are retained. Experimental model archives and changed training regularization were not imported. Select a benchmark-appropriate model and policy explicitly when that experiment is defined. A GMM can come from TP-GMM+RL or any other producer of the same message.

## Build and launch

All three integration repositories use `gazebo_exps`: `tp_gmm`, `moveit2`, and `moveit_resources`. The latter two branches were created from the pre-sampler bases (`main` and `moveit_gazebo`). All `sampling-approaches-cod` refs remain unchanged.

```bash
cd /home/zizo/the_folder/ws_moveit
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --executor sequential \
  --packages-select moveit_ros_planning moveit_planners_ompl tp_gmm moveit_resources_panda_moveit_config \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Restart MoveIt and the TP-GMM node after switching branches/building. The report API on this branch is intentionally runtime-only. Rebuild `tp_gmm` when switching back to the comparison branch as well; its generated service interface differs.

Controller-free validation (separate ROS domain in every terminal):

```bash
export ROS_DOMAIN_ID=81
ros2 launch tp_gmm gmm_sampling.launch.py lfd:=false rviz:=true
```

For the existing Gazebo execution pipeline, use its normal bringup and the model provider:

```bash
ros2 launch moveit_resources_panda_moveit_config gazebo_sim.launch.py
# Separate terminal, same ROS domain:
ros2 launch tp_gmm lfd_launch.py \
  policy_ckpt_path:=/absolute/path/to/the/intended/checkpoint.pt \
  gmm_cutoff:=2.0
# Separate terminal after the model provider starts:
ros2 run tp_gmm run_experiments.py --sampling-mode cartesian_ik
# Select projection instead:
ros2 run tp_gmm run_experiments.py --sampling-mode joint_projected \
  --sampler-config /home/zizo/the_folder/ws_moveit/src/tp_gmm/config/sampling.json
```

`run_experiments.py` is the original Gazebo execution workflow, now connected to the reusable client; it executes motion. It is not a comparison runner. Do not start both execution examples concurrently. Changing `/gazebo_experiment_runner` parameter `sampling_mode` selects the next prepared request; an in-flight request remains unchanged.

For a different robot's MoveIt bringup, load the allocator on its `move_group` node:

```python
{"constraint_samplers": "tp_gmm/GMMConstraintSamplerAllocator"}
```

The two MoveIt fixes preserve the request constraint name and honor this parameter. The Panda configuration also retains the denser `longest_valid_segment_fraction=0.0005` edge checking. Other robot configurations need their own collision/edge-resolution validation.

## Calling from a new benchmark or application

After sourcing the workspace, the client imports normally; no script-directory path manipulation is needed:

```python
from tp_gmm_sampling import SamplingClient

client = SamplingClient(node)  # namespace='/robot1' for a namespaced move_group
with client.session(
    deformation_response.deformed_gmm,
    group_name, link_name, proposal_orientation,
    mode="joint_projected", seed=42, visualize=True,
    cutoff=2.0, uniform_fraction=0.1, cartesian_fraction=0.1,
) as prepared:
    move_group_goal.request.path_constraints = prepared.constraints
    # Send the normal MoveGroup request, and wait for its terminal result.
    # On cancellation, wait for cancellation/completion before leaving this scope.
    result = send_and_wait(move_group_goal)  # supplied by your application
    diagnostics = client.report(prepared.request_id, release=False)
# The context manager releases the request even when the caller raises.
```

`send_and_wait` above is an application placeholder, not a packaged function. Start state, goal, planner, planning scene and execution policy belong to the application. The synchronous client spins its node while waiting for services; do not call it from a callback already spinning that same node. Asynchronous applications can use `tp_gmm.srv.PrepareSampling` and `SamplingReport` directly.

Service contract:

1. `/gmm_sampling/prepare` returns a unique ID and matching `moveit_msgs/Constraints`. Submit those constraints unchanged; never reconstruct the name from an RViz topic.
2. `/gmm_sampling/report` returns lightweight sampler counters/timings. Set `release=false` to keep the request. It accepts no trajectory and computes no benchmark metrics.
3. Release after planning completes, through `client.release(id)` or the context manager. Unknown/released IDs and mismatched constraints fail rather than silently reverting to another sampler. The registry has a capacity limit and expires abandoned inactive sessions.

The existing deformation service keeps its original `gazebo_exps` schema: original and deformed GMM messages. Comparison-only timings, trajectory arrays and policy-provenance fields are not imported. The Gazebo runner uses that exact response for planning and no longer waits for a latest-model bounding-box topic. Model conversion during reproduction/deformation no longer rewrites visualization bags.

## Sampling semantics and settings

The two supported modes are `cartesian_ik` and `joint_projected`. Covariance-mode support is the only sampler corridor policy on this branch. Defaults are in `config/sampling.json`:

- `cutoff=2.0`, `covariance_floor=1e-8`: truncated Gaussian radius and numerical eigenvalue floor. Mixture weights select components; they do not scale geometry.
- `uniform_fraction=0.1`: uniform exploration inside the same hard corridor, checked for bounds, constraints, feasibility and collisions. It is not workspace-wide exploration.
- `cartesian_fraction=0.1`: in projected mode, a fraction of non-uniform attempts use Cartesian IK. At zero, components without valid projection anchors are rejected (`missing_anchor_rejections`); there is no online Cartesian IK rescue. With a positive fraction, the existing unanchored-component rescue remains available and is counted.
- `ik_timeout=0.005`, `branches=3`, `anchor_attempts=16`: IK budget and projection anchor coverage.
- `nullspace_stddev=0.08`, `max_joint_delta=0.6`, `linearization_tolerance=0.01`: local projection trust and FK-residual limits.

Both modes enforce the same oriented box union with full side lengths `2 * cutoff * sqrt(eigenvalue)`. Raw Cartesian proposals are truncated inside their component ellipsoid. Boxes enclose ellipsoids, so some uniform samples or path segments can be outside the displayed ellipsoids while still satisfying the box constraints. Joint-projected FK is approximate and is checked against both support and nonlinear residual tolerance. Successful IK/collision filtering changes the distribution of accepted targets; RRT vertices are not raw GMM draws.

The proposal orientation is a bias, not an extra hard orientation constraint. This version constrains the selected link origin; task-specific tool offsets, additional benchmark constraints and different robot kinematics require deliberate integration and validation.

To run a normal unconstrained OMPL planner in a future benchmark, omit GMM path constraints. Do not pass a third sampler mode to this plugin. Benchmark success metrics, analysis, algorithm comparisons and the EV battery scene belong outside this runtime package and will be designed with the benchmark specification.

## Visualization and checks

`/gmm_sampling/markers` provides raw proposals, valid/rejected FK samples, uniform fallback samples and covariance support ellipsoids. Data are request-scoped, bounded, transient-local, and remain visible for up to 30 seconds after release. Match `gmm_cutoff` and `gmm_covariance_floor` on `lfd_launch.py` to custom sampler settings so original/deformed converter geometry agrees.

`/gmm_sampling/paths` restores the successful planned end-effector path display in `sampling.rviz`: green for `cartesian_ik`, blue for `joint_projected`. The runner publishes only after successful constrained planning/execution. It retains the latest path per mode while the runner is alive, including for late RViz subscribers; paths are not a history of trials and are not measured execution traces. Existing RViz sessions can add a MarkerArray display for this topic with Transient Local durability.

Both displays default to enabled and have independent switches:

```bash
# Keep successful paths, hide sampled points and support markers:
ros2 run tp_gmm run_experiments.py --no-sample-viz
# Keep samples, hide successful paths:
ros2 run tp_gmm run_experiments.py --no-path-viz
# Disable both for timing runs:
ros2 run tp_gmm run_experiments.py --no-sample-viz --no-path-viz
```

Path visualization performs FK after the action succeeds, with joint interpolation at at most 0.02 radians in joint-vector distance (for Panda). Its additional wall time is outside MoveIt's planning time. Work is bounded to 2000 points and 10 seconds; visualization errors warn without changing the motion result. Applications can reuse `PathVisualizer(node, namespace='').publish(result.planned_trajectory, result.trajectory_start, link_name, frame_id, mode)` after a successful result. Keep the visualizer alive to retain its transient-local paths, and call it outside callbacks already spinning the node.

```bash
ctest --test-dir build/tp_gmm -R sampling_math --output-on-failure
# With the controller-free launch on the same ROS domain:
python3 src/tp_gmm/tests/test_sampling_protocol.py
python3 src/tp_gmm/tests/test_sampling_runtime.py
```

These are functional regression checks, not comparison experiments. They verify malformed-model rejection, immutable request isolation/release, actual allocation of both methods, contained raw proposals and collision/corridor validity along returned path interpolation. No new battery assets or benchmark algorithms are assumed.
