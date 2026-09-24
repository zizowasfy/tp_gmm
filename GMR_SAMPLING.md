# GMR reference-path sampling

The `sampling-approaches-cod` branch supports three independent proposal choices with Cartesian IK: `gmm` (existing default), `gmr_path`, and `hybrid`. The reference is the request-matched **post-RL DSGMR rollout**, not a line through component means or the instantaneous GMR conditional mean. No model or policy retraining is required.

The first completed experiment is recorded in [GMR_PILOT.md](GMR_PILOT.md).

## Distribution and constraints

For `gmr_path`, choose a position uniformly along reference arc length, then add `gmr_stddev * z`, where `z` is a standard isotropic 3-D normal conditioned on `||z|| <= gmr_cutoff`. Default standard deviation is 0.01 m; the provided presets use 0.015 m. At cutoff 2, the preset's maximum displacement from its selected reference point is 0.03 m. This is not a claim that 95% of a 3-D Gaussian is inside that radius. Standard deviation zero is an exact-reference diagnostic option.

`hybrid` chooses the GMR proposal with probability `gmr_fraction` (default 0.8), otherwise the original GMM proposal. This choice follows the existing `uniform_fraction` fallback, so nonzero uniform mixing changes the overall proportions. Both GMR presets and the paired pilot disable that extra fallback for a clean ablation.

**The original GMM-derived hard corridor remains unchanged.** The reference neighborhood is a proposal, not an additional constraint or a replacement corridor. IK seeds, collision checks, joint limits and corridor checks match Cartesian GMM sampling. Proposals outside the corridor are rejected; the reference itself need not lie wholly inside it. The resulting motion can deviate from the reference because RRT connections and simplification operate in joint space.

The full 3-D reference covariance and orientation are not used. Path `PoseArray` orientations are ignored; the request's fixed proposal orientation is used for IK. GMR path proposals with `joint_projected` or `uniform` mapping are explicitly rejected in this first implementation. Selecting `proposal=gmm` keeps those existing modes available, including projected-only behavior when `cartesian_fraction=0`.

## Modules for reuse

| File/module | Role |
|---|---|
| `include/tp_gmm/path_proposal.hpp` | ROS-independent arc-length interpolation and point-to-polyline distance |
| `src/sampling/gmr_path.cpp` | Truncated GMR neighborhood proposal and Cartesian IK dispatch |
| `src/sampling/cartesian_ik.cpp`, `joint_projected.cpp` | Existing GMM mapping implementations |
| `src/sampling/sampler.cpp` | Proposal routing, shared validity and counters |
| `src/sampling/request_registry.cpp` | Immutable model/reference validation and session lifecycle |
| `src/sampling/reporting.cpp`, `visualization.cpp` | Path audits, reference deviation, optional RViz markers |
| `python/tp_gmm_sampling/client.py` | Importable client and exception-safe session context manager |
| `python/tp_gmm_sampling/reference_baseline.py` | Optional direct-reference IK feasibility baseline; never executes |
| `scripts/compare_gmr_proposals.py` | Paired experiment adapter, separate from the sampler |

Reference paths must contain 2..4096 finite positions in **exactly the model frame**, with positive total length. Consecutive duplicates are removed. Mismatched frames, missing paths and unsupported mappings fail explicitly. Path geometry is copied and prepared once per request; no sampling-time subscriptions to latest visualization topics are used.

```python
from tp_gmm_sampling import SamplingClient

client = SamplingClient(node)
with client.session(
    response.deformed_gmm, group, link, orientation,
    mode='cartesian_ik', proposal='gmr_path',
    reference_path=response.deformed_trajectory,
    gmr_stddev=0.015, gmr_cutoff=2.0, uniform_fraction=0.0,
    visualize=True, visualize_paths=True,
) as prepared:
    goal.request.path_constraints = prepared.constraints
    result = send_and_wait(goal)  # application-owned; wait for completion/cancellation
    metrics = client.report(prepared.request_id, result, release=False)
```

These synchronous helpers spin the caller node. Use them outside callbacks already spinning that node, or use the underlying ROS services asynchronously. Other benchmark applications can supply their own reference polyline and GMM; the plugin has no TP-GMM, policy, task, robot-name or RRT dependency.

## Build and use

The `PrepareSampling` interface changed. Rebuild and restart MoveIt and all clients after switching branches. The trained files and checkpoint on this experimental branch are used as-is.

```bash
cd /home/zizo/the_folder/ws_moveit
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --packages-select moveit_ros_planning moveit_planners_ompl tp_gmm moveit_resources_panda_moveit_config \
  --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

With the normal Gazebo pipeline running, these commands **execute robot motion**:

```bash
ros2 run tp_gmm run_experiments.py --sampling-mode cartesian_ik \
  --sampler-config src/tp_gmm/config/sampling_gmr.json
ros2 run tp_gmm run_experiments.py --sampling-mode cartesian_ik \
  --sampler-config src/tp_gmm/config/sampling_gmr_hybrid.json
# CLI overrides are available independently of JSON:
ros2 run tp_gmm run_experiments.py --sampling-mode cartesian_ik \
  --sampler-config src/tp_gmm/config/sampling_pure.json \
  --sampling-proposal gmr_path --gmr-stddev 0.015 --no-sample-viz
```

`--no-sample-viz` controls proposal/support markers. `--no-path-viz` independently disables successful path markers. Existing runs keep `gmm` as their default proposal. The clearance sweep can use either preset via its existing `--sampler-config` option, selecting `--samplers cartesian_ik`.

## Plan-only comparison

Use a separate ROS domain so that scene updates do not affect a live Gazebo session. The launch has no execution controllers. Use the same domain in both terminals:

```bash
export ROS_DOMAIN_ID=81
ros2 launch tp_gmm sampling_demo.launch.py rviz:=true \
  policy_ckpt_path:=/absolute/path/to/checkpoints/best_agent.pt
# Second terminal after sourcing the workspace:
export ROS_DOMAIN_ID=81
ros2 run tp_gmm compare_gmr_proposals.py \
  --gmr-stddevs 0.005 0.015 0.03 --gmr-fraction 0.8 \
  --repeats 5 --warmup 1 --planning-time 3 \
  --output gmr_results/my_run
```

Each case/repeat shares endpoint joint states and one TP-GMM/RL/DSGMR response across all variants. Variant order is shuffled. The script checks successful planner trajectories along joint interpolation and restores its scene objects afterwards. It requires the RL policy to be applied unless `--allow-no-policy` is explicitly selected. Random seeds cover proposal/IK seeds and trial order, not all internal OMPL/IK RNGs.

The direct-reference baseline sequentially solves IK along the reference, connects the actual start and fixed goal configuration, and audits all joint segments. It uses the same corridor. A partial path or invalid connector is a failure. Its public-service IK/FK/validity overhead is included in its budget; it is a feasibility baseline, not an equal-overhead comparison to in-process RRT. It has no trajectory timing or execution stage. Use `--skip-direct` to omit it.

Outputs include `results.json` (models, exact references, policy hash, trajectories, seeds and raw counters), `trials.csv`, `summary.md`, `comparison.png/pdf`, `gmr_diagnostics.png/pdf`, and `gmr_paths.png/pdf`. Plots can be regenerated offline with `plot_gmr_comparison.py results.json`.

The paired runner also records exact endpoint joint states and reference endpoint position errors. Metrics distinguish total pipeline latency, sampler preparation, draw/IK/validation costs, acceptance, joint/EE path length, sampled robot-world clearance, EE-obstacle clearance, and mean/max nearest-reference distance. The reported end-to-end latency ends at the planner result: it excludes post-hoc report audits, serialization, plotting and logging. The direct IK baseline necessarily includes its service-based audit. Nested timings must not be added together. Reference deviation is averaged over the joint-interpolation audit points; it is not a time-weighted tracking error. Whole-robot clearance is unavailable for the direct baseline. The plotted cylinder distances concern the link origin; neither those distances nor finite interpolation audits certify continuous whole-robot clearance.

## RViz and regression checks

`/gmm_sampling/markers` adds magenta GMR proposals, the reference line, and translucent spheres illustrating the local neighborhood. Original GMM proposals stay cyan; accepted/rejected FK samples stay green/red. The neighborhood spheres are illustrative, not collision geometry. `/gmm_sampling/paths` keeps separate latest paths for each mapping/proposal pair; different widths within the same proposal replace the previous path. Use Transient Local durability.

```bash
ctest --test-dir build/tp_gmm -R 'sampling_math|path_proposal' --output-on-failure
python3 -m unittest discover -s src/tp_gmm/tests -p 'test_gmr_client.py'
# With the isolated controller-free launch running:
python3 src/tp_gmm/tests/test_sampling_protocol.py
python3 src/tp_gmm/tests/test_gmr_runtime.py
```

The GMR reference is a learned preference. It may intersect obstacles, miss the requested endpoints or require an unavailable IK branch. The sampler does not guarantee reference following, improved planning, desired clearance or unrestricted-space probabilistic completeness. The current constrained fallback cannot explore outside the hard GMM corridor.
