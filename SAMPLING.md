# GMM sampling with Codex

This implementation starts at `gazebo_exps` (`56c6a33`), on `sampling-approaches-cod`. The original `sampling-approaches-gem` commits are unchanged. `moveit2` and `moveit_resources` also use `sampling-approaches-cod`, based on their pre-sampling commits `c154f941c` and `683e406`. Other repositories need no changes.

## Build and launch

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select moveit_core moveit_ros_planning moveit_planners_ompl tp_gmm moveit_resources_panda_moveit_config --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Restart MoveIt and TP-GMM after rebuilding: the deformation service has additional response fields and the sampler library has changed. The shared build/install tree reflects the currently built branch; switching Git branches alone does not switch installed libraries.

Launch a controller-free planning laboratory, with RViz and TP-GMM/RL:

```bash
ros2 launch tp_gmm sampling_demo.launch.py \
  policy_ckpt_path:=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-03_20-31-51_ppo_torch/checkpoints/best_agent.pt
```

This launch supplies robot states and MoveIt planning services, with trajectory execution disabled. Use `rviz:=false` for headless runs. A missing checkpoint is reported; the benchmark refuses to call it an RL-deformed experiment unless `--allow-no-policy` is explicitly given. Use the same `ROS_DOMAIN_ID` in every terminal; a separate domain lets the laboratory run alongside another ROS setup.

For the existing Gazebo execution pipeline, launch its normal Gazebo and `lfd_launch.py` nodes. The updated Panda `demo.launch.py` loads the sampler and selects `sampling.rviz` by default:

```bash
ros2 launch moveit_resources_panda_moveit_config gazebo_sim.launch.py
# Another terminal:
ros2 launch tp_gmm lfd_launch.py
# Another terminal; this existing experiment pipeline DOES execute robot trajectories:
ros2 run tp_gmm run_experiments.py --trials 5 --sampling-mode cartesian_ik
```

## Change sampling approaches

Before starting, pass `--sampling-mode cartesian_ik`, `--sampling-mode joint_projected`, or `--sampling-mode uniform` to `run_experiments.py`. During an experiment batch:

```bash
ros2 param set /gazebo_experiment_runner sampling_mode joint_projected
ros2 param set /gazebo_experiment_runner sampling_mode cartesian_ik
```

The next plan uses the new mode; an in-flight request retains its original model and settings. Invalid modes are rejected. The parameter is serviced while the runner spins ROS callbacks; manual prompts or other blocking work can delay its acknowledgement.

Pass `--sampler-config src/tp_gmm/config/sampling.json` to either runner to override defaults. The file is a JSON object; omitted keys retain defaults.

| Setting | Default | Meaning |
|---|---:|---|
| `cutoff` | 3 | Reject standard-normal position draws with radius greater than this |
| `covariance_floor` | 1e-8 m² | Small eigenvalue floor, used identically for sampling and corridor geometry |
| `uniform_fraction` | 0.1 | Probability of using MoveIt's ordinary constrained sampler |
| `cartesian_fraction` | 0.1 | In projected mode, Cartesian + IK fraction of remaining proposals |
| `ik_timeout` | 0.005 s | Per IK call budget |
| `branches` | 3 | Maximum distinct anchors per Cartesian component |
| `anchor_attempts` | 16 | Maximum nominal IK attempts per component |
| `nullspace_stddev` | 0.08 rad | Standard deviation in an orthonormal position-null basis |
| `max_joint_delta` | 0.6 | Maximum Euclidean joint displacement from an anchor |
| `linearization_tolerance` | 0.01 m | Maximum measured nonlinear FK residual |

The optional `config/sampling_pure.json` profile disables the two explicit exploration fractions. To isolate the custom proposals, set `uniform_fraction` and `cartesian_fraction` to zero. Unanchored components still use Cartesian IK rescue and are counted in `missing_anchor_fallbacks`; confirm that this and `online_ik_calls` are zero before describing a projected trial as having no online IK. OMPL also has its own uniform state-space fallback after repeated complete sampler failures; `failed_calls` exposes those failures. Successful-but-narrow proposals therefore still benefit from the explicit exploration fraction.

## Quick paired comparison

With the planning laboratory or the existing MoveIt + TP-GMM nodes running:

```bash
ros2 run tp_gmm compare_sampling_approaches.py \
  --repeats 3 --warmup 1 --planning-time 3 \
  --output sampling_results/cod-comparison
```

The default three cases are open space, a central cylinder, and an offset cylinder. Edit `config/comparison_cases.json` or pass `--cases YOUR_CASES.json`. Each case uses a table plus a cylinder. Cylinder coordinates denote its centre; the policy receives the cylinder top, following the existing pipeline convention.

The runner:

- Performs endpoint IK once per case, then reuses exactly the same complete start state and joint goal across modes.
- Applies an acknowledged planning-scene update and prepares one reproduced/deformed GMM per pair. The exact model, its hash, and all proposal settings are saved.
- Randomizes mode order and records sampler seeds, uses RRTConnect with one planning attempt, and performs plan-only requests. It never sends execution goals or changes Gazebo entities.
- Checks both endpoints against the shared corridor before planning. An invalid corridor endpoint becomes an explicit failure, never an unconstrained retry.
- Restores the benchmark's two collision objects on exit, including any pre-existing objects with those IDs. Existing unrelated scene objects are retained and can influence results.
- Saves after every trial. Transport/configuration errors abort the run to avoid continuing with an uncertain server state.

Outputs:

- `results.json`: exact models, full trajectories/start states, pipeline stages, per-request counters, per-component proposal/FK moments, distribution checks and aggregate statistics.
- `trials.csv`: tabular timings, success/failure, efficiency, path lengths and clearance.
- `summary.md`: success counts, median/p95 action times, acceptance rates and metric caveats.
- `comparison.png` / `comparison.pdf`: six panels covering success, timing, clearance, path length, efficiency and sampler cost.

Regenerate the figure without ROS:

```bash
python3 src/tp_gmm/scripts/plot_sampling_comparison.py sampling_results/cod-comparison/results.json
```

Success requires both MoveIt success and a passing interpolated path audit. Raw `planner_success` and `path_audit_passed` are retained separately; post-plan audit failures are explicit failures. Panda edge checking uses `longest_valid_segment_fraction: 0.0005` equally for all modes. Success rates include all measured trials; warmups are excluded. Timing summaries distinguish all requests from successful requests, and path-quality statistics only use successes. Wilson 95% intervals indicate how uncertain a short run's success estimate is. Run more repetitions and harder cases before drawing performance conclusions. No superiority claim is built into the report.

## RViz inspection

The `sampling.rviz` configuration enables two MarkerArray displays:

- `/gmm_sampling/markers`: cyan task-space proposals, green valid FK samples, red FK samples rejected by projection or validity checks, yellow valid uniform-fallback samples, and translucent purple cutoff ellipsoids.
- `/gmm_sampling/paths`: latest returned paths per mode: Cartesian green, projection blue, uniform orange. A path that fails the interpolated audit is red.

Run the comparison with `--visualize --repeats 1 --warmup 0`. For the executing experiment pipeline, sample visualization is enabled unless `--no-sample-viz` is passed. Expand the marker display's namespaces to isolate a request or proposal stage. Sample buffers contain the latest 1,000 points per stage, publish at 10 Hz, and remain visible for 30 seconds after the request is released. Fast plans also publish a final snapshot. Paths persist until replaced for their mode. For timing runs, keep visualization off.

Cyan points in projected mode are the *linear task-space draws*, before nonlinear FK; green/red points show actual FK. Red points do not cover IK failures or rejected joint vectors for which FK was not computed. Neither point cloud is the RRT tree: steering and interpolation generate additional planner vertices.

Visual containment alone does not establish distribution fidelity. `distribution_checks` in the JSON compares empirical per-component means/covariances with the analytically truncated Gaussian, with sample counts reported. Low counts are noisy. The exact proposal reference is appropriate for Cartesian draws and projected linear draws; projected FK and accepted states are expected to differ because of nonlinearity, IK, bounds and collision conditioning.

## Mathematics and scope

The spatial marginal is extracted from either `[x,y,z]` or row-major `[time,x,y,z]` messages. The whole model is validated before registration: dimensions, finite values, positive-semidefinite spatial covariances and nonnegative weights with positive total. Covariance symmetrization is alias-safe; materially negative eigenvalues are rejected. The canonical eigensystem is used for every downstream representation.

Cartesian proposals select `k ~ Categorical(normalized weights)` and draw `x = mu[k] + L[k] z`, rejecting until `||z|| <= cutoff`. There is no radial clipping or joint-limit clamping. For cutoff 3 in 3-D the retained probability is approximately 97.1%, and the retained covariance is approximately `0.917807 * Sigma`. Each target gets a randomized IK seed and the configured orientation bias. Joint bounds, all configured path constraints, feasibility, collisions and any inherited validity callback are checked before reporting a valid target.

Projection solves several diverse IK anchors per component and verifies their FK residual. An SVD gives a full-row-rank right inverse `A` and an orthonormal null basis `Z`, with `J A = I` and `J Z = 0`. A draw is

```
q = q_anchor + A L z + nullspace_stddev Z u
```

Before truncation, the local joint covariance is `A Sigma A^T + sigma_null² Z Z^T`. Singular anchors are excluded rather than approximated with task-space-leaking damped null noise. Position null space does not preserve orientation, and finite null-space steps still cause nonlinear position drift. Bounds, trust-region displacement, actual FK Mahalanobis support, and measured linearization residual are checked by rejection. This is a **local joint proposal**, not an exact inverse of the Cartesian GMM. Its covariance and accepted distribution must be assessed empirically.

Known fixed GMM frames, including a rotated/translated fixed robot base, are supported. IK targets transform into the model frame; FK and Jacobians transform into the GMM frame. Moving GMM frames are rejected. This version constrains the named link origin (zero tool-point offset) and exposes position corridors; the quaternion is a proposal bias, not a hard path orientation constraint. The validated robot/group is the fixed-base Panda `panda_arm` / `panda_hand`.

The hard corridor is a union of oriented enclosing boxes with full dimensions `2 * cutoff * sqrt(eigenvalue)`. Weights affect selection probability only. Boxes enclose ellipsoids; they are not an exact ellipsoidal constraint. The `uniform` baseline wraps MoveIt's unmodified IK constraint sampler: primitive indices and orientations retain its standard policy, so this baseline differs in more than just the positional proposal. Both GMM modes share the same configured orientation/exploration policy.

## Request ownership and timing

`/gmm_sampling/prepare` returns an immutable model/corridor/settings snapshot and a `tpgmm:<request_id>` constraint name. The allocator only services that prefix. IDs are unique across server restarts; unknown IDs or changed corridors fail rather than falling back silently. The caller uses the deformation *service response*, never a latest-model topic or marker-derived boxes.

`/gmm_sampling/report` is queried after the action result. Counters from all sampler instances are aggregated by request. Reporting does not depend on destructor timing or late DDS statistics messages. Completed requests are released; the store is bounded to 64 requests, and abandoned, unreferenced requests expire after ten minutes. Active samplers own their snapshots until MoveIt releases them.

Timing is nested:

- `model_load_s`, `reproduction_s`, `rl_deformation_s`, `deformed_regression_s`: disjoint stages inside the TP-GMM service; CUDA is synchronized for inference timing. Deformation conversion does not rewrite a rosbag.
- `prepare_service_wall_s`: model/corridor registration round trip.
- `setup_s`: sampler configuration, including `anchor_ik_s` and `jacobian_setup_s` in projected mode.
- `sampling_s`: custom sampling loop, including `draw_s`, `online_ik_s`, `fk_mapping_s`, `validity_s`, and `uniform_sampler_s`. These sub-timers are not exhaustive and must not be added to their parent.
- `moveit_planning_s`: MoveIt's reported planning time. `action_wall_s` includes request/action processing and response adapters.
- `end_to_end_wall_s`: attributed deformation-service wall time + preparation/endpoint preflight + action latency for that mode. Common endpoint IK and post-plan metric evaluation are excluded. Since deformation is shared by a pair, this attributed value is not total benchmark runtime.

Whole-robot obstacle clearance uses MoveIt's padded world-distance query and its allowed-collision matrix; it excludes self-distance. A negative backend collision sentinel is reported as zero clearance, not as a penetration depth. It is measured on interpolated states at at most 0.02 MoveIt group-distance increments and is a sampled estimate. `path_invalid_samples` separately checks collisions, constraints and feasibility on those states. Joint path length uses MoveIt's group distance (Panda revolute joints); end-effector length is in metres. Telemetry counts returned valid targets, not inserted RRT vertices or all of OMPL's collision checks. Missing trajectory metrics are unavailable, while applicable zero counters/timers are explicit zeros.

Sampler and randomized IK-seed generators have per-instance seeds. OMPL, the fallback sampler and the IK plugin can have independent internal RNGs; identical custom seeds do not promise bit-identical planning results.

## Tests

```bash
ctest --test-dir build/tp_gmm -R sampling_math --output-on-failure
ROS_LOG_DIR=/tmp/tpgmm-tests python3 -m unittest discover -s src/tp_gmm/tests
# Requires the plan-only laboratory on the same ROS domain:
python3 src/tp_gmm/tests/test_sampling_protocol.py
```

The mathematical test covers covariance factorization/symmetrization, malformed covariance rejection, a 150,000-draw truncation check, Mahalanobis coordinates, and SVD task covariance/null-space identities. Protocol tests cover malformed arrays/NaNs/indefinite covariance, transactional registration, interleaved model identities, released IDs and late-joining RViz marker delivery. Reporting tests cover failed trials, warmups, zero timings and absent metrics. See the workspace validation output for measured planning results.

The live sample-stream integration check also runs nine plan-only requests, verifies raw sample containment against their request-matched RViz ellipsoids, and checks that all three path displays arrive:

```bash
python3 src/tp_gmm/tests/test_sampling_visual_stream.py --output sampling_results/visual-check
```
