# RL clearance generalization experiment

`run_clearance_sweep.py` tests how the retrained policy deforms the same reproduced TP-GMM as requested clearance changes. It is **plan-only**: no Gazebo entities or robot controllers are commanded. It adds the table and cylinder to MoveIt's planning scene and restores its own objects on exit. Use an isolated planning laboratory for evaluation; other scene objects remain active and are recorded in the results. Do not run concurrent scene-changing experiments on the same ROS domain.

Each trial samples one environment (start, goal, cylindrical obstacle), rejects unreachable/colliding endpoint IK, then fixes both endpoint joint states for the complete sweep. The default is **ten clearance values, 0.1 through 1.0**. `--include-zero` adds the 0.0 reference for eleven values. Clearance order and sampler order are shuffled; one deformation response is shared by all samplers at each level. Candidate rejection reasons are retained, and the generator aborts explicitly if it cannot find a valid environment. It never resamples based on planning success.

## Run

Build once, then source the overlay in every terminal. The deformation service now also returns request-matched original/deformed DSGMR trajectories and policy provenance, so restart the TP-GMM service after building.

```bash
cd /home/zizo/the_folder/ws_moveit
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select tp_gmm --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

For a separate controller-free laboratory, use the same isolated domain in both terminals:

```bash
export ROS_DOMAIN_ID=81
ros2 launch tp_gmm sampling_demo.launch.py rviz:=false \
  policy_ckpt_path:=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt
```

Second terminal, after sourcing the overlay:

```bash
export ROS_DOMAIN_ID=81
# 5 random environments × 10 clearance values = 50 plans.
ros2 run tp_gmm run_clearance_sweep.py \
  --trials 5 --sampler cartesian_ik --seed 42 \
  --output clearance_results/cartesian

# Paired comparison: 5 × 10 × 3 = 150 plans.
ros2 run tp_gmm run_clearance_sweep.py \
  --trials 5 --samplers cartesian_ik joint_projected ompl_uniform \
  --seed 42 --planning-time 5 --output clearance_results/paired

# Add zero clearance; isolate pure Gaussian/projection sampling with the existing profile.
ros2 run tp_gmm run_clearance_sweep.py \
  --trials 5 --sampler joint_projected --include-zero \
  --sampler-config src/tp_gmm/config/sampling_pure.json \
  --output clearance_results/projected_with_zero
```

The Gazebo/MoveIt + `lfd_launch.py` setup can also supply the required services. In that case use its ROS domain and restart the TP-GMM launch to load the expanded service. The script changes planning-scene geometry only; Gazebo visuals/physical obstacle placement are not synchronized by this analysis runner. `--visualize` enables sampler markers for the two GMM approaches; `ompl_uniform` does not publish custom sampler clouds. Static figures are generated regardless of this flag.

`ompl_uniform` is MoveIt's OMPL planner with **empty path constraints**, bypassing the GMM allocator. On the Panda configuration this uses the joint-model state space over robot joint bounds. It is not uniform over Cartesian volume, and it is different from the existing `uniform` mode, which samples within the GMM's box corridor. Clearance does not affect the OMPL baseline request, so fluctuations in that baseline reflect stochastic planning, not RL clearance control. Custom sampler timing counters are unavailable for that baseline; common action/FK/path metrics remain available.

## Design and metric definitions

- The scalar supplied to the policy is normalized in [0, 1], not metres. The current training code uses `physical_margin = clearance × 0.30 m`, plus a base buffer of `0.08 m`. Override `--max-clearance-margin` and `--base-buffer` if the selected checkpoint was trained differently. These arguments only interpret the target in analysis; the policy receives the original normalized scalar.
- The training clearance reward compares the **3-D distance from intermediate Gaussian means to the policy obstacle reference point**, minus obstacle radius, against `0.08 + clearance × 0.30`. The first/last components are excluded. This diagnostic reproduces that part of the reward, not its consistency/smoothness/endpoint terms.
- `--obstacle-reference top` matches the current executing runner's obstacle-top convention. `--obstacle-reference center` supplies the cylinder center instead. The training environment uses its obstacle root position, so choose the convention corresponding to the checkpoint and intended deployment. The exact reference point is recorded.
- Actual clearance is the **signed Euclidean distance to the finite vertical cylinder surface**, including its side and end caps. Negative means the point is inside the cylinder. It measures the end-effector link origin (`panda_hand`) or a DSGMR trajectory point; it is not whole-robot surface clearance.
- Original and deformed DSGMR curves come from the same service response as their GMMs. They are policy/reproduction outputs, not robot trajectories. Planned curves are computed using FK along joint interpolation, with validity checks against the full planning scene and the relevant path constraints.
- FK/validity sampling defaults to at most `--joint-step 0.02` joint-vector norm between states. All xyz polylines are subdivided at `--curve-step 0.002` m before measuring surface distance. This catches crossings between input curve vertices, but it is not a continuous collision proof for the nonlinear robot motion between FK samples. Analysis time is measured separately from planning.
- Full-curve minimum, 5th percentile, median, path length, closest point, endpoint error, and an additional interior minimum are saved. The interior measurement excludes each endpoint's `--trim-fraction 0.1` of arc length. Full minima remain available, because endpoints can legitimately limit achievable clearance.
- Policy mean displacement, target violation, deformation gain relative to the prior, success/failure stage, action/planner latency, reproduction/RL stage times, joint/EE path lengths, sampled validity, and available sampler/IK/Jacobian counters are recorded. Failed audits remain diagnostics and are excluded from successful-path clearance aggregates.
- The reported end-to-end time attributes the shared deformation time to each mode and excludes analysis queries/plotting. Nested sampler sub-timers must not be summed with their parent timers.

## Reproduce and present

Default environment ranges are explicit in `config/clearance_environment.json`; start/goal orientations are fixed downward. `--environment-config` accepts overrides to position, obstacle fraction/offset, radius/height, and endpoint separation ranges. The sampling domain is an evaluation choice, not an assertion that it matches the training distribution. Seeds reproduce the environment and order generators; OMPL and IK can have separate RNGs.

Replay the same accepted pose environments:

```bash
ros2 run tp_gmm run_clearance_sweep.py --trials 5 \
  --sampler joint_projected --environments clearance_results/cartesian/environments.json \
  --output clearance_results/replay_projected
```

The pose environments are replayed; IK is solved again. Within every run, exact start/goal joint states are fixed across all levels and saved in metadata. Use a single `--samplers ...` invocation for the strongest paired comparison.

Outputs are incrementally saved after each completed plan:

- `results.json`: complete run, exact endpoint states, scene snapshot, rejected candidates, model hashes, policy path/hash/action scale, curve samples, planning results, summaries and run status.
- `models/*.json`: exact original/deformed Cartesian GMM pairs indexed by hash.
- `environments.json`: replayable accepted environments.
- `trials.csv`: scalar comparison and timing metrics; absent metrics are blank.
- `trajectory_points.csv`: xyz, normalized arc length and signed obstacle-surface distance for each curve.
- `summary.md`: success counts, paired monotonicity and interpretation notes.
- `figures/clearance_overview.{png,pdf}`: clearance response, policy target tracking, planning success/time and path length. Median lines and interquartile bands describe the variation between environments.
- `figures/paired_environment_response.{png,pdf}`: individual environmental response curves and improvement over the undeformed prior.
- `figures/environment_*`: three-dimensional prior/deformed/planned overlays plus distance-along-path profiles, colored by requested clearance. The first three environments are rendered by default.

Regenerate figures offline, including more environment examples:

```bash
python3 src/tp_gmm/scripts/plot_clearance_analysis.py \
  clearance_results/paired/results.json --max-examples 10
```

The summary's nondecreasing fraction uses only adjacent observed clearance levels and a 2 mm tolerance. It never bridges missing levels. Failure rates must be presented alongside successful-path metrics to avoid implying that large requested clearances always produce a feasible plan. A small sweep demonstrates behavior in its sampled environments; it does not establish generalization to arbitrary environments.

## Checks

```bash
python3 -m unittest discover -s src/tp_gmm/tests -p 'test_clearance*.py'
```

The tests cover signed finite-cylinder distances, segment crossings, training-reference geometry, reproducible environment generation, clearance levels, treatment of missing/failed levels, and the plan-only unconstrained OMPL request contract.
