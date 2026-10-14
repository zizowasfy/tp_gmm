# Strict GMM/GMR comparison — benchmark-aligned protocol

The historical **5 October four-method** results and full raw-data archives are in [case_studies/2026-10-05-gmr](case_studies/2026-10-05-gmr/README.md), with [findings](case_studies/2026-10-05-gmr/FINDINGS.md), figures and checksums. The historical pilot remains a separate archive.

The current fixed design compares **GMM**, **GMR 30 mm** and **reference_ik**, with clearance input **0.7**, GMM covariance cutoff **3.0**, and GMR radial cutoff **3.0**. GMR's standard deviation is **0.030 m** along each Cartesian axis, with radial rejection at 0.090 m; this is an isotropic neighborhood of the post-RL DSGMR polyline, not a learned conditional GMR covariance. Hybrid is retained only in historical archives/API compatibility. Random proposals use Cartesian + IK mapping. GMM uses a hard ellipsoid union; GMR and reference IK use the endpoint-connected 90 mm reference tube. No uniform component or fallback is allowed, including MoveIt's outer constraint-sampler wrapper.

## Design fixed before results

- 120 saved randomized environments: 40 central, 40 offset, 40 distant cylinder layouts, using the established `clearance_analysis.DEFAULT_ENVIRONMENT` ranges and original table (top 0.25 m). Five repeats per environment and method: 1,800 measured outcomes.
- Three original pilot scenes, ten repeats each and method: 90 measured outcomes, analyzed separately as descriptive replication. Nine excluded warmups (each original scene × method) precede all measured trials.
- Seed 20261005. Environment candidates are frozen before evaluating feasibility. Endpoint IK failures remain failures for all methods; no outcome-based resampling. Method order is randomized within environment/repeat; environment order randomized. OMPL and IK have independent RNG streams, so repeats do not represent common random numbers.
- Same exact joint endpoints, collision scene, RL-deformed GMM and DSGMR reference are reused across methods/repeats. Deformation is performed once per environment; its measured cost is attributed to each method's pipeline cost. This measures planning variation conditional on that deformation, not RL inference variance.
- Original pilot checkpoint `2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt`, SHA256 `d9ec51ff464b93f8ad87e4d5dc2b3bb649f89dbb5fa4155b5df46ed788d2e5ba`, action scale 0.15. Raw regression outputs are preserved; actual FK endpoints are connected for GMR support and reference IK.
- RRTConnect, single attempt, 10 s planning budget, range 0.0 (automatic), simplification disabled, longest-valid-segment fraction 0.00005. Reference IK keeps the pilot algorithm and 10 s budget, including public IK/FK/validity-service overhead; it has no search or repair. Its geometric path has no time parameterization. Runtime differences involving it therefore also reflect implementation architecture.
- Every successful path receives the same offline dense C++ joint-space audit (joint-vector Euclidean step 0.005 rad), including self/world collisions, bounds and hard corridor. Geometry is verified by scene readback before use. Dense checks are discrete, not a proof of continuous collision freedom.
- No execution, controllers or Gazebo changes. Visualizations disabled during timing.

## Prespecified analysis

The independent unit is the randomized environment, not the individual planning request. Use 20,000 layout-stratified environment bootstrap resamples for 95% confidence intervals and 100,000 paired environment sign permutations (exact enumeration for ≤16 nonzero differences). Primary family: all three method pairs × success and PAR2 (six two-sided tests), Holm correction. PAR2 uses successful action wall time capped at 10 s, and 20 s for any failure, including endpoint/preflight failures.

Secondary quality family: one GMM/GMR proposal pair × joint length, EE length, minimum world clearance, and mean reference deviation (four Holm-adjusted tests), evaluated only on paired successful repeats then averaged per environment. Report conditioning and sample counts. Baseline quality is descriptive if successes are sparse. Runtime, timing breakdown, EE-cylinder clearance, sample acceptance, failure stages and reference endpoint errors are additional descriptive diagnostics. No sample-size extension based on observed significance. No pooling with historical pilot data, which predates strict fallback enforcement.

Clearance 0.7 is the policy's normalized input, not a guarantee of 0.7 m geometric separation. Reference distance is measured on the complete end-effector path, including connectors from/to the actual endpoints. The original goal-height range remains 0.45–0.54 m; claims concern this test distribution, not all reachable tasks or the EV battery benchmark.

The previous four-method results remain historical. See [framework alignment](FRAMEWORK_ALIGNMENT.md) for settings, preserved cohorts and interpretation limits.

## Reproduce

From the workspace root, in an isolated ROS domain:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=83
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch tp_gmm sampling_demo.launch.py rviz:=false \
  policy_ckpt_path:=/home/zizo/the_folder/Reach_direct/logs/skrl/cartpole_direct/2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt
```

In another terminal with the same environment:

```bash
python3 src/tp_gmm/tests/test_strict_sampling_runtime.py --cutoff 3.0
python3 src/tp_gmm/scripts/rerun_framework_study.py --study gmr \
  --source gmr_results/2026-10-05-extensive \
  --output gmr_results/2026-10-13-framework
```

`--design-only` freezes the inputs without measuring. `--resume` checks source/binary hashes, runtime parameters and design settings before appending missing trial keys. Infrastructure or purity errors stop the run; they are not silently counted as planner failures. The append-only JSONL log, manifest, full frozen inputs and source snapshot preserve provenance.
