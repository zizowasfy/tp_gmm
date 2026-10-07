# Covariance cutoff 3.0 on the original-height scenes

Protocol fixed on 29 September 2026 before the new measured plans. This is a paired sensitivity rerun of the original 26 September cutoff-2.0 case study, not new independent confirmation.

- Reuse all 120 original saved environments: the primary cohort has 60 environments × 10 repeats × 3 methods (1,800 outcomes); the second cohort has 60 × 5 × 3 (900). Total 2,700 outcomes. No outcome-based extension or exclusions.
- Preserve the exact original table (surface z=0.25 m), cylinder poses/dimensions, start/goal joint states and original goal-height range (z=0.45–0.54 m). Requested RL clearances remain 0.2 and 0.8 with their original environment assignment. This is not a within-environment clearance sweep.
- The only intentional planning change is covariance cutoff **2.0 → 3.0** for both custom methods. Both the proposal truncation and the covariance-derived hard box corridor grow. Component means, covariances and weights are unchanged; no covariance retraining or scaling is performed. Consequently this does not isolate proposal density from feasible-region size.
- Keep the original checkpoint/action scale and runtime binaries. Recompute TP-GMM/RL preparation once per environment and verify both original and deformed GMM payloads match the saved source, ignoring only header timestamps. Preserve the full scenes and exact joint endpoints. Record hashes and perform live scene read-back verification before planning.
- Pure Cartesian + IK and pure joint projection: `proposal=gmm`, covariance corridor, zero uniform/cartesian mixture fractions, strict MoveIt fallback disabled. Verify recorded component cutoffs equal 3.0, no uniform attempts, no missing-anchor rescues, and no projected online IK. Run the exhausted-sampler runtime regression at cutoff 3.0 before measurements.
- Unrestricted OMPL retains empty path constraints and bounded uniform joint-space sampling. Its problem has no cutoff parameter; any before/after differences are stochastic rerun variability.
- Keep RRTConnect, one planning attempt, 3-second budget, no simplification, original custom seeds and method/repeat order, and the same dense returned-path audit. Visualization off during timed measurements; controller-free and plan-only in isolated ROS domain 81. Do not overlap the cohorts or run statistical analysis/builds during timing.
- Primary cutoff-sensitivity family: three two-sided environment-level paired success sign-permutation tests (one per sampler), Holm correction at 0.05. Give each source environment equal weight across cohorts; retain repeats within clusters. Use 20,000 bootstrap replicates stratified by cohort, obstacle layout and clearance, and 100,000 sign permutations (exact for at most 16 nonzero differences). Pointwise 95% intervals are not multiplicity-adjusted. Report raw counts separately from environment-weighted estimates.
- Secondary: failure stages, PAR2 (failure costs 6 s), action/pipeline time, preparation and nested sampler timers, efficiency, path lengths and obstacle clearance. Compare path quality only on matched successful requests and report survivor selection. Include descriptive results per cohort and clearance. Original within-cohort method comparisons are reproduced as sensitivity analyses; reused environments cannot provide another independent confirmation. Host timing drift and independent OMPL/IK RNG streams limit causal runtime attribution.

From the sourced workspace root, use the original checkpoint and isolated `sampling_demo.launch.py` laboratory, then run sequentially:

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
```

`--design-only` freezes inputs without measured plans. `--resume` requires the same source, cutoff, clearance mapping and runtime. Existing studies are not overwritten. Analyze only after both cohorts finish.
