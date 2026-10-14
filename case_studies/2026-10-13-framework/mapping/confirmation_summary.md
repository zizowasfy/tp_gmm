# Pure sampling case study results

60 source environments, 5 repeats per method, 900 request outcomes; fixed 10 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 300/300 (100.0%) | 100.0–100.0% | 0.1937 | 0.1823 | 0.7576 | 0.0132 |
| joint_projected | 296/300 (98.7%) | 97.0–100.0% | 0.4706 | 0.1865 | 0.6496 | 0.0130 |
| ompl_uniform | 300/300 (100.0%) | 100.0–100.0% | 0.2076 | 0.2070 | 2.2128 | 0.0185 |

Paired secondary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 1.3333 | [0.0000, 3.0000] | 0.841952 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.2770 | [-0.6181, -0.0119] | 0.141549 |
| cartesian_ik / ompl_uniform | success (pp) | 0.0000 | [0.0000, 0.0000] | 1 |
| cartesian_ik / ompl_uniform | PAR2 (s) | -0.0139 | [-0.0233, -0.0038] | 0.0824392 |
| joint_projected / ompl_uniform | success (pp) | -1.3333 | [-3.0000, 0.0000] | 0.841952 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.2631 | [-0.0047, 0.6069] | 0.841952 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0107 | 0.0105 | 0.0000 | 0.0000 | 0.0000 | 0.5184 |
| joint_projected | 0.0210 | 0.0002 | 0.0000 | 0.0208 | 0.0001 | 0.0000 | 0.1317 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004377 | 0.004370 |
| reproduction_s | 0.008237 | 0.008171 |
| rl_deformation_s | 0.000890 | 0.000797 |
| deformed_regression_s | 0.001746 | 0.001742 |
| total_s | 0.016885 | 0.016776 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a benchmark-framework alignment replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is an exact union of covariance ellipsoids; sampler targets, planner states and interpolated edges must satisfy it. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.

## Replayed confirmation cohort (sensitivity analysis)

This rerun reuses the original confirmation environments for benchmark-framework alignment, so it is not another independent confirmation. The original directional path-length tests are reproduced below as sensitivity analyses.

| Outcome (Cartesian minus projected) | Effect | 95% cluster CI | One-sided Holm p | Matched pairs / environments |
|---|---:|---:|---:|---:|
| joint_path_length | 1.32057 | [0.70319, 1.96095] | 0.000199998 | 296 / 60 |
| ee_path_length_m | 0.06050 | [0.01517, 0.10540] | 0.00763992 | 296 / 60 |
