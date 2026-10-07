# Pure sampling case study results

60 source environments, 10 repeats per method, 1800 request outcomes; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 579/600 (96.5%) | 91.5–100.0% | 0.3394 | 0.1267 | 0.6922 | 0.0137 |
| joint_projected | 579/600 (96.5%) | 91.5–100.0% | 0.3334 | 0.1205 | 0.6440 | 0.0125 |
| ompl_uniform | 599/600 (99.8%) | 99.5–100.0% | 0.1641 | 0.1471 | 2.1793 | 0.0166 |

Paired primary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.0000 | [-0.5000, 0.5000] | 1 |
| cartesian_ik / joint_projected | PAR2 (s) | 0.0060 | [-0.0213, 0.0331] | 1 |
| cartesian_ik / ompl_uniform | success (pp) | -3.3333 | [-8.3333, 0.1667] | 1 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 0.1753 | [-0.0310, 0.4718] | 1 |
| joint_projected / ompl_uniform | success (pp) | -3.3333 | [-8.3333, 0.1667] | 1 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.1693 | [-0.0373, 0.4651] | 1 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Pipeline times start after scene and endpoint design. They exclude the shared endpoint IK solve and offline dense path audit. Model-preparation wall time is included for custom methods, while the unrestricted baseline needs no deformation.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0119 | 0.0116 | 0.0000 | 0.0000 | 0.0000 | 0.5609 |
| joint_projected | 0.0190 | 0.0002 | 0.0000 | 0.0189 | 0.0001 | 0.0000 | 0.2791 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004330 | 0.004225 |
| reproduction_s | 0.008124 | 0.008058 |
| rl_deformation_s | 0.002513 | 0.000759 |
| deformed_regression_s | 0.001714 | 0.001712 |
| total_s | 0.018283 | 0.016362 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

This is a goal/table-height sensitivity replay of previously used environments, not new independent confirmation. All path-quality results below are conditional on success. Matched-pair effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.
