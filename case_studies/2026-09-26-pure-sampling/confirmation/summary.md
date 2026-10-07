# Pure sampling case study results

60 independent environments, 5 repeats per method, 900 plans; fixed 3 s planning budget.

| Method | Audited success | Cluster 95% CI | Mean PAR2 (s) | Success median action (s) | Median EE length (m) | Median world clearance (m) |
|---|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 284/300 (94.7%) | 88.3–99.7% | 0.4546 | 0.1207 | 0.6944 | 0.0150 |
| joint_projected | 282/300 (94.0%) | 88.0–99.0% | 0.4986 | 0.1228 | 0.6588 | 0.0125 |
| ompl_uniform | 300/300 (100.0%) | 100.0–100.0% | 0.1517 | 0.1474 | 2.1159 | 0.0203 |

Paired secondary contrasts (first minus second; negative time favors first):

| First / second | Outcome | Difference | Cluster 95% CI | Holm p |
|---|---|---:|---:|---:|
| cartesian_ik / joint_projected | success (pp) | 0.6667 | [-0.6667, 2.3333] | 0.75 |
| cartesian_ik / joint_projected | PAR2 (s) | -0.0440 | [-0.1395, 0.0336] | 0.605294 |
| cartesian_ik / ompl_uniform | success (pp) | -5.3333 | [-11.6667, -0.3333] | 0.491115 |
| cartesian_ik / ompl_uniform | PAR2 (s) | 0.3029 | [0.0120, 0.6644] | 0.491115 |
| joint_projected / ompl_uniform | success (pp) | -6.0000 | [-12.0000, -1.0000] | 0.3125 |
| joint_projected / ompl_uniform | PAR2 (s) | 0.3469 | [0.0614, 0.7025] | 0.274017 |

All three pairwise success/PAR2 tests belong to one Holm family. Intervals are pointwise, not multiplicity-adjusted. Zero-width intervals in all-success samples do not prove perfect population reliability.

Sampler stage medians in seconds (setup and sampling contain the nested timers):

| Method | Setup | Draw loop | Online IK | Anchor IK | Jacobian | FK mapping | Valid / attempted |
|---|---:|---:|---:|---:|---:|---:|---:|
| cartesian_ik | 0.0000 | 0.0129 | 0.0127 | 0.0000 | 0.0000 | 0.0000 | 0.5046 |
| joint_projected | 0.0201 | 0.0002 | 0.0000 | 0.0199 | 0.0001 | 0.0000 | 0.1608 |
| ompl_uniform | — | — | — | — | — | — | — |

Shared model-preparation stage times (one measurement per environment; seconds):

| Stage | Mean | Median |
|---|---:|---:|
| model_load_s | 0.004416 | 0.004124 |
| reproduction_s | 0.008255 | 0.008047 |
| rl_deformation_s | 0.000892 | 0.000720 |
| deformed_regression_s | 0.001701 | 0.001694 |
| total_s | 0.016843 | 0.016172 |

Pure sampler counters: zero internal uniform attempts and missing-anchor Cartesian rescues are required. The separate strict runtime regression verifies the MoveIt wrapper failure path. The unrestricted baseline has no GMM session/counters.

Path quality is conditional on success; only the prespecified confirmation endpoints have confirmatory status. Matched-pair quality effects and retained sample counts are in `statistics.json`. Minimum world clearance includes the table and all robot links; it is not the RL reward or only an EE-to-cylinder distance.

The custom hard corridor is a union of oriented boxes enclosing the cutoff ellipsoids; accepted targets obey the ellipsoidal cutoff, while tree edges may enter box corners. The baseline has no GMM corridor, so its contrasts are between different feasible-region configurations. Projected proposals only approximate the Cartesian Gaussian under a local linearization. The two custom methods also differ in proposal orientation behavior.

Raw inputs and outputs: `manifest.json`, `environments/*.json`, `results.jsonl`, `trials.csv`. Primary figures are supplied as PDF and PNG. No result from this study alone establishes transfer to EV battery disassembly.

## Prespecified independent confirmation

This phase tests the exploratory path-quality observation on new environments. Its primary family comprises the two directional path-length tests below; the success/PAR2 tests above are secondary in this phase.

| Outcome (Cartesian minus projected) | Effect | 95% cluster CI | One-sided Holm p | Matched pairs / environments |
|---|---:|---:|---:|---:|
| joint_path_length | 0.80128 | [0.14651, 1.48097] | 0.0426196 | 281 / 57 |
| ee_path_length_m | 0.02552 | [-0.01325, 0.06495] | 0.122059 | 281 / 57 |
