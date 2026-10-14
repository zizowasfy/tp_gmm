# Strict GMM / GMR proposal comparison

Completed 1800 measured outcomes in 120 frozen randomized environments, plus 90 measured outcomes on the three original pilot scenes. 9 warmups excluded.

**Settings:** GMM cutoff 3.0; GMR radial cutoff 3.0; clearance input 0.7; GMR standard deviation 30 mm; Cartesian + IK mapping; RRTConnect, range 0.0 (automatic), 10 s budget, no simplification. The reference is the frozen post-RL DSGMR rollout. Uniform mixture and MoveIt wrapper fallback are disabled.

| Method | Audited success | Environment-bootstrap 95% CI | Mean PAR2 (s) | Successful median action (ms) | Proposal acceptance |
|---|---:|---:|---:|---:|---:|
| GMM | 565/600 (94.2%) | 90.0%–98.3% | 1.339 | 177.5 | 54.2% |
| GMR 30 mm | 556/600 (92.7%) | 87.7%–96.8% | 1.680 | 198.5 | 55.7% |
| Reference IK | 376/600 (62.7%) | 54.3%–71.0% | 7.732 | 406.3 | n/a |

![Outcomes](outcomes.png)

## Inference

Environment is the independent unit. Confidence intervals use 20,000 layout-stratified cluster bootstrap samples. Primary comparisons use paired environment sign permutations and Holm correction across all three method pairs × two metrics. PAR2 assigns 20 s to every failure; successful action time is capped at 10 s. Repeats are not treated as independent experimental units.

| Contrast (A − B) | Metric | Difference [95% CI] | Holm p |
|---|---|---:|---:|
| GMM − GMR 30 mm | success | 1.500 [0.000, 3.833] pp | 0.5 |
| GMM − GMR 30 mm | par2_s | -0.341 [-0.821, -0.025] s | 6e-05 |
| GMM − Reference IK | success | 31.500 [23.333, 39.833] pp | 6e-05 |
| GMM − Reference IK | par2_s | -6.393 [-8.032, -4.794] s | 6e-05 |
| GMR 30 mm − Reference IK | success | 30.000 [21.996, 38.167] pp | 6e-05 |
| GMR 30 mm − Reference IK | par2_s | -6.052 [-7.664, -4.464] s | 6e-05 |

## Path quality on paired successes

Each contrast uses repeats where both methods succeeded, averages the paired difference within each environment, then weights environments equally. These results are conditional on success and do not replace the reliability comparison. The separate four-test secondary family covers one RRT proposal pair × four quality metrics. Reference IK quality is descriptive.

| Contrast (A − B) | Metric | Mean difference [95% CI] | Matched repeats / environments | Holm p |
|---|---|---:|---:|---:|
| GMM − GMR 30 mm | joint_path_length | 0.52819 [-0.02147, 1.07098] | 556 / 112 | 0.06682 |
| GMM − GMR 30 mm | ee_path_length_m | 0.11257 [0.08043, 0.14417] | 556 / 112 | 4e-05 |
| GMM − GMR 30 mm | min_world_clearance_m | 0.00431 [0.00223, 0.00657] | 556 / 112 | 0.00016 |
| GMM − GMR 30 mm | reference_deviation_mean_m | 0.04177 [0.03929, 0.04430] | 556 / 112 | 4e-05 |

![Path quality](path_quality.png)

## Descriptive path medians

These use each method’s successful requests; reference IK has a smaller conditioning set. Use the paired contrasts above for comparative inference.

| Method | Successful requests | Joint length (MoveIt distance, rad) | EE length (m) | World clearance (mm) | Mean reference deviation (mm) |
|---|---:|---:|---:|---:|---:|
| GMM | 565 | 10.414 | 0.757 | 13.576 | 76.684 |
| GMR 30 mm | 556 | 9.016 | 0.663 | 11.982 | 38.867 |
| Reference IK | 376 | 4.995 | 0.704 | 25.698 | 5.531 |

## Failures and original-scene replication

- **GMM:** randomized failures `{"endpoint_ik": 35}`; original scenes 30/30 successes.
- **GMR 30 mm:** randomized failures `{"endpoint_ik": 35, "planning": 9}`; original scenes 30/30 successes.
- **Reference IK:** randomized failures `{"endpoint_ik": 35, "reference_edge_invalid": 74, "reference_ik": 115}`; original scenes 10/30 successes.

![Failures](failure_stages.png)

![Sampler diagnostics](sampling_diagnostics.png)

![Example paths](example_paths.png)

## Interpretation and limits

- GMM explores the full truncated learned mixture. GMR concentrates its proposals around the reproduced reference, with a 90 mm hard tube connected to the actual FK endpoints. Accepted targets are also conditioned on IK feasibility, collisions and each method’s hard support (GMM ellipsoid union or GMR tube); RRT vertices need not follow the proposal distribution.
- Reference IK follows the rollout without search or detour repair and must connect the actual task endpoints. Its timing includes public IK/FK/validity-service calls and it returns an untimed geometric path. It is a feasibility baseline; runtime differences also reflect architecture. It is not an unrestricted OMPL baseline.
- All three methods use identical frozen inputs and endpoints per environment. Endpoint IK failures count for every method; no candidate is replaced after observing difficulty. Inspect `reference_diagnostics.csv` for endpoint mismatch and cylinder intersections.
- Mean reference deviation uses the whole audited EE polyline, resampled at at most 2 mm, and exact point-to-reference-segment distances. World clearance is robot-versus-world and excludes self-distance; EE-cylinder clearance is a separate point metric.
- Clearance 0.7 is normalized RL input, not 0.7 m guaranteed separation. The original table and goal-height range are retained. These conclusions are limited to this distribution and checkpoint.
- A framework replay reuses the saved environments and is not new independent confirmation. Only its fresh outcomes enter this report; historical outcomes are not pooled.
- Deformation is run once per environment and its measured cost is attributed to each request. End-to-end figures are additive estimates, not repeated independent RL inference measurements. Offline audit and analysis time is excluded from planning timing.
- Bootstrap intervals can collapse when every observed environment has the same outcome; this does not establish perfect population reliability. No equivalence claim follows from a nonsignificant difference.
- Historical pilot results are not pooled: their outer MoveIt wrapper could silently fall back. This run checks the wrapper parameter, exhausted-sampler behavior, per-request source counters, scene readback and dense returned-path audits.

Purity totals (including warmups): `{"fallback_enabled": false, "projected_attempts": 0, "uniform_attempts": 0}`.

See `statistics.json`, `trials.csv`, contrast CSVs and `endpoint_audit.json` for complete values. Vector PDF versions accompany every figure. Raw JSONL, frozen inputs, source snapshots and protocol are located through [the data index](../README.md).
