# Findings: strict GMM, GMR 30 mm, Hybrid 30 mm and reference IK

The expanded experiment supports **GMR guidance for closer adherence to the reproduced path**, but does **not demonstrate better planning success, speed, path length or obstacle clearance than GMM** in this environment distribution. Hybrid retains that adherence benefit, with no demonstrated advantage over pure GMR. Direct reference IK is substantially less reliable than all three RRT proposals.

## Experiment and reliability

We ran 2,400 measured outcomes on 120 new environments (40 central, 40 offset and 40 distant cylinder layouts; five repeats per method), plus 120 measured outcomes on the three original pilot scenes. Twelve warmups were excluded. The table below concerns only the randomized cohort. The original scenes and historical pilot are not pooled into this inference.

Both GMM and GMR cutoffs were 3.0, the clearance input was 0.7, and the GMR tube used a nominal 30 mm standard deviation per Cartesian axis with radial truncation at 90 mm. Hybrid used 80% GMR / 20% GMM. All three proposals used Cartesian + IK and the same GMM-derived hard corridor, original table, checkpoint, frozen endpoints and post-RL DSGMR reference. Reference IK used its existing sequential IK and edge-validation procedure. RRTConnect had one attempt and a 3 s budget, without simplification.

| Method | Success / all 600 requests | Success / 565 endpoint-solved requests | Median successful action time | Observed valid targets / attempts |
|---|---:|---:|---:|---:|
| GMM | 565/600 (94.17%) | 565/565 (100%) | 131.4 ms | 53.75% |
| GMR 30 mm | 565/600 (94.17%) | 565/565 (100%) | 131.9 ms | 67.48% |
| Hybrid 30 mm | 565/600 (94.17%) | 565/565 (100%) | 130.2 ms | 65.17% |
| reference_ik | 405/600 (67.50%) | 405/565 (71.68%) | 138.1 ms | Not a random sampler |

Seven environments failed endpoint IK before any method-specific planning. Their 35 repeated requests count as failures for every method. “Endpoint-solved” means the configured IK call found the endpoints; a failure does not establish geometric unreachability. None of the three RRT proposals had an additional planning failure in the randomized cohort.

The environment-bootstrap 95% interval for overall success is 90.00–98.33% for each RRT proposal and 59.17–75.83% for reference IK. Each RRT proposal exceeds reference IK by **26.67 percentage points**, with a paired 95% interval of **19.17–35.00 points** and Holm-adjusted p = **0.000120**. There is no observed success difference among the RRT proposals. This reliability ceiling limits the study's ability to distinguish them; it does not prove equal reliability on other tasks.

On the repeated original scenes, each RRT proposal succeeded in **30/30** requests. Reference IK succeeded in **10/30**: all ten open-scene runs, none in either obstacle scene. These are descriptive replications across three environments, not 30 independent environments.

![Outcomes](analysis/outcomes.png)

## Guidance improves reference adherence, not demonstrated search performance

On the 565 paired successes from 113 endpoint-solved environments, GMR reduced mean end-effector distance to the reference by **17.20 mm** relative to GMM (95% CI **13.44–21.05 mm**, Holm p = **0.000120**). Hybrid reduced it by **13.41 mm** (95% CI **9.33–17.50 mm**, Holm p = **0.000120**). GMR's estimated additional improvement over Hybrid was **3.79 mm**, but its interval included zero (−0.44 to 8.04 mm reduction; Holm p = 0.668).

These differences use near arc-uniform sampling of the complete returned end-effector path, including connections to the actual endpoints, and exact distances to reference segments. They describe the geometry of the returned plan, not merely the proximity of proposal targets.

GMR's end-effector path was an estimated **32.76 mm shorter** than GMM's, but the interval crossed zero (−1.58 to 67.53 mm shorter; Holm p = 0.607). No pairwise difference in joint path length, EE path length or minimum robot–world clearance survived the prespecified secondary correction. In particular, Hybrid's unadjusted clearance difference must not be presented as a confirmed finding: its Holm p was 0.148.

Mean failure-penalized time (PAR2) was **0.478 s** for GMM, **0.480 s** for GMR, **0.478 s** for Hybrid and **2.048 s** for reference IK. All primary RRT-versus-RRT PAR2 comparisons had Holm p = 1.0. Thus the greater observed target acceptance of GMR and Hybrid did not produce a demonstrated action-level speedup. Mean recorded online-IK time was lower for those proposals, but IK is only part of the full planning action. Sampling acceptance and stage timings are descriptive metrics, not additional uncorrected significance claims.

![Path quality](analysis/path_quality.png)

## Descriptive path medians

| Method | Successful requests | Joint path length (MoveIt distance, rad) | EE path length (m) | Minimum world clearance (mm) | Mean reference deviation (mm) |
|---|---:|---:|---:|---:|---:|
| GMM | 565 | 11.240 | 0.795 | 14.72 | 83.48 |
| GMR 30 mm | 565 | 11.345 | 0.755 | 13.81 | 62.24 |
| Hybrid 30 mm | 565 | 11.360 | 0.758 | 13.50 | 65.13 |
| reference_ik | 405 | 4.939 | 0.709 | 24.12 | 3.93 |

Medians use each method's successful requests; reference IK therefore has a different conditioning set. The paired quality contrasts above, not these unpaired medians, support the comparative conclusions. The joint metric is MoveIt's group distance, not the Euclidean joint-vector norm.

## Why direct reference IK can still fail

Reference IK had **110 IK failures** and **50 edge-validation failures**, in addition to the 35 shared endpoint failures. No timeout failures occurred. The stage labels do not isolate whether an invalid edge violated a collision constraint, the corridor, or another validity condition; no more specific cause is inferred from them.

All 120 new reference end-effector polylines had positive finite-cylinder clearance (minimum across environments **43.4 mm**). Nevertheless, an obstacle-clearing end-effector curve does not guarantee a realizable, collision-free joint trajectory. Reference endpoints also missed the requested task endpoints: median errors were **29.8 mm at the start** and **68.1 mm at the goal**, with goal errors up to **139.8 mm**. The baseline therefore had to connect the actual joint endpoints to the reproduced curve. We preserved that behavior rather than correcting the reference after seeing failures.

Reference IK's successful paths closely followed the reference and had short joint paths, but they came from a smaller, selected subset of environments. Its quality summaries are descriptive and cannot be directly interpreted as dominance over the RRT paths. Its timing also includes public IK/FK/validity-service calls and the output is an untimed geometric path.

## Implications for the paper and later benchmark

- **GMM:** broad exploration within the learned mixture, with the same observed reliability and comparable action time as the more concentrated proposals here. It has weaker reference adherence and lower observed target acceptance.
- **GMR 30 mm:** a justified option when following the reproduced motion matters. It improved reference adherence and observed target acceptance without an observed reliability loss. The current experiment does not establish faster search or shorter/safer paths.
- **Hybrid 30 mm:** retains a GMM component and improves reference adherence over GMM. The anticipated robustness benefit of broader proposal coverage was not demonstrated in this high-success cohort, and it did not outperform GMR after correction.
- **reference_ik:** useful as a direct-rollout feasibility baseline. It solved many easy cases, but the substantial additional failures support retaining a search-based planner rather than relying only on sequential IK.

These results do not compare the GMR proposals with `joint_projected` or unrestricted OMPL. They should not overturn the earlier mapping comparison by combining unlike experimental conditions. Stronger general claims need the later benchmark or another independently specified distribution with enough search failures to distinguish robustness.

## Integrity and statistical scope

Both plugin uniform mixture and MoveIt's outer uniform fallback were disabled. The unreachable-tube runtime test returned only GMR attempts and no valid samples or fallback; the generic strict-wrapper exhaustion test also passed. Every recorded request had matching scene readback; proposal counters contained **zero uniform attempts**, **zero joint-projection attempts**, and **no GMM attempts in pure GMR**. Successful paths all passed the common dense audit and exact joint-endpoint check. All 115 recorded source/binary hashes matched at measurement completion.

The independent statistical unit is the environment. Inference used 20,000 layout-stratified environment bootstrap samples and up to 100,000 paired sign permutations. Twelve primary tests (all six method pairs × success/PAR2) and twelve secondary tests (three RRT pairs × four quality metrics) were corrected as separate prespecified Holm families. Quality comparisons condition on paired success. Bootstrap intervals are pointwise; the adjusted p-values govern the stated discoveries. No observed outcome changed the fixed sample size.

The normalized clearance input is not a guarantee of 0.7 m physical separation. The original table and goal-height distribution, one checkpoint and one deformation per environment were retained. Deformation time is attributed to each request, while the offline audit is excluded from planning timing. This study concerns planning variation conditional on the frozen deformations. It does not test RL inference variability, continuous collision-freedom, or generalization to the EV battery scene.

[Full analysis, tables and figures](analysis/report.md) · [Protocol](PROTOCOL.md) · [Machine-readable statistics](analysis/statistics.json) · [Raw outcomes and extraction instructions](README.md)
