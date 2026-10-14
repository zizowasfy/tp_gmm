# Paper framework alignment — 13 October 2026

These case studies use the sampling framework from TP-GMM `gazebo_exps` commit `829391e` and the EV benchmark protocol at `5434310` / `4fd1864`. Implementation and case-study work remain on `sampling-approaches-cod`; `gazebo_exps` and the completed EV benchmark are preserved.

The source commit changes a small set of sampler files but overlaps the independently developed reporting and test code on this branch. A selective **noncommitting cherry-pick**, with those overlaps resolved against the case-study implementation, was used. Merging the entire branch would also bring unrelated branch history and benchmark-specific changes into the experiment branch.

## Shared planning settings

| Setting | Aligned case studies |
|---|---|
| Active proposals | Pure GMM; pure GMR with 30 mm isotropic standard deviation |
| GMM support | Union of covariance ellipsoids; cutoff 3.0; eigenvalue floor 1e-8 |
| GMR support | Union of capsules along the post-RL reference, connected to actual FK start/goal; radius 0.030 × 3 = 0.090 m |
| Support tolerance | Zero geometric expansion; floating-point roundoff only |
| Uniform/cross-mapping mixtures | Both zero |
| Exhaustion behavior | Fail without OMPL uniform fallback |
| Planner | RRTConnect; one attempt; requested range 0.0 (OMPL automatic range) |
| Planning budget | 10 seconds |
| Edge resolution | Longest valid segment fraction 0.00005 |
| Independent audit | Joint-vector Euclidean spacing at most 0.005 rad; bounds, self/world collision, hard support |
| Simplification | Disabled |
| Timing/visualization | Sample visualization disabled; geometric plan only, no time parameterization |
| RL checkpoint | `2026-09-11_21-20-28_ppo_torch/checkpoints/best_agent.pt`; action scale 0.15 |

The additional essential changes beyond proposal selection and planner range are endpoint-connected GMR tubes, exact support checks on **planner states and interpolated edges**, finer edge checking, independent dense validation, and preserving path geometry. A boxes-only MoveIt path constraint would otherwise admit parts of the workspace outside the ellipsoids even when every random target came from the GMM.

The sampler exports canonical eigensystems in `PrepareSampling.corridor_json`. MoveIt's optional `ConstraintStateValidatorAllocator` interface supplies an immutable predicate to the OMPL planning context. All state-validity-checker paths apply it, including interpolation performed by OMPL's motion validator. The independent audit consumes the descriptor and checks support again without relying on the live sampler session. Historical native box mode remains available explicitly for compatibility; paper runners require ellipsoids/tubes.

## Preserved experimental design

1. **Clearance generalization:** the same five saved environments and joint endpoints, levels 0.1–1.0, three methods, 150 requests. These five environments support a descriptive generalization analysis, not a broad population claim.
2. **Mapping comparison:** original-height scenes and clearances 0.2/0.8; 60 primary environments × 10 repeats × 3 methods, plus 60 independent confirmation environments × 5 repeats × 3 methods. Compare Cartesian IK, joint projection and unrestricted OMPL; 2,700 requests. No cutoff-pair comparison is included.
3. **Proposal comparison:** clearance 0.7; the same 120 randomized environments × 5 repeats × 3 methods, plus three pilot scenes × 10 repeats × 3 methods. Compare GMM, GMR 30 mm and reference IK; 1,890 measured requests plus nine excluded warmups. Hybrid is removed from the active design.

Every endpoint-IK failure belonging to a frozen design is retained. Cases 1 and 2 were originally conditioned on endpoint-feasible environments; their recorded candidate rejections remain in the provenance. Case 3 keeps all original candidates, including endpoint failures. Frozen scenes are restored and read back before planning. Reproduction and deformation are timed once per environment and attributed to the relevant requests; mapping/proposal replay also checks the new models and raw references against saved inputs. Proposal seeds, environment order and method order are retained after removing Hybrid. Fresh manifests, source/binary snapshots and append-only outcomes are stored separately from historical archives. Resume refuses mismatched runtime or input hashes.

## Scope of consistency

The EV benchmark's fixtures, execution controllers, stop-at-corners retimer and learning baselines do not belong in these plan-only case studies. Case-study paths remain geometric, so the launch disables the response retiming adapter that could resample or cut corners. Duration is consequently zero and is **not an execution-time metric**. Report planning latency and geometric quality instead.

The case-study MoveGroup process retains independent OMPL/IK random streams, as in the earlier studies; custom proposal seeds are reproducible. The EV benchmark uses a fresh seeded worker per request. These designs share planner/support settings but do not provide identical OMPL random numbers. Comparisons here are paired by environment and frozen joint endpoints, with environment-level statistical inference. Reference IK uses public ROS services, so its runtime includes service overhead and should not be interpreted as a comparison of only mathematical proposal costs.

Discrete dense checking is not a proof of continuous collision freedom. GMM and GMR have deliberately different hard supports, so their comparison includes both proposal bias and permitted search region. Do not describe it as an isolated change of proposal density. Historical box-corridor outcomes are not pooled with these results.

See [COMMANDS-cod.md](COMMANDS-cod.md) for the current launch, validation, rerun and analysis commands.

The completed [results and figures](case_studies/2026-10-13-framework/FINDINGS.md) cover all 4,740 measured requests. Raw archives are indexed in the [result package](case_studies/2026-10-13-framework/README.md).
