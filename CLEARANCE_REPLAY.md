# Clearance-only replay of the sampler case study

The 28 September 2026 rerun preserves the original 120 environments, exact joint start/goal states, frozen planning scenes, trained model/checkpoint, sampler settings, planner budget, repeat counts and execution schedule. It reruns 1,800 plans in the original primary cohort and 900 in the original confirmation cohort.

The original two clearance levels, 0.2 and 0.8, map to 0.5 and 0.7 respectively. This preserves the two-level experimental design. Only the 0.8→0.7 cohort represents a clearance reduction; 0.2→0.5 is an increase and is reported separately. These are normalized policy inputs, not clearance distances in metres.

The replay recomputes RL deformation with the requested clearance and checks exact equality of the reproduced prior GMM (excluding the header timestamp). It also checks policy SHA/action scale and the original sampler, OMPL, audit executable and model hashes before running. It reuses the saved joint endpoints without rerunning endpoint IK. New outcomes never replace the original data. Cutoff remains 2.0, both explicit mixture fractions remain zero, and the strict MoveIt wrapper remains enabled. A synthetic exhausted-sampler test checks that the installed wrapper does not fall back to uniform sampling.

The checkpoint's saved training configuration lists goal z in [0.10, 0.30] m. The original case study samples goal z in [0.45, 0.54] m. That range is deliberately unchanged. This experiment tests clearance sensitivity under the existing goal-height mismatch; it does not identify the mismatch as the cause of failures or test an in-training-range goal distribution.

Before/after comparisons use the same source-environment identities. Repeats remain nested within environments. Report both old/new success and failure categories, particularly corridor-incompatible endpoints, with paired environment-cluster bootstrap intervals. Analyze reduced and increased clearance cohorts separately. For pooled inference, give each environment equal weight (60 per clearance transition across both source cohorts), while also reporting raw attempt counts. Use two-sided environment sign permutations with Holm correction across three samplers for each transition's success comparison. No significance-based extension of the run or outcome-based environment rejection is allowed.

The historical confirmation cohort is reused, so this rerun is a sensitivity analysis, not an additional independent confirmation. The unrestricted OMPL request is unchanged by clearance; differences in its repeated plans reflect random planner/IK streams and timing variability. Matching custom seeds does not provide identical OMPL random numbers.

## Scene-restoration amendment, before the retained replay

The first replay attempt (2,700 plans) was invalidated in full after an unrestricted goal rejection exposed a scene-restoration defect. A full `ApplyPlanningScene` update returned success while MoveIt's monitored child scene retained the preceding obstacle. Replaying the update sequence reproduced this discrepancy: the reported live obstacle was environment 59 while environment 24 had been applied. A fresh offline audit of environment 24 found its unchanged goal valid. This invalidates the experiment's live-scene control, irrespective of individual outcomes.

The retained replay therefore synchronizes the frozen scene through an explicit diff and reads back collision geometry, transforms, allowed collisions, padding and scale before planning each environment. Any mismatch aborts the run. Both complete cohorts are rerun with the same frozen deformations, schedule, endpoints and all sampler/planner settings; no results from the invalidated attempt are pooled or selectively replaced. The failed attempt and diagnostic evidence are preserved in directories suffixed `-invalidated-scene-update`. The original 26 September runner used scene diffs and is unaffected by this newly introduced full-scene replay transport issue. Each retained environment has a `scene_verification` hash record.

## Run from the workspace root

Use the same sourced workspace and isolated `ROS_DOMAIN_ID=81` as the main study. Launch `sampling_demo.launch.py` with the original checkpoint, then:

```bash
python3 src/tp_gmm/tests/test_strict_sampling_runtime.py
python3 src/tp_gmm/scripts/replay_sampling_study.py \
  --source sampling_results/2026-09-26-pure-study \
  --output sampling_results/2026-09-28-clearance-primary --clearances 0.5 0.7
python3 src/tp_gmm/scripts/replay_sampling_study.py \
  --source sampling_results/2026-09-26-path-confirmation \
  --output sampling_results/2026-09-28-clearance-confirmation --clearances 0.5 0.7
python3 src/tp_gmm/scripts/analyze_sampling_study.py sampling_results/2026-09-28-clearance-primary
python3 src/tp_gmm/scripts/analyze_sampling_study.py sampling_results/2026-09-28-clearance-confirmation
python3 src/tp_gmm/scripts/compare_clearance_replays.py \
  --before sampling_results/2026-09-26-pure-study sampling_results/2026-09-26-path-confirmation \
  --after sampling_results/2026-09-28-clearance-primary sampling_results/2026-09-28-clearance-confirmation \
  --output sampling_results/2026-09-28-clearance-comparison
```

`--resume` continues a frozen replay with identical arguments. `--design-only` creates the new deformed models without planning. Supplying one `--clearances` value applies that value to all source cases. Run separate replay directories for additional levels to preserve the source-environment cluster identities.

Completed verified replay: [results, figures and raw archives](case_studies/2026-09-28-clearance-replay/README.md). The 0.8→0.7 subset reduced observed GMM failures, without statistically decisive environment-level evidence; the increased 0.2→0.5 group introduced additional endpoint incompatibilities.
