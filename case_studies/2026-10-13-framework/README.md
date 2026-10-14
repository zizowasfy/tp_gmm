# Benchmark-aligned case studies — 13 October 2026

All three studies are complete. Start with [FINDINGS.md](FINDINGS.md) for the results, statistical qualifications and sampler tradeoffs. [FRAMEWORK_ALIGNMENT.md](../../FRAMEWORK_ALIGNMENT.md) explains the shared settings and implementation changes; [COMMANDS-cod.md](../../COMMANDS-cod.md) contains the run commands.

| Study | Measured requests | Main result |
|---|---:|---|
| RL clearance sweep | 150 | All plans succeeded; reference clearance usually increased, but 0/50 unique deformations met the strict training-clearance target |
| Cartesian IK / projection / unrestricted OMPL | 2,700 | 900/900, 896/900 and 900/900 successes; projection shortened paths without a supported reliability/latency advantage |
| GMM / GMR 30 mm / reference IK | 1,890 | Randomized successes 565/600, 556/600 and 376/600; GMR followed the reference more closely, while GMM had lower PAR2 |

The proposal study also has nine excluded warmups; its 90 pilot-scene measurements are separate from the 1,800 randomized outcomes. No Hybrid outcomes or historical box-corridor results are pooled into the analyses. These are replays of previously examined environments, not new independent confirmation.

## Browsable results

- `clearance/`: measurement notes, 150-row metrics, metadata and PDF/PNG response plots.
- `mapping/`: all estimates and adjusted tests, CSV trial/environment tables, PDF/PNG figures, separate cohort summaries and analysis sources.
- `gmr/`: full report, statistics, endpoint checks, reference/failure diagnostics, CSV tables, PDF/PNG figures and manifest.
- `validation/`: runtime strictness/exact-support fixtures, regression logs, complete outcome integrity checks and postprocessing verification.
- [data_sources.json](data_sources.json): original workspace locations, raw identities and archive locations.
- [archive_verification.json](archive_verification.json): member-by-member hashes for all four complete raw archives.
- `SHA256SUMS`: checksums of this browsable package.

## Raw data and extraction

The full uncompressed runs remain in these workspace directories:

```text
clearance_results/2026-10-13-framework
sampling_results/2026-10-13-framework-primary
sampling_results/2026-10-13-framework-confirmation
gmr_results/2026-10-13-framework
```

Complete compressed copies are in **workspace `case_study_archives/2026-10-13-framework/`**, outside the source repositories. They include every raw trajectory, dense audit path, frozen scene/model, measurement record and available source/binary snapshot. These multi-hundred-megabyte archives exceed normal Git-host file limits and are kept separately from this small, browsable result package. Keep that archive directory when transferring the complete dataset. Every member was verified byte-identical against the original run tree.

From the workspace root, extract into a new directory:

```bash
mkdir -p /tmp/paper-framework-20261013
tar -xJf case_study_archives/2026-10-13-framework/raw-clearance.tar.xz \
  -C /tmp/paper-framework-20261013
for study in mapping-primary mapping-confirmation gmr; do
  tar --zstd -xf "case_study_archives/2026-10-13-framework/raw-${study}.tar.zst" \
    -C /tmp/paper-framework-20261013
done
```

The archive checksums and exact source paths are in `archive_verification.json`; `SHA256SUMS` in the archive directory verifies the compressed files. The clearance result records 67 source/binary hashes. All 67 match the copies preserved in the primary mapping run's `provenance/` tree, as checked in `validation/clearance_source_coverage.json`.

## Reanalysis

Use the workspace's Python/ROS environment; a running ROS server or Gazebo is unnecessary for offline analysis. Substitute the extraction root for the workspace root when working from the archives. Use a fresh output directory for the combined mapping report.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python3 src/tp_gmm/scripts/plot_clearance_analysis.py \
  clearance_results/2026-10-13-framework/results.json
python3 src/tp_gmm/scripts/analyze_sampling_study.py \
  sampling_results/2026-10-13-framework-primary
python3 src/tp_gmm/scripts/analyze_sampling_study.py \
  sampling_results/2026-10-13-framework-confirmation
python3 src/tp_gmm/scripts/summarize_sampler_case_study.py \
  sampling_results/2026-10-13-framework-primary \
  sampling_results/2026-10-13-framework-confirmation \
  --output /tmp/paper-framework-mapping-analysis
python3 src/tp_gmm/scripts/analyze_gmr_study.py \
  gmr_results/2026-10-13-framework
python3 src/tp_gmm/case_studies/2026-10-13-framework/validation/verify_results.py \
  /home/zizo/the_folder/ws_moveit
```

Reanalysis rewrites derived outputs; retain an untouched extraction for verifying the original archive hashes. Measurements were not rerun, excluded or replaced in response to their outcomes.

## Post-measurement maintenance

The measurement source snapshots preserve the exact code used. After collection, descriptive GMR analysis-family text and inherited replay labels were corrected, and the mapping success plot was expanded to show the complete confidence interval. All four statistical JSON outputs were checked for exact equality before and after these report/plot corrections. Manifest `metadata_corrections` records the wording changes. No schedule, planner setting, measurement or statistical computation changed. The browsable GMR report changes only its raw-data link relative to the archived report.
