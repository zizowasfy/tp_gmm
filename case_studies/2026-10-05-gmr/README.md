# GMR sampling studies: archived data and analysis

Start with [FINDINGS.md](FINDINGS.md) for the extensive comparison of strict GMM, GMR 30 mm, Hybrid 30 mm and reference IK, using cutoffs 3.0 and clearance input 0.7. The independent randomized cohort contains 120 environments and 2,400 measured outcomes; 120 repetitions on three original scenes are reported separately. Twelve warmups are excluded.

- [PROTOCOL.md](PROTOCOL.md) and [manifest.json](manifest.json) preserve the design and frozen environment identities.
- [analysis/report.md](analysis/report.md), [analysis/statistics.json](analysis/statistics.json), and the adjacent CSV tables and PDF/PNG figures contain the analysis.
- `verification/` preserves runtime, source-hash and purity checks, including the documented post-run maintenance changes.
- `raw-extensive.tar.xz` contains the complete, unchanged `gmr_results/2026-10-05-extensive/` tree: raw JSONL outcomes, inputs, source snapshots, verification and original reports.
- `raw-historical-pilot.tar.xz` contains the complete, unchanged `gmr_results/2026-09-24-pilot/` tree. It is historical exploratory evidence and is not pooled with the extensive study.
- `archive_verification.json` records member-by-member SHA256 comparisons against both original trees. `SHA256SUMS` covers the files in this repository package.

The archives keep the large raw JSONL file below GitHub's per-file limit without removing data. The extracted content is byte-identical to the original workspace data. This browsable findings copy changes only its raw-data link to point to these extraction instructions; the original report remains in the archive.

## Extraction and analysis

From the workspace root, choose a new empty output directory:

```bash
mkdir -p /tmp/gmr-study-archive
tar -xJf src/tp_gmm/case_studies/2026-10-05-gmr/raw-extensive.tar.xz \
  -C /tmp/gmr-study-archive
tar -xJf src/tp_gmm/case_studies/2026-10-05-gmr/raw-historical-pilot.tar.xz \
  -C /tmp/gmr-study-archive
cd /tmp/gmr-study-archive/2026-10-05-extensive
sha256sum -c SHA256SUMS
```

To regenerate the extensive analysis in this extracted copy, run from the workspace root:

```bash
python3 src/tp_gmm/scripts/analyze_gmr_study.py \
  /tmp/gmr-study-archive/2026-10-05-extensive
```

Analysis is offline and does not require Gazebo or a planning server. Reanalysis writes analysis outputs, so retain an untouched extraction when checking the original checksums. See [GMR_STUDY.md](../../GMR_STUDY.md) for running new planning experiments. Strict resume checks the exact original source/binary hashes; use the archived snapshot when reproducing a historical runtime.
