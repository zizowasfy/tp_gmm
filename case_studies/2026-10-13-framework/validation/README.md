# Validation before measurement

- Rebuilt `moveit_planners_ompl` and `tp_gmm` successfully in Release mode.
- Both C++ sampler test targets passed (`sampling_math`, `path_proposal`).
- Selected Python tests passed for geometry/client validation, study purity, replay/paired statistics, clearance analysis and reporting.
- Exhausted joint projection: zero anchors, zero valid samples, zero uniform attempts, planning failed as required.
- Exhausted GMR: 100 GMR attempts, zero GMM/uniform attempts and zero valid samples, planning failed as required. This fixture uses the explicit legacy box envelope solely to admit endpoints while placing every proposal out of reach; the measured study uses endpoint-connected tubes.
- Exact support: a state inside a native covariance bounding box but outside the ellipsoid was rejected by OMPL and independently by the audit. The exact-support callback was invoked 79 times and rejected all 79 checks in this fixture.
- Saved-scene smoke test: GMM, GMR 30 mm and reference IK all returned paths with zero invalid audit samples. GMM invoked the exact support validator 9,042 times.

The recorded logs accompany these checks. Counts from smoke tests are diagnostics, not case-study measurements. No smoke-test outcomes are pooled into the paper comparisons.

After measurement, all six MoveIt OMPL C++ regression targets passed on a separate ROS domain (see `moveit-ompl-regression-tests.log`).

Every saved request passed the independent endpoint/audit/purity verification (`results_integrity.json`). That integrity file includes the nine warmups; the statistical reports exclude them. The postprocessing checks confirmed identical statistical JSON outputs after report/plot corrections. Four mapping-analysis tests and three GMR-analysis tests passed; an initial misspelled GMR discovery pattern matched no tests, and the corrected discovery log is `gmr-analysis-tests.log`.
