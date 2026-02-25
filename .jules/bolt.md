## 2025-05-15 - GaussPDF Optimization
**Learning:** `np.tile` and `np.transpose` allocate large intermediate arrays when broadcasting against many data points. Standard broadcasting (using `(N, 1)` vs `(N, M)` shapes) is much faster and memory efficient.
**Action:** Always check `np.tile` usage in critical loops. Use `keepdims=True` or explicit `[:, np.newaxis]` to leverage broadcasting.
