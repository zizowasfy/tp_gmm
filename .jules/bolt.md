## 2025-05-21 - [Optimization] GaussPDFfast
**Learning:** `np.tile` and explicit `np.transpose` for matrix expansion are significantly slower than implicit broadcasting. In `gaussPDFfast`, switching to broadcasting yielded a ~2.4x speedup.
**Action:** Always prefer broadcasting over tiling for dimension matching in NumPy operations.
