import unittest
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_gmr_study import reference_distances, finite_stats

class GeometryTest(unittest.TestCase):
    def test_distance_to_segment_interior_and_endpoints(self):
        got=reference_distances([[.5,.3,0],[-.2,0,0],[1.2,0,0]],[[0,0,0],[1,0,0]])
        np.testing.assert_allclose(got,[.3,.2,.2])
    def test_repeated_reference_vertices_and_three_dimensions(self):
        got=reference_distances([[.5,.5,.1]],[[0,0,0],[1,0,0],[1,0,0],[1,1,0]])
        np.testing.assert_allclose(got,[np.sqrt(.26)])
    def test_empty_success_subset(self):
        self.assertEqual(finite_stats([]),dict(n=0,mean=None,median=None,p95=None))

if __name__=='__main__':unittest.main()
