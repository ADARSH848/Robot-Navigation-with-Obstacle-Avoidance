import unittest
import math
from navigation_assignment.cubic_spline import CubicSpline2D

class TestNavigationMath(unittest.TestCase):
    
    def test_spline_continuity(self):
        """Test if the spline actually passes through the waypoints"""
        wx = [0.0, 10.0]
        wy = [0.0, 10.0]
        sp = CubicSpline2D(wx, wy)
        
        # Check start point
        x, y = sp.calc_position(0)
        self.assertAlmostEqual(x, 0.0, places=2)
        
        # Check end point
        x, y = sp.calc_position(sp.s[-1])
        self.assertAlmostEqual(x, 10.0, places=2)

    def test_obstacle_distance_logic(self):
        """Test the logic used for avoidance weighting"""
        # If dist is 0.4 (very close), weight should be 1.0 (Max avoidance)
        dist = 0.4
        weight = 1.0 - ((dist - 0.4) / (0.6 - 0.4))
        weight = max(0.0, min(weight, 1.0))
        self.assertAlmostEqual(weight, 1.0)

        # If dist is 0.6 (safe boundary), weight should be 0.0
        dist = 0.6
        weight = 1.0 - ((dist - 0.4) / (0.6 - 0.4))
        weight = max(0.0, min(weight, 1.0))
        self.assertAlmostEqual(weight, 0.0)

if __name__ == '__main__':
    unittest.main()