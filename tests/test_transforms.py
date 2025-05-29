"""
Unit tests for coordinate transformation utilities.
"""

import unittest
import numpy as np
import os
import sys
import cv2

# Add parent directory to path to import modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import functions to test from main.py
from main import quaternion_multiply, convert_to_z_up_orientation, verify_top_approach


class TestTransformations(unittest.TestCase):
    """Test cases for coordinate transformations."""
    
    def test_quaternion_multiply(self):
        """Test quaternion multiplication."""
        # Test case 1: Identity quaternion * arbitrary quaternion = arbitrary quaternion
        identity = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        arbitrary = np.array([0.7071, 0.7071, 0.0, 0.0])  # 90 deg rotation around x-axis
        result = quaternion_multiply(identity, arbitrary)
        np.testing.assert_array_almost_equal(result, arbitrary)
        
        # Test case 2: Arbitrary quaternion * identity quaternion = arbitrary quaternion
        result = quaternion_multiply(arbitrary, identity)
        np.testing.assert_array_almost_equal(result, arbitrary)
        
        # Test case 3: Two 90-degree rotations around x-axis = 180-degree rotation
        q1 = np.array([0.7071, 0.7071, 0.0, 0.0])  # 90 deg around x
        q2 = np.array([0.7071, 0.7071, 0.0, 0.0])  # 90 deg around x
        expected = np.array([0.0, 1.0, 0.0, 0.0])  # 180 deg around x
        result = quaternion_multiply(q1, q2)
        np.testing.assert_array_almost_equal(result, expected, decimal=4)
        
        # Test case 4: Two different rotations combined
        q1 = np.array([0.7071, 0.7071, 0.0, 0.0])  # 90 deg around x
        q2 = np.array([0.7071, 0.0, 0.7071, 0.0])  # 90 deg around y
        result = quaternion_multiply(q1, q2)
        # The result should be a valid quaternion (unit norm)
        self.assertAlmostEqual(np.linalg.norm(result), 1.0, places=4)
    
    def test_convert_to_z_up_orientation(self):
        """Test conversion to Z-up orientation."""
        # Create a simple rotation vector and translation vector
        rvec = np.array([[0.0], [0.0], [0.0]])  # Identity rotation
        tvec = np.array([[10.0], [20.0], [30.0]])  # Translation
        
        # Convert to Z-up orientation
        rvec_z_up, tvec_z_up = convert_to_z_up_orientation(rvec, tvec)
        
        # Check that translation is preserved
        np.testing.assert_array_equal(tvec_z_up, tvec)
        
        # Check that rotation has changed
        self.assertFalse(np.array_equal(rvec_z_up, rvec))
        
        # Convert rotation vector to matrix and check properties
        R_z_up, _ = cv2.Rodrigues(rvec_z_up)
        
        # Z-axis should point upward now (in the original Y direction)
        # Extract Z axis (third column of rotation matrix)
        z_axis = R_z_up[:, 2]
        np.testing.assert_array_almost_equal(z_axis, [0, 1, 0], decimal=4)
    
    def test_verify_top_approach(self):
        """Test verification of top-down approach."""
        # Test case 1: Perfect top-down approach
        # Quaternion representing Z-axis pointing directly downward
        q_down = np.array([0, 1, 0, 0])  # w, x, y, z - 180° around X
        self.assertTrue(verify_top_approach(q_down))
        
        # Test case 2: Not a top-down approach
        # Quaternion representing Z-axis pointing sideways
        q_side = np.array([0.7071, 0.0, 0.7071, 0.0])  # 90° around Y
        self.assertFalse(verify_top_approach(q_side))
        
        # Test case 3: Almost top-down approach but not quite
        # Quaternion representing Z-axis pointing mostly down but slightly tilted
        q_almost = np.array([0.9659, 0.2588, 0.0, 0.0])  # 30° around X
        self.assertFalse(verify_top_approach(q_almost))
        
        # Test case 4: Very close to top-down approach
        # Quaternion representing Z-axis pointing down with very slight tilt
        q_very_close = np.array([0.9962, 0.0872, 0.0, 0.0])  # 10° around X
        self.assertTrue(verify_top_approach(q_very_close))


if __name__ == '__main__':
    unittest.main()
