"""
Unit tests for ArUco detector module.
"""

import unittest
import cv2
import numpy as np
import os
import sys

# Add parent directory to path to import modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from camera.detection import ArUcoDetector
from config.settings import Config


class TestArUcoDetector(unittest.TestCase):
    """Test cases for ArUco marker detector."""
    
    def setUp(self):
        """Set up test environment."""
        self.dict_type = cv2.aruco.DICT_5X5_50
        self.marker_size = 30.0
        self.detector = ArUcoDetector(dict_type=self.dict_type, marker_size=self.marker_size)
        
        # Create a synthetic test image with an ArUco marker
        self.test_image_path = os.path.join(os.path.dirname(__file__), "test_aruco.png")
        self._create_test_image()
        
        # Camera matrix and distortion coefficients for testing
        self.camera_matrix = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float64)
        
        self.dist_coeffs = np.zeros(5, dtype=np.float64)
    
    def _create_test_image(self):
        """Create a synthetic test image with an ArUco marker."""
        # Create ArUco marker
        dictionary = cv2.aruco.getPredefinedDictionary(self.dict_type)
        marker_id = 0
        marker_size_pixels = 200
        marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size_pixels)
        
        # Create a larger image and place the marker in it
        image_size = (640, 480)
        test_image = np.ones((image_size[1], image_size[0]), dtype=np.uint8) * 255
        
        # Place marker in center of image
        x_offset = (image_size[0] - marker_size_pixels) // 2
        y_offset = (image_size[1] - marker_size_pixels) // 2
        test_image[y_offset:y_offset+marker_size_pixels, 
                  x_offset:x_offset+marker_size_pixels] = marker_image
        
        # Save the test image
        cv2.imwrite(self.test_image_path, test_image)
    
    def tearDown(self):
        """Clean up after tests."""
        # Remove test image
        if os.path.exists(self.test_image_path):
            os.remove(self.test_image_path)
    
    def test_init(self):
        """Test initialization of ArUco detector."""
        self.assertEqual(self.detector.dict_type, self.dict_type)
        self.assertEqual(self.detector.marker_size, self.marker_size)
        self.assertIsNotNone(self.detector.dictionary)
        self.assertIsNotNone(self.detector.parameters)
        self.assertIsNotNone(self.detector.detector)
    
    def test_detect_markers(self):
        """Test marker detection."""
        # Load test image
        test_image = cv2.imread(self.test_image_path, cv2.IMREAD_GRAYSCALE)
        self.assertIsNotNone(test_image)
        
        # Detect markers
        corners, ids, rejected = self.detector.detect_markers(test_image)
        
        # Check results
        self.assertIsNotNone(ids)
        self.assertEqual(len(ids), 1)
        self.assertEqual(ids[0][0], 0)  # Marker ID should be 0
        self.assertEqual(len(corners), 1)
        self.assertEqual(corners[0].shape[1], 4)  # Should have 4 corners
    
    def test_draw_markers(self):
        """Test drawing detected markers."""
        # Load test image
        test_image = cv2.imread(self.test_image_path, cv2.IMREAD_COLOR)
        if test_image.shape[2] == 1:  # Convert grayscale to color if needed
            test_image = cv2.cvtColor(test_image, cv2.COLOR_GRAY2BGR)
        
        # Detect markers
        corners, ids, rejected = self.detector.detect_markers(test_image)
        
        # Draw markers
        result_image = self.detector.draw_markers(test_image, corners, ids)
        
        # Check result
        self.assertIsNotNone(result_image)
        self.assertEqual(result_image.shape, test_image.shape)
        
        # The result image should be different from the input
        # (since markers have been drawn)
        self.assertFalse(np.array_equal(result_image, test_image))
    
    def test_estimate_poses(self):
        """Test pose estimation for detected markers."""
        # Load test image
        test_image = cv2.imread(self.test_image_path, cv2.IMREAD_GRAYSCALE)
        
        # Detect markers
        corners, ids, rejected = self.detector.detect_markers(test_image)
        
        # Estimate poses
        marker_poses = self.detector.estimate_poses(corners, ids, 
                                               self.camera_matrix, 
                                               self.dist_coeffs)
        
        # Check results
        self.assertIsNotNone(marker_poses)
        self.assertEqual(len(marker_poses), 1)
        self.assertIn(0, marker_poses)  # Marker ID 0 should be in the poses
        
        # Get pose for marker 0
        rvec, tvec = marker_poses[0]
        
        # Check pose format
        self.assertEqual(rvec.shape, (3, 1))
        self.assertEqual(tvec.shape, (3, 1))


if __name__ == '__main__':
    unittest.main()
