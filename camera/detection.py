import cv2
import numpy as np
from config.settings import Config

class ArUcoDetector:
    """ArUco marker detector with OpenCV 4.11.0+ compatibility."""
    
    def __init__(self, dict_type=cv2.aruco.DICT_5X5_50, marker_size=30.0):
        """Initialize ArUco detector.
        
        Args:
            dict_type: The dictionary type to use (see cv2.aruco.DICT_*)
            marker_size: The size of the marker in mm
        """
        self.dict_type = dict_type
        self.marker_size = marker_size
        
        # Create dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
        
        # Create detector parameters
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        
        # Create detector with dictionary and parameters
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        print(f"Using OpenCV {cv2.__version__} ArUco API")
        print(f"ArUco detector initialized with marker size: {marker_size} mm")
        
    def detect_markers(self, frame):
        """Detect ArUco markers in the frame.
        
        Args:
            frame: Input image
            
        Returns:
            corners: List of marker corners
            ids: List of marker IDs
            rejected: List of rejected candidates
        """
        # Convert to grayscale if needed
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
            
        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        return corners, ids, rejected
        
    def draw_markers(self, frame, corners, ids):
        """Draw detected markers on the frame.
        
        Args:
            frame: Input/output image
            corners: Marker corners from detectMarkers
            ids: Marker IDs from detectMarkers
            
        Returns:
            frame: Image with drawn markers
        """
        if ids is not None:
            # Draw markers on frame
            result_frame = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            return result_frame
        return frame.copy()
        
    def estimate_poses(self, corners, ids, camera_matrix, dist_coeffs):
        """Estimate poses of detected markers using updated OpenCV 4.11.0+ API.
        
        Args:
            corners: Marker corners from detectMarkers
            ids: Marker IDs from detectMarkers
            camera_matrix: Camera matrix from calibration
            dist_coeffs: Distortion coefficients from calibration
            
        Returns:
            marker_poses: Dictionary mapping marker IDs to (rvec, tvec) tuples
        """
        if ids is None or len(ids) == 0:
            return {}
            
        marker_poses = {}
        
        for i, marker_id in enumerate(ids.flatten()):
            # Get marker corners
            marker_corners = corners[i]
            
            # For OpenCV 4.11.0+, we need to use the solvePnP approach
            # First, define the 3D coordinates of the marker corners in the marker coordinate system
            objPoints = np.array([
                [self.marker_size/2, -self.marker_size/2, -Config.CUBE_SIZE/2],   # Top-left corner
                [self.marker_size/2, self.marker_size/2, -Config.CUBE_SIZE/2],    # Top-right corner
                [-self.marker_size/2, +self.marker_size/2, -Config.CUBE_SIZE/2],   # Bottom-right corner
                [-self.marker_size/2, -self.marker_size/2, -Config.CUBE_SIZE/2],  # Bottom-left corner
            ], dtype=np.float32)
            
            # The 2D coordinates are already in marker_corners
            imgPoints = marker_corners.reshape(4, 2)
            
            # Use solvePnP to get the pose
            success, rvec, tvec = cv2.solvePnP(
                objPoints, 
                imgPoints, 
                camera_matrix, 
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if success:
                # Store pose
                marker_poses[marker_id] = (rvec, tvec)
                
        return marker_poses