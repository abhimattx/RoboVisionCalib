"""
Configuration settings for the robot calibration system.
"""

import cv2
from cv2 import aruco

class Config:
    """Global configuration parameters"""
    # Display settings
    WAIT_TIME = 1000  # ms
    WINDOW_WIDTH = 800
    WINDOW_HEIGHT = 600
    
    # Checkerboard settings
    CHESS_ROWS = 6  # Number of internal corners in height direction
    CHESS_COLS = 7  # Number of internal corners in width direction
    
    # Charuco board settings
    CHARUCO_ROWS = 8
    CHARUCO_COLS = 11
    SQUARE_LENGTH = 0.015  # meters
    MARKER_LENGTH = 0.011  # meters
    
    # Robot connection
    ROBOT_IP = "192.168.125.1"
    ROBOT_PORT = 1025
    
    # File paths
    CALIB_DIR = 'Calib_images'
    ARUCO_DIR = 'Aruco_images'
    CIRCLE_DIR = 'Circle_images'
    CALIB_FILE = 'camera_calibration.yaml'
    
    # ArUco settings
    ARUCO_DICT_TYPE = cv2.aruco.DICT_5X5_50
    0  # For older OpenCV
    # Alternative: ARUCO_DICT_TYPE = 6  # For newer OpenCV 
    ARUCO_MARKER_SIZE = 0.030  # Size in mm
    PICK_APPROACH_HEIGHT = 50.0  # Height above object for approach in mm
    PLACE_APPROACH_HEIGHT = 50.0  # Height above place location for approach in mm

class CalibrationConfig:
    """Configuration parameters for calibration process."""
    # Pattern types
    PATTERN_CHECKERBOARD = 'checkerboard'
    PATTERN_CIRCLES = 'circles'
    PATTERN_CHARUCO = 'charuco'
    
    # Timing
    ROBOT_PAUSE_TIME = 5  # seconds between movements
    IMAGE_CAPTURE_DELAY = 0.5  # seconds after robot stops
    
    # File paths
    CALIB_DIR = 'calib_images'
    CALIB_FILE = 'camera_calibration.yaml'

# Initialize ArUco and Charuco
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

CHARUCO_BOARD = aruco.CharucoBoard(
    size=(Config.CHARUCO_COLS, Config.CHARUCO_ROWS),
    squareLength=Config.SQUARE_LENGTH,
    markerLength=Config.MARKER_LENGTH,
    dictionary=ARUCO_DICT
)