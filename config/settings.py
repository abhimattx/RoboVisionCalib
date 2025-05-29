"""
Configuration settings for the robot calibration system.

This module contains all configuration parameters used throughout the system,
organized into classes for different subsystems. It provides central configuration
management for camera parameters, robot settings, calibration options, and more.
"""

import cv2
from cv2 import aruco
import os
import yaml
import numpy as np

class Config:
    """Global configuration parameters for the robot calibration system.
    
    Attributes:
        CUBE_SIZE (float): Size of calibration cube in millimeters.
        marker_size (float): Size of ArUco markers in millimeters.
        WAIT_TIME (int): Default wait time for GUI operations in milliseconds.
        WINDOW_WIDTH (int): Default window width for visualization.
        WINDOW_HEIGHT (int): Default window height for visualization.
        CHESS_ROWS (int): Number of internal corners in checkerboard height.
        CHESS_COLS (int): Number of internal corners in checkerboard width.
        CHARUCO_ROWS (int): Number of rows in CharUco board.
        CHARUCO_COLS (int): Number of columns in CharUco board.
        SQUARE_LENGTH (float): Square size in CharUco board in meters.
        MARKER_LENGTH (float): Marker size in CharUco board in meters.
        ROBOT_IP (str): IP address of the robot controller.
        ROBOT_PORT (int): Communication port for robot controller.
        CALIB_DIR (str): Directory for calibration images.
        ARUCO_DIR (str): Directory for ArUco marker images.
        CIRCLE_DIR (str): Directory for circle grid images.
        CALIB_FILE (str): Filename for camera calibration data.
        ARUCO_DICT_TYPE (int): ArUco dictionary type constant.
        ARUCO_MARKER_SIZE (float): ArUco marker size in meters.
        PICK_APPROACH_HEIGHT (float): Height for pick approach in mm.
        PLACE_APPROACH_HEIGHT (float): Height for place approach in mm.
    """

    #cube size
    CUBE_SIZE = 40.0  # mm
    #marker size
    marker_size = 30.0  # mm
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
    ARUCO_MARKER_SIZE = 0.030  # Size in meters
    PICK_APPROACH_HEIGHT = 50.0  # Height above object for approach in mm
    PLACE_APPROACH_HEIGHT = 50.0  # Height above place location for approach in mm
    
    @classmethod
    def validate(cls):
        """Validate configuration parameters for consistency and correctness.
        
        Returns:
            bool: True if all parameters are valid, False otherwise
        """
        valid = True
        
        # Check numeric parameters are positive
        for name in ['CUBE_SIZE', 'marker_size', 'WAIT_TIME', 'WINDOW_WIDTH', 
                     'WINDOW_HEIGHT', 'CHESS_ROWS', 'CHESS_COLS', 'CHARUCO_ROWS', 
                     'CHARUCO_COLS', 'SQUARE_LENGTH', 'MARKER_LENGTH', 'ROBOT_PORT',
                     'ARUCO_MARKER_SIZE', 'PICK_APPROACH_HEIGHT', 'PLACE_APPROACH_HEIGHT']:
            value = getattr(cls, name)
            if not isinstance(value, (int, float)) or value <= 0:
                print(f"Configuration error: {name} must be a positive number")
                valid = False
        
        # Check directories exist
        for dir_name in ['CALIB_DIR', 'ARUCO_DIR', 'CIRCLE_DIR']:
            directory = getattr(cls, dir_name)
            if not os.path.exists(directory) and not os.path.isdir(directory):
                print(f"Warning: Directory {directory} does not exist")
                # We don't fail validation for missing directories since they can be created
        
        return valid

class CalibrationConfig:
    """Configuration parameters for calibration process.
    
    Attributes:
        PATTERN_CHECKERBOARD (str): Identifier for checkerboard pattern.
        PATTERN_CIRCLES (str): Identifier for circle grid pattern.
        PATTERN_CHARUCO (str): Identifier for CharUco pattern.
        ROBOT_PAUSE_TIME (int): Pause time between robot movements in seconds.
        IMAGE_CAPTURE_DELAY (float): Delay after robot stops before capturing image.
        CALIB_DIR (str): Directory for calibration images.
        CALIB_FILE (str): Filename for camera calibration data.
    """
    # Pattern types
    PATTERN_CHECKERBOARD = 'checkerboard'
    PATTERN_CIRCLES = 'circles'
    PATTERN_CHARUCO = 'charuco'
    
    # Timing
    ROBOT_PAUSE_TIME = 3  # seconds between movements
    IMAGE_CAPTURE_DELAY = 0.5  # seconds after robot stops
    
    # File paths
    CALIB_DIR = 'calib_images'
    CALIB_FILE = 'camera_calibration.yaml'
    
    @classmethod
    def get_pattern_dimensions(cls, pattern_type):
        """Get the dimensions for a specific calibration pattern.
        
        Args:
            pattern_type (str): One of the PATTERN_* constants
            
        Returns:
            tuple: (rows, cols) for the pattern
        """
        if pattern_type == cls.PATTERN_CHECKERBOARD:
            return (Config.CHESS_ROWS, Config.CHESS_COLS)
        elif pattern_type == cls.PATTERN_CHARUCO:
            return (Config.CHARUCO_ROWS, Config.CHARUCO_COLS)
        elif pattern_type == cls.PATTERN_CIRCLES:
            # Circle grid typically has the same dimensions as checkerboard
            return (Config.CHESS_ROWS, Config.CHESS_COLS)
        else:
            raise ValueError(f"Unknown pattern type: {pattern_type}")


# Initialize ArUco and Charuco
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)

CHARUCO_BOARD = aruco.CharucoBoard(
    size=(Config.CHARUCO_COLS, Config.CHARUCO_ROWS),
    squareLength=Config.SQUARE_LENGTH,
    markerLength=Config.MARKER_LENGTH,
    dictionary=ARUCO_DICT
)


def save_config_to_yaml(filename="config_export.yaml"):
    """Save current configuration to a YAML file.
    
    Args:
        filename (str): Output filename
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        config_dict = {
            'camera': {
                'cube_size': Config.CUBE_SIZE,
                'marker_size': Config.marker_size,
                'window': {
                    'width': Config.WINDOW_WIDTH,
                    'height': Config.WINDOW_HEIGHT,
                    'wait_time': Config.WAIT_TIME
                },
                'calibration': {
                    'checkerboard': {
                        'rows': Config.CHESS_ROWS,
                        'cols': Config.CHESS_COLS
                    },
                    'charuco': {
                        'rows': Config.CHARUCO_ROWS,
                        'cols': Config.CHARUCO_COLS,
                        'square_length': Config.SQUARE_LENGTH,
                        'marker_length': Config.MARKER_LENGTH
                    }
                },
                'aruco': {
                    'dict_type': Config.ARUCO_DICT_TYPE,
                    'marker_size': Config.ARUCO_MARKER_SIZE
                }
            },
            'robot': {
                'ip': Config.ROBOT_IP,
                'port': Config.ROBOT_PORT,
                'pick_approach_height': Config.PICK_APPROACH_HEIGHT,
                'place_approach_height': Config.PLACE_APPROACH_HEIGHT
            },
            'paths': {
                'calib_dir': Config.CALIB_DIR,
                'aruco_dir': Config.ARUCO_DIR,
                'circle_dir': Config.CIRCLE_DIR,
                'calib_file': Config.CALIB_FILE
            },
            'calibration': {
                'robot_pause_time': CalibrationConfig.ROBOT_PAUSE_TIME,
                'image_capture_delay': CalibrationConfig.IMAGE_CAPTURE_DELAY
            }
        }
        
        with open(filename, 'w') as f:
            yaml.dump(config_dict, f, default_flow_style=False)
        
        print(f"Configuration saved to {filename}")
        return True
    except Exception as e:
        print(f"Error saving configuration: {e}")
        return False


def load_config_from_yaml(filename):
    """Load configuration from a YAML file.
    
    Args:
        filename (str): Input filename
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        if not os.path.exists(filename):
            print(f"Configuration file not found: {filename}")
            return False
            
        with open(filename, 'r') as f:
            config_dict = yaml.safe_load(f)
        
        # Update Config class attributes
        if 'camera' in config_dict:
            cam_config = config_dict['camera']
            if 'cube_size' in cam_config:
                Config.CUBE_SIZE = cam_config['cube_size']
            if 'marker_size' in cam_config:
                Config.marker_size = cam_config['marker_size']
            
            if 'window' in cam_config:
                window = cam_config['window']
                if 'width' in window:
                    Config.WINDOW_WIDTH = window['width']
                if 'height' in window:
                    Config.WINDOW_HEIGHT = window['height']
                if 'wait_time' in window:
                    Config.WAIT_TIME = window['wait_time']
            
            if 'aruco' in cam_config:
                aruco_config = cam_config['aruco']
                if 'dict_type' in aruco_config:
                    Config.ARUCO_DICT_TYPE = aruco_config['dict_type']
                if 'marker_size' in aruco_config:
                    Config.ARUCO_MARKER_SIZE = aruco_config['marker_size']
                    
        # Update robot settings
        if 'robot' in config_dict:
            robot_config = config_dict['robot']
            if 'ip' in robot_config:
                Config.ROBOT_IP = robot_config['ip']
            if 'port' in robot_config:
                Config.ROBOT_PORT = robot_config['port']
            if 'pick_approach_height' in robot_config:
                Config.PICK_APPROACH_HEIGHT = robot_config['pick_approach_height']
            if 'place_approach_height' in robot_config:
                Config.PLACE_APPROACH_HEIGHT = robot_config['place_approach_height']
        
        # Validate the loaded configuration
        valid = Config.validate()
        if not valid:
            print("Warning: Loaded configuration has validation issues")
        
        print(f"Configuration loaded from {filename}")
        return True
    except Exception as e:
        print(f"Error loading configuration: {e}")
        return False