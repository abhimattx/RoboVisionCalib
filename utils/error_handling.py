"""
Error handling utilities for the Robot Calibration System.

This module provides consistent error handling and logging across the system.
"""

import os
import sys
import logging
import traceback
from datetime import datetime
from functools import wraps

# Configure logging
LOG_DIR = 'logs'
os.makedirs(LOG_DIR, exist_ok=True)

# Create logger
logger = logging.getLogger('robot_calibration')
logger.setLevel(logging.DEBUG)

# Create file handler
log_file = os.path.join(LOG_DIR, f'calibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
file_handler = logging.FileHandler(log_file)
file_handler.setLevel(logging.DEBUG)

# Create console handler
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

# Create formatter and add to handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

# Add handlers to logger
logger.addHandler(file_handler)
logger.addHandler(console_handler)


class CalibrationError(Exception):
    """Base exception for all calibration related errors."""
    pass


class CameraError(CalibrationError):
    """Exception raised for camera-related errors."""
    pass


class RobotError(CalibrationError):
    """Exception raised for robot-related errors."""
    pass


class DetectionError(CalibrationError):
    """Exception raised for marker detection errors."""
    pass


class TransformationError(CalibrationError):
    """Exception raised for coordinate transformation errors."""
    pass


class ConfigurationError(CalibrationError):
    """Exception raised for configuration errors."""
    pass


def log_exception(e, context=""):
    """Log an exception with context information.
    
    Args:
        e: The exception object
        context: Additional context information
    """
    error_type = type(e).__name__
    error_msg = str(e)
    stack_trace = traceback.format_exc()
    
    logger.error(f"Exception in {context}: {error_type} - {error_msg}")
    logger.debug(f"Stack trace:\n{stack_trace}")


def error_boundary(func):
    """Decorator to provide consistent error handling for functions.
    
    This decorator catches exceptions, logs them, and re-raises custom exceptions.
    
    Args:
        func: The function to wrap
        
    Returns:
        Wrapped function with error handling
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except CameraError as e:
            log_exception(e, func.__name__)
            print(f"Camera error: {e}")
            raise
        except RobotError as e:
            log_exception(e, func.__name__)
            print(f"Robot error: {e}")
            raise
        except DetectionError as e:
            log_exception(e, func.__name__)
            print(f"Detection error: {e}")
            raise
        except TransformationError as e:
            log_exception(e, func.__name__)
            print(f"Transformation error: {e}")
            raise
        except ConfigurationError as e:
            log_exception(e, func.__name__)
            print(f"Configuration error: {e}")
            raise
        except CalibrationError as e:
            log_exception(e, func.__name__)
            print(f"Calibration error: {e}")
            raise
        except Exception as e:
            # Unexpected exception - log and wrap in CalibrationError
            log_exception(e, func.__name__)
            print(f"Error: {e}")
            raise CalibrationError(f"Unexpected error in {func.__name__}: {e}") from e
    
    return wrapper


def safe_file_access(filename, mode='r', error_message=None):
    """Context manager for safe file access with proper error handling.
    
    Args:
        filename: Path to the file
        mode: File access mode ('r', 'w', etc.)
        error_message: Custom error message on failure
        
    Yields:
        File object
    """
    try:
        file = open(filename, mode)
        yield file
    except IOError as e:
        message = error_message or f"Error accessing file {filename}: {e}"
        logger.error(message)
        raise CalibrationError(message) from e
    finally:
        if 'file' in locals():
            file.close()
