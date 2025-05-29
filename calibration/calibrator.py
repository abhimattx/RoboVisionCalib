"""
Camera calibration module for robot-camera system.

This module provides camera calibration capabilities using various calibration 
patterns (checkerboard, circles grid, ChArUco). It supports both manual test mode
and automated robot-assisted calibration.
"""

import os
import cv2
import numpy as np
import time
from cv2 import aruco
from config.settings import Config, CalibrationConfig, CHARUCO_BOARD, ARUCO_DICT
from camera.camera import CameraCapture
from robot.controller import RobotController

class CalibrationProcess:
    """
    Manages the camera calibration workflow for robot-camera systems.
    
    Supports multiple calibration pattern types and can operate in test mode
    (manual image capture) or with a robot controller for automated calibration.
    """
    
    def __init__(self, pattern_type, test_mode=False):
        """
        Initialize the calibration process.
        
        Args:
            pattern_type: Type of calibration pattern to use (from CalibrationConfig)
            test_mode: If True, run in manual test mode without robot control
        """
        self.pattern_type = pattern_type
        self.test_mode = test_mode
        self.camera_capture = CameraCapture()
        self.robot_controller = None if test_mode else RobotController()
        
    def run_calibration(self):
        """
        Execute the complete calibration workflow.
        
        Returns:
            bool: True if calibration was successful, False otherwise
        """
        try:
            # Initialize and prepare
            print(f"\nStarting calibration with {self.pattern_type} pattern")
            print("Mode:", "Test" if self.test_mode else "Robot")
            
            os.makedirs(CalibrationConfig.CALIB_DIR, exist_ok=True)
            
            self.camera_capture.start()
            if not self.test_mode and not self.robot_controller.connect():
                raise RuntimeError("Failed to connect to robot")
            
            # Capture and process
            images, configs = self._capture_calibration_images()
            if not images:
                raise RuntimeError("No images captured")
            
            calibration_results = self._calibrate_camera(images)
            if calibration_results:
                self._save_calibration_results(calibration_results)
                print("\nCalibration completed successfully")
                return True
            return False
                
        except Exception as e:
            print(f"\nCalibration failed: {str(e)}")
            return False
            
        finally:
            self.cleanup()
    
    def _capture_calibration_images(self):
        """
        Capture calibration images based on the selected mode.
        
        Returns:
            tuple: (list of captured images, list of robot configurations)
        """
        if self.test_mode:
            return self._capture_test_mode_images()
        else:
            return self._capture_robot_mode_images()
            
    def _capture_test_mode_images(self):
        """
        Capture images manually in test mode.
        
        Returns:
            tuple: (list of captured images, empty list of configs)
        """
        images = []
        
        # Setup display window
        cv2.namedWindow('Camera Preview', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera Preview', 1280, 720)
        
        print("\nTest Mode: Manual Image Capture")
        print("Press SPACE to capture an image, ESC to finish")
        
        try:
            while True:
                # Get and display frame
                img = self.camera_capture.get_frame()
                if img is None:
                    print("Failed to capture frame, retrying...")
                    time.sleep(0.5)
                    continue
                
                cv2.imshow('Camera Preview', img)
                
                # Process key commands
                key = cv2.waitKey(1) & 0xFF
                
                if key == 32:  # SPACE key - capture image
                    images.append(img.copy())
                    filename = f"{CalibrationConfig.CALIB_DIR}/test_calib_{len(images)}.jpg"
                    cv2.imwrite(filename, img)
                    print(f"Captured image {len(images)}: {filename}")
                    
                    # Visual feedback - flash screen white
                    flash = np.ones_like(img) * 255
                    cv2.imshow('Camera Preview', flash)
                    cv2.waitKey(100)
                    
                elif key == 27:  # ESC key - exit
                    break
        finally:
            cv2.destroyAllWindows()
            
        print(f"Captured {len(images)} test images")
        return images, []  # No configs in test mode

    def _process_ui_events(self):
        """Process UI events to keep the interface responsive."""
        for _ in range(5):
            cv2.waitKey(1)
            time.sleep(0.01)
            
    def _capture_robot_mode_images(self):
        """
        Capture images automatically using robot positioning.
        
        Returns:
            tuple: (list of captured images, list of robot configurations)
        """
        images = []
        configs = []
        
        # Setup UI
        cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Robot View', 640, 480)
        
        print("\nAutomatic Configuration Capture Mode")
        print("----------------------------------")
        
        # Ensure robot connection
        if not self.robot_controller.connected and not self.robot_controller.connect():
            print("Failed to connect to robot")
            return [], []
        
        # Initialize configuration storage
        config_yaml = "configurations.yaml"
        fs = cv2.FileStorage(config_yaml, cv2.FILE_STORAGE_WRITE)
        fs.write('timestamp', time.strftime("%Y%m%d-%H%M%S"))
        fs.write('total_configs', 0)  # Will be updated at the end
        
        try:
            valid_configs = self._perform_robot_image_capture(images, configs, fs)
            
            # Finalize configuration file
            fs.write('total_configs', valid_configs)
            print(f"\nSaved {valid_configs} configurations to {config_yaml}")
            
        finally:
            fs.release()
            cv2.destroyWindow('Robot View')
            
        return images, configs
        
    def _perform_robot_image_capture(self, images, configs, fs):
        """
        Capture images at each robot position with status updates.
        
        Args:
            images: List to store captured images
            configs: List to store robot configurations
            fs: FileStorage object for saving configuration data
            
        Returns:
            int: Number of valid configurations captured
        """
        valid_configs = 0
        
        for config_num in range(1, self.robot_controller.max_configs + 1):
            # Show moving status
            self._display_status(f"Moving to config {config_num}", "Please wait...")
            print(f"\nMoving to configuration {config_num}")
            
            # Get robot configuration with retries
            config = self._get_robot_config(config_num)
            if not config:
                continue
                
            # Store configuration data
            configs.append(config)
            valid_configs += 1
                
            # Save configuration to YAML
            fs.write(f'config_{config_num}_position', config['position'])
            fs.write(f'config_{config_num}_quaternion', config['quaternion'])
                
            # Wait for robot to settle with countdown
            if self._wait_for_robot_settle(config_num):
                return valid_configs  # User canceled
                
            # Capture image with retries
            img = self._capture_image_with_retries()
            if img is None:
                print("Failed to capture image")
                continue
                
            # Save and display captured image
            images.append(img.copy())
            filename = f"{CalibrationConfig.CALIB_DIR}/calib_{config_num}.jpg"
            cv2.imwrite(filename, img)
            fs.write(f'config_{config_num}_image', filename)
            print(f"Saved image to {filename}")
            
            # Show capture feedback
            cv2.imshow('Robot View', img)
            cv2.putText(img, f"Captured #{config_num}", 
                      (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.waitKey(300)
            
            # Show progress
            if self._show_capture_progress(config_num):
                return valid_configs  # User canceled
                
        return valid_configs
        
    def _display_status(self, primary_msg, secondary_msg=""):
        """Display a status message with consistent formatting."""
        status_img = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(status_img, primary_msg, 
                   (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        if secondary_msg:
            cv2.putText(status_img, secondary_msg, 
                       (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                       
        cv2.imshow('Robot View', status_img)
        self._process_ui_events()
        
    def _get_robot_config(self, config_num):
        """
        Get robot configuration with multiple retry attempts.
        
        Args:
            config_num: Configuration number to request
            
        Returns:
            dict or None: Robot configuration if successful, None otherwise
        """
        for attempt in range(3):
            self._display_status(
                f"Config {config_num}, Attempt {attempt+1}/3",
                "Communicating with robot..."
            )
            print(f"Attempt {attempt+1}/3")
            
            config = self.robot_controller.move_to_configuration(config_num)
            if config:
                return config
                
            time.sleep(1)
            self._process_ui_events()
            
        print(f"Failed to get configuration {config_num} after 3 attempts")
        print("Continuing to next position...")
        return None
        
    def _wait_for_robot_settle(self, config_num):
        """
        Wait for robot to stabilize at position with visual countdown.
        
        Args:
            config_num: Current configuration number
            
        Returns:
            bool: True if user cancelled, False otherwise
        """
        wait_time = CalibrationConfig.ROBOT_PAUSE_TIME
        print(f"Waiting {wait_time} seconds for robot to settle...")
        
        for i in range(wait_time, 0, -1):
            status_img = np.zeros((300, 400, 3), dtype=np.uint8)
            cv2.putText(status_img, f"Robot settling: {i}s", 
                      (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(status_img, f"Position {config_num}/{self.robot_controller.max_configs}", 
                      (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(status_img, "Press 'ESC' to cancel", 
                      (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow('Robot View', status_img)
            
            # Check for escape key
            if cv2.waitKey(1) & 0xFF == 27:
                print("Calibration canceled by user")
                return True
                
            time.sleep(1)
            
        return False
        
    def _capture_image_with_retries(self):
        """
        Attempt to capture an image with multiple retries.
        
        Returns:
            numpy.ndarray or None: Captured image if successful, None otherwise
        """
        for attempt in range(3):
            img = self.camera_capture.get_frame()
            if img is not None:
                return img
                
            self._process_ui_events()
            time.sleep(0.3)
            
        return None
        
    def _show_capture_progress(self, config_num):
        """
        Show capture progress with visual progress bar.
        
        Args:
            config_num: Current configuration number
            
        Returns:
            bool: True if user cancelled, False otherwise
        """
        # Create status image with progress information
        status_img = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(status_img, f"Captured position {config_num}", 
                  (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        progress = int((config_num / self.robot_controller.max_configs) * 100)
        cv2.putText(status_img, f"Progress: {progress}%", 
                  (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(status_img, "Press 'ESC' to cancel", 
                  (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Draw progress bar
        bar_length = 300
        filled_length = int(bar_length * config_num / self.robot_controller.max_configs)
        cv2.rectangle(status_img, (20, 200), (20 + bar_length, 230), (50, 50, 50), -1)
        cv2.rectangle(status_img, (20, 200), (20 + filled_length, 230), (0, 255, 0), -1)
        
        cv2.imshow('Robot View', status_img)
        
        # Check for escape key
        if cv2.waitKey(1) & 0xFF == 27:
            print("Calibration canceled by user")
            return True
            
        return False
    
    def _calibrate_camera(self, images):
        """
        Perform camera calibration based on the selected pattern type.
        
        Args:
            images: List of captured calibration images
            
        Returns:
            tuple: Calibration results if successful, None otherwise
        """
        if self.pattern_type == CalibrationConfig.PATTERN_CHECKERBOARD:
            return self._calibrate_checkerboard(images)
        elif self.pattern_type == CalibrationConfig.PATTERN_CIRCLES:
            return self._calibrate_circles(images)
        elif self.pattern_type == CalibrationConfig.PATTERN_CHARUCO:
            return self._calibrate_charuco(images)
        else:
            raise ValueError(f"Unknown pattern type: {self.pattern_type}")
    
    def _save_calibration_results(self, results):
        """
        Save calibration results to a YAML file.
        
        Args:
            results: Calibration results tuple from calibration function
        """
        if len(results) == 6:  # Checkerboard calibration with square size
            ret, mtx, dist, rvecs, tvecs, square_size = results
        else:  # Other calibration methods
            ret, mtx, dist, rvecs, tvecs = results
            square_size = 0.0
        
        fs = cv2.FileStorage(CalibrationConfig.CALIB_FILE, cv2.FILE_STORAGE_WRITE)
        fs.write('camera_matrix', mtx)
        fs.write('dist_coeff', dist)
        fs.write('rotation_vectors', np.array(rvecs))
        fs.write('translation_vectors', np.array(tvecs))
        fs.write('calibration_error', ret)
        fs.write('calibration_time', time.strftime("%Y%m%d-%H%M%S"))
        fs.write('pattern_type', self.pattern_type)
        
        if square_size > 0:
            fs.write('square_size', square_size)
            
        fs.release()
        print(f"\nCalibration results saved to {CalibrationConfig.CALIB_FILE}")
    
    def cleanup(self):
        """Release all resources."""
        self.camera_capture.stop()
        if self.robot_controller:
            self.robot_controller.close()

    def _calibrate_checkerboard(self, images):
        """
        Calibrate camera using checkerboard pattern.
        
        Args:
            images: List of captured calibration images
            
        Returns:
            tuple: Calibration results if successful, None otherwise
        """
        print("\nCalibrating with checkerboard pattern...")
        
        # Define checkerboard dimensions
        chess_rows = 6
        chess_cols = 7
        
        # Get and validate square size input
        square_size = 0.015  # Default: 15mm
        square_size_input = input(f"Enter checkerboard square size in meters (default: {square_size}): ")
        if square_size_input.strip():
            try:
                square_size = float(square_size_input)
                print(f"Using square size: {square_size} meters")
            except ValueError:
                print(f"Invalid input. Using default square size: {square_size} meters")
        
        print(f"Looking for {chess_cols}×{chess_rows} internal corners pattern")
        
        # Create object points (3D points in real world space)
        objp = np.zeros((chess_rows * chess_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:chess_cols, 0:chess_rows].T.reshape(-1, 2) * square_size
        
        # Arrays to store object points and image points
        objpoints = []
        imgpoints = []
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        
        # Visual feedback window
        cv2.namedWindow('Checkerboard Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Checkerboard Detection', 800, 600)
        
        # Process each image
        successful_images = 0
        
        for i, img in enumerate(images):
            print(f"Processing image {i+1}/{len(images)}...")
            
            # Convert to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray_enhanced = cv2.equalizeHist(gray)
            
            # Try detecting with different approaches
            ret_flipped = False
            ret, corners = cv2.findChessboardCorners(gray, (chess_cols, chess_rows), flags)
            
            if not ret:
                print("  Trying with enhanced contrast...")
                ret, corners = cv2.findChessboardCorners(gray_enhanced, (chess_cols, chess_rows), flags)
                
            if not ret:
                print(f"  Trying with flipped dimensions ({chess_rows}×{chess_cols})...")
                ret_flipped, corners = cv2.findChessboardCorners(gray, (chess_rows, chess_cols), flags)
                if ret_flipped:
                    print("  Found with flipped dimensions!")
                    ret = True
                    # Create new object points with flipped dimensions
                    objp_flipped = np.zeros((chess_cols * chess_rows, 3), np.float32)
                    objp_flipped[:, :2] = np.mgrid[0:chess_rows, 0:chess_cols].T.reshape(-1, 2) * square_size
                    objp = objp_flipped
            
            display_img = img.copy()
            
            if ret:
                successful_images += 1
                print(f"  Chessboard found in image {i+1}! (Successful: {successful_images}/{i+1})")
                
                # Refine corner positions
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners2)
                
                # Draw and display corners
                pattern_size = (chess_rows, chess_cols) if ret_flipped else (chess_cols, chess_rows)
                cv2.drawChessboardCorners(display_img, pattern_size, corners2, ret)
                
                # Save detection result
                filename = f"{CalibrationConfig.CALIB_DIR}/corners_{i+1}.jpg"
                cv2.imwrite(filename, display_img)
                
                cv2.putText(display_img, f"Image {i+1}: Success!", (30, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                print(f"  No corners found in image {i+1}")
                cv2.putText(display_img, f"Image {i+1}: Failed", (30, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Display progress
            cv2.imshow('Checkerboard Detection', display_img)
            key = cv2.waitKey(500)
            if key == 27:  # ESC key to cancel
                print("Calibration process canceled by user")
                cv2.destroyAllWindows()
                return None
        
        cv2.destroyAllWindows()
        print(f"\nFound chessboard pattern in {successful_images} out of {len(images)} images")
        
        # Perform camera calibration if we have enough data
        if not objpoints:
            print("\nCalibration failed: No valid images with detected patterns")
            self._print_troubleshooting_tips("checkerboard", chess_cols, chess_rows)
            return None
            
        print(f"\nPerforming calibration with {len(objpoints)} images...")
        
        # Add calibration flags for better results
        calib_flags = cv2.CALIB_RATIONAL_MODEL
        
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None, flags=calib_flags
        )
        
        print("Calibration complete!")
        print(f"Calibration error (RMS): {ret}")
        print(f"Camera matrix:\n{mtx}")
        print(f"Distortion coefficients: {dist.ravel()}")
        
        # Calculate reprojection error
        self._calculate_reprojection_error(objpoints, imgpoints, mtx, dist, rvecs, tvecs)
        
        # Create and save undistorted sample image
        self._save_undistorted_sample(images[0] if images else None, mtx, dist)
        
        return ret, mtx, dist, rvecs, tvecs, square_size
        
    def _calculate_reprojection_error(self, objpoints, imgpoints, mtx, dist, rvecs, tvecs):
        """Calculate and print the mean reprojection error."""
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        
        print(f"Mean reprojection error: {mean_error/len(objpoints)}")
        
    def _save_undistorted_sample(self, img, mtx, dist):
        """Save an undistorted sample image for visualization."""
        if img is not None:
            h, w = img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
            undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)
            cv2.imwrite(f"{CalibrationConfig.CALIB_DIR}/undistorted_sample.jpg", undistorted)
            print(f"Saved undistorted sample image to {CalibrationConfig.CALIB_DIR}/undistorted_sample.jpg")

    def _print_troubleshooting_tips(self, pattern_type, cols=None, rows=None):
        """Print troubleshooting tips for calibration failures."""
        print("\nTroubleshooting tips:")
        print("1. Ensure good, even lighting")
        print("2. Hold the calibration pattern still when capturing")
        print("3. Capture the pattern from different angles")
        
        if pattern_type == "checkerboard" and cols and rows:
            print(f"4. Verify the checkerboard dimensions ({cols}×{rows} internal corners)")
        elif pattern_type == "charuco":
            print("4. Check if board dimensions match Config settings")
            print("5. Ensure markers are clearly visible")

    def _calibrate_circles(self, images):
        """
        Calibrate camera using circle grid pattern.
        
        Args:
            images: List of captured calibration images
            
        Returns:
            tuple: Calibration results if successful, None otherwise
        """
        print("\nCalibrating with circle grid pattern...")
        
        # Circle grid parameters - using symmetric grid pattern
        circle_grid_size = (6, 7)  # (width, height)
        circle_size = 0.020  # Distance between circles in meters
        
        # Prepare object points
        objp = np.zeros((circle_grid_size[0] * circle_grid_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:circle_grid_size[1], 0:circle_grid_size[0]].T.reshape(-1, 2)
        objp *= circle_size
        
        objpoints = []
        imgpoints = []
        successful_images = 0
        
        # Process each image
        for i, img in enumerate(images):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find circle grid using symmetric grid pattern
            ret, centers = cv2.findCirclesGrid(gray, circle_grid_size, 
                                            flags=cv2.CALIB_CB_SYMMETRIC_GRID)
            
            if ret:
                successful_images += 1
                objpoints.append(objp)
                imgpoints.append(centers)
                
                # Draw and display centers
                vis_img = img.copy()
                cv2.drawChessboardCorners(vis_img, circle_grid_size, centers, ret)
                filename = f"{CalibrationConfig.CALIB_DIR}/circles_{i+1}.jpg"
                cv2.imwrite(filename, vis_img)
                print(f"Found circles in image {i+1}")
            else:
                print(f"No circles found in image {i+1}")
        
        print(f"\nFound circle patterns in {successful_images} out of {len(images)} images")
        
        # Perform camera calibration if we have enough data
        if not objpoints:
            print("Calibration failed: No valid images with detected patterns")
            self._print_troubleshooting_tips("circles")
            return None
            
        print(f"\nPerforming calibration with {len(objpoints)} images...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )
        
        print(f"Calibration error (RMS): {ret}")
        print(f"Camera matrix:\n{mtx}")
        print(f"Distortion coefficients: {dist.ravel()}")
        
        # Calculate reprojection error
        self._calculate_reprojection_error(objpoints, imgpoints, mtx, dist, rvecs, tvecs)
        
        # Create and save undistorted sample image
        self._save_undistorted_sample(images[0] if images else None, mtx, dist)
        
        return ret, mtx, dist, rvecs, tvecs

    def _calibrate_charuco(self, images):
        """
        Calibrate camera using ChArUco pattern.
        
        Args:
            images: List of captured calibration images
            
        Returns:
            tuple: Calibration results if successful, None otherwise
        """
        print("\nCalibrating with ChArUco pattern...")
        
        all_corners = []
        all_ids = []
        successful_images = 0
        
        # Set up ArUco detector with optimized parameters
        detector_params = self._create_aruco_detector_params()
        aruco_detector = aruco.ArucoDetector(ARUCO_DICT, detector_params)
        charuco_detector = aruco.CharucoDetector(CHARUCO_BOARD)

        # Create window for visualization
        cv2.namedWindow('Detection Window', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Detection Window', Config.WINDOW_WIDTH, Config.WINDOW_HEIGHT)

        # Process each image
        for i, img in enumerate(images):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            marker_corners, marker_ids, rejected = aruco_detector.detectMarkers(gray)
            
            if marker_ids is not None and len(marker_ids) > 0:
                # Draw detected markers
                img_markers = aruco.drawDetectedMarkers(img.copy(), marker_corners, marker_ids)
                
                # Detect ChArUco corners
                result = charuco_detector.detectBoard(
                    gray,
                    charucoCorners=None,
                    charucoIds=None,
                    markerCorners=marker_corners,
                    markerIds=marker_ids
                )
                
                if result[0] is not None and result[1] is not None and len(result[1]) > 4:
                    charuco_corners = result[0]
                    charuco_ids = result[1]
                    
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    successful_images += 1
                    
                    # Visualize detection results
                    debug_image = img_markers.copy()
                    aruco.drawDetectedCornersCharuco(debug_image, charuco_corners, charuco_ids)
                    
                    # Add detection count
                    cv2.putText(debug_image, f"Detected: {len(charuco_ids)} corners", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Save debug image
                    filename = f"{CalibrationConfig.CALIB_DIR}/charuco_debug_{i+1}.jpg"
                    cv2.imwrite(filename, debug_image)
                    print(f"Found {len(charuco_ids)} ChArUco corners in image {i+1}")
                    
                    cv2.imshow('Detection Window', debug_image)
                else:
                    print(f"No complete ChArUco board found in image {i+1}")
                    cv2.imshow('Detection Window', img_markers)
            else:
                print(f"No markers found in image {i+1}")
                cv2.imshow('Detection Window', img)
            
            cv2.waitKey(Config.WAIT_TIME)

        cv2.destroyAllWindows()
        print(f"Successfully processed {successful_images} out of {len(images)} images")

        # Perform calibration if we have enough data
        if not all_corners:
            print("Calibration failed: No valid images with detected patterns")
            self._print_troubleshooting_tips("charuco")
            return None
            
        try:
            # Calibration flags for better results
            flags = (
                cv2.CALIB_ZERO_TANGENT_DIST +
                cv2.CALIB_FIX_K3 +
                cv2.CALIB_USE_LU
            )
            
            # Perform calibration
            ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=all_corners,
                charucoIds=all_ids,
                board=CHARUCO_BOARD,
                imageSize=gray.shape[::-1],
                cameraMatrix=None,
                distCoeffs=None,
                flags=flags
            )
            
            if ret:
                print(f"\nCalibration successful! RMS error: {ret}")
                print(f"Camera matrix:\n{mtx}")
                print(f"Distortion coefficients: {dist.ravel()}")
                
                # Create and save undistorted sample image
                self._save_undistorted_sample(images[0] if images else None, mtx, dist)
                
                return ret, mtx, dist, rvecs, tvecs
                
        except cv2.error as e:
            print(f"Calibration failed: {str(e)}")
            print("Try capturing more images with better marker visibility")
        
        self._print_troubleshooting_tips("charuco")
        return None
        
    def _create_aruco_detector_params(self):
        """Create optimized ArUco detector parameters."""
        params = aruco.DetectorParameters()
        
        # Adaptive thresholding parameters
        params.adaptiveThreshWinSizeMin = 7
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 2
        params.adaptiveThreshConstant = 7
        
        # Contour filtering parameters
        params.minMarkerPerimeterRate = 0.03
        params.maxMarkerPerimeterRate = 0.5
        params.polygonalApproxAccuracyRate = 0.05
        
        # Corner refinement parameters
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.1
        
        return params
