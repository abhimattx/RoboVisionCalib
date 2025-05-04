"""
Camera calibration process implementation.
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
    """Manages the complete calibration workflow."""
    
    def __init__(self, pattern_type, test_mode=False):
        self.pattern_type = pattern_type
        self.test_mode = test_mode
        self.camera_capture = CameraCapture()
        self.robot_controller = None if test_mode else RobotController()
        
    def run_calibration(self):
        """Execute the complete calibration process."""
        try:
            print(f"\nStarting calibration with {self.pattern_type} pattern")
            print("Test mode:" if self.test_mode else "Robot mode:")
            
            # Create output directory
            os.makedirs(CalibrationConfig.CALIB_DIR, exist_ok=True)
            
            # Initialize devices
            self.camera_capture.start()
            if not self.test_mode:
                if not self.robot_controller.connect():
                    raise RuntimeError("Failed to connect to robot")
            
            # Capture calibration images
            images, configs = self._capture_calibration_images()
            if not images:
                raise RuntimeError("No images captured")
            
            # Perform calibration
            calibration_results = self._calibrate_camera(images)
            if calibration_results:
                self._save_calibration_results(calibration_results)
                print("\nCalibration completed successfully")
                return True
                
        except Exception as e:
            print(f"\nCalibration failed: {str(e)}")
            return False
            
        finally:
            self.cleanup()
    
    def _capture_calibration_images(self):
        """Capture images at each robot configuration without user prompts."""
        images = []
        configs = []
        
        if self.test_mode:
            # Test mode - capture images manually
            cv2.namedWindow('Camera Preview', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Camera Preview', 1280, 720)
            
            print("\nTest Mode: Manual Image Capture")
            print("Press SPACE to capture an image, ESC to finish")
            
            while True:
                # Get frame
                img = self.camera_capture.get_frame()
                if img is None:
                    print("Failed to capture frame")
                    time.sleep(0.5)
                    continue
                    
                # Display frame
                cv2.imshow('Camera Preview', img)
                
                # Handle key press
                key = cv2.waitKey(1) & 0xFF
                
                # Space = capture
                if key == 32:  # SPACE key
                    images.append(img.copy())
                    filename = f"{CalibrationConfig.CALIB_DIR}/test_calib_{len(images)}.jpg"
                    cv2.imwrite(filename, img)
                    print(f"Captured image {len(images)}: {filename}")
                    
                    # Flash screen to indicate capture
                    flash = np.ones_like(img) * 255
                    cv2.imshow('Camera Preview', flash)
                    cv2.waitKey(100)
                    
                # ESC = exit
                elif key == 27:
                    break
                    
            cv2.destroyAllWindows()
            print(f"Captured {len(images)} test images")
        else:
            # Create window with smaller timeout to prevent blocking
            cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Robot View', 640, 480)
            
            print("\nAutomatic Configuration Capture Mode")
            print("----------------------------------")
            
            # First, check connection
            if not self.robot_controller.connected:
                print("Robot not connected! Reconnecting...")
                if not self.robot_controller.connect():
                    print("Failed to connect to robot")
                    return [], []
            
            # Initialize YAML file for configurations
            config_yaml = "configurations.yaml"  # In main directory
            fs = cv2.FileStorage(config_yaml, cv2.FILE_STORAGE_WRITE)
            fs.write('timestamp', time.strftime("%Y%m%d-%H%M%S"))
            fs.write('total_configs', 0)  # Will update this at the end
            
            # Keep UI responsive by processing events periodically
            def process_ui_events():
                for _ in range(5):  # Process multiple events
                    cv2.waitKey(1)
                    time.sleep(0.01)
            
            # Capture images at each configuration
            valid_configs = 0
            for config_num in range(1, self.robot_controller.max_configs + 1):
                print(f"\nMoving to configuration {config_num}")
                
                # Show status in window
                status_img = np.zeros((300, 400, 3), dtype=np.uint8)
                cv2.putText(status_img, f"Moving to config {config_num}", 
                           (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(status_img, "Please wait...", 
                           (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow('Robot View', status_img)
                process_ui_events()
                
                # Get configuration from robot - try up to 3 times
                config = None
                for attempt in range(3):
                    print(f"Attempt {attempt+1}/3")
                    
                    # Keep UI responsive during network operations
                    status_img = np.zeros((300, 400, 3), dtype=np.uint8)
                    cv2.putText(status_img, f"Config {config_num}, Attempt {attempt+1}", 
                               (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.putText(status_img, "Communicating with robot...", 
                               (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.imshow('Robot View', status_img)
                    process_ui_events()
                    
                    # Try to get configuration
                    config = self.robot_controller.move_to_configuration(config_num)
                    if config:
                        break
                    time.sleep(1)
                    process_ui_events()  # Keep UI responsive during sleep
                        
                if not config:
                    print(f"Failed to get configuration {config_num} after 3 attempts")
                    print("Continuing to next position...")
                    continue
                        
                # Store configuration data
                configs.append(config)
                valid_configs += 1
                    
                # Save configuration to YAML
                fs.write(f'config_{config_num}_position', config['position'])
                fs.write(f'config_{config_num}_quaternion', config['quaternion'])
                    
                # Show waiting message
                wait_time = CalibrationConfig.ROBOT_PAUSE_TIME
                print(f"Waiting {wait_time} seconds for robot to settle...")
                
                # Visual countdown
                for i in range(wait_time, 0, -1):
                    status_img = np.zeros((300, 400, 3), dtype=np.uint8)
                    cv2.putText(status_img, f"Robot settling: {i}s", 
                              (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.putText(status_img, f"Position {config_num}/{self.robot_controller.max_configs}", 
                              (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(status_img, "Press 'ESC' to cancel", 
                              (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.imshow('Robot View', status_img)
                    
                    # Check for escape key to cancel
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC key
                        print("Calibration canceled by user")
                        fs.write('total_configs', valid_configs)
                        fs.release()
                        return images, configs
                        
                    time.sleep(1)
                    
                # Capture and save image - try up to 3 times
                img = None
                for attempt in range(3):
                    img = self.camera_capture.get_frame()
                    if img is not None:
                        break
                    process_ui_events()  # Keep UI responsive
                    time.sleep(0.3)  # Short delay before retry
                        
                if img is not None:
                    images.append(img.copy())
                    filename = f"{CalibrationConfig.CALIB_DIR}/calib_{config_num}.jpg"
                    cv2.imwrite(filename, img)
                    print(f"Saved image to {filename}")
                        
                    # Link image file to configuration
                    fs.write(f'config_{config_num}_image', filename)
                        
                    # Show captured image briefly
                    cv2.imshow('Robot View', img)
                    cv2.putText(img, f"Captured #{config_num}", 
                              (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.waitKey(300)
                    
                    # Display progress
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
                    
                    # Check for escape key to cancel
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC key
                        print("Calibration canceled by user")
                        fs.write('total_configs', valid_configs)
                        fs.release()
                        return images, configs
                else:
                    print("Failed to capture image")
            
            # Update total configurations and close YAML file
            fs.write('total_configs', valid_configs)
            fs.release()
                
            print(f"\nSaved all {valid_configs} configurations to {config_yaml}")
            cv2.destroyWindow('Robot View')
        
        return images, configs
    
    def _calibrate_camera(self, images):
        """Perform camera calibration based on pattern type."""
        if self.pattern_type == CalibrationConfig.PATTERN_CHECKERBOARD:
            return self._calibrate_checkerboard(images)
        elif self.pattern_type == CalibrationConfig.PATTERN_CIRCLES:
            return self._calibrate_circles(images)
        elif self.pattern_type == CalibrationConfig.PATTERN_CHARUCO:
            return self._calibrate_charuco(images)
        else:
            raise ValueError(f"Unknown pattern type: {self.pattern_type}")
    
    def _save_calibration_results(self, results):
        """Save calibration results to YAML file."""
        if len(results) == 6:  # Checkerboard calibration with square size
            ret, mtx, dist, rvecs, tvecs, square_size = results
        else:  # Other calibration methods
            ret, mtx, dist, rvecs, tvecs = results
            square_size = 0.0  # Not applicable
        
        fs = cv2.FileStorage(CalibrationConfig.CALIB_FILE, cv2.FILE_STORAGE_WRITE)
        fs.write('camera_matrix', mtx)
        fs.write('dist_coeff', dist)
        fs.write('rotation_vectors', np.array(rvecs))
        fs.write('translation_vectors', np.array(tvecs))
        fs.write('calibration_error', ret)
        fs.write('calibration_time', time.strftime("%Y%m%d-%H%M%S"))
        fs.write('pattern_type', self.pattern_type)
        
        # Save square size if applicable
        if square_size > 0:
            fs.write('square_size', square_size)
            
        fs.release()
        
        print(f"\nCalibration results saved to {CalibrationConfig.CALIB_FILE}")
    
    def cleanup(self):
        """Cleanup resources."""
        self.camera_capture.stop()
        if self.robot_controller:
            self.robot_controller.close()

    def _calibrate_checkerboard(self, images):
        """Calibrate camera using checkerboard pattern with enhanced detection."""
        print("\nCalibrating with checkerboard pattern...")
        
        # Define checkerboard dimensions
        chess_rows = 6  # Number of internal corners in height direction
        chess_cols = 7  # Number of internal corners in width direction
        
        # Get square size from user with validation
        square_size = 0.015  # Default value in meters (15mm)
        square_size_input = input(f"Enter checkerboard square size in meters (default: {square_size}): ")
        try:
            if square_size_input.strip():
                square_size = float(square_size_input)
            print(f"Using square size: {square_size} meters")
        except:
            print(f"Invalid input. Using default square size: {square_size} meters")
        
        print(f"Looking for {chess_cols}x{chess_rows} internal corners pattern")
        
        # Create object points with actual physical dimensions
        objp = np.zeros((chess_rows * chess_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:chess_cols, 0:chess_rows].T.reshape(-1, 2) * square_size
        
        # Arrays to store object points and image points
        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # Detection flags for better corner detection
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        
        # Visual feedback window
        cv2.namedWindow('Checkerboard Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Checkerboard Detection', 800, 600)
        
        successful_images = 0
        
        for i, img in enumerate(images):
            print(f"Processing image {i+1}/{len(images)}...")
            
            # Convert to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Try enhancing the image for better detection
            gray_enhanced = cv2.equalizeHist(gray)
            
            # Initialize ret_flipped to avoid the error
            ret_flipped = False
            
            # First attempt with original image
            ret, corners = cv2.findChessboardCorners(gray, (chess_cols, chess_rows), flags)
            
            # If not found, try with enhanced image
            if not ret:
                print(f"  Trying with enhanced contrast...")
                ret, corners = cv2.findChessboardCorners(gray_enhanced, (chess_cols, chess_rows), flags)
                
            # If still not found, try with flipped dimensions (in case rows/cols were swapped)
            if not ret:
                print(f"  Trying with flipped dimensions ({chess_rows}x{chess_cols})...")
                ret_flipped, corners = cv2.findChessboardCorners(gray, (chess_rows, chess_cols), flags)
                if ret_flipped:
                    print(f"  Found with flipped dimensions!")
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
                
                # Draw and display corners - fixed to use the correct dimensions based on detection
                cv2.drawChessboardCorners(display_img, 
                                        (chess_rows, chess_cols) if ret_flipped else (chess_cols, chess_rows), 
                                        corners2, ret)
                
                # Save detection result
                filename = f"{CalibrationConfig.CALIB_DIR}/corners_{i+1}.jpg"
                cv2.imwrite(filename, display_img)
                
                # Add success text
                cv2.putText(display_img, f"Image {i+1}: Success!", (30, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                print(f"  No corners found in image {i+1}")
                # Add failure text
                cv2.putText(display_img, f"Image {i+1}: Failed", (30, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Display progress
            cv2.imshow('Checkerboard Detection', display_img)
            key = cv2.waitKey(500)  # Show each result briefly
            if key == 27:  # ESC to cancel
                print("Calibration process canceled by user")
                cv2.destroyAllWindows()
                return None
        
        cv2.destroyAllWindows()
        print(f"\nFound chessboard pattern in {successful_images} out of {len(images)} images")
        
        # Perform camera calibration
        if len(objpoints) > 0:
            print(f"\nPerforming calibration with {len(objpoints)} images...")
            
            # Add calibration flags for better results
            calib_flags = cv2.CALIB_RATIONAL_MODEL
            
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None, flags=calib_flags
            )
            
            print(f"Calibration complete!")
            print(f"Calibration error (RMS): {ret}")
            print(f"Camera matrix:\n{mtx}")
            print(f"Distortion coefficients: {dist.ravel()}")
            
            # Evaluate calibration quality through reprojection error
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                mean_error += error
            
            print(f"Mean reprojection error: {mean_error/len(objpoints)}")
            
            # Save a sample undistorted image for visualization
            if len(images) > 0:
                h, w = images[0].shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
                undistorted = cv2.undistort(images[0], mtx, dist, None, newcameramtx)
                cv2.imwrite(f"{CalibrationConfig.CALIB_DIR}/undistorted_sample.jpg", undistorted)
                print(f"Saved undistorted sample image to {CalibrationConfig.CALIB_DIR}/undistorted_sample.jpg")
            
            return ret, mtx, dist, rvecs, tvecs, square_size
        else:
            print("\nCalibration failed: No valid images with detected patterns")
            print("Troubleshooting tips:")
            print("1. Ensure the checkerboard is fully visible in the images")
            print("2. Try different lighting conditions")
            print(f"3. Verify the checkerboard dimensions ({chess_cols}x{chess_rows} internal corners)")
            print("4. Hold the checkerboard still when capturing images")
            return None

    def _calibrate_circles(self, images):
        """Calibrate camera using circle grid pattern."""
        print("\nCalibrating with circle grid pattern...")
        
        # Circle grid parameters - using symmetric grid pattern
        circle_grid_size = (6, 7)  # Number of circles in width and height
        circle_size = 0.020  # Distance between circles in meters (adjust as needed)
        
        # Prepare object points
        objp = np.zeros((circle_grid_size[0] * circle_grid_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:circle_grid_size[1], 0:circle_grid_size[0]].T.reshape(-1, 2)
        objp *= circle_size
        
        # Arrays to store object points and image points
        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane
        
        for i, img in enumerate(images):
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find circle grid using symmetric grid pattern
            ret, centers = cv2.findCirclesGrid(gray, circle_grid_size, 
                                             flags=cv2.CALIB_CB_SYMMETRIC_GRID)
            
            if ret:
                objpoints.append(objp)
                imgpoints.append(centers)
                
                # Draw and display centers
                cv2.drawChessboardCorners(img, circle_grid_size, centers, ret)
                filename = f"{CalibrationConfig.CALIB_DIR}/circles_{i+1}.jpg"
                cv2.imwrite(filename, img)
                print(f"Found circles in image {i+1}")
            else:
                print(f"No circles found in image {i+1}")
        
        # Perform camera calibration
        if len(objpoints) > 0:
            print(f"\nPerforming calibration with {len(objpoints)} images...")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )
            
            print(f"Calibration error (RMS): {ret}")
            print(f"Camera matrix:\n{mtx}")
            print(f"Distortion coefficients: {dist.ravel()}")
            
            return ret, mtx, dist, rvecs, tvecs
        else:
            print("Calibration failed: No valid images with detected patterns")
            return None

    def _calibrate_charuco(self, images):
        """Calibrate camera using ChArUco pattern."""
        print("\nCalibrating with Charuco pattern...")
        
        all_corners = []
        all_ids = []
        counter = 0
        
        # Create ArUco detector with refined parameters
        arucoParams = aruco.DetectorParameters()
        arucoParams.adaptiveThreshWinSizeMin = 7
        arucoParams.adaptiveThreshWinSizeMax = 23
        arucoParams.adaptiveThreshWinSizeStep = 2
        arucoParams.adaptiveThreshConstant = 7
        arucoParams.minMarkerPerimeterRate = 0.03
        arucoParams.maxMarkerPerimeterRate = 0.5
        arucoParams.polygonalApproxAccuracyRate = 0.05
        arucoParams.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        arucoParams.cornerRefinementWinSize = 5
        arucoParams.cornerRefinementMaxIterations = 30
        arucoParams.cornerRefinementMinAccuracy = 0.1

        # Create detectors
        arucoDetector = aruco.ArucoDetector(ARUCO_DICT, arucoParams)
        charucoDetector = aruco.CharucoDetector(CHARUCO_BOARD)

        # Create window for visualization
        cv2.namedWindow('Detection Window', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Detection Window', Config.WINDOW_WIDTH, Config.WINDOW_HEIGHT)

        for i, img in enumerate(images):
            # Convert to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            markerCorners, markerIds, rejected = arucoDetector.detectMarkers(gray)
            
            if markerIds is not None and len(markerIds) > 0:
                # Draw detected markers
                img_markers = cv2.aruco.drawDetectedMarkers(img.copy(), markerCorners, markerIds)
                
                # Detect Charuco corners
                result = charucoDetector.detectBoard(
                    gray,
                    charucoCorners=None,
                    charucoIds=None,
                    markerCorners=markerCorners,
                    markerIds=markerIds
                )
                
                if result[0] is not None and result[1] is not None and len(result[1]) > 4:
                    charucoCorners = result[0]
                    charucoIds = result[1]
                    
                    all_corners.append(charucoCorners)
                    all_ids.append(charucoIds)
                    counter += 1
                    
                    # Draw detection results
                    debug_image = img_markers.copy()
                    cv2.aruco.drawDetectedCornersCharuco(debug_image, charucoCorners, charucoIds)
                    
                    # Add detection count
                    cv2.putText(debug_image, f"Detected: {len(charucoIds)} corners", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                               1, (0, 255, 0), 2)
                    
                    # Save debug image
                    filename = f"{CalibrationConfig.CALIB_DIR}/charuco_debug_{i+1}.jpg"
                    cv2.imwrite(filename, debug_image)
                    print(f"Found {len(charucoIds)} charuco corners in image {i+1}")
                    
                    cv2.imshow('Detection Window', debug_image)
                else:
                    print(f"No complete charuco board found in image {i+1}")
                    cv2.imshow('Detection Window', img_markers)
            else:
                print(f"No markers found in image {i+1}")
                cv2.imshow('Detection Window', img)
            
            cv2.waitKey(Config.WAIT_TIME)

        cv2.destroyAllWindows()
        print(f"Successfully processed {counter} out of {len(images)} images")

        if len(all_corners) > 0:
            try:
                # Calibration flags
                flags = (
                    cv2.CALIB_ZERO_TANGENT_DIST +
                    cv2.CALIB_FIX_K3 +
                    cv2.CALIB_USE_LU
                )
                
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
                    return ret, mtx, dist, rvecs, tvecs
            except cv2.error as e:
                print(f"Calibration failed: {str(e)}")
                print("Try capturing more images with better marker visibility")
        
        print("\nTroubleshooting tips:")
        print("1. Ensure good, even lighting")
        print("2. Hold the board still when capturing")
        print("3. Capture the entire board in each image")
        print("4. Try different viewing angles")
        print("5. Check if board dimensions match Config settings")
        return None