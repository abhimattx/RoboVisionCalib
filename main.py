"""
Camera Calibration and Robot Control System - Main Entry Point

This module provides the main program entry point and user interface.
"""

import os
import numpy as np
import cv2
from config.settings import Config, CalibrationConfig
from camera.camera import CameraCapture
from robot.controller import RobotController
from calibration.calibrator import CalibrationProcess
from calibration.handeye import perform_hand_eye_calibration
import time
import math

# Add new imports
from camera.detection import ArUcoDetector
from tasks.pick_place import PickAndPlaceTask

def create_directories():
    """Create necessary directories for storing images."""
    dirs = [Config.CALIB_DIR, Config.ARUCO_DIR, Config.CIRCLE_DIR]
    for directory in dirs:
        os.makedirs(directory, exist_ok=True)

def main():
    """Main program entry point."""
    # Create necessary directories
    create_directories()
    
    # Print OpenCV version for diagnostics
    print(f"OpenCV version: {cv2.__version__}")
    
    while True:
        print("\nCamera Calibration System")
        print("------------------------")
        print("1. Calibrate with Checkerboard")
        print("2. Calibrate with Circle Grid")
        print("3. Calibrate with Charuco")
        print("4. Test Mode (No Robot)")
        print("5. Perform Hand-Eye Calibration")
        print("6. ArUco Detection Test")
        print("7. Pick and Place with ArUco")
        print("8. Automated Pick and Place")  # New option
        print("9. Exit")                      # Changed from 8 to 9
        
        choice = input("\nEnter your choice (1-9): ")
        
        if choice == '1':
            calibrator = CalibrationProcess(CalibrationConfig.PATTERN_CHECKERBOARD)
            calibrator.run_calibration()
        elif choice == '2':
            calibrator = CalibrationProcess(CalibrationConfig.PATTERN_CIRCLES)
            calibrator.run_calibration()
        elif choice == '3':
            calibrator = CalibrationProcess(CalibrationConfig.PATTERN_CHARUCO)
            calibrator.run_calibration()
        elif choice == '4':
            pattern = input("Select pattern (1-Checkerboard, 2-Circles, 3-Charuco): ")
            if pattern == '1':
                pattern_type = CalibrationConfig.PATTERN_CHECKERBOARD
            elif pattern == '2':
                pattern_type = CalibrationConfig.PATTERN_CIRCLES
            elif pattern == '3':
                pattern_type = CalibrationConfig.PATTERN_CHARUCO
            else:
                print("Invalid pattern selection")
                continue
            
            calibrator = CalibrationProcess(pattern_type, test_mode=True)
            calibrator.run_calibration()
        elif choice == '5':
            # Add hand-eye calibration option
            method = input("Select calibration method (1-TSAI, 2-PARK, 3-HORAUD, 4-ANDREFF, 5-DANIILIDIS): ")
            methods = {
                '1': 'TSAI',
                '2': 'PARK',
                '3': 'HORAUD',
                '4': 'ANDREFF',
                '5': 'DANIILIDIS'
            }
            if method in methods:
                perform_hand_eye_calibration(
                    robot_config_file="configurations.yaml",
                    method=methods[method]
                )
            else:
                print("Invalid method selection, using default (TSAI)")
                perform_hand_eye_calibration(robot_config_file="configurations.yaml")
        elif choice == '6':
            # ArUco detection test with enhanced visualization
            print("\nArUco Marker Detection Test")
            print("--------------------------")
            
            camera = CameraCapture()
            if not camera.start():  # Initialize camera and check success
                print("Failed to initialize camera")
                continue
            
            # Create a more configurable detector
            dict_type = cv2.aruco.DICT_6X6_250  # Default dictionary
            print("\nSelect ArUco dictionary type:")
            print("1. DICT_4X4_50")
            print("2. DICT_5X5_50")
            print("3. DICT_6X6_250 (default)")
            print("4. DICT_7X7_50")
            dict_choice = input("Enter choice (or press Enter for default): ")

            # Map choices to dictionary types for OpenCV 4.11.0+
            dict_types = {
                '1': cv2.aruco.DICT_4X4_50,
                '2': cv2.aruco.DICT_5X5_50,
                '3': cv2.aruco.DICT_6X6_250,
                '4': cv2.aruco.DICT_7X7_50
            }

            dict_type = dict_types.get(dict_choice, cv2.aruco.DICT_6X6_250)
            
            # Get marker size
            size_str = input("Enter marker size in mm (default 50): ")
            marker_size = 50.0  # Default size
            if size_str.strip():
                try:
                    marker_size = float(size_str)
                except ValueError:
                    print("Invalid size, using default 50mm")
            
            detector = ArUcoDetector(dict_type=dict_type, marker_size=marker_size)
            
            # Load camera calibration
            try:
                fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_READ)
                camera_matrix = fs.getNode('camera_matrix').mat()
                dist_coeffs = fs.getNode('dist_coeffs').mat()
                fs.release()
                print("Loaded camera calibration successfully")
            except Exception as e:
                print(f"Using default camera parameters: {e}")
                camera_matrix = np.array([
                    [Config.FX, 0, Config.CX],
                    [0, Config.FY, Config.CY],
                    [0, 0, 1]
                ])
                dist_coeffs = np.array(Config.DIST_COEFFS)
            
            print("\nControls:")
            print("- Press 'q' to exit")
            print("- Press 's' to save a snapshot")
            print("- Press 'd' to toggle debug view")
            
            debug_mode = False
            frame_count = 0
            detect_count = 0
            
            while True:
                # Get frame from camera
                frame = camera.get_frame()
                if frame is None:
                    print("Error capturing frame")
                    break
                
                frame_count += 1
                original_frame = frame.copy()
                
                # Process frame for better detection (optional)
                # You might want to add image processing steps here
                
                # Detect markers
                corners, ids, rejected = detector.detect_markers(frame)
                
                # Create display frame
                display_frame = frame.copy()
                
                # Debug information
                cv2.putText(display_frame, f"Frame: {frame_count}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Detection and visualization
                if ids is not None:
                    detect_count += 1
                    marker_poses = detector.estimate_poses(corners, ids, camera_matrix, dist_coeffs)
                    # Draw detected markers first
                    display_frame = detector.draw_markers(display_frame, corners, ids)

                    # Then manually draw axes for each marker
                    if marker_poses:
                        for marker_id, (rvec, tvec) in marker_poses.items():
                            try:
                                # Draw 3D axes
                                cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)
                                
                                # Add text with position information
                                idx = np.where(ids == marker_id)[0]
                                if len(idx) > 0:
                                    marker_idx = idx[0]
                                    corner = corners[marker_idx][0][0]
                                    text_pos = (int(corner[0]), int(corner[1] - 10))
                                    text = f"ID:{marker_id} x:{float(tvec[0]):.1f} y:{float(tvec[1]):.1f} z:{float(tvec[2]):.1f}"
                                    cv2.putText(display_frame, text, text_pos,
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            except Exception as e:
                                print(f"Error drawing axes for marker {marker_id}: {e}")
                    
                    # Print marker information (limit frequency to reduce console spam)
                    if frame_count % 10 == 0:
                        print(f"Detected {len(ids)} markers:")
                        for marker_id, (rvec, tvec) in marker_poses.items():
                            print(f"  Marker {marker_id}: Position (mm) = [{float(tvec[0]):.1f}, {float(tvec[1]):.1f}, {float(tvec[2]):.1f}]")
                else:
                    cv2.putText(display_frame, "No markers detected", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Show detection rate
                if frame_count % 30 == 0:  # Update every 30 frames
                    detection_rate = (detect_count / frame_count) * 100
                    cv2.putText(display_frame, f"Detection rate: {detection_rate:.1f}%", 
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                # Show debug view if enabled
                if debug_mode:
                    # Convert to grayscale for visualization
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    # Apply adaptive threshold to see what the detector might be seeing
                    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                 cv2.THRESH_BINARY, 11, 2)
                    # Show the threshold image
                    cv2.imshow("Debug View", thresh)
                    
                    # Display rejected candidates if any
                    if rejected is not None and len(rejected) > 0:
                        rejected_frame = original_frame.copy()
                        rejected_frame = cv2.aruco.drawDetectedMarkers(rejected_frame, rejected, None, (0, 0, 255))
                        cv2.imshow("Rejected Candidates", rejected_frame)
                
                # Show the result
                cv2.imshow("ArUco Detection", display_frame)
                
                # Check for key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    # Save the current frame
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"aruco_detection_{timestamp}.jpg", display_frame)
                    print(f"Saved frame as aruco_detection_{timestamp}.jpg")
                elif key == ord('d'):
                    # Toggle debug mode
                    debug_mode = not debug_mode
                    if not debug_mode:
                        cv2.destroyWindow("Debug View")
                        if cv2.getWindowProperty("Rejected Candidates", cv2.WND_PROP_VISIBLE) > 0:
                            cv2.destroyWindow("Rejected Candidates")
            
            # Clean up
            cv2.destroyAllWindows()
            camera.stop()
            print(f"Detection session ended. Detected markers in {detect_count} of {frame_count} frames ({(detect_count/frame_count)*100:.1f}%)")
            
        elif choice == '7':
            # Pick and Place with ArUco
            print("\nPick and Place with ArUco")
            print("-----------------------")
            
            # Check if calibration files exist
            if not os.path.exists("camera_calibration.yaml"):
                print("Error: Camera calibration not found. Please run camera calibration first.")
                continue
                
            if not os.path.exists("hand_eye_calibration.yaml"):
                print("Error: Hand-eye calibration not found. Please run hand-eye calibration first.")
                continue
            
            # Initialize components
            camera = CameraCapture()
            print("Initializing camera...")
            if not camera.start():
                print("Failed to initialize camera")
                continue
            
            # Test camera by getting a test frame
            test_frame = camera.get_frame()
            if test_frame is None:
                print("Camera initialized but no frames available. Check camera connections.")
                camera.stop()
                continue
            else:
                print(f"Camera test successful. Got frame of size {test_frame.shape}")

            # Add camera readiness delay and warm-up
            print("Warming up camera...")
            for _ in range(10):  # Get several frames to "warm up" the camera
                frame = camera.get_frame()
                if frame is not None:
                    cv2.imshow("Camera Test", frame)
                    cv2.waitKey(1)
            time.sleep(1)  # Give the camera a moment to stabilize
            cv2.destroyWindow("Camera Test")
            
            # Load camera calibration
            try:
                fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_READ)
                camera_matrix = fs.getNode('camera_matrix').mat()
                dist_coeffs = fs.getNode('dist_coeffs').mat()
                fs.release()
                print("Loaded camera calibration successfully")
            except Exception as e:
                print(f"Error loading camera calibration: {e}")
                camera.stop()
                continue
            
            # Create detector for visualization
            detector = ArUcoDetector(dict_type=cv2.aruco.DICT_6X6_250, marker_size=50.0)
            
            # Initialize robot
            robot = RobotController()
            print("Connecting to robot...")
            if not robot.connect():
                print("Failed to connect to robot. Would you like to continue in simulation mode? (y/n)")
                if input().lower() != 'y':
                    print("Aborting pick and place operation")
                    camera.stop()
                    continue
                print("Continuing in simulation mode")
            else:
                print("Robot connected successfully")
            
            # Create shared resources for thread communication
            import threading
            
            # This lock protects shared resources between threads
            frame_lock = threading.Lock()
            # This will store the latest frame from the camera
            latest_frame = None
            # Flag to indicate when pick and place is complete
            pick_place_done = threading.Event()
            # Flag to indicate when a new frame is available
            new_frame_available = threading.Event()
            # Flag to signal the task thread to stop
            stop_task = threading.Event()
            
            # Modify PickAndPlaceTask to use shared frame
            class ThreadSafePickAndPlaceTask(PickAndPlaceTask):
                def detect_marker(self, marker_id):
                    """Detect a specific ArUco marker using shared frame."""
                    max_attempts = 30
                    frames_with_no_data = 0
                    max_no_data_frames = 10  # After this many consecutive empty frames, exit with error
                    
                    for attempt in range(max_attempts):
                        # Check if we should stop
                        if stop_task.is_set():
                            print("Detection stopped by user")
                            return None
                        
                        # Use the shared frame
                        frame = None
                        with frame_lock:
                            if latest_frame is None:
                                frames_with_no_data += 1
                                print(f"No frame available (count: {frames_with_no_data})")
                                
                                # Exit if we've had too many consecutive empty frames
                                if frames_with_no_data >= max_no_data_frames:
                                    print("Too many consecutive empty frames, aborting detection")
                                    return None
                            else:
                                frames_with_no_data = 0  # Reset counter when we
                                # Make a deep copy to work with
                                frame = latest_frame.copy()
                        
                        # If no valid frame was obtained, wait and try again
                        if frame is None:
                            time.sleep(0.2)  # Longer wait between retries
                            continue
                            
                        # Show the frame that's being processed
                        cv2.imshow("Current Detection Frame", frame)
                        cv2.waitKey(1)
                        
                        # Detect markers
                        corners, ids, _ = self.detector.detect_markers(frame)
                        if ids is not None and marker_id in ids.flatten():
                            # Found the marker we're looking for
                            print(f"Found marker {marker_id} on attempt {attempt+1}")
                            marker_poses = self.detector.estimate_poses(corners, ids, 
                                                                     self.camera_matrix, 
                                                                     self.dist_coeffs)
                            return marker_poses.get(marker_id)
                        
                        print(f"Attempt {attempt+1}/{max_attempts}: Marker {marker_id} not found")
                        time.sleep(0.2)
                    
                    print(f"Failed to detect marker {marker_id} after {max_attempts} attempts")
                    return None
            
            # Create task with thread-safe detection
            task = ThreadSafePickAndPlaceTask(robot, camera)
            
            # Get marker ID
            marker_id = int(input("Enter marker ID to pick: "))

            # Enter offset from marker to grasp point (if any)
            use_offset = input("Use offset from marker to grasp point? (y/n): ")
            if use_offset.lower() == 'y':
                x_offset = float(input("Offset X (mm): "))
                y_offset = float(input("Offset Y (mm): "))
                z_offset = float(input("Offset Z (mm): "))
                marker_to_object = np.array([x_offset, y_offset, z_offset])
            else:
                marker_to_object = np.array([0, 0, 0])
            
            # Get place position
            print("\nEnter place position:")
            place_x = float(input("X position (mm): "))
            place_y = float(input("Y position (mm): "))
            place_z = float(input("Z position (mm): "))
            place_position = np.array([place_x, place_y, place_z])
            
            # Add grasp height verification
            marker_pose = task.detect_marker(marker_id)
            if marker_pose is not None:
                object_position = task.calculate_object_position(marker_pose, marker_to_object)
                print(f"Raw detected position: {object_position}")
                
                if object_position[2] < 15:
                    print(f"Warning: Detected Z height ({object_position[2]:.2f}mm) is too low for safe grasping.")
                    adjust = input("Would you like to adjust the Z height? (y/n): ")
                    if adjust.lower() == 'y':
                        new_z = float(input("Enter desired Z height in mm (recommended: 15-30): "))
                        marker_to_object[2] += (new_z - object_position[2])
                        print(f"Updated marker-to-object offset: {marker_to_object}")
            
            # Function for pick and place thread
            def run_pick_place():
                try:
                    success = task.run_pick_and_place(marker_id, place_position, marker_to_object)
                    if success:
                        print("Pick and place operation completed successfully")
                    else:
                        print("Pick and place operation failed")
                except Exception as e:
                    print(f"Error during pick and place: {e}")
                finally:
                    pick_place_done.set()
            
            # Start pick and place in separate thread
            pick_place_thread = threading.Thread(target=run_pick_place)
            pick_place_thread.daemon = True
            pick_place_thread.start()
            
            print("\nShowing live camera view. Press 'q' to stop.")
            print("Pick and place operation running in background...")
            
            # Main camera capture loop - only this thread will directly access the camera
            frame_count = 0
            empty_frame_count = 0

            while not pick_place_done.is_set():
                # Get frame from camera (only in main thread)
                frame = camera.get_frame()
                
                if frame is not None:
                    frame_count += 1
                    # Update the shared frame for the detection thread
                    with frame_lock:
                        latest_frame = frame.copy()
                        new_frame_available.set()  # Signal that a new frame is ready
                    
                    # Create display frame with marker detection
                    display_frame = frame.copy()
                    
                    # Detect markers
                    corners, ids, _ = detector.detect_markers(frame)
                    
                    # Draw markers and highlight target marker
                    if ids is not None:
                        # Draw all detected markers
                        display_frame = detector.draw_markers(display_frame, corners, ids)
                        
                        # Estimate poses
                        marker_poses = detector.estimate_poses(corners, ids, camera_matrix, dist_coeffs)
                        
                        # Draw axes and highlight target marker
                        if marker_poses:
                            for mid, (rvec, tvec) in marker_poses.items():
                                try:
                                    # Draw 3D axes
                                    cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec, tvec, 25)
                                    
                                    # Add position text
                                    idx = np.where(ids == mid)[0]
                                    if len(idx) > 0:
                                        marker_idx = idx[0]
                                        corner = corners[marker_idx][0][0]
                                        text_pos = (int(corner[0]), int(corner[1] - 10))
                                        
                                        # Highlight target marker
                                        if mid == marker_id:
                                            text = f"TARGET ID:{mid} x:{float(tvec[0]):.1f} y:{float(tvec[1]):.1f} z:{float(tvec[2]):.1f}"
                                            cv2.putText(display_frame, text, text_pos,
                                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                                            
                                            # Draw pick position with offset
                                            if any(marker_to_object):
                                                # Calculate grasp position in camera frame
                                                R, _ = cv2.Rodrigues(rvec)
                                                grasp_point = tvec + R @ marker_to_object.reshape(3, 1)
                                                
                                                # Project grasp point to image
                                                grasp_2d, _ = cv2.projectPoints(grasp_point, np.zeros(3), np.zeros(3), 
                                                                             camera_matrix, dist_coeffs)
                                                grasp_2d = grasp_2d.reshape(-1, 2)
                                                
                                                # Draw grasp point
                                                cv2.drawMarker(display_frame, (int(grasp_2d[0][0]), int(grasp_2d[0][1])), 
                                                               (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                                                cv2.putText(display_frame, "Grasp", (int(grasp_2d[0][0])+10, int(grasp_2d[0][1])+10), 
                                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                                        else:
                                            text = f"ID:{mid} x:{float(tvec[0]):.1f} y:{float(tvec[1]):.1f} z:{float(tvec[2]):.1f}"
                                            cv2.putText(display_frame, text, text_pos,
                                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                except Exception as e:
                                    print(f"Error drawing marker {mid}: {e}")
                    
                    # Add status information
                    cv2.putText(display_frame, "Pick and place in progress...", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # Show the live feed
                    cv2.imshow("Pick and Place Camera View", display_frame)
                    
                    # Check for key press to exit
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        stop_task.set()  # Signal detection thread to stop
                        break
                else:
                    empty_frame_count += 1
                    if empty_frame_count % 10 == 0:  # Only print every 10th empty frame
                        print(f"Warning: No frame available ({empty_frame_count} total empty frames)")
                    time.sleep(0.05)  # Small sleep to prevent CPU hogging
                
                # Status update
                if frame_count % 50 == 0:
                    success_rate = 100 * (frame_count / (frame_count + empty_frame_count)) if frame_count > 0 else 0
                    print(f"Camera stats: {frame_count} frames, {empty_frame_count} empty frames ({success_rate:.1f}% success)")
            
            # Wait for pick and place to finish if it hasn't already
            if not pick_place_done.is_set():
                print("Waiting for pick and place operation to complete...")
                pick_place_done.wait()
            
            # Clean up
            cv2.destroyAllWindows()
            camera.stop()
            print("Pick and place operation complete")
            
        elif choice == '8':
            # Run the fully automated pick and place with correct marker size and dictionary
            print("Running automated pick and place with 5x5 dictionary and 30mm marker")
            run_automated_pick_and_place()
            
        elif choice == '9':  # Changed from 8 to 9
            break
        else:
            print("Invalid choice. Please try again.")

def run_automated_pick_and_place():
    """Fully automated pick and place with no user input and improved detection."""
    print("\nAutomated Pick and Place")
    print("----------------------")
    
    # Check if calibration files exist
    if not os.path.exists("camera_calibration.yaml"):
        print("Error: Camera calibration not found. Please run camera calibration first.")
        return
        
    if not os.path.exists("hand_eye_calibration.yaml"):
        print("Error: Hand-eye calibration not found. Please run hand-eye calibration first.")
        return
    
    # Initialize components
    camera = CameraCapture()
    print("Initializing camera...")
    if not camera.start():
        print("Failed to initialize camera")
        return
    
    # Test camera by getting a test frame
    test_frame = camera.get_frame()
    if test_frame is None:
        print("Camera initialized but no frames available. Check camera connections.")
        camera.stop()
        return
    
    print(f"Camera test successful. Got frame of size {test_frame.shape}")

    # Load camera calibration
    try:
        fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode('camera_matrix').mat()
        dist_coeffs = fs.getNode('dist_coeffs').mat()
        fs.release()
        print("Loaded camera calibration successfully")
    except Exception as e:
        print(f"Error loading camera calibration: {e}")
        camera.stop()
        return
    
    # Load hand-eye calibration
    try:
        fs = cv2.FileStorage("hand_eye_calibration.yaml", cv2.FILE_STORAGE_READ)
        T_cam2base = fs.getNode('transform').mat()
        fs.release()
        print("Loaded hand-eye calibration successfully")
    except Exception as e:
        print(f"Error loading hand-eye calibration: {e}")
        camera.stop()
        return
    
    # Create a window for visualization
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Detection", 1024, 768)
    
    # Updated dictionary types for OpenCV 4.11.0+
    dict_types = [
        cv2.aruco.DICT_5X5_50,  # Start with your marker type (5x5)
        cv2.aruco.DICT_4X4_50,
        cv2.aruco.DICT_6X6_250,
        cv2.aruco.DICT_7X7_50
    ]
    dict_names = ["5x5_50", "4x4_50", "6x6_250", "7x7_50"]
    
    print("Testing different ArUco dictionaries...")
    
    # Define cube offset parameters - tune these values based on your setup
    # Offset from marker center to object center for grasping
    marker_to_object_offset = np.array([0.0, 0.0, 15.0])  # [x, y, z] in mm
    
    # Define approach and retreat distances
    approach_distance = 50.0  # mm above the target before final approach
    retreat_distance = 50.0   # mm above the picked object before moving to place
    
    # Gripper parameters
    gripper_offset = np.array([0.0, 0.0, 15.0])  # Offset from tool center to gripper tip
    
    # Safety parameters
    minimum_safe_z = 30  # Minimum safe Z height in mm
    
    for dict_idx, dict_type in enumerate(dict_types):
        # Initialize ArUco detector with current dictionary - use your correct marker size (30mm)
        print(f"Trying dictionary: {dict_names[dict_idx]}")
        detector = ArUcoDetector(dict_type=dict_type, marker_size=30.0)
        
        # Marker detection parameters
        max_attempts = 10  # Fewer attempts per dictionary
        
        # Capture several frames for analysis with this dictionary
        print(f"Looking for markers with dictionary {dict_names[dict_idx]}...")
        
        # Try to detect a marker
        for attempt in range(max_attempts):
            frame = camera.get_frame()
            if frame is None:
                print(f"No frame available (attempt {attempt+1}/{max_attempts})")
                time.sleep(0.2)
                continue
                
            # Try different image processing to improve detection
            display_frame = frame.copy()
            processed_frames = []
            
            # 1. Original frame
            processed_frames.append(("Original", frame.copy()))
            
            # 2. Grayscale conversion
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            processed_frames.append(("Grayscale", cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)))
            
            # 3. Adaptive threshold
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY, 11, 2)
            processed_frames.append(("Threshold", cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)))
            
            # 4. Enhanced contrast
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced = clahe.apply(gray)
            processed_frames.append(("Enhanced", cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)))
            
            # Show all processing methods
            montage_width = 2
            montage_height = math.ceil(len(processed_frames) / montage_width)
            montage = np.zeros((frame.shape[0] * montage_height, frame.shape[1] * montage_width, 3), dtype=np.uint8)
            
            for i, (label, proc_frame) in enumerate(processed_frames):
                row = i // montage_width
                col = i % montage_width
                montage[row*frame.shape[0]:(row+1)*frame.shape[0], 
                       col*frame.shape[1]:(col+1)*frame.shape[1]] = proc_frame
                cv2.putText(montage, label, (col*frame.shape[1] + 10, row*frame.shape[0] + 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Try detection on all processed images - For OpenCV 4.11.0+
            best_result = None
            
            for label, proc_frame in processed_frames:
                # Convert back to grayscale if needed for detection
                if len(proc_frame.shape) == 3:
                    detect_frame = cv2.cvtColor(proc_frame, cv2.COLOR_BGR2GRAY)
                else:
                    detect_frame = proc_frame
                    
                # Updated ArUco detection for OpenCV 4.11.0+
                dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
                parameters = cv2.aruco.DetectorParameters()
                parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                
                # Create detector
                detector_aruco = cv2.aruco.ArucoDetector(dictionary, parameters)
                
                # Detect markers
                corners, ids, rejected = detector_aruco.detectMarkers(detect_frame)
                
                if ids is not None and len(ids) > 0:
                    print(f"Found marker(s) with {label} processing!")
                    best_result = (detect_frame, corners, ids, rejected, label)
                    break
            
            # Display results
            display_frame = montage.copy()
            cv2.putText(display_frame, f"Dictionary: {dict_names[dict_idx]} - Attempt {attempt+1}/{max_attempts}", 
                       (10, montage.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Detection", display_frame)
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                print("Detection cancelled by user")
                camera.stop()
                cv2.destroyAllWindows()
                return
            
            # Process detection results if found - For OpenCV 4.11.0+
            if best_result:
                detect_frame, corners, ids, rejected, method = best_result
                
                print(f"Success! Found marker(s) with IDs: {ids.flatten()} using {method} processing")
                
                # Get marker pose in camera frame
                marker_id = ids[0][0]  # Use the first detected marker
                
                # Use the detector's pose estimation with the right frame - For OpenCV 4.11.0+
                if method == "Original":
                    # Use original frame for pose estimation
                    marker_poses = detector.estimate_poses(corners, ids, camera_matrix, dist_coeffs)
                else:
                    # Need to re-detect on original frame for proper pose estimation
                    gray_orig = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    
                    # Updated ArUco detection
                    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
                    parameters = cv2.aruco.DetectorParameters()
                    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
                    detector_aruco = cv2.aruco.ArucoDetector(dictionary, parameters)
                    corners_orig, ids_orig, _ = detector_aruco.detectMarkers(gray_orig)
                    
                    marker_poses = detector.estimate_poses(corners_orig, ids_orig, camera_matrix, dist_coeffs)
                
                if not marker_id in marker_poses:
                    print(f"Error: Could not estimate pose for marker {marker_id}")
                    continue
                    
                rvec, tvec = marker_poses[marker_id]
                
                # Draw the detected marker on a result frame
                result_frame = frame.copy()
                if method == "Original":
                    result_frame = detector.draw_markers(result_frame, corners, ids)
                else:
                    # Use original frame corners
                    result_frame = detector.draw_markers(result_frame, corners_orig, ids_orig)
                    
                cv2.drawFrameAxes(result_frame, camera_matrix, dist_coeffs, rvec, tvec, 25)
                
                # Connect to the robot
                robot = RobotController()
                print("Connecting to robot...")
                if not robot.connect():
                    print("Failed to connect to robot.")
                    camera.stop()
                    cv2.destroyAllWindows()
                    return
                
                print("Robot connected successfully")
                
                # Convert marker position to robot base frame
                from scipy.spatial.transform import Rotation as R
                
                R_marker, _ = cv2.Rodrigues(rvec)
                T_cam2marker = np.eye(4)
                T_cam2marker[:3, :3] = R_marker
                T_cam2marker[:3, 3] = tvec.flatten()

                # Build homogeneous offset in marker frame (grasp point relative to marker center)
                object_point_in_marker = np.array([
                    marker_to_object_offset[0],
                    marker_to_object_offset[1],
                    marker_to_object_offset[2],
                    1.0  # Homogeneous coordinate
                ])


                
                # Calculate the object position with the marker_to_object_offset
                # Apply the offset in marker frame
                object_point_in_marker = np.array([marker_to_object_offset[0], marker_to_object_offset[1], marker_to_object_offset[2], 1])
                object_point_in_camera = T_cam2marker @ object_point_in_marker
                
                # Transform into robot base frame
                object_point_in_base = T_cam2base @ object_point_in_camera
                
                # Extract target position for grasping
                target_position = object_point_in_base[:3]
                
                # Apply safety check for Z coordinate
                if target_position[2] < minimum_safe_z:
                    print(f"Warning: Z coordinate too low ({target_position[2]:.2f}mm), adjusting to {minimum_safe_z}mm")
                    target_position[2] = minimum_safe_z
                
                # Show current target on image with detailed coordinates
                detailed_frame = result_frame.copy()
                cv2.putText(detailed_frame, f"DETECTED POSITION:", (10, 210),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(detailed_frame, f"X: {target_position[0]:.2f} mm", (10, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                cv2.putText(detailed_frame, f"Y: {target_position[1]:.2f} mm", (10, 270),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                cv2.putText(detailed_frame, f"Z: {target_position[2]:.2f} mm", (10, 300),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                cv2.putText(detailed_frame, "Press 'a' to adjust or 'c' to continue", (10, 330),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Position Adjustment", detailed_frame)
                
                # Always adjust position first
                print("\n--- Position Adjustment Required ---")
                print(f"Current target position: X={target_position[0]:.2f}, Y={target_position[1]:.2f}, Z={target_position[2]:.2f}")
                
                # Wait for key press to adjust or continue
                key = cv2.waitKey(0) & 0xFF
                
                # If user wants to adjust
                if key == ord('a'):
                    # Add manual adjustment offsets
                    try:
                        x_offset = float(input("Enter X offset adjustment (mm): "))
                        target_position[0] += x_offset
                        
                        z_offset = float(input("Enter Z offset adjustment (mm): "))
                        target_position[2] += z_offset
                        
                        print(f"Adjusted target position: X={target_position[0]:.2f}, Y={target_position[1]:.2f}, Z={target_position[2]:.2f}")
                        
                        # Apply safety check again after adjustment
                        if target_position[2] < minimum_safe_z:
                            print(f"Warning: Z coordinate too low ({target_position[2]:.2f}mm), adjusting to {minimum_safe_z}mm")
                            target_position[2] = minimum_safe_z
                    except ValueError as e:
                        print(f"Invalid input: {e}. Using detected position.")
                        
                cv2.destroyWindow("Position Adjustment")
                
                # Print detected and calculated positions
                print(f"Marker position: [{tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f}]")
                print(f"Final target position: [{target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f}]")
                
                # Display the positions on the result frame
                cv2.putText(result_frame, f"Marker: [{tvec[0][0]:.1f}, {tvec[1][0]:.1f}, {tvec[2][0]:.1f}]", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.putText(result_frame, f"Target: [{target_position[0]:.1f}, {target_position[1]:.1f}, {target_position[2]:.1f}]", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(result_frame, "Moving to pick position...", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Detection Result", result_frame)
                cv2.waitKey(1000)  # Show for 1 second
                
                # Use a downward-facing orientation for grasping
                down_quaternion = np.array([0.0, 0.0, 1.0, 0.0])  # Tool Z pointing down
                
                # Send single pick command
                print("Moving to pick position...")
                success = robot.send_position_command_simple(
                    command_prefix="pick_", 
                    position=target_position.reshape(3,1),
                    quaternion=down_quaternion,
                    timeout=40
                )
                
                if not success:
                    print("Failed to reach pick position")
                    cv2.putText(result_frame, "Pick position failed!", (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.imshow("Detection Result", result_frame)
                    cv2.waitKey(2000)
                    cv2.destroyAllWindows()
                    camera.stop()
                    return
                else:
                    print("Pick operation completed successfully!")
                    cv2.putText(result_frame, "Pick operation successful!", (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display final result
                cv2.imshow("Detection Result", result_frame)
                cv2.waitKey(3000)  # Show result for 3 seconds
                cv2.destroyAllWindows()
                camera.stop()
                print("Automated pick operation complete")
                return
        
        print(f"No markers found with dictionary {dict_names[dict_idx]}")
    
    print("Failed to detect any markers with all dictionaries")
    cv2.destroyAllWindows()
    camera.stop()
    print("Automated pick and place operation complete")

if __name__ == "__main__":
    main()