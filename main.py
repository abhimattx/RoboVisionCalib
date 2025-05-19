##last update

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
from scipy.spatial.transform import Rotation as RR

import camera.detection as detection 
from config.settings import Config

# Add new imports
from camera.detection import ArUcoDetector
from tasks.pick_place import PickAndPlaceTask

def create_directories():
    """Create necessary directories for storing images."""
    dirs = [Config.CALIB_DIR, Config.ARUCO_DIR, Config.CIRCLE_DIR]
    for directory in dirs:
        os.makedirs(directory, exist_ok=True)

def quaternion_multiply(quaternion1, quaternion0):
    """
    Multiply two quaternions [w, x, y, z]
    This can be used to combine rotations
    """
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([
        w1*w0 - x1*x0 - y1*y0 - z1*z0,  # w
        w1*x0 + x1*w0 + y1*z0 - z1*y0,  # x
        w1*y0 - x1*z0 + y1*w0 + z1*x0,  # y
        w1*z0 + x1*y0 - y1*x0 + z1*w0   # z
    ], dtype=np.float64)

def verify_top_approach(quaternion):
    """
    Verify that the quaternion will result in a top-down approach.
    
    Args:
        quaternion: Quaternion in [w, x, y, z] format
        
    Returns:
        bool: True if the approach is from the top, False otherwise
    """
    # Convert quaternion to rotation matrix
    qw, qx, qy, qz = quaternion
    rotation_matrix = np.array([
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
    ])
    
    # Extract z-axis direction (third column) - this is the approach direction
    z_axis = rotation_matrix[:, 2]
    
    # Check if z-axis points downward (negative Z in base frame)
    # For a perfect top-down approach, z_axis[2] should be close to -1
    z_down = z_axis[2] < -0.9  # Should be close to -1 for directly downward
    
    print(f"Approach direction check:")
    print(f"Z-axis vector: [{z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f}]")
    print(f"Is top-down approach: {z_down}")
    
    return z_down

# Add this function before main() function:
def convert_to_z_up_orientation(rvec, tvec):
    """
    Convert ArUco marker pose (where Z points toward camera) to Z-up orientation
    where Z points upward and X/Y lie in the horizontal plane.
    
    Args:
        rvec: Rotation vector in camera frame
        tvec: Translation vector in camera frame
        
    Returns:
        rvec_z_up: Rotation vector with Z-up orientation
        tvec_z_up: Translation vector in Z-up frame
    """
    # Convert rotation vector to rotation matrix
    R_marker2cam, _ = cv2.Rodrigues(rvec)
    
    # Define the rotation to convert from marker's orientation (Z toward camera)
    # to Z-up orientation (90 degree rotation around X-axis)
    # This makes the marker's Z point upward
    R_z_up = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])
    
    # Apply the rotation to the marker's orientation
    R_z_up_marker2cam = R_marker2cam @ R_z_up
    
    # Convert back to rotation vector
    rvec_z_up, _ = cv2.Rodrigues(R_z_up_marker2cam)
    
    # Translation remains the same in this case
    tvec_z_up = tvec.copy()
    
    return rvec_z_up, tvec_z_up

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
                                # Draw original 3D axes (camera frame, Z toward camera)
                                cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)
                                
                                # Convert to Z-up orientation
                                rvec_z_up, tvec_z_up = convert_to_z_up_orientation(rvec, tvec)
                                
                                # Draw Z-up axes in a different position (slightly offset for visibility)
                                tvec_offset = tvec.copy()
                                tvec_offset[0] += 10  # Offset in X direction
                                cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec_z_up, tvec_offset, marker_size/2)
                                
                                # Add position info - fix the array indexing issue
                                idx = np.where(ids == marker_id)[0]
                                if len(idx) > 0:
                                    marker_idx = idx[0]
                                    corner = corners[marker_idx][0][0]
                                    text_pos = (int(corner[0]), int(corner[1] - 10))
                                    
                                    # Original pose info
                                    text1 = f"ID:{marker_id} Cam: x:{float(tvec[0][0]):.1f} y:{float(tvec[1][0]):.1f} z:{float(tvec[2][0]):.1f}"
                                    cv2.putText(display_frame, text1, text_pos,
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                    
                                    # Z-up pose info (slightly below)
                                    text2 = f"Z-up: x:{float(tvec_z_up[0][0]):.1f} y:{float(tvec_z_up[1][0]):.1f} z:{float(tvec_z_up[2][0]):.1f}"
                                    cv2.putText(display_frame, text2, (text_pos[0], text_pos[1] + 20),
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)
                                    
                            except Exception as e:
                                print(f"Error drawing axes for marker {marker_id}: {e}")
                    
                    # Print marker information (limit frequency to reduce console spam)
                    if frame_count % 10 == 0:
                        print(f"Detected {len(ids)} markers:")
                        for marker_id, (rvec, tvec) in marker_poses.items():
                            # Original camera frame pose
                            print(f"  Marker {marker_id}: Camera frame (mm) = [{float(tvec[0][0]):.1f}, {float(tvec[1][0]):.1f}, {float(tvec[2][0]):.1f}]")
                            
                            # Z-up frame pose
                            rvec_z_up, tvec_z_up = convert_to_z_up_orientation(rvec, tvec)
                            print(f"  Marker {marker_id}: Z-up frame (mm) = [{float(tvec_z_up[0][0]):.1f}, {float(tvec_z_up[1][0]):.1f}, {float(tvec_z_up[2][0]):.1f}]")
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
                                            text = f"TARGET ID:{mid} x:{float(tvec[0]):.1f} y:{float(tvec[1])::.1f} z:{float(tvec[2]):.1f}"
                                            cv2.putText(display_frame, text, text_pos,
                                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                                            
                                            # Draw pick position with offset
                                            if any(marker_to_object):
                                                # Calculate grasp position in camera frame
                                                R, _ = cv2.Rodrigues(rvec)
                                                grasp_point = tvec + R @ marker_to_object.reshape(3,1)
                                                
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
    
    # Load camera calibration parameters (camera_matrix, dist_coeffs)
    try:
        fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode('camera_matrix').mat()
        dist_coeffs = fs.getNode('dist_coeffs').mat()
        fs.release()
        print("Loaded camera calibration successfully")
    except Exception as e:
        print(f"Error loading camera calibration: {e}")
        return
    
    # Load hand-eye calibration parameters (T_cam2base)
    try:
        fs = cv2.FileStorage("hand_eye_calibration.yaml", cv2.FILE_STORAGE_READ)
        T_cam2base = fs.getNode('transform').mat()
        fs.release()
        print("Loaded hand-eye calibration successfully")
    except Exception as e:
        print(f"Error loading hand-eye calibration: {e}")
        return
    
    # Initialize camera
    camera = CameraCapture()
    print("Initializing camera...")
    if not camera.start():
        print("Failed to initialize camera")
        return
    
    # Initialize aruco detector with 5x5_50 dictionary
    dict_type = cv2.aruco.DICT_5X5_50
    marker_size = 30.0  # marker size in mm
    detector = ArUcoDetector(dict_type=dict_type, marker_size=marker_size)
    
    # Create a window for visualization
    cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Detection", 1024, 768)
    
    # Define the object points (3D points in marker coordinate system)
    # For a marker, these are the four corners in counter-clockwise order
    # with Z=0, measured from the center
    objPoints = np.array([
        [Config.marker_size/2, -Config.marker_size/2, -Config.CUBE_SIZE/2],   # Top-left corner
        [Config.marker_size/2, Config.marker_size/2, -Config.CUBE_SIZE/2],    # Top-right corner
        [-Config.marker_size/2, +Config.marker_size/2, -Config.CUBE_SIZE/2],   # Bottom-right corner
        [-Config.marker_size/2, -Config.marker_size/2, -Config.CUBE_SIZE/2],  # Bottom-left corner
    ], dtype=np.float32)
      # Initialize robot
    robot = RobotController()
    print("Connecting to robot...")
    if not robot.connect():
        print("Failed to connect to robot. Continuing in simulation mode.")
    else:
        print("Robot connected successfully")
    
    min_safe_z = 70  # Minimum safe Z height in mm
    max_x = 1000      # Maximum X coordinate
    min_x = -0     # Minimum X coordinate
    max_y = 400     # Maximum Y coordinate
    min_y = -400.0     # Minimum Y coordinate
    
    print("\nStarting detection loop. Press 'q' to exit.")
    
    while True:
        # Grab image
        frame = camera.get_frame()
        if frame is None:
            print("Failed to capture frame")
            time.sleep(0.2)
            continue
        
        # Update image in window
        cv2.imshow("Detection", frame)
        
        # Detect aruco markers in the image
        corners, ids, rejected = detector.detect_markers(frame)
        
        # Print number of detected aruco markers in the terminal
        if ids is not None:
            num_markers = len(ids)
            print(f"Detected {num_markers} ArUco markers with IDs: {ids.flatten()}")
        else:
            num_markers = 0
            print("No ArUco markers detected")
        
        if num_markers > 0:
            # Print the lowest ID in the detected markers in the terminal
            lowest_id = int(np.min(ids))
            print(f"Lowest marker ID: {lowest_id}")
            
            # Find the marker with the lowest ID
            idx = np.where(ids == lowest_id)[0][0]
            marker_corners = corners[idx]
            
            # Estimate the pose using solvePnP using camera parameters
            ret, rvec, tvec = cv2.solvePnP(
                objPoints, marker_corners, 
                camera_matrix, dist_coeffs
            )
            
            # Convert rotation vector to rotation matrix
            R, _ = cv2.Rodrigues(rvec)
            print(f"Rotation vector: {rvec.flatten()}")
            print(f"Translation vector: {tvec.flatten()}")
            print(f"Rotation matrix: {R}")


        

            
            # Draw frame axes into imagePosition out of allowed limits, skipping robot command
            display_frame = frame.copy()
            cv2.drawFrameAxes(display_frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)
            
            # Draw detected marker
            cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
            
            # Update image in window
            cv2.imshow("Detection", display_frame)
            
            # Transform to robot's coordinate frame
            T_cam2marker = np.eye(4)
            T_cam2marker[:3, :3] = R
            T_cam2marker[:3, 3] = tvec.flatten()
            
            # Transform marker to robot base frame
            T_base2marker = T_cam2base @ T_cam2marker
              # Extract position and rotation in robot base frame
            position = T_base2marker[:3, 3]
            rotation_matrix = T_base2marker[:3, :3]
            
            z_correction = np.array([0.9961947, 0, 0, 0.0871557])  # [w, x, y, z] format - ~10° Z rotation
            # Convert z_correction quaternion to rotation matrix
            qw, qx, qy, qz = z_correction
            z_correction_matrix = np.array([
                [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
            ])
            print(f"Z correction matrix: {z_correction_matrix}")
    
            a = np.radians(50)
            z_correction_matrix= np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
            rotation_matrix = z_correction_matrix @ rotation_matrix 
            # Print the original rotation matrix for verification   
            


            # Convert corrected rotation matrix back to quaternion
            corrected_r = RR.from_matrix(rotation_matrix)
            corrected_quaternion_xyzw = corrected_r.as_quat()  # (x, y, z, w)
            final_quaternion = np.array([
                corrected_quaternion_xyzw[3],  # w
                corrected_quaternion_xyzw[0],  # x
                corrected_quaternion_xyzw[1],  # y
                corrected_quaternion_xyzw[2]   # z
            ])

        
            # Clamp position values to ensure they stay within limits
            original_position = position.copy()
            position[0] = max(min_x, min(max_x, position[0]))  # Clamp X
            position[1] = max(min_y, min(max_y, position[1]))  # Clamp Y
            position[2] = max(min_safe_z, position[2])         # Clamp Z
            
            # Check if clamping was applied
            if not np.array_equal(original_position, position):
                print(f"WARNING: Position was clamped to stay within limits:")
                print(f"Original: [{original_position[0]:.2f}, {original_position[1]:.2f}, {original_position[2]:.2f}]")
                print(f"Clamped: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]")
            
            # Print position and orientation in the terminal
            print(f"Marker position in robot base frame: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]")
            
            # Convert rotation matrix to quaternion (for robot command)
            '''
            r = RR.from_matrix(rotation_matrix)
            quaternion_xyzw = r.as_quat()  # (x, y, z, w)
            quaternion_wxyz = np.array([quaternion_xyzw[3], quaternion_xyzw[0], quaternion_xyzw[1], quaternion_xyzw[2]])
            
            print(f"Quaternion in robot base frame: [{quaternion_wxyz[0]:.4f}, {quaternion_wxyz[1]:.4f}, {quaternion_wxyz[2]:.4f}, {quaternion_wxyz[3]:.4f}]")
            '''
            # Correct the error (rotation about Z-axis)
            # Apply a 10-degree correction around the Z-axis
            #z_correction = np.array([0.9961947, 0, 0, 0.0871557])  # [w, x, y, z] format - ~10° Z rotation
            #corrected_quaternion = quaternion_wxyz
            
            
            # Ensure Z-axis points downward
            # Check if the Z-axis of the gripper points downward using the corrected quaternion



            # Extract z-axis direction (third column) - this is the approach direction
            z_axis = rotation_matrix[:, 2]
            '''
            # Print corrected position and orientation
            print(f"Corrected quaternion: [{corrected_quaternion[0]:.4f}, {corrected_quaternion[1]:.4f}, {corrected_quaternion[2]:.4f}, {corrected_quaternion[3]:.4f}]")
            print(f"Gripper Z-axis direction: [{z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f}]")
              # Check if position is within limits and Z-axis is pointing downwards
            '''
            position_in_limits = True  # Position is already clamped to limits
            
            z_pointing_down = z_axis[2] < -0.9  # Z-axis should be close to -1 for pointing down
            if position_in_limits and z_pointing_down:
                print("Position is within limits and Z-axis is pointing downwards")
                print("Sending command to robot...")
                # Send to the robot
                success = robot.send_position_command_simple(
                    command_prefix="pick_",
                    position=position.reshape(3,1),
                    quaternion=final_quaternion,
                    timeout=30
                )
                
                # Wait for the response in while loop
                if success:
                    print("Command sent successfully. Waiting for robot movement...")
                    time_sent = time.time()
                    timeout = 15  # seconds
                    
                    # Wait for robot response (this would be expanded in a real implementation)
                    while time.time() - time_sent < timeout:
                        print("Waiting for robot to complete movement...")
                        time.sleep(1)
                        # In a real implementation, you would check for actual robot status here
                    
                    print("Robot movement completed or timed out")
                else:
                    print("Failed to send command to robot")
            else:
                if not position_in_limits:
                    print("Position out of allowed limits, skipping robot command")
                if not z_pointing_down:
                    print("Z-axis not pointing downwards, skipping robot command")
        
        # Check for key press to exit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Exiting detection loop")
            break
    
    # Clean up
    cv2.destroyAllWindows()
    camera.stop()
    robot.disconnect()
    print("Automated pick and place operation complete")

# Define a fallback "safe" down quaternion for when the calculated one is unsafe
# This quaternion is defined to ensure the robot gripper points exactly downward
# We're using the identity orientation (gripper Z-axis aligned with world Z-axis) and then rotating 180° around X-axis
# This ensures Z-axis of gripper points exactly opposite to world Z-axis (downward)
# Format: [w, x, y, z]
down_quaternion = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)  # 180° rotation around X axis
down_quaternion /= np.linalg.norm(down_quaternion)  # Ensure it's normalized

# Set safety parameters
MIN_SAFE_Z_HEIGHT = 75.0  # Minimum safe height above the table in mm
GRIPPER_OPEN_WIDTH = 120  # Open width in mm
GRIPPER_CLOSED_WIDTH = 30  # Closed width in mm

if __name__ == "__main__":
    main()