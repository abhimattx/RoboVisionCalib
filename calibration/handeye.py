"""
Hand-eye calibration module for robot-camera coordination.
"""

import os
import cv2
import numpy as np
import time
from scipy.spatial.transform import Rotation
from utils.loader import load_all_configurations

def perform_hand_eye_calibration(camera_calib_file="camera_calibration.yaml", 
                               robot_config_file="configurations.yaml",  
                               output_file="hand_eye_calibration.yaml",
                               method="TSAI"):
    """
    Perform robot-camera hand-eye calibration using pre-recorded data.
    """
    print("\nHand-Eye Calibration")
    print("-" * 50)

    if not os.path.exists(camera_calib_file):
        print(f"Error: Camera calibration file not found: {camera_calib_file}")
        return False

    if not os.path.exists(robot_config_file):
        print(f"Error: Robot configuration file not found: {robot_config_file}")
        return False

    method_map = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS
    }

    calib_method = method_map.get(method, cv2.CALIB_HAND_EYE_TSAI)

    try:
        camera_fs = cv2.FileStorage(camera_calib_file, cv2.FILE_STORAGE_READ)
        rvecs = camera_fs.getNode('rotation_vectors').mat()
        tvecs = camera_fs.getNode('translation_vectors').mat() * 1000 # Convert to mm
        pattern_type = camera_fs.getNode('pattern_type').string()
        calib_time = camera_fs.getNode('calibration_time').string()
        camera_fs.release()

        print(f"Loaded camera calibration (pattern: {pattern_type}, calibrated on: {calib_time})")

        # Use local function directly instead of recursive import
        configs = load_all_configurations(robot_config_file)
        if not configs:
            print("Error: No valid robot configurations found.")
            return False

        print(f"Found {len(configs)} robot configurations")

        R_base2gripper = []
        t_base2gripper = []
        R_target2cam = []
        t_target2cam = []

        for idx, config in enumerate(configs):
            try:
                position = config['position'].flatten()
                quaternion = config['quaternion']
                quaternion = quaternion / np.linalg.norm(quaternion)

                # Convert quaternion to rotation matrix (x, y, z, w order for scipy)
                w, x, y, z = quaternion.flatten()
                rot = Rotation.from_quat([x, y, z, w])
                rot_matrix = rot.as_matrix()

                # Invert T_gripper2base to get T_base2gripper
                rot_inv = rot_matrix.T
                pos_inv = -rot_inv @ position

                R_base2gripper.append(rot_inv)
                t_base2gripper.append(pos_inv.reshape(3, 1))

                if idx < len(rvecs):
                    rvec = rvecs[idx]
                    tvec = tvecs[idx].reshape(3, 1)
                    R_cam, _ = cv2.Rodrigues(rvec)

                    R_target2cam.append(R_cam)
                    t_target2cam.append(tvec)

                    print(f"Processed pair {idx+1}: Valid")
                else:
                    print(f"Warning: Missing camera calibration data for config {idx+1}")

            except Exception as e:
                print(f"Error processing config {idx+1}: {e}")

        R_base2gripper = np.array(R_base2gripper)
        t_base2gripper = np.array(t_base2gripper)
        R_target2cam = np.array(R_target2cam)
        t_target2cam = np.array(t_target2cam)

        print(f"\nPerforming hand-eye calibration with {len(R_base2gripper)} pose pairs using {method} method...")

        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            R_base2gripper, t_base2gripper,
            R_target2cam, t_target2cam,
            method=calib_method
        )

            
        # Fix the 

        print("\nHand-eye calibration completed successfully!")
        print("\nGripper to Camera Transformation (gTc):")
        print("Rotation matrix:")
        print(R_cam2base)
        print("Translation vector (millimeters):")
        print(t_cam2base.flatten())

        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3] = t_cam2base.flatten()

        fs_out = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
        fs_out.write('transform', T_cam2base)
        fs_out.write('calibration_time', time.strftime("%Y%m%d-%H%M%S"))
        fs_out.release()

        return True

    except Exception as e:
        print(f"Hand-eye calibration failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False