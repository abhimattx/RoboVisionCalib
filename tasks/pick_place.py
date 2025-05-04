import cv2
import numpy as np
from camera.detection import ArUcoDetector
from config.settings import Config
import time

class PickAndPlaceTask:

    def __init__(self, robot, camera):
        self.robot = robot
        self.camera = camera
        
        try:
            fs = cv2.FileStorage("camera_calibration.yaml", cv2.FILE_STORAGE_READ)
            self.camera_matrix = fs.getNode('camera_matrix').mat()
            self.dist_coeffs = fs.getNode('dist_coeffs').mat()
            fs.release()
            print("Loaded camera calibration for pick and place")
        except Exception as e:
            print(f"Error loading camera calibration: {e}")
            self.camera_matrix = np.array([
                [1000.0, 0, 640.0],
                [0, 1000.0, 480.0],
                [0, 0, 1]
            ])
            self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        try:
            fs = cv2.FileStorage("hand_eye_calibration.yaml", cv2.FILE_STORAGE_READ)
            self.T_cam2base = fs.getNode('transform').mat()
            fs.release()
            print("Loaded hand-eye calibration for pick and place")
        except Exception as e:
            print(f"Error loading hand-eye calibration: {e}")
            raise

        self.detector = ArUcoDetector(dict_type=cv2.aruco.DICT_5X5_50, marker_size=30.0)
    

    def detect_marker(self, marker_id):
        max_attempts = 30
        for attempt in range(max_attempts):
            image = self.camera.get_frame()
            if image is None:
                print("Warning: Got empty frame, retrying...")
                continue

            corners, ids, _ = self.detector.detect_markers(image)
            if ids is not None and marker_id in ids.flatten():
                print(f"Found marker {marker_id} on attempt {attempt+1}")
                marker_poses = self.detector.estimate_poses(corners, ids, 
                                                          self.camera_matrix, 
                                                          self.dist_coeffs)
                return marker_poses.get(marker_id)
            
            print(f"Attempt {attempt+1}/{max_attempts}: Marker {marker_id} not found")
            time.sleep(0.1)
        
        print(f"Failed to detect marker {marker_id} after {max_attempts} attempts")
        return None

    def calculate_object_position(self, marker_pose, marker_to_object):
        rvec, tvec = marker_pose

        if np.linalg.norm(tvec) > 2.0:  # Check for crazy detection
            raise ValueError("Marker detection tvec too large. Possibly invalid detection.")

        R, _ = cv2.Rodrigues(rvec)
        T_cam2marker = np.eye(4)
        T_cam2marker[:3, :3] = R
        T_cam2marker[:3, 3] = tvec.flatten()

        object_in_marker = np.append(marker_to_object, 1.0)
        object_in_camera = T_cam2marker @ object_in_marker
        object_in_base = self.T_cam2base @ object_in_camera

        return object_in_base[:3]

    def run_pick_and_place(self, marker_id, place_position, marker_to_object=None):
        if marker_to_object is None:
            marker_to_object = np.array([0, 0, 0])

        print(f"Looking for marker ID {marker_id}...")

        marker_pose = self.detect_marker(marker_id)
        if marker_pose is None:
            print(f"Could not find marker {marker_id}")
            return False

        try:
            object_position = self.calculate_object_position(marker_pose, marker_to_object)
        except ValueError as ve:
            print(f"[ERROR] {ve}")
            return False

        print(f"[DEBUG] Calculated pick position: {object_position.flatten()}")
        print(f"[DEBUG] Intended place position: {place_position.flatten() if isinstance(place_position, np.ndarray) else place_position}")

        x, y, z = object_position.flatten()
        if not (325 <= x <= 743 and -40 <= y <= 160 and 80 <= z <= 95):
            print(f"[ERROR] Object position out of robot range! x:{x:.2f} y:{y:.2f} z:{z:.2f}")
            return False

        try:
            success = self.robot.pick_and_place(object_position, place_position)
            return success
        except Exception as e:
            print(f"Error during pick and place operation: {e}")
            return False
