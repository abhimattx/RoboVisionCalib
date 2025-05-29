# Robot Calibration System: API Reference

## Main Module (`main.py`)

### Key Functions

#### `create_directories()`
Creates necessary directories for storing calibration and detection images.

#### `quaternion_multiply(quaternion1, quaternion0)`
Multiplies two quaternions in [w, x, y, z] format to combine rotations.

**Parameters:**
- `quaternion1`: First quaternion [w, x, y, z]
- `quaternion0`: Second quaternion [w, x, y, z]

**Returns:** Combined quaternion as numpy array

#### `verify_top_approach(quaternion)`
Verifies that a quaternion will result in a top-down approach orientation.

**Parameters:**
- `quaternion`: Quaternion in [w, x, y, z] format

**Returns:** Boolean indicating if approach is top-down

#### `convert_to_z_up_orientation(rvec, tvec)`
Converts ArUco marker pose to Z-up orientation.

**Parameters:**
- `rvec`: Rotation vector in camera frame
- `tvec`: Translation vector in camera frame

**Returns:** Tuple of (rvec_z_up, tvec_z_up)

#### `load_camera_calibration()`
Loads camera calibration parameters from file.

**Returns:** Tuple of (camera_matrix, dist_coeffs)

#### `load_hand_eye_calibration()`
Loads hand-eye calibration parameters from file.

**Returns:** Transformation matrix T_cam2base

#### `aruco_detection_test()`
Runs ArUco marker detection test with enhanced visualization.

#### `run_automated_pick_and_place()`
Executes fully automated pick and place operation.

#### `main()`
Main program entry point with menu interface.

## Camera Module (`camera/camera.py`)

### `CameraCapture` Class

#### `__init__(camera_id=0, width=640, height=480)`
Initializes camera capture interface.

#### `start()`
Starts camera capture.

**Returns:** Boolean success status

#### `stop()`
Stops camera capture and releases resources.

#### `get_frame()`
Captures and returns a single frame.

**Returns:** Image as numpy array

## Detection Module (`camera/detection.py`)

### `ArUcoDetector` Class

#### `__init__(dict_type=cv2.aruco.DICT_5X5_50, marker_size=30.0)`
Initializes ArUco marker detector.

#### `detect_markers(frame)`
Detects ArUco markers in the image.

**Parameters:**
- `frame`: Input image

**Returns:** Tuple of (corners, ids, rejected)

#### `draw_markers(frame, corners, ids)`
Draws detected markers on the image.

**Returns:** Image with markers drawn

#### `estimate_poses(corners, ids, camera_matrix, dist_coeffs)`
Estimates 3D poses of detected markers.

**Returns:** Dictionary mapping marker IDs to (rvec, tvec) tuples

## Calibration Module (`calibration/calibrator.py`)

### `CalibrationProcess` Class

#### `__init__(pattern_type, test_mode=False)`
Initializes calibration process.

**Parameters:**
- `pattern_type`: Type of calibration pattern
- `test_mode`: Whether to run in test mode (without robot)

#### `run_calibration()`
Runs the complete calibration process.

#### `capture_calibration_images()`
Captures images for calibration.

**Returns:** List of captured images

#### `detect_pattern(image)`
Detects calibration pattern in an image.

**Returns:** Boolean success status and corners

#### `calibrate_camera(images, corners_list)`
Performs camera calibration with captured images.

**Returns:** Tuple of (ret, camera_matrix, dist_coeffs, rvecs, tvecs)

## Hand-Eye Calibration (`calibration/handeye.py`)

#### `perform_hand_eye_calibration(camera_calib_file, robot_config_file, output_file, method)`
Performs robot-camera hand-eye calibration.

**Parameters:**
- `camera_calib_file`: Path to camera calibration file
- `robot_config_file`: Path to robot configuration file
- `output_file`: Path to output calibration file
- `method`: Calibration method ('TSAI', 'PARK', etc.)

**Returns:** Boolean success status

## Robot Control (`robot/controller.py`)

### `RobotController` Class

#### `__init__(ip=None, port=None)`
Initializes robot controller.

#### `connect()`
Establishes connection to the robot.

**Returns:** Boolean success status

#### `disconnect()`
Closes connection to the robot.

#### `send_position_command_simple(command_prefix, position, quaternion, timeout)`
Sends a position command to the robot.

**Parameters:**
- `command_prefix`: Command identifier prefix
- `position`: Target position [x, y, z]
- `quaternion`: Target orientation [w, x, y, z]
- `timeout`: Command timeout in seconds

**Returns:** Boolean success status

## Pick and Place (`tasks/pick_place.py`)

### `PickAndPlaceTask` Class

#### `__init__(camera_matrix, dist_coeffs, T_cam2base)`
Initializes pick and place task.

#### `pick_object(marker_id)`
Picks an object identified by a marker.

**Parameters:**
- `marker_id`: ID of the marker to pick

**Returns:** Boolean success status

#### `place_object(position, orientation)`
Places the currently held object.

**Parameters:**
- `position`: Target position [x, y, z]
- `orientation`: Target orientation [w, x, y, z]

**Returns:** Boolean success status

## Configuration (`config/settings.py`)

### `Config` Class
Contains global configuration parameters.

### `CalibrationConfig` Class
Contains calibration-specific parameters.

#### `validate()`
Validates configuration parameters.

**Returns:** Boolean indicating if all parameters are valid

#### `get_pattern_dimensions(pattern_type)`
Gets dimensions for a specific calibration pattern.

**Returns:** Tuple of (rows, cols)

#### `save_config_to_yaml(filename)`
Saves current configuration to a YAML file.

**Returns:** Boolean success status

#### `load_config_from_yaml(filename)`
Loads configuration from a YAML file.

**Returns:** Boolean success status

## Utility Functions (`utils/transforms.py`)

#### `rotation_matrix_to_quaternion(R)`
Converts 3x3 rotation matrix to quaternion.

**Returns:** Quaternion [w, x, y, z]

#### `quaternion_to_rotation_matrix(q)`
Converts quaternion to 3x3 rotation matrix.

**Returns:** 3x3 rotation matrix

#### `transform_point(point, transform_matrix)`
Transforms a 3D point using a 4x4 transformation matrix.

**Returns:** Transformed point

## Testing (`tests/run_tests.py`)

#### `run_all_tests()`
Runs all test cases in the tests directory.

**Returns:** Boolean indicating if all tests passed
