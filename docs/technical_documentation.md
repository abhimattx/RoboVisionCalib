# Robot Vision Calibration System: Technical Documentation

## Mathematical Foundation

### Coordinate Transformations

The system implements multiple coordinate transformations essential for robotic vision:

1. **Camera to Marker Transformation** ($T_{cam2marker}$):
   - Obtained through ArUco marker detection and pose estimation
   - Represents the transformation from camera coordinate frame to marker coordinate frame
   - Implemented using OpenCV's solvePnP function with the ArUco marker corners

2. **Base to Gripper Transformation** ($T_{base2gripper}$):
   - Obtained from robot kinematics
   - Represents the transformation from robot base frame to end-effector frame

3. **Camera to Base Transformation** ($T_{cam2base}$):
   - Obtained through hand-eye calibration
   - Critical for mapping camera observations to robot coordinates
   - The system implements multiple calibration methods: TSAI, PARK, HORAUD, ANDREFF, and DANIILIDIS

4. **Z-up Orientation Transformation**:
   - Custom transformation to convert from camera-centered coordinates to a consistent Z-up orientation
   - Implemented through a 90-degree rotation around the X-axis

### Camera Calibration

The camera calibration implements the standard pinhole camera model with distortion correction:

$$
\begin{pmatrix} x' \\ y' \\ 1 \end{pmatrix} = 
\begin{pmatrix} 
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{pmatrix}
\begin{pmatrix} X_c/Z_c \\ Y_c/Z_c \\ 1 \end{pmatrix}
$$

Where:
- $(x', y')$ are the distorted pixel coordinates
- $(X_c, Y_c, Z_c)$ are the 3D coordinates in the camera frame
- $(f_x, f_y)$ are the focal lengths in pixels
- $(c_x, c_y)$ are the principal point coordinates

The system supports three calibration patterns:
- Checkerboard
- Asymmetric circle grid
- ChArUco board

### Hand-Eye Calibration

The hand-eye calibration solves the AX=XB problem, where:
- A is the transformation from robot base to end-effector (gripper)
- B is the transformation from calibration target to camera
- X is the unknown transformation from camera to robot base

The system implements five different solution methods:
1. Tsai's method
2. Park's method
3. Horaud's method
4. Andreff's method
5. Daniilidis's method

### Quaternion Operations

The system uses quaternions for representing rotations, with operations including:
- Quaternion multiplication for combining rotations
- Conversion between quaternions and rotation matrices
- Normalization to ensure valid rotation representations

## System Architecture

### Module Relationships

```
main.py
  │
  ├── camera/
  │     ├── camera.py (Camera interface)
  │     └── detection.py (ArUco detection)
  │
  ├── calibration/
  │     ├── calibrator.py (Camera calibration)
  │     └── handeye.py (Hand-eye calibration)
  │
  ├── robot/
  │     └── controller.py (Robot control interface)
  │
  ├── tasks/
  │     └── pick_place.py (Task execution)
  │
  ├── config/
  │     └── settings.py (Configuration parameters)
  │
  └── utils/
        ├── loader.py (Data loading utilities)
        └── transforms.py (Coordinate transformations)
```

### Data Flow

1. Camera calibration produces intrinsic parameters
2. Hand-eye calibration combines robot and camera data to produce extrinsic transformation
3. ArUco detection identifies markers in camera frame
4. Coordinate transformations map camera observations to robot base frame
5. Robot controller executes movements based on transformed coordinates

## Implementation Details

### Camera Calibration Process

The camera calibration implementation follows these steps:
1. Capture multiple images of a calibration pattern from different angles
2. Detect pattern features (corners, circles) in each image
3. Compute intrinsic camera parameters and distortion coefficients
4. Save calibration results to YAML file for later use

### Hand-Eye Calibration Process

The hand-eye calibration follows these steps:
1. Move robot to multiple poses and capture calibration pattern images
2. Detect pattern and estimate its pose relative to camera
3. Record robot end-effector poses from robot controller
4. Apply chosen calibration algorithm to solve for camera-to-robot transformation
5. Save calibration results to YAML file

### Marker Detection and Pose Estimation

The ArUco marker detection pipeline:
1. Convert image to grayscale
2. Detect markers using ArUco detector
3. For each detected marker, estimate 3D pose using solvePnP
4. Transform pose from camera frame to robot base frame using hand-eye calibration
5. Apply orientation corrections to ensure proper pick approach
6. Validate pose is within safety limits before sending to robot

### Threading Model

The system implements thread-safe operations for camera access and marker detection:
1. Camera capture runs in a dedicated thread
2. Frame access is protected by locks to prevent race conditions
3. Detection operations work with thread-safe frame copies

## Experimental Validation

The system's performance has been validated through:
1. Camera calibration accuracy assessment
2. Hand-eye calibration error analysis
3. Marker detection reliability testing
4. Pick and place success rate evaluation

Detailed experimental results and error analysis would be included in a complete thesis presentation.
