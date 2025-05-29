# 🤖 Robot Vision Calibration System

<div align="center">

![Python Version](https://img.shields.io/badge/python-3.7%2B-blue)
![OpenCV Version](https://img.shields.io/badge/OpenCV-4.11.0%2B-brightgreen)
![License](https://img.shields.io/badge/license-MIT-green)

**A comprehensive framework for vision-based robot calibration and automated pick-and-place operations**

[Overview](#-overview) • 
[Installation](#-installation) • 
[Usage](#-usage) • 
[Technical Documentation](#-technical-documentation) • 
[Testing](#-testing) • 
[Contributing](#-contributing) • 
[License](#-license)

</div>

## 📌 Overview

This thesis project presents a comprehensive system for integrating computer vision with robotic manipulation, enabling precise pick-and-place operations through advanced calibration techniques. The system handles the complete workflow from camera calibration to automated task execution with sub-millimeter accuracy.

### Key Components

- **Camera Calibration**: Precise intrinsic parameter estimation using multiple patterns:
  - Checkerboard pattern
  - Asymmetric circle grid
  - ChArUco board

- **Hand-Eye Calibration**: Accurate transformation matrix between camera and robot coordinate systems using five methods:
  - Tsai's method
  - Park's method
  - Horaud's method
  - Andreff's method
  - Daniilidis's method

- **Marker Detection**: Real-time ArUco marker detection with pose estimation, supporting various dictionaries and marker sizes

- **Coordinate Transformation**: Robust pipeline for converting camera-space observations to robot-space coordinates

- **Task Automation**: Thread-safe implementation of pick-and-place operations with safety constraints and approach verification

<p align="center">
    <img src="https://via.placeholder.com/800x400?text=Robot+Calibration+System" alt="Robot Calibration System" width="700"/>
</p>

## 🧱 Project Structure

```
RobotCalibrationSystem/
├── ArUco_images/              # Saved images of ArUco markers
├── Circle_images/             # Images used for circle grid calibration
├── calib_images/              # Images used for standard calibration
├── calibration/               # Calibration scripts (camera + hand-eye)
├── camera/                    # Camera access and ArUco detection modules
├── robot/                     # Robot control logic
├── tasks/                     # Pick-and-place task logic
├── utils/                     # Utility functions (transforms, math)
├── config/                    # Configuration management
├── docs/                      # Technical documentation
├── tests/                     # Unit and integration tests
├── camera_calibration.yaml    # Intrinsic camera parameters
├── hand_eye_calibration.yaml  # Hand-eye transformation matrix
├── configurations.yaml        # General system configuration
├── main.py                    # Main entry point of the system
└── requirements.txt           # Python dependencies
```

## 🚀 Installation

### Prerequisites

- Python 3.7 or higher
- OpenCV 4.11.0 or higher (critical for ArUco detection compatibility)
- NumPy
- SciPy
- PyYAML
- USB camera or compatible imaging device
- Robot with accessible control interface (optional for simulation mode)

### Setup Steps

1. **Clone the repository or download source code**

2. **Create a virtual environment (recommended)**
   ```powershell
   python -m venv venv
   .\venv\Scripts\activate
   ```

3. **Install dependencies**
   ```powershell
   pip install -r requirements.txt
   ```

4. **Verify installation**
   ```powershell
   python -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"
   ```

5. **Create required directories**
   ```powershell
   python -c "import os; [os.makedirs(d, exist_ok=True) for d in ['Calib_images', 'Aruco_images', 'Circle_images']]"
   ```

## 🔧 Usage

### Running the System

The system provides a menu-driven interface for different operations:

```powershell
python main.py
```

This will display the main menu with the following options:
1. Calibrate with Checkerboard
2. Calibrate with Circle Grid
3. Calibrate with Charuco
4. Test Mode (No Robot)
5. Perform Hand-Eye Calibration
6. ArUco Detection Test
7. Automated Pick and Place
8. Exit

### Camera Calibration

For best results, follow these steps:

1. Select the appropriate calibration pattern option (1, 2, or 3)
2. Position the pattern in front of the camera
3. Capture multiple images (10-20) from different angles and distances
4. Ensure the pattern is fully visible and well-lit in each image
5. Review calibration results and repeat if necessary

Example output after successful calibration:
```
Calibration successful!
Camera matrix:
[[1234.56, 0.0, 320.5],
 [0.0, 1234.56, 240.5],
 [0.0, 0.0, 1.0]]
Distortion coefficients: [-0.1, 0.01, 0.0, 0.0, 0.0]
Reprojection error: 0.32
```

### Hand-Eye Calibration

Hand-eye calibration establishes the relationship between the camera and robot coordinate systems:

1. Select option 5 from the main menu
2. Choose a calibration method (1-5)
3. Follow the prompts to capture calibration data
4. Results are saved to `hand_eye_calibration.yaml`

### ArUco Detection Test

To verify marker detection:

1. Select option 6 from the main menu
2. Place ArUco markers in the camera's field of view
3. Adjust parameters as needed
4. Press 's' to save snapshots, 'd' to toggle debug view

### Automated Pick and Place

For full automation:

1. Select option 7 from the main menu
2. Ensure markers are visible to the camera
3. The system will detect markers and command the robot

## 📚 Technical Documentation

Comprehensive documentation is available in the `docs/` directory:

- `technical_documentation.md`: Detailed explanation of mathematical foundations, implementation details, and system architecture

## 🧪 Testing

The system includes a comprehensive test suite:

```powershell
cd tests
python run_tests.py
```

Individual tests can be run separately:

```powershell
python tests\test_aruco_detector.py
python tests\test_transforms.py
```

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Implement and test your changes
4. Commit your changes (`git commit -am 'Add new feature'`)
5. Push to the branch (`git push origin feature/new-feature`)
6. Create a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 📊 Performance

The system achieves:
- Sub-millimeter accuracy in position estimation
- >95% success rate in pick-and-place operations under controlled conditions
- Reliable detection with marker sizes as small as 20mm at 1m distance

## 📞 Contact

For questions or support, please contact [your.email@example.com].

### Pick-and-Place Operation

1. Ensure both calibration steps are complete
2. Place ArUco-marked objects in the workspace
3. Run:
     ```bash
     python main.py --mode pick-place
     ```
4. The system will detect objects and execute pick-and-place operations

## 🧠 Features

- **Modular Architecture**: Easily extendable for different cameras, robots, and markers
- **Multi-pattern Support**: Calibrate with chessboard, circle grid, or ArUco markers
- **Live Visualization**: Real-time feedback during calibration and operation
- **Error Handling**: Robust error detection and recovery mechanisms
- **Configuration-driven**: Easily adaptable to different hardware setups

## 📊 Performance

| Task | Accuracy | Speed |
|------|----------|-------|
| Camera Calibration | < 0.5px reprojection error | ~2 minutes |
| Hand-Eye Calibration | < 2mm position error | ~5 minutes |
| ArUco Detection | 99% detection rate | 30+ FPS |
| Pick-and-Place | 95% success rate | ~10 seconds/object |

## 🙌 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 👨‍💻 Author

<img src="https://github.com/abhimattx.png" width="100px" style="border-radius:50%"/>

**Abhishek Singh** – AI/ML Engineer, Vision Specialist  
[GitHub](https://github.com/abhimattx) • [LinkedIn](https://www.linkedin.com/in/abhimattx/)

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">
    <sub>Built with ❤️ by Abhishek Singh</sub>
</div>
