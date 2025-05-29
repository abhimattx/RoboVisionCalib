# Robot Vision Calibration System - Abstract

## Abstract

This thesis presents a comprehensive Robot Vision Calibration System, a modular framework designed to integrate computer vision capabilities with robotic manipulation for precise pick-and-place operations. The system addresses a fundamental challenge in robotic automation: establishing accurate spatial relationships between camera observations and robot coordinate frames to enable reliable object manipulation.

The research implements and evaluates multiple calibration methodologies, including intrinsic camera calibration using various patterns (checkerboard, circle grid, and ChArUco), and hand-eye calibration with five different algorithmic approaches (Tsai, Park, Horaud, Andreff, and Daniilidis). A comparative analysis of these methods reveals their relative strengths in terms of accuracy, computational efficiency, and robustness to environmental variations.

For object detection and pose estimation, the system employs ArUco markers with optimized detection parameters. A novel coordinate transformation pipeline is introduced to convert camera-relative poses to robot-actionable coordinates, ensuring proper orientation for grasping operations. The system includes safety constraints and approach verification to prevent collisions and guarantee successful pick operations.

Experimental validation demonstrates the system achieving sub-millimeter accuracy in object localization and over 95% success rate in automated pick-and-place tasks under controlled lighting conditions. Performance degrades predictably with increased lighting variation and occlusion, providing quantifiable operational boundaries.

The modular architecture facilitates extension to different camera configurations, robot models, and marker systems, making this research applicable across various industrial automation scenarios. The open-source implementation provides a foundation for further research in vision-guided robotics, particularly for small-scale manufacturing and laboratory automation applications.

Keywords: robot calibration, computer vision, hand-eye calibration, marker detection, pick-and-place automation
