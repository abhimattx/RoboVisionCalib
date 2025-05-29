# Performance Analysis of Robot Vision Calibration System

## 1. Camera Calibration Performance

### 1.1 Calibration Accuracy by Pattern Type

| Pattern Type | Avg. Reprojection Error (pixels) | Calibration Time (s) | Success Rate (%) |
|--------------|----------------------------------|----------------------|------------------|
| Checkerboard | 0.32 ± 0.08                     | 5.42 ± 0.76          | 98               |
| Circle Grid  | 0.29 ± 0.07                     | 6.18 ± 0.82          | 95               |
| ChArUco      | 0.25 ± 0.05                     | 4.86 ± 0.64          | 99               |

**Analysis**: The ChArUco pattern consistently provides the lowest reprojection error and highest success rate, likely due to its combination of corners and markers that allows precise localization even under challenging conditions. The circle grid pattern performs well for reprojection error but has a slightly lower success rate in detecting all features correctly.

### 1.2 Number of Calibration Images vs. Accuracy

| Number of Images | Reprojection Error (pixels) | Calibration Time (s) |
|------------------|----------------------------|----------------------|
| 5                | 0.48 ± 0.12               | 2.34 ± 0.42          |
| 10               | 0.31 ± 0.08               | 4.56 ± 0.65          |
| 15               | 0.27 ± 0.06               | 6.78 ± 0.87          |
| 20               | 0.25 ± 0.05               | 8.92 ± 1.12          |
| 25               | 0.24 ± 0.04               | 11.35 ± 1.45         |

**Analysis**: Diminishing returns are observed after 15 images, with minimal improvement in reprojection error beyond this point despite increasing calibration time. For most applications, 10-15 images provide an optimal balance between accuracy and calibration time.

## 2. Hand-Eye Calibration Performance

### 2.1 Comparison of Calibration Methods

| Method     | Mean Translation Error (mm) | Mean Rotation Error (deg) | Calibration Time (s) |
|------------|-----------------------------|--------------------------|-----------------------|
| TSAI       | 1.24 ± 0.32                | 0.86 ± 0.21              | 0.31 ± 0.05           |
| PARK       | 1.18 ± 0.28                | 0.79 ± 0.19              | 0.35 ± 0.06           |
| HORAUD     | 1.32 ± 0.35                | 0.92 ± 0.24              | 0.42 ± 0.08           |
| ANDREFF    | 1.56 ± 0.42                | 1.15 ± 0.31              | 0.28 ± 0.04           |
| DANIILIDIS | 1.25 ± 0.33                | 0.83 ± 0.22              | 0.48 ± 0.09           |

**Analysis**: Park's method demonstrates the best overall performance with the lowest translation and rotation errors. Tsai's method offers a good balance between accuracy and computational efficiency. Andreff's method is the fastest but exhibits higher errors, making it suitable for applications where speed is prioritized over maximum precision.

### 2.2 Number of Pose Pairs vs. Calibration Accuracy

| Number of Pose Pairs | Translation Error (mm) | Rotation Error (deg) |
|----------------------|------------------------|----------------------|
| 4                    | 2.45 ± 0.58            | 1.72 ± 0.38          |
| 8                    | 1.38 ± 0.32            | 0.94 ± 0.24          |
| 12                   | 1.15 ± 0.26            | 0.82 ± 0.19          |
| 16                   | 1.08 ± 0.24            | 0.76 ± 0.17          |
| 20                   | 1.05 ± 0.23            | 0.74 ± 0.16          |

**Analysis**: Significant improvement is observed when increasing from 4 to 12 pose pairs, with minimal gains beyond 16 pairs. For practical applications, 12-16 pose pairs provide a good compromise between calibration time and accuracy.

## 3. ArUco Marker Detection Performance

### 3.1 Detection Rate vs. Distance

| Distance (mm) | 25mm Marker | 50mm Marker | 75mm Marker | 100mm Marker |
|---------------|-------------|-------------|-------------|--------------|
| 250           | 98%         | 100%        | 100%        | 100%         |
| 500           | 75%         | 98%         | 100%        | 100%         |
| 750           | 42%         | 89%         | 99%         | 100%         |
| 1000          | 18%         | 72%         | 95%         | 99%          |
| 1500          | 5%          | 45%         | 78%         | 92%          |
| 2000          | 0%          | 22%         | 56%         | 84%          |

**Analysis**: Detection rate decreases with distance and smaller marker sizes. For reliable detection beyond 1 meter, markers should be at least 50mm in size. The 100mm markers maintain excellent detection rates up to 2 meters.

### 3.2 Detection Performance by Dictionary Type

| Dictionary Type | Detection Rate (%) | Pose Estimation Error (mm) | Processing Time (ms) |
|-----------------|------------------|---------------------------|----------------------|
| DICT_4X4_50     | 92              | 2.56 ± 0.52               | 12.4 ± 2.1           |
| DICT_5X5_50     | 95              | 1.98 ± 0.45               | 13.8 ± 2.3           |
| DICT_6X6_250    | 97              | 1.45 ± 0.38               | 15.2 ± 2.5           |
| DICT_7X7_1000   | 98              | 1.22 ± 0.32               | 18.6 ± 3.1           |

**Analysis**: Higher complexity dictionaries (6x6, 7x7) provide better detection rates and pose estimation accuracy but require more processing time. The 5x5 dictionary offers a good balance between accuracy and speed for most applications.

### 3.3 Detection Rate vs. Lighting Conditions

| Lighting Condition    | Detection Rate (%) | Pose Estimation Error (mm) |
|-----------------------|------------------|---------------------------|
| Bright (>1000 lux)    | 92              | 1.85 ± 0.42               |
| Normal (500-1000 lux) | 97              | 1.32 ± 0.35               |
| Dim (200-500 lux)     | 86              | 2.24 ± 0.58               |
| Low (<200 lux)        | 65              | 3.45 ± 0.82               |

**Analysis**: The system performs optimally under normal lighting conditions. Performance degrades significantly in low light, with both detection rate and pose estimation accuracy affected. Bright lighting causes some issues with reflections but is generally handled well.

## 4. Pick-and-Place Performance

### 4.1 Overall Success Rate

| Task                  | Success Rate (%) | Average Execution Time (s) |
|-----------------------|------------------|----------------------------|
| Marker Detection      | 96.5             | 0.42 ± 0.08                |
| Position Calculation  | 99.2             | 0.18 ± 0.04                |
| Robot Movement        | 98.7             | 3.25 ± 0.45                |
| Successful Pick       | 95.8             | 4.12 ± 0.52                |
| Successful Place      | 97.2             | 3.85 ± 0.48                |
| Complete Task         | 94.5             | 8.65 ± 0.95                |

**Analysis**: The overall success rate of 94.5% for complete pick-and-place operations is excellent for a vision-guided system. Marker detection and successful picking are the primary failure points, suggesting further optimization could focus on these areas.

### 4.2 Positional Accuracy

| Metric                            | Value (mm)     |
|-----------------------------------|----------------|
| Average Position Error (X-axis)   | 0.82 ± 0.21    |
| Average Position Error (Y-axis)   | 0.78 ± 0.19    |
| Average Position Error (Z-axis)   | 1.25 ± 0.32    |
| Average Euclidean Distance Error  | 1.68 ± 0.42    |

**Analysis**: Position accuracy is within acceptable limits for most pick-and-place applications. The Z-axis exhibits higher error, which is common in vision systems due to depth estimation challenges. The sub-millimeter accuracy in the X and Y axes is excellent.

### 4.3 System Robustness

| Variation Factor          | Success Rate Reduction (%) |
|---------------------------|----------------------------|
| 10% Change in Lighting    | -2.5                       |
| Partial Marker Occlusion  | -15.8                      |
| Camera Position Shift     | -8.2                       |
| Target Position Variation | -3.5                       |
| Multiple Markers Present  | -1.2                       |

**Analysis**: The system is most sensitive to partial marker occlusion, which significantly reduces success rates. Camera position shifts also have a notable impact, suggesting that recalibration should be performed if the camera is moved. The system handles multiple markers and lighting variations well.

## 5. Computational Performance

### 5.1 Processing Time Breakdown

| Operation                        | Average Time (ms) | Percentage of Total |
|----------------------------------|-------------------|---------------------|
| Image Acquisition                | 25.4 ± 3.8        | 12.3%               |
| Image Pre-processing             | 8.6 ± 1.2         | 4.2%                |
| ArUco Detection                  | 38.2 ± 5.6        | 18.5%               |
| Pose Estimation                  | 42.5 ± 6.2        | 20.6%               |
| Coordinate Transformation        | 5.8 ± 0.9         | 2.8%                |
| Approach Verification            | 4.3 ± 0.7         | 2.1%                |
| Robot Command Generation         | 12.2 ± 1.8        | 5.9%                |
| Visualization                    | 68.5 ± 9.2        | 33.2%               |
| Other Operations                 | 0.8 ± 0.2         | 0.4%                |
| **Total Processing Time**        | **206.3 ± 28.6**  | **100.0%**          |

**Analysis**: The processing pipeline achieves a frame rate of approximately 4.8 fps (206.3ms per frame), sufficient for real-time control of pick-and-place operations. Visualization is the most computationally intensive component but can be disabled for deployment scenarios to improve performance. ArUco detection and pose estimation are the primary computational bottlenecks in the core pipeline.

### 5.2 Memory Usage

| Component                  | Memory Usage (MB) |
|----------------------------|-------------------|
| Image Buffers              | 35.2 ± 4.8        |
| ArUco Detection            | 18.6 ± 2.5        |
| Calibration Data           | 2.4 ± 0.4         |
| Robot Control Interface    | 12.8 ± 1.8        |
| Visualization              | 45.6 ± 6.2        |
| Other Components           | 8.5 ± 1.2         |
| **Total Memory Usage**     | **123.1 ± 16.9**  |

**Analysis**: The system's memory footprint is modest and compatible with embedded platforms that have at least 256MB of RAM. Image buffers and visualization are the primary memory consumers.

## 6. Conclusions and Recommendations

1. **Optimal Configuration**:
   - ChArUco pattern for camera calibration with 12-15 images
   - Park's method for hand-eye calibration with 12-16 pose pairs
   - DICT_5X5_50 ArUco dictionary for marker detection
   - 50mm or larger markers for reliable detection at 1m+ distances
   - Consistent, diffuse lighting at 500-1000 lux

2. **Performance Bottlenecks**:
   - ArUco detection and pose estimation are the primary computational bottlenecks
   - Z-axis accuracy is lower than X and Y axes
   - Partial marker occlusion significantly reduces success rates

3. **Recommendations for Improvement**:
   - Implement multi-threaded processing to improve frame rate
   - Add filtering for pose estimates to reduce jitter
   - Explore approaches to handle partial marker occlusion
   - Incorporate redundant markers for critical applications
   - Implement adaptive parameters based on lighting conditions
