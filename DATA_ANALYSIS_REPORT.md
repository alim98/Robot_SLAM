# RoboEx Data Analysis Report

## Overview
This document provides a comprehensive analysis of the robotics experimental data contained in the `/Users/ali/Documents/roboex/data` directory. The data consists of three distinct experimental datasets designed for robotics research:

1. **Sensor Modeling**: LiDAR calibration at known distances
2. **Robot Motion Modeling**: Systematic movement patterns for odometry calibration
3. **SLAM & Mapping**: Maze navigation with loop closure detection

All measurements use standard units: distances in meters, angles in radians, and times in seconds.

## Directory Structure

```
data/
├── calibration/
│   ├── lidar/
│   │   ├── distance_5.csv    (9,947 bytes)
│   │   ├── distance_10.csv   (9,882 bytes)
│   │   ├── distance_15.csv   (9,854 bytes)
│   │   ├── distance_20.csv   (9,860 bytes)
│   │   ├── distance_25.csv   (9,703 bytes)
│   │   ├── distance_30.csv   (9,685 bytes)
│   │   └── distance_37.csv   (9,694 bytes)
│   └── motion/
│       └── odom_data.csv     (2,032,451 bytes)
└── maze/
    ├── odom_data.csv         (3,780,262 bytes)
    └── range_data.csv        (121,166 bytes)
```

## Data Categories

### 1. Calibration Data

#### LiDAR Sensor Modeling (`/data/calibration/lidar/`)
- **Purpose**: LiDAR sensor calibration and error characterization
- **Experimental Setup**: Robot positioned at known distances from a wall
- **Files**: 7 CSV files for distances: 10, 15, 20, 25, 30, and 37 centimeters
- **Data Schema**:
  ```
  Column: range (float) - measured distance in meters
  ```
- **Sample Data** (distance_10.csv):
  ```
  range
  0.10687482597647706  # ~10.7cm measured vs 10cm actual
  0.12960441419157775  # ~13.0cm measured vs 10cm actual
  0.130213367947843    # ~13.0cm measured vs 10cm actual
  ```
- **Analysis**: 
  - Each file contains ~500 LiDAR measurements at fixed distances
  - Ground truth distances: 0.10, 0.15, 0.20, 0.25, 0.30, 0.37 meters
  - Used for sensor error modeling and calibration
  - Enables analysis of measurement accuracy and precision
  - Note: Missing distance_5.csv file (may be named differently)

#### Robot Motion Modeling (`/data/calibration/motion/`)
- **Purpose**: Odometry sensor calibration through controlled movement patterns
- **Experimental Setup**: 
  - Robot moves around a 1m × 1m square path
  - **First 4 rounds**: Clockwise movement
  - **Next 4 rounds**: Counterclockwise movement
  - At each corner: 90-degree rotation followed by straight movement
- **File**: `odom_data.csv` (2.03 MB)
- **Data Schema**:
  ```
  Columns: t, x, y, theta
  - t: timestamp (seconds, starting time ≠ 0)
  - x: x-coordinate (meters)
  - y: y-coordinate (meters)  
  - theta: orientation angle (radians)
  ```
- **Sample Data**:
  ```
  t,x,y,theta
  33127.521,7.162045819116943e-06,4.304240088108622e-06,0.0002668420782171
  33127.531,-0.0015284127139952443,0.0005145535743884883,-0.005582728516790552
  ```
- **Analysis**:
  - Contains ~29,925 odometry measurements over 8 complete square circuits
  - 100Hz sampling rate (10ms intervals)
  - Ground truth: Known geometric path (1m sides, 90° turns)
  - Enables systematic analysis of odometry drift and error accumulation
  - Useful for wheel encoder calibration and motion model validation

### 2. Experimental Data (`/data/maze/`)

#### Maze Navigation & SLAM (`/data/maze/odom_data.csv`)
- **Purpose**: Position estimation and mapping during maze exploration
- **Experimental Setup**:
  - Robot navigates through maze environment
  - Path designed to create multiple loop closures
  - Robot rotations also treated as loop closure events
- **File Size**: 3.78 MB (largest dataset)
- **Data Schema**:
  ```
  Columns: t, x, y, theta, loop_closure
  - t: timestamp (seconds, starting time ≠ 0)
  - x: x-coordinate (meters)
  - y: y-coordinate (meters)
  - theta: orientation angle (radians)
  - loop_closure: binary flag (1 = start/end of loop closure, 0 = normal)
  ```
- **Sample Data**:
  ```
  t,x,y,theta,loop_closure
  102.338,2.847021443225774e-05,2.380314709709491e-05,0.0010050231338915,0
  102.348,0.0018321289463328258,-0.0002419873203254922,-0.00021185552522368849,0
  ```
- **Analysis**:
  - Contains ~55,667 odometry measurements
  - 100Hz sampling rate (10ms intervals)
  - Loop closure flags enable SLAM algorithm validation
  - Multiple revisited locations for map consistency checking
  - Suitable for pose graph optimization and map correction

#### Maze Distance-to-Wall Sensor (`/data/maze/range_data.csv`)
- **Purpose**: Wall detection and distance measurement during maze navigation
- **Experimental Setup**: 
  - Synchronized with odometry data but at different sampling rate
  - Used for environment mapping and obstacle avoidance
- **File Size**: 121 KB
- **Data Schema**:
  ```
  Columns: t, range
  - t: timestamp (seconds, synchronized with odometry)
  - range: distance to nearest wall/obstacle (meters)
  ```
- **Sample Data**:
  ```
  t,range
  102.309,0.404816900519912  # ~40cm to wall
  102.509,0.40341016271555097 # ~40cm to wall
  102.609,0.39046614224371623 # ~39cm to wall
  ```
- **Analysis**:
  - Contains ~4,378 distance measurements
  - Lower sampling rate (~5-10Hz) compared to odometry
  - Range values typically 0.3-0.5 meters (maze corridor width)
  - Essential for occupancy grid mapping and localization
  - Complements odometry for full SLAM implementation

## Data Quality Assessment

### Temporal Characteristics
- **Calibration Motion**: Timestamp range ~33127s, 10ms intervals
- **Maze Experiment**: Timestamp range ~102s, 10ms intervals for odometry
- **Range Data**: Lower frequency sampling, synchronized with odometry

### Precision & Accuracy
- **Position**: Sub-millimeter precision (scientific notation: e-05, e-06)
- **Orientation**: Milliradians precision
- **Range**: Centimeter-level precision
- **Timing**: Millisecond precision

### Data Completeness
- All files contain headers
- No obvious missing values in sampled data
- Consistent data formatting across files

### Experimental Context

### Robot Configuration
- **Sensors**: Wheel encoders (odometry), LiDAR/distance sensor
- **Capabilities**: 2D navigation, SLAM with loop closure detection
- **Environment**: Controlled indoor laboratory setting
- **Movement**: Differential drive or similar wheeled platform

### Experimental Procedures

#### Phase 1: Sensor Modeling
1. **LiDAR Calibration**: Robot positioned at 7 known distances (10-37cm) from wall
2. **Static Measurements**: Multiple samples at each distance for error analysis
3. **Purpose**: Characterize sensor noise, bias, and accuracy

#### Phase 2: Motion Modeling  
1. **Controlled Path**: 1m × 1m square with precise geometry
2. **Systematic Movement**: 4 clockwise + 4 counterclockwise circuits
3. **Corner Behavior**: 90° rotations at each corner
4. **Purpose**: Odometry drift analysis and wheel encoder calibration

#### Phase 3: SLAM Validation
1. **Maze Navigation**: Complex path with multiple revisited locations
2. **Loop Closure Events**: Marked when robot returns to previous locations
3. **Multi-sensor Data**: Synchronized odometry and range measurements
4. **Purpose**: Real-world SLAM algorithm testing and validation

### Research Applications
- **Sensor Characterization**: Error modeling for both LiDAR and odometry
- **SLAM Development**: Complete dataset for algorithm development/testing
- **Localization**: Particle filter, EKF, or graph-based approaches
- **Mapping**: Occupancy grid or feature-based map construction
- **Sensor Fusion**: Combining odometry and range data optimally

## Recommendations for Analysis

### Potential Analyses

#### Sensor Characterization
1. **LiDAR Error Analysis**: Compare measured vs. true distances (10-37cm)
2. **Noise Characterization**: Statistical analysis of measurement variance
3. **Calibration Function**: Derive correction factors for sensor readings

#### Motion Analysis
4. **Odometry Drift**: Analyze cumulative error over 8 square circuits
5. **Turn Accuracy**: Evaluate 90° rotation precision at corners
6. **Straight-line Motion**: Assess linear movement accuracy over 1m segments

#### SLAM Evaluation
7. **Loop Closure Detection**: Validate marked loop closure events
8. **Map Reconstruction**: Build occupancy grid from range + odometry data
9. **Trajectory Optimization**: Use loop closures for pose graph correction
10. **Localization Accuracy**: Compare corrected vs. raw odometry estimates

### Data Processing Considerations
- **Synchronization**: Handle different sampling rates between sensors
- **Ground Truth**: Use known geometries (square path, wall distances) for validation
- **Loop Closures**: Leverage marked events for SLAM algorithm testing
- **Multi-phase Analysis**: Each dataset serves different validation purposes
- **Real-time Capability**: High-frequency data suitable for online algorithm testing

## File Statistics Summary

| Category | Files | Total Size | Records | Sampling Rate |
|----------|-------|------------|---------|---------------|
| LiDAR Calibration | 7 | ~68 KB | ~3,500 | Static |
| Motion Calibration | 1 | 2.03 MB | ~29,925 | 100 Hz |
| Maze Odometry | 1 | 3.78 MB | ~55,667 | 100 Hz |
| Maze Range | 1 | 121 KB | ~4,378 | ~5-10 Hz |
| **Total** | **10** | **~6 MB** | **~93,470** | **Variable** |

---
*Report generated on: 2025-09-13*
*Data location: `/Users/ali/Documents/roboex/data`*
