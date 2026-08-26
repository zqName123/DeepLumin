# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DeepLumin is an autonomous driving algorithm system for underground mining vehicles, built on ROS Noetic (Ubuntu 20.04). The project follows a modular layered architecture inspired by Autoware.

## Build Commands

```bash
# Build the entire workspace
cd /home/hhy/2004ros/DeepLumin
catkin_make -DCMAKE_BUILD_TYPE=Release

# Build a specific package
catkin_make --pkg <package_name>

# After modifying deeplumin_msgs, rebuild messages first
catkin_make --pkg deeplumin_msgs
catkin_make
```

## Environment Setup

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## Architecture

The system uses a layered modular architecture where all business packages depend only on `deeplumin_msgs` (the message package) and never on each other.

### Module Layers

| Layer | Directory | Purpose |
|-------|-----------|---------|
| **Sensing** | `src/sensing/` | Sensor drivers (LiDAR, camera, IMU, GNSS) |
| **Perception** | `src/perception/` | Obstacle detection and tracking |
| **Prediction** | `src/prediction/` | Trajectory prediction |
| **Localization** | `src/localization/` | SLAM, odometry, relocalization, pose fusion |
| **Planning** | `src/planning/` | Global/local path planning |
| **Control** | `src/control/` | Vehicle motion control |
| **Vehicle Interface** | `src/vehicle_interface/` | CAN bus abstraction layer |
| **System** | `src/system/` | Monitoring, logging, communication |

### Key Design Patterns

1. **Independent Message Package**: `deeplumin_msgs` contains all custom messages organized by domain (`msg/perception/`, `msg/localization/`, etc.). Business packages only depend on this package, never on each other.

2. **Unified CMake Management**: `deeplumin_cmake` manages dependency versions (PCL 1.10, OpenCV 4.2, Eigen 3.3, optional CUDA/TensorRT). All packages should use `find_package(deeplumin_cmake REQUIRED)` before other find_package calls.

3. **Factory Pattern**: Localization modules use factory registries for algorithm implementations (e.g., `slam_odometry` supports multiple SLAM backends).

4. **Coordinate Frames**: Standard ROS frames - `map`, `odom`, `base_link`, `lidar_link`, `imu_link`.

### Localization Module (Most Active)

The localization module is the most developed, containing:
- `slam_odometry` / `slam_odometry_v2`: LiDAR-inertial odometry (Faster-LIO based)
- `slam_mapping` / `slam_mapping_v2`: Backend mapping with loop closure
- `global_matcher` / `global_matcher_v2`: Global localization via descriptor matching
- `relocalization`: Scan Context + GICP based relocalization
- `dr_odometry`: Dead reckoning (ESKF fusion of IMU/wheel/GNSS)
- `pose_fusion`: Multi-source pose fusion
- `localization_manager`: State machine managing localization modes
- `simple_odom_fusion`: Lightweight odometry fusion

### Third-Party Libraries

- `fast_gicp`: Fast Generalized ICP implementation
- `small_gicp`: Lightweight GICP variant
- Faster-LIO: Embedded in `slam_odometry/thirdparty/`

## Message Usage

All custom messages are imported from `deeplumin_msgs`:
```cpp
#include <deeplumin_msgs/ObstacleArray.h>
#include <deeplumin_msgs/ControlCommand.h>
```

## Configuration

- Parameters are managed via ROS Parameter Server
- Config files are YAML format in each package's `config/` directory
- Launch files in `launch/` directories, organized by module

## Development Conventions

- C++ follows Google Style Guide
- Classes use PascalCase, functions/variables use snake_case
- Use ROS logging macros: `ROS_INFO`, `ROS_WARN`, `ROS_ERROR`
- Each package has its own `README.md` with detailed documentation
