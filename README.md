# 井工矿特种车辆无人驾驶算法系统 (DeepLumin)

## 项目简介

DeepLumin（深地之光）是第一代井工矿特种车辆无人驾驶算法系统，基于ROS Noetic构建，旨在实现井下特种车辆的自主导航与作业能力。

## 环境要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS版本**: Noetic Ninjemys
- **编译器**: GCC 9.3.0+
- **依赖库**: PCL 1.10+, OpenCV 4.2+, Eigen 3.3+

## 项目结构

```
DeepLumin/
├── src/                          # ROS源代码目录
│   ├── deeplumin_msgs/           # 消息定义包 - 统一管理所有自定义消息类型
│   ├── sensing/                  # 传感器驱动层 - 激光雷达/摄像头/IMU/GNSS驱动
│   ├── perception/               # 感知理解层 - 障碍物检测与追踪
│   ├── prediction/               # 预测层 - 障碍物轨迹预测
│   ├── localization/             # 定位层 - 激光SLAM与位姿融合
│   ├── planning/                 # 规划层 - 全局/局部路径规划
│   ├── control/                  # 控制层 - 车辆运动控制
│   ├── vehicle_interface/        # 车辆接口层 - CAN总线/执行器对接
│   ├── system/                   # 系统层 - 监控/记录/通信网关
│   ├── simulation/               # 仿真模块 - Gazebo仿真环境
│   ├── config/                   # 配置模块 - 系统配置文件
│   ├── launch/                   # 全局启动文件
│   ├── urdf/                     # URDF模型文件
│   ├── worlds/                   # Gazebo世界文件
│   └── models/                   # Gazebo模型文件
├── build/                        # 编译输出目录
├── devel/                        # 开发环境目录
├── install/                      # 安装目录
├── .catkin_workspace             # CATKIN工作空间标识文件
├── README.md                     # 项目说明文档
└── setup.bash                    # 环境配置脚本
```

## 模块架构设计

### 包与节点划分原则

本系统采用模块化分层架构，遵循以下划分原则：

| 层级 | 划分依据 | 说明 |
|------|---------|------|
| **模块层** | 功能领域 | 按无人驾驶技术栈划分：感知、定位、规划、控制等 |
| **包层** | 功能子系统 | 每个模块下按具体功能划分为多个ROS包 |
| **节点层** | 独立进程 | 每个包包含一个或多个可执行节点，独立运行、通过话题通信 |

### 包与节点关系

```
模块 (Module)
├── 包A (Package)
│   ├── 节点1 (Node) - 独立可执行文件，完成单一功能
│   ├── 节点2 (Node) - 独立可执行文件，完成单一功能
│   ├── 消息定义 (msg/)
│   ├── 服务定义 (srv/)
│   └── 配置文件 (config/)
├── 包B (Package)
│   └── 节点3 (Node)
└── ...
```

**节点设计原则**:
1. **单一职责**: 每个节点只负责一个核心功能
2. **松耦合**: 节点间通过ROS话题/服务通信，避免直接依赖
3. **可替换**: 同类功能的节点可相互替换，支持算法迭代

### Autoware 架构对照

本系统参考 [Autoware](https://autoware.ai/) 自动驾驶框架的分层思想，结合井工矿场景特点进行适配：

| Autoware 模块 | DeepLumin 对应模块 | 说明 |
|--------------|-------------------|------|
| **Sensing** | `sensing` | **补齐**：传感器驱动与数据采集 |
| **Perception** | `perception` + `prediction` | 障碍物检测 + 轨迹预测分离 |
| **Localization** | `localization` | 激光SLAM + 多传感器融合定位 |
| **Planning** | `planning` | 全局/局部路径规划 |
| **Control** | `control` | 横向/纵向运动控制 |
| **Vehicle Interface** | `vehicle_interface` | 车辆CAN总线隔离层 |
| **Prediction** | `prediction` | 障碍物轨迹预测层 |
| **System** | `system` | 监控/记录/通信网关统一 |

**参考 Autoware 的关键设计改进**：

1. **独立消息包 `deeplumin_msgs`**：与 `autoware_msgs` 同理，所有业务包只依赖消息包，互不耦合
2. **传感器驱动层 `sensing`**：参考 Autoware Sensing 模块，将传感器驱动从感知算法中分离
3. **预测层独立 `prediction`**：参考 Autoware 将感知与预测分离，便于独立迭代预测算法
4. **车辆接口层 `vehicle_interface`**：参考 Autoware `vehicle_interface`，在控制算法与车辆硬件之间增加隔离层，支持不同车型快速适配
5. **系统层 `system`**：参考 Autoware `system_monitor` + `data_logger`，将通信、监控、记录统一为系统服务层
6. **消息按功能域分目录**：`deeplumin_msgs/msg/perception/`、`localization/` 等，便于管理和扩展

---

## 核心模块说明

### 1. 传感器驱动层 (sensing)

负责激光雷达、摄像头、IMU、GNSS等传感器的驱动、数据采集与时间同步。（参考 Autoware Sensing 模块）

#### 包结构

```
sensing/
├── lidar_driver/                 # 激光雷达驱动包
│   ├── nodes/
│   │   ├── lidar_driver.cpp        # 激光雷达数据采集与点云发布
│   │   ├── pointcloud_converter.cpp # 原始数据转 PointCloud2
│   │   └── lidar_diagnostic.cpp    # 雷达健康状态监测
│   └── config/
│       └── lidar_params.yaml
├── camera_driver/                # 摄像头驱动包
│   ├── nodes/
│   │   ├── camera_driver.cpp       # 图像采集与发布
│   │   ├── image_rectifier.cpp     # 图像去畸变
│   │   └── camera_diagnostic.cpp   # 摄像头健康监测
│   └── config/
│       └── camera_params.yaml
├── imu_driver/                   # IMU驱动包
│   ├── nodes/
│   │   ├── imu_driver.cpp          # IMU数据采集与发布
│   │   └── imu_calibrator.cpp      # IMU偏置校准
│   └── config/
│       └── imu_params.yaml
├── gnss_driver/                  # GNSS驱动包
│   ├── nodes/
│   │   ├── gnss_driver.cpp         # GNSS定位数据解析与发布
│   │   └── gnss_converter.cpp      # 坐标系转换（WGS84->局部坐标）
│   └── config/
│       └── gnss_params.yaml
└── sensor_sync/                  # 传感器时间同步包
    ├── nodes/
    │   ├── message_filter.cpp      # 多传感器消息时间对齐
    │   └── sync_publisher.cpp      # 同步数据发布
    └── config/
        └── sync_params.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| lidar_driver | lidar_driver | 激光雷达数据采集 | - | /points_raw |
| camera_driver | camera_driver | 摄像头图像采集 | - | /image_raw |
| imu_driver | imu_driver | IMU惯性数据读取 | - | /imu/data |
| gnss_driver | gnss_driver | GNSS定位数据解析 | - | /gnss/fix |
| message_filter | sensor_sync | 多传感器时间对齐 | /points_raw, /image_raw, /imu/data | /sync/sensors |

---

### 2. 感知模块 (perception)

负责处理激光雷达、摄像头等传感器数据，实现井下环境感知。

#### 包结构

```
perception/
├── pointcloud_preprocessing/    # 点云预处理包
│   ├── nodes/
│   │   ├── pointcloud_filter.py    # 点云滤波节点（去噪、降采样）
│   │   ├── ground_segmentation.py  # 地面点分割节点
│   │   └── pointcloud_fusion.py    # 多雷达点云融合节点
│   └── config/
│       └── filter_params.yaml
├── lidar_obstacle_detection/     # 激光障碍物检测包
│   ├── nodes/
│   │   ├── obstacle_detector.py    # 基于聚类的障碍物检测
│   │   ├── obstacle_tracker.py     # 动态障碍物追踪
│   │   └── obstacle_classifier.py  # 障碍物分类识别
│   └── config/
│       └── detection_params.yaml
└── camera_obstacle_detection/    # 视觉障碍物检测包
    ├── nodes/
    │   ├── object_detector.py      # 目标检测节点（YOLO）
    │   ├── semantic_segmentation.py# 语义分割节点
    │   └── depth_estimation.py     # 深度估计节点
    └── config/
        └── camera_params.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| pointcloud_filter | pointcloud_preprocessing | 点云去噪与降采样 | /points_raw | /points_filtered |
| ground_segmentation | pointcloud_preprocessing | 地面点与障碍物点分离 | /points_filtered | /points_no_ground |
| obstacle_detector | lidar_obstacle_detection | 基于DBSCAN的障碍物聚类检测 | /points_no_ground | /obstacles |
| obstacle_tracker | lidar_obstacle_detection | 多帧障碍物关联追踪 | /obstacles | /tracked_obstacles |
| object_detector | camera_obstacle_detection | 图像目标检测与识别 | /image_raw | /detected_objects |

---

### 3. 预测层 (prediction)

基于感知结果预测井下动态障碍物的未来运动轨迹，为规划层提供决策依据。（参考 Autoware `prediction` 模块）

#### 包结构

```
prediction/
├── obstacle_predictor/           # 障碍物轨迹预测包
│   ├── nodes/
│   │   ├── kalman_predictor.py     # 基于卡尔曼滤波的轨迹预测
│   │   ├── maneuver_predictor.py   # 基于意图识别的行为预测
│   │   └── trajectory_generator.py # 预测轨迹生成
│   └── config/
│       └── predictor_params.yaml
└── map_based_predictor/          # 地图约束预测包
    ├── nodes/
    │   ├── lanelet_predictor.py    # 基于车道网络的轨迹预测
    │   └── intersection_predictor.py # 交叉口行为预测
    └── config/
        └── map_predictor_params.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| kalman_predictor | obstacle_predictor | 卡尔曼滤波轨迹预测 | /tracked_obstacles | /predicted_trajectories |
| maneuver_predictor | obstacle_predictor | 障碍物行为意图预测 | /tracked_obstacles | /predicted_maneuvers |
| lanelet_predictor | map_based_predictor | 地图约束轨迹预测 | /tracked_obstacles, /vector_map | /predicted_paths |

---

### 4. 定位模块 (localization)

基于激光SLAM和多传感器融合实现车辆精确定位。

#### 包结构

```
localization/
├── laser_slam/                   # 激光SLAM包
│   ├── nodes/
│   │   ├── laser_odometry.py       # 激光里程计计算
│   │   ├── loop_detection.py       # 回环检测
│   │   └── pose_graph_optimization.py # 位姿图优化
│   └── config/
│       └── slam_params.yaml
├── odom_fusion/                  # 里程计融合包
│   ├── nodes/
│   │   ├── imu_processor.py        # IMU数据处理
│   │   ├── wheel_odom_fusion.py    # 轮式里程计融合
│   │   └── ekf_filter.py           # 扩展卡尔曼滤波融合
│   └── config/
│       └── fusion_params.yaml
└── pose_publisher/               # 位姿发布包
    ├── nodes/
    │   ├── pose_publisher.py       # 位姿信息发布
    │   └── tf_broadcaster.py       # TF坐标变换广播
    └── config/
        └── pose_params.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| laser_odometry | laser_slam | 基于ICP的激光里程计 | /points_filtered | /laser_odom |
| loop_detection | laser_slam | 基于特征匹配的回环检测 | /laser_odom, /points_filtered | /loop_constraints |
| pose_graph_optimization | laser_slam | GTSAM位姿图优化 | /loop_constraints | /optimized_pose |
| ekf_filter | odom_fusion | 多传感器EKF融合 | /laser_odom, /imu/data, /wheel_odom | /odometry/filtered |
| tf_broadcaster | pose_publisher | 发布TF变换 | /optimized_pose | /tf |

---

### 5. 规划模块 (planning)

负责生成车辆行驶路径，包括全局路径规划和局部避障。

#### 包结构

```
planning/
├── global_planner/               # 全局规划包
│   ├── nodes/
│   │   ├── global_planner.py       # A*/Dijkstra路径搜索
│   │   ├── map_loader.py           # 地图加载与管理
│   │   └── path_validator.py       # 路径可行性检查
│   └── config/
│       └── global_planner_params.yaml
├── local_planner/                # 局部规划包
│   ├── nodes/
│   │   ├── dwa_planner.py          # DWA动态窗口法规划
│   │   ├── teb_planner.py          # TEb时间弹性带规划
│   │   └── obstacle_avoidance.py   # 实时避障处理
│   └── config/
│       └── local_planner_params.yaml
└── path_smoother/                # 路径平滑包
    ├── nodes/
    │   ├── spline_smoother.py      # B样条路径平滑
    │   ├── curvature_optimizer.py  # 曲率约束优化
    │   └── speed_profile_generator.py # 速度剖面生成
    └── config/
        └── smoother_params.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| global_planner | global_planner | 全局路径搜索 | /map, /amcl_pose, /goal | /global_path |
| dwa_planner | local_planner | DWA局部路径规划 | /global_path, /obstacles, /odom | /local_path |
| obstacle_avoidance | local_planner | 实时避障与路径调整 | /local_path, /tracked_obstacles | /safe_path |
| spline_smoother | path_smoother | B样条路径平滑处理 | /global_path | /smoothed_path |
| speed_profile_generator | path_smoother | 基于路径的速度规划 | /smoothed_path | /speed_profile |

---

### 6. 控制模块 (control)

将规划指令转换为车辆执行器控制信号。

#### 包结构

```
control/
├── motion_controller/            # 运动控制包
│   ├── nodes/
│   │   ├── lateral_controller.py   # 横向控制器（转向）
│   │   ├── longitudinal_controller.py # 纵向控制器（速度）
│   │   └── motion_model.py         # 车辆动力学模型
│   └── config/
│       └── controller_params.yaml
├── steering_control/             # 转向控制包
│   ├── nodes/
│   │   ├── steering_controller.py  # 转向角闭环控制
│   │   ├── steering_limiter.py     # 转向限位保护
│   │   └── steering_executor.py    # 转向执行器驱动
│   └── config/
│       └── steering_params.yaml
└── speed_control/                # 速度控制包
    ├── nodes/
    │   ├── speed_controller.py     # 速度闭环控制
    │   ├── acceleration_limiter.py # 加速度限制
    │   └── speed_executor.py       # 速度执行器驱动
    └── config/
        └── speed_params.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| lateral_controller | motion_controller | 横向控制计算（PID/LQR） | /local_path, /odom | /steering_command |
| longitudinal_controller | motion_controller | 纵向控制计算 | /speed_profile, /odom | /speed_command |
| steering_controller | steering_control | 转向角闭环控制 | /steering_command | /steering_angle |
| speed_controller | speed_control | 速度闭环控制 | /speed_command | /throttle, /brake |

---

### 7. 车辆接口层 (vehicle_interface)

负责与井下特种车辆底层 CAN 总线/执行器的通信对接，作为自动驾驶算法与车辆硬件之间的隔离层。（参考 Autoware `vehicle_interface` 模块）

#### 包结构

```
vehicle_interface/
├── vehicle_interface_node/       # 车辆接口主节点
│   ├── nodes/
│   │   ├── vehicle_interface_node.cpp  # CAN 通信与协议转换
│   │   ├── command_converter.cpp       # ROS 指令转车辆协议
│   │   └── state_converter.cpp         # 车辆状态转 ROS 消息
│   └── config/
│       └── vehicle_interface.yaml
└── vehicle_state_receiver/       # 车辆状态接收包
    ├── nodes/
    │   ├── can_receiver.cpp            # CAN 总线数据接收
    │   ├── odometry_receiver.cpp       # 里程计数据解析
    │   └── diagnostic_receiver.cpp     # 车辆诊断信息
    └── config/
        └── state_receiver.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| vehicle_interface_node | vehicle_interface | CAN 协议转换与指令下发 | /control/command | /vehicle/status |
| command_converter | vehicle_interface | ROS 控制指令转车辆协议 | /control/command | /vehicle/can_cmd |
| can_receiver | vehicle_state_receiver | CAN 总线原始数据接收 | - | /vehicle/can_raw |

---

### 8. 系统层 (system)

负责系统监控、数据记录、地面通信网关和参数管理，替代原有的 communication 和 tools 模块。（参考 Autoware `system` 模块）

#### 包结构

```
system/
├── system_monitor/               # 系统监控包
│   ├── nodes/
│   │   ├── system_monitor.cpp      # 节点存活监控与资源统计
│   │   ├── health_checker.cpp      # 模块健康状态检测
│   │   └── emergency_handler.cpp   # 紧急停车处理
│   └── config/
│       └── system_monitor.yaml
├── data_logger/                  # 数据记录包
│   ├── nodes/
│   │   ├── rosbag_recorder.cpp     # ROS bag 自动录制
│   │   ├── data_saver.cpp          # 传感器数据保存
│   │   └── log_manager.cpp         # 日志轮转与管理
│   └── config/
│       └── logger_params.yaml
├── communication_gateway/        # 通信网关包
│   ├── nodes/
│   │   ├── gateway_server.cpp      # 地面监控中心通信
│   │   ├── data_encoder.cpp        # 数据编码与压缩
│   │   └── data_decoder.cpp        # 数据解码
│   └── config/
│       └── gateway_params.yaml
└── parameter_server/             # 参数管理包
    ├── nodes/
    │   ├── parameter_server.cpp    # 全局参数服务
    │   └── parameter_loader.cpp    # 启动参数加载
    └── config/
        └── parameter_config.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| system_monitor | system_monitor | 节点存活与资源监控 | - | /system/status |
| health_checker | system_monitor | 模块健康检测 | /system/status | /system/health |
| emergency_handler | system_monitor | 紧急停车响应 | /system/health | /vehicle/emergency |
| rosbag_recorder | data_logger | ROS bag 录制 | - | - |
| gateway_server | communication_gateway | 地面通信网关 | /vehicle/status | /ground/command |
| parameter_server | parameter_server | 全局参数服务 | - | - |

---

### 9. 仿真模块 (simulation)

基于Gazebo构建井下环境与车辆仿真系统。

#### 包结构

```
simulation/
├── mine_world/                   # 井下环境包
│   ├── worlds/
│   │   ├── mine_tunnel.world       # 矿井隧道场景
│   │   ├── underground_parking.world # 井下停车场场景
│   │   └── cross_section.world     # 交叉路口场景
│   ├── models/
│   │   ├── tunnel/                 # 隧道模型
│   │   ├── rock/                   # 岩石障碍物模型
│   │   └── equipment/              # 井下设备模型
│   └── launch/
│       └── mine_environment.launch
├── vehicle_model/                # 车辆模型包
│   ├── urdf/
│   │   ├── vehicle.urdf            # 车辆URDF模型
│   │   └── vehicle.xacro           # 车辆Xacro宏定义
│   ├── config/
│   │   └── vehicle_params.yaml
│   └── launch/
│       └── vehicle_spawn.launch
└── sensor_simulator/             # 传感器仿真包
    ├── nodes/
    │   ├── lidar_simulator.py      # 激光雷达仿真
    │   ├── camera_simulator.py     # 摄像头仿真
    │   └── imu_simulator.py        # IMU仿真
    └── config/
        └── sensor_sim_params.yaml
```

#### 核心节点说明

| 节点名称 | 所属包 | 功能描述 | 订阅话题 | 发布话题 |
|---------|-------|---------|---------|---------|
| lidar_simulator | sensor_simulator | Gazebo激光雷达仿真 | - | /scan, /points_raw |
| camera_simulator | sensor_simulator | Gazebo摄像头仿真 | - | /image_raw |
| imu_simulator | sensor_simulator | Gazebo IMU仿真 | - | /imu/data |

## 消息通信架构

### 设计原则：独立消息包（解耦）

为避免业务包之间的强耦合，系统采用**独立消息包**设计：

```
deeplumin_msgs （消息定义层，无业务逻辑）
    │
    ├── sensing
    ├── perception
    ├── prediction
    ├── localization
    ├── planning
    ├── control
    ├── vehicle_interface
    ├── system
    └── simulation （业务层，只依赖消息包，互不依赖）
```

**优势**：
- 业务包之间零耦合，只依赖统一的消息包
- 消息定义集中管理，版本控制更清晰
- 新增消息无需修改多个包的 CMakeLists.txt
- 编译顺序简单：消息包最先编译，其余包并行编译

### 消息定义清单

所有自定义消息按功能域分目录存放于 `deeplumin_msgs/msg/` 下：

```
deeplumin_msgs/msg/
├── perception/
│   ├── Obstacle.msg
│   ├── ObstacleArray.msg
│   └── FilteredPointCloud.msg
├── localization/
│   ├── SlamPose.msg
│   └── FusedOdometry.msg
├── planning/
│   ├── PathState.msg
│   └── SpeedProfile.msg
├── control/
│   ├── ControlCommand.msg
│   └── ControlStatus.msg
└── system/
    ├── SystemStatus.msg
    └── VehicleStatus.msg
```

| 功能域 | 消息文件 | 说明 | 依赖的标准消息 |
|--------|---------|------|--------------|
| **perception** | `Obstacle.msg` | 单个障碍物信息 | `std_msgs`, `geometry_msgs` |
| **perception** | `ObstacleArray.msg` | 障碍物数组 | `std_msgs`, `deeplumin_msgs/Obstacle` |
| **perception** | `FilteredPointCloud.msg` | 滤波后点云 | `std_msgs`, `sensor_msgs` |
| **localization** | `SlamPose.msg` | SLAM位姿结果 | `std_msgs`, `geometry_msgs` |
| **localization** | `FusedOdometry.msg` | 融合里程计 | `std_msgs`, `geometry_msgs`, `nav_msgs` |
| **planning** | `PathState.msg` | 路径状态 | `std_msgs`, `nav_msgs` |
| **planning** | `SpeedProfile.msg` | 速度剖面 | `std_msgs` |
| **control** | `ControlCommand.msg` | 控制指令 | `std_msgs` |
| **control** | `ControlStatus.msg` | 控制状态 | `std_msgs` |
| **system** | `SystemStatus.msg` | 系统状态 | `std_msgs` |
| **system** | `VehicleStatus.msg` | 车辆状态 | `std_msgs`, `geometry_msgs` |

### 消息包配置

独立消息包 [deeplumin_msgs](file:///home/ubuntu/G1_workspace/DeepLumin/src/deeplumin_msgs) 只负责消息生成，不依赖任何业务包：

**package.xml**：

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

**CMakeLists.txt**：

```cmake
find_package(catkin REQUIRED COMPONENTS
  std_msgs geometry_msgs nav_msgs sensor_msgs message_generation
)

add_message_files(
  DIRECTORY msg/perception
  FILES Obstacle.msg ObstacleArray.msg FilteredPointCloud.msg
)

add_message_files(
  DIRECTORY msg/localization
  FILES SlamPose.msg FusedOdometry.msg
)

add_message_files(
  DIRECTORY msg/planning
  FILES PathState.msg SpeedProfile.msg
)

add_message_files(
  DIRECTORY msg/control
  FILES ControlCommand.msg ControlStatus.msg
)

add_message_files(
  DIRECTORY msg/system
  FILES SystemStatus.msg VehicleStatus.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
```

### 业务包使用消息

业务包只需依赖 `deeplumin_msgs`，无需关心消息来自哪个模块：

**package.xml**：

```xml
<build_depend>deeplumin_msgs</build_depend>
<exec_depend>deeplumin_msgs</exec_depend>
```

**CMakeLists.txt**：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs ... deeplumin_msgs
)

catkin_package(CATKIN_DEPENDS ... deeplumin_msgs)
```

### C++代码中使用消息

无论消息定义在哪个业务模块，统一从 `deeplumin_msgs` 包引入：

```cpp
// 包含头文件
#include <deeplumin_msgs/ObstacleArray.h>
#include <deeplumin_msgs/ControlCommand.h>
#include <deeplumin_msgs/SpeedProfile.h>

// 订阅回调
void obstacleCallback(const deeplumin_msgs::ObstacleArray::ConstPtr& msg)
{
    for (const auto& obs : msg->obstacles) {
        ROS_INFO("障碍物 ID=%u, 位置=(%.2f, %.2f, %.2f)",
                 obs.id, obs.position.x, obs.position.y, obs.position.z);
    }
}

// 发布消息
deeplumin_msgs::ControlCommand cmd;
cmd.steering_angle = 0.1;
cmd.speed = 2.0;
cmd.mode = deeplumin_msgs::ControlCommand::AUTO;
pub.publish(cmd);
```

### 编译注意事项

1. **编译顺序**：`deeplumin_msgs` 会自动被 catkin 优先编译，其余业务包并行编译
2. **消息头文件位置**：`devel/include/deeplumin_msgs/`
3. **IDE配置**：将 `devel/include/` 加入 include path
4. **修改消息后重建**：`catkin_make --pkg deeplumin_msgs` 只重新编译消息包，再 `catkin_make` 编译全部

## 消息体构建设计思路

### 设计原则

本系统的消息体构建遵循以下核心原则，确保消息定义既满足当前需求，又具备良好的扩展性：

#### 1. Header 优先设计

每条自定义消息必须包含 `std_msgs/Header` 字段：

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
```

**设计意图**：
- **时间同步**：通过 `header.stamp` 实现跨节点、跨传感器的时间对齐
- **坐标系追踪**：通过 `header.frame_id` 明确数据所在的坐标系（如 `lidar_link`, `base_link`）
- **序列号追踪**：通过 `header.seq` 检测消息丢失或乱序
- **ROS 生态兼容**：与 `sensor_msgs`, `nav_msgs` 等标准消息格式一致，便于使用 ROS 工具链（rviz, rosbag, tf）

#### 2. 分层粒度设计

| 粒度层级 | 消息类型 | 使用场景 | 设计理由 |
|---------|---------|---------|---------|
| **原始数据** | `sensor_msgs/PointCloud2` | 传感器直接输出 | 不重新定义，直接使用 ROS 标准消息 |
| **预处理数据** | `FilteredPointCloud` | 点云滤波后 | 增加 header 和滤波参数，保留原始格式兼容性 |
| **语义数据** | `ObstacleArray` | 障碍物检测结果 | 结构化封装，包含障碍物列表和元信息 |
| **决策数据** | `ControlCommand` | 控制指令 | 精简字段，降低延迟，保证实时性 |
| **状态数据** | `SystemStatus` | 系统健康状态 | 聚合多模块状态，便于监控和诊断 |

#### 3. 组合优于继承

采用 ROS 消息的组合方式，而非复杂的继承结构：

```
# Obstacle.msg - 基础障碍物单元
std_msgs/Header header
uint32 id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 confidence

# ObstacleArray.msg - 障碍物集合（组合 Obstacle）
std_msgs/Header header
Obstacle[] obstacles
```

**设计理由**：
- ROS 消息不支持继承，组合是唯一方式
- `Obstacle` 作为原子单元，可被 `ObstacleArray`、预测轨迹、规划避障等多个模块复用
- 数组类型 `[]` 天然支持可变长度，无需预分配

#### 4. 枚举类型规范

状态、模式等离散量使用 `uint8` + 常量定义：

```
# ControlCommand.msg
std_msgs/Header header
uint8 mode
uint8 MODE_MANUAL = 0
uint8 MODE_AUTO = 1
uint8 MODE_EMERGENCY_STOP = 2
float32 steering_angle
float32 speed
```

**设计理由**：
- `uint8` 占用 1 字节，传输高效
- 常量命名使用大写下划线（`MODE_AUTO`），与 C++ 枚举风格一致
- 新增状态只需追加常量，不破坏已有二进制兼容性

#### 5. 向后兼容字段预留

在关键消息末尾预留扩展字段：

```
# VehicleStatus.msg
std_msgs/Header header
float32 speed
float32 steering_angle
float32 battery_voltage
float32[] reserved_floats    # 预留浮点数组，用于未来扩展
string[] reserved_strings      # 预留字符串数组
```

**设计理由**：
- 预留字段不影响现有代码的解析（ROS 消息解析器忽略未知字段）
- 紧急需求时可快速使用预留字段，无需立即重构消息定义
- 正式版本迭代时，将预留字段中的常用项提升为正式字段

### 消息命名规范

| 命名规则 | 示例 | 说明 |
|---------|------|------|
| **PascalCase** | `ObstacleArray.msg` | 消息文件名使用大驼峰 |
| **名词优先** | `SpeedProfile` | 描述消息内容，而非动作 |
| **避免缩写** | `FusedOdometry` | 不使用 `FuseOdom` 等模糊缩写 |
| **语义明确** | `ControlCommand` vs `ControlStatus` | 同域消息通过后缀区分（Command/Status/Feedback） |

## 最小兼容与可扩展性设计

### 设计目标

- **最小兼容**：新模块接入时，只需改动消息包，无需修改现有业务包
- **可扩展**：新增传感器、算法、车型时，系统架构无需重构

### 实现策略

#### 1. 消息包作为唯一耦合点

```
现有系统：
  perception --> planning --> control --> vehicle_interface

新增传感器（如毫米波雷达）：
  sensing/radar_driver (新节点)
      |
      v
  perception/radar_detector (新节点)
      |
      v
  复用 ObstacleArray.msg (已有消息)
      |
      v
  planning, control (无需任何修改)
```

**关键点**：
- 新传感器只需在 `sensing/` 新增驱动节点
- 新检测算法只需在 `perception/` 新增处理节点
- 下游模块（planning、control）通过统一的消息接口 `ObstacleArray` 消费数据，完全无感知

#### 2. 车辆接口抽象层

```
控制算法层 (control)
    │ 发布 /control/command (ControlCommand)
    v
vehicle_interface (协议转换层)
    │ 车型A协议：CAN ID 0x100
    │ 车型B协议：CAN ID 0x200
    v
车辆硬件
```

**关键点**：
- 控制算法只发布 ROS 消息 `ControlCommand`
- `vehicle_interface` 负责将 ROS 消息转换为具体车型的 CAN/以太网协议
- 更换车型时，只需替换/修改 `vehicle_interface` 中的协议转换节点，控制算法零改动

#### 3. 消息版本兼容性

```
deeplumin_msgs/msg/
├── v1.0/          # 稳定版本
│   └── Obstacle.msg
└── v1.1/          # 扩展版本（新增字段）
    └── Obstacle.msg
```

**关键点**：
- ROS 消息解析器对未知字段采取**忽略策略**（deserialization 跳过未知字段）
- 旧节点读取新消息：新增字段被忽略，不影响功能
- 新节点读取旧消息：新增字段取默认值（0/空），需代码中做好默认值处理
- 需要破坏性变更时，通过消息重命名（`ObstacleV2`）实现版本隔离

#### 4. 节点替换无需重新编译

```bash
# 替换局部规划算法：无需重新编译整个系统
# 旧节点
rosrun planning dwa_planner

# 新节点（只需替换可执行文件）
rosrun planning teb_planner
# 两者订阅/发布相同的话题，planning 模块内其他节点无感知
```

**关键点**：
- 节点间通过 ROS Topic 通信，运行时松耦合
- 替换算法节点只需替换二进制文件，无需修改 CMakeLists.txt
- 支持 A/B 测试：同时运行新旧算法，通过 remapping 切换话题订阅

#### 5. 启动文件分层组织

```
launch/
├── system.launch           # 全系统启动（生产环境）
├── simulation.launch       # 仿真环境启动
├── modules/                # 模块级启动
│   ├── sensing.launch
│   ├── perception.launch
│   ├── localization.launch
│   ├── planning.launch
│   ├── control.launch
│   └── system.launch
└── algorithms/             # 算法级启动（可选替换）
    ├── dwa_local_planner.launch
    ├── teb_local_planner.launch
    └── mpc_controller.launch
```

**关键点**：
- 模块级 launch 文件定义该模块的**标准话题接口**
- 算法级 launch 文件定义具体的**算法实现节点**
- 替换算法时，只需在模块级 launch 中 `<include>` 不同的算法级 launch
- 系统级 launch 文件只包含模块级 launch，完全不感知具体算法

#### 6. 参数外部化

```yaml
# config/vehicle_params.yaml
vehicle:
  max_speed: 5.0              # m/s
  max_steering_angle: 0.52    # rad (30度)
  wheelbase: 2.8              # m
  max_acceleration: 1.5       # m/s^2
  max_deceleration: 3.0       # m/s^2

# config/mine_a_params.yaml（矿井A参数）
vehicle:
  max_speed: 3.0              # 狭窄巷道限速
  max_steering_angle: 0.35    # 更小转弯半径
```

**关键点**：
- 所有可调参数通过 ROS Parameter Server 外部化
- 不同矿井/车型只需替换参数文件，无需修改源码
- 参数文件按场景组织（`mine_a_params.yaml`, `mine_b_params.yaml`）
- 运行时通过 `roslaunch` 的 `<arg>` 切换参数文件

## 快速开始

### 1. 环境配置

```bash
# 安装ROS Noetic依赖


# 激活工作空间
source /opt/ros/noetic/setup.bash
```

### 2. 编译项目

```bash
cd /home/ubuntu/G1_workspace/DeepLumin
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 3. 运行系统

```bash
# 激活工作空间
source devel/setup.bash

# 启动完整系统


# 或单独启动各模块

```

### 4. 仿真模式

```bash
# 启动仿真环境
roslaunch

# 启动仿真车辆
roslaunch
```

## ROS话题接口

| 话题名称 | 类型 | 说明 |
| --------- | ------ | ------ |
| /scan | sensor_msgs/LaserScan | 激光雷达数据 |
| /points_raw | sensor_msgs/PointCloud2 | 原始点云数据 |
| /image_raw | sensor_msgs/Image | 原始图像数据 |
| /imu/data | sensor_msgs/Imu | IMU数据 |
| /gnss/fix | sensor_msgs/NavSatFix | GNSS定位数据 |
| /obstacles | deeplumin_msgs/ObstacleArray | 障碍物检测结果 |
| /tracked_obstacles | deeplumin_msgs/ObstacleArray | 追踪后障碍物 |
| /predicted_trajectories | nav_msgs/Path[] | 预测轨迹（数组） |
| /odom | nav_msgs/Odometry | 里程计数据 |
| /amcl_pose | geometry_msgs/PoseWithCovarianceStamped | 定位结果 |
| /global_path | nav_msgs/Path | 全局路径 |
| /local_path | nav_msgs/Path | 局部路径 |
| /speed_profile | deeplumin_msgs/SpeedProfile | 速度剖面 |
| /control/command | deeplumin_msgs/ControlCommand | 控制指令 |
| /vehicle/status | deeplumin_msgs/VehicleStatus | 车辆状态 |
| /system/status | deeplumin_msgs/SystemStatus | 系统状态 |

## 开发规范

1. **代码风格**: 遵循Google C++ Style Guide
2. **命名规范**: 类名使用驼峰命名，函数和变量使用下划线命名
3. **日志规范**: 使用ROS标准日志宏(ROS_INFO, ROS_WARN, ROS_ERROR)
4. **配置管理**: 所有参数通过ROS Parameter Server管理
5. **文档规范**: 每个模块提供详细的README说明
