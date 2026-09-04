# ArUco Relocalization 详细设计文档

## 目录

1. [概述](#1-概述)
2. [目录结构说明](#2-目录结构说明)
3. [整体架构与数据流](#3-整体架构与数据流)
4. [核心数据结构](#4-核心数据结构)
5. [在线重定位模块](#5-在线重定位模块)
6. [离线建图 V2 模块](#6-离线建图-v2-模块)
7. [离线建图 V1 模块](#7-离线建图-v1-模块)
8. [配置参数详细说明](#8-配置参数详细说明)
9. [代码功能介绍](#9-代码功能介绍)
10. [输入输出详细说明](#10-输入输出详细说明)
11. [启动方式与使用示例](#11-启动方式与使用示例)
12. [调试与可视化](#12-调试与可视化)
13. [Python 辅助脚本](#13-python-辅助脚本)
14. [附录：结构图](#14-附录结构图)

---

## 1. 概述

### 1.1 功能定位

`aruco_relocalization` 是基于 **ArUco 视觉标记（Marker）** 的全局重定位包，属于 DeepLumin 定位系统的一部分。包含两个核心功能：

| 功能 | 节点名称 | 可执行文件 | 说明 |
|------|----------|-----------|------|
| **在线重定位** | `aruco_reloc_online` | `aruco_reloc_online_node` | 实时检测 ArUco Marker，与预建地图匹配，解算全局位姿 |
| **离线建图 V2** | `offline_mapping_v2` | `offline_mapping_v2_node` | 读取 SLAM 关键帧（CSV + 图像），遍历检测 Marker，生成地图 |
| **离线建图 V1** | `aruco_offline_mapping` | `aruco_reloc_offline_mapping_node` | 订阅 SLAM 实时轨迹和图像，在线聚合 Marker 位姿 |

### 1.2 核心依赖

| 依赖 | 用途 |
|------|------|
| `OpenCV (aruco 模块)` | ArUco Marker 检测与位姿解算 |
| `PCL` | 点云处理（VGICP 验证时） |
| `small_gicp` | 点云精匹配（VGICP 两级配准） |
| `Eigen3` | 线性代数与坐标变换 |
| `yaml-cpp` | YAML 配置文件读写 |
| `deeplumin_msgs` | 重定位结果消息定义 |

---

## 2. 目录结构说明

```
aruco_relocalization/
├── CMakeLists.txt                          # 构建配置，定义库与可执行文件
├── package.xml                             # ROS 包描述
│
├── config/
│   ├── aruco_relocalization.yaml           # 主配置文件（含所有模块参数）
│   └── marker_map.yaml                     # 生成的 Marker 地图文件
│
├── launch/
│   ├── aruco_relocalization.launch         # 在线重定位启动文件
│   ├── offline_mapping_v2.launch           # 离线建图 V2 启动文件
│   └── offline_mapping.launch              # 离线建图 V1 启动文件
│
├── include/aruco_relocalization/
│   ├── common/
│   │   └── types.hpp                       # 核心数据结构定义
│   ├── detector/
│   │   └── marker_detector.hpp             # Marker 检测器头文件
│   ├── map/
│   │   └── marker_map_loader.hpp           # Marker 地图加载器头文件
│   ├── solver/
│   │   └── relocalization_solver.hpp       # 重定位求解器头文件
│   ├── validator/
│   │   └── relocalization_validator.hpp    # 重定位校验器头文件
│   ├── verifier/
│   │   └── pointcloud_verifier.hpp         # 点云验证器头文件
│   ├── visualizer/
│   │   ├── marker_visualizer.hpp           # 通用 Marker 可视化器头文件
│   │   └── debug_visualizer.hpp            # 调试可视化器头文件
│   ├── ros/
│   │   └── param_loader.hpp                # ROS 参数加载头文件
│   ├── mapping/
│   │   └── offline_mapper.hpp              # 离线建图 V1 器头文件
│   └── offline_mapping_v2/
│       └── offline_mapping_v2.hpp          # 离线建图 V2 头文件
│
├── src/
│   ├── detector/
│   │   └── marker_detector.cpp             # Marker 检测器实现
│   ├── map/
│   │   └── marker_map_loader.cpp           # Marker 地图加载器实现
│   ├── solver/
│   │   └── relocalization_solver.cpp       # 重定位求解器实现
│   ├── validator/
│   │   └── relocalization_validator.cpp    # 重定位校验器实现
│   ├── verifier/
│   │   └── pointcloud_verifier.cpp         # 点云验证器实现
│   ├── visualizer/
│   │   ├── marker_visualizer.cpp           # 通用 Marker 可视化器实现
│   │   └── debug_visualizer.cpp            # 调试可视化器实现
│   ├── ros/
│   │   ├── param_loader.cpp                # 参数加载实现
│   │   ├── online_node.cpp                 # 在线重定位节点主文件
│   │   ├── offline_mapping_node.cpp        # 离线建图 V1 节点主文件
│   │   └── offline_mapping_v2_node.cpp     # 离线建图 V2 节点主文件
│   ├── mapping/
│   │   └── offline_mapper.cpp              # 离线建图 V1 实现
│   └── offline_mapping_v2/
│       └── offline_mapping_v2.cpp          # 离线建图 V2 实现
│
├── scripts/
│   ├── test_aruco_detection.py             # ArUco 检测测试脚本
│   └── analyze_aruco_accuracy.py           # ArUco 精度分析脚本
│
└── docs/
    ├── detailed_design.md                  # 详细设计文档（在线重定位）
    ├── offline_mapping_v2_design.md        # 离线建图 V2 设计文档
    └── aruco_relocalization_design.md      # 本文档（综合设计说明）
```

---

## 3. 整体架构与数据流

### 3.1 在线重定位数据流

```
相机图像 (/camera/li_image)
    |
    v
+-------------------+
|  图像旋转处理      |  <- image_rotation 参数（默认逆时针90度）
+--------+----------+
         |
         v
+-------------------+
|  MarkerDetector   |  <- OpenCV aruco::detectMarkers + estimatePoseSingleMarkers
|  检测 ArUco Marker |
+--------+----------+
         |
         v
+-------------------+
|  ID 匹配过滤       |  <- 与 marker_map.yaml 中的 ID 匹配
+--------+----------+
         |
         v
+-------------------+
|  Relocalization   |  <- 单 Marker: T_map_cam = T_map_marker * T_cam_marker^-1
|  Solver 求解       |  <- 多 Marker: cv::solvePnPRANSAC 联合求解
+--------+----------+
         |
         v
+-------------------+
|  Validator 校验    |  <- 重投影误差 / 几何一致性 / 运动一致性
+--------+----------+
         | (可选)
         v
+-------------------+
|  PointCloudVerifier|  <- VGICP 点云精匹配 + 最近邻 inlier 统计
|  点云验证          |
+--------+----------+
         |
         v
+-------------------+
|  发布结果          |  -> /localization/relocalization/result
|                    |  -> /aruco_reloc/pose
|                    |  -> /aruco_reloc/status
+-------------------+
```

### 3.2 离线建图 V2 数据流

```
slam_mapping_v2 输出
    |
    +-- keyframe_poses.csv  (id, timestamp, x, y, z, qx, qy, qz, qw)
    +-- images/             (id.png, id.jpg ...)
    |
    v
+-------------------+
|  读取 CSV 文件      |  -> 解析关键帧位姿
+--------+----------+
         |
         v
+-------------------+
|  遍历每个关键帧     |
|  +---------------+ |
|  | 读取图像       | |  -> images/{id}.png
|  | 图像旋转       | |  <- image_rotation
|  | Marker 检测    | |  -> ids, corners, T_cam_marker
|  | 位姿计算       | |  -> T_map_marker = T_map_base * T_base_cam * T_cam_marker
|  | 误差校验       | |  <- max_reproj_error
|  +---------------+ |
+--------+----------+
         |
         v
+-------------------+
|  聚合观测          |  <- 加权平均（权重 = 1/reproj_error）
+--------+----------+
         |
         v
+-------------------+
|  输出 marker_map   |  -> config/marker_map.yaml
|  .yaml            |
+-------------------+
```

### 3.3 模块间关系

```
aruco_relocalization 包
|
+-- 核心库 (aruco_reloc_core)
|   +-- MarkerDetector         <- 所有模块共用
|   +-- MarkerMapLoader        <- 在线重定位使用
|   +-- RelocalizationSolver   <- 在线重定位使用
|   +-- RelocalizationValidator <- 在线重定位使用
|   +-- PointCloudVerifier     <- 在线重定位使用（可选）
|   +-- MarkerVisualizer       <- 离线建图 V1 使用
|   +-- DebugVisualizer        <- 在线重定位使用（可选）
|   +-- ParamLoader            <- 所有节点共用
|   +-- OfflineMapper          <- 离线建图 V1 使用
|   +-- OfflineMappingV2       <- 离线建图 V2 使用
|
+-- 在线重定位节点 (aruco_reloc_online_node)
|   +-- 使用: Detector + Solver + Validator + Verifier + Visualizer
|
+-- 离线建图 V1 节点 (aruco_reloc_offline_mapping_node)
|   +-- 使用: Detector + OfflineMapper + Visualizer
|
+-- 离线建图 V2 节点 (offline_mapping_v2_node)
    +-- 使用: Detector + OfflineMappingV2
```

---

## 4. 核心数据结构

定义于 [types.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/common/types.hpp)

### 4.1 MarkerObservation（单 Marker 观测）

```cpp
struct MarkerObservation {
  int id = -1;                                    // marker ID
  std::array<cv::Point2f, 4> corners;             // 四个角点像素坐标
  Eigen::Isometry3d T_cam_marker = Eigen::Isometry3d::Identity();  // 相机到 marker 的变换
  double reprojection_error = 0.0;                // 角点重投影误差
};
```

### 4.2 FrameDetection（一帧检测结果）

```cpp
struct FrameDetection {
  double timestamp = 0.0;
  std::vector<MarkerObservation> markers;
};
```

### 4.3 MarkerItem（地图中的 Marker）

```cpp
struct MarkerItem {
  int id = -1;
  Eigen::Isometry3d T_map_marker = Eigen::Isometry3d::Identity();  // map 坐标系下 marker 位姿
  double size = 0.2;                              // marker 物理尺寸（米）
  int observation_count = 0;                      // 观测帧数
  double mean_reproj_error = 0.0;                 // 平均重投影误差
};
```

### 4.4 SolveResult（求解结果）

```cpp
struct SolveResult {
  bool success = false;
  Eigen::Isometry3d T_map_cam = Eigen::Isometry3d::Identity();   // 相机在 map 下的位姿
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();  // base_link 在 map 下的位姿
  int num_markers_used = 0;
  int marker_id = -1;                // 单 marker 解算时的 marker ID
  std::vector<int> marker_ids;       // 多 marker 时的 ID 列表
  std::vector<int> inlier_indices;   // RANSAC 内点索引
  double reproj_error = 1e9;         // 重投影误差
  double solve_time_ms = 0.0;
  std::string failure_reason;
};
```

### 4.5 ValidationResult（校验结果）

```cpp
struct ValidationResult {
  bool passed = false;
  double mean_reproj_error = 0.0;
  double max_marker_distance_error = 0.0;
  double translation_jump = 0.0;
  double rotation_jump = 0.0;
  int num_valid_markers = 0;
  std::string reject_reason;
};
```

### 4.6 PointCloudVerifyResult（点云验证结果）

```cpp
struct PointCloudVerifyResult {
  bool valid = false;
  bool vgicp_converged = false;
  Eigen::Isometry3d T_map_base_refined = Eigen::Isometry3d::Identity();  // VGICP 精化后的位姿
  double inlier_ratio = 0.0;
  double inlier_rmse = 1e9;
  int inlier_count = 0;
  int correspondence_count = 0;
  double delta_translation = 0.0;  // VGICP 修正的平移量（米）
  double delta_rotation_deg = 0.0; // VGICP 修正的旋转量（度）
  double elapsed_ms = 0.0;
  std::string reject_reason;
  CloudPtr source_in_map;      // 变换到 map 系的当前帧点云
  CloudPtr submap;             // 从全局地图裁剪的局部子图
  CloudPtr aligned_source;     // VGICP 匹配后的点云
};
```

### 4.7 关键帧与 Marker 聚合数据结构（离线建图 V2）

```cpp
// 关键帧数据
struct KeyframeData {
  int id;
  double timestamp;
  Eigen::Isometry3d T_map_base;  // 优化后的 base_link 在 map 下的位姿
};

// Marker 观测（用于聚合）
struct MarkerObservationV2 {
  int keyframe_id;
  Eigen::Isometry3d T_map_marker;  // marker 在 map 下的位姿
  double reprojection_error;
};
```

---

## 5. 在线重定位模块

### 5.1 MarkerDetector（Marker 检测器）

**文件**: [marker_detector.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/detector/marker_detector.hpp) / [marker_detector.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/detector/marker_detector.cpp)

#### 功能
- 使用 OpenCV `cv::aruco::detectMarkers()` 检测图像中的 ArUco Marker
- 对每个检测到的 Marker，通过 `cv::aruco::estimatePoseSingleMarkers()` 估计 `T_cam_marker`
- 计算每个 Marker 的角点重投影误差

#### 配置参数（DetectorConfig）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `dictionary` | string | `DICT_4X4_50` | ArUco 字典类型 |
| `marker_size` | double | 0.2 | Marker 物理边长（米） |
| `corner_subpix` | bool | true | 是否启用亚像素角点精化 |
| `corner_refinement_win_size` | double | 5 | 角点精化窗口大小 |

#### 输入输出

| 方向 | 内容 | 说明 |
|------|------|------|
| **输入** | `cv::Mat image` | BGR 格式图像 |
| **输入** | `double timestamp` | 图像时间戳 |
| **输出** | `FrameDetection` | 检测到的所有 Marker 列表 |

#### 处理流程

```
1. 输入图像
2. cv::aruco::detectMarkers() -> 检测 Marker 角点 + ID
3. cv::aruco::estimatePoseSingleMarkers() -> 估计每个 Marker 的 rvec/tvec
4. 对每个 Marker：计算角点重投影误差
5. 组装 FrameDetection 返回
```

### 5.2 MarkerMapLoader（Marker 地图加载器）

**文件**: [marker_map_loader.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/map/marker_map_loader.hpp) / [marker_map_loader.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/map/marker_map_loader.cpp)

#### 功能
- 从 `marker_map.yaml` 文件加载预构建的 Marker 地图
- 提供 ID 查询接口（单 ID 和批量 ID）

#### 输入输出

| 方向 | 内容 | 说明 |
|------|------|------|
| **输入** | `string yaml_path` | marker_map.yaml 路径 |
| **输出** | `MarkerMap` | `unordered_map<int, MarkerItem>` 地图数据 |

### 5.3 RelocalizationSolver（重定位求解器）

**文件**: [relocalization_solver.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/solver/relocalization_solver.hpp) / [relocalization_solver.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/solver/relocalization_solver.cpp)

#### 功能
- 将检测到的 Marker 与地图匹配，过滤未在地图中注册的 ID
- 根据匹配到的 Marker 数量，选择单 Marker 求解或多 Marker 联合 PnP 求解
- 将求解结果从 `camera` 坐标系转换到 `base_link` 坐标系

#### 配置参数（SolverConfig）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_markers` | int | 1 | 最小有效 marker 数 |
| `use_multi_marker` | bool | false | 是否启用多 marker 联合 PnP 解算 |
| `use_ransac` | bool | true | 多 marker 时是否使用 RANSAC |
| `ransac_iterations` | int | 100 | RANSAC 迭代次数 |
| `ransac_reproj_error` | double | 8.0 | RANSAC 重投影误差阈值（像素） |
| `ransac_confidence` | double | 0.99 | RANSAC 置信度 |

#### 求解策略

**单 Marker 求解**（默认策略）：
- 对每个匹配到的 Marker 单独求解：`T_map_cam = T_map_marker * T_cam_marker^-1`
- 选择重投影误差最小的结果
- 速度最快，适合实时性要求高的场景

**多 Marker 联合 PnP 求解**（可选）：
- 构建 2D-3D 点对：所有 Marker 的 4 个角点
- 调用 `cv::solvePnPRANSAC()` 或 `cv::solvePnP()` 联合求解
- 精度更高，但计算量更大

#### 坐标系转换

```
T_map_base = T_map_cam * T_base_cam^-1
T_base_cam = T_base_lidar * T_lidar_camera
```

#### 输入输出

| 方向 | 内容 | 说明 |
|------|------|------|
| **输入** | `FrameDetection` | 当前帧检测结果 |
| **输入** | `MarkerMapLoader` | 预加载的 Marker 地图 |
| **输出** | `SolveResult` | 求解结果（含位姿、误差、求解时间） |

### 5.4 RelocalizationValidator（重定位校验器）

**文件**: [relocalization_validator.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/validator/relocalization_validator.hpp) / [relocalization_validator.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/validator/relocalization_validator.cpp)

#### 功能
- 对求解器输出的候选位姿进行多重校验，过滤异常结果
- 提供明确的拒绝原因（`reject_reason`），便于调试

#### 配置参数（ValidatorConfig）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_reproj_error` | double | 100.0 | 最大允许平均重投影误差（像素） |
| `max_marker_distance_error` | double | 0.1 | 最大允许 Marker 间距误差（米） |
| `max_translation_jump` | double | 2.0 | 最大允许平移跳变（米） |
| `max_rotation_jump` | double | 30.0 | 最大允许旋转跳变（度） |
| `use_motion_check` | bool | true | 是否启用运动一致性校验 |
| `min_markers` | int | 1 | 校验通过所需最小 Marker 数 |

#### 校验流程（4 级校验）

```
1. 数量阈值校验
   -> num_valid_markers >= min_markers
   -> 失败: "not_enough_markers"

2. 重投影误差校验
   -> mean_reproj_error <= max_reproj_error
   -> 失败: "reproj_error_too_large"

3. 几何一致性校验（多 Marker 时）
   -> max_marker_distance_error <= max_marker_distance_error
   -> 失败: "geom_inconsistency"

4. 运动一致性校验（启用时）
   -> translation_jump <= max_translation_jump
   -> rotation_jump <= max_rotation_jump
   -> 失败: "translation_jump" 或 "rotation_jump"

全部通过 -> passed = true
```

#### 输入输出

| 方向 | 内容 | 说明 |
|------|------|------|
| **输入** | `SolveResult` | 求解器输出的候选位姿 |
| **输入** | `FrameDetection` | 原始检测结果 |
| **输入** | `MarkerMapLoader` | Marker 地图 |
| **输入** | `Eigen::Isometry3d* last_pose` | 上一帧位姿（可选，用于运动校验） |
| **输出** | `ValidationResult` | 校验结果（含各指标和拒绝原因） |

### 5.5 PointCloudVerifier（点云验证器）

**文件**: [pointcloud_verifier.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/verifier/pointcloud_verifier.hpp) / [pointcloud_verifier.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/verifier/pointcloud_verifier.cpp)

#### 功能
- 使用 VGICP（体素化广义迭代最近点）配准算法精化 ArUco 位姿
- 通过最近邻搜索统计内点率（inlier_ratio）和内点 RMSE（inlier_rmse）
- 支持非 VGICP 模式：直接用 ArUco 位姿计算 inlier 指标

#### 处理流程

```
1. 从缓冲区获取与图像时间戳最接近的点云
2. 坐标系旋转（lidar -> base_link，-90度 绕 Z 轴）
3. 点云预处理（距离裁剪 + Z 滤波 + 体素下采样）
4. 从全局地图裁剪局部子图（以候选位姿为中心）
5. VGICP 两级配准（粗配准 -> 精匹配，可选）
6. 最近邻搜索，统计 inlier_ratio / inlier_rmse
7. 阈值判断 -> 输出验证结果
```

#### 配置参数（PointCloudVerifyConfig）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | false | 是否启用点云验证 |
| `use_vgicp` | bool | true | 是否使用 VGICP 精化 |
| `cloud_topic` | string | `/ouster/points` | 点云订阅话题 |
| `rotate_xy_y_negx` | bool | true | 是否额外 -90度 旋转（lidar->base） |
| `query_yaw_deg` | double | 0.0 | 额外绕 Z 轴旋转角度 |
| `global_map_pcd` | string | - | 全局地图 PCD 路径 |
| `crop_forward/backward/left/right/down/up` | double | 30/30/30/30/5/10 | 子图裁剪范围（米） |
| `voxel_leaf` | double | 0.1 | 体素下采样分辨率 |
| `min_range/max_range` | double | 3.0/20.0 | 点云距离裁剪范围（米） |
| `z_filter_enable/min/max` | bool/double | false/-1.0/5.0 | Z 轴滤波 |
| `max_correspondence_distance` | double | 2.0 | VGICP 最大对应距离 |
| `transformation_epsilon` | double | 0.001 | VGICP 收敛阈值 |
| `max_iterations` | int | 64 | VGICP 最大迭代次数 |
| `inlier_distance` | double | 0.5 | 最近邻内点距离阈值（米） |
| `min_inlier_ratio` | double | 0.8 | 最小内点率 |
| `max_inlier_rmse` | double | 0.4 | 最大内点 RMSE（米） |

#### 输入输出

| 方向 | 内容 | 说明 |
|------|------|------|
| **输入** | `Eigen::Isometry3d T_map_base` | ArUco 求解的初始位姿 |
| **输入** | `double image_timestamp` | 图像时间戳（用于点云同步） |
| **输出** | `PointCloudVerifyResult` | 验证结果（含精化位姿、inlier 指标） |

### 5.6 在线节点主控（ArucoRelocNode）

**文件**: [online_node.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/ros/online_node.cpp)

#### 功能
- 整合所有子模块，构建完整的数据处理流水线
- 订阅图像、点云、控制命令话题
- 发布重定位结果、位姿、状态信息
- 响应 `start`/`stop` 控制命令

#### 回调处理流程（onImage）

```
1. 检查是否启用（enabled_ atomic flag）
2. 图像转换（cv_bridge） + 图像旋转
3. MarkerDetector::detect() -> 检测结果
4. RelocalizationSolver::solve() -> 求解位姿
5. RelocalizationValidator::validate() -> 校验
6. （可选）PointCloudVerifier::verify() -> 点云验证
7. （可选）DebugVisualizer 发布调试图像/点云
8. 发布结果
```

#### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/localization/relocalization/result` | `deeplumin_msgs::RelocalizationResult` | 重定位结果 |
| `/aruco_reloc/pose` | `geometry_msgs::PoseStamped` | 当前位姿 |
| `/aruco_reloc/status` | `std_msgs::String` | 状态信息 |

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/li_image` | `sensor_msgs::Image` | 相机图像 |
| `/ouster/points` | `sensor_msgs::PointCloud2` | 激光点云（可选） |
| `/localization_v2/relocalization/command` | `std_msgs::String` | 控制命令（start/stop） |

---

## 6. 离线建图 V2 模块

### 6.1 OfflineMappingV2（离线建图 V2 核心）

**文件**: [offline_mapping_v2.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/offline_mapping_v2/offline_mapping_v2.hpp) / [offline_mapping_v2.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/offline_mapping_v2/offline_mapping_v2.cpp)

#### 功能
- 读取 slam_mapping_v2 保存的关键帧位姿（CSV）和图像（images/ 文件夹）
- 遍历每个关键帧，检测 ArUco Marker
- 使用优化后的位姿计算 Marker 在 map 坐标系下的位姿
- 聚合多帧观测，输出 `marker_map.yaml`

#### 配置参数（OfflineMappingV2Config）

| 参数 | 路径 | 类型 | 默认值 | 说明 |
|------|------|------|--------|------|
| `keyframe_poses_csv` | `offline_mapping_v2/keyframe_poses_csv` | string | - | 关键帧位姿 CSV 文件路径（必需） |
| `keyframe_images_dir` | `offline_mapping_v2/keyframe_images_dir` | string | - | 关键帧图像目录路径（必需） |
| `output_marker_map` | `offline_mapping_v2/output_marker_map` | string | `$(find aruco_relocalization)/config/marker_map.yaml` | 输出 marker map 路径 |
| `image_rotation` | `image_rotation` | int | 0 | 图像旋转（0/1/2/3） |
| `max_reproj_error` | `offline_mapping_v2/max_reproj_error` | double | 100.0 | 最大允许重投影误差（像素） |
| `min_observations` | `offline_mapping_v2/min_observations` | int | 2 | 每个 marker 至少被观测到的次数 |
| `dictionary` | `aruco/dictionary` | string | `DICT_5X5_1000` | ArUco 字典 |
| `marker_size` | `aruco/marker_size` | double | 0.2 | Marker 物理尺寸（米） |
| `corner_subpix` | `aruco/corner_subpix` | bool | true | 亚像素精化 |

#### 处理流程

```
initialize()
1. 初始化 MarkerDetector
2. loadKeyframePoses() -> 读取 CSV，解析关键帧位姿
3. 打印统计信息

process()
1. 遍历所有关键帧
2. 对每个关键帧调用 processKeyframe()
3. aggregateMarkerObservations() -> 聚合观测

processKeyframe(kf)
1. loadKeyframeImage(kf.id) -> 读取图像
2. 图像旋转处理
3. MarkerDetector::detect() -> 检测结果
4. 对每个检测到的 Marker：
   a. 检查重投影误差 <= max_reproj_error
   b. T_map_marker = T_map_base * T_base_cam * T_cam_marker
   c. 保存观测到 marker_aggregates_

aggregateMarkerObservations()
1. 对每个 Marker：
   a. 检查观测次数 >= min_observations
   b. 位置加权平均（权重 = 1/reproj_error）
   c. 旋转加权平均（四元数，处理双覆盖）
   d. 计算平均重投影误差

saveMarkerMap()
1. 构建 YAML 节点（frame_id, source, config, markers）
2. 写入文件
```

#### 坐标变换公式

```
T_map_marker = T_map_base * T_base_lidar * T_lidar_camera * T_cam_marker
```

#### 输入输出

| 方向 | 内容 | 说明 |
|------|------|------|
| **输入** | CSV 文件 | 关键帧位姿（id, timestamp, x, y, z, qx, qy, qz, qw） |
| **输入** | 图像文件 | 关键帧图像（images/{id}.png/.jpg） |
| **输出** | YAML 文件 | marker_map.yaml（含所有 Marker 的 6DoF 位姿） |

### 6.2 离线建图 V2 节点（offline_mapping_v2_node）

**文件**: [offline_mapping_v2_node.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/ros/offline_mapping_v2_node.cpp)

#### 功能
- 独立的 ROS 节点，一次性运行完成离线建图
- 从参数服务器加载配置
- 创建 OfflineMappingV2 对象，执行处理流程
- 处理完成后自动退出

#### 启动方式

```bash
roslaunch aruco_relocalization offline_mapping_v2.launch
```

---

## 7. 离线建图 V1 模块

### 7.1 OfflineMapper（离线建图 V1 核心）

**文件**: [offline_mapper.hpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/mapping/offline_mapper.hpp) / [offline_mapper.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/mapping/offline_mapper.cpp)

#### 功能
- 订阅 slam_mapping_v2 的实时关键帧轨迹（`nav_msgs::Path`）
- 订阅相机图像，实时检测 ArUco Marker
- 通过时间戳同步找到最近的关鍵帧位姿
- 聚合 Marker 观测，最后输出 `marker_map.yaml`

#### 配置参数（OfflineMappingConfig）

| 参数 | 路径 | 类型 | 默认值 | 说明 |
|------|------|------|--------|------|
| `path_topic` | `path_topic` | string | `/slam_mapping_v2/path` | 关键帧轨迹话题 |
| `image_topic` | `image_topic` | string | `/camera/image_raw` | 图像话题 |
| `camera_info_topic` | `camera_info_topic` | string | `/camera/camera_info` | 相机内参话题 |
| `output_path` | `output_path` | string | - | 输出 marker map 路径 |
| `image_rotation` | `image_rotation` | int | 3 | 图像旋转 |
| `time_sync_tolerance` | `time_sync_tolerance` | double | 0.5 | 时间同步容差（秒） |
| `min_observations` | `min_observations` | int | 5 | 最小观测次数 |
| `max_reproj_error` | `max_reproj_error` | double | 5.0 | 最大重投影误差（像素） |

#### 输入输出

| 方向 | 内容 | 说明 |
|------|------|------|
| **输入** | `nav_msgs::Path` | SLAM 关键帧轨迹 |
| **输入** | `sensor_msgs::Image` | 相机图像 |
| **输入** | `sensor_msgs::CameraInfo` | 相机内参 |
| **输出** | YAML 文件 | marker_map.yaml |

### 7.2 离线建图 V1 节点（offline_mapping_node）

**文件**: [offline_mapping_node.cpp](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/ros/offline_mapping_node.cpp)

#### 功能
- 整合 SLAM 建图和离线建图：启动 SLAM 的同时采集 Marker 数据
- 支持 SIGINT 信号处理，退出时自动保存地图
- 集成 MarkerVisualizer 用于调试可视化

#### 启动方式

```bash
roslaunch aruco_relocalization offline_mapping.launch
```

---

## 8. 配置参数详细说明

配置文件：[aruco_relocalization.yaml](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/config/aruco_relocalization.yaml)

### 8.1 共用配置（公有命名空间）

```yaml
# 图像旋转配置（0: 不旋转, 1: 顺时针90度, 2: 180度, 3: 逆时针90度）
image_rotation: 3

# 相机内参（3x3 矩阵，行优先）
camera_matrix: [960.0, 0.0, 510.0, 0.0, 960.0, 960.0, 0.0, 0.0, 1.0]
dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]

# 相机外参（T_base_camera = T_base_lidar * T_lidar_camera）
extrinsic:
  base_to_lidar:           # base_link -> lidar 变换
    translation: [0.0, 0.0, 0.0]
    rotation: [1.0, 0.0, 0.0, 0.0]    # qw, qx, qy, qz
  lidar_to_camera:         # lidar -> camera 变换
    translation: [0.0, 0.0, -0.3]      # 相机在 lidar 下方30cm
    rotation: [0.5, -0.5, 0.5, -0.5]  # qw, qx, qy, qz

# ArUco 配置
aruco:
  dictionary: "DICT_5X5_1000"   # 字典类型
  marker_size: 0.2               # Marker 物理尺寸（米）
  corner_subpix: true            # 亚像素精化
```

### 8.2 在线重定位私有配置

```yaml
# 话题
image_topic: "/camera/li_image"
result_topic: "/localization/relocalization/result"
pose_topic: "/aruco_reloc/pose"
status_topic: "/aruco_reloc/status"
command_topic: "/localization_v2/relocalization/command"

# 坐标系
frame_id: "map"
child_frame_id: "base_link"

# Marker 地图路径
marker_map_path: "$(find aruco_relocalization)/config/marker_map.yaml"

# 求解器配置
solver:
  min_markers: 1
  use_multi_marker: false
  use_ransac: true
  ransac_iterations: 100
  ransac_reproj_error: 8.0
  ransac_confidence: 0.99

# 校验器配置
validator:
  max_reproj_error: 100.0
  max_marker_distance_error: 0.1
  max_translation_jump: 2.0
  max_rotation_jump: 30.0
  use_motion_check: true
  min_markers: 1

# 点云验证配置
pointcloud_verify:
  enabled: true
  cloud_topic: "/ouster/points"
  # ...（裁剪、预处理、VGICP、内点判断等参数）

# 可视化配置
visualizer:
  enabled: true
  debug_image_topic: "/aruco_reloc/debug_image"
  # ...（点云可视化话题）
```

### 8.3 离线建图 V2 私有配置

```yaml
offline_mapping_v2:
  keyframe_poses_csv: "/path/to/slam_mapping_v2/keyframes.csv"
  keyframe_images_dir: "/path/to/slam_mapping_v2/images"
  output_marker_map: "/path/to/aruco_relocalization/config/marker_map.yaml"
  max_reproj_error: 100.0
  min_observations: 2
```

### 8.4 Marker 地图文件格式（marker_map.yaml）

```yaml
frame_id: map
source: offline_mapping_v2
generation_time: 1788423578.653360
config:
  camera_matrix: [960, 0, 510, 0, 960, 960, 0, 0, 1]
  marker_size: 0.2
  dictionary: DICT_5X5_1000
markers:
  - id: 0
    x: 4.87
    y: -14.30
    z: 0.62
    qw: 0.54
    qx: 0.41
    qy: 0.46
    qz: 0.57
    size: 0.2
    observation_count: 4
    mean_reproj_error: 77.7
    keyframe_ids: [45, 46, 47, 48]
  - id: 1
    # ...
```

---

## 9. 代码功能介绍

### 9.1 核心库（aruco_reloc_core）

| 源文件 | 头文件 | 类/模块 | 功能说明 |
|--------|--------|---------|----------|
| `src/detector/marker_detector.cpp` | `include/.../detector/marker_detector.hpp` | `MarkerDetector` | ArUco Marker 检测与位姿估计 |
| `src/map/marker_map_loader.cpp` | `include/.../map/marker_map_loader.hpp` | `MarkerMapLoader` | 加载和查询 marker_map.yaml |
| `src/solver/relocalization_solver.cpp` | `include/.../solver/relocalization_solver.hpp` | `RelocalizationSolver` | 单/多 Marker PnP 位姿求解 |
| `src/validator/relocalization_validator.cpp` | `include/.../validator/relocalization_validator.hpp` | `RelocalizationValidator` | 4 级多重校验 |
| `src/verifier/pointcloud_verifier.cpp` | `include/.../verifier/pointcloud_verifier.hpp` | `PointCloudVerifier` | 点云 VGICP 匹配验证 |
| `src/visualizer/marker_visualizer.cpp` | `include/.../visualizer/marker_visualizer.hpp` | `MarkerVisualizer` | 通用 Marker 可视化 |
| `src/visualizer/debug_visualizer.cpp` | `include/.../visualizer/debug_visualizer.hpp` | `DebugVisualizer` | 调试可视化（图像+点云） |
| `src/ros/param_loader.cpp` | `include/.../ros/param_loader.hpp` | - | ROS 参数加载函数 |
| `src/offline_mapping_v2/offline_mapping_v2.cpp` | `include/.../offline_mapping_v2/offline_mapping_v2.hpp` | `OfflineMappingV2` | 离线建图 V2 核心逻辑 |
| `src/mapping/offline_mapper.cpp` | `include/.../mapping/offline_mapper.hpp` | `OfflineMapper` | 离线建图 V1 核心逻辑 |

### 9.2 可执行节点

| 源文件 | 可执行文件 | 节点名 | 功能说明 |
|--------|-----------|--------|----------|
| `src/ros/online_node.cpp` | `aruco_reloc_online_node` | `aruco_reloc_online` | 在线重定位主节点 |
| `src/ros/offline_mapping_v2_node.cpp` | `offline_mapping_v2_node` | `offline_mapping_v2` | 离线建图 V2 节点 |
| `src/ros/offline_mapping_node.cpp` | `aruco_reloc_offline_mapping_node` | `aruco_offline_mapping` | 离线建图 V1 节点 |

### 9.3 Python 辅助脚本

| 脚本 | 功能说明 |
|------|----------|
| `scripts/test_aruco_detection.py` | ArUco 检测功能测试：批量检测图像，统计检测率、重投影误差、距离等 |
| `scripts/analyze_aruco_accuracy.py` | ArUco 精度详细分析：绘制误差-距离/视角关系图、3D 位姿分布图 |

---

## 10. 输入输出详细说明

### 10.1 在线重定位

| 类别 | 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|------|
| **订阅** | `image_topic` | `sensor_msgs::Image` | 输入 | 相机图像（BGR8） |
| **订阅** | `cloud_topic` | `sensor_msgs::PointCloud2` | 输入 | 激光雷达点云（可选） |
| **订阅** | `command_topic` | `std_msgs::String` | 输入 | 控制命令（"start"/"stop"） |
| **发布** | `result_topic` | `deeplumin_msgs::RelocalizationResult` | 输出 | 重定位结果 |
| **发布** | `pose_topic` | `geometry_msgs::PoseStamped` | 输出 | 当前位姿 |
| **发布** | `status_topic` | `std_msgs::String` | 输出 | 状态信息 |
| **发布** | `debug_image_topic` | `sensor_msgs::Image` | 输出 | 调试图像 |
| **发布** | `source_cloud_topic` | `sensor_msgs::PointCloud2` | 输出 | 变换到 map 系的源点云 |
| **发布** | `submap_topic` | `sensor_msgs::PointCloud2` | 输出 | 裁剪的局部子图 |
| **发布** | `aligned_cloud_topic` | `sensor_msgs::PointCloud2` | 输出 | VGICP 匹配后的点云 |

### 10.2 离线建图 V2

| 类别 | 名称 | 方向 | 说明 |
|------|------|------|------|
| **输入** | keyframe_poses_csv | 文件输入 | CSV 格式关键帧位姿（id, timestamp, x, y, z, qx, qy, qz, qw） |
| **输入** | keyframe_images_dir | 文件输入 | 关键帧图像目录（{id}.png 或 {id}.jpg） |
| **输出** | output_marker_map | 文件输出 | marker_map.yaml（含所有 Marker 的 6DoF 位姿） |

### 10.3 离线建图 V1

| 类别 | 名称 | 类型 | 方向 | 说明 |
|------|------|------|------|------|
| **订阅** | `path_topic` | `nav_msgs::Path` | 输入 | SLAM 关键帧轨迹 |
| **订阅** | `image_topic` | `sensor_msgs::Image` | 输入 | 相机图像 |
| **订阅** | `camera_info_topic` | `sensor_msgs::CameraInfo` | 输入 | 相机内参信息 |
| **输出** | `output_path` | 文件输出 | YAML 格式 marker map |

### 10.4 RelocalizationResult 消息详细字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `header.stamp` | time | 时间戳 |
| `header.frame_id` | string | "map" |
| `child_frame_id` | string | "base_link" |
| `pose.pose.position` | Point | 位置（x, y, z） |
| `pose.pose.orientation` | Quaternion | 姿态（w, x, y, z） |
| `pose.covariance` | float[36] | 协方差矩阵 |
| `success` | bool | 是否求解成功 |
| `accepted` | bool | 是否被定位管理器接受 |
| `method` | string | "aruco" |
| `reject_reason` | string | 拒绝原因 |
| `inlier_count` | int | 有效 marker 数 |
| `inlier_ratio` | float | 点云内点率 |
| `inlier_rmse` | float | 点云内点 RMSE |
| `fitness_score` | float | 匹配得分 |
| `elapsed_ms` | float | 求解耗时（毫秒） |
| `map_id` | string | Marker 地图路径 |

---

## 11. 启动方式与使用示例

### 11.1 离线建图 V2（推荐）

```bash
# 1. 先运行 slam_mapping_v2 完成建图，确保关键帧数据已保存

# 2. 编辑配置文件，设置关键帧路径
#    config/aruco_relocalization.yaml
#    offline_mapping_v2:
#      keyframe_poses_csv: "/path/to/keyframes.csv"
#      keyframe_images_dir: "/path/to/images"
#      output_marker_map: "$(find aruco_relocalization)/config/marker_map.yaml"

# 3. 启动离线建图 V2
roslaunch aruco_relocalization offline_mapping_v2.launch

# 4. 检查输出 config/marker_map.yaml 已生成
```

### 11.2 在线重定位

```bash
# 1. 确保 marker_map.yaml 已存在

# 2. 启动在线重定位
roslaunch aruco_relocalization aruco_relocalization.launch

# 3. 也可同时启动 SLAM 建图（可选）
roslaunch aruco_relocalization aruco_relocalization.launch start_slam:=true

# 4. 发送控制命令
rostopic pub /localization_v2/relocalization/command std_msgs/String "data: 'start'"
rostopic pub /localization_v2/relocalization/command std_msgs/String "data: 'stop'"

# 5. 查看结果
rostopic echo /aruco_reloc/pose
rostopic echo /aruco_reloc/status
rostopic echo /localization/relocalization/result
```

### 11.3 离线建图 V1（实时建图）

```bash
# 启动 SLAM 建图 + 离线建图（同时进行）
roslaunch aruco_relocalization offline_mapping.launch

# 回放 bag 文件
rosbag play your_data.bag

# 按 Ctrl+C 退出，自动保存 marker_map.yaml
```

### 11.4 Python 测试脚本

```bash
# 测试 ArUco 检测
python3 scripts/test_aruco_detection.py

# 精度分析
python3 scripts/analyze_aruco_accuracy.py
```

---

## 12. 调试与可视化

### 12.1 调试图像

当 `visualizer.enabled = true` 时，在线重定位节点会发布调试图像话题 `/aruco_reloc/debug_image`，包含：
- Marker 角点（绿色圆圈）
- Marker ID（青色文字）
- 坐标系（RGB 三轴）
- 重投影误差和位姿信息

在 RViz 中添加 `Image` 显示即可查看。

### 12.2 点云可视化

当 `pointcloud_verify.enabled = true` 时，发布以下点云话题：
- `/aruco_reloc/source_cloud`：变换到 map 系的当前帧点云
- `/aruco_reloc/submap`：从全局地图裁剪的局部子图
- `/aruco_reloc/aligned_cloud`：VGICP 匹配后的点云

### 12.3 状态监控

```bash
# 实时查看状态信息
rostopic echo /aruco_reloc/status

# 输出示例：
# success=1 marker_id=5 reproj=2.34 time_ms=3.2 inlier_ratio=0.95 inlier_rmse=0.23
```

---

## 13. Python 辅助脚本

### 13.1 test_aruco_detection.py

**文件**: [test_aruco_detection.py](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/scripts/test_aruco_detection.py)

#### 功能
- 批量测试 ArUco 检测和位姿解算
- 统计检测率、重投影误差、距离分布
- 按 Marker ID 分组统计
- 可视化检测结果（角点+坐标轴）

#### 使用方法
```bash
python3 scripts/test_aruco_detection.py
```

#### 输出
- 控制台输出：检测率、重投影误差统计、距离统计、逐 Marker 统计
- 检测结果图像：`image-*_detected.jpg`（含角点标注和坐标轴）

### 13.2 analyze_aruco_accuracy.py

**文件**: [analyze_aruco_accuracy.py](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/scripts/analyze_aruco_accuracy.py)

#### 功能
- 详细分析 ArUco 位姿解算精度
- 分析误差来源（距离、视角、Marker 图像大小）
- 绘制 4 种误差分析图
- 绘制 3D 位姿分布图

#### 输出
- `error_analysis.png`：误差分析图（4 个子图）
  - 误差 vs 距离
  - 误差 vs 视角
  - 相对误差 vs 距离
  - Marker 图像大小 vs 误差
- `marker_positions_3d.png`：相机坐标系下 Marker 3D 位姿分布

---

## 14. 附录：结构图

### 14.1 包结构图

```
+---------------------------------------------------------------+
|                    aruco_relocalization                        |
+---------------------------------------------------------------+
|                                                               |
|  +---------+  +----------+  +-----------+  +---------------+  |
|  | config/ |  | launch/  |  | include/  |  | src/          |  |
|  | *.yaml  |  | *.launch |  | *.hpp     |  | *.cpp         |  |
|  +---------+  +----------+  +-----------+  +---------------+  |
|                                                   |           |
|                              +--------------------+-------+   |
|                              |      核心库          |       |   |
|                              | (aruco_reloc_core)   |       |   |
|                              +--------------------+       |   |
|                              | detector/   -> MarkerDetector |
|                              | map/        -> MarkerMapLoader |
|                              | solver/     -> Relocalization  |
|                              | validator/  -> Relocalization  |
|                              | verifier/   -> PointCloud      |
|                              | visualizer/ -> MarkerVisualizer |
|                              |              + DebugVisualizer |
|                              | ros/        -> ParamLoader     |
|                              | mapping/    -> OfflineMapper   |
|                              | offline_    -> OfflineMappingV2 |
|                              | mapping_v2/                    |
|                              +--------------------------------+
|                                                               |
|  +--------------------------------------------------------+  |
|  |  可执行节点                                             |  |
|  +--------------------------------------------------------+  |
|  | online_node.cpp         -> aruco_reloc_online_node     |  |
|  | offline_mapping_v2_node.cpp -> offline_mapping_v2_node |  |
|  | offline_mapping_node.cpp -> aruco_reloc_offline_mapping|  |
|  +--------------------------------------------------------+  |
|                                                               |
|  +--------------------------------------------------------+  |
|  | Python 脚本                                             |  |
|  +--------------------------------------------------------+  |
|  | scripts/test_aruco_detection.py                        |  |
|  | scripts/analyze_aruco_accuracy.py                      |  |
|  +--------------------------------------------------------+  |
|                                                               |
+---------------------------------------------------------------+
```

### 14.2 在线重定位模块结构图

```
+---------------------------------------------------------------+
|                    ArucoRelocNode                              |
+---------------------------------------------------------------+
|                                                               |
|  订阅:                                                       |
|  +--------------------------------------------------------+  |
|  | /camera/li_image         -> onImage()                  |  |
|  | /ouster/points           -> onCloud()                  |  |
|  | /localization_v2/        -> onCommand()                |  |
|  |   relocalization/command                                |  |
|  +--------------------------------------------------------+  |
|                                                               |
|  处理流水线:                                                  |
|  +--------------------------------------------------------+  |
|  | 1. cv_bridge -> 图像旋转                                |  |
|  | 2. MarkerDetector::detect()                            |  |
|  | 3. RelocalizationSolver::solve()                       |  |
|  | 4. RelocalizationValidator::validate()                 |  |
|  | 5. PointCloudVerifier::verify()    (可选)              |  |
|  | 6. DebugVisualizer::publish*()    (可选)               |  |
|  +--------------------------------------------------------+  |
|                                                               |
|  发布:                                                       |
|  +--------------------------------------------------------+  |
|  | /localization/relocalization/result -> Relocalization   |  |
|  | /aruco_reloc/pose              -> PoseStamped           |  |
|  | /aruco_reloc/status            -> String                |  |
|  | /aruco_reloc/debug_image       -> Image (可选)          |  |
|  | /aruco_reloc/source_cloud      -> PointCloud2 (可选)    |  |
|  | /aruco_reloc/submap            -> PointCloud2 (可选)    |  |
|  | /aruco_reloc/aligned_cloud     -> PointCloud2 (可选)    |  |
|  +--------------------------------------------------------+  |
|                                                               |
+---------------------------------------------------------------+
```

### 14.3 离线建图 V2 模块结构图

```
+---------------------------------------------------------------+
|               offline_mapping_v2_node                         |
+---------------------------------------------------------------+
|                                                               |
|  输入文件:                                                   |
|  +--------------------------------------------------------+  |
|  | keyframe_poses.csv  (id, timestamp, x, y, z, qx, qy, qz, qw)|
|  | images/             (keyframe_id.png)                   |  |
|  +--------------------------------------------------------+  |
|                           |                                   |
|  +--------------------------------------------------------+  |
|  | OfflineMappingV2                                       |  |
|  |                                                        |  |
|  | 1. initialize()                                        |  |
|  |    +-- 初始化 MarkerDetector                           |  |
|  |    +-- loadKeyframePoses() -> 读取 CSV                  |  |
|  |                                                        |  |
|  | 2. process()                                           |  |
|  |    +-- 遍历所有关键帧                                   |  |
|  |    |   +-- processKeyframe()                           |  |
|  |    |       +-- loadKeyframeImage() -> 读取图像           |  |
|  |    |       +-- 图像旋转                                 |  |
|  |    |       +-- MarkerDetector::detect()                 |  |
|  |    |       +-- 计算 T_map_marker                       |  |
|  |    |       +-- 保存观测到 marker_aggregates_             |  |
|  |    |                                                    |  |
|  |    +-- aggregateMarkerObservations()                    |  |
|  |        +-- 加权平均 -> 最终位姿                          |  |
|  |                                                        |  |
|  | 3. saveMarkerMap()                                     |  |
|  |    +-- 写入 marker_map.yaml                             |  |
|  +--------------------------------------------------------+  |
|                           |                                   |
|  输出文件:                                                   |
|  +--------------------------------------------------------+  |
|  | marker_map.yaml                                        |  |
|  +--------------------------------------------------------+  |
|                                                               |
+---------------------------------------------------------------+
```

### 14.4 坐标变换关系图

```
map 坐标系
    |
    | T_map_base（来自 SLAM 或求解结果）
    |
base_link 坐标系
    |
    | T_base_lidar（外参配置）
    |
lidar 坐标系
    |
    | T_lidar_camera（外参配置）
    |
camera 坐标系
    |
    | T_cam_marker（ArUco 检测结果）
    |
marker 坐标系

公式：
T_map_marker = T_map_base * T_base_lidar * T_lidar_camera * T_cam_marker
T_map_cam    = T_map_marker * T_cam_marker^-1
T_map_base   = T_map_cam * (T_base_lidar * T_lidar_camera)^-1
```

### 14.5 数据流图

```
+--------------+    +--------------+    +--------------+
| 相机传感器    |--->| 图像话题      |--->|  在线重定位   |
|              |    | /camera/     |    |  节点        |
+--------------+    | li_image     |    |              |
                    +--------------+    |  检测 Marker  |
+--------------+    +--------------+    |  求解位姿     |
| 激光雷达      |--->| 点云话题      |--->|  校验+验证   |
|              |    | /ouster/     |    |              |
+--------------+    | points       |    +------+-------+
                    +--------------+           |
                                               |
+--------------+    +--------------+    +------+-------+
| SLAM 建图    |--->| 关键帧位姿    |    | 位姿结果     |
| (slam_      |    | + 图像       |    | -> localization|
|  mapping_v2)|    |              |    |   _manager   |
+--------------+    +------+-------+    +--------------+
                           |
                           |
                    +------+-------+
                    | 离线建图 V2  |
                    |              |
                    | 生成 marker  |
                    | _map.yaml    |
                    +------+-------+
                           |
                           |
                    +------+-------+
                    | marker_map   |
                    | .yaml 地图   |<--- 在线重定位读取
                    +--------------+
```