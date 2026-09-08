# ArUco Relocalization 详细设计文档

## 1. 概述

### 1.1 节点名称与定位

- **Package**: `aruco_relocalization`
- **在线节点**: `aruco_reloc_online_node`（节点名：`aruco_reloc_online`）
- **离线建图节点**: `aruco_reloc_offline_mapping_node`（节点名：`aruco_offline_mapping`）
- **所属模块**: DeepLumin 定位系统 —— 全局重定位子模块
- **上游依赖**: `slam_mapping_v2`（提供关键帧轨迹，仅离线建图阶段）
- **下游消费者**: `localization_manager` → `pose_fusion`

### 1.2 核心职责

`aruco_relocalization` 是基于 **ArUco 视觉标记（Marker）** 的全局重定位节点，承担以下职责：

1. **离线建图**：在 SLAM 建图阶段，通过订阅 `slam_mapping_v2/path` 关键帧轨迹和相机图像，自动检测场景中的 ArUco Marker，并计算每个 Marker 在全局地图（`map` 坐标系）下的 6DoF 位姿，最终输出 `marker_map.yaml`。
2. **在线重定位**：在机器人运行阶段，实时检测相机视野中的 ArUco Marker，通过与预构建的 `marker_map.yaml` 匹配，解算相机（及 `base_link`）在全局地图下的位姿，并将重定位结果发送给 `localization_manager` 进行门控与融合。

### 1.3 设计目标

| 目标 | 说明 |
|------|------|
| **高精度** | 利用多 Marker 联合 PnP 求解，配合 RANSAC 和多重校验，确保位姿精度 |
| **高鲁棒性** | 支持单 Marker 退化场景；通过运动一致性校验过滤异常跳变 |
| **低延迟** | 图像回调直通处理，无额外线程，单次求解耗时 < 10ms |
| **易部署** | 提供一键离线建图 launch，自动生成 marker 地图文件 |

---

## 2. 整体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                    aruco_relocalization 架构                         │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐            │
│  │   Detector   │ → │    Solver    │ → │   Validator  │            │
│  │(MarkerDetector)│  │(Relocalization│  │(Relocalization│           │
│  │              │   │    Solver)   │   │  Validator)  │            │
│  └──────────────┘   └──────────────┘   └──────────────┘            │
│         ↑                                    ↓                      │
│  ┌──────────────┐                    ┌──────────────┐              │
│  │  MarkerMap   │                    │  Result Pub  │              │
│  │   Loader     │                    │  Pose Pub    │              │
│  │(marker_map  │                    │  Status Pub  │              │
│  │   .yaml)     │                    └──────────────┘              │
│  └──────────────┘                                                   │
├─────────────────────────────────────────────────────────────────────┤
│  离线建图额外模块：                                                   │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐            │
│  │ Path Sub     │ → │ OfflineMapper│ → │ YAML Writer  │            │
│  │(KeyframePose)│   │(Aggregate)   │   │(marker_map  │            │
│  │              │   │              │   │   .yaml)     │            │
│  └──────────────┘   └──────────────┘   └──────────────┘            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. 核心数据结构

定义于 [`include/aruco_relocalization/common/types.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/common/types.hpp)

### 3.1 MarkerObservation（单 Marker 观测）

```cpp
struct MarkerObservation {
  int id = -1;                                    // marker ID
  std::array<cv::Point2f, 4> corners;             // 四个角点像素坐标
  Eigen::Isometry3d T_cam_marker = Eigen::Isometry3d::Identity();  // 相机到 marker 的变换
  double reprojection_error = 0.0;                // 角点重投影误差
};
```

### 3.2 FrameDetection（一帧检测结果）

```cpp
struct FrameDetection {
  double timestamp = 0.0;
  std::vector<MarkerObservation> markers;
};
```

### 3.3 MarkerItem（地图中的 Marker）

```cpp
struct MarkerItem {
  int id = -1;
  Eigen::Isometry3d T_map_marker = Eigen::Isometry3d::Identity();  // map 坐标系下 marker 位姿
  double size = 0.2;                              // marker 物理尺寸（米）
  int observation_count = 0;                      // 观测帧数
  double mean_reproj_error = 0.0;                 // 平均重投影误差
};
```

### 3.4 SolveResult（求解结果）

```cpp
struct SolveResult {
  bool success = false;
  Eigen::Isometry3d T_map_cam = Eigen::Isometry3d::Identity();   // 相机在 map 下的位姿
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();  // base_link 在 map 下的位姿
  int num_markers_used = 0;
  std::vector<int> marker_ids;
  std::vector<int> inlier_indices;
  double solve_time_ms = 0.0;
  std::string failure_reason;
};
```

### 3.5 ValidationResult（校验结果）

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

---

## 4. 子模块详细设计

### 4.1 MarkerDetector（Marker 检测器）

**文件**: [`include/aruco_relocalization/detector/marker_detector.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/detector/marker_detector.hpp) / [`src/detector/marker_detector.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/detector/marker_detector.cpp)

#### 职责
- 使用 OpenCV `cv::aruco` 模块检测图像中的 ArUco Marker
- 对每个检测到的 Marker，通过 `cv::aruco::estimatePoseSingleMarkers` 估计 `T_cam_marker`
- 计算每个 Marker 的角点重投影误差

#### 配置参数（DetectorConfig）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `dictionary` | string | `DICT_4X4_50` | ArUco 字典类型，支持 4x4/5x5/6x6/7x7/ARUCO_ORIGINAL |
| `marker_size` | double | 0.2 | Marker 物理边长（米） |
| `corner_subpix` | bool | true | 是否启用亚像素角点精化 |
| `corner_refinement_win_size` | int | 5 | 角点精化窗口大小 |

#### 处理流程

```
输入图像 (cv::Mat)
    ↓
cv::aruco::detectMarkers() → ids, corners
    ↓
cv::aruco::estimatePoseSingleMarkers() → rvecs, tvecs
    ↓
计算重投影误差（projectPoints 对比原始 corners）
    ↓
输出 FrameDetection（含 T_cam_marker 和 reprojection_error）
```

#### 支持的 ArUco 字典

- `DICT_4X4_50/100/250/1000`
- `DICT_5X5_50/100/250/1000`
- `DICT_6X6_50/100/250/1000`
- `DICT_7X7_50/100/250/1000`
- `DICT_ARUCO_ORIGINAL`

---

### 4.2 RelocalizationSolver（重定位求解器）

**文件**: [`include/aruco_relocalization/solver/relocalization_solver.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/solver/relocalization_solver.hpp) / [`src/solver/relocalization_solver.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/solver/relocalization_solver.cpp)

#### 职责
- 将检测到的 Marker 与地图匹配，过滤未在地图中注册的 ID
- 根据匹配到的 Marker 数量，选择单 Marker 求解或多 Marker 联合 PnP 求解
- 将求解结果从 `camera` 坐标系转换到 `base_link` 坐标系

#### 配置参数（SolverConfig）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_markers` | int | 1 | 最小有效 marker 数 |
| `use_ransac` | bool | true | 是否使用 RANSAC（多 Marker 时） |
| `ransac_iterations` | int | 100 | RANSAC 迭代次数 |
| `ransac_reproj_error` | double | 8.0 | RANSAC 重投影误差阈值（像素） |
| `ransac_confidence` | double | 0.99 | RANSAC 置信度 |

#### 求解策略

**单 Marker 求解**（`matched_obs.size() == 1`）
- 直接使用 `T_map_cam = T_map_marker * T_cam_marker.inverse()`
- 无迭代优化，速度最快
- 精度受限于单 Marker 位姿估计的噪声

**多 Marker 联合 PnP 求解**（`matched_obs.size() >= 2`）
- 构建 2D-3D 点对：所有 Marker 的 4 个角点（图像坐标 → 地图坐标）
- 调用 `cv::solvePnPRansac` 或 `cv::solvePnP` 求解相机位姿
- 输出 inlier 索引，用于后续校验

**坐标系转换**
```
T_map_base = T_map_cam * T_base_cam.inverse()
```
其中 `T_base_cam` 为相机外参（base_link → camera_link）。

---

### 4.3 RelocalizationValidator（重定位校验器）

**文件**: [`include/aruco_relocalization/validator/relocalization_validator.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/validator/relocalization_validator.hpp) / [`src/validator/relocalization_validator.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/validator/relocalization_validator.cpp)

#### 职责
- 对求解器输出的候选位姿进行多重校验，过滤异常结果
- 提供明确的拒绝原因（reject_reason），便于调试

#### 配置参数（ValidatorConfig）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_reproj_error` | double | 10.0 | 最大允许平均重投影误差（像素） |
| `max_marker_distance_error` | double | 0.1 | 最大允许 Marker 间距误差（米） |
| `max_translation_jump` | double | 2.0 | 最大允许平移跳变（米） |
| `max_rotation_jump` | double | 30.0 | 最大允许旋转跳变（度） |
| `use_motion_check` | bool | true | 是否启用运动一致性校验 |
| `min_markers` | int | 2 | 校验通过所需最小 Marker 数 |

#### 校验流程

```
候选位姿 SolveResult
    ↓
1. 数量阈值检查（num_valid_markers >= min_markers）
    ↓
2. 重投影误差校验（mean_reproj_error <= max_reproj_error）
    ↓
3. 几何一致性校验（多 Marker 时，marker 间距误差 <= max_marker_distance_error）
    ↓
4. 运动一致性校验（与上一帧位姿对比，平移/旋转跳变在阈值内）
    ↓
全部通过 → ValidationResult.passed = true
任一失败 → 设置 reject_reason，返回失败
```

#### 校验方法详解

**重投影误差计算**
- 将地图中 Marker 的 3D 角点通过当前 `T_map_cam` 投影到图像平面
- 与检测到的角点坐标对比，计算像素误差

**几何一致性计算**
- 计算地图中任意两个 Marker 的 3D 距离（`dist_map`）
- 通过当前帧估计的 `T_map_cam` 和 `T_cam_marker` 计算相同 Marker 的估计距离（`dist_est`）
- 取所有 Marker 对中的最大误差：`max_error = max(|dist_map - dist_est|)`

**运动一致性计算**
- 平移跳变：`trans_jump = ||current.translation - last.translation||`
- 旋转跳变：通过相对旋转矩阵的 trace 计算轴角：`rot_jump = acos((trace - 1) / 2) * 180 / π`

---

### 4.4 MarkerMapLoader（Marker 地图加载器）

**文件**: [`include/aruco_relocalization/map/marker_map_loader.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/map/marker_map_loader.hpp) / [`src/map/marker_map_loader.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/map/marker_map_loader.cpp)

#### 职责
- 从 YAML 文件加载预构建的 Marker 地图
- 提供按 ID 查询 Marker 位姿的接口

#### YAML 格式（marker_map.yaml）

```yaml
frame_id: "map"
source_pose_topic: "/slam_mapping_v2/path"
generation_time: "2026-08-26 10:30:00"

markers:
  - id: 1
    x: 1.234
    y: 2.345
    z: 0.567
    qw: 0.707
    qx: 0.0
    qy: 0.0
    qz: 0.707
    size: 0.2
    observation_count: 42
    mean_reproj_error: 1.23
```

---

### 4.5 OfflineMapper（离线建图器）

**文件**: [`include/aruco_relocalization/mapping/offline_mapper.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/mapping/offline_mapper.hpp) / [`src/mapping/offline_mapper.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/mapping/offline_mapper.cpp)

#### 职责
- 订阅 `slam_mapping_v2/path` 获取关键帧位姿序列
- 订阅相机图像，检测 Marker
- 通过时间同步找到与图像时间戳最近的关键帧位姿
- 聚合同一 Marker 在多帧中的观测，计算加权平均位姿
- 输出 `marker_map.yaml`

#### 建图流程

```
slam_mapping_v2/path → 存储关键帧位姿序列（KeyframePose）
    ↓
camera/image_raw → 检测 Marker → T_cam_marker
    ↓
时间同步：findNearestKeyframe(image_time)
    ↓
T_map_marker = T_map_base * T_base_cam * T_cam_marker
    ↓
aggregateMarker(id, T_map_marker, reproj_error)
    ↓
多帧聚合 → 加权平均（位置按 1/reproj_error 加权，旋转按四元数加权平均）
    ↓
保存 marker_map.yaml
```

#### 聚合参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `time_sync_tolerance` | double | 0.5 | 图像与关键帧的最大时间差（秒） |
| `min_observations` | int | 5 | 保存 Marker 所需最小观测次数 |
| `max_reproj_error` | double | 5.0 | 参与聚合的最大重投影误差（像素） |

---

## 5. ROS 接口设计

### 5.1 在线重定位节点（aruco_reloc_online_node）

#### 订阅话题（Subscribers）

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/camera/li_image` | `sensor_msgs/Image` | 相机图像（BGR8） |
| `/localization/points` | `sensor_msgs/PointCloud2` | 点云（用于位姿验证） |
| `/localization_v2/relocalization/command` | `std_msgs/String` | 控制命令：`"start"` / `"stop"` |

#### 发布话题（Publishers）

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/localization/relocalization/result` | `deeplumin_msgs/RelocalizationResult` | 重定位结果（含协方差、拒绝原因、点云验证指标） |
| `/aruco_reloc/pose` | `geometry_msgs/PoseStamped` | 重定位位姿（可视化用） |
| `/aruco_reloc/status` | `std_msgs/String` | 状态字符串（调试用） |

#### deeplumin_msgs/RelocalizationResult 字段映射

| 字段 | 来源 |
|------|------|
| `header.frame_id` | `config.frame_id`（默认 `"map"`） |
| `child_frame_id` | `config.child_frame_id`（默认 `"base_link"`） |
| `pose.pose` | `solve_result.T_map_base` |
| `pose.covariance` | 固定值：位置 0.01，姿态 0.001 |
| `success` | `solve_result.success && reject_reason.empty()` |
| `accepted` | `false`（由 localization_manager 门控后设置） |
| `method` | `"aruco"` |
| `reject_reason` | `validation.reject_reason` |
| `inlier_count` | `solve_result.num_markers_used` |
| `elapsed_ms` | `solve_result.solve_time_ms` |

### 5.2 离线建图节点（aruco_reloc_offline_mapping_node）

#### 订阅话题（Subscribers）

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/slam_mapping_v2/path` | `nav_msgs/Path` | SLAM 关键帧轨迹 |
| `/camera/image_raw` | `sensor_msgs/Image` | 相机图像 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |

#### 输出文件

| 文件 | 说明 |
|------|------|
| `marker_map.yaml` | 生成的 Marker 地图，供在线节点加载 |

---

## 6. 参数配置

### 6.1 在线节点配置（aruco_relocalization.yaml）

```yaml
# 话题配置
image_topic: "/camera/li_image"
result_topic: "/localization/relocalization/result"
pose_topic: "/aruco_reloc/pose"
status_topic: "/aruco_reloc/status"
command_topic: "/localization_v2/relocalization/command"

# 坐标系
frame_id: "map"
child_frame_id: "base_link"

# marker 地图路径
marker_map_path: "$(find aruco_relocalization)/config/marker_map.yaml"

# 相机外参（base_link -> camera_link）
extrinsic:
  base_to_camera:
    translation: [0.0, 0.0, 0.0]
    rotation: [1.0, 0.0, 0.0, 0.0]  # qw, qx, qy, qz

# 是否从 TF 获取外参（优先级高于配置文件）
use_tf_extrinsic: false

# ArUco 配置
aruco:
  dictionary: "DICT_4X4_50"
  marker_size: 0.2  # meters
  corner_subpix: true

# 求解器配置
solver:
  min_markers: 1
  use_ransac: true
  ransac_iterations: 100
  ransac_reproj_error: 8.0
  ransac_confidence: 0.99

# 校验器配置
validator:
  max_reproj_error: 10.0  # pixels
  max_marker_distance_error: 0.1  # meters
  max_translation_jump: 2.0  # meters
  max_rotation_jump: 30.0  # degrees
  use_motion_check: true
  min_markers: 2  # 推荐 >=2
```

### 6.2 参数调优建议

| 场景 | 建议调整 |
|------|----------|
| **Marker 稀疏** | `solver.min_markers = 1`, `validator.min_markers = 1` |
| **高精度要求** | 降低 `validator.max_reproj_error` 到 5.0，提高 `solver.ransac_reproj_error` 到 4.0 |
| **高动态场景** | 降低 `validator.max_translation_jump` 到 1.0，`max_rotation_jump` 到 15.0 |
| **大场景** | 使用更大的 Marker（`marker_size > 0.3`），或选择 `DICT_5X5_250` 字典 |

---

## 7. Launch 文件

### 7.1 在线重定位

**文件**: [`launch/aruco_relocalization.launch`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/launch/aruco_relocalization.launch)

```xml
<launch>
  <arg name="config_file" default="$(find aruco_relocalization)/config/aruco_relocalization.yaml"/>
  <rosparam command="load" file="$(arg config_file)" ns="aruco_reloc"/>

  <node pkg="aruco_relocalization" type="aruco_reloc_online_node" name="aruco_reloc_online" output="screen">
    <param name="marker_map_path" value="$(find aruco_relocalization)/config/marker_map.yaml"/>
  </node>
</launch>
```

### 7.2 离线建图

**文件**: [`launch/offline_mapping.launch`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/launch/offline_mapping.launch)

```xml
<launch>
  <arg name="config_file" default="$(find aruco_relocalization)/config/aruco_relocalization.yaml"/>
  <rosparam command="load" file="$(arg config_file)" ns="aruco_offline_mapping"/>

  <arg name="output_path" default="$(find aruco_relocalization)/config/marker_map.yaml"/>
  <arg name="path_topic" default="/slam_mapping_v2/path"/>
  <arg name="image_topic" default="/camera/image_raw"/>
  <arg name="camera_info_topic" default="/camera/camera_info"/>

  <node pkg="aruco_relocalization" type="aruco_reloc_offline_mapping_node" name="aruco_offline_mapping" output="screen">
    <param name="path_topic" value="$(arg path_topic)"/>
    <param name="image_topic" value="$(arg image_topic)"/>
    <param name="camera_info_topic" value="$(arg camera_info_topic)"/>
    <param name="output_path" value="$(arg output_path)"/>
  </node>
</launch>
```

> **注意**: 离线建图需同时启动 `slam_mapping_v2` 节点，确保 `/slam_mapping_v2/path` 有数据输出。

---

## 8. 处理流程时序

### 8.1 在线重定位时序

```
[初始化阶段]
  1. 加载 marker_map.yaml
  2. 订阅 camera_info，提取内参矩阵 K 和畸变系数 D
  3. 初始化 Detector、Solver、Validator
  4. 设置 enabled_ = true（默认启用）

[运行阶段 —— 每帧图像]
  1. onImage() 触发
  2. 检查 enabled_ && camera_initialized_
  3. cv_bridge 转换图像为 cv::Mat
  4. detector_.detect() → FrameDetection
  5. 若 markers.empty()，直接返回
  6. solver_.solve() → SolveResult
     - ID 匹配过滤
     - 单/多 Marker PnP 求解
     - 转换到 base_link
  7. validator_.validate() → ValidationResult
     - 数量阈值
     - 重投影误差
     - 几何一致性
     - 运动一致性
  8. 若校验通过：
     - 更新 last_reloc_pose_
     - publishResult() → /aruco_reloc/result
     - publishPose() → /aruco_reloc/pose
  9. 若校验失败：
     - publishResult()（success=false，附 reject_reason）
```

### 8.2 离线建图时序

```
[初始化阶段]
  1. 加载配置参数
  2. 初始化 OfflineMapper

[运行阶段]
  1. onPath()：接收并缓存关键帧位姿序列（按时间排序）
  2. onCameraInfo()：接收相机内参，初始化 Detector
  3. onImage()：
     - 检测 Marker
     - 时间同步：findNearestKeyframe(image_time)
     - 计算 T_map_marker = T_map_base * T_base_cam * T_cam_marker
     - 聚合到 marker_observations_

[退出阶段]
  1. SIGINT 或节点析构触发 saveMarkerMap()
  2. 对每个 Marker：
     - 位置加权平均（权重 = 1 / reproj_error）
     - 旋转加权平均（四元数，处理双覆盖）
     - 计算平均重投影误差
  3. 写入 marker_map.yaml
```

---

## 9. 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 单次求解耗时 | < 10ms | 单 Marker 约 1-2ms，多 Marker PnP 约 5-10ms |
| 检测帧率 | 与相机帧率同步 | 通常 30Hz，无额外降频 |
| 重投影误差 | < 10 pixels | 校验阈值，实际通常 1-5 pixels |
| 平移精度 | ~5cm | 依赖 Marker 大小和距离 |
| 旋转精度 | ~2° | 依赖 Marker 大小和视角 |

---

## 10. 调试与工具

### 10.1 分析脚本

| 脚本 | 功能 |
|------|------|
| [`scripts/test_aruco_detection.py`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/scripts/test_aruco_detection.py) | 批量测试 ArUco 检测，统计检测率、重投影误差、距离分布 |
| [`scripts/analyze_aruco_accuracy.py`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/scripts/analyze_aruco_accuracy.py) | 详细精度分析，绘制误差 vs 距离/视角/Marker 大小关系图 |

### 10.2 状态输出

在线节点通过 `/aruco_reloc/status` 发布实时状态，格式示例：
```
success=1 markers=3 time_ms=4.231
success=0 markers=2 time_ms=2.156 reject=reproj_error_too_large (15.3/10.0)
```

### 10.3 常见问题排查

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| 无 Marker 检测 | 字典不匹配 / Marker 不在视野 | 检查 `dictionary` 配置；运行 `test_aruco_detection.py` |
| 重投影误差大 | Marker 尺寸不准 / 相机标定差 | 精确测量 Marker 边长；重新标定相机 |
| 位姿跳变 | 运动校验过严 / 单 Marker 噪声大 | 放宽 `max_translation_jump`；增加 Marker 数量 |
| 离线建图无输出 | 时间同步失败 / 无关键帧 | 检查 `path_topic` 是否有数据；调整 `time_sync_tolerance` |

---

## 11. 依赖关系

### 11.1 ROS 依赖

- `roscpp`
- `cv_bridge`
- `image_transport`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `std_srvs`
- `tf2` / `tf2_ros`
- `deeplumin_msgs`

### 11.2 第三方库

- **OpenCV**（`core`, `aruco`, `calib3d`, `imgproc`）
- **Eigen3**
- **yaml-cpp**

### 11.3 编译依赖

- `deeplumin_cmake`（提供 CMake 工具链）
- C++17

---

## 12. 文件清单

| 路径 | 说明 |
|------|------|
| [`CMakeLists.txt`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/CMakeLists.txt) | 构建配置 |
| [`package.xml`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/package.xml) | 包描述 |
| [`config/aruco_relocalization.yaml`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/config/aruco_relocalization.yaml) | 在线节点参数 |
| [`config/marker_map.yaml`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/config/marker_map.yaml) | Marker 地图（示例/生成） |
| [`launch/aruco_relocalization.launch`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/launch/aruco_relocalization.launch) | 在线节点启动 |
| [`launch/offline_mapping.launch`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/launch/offline_mapping.launch) | 离线建图启动 |
| [`include/aruco_relocalization/common/types.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/common/types.hpp) | 核心数据结构 |
| [`include/aruco_relocalization/detector/marker_detector.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/detector/marker_detector.hpp) | 检测器接口 |
| [`include/aruco_relocalization/solver/relocalization_solver.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/solver/relocalization_solver.hpp) | 求解器接口 |
| [`include/aruco_relocalization/validator/relocalization_validator.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/validator/relocalization_validator.hpp) | 校验器接口 |
| [`include/aruco_relocalization/map/marker_map_loader.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/map/marker_map_loader.hpp) | 地图加载器接口 |
| [`include/aruco_relocalization/mapping/offline_mapper.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/mapping/offline_mapper.hpp) | 离线建图器接口 |
| [`include/aruco_relocalization/ros/param_loader.hpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/include/aruco_relocalization/ros/param_loader.hpp) | 参数加载 |
| [`src/ros/online_node.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/ros/online_node.cpp) | 在线节点主程序 |
| [`src/ros/offline_mapping_node.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/ros/offline_mapping_node.cpp) | 离线建图节点主程序 |
| [`src/detector/marker_detector.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/detector/marker_detector.cpp) | 检测器实现 |
| [`src/solver/relocalization_solver.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/solver/relocalization_solver.cpp) | 求解器实现 |
| [`src/validator/relocalization_validator.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/validator/relocalization_validator.cpp) | 校验器实现 |
| [`src/map/marker_map_loader.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/map/marker_map_loader.cpp) | 地图加载器实现 |
| [`src/mapping/offline_mapper.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/mapping/offline_mapper.cpp) | 离线建图器实现 |
| [`src/ros/param_loader.cpp`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/src/ros/param_loader.cpp) | 参数加载实现 |
| [`scripts/test_aruco_detection.py`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/scripts/test_aruco_detection.py) | 检测测试脚本 |
| [`scripts/analyze_aruco_accuracy.py`](file:///home/hhy/2004ros/DeepLumin/src/localization/aruco_relocalization/scripts/analyze_aruco_accuracy.py) | 精度分析脚本 |