# localization_v2 详细设计文档

## 1. 概述

### 1.1 项目背景
`localization_v2` 是新一代定位融合节点，基于 `ysw_style_fusion` 的 `map->odom` 修正思路，融合 `slam_odometry_v2` 局部里程计与 `global_matcher` 全局匹配结果，输出平滑且全局一致的 `map->base_link` 位姿，并预留 `relocalization` 重定位初始化接口。

### 1.2 设计目标
- **高频平滑输出**：以 100Hz 输出融合位姿，满足控制闭环需求
- **全局一致性**：通过低频全局匹配结果修正 `map->odom`，消除里程计累积漂移
- **多源初始化**：支持重定位、人工初值、全局匹配三种初始化来源，带仲裁机制
- **解耦可测试**：核心融合逻辑纯 C++，不依赖 ROS，便于单元测试与跨平台复用

### 1.3 术语定义
| 术语 | 定义 |
|------|------|
| map 系 | 全局地图坐标系，通常为地图原点 |
| odom 系 | SLAM 局部坐标系，由 slam_odometry_v2 维护 |
| base 系 | 车辆本体坐标系（base_link） |
| map->odom | 全局坐标系到局部坐标系的修正变换，由本节点维护 |
| map->base | 车辆在全局地图中的位姿，本节点最终输出 |
| odom->base | 局部里程计位姿，由 slam_odometry_v2 高频输出 |
| jump guard | 跳变门限，防止单次全局匹配结果导致位姿突变 |

---

## 2. 系统架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        localization_v2_node                          │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │
│  │   ROS 订阅层     │  │   初始化状态机   │  │     ROS 发布层       │  │
│  │                 │  │                 │  │                     │  │
│  │ • slam_odom     │  │ • UNINITIALIZED │  │ • fused_odom        │  │
│  │ • global_match  │→ │ • RELOCALIZING  │→ │ • map_to_odom       │  │
│  │ • initial_pose  │  │ • TRACKING      │  │ • path              │  │
│  │ • relocalization│  │                 │  │ • status            │  │
│  │ • source_cloud  │  │                 │  │ • tf (map->odom)    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────────┘  │
│           │                    │                      │              │
│           ▼                    ▼                      ▼              │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                      FusionCore (纯 C++)                     │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │    │
│  │  │  odom_base  │  │  map_odom   │  │     jump guard      │  │    │
│  │  │  (缓存)     │  │  (修正量)   │  │   (跳变门限)        │  │    │
│  │  └─────────────┘  └─────────────┘  └─────────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────┘    │
│           ▲                                                          │
│           │                                                          │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │              IRelocalizationClient (抽象接口)                │    │
│  │         ┌─────────────────────────────────────┐              │    │
│  │         │    RelocalizationRosAdapter         │              │    │
│  │         │  (默认 ROS 话题实现)                 │              │    │
│  │         └─────────────────────────────────────┘              │    │
│  └─────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 模块划分

| 模块 | 职责 | 对应目录/文件 |
|------|------|--------------|
| ROS 节点层 | 话题订阅/发布、TF 广播、参数加载、定时器 | `src/ros/localization_v2_node.cpp` |
| 配置加载 | 从 ROS 参数服务器加载全部配置 | `src/ros/localization_v2_config.cpp` |
| 融合核心 | 位姿融合数学、跳变门控 | `src/core/fusion_core.cpp` |
| 初始化状态机 | 启动阶段状态切换、重定位命令决策 | `src/core/initialization.cpp` |
| 重定位接口 | 抽象接口 + ROS 默认实现 | `src/ros/relocalization_ros_adapter.cpp` |
| 数据类型 | 位姿数学工具、通用结构体 | `include/localization_v2/core/types.hpp` |

---

## 3. 代码文件作用

### 3.1 核心层（core/）

#### `include/localization_v2/core/types.hpp`
- **作用**：定义纯 C++ 数据类型与位姿数学工具
- **关键内容**：
  - `Pose3d`：位姿结构（position + orientation）
  - `Vec3d` / `Quatd`：Eigen 别名
  - `compose(a, b)`：位姿复合（先 b 后 a）
  - `inverse(a)`：位姿求逆
  - `between(from, to)`：相对位姿
  - `yawDiffDeg(a, b)`：偏航角差（度，范围 [-180, 180]）
  - `translationDiff(a, b)`：平移距离
  - `isFinite`：数值合法性检查

#### `include/localization_v2/core/fusion_core.hpp` / `src/core/fusion_core.cpp`
- **作用**：融合核心，维护 `odom->base` 和 `map->odom`，输出 `map->base`
- **关键内容**：
  - `JumpGuardConfig`：跳变门限配置
  - `GlobalMatchUpdate`：全局匹配结果数据结构
  - `RelocalizationUpdate`：重定位结果数据结构
  - `FusionCore` 类：融合计算与门控逻辑

#### `include/localization_v2/core/initialization.hpp` / `src/core/initialization.cpp`
- **作用**：初始化状态机，管理启动阶段的状态流转
- **关键内容**：
  - `InitConfig`：初始化策略配置
  - `RelocalizationPolicy`：重定位启停策略
  - `InitStateMachine` 类：三状态状态机

### 3.2 接口层（interface/）

#### `include/localization_v2/interface/relocalization_interface.hpp`
- **作用**：重定位客户端抽象接口，解耦节点与重定位模块
- **关键内容**：
  - `IRelocalizationClient`：纯虚接口（start/stop/setResultCallback）
  - `RelocalizationSignal`：重定位结果回调数据结构
  - `RelocalizationResultCallback`：回调函数类型

### 3.3 ROS 适配层（ros/）

#### `include/localization_v2/ros/localization_v2_config.hpp` / `src/ros/localization_v2_config.cpp`
- **作用**：配置结构体定义与从 ROS 参数服务器加载
- **关键内容**：
  - `LocalizationV2Config`：包含全部可调参数的结构体
  - `loadLocalizationV2Config()`：从 `~` 命名空间加载参数

#### `include/localization_v2/ros/relocalization_ros_adapter.hpp` / `src/ros/relocalization_ros_adapter.cpp`
- **作用**：`IRelocalizationClient` 的默认 ROS 实现
- **关键内容**：
  - `RelocalizationRosAdapter`：订阅重定位结果话题，发布 start/stop 命令

#### `src/ros/localization_v2_node.cpp`
- **作用**：节点主入口，整合所有模块
- **关键内容**：
  - `LocalizationV2Node` 类：ROS 回调、定时器、初始化仲裁、发布逻辑

### 3.4 其他文件

#### `config/localization_v2.yaml`
- 唯一配置文件，包含坐标系、话题、运行时、全局匹配、初始化、重定位、人工初值等全部参数

#### `launch/localization_v2.launch`
- 完整定位链路启动文件，包含 slam_odometry_v2、global_matcher_v2、localization_v2、可选重定位模块

---

## 4. 数据结构设计

### 4.1 位姿类型（types.hpp）

```cpp
using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;

struct Pose3d {
  Vec3d position = Vec3d::Zero();
  Quatd orientation = Quatd::Identity();
};
```

**含义**：
- `Pose3d` 表示 "parent -> child" 的刚体变换
- `position`：child 在 parent 系中的平移
- `orientation`：child 相对于 parent 的旋转（四元数）

### 4.2 跳变门限配置（fusion_core.hpp）

```cpp
struct JumpGuardConfig {
  bool enabled = false;
  double max_translation = 5.0;
  double max_yaw_deg = 50.0;
};
```

**含义**：
- `enabled`：是否启用跳变门限
- `max_translation`：单次修正最大平移跳变（米）
- `max_yaw_deg`：单次修正最大偏航跳变（度）

### 4.3 全局匹配更新（fusion_core.hpp）

```cpp
struct GlobalMatchUpdate {
  bool success = false;
  bool has_pose = false;
  Pose3d pose = Pose3d();
  bool pose_is_map_base = false;
  std::string map_id;
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
};
```

**含义**：
- `success`：匹配是否成功
- `has_pose`：消息是否携带可用位姿
- `pose`：位姿值（map->odom 或 map->base，取决于 `pose_is_map_base`）
- `pose_is_map_base`：true 表示 pose 为 map->base，false 表示 map->odom
- `map_id`：地图标识
- `fitness_score`：匹配 fitness 分数
- `inlier_ratio`：内点比率

### 4.4 重定位更新（fusion_core.hpp）

```cpp
struct RelocalizationUpdate {
  bool success = false;
  bool accepted = false;
  Pose3d map_base = Pose3d();
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  std::string map_id;
  std::string method;
};
```

**含义**：
- `success`：重定位是否成功
- `accepted`：是否被接受（外部质量门限）
- `map_base`：重定位给出的 map->base_link 位姿
- `fitness_score` / `inlier_ratio` / `inlier_rmse`：质量指标
- `map_id` / `method`：地图标识与重定位方法名

### 4.5 初始化配置（initialization.hpp）

```cpp
struct InitConfig {
  bool require_initial_pose = false;
  bool use_relocalization_init = true;
  bool use_manual_initialpose = true;
  bool use_global_matcher_init = true;
};

struct RelocalizationPolicy {
  bool enabled = true;
  bool auto_start = true;
  bool stop_on_success = true;
  bool stop_on_manual_pose = true;
  bool stop_on_global_match = true;
};
```

**含义**：
- `require_initial_pose`：是否必须获得初始位姿后才输出定位
- `use_relocalization_init`：是否允许使用重定位作为初始位姿来源
- `use_manual_initialpose`：是否允许人工 initial pose 完成初始化
- `use_global_matcher_init`：是否允许 global_matcher 自身成功即视为初始化完成
- `enabled`：重定位功能总开关
- `auto_start`：未初始化时是否自动请求启动重定位
- `stop_on_success` / `stop_on_manual_pose` / `stop_on_global_match`：对应事件发生后是否停止重定位

### 4.6 重定位信号（relocalization_interface.hpp）

```cpp
struct RelocalizationSignal {
  bool success = false;
  bool accepted = false;
  Pose3d map_base;
  double fitness_score = 0.0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  std::string map_id;
  std::string method;
};
```

**含义**：与 `RelocalizationUpdate` 对应，用于回调接口传递

### 4.7 节点配置（localization_v2_config.hpp）

```cpp
struct LocalizationV2Config {
  // 坐标系
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";

  // 输入话题
  std::string slam_odom_topic = "/localization/slam_odom";
  std::string global_matcher_result_topic = "/localization/global_matcher/result";
  std::string initial_pose_topic = "/initialpose";
  std::string source_cloud_topic = "/localization/cloud_registered_effect";

  // 输出话题
  std::string fused_odom_topic = "/localization_v2/odom";
  std::string map_to_odom_topic = "/localization_v2/map_to_odom";
  std::string fused_path_topic = "/localization_v2/path";
  std::string status_topic = "/localization_v2/status";
  std::string current_scan_in_map_topic = "/localization_v2/current_scan_in_map";
  std::string gm_anchor_topic = "/localization_v2/gm_anchor";

  // 运行时
  double output_rate_hz = 100.0;
  int max_path_size = 5000;
  bool publish_tf = true;
  bool publish_map_to_odom_tf = true;
  bool publish_odom_to_base_tf = true;
  bool publish_path = false;
  bool publish_current_scan_in_map = false;

  // global_matcher 结果语义与门控
  std::string gm_result_pose_frame = "odom";
  bool enable_jump_guard = true;
  double max_jump_translation = 5.0;
  double max_jump_yaw_deg = 50.0;

  // 初始化策略
  bool require_initial_pose = false;
  bool use_relocalization_init = true;
  bool use_manual_initialpose = true;
  bool use_global_matcher_init = true;

  // 重定位策略
  bool relocalization_enabled = true;
  bool relocalization_auto_start = true;
  bool relocalization_stop_on_success = true;
  bool relocalization_stop_on_manual_pose = true;
  bool relocalization_stop_on_global_match = true;
  bool relocalization_require_accepted = false;
  double relocalization_max_fitness_score = 1.0;
  double relocalization_min_inlier_ratio = 0.5;
  double relocalization_max_inlier_rmse = 0.8;
  double relocalization_fallback_timeout = 8.0;

  // 人工初始位姿 z 补全
  bool initial_pose_z_completion = true;
  double initial_pose_z_search_radius = 5.0;
  std::string initial_pose_map_pcd_path;
};
```

---

## 5. 接口设计

### 5.1 FusionCore 接口（fusion_core.hpp）

| 接口 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `setJumpGuard` | `const JumpGuardConfig&` | void | 设置跳变门限配置 |
| `requestJumpGuardBypass` | - | void | 请求下次跳过跳变门限（一次性） |
| `setOdomBase` | `const Pose3d&` | void | 设置 slam 局部里程计位姿 |
| `hasSlam` | - | bool | 是否已收到 slam 数据 |
| `odomBase` | - | `const Pose3d&` | 获取当前 odom->base |
| `hasMapOdom` | - | bool | 是否已设置 map->odom |
| `mapOdom` | - | `const Pose3d&` | 获取当前 map->odom |
| `updateFromGlobalMatch` | `const GlobalMatchUpdate&` | bool | 用 GM 结果更新 map->odom，带跳变门限 |
| `setMapOdom` | `const Pose3d&` | bool | 直接设置 map->odom（无跳变门限） |
| `updateMapOdomFromMapBase` | `const Pose3d&, bool` | bool | 用 map->base 反推 map->odom |
| `fusedPose` | - | Pose3d | 输出融合位姿 map->base |
| `isInitialized` | - | bool | 是否已初始化（有 map->odom） |

### 5.2 InitStateMachine 接口（initialization.hpp）

| 接口 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `configure` | `const InitConfig&, const RelocalizationPolicy&` | void | 配置状态机 |
| `state` | - | State | 获取当前状态 |
| `initialized` | - | bool | 是否已初始化（TRACKING） |
| `relocalizationRunning` | - | bool | 是否处于重定位阶段 |
| `stateName` | - | `const std::string&` | 获取状态名字符串 |
| `takeCommand` | - | RelocCommand | 取走并清零待下发的重定位命令 |
| `onSlamOdom` | - | void | slam 数据到达事件 |
| `onManualPose` | - | void | 人工初值到达事件 |
| `onRelocalizationResult` | bool | void | 重定位结果事件 |
| `onGlobalMatchSuccess` | - | void | GM 匹配成功事件 |

### 5.3 IRelocalizationClient 接口（relocalization_interface.hpp）

| 接口 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `start` | - | void | 请求启动重定位（幂等） |
| `stop` | - | void | 请求停止重定位（幂等） |
| `setResultCallback` | `RelocalizationResultCallback` | void | 注册结果回调 |

### 5.4 RelocalizationRosAdapter 接口（relocalization_ros_adapter.hpp）

继承 `IRelocalizationClient`，构造函数参数：
- `ros::NodeHandle& nh`：ROS 节点句柄
- `const std::string& result_topic`：重定位结果订阅话题
- `const std::string& command_topic`：重定位命令发布话题

---

## 6. 输入输出定义

### 6.1 输入（订阅话题）

| 话题名 | 消息类型 | 来源 | 含义 | 频率 |
|--------|----------|------|------|------|
| `/localization/slam_odom` | `nav_msgs/Odometry` | slam_odometry_v2 | 局部里程计 `odom->base_link` | 高频（100Hz+） |
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | global_matcher | 全局匹配结果 | 低频（1-10Hz） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz / 人工 | 人工初始位姿 | 事件触发 |
| `/localization/relocalization/result` | `deeplumin_msgs/RelocalizationResult` | relocalization | 重定位结果 | 事件触发 |
| `/localization/cloud_registered_effect` | `sensor_msgs/PointCloud2` | 点云预处理 | 当前帧点云（odom 系） | 与 lidar 同频 |

**GlobalMatchResult 字段含义**：
- `success`：匹配是否成功
- `pose`：位姿（map->odom 或 map->base，取决于配置）
- `map_id`：地图标识
- `fitness_score`：匹配 fitness 分数
- `inlier_ratio`：内点比率
- `inlier_rmse`：内点 RMSE
- `reject_reason`：拒绝原因（失败时）

**RelocalizationResult 字段含义**：
- `success`：重定位是否成功
- `accepted`：是否被接受
- `pose`：位姿（map->base_link）
- `fitness_score` / `inlier_ratio` / `inlier_rmse`：质量指标
- `map_id` / `method`：地图标识与重定位方法

### 6.2 输出（发布话题）

| 话题名 | 消息类型 | 订阅方 | 含义 | 频率 |
|--------|----------|--------|------|------|
| `/localization_v2/odom` | `nav_msgs/Odometry` | 控制/规划 | 融合位姿 `map->base_link` | 100Hz（定时器） |
| `/localization_v2/map_to_odom` | `nav_msgs/Odometry` | 可视化/调试 | 修正量 `map->odom` | 100Hz / 事件触发 |
| `/localization_v2/path` | `nav_msgs/Path` | RViz | 融合轨迹历史 | 100Hz（可选） |
| `/localization_v2/status` | `deeplumin_msgs/FusionStatus` | 监控 | 定位状态 | 100Hz |
| `/localization_v2/current_scan_in_map` | `sensor_msgs/PointCloud2` | RViz | 当前帧点云在 map 系 | 100Hz（可选） |
| `/localization_v2/gm_anchor` | `geometry_msgs/PoseWithCovarianceStamped` | global_matcher | 初始化仲裁重定位胜出时通知 GM 重锚 | 事件触发 |
| `TF: map->odom` | `geometry_msgs/TransformStamped` | TF 树 | 全局到局部坐标系修正 | 100Hz |
| `TF: odom->base` | `geometry_msgs/TransformStamped` | TF 树 | 局部里程计（可选，避免重复） | 100Hz |

**FusionStatus 字段含义**：
- `level`：状态等级（NORMAL / WARNING / ERROR）
- `state`：当前状态字符串（UNINITIALIZED / RELOCALIZING / TRACKING）
- `initialized`：是否已完成初始化
- `output_valid`：当前输出是否有效
- `current_map_id`：当前地图标识
- `is_slam_available`：slam 数据是否可用
- `is_global_matcher_available`：GM 是否可用
- `is_relocalization_available`：重定位是否可用
- `used_sources`：当前使用的数据源列表

---

## 7. 配置参数详解

### 7.1 坐标系（frames）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `frames/map` | string | "map" | 全局地图坐标系 |
| `frames/odom` | string | "odom" | SLAM 局部坐标系 |
| `frames/base_link` | string | "base_link" | 车辆本体坐标系 |

### 7.2 话题（topics）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `topics/slam_odom` | string | "/localization/slam_odom" | slam 里程计输入 |
| `topics/global_matcher_result` | string | "/localization/global_matcher/result" | GM 结果输入 |
| `topics/initial_pose` | string | "/initialpose" | 人工初值输入 |
| `topics/source_cloud` | string | "/localization/cloud_registered_effect" | 点云输入（可视化） |
| `topics/fused_odom` | string | "/localization_v2/odom" | 融合里程计输出 |
| `topics/map_to_odom` | string | "/localization_v2/map_to_odom" | 修正量输出 |
| `topics/fused_path` | string | "/localization_v2/path" | 轨迹输出 |
| `topics/status` | string | "/localization_v2/status" | 状态输出 |
| `topics/current_scan_in_map` | string | "/localization_v2/current_scan_in_map" | 点云 map 系输出 |
| `topics/gm_anchor` | string | "/localization_v2/gm_anchor" | GM 锚点通知 |

### 7.3 运行时（runtime）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `runtime/output_rate_hz` | double | 100.0 | 输出频率（Hz） |
| `runtime/max_path_size` | int | 5000 | 轨迹最大保留点数 |
| `runtime/publish_tf` | bool | true | 是否发布 TF |
| `runtime/publish_map_to_odom_tf` | bool | true | 是否发布 map->odom TF |
| `runtime/publish_odom_to_base_tf` | bool | true | 是否发布 odom->base TF |
| `runtime/publish_path` | bool | false | 是否发布轨迹 |
| `runtime/publish_current_scan_in_map` | bool | false | 是否发布 map 系点云 |

### 7.4 全局匹配结果（global_matcher）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `global_matcher/result_pose_frame` | string | "odom" | GM 结果坐标系（"odom" 或 "base"） |
| `global_matcher/enable_jump_guard` | bool | true | 是否启用跳变门限 |
| `global_matcher/max_jump_translation` | double | 5.0 | 最大平移跳变（米） |
| `global_matcher/max_jump_yaw_deg` | double | 50.0 | 最大偏航跳变（度） |

### 7.5 初始化策略（initialization）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `initialization/require_initial_pose` | bool | false | 是否必须初始位姿后才输出 |
| `initialization/use_relocalization_init` | bool | true | 允许重定位初始化 |
| `initialization/use_manual_initialpose` | bool | true | 允许人工初值初始化 |
| `initialization/use_global_matcher_init` | bool | true | 允许 GM 自身成功初始化 |

### 7.6 重定位策略（relocalization）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `relocalization/enabled` | bool | true | 重定位功能总开关 |
| `relocalization/auto_start` | bool | true | 未初始化时自动启动重定位 |
| `relocalization/stop_on_success` | bool | true | 重定位成功后停止 |
| `relocalization/stop_on_manual_pose` | bool | true | 人工初值后停止 |
| `relocalization/stop_on_global_match` | bool | true | GM 成功后停止 |
| `relocalization/require_accepted` | bool | false | 是否要求 accepted=true |
| `relocalization/max_fitness_score` | double | 1.0 | 最大 fitness 分数门限 |
| `relocalization/min_inlier_ratio` | double | 0.5 | 最小内点比率门限 |
| `relocalization/max_inlier_rmse` | double | 0.8 | 最大内点 RMSE 门限 |
| `relocalization/fallback_timeout` | double | 8.0 | 双源初始化仲裁超时（秒） |

### 7.7 人工初始位姿（initial_pose）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `initial_pose/z_completion_enabled` | bool | true | 是否启用 z 补全 |
| `initial_pose/z_search_radius` | double | 5.0 | z 补全搜索半径（米） |
| `initial_pose/map_pcd_path` | string | "" | 地图 PCD 路径（留空关闭） |

---

## 8. 详细运行流程

### 8.1 启动流程

```
1. 节点启动
   ├── 加载配置（loadLocalizationV2Config）
   ├── 初始化 FusionCore（设置 jump guard）
   ├── 初始化 InitStateMachine（配置初始化策略与重定位策略）
   ├── 创建 IRelocalizationClient（默认 RelocalizationRosAdapter）
   ├── 注册订阅（slam_odom, global_match, initial_pose, source_cloud）
   ├── 注册发布（fused_odom, map_to_odom, path, status, gm_anchor, current_scan_in_map）
   └── 创建定时器（100Hz）
```

### 8.2 正常运行流程（TRACKING 状态）

```
slam_odom 回调（高频）
  ├── 更新 odom_base（FusionCore::setOdomBase）
  ├── 触发状态机事件（InitStateMachine::onSlamOdom，TRACKING 状态无动作）
  ├── 发布当前帧点云到 map 系（可选）
  └── 发布输出（融合位姿、TF、状态）

global_match 回调（低频）
  ├── 解析 GM 结果
  ├── 若 success=false，直接忽略
  ├── 转换坐标系（若 GM 输出 map->base，用当前 odom_base 反推 map->odom）
  ├── 缓存候选（初始化阶段）
  ├── TRACKING 阶段：
  │   ├── 跳变门限检查（FusionCore::updateFromGlobalMatch）
  │   ├── 若通过：更新 map_odom，发布 map_to_odom
  │   └── 若拒绝：记录日志，保持当前 map_odom
  └── 初始化阶段：交由初始化仲裁处理

timer 回调（100Hz）
  ├── 若无 slam 数据：发布单位 map->odom TF，等待 slam
  ├── 尝试初始化仲裁（若在 RELOCALIZING 状态）
  ├── 发布当前帧点云到 map 系（可选）
  └── 发布输出（融合位姿、TF、状态）
```

### 8.3 初始化流程

```
状态：UNINITIALIZED
  │
  ▼（收到第一条 slam_odom）
状态：RELOCALIZING（若启用 auto_start 和 use_relocalization_init）
  │   └── 下发 start 命令给重定位模块
  │
  ├── 重定位结果到达 ──→ 质量门限检查 ──→ 缓存结果 ──→ 尝试初始化仲裁
  ├── 人工初值到达 ────→ 直接完成初始化
  └── GM 结果到达 ────→ 缓存候选 ──→ 尝试初始化仲裁

初始化仲裁（tryCompleteInitialization）：
  ├── 单源模式（仅重定位或仅 GM）：
  │   └── 结果/候选就绪即完成初始化
  └── 双源模式（重定位 + GM）：
      ├── 两者都就绪：按质量仲裁（inlier_ratio 高且 rmse 低者胜）
      └── 超时（fallback_timeout）：取已就绪的一源

初始化完成：
  ├── 设置 map_odom
  ├── 状态切换为 TRACKING
  ├── 下发 stop 命令给重定位模块
  └── 若重定位胜出：发布 gm_anchor 通知 GM 重锚
```

### 8.4 融合公式

```
// 全局修正（低频，GM / 人工初值 / 重定位 更新）
map_odom = map_base ⊕ (odom_base)⁻¹

// 高频输出（100Hz，每次 slam_odom 或定时器触发）
map_base = map_odom ⊕ odom_base
```

**说明**：
- `map_odom`：全局坐标系到局部坐标系的修正变换，由本节点维护
- `odom_base`：局部里程计位姿，由 slam_odometry_v2 高频输出
- `map_base`：车辆在全局地图中的位姿，本节点最终输出

### 8.5 跳变门限流程

```
GM 结果到达（TRACKING 状态）
  ├── 计算候选 map_odom 与当前 map_odom 的差异
  │   ├── 平移差异：translationDiff(current, candidate)
  │   └── 偏航差异：yawDiffDeg(current, candidate)
  ├── 检查是否超过门限
  │   ├── 平移 > max_jump_translation 或 偏航 > max_jump_yaw_deg
  │   └── 且未设置 bypass 标志
  ├── 若超过：拒绝本次更新，记录 WARN 日志
  └── 若未超过：接受更新，更新 map_odom

特殊情况：
  ├── 人工初值后：设置 bypass 标志，下次 GM 结果跳过门限
  └── 重定位初始化后：设置 bypass 标志，下次 GM 结果跳过门限
```

---

## 9. 状态切换

### 9.1 状态机定义

```
┌─────────────┐     onSlamOdom()      ┌─────────────┐
│ UNINITIALIZED│ ────────────────────→ │ RELOCALIZING│
│             │  (auto_start=true)    │             │
└─────────────┘                       └──────┬──────┘
       ▲                                     │
       │                                     │
       │ onManualPose()                      │ onRelocalizationResult(accepted)
       │ onGlobalMatchSuccess()              │ onGlobalMatchSuccess()
       │                                     │
       └─────────────────────────────────────┘
                        │
                        ▼
                 ┌─────────────┐
                 │  TRACKING   │
                 │  (正常运行)  │
                 └─────────────┘
```

### 9.2 状态转换表

| 当前状态 | 事件 | 条件 | 下一状态 | 动作 |
|----------|------|------|----------|------|
| UNINITIALIZED | onSlamOdom | policy.enabled && policy.auto_start && init.use_relocalization_init | RELOCALIZING | 下发 start 命令 |
| UNINITIALIZED | onManualPose | init.use_manual_initialpose | TRACKING | 下发 stop 命令（若启用） |
| UNINITIALIZED | onGlobalMatchSuccess | init.use_global_matcher_init | TRACKING | 下发 stop 命令（若启用） |
| RELOCALIZING | onManualPose | init.use_manual_initialpose | TRACKING | 下发 stop 命令 |
| RELOCALIZING | onRelocalizationResult | accepted=true | TRACKING | 下发 stop 命令 |
| RELOCALIZING | onGlobalMatchSuccess | init.use_global_matcher_init | TRACKING | 下发 stop 命令 |
| TRACKING | （任意） | - | TRACKING | 无状态切换 |

### 9.3 重定位命令决策

| 状态 | takeCommand() 返回值 | 说明 |
|------|---------------------|------|
| UNINITIALIZED | kNone | 无命令 |
| RELOCALIZING | kStart（进入时） | 请求启动重定位 |
| TRACKING | kStop（进入时） | 请求停止重定位 |

---

## 10. 初值选择

### 10.1 初始化来源优先级

| 来源 | 优先级 | 说明 |
|------|--------|------|
| 重定位 | 高（双源仲裁时） | 提供全局初始位姿，带质量指标 |
| 人工初值 | 中 | RViz 2D Pose Estimate，可补全 z |
| GM 自身 | 低（双源仲裁时） | 全局匹配自身成功 |

### 10.2 双源仲裁逻辑

```cpp
// 重定位结果 vs GM 候选
const bool reloc_better =
    reloc_result_inlier_ratio >= gm_candidate_inlier_ratio &&
    reloc_result_inlier_rmse <= gm_candidate_inlier_rmse;

if (reloc_better) {
  completeWithReloc();  // 使用重定位结果
} else {
  completeWithGm();     // 使用 GM 候选
}
```

**仲裁指标**：
- `inlier_ratio`：越高越好（内点比率）
- `inlier_rmse`：越低越好（内点均方根误差）

### 10.3 人工初值 z 补全

```
1. 接收 RViz 2D 初值（x, y, yaw）
2. 加载地图 PCD（首次使用时缓存）
3. 以 (x, y) 为中心，z_search_radius 为半径搜索地图点云
4. 取搜索范围内 z 的最小值和最大值
5. z_out = 0.5 * (z_min + z_max)
6. 将补全后的 z 赋值给初值位姿
```

---

## 11. 质量控制

### 11.1 重定位结果质量门限

| 指标 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| fitness_score | `relocalization/max_fitness_score` | 1.0 | 匹配 fitness 分数上限 |
| inlier_ratio | `relocalization/min_inlier_ratio` | 0.5 | 内点比率下限 |
| inlier_rmse | `relocalization/max_inlier_rmse` | 0.8 | 内点 RMSE 上限 |
| accepted | `relocalization/require_accepted` | false | 是否要求 accepted=true |

### 11.2 GM 结果跳变门限（TRACKING 阶段）

| 指标 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| 平移跳变 | `global_matcher/max_jump_translation` | 5.0m | 单次修正最大平移 |
| 偏航跳变 | `global_matcher/max_jump_yaw_deg` | 50.0° | 单次修正最大偏航 |

### 11.3 初始化仲裁超时

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `relocalization/fallback_timeout` | 8.0s | 双源等待超时后取已就绪源 |

### 11.4 重定位结果时效窗

| 常量 | 值 | 说明 |
|------|-----|------|
| `kRelocResultMaxAge` | 3.0s | 重定位结果在仲裁中的有效时间 |

---

## 12. 线程安全

- **无显式线程**：localization_v2 为单线程 ROS 节点，所有回调在 ROS spinner 线程中串行执行
- **状态一致性**：FusionCore 和 InitStateMachine 的内部状态由 ROS 回调串行访问，无需额外锁
- **TF 广播**：使用 `tf2_ros::TransformBroadcaster`，线程安全

---

## 13. 依赖关系

### 13.1 编译依赖

| 包/库 | 用途 |
|--------|------|
| roscpp | ROS C++ 客户端 |
| std_msgs / geometry_msgs / nav_msgs / sensor_msgs | 标准消息类型 |
| tf2_ros | TF 广播 |
| pcl_ros / pcl_conversions | 点云处理 |
| deeplumin_msgs | 自定义消息（FusionStatus, GlobalMatchResult, RelocalizationResult） |
| Eigen3 | 矩阵/位姿运算 |
| PCL | 点云加载与变换 |

### 13.2 运行依赖

| 节点 | 输入/输出 | 说明 |
|------|----------|------|
| slam_odometry_v2 | 输入：odom->base | 局部里程计 |
| global_matcher(_v2) | 输入：map->odom 或 map->base | 全局匹配结果 |
| relocalization | 输入：重定位结果；输出：start/stop 命令 | 可选重定位模块 |
| RViz | 输入：/initialpose | 人工初值 |

---

## 14. 扩展性设计

### 14.1 重定位接口解耦

`IRelocalizationClient` 抽象接口允许替换重定位通信协议：
- **当前**：ROS 话题（RelocalizationRosAdapter）
- **未来**：ROS Service、ROS Action、gRPC 等

替换方式：
1. 实现新的 `IRelocalizationClient` 子类
2. 在 `LocalizationV2Node` 构造函数中替换 `relocalization_` 的实例化

### 14.2 核心层可测试性

`FusionCore` 和 `InitStateMachine` 纯 C++，不依赖 ROS：
- 可直接编写 gtest 单元测试
- 可跨平台复用（非 ROS 环境）

---

## 15. 日志与调试

### 15.1 关键日志

| 场景 | 级别 | 内容 |
|------|------|------|
| 节点启动 | INFO | 话题名、配置参数 |
| 初始化完成 | INFO | 初始化来源、map_id、质量指标 |
| GM 更新 | INFO | map_id、fitness、inlier、rmse |
| GM 跳变拒绝 | WARN | 平移/偏航差异、门限值 |
| 重定位结果拒绝 | WARN | 质量指标、门限值 |
| 等待 slam | WARN | 等待话题名 |
| 初始化超时 | WARN | 超时时间、保持初始化状态 |

### 15.2 调试话题

| 话题 | 用途 |
|------|------|
| `/localization_v2/map_to_odom` | 观察 map->odom 修正量变化 |
| `/localization_v2/status` | 观察状态机状态、数据源可用性 |
| `/localization_v2/path` | 观察融合轨迹 |
| `/localization_v2/current_scan_in_map` | 观察点云在 map 系的对齐效果 |