# slam_mapping 包

## 概述

`slam_mapping` 是 DeepLumin 自动驾驶系统中的激光雷达 SLAM 建图模块，基于 **Faster-LIO** 算法实现，并增加了关键帧管理、回环检测和位姿图优化功能。该模块用于构建高精度点云地图，支持在线建图和离线地图保存。

### 核心特性

- **实时建图**: 基于 Faster-LIO 的高频实时位姿估计和地图构建
- **关键帧管理**: 智能关键帧选取策略，平衡地图精度和存储效率
- **回环检测**: 基于子地图配准的回环检测，消除累积误差
- **位姿图优化**: 使用 GTSAM 进行全局位姿图优化
- **多场景支持**: 支持道路和停车场等多种场景建图
- **自动地图保存**: 支持程序退出时自动保存地图数据

---

## 目录结构

```
slam_mapping/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS 包配置
├── README.md                   # 本说明文档
│
├── config/                     # 配置文件目录
│   ├── slam_mapping.yaml       # 默认建图配置
│   └── slam_mapping_parking_rslidar.yaml  # 停车场场景配置
│
├── docs/                       # 文档目录
│   └── slam_mapping_alignment_checklist.md  # 对齐检查清单
│
├── include/slam_mapping/       # 头文件目录
│   ├── common/                 # 通用类型和配置
│   │   ├── config.hpp          # 配置结构体定义
│   │   └── types.hpp           # 基础类型定义
│   │
│   ├── core/                   # 核心算法实现
│   │   ├── i_loop_closure_backend.hpp   # 回环检测后端接口
│   │   ├── keyframe_manager.hpp         # 关键帧管理器
│   │   ├── loop_closure_backend.hpp     # 回环检测后端
│   │   ├── loop_closure_fastlio2_backend.hpp  # Fast-LIO2 回环后端
│   │   ├── map_saver.hpp                # 地图保存器
│   │   └── mapping_runtime.hpp          # 建图运行时核心
│   │
│   └── ros/                    # ROS 接口层
│       ├── param_loader.hpp             # 参数加载器
│       ├── ros_adapter.hpp              # ROS 适配器主类
│       ├── ros_converters.hpp           # ROS 消息转换
│       └── visualization_publisher.hpp  # 可视化发布器
│
├── launch/                     # 启动文件目录
│   ├── slam_mapping.launch                      # 默认建图启动
│   ├── slam_mapping_manager_backend.launch      # 管理后端启动
│   └── slam_mapping_parking_rslidar.launch      # 停车场场景启动
│
└── src/                        # 源代码目录
    ├── slam_mapping_node.cpp   # 主节点入口
    │
    ├── core/                   # 核心实现
    │   ├── keyframe_manager.cpp
    │   ├── loop_closure_backend.cpp
    │   ├── loop_closure_fastlio2_backend.cpp
    │   ├── map_saver.cpp
    │   └── mapping_runtime.cpp
    │
    └── ros/                    # ROS 接口实现
        ├── param_loader.cpp
        ├── ros_adapter.cpp
        ├── ros_converters.cpp
        └── visualization_publisher.cpp
```

### 目录结构说明

| 目录 | 说明 |
|------|------|
| `config/` | 配置文件，支持不同场景（道路/停车场） |
| `docs/` | 技术文档和操作指南 |
| `include/core/` | 核心算法：关键帧、回环检测、地图保存 |
| `include/ros/` | ROS 接口：参数加载、消息转换、可视化 |
| `launch/` | 多种场景的启动文件 |
| `src/core/` | 核心算法实现 |
| `src/ros/` | ROS 接口实现 |

---

## 运行方式

### 1. 编译

```bash
# 在 DeepLumin 工作空间根目录执行
cd /home/hhy/2004ros/DeepLumin
catkin_make --pkg slam_mapping
# 或编译整个工作空间
catkin_make
```

### 2. 启动节点

#### 方式一：使用 Launch 文件（推荐）

```bash
# 默认道路场景
roslaunch slam_mapping slam_mapping.launch

# 停车场场景（使用 Robosense 激光雷达）
roslaunch slam_mapping slam_mapping_parking_rslidar.launch
```

#### 方式二：直接运行节点

```bash
# 加载参数后运行
rosparam load /home/hhy/2004ros/DeepLumin/src/localization/slam_mapping/config/slam_mapping.yaml
rosrun slam_mapping slam_mapping_node
```

### 3. 运行依赖

确保以下话题有数据发布：
- `/ouster/imu` - IMU 数据
- `/ouster/points` - 激光雷达点云
- `/can_receive_info` - 轮速计数据（可选）
- `/gnss_chc_data` - GNSS 姿态数据（可选）

---

## 输入数据

### 1. IMU 数据

**话题名称**: `/ouster/imu` (可配置)

**消息类型**: `sensor_msgs/Imu`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `orientation` | 姿态四元数 |
| `angular_velocity` | 角速度（rad/s） |
| `linear_acceleration` | 线加速度（m/s²） |
| `header.stamp` | 时间戳 |

**数据要求**: 频率建议 ≥ 200Hz

---

### 2. 激光雷达点云

**话题名称**: `/ouster/points` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `x, y, z` | 三维坐标（米） |
| `intensity` | 反射强度 |
| `t/time/timestamp` | 点时间戳（可选） |
| `ring` | 扫描线编号 |

**数据要求**: 10Hz 或 20Hz，支持 64/128 线激光雷达

---

### 3. 轮速计数据

**话题名称**: `/can_receive_info` (可配置)

**消息类型**: `deeplumin_msgs/CanReceiveInfo`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `speed` | 车速（km/h） |
| `gear` | 档位 |
| `valid` | 数据有效性 |

**使用条件**: `slam/faster_lio/use_vel: true`

---

### 4. GNSS 姿态数据

**话题名称**: `/gnss_chc_data` (可配置)

**消息类型**: `deeplumin_msgs/Gpchc`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `heading` | 航向角（度） |
| `pitch` | 俯仰角（度） |
| `roll` | 横滚角（度） |

**使用条件**: `slam/faster_lio/use_gnss_attitude: true`

---

## 输出数据

### 1. 里程计输出

**话题名称**: `/slam_mapping/odom` (可配置)

**消息类型**: `nav_msgs/Odometry`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `pose.pose.position` | 位置（x, y, z） |
| `pose.pose.orientation` | 姿态四元数 |
| `pose.covariance` | 位姿协方差 |

**发布频率**: 与激光雷达频率一致

---

### 2. 轨迹路径

**话题名称**: `/slam_mapping/path` (可配置)

**消息类型**: `nav_msgs/Path`

**内容说明**:
- 历史位姿轨迹，用于可视化
- 包含所有关键帧位姿

---

### 3. 关键帧标记

**话题名称**: `/slam_mapping/keyframes` (可配置)

**消息类型**: `visualization_msgs/MarkerArray`

**内容说明**:
- 关键帧位置和姿态的可视化标记
- 可用于观察关键帧分布

---

### 4. 局部地图

**话题名称**: `/slam_mapping/local_map` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
- 当前局部子地图点云
- 基于 iVox 数据结构

---

### 5. 当前扫描

**话题名称**: `/slam_mapping/current_scan` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
- 当前帧配准后的点云

---

### 6. 地图预览

**话题名称**: `/slam_mapping/map_preview` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
- 累积地图预览（降采样后）
- 用于实时观察建图效果

**发布频率**: 可配置（默认 2 秒一次）

---

### 7. 回环检测可视化

**话题名称**: 
- `/slam_mapping/loop_markers` - 回环连接标记
- `/slam_mapping/loop_current_submap` - 当前子地图
- `/slam_mapping/loop_history_submap` - 历史子地图

**消息类型**: 
- `visualization_msgs/MarkerArray`
- `sensor_msgs/PointCloud2`

**内容说明**:
- 回环检测的可视化结果
- 显示回环连接线和配准子地图

---

### 8. 状态信息

**话题名称**: `/slam_mapping/status` (可配置)

**消息类型**: 自定义状态消息

**内容说明**:
- 建图状态（运行/停止）
- 关键帧数量
- 回环检测数量
- 地图保存路径

---

### 9. TF 变换

**发布内容**:
- `map` → `odom` → `base_link`

**发布条件**: `mapping/publish_tf: true`

---

## 数据处理方式

### 1. 关键帧选取策略

关键帧选取基于以下阈值（可配置）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `distance_thresh` | 2.0 m | 位置变化阈值 |
| `rotation_thresh_rad` | 5.0 deg | 旋转变化阈值 |
| `time_thresh_sec` | 0.5 s | 时间间隔阈值 |
| `min_points` | 50 | 最小点云数量 |
| `min_frame_gap` | 10 | 最小帧间隔 |

**选取逻辑**：
```
当满足以下任一条件时，创建新的关键帧：
1. 与上一关键帧距离 > distance_thresh
2. 与上一关键帧旋转 > rotation_thresh_rad
3. 与上一关键帧时间差 > time_thresh_sec
4. 且帧间隔 >= min_frame_gap
5. 且点云数量 >= min_points
```

---

### 2. 回环检测流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                        回环检测流程                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  输入: 新关键帧                                                      │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 1: 候选关键帧搜索                                       │   │
│  │                                                             │   │
│  │  - 在空间半径 search_radius 内搜索历史关键帧               │   │
│  │  - 排除时间/索引距离过近的关键帧                           │   │
│  │  - 选择最近的历史关键帧作为候选                            │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 2: 子地图构建                                           │   │
│  │                                                             │   │
│  │  当前子地图:                                                │   │
│  │  - 以当前关键帧为中心                                       │   │
│  │  - 聚合周围 submap_search_num 个关键帧的点云               │   │
│  │  - 降采样到 submap_resolution                               │   │
│  │                                                             │   │
│  │  历史子地图:                                                │   │
│  │  - 以候选关键帧为中心                                       │   │
│  │  - 同样方式构建历史子地图                                   │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 3: 子地图配准 (ICP)                                     │   │
│  │                                                             │   │
│  │  - 使用 ICP 配准当前子地图和历史子地图                     │   │
│  │  - 计算 fitness score                                       │   │
│  │  - 如果 fitness < icp_fitness_thresh，回环检测成功         │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 4: 位姿图优化 (GTSAM)                                   │   │
│  │                                                             │   │
│  │  - 添加关键帧位姿作为节点                                   │   │
│  │  - 添加里程计边（连续关键帧之间）                           │   │
│  │  - 添加回环边（检测到回环的关键帧对）                       │   │
│  │  - 执行 Levenberg-Marquardt 优化                           │   │
│  │  - 更新所有关键帧位姿                                       │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  输出: 优化后的全局一致轨迹                                          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

### 3. 坐标变换链

```
map (全局地图坐标系，经过回环优化)
  │
  │  (回环优化修正)
  ▼
odom (里程计坐标系，局部一致)
  │
  │  T_odom_base (Faster-LIO 输出)
  ▼
base_link (车辆本体坐标系)
  │
  ├── T_base_imu ──► imu_link
  │
  └── T_base_lidar ──► lidar_link
```

---

### 4. 地图保存

**保存内容**:

| 文件 | 说明 |
|------|------|
| `map.pcd` | 全局点云地图 |
| `trajectory.csv` | 轨迹文件（时间戳、位姿） |
| `keyframes.csv` | 关键帧位姿 |
| `keyframes/` | 关键帧点云文件夹 |
| `loop_pairs.csv` | 回环对信息 |
| `loop_results/` | 回环检测结果 |

**保存触发条件**:
1. 调用服务手动保存
2. 程序正常退出时自动保存（`save_on_stop: true`）
3. 建图任务停止时保存

---

## 算法执行流程

### 主循环流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                         主循环 (5000Hz)                              │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │ 1. ros::spinOnce() - 处理传感器回调和服务调用                  │ │
│  │    - IMU 回调: 缓存 IMU 数据，更新预热状态                     │ │
│  │    - 点云回调: 缓存点云（预热完成后）                          │ │
│  │    - CAN 回调: 缓存轮速数据                                    │ │
│  │    - GNSS 回调: 缓存 GNSS 姿态                                 │ │
│  │    - 服务调用: start/stop/save 等控制命令                      │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                              │                                      │
│                              ▼                                      │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │ 2. node.spinOnce() - 建图处理                                  │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ a. 检查运行状态                                       │  │ │
│  │    │    - 如果未启动，跳过处理                             │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  │                              │                                 │ │
│  │                              ▼                                 │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ b. Faster-LIO 处理                                    │  │ │
│  │    │    - IMU 积分预测                                     │  │ │
│  │    │    - 点云配准更新                                     │  │ │
│  │    │    - 输出当前位姿和局部地图                           │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  │                              │                                 │ │
│  │                              ▼                                 │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ c. 关键帧管理                                         │  │ │
│  │    │    - 检查是否满足关键帧条件                           │  │ │
│  │    │    - 如果是，创建新关键帧                             │  │ │
│  │    │    - 添加到关键帧数据库                               │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  │                              │                                 │ │
│  │                              ▼                                 │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ d. 回环检测（如果启用）                               │  │ │
│  │    │    - 搜索候选关键帧                                   │  │ │
│  │    │    - 构建子地图                                       │  │ │
│  │    │    - ICP 配准                                         │  │ │
│  │    │    - 如果成功，执行位姿图优化                         │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  │                              │                                 │ │
│  │                              ▼                                 │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ e. 可视化发布                                         │  │ │
│  │    │    - 发布里程计、路径、关键帧                         │  │ │
│  │    │    - 发布局部地图和当前扫描                           │  │ │
│  │    │    - 发布地图预览                                     │  │ │
│  │    │    - 发布回环检测结果                                 │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                              │                                      │
│                              ▼                                      │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │ 3. rate.sleep() - 维持循环频率                                 │ │
│  └───────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

### 关键帧管理流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                        关键帧管理流程                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  输入: 当前位姿 + 当前扫描 + 局部地图                                │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 1: 检查关键帧条件                                       │   │
│  │                                                             │   │
│  │  计算与上一关键帧的差异：                                   │   │
│  │  - 距离差: delta_pos = |current_pos - last_keyframe_pos|   │   │
│  │  - 角度差: delta_rot = angle_between(current_rot, last_rot)│   │
│  │  - 时间差: delta_time = current_time - last_keyframe_time  │   │
│  │  - 帧间隔: frame_gap = current_frame_id - last_frame_id    │   │
│  │                                                             │   │
│  │  条件检查：                                                 │   │
│  │  - delta_pos > distance_thresh ?                           │   │
│  │  - delta_rot > rotation_thresh_rad ?                       │   │
│  │  - delta_time > time_thresh_sec ?                          │   │
│  │  - frame_gap >= min_frame_gap ?                            │   │
│  │  - scan.points.size() >= min_points ?                      │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 2: 创建关键帧                                           │   │
│  │                                                             │   │
│  │  关键帧数据结构：                                           │   │
│  │  - id: 关键帧唯一标识                                       │   │
│  │  - timestamp: 时间戳                                        │   │
│  │  - pose: 位姿（位置 + 姿态）                                │   │
│  │  - pointcloud: 点云数据                                     │   │
│  │  - local_map: 局部子地图                                    │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 3: 添加到关键帧数据库                                   │   │
│  │                                                             │   │
│  │  - 插入到关键帧列表                                         │   │
│  │  - 更新空间索引（用于快速近邻搜索）                         │   │
│  │  - 触发回环检测（如果启用）                                 │   │
│  │                                                             │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  输出: 更新后的关键帧数据库                                          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 配置文件说明

主要配置文件: `config/slam_mapping.yaml`

### 关键参数说明

#### 1. 建图任务配置 (`mapping`)

```yaml
mapping:
  auto_start: false              # 是否自动启动建图
  task_name: "manual_mapping"    # 任务名称
  output_root: "/home/hhy/2004ros/deeplumin_mapping"  # 输出根目录
  publish_tf: true               # 是否发布 TF
  process_rate_hz: 5000.0        # 处理频率
```

#### 2. 坐标系配置 (`frames`)

```yaml
frames:
  map: "map"              # 全局地图坐标系
  odom: "odom"            # 里程计坐标系
  base_link: "base_link"  # 车辆本体坐标系
  lidar: "lidar_link"     # 激光雷达坐标系
  imu: "imu_link"         # IMU 坐标系
```

#### 3. 关键帧配置 (`keyframe`)

```yaml
keyframe:
  distance_thresh: 2.0           # 距离阈值（米）
  rotation_thresh_rad: 5.0       # 旋转阈值（度）
  time_thresh_sec: 0.5           # 时间阈值（秒）
  min_points: 50                 # 最小点云数量
  min_frame_gap: 10              # 最小帧间隔
```

#### 4. 回环检测配置 (`loop_closure`)

```yaml
loop_closure:
  enable: true                   # 是否启用回环检测
  type: "fastlio2_sam_lc"        # 回环检测类型
  search_radius: 15.0            # 搜索半径（米）
  min_index_gap: 20              # 最小索引间隔
  time_thresh_sec: 30.0          # 时间阈值（秒）
  submap_search_num: 25          # 子地图关键帧数量
  submap_resolution: 0.3         # 子地图分辨率（米）
  icp_fitness_thresh: 0.3        # ICP fitness 阈值
  pose_graph_enable: true        # 是否启用位姿图优化
```

#### 5. 地图保存配置 (`map_saver`)

```yaml
map_saver:
  default_resolution: 0.2        # 默认地图分辨率
  save_on_stop: true             # 停止时自动保存
  save_keyframes: true           # 保存关键帧
  save_keyframe_poses: true      # 保存关键帧位姿
  save_trajectory: true          # 保存轨迹
  map_filename: "map.pcd"        # 地图文件名
  trajectory_filename: "trajectory.csv"  # 轨迹文件名
```

#### 6. 可视化配置 (`visualization`)

```yaml
visualization:
  publish_path: true             # 发布轨迹
  publish_keyframes: true        # 发布关键帧
  publish_local_map: true        # 发布局部地图
  publish_current_scan: true     # 发布当前扫描
  publish_accumulated_map_preview: true  # 发布地图预览
  publish_loop_markers: true     # 发布回环标记
  accumulated_map_publish_period: 2.0    # 地图预览发布周期
  accumulated_map_voxel_leaf: 0.5        # 地图预览体素大小
```

---

## 服务接口

### 1. 启动建图

**服务名**: `/slam_mapping/start`

**请求参数**:
| 字段 | 类型 | 说明 |
|------|------|------|
| `task_name` | string | 任务名称 |
| `output_dir` | string | 输出目录（可选） |

**响应**:
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否成功 |
| `message` | string | 状态信息 |

---

### 2. 停止建图

**服务名**: `/slam_mapping/stop`

**请求参数**:
| 字段 | 类型 | 说明 |
|------|------|------|
| `save_map` | bool | 是否保存地图 |

**响应**:
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否成功 |
| `message` | string | 状态信息 |

---

### 3. 保存地图

**服务名**: `/slam_mapping/save`

**请求参数**:
| 字段 | 类型 | 说明 |
|------|------|------|
| `output_dir` | string | 输出目录（可选，默认使用配置） |

**响应**:
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否成功 |
| `map_path` | string | 地图保存路径 |

---

## 调试与可视化

### RViz 配置

建议添加以下显示项：

| 显示项 | 话题 | 说明 |
|--------|------|------|
| Odometry | `/slam_mapping/odom` | 当前位姿 |
| Path | `/slam_mapping/path` | 轨迹 |
| MarkerArray | `/slam_mapping/keyframes` | 关键帧标记 |
| PointCloud2 | `/slam_mapping/local_map` | 局部地图 |
| PointCloud2 | `/slam_mapping/current_scan` | 当前扫描 |
| PointCloud2 | `/slam_mapping/map_preview` | 地图预览 |
| MarkerArray | `/slam_mapping/loop_markers` | 回环标记 |
| TF | - | 坐标系变换 |

### 日志输出

关键日志信息：

```
# 建图启动
[slam_mapping] Mapping started: task_name=manual_mapping output=/home/hhy/2004ros/deeplumin_mapping/manual_mapping_20240101_120000

# 关键帧创建
[slam_mapping] Keyframe added: id=10 pose=(x: 12.34, y: 56.78, z: 0.12) total_keyframes=11

# 回环检测
[slam_mapping] Loop closure detected: current=25 candidate=5 fitness=0.15
[slam_mapping] Pose graph optimization: iterations=10 error_before=2.34 error_after=0.12

# 地图保存
[slam_mapping] Map saved: path=/home/hhy/2004ros/deeplumin_mapping/manual_mapping_20240101_120000/map.pcd size=1234567 points
```

---

## 常见问题

### Q1: 建图启动后没有输出

**现象**: 启动后没有里程计输出，RViz 看不到轨迹

**可能原因**:
1. 未调用 start 服务（`auto_start: false` 时）
2. IMU 预热未完成
3. 传感器数据未到达

**解决**:
```bash
# 手动启动建图
rosservice call /slam_mapping/start "task_name: 'my_mapping'"
```

---

### Q2: 地图漂移严重

**可能原因**:
1. 场景特征不足（长走廊、隧道）
2. 回环检测未触发
3. IMU 噪声参数不当

**解决**:
1. 调整回环检测参数，降低阈值
2. 增加 `search_radius` 和 `submap_search_num`
3. 调整 IMU 协方差参数

---

### Q3: 关键帧过于密集或稀疏

**解决**: 调整关键帧阈值
```yaml
keyframe:
  distance_thresh: 1.0      # 减小以增加密度
  rotation_thresh_rad: 3.0  # 减小以增加密度
  time_thresh_sec: 0.3      # 减小以增加密度
```

---

### Q4: 回环检测失败

**排查步骤**:
1. 检查 `loop_markers` 话题是否有输出
2. 查看 ICP fitness score 是否过高
3. 调整 `icp_fitness_thresh` 参数

---

## 参考资料

1. **Faster-LIO 论文**: 
   - "Faster-LIO: Lightweight Tightly Coupled LiDAR-Inertial Odometry"

2. **GTSAM 库**:
   - 用于位姿图优化的因子图库

3. **PCL 库**:
   - 点云处理库，用于 ICP 配准

---

## 维护信息

- **作者**: DeepLumin Team
- **版本**: 1.0.0
- **最后更新**: 2024

如有问题，请联系维护团队或提交 Issue。