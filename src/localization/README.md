# DeepLumin Localization 空间文档

## 1. 概述

本目录包含 DeepLumin 自动驾驶系统的定位与建图模块，采用分层架构设计：

- **前端里程计**：提供高频局部位姿估计（DR + SLAM）
- **后端建图**：关键帧管理、回环检测、全局优化
- **重定位**：基于全局描述子的位置识别与位姿估计
- **融合层**：多源里程计融合，输出全局一致的定位结果

## 2. 坐标系约定

| 坐标系 | 说明 | 原点 |
|--------|------|------|
| `map` | 全局地图坐标系 | 建图起点或 GNSS 原点 |
| `odom` | 里程计坐标系 | 启动时刻车体位置 |
| `base_link` | 车体坐标系 | 车辆几何中心 |
| `lidar_link` | 激光雷达坐标系 | LiDAR 光学中心 |
| `imu_link` | IMU 坐标系 | IMU 安装位置 |

**变换约定**：`T_base_sensor` 表示将 sensor 坐标系中的点/向量变换到 base_link 坐标系。

---

## 3. dr_odometry（航位推算）

### 3.1 功能
基于 ESKF（Error State Kalman Filter）融合 IMU、轮速计、GNSS 数据，输出高频（100Hz）局部里程计。

### 3.2 输入结构体

#### `ImuData`（IMU 采样）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳（秒） |
| `gyro` | `Vec3d` | 角速度 ω (rad/s)，body 系 |
| `accel` | `Vec3d` | 线加速度 a (m/s²)，含重力比力 |

#### `CanData`（底盘/CAN 车速）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳 |
| `speed_mps` | `double` | 带符号车速（m/s，倒车为负） |
| `gear` | `double` | 档位信息 |
| `valid` | `bool` | 数据有效性 |

#### `GnssData`（GNSS/组合导航）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳 |
| `latitude` | `double` | 纬度（度） |
| `longitude` | `double` | 经度（度） |
| `altitude` | `double` | 海拔（米） |
| `heading_rad` | `double` | ENU yaw（0=东，逆时针为正） |
| `pitch_rad` | `double` | 俯仰角（rad） |
| `roll_rad` | `double` | 横滚角（rad） |
| `velocity_enu` | `Vec3d` | ENU 速度 (ve, vn, vu) |
| `speed_mps` | `double` | 地速（m/s） |
| `heading_valid` | `bool` | 航向有效标志 |
| `position_valid` | `bool` | 位置有效标志 |
| `velocity_valid` | `bool` | 速度有效标志 |

### 3.3 输出结构体

#### `OdomResult`（对外里程计结果）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳 |
| `position` | `Vec3d` | odom 系位置（m） |
| `velocity` | `Vec3d` | odom 系速度（m/s） |
| `orientation` | `Quatd` | body→odom 四元数 |
| `covariance` | `Matrix6d` | 6×6 位姿协方差（位置3 + 姿态3） |
| `valid` | `bool` | 输出有效性 |
| `frame_id` | `string` | 父坐标系（默认"odom"） |

#### `DrState`（ESKF 标称状态）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳 |
| `position` | `Vec3d` | 世界系位置 |
| `velocity` | `Vec3d` | 世界系速度 |
| `orientation` | `Quatd` | body→world 四元数 |
| `gyro_bias` | `Vec3d` | 陀螺零偏（rad/s） |
| `accel_bias` | `Vec3d` | 加计零偏（m/s²） |
| `covariance` | `Mat15d` | 15×15 误差状态协方差 |
| `initialized` | `bool` | 滤波器初始化标志 |

**误差状态 15 维布局**：`[0:3) δp` | `[3:6) δv` | `[6:9) δθ` | `[9:12) δbg` | `[12:15) δba`

### 3.4 ROS 输入/输出话题

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 输入 | `/ouster/imu` | `sensor_msgs/Imu` | IMU 数据 |
| 输入 | `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` | CAN 车速 |
| 输入 | `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | GNSS 组合导航 |
| 输出 | `/localization/dr_odom` | `nav_msgs/Odometry` | DR 里程计 |
| 输出 | `/localization/dr_status` | `deeplumin_msgs/LocalizationStatus` | 健康状态 |
| 输出 | `/localization/dr_path` | `nav_msgs/Path` | 轨迹可视化 |

---

## 4. slam_odometry（SLAM 里程计）

### 4.1 功能
基于 Faster-LIO 的激光雷达惯性里程计，融合 IMU、LiDAR、轮速、GNSS 姿态，输出中频（10Hz）局部里程计。

### 4.2 输入结构体

#### `ImuData`（IMU 采样）
同 dr_odometry，但消费路径：`feedImu` → Faster-LIO 的 IMU 处理管线

#### `TimestampedPointCloud`（带时间戳点云）
| 字段 | 类型 | 说明 |
|------|------|------|
| `cloud` | `PointCloudPtr` | 点云数据（PointXYZINormal） |
| `timestamp` | `double` | 帧时间戳 |
| `frame_id` | `string` | 坐标系 |

**注意**：`curvature` 字段存储点时间（ms），用于去畸变。

#### `CanVelocityData`（轮速）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳 |
| `speed_kmh` | `double` | 车速（km/h，倒车为负） |
| `valid` | `bool` | 有效性 |

#### `GnssAttitudeData`（GNSS 姿态）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳 |
| `heading_deg` | `double` | 航向角（度） |
| `pitch_deg` | `double` | 俯仰角（度） |
| `roll_deg` | `double` | 横滚角（度） |
| `valid` | `bool` | 有效性 |

### 4.3 输出结构体

#### `OdomResult`（统一里程计输出）
| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `double` | 时间戳 |
| `position` | `Vec3d` | odom 系位置 |
| `orientation` | `Quatd` | body→odom 四元数 |
| `velocity` | `Vec3d` | odom 系速度 |
| `covariance` | `Mat6d` | 6×6 位姿协方差 |
| `valid` | `bool` | 有效性 |
| `is_degenerate` | `bool` | 退化标志 |
| `degeneracy_factor` | `double` | 退化因子 |

#### `TimestampedPointCloud`（局部地图/当前扫描）
通过 `getLocalMap()` 和 `getCurrentScan()` 获取，用于可视化。

### 4.4 ROS 输入/输出话题

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 输入 | `/ouster/imu` | `sensor_msgs/Imu` | IMU 数据 |
| 输入 | `/ouster/points` | `sensor_msgs/PointCloud2` | 激光点云 |
| 输入 | `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` | CAN 轮速 |
| 输入 | `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | GNSS 姿态 |
| 输出 | `/localization/slam_odom` | `nav_msgs/Odometry` | SLAM 里程计 |
| 输出 | `/localization/local_map` | `sensor_msgs/PointCloud2` | 局部地图 |
| 输出 | `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 配准后点云 |
| 输出 | `/localization/global_map` | `sensor_msgs/PointCloud2` | 全局地图（可视化） |

---

## 5. slam_mapping（建图与回环）

### 5.1 功能
基于 slam_odometry 的前端输出，进行关键帧管理、回环检测、位姿图优化，构建全局一致地图。

### 5.2 输入结构体

#### `MappingKeyframe`（关键帧）
| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | `int` | 关键帧 ID |
| `timestamp` | `double` | 时间戳 |
| `pose_odom` | `Pose` | odom 系位姿（前端输出） |
| `pose_map` | `Pose` | map 系位姿（优化后） |
| `cloud_body` | `PointCloudPtr` | body 系点云 |
| `point_count` | `size_t` | 点数 |
| `optimized` | `bool` | 是否已优化 |
| `loop_closed` | `bool` | 是否参与回环 |

#### `RuntimeStartRequest`（启动请求）
| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `task_name` | `string` | 任务名称 |
| `output_dir` | `string` | 输出目录 |
| `enable_loop_closure` | `bool` | 是否启用回环 |
| `enable_visualization` | `bool` | 是否启用可视化 |

### 5.3 输出结构体

#### `MappingRuntimeStatus`（运行时状态）
| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | `string` | 任务 ID |
| `state` | `RuntimeState` | 运行状态 |
| `frame_count` | `uint64` | 处理帧数 |
| `keyframe_count` | `uint64` | 关键帧数 |
| `loop_count` | `uint64` | 回环数 |
| `output_dir` | `string` | 输出目录 |
| `map_path` | `string` | 地图文件路径 |

#### `LoopClosureResult`（回环结果）
| 字段 | 类型 | 说明 |
|------|------|------|
| `updated` | `bool` | 是否触发优化 |
| `loop_count` | `int` | 累计回环数 |
| `latest_score` | `double` | 最新回环得分 |
| `loop_pairs` | `vector<pair<int,int>>` | 回环对列表 |

### 5.4 ROS 输入/输出话题与服务

| 方向 | 话题/服务 | 消息类型 | 说明 |
|------|-----------|----------|------|
| 输入 | `/ouster/imu` | `sensor_msgs/Imu` | IMU 数据 |
| 输入 | `/ouster/points` | `sensor_msgs/PointCloud2` | 激光点云 |
| 输入 | `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` | CAN 数据 |
| 输入 | `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | GNSS 数据 |
| 输出 | `/slam_mapping/odom` | `nav_msgs/Odometry` | 建图里程计 |
| 输出 | `/slam_mapping/path` | `nav_msgs/Path` | 轨迹 |
| 输出 | `/slam_mapping/local_map` | `sensor_msgs/PointCloud2` | 局部地图 |
| 输出 | `/slam_mapping/current_scan` | `sensor_msgs/PointCloud2` | 当前扫描 |
| 输出 | `/slam_mapping/keyframes` | `visualization_msgs/MarkerArray` | 关键帧标记 |
| 服务 | `/slam_mapping/start` | `deeplumin_msgs/StartMappingRuntime` | 启动建图 |
| 服务 | `/slam_mapping/stop` | `deeplumin_msgs/StopMappingRuntime` | 停止建图 |
| 服务 | `/slam_mapping/save` | `deeplumin_msgs/SaveMappingRuntime` | 保存地图 |
| 服务 | `/slam_mapping/reset` | `std_srvs/Trigger` | 重置 |
| 服务 | `/slam_mapping/get_status` | `deeplumin_msgs/GetMappingRuntimeStatus` | 获取状态 |

---

## 6. relocalization（重定位）

### 6.1 功能
基于全局描述子（Scan Context / CSC）进行位置识别，通过 GICP/VGICP 精配准，输出 map→base_link 绝对位姿。

### 6.2 输入结构体

#### `CloudConstPtr`（查询点云）
`pcl::PointCloud<pcl::PointXYZI>::ConstPtr`，来自当前帧激光雷达扫描。

#### `DescriptorCandidate`（描述子候选）
| 字段 | 类型 | 说明 |
|------|------|------|
| `keyframe_id` | `int` | 候选关键帧 ID |
| `score` | `double` | 相似度得分（越小越相似） |
| `ring_dist` | `double` | RingKey 距离 |
| `sector_shift` | `int` | 最佳扇区偏移 |
| `yaw_diff` | `double` | 估计的 yaw 差异（rad） |
| `pose` | `Pose` | 候选位姿 |

### 6.3 输出结构体

#### `MatchResult`（匹配结果）
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 整体成功标志 |
| `converged` | `bool` | GICP 收敛标志 |
| `candidate_id` | `int` | 最佳候选 ID |
| `fitness` | `double` | 配准 fitness 得分 |
| `inlier_ratio` | `double` | 内点比例 |
| `inlier_rmse` | `double` | 内点 RMSE |
| `transform` | `Matrix4d` | source→target 变换矩阵 |
| `aligned` | `CloudPtr` | 对齐后的点云 |
| `elapsed_ms` | `double` | 耗时（ms） |

#### `RelocalizationQueryResult`（查询结果）
| 字段 | 类型 | 说明 |
|------|------|------|
| `candidates` | `vector<DescriptorCandidate>` | 候选列表 |
| `match` | `MatchResult` | 最佳匹配结果 |

### 6.4 ROS 输入/输出话题

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 输入 | `/localization/points` | `sensor_msgs/PointCloud2` | 查询点云 |
| 输出 | `/localization/relocalization/result` | `geometry_msgs/PoseStamped` | 重定位位姿 |
| 输出 | `/offline_eval/global_map` | `sensor_msgs/PointCloud2` | 全局地图（可视化） |
| 输出 | `/offline_eval/query_raw` | `sensor_msgs/PointCloud2` | 原始查询点云 |
| 输出 | `/offline_eval/query_reloc_aligned` | `sensor_msgs/PointCloud2` | 对齐后点云 |
| 输出 | `/offline_eval/pose_markers` | `visualization_msgs/MarkerArray` | 位姿标记 |

---

## 7. simple_odom_fusion（里程计融合）

### 7.1 功能
融合 DR 里程计（高频、短期精确）和 SLAM 里程计（中频、局部一致），结合全局定位修正，输出全局一致的 map→base_link 位姿。

### 7.2 输入结构体

#### `OdomSample`（里程计样本）
| 字段 | 类型 | 说明 |
|------|------|------|
| `stamp` | `double` | 时间戳 |
| `pose` | `Pose3d` | 位姿（位置 + 四元数） |
| `linear_velocity` | `Vec3d` | 线速度 |
| `frame_id` | `string` | 父坐标系 |
| `child_frame_id` | `string` | 子坐标系 |

#### `GlobalObservation`（全局定位观测）
| 字段 | 类型 | 说明 |
|------|------|------|
| `stamp` | `double` | 时间戳 |
| `map_base` | `Pose3d` | map→base_link 位姿 |
| `success` | `bool` | 观测有效性 |

### 7.3 输出结构体

#### `FusionOutput`（融合输出）
| 字段 | 类型 | 说明 |
|------|------|------|
| `valid` | `bool` | 输出有效性 |
| `stamp` | `double` | 时间戳 |
| `odom_base` | `Pose3d` | odom→base_link（局部里程计） |
| `map_odom` | `Pose3d` | map→odom（全局修正量） |
| `map_base` | `Pose3d` | map→base_link（融合结果） |
| `velocity_map` | `Vec3d` | map 系下的线速度 |

#### `FusionState`（融合状态）
| 字段 | 类型 | 说明 |
|------|------|------|
| `initialized` | `bool` | 是否已初始化 |
| `stamp` | `double` | 最新时间戳 |
| `odom_base` | `Pose3d` | 当前 odom→base_link |
| `map_odom` | `Pose3d` | 当前 map→odom |
| `velocity_odom` | `Vec3d` | odom 系速度 |

### 7.4 ROS 输入/输出话题

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 输入 | `/localization/dr_odom` | `nav_msgs/Odometry` | DR 里程计 |
| 输入 | `/localization/slam_odom` | `nav_msgs/Odometry` | SLAM 里程计 |
| 输入 | `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | 全局定位结果 |
| 输入 | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始位姿（RViz） |
| 输入 | `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 源点云 |
| 输出 | `/localization/simple_fused_odom` | `nav_msgs/Odometry` | 融合里程计 |
| 输出 | `/localization/simple_fused_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 融合位姿 |
| 输出 | `/localization/simple_fused_path` | `nav_msgs/Path` | 融合轨迹 |
| 输出 | `/localization/simple_fusion/gm_source_cloud_odom` | `sensor_msgs/PointCloud2` | 转发点云 |

---

## 8. 节点间数据流关系

### 8.1 定位模式数据流

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   传感器驱动     │     │   传感器驱动     │     │   传感器驱动     │
│  /ouster/imu    │     │ /ouster/points  │     │/can_receive_info│
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  dr_odometry    │     │ slam_odometry   │     │ slam_odometry   │
│  (ESKF 100Hz)   │     │ (Faster-LIO)    │     │ (速度辅助)      │
│                 │     │                 │     │                 │
│ 输出:           │     │ 输出:           │     │                 │
│ /localization/  │     │ /localization/  │     │                 │
│   dr_odom       │     │   slam_odom     │     │                 │
│ /localization/  │     │ /localization/  │     │                 │
│   cloud_registered│   │   local_map     │     │                 │
└────────┬────────┘     └────────┬────────┘     └─────────────────┘
         │                       │
         │                       │
         ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    simple_odom_fusion                            │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  融合策略:                                                 │  │
│  │  - DR 预测: 高频局部增量 (100Hz)                           │  │
│  │  - SLAM 修正: 中频局部一致性 (10Hz)                        │  │
│  │  - 全局定位修正: 低频消除漂移 (1Hz)                        │  │
│  └───────────────────────────────────────────────────────────┘  │
│  输出:                                                           │
│  - /localization/simple_fused_odom (map→base_link)              │
│  - /localization/simple_fused_pose                              │
│  - /localization/simple_fused_path                              │
│  - TF: map→odom→base_link                                       │
└─────────────────────────────────────────────────────────────────┘
         ▲
         │
         │  /localization/global_matcher/result
         │
┌────────┴────────┐
│  relocalization │
│ (ScanContext/   │
│  CSC + GICP/    │
│  VGICP)         │
│                 │
│ 输入:           │
│ /localization/  │
│   points        │
│ 输出:           │
│ map→base_link   │
│ 绝对位姿        │
└─────────────────┘
```

### 8.2 建图模式数据流

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  /ouster/imu    │     │ /ouster/points  │     │/can_receive_info│
│  /gnss_chc_data │     │                 │     │                 │
└────────┬────────┘     └────────┬────────┘     └────────┬────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    slam_mapping                                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  内部流程:                                                 │  │
│  │  1. Faster-LIO 前端 → odom 系位姿                         │  │
│  │  2. 关键帧提取（距离/旋转/时间阈值）                       │  │
│  │  3. 回环检测（Fast-LIO2-SAM-LC）                          │  │
│  │  4. ISAM2 位姿图优化 → map 系位姿                         │  │
│  │  5. 地图保存（PCD/CSV/关键帧）                            │  │
│  └───────────────────────────────────────────────────────────┘  │
│  输出:                                                           │
│  - /slam_mapping/odom                                           │
│  - /slam_mapping/path                                           │
│  - /slam_mapping/local_map                                      │
│  - /slam_mapping/current_scan                                   │
│  - 文件: map.pcd, trajectory.csv, keyframes/                    │
└─────────────────────────────────────────────────────────────────┘
```

### 8.3 重定位模式数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                    relocalization 节点                           │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  离线评估流程:                                             │  │
│  │  1. 加载关键帧数据库 + 全局描述子库                        │  │
│  │  2. 加载全局地图 PCD                                       │  │
│  │  3. 对每帧查询点云:                                        │  │
│  │     a. 生成描述子 (CSC/ScanContext)                       │  │
│  │     b. RingKey 粗筛 → Top-K 候选                          │  │
│  │     c. GICP/VGICP 精配准                                  │  │
│  │     d. 输出 map→base_link 位姿                            │  │
│  │  4. 与真值对比 / 地图贴合评估                              │  │
│  └───────────────────────────────────────────────────────────┘  │
│  输入: /localization/points (或 bag 回放)                        │
│  输出: /localization/relocalization/result                       │
│  可视化: /offline_eval/* (RViz)                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 8.4 完整定位闭环数据流

```
┌──────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│   原始传感器  │   │ sensor_frame │   │  slam_       │   │   simple_    │
│   数据       │──▶│ _adapter     │──▶│  odometry    │──▶│  odom_fusion │──▶ 最终定位
│              │   │ (坐标系转换)  │   │  (前端里程计) │   │  (融合输出)  │
└──────────────┘   └──────────────┘   └──────┬───────┘   └──────┬───────┘
                                             │                  ▲
                                             │                  │
                                             ▼                  │
                                        ┌──────────────┐        │
                                        │  relocalization│       │
                                        │  (全局定位)    │───────┘
                                        │              │
                                        │ 输入: 点云    │
                                        │ 输出: map→base│
                                        └──────────────┘
                                             ▲
                                             │
                                        ┌──────────────┐
                                        │  slam_mapping │
                                        │  (离线建图)   │
                                        │  输出: 地图   │
                                        │  关键帧位姿   │
                                        └──────────────┘
```

---

## 9. 配置文件索引

| 模块 | 配置文件 | 说明 |
|------|----------|------|
| dr_odometry | `dr_odometry/config/dr_odometry.yaml` | ESKF 参数、话题名 |
| slam_odometry | `slam_odometry/config/slam_odometry_ysw.yaml` | Faster-LIO 参数 |
| slam_mapping | `slam_mapping/config/slam_mapping.yaml` | 建图、回环、保存参数 |
| relocalization | `relocalization/config/relocalization.yaml` | 描述子、匹配器参数 |
| simple_odom_fusion | `simple_odom_fusion/config/simple_odom_fusion.yaml` | 融合策略参数 |

---

## 10. 关键设计决策

1. **ESKF 误差状态设计**：15 维误差状态（位置3、速度3、姿态3、陀螺零偏3、加计零偏3），与 Faster-LIO 的 IEKF 形成互补。

2. **两阶段检索**：relocalization 采用 RingKey 粗筛（O(n log k)）+ 描述子精排（旋转对齐），平衡效率与精度。

3. **增量融合策略**：simple_odom_fusion 支持 `use_slam_increment` 模式，使用 SLAM 增量而非绝对位姿，避免 SLAM 跳变对融合的影响。

4. **回环触发优化**：slam_mapping 的后端优化仅在检测到回环或需要处理新里程计因子时触发，非每关键帧强制优化。

5. **坐标系分离**：`odom` 用于局部里程计（高频、平滑），`map` 用于全局定位（低频、无漂移），通过 `map→odom` 修正量桥接。