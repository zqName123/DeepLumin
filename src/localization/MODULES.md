# DeepLumin 定位系统模块介绍

> 本文档介绍 `src/localization/` 下各模块的**作用、内部结构、输入输出**，并给出端到端数据流。
> 注意：目录中已有的 `README.md` 描述的是早期包（`slam_odometry` / `slam_mapping` / `simple_odom_fusion` 等，均已不在本目录中），以本文档为准。

---

## 1. 系统概述

定位系统按功能分为四层：

| 层 | 模块 | 职责 |
|---|---|---|
| **前端里程计** | `dr_odometry`、`lio_odometry*`、`slam_odometry_v2` | 高/中频局部里程计（odom→base_link） |
| **全局定位 / 重定位** | `global_matcher_v2`、`relocalization`、`aruco_relocalization` | 在环全局匹配、丢失后重定位（map→odom / map→base_link） |
| **建图** | `slam_mapping_v2`、`mapping_manager`、`mapping_gui` | 关键帧、回环、位姿图优化、地图保存、建图流程编排 |
| **融合与编排** | `localization_v2` | 融合 odom + 全局修正，状态机管理，对下游输出 |

辅助模块：`sensor_frame_adapter`（点云坐标系归一化）、`loclog`（运行日志目录）。

### 1.1 坐标系约定

| 坐标系 | 说明 |
|--------|------|
| `map` | 全局地图系（建图起点 / 地图原点） |
| `odom` | 里程计系（启动时刻车体位置，只增不跳） |
| `base_link` | 车体系 |
| `lidar_link` / `imu_link` | 传感器系 |

变换约定：`T_base_sensor` 将 sensor 系中的点/向量变换到 base_link 系。
**桥接关系**：`map→base_link = map→odom ⊕ odom→base_link`，其中 `map→odom` 由全局匹配/重定位低频修正，`odom→base_link` 由前端里程计高频维护。

### 1.2 端到端数据流

```
                     【定位模式】
/ouster/points ──► sensor_frame_adapter ──► /localization/points
/ouster/imu ─┐
/ouster/points ─┼─► lio_odometry* / slam_odometry_v2 ──► /slam_odometry/* 或 /localization/slam_odom
/can_receive_info ┤        │
/gnss_chc_data ──┘        ├─► global_matcher_v2 ──► /localization/global_matcher/result (map→odom)
                          │                              │
                          │   relocalization / aruco ────┤ /localization/relocalization/result (map→base_link)
                          │                              ▼
                          └──────────► localization_v2 ──► /localization_v2/odom (map→base_link, 100Hz)
                                       (状态机 + 融合)  ──► /localization_v2/loc_point (下游规划)
                                                        ──► TF map→odom, map→base_link
                                                        ──► /localization_v2/relocalization/command

                     【建图模式】
里程计 ──► slam_mapping_v2（关键帧/回环/ISAM2）──► map.pcd / trajectory.csv / keyframes/
              ▲                    ▲
              │ services           │
        mapping_manager ◄──services── mapping_gui（tkinter 界面）
        （+ rosbag 录制、环境检测）
```

---

## 2. 公共消息（deeplumin_msgs）

各模块间通过 `deeplumin_msgs` 通信，定位相关定义在 `src/deeplumin_msgs/msg/localization/`：

| 消息 | 用途 | 关键字段 |
|---|---|---|
| `GlobalMatchResult` | 全局匹配结果 | `pose`（PoseWithCovariance，map→odom 或 map→base）、`success`、`converged`、`map_id`、`fitness_score`、`inlier_ratio`、`inlier_rmse`、`initial_translation_error/yaw_error` |
| `RelocalizationResult` | 重定位结果 | `pose`（map→base_link）、`success`、`accepted`、`method`、`reject_reason`、`score/descriptor_score/fitness_score/inlier_ratio/inlier_rmse`、`keyframe_id`、`yaw_diff` |
| `FusionStatus` | 融合节点健康状态 | `level`(NORMAL/WARNING/DEGRADED/FAILURE)、`state`、`failure_reason`、各源 age、`used_sources/rejected_sources` |
| `LocalizationStatus` | 单模块状态 | `level`、`quality_score`、`is_gnss/slam/dr_available`、`current_mode` |
| `MappingStatus` | 建图运行状态 | `state`、`frame/keyframe/loop_count`、`output_dir/map_path/trajectory_path`、`last_loop_score` |
| `LocPoint` | 下游规划位姿 | `x/y/yaw/pitch`（**yaw/pitch 单位为度**）、`vec`（线速度） |
| `MappingRecordStatus` / `MappingEnvironmentStatus` | 录包 / 环境检测状态 | — |

传感器输入消息：`system/CanReceiveInfo`（`speed`、`gear`、`valid`…）、`system/Gpchc`（`heading/pitch/roll`、`lat/lon/alt`、`ve/vn/vu`、`nsv1/nsv2/status`…）。

定位相关服务（`srv/localization/`）：`Start/StopMappingRuntime`、`SaveMappingRuntime`、`GetMappingRuntimeStatus`、`Start/StopMapping`、`GetMappingStatus`、`CheckMappingEnvironment`、`Start/StopBagRecording`。

---

## 3. 前端里程计模块

### 3.1 dr_odometry — 航位推算（ESKF）

**作用**：15 维误差状态 ESKF 航位推算：IMU 预测 + 轮速（纵向速度 + 非完整约束 NHC）+ GNSS（航向/速度/位置观测）更新，**无 LiDAR**，输出 100Hz 高频里程计。误差状态布局：`[0:3)δp | [3:6)δv | [6:9)δθ | [9:12)δbg | [12:15)δba`。

**结构**：
- `core/types.hpp` — `ImuData` / `CanData` / `GnssData` / `DrState` / `DrConfig` / `DrHealth` / `OdomResult`
- `core/dr_eskf.cpp` — `DrEskf`：`feedImu/feedCan/feedGnss`、`predict`、`correctWheel/correctGnss`、`injectError`
- `ros/ros_adapter.cpp` — ROS↔核心类型转换
- `ros/global_map_viz.hpp` — 全局地图可视化（仅 RViz，算法不用）
- 可执行：`dr_odometry_node`（另有 `dr_odometry_debug_node`，支持观测消融开关）

**输入**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/ouster/imu` | `sensor_msgs/Imu` | IMU |
| `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` | 轮速（`can_speed_is_kmh` 控制单位） |
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | 仅 `dr/use_gnss: true` 时订阅 |

**输出**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/dr_odom` | `nav_msgs/Odometry` | odom→base_link，含 6×6 协方差 |
| `/localization/dr_status` | `deeplumin_msgs/LocalizationStatus` | 健康状态 |
| `/localization/dr_path` | `nav_msgs/Path` | 轨迹（缓存 5000 点） |

TF `odom→base_link` 仅 `dr/publish_tf: true` 时发布（默认 false）。配置：`config/dr_odometry.yaml`（观测开关、噪声、超时门限、外参）。

---

### 3.2 lio_odometry 系列 — 激光惯性里程计（4 个演进分支）

四个包是**同一代码族的演进分支**，ROS 接口完全一致，差异在局部地图结构与运动约束。

#### 3.2.1 lio_odometry（基线，iVox 版）

**作用**：紧耦合 LiDAR-Inertial 里程计（Faster-LIO 等价实现）：23 维手写 ESKF（`NavState`）+ IMU 预积分去畸变 + 点到面观测 + **iVox 增量体素局部地图**；可选 Anderson Acceleration、轮速（1D/3D）与 GNSS 姿态辅助观测（`CustomObservationModel` 回调）。核心库零 ROS 依赖。

**结构**：
- `core/laser_odometry_core.cpp` — `LaserOdometryCore`：`ProcessPointCloud/ProcessIMU`、`ObsModel`（点面）/`ObsModelWheelspeed`/`ObsModelGPS`、`MapIncremental`、GNSS 基准重零
- `core/eskf.cpp` — 手写 ESKF，`ObsType: LIDAR/WHEEL_SPEED/GPS/BIAS/...`
- `core/imu_processing.cpp` — IMU 初始化、去畸变、断流处理
- `core/pointcloud_preprocess.cpp` — Ouster/Velodyne/Livox 多雷达预处理
- `ros/ros_adapter.cpp` + `ros/config.cpp` — ROS 适配与 YAML 加载
- `thirdparty/ivox3d`、`thirdparty/Sophus`
- 可执行：`lio_odometry_node`

**输入**（四分支一致）：

| 话题 | 类型 | 开关 |
|---|---|---|
| `/ouster/imu` | `sensor_msgs/Imu` | 始终 |
| `/ouster/points` | `sensor_msgs/PointCloud2` | 始终 |
| `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` | `vel_gnss.use_wheel_speed` |
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | `vel_gnss.use_gnss` |
| `/slam_mapping_v2/correction` | `nav_msgs/Odometry` | `external.use_correction`（默认关） |

**输出**（四分支一致）：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/slam_odometry/odom` | `nav_msgs/Odometry` | odom→base_link（twist.angular 存的是陀螺零偏） |
| `/slam_odometry/cloud_registered` | `sensor_msgs/PointCloud2` | odom 系配准扫描 |
| `/slam_odometry/cloud_effect` | `sensor_msgs/PointCloud2` | 有效对应点 |

TF `odom→base_link` 默认不发。配置：`config/lio_odometry.yaml`。

**关键内部类型**（`core/types.h`）：`NavState`（23 维：pos/rot/vel/bg/ba/grav + LiDAR-IMU 外参）、`MeasureGroup`（IMU 队列 + 扫描）、`Pose6D`、`ESKF::CustomObservationModel`。

#### 3.2.2 四分支差异速查

| 维度 | lio_odometry | lio_odometry_plus | lio_odometry_Voxlmap | lio_odometry_bievr |
|---|---|---|---|---|
| **定位** | 基线 | +车辆运动约束 | 换 VoxelMap 地图 | +凹凸图配准 |
| 局部地图 | iVox | iVox | **FAST-LIVO2 VoxelMap 八叉树平面**（RANSAC 平面拟合、reinit、LRU、滑窗） | VoxelMap + **16×16 bump 凹凸高度图** |
| 残差 | 点到面 | 点到面 | 点到面（自适应 R、Mahalanobis 门控、OpenMP） | 点到面 + 点到凹凸图插值（`bump_weight_in_residual`） |
| ZUPT/NHC | 无 | **有**（静止检测→速度归零、停车在线标定陀螺零偏、重力对齐软观测、NHC vy=vz=0） | 无 | 无 |
| 轮速 | 1D/3D 可选（默认 3D） | **1D+Huber 启用**（抗滑移） | 同基线 | 默认 `use_wheel_speed: false` |
| GNSS | 姿态观测 + 停车 rezero | 同左 | 姿态观测（无 rezero） | 默认 `use_gnss: false`（纯 LIO+bump） |
| ESKF 扩展 | — | +`NHC`/`ACC_AS_GRAVITY` 更新分支 | +对角 variance R、Update 可拒绝野值 | 同基线 |
| 可执行 | `lio_odometry_node` | `lio_odometry_plus_node` | `lio_odometry_voxelmap_node` | `lio_odometry_bievr_node` |

> 已知问题：`lio_odometry_bievr/launch/lio_odometry.launch` 内容仍指向 voxelmap 的包名/配置（复制未改），运行需手动 `rosrun` 或修正。
> `lio_odometry_plus` 的设计文档见 `docs/zupt_nhc_design.md`；Voxlmap 见 `docs/VoxelMap_Optimization_Design.md`；bievr 见 `doc/bump_integration_into_voxelmap.md`。

---

### 3.3 slam_odometry_v2 — faster-lio 原版移植 + 辅助观测

**作用**：基于 faster_lio_ws 原始源码逐行移植的纯里程计节点：保留 IMU/LiDAR 同步、IMU 去畸变、**IKFoM/MTK 流形 ESKF**（`esekfom.hpp`）、iVox 局部地图；建图职责剥离给 `slam_mapping_v2`。在此之上叠加：轮速 3D 伪观测、GNSS 姿态观测、**DCReg 退化检测**（默认仅检测+日志）、可选 IMU 重力观测/自适应噪声/地面约束（默认全关）。**这是 `localization_v2.launch` 默认拉起的前端里程计。**

**结构**：
- `faster_lio/laser_odometry.cc` — `LaserOdometry` 主类（`ApplyWheelSpeedObservation`/`ApplyGnssAttitudeObservation`/`ApplyDCRegAndWeights`…）
- `faster_lio/IKFoM_toolkit/`（esekfom + mtk）、`use-ikfom.hpp` — 流形状态 `state_ikfom`
- `faster_lio/dcreg.hpp` 等 — 退化检测（`DCRegResult {is_degenerate, degenerate_mask[6], degeneracy_scores[6]}`）
- `ros/slam_odometry_v2_config.cpp`、`src/slam_odometry_v2_node.cc`
- 可执行：`slam_odometry_v2_node`

**输入**：

| 话题 | 类型 | 开关 |
|---|---|---|
| `/ouster/imu` | `sensor_msgs/Imu` | 始终 |
| `/ouster/points` | `sensor_msgs/PointCloud2` | 始终 |
| `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` | `vel_gnss.use_vel` |
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | `vel_gnss.use_gnss_attitude` |

**输出**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/slam_odom` | `nav_msgs/Odometry` | odom→base_link |
| `/localization/local_map` | `sensor_msgs/PointCloud2` | iVox 局部地图（每 10 帧、有订阅者才发） |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 配准扫描 |
| `/localization/cloud_registered_effect` | `sensor_msgs/PointCloud2` | 有效对应点（global_matcher_v2 的输入源） |

配置：`config/slam_odometry_v2.yaml`。TF `odom→base_link` 默认不发。

**与 lio_odometry 系列的区别**：输出话题命名空间不同（`/localization/*` vs `/slam_odometry/*`）；ESKF 实现不同（MTK 流形 vs 手写 NavState）；额外发布 `local_map`；带 DCReg/自适应噪声等辅助模块。

---

## 4. 全局定位 / 重定位模块

### 4.1 global_matcher_v2 — 在环全局匹配

**作用**：以固定频率（`match_rate: 10 Hz`）将当前帧配准点云与全局地图做**点云精配准**，输出 `map→odom` 修正量。属于"**在环**全局匹配器"（跟踪中持续消除漂移），**不做描述子检索**。附带差异检测：当前点云中不在图上的点占比 > `rate_threshold: 0.25` 判定定位失效（`/loc_available`）。

**结构**：
- `core/map_manager.cpp` — `MapManager`：single 整图 / chunked 分片地图（异步加载 + LRU `max_loaded_chunks: 64`），预构建协方差/KDTree/VGICP 体素
- `core/matcher.cpp` — `PointCloudMatcher`：icp/gicp/vgicp（small_gicp，支持 Huber/Cauchy 鲁棒核）、fast_gicp 混合分支
- `ros/global_matcher_v2_node.cpp` — `MatcherRunMode {BOOTSTRAP, TRACKING}`、初值选择、质量门控、差异检测
- 可执行：`global_matcher_v2_node`；文档：`docs/matching_flow.md`

**输入**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/cloud_registered_effect` | `sensor_msgs/PointCloud2` | 当前帧点云（launch 覆盖值） |
| `/localization/slam_odom` | `nav_msgs/Odometry` | odom→base_link（launch 覆盖值） |
| `/localization_v2/odom` | `nav_msgs/Odometry` | 跟踪初值来源（launch 覆盖 `initial_pose`） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 人工初值 |
| `/localization_v2/gm_anchor` | `geometry_msgs/PoseWithCovarianceStamped` | 初始化仲裁重锚 |
| `/localization/global_matcher/load_map_command` | `std_msgs/String` | single 模式热替换整图 |

**输入文件**：`map.pcd_path`（single 模式全局地图 PCD）或 `map.map_dir`（chunked 模式，含 `chunks.txt`/`index.txt`）。

**输出**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | `result.pose_frame: odom` 时为 map→odom |
| `/localization/global_matcher/status` | `deeplumin_msgs/LocalizationStatus` | 状态 |
| `/localization/global_matcher/global_map` | `sensor_msgs/PointCloud2` | latched 地图预览 |
| `/loc_available` | `std_msgs/Int32` | 差异检测结果（0=定位失效） |

不发布 TF。配置：`config/global_matcher_v2.yaml`（matcher 类型、质量门 `min_inlier_ratio: 0.69`/`max_inlier_rmse: 0.4`、jump 门限）。

**初值优先级**（TRACKING）：上次匹配结果 / fused_odom；（BOOTSTRAP）`/localization/manager/initial_pose`、`/initialpose`、默认/单位初值。

---

### 4.2 relocalization — LiDAR 描述子重定位（Scan Context / CSC）

**作用**：基于**全局描述子检索 + GICP/VGICP 精配准**的全局重定位："丢失后从零找回"，输出一次性 `map→base_link` 绝对位姿。

**检索流程**：query 点云预处理 → 描述子提取（`scan_context` 单通道 SC 或 `csc` 4 通道 CSC）→ Ring Key / Z-Key 粗筛（SC 100 / CSC 300 候选）→ Sector Key 精排 + 循环移位求 `yaw_diff` → Top-K（`top_k: 20`）→ 可选空间门控 → GICP/VGICP 两级精配准（初值 = 候选位姿 + `yaw_diff` 修正）→ 质量门（`converged && fitness && inlier_ratio && inlier_rmse`）。

**结构**：
- `core/relocalization_core.cpp` — `RelocalizationCore` 总控
- `descriptor/scan_context_descriptor.cpp` / `csc_descriptor.cpp` — `IRelocalizationDescriptor` 实现（RingKey/SectorKey/快速扇区对齐）
- `matcher/gicp_global_matcher.cpp` / `vgicp_global_matcher.cpp` — PCL GICP / small_gicp 两级 VGICP
- `core/keyframe_database.cpp` — 关键帧库（`buildSubmapAround`/`cropOriented`/`cropGlobalMap`）
- `ros/global_relocalization_node.cpp`（在线）、`ros/bag_eval_node.cpp`（rosbag 离线评估）
- 可执行：`global_relocalization_node`、`bag_eval_node`（旧 `relocalization_node` 保留兼容）

**输入**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/points` | `sensor_msgs/PointCloud2` | 查询点云 |
| `/localization_v2/relocalization/command` | `std_msgs/String` | `"start"`/`"stop"` |

**输入文件**：关键帧 PCD 目录（`keyframe_dir`）、TUM 位姿文件（`pose_file`）、全局地图 PCD（`global_map_pcd`，crop 模式）、**描述子库 `.bin`**（`scan_context_db`，由外部工具 `relocalization_framework` 建库，本包只加载）。

**输出**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/relocalization/result` | `deeplumin_msgs/RelocalizationResult` | latched，`method: "csc_vgicp"` |
| `/reloc_pose` | `geometry_msgs/PoseStamped` | latched |
| `/reloc_status` | `std_msgs/String` | latched 状态摘要 |

bag 评估模式另发 `/offline_eval/*` 可视化话题 + CSV 报告。不发布 TF。配置：`config/relocalization.yaml`。

---

### 4.3 aruco_relocalization — 视觉 ArUco 标记重定位

**作用**：基于 **ArUco 视觉标记** 的全局重定位，共 3 个节点：

| 节点 | 可执行 | 功能 |
|---|---|---|
| `offline_mapping_v2` | `offline_mapping_v2_node` | 读 SLAM 关键帧 CSV+图像，批量检测 Marker，生成 `marker_map.yaml` |
| `aruco_offline_mapping` | `aruco_reloc_offline_mapping_node` | 订阅实时轨迹 + 图像，在线聚合 Marker（V1） |
| `aruco_reloc_online` | `aruco_reloc_online_node` | 在线检测 → PnP 解算 → 校验 → [VGICP 精化] → 发布位姿 |

**在线 pipeline**：图像（`image_rotation` 旋转）→ ArUco 检测 + 亚像素精化 → 单 Marker PnP（或多 Marker RANSAC）反解 `T_map_base` → 4 级校验（marker 数量 / 重投影误差 / marker 间距 / 运动一致性跳变）→ 可选**点云验证**（VGICP 对全局地图子图配准 + inlier 统计，通过后用精化位姿发布）。

**结构**：
- `detector/marker_detector.cpp`、`solver/relocalization_solver.cpp`、`validator/relocalization_validator.cpp`
- `verifier/pointcloud_verifier.cpp` — 点云缓冲、子图裁剪、VGICP 精化验证
- `map/marker_map_loader.cpp`、`offline_mapping_v2/offline_mapping_v2.cpp`
- `ros/online_node.cpp`、`ros/param_loader.cpp`

**输入**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/li_image` | `sensor_msgs/Image` (bgr8) | 相机图像 |
| `/ouster/points` | `sensor_msgs/PointCloud2` | 点云验证用 |
| `/localization_v2/relocalization/command` | `std_msgs/String` | 启停 |

**输入文件**：`marker_map.yaml`（Marker 地图）、全局地图 PCD（点云验证用）。

**输出**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/relocalization/result` | `deeplumin_msgs/RelocalizationResult` | latched，`method="aruco"`，与激光重定位**共用话题**（二选一） |
| `/aruco_reloc/pose` | `geometry_msgs/PoseStamped` | latched |
| `/aruco_reloc/status` | `std_msgs/String` | latched |
| `/aruco_reloc/debug_image`、`/aruco_reloc/*_cloud` | Image / PointCloud2 | 调试层 |

配置：`config/aruco_relocalization.yaml`（相机内参、`lidar_to_camera` 外参、`DICT_5X5_1000`、`marker_size: 0.2`、校验/验证门限）。

### 4.4 三个全局定位模块的定位区别

| | global_matcher_v2 | relocalization | aruco_relocalization |
|---|---|---|---|
| 触发方式 | 在环 10Hz 持续匹配 | 丢失后命令触发，一次性 | 同左（视觉） |
| 手段 | 点云对全局地图精配准 | 描述子 Top-K 检索 + GICP | ArUco 检测 + PnP（+VGICP 验证） |
| 初值依赖 | 上次位姿 / 人工初值 | 无需初值（检索） | 无需初值（看到 marker 即可） |
| 输出语义 | map→**odom** 修正 | map→**base_link** | map→**base_link** |
| 输出消息 | `GlobalMatchResult` | `RelocalizationResult` | `RelocalizationResult`（`method=aruco`） |
| 依赖资源 | 全局地图 PCD | 地图 + 描述子库 `.bin` + 关键帧库 | `marker_map.yaml` +（可选）全局 PCD |

---

## 5. 建图模块

### 5.1 slam_mapping_v2 — 后端建图（关键帧 / 回环 / 位姿图优化）

**作用**：订阅里程计节点输出，完成全局地图维护、关键帧管理、GNSS 后端因子、回环检测（**激光 KD-Tree+GICP / ORB 视觉 BoW / hybrid**）与 **GTSAM ISAM2** 位姿图优化。多线程：输入回调仅入队；主线程做 odom/scan 配对与关键帧判定；后端线程做 ISAM2+回环；可视化线程发布 path/marker。

**结构**：
- `src/slam_mapping_v2_node.cc` — 节点入口、服务处理、TF map→odom 发布
- `core/mapping_runtime.cpp` — `MappingRuntime`/`LoopClosureBackend` 主循环
- `core/keyframe_manager.cpp` — 关键帧选取（距离/旋转/时间/点数/帧间隔门限）
- `core/loop_detector_kdtree_gicp.cpp` / `visual_loop_detector.cpp` — 激光 / 视觉（DBoW2）回环，`ILoopDetector` 接口
- `core/pose_graph_optimizer.cpp` — ISAM2（odom 因子 + 回环因子 + GNSS 因子）
- `core/global_map.cpp` / `map_saver.cpp` — iVox 全局地图与结果保存
- 可执行：`slam_mapping_v2_node`

**输入**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/slam_odometry/odom` | `nav_msgs/Odometry` | 前端里程计（yaml 激活值；见 §7 注意事项） |
| `/slam_odometry/cloud_registered` | `sensor_msgs/PointCloud2` | odom 系配准扫描 |
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | 仅 `loop_closure/gnss.enable` 时 |
| `/camera/li_image` | `sensor_msgs/Image` | 仅视觉回环时 |

**输出**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/slam_mapping_v2/status` | `deeplumin_msgs/MappingStatus` | 10Hz 状态 |
| `/slam_mapping_v2/path` | `nav_msgs/Path` | 优化后轨迹 |
| `/slam_mapping_v2/map` | `sensor_msgs/PointCloud2` | 地图预览（降采样） |
| `/slam_mapping_v2/loop_markers`、`/slam_mapping_v2/keyframes` | `visualization_msgs/MarkerArray` | 调试标记 |

**服务**：`/slam_mapping_v2/start|stop|save|get_status`（`deeplumin_msgs/*MappingRuntime*`）、`/slam_mapping_v2/reset`（`std_srvs/Trigger`）。

TF `map→odom`（`runtime.publish_tf: true`）。保存产物（`output_root`，默认 `/home/hhy/2004ros/deeplumin_mapping_2`）：`map.pcd`、`trajectory.csv`、`keyframes.csv`、`keyframes/<id>.pcd`（+`images/`）、`loop_results/`。

配置：`config/slam_mapping_v2.yaml`（关键帧门限 `distance_thresh: 0.6`、回环 `detector_type: laser|visual|hybrid`、GNSS 因子、iVox 分辨率、线程周期）。

### 5.2 mapping_manager — 建图编排管理

**作用**：建图系统中央管理节点：向上给 GUI 提供统一服务，向下协调 `slam_mapping_v2`，同时管理 **rosbag 录制**（fork/exec `rosbag record`）、环境检测（ROS Master / 配置 / 输出目录 / 后端服务可用性）与状态聚合。状态机：`IDLE → CHECKING_ENV → READY → STARTING → RUNNING → STOPPING → SAVING_RESULTS → FINISHED`（任一步可入 `FAILED`）。

**结构**：
- `src/mapping_manager_node.cpp` — 主节点（6 服务、3 发布、5Hz 状态定时器）
- `core/manager_core.cpp` — 纯业务状态机（不依赖 ROS）
- `ros/bag_recorder.cpp` — rosbag record 子进程管理
- 可执行：`mapping_manager_node`（`scripts/mapping_manager_node.py` 为 Python 备用实现）

**输入**：订阅 `/slam_mapping_v2/status`；作为 client 调用 `/slam_mapping_v2/start|stop|get_status`。

**输出服务**（对外）：

| 服务 | 类型 |
|---|---|
| `/mapping/check_environment` | `CheckMappingEnvironment` |
| `/mapping/start` / `/mapping/stop` | `StartMapping` / `StopMapping` |
| `/mapping/start_bag_recording` / `/mapping/stop_bag_recording` | `StartBagRecording` / `StopBagRecording` |
| `/mapping/get_status` | `GetMappingStatus` |

**输出话题**：`/mapping/status`（`MappingStatus`，5Hz）、`/mapping/record_status`（`MappingRecordStatus`）、`/mapping/environment_last_result`（latched）。产物目录 `{output_root}/{task_id}/`。配置：`config/mapping_manager.yaml`（backend 服务名、状态频率、默认 bag 话题列表）。

### 5.3 mapping_gui — 建图图形控制台

**作用**：Python3 + tkinter 图形化建图控制台：任务配置（任务名/配置路径/输出根目录/启用回环/自动录包）、操作按钮（检测环境/开始/结束建图/录包）、状态监控与事件日志。**不跑算法、不做 3D 渲染**（3D 由 RViz 负责），全部操作经 Service 异步调用 `mapping_manager`。

**结构**：单脚本 `scripts/mapping_gui_node.py`（tkinter UI + Service 客户端 + 500ms 刷新）；`launch/mapping_gui.launch` 一键拉起 GUI + mapping_manager + slam_mapping_v2。

**输入**（订阅）：`/mapping/status`、`/mapping/record_status`、`/mapping/environment_last_result`。
**输出**（调用服务）：`/mapping/check_environment`、`/mapping/start`、`/mapping/stop`、`/mapping/start|stop_bag_recording`、`/mapping/get_status`。自身不发布话题/TF/文件。

---

## 6. 融合与编排

### 6.1 localization_v2 — 定位融合节点（核心编排）

**作用**：融合局部里程计（odom→base_link 高频）与全局匹配（map→odom 低频修正），输出平滑且全局一致的 `map→base_link` 位姿；管理定位生命周期状态机（初始化/跟踪/丢失/失败），编排重定位恢复，对下游发布 `LocPoint`。

**融合公式**：
```
map_odom = map_base ⊕ (odom_base)⁻¹    # GM / 人工初值 / 重定位更新时
map_base = map_odom ⊕ odom_base        # 100Hz 高频输出
```

**结构**：
- `src/ros/localization_v2_node.cpp` — 主节点：初始化仲裁、GM 质量三级分级、跟踪失效处理、traj 落盘
- `core/fusion_core.cpp` — `FusionCore`：map↔odom↔base 组合、jump guard
- `core/initialization.cpp` — `LocalizationStateMachine`（5 状态）+ `RelocCommand` 队列
- `interface/relocalization_interface.hpp` + `ros/relocalization_ros_adapter.cpp` — 重定位客户端抽象（可替换实现）
- 可执行：`localization_v2_node`；设计文档：`docs/state_machine_design.md`、`docs/detailed_design.md`

**输入**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization/slam_odom` | `nav_msgs/Odometry` | 局部里程计 odom→base_link |
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | 全局匹配（`result_pose_frame: odom` 时 pose 为 map→odom） |
| `/localization/relocalization/result` | `deeplumin_msgs/RelocalizationResult` | 重定位（map→base_link） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 人工初值（支持 map/odom/base_link 三种 frame） |
| `/localization/cloud_registered_effect` | `sensor_msgs/PointCloud2` | 仅可视化转 map 系 |

**输出**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/localization_v2/odom` | `nav_msgs/Odometry` | map→base_link 融合里程计，**100Hz** |
| `/localization_v2/map_to_odom` | `nav_msgs/Odometry` | map→odom 修正量 |
| `/localization_v2/path` | `nav_msgs/Path` | 融合轨迹 |
| `/localization_v2/status` | `deeplumin_msgs/FusionStatus` | 状态机 + 各源健康 |
| `/localization_v2/loc_point` | `deeplumin_msgs/LocPoint` | 下游规划位姿（**yaw/pitch 单位为度**） |
| `/localization_v2/current_scan_in_map` | `sensor_msgs/PointCloud2` | 点云转 map 系（可视化） |
| `/localization_v2/gm_anchor` | `geometry_msgs/PoseWithCovarianceStamped`（latched） | 重定位胜出时通知 GM 重锚 |
| `/localization_v2/relocalization/command` | `std_msgs/String`（latched） | `"start"`/`"stop"` 重定位命令 |

**TF**：`map→odom`、`map→base_link` 始终发布（launch 设置 `base_yaw_offset=-1.5707963`，即 yaw 加 -90° 适配下游 y 朝前约定）；`odom→base_link` 由里程计节点发。无 Service，控制全走 topic 命令。

**状态机（5 状态）**：

| 状态 | 语义 | output_valid | level |
|---|---|---|---|
| `UNINITIALIZED` | 等待初始位姿 | false | WARNING |
| `RELOCALIZING` | 重定位获取初值中 | false | WARNING |
| `TRACKING` | 正常跟踪 | true | NORMAL |
| `TRACKING_LOST` | 跟踪丢失、恢复中 | true（降级） | DEGRADED |
| `LOCALIZATION_FAILURE` | 定位失败、需人工干预 | false | FAILURE |

关键转换：
- `RELOCALIZING → TRACKING`：重定位成功过质量门限 / 人工 `/initialpose` / GM 自身成功（任一）。初始化采用**双源仲裁**：缓存 GM 与重定位候选（重定位时效窗 3s），评分 `score = inlier_ratio - 0.3*inlier_rmse`，重定位优先并向 GM 发 anchor；`fallback_timeout: 8.0` 超时取已就绪源。
- `TRACKING → TRACKING_LOST`：GM 连续失败 ≥ 50，或连续退化帧 ≥ 50（退化：`0.4 ≤ inlier_ratio < 0.69` 且 rmse≤0.4）。
- `TRACKING_LOST → TRACKING`：恢复需过确认窗口（GM 3 次高质量 / reloc 1 次）+ 驻留时间防抖（2.0s / 1.0s）。
- `TRACKING_LOST → LOCALIZATION_FAILURE`：恢复超时 30s 或重定位连续失败 ≥ 50；失败后仅允许高质量 GM 恢复。

GM 结果应用前过三级质量分级 + **jump guard**（位姿跳变 > 10m 或 > 20° 拒绝；人工初值/重定位可绕过）。TRACKING_LOST 时可继续输出并标记降级（`degraded.allow_degraded_output: true`）。

**launch 编排**（`launch/localization_v2.launch` 一次性拉起整条链路）：`slam_odometry_v2_node`（`start_slam:=true`）→ `localization_v2_node` → `global_matcher_v2_node`（`start_global_matcher:=true`）→ `global_relocalization_node`（`start_relocalization:=true`）；可选 `aruco_reloc_online_node`（`start_aruco_reloc:=false`，与激光重定位二选一）、RViz、rosbag play。

配置：`config/localization_v2.yaml`（输出频率、jump guard 阈值、质量门、状态机门限、防抖参数）。轨迹落盘：`traj_path` 指向 `traj.txt`（仅 TRACKING 状态记录）。

---

## 7. 辅助模块

### 7.1 sensor_frame_adapter — 传感器坐标系适配

**作用**：把原始 LiDAR 点云归一化到标准坐标系：对 x/y/z 施加固定 4×4 变换（平移 + RPY + 可选 `rotate_xy_y_negx` 等效 yaw=-90°，兼容 ysw 约定）、替换 `frame_id`、可选去 NaN、保留时间戳。

**输入**：`/ouster/points`（`sensor_msgs/PointCloud2`，需含 FLOAT32 x/y/z）。
**输出**：`/localization/points`（`frame_id` 改写为 `lidar_link`）。无服务、无 TF。
**结构**：单文件节点 `src/sensor_frame_adapter_node.cpp`；配置 `config/sensor_frame_adapter.yaml`。

### 7.2 loclog — 运行日志目录

非 ROS 包。各节点 `log_dir` 参数指向此处，落盘 `lio_odometry*.log`、`GM_V2.log`、`localization_v2.log`、`relocalization.log` 等（syslog 格式）。可用于核对实际运行时的话题连接关系。

---

## 8. 运行方式速查

| 场景 | 入口 |
|---|---|
| **建图**（GUI） | `roslaunch mapping_gui mapping_gui.launch`（拉起 GUI + mapping_manager + slam_mapping_v2） |
| **建图**（无 GUI） | `roslaunch slam_mapping_v2 slam_mapping_v2.launch`，再 `rosservice call /slam_mapping_v2/start ...` |
| **定位** | `roslaunch localization_v2 localization_v2.launch`（slam_odometry_v2 + localization_v2 + global_matcher_v2 + relocalization + RViz） |
| **重定位评估** | `roslaunch relocalization bag_eval_viz.launch` |
| **ArUco 建图 / 重定位** | `roslaunch aruco_relocalization offline_mapping_v2.launch` / `aruco_relocalization.launch` |
| **独立里程计** | `roslaunch lio_odometry lio_odometry.launch`（或 plus / voxelmap / bievr、`slam_odometry_v2.launch`） |

---

## 9. 注意事项（话题命名的历史迁移痕迹）

1. **slam_mapping_v2 输入话题**：yaml 当前激活值为 `/slam_odometry/odom`、`/slam_odometry/cloud_registered`（对应 lio_odometry 系列输出）；其 README 表格中的 `/localization/slam_odom` 是被注释的旧配置。若前端换用 `slam_odometry_v2` 需改 yaml。
2. **localization_v2.launch** 中 `start_global_matcher` 参数存在但未对 `global_matcher_v2` 节点加 `if` 条件（总是启动）。
3. **TF yaw 偏置**：localization_v2 发布的 `map→base_link` 带 `base_yaw_offset=-π/2`（y 朝前约定），下游直接用该 TF 需注意。
4. **LocPoint 单位**：`yaw/pitch` 是**度**，而融合里程计姿态是四元数/弧度。
5. **lio_odometry_bievr 的 launch 文件损坏**（仍指向 voxelmap 包名/配置）。
6. **重定位二选一**：`relocalization`（激光）与 `aruco_relocalization`（视觉）共用 `/localization/relocalization/result`，同时开启会互相覆盖。
7. **描述子库需外建**：`relocalization` 的 `.bin` 描述子库由 `relocalization_framework` 工具离线生成；修改 `min_range`/`max_radius` 等描述子参数后必须重建库。
