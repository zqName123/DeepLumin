# DeepLumin 定位模块开发分析书

> 适用工程：DeepLumin  
> 适用目录：`src/localization/`、`src/deeplumin_msgs/msg/localization/`  
> ROS 版本：ROS1 Noetic  
> 编写目的：说明 DeepLumin 定位功能域的模块结构、功能边界、接口契约、输入输出和后续开发约束，作为 `slam_odometry`、`relocalization`、`dr_odometry`、`pose_fusion`、`global_matcher`、`localization_manager` 的开发依据。

---

## 1. 文档范围与定位

DeepLumin 采用“功能域聚合、子包独立”的组织方式。定位功能域位于：

```text
DeepLumin/src/localization/
├── slam_odometry/          # SLAM 局部里程计，当前默认 Faster-LIO
├── relocalization/         # 基于 Scan Context + GICP 的全局重定位
├── dr_odometry/            # 航位推算，占位节点，待实现
├── pose_fusion/            # 多源位姿融合，占位节点，待实现
├── global_matcher/         # 全局匹配，占位节点，待实现或与 relocalization 合并职责
└── localization_manager/   # 定位管理器，占位节点，待实现
```

本文档以 DeepLumin 当前代码结构为准，吸收 `localization_ws2/plan/design.md` 与 `architecture.md` 中的接口化、场景化 GNSS、多源融合、重定位触发等设计思想，但不照搬其单工作空间三包结构。

定位功能域的核心目标：

1. 井下、露天、过渡区域均可输出稳定车辆定位。
2. `slam_odometry`、`dr_odometry`、`pose_fusion`、`relocalization` 等模块既可单独作为节点运行，也可通过 ROS 话题/服务互通，组合成完整定位链路。
3. 算法核心与 ROS 适配分离，SLAM、重定位、融合滤波、匹配器均应可替换。
4. GNSS 是场景相关观测源，不应在井下默认强依赖；应通过场景状态和质量评估决定其使用方式与权重。

---

## 2. 总体架构

### 2.1 分层原则

DeepLumin 定位模块采用三层结构：

```text
ROS 节点层
  - 参数加载、话题订阅/发布、TF 发布、服务入口、运行频率控制

核心算法层
  - SLAM、DR、融合、重定位、地图/关键帧、匹配、状态机
  - 尽量使用 Eigen/PCL/标准 C++ 数据结构，降低 ROS 耦合

算法实现层
  - Faster-LIO、Scan Context、GICP、后续 NDT/ICP/ESKF/UKF 等可替换实现
```

当前 `slam_odometry` 已经形成较清晰的分离：

```text
slam_odometry/
├── include/localization/interface/i_slam_odometry.hpp
├── include/localization/common/types.hpp
├── include/localization/common/config.hpp
├── include/localization/ros/
├── src/slam/
└── src/slam_odometry_node.cpp
```

当前 `relocalization` 也采用接口化结构：

```text
relocalization/
├── include/relocalization/interface/i_descriptor.hpp
├── include/relocalization/interface/i_global_matcher.hpp
├── include/relocalization/core/
├── include/relocalization/descriptor/
├── include/relocalization/matcher/
├── src/core/
├── src/descriptor/
├── src/matcher/
└── src/ros/
```

后续 `dr_odometry`、`pose_fusion`、`global_matcher`、`localization_manager` 应按同样模式补齐：`include/<package>/interface` 放抽象接口，`src/core` 或 `src/<algorithm>` 放核心实现，`src/<package>_node.cpp` 只做 ROS 适配。

### 2.2 推荐数据流

```text
Sensing / Vehicle Interface
  ├── /ouster/imu 或 /sensor/imu
  ├── /ouster/points 或 /sensor/lidar/points
  ├── /can_receive_info
  ├── /gnss_chc_data 或 /sensor/gnss/fix
  └── /system/scene_state

Localization
  ├── dr_odometry
  │   ├── 输入：IMU、CAN 车速、GNSS heading/position（可选）
  │   └── 输出：/localization/dr_odom
  ├── slam_odometry
  │   ├── 输入：IMU、LiDAR 点云、动态点云 mask（预留）
  │   └── 输出：/localization/slam_odom、/localization/local_map、/localization/cloud_registered
  ├── relocalization / global_matcher
  │   ├── 输入：当前扫描、关键帧库、描述子库、地图子图
  │   └── 输出：重定位位姿、匹配状态、对齐点云、目标子图
  ├── pose_fusion
  │   ├── 输入：DR、SLAM、GNSS、重定位事件、场景状态
  │   └── 输出：/localization/fused_odom、/localization/status、TF map/odom/base_link
  └── localization_manager
      ├── 输入：各模块状态、系统场景、人工命令
      └── 输出：模式切换、重定位触发、健康状态
```

### 2.3 坐标系与 TF

推荐统一使用：

| 坐标系 | 含义 | 发布者 |
|---|---|---|
| `map` | 全局地图坐标系，重定位和全局地图所在坐标 | `pose_fusion` 或 `localization_manager` |
| `odom` | 连续局部里程计坐标系，短时平滑但可能漂移 | `slam_odometry` / `dr_odometry` / `pose_fusion` |
| `base_link` | 车体坐标系 | 定位输出链路 |
| `imu_link` | IMU 坐标系 | 静态外参或 sensing |
| `lidar_link` | 激光雷达坐标系 | 静态外参或 sensing |

原则：

1. `slam_odometry` 可发布 `odom -> base_link` 调试 TF。
2. 完整定位链路中，最终 TF 应由 `pose_fusion` 统一发布，避免多节点重复发布同一 TF。
3. 重定位结果不应直接造成 `base_link` 跳变；推荐更新 `map -> odom` 或作为融合滤波 reset/校正事件。
4. 井下无 GNSS 时，`map` 仍由先验点云地图或建图结果定义。

---

## 3. 公共消息与配置

### 3.1 DeepLumin 自定义消息

定位相关自定义消息位于：

```text
DeepLumin/src/deeplumin_msgs/msg/localization/
├── FusedOdometry.msg
├── LocalizationStatus.msg
├── SlamPose.msg
└── SlamHealth.msg
```

现有消息用途：

| 消息 | 主要字段 | 用途 |
|---|---|---|
| `FusedOdometry` | `pose`、`twist`、`pose_covariance`、`twist_covariance`、`source_sensors` | 融合定位结果，适合规划/控制消费 |
| `LocalizationStatus` | `level`、`failure_reason`、`quality_score`、`is_gnss_available`、`is_slam_available`、`is_dr_available`、`current_mode` | 定位健康状态与模式 |
| `SlamPose` | `pose`、`covariance`、`loop_detected`、`slam_confidence` | SLAM 位姿或建图/回环调试 |
| `SlamHealth` | `score`、`feature_count`、`condition_number`、`match_residual`、`is_degenerated` | SLAM 退化与匹配质量 |

建议后续新增或扩展：

| 建议消息/服务 | 类型 | 用途 |
|---|---|---|
| `RelocalizationResult.msg` | Message | 表达重定位是否成功、候选帧、匹配分数、变换、协方差 |
| `DrHealth.msg` | Message | 表达 IMU/CAN/GNSS 输入状态、静止检测、轮速尺度、bias 收敛状态 |
| `SetLocalizationMode.srv` | Service | 切换 `LOCALIZATION`、`MAPPING`、`RELOCALIZATION`、`PURE_SLAM` |
| `TriggerRelocalization.srv` | Service | 人工触发重定位，可传初值或搜索范围 |
| `ResetLocalization.srv` | Service | 使用指定初始位姿重置融合状态 |

### 3.2 公共 C++ 数据类型

`slam_odometry/include/localization/common/types.hpp` 已经定义了可复用定位核心类型：

| 类型 | 用途 |
|---|---|
| `ImuData` | IMU 时间戳、角速度、线加速度、协方差 |
| `WheelData` | 轮速、转角、速度噪声 |
| `GnssData` | 经纬高、fix 类型、ENU 位置、位置噪声 |
| `TimestampedPointCloud` | 带时间戳点云与 frame_id |
| `OdomResult` | 统一里程计输出，含位姿、速度、协方差、退化信息 |
| `DrState` | DR/ESKF 名义状态，含 15 维协方差 |
| `RelocCandidate` | 重定位候选关键帧 |
| `SceneState` | `OPEN_PIT`、`TRANSITION`、`UNDERGROUND` |
| `ObserverWeights` | SLAM、GNSS、DR 预测权重 |
| `RunMode` | `LOCALIZATION`、`MAPPING`、`RELOCALIZATION`、`PURE_SLAM` |

建议：后续定位子包尽量复用这些类型，或将其抽出到独立公共包，避免每个子包重复定义不兼容的数据结构。

### 3.3 话题配置

`slam_odometry/include/localization/ros/sensor_topics.hpp` 已经给出统一话题配置结构：

| 配置项 | 默认含义 |
|---|---|
| `sensors.imu_topic` | IMU 输入，当前配置常用 `/ouster/imu` |
| `sensors.lidar_topic` | 点云输入，当前配置常用 `/ouster/points` |
| `sensors.wheel_topic` | CAN 车速输入，当前配置常用 `/can_receive_info` |
| `sensors.gnss_topic` | GNSS 输入，当前配置常用 `/gnss_chc_data` |
| `sensors.scene_topic` | 场景状态输入，当前建议 `/system/scene_state` |
| `outputs.dr_topic` | DR 输出 `/localization/dr_odom` |
| `outputs.slam_topic` | SLAM 输出 `/localization/slam_odom` |
| `outputs.fused_topic` | 融合输出 `/localization/fused_odom` |
| `outputs.status_topic` | 定位状态 `/localization/status` |
| `outputs.local_map_topic` | 局部地图 `/localization/local_map` |
| `outputs.registered_scan_topic` | 配准后当前帧 `/localization/cloud_registered` |

---

## 4. `slam_odometry` 模块分析

### 4.1 模块定位

`slam_odometry` 是局部 LiDAR-Inertial SLAM 里程计模块，当前默认实现为 Faster-LIO。它负责接收 LiDAR 点云和 IMU，输出局部连续位姿、局部地图和配准后的当前帧点云。

该模块应作为低频高精度局部观测源供 `pose_fusion` 使用，也可单独运行进行 SLAM 调试或建图前端。

### 4.2 目录结构

```text
slam_odometry/
├── include/localization/interface/i_slam_odometry.hpp
├── include/localization/common/
│   ├── types.hpp
│   ├── config.hpp
│   └── math_utils.hpp
├── include/localization/slam/
│   ├── faster_lio_impl.hpp
│   └── laser_mapping_core.hpp
├── include/localization/ros/
│   ├── ros_converters.hpp
│   ├── sensor_topics.hpp
│   ├── pointcloud_preprocess.hpp
│   └── ouster_point.hpp
├── src/slam/
│   ├── faster_lio_impl.cpp
│   └── laser_mapping_core.cpp
├── src/ros/
│   ├── ros_converters.cpp
│   ├── sensor_topics.cpp
│   └── pointcloud_preprocess.cpp
├── src/slam_odometry_node.cpp
├── thirdparty/faster_lio/
├── config/slam_odometry_ysw.yaml
└── launch/slam_odometry_ysw.launch
```

### 4.3 核心接口

`ISlamOdometry` 是 SLAM 里程计抽象接口：

```cpp
class ISlamOdometry {
public:
    virtual bool initialize(const SlamConfig& cfg) = 0;
    virtual void feedImu(const ImuData& imu) = 0;
    virtual void feedLidar(const TimestampedPointCloud& pc) = 0;
    virtual void feedDynamicMask(const TimestampedPointCloud& mask) = 0;
    virtual bool processOnce() = 0;
    virtual OdomResult getOdometry() const = 0;
    virtual TimestampedPointCloud getLocalMap() const = 0;
    virtual TimestampedPointCloud getCurrentScan() const = 0;
    virtual bool isDegenerate() const = 0;
    virtual double getDegeneracyFactor() const = 0;
    virtual void reset() = 0;
};
```

接口分析：

| 接口 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `initialize` | `SlamConfig` | `bool` | 加载算法参数、外参、滤波和退化阈值 |
| `feedImu` | `ImuData` | 无 | 缓存 IMU，用于运动预测和点云去畸变 |
| `feedLidar` | `TimestampedPointCloud` | 无 | 缓存当前 LiDAR 帧 |
| `feedDynamicMask` | `TimestampedPointCloud` | 无 | 预留动态物体 mask，后续可过滤动态点 |
| `processOnce` | 内部缓存 | `bool` | 执行一次 SLAM 主流程，有新结果返回 true |
| `getOdometry` | 无 | `OdomResult` | 输出局部位姿、速度、协方差、退化状态 |
| `getLocalMap` | 无 | 点云 | 输出局部地图 |
| `getCurrentScan` | 无 | 点云 | 输出当前帧点云 |
| `isDegenerate` | 无 | `bool` | 判断环境退化 |
| `getDegeneracyFactor` | 无 | `double` | 退化程度，供融合调权 |
| `reset` | 无 | 无 | 重置 SLAM 状态 |

### 4.4 ROS 节点输入输出

`slam_odometry_node` 当前订阅：

| 话题 | 消息类型 | 来源 | 说明 |
|---|---|---|---|
| `/ouster/imu` 或配置项 `sensors.imu_topic` | `sensor_msgs/Imu` | sensing/IMU 或 Ouster | SLAM 内部 IMU 输入 |
| `/ouster/points` 或配置项 `sensors.lidar_topic` | `sensor_msgs/PointCloud2` | sensing/LiDAR | 原始点云 |

当前发布：

| 话题 | 消息类型 | 坐标系 | 说明 |
|---|---|---|---|
| `/localization/slam_odom` | `nav_msgs/Odometry` | `odom -> base_link` | SLAM 局部里程计 |
| `/localization/local_map` | `sensor_msgs/PointCloud2` | `odom` | 局部地图 |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | `odom` | 当前帧变换到 odom 后的点云 |
| TF | `tf` | `odom -> base_link` | 单独调试时使用 |

### 4.5 参数分析

`config/slam_odometry_ysw.yaml` 中与 SLAM 相关的关键参数：

| 参数 | 说明 |
|---|---|
| `slam.type` | 当前支持 `faster_lio`，未来可扩展 `lio_sam`、`fast_livo` |
| `slam.faster_lio.scan_rate` | 点云帧率 |
| `slam.faster_lio.preprocess.lidar_type` | 雷达类型，当前 Ouster 为 3 |
| `slam.faster_lio.preprocess.scan_line` | 雷达线数 |
| `slam.faster_lio.preprocess.blind` | 近距离盲区过滤 |
| `slam.faster_lio.mapping.acc_cov/gyr_cov` | LIO 内部 IMU 噪声 |
| `slam.faster_lio.mapping.extrinsic_T/R` | LiDAR-IMU 外参 |
| `slam.faster_lio.ivox.*` | iVox 地图参数 |
| `slam.faster_lio.degeneracy_threshold` | 退化判断阈值 |

### 4.6 开发要求

1. `slam_odometry` 应继续保持核心实现可作为库复用，ROS 节点只做适配。
2. `OdomResult.valid == false` 时不能发布有效位姿给融合模块。
3. 退化信息必须输出给 `pose_fusion`，融合层根据退化程度增大 SLAM 观测噪声或临时降权。
4. `feedDynamicMask` 已预留，应在感知模块输出动态障碍物 mask 后用于动态点剔除。
5. 完整定位链路中不建议由 `slam_odometry` 和 `pose_fusion` 同时发布 `odom -> base_link`。

---

## 5. `relocalization` 模块分析

### 5.1 模块定位

`relocalization` 是全局重定位模块，用于在已知地图或关键帧数据库中，根据当前点云查询候选位置并执行精匹配，输出车辆在 `map` 坐标系下的全局位姿。

当前实现路线：

```text
当前点云
  -> 点云预处理
  -> Scan Context 描述子生成
  -> 描述子数据库 Top-K 检索
  -> 候选关键帧/子图构建
  -> GICP 精匹配
  -> 输出重定位位姿与匹配质量
```

### 5.2 目录结构

```text
relocalization/
├── include/relocalization/core/
│   ├── common.hpp
│   ├── keyframe_database.hpp
│   └── relocalization_core.hpp
├── include/relocalization/interface/
│   ├── i_descriptor.hpp
│   └── i_global_matcher.hpp
├── include/relocalization/descriptor/scan_context_descriptor.hpp
├── include/relocalization/matcher/gicp_global_matcher.hpp
├── include/relocalization/ros/
│   ├── param_loader.hpp
│   └── eval_visualizer.hpp
├── src/core/
├── src/descriptor/
├── src/matcher/
├── src/ros/
│   ├── global_relocalization_node.cpp
│   ├── bag_eval_node.cpp
│   └── param_loader.cpp
├── config/relocalization.yaml
└── launch/
```

### 5.3 核心接口

描述子接口：

```cpp
class IRelocalizationDescriptor {
public:
  virtual bool load(const std::string& path) = 0;
  virtual void setRingKeyCandidates(int n) = 0;
  virtual Eigen::MatrixXd makeDescriptor(const CloudConstPtr& cloud) const = 0;
  virtual std::vector<DescriptorCandidate> query(
      const Eigen::MatrixXd& descriptor, int top_k) const = 0;
  virtual std::size_t size() const = 0;
  virtual std::string summary() const = 0;
};
```

全局匹配接口：

```cpp
class IGlobalMatcher {
public:
  virtual MatchResult alignCandidates(
      const CloudConstPtr& source,
      const std::vector<DescriptorCandidate>& candidates,
      const KeyframeDatabase& database) const = 0;
  virtual std::string summary() const = 0;
};
```

核心编排类：

```cpp
class RelocalizationCore {
public:
  bool initialize(const RelocalizationCoreConfig& config);
  std::vector<DescriptorCandidate> queryCandidates(const CloudConstPtr& source) const;
  MatchResult matchCandidates(
      const CloudConstPtr& source,
      const std::vector<DescriptorCandidate>& candidates) const;
  RelocalizationQueryResult relocalize(const CloudConstPtr& source) const;
};
```

接口分析：

| 接口 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `IRelocalizationDescriptor::load` | 描述子数据库路径 | `bool` | 加载离线 Scan Context 数据库 |
| `makeDescriptor` | 当前点云 | 描述子矩阵 | 生成 query 描述子 |
| `query` | query 描述子、Top-K | 候选关键帧 | 粗检索 |
| `IGlobalMatcher::alignCandidates` | 当前点云、候选、关键帧库 | `MatchResult` | 对候选子图执行 GICP/NDT/ICP |
| `RelocalizationCore::relocalize` | 当前点云 | 候选+最终匹配 | 完整重定位流程 |

### 5.4 ROS 节点输入输出

`global_relocalization_node` 当前订阅：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `lidar_topic`，默认 `/ouster/points` | `sensor_msgs/PointCloud2` | 当前点云 query |

当前发布：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `reloc_pose` | `geometry_msgs/PoseStamped` | 重定位成功后的 `map` 系位姿 |
| `reloc_status` | `std_msgs/String` | 匹配状态字符串，含 candidate、score、fitness、inlier 等 |
| `reloc_aligned_scan` | `sensor_msgs/PointCloud2` | 已对齐的 source 点云 |
| `reloc_target_submap` | `sensor_msgs/PointCloud2` | 候选目标子图 |

建议后续改进：

1. `reloc_status` 应升级为结构化 `deeplumin_msgs/RelocalizationResult`。
2. `reloc_pose` 应携带匹配协方差或质量等级，供 `pose_fusion` 判断是否接受。
3. 增加 `TriggerRelocalization.srv`，支持人工触发、自动触发和带初值触发。
4. 允许 `relocalization` 作为独立节点运行，也允许作为 `localization_manager` 内部服务被调用。

### 5.5 配置分析

`config/relocalization.yaml` 关键配置：

| 参数 | 说明 |
|---|---|
| `descriptor.type` | 描述子类型，当前 `scan_context` |
| `matcher.type` | 精匹配类型，当前 `gicp` |
| `keyframe_dir` | 关键帧 PCD 目录 |
| `pose_file` | 关键帧位姿文件 |
| `global_map_pcd` | 全局点云地图 |
| `scan_context_db` | Scan Context 描述子库 |
| `top_k` | 粗检索候选数量 |
| `query_frame.*` | query 点云坐标系转换 |
| `preprocess.*` | query 和建库一致的滤波配置 |
| `scan_context.*` | 描述子网格参数 |
| `submap.*` | 精匹配目标子图构建策略 |
| `gicp.*` | GICP 迭代、距离、fitness、inlier 验真参数 |

重要约束：

1. 建库和查询必须使用一致的 `preprocess` 与 Scan Context 参数。
2. `preprocess.max_range` 应与 `scan_context.max_radius` 一致。
3. 如果对建库点云启用天花板过滤或 z 过滤，query 端也应保持一致，否则描述子分布和匹配目标会不一致。

### 5.6 与完整定位链路的关系

重定位结果可用于：

1. 系统启动时初始化 `map -> odom`。
2. SLAM 退化或融合协方差过大时恢复全局位姿。
3. 地图切换或场景切换时重新建立全局约束。
4. 人工远程干预时提供位姿校正。

不建议：

1. 每帧都执行全局重定位。
2. 直接用一次低质量匹配结果硬重置定位。
3. 在井下强依赖 GNSS 初值触发重定位。

---

## 6. `dr_odometry` 模块设计与预留接口

### 6.1 当前状态

`dr_odometry` 当前为占位节点，仅启动 ROS 节点并 `spin`。该模块尚未实现核心算法、接口和消息转换。

### 6.2 模块定位

`dr_odometry` 是高频航位推算模块，目标输出 50-100Hz 连续位姿/速度预测。它在 SLAM 帧率较低、短时退化、点云延迟或临时无地图约束时提供连续运动估计。

DR 模块必须预留三类输入：

1. GNSS：露天或过渡场景可用，尤其使用 GNSS heading 作为方向观测。
2. IMU：角速度和加速度，作为状态传播主输入。
3. CAN receive info：车辆速度，作为纵向速度观测约束。

### 6.3 推荐核心接口

建议新增：

```text
dr_odometry/include/dr_odometry/interface/i_dr_odometry.hpp
dr_odometry/include/dr_odometry/core/dr_odometry_estimator.hpp
dr_odometry/include/dr_odometry/core/dr_types.hpp
dr_odometry/src/core/
dr_odometry/src/ros/
```

推荐接口：

```cpp
class IDrOdometry {
public:
  virtual ~IDrOdometry() = default;
  virtual bool initialize(const localization::DrConfig& config) = 0;
  virtual void feedImu(const localization::ImuData& imu) = 0;
  virtual void feedCanReceiveInfo(const CanReceiveInfoData& can) = 0;
  virtual void feedGnss(const localization::GnssData& gnss) = 0;
  virtual bool processUntil(double timestamp) = 0;
  virtual localization::DrState getState() const = 0;
  virtual localization::OdomResult getOdometry() const = 0;
  virtual DrHealth getHealth() const = 0;
  virtual void reset(const localization::DrState& initial_state) = 0;
};
```

其中 `CanReceiveInfoData` 至少应包含：

```cpp
struct CanReceiveInfoData {
  double timestamp = 0.0;
  double speed = 0.0;          // m/s，倒车为负
  double steer_angle = 0.0;    // rad，可选
  int gear = 0;                // 可选
  bool speed_valid = false;
};
```

`GnssData` 需要扩展或封装 heading：

```cpp
struct GnssHeadingData {
  double timestamp = 0.0;
  double heading = 0.0;        // rad，建议 ENU/map 系 yaw
  double heading_std = 0.0;
  bool heading_valid = false;
};
```

如果 GNSS 驱动只提供经纬高和姿态，应在 ROS 适配层转换到统一 ENU/map 约定后再进入核心。

### 6.4 融合策略预留

DR 实现细节暂不固定，但接口必须支持以下策略：

#### 6.4.1 有 GNSS 场景

适用：露天、井口、露天到井下过渡段，且 GNSS heading 质量可靠。

推荐策略：

1. IMU 用于高频状态传播。
2. `can_receive_info.speed` 作为车体 x 方向速度观测。
3. GNSS heading 作为 yaw 方向观测。
4. 如 GNSS position 质量可靠，可作为低频位置观测，但其权重由场景状态决定。
5. GNSS heading 与 CAN speed 可共同约束 IMU yaw 漂移和速度尺度。

#### 6.4.2 无 GNSS 场景

适用：井下巷道、GNSS 遮挡、GNSS 质量低。

推荐策略：

1. IMU 用于短时角速度和加速度传播。
2. `can_receive_info.speed` 作为前向速度主观测。
3. 使用非完整约束：车体 y/z 向速度近似为 0。
4. yaw 漂移主要依赖 IMU gyro bias 估计、静止检测、SLAM/重定位校正。
5. 不应把失效 GNSS heading 强行注入滤波。

### 6.5 ROS 输入输出

推荐订阅：

| 话题 | 消息类型 | 必需 | 说明 |
|---|---|---|---|
| `/ouster/imu` 或 `/sensor/imu` | `sensor_msgs/Imu` | 是 | IMU 高频输入 |
| `/can_receive_info` | 当前车辆 CAN 消息类型 | 是 | 车速观测，字段需转换为 m/s |
| `/gnss_chc_data` 或 `/sensor/gnss/fix` | GNSS 驱动消息 | 否 | 位置、heading、fix 状态 |
| `/system/scene_state` | 场景状态消息 | 否 | 控制 GNSS 是否参与 |

推荐发布：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/localization/dr_odom` | `nav_msgs/Odometry` 或 `deeplumin_msgs/FusedOdometry` | DR 位姿/速度 |
| `/localization/dr_health` | 建议新增 `DrHealth` | 输入状态、bias、wheel scale、GNSS 可用性 |
| `/localization/status` | `deeplumin_msgs/LocalizationStatus` | 可选，或由 manager 聚合发布 |

### 6.6 开发要求

1. `dr_odometry` 不应依赖 `slam_odometry` 内部类，只通过公共数据类型或 ROS 消息互通。
2. CAN 速度转换应集中在 ROS adapter，核心层只接收标准 m/s。
3. GNSS heading 与 GNSS position 的可用性必须分开判断；heading 可用不等于 position 可用。
4. GNSS 观测是否注入 DR/融合，应同时受 fix 类型、协方差、场景状态和时间同步约束。
5. DR 输出必须携带协方差，便于 `pose_fusion` 判断短时可靠性。

---

## 7. `pose_fusion` 模块设计

### 7.1 当前状态

`pose_fusion` 当前为占位节点，尚未实现 ESKF/UKF 或其他融合核心。

### 7.2 模块定位

`pose_fusion` 是定位功能域的最终位姿输出模块。它融合 DR 高频预测、SLAM 低频观测、GNSS 场景化观测和重定位事件，输出规划/控制使用的连续定位结果。

### 7.3 推荐核心接口

```cpp
class IPoseFusion {
public:
  virtual ~IPoseFusion() = default;
  virtual bool initialize(const localization::FusionConfig& config) = 0;
  virtual void feedDrState(const localization::DrState& dr) = 0;
  virtual void feedSlamOdom(const localization::OdomResult& slam) = 0;
  virtual void feedGnss(const localization::GnssData& gnss) = 0;
  virtual void feedRelocalizationResult(const RelocalizationObservation& reloc) = 0;
  virtual void setSceneState(localization::SceneState scene) = 0;
  virtual bool processOnce(double now) = 0;
  virtual localization::OdomResult getFusedOdometry() const = 0;
  virtual localization::ObserverWeights getObserverWeights() const = 0;
  virtual FusionHealth getHealth() const = 0;
  virtual void reset(const localization::Pose& initial_pose) = 0;
};
```

### 7.4 观测源分析

| 观测源 | 频率 | 主要优点 | 主要风险 | 融合使用方式 |
|---|---:|---|---|---|
| DR | 50-100Hz | 连续、低延迟 | 长时漂移 | 预测或短时约束 |
| SLAM | 10-20Hz | 局部精度高 | 退化、动态物体、无纹理/几何重复 | 位姿观测，退化时增大噪声 |
| GNSS position | 1-20Hz | 露天全局约束 | 井下不可用、多路径、跳变 | 仅露天/过渡且质量可靠时使用 |
| GNSS heading | 1-20Hz | yaw 全局约束 | 低速或遮挡时不稳定 | 有效时作为方向观测 |
| Relocalization | 事件触发 | 全局恢复 | 错配会造成大跳变 | 通过验真和协方差门限后 reset 或校正 |

### 7.5 场景化 GNSS 策略

定位模块应支持三类场景：

| 场景 | GNSS 使用策略 | 推荐观测权重 |
|---|---|---|
| `OPEN_PIT` 露天 | GNSS position 和 heading 可参与融合，但需质量门限 | GNSS 高，SLAM/DR 正常 |
| `TRANSITION` 过渡 | GNSS 权重渐退或渐入，避免井口跳变 | GNSS 中低，SLAM/DR 高 |
| `UNDERGROUND` 井下 | 默认不使用 GNSS position；heading 仅在明确有效且策略允许时使用 | GNSS 0，SLAM/DR 高 |

GNSS 接入判断至少包含：

1. fix 类型：无解、float、fixed 分级处理。
2. position covariance 或驱动提供的精度指标。
3. heading 是否有效及 heading 标准差。
4. 观测时间戳与当前融合时间差。
5. 与当前融合状态的残差门限，超限观测拒绝。
6. 场景状态和过渡时间。

### 7.6 ROS 输入输出

推荐订阅：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/localization/dr_odom` | `nav_msgs/Odometry` | DR 高频预测 |
| `/localization/slam_odom` | `nav_msgs/Odometry` | SLAM 低频观测 |
| `/localization/slam_health` | `deeplumin_msgs/SlamHealth` | SLAM 退化信息 |
| `/gnss_chc_data` 或 `/sensor/gnss/fix` | GNSS 消息 | GNSS position/heading |
| `/localization/reloc_result` | 建议新增消息 | 重定位事件 |
| `/system/scene_state` | 场景状态 | GNSS 权重策略 |

推荐发布：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/localization/fused_odom` | `nav_msgs/Odometry` 和/或 `deeplumin_msgs/FusedOdometry` | 最终定位结果 |
| `/localization/status` | `deeplumin_msgs/LocalizationStatus` | 融合状态 |
| `/localization/covariance` | 可视化消息 | 协方差调试 |
| TF | `map -> odom`、`odom -> base_link` | 完整定位 TF |

### 7.7 开发要求

1. `pose_fusion` 是最终定位输出权威节点，规划/控制应优先消费其输出。
2. 所有观测源必须有时间戳检查和超时处理。
3. SLAM 退化时不能简单丢弃，应根据退化程度调大协方差；严重退化或连续退化再触发重定位。
4. GNSS 使用必须由场景状态控制，不能在井下默认注入。
5. 重定位结果应通过质量门限、残差门限、平滑过渡策略后再影响最终位姿。

---

## 8. `global_matcher` 模块分析

### 8.1 当前状态

`global_matcher` 当前为占位节点。由于 `relocalization` 已经包含 `IGlobalMatcher` 和 `GicpGlobalMatcher`，需要明确二者关系，避免重复开发。

### 8.2 推荐职责边界

有两种合理方案：

#### 方案 A：保留为独立全局匹配服务

`global_matcher` 专注提供点云到地图/子图的匹配服务，不负责 Scan Context 检索：

```text
输入：source 点云、target 地图/子图、初始位姿
输出：精匹配位姿、fitness、inlier ratio、协方差估计
```

`relocalization` 调用它完成候选精匹配。

#### 方案 B：合并到 `relocalization`

将 `global_matcher` 作为后续保留包，不单独运行；全局检索和精匹配均由 `relocalization` 内部完成。

当前更推荐方案 B，原因是已有 `relocalization/interface/i_global_matcher.hpp` 和 `matcher/gicp_global_matcher`，功能已经覆盖全局匹配核心。

### 8.3 若独立实现的接口

```cpp
class IGlobalMatcherService {
public:
  virtual bool initialize(const GlobalMatcherServiceConfig& config) = 0;
  virtual MatchResult align(
      const localization::TimestampedPointCloud& source,
      const localization::TimestampedPointCloud& target,
      const Eigen::Matrix4d& initial_guess) = 0;
};
```

推荐发布服务：

| 服务 | 说明 |
|---|---|
| `/localization/global_matcher/align` | 输入 source/target/initial_guess，输出 transform 和质量 |
| `/localization/global_matcher/load_map` | 加载目标地图 |

---

## 9. `localization_manager` 模块设计

### 9.1 当前状态

`localization_manager` 当前为占位节点。

### 9.2 模块定位

`localization_manager` 是定位功能域的运行编排和状态管理节点，不直接实现核心定位算法。它负责：

1. 汇总 DR、SLAM、融合、GNSS、重定位健康状态。
2. 根据场景状态切换 GNSS 使用策略。
3. 触发重定位：人工触发、启动触发、退化触发、协方差触发。
4. 管理运行模式：`LOCALIZATION`、`MAPPING`、`RELOCALIZATION`、`PURE_SLAM`。
5. 向系统层发布统一 `LocalizationStatus`。

### 9.3 状态机

推荐模式：

| 模式 | 说明 |
|---|---|
| `PURE_SLAM` | 单独运行 SLAM 调试，不输出最终融合定位 |
| `LOCALIZATION` | 正常定位，DR + SLAM + Fusion |
| `RELOCALIZATION` | 重定位搜索/匹配中 |
| `MAPPING` | 建图或关键帧数据库生成 |
| `DEGRADED` | 定位降级，仍可输出但质量下降 |
| `FAILURE` | 定位失败，规划/控制应进入安全策略 |

推荐触发条件：

| 触发 | 条件 | 行为 |
|---|---|---|
| 启动重定位 | 有先验地图且无可靠初始位姿 | 调用 relocalization |
| SLAM 连续退化 | `is_degenerated` 连续超过阈值 | 降低 SLAM 权重，必要时触发重定位 |
| 融合协方差过大 | position/yaw covariance 超阈值 | 触发重定位或请求人工介入 |
| GNSS 进入可用 | 场景从井下到露天且 GNSS fixed | 渐进增加 GNSS 权重 |
| GNSS 失效 | 进入井下或质量下降 | 渐进降低 GNSS 权重 |
| 人工触发 | 服务调用 | 强制进入 `RELOCALIZATION` |

### 9.4 ROS 输入输出

推荐订阅：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/localization/dr_health` | 建议新增 | DR 状态 |
| `/localization/slam_health` | `deeplumin_msgs/SlamHealth` | SLAM 状态 |
| `/localization/fused_odom` | `nav_msgs/Odometry` | 最终定位 |
| `/localization/status` | `deeplumin_msgs/LocalizationStatus` | 可由 fusion 发布，也可 manager 聚合 |
| `/system/scene_state` | 场景状态 | GNSS 策略 |

推荐发布：

| 话题/服务 | 类型 | 说明 |
|---|---|---|
| `/localization/manager/status` | `deeplumin_msgs/LocalizationStatus` | 聚合状态 |
| `/localization/set_mode` | Service | 模式切换 |
| `/localization/trigger_relocalization` | Service | 触发重定位 |
| `/localization/reset` | Service | 定位 reset |

---

## 10. 模块间接口矩阵

| 生产者 | 消费者 | 数据 | ROS 话题/服务 | 说明 |
|---|---|---|---|---|
| sensing/IMU | `slam_odometry` | IMU | `/ouster/imu` | SLAM 点云去畸变和预测 |
| sensing/LiDAR | `slam_odometry` | 点云 | `/ouster/points` | SLAM 主输入 |
| sensing/IMU | `dr_odometry` | IMU | `/ouster/imu` | DR 主输入 |
| vehicle_interface | `dr_odometry` | CAN speed | `/can_receive_info` | DR 速度约束 |
| sensing/GNSS | `dr_odometry` | heading/position | `/gnss_chc_data` | 有 GNSS 时辅助 DR |
| `dr_odometry` | `pose_fusion` | DR odom | `/localization/dr_odom` | 高频预测 |
| `slam_odometry` | `pose_fusion` | SLAM odom | `/localization/slam_odom` | 低频观测 |
| `slam_odometry` | `relocalization` | 当前扫描 | `/localization/cloud_registered` 或原始点云 | 重定位 query |
| `relocalization` | `pose_fusion` | 全局位姿 | `/localization/reloc_result` | 重定位事件 |
| sensing/GNSS | `pose_fusion` | GNSS position/heading | `/gnss_chc_data` | 场景化全局观测 |
| system/scene | `pose_fusion` | 场景状态 | `/system/scene_state` | GNSS 权重切换 |
| `pose_fusion` | planning/control | 最终定位 | `/localization/fused_odom` | 下游主输入 |
| `localization_manager` | system/planning/control | 定位状态 | `/localization/manager/status` | 健康与降级状态 |

---

## 11. 场景化定位策略

### 11.1 露天场景

露天场景 GNSS 一般可用，但仍需做质量门限：

1. GNSS fixed 且 covariance 小时，GNSS position 可作为全局位置观测。
2. GNSS heading 有效时，可作为 yaw 观测注入 DR 或 Fusion。
3. SLAM 仍作为局部约束，避免 GNSS 短时抖动影响控制。
4. CAN speed 继续作为纵向速度约束。

输出策略：

```text
DR(IMU + speed + GNSS heading)
  + SLAM local odom
  + GNSS position
  -> pose_fusion
  -> fused_odom
```

### 11.2 井下场景

井下 GNSS 默认不可用：

1. GNSS position 权重为 0。
2. GNSS heading 默认不使用，除非明确有外部高质量 heading 来源。
3. 定位主要依赖 SLAM + DR。
4. 重定位依赖先验点云地图、Scan Context 和 GICP/NDT。

输出策略：

```text
DR(IMU + speed)
  + SLAM local odom
  + event-based relocalization
  -> pose_fusion
  -> fused_odom
```

### 11.3 过渡场景

过渡场景是 GNSS 使用最容易出问题的区域：

1. GNSS 权重应渐变，不应在井口瞬间切换。
2. GNSS position 残差过大时拒绝观测。
3. heading 可先于 position 使用，但同样需要质量判断。
4. 若 SLAM 与 GNSS 差异过大，应触发状态检查或重定位，不应盲目融合。

推荐权重：

```text
OPEN_PIT      : gnss_weight = high
TRANSITION    : gnss_weight = fade in/out
UNDERGROUND   : gnss_weight = 0
```

---

## 12. 分阶段开发建议

### P0：接口和消息补齐

1. 在 `dr_odometry` 中新增 DR 核心接口和 ROS adapter。
2. 在 `pose_fusion` 中新增融合接口和状态结构。
3. 新增结构化 `RelocalizationResult`、`DrHealth` 或等价消息。
4. 明确 `can_receive_info` 的消息类型和速度单位转换。

### P1：DR 基础实现

1. 实现 IMU propagation。
2. 实现 CAN speed 速度观测。
3. 实现静止检测和基础 bias 估计。
4. 预留 GNSS heading 输入，不强依赖具体驱动。
5. 发布 `/localization/dr_odom` 和健康状态。

### P2：Fusion 基础实现

1. 以 DR 为高频预测。
2. 以 SLAM 为低频观测。
3. 输出 `/localization/fused_odom`。
4. 根据 SLAM 退化信息调节观测噪声。

### P3：GNSS 场景化接入

1. 接入 GNSS position/heading adapter。
2. 实现场景状态机：露天、过渡、井下。
3. 实现 GNSS 质量门限、残差门限和渐变权重。

### P4：重定位闭环

1. 将 `relocalization` 输出改为结构化结果。
2. `localization_manager` 根据退化/协方差/人工命令触发重定位。
3. `pose_fusion` 接收重定位事件并平滑 reset 或校正。

### P5：建图与地图数据库

1. 复用 `slam_odometry` 输出关键帧。
2. 生成关键帧库、Scan Context 数据库和全局地图。
3. 建立地图版本、地图切换和子图加载机制。

---

## 13. 关键开发约束

1. 各模块必须可单独作为 ROS 节点运行，不允许只能在单进程大节点内使用。
2. 模块之间通过 ROS 话题/服务或清晰 C++ 接口互通，避免直接访问对方内部状态。
3. `slam_odometry`、`dr_odometry`、`pose_fusion` 的输出都必须带时间戳和 frame_id。
4. 最终给规划/控制使用的定位以 `pose_fusion` 输出为准。
5. GNSS 使用必须区分露天、过渡、井下场景，不能在定位链路中写死 always on。
6. GNSS heading、GNSS position、CAN speed 是不同观测，必须分别判断有效性。
7. 重定位是事件触发的全局校正，不应替代连续定位主链路。
8. 退化、协方差过大、输入超时必须进入 `LocalizationStatus`。
9. 参数路径不能硬编码到代码，地图、描述子库、topic、frame 都应来自 YAML/launch。
10. 新增算法实现必须挂在抽象接口后面，避免上层节点依赖具体算法类。

---

## 14. 当前实现与目标状态差距

| 模块 | 当前状态 | 目标状态 | 优先级 |
|---|---|---|---|
| `slam_odometry` | 已有 Faster-LIO 核心、接口、ROS 节点 | 补健康消息、动态 mask、融合专用输出 | 高 |
| `relocalization` | 已有 Scan Context + GICP、在线节点和 bag eval | 结构化结果、服务触发、融合闭环 | 高 |
| `dr_odometry` | 占位节点 | IMU + CAN speed + GNSS heading/position 预留 | 高 |
| `pose_fusion` | 占位节点 | DR/SLAM/GNSS/Reloc 融合输出 | 高 |
| `global_matcher` | 占位节点 | 明确合并或独立匹配服务 | 中 |
| `localization_manager` | 占位节点 | 模式管理、健康聚合、重定位触发、场景策略 | 高 |
| `deeplumin_msgs` | 已有基础定位消息 | 补重定位、DR、服务消息 | 中 |

---

## 15. 推荐最终运行组合

### 15.1 井下定位

```text
slam_odometry_node
dr_odometry_node
pose_fusion_node
relocalization/global_relocalization_node
localization_manager_node
```

数据组合：

```text
IMU + LiDAR -> slam_odometry -> slam_odom
IMU + CAN speed -> dr_odometry -> dr_odom
slam_odom + dr_odom + relocalization event -> pose_fusion -> fused_odom
manager monitors health and triggers relocalization
```

### 15.2 露天定位

```text
slam_odometry_node
dr_odometry_node
pose_fusion_node
localization_manager_node
```

数据组合：

```text
IMU + CAN speed + GNSS heading -> dr_odometry
GNSS position + SLAM + DR -> pose_fusion
scene_state controls GNSS weight
```

### 15.3 调试模式

单独 SLAM：

```bash
roslaunch slam_odometry slam_odometry_ysw.launch
```

单独重定位：

```bash
roslaunch relocalization global_relocalization.launch
```

后续应补齐：

```bash
roslaunch dr_odometry dr_odometry.launch
roslaunch pose_fusion pose_fusion.launch
roslaunch localization_manager localization_manager.launch
```

---

## 16. 结论

DeepLumin 定位模块已经具备两个较完整的基础能力：`slam_odometry` 提供接口化 Faster-LIO 局部里程计，`relocalization` 提供 Scan Context + GICP 全局重定位。后续开发重点应放在三件事：

1. 补齐 `dr_odometry`，明确预留 GNSS、IMU、CAN receive info 三类输入；有 GNSS 时用 heading 与 CAN speed 约束 IMU，无 GNSS 时以 speed + IMU + 非完整约束实现短时航位推算。
2. 补齐 `pose_fusion`，以 DR 高频预测、SLAM 低频观测、GNSS 场景化观测、重定位事件构成完整定位输出。
3. 补齐 `localization_manager`，统一模式管理、健康状态、场景切换和重定位触发，使各模块既可单独运行，也能通过数据互通完成完整定位功能。
