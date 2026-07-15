# DeepLumin pose_fusion 详细开发设计文档

> 适用模块：`DeepLumin/src/localization/pose_fusion`  
> 协作模块：`dr_odometry`、`slam_odometry`、`global_matcher`、`relocalization`、`localization_manager`  
> 目标：实现 DeepLumin 定位系统的最终位姿融合输出，使定位效果至少达到 `ysw_loc` 中“里程计连续性 + 地图匹配校正”的工程效果，并保留露天、半开放、室内、GNSS 失效等场景的扩展能力。

---

## 0. 与当前 global_matcher 实现的同步

`global_matcher` 当前已经具备可运行实现，`pose_fusion` 开发时应直接按以下事实接入，而不是重新定义一套地图匹配接口：

1. 地图匹配结果话题：`/localization/global_matcher/result`。
2. 消息类型：`deeplumin_msgs/GlobalMatchResult`，语义是 `T_map_base_match`。
3. `global_matcher` 默认使用 `fast_gicp::FastGICP`，保留 `gicp`/`icp` 配置回退。
4. 当前匹配结果包含 `map_id`、`success`、`converged`、`fitness_score`、`inlier_ratio`、`inlier_rmse`、`initial_translation_error`、`initial_yaw_error`。
5. `global_matcher` 不发布最终 TF，也不修改 `map->odom`。`pose_fusion` 仍是唯一最终 TF owner。
6. RViz 调试点云 `source_scan_initial`、`target_submap`、`aligned_scan` 只用于诊断，不作为 fusion 输入。

因此，`pose_fusion` 的 P3 开发应优先完成 `GlobalMatchResult -> PoseObservation -> map->odom smooth update`，这是达到 ysw_loc 定位效果的关键闭环。

## 1. 模块定位

`pose_fusion` 是 localization 中唯一负责输出最终定位结果的模块。它不直接做点云匹配、不直接做全局重定位、不直接处理原始 IMU 预积分，而是接收各定位子模块的结果，将它们统一成观测并进行时序融合、质量判断、状态切换和 TF 发布。

总体链路：

```text
dr_odometry
  -> 高频短时连续预测

slam_odometry
  -> 局部 LiDAR-IMU 里程计约束

global_matcher
  -> 基于初值的地图/子图匹配校正

relocalization
  -> 全局丢失后的恢复结果，由 localization_manager 验收后进入 pose_fusion

GNSS
  -> 露天场景的绝对位置/航向观测，可由 localization_manager 按场景启停或降权

pose_fusion
  -> /localization/fused_odom
  -> /localization/fused_pose
  -> map -> odom -> base_link TF
  -> fusion status / diagnostics / path
```

`pose_fusion` 与 `ysw_loc` 的核心对应关系：

| ysw_loc 能力 | DeepLumin 对应设计 |
|---|---|
| `laser_mapping` 输出连续局部 odom | `slam_odometry` 输出局部里程计观测 |
| 全局定位节点根据 odom 初值做地图匹配 | `global_matcher` 输出地图校正观测 |
| 通过匹配结果更新全局位姿 | `pose_fusion` 平滑更新 `map -> odom` 或融合状态 |
| 失败时依赖人工/重定位恢复 | `localization_manager` 触发 `relocalization`，验收后交给 `pose_fusion` reset |

DeepLumin 不应让 `global_matcher` 或 `relocalization` 直接发布最终 TF。所有影响最终定位的结果必须进入 `pose_fusion`，由统一的质量门控和状态机处理。

---

## 2. 设计目标

1. 最终定位效果不低于 ysw_loc：在有先验地图和可用点云匹配的情况下，车辆轨迹应稳定贴合地图，局部连续性和全局一致性同时满足。
2. 各模块可独立运行：`dr_odometry`、`slam_odometry`、`global_matcher`、`relocalization` 均可单独启动、调试和替换。
3. 融合接口统一：所有外部结果进入 `pose_fusion` 前转换为 `PoseObservation`、`VelocityObservation`、`ResetObservation` 或 `HealthObservation`。
4. 场景可切换：露天场景可使用 GNSS，室内或 GNSS 不可靠场景禁用/降权 GNSS，并依赖 DR + SLAM + 地图匹配。
5. 故障可解释：每个观测必须记录是否使用、拒绝原因、协方差、时间戳、来源模块和质量指标。
6. 输出稳定：周期性发布最终 odom 和 TF，地图校正不能造成高频跳变；只有 manager 验收的重定位允许硬 reset。

---

## 3. 总体架构

```text
pose_fusion_node
  |
  |-- RosAdapter
  |   |-- 订阅 dr/slam/global_matcher/relocalization/GNSS/manager policy
  |   |-- 发布 fused odom/status/path/TF/debug
  |
  |-- ObservationBuffer
  |   |-- 按时间戳缓存各来源观测
  |   |-- 做超时、乱序、插值和来源有效性检查
  |
  |-- FusionCore
  |   |-- StateEstimator
  |   |   |-- DR delta propagation
  |   |   |-- pose update
  |   |   |-- velocity update
  |   |   `-- reset / smooth correction
  |   |-- CovarianceManager
  |   |-- QualityGate
  |   `-- FusionStateMachine
  |
  |-- TfManager
  |   |-- map -> odom
  |   `-- odom -> base_link / base_link pose query
  |
  `-- Diagnostics
      |-- observation usage
      |-- delay / rate / covariance
      `-- degraded reason
```

推荐包结构：

```text
pose_fusion/
├── CMakeLists.txt
├── package.xml
├── include/pose_fusion/
│   ├── common/
│   │   ├── types.hpp
│   │   ├── config.hpp
│   │   └── math_utils.hpp
│   ├── core/
│   │   ├── fusion_core.hpp
│   │   ├── observation_buffer.hpp
│   │   ├── quality_gate.hpp
│   │   ├── covariance_manager.hpp
│   │   └── fusion_state_machine.hpp
│   ├── interface/
│   │   ├── i_pose_fusion.hpp
│   │   └── i_observer_model.hpp
│   └── ros/
│       ├── ros_adapter.hpp
│       ├── param_loader.hpp
│       └── tf_manager.hpp
├── src/
│   ├── core/
│   ├── ros/
│   └── pose_fusion_node.cpp
├── config/pose_fusion.yaml
├── launch/pose_fusion.launch
└── test/
```

---

## 4. 坐标系和 TF 职责

统一坐标系：

| Frame | 语义 |
|---|---|
| `map` | 全局地图坐标系，地图匹配、GNSS 和重定位结果所在坐标系 |
| `odom` | 局部连续坐标系，短时间内连续平滑，可随全局校正缓慢变化 |
| `base_link` | 车体坐标系 |
| `lidar_link` | LiDAR 坐标系 |
| `imu_link` | IMU 坐标系 |
| `gnss_sensor` | GNSS 天线坐标系 |

TF 发布原则：

1. `pose_fusion` 是最终 `map -> odom` 的唯一发布者。
2. `slam_odometry` 和 `dr_odometry` 可以发布调试 TF，但默认不能覆盖最终定位 TF。
3. `global_matcher` 只发布匹配结果和可视化，不直接修改 `map -> odom`。
4. `relocalization` 只发布候选结果，只有 `localization_manager` 验收后才能触发 `pose_fusion` reset。

推荐维护方式：

```text
T_map_base_fused = T_map_odom * T_odom_base_current
```

当 `global_matcher` 给出 `T_map_base_match` 时，计算观测到的全局校正：

```text
T_map_odom_observed = T_map_base_match * inverse(T_odom_base_current)
```

注意：当前 `global_matcher` 的 `source_cloud_is_odom_frame=true` 在线模式会使用 `/localization/fused_odom` 与 `/localization/slam_odom` 形成初值。`pose_fusion` 必须稳定发布 `/localization/fused_odom`，否则 global_matcher 会退化为缺少可靠初值的匹配，错误率会显著增加。

正常定位中优先平滑更新 `T_map_odom`，保证 `odom -> base_link` 的短时连续性；发生重定位时才允许硬 reset。

---

## 5. 状态定义

`pose_fusion` 推荐使用 15 维误差状态 ESKF 或等价的误差状态融合框架。由于 IMU 原始积分已经在 `dr_odometry` 中处理，`pose_fusion` 的传播可以先以 DR delta 为主，后续再扩展为直接 IMU 传播。

名义状态：

```cpp
struct FusionState {
  double timestamp = 0.0;
  Eigen::Vector3d p_map = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_map = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_map = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> covariance;
};
```

误差状态：

```text
delta_x = [delta_p, delta_v, delta_theta, delta_bg, delta_ba]
```

初期实现可分两阶段：

| 阶段 | 传播方式 | 更新方式 | 适用目标 |
|---|---|---|---|
| P0 | DR odom delta | pose/velocity 观测更新 | 快速达到 ysw_loc 同等效果 |
| P1 | IMU 或 DR motion model | pose/velocity/yaw 观测更新 | 更强鲁棒性和长时间 GNSS 失效能力 |

P0 阶段不要把实现做复杂。先保证 DR 连续预测 + SLAM/GLOBAL 校正 + reset 机制正确，再扩展完整 IMU 状态传播。

---

## 6. 输入接口

### 6.1 DR 输入

`dr_odometry` 是融合主预测源，提供高频、连续、低延迟的短时运动约束。

建议 topic：

| Topic | Type | 用途 |
|---|---|---|
| `/localization/dr_odom` | `nav_msgs/Odometry` 或 `deeplumin_msgs/FusedOdometry` | DR 位姿/速度 |
| `/localization/dr_status` | `deeplumin_msgs/LocalizationStatus` | DR 健康状态 |

处理方式：

```text
T_dr_delta = inverse(T_dr_last) * T_dr_now
T_pred = T_fused_last * T_dr_delta
```

注意：

1. DR 的坐标系通常是局部 `odom`，不要直接认为它在 `map` 下绝对正确。
2. 只有 DR 时间戳连续、速度方向可信、状态健康时才用于传播。
3. 当 DR 重启或跳变时，需要清除 `last_dr_pose`，等待重新初始化。

### 6.2 SLAM 输入

`slam_odometry` 提供 LiDAR-IMU 局部里程计，主要用于局部约束和退化状态判断。

建议 topic：

| Topic | Type | 用途 |
|---|---|---|
| `/localization/slam_odom` | `nav_msgs/Odometry` 或 `deeplumin_msgs/SlamPose` | SLAM 位姿观测 |
| `/localization/slam_health` | `deeplumin_msgs/SlamHealth` | 点云匹配质量、退化、有效点数 |

处理方式：

1. 若 SLAM 输出为 `odom` 局部坐标，作为局部相对约束或速度/增量约束使用。
2. 若 SLAM 已经通过地图初始化到 `map`，可作为 `PoseObservation` 使用。
3. SLAM 退化时只增大协方差，不应直接关闭；连续严重退化再由状态机降级。

SLAM 是否可以融合到 DR：

从理论上可行，SLAM 位姿可以作为外部观测反向校正 DR 的姿态、速度或 bias，使 DR 在 GNSS 失效时更稳定。但工程上不建议默认把 SLAM 写回 `dr_odometry` 内部滤波器，原因是：

1. DR 应保持原始轮速/IMU/GNSS 融合语义，便于独立调试。
2. SLAM 退化和地图匹配失败时，若直接写回 DR，错误会污染预测源。
3. `pose_fusion` 已经是融合中心，把 SLAM 作为观测进入融合更清晰。

推荐方案：

| 模式 | 说明 | 默认 |
|---|---|---|
| `slam_to_pose_fusion` | SLAM 只进入 `pose_fusion` 更新最终位姿 | 是 |
| `slam_feedback_to_dr_reset` | 仅在 manager 允许时重置 DR 初值 | 可选 |
| `slam_feedback_to_dr_filter` | SLAM 作为 DR 内部滤波观测 | 暂不默认 |

### 6.3 global_matcher 输入

`global_matcher` 提供 ysw_loc 风格的“基于初值的地图/子图匹配校正”，是达到 ysw_loc 效果的关键观测。

建议 topic：

| Topic | Type | 用途 |
|---|---|---|
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | 地图匹配结果 |
| `/localization/global_matcher/status` | `deeplumin_msgs/LocalizationStatus` | 匹配状态 |

当前实际字段：

```text
std_msgs/Header header              # map frame
string child_frame_id               # usually base_link
geometry_msgs/PoseWithCovariance pose
bool success
bool converged
string map_id
string reject_reason
float64 fitness_score
float64 inlier_ratio
float64 inlier_rmse
uint32 inlier_count
uint32 source_points
uint32 target_points
float64 elapsed_ms
float64 initial_translation_error
float64 initial_yaw_error
```

处理策略：

1. `success=false` 或 `converged=false` 只记录诊断，不更新滤波器。
2. `map_id` 必须与当前 manager/pose_fusion 的 active map 一致，否则拒绝，避免地图段切换期间混用结果。
3. `pose` 是 `T_map_base_match`，需要结合当前 `T_odom_base_current` 计算 `T_map_odom_observed`。
4. 小误差校正使用平滑更新或 ESKF pose update。
5. 大误差但匹配质量非常高时，交给 `localization_manager` 判断是否触发重定位或受控 reset。
6. `fitness_score` 不同 matcher 后端数值尺度不同；`fast_gicp` 下不能沿用 PCL GICP 的固定阈值，必须结合 `inlier_ratio`、`inlier_rmse` 和初值误差综合判断。

### 6.4 relocalization 输入

`relocalization` 的结果只能作为恢复事件使用，不参与普通周期性融合。

推荐链路：

```text
relocalization candidate
  -> localization_manager quality check
  -> /localization/manager/accepted_relocalization
  -> pose_fusion HARD_RESET
```

`pose_fusion` 不直接订阅裸 relocalization candidate，除非是调试模式。

### 6.5 GNSS 输入

GNSS 的使用必须区分场景：

| 场景 | GNSS 使用方式 |
|---|---|
| 露天且 RTK 固定 | 可作为绝对位置和航向强观测 |
| 露天但质量一般 | 只作为低权重位置观测，航向按质量判断 |
| 半开放遮挡 | 低频辅助，不允许直接 reset |
| 室内/隧道/无效 | 禁用，仅保留状态监控 |

建议 topic：

| Topic | Type | 用途 |
|---|---|---|
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | GNSS 原始数据 |
| `/localization/gnss_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 可选的标准化 GNSS 位姿 |

GNSS 不应绕过 `localization_manager` 场景策略。推荐由 manager 发布 `ObserverPolicy`：

```text
gnss.enabled
gnss.position_weight
gnss.heading_weight
gnss.min_fix_quality
gnss.max_age
```

---

## 7. 统一观测模型

所有输入先转换为统一结构：

```cpp
enum class ObservationSource {
  DR,
  SLAM,
  GLOBAL_MATCHER,
  RELOCALIZATION,
  GNSS,
  MANUAL
};

enum class ObservationType {
  POSE_ABSOLUTE,
  POSE_RELATIVE,
  VELOCITY,
  YAW,
  RESET
};

struct PoseObservation {
  ObservationSource source;
  ObservationType type;
  double timestamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double, 6, 6> covariance;
  std::string frame_id;
  std::string child_frame_id;
  double quality_score = 0.0;
  bool valid = false;
  std::string reject_reason;
};

struct VelocityObservation {
  ObservationSource source;
  double timestamp = 0.0;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance;
  std::string frame_id;
  bool valid = false;
};

struct ResetObservation {
  ObservationSource source;
  double timestamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double, 6, 6> covariance;
  std::string reason;
  std::string approved_by;
};
```

融合核心只依赖这些结构，不依赖 ROS message 类型。

---

## 8. 融合算法

### 8.1 主循环

推荐频率：

| 功能 | 频率 |
|---|---|
| DR 输入 | 50-200 Hz |
| 融合输出 | 50-100 Hz |
| SLAM 更新 | 5-20 Hz |
| global_matcher 更新 | 0.5-2 Hz |
| relocalization | 事件触发 |
| GNSS | 1-20 Hz |

主循环：

```text
while ros ok:
  1. 从 ObservationBuffer 取出当前时间窗口内的 DR delta
  2. 使用 DR delta 传播 fused state
  3. 处理 SLAM 观测
  4. 处理 GNSS 观测
  5. 处理 global_matcher 观测
  6. 处理 manager accepted relocalization reset
  7. 更新 fusion state machine
  8. 发布 fused_odom / TF / status / debug
```

### 8.2 DR delta 传播

第一阶段建议使用 DR 位姿增量传播：

```text
T_delta = inverse(T_dr_last) * T_dr_now
T_fused_pred = T_fused_last * T_delta
```

速度更新：

```text
v_pred = R_fused * v_dr_body
```

如果 DR 只提供速度而无可靠位姿，则使用：

```text
delta_s = speed * dt
delta_p = R_fused * [delta_s, 0, 0]^T
q_pred = q_fused * Exp(omega_z * dt)
```

这里的方向来自当前融合状态或 IMU/SLAM/GNSS 观测后的姿态，而不是来自标量 speed 本身。标量 speed 只能约束前向速度，不能单独提供 heading。

### 8.3 pose update

绝对位姿观测残差：

```text
r_p = p_obs - p_pred
r_theta = Log(q_pred.inverse() * q_obs)
r = [r_p, r_theta]
```

观测矩阵：

```text
H_pose = [I3, 03, 03, 03, 03
          03, 03, I3, 03, 03]
```

更新：

```text
S = H * P * H^T + R
K = P * H^T * inverse(S)
delta_x = K * r
inject(delta_x)
P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
```

必须使用 Joseph form 更新协方差，避免数值发散。

### 8.4 velocity update

速度观测残差：

```text
r_v = v_obs - v_pred
H_v = [03, I3, 03, 03, 03]
```

轮速或 DR 的前向速度观测如果进入 `pose_fusion`，应转换到对应 frame：

```text
v_body_obs = [speed, 0, 0]
v_map_obs = R_map_base * v_body_obs
```

倒车档位需要处理符号：

```text
if gear == reverse:
  speed = -abs(speed)
else:
  speed = abs(speed)
```

### 8.5 global_matcher 平滑校正

`global_matcher` 是 ysw_loc 对齐地图效果的关键，但它低频且可能有偶发误匹配，因此不能简单瞬时跳变。

推荐三种策略：

| 策略 | 使用条件 | 行为 |
|---|---|---|
| `SMOOTH_UPDATE` | 正常地图匹配，小中等残差 | 作为 pose observation 更新滤波状态或平滑更新 `map -> odom` |
| `SOFT_RESET` | 残差较大但连续多帧一致 | 在 0.5-2 秒内插值修正 `map -> odom` |
| `HARD_RESET` | manager 验收的重定位 | 直接重置融合状态和 `map -> odom` |

平滑更新建议：

```text
alpha = clamp(config.global_matcher.smooth_gain, 0.02, 0.3)
p_new = (1 - alpha) * p_old + alpha * p_match
q_new = slerp(q_old, q_match, alpha)
```

如果使用 ESKF 观测更新，则通过调节 `R_global_matcher` 实现平滑，不再额外插值。工程上建议 P0 阶段先实现 `map -> odom` 平滑修正，P1 阶段再切换为完整 ESKF 更新。

---

## 9. 质量门控和协方差

### 9.1 通用门控

每个观测必须经过以下检查：

1. 时间戳是否有效，是否超时。
2. frame 是否符合预期，TF 是否可查询。
3. 协方差是否有限且正定。
4. 与当前预测状态的残差是否超过阈值。
5. 来源模块 health 是否允许使用。

马氏距离门控：

```text
d2 = r^T * inverse(S) * r
accept if d2 < chi_square_threshold
```

建议阈值：

| 观测维度 | 置信度 | chi-square |
|---|---|---|
| 3D position | 99% | 11.34 |
| 6D pose | 99% | 16.81 |

### 9.2 SLAM 协方差策略

SLAM 协方差根据健康状态动态缩放：

```text
scale = 1.0
if feature_points < min_points: scale *= 5
if degeneracy: scale *= 10
if scan_match_score poor: scale *= 3
R_slam = base_R_slam * scale
```

SLAM 只在短时退化时降权；连续退化超过阈值才进入 `DEGRADED`。

### 9.3 global_matcher 协方差策略

地图匹配观测协方差应由质量指标推导：

```text
quality_scale = 1.0
quality_scale *= clamp(fitness / accept_fitness, 1.0, 10.0)
quality_scale *= clamp(min_inlier_ratio / inlier_ratio, 1.0, 10.0)
quality_scale *= clamp(inlier_rmse / target_rmse, 1.0, 10.0)
R_global = base_R_global * quality_scale
```

直接拒绝条件：

| 条件 | 默认阈值 |
|---|---|
| `success == false` 或 `converged == false` | 拒绝 |
| source points 过少 | `< 200` |
| target points 过少 | `< 1000` |
| fitness 过大 | 按 matcher 和地图标定；fast_gicp 的数值尺度不能沿用 PCL GICP 固定阈值 |
| inlier ratio 过低 | 默认 `< 0.5` 拒绝，当前 keyframe 验证正常可到 `1.0` |
| inlier RMSE 过大 | 默认 `> 0.8 m` 拒绝，当前 keyframe 验证约 `0.068 m` |
| 与预测残差过大 | 普通校正拒绝；大误差只允许 manager 验收后 reset |
| yaw 残差过大 | 普通校正拒绝；阈值按车速和场景标定 |

### 9.4 GNSS 协方差策略

GNSS 应由定位质量、卫星状态、解状态和场景策略共同决定权重。

```text
if gnss disabled by manager:
  reject
elif fix quality high and heading valid:
  use position + yaw
elif position valid but heading invalid:
  use position only
else:
  reject
```

GNSS 不允许在室内场景直接 reset。室内误用 GNSS 是定位跳变的高风险来源。

---

## 10. 状态机

`pose_fusion` 内部状态：

| 状态 | 含义 | 进入条件 | 退出条件 |
|---|---|---|---|
| `UNINITIALIZED` | 未初始化 | 节点启动、reset 后无初值 | 收到有效初值 |
| `DR_ONLY` | 仅 DR 可用 | 有 DR，无 SLAM/GLOBAL/GNSS | 收到稳定 SLAM 或 GLOBAL |
| `LOCAL_FUSION` | DR + SLAM 局部融合 | SLAM 可用，地图约束暂不可用 | global_matcher 连续有效 |
| `MAP_CONSTRAINED` | 地图约束融合 | global_matcher 周期有效 | global_matcher 超时或失败 |
| `GNSS_CONSTRAINED` | GNSS 约束融合 | 露天且 GNSS 质量高 | GNSS 失效或进入室内 |
| `RELOCALIZING` | 等待重定位恢复 | 严重退化或定位丢失 | manager 验收 relocalization |
| `DEGRADED` | 降级运行 | 主要观测超时或质量差 | 观测恢复 |
| `FAILURE` | 定位不可用 | 长时间无有效预测或 reset 失败 | 人工/重定位恢复 |

状态转移核心逻辑：

```text
启动:
  GNSS有效 -> GNSS初始化
  manager给初值 -> 使用初值初始化
  relocalization通过 -> 使用重定位初始化
  否则等待

正常:
  DR持续传播
  SLAM增强局部稳定
  global_matcher周期校正map一致性

退化:
  global_matcher失败但SLAM/DR有效 -> LOCAL_FUSION
  SLAM退化但DR有效 -> DR_ONLY/DEGRADED
  DR也无效 -> FAILURE

丢失:
  manager触发relocalization
  验收通过 -> HARD_RESET
```

---

## 11. ROS 接口设计

### 11.1 订阅

| Topic | Type | 默认频率 | 说明 |
|---|---|---:|---|
| `/localization/dr_odom` | `nav_msgs/Odometry` | 50-200 Hz | DR 预测源 |
| `/localization/dr_status` | `deeplumin_msgs/LocalizationStatus` | 1-10 Hz | DR 健康状态 |
| `/localization/slam_odom` | `nav_msgs/Odometry` / `deeplumin_msgs/SlamPose` | 5-20 Hz | SLAM 观测 |
| `/localization/slam_health` | `deeplumin_msgs/SlamHealth` | 5-20 Hz | SLAM 健康状态 |
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | 0.5-2 Hz | 地图匹配结果 |
| `/localization/global_matcher/status` | `deeplumin_msgs/LocalizationStatus` | 1-10 Hz | 地图匹配健康状态 |
| `/localization/manager/accepted_relocalization` | `deeplumin_msgs/RelocalizationResult` | event | manager 验收后的重定位 |
| `/localization/manager/observer_policy` | `deeplumin_msgs/ObserverPolicy` | 1-5 Hz | 场景策略和观测开关 |
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | 1-20 Hz | GNSS 原始输入，可选 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | event | RViz 人工初值 |

### 11.2 发布

| Topic | Type | 频率 | 说明 |
|---|---|---:|---|
| `/localization/fused_odom` | `nav_msgs/Odometry` | 50-100 Hz | 最终定位 odom |
| `/localization/fused_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 50-100 Hz | 最终 map 位姿 |
| `/localization/fused_path` | `nav_msgs/Path` | 1-10 Hz | RViz 轨迹 |
| `/localization/fusion_status` | `deeplumin_msgs/FusionStatus` | 5-10 Hz | 融合状态、观测使用情况 |
| `/localization/fusion_debug/used_observations` | `deeplumin_msgs/ObservationDebug` | 5-10 Hz | 调试观测 |
| `/tf` | `tf2_msgs/TFMessage` | 50-100 Hz | `map -> odom` 或最终 TF |

### 11.3 服务

| Service | 用途 |
|---|---|
| `/localization/pose_fusion/reset_pose` | 人工或 manager reset 融合状态 |
| `/localization/pose_fusion/set_policy` | 修改观测开关和权重 |
| `/localization/pose_fusion/get_state` | 查询当前状态、协方差、观测使用情况 |
| `/localization/pose_fusion/clear_history` | 清空 path 和观测缓存 |

---

## 12. 已定义消息和接口

### 12.1 `GlobalMatchResult.msg`

该消息已经在 `deeplumin_msgs/msg/localization/GlobalMatchResult.msg` 中定义，`pose_fusion` 直接订阅使用：

```text
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
bool success
bool converged
string map_id
string reject_reason
float64 fitness_score
float64 inlier_ratio
float64 inlier_rmse
uint32 inlier_count
uint32 source_points
uint32 target_points
float64 elapsed_ms
float64 initial_translation_error
float64 initial_yaw_error
```

`pose_fusion` 不应依赖可视化点云判断是否融合，只依赖该消息的结构化字段和 manager policy。

### 12.2 `RelocalizationResult.msg`

该消息已经定义，manager 验收后应将 `accepted=true` 的结果交给 `pose_fusion` 作为 reset 事件：

```text
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
bool success
bool accepted
string map_id
string method
string reject_reason
float64 score
float64 descriptor_score
float64 fitness_score
float64 inlier_ratio
float64 inlier_rmse
uint32 inlier_count
int32 candidate_index
int32 keyframe_id
float64 yaw_diff
float64 translation_diff
float64 elapsed_ms
```

### 12.3 `ObserverPolicy.msg`

该消息已经定义，用于 manager 控制各观测源启停和权重：

```text
std_msgs/Header header
string scene_type
string reason
bool use_dr
bool use_slam
bool use_global_matcher
bool use_relocalization
bool use_gnss
bool use_gnss_position
bool use_gnss_heading
float64 dr_weight
float64 slam_weight
float64 global_matcher_weight
float64 relocalization_weight
float64 gnss_position_weight
float64 gnss_heading_weight
bool allow_fusion_reset
bool allow_dr_reset
bool allow_global_matcher_trigger
bool allow_relocalization_trigger
```

### 12.4 `FusionStatus.msg`

```text
std_msgs/Header header
string state
bool initialized
bool output_valid
string current_map_id
string active_scene
float64 last_dr_age
float64 last_slam_age
float64 last_global_matcher_age
float64 last_gnss_age
float64 position_std
float64 yaw_std
string[] used_sources
string[] rejected_sources
string[] reject_reasons
```

---

## 13. 配置文件模板

`config/pose_fusion.yaml`：

```yaml
frames:
  map_frame: map
  odom_frame: odom
  base_frame: base_link
  lidar_frame: lidar_link
  imu_frame: imu_link
  gnss_frame: gnss_sensor

topics:
  dr_odom: /localization/dr_odom
  dr_status: /localization/dr_status
  slam_odom: /localization/slam_odom
  slam_health: /localization/slam_health
  global_matcher_result: /localization/global_matcher/result
  global_matcher_status: /localization/global_matcher/status
  accepted_relocalization: /localization/manager/accepted_relocalization
  observer_policy: /localization/manager/observer_policy
  gnss: /gnss_chc_data
  initial_pose: /initialpose
  fused_odom: /localization/fused_odom
  fused_pose: /localization/fused_pose
  fusion_status: /localization/fusion_status

runtime:
  output_rate_hz: 100.0
  publish_tf: true
  publish_path: true
  max_path_size: 5000
  allow_debug_relocalization_direct: false

buffer:
  max_observation_age: 0.5
  max_dr_gap: 0.2
  max_slam_age: 0.5
  max_global_matcher_age: 3.0
  max_gnss_age: 1.0

initialization:
  prefer_manager_initial_pose: true
  allow_gnss_initialization: true
  allow_relocalization_initialization: true
  initial_cov_position: 1.0
  initial_cov_yaw_deg: 5.0

prediction:
  source: dr_delta
  use_dr_velocity: true
  min_speed_for_yaw_update: 0.2
  process_noise_position: 0.05
  process_noise_velocity: 0.2
  process_noise_yaw_deg: 0.5

observers:
  dr:
    enabled: true
    weight: 1.0
  slam:
    enabled: true
    base_cov_position: 0.15
    base_cov_yaw_deg: 1.0
    degeneracy_scale: 10.0
  global_matcher:
    enabled: true
    base_cov_position: 0.08
    base_cov_yaw_deg: 0.5
    smooth_mode: true
    smooth_gain: 0.15
    max_position_residual: 5.0
    max_yaw_residual_deg: 20.0
    min_inlier_ratio: 0.5
    max_fitness: 1.0
    max_inlier_rmse: 0.8
    consecutive_accept_for_soft_reset: 3
  relocalization:
    enabled: true
    require_manager_approval: true
    hard_reset: true
  gnss:
    enabled: false
    use_position: true
    use_heading: true
    base_cov_position_fixed: 0.05
    base_cov_position_float: 0.5
    base_cov_heading_deg: 1.0

gating:
  enable_mahalanobis_gate: true
  chi_square_pose_6d: 16.81
  chi_square_position_3d: 11.34
  reject_large_jump: true
  max_reset_distance_without_manager: 2.0

state_machine:
  degraded_timeout: 2.0
  failure_timeout: 5.0
  relocalization_trigger_timeout: 3.0
```

---

## 14. 与 localization_manager 的协作流程

### 14.1 启动流程

```text
1. localization_manager 读取场景配置：
   - outdoor
   - semi_outdoor
   - indoor
   - tunnel

2. manager 启动或检查子模块：
   - dr_odometry 必须可运行
   - slam_odometry 按传感器状态启动
   - global_matcher 按地图可用性启动
   - relocalization 仅在需要时触发

3. manager 下发 ObserverPolicy：
   - outdoor: GNSS enabled, global_matcher enabled
   - indoor: GNSS disabled, global_matcher enabled
   - no_map: global_matcher disabled, SLAM/DR only

4. pose_fusion 等待初始化：
   - manager initial pose
   - GNSS 初值
   - accepted relocalization
   - RViz initialpose
```

### 14.2 正常运行流程

```text
dr_odometry -> pose_fusion 高频预测
slam_odometry -> pose_fusion 局部观测更新
pose_fusion -> global_matcher 提供当前 fused 初值
global_matcher -> pose_fusion 地图校正观测
pose_fusion -> manager 发布 fusion_status
manager -> pose_fusion 动态调整 observer_policy
```

### 14.3 定位退化流程

```text
pose_fusion 检测到 global_matcher 连续失败:
  -> 降级 LOCAL_FUSION
  -> status 标记 map constraint lost
  -> manager 可提高 global_matcher 频率或触发局部重试

pose_fusion 检测到 SLAM + global_matcher 均退化:
  -> 进入 DEGRADED
  -> manager 触发 relocalization

pose_fusion 检测到 DR 也不可用:
  -> FAILURE
  -> 停止发布 valid final localization 或标记 output_valid=false
```

### 14.4 重定位恢复流程

```text
1. manager 触发 relocalization
2. relocalization 输出 candidate
3. manager 验证：
   - scan context score
   - GICP fitness
   - 与最近 DR/SLAM 轨迹是否物理可达
   - map_id 是否正确
4. 验收通过后发布 accepted_relocalization
5. pose_fusion HARD_RESET：
   - 重置 fused state
   - 重置 map->odom
   - 清空观测缓存中的旧观测
   - 通知 global_matcher 使用新初值
```

---

## 15. 达到 ysw_loc 效果的关键实现点

ysw_loc 的效果主要来自两个部分：

1. LiDAR/IMU/速度带来的局部连续位姿。
2. 当前点云与全局地图/子图匹配带来的地图约束。

DeepLumin 中对应必须做到：

1. `slam_odometry` 输出要作为稳定局部约束，而不是只作为调试轨迹。
2. `global_matcher` 要周期性使用 `pose_fusion` 当前位姿作为初值进行地图匹配。
3. `pose_fusion` 必须消费 `global_matcher` 的匹配结果，并更新最终 `map` 位姿或 `map -> odom`。
4. global_matcher 失败时不能让定位瞬间跳坏，应保持 DR + SLAM 连续输出。
5. global_matcher 恢复时应平滑拉回地图，而不是直接造成轨迹跳变。
6. 重定位只用于丢失恢复，不替代正常 global_matcher 周期校正。

验收时至少需要对比：

| 指标 | ysw_loc | DeepLumin 要求 |
|---|---|---|
| 局部轨迹连续性 | 无明显跳变 | 不低于 ysw_loc |
| 地图贴合度 | 点云与地图稳定重合 | 不低于 ysw_loc |
| 短时遮挡/退化 | 可依赖 odom 延续 | DR + SLAM 延续 |
| 地图校正恢复 | 匹配成功后回到地图 | global_matcher 平滑校正 |
| 丢失恢复 | 依赖人工/全局定位 | manager + relocalization |

---

## 16. 开发阶段

### P0：基础框架和接口

1. 建立 `include/pose_fusion` 目录结构。
2. 实现 `types.hpp`、`config.hpp`、`math_utils.hpp`。
3. 实现 `RosAdapter` 的订阅、发布和参数加载。
4. 定义或补齐 `GlobalMatchResult`、`RelocalizationResult`、`ObserverPolicy`、`FusionStatus` 消息。
5. `pose_fusion_node` 能正常启动、读取配置、发布空状态。

### P1：DR 预测和基础输出

1. 订阅 `/localization/dr_odom`。
2. 实现 DR delta 传播。
3. 发布 `/localization/fused_odom`、`/localization/fused_pose`、TF、path。
4. 支持 `/initialpose` 初始化。

验收：仅启动 DR 时，`pose_fusion` 可输出连续轨迹。

### P2：SLAM 观测融合

1. 订阅 `/localization/slam_odom` 和 `/localization/slam_health`。
2. 实现 SLAM pose/relative pose update。
3. 根据 SLAM health 动态调整协方差。
4. 增加 SLAM 退化状态。

验收：GNSS 和 global_matcher 不启用时，DR + SLAM 轨迹不比原 SLAM 更差，且输出连续。

### P3：global_matcher 融合

1. 订阅 `/localization/global_matcher/result`。
2. 实现地图匹配质量门控。
3. 实现 `T_map_odom_observed` 计算。
4. 实现平滑更新 `map -> odom` 或 ESKF pose update。
5. 发布观测使用和拒绝原因。

验收：在有地图的场景中，最终轨迹能稳定贴合地图，效果达到 ysw_loc 对应全局定位节点效果。

### P4：manager 和 relocalization 接入

1. 订阅 `ObserverPolicy`。
2. 只接受 manager 验收后的重定位结果。
3. 实现 `HARD_RESET`。
4. 实现 `RELOCALIZING`、`DEGRADED`、`FAILURE` 状态。

验收：模拟定位丢失后，manager 接受重定位结果，pose_fusion 能恢复输出且 TF 正确。

### P5：GNSS 场景化融合

1. 接入 `/gnss_chc_data` 或标准化 `/localization/gnss_pose`。
2. 按场景策略启停 GNSS。
3. 支持位置、航向分离使用。
4. GNSS 异常时自动降权或拒绝。

验收：露天场景 GNSS 能增强绝对约束；室内场景禁用 GNSS 后不会发生 GNSS 跳变污染。

---

## 17. 测试和验收

### 17.1 单元测试

| 测试 | 内容 |
|---|---|
| `test_pose_math` | SE3 组合、逆、Log/Exp、yaw 提取 |
| `test_observation_buffer` | 乱序、超时、插值 |
| `test_quality_gate` | 残差门控、协方差门控、拒绝原因 |
| `test_fusion_state_machine` | 状态转移 |
| `test_global_matcher_update` | `T_map_odom_observed` 计算 |

### 17.2 rosbag 回放测试

| Case | 输入 | 预期 |
|---|---|---|
| DR only | DR | 连续但可能漂移 |
| DR + SLAM | DR、SLAM | 连续性提升 |
| DR + SLAM + global_matcher | DR、SLAM、地图匹配 | 轨迹贴合地图 |
| global_matcher intermittent fail | 间歇失败匹配 | 输出不中断，不跳变 |
| relocalization reset | 人工触发丢失和恢复 | reset 后 TF 正确 |
| outdoor GNSS | GNSS 高质量 | 绝对位置更稳 |
| indoor no GNSS | GNSS 禁用 | 无 GNSS 污染 |

### 17.3 对齐 ysw_loc 的验收标准

1. 使用同一段 rosbag 和同一张地图。
2. ysw_loc 输出轨迹作为 baseline。
3. DeepLumin 开启 `slam_odometry + global_matcher + pose_fusion`。
4. 对比：
   - 轨迹与地图重合程度。
   - 点云投影到地图后的残差。
   - 匹配成功率。
   - 定位跳变次数。
   - 退化恢复时间。
5. 若 DeepLumin 低于 ysw_loc，优先检查：
   - global_matcher 是否真的被 pose_fusion 使用。
   - `map -> odom` 是否由 pose_fusion 唯一发布。
   - global_matcher 初值是否来自 fused pose。
   - SLAM 退化协方差是否过大或过小。
   - frame 外参是否正确。

---

## 18. 实现注意事项

1. 不要在多个模块同时发布最终 `map -> odom`。
2. 不要让 `relocalization` 绕过 manager 直接 reset。
3. 不要把 `global_matcher` 当作普通高频 odom，它是低频地图观测。
4. 不要在 GNSS 质量不明时使用航向强约束。
5. 所有观测都必须带 timestamp 和 covariance。
6. 所有拒绝都必须有 reject reason，便于 rosbag 调试。
7. `pose_fusion` 的输出频率应稳定，不应跟随低频 global_matcher 抖动。
8. 初期优先实现可验证的 DR delta + SLAM/GLOBAL pose update，不要过早把所有 IMU 细节塞进 `pose_fusion`。
9. 如果要将 SLAM 反馈给 DR，必须通过 manager 策略控制，并在日志中记录 reset/feedback 事件。
10. 地图段切换时应由 `global_matcher` 报告 `map_id`，由 `pose_fusion` 更新状态，由 manager 统一编排。

---

## 19. 最小可落地版本

为了尽快达到 ysw_loc 效果，推荐最小实现如下：

```text
1. /initialpose 或 manager 初值初始化 fused pose
2. DR odom delta 高频传播 fused pose
3. SLAM odom 作为局部 pose observation 更新
4. global_matcher result 作为 map pose observation 更新
5. pose_fusion 唯一发布 /localization/fused_odom 和 map->odom
6. manager accepted relocalization 触发 hard reset
7. FusionStatus 输出每个观测的使用/拒绝原因
```

这个版本已经具备 ysw_loc 的核心闭环：局部连续运动 + 地图匹配校正。后续再逐步增强 GNSS 场景化、完整 ESKF 传播、SLAM 反馈 DR 和更细的状态机策略。
