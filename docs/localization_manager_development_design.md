# DeepLumin localization_manager 详细开发设计文档

> 适用模块：`DeepLumin/src/localization/localization_manager`  
> 协作模块：`dr_odometry`、`slam_odometry`、`global_matcher`、`pose_fusion`、`relocalization`、GNSS/CAN/IMU/LiDAR 输入  
> 目标：实现定位系统的统一编排、健康监控、场景策略、退化处理、重定位验收和联调入口，使当前所有 localization 模块能组成完整定位链路，并在短时传感器退化时保持可解释、可控的降级定位能力。

---

## 0. 与当前 global_matcher 实现的同步

`global_matcher` 当前已经完成可运行版本，manager 文档中的编排策略需要以当前接口为基线：

1. `global_matcher` 默认周期自动匹配，配置项为 `runtime/auto_match` 和 `runtime/match_rate`。
2. 当前尚未实现严格请求/响应的 trigger service/action，manager 第一阶段可通过 observer policy、启动参数和健康监控间接编排；后续再补 `trigger` 服务。
3. 地图段加载和切换当前已实现为 `std_msgs/String` 命令话题：`load_map_command`、`set_map_segment_command`。
4. 匹配结果已通过 `deeplumin_msgs/GlobalMatchResult` 发布，manager 可直接订阅用于健康统计、地图段一致性检查和 relocalization 触发判断。
5. 可视化点云只用于调试，不作为 manager 决策输入。

manager 后续开发不能再把 `global_matcher` 当占位节点处理，应按“已可用地图约束观测源”纳入联调。

## 1. 模块定位

`localization_manager` 是定位功能域的编排中枢。它不实现 DR、SLAM、点云匹配、重定位或融合算法，但负责决定这些模块何时启用、何时降权、何时触发、哪些结果可以进入最终融合。

核心原则：

1. `pose_fusion` 是最终定位输出权威节点。
2. `global_matcher` 是正常定位过程中的地图/子图匹配校正模块。
3. `relocalization` 是定位丢失或无可靠初值时的全局恢复模块。
4. `localization_manager` 负责场景判断、健康聚合、触发策略和验收，不直接发布最终 `map -> odom`。
5. 所有 reset、relocalization 接受、GNSS 启停、global_matcher 触发都必须有状态记录和拒绝原因。

系统关系：

```text
sensor inputs
  |-- IMU / CAN / GNSS -> dr_odometry
  |-- LiDAR / IMU      -> slam_odometry
  |-- LiDAR / map / fused pose -> global_matcher
  |-- LiDAR / keyframe database -> relocalization

dr_odometry        \
slam_odometry       \
global_matcher       -> pose_fusion -> fused localization
GNSS policy          /
accepted reloc      /

localization_manager
  |-- 监控所有模块状态
  |-- 下发 observer policy
  |-- 触发 global_matcher
  |-- 触发并验收 relocalization
  |-- 决定 GNSS 使用方式
  `-- 对外发布统一定位状态
```

---

## 2. 当前代码状态

当前仓库状态：

| 模块 | 当前状态 | 对 manager 的要求 |
|---|---|---|
| `dr_odometry` | 已有 ESKF、ROS adapter、debug 节点、配置文件 | 监控 `/localization/dr_odom`、`/localization/dr_status` |
| `slam_odometry` | 已有 Faster-LIO 风格结构和 ysw 配置 | 监控 `/localization/slam_odom`、`/localization/slam_health`、注册点云 |
| `relocalization` | 已有 Scan Context + GICP、离线/在线配置 | 由 manager 触发，结果由 manager 验收 |
| `global_matcher` | 已实现 PCD/地图段加载、FOV 裁剪、fast_gicp 匹配、质量门控、keyframe 可视化测试 | manager 监控结果和失败计数，编排地图段切换，后续补 trigger service |
| `pose_fusion` | 已完成第一版：可接收 DR/SLAM/global_matcher/accepted relocalization/observer policy，维护 `T_map_odom` 并发布 fused odom/path/TF | manager 必须提供统一初始位姿、下发 observer policy、验收 relocalization/reset |
| `localization_manager` | 当前仍为占位节点 | 下一步优先实现系统初始化、初始位姿下发、GNSS 场景策略、模块健康聚合和与 pose_fusion 第一版联调 |

结论：当前代码已经具备 `global_matcher` 地图约束能力，`pose_fusion` 也已有第一版融合链路；当前最缺的是 `localization_manager` 对系统初始化和场景策略的编排。没有 manager 下发统一初始位姿时，DR/SLAM 各自以本模块的 `odom` 起点运行，方向和原点可能不同，RViz 中很难直接看到有效融合效果。完成本文档对应实现后，系统可以先由 manager 将 RViz `/initialpose`、`relocalization` 或可靠 GNSS 产生的 `T_map_base_init` 下发给 `pose_fusion`，再进入 DR/SLAM/global_matcher 的连续定位与融合；LiDAR 短时失效时由 DR 继续预测，SLAM/global_matcher 降权，fusion 协方差增长，manager 标记 degraded，并在 LiDAR 恢复后用 global_matcher/relocalization 拉回地图。

---

## 3. 开发目标

### 3.1 功能目标

1. 联调当前所有 localization 模块：
   - `dr_odometry`
   - `slam_odometry`
   - `global_matcher`
   - `pose_fusion`
   - `relocalization`
2. 支持多场景：
   - 露天 GNSS 可用
   - 半开放 GNSS 间歇可用
   - 室内/矿区/隧道 GNSS 不可用
   - 有地图定位
   - 无可靠初值重定位
3. 支持短时传感器退化：
   - LiDAR 短时无点云或点云质量差
   - GNSS 失效或跳变
   - CAN 速度短时丢失
   - IMU 时间跳变或断流
   - SLAM 退化
   - global_matcher 连续失败
4. 支持统一状态发布：
   - 正常
   - 警告
   - 降级
   - 失败
5. 支持人工调试：
   - 手动设置模式
   - 手动触发 global_matcher
   - 手动触发 relocalization
   - 手动 reset pose_fusion
   - 手动切换 GNSS 使用策略

### 3.2 工程目标

1. manager 不依赖具体算法实现，只依赖 ROS topic/service/action 和统一状态。
2. 所有模块可以单独运行，也可以被 manager 统一编排。
3. 所有状态切换有去抖和超时，避免频繁抖动。
4. 所有触发、拒绝、reset、降级都能在 rosbag 中复盘。
5. 默认策略保守：宁可降级输出，也不能错误 reset 或错误使用 GNSS。


---

## 4. 系统初始化与统一坐标系

`localization_manager` 第一优先级不是融合算法本身，而是建立完整定位链路的统一初始条件。当前 `dr_odometry` 和 `slam_odometry` 单独运行时都会形成各自的 `odom` 坐标系：

```text
dr_odometry   -> T_dr_odom_base
slam_odometry -> T_slam_odom_base
pose_fusion   -> 需要 T_map_odom 才能输出 T_map_base_fused
```

如果没有 `T_map_base_init`，`pose_fusion` 只能默认 `map == odom`，这时 DR/SLAM 的原点和方向未被地图约束统一，不能用原始 path 是否同向来判断完整定位效果。因此 manager 必须先完成初始化，再允许进入正常里程计估计与融合。

### 4.1 初始位姿来源

初始位姿语义统一为：

```text
T_map_base_init: 初始化时刻 base_link 在当前地图 map 下的位姿
```

manager 支持三类来源，优先级如下：

| 优先级 | 来源 | Topic/接口 | 适用场景 | 备注 |
|---:|---|---|---|---|
| 1 | RViz 人工初值 | `/initialpose` | bag 调试、人工指定地图内起点 | 最适合当前 bag 测试；不依赖 GNSS 坐标与地图一致 |
| 2 | relocalization | `/localization/relocalization/result`，验收后转发 `/localization/manager/accepted_relocalization` | 自动启动、定位丢失恢复 | 推荐真实系统自动初始化主路径 |
| 3 | 可靠 GNSS | `/gnss_chc_data` 转 map pose | 露天、GNSS 与地图坐标已标定 | 不适合当前 bag 地图测试，除非已知 GNSS ENU 与 PCD map 的外参 |

禁止把未验收的 relocalization 或跳变 GNSS 直接作为 fusion reset。所有初值必须经过 manager 的质量门控并记录事件。

### 4.2 初始位姿下发给 pose_fusion

manager 生成 `T_map_base_init` 后，不直接发布最终 TF，而是下发初始化事件给 `pose_fusion`。`pose_fusion` 根据当前可用的局部里程计计算外层变换：

```text
T_map_odom = T_map_base_init * inverse(T_odom_base_current)
T_map_base = T_map_odom * T_odom_base_current
```

其中 `T_odom_base_current` 的选择策略：

| 当前可用输入 | 初始化参考 | 说明 |
|---|---|---|
| DR + SLAM 都可用 | 优先 DR 或 fusion 当前 local state，SLAM 作为观测 | DR 高频连续，SLAM 后续校正 |
| 仅 DR 可用 | DR odom | 允许进入短时 DR 初始化，但状态可为 WARNING |
| 仅 SLAM 可用 | SLAM odom | 允许纯 SLAM 调试，完整系统需等待 DR 或明确配置 |
| 都不可用 | 不初始化 | 保持 `WAIT_INITIAL_POSE` |

第一版可只维护一个 `odom` 参考，优先使用 pose_fusion 已接入的 DR 作为 propagation 主轴；后续若需要同时严格统一 `dr_odom` 和 `slam_odom`，可在 pose_fusion 内分别维护 `T_map_dr_odom`、`T_map_slam_odom` 或将 SLAM 转成相对观测。

### 4.3 bag 测试推荐策略

当前 bag 测试的目标是验证地图定位链路，而不是验证 GNSS 绝对坐标。因此推荐：

```text
GNSS position: 不用于初始化，不用于 fusion position hard constraint
GNSS heading : 可用于 DR 航向稳定，但需确认 heading 与 base_link 前向一致
initial pose : 优先 RViz /initialpose 或 relocalization 输出
map alignment: 由 pose_fusion 的 T_map_odom 完成
```

如果使用 GNSS 位置初始化，则会把系统固定到 GNSS ENU 原点；当 PCD 地图不是同一个 ENU 坐标或存在 yaw/translation offset 时，轨迹会与地图不对齐，反而掩盖 localization_manager/pose_fusion 的坐标系统一问题。

### 4.4 初始化状态流

```text
INIT
  -> 等待 DR/SLAM 至少一路 odom 可用
  -> 读取地图状态和当前 map_id
  -> 进入 WAIT_INITIAL_POSE

WAIT_INITIAL_POSE
  -> 收到 /initialpose: 生成 manual init candidate
  -> relocalization 成功: 生成 reloc init candidate
  -> GNSS reliable 且允许 GNSS init: 生成 gnss init candidate
  -> candidate 通过验收: 发布 initialize/reset event 给 pose_fusion
  -> 等待 /localization/fused_odom valid
  -> LOCALIZATION
```

初始化事件必须包含：

```text
source: MANUAL_INITIALPOSE / RELOCALIZATION / GNSS
map_id
stamp
T_map_base_init
position covariance
yaw covariance
quality score
reason
```

### 4.5 初始化验收门控

| 来源 | 必要条件 | 拒绝条件 |
|---|---|---|
| `/initialpose` | map frame 正确，姿态四元数有效 | frame 不是 map、时间异常、协方差过大且未强制允许 |
| relocalization | descriptor/fine matcher 质量达标，map_id 一致 | fitness/RMSE/inlier 不达标，候选不稳定，地图段不一致 |
| GNSS | 场景允许、fix/heading/卫星数/时间戳可靠，已知 `T_map_enu` | 室内/矿区、质量差、跳变、未标定 GNSS ENU 到 map |

---

## 5. 状态机设计

### 5.1 Manager 主状态

```cpp
enum class ManagerState {
  INIT,
  WAIT_INITIAL_POSE,
  LOCALIZATION,
  GLOBAL_MATCHING,
  RELOCALIZING,
  DEGRADED,
  FAILURE,
  PURE_DR,
  PURE_SLAM,
  MAPPING
};
```

| 状态 | 含义 | 输出策略 | 典型进入条件 |
|---|---|---|---|
| `INIT` | 参数、地图、topic、模块状态初始化 | 不给规划有效定位 | 节点启动 |
| `WAIT_INITIAL_POSE` | 等待人工初值、GNSS 初值或重定位结果 | `output_valid=false` | 无可靠初值 |
| `LOCALIZATION` | 正常融合定位 | `output_valid=true` | DR + 至少一个约束源正常 |
| `GLOBAL_MATCHING` | 子图匹配校正进行中 | fusion 继续发布 | 周期触发或质量下降触发 |
| `RELOCALIZING` | 全局重定位进行中 | 可保持 degraded 输出 | 严重退化、丢失、人工触发 |
| `DEGRADED` | 部分传感器失效但可短时运行 | `output_valid=true`，状态降级 | LiDAR/GNSS/global_matcher 退化 |
| `FAILURE` | 定位不可用 | `output_valid=false` | DR 也不可用或协方差超限 |
| `PURE_DR` | DR 调试/应急模式 | 只发布 DR/fusion 预测 | 人工模式 |
| `PURE_SLAM` | SLAM 调试模式 | 不作为完整定位链路 | 人工模式 |
| `MAPPING` | 建图或数据库生成 | 不给规划常规定位 | 人工模式 |

### 5.2 状态转移

```text
INIT
  -> WAIT_INITIAL_POSE: 参数和必要模块 ready，但无初值
  -> LOCALIZATION: 有人工初值/GNSS初值/已验收relocalization
  -> FAILURE: 必要传感器长时间不可用

WAIT_INITIAL_POSE
  -> LOCALIZATION: 初值通过验收并 reset pose_fusion
  -> RELOCALIZING: 有地图和点云，自动触发重定位
  -> FAILURE: 超时且无可用恢复方式

LOCALIZATION
  -> GLOBAL_MATCHING: 需要子图匹配校正
  -> DEGRADED: 某个关键观测源退化但 DR/fusion 可维持
  -> RELOCALIZING: 严重退化或定位丢失
  -> FAILURE: DR/fusion 均不可用

GLOBAL_MATCHING
  -> LOCALIZATION: 匹配成功或匹配失败但系统健康
  -> DEGRADED: 连续失败但 DR/SLAM 仍可用
  -> RELOCALIZING: 连续失败且融合协方差超限

RELOCALIZING
  -> LOCALIZATION: 重定位验收通过并 reset fusion
  -> DEGRADED: 重定位失败但 DR/fusion 仍可短时维持
  -> FAILURE: 重定位失败且无可信定位输出

DEGRADED
  -> LOCALIZATION: 退化传感器恢复并通过稳定计数
  -> RELOCALIZING: 漂移/协方差/失败计数超限
  -> FAILURE: DR/fusion 预测也失效
```

### 5.3 去抖策略

所有状态切换必须满足连续计数或持续时间：

| 条件 | 默认值 | 说明 |
|---|---:|---|
| 进入 degraded 连续失败次数 | 3 | 防止单帧误判 |
| 退出 degraded 连续正常次数 | 5 | 防止刚恢复就切回 |
| 触发 relocalization 连续严重失败 | 5 | 避免误触发 |
| failure 超时 | 5.0 s | 长时间无可信输出 |
| LiDAR 短时失效容忍 | 1.0-3.0 s | 依赖 DR 预测 |
| GNSS 跳变隔离时间 | 2.0 s | 禁止立刻强约束 |

---

## 6. 模块健康模型

### 6.1 统一健康结构

```cpp
enum class ModuleLevel {
  NORMAL,
  WARNING,
  DEGRADED,
  FAILURE
};

struct ModuleHealth {
  std::string name;
  ModuleLevel level = ModuleLevel::FAILURE;
  ros::Time last_msg_time;
  double age_sec = 1e9;
  double quality_score = 0.0;
  int consecutive_ok = 0;
  int consecutive_warn = 0;
  int consecutive_fail = 0;
  bool available = false;
  bool timeout = true;
  std::string reason;
};
```

manager 内部维护：

```text
health.dr
health.slam
health.global_matcher
health.pose_fusion
health.relocalization
health.gnss
health.can
health.imu
health.lidar
```

### 6.2 现有消息适配

已有 `deeplumin_msgs/LocalizationStatus.msg`：

```text
std_msgs/Header header
uint8 level
uint8 LEVEL_NORMAL = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_DEGRADED = 2
uint8 LEVEL_FAILURE = 3
string failure_reason
float32 quality_score
bool is_gnss_available
bool is_slam_available
bool is_dr_available
string current_mode
```

已有 `deeplumin_msgs/SlamHealth.msg`：

```text
std_msgs/Header header
float32 score
uint32 feature_count
float32 condition_number
float32 match_residual
bool is_degenerated
```

manager 第一阶段应直接使用已有消息；第二阶段再补齐更细的 `FusionStatus`、`GlobalMatchResult`、`RelocalizationResult`、`ObserverPolicy`。

### 6.3 健康判定

| 模块 | NORMAL | DEGRADED | FAILURE |
|---|---|---|---|
| DR | `/localization/dr_odom` 频率正常，status normal | 轮速/GNSS 部分缺失但 IMU 可预测 | DR odom 超时或跳变 |
| SLAM | feature 足够，未退化，残差正常 | 点数少、退化、残差升高 | LiDAR 超时或 SLAM 无输出 |
| global_matcher | 周期成功，fitness/inlier 合格 | 连续失败但未超限 | 长时间失败且需要地图约束 |
| pose_fusion | fused odom 高频稳定，协方差合理 | 仅 DR 预测、协方差增长 | fused odom 超时或 invalid |
| relocalization | 待机或成功结果可信 | 多次候选不稳定 | 连续失败且无其他恢复 |
| GNSS | fix/heading 质量符合场景 | float/遮挡/跳变疑似 | 超时或室内禁用 |
| LiDAR | 点云频率正常、点数足够 | 点数下降/遮挡 | 点云超时 |
| CAN | speed 有效且频率正常 | 短时丢包 | 超时或 invalid |
| IMU | 高频、时间连续 | 抖动/短时缺样 | 超时或时间倒退 |

---

## 7. 场景策略

### 7.1 场景枚举

```cpp
enum class SceneType {
  OUTDOOR_OPEN,
  SEMI_OUTDOOR,
  INDOOR,
  TUNNEL,
  MINE,
  NO_MAP,
  UNKNOWN
};
```

### 7.2 场景对应观测策略

| 场景 | GNSS | SLAM | global_matcher | relocalization |
|---|---|---|---|---|
| `OUTDOOR_OPEN` | 位置+航向可用，高质量时强约束 | 可用 | 有地图时低频校正 | 丢失时触发 |
| `SEMI_OUTDOOR` | 低权重或仅航向，严防跳变 | 主约束 | 提高频率接管地图约束 | 严重退化触发 |
| `INDOOR` | 默认禁用 | 主约束 | 主全局校正 | 启动/丢失触发 |
| `TUNNEL` | 禁用或仅监控 | 可能退化 | 有地图时关键 | 退化触发 |
| `MINE` | 禁用 | 主约束 | 主全局校正 | 关键恢复手段 |
| `NO_MAP` | 可用则辅助 | 主约束 | 禁用 | 禁用或只做数据库恢复 |

### 7.3 ObserverPolicy

manager 周期性向 `pose_fusion` 发布或服务设置观测策略：

```text
use_dr
use_slam
use_global_matcher
use_relocalization
use_gnss
dr_weight
slam_weight
global_matcher_weight
gnss_position_weight
gnss_heading_weight
scene_type
reason
```

策略原则：

1. DR 是短时连续预测基础，除非 DR failure，否则一直启用。
2. SLAM 正常时用于局部约束；退化时降权，不直接关闭。
3. global_matcher 正常定位时低频校正；退化或 GNSS 失效过渡时提高触发频率。
4. relocalization 只在启动、丢失、严重退化、人工触发时启用。
5. GNSS 只能按场景和质量使用；室内默认禁用。

### 7.4 GNSS 场景判断：手动优先，自动辅助

场景判断可以参考 GNSS 有无和可靠性，但不能只依赖 GNSS。原因是“没有 GNSS”通常能说明处于室内/遮挡/矿区，但“有 GNSS”不一定说明 GNSS 与当前地图坐标一致，也不一定可靠。

manager 应支持两种模式：

| 模式 | 参数 | 行为 |
|---|---|---|
| 手动场景 | `scene/source: manual` | 由 launch、服务或 RViz 工具设置 `OUTDOOR_OPEN/INDOOR/MINE/...`，最适合 bag 测试 |
| 自动场景 | `scene/source: auto_gnss` | 根据 GNSS 新鲜度、状态、卫星数、速度/heading 一致性推断 outdoor/semi/indoor |
| 混合模式 | `scene/source: auto_with_manual_override` | 默认自动，人工设置后在超时时间内优先生效 |

自动判断建议：

```text
GNSS no message or timeout:
  scene_hint = INDOOR_OR_OCCLUDED
  gnss_position_allowed = false
  gnss_heading_allowed = false or heading_only_if_external_reliable

GNSS status good + satellites enough + age fresh + no jump:
  scene_hint = OUTDOOR_OPEN or SEMI_OUTDOOR
  gnss_position_allowed = true only if map_enu_calibrated
  gnss_heading_allowed = true if speed/heading gate passed

GNSS intermittent or residual inconsistent:
  scene_hint = SEMI_OUTDOOR
  gnss_position_allowed = false or low weight
  gnss_heading_allowed = low weight / heading only
```

GNSS 可靠性分为三层：

| 层级 | 可用性 | 用途 |
|---|---|---|
| `GNSS_UNAVAILABLE` | 无消息、超时、状态无效 | 不进入 DR/Fusion，只做日志 |
| `GNSS_HEADING_ONLY` | heading 可信但 position 不可信 | 可给 DR 或 fusion yaw 观测，不能初始化位置 |
| `GNSS_FULL` | position + heading + velocity 可信，且 map/ENU 外参已知 | 可初始化、可作为 fusion 全局观测 |

### 7.5 DR 模块中的 GNSS 使用编排

`dr_odometry` 的 GNSS 使用应由 manager 策略决定，而不是长期写死。推荐把 DR 的 GNSS 使用拆成三个独立开关：

```text
dr_use_gnss_heading
dr_use_gnss_velocity
dr_use_gnss_position
```

策略如下：

| 场景/质量 | DR heading | DR velocity | DR position | 说明 |
|---|---|---|---|---|
| bag 地图测试 | 可选 | 默认关 | 关 | 初始位姿由 `/initialpose` 或 relocalization 给出，避免 GNSS ENU 锁死地图 |
| 露天 GNSS 可靠 | 开 | 可开 | 谨慎开 | 如果 GNSS ENU 与 map 已标定，可用于约束 DR |
| 半开放 | 可低权重开 | 关或低权重 | 关 | 防止 GNSS 跳变污染 DR |
| 室内/矿区/隧道 | 关 | 关 | 关 | DR 使用 IMU + wheel，地图约束由 SLAM/global_matcher/relocalization 提供 |
| GNSS heading 与地图 yaw 不一致但固定 | 可开 | 视情况 | 关 | yaw 偏差由 `T_map_odom` 吸收；不能把 DR 直接当 map 位姿 |

实现方式分两阶段：

1. P1：通过 launch/config 启动前配置 DR 的 GNSS 开关，manager 只发布策略和状态。
2. P2：为 `dr_odometry` 增加运行时服务或 topic，例如 `/localization/dr_odometry/set_sensor_policy`，manager 可在线切换 GNSS heading/velocity/position。

注意：即使 DR 使用 GNSS heading，DR 仍应输出自身 `odom` 下的相对位姿。地图方向与 GNSS/ENU 方向的差异必须由 `pose_fusion` 的 `T_map_odom` 解决。

---

## 8. 与各模块的接口

### 8.1 dr_odometry

当前配置：

| 输入 | Topic |
|---|---|
| IMU | `/ouster/imu` |
| CAN | `/can_receive_info` |
| GNSS | `/gnss_chc_data` |

| 输出 | Topic |
|---|---|
| DR odom | `/localization/dr_odom` |
| DR status | `/localization/dr_status` |
| DR path | `/localization/dr_path` |

manager 订阅：

```text
/localization/dr_odom
/localization/dr_status
/can_receive_info
/ouster/imu
```

manager 动作：

1. 判断 DR 是否可作为 `pose_fusion` 预测源。
2. 判断 CAN/IMU 是否超时。
3. 在重定位成功后，可选调用 DR reset/yaw correction 接口。
4. 根据场景策略决定 DR 是否使用 GNSS heading/velocity/position；第一阶段通过 launch/config 固定，第二阶段通过运行时 sensor policy 服务动态切换。
5. bag 地图测试默认不允许 DR 使用 GNSS position；可选择使用 GNSS heading 稳定 yaw，但必须通过 `pose_fusion` 的 `T_map_odom` 与地图坐标统一。
6. GNSS 不可靠或室内/矿区场景下，DR 应退回 IMU + wheel 模式，GNSS 只做状态监控。

### 8.2 slam_odometry

当前配置：

| 输入 | Topic |
|---|---|
| LiDAR | `/ouster/points` |
| IMU | `/ouster/imu` |
| CAN | `/can_receive_info` |

| 输出 | Topic |
|---|---|
| SLAM odom | `/localization/slam_odom` |
| SLAM health | `/localization/slam_health` |
| registered cloud | `/localization/cloud_registered` |
| local map | `/localization/local_map` |

manager 订阅：

```text
/localization/slam_odom
/localization/slam_health
/localization/cloud_registered
/ouster/points
```

manager 动作：

1. 根据 `SlamHealth` 判断退化程度。
2. LiDAR 短时失效时，通知 `pose_fusion` 降低 SLAM 权重。
3. SLAM 恢复后，需要连续 N 帧健康后再恢复权重。
4. 若 SLAM 长时间失效且 global_matcher 不可用，触发 relocalization 或进入 failure。

### 8.3 global_matcher

manager 与 global_matcher 的关系：

```text
pose_fusion/fused_odom -> global_matcher initial guess
slam_odometry/cloud_registered -> global_matcher source cloud
manager -> global_matcher load_map_command / set_map_segment_command
global_matcher/result -> pose_fusion observation and manager health statistics
global_matcher/status -> manager health
```

推荐接口：

| 接口 | 方向 | 说明 |
|---|---|---|
| `/localization/global_matcher/load_map_command` | manager -> global_matcher | 当前已实现，`map_id|pcd_path[|activate]` |
| `/localization/global_matcher/set_map_segment_command` | manager -> global_matcher | 当前已实现，`map_id` |
| `/localization/global_matcher/status` | global_matcher -> manager | 匹配状态 |
| `/localization/global_matcher/result` | global_matcher -> pose_fusion/manager | 匹配结果 |
| `/localization/global_matcher/trigger` | manager -> global_matcher | 后续扩展，触发一次匹配 |

manager 动作：

1. 第一阶段使用 `global_matcher` 自身 `runtime/match_rate` 周期匹配，manager 通过 health/result 监控是否正常。
2. 后续增加 trigger service 后，manager 在正常定位时按 0.5-2Hz 触发，在 GNSS 失效过渡、SLAM 轻度退化、DR/SLAM 残差增大时提高触发频率。
3. 匹配失败不直接重定位，连续失败并叠加 fusion 退化才触发 relocalization。
4. 地图段切换时先通过 `load_map_command`/`set_map_segment_command` 让 global_matcher 加载/切换地图，再允许对应 `map_id` 的结果进入 fusion。

### 8.4 pose_fusion

manager 与 pose_fusion 的关系：

| 接口 | 方向 | 说明 |
|---|---|---|
| `/localization/fused_odom` | pose_fusion -> manager | 最终定位 |
| `/localization/fusion_status` | pose_fusion -> manager | 融合状态和观测使用情况 |
| `/localization/manager/observer_policy` | manager -> pose_fusion | 观测启停和权重 |
| `/localization/manager/accepted_relocalization` | manager -> pose_fusion | 已验收重定位 |
| `/localization/pose_fusion/reset_pose` | manager -> pose_fusion | 人工或重定位 reset |

manager 动作：

1. 在 `WAIT_INITIAL_POSE` 阶段给出 `T_map_base_init`，通过 `/initialpose`、已验收 relocalization 或可靠 GNSS 初始化 pose_fusion。
2. 监控 fused odom 频率、协方差、跳变。
3. 根据场景和模块健康下发 observer policy。
4. 只有 manager 验收后的 relocalization 结果允许 hard reset。
5. fusion 协方差超限时触发 global_matcher 或 relocalization。
6. 若 pose_fusion 第一版只支持 `accepted_relocalization` reset，则 manager 可将人工 `/initialpose` 和 GNSS 初始化统一封装成同一类 accepted initialization/reset 事件。

### 8.5 relocalization

当前 `relocalization` 具备 Scan Context + GICP 能力。manager 负责触发和验收。

推荐接口：

| 接口 | 方向 | 说明 |
|---|---|---|
| `/localization/trigger_relocalization` | manager/manual -> relocalization | 触发重定位 |
| `/localization/relocalization/result` | relocalization -> manager | 候选结果 |
| `/localization/manager/accepted_relocalization` | manager -> pose_fusion | 验收后的结果 |

验收条件：

1. Scan Context score 达标。
2. GICP fitness/inlier/RMSE 达标。
3. 与当前 fused pose 的差异在物理可达范围内；若当前定位已 failure，可放宽该条件。
4. 最好连续两帧或二次验证一致。
5. map_id 与当前地图段一致。

### 8.6 GNSS

当前 GNSS topic：

```text
/gnss_chc_data: deeplumin_msgs/Gpchc
frame_id: gnss_sensor
```

manager 动作：

1. 根据 `status`、速度、卫星数、时间戳判断可用性。
2. 根据场景决定是否允许 GNSS 进入 `pose_fusion`。
3. GNSS 跳变时隔离，不能直接 reset。
4. 露天高质量 GNSS 可用于初始化和绝对约束。
5. 室内/矿区/隧道默认禁用 GNSS，只保留状态监控。

---

## 9. 联调当前所有模块

### 9.1 联调目标

完整联调链路：

```text
dr_odometry:
  /ouster/imu + /can_receive_info + optional /gnss_chc_data
  -> /localization/dr_odom
  -> /localization/dr_status

slam_odometry:
  /ouster/points + /ouster/imu
  -> /localization/slam_odom
  -> /localization/slam_health
  -> /localization/cloud_registered

global_matcher:
  /localization/cloud_registered + /localization/fused_odom + map
  -> /localization/global_matcher/result
  -> /localization/global_matcher/status

pose_fusion:
  /localization/dr_odom
  /localization/slam_odom
  /localization/global_matcher/result
  /gnss_chc_data
  /localization/manager/accepted_relocalization
  -> /localization/fused_odom
  -> /localization/fusion_status

relocalization:
  /ouster/points or /localization/cloud_registered + scan context db
  -> /localization/relocalization/result

localization_manager:
  monitor all
  -> /localization/manager/status
  -> /localization/manager/observer_policy
  -> trigger global_matcher / relocalization
```

### 9.2 启动顺序

推荐联调启动顺序：

```text
1. roscore
2. 传感器或 rosbag
3. dr_odometry
4. slam_odometry
5. pose_fusion
6. global_matcher
7. relocalization online node
8. localization_manager
9. RViz 可视化
```

原因：

1. `dr_odometry` 和 `slam_odometry` 提供基础输入。
2. `pose_fusion` 需要先有 DR/SLAM 才能输出初值。
3. `global_matcher` 需要 fused pose 和 registered cloud。
4. `localization_manager` 最后启动，避免启动阶段误判模块超时。

### 9.3 最小联调模式

第一阶段：

```text
DR + SLAM + pose_fusion + manager
```

目标：

1. manager 能看到 DR/SLAM/fusion 状态。
2. manager 能下发 observer policy。
3. LiDAR 正常时 fusion 使用 SLAM。
4. LiDAR 短时失效时 fusion 继续由 DR 预测。

第二阶段：

```text
DR + SLAM + global_matcher + pose_fusion + manager
```

目标：

1. manager 周期触发 global_matcher。
2. global_matcher 输出结果进入 pose_fusion。
3. 最终轨迹贴合地图，达到 ysw_loc 的地图约束效果。

第三阶段：

```text
DR + SLAM + global_matcher + pose_fusion + relocalization + manager
```

目标：

1. 模拟定位丢失。
2. manager 触发 relocalization。
3. manager 验收结果。
4. pose_fusion reset 后恢复正常定位。

第四阶段：

```text
Outdoor GNSS + DR + SLAM + pose_fusion + manager
```

目标：

1. 露天 GNSS 高质量时参与初始化和约束。
2. GNSS 失效或切换室内后，manager 禁用/降权 GNSS。
3. 系统无 GNSS 跳变污染。

### 9.4 RViz 联调内容

必须显示：

| RViz 项 | Topic/TF |
|---|---|
| 最终轨迹 | `/localization/fused_path` |
| DR 轨迹 | `/localization/dr_path` |
| SLAM odom | `/localization/slam_odom` |
| global_matcher aligned scan | `/localization/global_matcher/aligned_scan` |
| global_matcher submap | `/localization/global_matcher/target_submap` |
| registered cloud | `/localization/cloud_registered` |
| TF | `map -> odom -> base_link` |
| manager 状态 | `/localization/manager/status` |

---

## 10. 传感器退化能力分析

### 10.1 当前系统能否承受退化

按当前代码状态，结论分两层：

1. **现有代码立即运行**：`pose_fusion` 已有第一版融合输出，但仍不能完整承受系统级传感器退化。原因是 `localization_manager` 仍是占位节点，缺少统一初始化、退化状态机、观测降权和最终输出策略。
2. **按本文档实现后**：可以承受短时退化，尤其是 LiDAR 短时失效时，依赖 `dr_odometry` 的 IMU + wheel 预测继续输出短时间定位，同时标记 degraded，并在 LiDAR 恢复后通过 SLAM/global_matcher 校正回地图。

### 10.2 LiDAR 短时失效

LiDAR 暂时失效包括：

1. `/ouster/points` 无消息。
2. 点云点数过少。
3. SLAM feature count 过低。
4. `SlamHealth.is_degenerated=true`。
5. scan match residual 变大。

处理流程：

```text
LiDAR abnormal for < lidar_short_dropout:
  manager 标记 slam WARNING
  pose_fusion 降低 SLAM 权重
  DR 继续传播 fused pose
  global_matcher 暂停或等待有效 cloud
  output_valid=true, level=WARNING/DEGRADED

LiDAR abnormal for lidar_short_dropout ~ lidar_long_dropout:
  manager 标记 DEGRADED
  pose_fusion 只使用 DR + optional GNSS
  fusion covariance 持续增长
  限制可接受最大速度/时间窗口

LiDAR abnormal > lidar_long_dropout:
  若 GNSS 高质量且场景允许 -> GNSS_CONSTRAINED
  否则进入 RELOCALIZING 或 FAILURE
```

能保持的效果：

| 失效时间 | 预期能力 |
|---|---|
| `< 1s` | 基本无感，DR 可平滑补偿 |
| `1-3s` | 可继续短时运行，但横向/航向误差增长 |
| `3-10s` | 只能低速/保守运行，需要状态降级 |
| `> 10s` | 无 GNSS/地图约束时不应继续给高置信定位 |

限制：

1. DR 的 heading 主要靠 IMU yaw 积分，长时间会漂移。
2. wheel speed 是标量，只能约束前向速度，不能独立提供方向。
3. 如果 IMU yaw 初始方向错误，LiDAR 失效时 DR 会沿错误方向继续走。
4. 因此 LiDAR 恢复后必须通过 SLAM/global_matcher 校正，不应长期纯 DR 运行。

### 10.3 GNSS 退化

GNSS 退化包括遮挡、float、跳变、heading 不可信。

处理策略：

```text
GNSS quality poor:
  manager 禁用 gnss position 或降权
  若 heading 仍可信，可仅使用 heading
  pose_fusion 不允许 GNSS 触发 hard reset

GNSS jump detected:
  隔离 gnss_jump_quarantine_time
  记录 reject reason
  等连续稳定后再恢复低权重
```

室内/矿区/隧道中，GNSS 默认禁用。即使收到 `/gnss_chc_data`，也只做状态监控。

### 10.4 CAN 轮速退化

CAN 退化时：

1. `dr_odometry` 缺少 speed 观测，IMU 预测漂移会变快。
2. manager 将 DR 质量从 normal 降到 warning/degraded。
3. 如果 SLAM 正常，pose_fusion 仍可依赖 SLAM 维持。
4. 如果 LiDAR 与 CAN 同时失效，只剩 IMU，定位不应继续高置信输出。

处理：

```text
CAN timeout but IMU + SLAM valid:
  output_valid=true
  DR weight down

CAN timeout and LiDAR invalid:
  only IMU propagation
  quickly enter FAILURE unless GNSS valid
```

### 10.5 IMU 退化

IMU 是 DR 和 LiDAR-IMU SLAM 的基础传感器。IMU 失效比 LiDAR 失效更严重。

处理：

| 情况 | 策略 |
|---|---|
| 短时丢样 | 允许插值或跳过，增大过程噪声 |
| 时间倒退 | 拒绝该段数据，标记 warning |
| 长时间无 IMU | DR failure，SLAM 很可能 failure，进入 FAILURE |

### 10.6 多传感器组合退化

| 可用组合 | 是否可继续定位 | 策略 |
|---|---|---|
| DR + SLAM + global_matcher | 是 | 正常定位 |
| DR + SLAM，无 global_matcher | 是 | 局部定位，可能全局漂移 |
| DR + global_matcher，LiDAR registered cloud 可用 | 是 | 地图约束可维持 |
| DR only | 短时可以 | DEGRADED，协方差快速增长 |
| SLAM only | 可调试，不建议最终输出 | PURE_SLAM 或 LOCAL_FUSION |
| GNSS + DR | 露天可用 | GNSS_CONSTRAINED |
| IMU only | 不应长期可用 | 很快 FAILURE |
| 无 DR，无 SLAM，无 GNSS | 不可用 | FAILURE |

### 10.7 退化时对规划/控制的输出

manager 对外必须明确定位可信等级：

| 等级 | 建议下游行为 |
|---|---|
| `NORMAL` | 正常规划控制 |
| `WARNING` | 保持运行，限制激进行为 |
| `DEGRADED` | 降速、缩短规划 horizon、准备停车 |
| `FAILURE` | 停车或进入安全策略 |

manager 不直接控制车辆，但必须给规划/控制足够明确的状态。

---

## 11. 重定位验收

`relocalization` 输出不能直接 reset `pose_fusion`。manager 必须验收：

```text
candidate
  -> descriptor score gate
  -> fine matcher gate
  -> map_id gate
  -> physical consistency gate
  -> second frame confirmation, optional
  -> accepted_relocalization
  -> pose_fusion hard reset
```

推荐阈值：

| 指标 | 默认值 |
|---|---:|
| Scan Context top score | 按数据库标定 |
| GICP accept fitness | `< 0.5-1.0` |
| min inlier ratio | `> 0.5-0.7` |
| max inlier RMSE | `< 0.3-0.8 m` |
| yaw diff when current pose trusted | `< 20 deg` |
| position diff when current pose trusted | `< 5 m` |
| 二次确认间隔 | `1-2 frames` |

如果当前 fusion 已 failure，物理一致性门限可放宽，但必须依赖 GICP 和二次确认。

---

## 12. global_matcher 编排策略

global_matcher 是达到 ysw_loc 定位效果的关键。manager 要把它作为正常定位链路的一部分，而不是只在丢失时触发。

### 12.1 触发条件

| 条件 | 触发策略 |
|---|---|
| 正常定位 | 当前由 global_matcher `match_rate` 周期运行；后续 manager trigger 为 0.5-2Hz |
| fusion 协方差增长 | 提高触发频率 |
| SLAM 轻度退化 | 提高触发频率 |
| GNSS 从可用变不可用 | 过渡期提高触发频率 |
| 地图段切换 | 先发 `load_map_command`/`set_map_segment_command`，后续 trigger service 完成后强制触发一次 |
| 人工命令 | 立即触发 |

### 12.2 抢占规则

1. 当前周期匹配模式下，manager 不重复启动节点；后续 trigger service 模式下，global_matcher 正在匹配时不重复触发。
2. relocalization 正在执行时，暂停 global_matcher 结果进入 fusion。
3. global_matcher 大残差结果不能自行 hard reset；只能作为观测或上报 manager。
4. global_matcher 连续失败不等于定位丢失，需要结合 fusion/SLAM/DR 状态判断。

---

## 13. ROS 接口设计

### 13.1 订阅

| Topic | Type | 说明 |
|---|---|---|
| `/localization/dr_odom` | `nav_msgs/Odometry` | DR 输出 |
| `/localization/dr_status` | `deeplumin_msgs/LocalizationStatus` | DR 状态 |
| `/localization/slam_odom` | `nav_msgs/Odometry` / `deeplumin_msgs/SlamPose` | SLAM 输出 |
| `/localization/slam_health` | `deeplumin_msgs/SlamHealth` | SLAM 健康 |
| `/localization/fused_odom` | `nav_msgs/Odometry` | 最终定位 |
| `/localization/fusion_status` | 建议新增 `FusionStatus` | fusion 状态 |
| `/localization/global_matcher/status` | `deeplumin_msgs/LocalizationStatus` | 匹配状态 |
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | 匹配结果 |
| `/localization/relocalization/result` | `deeplumin_msgs/RelocalizationResult` | 重定位候选 |
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | GNSS |
| `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` | CAN 轮速 |
| `/ouster/imu` | `sensor_msgs/Imu` | IMU |
| `/ouster/points` | `sensor_msgs/PointCloud2` | LiDAR |
| `/system/scene_state` | 建议新增或 std_msgs/String | 场景状态 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 人工初始化位姿 |

### 13.2 发布

| Topic | Type | 说明 |
|---|---|---|
| `/localization/manager/status` | `deeplumin_msgs/LocalizationStatus` | 聚合定位状态 |
| `/localization/manager/observer_policy` | `deeplumin_msgs/ObserverPolicy` | 观测策略 |
| `/localization/manager/accepted_relocalization` | `deeplumin_msgs/RelocalizationResult` | 已验收重定位 |
| `/localization/manager/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` 或统一 reset 消息 | manager 验收后的初始化事件 |
| `/localization/manager/events` | `diagnostic_msgs/KeyValue[]` 或自定义 | reset/触发/拒绝事件 |

### 13.3 服务和命令接口

| 接口 | 说明 |
|---|---|
| `/localization/manager/set_mode` | 切换 `LOCALIZATION/PURE_DR/PURE_SLAM/MAPPING` |
| `/localization/manager/set_scene` | 手动设置 `OUTDOOR/INDOOR/TUNNEL/MINE` |
| `/localization/global_matcher/load_map_command` | 当前已实现的 topic 命令，`std_msgs/String`，格式 `map_id|pcd_path[|activate]` |
| `/localization/global_matcher/set_map_segment_command` | 当前已实现的 topic 命令，`std_msgs/String`，格式 `map_id` |
| `/localization/manager/trigger_global_match` | 后续手动触发 global_matcher，一期可仅记录事件 |
| `/localization/manager/trigger_relocalization` | 手动触发 relocalization |
| `/localization/manager/reset_localization` | reset pose_fusion，并清理状态 |
| `/localization/manager/save_state` | 保存当前可靠定位，用于下次启动 |

---

## 14. 包结构

推荐结构：

```text
localization_manager/
├── CMakeLists.txt
├── package.xml
├── include/localization_manager/
│   ├── common/
│   │   ├── types.hpp
│   │   └── config.hpp
│   ├── core/
│   │   ├── manager_core.hpp
│   │   ├── health_monitor.hpp
│   │   ├── state_machine.hpp
│   │   ├── scene_policy.hpp
│   │   ├── trigger_scheduler.hpp
│   │   └── reloc_validator.hpp
│   └── ros/
│       ├── ros_adapter.hpp
│       └── param_loader.hpp
├── src/
│   ├── core/
│   ├── ros/
│   └── localization_manager_node.cpp
├── config/localization_manager.yaml
├── launch/localization_manager.launch
└── test/
```

---

## 15. 核心类设计

### 15.1 `ManagerCore`

```cpp
class ManagerCore {
public:
  bool initialize(const ManagerConfig& config);
  void update(const ros::Time& now);

  void onHealthUpdate(const ModuleHealth& health);
  void onFusionStatus(const FusionStatusView& status);
  void onRelocalizationResult(const RelocCandidate& candidate);
  void onGlobalMatchResult(const GlobalMatchView& result);
  void onManualCommand(const ManagerCommand& command);

  ManagerStatus status() const;
  ObserverPolicy observerPolicy() const;

private:
  HealthMonitor health_monitor_;
  StateMachine state_machine_;
  ScenePolicy scene_policy_;
  TriggerScheduler trigger_scheduler_;
  RelocValidator reloc_validator_;
};
```

### 15.2 `HealthMonitor`

职责：

1. 维护每个模块最后消息时间。
2. 根据 timeout 和质量字段更新健康等级。
3. 维护连续成功/失败计数。
4. 生成聚合健康摘要。

### 15.3 `ScenePolicy`

职责：

1. 根据场景决定 GNSS 使用方式。
2. 决定 SLAM/global_matcher 权重。
3. 生成 `ObserverPolicy`。
4. 处理 GNSS 到室内的过渡策略。

### 15.4 `TriggerScheduler`

职责：

1. 周期触发 global_matcher。
2. 按退化状态提高或降低触发频率。
3. 防止 global_matcher 和 relocalization 抢占。
4. 执行冷却时间控制。

### 15.5 `RelocValidator`

职责：

1. 验收 relocalization 候选。
2. 二次确认。
3. 生成 accepted/rejected 事件。
4. 给 pose_fusion 发送 reset 所需结果。

---

## 16. 配置模板

`config/localization_manager.yaml`：

```yaml
manager:
  rate_hz: 10.0
  startup_grace_sec: 3.0
  default_mode: LOCALIZATION
  default_scene: INDOOR
  publish_events: true

initialization:
  require_initial_pose: true
  allow_manual_initialpose: true
  allow_relocalization_init: true
  allow_gnss_init: false        # bag 地图测试默认 false
  prefer_source_order: [MANUAL_INITIALPOSE, RELOCALIZATION, GNSS]
  initialpose_timeout_sec: 10.0
  map_frame: map
  base_frame: base_link
  require_map_enu_calibration_for_gnss: true
  initial_position_cov_manual: 1.0
  initial_yaw_cov_manual_deg: 5.0
  initial_position_cov_reloc: 0.5
  initial_yaw_cov_reloc_deg: 2.0
  initial_position_cov_gnss: 2.0
  initial_yaw_cov_gnss_deg: 5.0

topics:
  dr_odom: /localization/dr_odom
  dr_status: /localization/dr_status
  slam_odom: /localization/slam_odom
  slam_health: /localization/slam_health
  fused_odom: /localization/fused_odom
  fusion_status: /localization/fusion_status
  global_matcher_status: /localization/global_matcher/status
  global_matcher_result: /localization/global_matcher/result
  relocalization_result: /localization/relocalization/result
  gnss: /gnss_chc_data
  can: /can_receive_info
  imu: /ouster/imu
  lidar: /ouster/points
  scene_state: /system/scene_state
  initialpose: /initialpose
  manager_status: /localization/manager/status
  observer_policy: /localization/manager/observer_policy
  accepted_relocalization: /localization/manager/accepted_relocalization
  manager_initial_pose: /localization/manager/initial_pose
  dr_sensor_policy: /localization/dr_odometry/set_sensor_policy

timeouts:
  imu: 0.1
  can: 0.5
  gnss: 1.0
  lidar: 0.3
  dr_odom: 0.2
  slam_odom: 0.5
  slam_health: 0.5
  fused_odom: 0.2
  global_matcher_status: 3.0
  relocalization_result: 5.0

degrade:
  enter_degraded_fail_count: 3
  exit_degraded_ok_count: 5
  trigger_reloc_fail_count: 5
  lidar_short_dropout_sec: 1.0
  lidar_long_dropout_sec: 5.0
  pure_dr_max_sec: 3.0
  failure_timeout_sec: 5.0
  max_fusion_position_std: 5.0
  max_fusion_yaw_std_deg: 15.0

scene:
  source: manual                 # manual / auto_gnss / auto_with_manual_override
  manual_override_timeout_sec: 30.0
  auto_gnss_confirm_count: 5
  auto_gnss_lost_count: 5

scene_policy:
  outdoor_open:
    use_gnss: true
    gnss_position_weight: 1.0
    gnss_heading_weight: 1.0
    use_global_matcher: true
  semi_outdoor:
    use_gnss: true
    gnss_position_weight: 0.3
    gnss_heading_weight: 0.5
    use_global_matcher: true
  indoor:
    use_gnss: false
    use_global_matcher: true
  tunnel:
    use_gnss: false
    use_global_matcher: true
  no_map:
    use_gnss: true
    use_global_matcher: false

global_matcher:
  enabled: true
  current_mode: auto_match_topic_monitor
  result_topic: /localization/global_matcher/result
  status_topic: /localization/global_matcher/status
  load_map_command_topic: /localization/global_matcher/load_map_command
  set_map_segment_command_topic: /localization/global_matcher/set_map_segment_command
  normal_period_sec: 1.0
  degraded_period_sec: 0.5
  min_trigger_interval_sec: 0.3
  cooldown_after_failure_sec: 1.0
  max_consecutive_fail_before_reloc: 5

relocalization:
  enabled: true
  require_second_confirmation: true
  max_attempts: 3
  cooldown_sec: 5.0
  accept_fitness: 0.8
  min_inlier_ratio: 0.5
  max_inlier_rmse: 0.8
  max_pose_diff_when_trusted: 5.0
  max_yaw_diff_deg_when_trusted: 20.0

dr_policy:
  runtime_control_enabled: false # P2 开启，P1 只通过 launch/config 控制
  bag_test_use_gnss_heading: true
  bag_test_use_gnss_velocity: false
  bag_test_use_gnss_position: false
  outdoor_use_gnss_heading: true
  outdoor_use_gnss_velocity: true
  outdoor_use_gnss_position: false

gnss:
  min_satellites: 10
  allow_heading_only: true
  jump_threshold_m: 3.0
  jump_quarantine_sec: 2.0
  min_speed_for_heading: 0.2
```

---

## 17. 开发阶段

### P0：manager 空框架

1. 建立 include/src/config/launch 目录。
2. 实现参数加载。
3. 实现 `/localization/manager/status` 周期发布。
4. 订阅 DR/SLAM/fusion/status 基础 topic。

验收：manager 能启动，并显示各模块在线/超时。

### P1：健康监控和状态机

1. 实现 `HealthMonitor`。
2. 实现 `StateMachine`。
3. 支持 `NORMAL/WARNING/DEGRADED/FAILURE`。
4. 对 LiDAR、IMU、CAN、GNSS、DR、SLAM、fusion 超时给出 reason。

验收：停止某个 topic 后，manager 能在规定时间内进入 warning/degraded/failure。

### P2：初始化编排和 pose_fusion 第一版联调

1. 订阅 `/initialpose`，生成 manual initialization candidate。
2. 订阅 relocalization result，验收后生成 relocalization initialization candidate。
3. 按配置决定是否允许可靠 GNSS 初始化；bag 测试默认关闭。
4. 向 pose_fusion 发布初始化/reset 事件，使其计算 `T_map_odom = T_map_base_init * inverse(T_odom_base_current)`。
5. 等待 `/localization/fused_odom` valid 后进入 `LOCALIZATION`。

验收：RViz 人工给定初始位姿后，`fused_path` 在 `map` 下与全局地图方向一致；DR/SLAM 原始 odom 方向不同不再影响最终 fused 输出。

### P3：ObserverPolicy、GNSS 场景策略和 DR 策略联调

1. 发布 `/localization/manager/observer_policy`。
2. 支持手动设置 scene。
3. 支持 GNSS 自动检测和手动覆盖策略。
4. 根据 GNSS 是否有且可靠，给出 DR 的 GNSS heading/velocity/position 使用策略。
5. 与 pose_fusion 联调 DR + SLAM。

验收：切换室内/露天策略后，pose_fusion 能按策略使用或拒绝 GNSS；bag 测试时 GNSS position 不会把系统钉死到 ENU，初值由 `/initialpose` 或 relocalization 提供。

### P4：global_matcher 编排

1. 第一阶段订阅已实现的 `/localization/global_matcher/result` 和 `/localization/global_matcher/status`，建立健康统计和失败计数。
2. 接入地图段命令话题：`/localization/global_matcher/load_map_command`、`/localization/global_matcher/set_map_segment_command`。
3. 检查 `map_id`、`success`、`converged`、`inlier_ratio`、`inlier_rmse` 和初值误差，决定是否允许结果进入 pose_fusion。
4. 后续实现 trigger service 后，再实现周期触发、退化触发、冷却和 pending 锁。

验收：manager 能监控 global_matcher 结果，地图段切换后只接受当前 `map_id` 的匹配结果；接入 trigger service 后，能周期触发并使最终轨迹贴合地图。

### P5：relocalization 编排和验收

1. 实现重定位触发服务。
2. 订阅 relocalization candidate。
3. 实现 `RelocValidator`。
4. 验收通过后发布 accepted relocalization 给 pose_fusion。

验收：模拟定位丢失后，manager 能触发重定位并恢复定位。

### P6：退化场景联调

1. rosbag 中屏蔽 LiDAR 1-3 秒。
2. 屏蔽 GNSS。
3. 屏蔽 CAN。
4. 模拟 SLAM 退化。
5. 验证 manager 状态、fusion 输出和恢复逻辑。

验收：LiDAR 短时失效时系统继续输出 degraded fused odom；LiDAR 恢复后回到 normal，并通过 global_matcher 拉回地图。

---

## 18. 测试和验收

### 18.1 单元测试

| 测试 | 内容 |
|---|---|
| `test_health_monitor` | timeout、连续计数、质量等级 |
| `test_state_machine` | INIT/LOCALIZATION/DEGRADED/RELOCALIZING/FAILURE 转移 |
| `test_scene_policy` | 不同场景下 ObserverPolicy |
| `test_trigger_scheduler` | global_matcher 触发频率、pending 锁、冷却 |
| `test_reloc_validator` | 重定位候选验收/拒绝 |

### 18.2 联调测试

| Case | 操作 | 预期 |
|---|---|---|
| 全模块正常 | DR+SLAM+global_matcher+fusion+manager | NORMAL，轨迹贴合地图 |
| LiDAR 短时失效 | rosbag 过滤 `/ouster/points` 1s | WARNING/DEGRADED，DR 维持输出 |
| LiDAR 长时失效 | 过滤 5s 以上 | 触发 relocalization 或 FAILURE |
| GNSS 跳变 | 注入 GNSS 大跳变 | GNSS 被拒绝，不 reset |
| global_matcher 连续失败 | 让匹配结果 failure | 降级，达到阈值后触发 relocalization |
| relocalization 成功 | 发布高质量候选 | manager 验收，fusion reset |
| relocalization 失败 | 发布低质量候选 | manager 拒绝，保留 degraded/failure |

### 18.3 ysw_loc 效果对齐

manager 本身不提升匹配精度，但它必须保证 ysw_loc 的关键闭环能稳定运行：

```text
SLAM/DR 连续里程计
  + global_matcher 地图匹配校正
  + pose_fusion 平滑输出
  + manager 退化/重定位编排
```

验收标准：

1. 正常场景下 `global_matcher` 周期参与定位，不是只在丢失后触发。
2. 最终轨迹与地图贴合程度不低于 ysw_loc。
3. 匹配失败不造成最终定位跳变。
4. LiDAR 短时失效后，轨迹短时连续，状态明确降级。
5. LiDAR 恢复后，系统能通过 SLAM/global_matcher 恢复地图约束。

---

## 19. 关键实现注意事项

1. manager 不直接发布最终定位 TF。
2. manager 不直接修改 `pose_fusion` 内部变量，只通过 topic/service。
3. `global_matcher` 和 `relocalization` 不能同时抢占 reset。
4. GNSS 场景策略必须保守，室内默认禁用。
5. LiDAR 失效时可以短时继续运行，但必须提升协方差并对外发布 degraded。
6. 只剩 IMU 时不应长期发布有效定位。
7. 所有拒绝原因必须写入 status/event，便于 rosbag 复盘。
8. 所有 reset 必须记录 reset 前后位姿差、来源、质量分数和时间戳。
9. 启动阶段应有 grace period，避免模块刚启动时被误判 failure。
10. 模块可独立调试，但进入完整定位链路时必须服从 manager 策略。

---

## 20. 最小可落地版本

为了尽快联调当前所有模块，推荐最小实现：

```text
1. localization_manager 订阅 dr_status、slam_health、fused_odom、fusion_status、/initialpose、GNSS
2. 完成 WAIT_INITIAL_POSE：优先 RViz /initialpose，其次 relocalization，最后可选可靠 GNSS
3. 给 pose_fusion 下发初始化/reset 事件，建立 `T_map_odom`，使 fused_path 与地图坐标一致
4. 周期发布 manager/status 和 observer_policy
5. 支持 scene 手动切换和 GNSS 自动检测，控制 GNSS 是否进入 fusion，并给出 DR GNSS 使用策略
6. 监控 LiDAR/SLAM 超时，LiDAR 短时失效时进入 DEGRADED
7. 监控 global_matcher 连续失败，后续接入 trigger service 后再主动调度
8. 手动或自动触发 relocalization
9. 验收 relocalization 后发布 accepted_relocalization 给 pose_fusion
```

该版本即可形成 DeepLumin 完整定位编排闭环，并为后续实现更复杂的自动场景识别、地图段切换和故障恢复打基础。
