# dr_odometry_v2 改造设计文档

> 版本：v1.0（设计 + 实施同步稿）
> 日期：2026-10-10
> 基线：`dr_odometry`（拷贝为 `dr_odometry_v2` 后改造）
> 关联：`localization_v3/docs/dr_odometry_fallback_design.md`——本文是其 P0（外部位姿观测接口）的完整落地，并按新要求重构整个模块。

---

## 0. 改造目标总览

| # | 要求 | 落点 |
|---|---|---|
| 1 | 数据源 IMU / 轮速 / GNSS / 定位结果全部开关配置；轮速 `(speed,0,0)` 约束；GNSS 姿态（yaw/pitch/roll）约束 + 可选 GNSS 加速度；定位结果多状态观测；ESKF 维度分析 | §3 状态模型、§4 观测模型 |
| 2 | 传感器时间同步：以谁为准、是否插值、定位结果时间对齐 | §5 时间同步 |
| 3 | 配置文件规范化（按功能分组、去冗余）；模块解耦、C++/ROS 解耦 | §6 模块结构、§7 配置 |
| 4 | 包名/CMake 改名，避免与 `dr_odometry` 同名冲突 | §2 |

**与 fallback 设计的关系**：`dr_odometry_v2` 订阅 `localization_v3` 的 status + map->base odom，在 status 正常时作观测标定零偏（fallback 文档 §4.1）；SLAM 断流后它就是备用里程计源。两个文档配合实施，观测接口/门控/跳变判断在本模块内实现。

---

## 1. v1 存在的问题（改造动因）

| # | 问题 | v1 位置 |
|---|---|---|
| 1 | GNSS 只有 heading 观测，无 roll/pitch 姿态约束 | `dr_eskf.cpp:260-275` |
| 2 | 无 GNSS 加速度利用；无外部定位观测（零偏不可观的关键缺口） | — |
| 3 | 轮速观测在世界系构造（`(v·cosψ, v·sinψ, 0)`），姿态耦合不进 H，yaw 无法被约束 | `dr_eskf.cpp:213-258` |
| 4 | 观测更新"挂在 IMU 回调里顺手做"，无时间对齐概念（新鲜度窗 + 取最近） | `dr_eskf.cpp:118-128` |
| 5 | 配置平铺、部分参数（`max_fitness` 等）语义重叠；C++/ROS 耦合（节点内混杂日志/发布/滤波调用） | `dr_odometry.yaml`、`dr_odometry_node.cpp` |
| 6 | 输出直接用最后 IMU 状态（滞后 0~1 个 IMU 周期），无前向外推 | `dr_odometry_node.cpp:125-137` |
| 7 | frames 与 SLAM 同名 `odom`；包名拷贝后与原包冲突 | `dr_odometry.yaml`、`package.xml` |

---

## 2. 命名迁移（同名冲突规避）

| 项 | dr_odometry | dr_odometry_v2 |
|---|---|---|
| 包名 / project | `dr_odometry` | **`dr_odometry_v2`** |
| 命名空间 | `dr_odometry::` / `dr_odometry_ros::` | **`dr_odometry_v2::` / `dr_odometry_v2_ros::`** |
| include 目录 | `include/dr_odometry/` | `include/dr_odometry_v2/` |
| 可执行 | `dr_odometry_node` | **`dr_odometry_v2_node`** |
| 库目标 | —（无独立库） | `dr_odometry_v2_core` / `dr_odometry_v2_ros` |
| 测试目标 | — | `test_v2_eskf` / `test_v2_time_sync` / `test_v2_pose_gate`（**带 v2 前缀**，避免 catkin 单构建树目标重名——localization_v3 的教训） |
| config / launch | `dr_odometry.yaml` | `dr_odometry_v2.yaml` / `dr_odometry_v2.launch` |

---

## 3. 状态模型：维度分析

**结论：采用 15 维 ESKF：`x = [p(3), v(3), θ(3), b_g(3), b_a(3)]`。**

| 方案 | 状态 | 评估 |
|---|---|---|
| **15 维（采用）** | p, v, θ, bg, ba | 标准惯性误差状态；观测充足时全部可观（见下） |
| 16 维（备选） | + 轮速标度 s | 标度误差影响短时桥接 ≤ 里程 1~2%（车辆标定前提），收益有限；**接口预留**，bag 回归发现标度漂移明显再扩展 |
| 18~21 维 | + 重力方向/安装角 | 重力固定 z-up（LIO/DR 一致），安装角已被 extrinsics 覆盖；无必要 |

**可观性（本模块观测配置下）**：

| 状态 | 观测来源 | 可观性 |
|---|---|---|
| θ（含 yaw） | GNSS 姿态（ypr）/ 定位位姿 | ✅ 直接观测（v1 的 yaw 不可观问题解决） |
| bg | θ 观测 + F 耦合 `δθ' = -δbg` | ✅ |
| v | 轮速 `(s,0,0)` / GNSS 速度 / 定位 twist | ✅ |
| ba | v/p 观测链 + 加速度观测 | ✅ |
| p | 定位位姿 / GNSS 位置 / 积分 | ✅ |

**误差状态与注入约定**（全模块统一）：右扰动 `q ← q ⊗ δq(δθ)`，`δθ` 在机体系；协方差 15×15 块序 `[p|v|θ|bg|ba]`。过程模型（连续→离散 F/Q）沿用 v1 已验证形式：

```
δp' = δv
δv' = -R[a]_× δθ - R δb_a        （use_accel=true 时）
δθ' = -δb_g
```

---

## 4. 观测模型（全部开关配置）

### 4.1 总览

| 源 | 开关（默认） | 观测 | 状态 | 观测形式 |
|---|---|---|---|---|
| IMU | `imu/enabled`(true)、`imu/use_accel`(true) | gyro/accel | — | **预测**（中值积分） |
| 轮速 | `wheel/enabled`(true) | `(speed,0,0)` | v, θ | body 系速度观测 |
| GNSS 姿态 | `gnss/enabled` + `gnss/use_attitude`(true) | yaw/pitch/roll | θ | 姿态观测（3 维） |
| GNSS 加速度 | `gnss/use_accel`(false) | acc_x/y/z | θ, ba | 比力/加速度观测 |
| GNSS 速度 | `gnss/use_velocity`(false) | ve/vn/vu | v | ENU 速度观测 |
| GNSS 位置 | `gnss/use_position`(false) | lat/lon/alt | p | ENU 位置观测 |
| 定位结果 | `localization/enabled`(true) | map->base + twist | p, θ, v | 位姿/速度观测 + 跳变判断 |

### 4.2 各观测模型

**① 轮速：`v_body = (speed, 0, 0)`（用户指定形式）**

```
h(x) = R(θ)ᵀ v                    （机体系速度）
z    = (±speed, 0, 0)              （gear 倒档取负）
r    = z - h(x)
H_v  = -Rᵀ,  H_θ = [Rᵀv]_×        （右扰动线性化）
R    = diag(σ_wheel², σ_nhc², σ_nhc²)   （纵向轮速噪声 / 非完整约束噪声）
```

对比 v1 的世界系构造：本形式**姿态进入 H**，运动时 yaw 误差体现为横向速度残差→可被约束（配合非完整约束）。

**② GNSS 姿态（yaw/pitch/roll 全姿态）**

```
R_meas = Rz(yaw)·Ry(pitch)·Rx(roll)      （由 GPCHC heading/pitch/roll 构造，含 heading_offset_deg 配置）
r = Log(R(θ)·R_measᵀ) ≈ δθ               （3 维姿态残差，小角度）
H_θ = I
R   = diag(σ_roll², σ_pitch², σ_yaw²)
```

- `gnss/heading_offset_deg` 用于对齐航向约定（GPCHC 北基准/顺时针 vs ENU 逆时针），默认 0 与 v1 行为一致；
- 低速时航向不可信：保留 `gnss/min_speed_for_heading`（仅对 yaw 分量拉大 R）。

**③ GNSS 加速度（可选，`gnss/use_accel`）**

约定可配 `gnss/accel_frame: world | body`（GPCHC acc 字段约定随厂商不同）：

```
world: h(x) = R(θ)(acc_m - b_a) + g       →  H_θ = -R[acc_m-b_a]_×,  H_ba = -R
body : h(x) = acc_m - b_a                 →  H_ba = -I
```

观测的是比力一致性 → 约束 [θ, ba]。噪声 `gnss/accel_noise`。

**④ GNSS 速度 / 位置（可选，保留 v1 能力）**

```
速度: h = v,  z = velocity_enu        → H_v = I
位置: h = p,  z = lla→enu(lat,lon,alt)→ H_p = I   （首帧锁 ENU 原点，同 v1）
```

**⑤ 定位结果（map->base + twist，`localization/enabled`）——多状态观测**

```
位姿: z = T_map_base   → r_p = p_obs - p,  r_θ = Log(R R_obsᵀ)     H_p = I, H_θ = I
速度: z = v_body(twist)→ r = z - Rᵀv                               （同轮速线性化）
```

**跳变判断（防伪跳变污染，接入 fallback 文档 §4.1.2）**——独立 `PoseGate` 模块：

| 档 | 条件（默认） | 处理 |
|---|---|---|
| 小跳 | Δ ≤ 0.5m / 2° | 直接收 |
| 大跳 | 0.5~5m / 2~15° | 连续 `confirm_frames(3)` 帧互相一致才接受（台阶修正：只修 p/θ，零偏不动） |
| 超限 | > 5m / 15° | 拒收（伪跳变），计数 |

**观测门控**（`localization/*`）：`status.state==TRACKING && output_valid && last_slam_age < 0.2s`（防"DR 模式回喂自己"闭环）；可选 `require_level_normal`；观测延迟超 `max_latency` 拒收。

---

## 5. 时间同步（用户要求 2）

**结论：以 IMU 时间戳为滤波主轴；观测在时间序上"传播到观测时刻再更新"（天然对齐，无需事后插值）；输出前向外推到发布时刻。**

### 5.1 时间基准选择

| 候选 | 评价 |
|---|---|
| **IMU 时间（采用）** | 最高频（~200Hz）、预测本来就是 IMU 驱动；所有状态语义挂在 IMU 时间轴 |
| 定位结果时间 | 100Hz 且是处理后输出，有处理延迟，不能当主轴 |
| ROS 时间 now | 受调度抖动，不能当主轴 |

### 5.2 事件时间序处理（核心机制）

```
事件队列（按 stamp 排序，容量上限）:
  ImuSample / WheelSample / GnssSample / PoseSample（定位）

处理循环（core 内，零 ROS）：
  for e in 队列（时间升序）:
    if e 是 IMU:  predict 到 e.stamp（与上一 IMU 中值积分）
    else:        propagate 到 e.stamp（IMU 输入 ZOH，dt≤max_dt）→ 门控 → 观测更新
```

- **对齐方式**：状态被传播到观测的**精确时刻**后更新——观测与状态同刻，这是最干净的"时间对齐"，优于事后插值观测；
- **是否需要插值**：观测本身**不需要插值**（按各自 stamp 处理）；需要插值的是 **IMU 输入**（两帧 IMU 之间到达的观测，用最近 IMU 测量 ZOH 或线性插值 gyro/accel）——实现取 ZOH（误差 ≤ dt·dot(ω) 可忽略）；
- **乱序/迟到**：`stamp < 已处理时间` → 丢弃并计数（`reordered_drop_count`）；v1 不做延迟缓冲回溯；
- **重复/零戳**：dt≤0 跳过；`header.stamp.isZero()` → 用接收时间兜底 + 计数告警；
- **定位结果对齐**（重点）：定位 `map->base` 的 stamp 是其发布时刻——入队后按时间序在该时刻被应用（状态已传播到该时刻）；延迟超 `localization/max_latency(0.5s)` 拒收（数据太旧污染滤波）。

### 5.3 输出时间

输出时刻 `t_out`（100Hz 定时）可能晚于最后 IMU：**前向外推**——用当前状态 + 最近 IMU 输入传播 `Δt ≤ output/max_extrapolate(0.1s)`，输出 stamp = t_out。超时无 IMU → 停发（等同 v1 未初始化行为）。

---

## 6. 模块结构与解耦

```
dr_odometry_v2/
├── include/dr_odometry_v2/
│   ├── core/                        【纯 C++，零 ROS，全部可单测】
│   │   ├── types.hpp                DrState / 各传感器样本 / DrConfig / OdomResult / Health
│   │   ├── math_utils.hpp           四元数、Log/Exp、插值、skew
│   │   ├── eskf.hpp/.cpp            15 维 ESKF：predict / 6 个观测模型 / update / inject
│   │   ├── event_queue.hpp/.cpp     时间序事件队列（去乱序、上限、统计）
│   │   └── pose_gate.hpp/.cpp       定位观测跳变判断（三档）
│   ├── ros/                         【薄适配：消息↔core、参数加载】
│   │   ├── config.hpp/.cpp          分组配置结构 + 加载
│   │   ├── converters.hpp/.cpp      IMU/CAN/Gpchc/FusionStatus/Odometry ↔ core
│   │   └── map_viz.hpp/.cpp         全局地图可视化发布（仅 RViz，算法不消费）
│   └── （无业务逻辑头文件）
├── src/
│   ├── core/*.cpp
│   ├── ros/*.cpp
│   └── dr_odometry_v2_node.cpp      【只编排：订阅→convert→入队→drain→发布】
├── config/dr_odometry_v2.yaml       按功能分组（§7）
├── launch/dr_odometry_v2.launch
├── test/test_v2_eskf.cpp · test_v2_event_queue.cpp · test_v2_pose_gate.cpp
└── docs/dr_odometry_v2_design.md
```

**解耦规则**：core 不 include 任何 ros/*.h；ros 不含滤波逻辑；node 不含观测数学。日志集中在 node 层（SYS_LOG），core 只返回结构化结果/计数。

---

## 7. 配置文件（按功能分组、去冗余）

```yaml
frames:            {odom: odom, base_link: base_link}
topics:            {imu, can, gnss, localization_pose, localization_status, odom, status, path}
extrinsics:        {base_to_imu, base_to_gnss, base_to_can}      # T_base_sensor

imu:               # 预测（必开）
  enabled: true
  use_accel: true              # 预测是否用加速度（false 则速度靠观测维持）
  max_dt: 0.05
  gyro_noise: 0.01
  accel_noise: 0.2
  gyro_bias_noise: 1.0e-5
  accel_bias_noise: 1.0e-4
  gravity: 9.80665

wheel:             # 轮速 (speed,0,0) 观测
  enabled: true
  speed_is_kmh: true
  use_valid_flag: false
  velocity_noise: 0.25         # 纵向
  nonholonomic_noise: 0.10     # 侧向/垂向
  timeout: 0.5

gnss:              # 姿态为主，加速度可选，速度/位置保留
  enabled: true
  use_attitude: true
  attitude_noise_roll_deg: 1.0
  attitude_noise_pitch_deg: 1.0
  attitude_noise_yaw_deg: 2.0
  heading_offset_deg: 0.0
  min_speed_for_heading: 0.2
  use_accel: false
  accel_frame: world           # world | body
  accel_noise: 0.5
  use_velocity: false
  velocity_noise: 0.4
  use_position: false
  position_noise: 1.5
  timeout: 1.0

localization:      # 定位结果观测（辅助标定核心）
  enabled: true
  pose_topic: /localization_v2/odom
  status_topic: /localization_v2/status
  require_tracking: true       # status.state==TRACKING && output_valid
  require_slam_fresh: true     # last_slam_age < slam_age_limit（防闭环回喂）
  slam_age_limit: 0.2
  max_latency: 0.5
  pose_noise_trans: 0.05
  pose_noise_yaw_deg: 0.5
  vel_noise: 0.2
  jump_soft_trans: 0.5         # 跳变判断三档
  jump_soft_yaw_deg: 2.0
  jump_hard_trans: 5.0
  jump_hard_yaw_deg: 15.0
  jump_confirm_frames: 3
  jump_confirm_consistency_trans: 0.5
  jump_confirm_consistency_yaw_deg: 2.0

output:            # 发布
  rate: 100.0
  publish_tf: false
  path_max_size: 5000
  max_extrapolate: 0.1

time_sync:         # 时间同步
  queue_size: 512
  reordered_drop_warn: 100
```

对比 v1 去除：`dr/output_rate` 与 `output/rate` 合并；`map/*`（全局地图可视化，调试遗留）移入 debug 工具不在正式节点；`can_*` 顶层散参并入 `wheel/`；`initial_yaw_deg`/`initial_position` 删除（定位观测 reanchor 定基准，冷启动由 GNSS 姿态定）。

---

## 8. 实施计划与验收

| 阶段 | 内容 | 验收 |
|---|---|---|
| P0 | 命名迁移（包/CMake/命名空间/目录），可编译 | 与 `dr_odometry` 同工作空间编译无冲突 |
| P1 | core：types/math_utils/eskf（predict+6 观测）/event_queue/pose_gate + 单测 | 单测：零偏收敛、各观测 H 数值检查（有限差分）、时间序、跳变三档 |
| P2 | ros：config/converters + node + yaml + launch | 跑 bag 出 odom/status |
| P3 | bag 验证：辅助标定 A/B（开/关 localization 观测）漂移率对比；时间对齐检查 | 漂移率降一个量级；无时间戳告警 |
| P4 | 与 localization_v3 集成（fallback 文档 P1~P5） | 断流接管全程验证 |

**单测重点**（core 可测性是本次重构收益）：

1. `test_v2_eskf`：predict 数值正确；每个观测模型 H 与有限差分一致；15 维协方差对称正定；
2. `test_v2_event_queue`：乱序丢弃、时间升序处理、零戳兜底；
3. `test_v2_pose_gate`：小跳直收 / 大跳 3 帧确认 / 超限拒收、确认后零偏不动。

---

## 9. 集成点（与 localization_v3 / fallback 方案）

| 交互 | 话题 | 说明 |
|---|---|---|
| 输入：定位位姿/状态 | `/localization_v2/odom`、`/localization_v2/status` | 辅助标定观测源（§4.2⑤） |
| 输出：备用里程计 | `/localization/dr_odom`（保持 v1 话题名） | LocalOdomSource 消费（fallback 文档 §3.2） |
| 输出：状态 | `/localization/dr_status` | 健康/计数/门控统计 |
| DR 接管期 | — | 观测自动关闭（status 门控防闭环），纯推算 |

frames 输出沿用 `odom->base_link`（`publish_tf=false`，由 localization_v3 拥有 TF）；v1 frames 同名问题在 P4 集成时按 fallback 文档 §4.6 建议处理（可配）。

---

## 附录 A：2026-07-14 bag 实测校准记录（25s 段）

**数据**：`2026-07-14-11-17-21.bag`（IMU 10Hz / GPCHC 19Hz / CAN 7Hz），前 25s：静止 14s + 机动 7m。

**发现并修复：航向约定**——GPCHC heading 为**罗盘约定（北基准顺时针）**，原实现按 ENU 逆时针直接用导致航向系统性错（"反号+90° 基准"，常数 offset 不可修）。判定依据：行驶段"航向 vs 瞬时轨迹方向"平均偏差，罗盘解释 6.0° vs ENU 解释 32.6°；与 lio_odometry 对 GPCHC heading 取负的处理同源。修复：`gnss/heading_convention: compass|enu`（默认 compass，`yaw = π/2 - heading`），`GnssSample.yaw` 改为存原始 `heading`、约定换算集中在 eskf。

**修复前后对比**（同一 25s 段，真值 = GNSS lat/lon→ENU）：

| 指标 | 修复前（ENU 误约定） | 修复后（compass） |
|---|---|---|
| 轨迹 RMSE | 1.27 m | **0.58 m**（↓54%） |
| 最大/终点误差 | 4.41 m | **1.61 m**（↓64%） |
| 行驶段 \|航向−轨迹方向\| | 16.6° | **9.0°**（转弯侧滑残差） |
| 终点 yaw vs 真值换算 | 语义错误 | **−79.8° vs −79.6°（差 0.2°）** |
| 静止漂移（14s） | 1.8 cm | 1.8 cm（不变） |

**结论**：IMU 预测 + 轮速 (speed,0,0) + GNSS 姿态观测的三源融合推理**有效**；残余 ~1.6m/25s 以纵向积分误差为主（轮速标度/滑移），属"短时桥接"固有量级，集成定位观测后由辅助标定进一步收敛。

**遗留观察项**：
- GNSS 安装外参（`base_to_gnss`）尚未应用到 ypr/acc（本数据 roll/pitch 小，影响待长测）；
- 传感器时钟与 bag 记录时间不同源（差约 9 个月），三源同钟即可，与时间序处理无冲突；
- pitch/roll 符号约定待长测确认（罗盘/北东地与 ENU 的俯仰符号差异）。
