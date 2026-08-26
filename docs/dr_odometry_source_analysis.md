# DeepLumin `dr_odometry` 源码分析

> 包路径：`DeepLumin/src/localization/dr_odometry/`  
> 适用版本：当前仓库实现（15 维误差状态 ESKF）  
> 阅读目标：理解模块职责、两个节点流程、参数含义、数学原理，并能对照源码定位函数。

---

## 目录

1. [模块定位与职责](#1-模块定位与职责)
2. [目录结构与分层](#2-目录结构与分层)
3. [坐标系与消息约定](#3-坐标系与消息约定)
4. [数据流总览](#4-数据流总览)
5. [两个节点的详细流程](#5-两个节点的详细流程)
6. [参数说明](#6-参数说明)
7. [核心类型（types）](#7-核心类型types)
8. [ROS 适配层（ros_adapter）](#8-ros-适配层ros_adapter)
9. [数学工具（math_utils）](#9-数学工具math_utils)
10. [ESKF 核心（DrEskf）：函数级说明](#10-eskf-核心dreskf函数级说明)
11. [详细数学原理](#11-详细数学原理)
12. [与 localization_ws2 DR 的差异](#12-与-localization_ws2-dr-的差异)
13. [调试与常见问题](#13-调试与常见问题)
14. [源码阅读路径建议](#14-源码阅读路径建议)

---

## 1. 模块定位与职责

`dr_odometry` 是 DeepLumin 定位域中的**航位推算（Dead Reckoning）**子包。它用：

- **IMU**：高频预测姿态、速度、位置；
- **CAN 轮速**：约束车体前进速度，并施加非完整约束（侧向/垂向速度≈0）；
- **可选 GNSS（GPCHC）**：航向 / ENU 速度 / ENU 位置观测；

输出局部里程计 `/localization/dr_odom`，供下游融合或可视化使用。

| 项目 | 说明 |
|------|------|
| 算法 | 15 维误差状态卡尔曼滤波（ESKF） |
| 时钟主轴 | IMU 回调驱动预测与观测更新 |
| 发布频率 | `dr/output_rate`（默认 100 Hz），与 IMU 预测解耦 |
| ROS 解耦 | 核心在 `dr_odometry::DrEskf`，无 ROS 类型 |

**不做的事：**

- 不做地图匹配、SLAM；
- 不查 IMU/GNSS 外参 TF（当前按“传感器已对齐到车体近似”处理）；
- 不做轮速尺度在线标定、ZUPT（相对 `localization_ws2` 更简化）。

---

## 2. 目录结构与分层

```text
dr_odometry/
├── package.xml / CMakeLists.txt
├── config/dr_odometry.yaml          # 参数
├── launch/
│   ├── dr_odometry.launch           # 正式节点
│   └── dr_odometry_debug.launch     # 调试节点（可消融观测）
├── include/dr_odometry/
│   ├── core/
│   │   ├── types.hpp                # 数据结构
│   │   ├── math_utils.hpp           # 姿态/反对称等小工具
│   │   └── dr_eskf.hpp              # ESKF 接口
│   └── ros/ros_adapter.hpp          # ROS ↔ 核心
└── src/
    ├── dr_odometry_node.cpp         # 正式节点
    ├── dr_odometry_debug_node.cpp   # 调试节点
    ├── core/dr_eskf.cpp
    └── ros/ros_adapter.cpp
```

分层：

```text
┌─────────────────────────────────────────┐
│  ROS 节点层                               │
│  dr_odometry_node / debug_node         │
│  订阅、发布、TF、主循环频率                 │
├─────────────────────────────────────────┤
│  ROS 适配层  dr_odometry_ros::*           │
│  参数加载、消息转换                         │
├─────────────────────────────────────────┤
│  算法核心层  dr_odometry::DrEskf          │
│  预测 / 轮速更新 / GNSS 更新 / ENU 转换     │
└─────────────────────────────────────────┘
```

编译产物：

| 目标 | 说明 |
|------|------|
| `dr_odometry_core` | 库：`dr_eskf.cpp` + `ros_adapter.cpp` |
| `dr_odometry_node` | 正式可执行文件 |
| `dr_odometry_debug_node` | 调试可执行文件 |

---

## 3. 坐标系与消息约定

### 3.1 坐标系

| 名称 | 含义 |
|------|------|
| **world / odom** | 近似局部 ENU：X 东、Y 北、Z 天；重力 (0,0,-g) |
| **body / base_link** | 车体；x 前、y 左、z 上（与常见 ROS 车体一致） |
| 姿态四元数 q | **body → world**，即 v_w = R(q)v_b |

### 3.2 航向约定（重要）

GPCHC `heading`：

- 0° = 北，**顺时针**为正（导航惯例）。

本模块内部 yaw（ENU）：

- 0 rad = 东，**逆时针**为正。

转换（`ros_adapter.cpp`）：

```text
yaw_ENU = normalize( (90° - heading_deg) * pi / 180 )
```

### 3.3 话题与消息

| 方向 | 默认话题 | ROS 类型（DeepLumin） |
|------|----------|------------------------|
| 入 | `/ouster/imu` | `sensor_msgs/Imu` |
| 入 | `/can_receive_info` | `deeplumin_msgs/CanReceiveInfo` |
| 入 | `/gnss_chc_data` | `deeplumin_msgs/Gpchc` |
| 出 | `/localization/dr_odom` | `nav_msgs/Odometry` |
| 出 | `/localization/dr_path` | `nav_msgs/Path` |
| 出 | `/localization/dr_status` | `deeplumin_msgs/LocalizationStatus` |
| 出 | TF `odom → base_link` | （可选） |

> 旧 bag 常标为 `msgs/gpchc` 等。字段 MD5 相同时，节点仍可订阅；`rostopic echo` 可能因本地没有 `msgs` 包失败。

---

## 4. 数据流总览

```text
 bag / 驱动
   │
   ├─ IMU  ──► fromRos ──► feedImu ──► predict
   │                              │
   │                              ├─► correctWheel（若 use_wheel 且新鲜）
   │                              └─► correctGnss （若 use_gnss 且新鲜）
   │
   ├─ CAN  ──► fromRos ──► feedCan （只缓存最新）
   │
   └─ GNSS ──► fromRos ──► feedGnss（缓存；可定 ENU 原点 / 预置 yaw）

 主循环 @ output_rate
   └─ odometry() / health() ──► toRos* ──► 发布 odom/path/status/TF
```

要点：

1. **滤波计算在 IMU 回调里完成**；主循环只负责按固定频率取样发布。
2. CAN / GNSS **异步到达**，在下一帧 IMU 上用“新鲜度”判断是否进入更新。
3. 未收到首帧有效 IMU 前，`odometry().valid == false`，节点不发布位姿。

---

## 5. 两个节点的详细流程

### 5.1 对比一览

| 项目 | `dr_odometry_node` | `dr_odometry_debug_node` |
|------|--------------------|---------------------------|
| Launch | `dr_odometry.launch` | `dr_odometry_debug.launch` |
| 节点名 | `/dr_odometry` | `/dr_odometry_debug` |
| 观测开关 | 仅 yaml `dr/*` | launch `debug/*` **覆盖** yaml |
| IMU 订阅 | 始终订阅 | `debug/use_imu` 可关 |
| CAN 订阅 | 始终订阅 | `debug/use_wheel` 控制 |
| GNSS 订阅 | `dr/use_gnss` | `debug/use_gnss` |
| Path 上限 | 5000（删 1000） | 10000（删 2000） |
| 用途 | 在线生产 | 消融实验 / 联调 |

### 5.2 正式节点流程（`dr_odometry_node.cpp`）

**启动：**

```bash
roslaunch dr_odometry dr_odometry.launch
# 或自定义配置
roslaunch dr_odometry dr_odometry.launch config:=/path/to.yaml
```

**构造阶段：**

1. `loadDrConfig` / `loadTopicConfig` / `loadFrameConfig`
2. 读 `can_speed_is_kmh`
3. `eskf_.initialize(config_)`
4. 订阅 IMU、CAN；若 `use_gnss` 则订阅 GNSS
5. advertise odom / status / path

**运行循环 `spin()`：**

```text
while ok:
  spinOnce()          # 触发 onImu / onCan / onGnss
  publish()           # 取最新状态发出
  sleep(1/output_rate)
```

**回调：**

| 回调 | 动作 |
|------|------|
| `onImu` | `eskf_.feedImu(fromRos(*msg))` |
| `onCan` | `eskf_.feedCan(fromRos(*msg, can_speed_is_kmh_))` |
| `onGnss` | `eskf_.feedGnss(fromRos(*msg))` |

**`publish()`：**

1. `odom = eskf_.odometry()`；`!valid` 则 return  
2. `toRosOdom` → 发布  
3. 若 `publish_tf` → `toRosTransform`  
4. 追加 Path 并截断  
5. `toStatusMsg(health(now))` → 发布  

### 5.3 调试节点流程（`dr_odometry_debug_node.cpp`）

**启动：**

```bash
roslaunch dr_odometry dr_odometry_debug.launch
roslaunch dr_odometry dr_odometry_debug.launch use_gnss:=false
roslaunch dr_odometry dr_odometry_debug.launch use_gnss_heading:=false use_gnss_velocity:=false
```

**参数加载顺序（重要）：**

1. `rosparam load` yaml → 参数服务器（全局 `frames/`、`topics/`、`dr/`…）
2. launch 写入私有参数 `~/debug/use_*`
3. 节点先 `loadDrConfig`（拿到 yaml 的 `dr/*`）
4. 再用 `pnh_.param("debug/...")` **覆盖** `config_` 中对应开关
5. `debug/use_imu` 只影响是否创建 IMU 订阅（不写入 `DrConfig`）

其余主循环与正式节点相同：`spinOnce` → `publish`。

**消融实验建议：**

| 目的 | 建议开关 |
|------|----------|
| 纯 IMU 积分行为 | `use_wheel:=false use_gnss:=false` |
| IMU + 轮速（井下典型） | `use_gnss:=false` |
| 看 GNSS 航向作用 | 其余默认，只开 `use_gnss_heading` |
| 看 ENU 速度作用 | 开 `use_gnss_velocity`，可关 heading |
| 位置锚定（慎用） | `use_gnss_position:=true` |

---

## 6. 参数说明

配置文件：`config/dr_odometry.yaml`。

### 6.1 frames / topics / 单位

| 参数 | 默认 | 含义 |
|------|------|------|
| `frames/odom` | `odom` | 输出父系 |
| `frames/base_link` | `base_link` | 输出子系 |
| `topics/imu` 等 | 见 yaml | 输入输出话题名 |
| `can_speed_is_kmh` | `true` | `true`：speed 为 km/h，适配层 `/3.6` |

### 6.2 运行与观测开关（`dr/`）

| 参数 | 默认（yaml 可能已改） | 含义 |
|------|----------------------|------|
| `output_rate` | 100 | 发布频率 Hz |
| `publish_tf` | true | 是否发 TF |
| `use_imu_accel` | true | 预测是否用加速度积分 |
| `use_wheel` | true | 轮速 + NHC |
| `use_gnss` | true | GNSS 总开关（正式节点是否订阅） |
| `use_gnss_heading` | yaml 当前可能为 false | 航向观测 |
| `use_gnss_velocity` | yaml 当前可能为 false | ENU 速度观测 |
| `use_gnss_position` | false | ENU 位置观测 |

> 调试节点的 `debug/*` 会覆盖上表中对应开关；yaml 与 launch 默认值可能不完全一致，以实际 launch/参数服务器为准。

### 6.3 噪声与门限

| 参数 | 单位（量级） | 进入何处 |
|------|--------------|----------|
| `gyro_noise` | rad/s 标度 | 过程噪声 Q 姿态相关块 |
| `accel_noise` | m/s² 标度 | Q 速度块 |
| `gyro_bias_noise` | 随机游走 | Q 的 d b_g |
| `accel_bias_noise` | 随机游走 | Q 的 d b_a |
| `wheel_velocity_noise` | m/s | 轮速观测 R_00 |
| `nonholonomic_noise` | m/s | 侧/垂向约束 R_11,R_22 |
| `gnss_heading_noise_deg` | deg | 航向观测，内部转 rad |
| `gnss_velocity_noise` | m/s | ENU 速度观测 |
| `gnss_position_noise` | m | ENU 位置观测 |
| `max_dt` | s | IMU 单步积分上限 |
| `wheel_timeout` | s | 轮速相对 IMU 过期 |
| `gnss_timeout` | s | GNSS 相对 IMU 过期 |
| `min_wheel_speed_for_heading` | m/s | 低于此车速不做航向更新 |
| `initial_yaw_deg` | deg | 初始 ENU yaw |

调参直觉：

- 轮速噪声 ↓ → 更信车速，轨迹更“贴地”，急加减速可能拉扯姿态；  
- `use_imu_accel:=false` → 速度几乎完全靠观测修正，适合加速度质量差时；  
- GNSS 位置噪声过小 + 原点固定 → 容易把相对 DR“钉死”在 GNSS 抖动上。

---

## 7. 核心类型（types）

文件：`include/dr_odometry/core/types.hpp`。

### 7.1 误差状态布局（15 维）

| 索引 | 符号 | 含义 | 单位 |
|------|------|------|------|
| 0:3 | dp | 位置误差 | m |
| 3:6 | dv | 速度误差 | m/s |
| 6:9 | dtheta | 姿态小角度误差 | rad |
| 9:12 | db_g | 陀螺零偏误差 | rad/s |
| 12:15 | db_a | 加计零偏误差 | m/s² |

协方差矩阵 `DrState::covariance` 为 15x 15，块索引与上表一致。

### 7.2 主要结构

| 类型 | 作用 |
|------|------|
| `ImuData` | 时间戳、gyro、accel |
| `CanData` | 带符号 `speed_mps`、档位、valid |
| `GnssData` | LLA、ENU 航向/速度、有效位 |
| `DrConfig` | 全部算法与开关参数 |
| `DrState` | 名义状态 + P + `initialized` |
| `DrHealth` | 传感器是否到达、年龄 |
| `OdomResult` | 对外位姿；`covariance` 为 6×6（位置3+姿态3） |

---

## 8. ROS 适配层（ros_adapter）

命名空间：`dr_odometry_ros`。  
文件：`include/.../ros_adapter.hpp`，`src/ros/ros_adapter.cpp`。

### 8.1 参数加载

| 函数 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `loadDrConfig(nh)` | NodeHandle | `DrConfig` | 读 `dr/*` |
| `loadTopicConfig(nh)` | NodeHandle | `TopicConfig` | 读 `topics/*` |
| `loadFrameConfig(nh)` | NodeHandle | `FrameConfig` | 读 `frames/*` |

### 8.2 输入转换 `fromRos`

#### IMU

| 项目 | 内容 |
|------|------|
| 输入 | `sensor_msgs::Imu` |
| 输出 | `ImuData` |
| 映射 | `angular_velocity`→gyro，`linear_acceleration`→accel，stamp→timestamp |

#### CAN

| 项目 | 内容 |
|------|------|
| 输入 | `CanReceiveInfo`，`speed_is_kmh` |
| 输出 | `CanData` |
| 方向 | `round(gear)==2` → 倒车，速度乘 -1 |
| 单位 | km/h 时 `speed/3.6` |
| 时间 | stamp 为 0 则用 `ros::Time::now()` |

#### GNSS（Gpchc）

| 项目 | 内容 |
|------|------|
| 输入 | `deeplumin_msgs::Gpchc` |
| 输出 | `GnssData` |
| heading | `headingDegToYawRad` → ENU yaw |
| 速度 | `(ve,vn,vu)` → `velocity_enu` |
| 有效性 | `status>0` 且有限；位置另要求 lat/lon 非接近 0 |

### 8.3 输出转换

| 函数 | 输入 | 输出 |
|------|------|------|
| `toRosOdom` | `OdomResult` + 父子 frame | `nav_msgs/Odometry`（pose 协方差 6×6 展平） |
| `toRosTransform` | 同上 | `TransformStamped` |
| `toStatusMsg` | `DrHealth` + stamp | `LocalizationStatus`：无 IMU → FAILURE；有 IMU → NORMAL，`current_mode=DR_ODOMETRY` |

---

## 9. 数学工具（math_utils）

文件：`include/dr_odometry/core/math_utils.hpp`。

| 函数 | 输入 | 输出 | 用途 |
|------|------|------|------|
| `skew(v)` | R^3 | [v]_x | 误差雅可比 |
| `deltaQuat(ω, dt)` | 角速度、时间 | 增量四元数 | 预测姿态、注入 dtheta |
| `normalizeAngle` | rad | (-pi,pi] | 航向残差 |
| `yawFromQuat` | 四元数 | yaw | GNSS 航向残差 |
| `quatFromYaw` | yaw | 四元数 | 初值 / GNSS 预置 |

---

## 10. ESKF 核心（DrEskf）：函数级说明

类：`dr_odometry::DrEskf`  
头文件 / 实现：`dr_eskf.hpp` / `dr_eskf.cpp`。

### 10.1 公有接口

| 函数 | 输入 | 输出/副作用 | 说明 |
|------|------|-------------|------|
| `initialize(config)` | `DrConfig` | 重置状态与 P 块 | 未设 `initialized`；等首帧 IMU |
| `feedImu(imu)` | `ImuData` | 预测 + 可能更新 | **主路径** |
| `feedCan(can)` | `CanData` | 写 `last_can_` | 不立刻更新 |
| `feedGnss(gnss)` | `GnssData` | 写缓存；可定原点、预置 yaw | 更新在随后 IMU 里 |
| `reset(state)` | `DrState` | 强制名义状态 | 清 `has_last_imu_` |
| `state()` | — | `const DrState&` | 调试用 |
| `health(now)` | 当前时间 s | `DrHealth` | 传感器在位与年龄 |
| `odometry()` | — | `OdomResult` | `valid = initialized` |

### 10.2 `feedImu` 逐步行为

```text
无效数据 → return
未 initialized → initializeFromImu → return
无 last_imu → 仅缓存 → return
dt = t - t_last ≤ 0 → 更新 last，跳过
dt = min(dt, max_dt)
中值 IMU → predict(mid, dt)
last_imu = imu
若 use_wheel → correctWheel(t)
若 use_gnss 且新鲜 → correctGnss(last_gnss)
```

### 10.3 私有函数摘要

| 函数 | 作用 |
|------|------|
| `initializeFromImu` | 置 `initialized`，对齐时间 |
| `predict` | 名义积分 + Parrow FPF^T+Q |
| `correctWheel` | body 速度观测 + NHC |
| `correctGnss` | heading / ENU vel / ENU pos（按开关） |
| `update` | K、注入误差、Joseph P |
| `injectError` | d x 注入名义量 |
| `wheelFresh` / `gnssFresh` | ||t-t_obs||<= timeout，且允许 −0.05 s 时钟误差 |
| `llaToEcef` / `llaToEnu` | WGS84 位置转换 |

### 10.4 成员缓存含义

| 成员 | 含义 |
|------|------|
| `last_imu_` / `last_can_` / `last_gnss_` | 最近一帧 |
| `origin_set_` + lat/lon/ecef | 首次有效 GNSS 位置锁定的 ENU 原点 |
| `has_*` | 是否收到过对应数据 |

---

## 11. 详细数学原理

> 本节公式一律用纯文本代码块书写，不依赖 LaTeX 渲染，避免预览器显示成“乱码”。

### 11.1 名义状态与 IMU 运动学

名义状态：

```text
x = { p, v, q, b_g, b_a }
```

去偏测量：

```text
omega = omega_m - b_g
a_b   = a_m - b_a
```

世界系加速度（`use_imu_accel`）：

```text
a_w = R(q) * a_b + g ,   g = (0, 0, -g)
```

若关闭加速度：`a_w = 0`（速度靠观测拉回）。

中值积分（离散）：

```text
p <- p + v * dt + 0.5 * a_w * dt^2
v <- v + a_w * dt
q <- q ⊗ delta_q(omega * dt)
```

零偏在预测步中保持不变（随机游走噪声进入过程噪声 Q）。

### 11.2 误差状态与 F 矩阵

误差状态（15 维）：

```text
dx = [ dp(3), dv(3), dtheta(3), db_g(3), db_a(3) ]^T
```

代码采用近似离散转移 `F ≈ I + F_c * dt`，非零块包括：

```text
F[p, v]         = I * dt
F[v, theta]     = -R * [a_b]_x * dt     (仅 use_imu_accel)
F[v, b_a]       = -R * dt               (仅 use_imu_accel)
F[theta, b_g]   = -I * dt
```

协方差传播：

```text
P <- F * P * F^T + Q
```

Q 为对角近似：速度/姿态块约正比于 `(sigma * dt)^2`，零偏块约正比于 `sigma_b^2 * dt`。

### 11.3 标准更新与误差注入

对残差 `r`、观测阵 `H`、噪声 `R`：

```text
S  = H * P * H^T + R
K  = P * H^T * inv(S)
dx = K * r
```

注入（姿态右乘小角度）：

```text
p   <- p + dp
v   <- v + dv
q   <- q ⊗ delta_q(dtheta)
b_g <- b_g + db_g
b_a <- b_a + db_a
```

协方差用 Joseph 形式：

```text
P <- (I - K*H) * P * (I - K*H)^T + K * R * K^T
```

> 经典 ESKF 常在注入后把 dx 清零；本实现每次更新后误差已并入名义状态，等价于“用完即清”。

### 11.4 轮速 + 非完整约束（NHC）

车体速度预测：

```text
v_body_hat = R^T * v
```

观测：

```text
z = [ v_wheel,  0,  0 ]^T
```

残差：

```text
r = z - v_body_hat
```

物理含义：

- 纵向：轮速约束前进速度；
- 侧向/垂向：理想刚体无侧滑/跳起时速度约等于 0（NHC）。

观测噪声对角：

```text
R = diag( sigma_wheel^2,  sigma_nhc^2,  sigma_nhc^2 )
```

代码中 H 先按 `v_body` 对 `(v, theta)` 的雅可比构造，再整体取负，与残差 `z - v_body_hat` 的线性化约定对齐。

### 11.5 GNSS 航向

残差（一维）：

```text
r = normalize( psi_gnss - yaw(q) )
```

H 仅在 `dtheta_z`（误差状态索引 8）置 1，把残差当作对 yaw 误差的直接观测。

门限：`|v_wheel| >= min_wheel_speed_for_heading`，避免低速/静止时航向噪声污染。

### 11.6 GNSS ENU 速度

```text
r = v_ENU_gnss - v
H = [ 0(3x3) , I(3x3) , 0 ]
```

即直接观测世界系速度。与轮速的区别：轮速在 **body**，ENU 速度在 **world**。

### 11.7 GNSS ENU 位置

1. 首次 `position_valid`：锁定原点 `LLA_0`，计算 `p_ecef_0`。
2. 之后：

```text
p_enu = LLA_to_ENU(LLA ; LLA_0)
r     = p_enu - p
```

H 对 `dp` 为单位阵。默认关闭，因绝对位置抖动会拉漂相对里程计风格的 odom。

### 11.8 LLA → ECEF → ENU

WGS84：

```text
N = a / sqrt(1 - e^2 * sin(phi)^2)

X = (N + h) * cos(phi) * cos(lambda)
Y = (N + h) * cos(phi) * sin(lambda)
Z = (N*(1 - e^2) + h) * sin(phi)
```

相对原点的 ECEF 差经标准旋转得到 `(E, N, U)`（见 `llaToEnu`）。

### 11.9 输出协方差

`OdomResult::covariance`（6×6）：

- 左上 3×3：来自 `P_pp`
- 右下 3×3：来自 `P_theta_theta`（误差状态姿态块）
- 交叉项未填速度块；`twist` 发布世界系线速度，未填角速度协方差。

---

## 12. 与 localization_ws2 DR 的差异

| 项目 | DeepLumin `dr_odometry` | localization_ws2 |
|------|-------------------------|------------------|
| 架构 | 节点 → `DrEskf` | `IDrOdometry` → Estimator → `DrEskfCore` |
| GNSS | 已实现（可开关） | debug 主路径仅 IMU+轮速 |
| 轮速时间对齐 | 最新帧 + timeout | 缓冲区插值到 IMU 时刻 |
| 轮速尺度 | 无 | 在线低通估计 |
| ZUPT | 无 | 有静止零速更新 |
| 初始化 | 首帧 IMU；可选 GNSS yaw | 等轮速可插值 + 重力对齐 |
| 过程噪声建模 | Q 对角直接堆到 15 维 | GQG^T（15×12）更标准 |

DeepLumin 版更适合“带 GNSS 消融实验、工程接口清晰”；ws2 版相对 DR 的时间同步与静止处理更细。

---

## 13. 调试与常见问题

### 13.1 推荐联调命令

```bash
# 终端1：调试节点
roslaunch dr_odometry dr_odometry_debug.launch

# 终端2：播放 bag（建议 --clock，并设 use_sim_time）
rosbag play xxx.bag --clock

# 终端3
rostopic echo /localization/dr_odom
rostopic hz /ouster/imu /can_receive_info /gnss_chc_data
```

RViz：Fixed Frame = `odom`，加 Path `/localization/dr_path`，Odometry `/localization/dr_odom`。

### 13.2 FAQ

**Q: `rostopic echo /gnss_chc_data` 报 `Cannot load message class for [msgs/gpchc]`，但节点在跑？**  
A: bag 类型名是 `msgs/gpchc`；echo 要加载 Python 类。节点用 `deeplumin_msgs/Gpchc`，MD5 一致即可收。详见此前说明。

**Q: 有 odom 输出但轨迹乱转？**  
A: 查 `initial_yaw_deg`、是否误开低质量 GNSS heading、轮速单位 `can_speed_is_kmh`、倒档 `gear` 是否正确。

**Q: 关掉 GNSS 后轨迹仍然可用？**  
A: 正常。主约束是 IMU + 轮速/NHC。

**Q: `use_gnss_position` 何时开？**  
A: 仅当需要把 odom 钉到 GNSS ENU、且接受 GNSS 抖动对轨迹的影响时；默认可关。

---

## 14. 源码阅读路径建议

按执行顺序阅读最高效：

1. `launch/dr_odometry_debug.launch` — 入口与开关  
2. `config/dr_odometry.yaml` — 参数语义  
3. `src/dr_odometry_debug_node.cpp` — 订阅发布与主循环  
4. `src/ros/ros_adapter.cpp` — 单位、航向、有效性  
5. `include/dr_odometry/core/types.hpp` — 状态布局  
6. `src/core/dr_eskf.cpp`  
   - 先 `feedImu` / `predict`  
   - 再 `correctWheel` / `correctGnss`  
   - 最后 `update` / `injectError`  
7. 对照本文 §11 公式与代码块注释  

正式节点在看完 debug 节点后，扫一眼 `dr_odometry_node.cpp` 差异即可（无 `debug/*` 覆盖）。

---

## 附录 A：一次 IMU 周期内的时序示意

```text
t = t_imu
  │
  ├─ mid = 0.5*(imu_prev + imu)
  ├─ predict:  p,v,q,P
  │
  ├─ [optional] correctWheel
  │     z=[v_can,0,0], r=z-Rᵀv → update
  │
  └─ [optional] correctGnss
        ├─ heading（车速门限）
        ├─ ENU velocity
        └─ ENU position（需原点）
```

## 附录 B：关键文件索引

| 文件 | 一句话 |
|------|--------|
| `dr_odometry_debug.launch` | 调试启动与消融开关 |
| `dr_odometry.launch` | 正式启动 |
| `dr_odometry.yaml` | 全参数 |
| `dr_odometry_debug_node.cpp` | 调试节点 |
| `dr_odometry_node.cpp` | 正式节点 |
| `ros_adapter.*` | ROS 边界 |
| `types.hpp` | 类型与维序 |
| `math_utils.hpp` | 小工具 |
| `dr_eskf.*` | 全部滤波数学 |

---

*文档根据当前仓库源码整理；若后续改动观测模型或默认参数，请以源码与 yaml 为准同步更新本节。*
