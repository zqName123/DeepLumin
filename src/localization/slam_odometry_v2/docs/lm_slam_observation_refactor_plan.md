# 轮速/GNSS 观测重构方案（对标 lm_slam）

> 状态：设计文档，未实施。
> 基准：lm_slam 解除以下两行注释后的运行效果（建图质量明显优于当前项目）：
> ```cpp
> kf_.Update(ESKF::ObsType::LIDAR, 1e-3);
> kf_.Update(ESKF::ObsType::WHEEL_SPEED, 0.33*0.33);   // zzq
> kf_.Update(ESKF::ObsType::GPS, 1e-3);
> ```

---

## 1. 结论摘要

lm_slam 效果好的核心不是某个精巧机制，而是**结构极简**：

```
LiDAR 迭代更新（R=1e-3/点）
  → 轮速 3D 伪观测（σ=0.33 m/s，松）
  → GNSS 3D 相对姿态（σ≈1.8°，紧，首帧基准）
```

三个观测**串行、互相独立、固定噪声、无门控级联、无自适应、无膨胀**，永不打架。

当前 slam_odometry_v2 是"4 类观测 × 3 套自适应 R × 5 处门控/阈值 × P 膨胀"的网状结构，机制之间互相干扰（前向 3D 与 NHC 重复计入、VZ 与 NHC-z 重复、GNSS roll/pitch 与 IMU 重力重复、AdaptR 与膨胀正反馈对抗）。

**方案：一期把观测栈删减到 lm_slam 同构（1 常开 + 1 开关），DCReg 膨胀作为二期增强单独回归。**

---

## 2. lm_slam 机制剖析（解除注释后）

### 2.1 调用序列

[laser_mapping.cc:918-920](file:///home/hhy/ros2/lm_slam/src/core/lio/laser_mapping.cc#L918-L920)，每帧 LiDAR 处理中顺序执行：

| 顺序 | 观测 | R（方差） | σ |
|---|---|---|---|
| 1 | LIDAR（点面残差，迭代 EKF） | 1e-3 / 点 | — |
| 2 | WHEEL_SPEED | 0.33² = 0.1089 | 0.33 m/s |
| 3 | GPS（3D 姿态） | 1e-3 | ≈0.0316 rad ≈ 1.8° |

每个 Update 都是独立的完整迭代 EKF 更新（内部有先验锚定 `dx boxminus start_x`、每轮 `P = P_propagated` 重置）。

### 2.2 轮速观测（ObsModelWelspeed）

- **一个 3D 世界系速度伪观测**：`z = R·[v_wheel, 0, 0]`，残差 `= z − vel`
- y/z 分量即 NHC（横/垂向速度=0），**不单独拆分、不单独调噪声**，统一 σ=0.33
- σ=0.33 非常松（约为当前项目 0.032 的 10 倍）：轮速只兜底、不主导，标定误差/颠簸/轻微打滑全部被噪声吸收
- 每帧用**最新缓存的 CAN 值**（`wheel_speed_obs` 成员变量），无超时/健康检查/门控/自适应
- H 只写速度块（+I），**旋转雅可比忽略**（旋转误差吸收进大噪声）

### 2.3 GNSS 姿态观测（ObsModelGPS）

- **3D 全姿态**（heading + pitch + roll 都用，不零化 yaw）——Gpchc 是双天线输出，heading 可靠
- **首帧基准校准**（gnss_callback 内，[L233-253](file:///home/hhy/ros2/lm_slam/src/core/lio/laser_mapping.cc#L233-L253)）：
  ```cpp
  if (first_time) { hc_heading_b = -heading; hc_pitch_b = -pitch; hc_roll_b = roll; }
  else { hc_heading = -heading_now - hc_heading_b; ... }   // 相对变化量
  ```
  观测的是**姿态相对首帧的变化**而非绝对值 → 安装角偏置、NED/ENU 坐标系差异、单双天线绝对误差全部被基准吸收
- pitch 经一阶低通滤波（`filter()`），heading/roll 未滤波
- 残差 `= log(R_est^T · R_gnss_rel)`，H 只写旋转块
- 无有效性检查（nsv/status 检查在回调里全被注释）、无新鲜度判断，每帧用缓存值

### 2.4 lm_slam 没有的东西（对照当前项目）

退化检测、P 膨胀、Huber、退化降权、自适应 R、VZ 地面约束、IMU 重力观测、马氏门控、健康检查、chi2 放宽——**全部没有**。

---

## 3. 当前项目（slam_odometry_v2）差距对照

| 机制 | lm_slam（效果好） | slam_odometry_v2 现状 | 问题 |
|---|---|---|---|
| 轮速观测 | 1 个 3D 伪观测，σ=0.33 | 前向 3D（σ²=0.001→修复后 0.01）+ NHC 2D（σ²=0.0001）+ VZ 1D | NHC σ=1cm/s 过严，颠簸必打架；前向 3D 的 y/z 与 NHC 重复计入；VZ 坡道错误模型 |
| GNSS 姿态 | 3D 相对姿态（首帧基准），σ=1.8°，含 yaw | 绝对 roll/pitch（yaw 零化），σ=10°，两套噪声键 | 单天线假设与实际硬件（双天线 Gpchc）不符；绝对姿态带安装偏置；yaw 白白丢弃 |
| 自适应 R | 无 | 3 套（wheel/gnss/imu_grav），IAE 公式缺 `−HPH^T` 项 | 与膨胀正反馈对抗；退化时 R 涨、削轮速话语权 |
| 门控 | 无 | 马氏门控 + 退化时 chi2 ×5 | P 膨胀已放宽马氏距离，×5 是双重放宽 |
| 观测频率 | 每帧复用最新缓存值 | 每帧复用（GNSS 同帧重复计入） | 同上（见 §8 风险，一期改为"新消息才更新"） |
| 退化处理 | 无 | DCReg 检测 + Huber + 点级降权 + P 膨胀 | 膨胀参数激进（vel σ=1.0）；一期关闭，二期单独回归 |

已完成的前置修复（本次会话，编译通过）：
- IMU 重力观测 H 符号（原两块同时差负号）
- `SetObservationParams` 重置自适应 R 和 floor（原 yaml 噪声不生效）
- 膨胀改用 DCReg 对齐基底（`aligned_rot/trans_basis`），沿退化轴 raise-to-floor

---

## 4. 改造方案

### 4.1 删除

| 项 | 涉及代码 | 说明 |
|---|---|---|
| VZ=0 地面约束 | `ApplyWheelSpeedObservation` 内观测 3 块（L673-696）；`use_ground_vz_constraint_`/`ground_vz_noise_`；yaml `ground_constraint` 节 | NHC-z 已覆盖 body 系垂向；世界系 z 在坡道是错误模型 |
| NHC 2D 独立观测 | `ApplyWheelSpeedObservation` 内 H2/residual_nhc 块 | 并入 3D 伪观测（y/z=0 分量由 σ=0.33 覆盖） |
| 前向+NHC 5D 合并 | H_full/residual_full 5D 组装，chi2 阈值改 3DOF | 5D→3D |
| 自适应 R 三套 | `AdaptR` 调用（wheel/gnss/imu_grav 三处）；`use_adaptive_*_`、`adaptive_*_alpha_`、`R_*_floor_`、inno_window；yaml `adaptive_observation_noise` 节 | 固定 R。AdaptR 模板代码可留（不再被调用）或一并删除 |
| chi2 退化 ×5 放宽 | `ApplyWheelSpeedObservation` 内 L627-630 | 双重放宽 |
| GNSS 绝对 roll/pitch 观测 | `ApplyGnssAttitudeObservation` 整体重写（见 4.2） | 被相对姿态方案替代 |
| `ApplyVelocityGnssAid` 死代码 | 函数体 + `can_vel_diff_thresh_kmh_`/`attitude_diff_thresh_deg_`/`attitude_correction_gain_` + `SetVelocityGnssParams` 对应参数 + yaml 键 | 硬重置遗留，已被软观测替代 |
| `isGnssAttitudeValid` 复杂门控 | min_gnss_status=33（RTK 固定）对姿态观测过严 | 用最小门控替代（见 4.2） |
| IMU 重力观测调用 | `use_imu_gravity_` 默认 false | 代码保留作兜底开关（H 符号已修好） |

### 4.2 修改

#### A. `ApplyWheelSpeedObservation` → 单 3D 伪观测（lm_slam 同构）

保留现有 H1（符号已验证正确，比 lm_slam 多保留旋转耦合，零成本更准）：

```cpp
// h(x) = R·[v_w,0,0] − vel 的伪观测（z=0 形式）
residual = state.vel − R_wl * can_body_vel;          // 现有 residual_vel
H.block<3,3>(0, 3)  = −R_wl * hat(can_body_vel);     // 现有，正确
H.block<3,3>(0, 12) = −I;                            // 现有，正确

R = wheel_velocity_noise_ * I;    // 固定，0.1089（σ=0.33 m/s）

// 基础马氏门控（3DOF，无退化放宽）——lm_slam 没有，作为 CAN 野值保险保留
MahalanobisOutlierRejection<3>(residual, H, P, R, CHI2_THRESHOLD_3DOF);

// CAN 新鲜度：last_can_ 时间戳距今 > can_timeout_sec_(0.5s) 则跳过
// （lm_slam 无此检查；CAN 断流后陈旧速度持续注入有真实风险，保留这一行）

kf_.update_single_observation(H, residual, R);
```

删除：H2/NHC 块、5D 组装、VZ 块、AdaptR 调用、chi2 ×5。

#### B. `ApplyGnssAttitudeObservation` → lm_slam 式相对姿态（重写）

核心思想：**约束"姿态相对首帧的变化"与 GNSS 一致**，常数偏置（安装角/坐标系/绝对误差）被基准吸收。

```cpp
// 首次使用时捕获基准（需 LIO 已收敛，见 §8 风险 R1）
if (!gnss_baseline_captured_) {
    R_gnss_first_ = EulerZYXToRot(last_gnss_attitude_);   // 当前 GNSS 姿态
    R_est_first_  = kf_.get_x().rot.toRotationMatrix();   // 当前 LIO 姿态
    gnss_baseline_captured_ = true;
    return;                                                // 首帧不更新
}

// 目标姿态 = GNSS 变化量作用在 LIO 初始姿态上
R_target = R_gnss_now * R_gnss_first_.inverse() * R_est_first_;

// 残差与 H（与现有 GNSS 观测同形式，但 3D 全维，不零化 yaw）
residual = log(R_est^T * R_target);        // 3 维
H.block<3,3>(0, 3) = I;                    // 旋转块（小角度近似，现有代码已验证该形式）
R = gnss_attitude_noise_ * I;              // 1e-3（σ≈1.8°）

// 最小门控：新消息才更新（gnss_msg_seq != last_used_seq）+ 宽松残差上限（如 0.3 rad）
// （lm_slam 每帧复用缓存值；差异见 §8 风险 R2）

kf_.update_single_observation(H, residual, R);
```

相比 lm_slam 逐分量减欧拉角的做法，`R_gnss·R_gnss_f⁻¹·R_est_f` 是严格的 SO3 相对旋转，且自动兼容 NED/ENU 与安装角（lm_slam 的 −heading/−pitch 符号翻转即手工处理此问题，我们不需要）。

可选：对 `R_gnss_now` 做一阶低通（lm_slam 只滤了 pitch）。一期不加，若姿态抖动明显再开（参数 `gnss_att_lpf_alpha`，建议 0.3）。

#### C. 调用顺序与开关

`ProcessLaserOdom` 内保持：LiDAR 迭代更新 → [二期：膨胀] → **轮速 → GNSS**（对齐 lm_slam 顺序；现 GNSS 在前，影响极小，顺手对齐）。

### 4.3 保留

| 项 | 一期状态 | 说明 |
|---|---|---|
| Huber 点级鲁棒 | **开**（`use_huber_weight: true`） | 与观测栈无交互，纯增益 |
| DCReg 检测 + 膨胀 + 降权 | **关**（`use_dcreg_degeneracy: false`） | 二期单独回归；膨胀已修好（对齐基底版），参数待调：`vel 0.4 / pos 0.3 / rot_max 15°` |
| IMU 重力观测 | 关（默认） | 长隧道 GNSS+LiDAR 双失效时再评估 |
| 马氏门控（轮速 3DOF / GNSS 3DOF 基础版） | 开 | lm_slam 没有，但成本为零、只拦真野值 |

---

## 5. 目标 yaml

```yaml
vel_gnss:
  use_vel: true
  use_gnss_attitude: true          # 一期开启（双天线 heading 可靠）
  can_timeout_sec: 0.5             # 轮速新鲜度（唯一保留的检查）
  wheel_velocity_noise: 0.1089     # σ=0.33 m/s，对齐 lm_slam（3D 伪观测统一噪声）
  gnss_attitude_noise: 0.001       # σ≈1.8°，对齐 lm_slam
  use_imu_gravity: false           # 兜底开关
  imu_gravity_noise: 0.05
  # ↑ 删除键：non_holonomic_noise / gnss_attitude_noise_rad2 /
  #          gnss_roll_noise_rad2 / gnss_pitch_noise_rad2 /
  #          can_vel_diff_thresh_kmh / attitude_diff_thresh_deg /
  #          attitude_correction_gain / gnss_attitude_timeout_sec /
  #          min_gnss_status / min_gnss_nsv / min_lidar_points

dcreg:
  use_dcreg_degeneracy: false      # 一期关，二期开
  use_huber_weight: true
  use_degeneracy_weight: false     # 随总开关
  dcreg_condition_threshold: 10.0
  huber_k: 0.5
  # 二期启用时：rot_inflation_base_deg: 2.0 / rot_inflation_max_deg: 15.0
  #            pos_inflation_base_m: 0.3 / vel_inflation_base_mps: 0.4

# 整节删除：adaptive_observation_noise、ground_constraint
```

同步修改：`LoadParamsFromYAML`（删对应读取）、`slam_odometry_v2_config.cpp`（删对应 `pnh.param`）、`SetObservationParams` 签名精简。

---

## 6. 代码增删汇总（按文件）

| 文件 | 增 | 删/改 |
|---|---|---|
| `laser_odometry.cc` | GNSS 基准捕获成员初始化；GNSS 相对姿态残差 | 重写 `ApplyGnssAttitudeObservation`；简化 `ApplyWheelSpeedObservation`（5D→3D，删 VZ/AdaptR/×5）；删 `ApplyVelocityGnssAid`、`isGnssAttitudeValid`；`LoadParamsFromYAML` 删死键 |
| `laser_odometry.h` | `R_gnss_first_/R_est_first_/gnss_baseline_captured_/gnss_msg_seq_` 等成员 | 删 `use_ground_vz_constraint_/ground_vz_noise_/can_vel_diff_thresh_kmh_/attitude_*_` 及自适应相关成员（或保留不引用） |
| `slam_odometry_v2_node.cc` | — | `SetVelocityGnssParams` 调用精简 |
| `slam_odometry_v2_config.cpp` | — | 删死参数读取 |
| `config/slam_odometry_v2.yaml` | — | 按 §5 |

---

## 7. 分阶段验证

**Phase 1（lm_slam 同构）**
1. 按本方案删减 + 重写，编译
2. 同一 bag 回放，与 lm_slam（解除注释版）对比：轨迹一致性、建图质量、隧道段表现
3. 预期：行为应与 lm_slam 高度一致（观测结构/噪声/顺序全对齐）；差异来源仅剩框架差异（update_single_observation 单次 vs 迭代、Huber 常开）

**Phase 2（DCReg 增强，仅当一期隧道段仍有漂移）**
1. `use_dcreg_degeneracy: true` + §5 二期参数
2. 只加这一个变量，回放对比一期，确认膨胀与轮速/GNSS 不再对抗（已无 AdaptR）
3. 若膨胀引入抖动，优先降 `vel_inflation_base_mps`（0.4→0.25）

**观测指标**：全图闭合差、隧道进出口位姿跳变、z 向漂移、轮速残差时间序列（应平稳、无门控频繁触发）。

---

## 8. 风险与注意

| # | 风险 | 缓解 |
|---|---|---|
| R1 | **首帧基准被污染**：LIO 未收敛时收到 GNSS，基准偏差固化，此后 GNSS 观测持续带偏 | 延迟捕获（如要求已完成 ≥N 帧 LiDAR 更新）；或检测到残差长期偏置时重新捕获 |
| R2 | **GNSS 更新频率**：lm_slam 每帧复用缓存值（10Hz 复用 1-5Hz 数据 → 等效放大权重、协方差过自信）；本方案默认"新消息才更新"。若一期效果不及 lm_slam，把开关切回"每帧复用"再验证 | `gnss_update_every_frame` 开关，默认 false |
| R3 | **双天线假设**：方案依赖 Gpchc heading 可靠。若实车是单天线，yaw 维度噪声应放大（R_yaw 单独给 0.05~0.1 rad²）或退回零化 yaw | `gnss_yaw_noise` 独立键，默认 1e-3 |
| R4 | **CAN 断流**：陈旧速度以 σ=0.33 持续注入，长时间断流时车辆加减速会被拖住 | `can_timeout_sec: 0.5` 新鲜度检查（唯一保留的检查） |
| R5 | **GNSS 长时间失效后重捕**（出隧道）：LIO 已漂移，相对姿态残差瞬时增大，可能触发门控跳过 | 门控阈值放宽到 0.3 rad；被拒绝的更新不影响后续（残差随 LIO 被 LiDAR 修正而回落） |

---

## 附录 A：lm_slam 两处实现瑕疵（勿照抄）

1. **h_x 放置维度不匹配**（[eskf.cc:272](file:///home/hhy/ros2/lm_slam/src/core/lio/eskf.cc#L272)）：`h_x_cur.topLeftCorner(dof,12) = h_x_`，但轮速/GPS 的 `h_x_` 是 3×3，赋给 3×12 区域尺寸不符（Release 下越界读，H 实际落到 pos 列而非 vel/rot 列，靠 P 的互协方差间接起作用）。**本方案用 slam_odometry_v2 现有的正确 H（vel/rot 块）实现同一意图**。
2. **R 覆盖**（[eskf.cc:273](file:///home/hhy/ros2/lm_slam/src/core/lio/eskf.cc#L273)）：`Update` 内 `R_ = R·I` 覆盖观测函数里设置的 `obs.R_`（0.33/0.03 是死代码）。有效噪声即调用参数：轮速 0.33²、GNSS 1e-3，本方案直接采用。

## 附录 B：slam_odometry_v2 现有 H 矩阵正确性（本次会话已验证）

- 轮速前向 3D（H1）：`H_rot = −R·hat(v_b)`，`H_vel = −I`，与 `update_single_observation` 的 `residual = z−h, H = ∂h/∂x, dx = K·residual` 及右乘 boxplus 约定一致 ✅
- NHC（H2）：符号正确 ✅（一期删除，被 3D 伪观测吸收）
- GNSS 姿态：`residual = log(R_est^T·R_gnss)`，`H_rot = +I` ✅（重写沿用此形式，扩为 3D）
- IMU 重力：原实现两块同时差负号（已修复：`H_rot = −hat(R^T·g)`，`H_ba = +I`）✅
