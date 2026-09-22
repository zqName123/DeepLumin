# 会话总结：slam_odometry_v2 多传感器融合EKF修复

> 导出时间: 2026-08-28
> 项目路径: `/home/hhy/2004ros/DeepLumin/src/localization/slam_odometry_v2`

---

## 一、项目概述

`slam_odometry_v2` 是基于 Faster-LIO 的纯里程计核心，实现 LiDAR + IMU + 轮速 + GNSS + IMU重力观测的多传感器融合，使用迭代扩展卡尔曼滤波（IEKF）进行状态估计。

### 状态向量（24维，实际23维自由度）

| 索引 | 名称 | 维度 | 描述 |
|------|------|------|------|
| 0-2 | pos | 3 | 位置 (世界坐标系) |
| 3-5 | rot | 3 | 旋转 (SO3流形) |
| 6-8 | offset_R_L_I | 3 | LiDAR-IMU外参旋转 |
| 9-11 | offset_T_L_I | 3 | LiDAR-IMU外参平移 |
| 12-14 | vel | 3 | 速度 (世界坐标系) |
| 15-17 | bg | 3 | 陀螺仪偏置 |
| 18-20 | ba | 3 | 加速度计偏置 |
| 21-22 | grav | 2 | 重力向量 (S2流形，内部3D但自由度2) |

### 核心文件

- **算法实现**: `src/faster_lio/laser_odometry.cc`
- **头文件**: `include/faster_lio/laser_odometry.h`
- **状态定义**: `include/faster_lio/use-ikfom.hpp`
- **ESKF核心**: `include/faster_lio/IKFoM_toolkit/esekfom/esekfom.hpp`
- **配置文件**: `config/slam_odometry_v2.yaml`

---

## 二、已发现并修复的问题

### 问题1: IMU重力观测残差符号错误

**现象**: 残差约为2倍重力加速度（19.17 m/s²），导致滤波器发散。

**根因**: 残差计算公式错误。
```cpp
// 错误（旧）:
residual = (last_acc_ - ba) - R.transpose() * grav_vec;
// 正确（新）:
residual = (last_acc_ - ba) + R.transpose() * grav_vec;
```

**修复**: 同步修正残差和雅可比矩阵符号。

---

### 问题2: IMU重力观测更新 grav 状态导致Z轴漂移

**现象**: 启用IMU重力观测后出现严重的Z轴位置漂移，关闭GNSS也无法消除。

**根因**: H矩阵包含 `grav` 状态（21-22维度）的雅可比，导致 `grav` 被频繁更新。`grav` 通过IMU传播方程 `vel_dot = R*(acc-ba) + grav` 影响速度积分，进而影响Z轴位置。

**修复**: 移除H矩阵中 `grav` 相关雅可比项，仅保留对 `rot`（3-5）和 `ba`（18-20）的更新。

**关键代码**:
```cpp
// 只更新 rot (3-5) 和 ba (18-20)，不更新 grav (21-22)
H.block<3, 3>(0, 3) = SKEW_SYM_MATRIX(Rt_grav);   // ∂r/∂rot
H.block<3, 3>(0, 18) = -Eigen::Matrix3d::Identity(); // ∂r/∂ba
// ⚠️ 不更新 grav (21-22) 状态！
```

---

### 问题3: GNSS姿态观测H矩阵设计缺陷导致yaw协方差虚假压缩

**现象**: GNSS观测后yaw方向协方差被人为降低，但实际yaw并不可观测。

**根因**: H矩阵第三行（yaw方向）为单位阵，但残差yaw分量被置零，导致EKF进行"零残差更新"，人为降低yaw协方差。

**修复**: 零化H矩阵第三行（yaw方向雅可比），避免虚假协方差压缩。

```cpp
H.block<2, 3>(0, 3) = Eigen::Matrix3d::Identity().topRows<2>();  // 只更新roll/pitch
H.row(2).setZero();  // 零化yaw方向
```

---

### 问题4: 轮速观测中非完整性约束（NHC）雅可比符号错误

**现象**: 横向/垂向速度残差从1.11 m/s正反馈膨胀至99 m/s，导致滤波器崩溃。

**根因**: NHC的H矩阵中 `∂h/∂rot` 符号错误，使用了 `-R^T * hat(v_world)` 而非正确的 `hat(R^T * vel) = hat(v_body)`。

**正确推导**:
```
h(x) = R^T[1:2, :] * vel (车体系横向/垂向速度)
R_new = R * Exp(hat(δθ))
R_new^T ≈ (I - hat(δθ)) * R^T
h_new = R_new^T * vel = R^T * vel - hat(δθ) * R^T * vel
      = R^T * vel + hat(R^T * vel) * δθ
因此: ∂h/∂rot = hat(R^T * vel) = hat(v_body)  (正号!)
```

**修复**:
```cpp
H2.block<2, 3>(0, 12) = R_wl.transpose().block<2, 3>(1, 0);  // ∂h/∂vel = +R^T[1:2,:]
H2.block<2, 3>(0, 3) = SKEW_SYM_MATRIX(v_body_actual).block<2, 3>(1, 0);  // ∂h/∂rot = hat(v_body)
```

---

### 问题5: 轮速观测中速度约束缺少旋转雅可比

**现象**: 仅使用 `∂h/∂vel = -I`，忽略旋转对速度观测的影响，轮速观测精度受限。

**修复**: 补充旋转雅可比项。
```cpp
H1.block<3, 3>(0, 3) = -R_wl * SKEW_SYM_MATRIX(can_body_vel);  // ∂h/∂rot = -R * hat(v_body)
H1.block<3, 3>(0, 12) = -Eigen::Matrix3d::Identity();           // ∂h/∂vel = -I
```

---

### 问题6: 退化检测后滤波崩溃

**现象**: 检测到退化日志后，P矩阵膨胀，但下一帧所有点云匹配失败（"No Effective Points"）。

**根因**: 三个因素叠加：
1. NHC雅可比符号错误导致速度残差从1.11 m/s膨胀至99 m/s（正反馈）
2. 退化检测仅膨胀旋转协方差（P_rot）而未膨胀速度协方差（P_vel），滤波器对错误速度过于自信
3. IMU重力观测中ba雅可比使用 `-0.1*I` 导致偏置校正不足，IMU速度积分长期漂移

**修复**:
1. 在 `InflateDegenerateP` 中新增速度协方差膨胀逻辑（从0.00026膨胀至1.0 m²/s²）
2. 在 `ApplyWheelSpeedObservation` 中新增残差门限检查（速度>2m/s或NHC>1m/s时跳过更新）
3. 修正ba雅可比从 `-0.1*I` 改为 `-I`

**关键代码**:
```cpp
// 速度协方差膨胀
constexpr double VEL_TARGET_SIGMA = 1.0;  // 1.0 m/s
constexpr double VEL_TARGET_VAR = VEL_TARGET_SIGMA * VEL_TARGET_SIGMA;  // 1.0
Eigen::Matrix3d P_vel = P.block<3,3>(12,12);
// 对每个特征值：如果小于目标方差则膨胀到目标值
P.block<3,3>(12,12) = V_vel * lambda_vel.asDiagonal() * V_vel.transpose();
```

```cpp
// 残差门限检查
const double VEL_RESIDUAL_THRESHOLD = 2.0;   // 正常轮速残差应 < 2 m/s
const double NHC_RESIDUAL_THRESHOLD = 1.0;   // 正常 NHC 残差应 < 1 m/s
if (residual_vel.norm() > VEL_RESIDUAL_THRESHOLD || residual_nhc.norm() > NHC_RESIDUAL_THRESHOLD) {
    LOG(WARNING) << "[Obs] Wheel speed residual too large, skip update";
    return;
}
```

---

### 问题7: IMU重力观测中加速度计偏置（ba）雅可比缩放错误

**现象**: ba校正不足，长期累积使IMU速度积分产生漂移。

**根因**: `∂r/∂ba` 使用 `-0.1*I` 而非正确的 `-I`，导致ba每次更新幅度仅为正确的10%。

**修复**:
```cpp
H.block<3, 3>(0, 18) = -Eigen::Matrix3d::Identity();  // 之前错误地用了 -0.1*I
```

---

## 三、关键设计决策

### 1. 观测更新顺序

在 `Run()` 函数中，每帧处理顺序为：
```
1. IMU 传播 (p_imu_->Process)
2. LiDAR IEKF 更新 (update_iterated_dyn_share_modified)
3. 退化检测 & P矩阵膨胀
4. 传感器健康状态更新
5. 轮速观测更新 (ApplyWheelSpeedObservation)
6. GNSS姿态观测更新 (ApplyGnssAttitudeObservation)
7. IMU重力观测更新 (ApplyImuGravityObservation)
```

### 2. 雅可比矩阵推导

#### NHC 旋转雅可比 (∂h/∂rot)
```
观测模型: h(x) = R^T[1:2, :] * vel
R → R * Exp(δθ) 时:
  R_new^T ≈ (I - hat(δθ)) * R^T
  h_new = R^T * vel + hat(R^T * vel) * δθ
  ∂h/∂rot = hat(R^T * vel) = hat(v_body)  (正号)
```

#### 轮速速度约束旋转雅可比 (∂h/∂rot)
```
观测模型: h(x) = R * v_body
R → R * Exp(δθ) 时:
  R_new ≈ R * (I + hat(δθ))
  h_new = R * v_body + R * hat(δθ) * v_body
        = R * v_body - R * hat(v_body) * δθ
  ∂h/∂rot = -R * hat(v_body)  (负号)
```

#### IMU重力观测雅可比 (∂r/∂rot)
```
残差: r = (z_acc - ba) + R^T * grav
R → R * Exp(δθ):
  R_new^T * grav = R^T * grav + hat(R^T * grav) * δθ
  ∂r/∂rot = hat(R^T * grav)
```

### 3. 协方差膨胀策略

| 状态 | 膨胀目标 | 依据 |
|------|----------|------|
| 旋转 (rot) | 2.0° = 1.22e-3 rad² | 允许轮速/GNSS修正 |
| 速度 (vel) | 1.0 m²/s² | 与轮速噪声量级匹配 |

### 4. 残差门限

| 观测 | 门限 | 含义 |
|------|------|------|
| 轮速残差 | 2.0 m/s | 正常应 < 2 m/s |
| NHC残差 | 1.0 m/s | 正常应 < 1 m/s |
| GNSS姿态残差 | 0.5 rad (28.6°) | 防止异常更新 |
| IMU重力残差 | 2.0 m/s² | 检测车辆加速 |

---

## 四、当前配置参数 (config/slam_odometry_v2.yaml)

```yaml
vel_gnss:
  use_vel: true                    # 启用轮速观测
  use_gnss_attitude: true          # 启用GNSS姿态观测
  can_vel_diff_thresh_kmh: 2.0
  can_timeout_sec: 0.1
  gnss_attitude_timeout_sec: 0.2
  attitude_diff_thresh_deg: 2.0
  attitude_correction_gain: 0.7

  # 观测噪声
  wheel_velocity_noise: 0.01       # 轮速观测噪声方差 (m²/s²)
  non_holonomic_noise: 0.1         # NHC噪声方差 (m²/s²)
  gnss_attitude_noise_rad2: 0.01   # GNSS姿态噪声方差 (rad²)

  # IMU重力观测
  use_imu_gravity: false           # 当前禁用
  imu_gravity_noise: 0.5           # 重力观测噪声方差
  imu_gravity_acc_threshold: 0.2   # 低加速检测阈值
```

**注意**: `use_imu_gravity: false` 当前禁用IMU重力观测，因为需要进一步验证其在隧道场景中的效果。

---

## 五、待处理问题

### 1. IMU重力观测在隧道场景中的验证

- 当前配置中 `use_imu_gravity: false`，因为IMU重力观测的残差门限和雅可比设计仍需实际测试
- 需要验证：在隧道中启用后，roll/pitch漂移是否被有效抑制
- 需要验证：Z轴位置是否不再漂移

### 2. GNSS与IMU重力观测同时启用的冲突

- 两个观测都约束roll/pitch，可能产生冲突导致状态震荡
- 建议：GNSS可用时优先使用GNSS，隧道中无GNSS时启用IMU重力

### 3. 协方差膨胀策略的进一步优化

- 当前速度协方差膨胀到1.0 m²/s²，可能过大或过小
- 需要根据实际测试调整

### 4. 轮速噪声参数调整

- 参考lm_slam使用 `wheel_velocity_noise: 0.1089`（0.33²）
- 当前使用 `0.01`，可能对轮速过度信任

---

## 六、运行日志关键解读

| 日志内容 | 含义 | 处理方式 |
|----------|------|----------|
| `[Degeneracy] DETECTED! ratio=X` | LiDAR观测退化 | 膨胀P矩阵，启用其他观测 |
| `[Degeneracy] inflating P_rot direction i` | 旋转协方差膨胀 | 允许其他观测修正旋转 |
| `[Degeneracy] inflating P_vel direction i` | 速度协方差膨胀 | 允许轮速修正速度 |
| `[Obs] Wheel pre-update P diag` | 轮速更新前协方差 | 调试用，检查协方差量级 |
| `[Obs] wheel speed update: residual_norm=X` | 轮速更新残差 | 正常应 < 2 m/s |
| `[Obs] GNSS attitude residual too large` | GNSS姿态残差过大 | 跳过更新，等待下一帧 |
| `[Obs] IMU gravity SKIP` | 车辆加速，跳过重力观测 | 等待低加速状态 |
| `No Effective Points!` | LiDAR匹配失败 | 检查退化/协方差膨胀是否合理 |
| `Failed to find match for field 't'` | 点云缺少字段 | 不影响算法，可忽略 |

---

## 七、参考：lm_slam 与 slam_odometry_v2 对比

| 特性 | lm_slam | slam_odometry_v2 |
|------|---------|-------------------|
| 更新方式 | 单次EKF更新 | IEKF迭代更新 |
| LiDAR观测维度 | 12维 | 12维 |
| 观测触发时机 | 各自回调中 | 同一时间点串行 |
| 轮速噪声 | 0.1089 (0.33²) | 0.01 |
| GNSS噪声 | 0.03 rad² | 0.01 rad² |
| 协方差锁死 | 无（单次更新） | 有（迭代收敛后P极小） |
| 退化处理 | 有 | 有（P矩阵膨胀） |

---

## 八、关键文件路径

| 文件 | 路径 |
|------|------|
| 算法主文件 | `src/faster_lio/laser_odometry.cc` |
| 头文件 | `include/faster_lio/laser_odometry.h` |
| 状态定义 | `include/faster_lio/use-ikfom.hpp` |
| ESKF核心 | `include/faster_lio/IKFoM_toolkit/esekfom/esekfom.hpp` |
| IMU处理 | `include/faster_lio/imu_processing.hpp` |
| 配置文件 | `config/slam_odometry_v2.yaml` |
| 节点代码 | `src/ros/slam_odometry_v2_node.cc` |
| 设计文档 | `docs/slam_mapping_v2_design.md` |
| 多传感器设计 | `docs/multi_sensor_observation_design.md` |
| **本总结** | `docs/session_summary.md` |