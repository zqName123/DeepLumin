# Laser Odometry 多传感器融合优化设计方案

> 基于 FusionCore 工程实践分析，针对当前激光雷达里程计中的 GNSS 姿态约束失效、协方差管理、自适应噪声等问题提出系统性优化方案。
>
> 文档版本：v1.0
> 日期：2026-09-08

---

## 目录

1. [问题分析：GNSS 无法有效约束 Roll 的根本原因](#1-问题分析gnss-无法有效约束-roll-的根本原因)
2. [优化优先级矩阵](#2-优化优先级矩阵)
3. [P0 优化方案：马氏距离异常值检测](#3-p0-优化方案马氏距离异常值检测)
4. [P0 优化方案：自适应测量噪声估计](#4-p0-优化方案自适应测量噪声估计)
5. [P1 优化方案：GNSS 姿态观测 H 矩阵修正](#5-p1-优化方案gnss-姿态观测-h-矩阵修正)
6. [P1 优化方案：DCReg 协方差膨胀修复与增强](#6-p1-优化方案dcreg-协方差膨胀修复与增强)
7. [P1 优化方案：地面约束伪测量增强](#7-p1-优化方案地面约束伪测量增强)
8. [P2 优化方案：IMU 重力方向观测激活](#8-p2-优化方案imu-重力方向观测激活)
9. [P2 优化方案：编码器 WZ 偏置在线估计](#9-p2-优化方案编码器-wz-偏置在线估计)
10. [P3 优化方案：延迟测量补偿](#10-p3-优化方案延迟测量补偿)
11. [实施路线图](#11-实施路线图)
12. [附录：关键数学推导](#12-附录关键数学推导)

---

## 1. 问题分析：GNSS 无法有效约束 Roll 的根本原因

### 1.1 现象描述

在每次 GNSS 姿态更新时，虽然 GNSS 提供了独立的 roll/pitch 观测，但：
- **P(3,3)（roll 方差）在 GNSS 更新前已经非常小**（通常 < 1e-6 rad²）
- GNSS 更新后 roll 几乎不变，P(3,3) 也不降
- 即使 GNSS 的 roll 存在较大偏差，滤波器也无法修正

### 1.2 根本原因链路

```
LiDAR IEKF 更新（点面匹配）
  ↓
  roll 被大量平面特征强约束
  ↓
  P(3,3) 收敛到 1e-6 ~ 1e-8 rad² 量级
  ↓
DCReg 退化检测 → 可能检测到 roll 退化
  ↓
  InflateDegeneratePFromDCReg() 膨胀 P(3,3)
    但存在以下问题：
    ① 物理轴膨胀丢失协方差结构（只设对角，不恢复交叉项）
    ② any_rot_degenerate 永远为 false → 速度块膨胀是死代码
  ↓
GNSS 姿态更新到达
  ↓
  Kalman 增益 K = P(3,3) / (P(3,3) + R_gnss)
  P(3,3) ≈ 1e-6, R_gnss ≈ 1e-4 → K ≈ 0.01
  ↓
  GNSS 观测几乎无法修正 roll ❌
```

### 1.3 四个关键 bug/设计缺陷

| 编号 | 缺陷 | 位置 | 影响 |
|------|------|------|------|
| #1 | `any_rot_degenerate` 永远为 false | L465-L470 被注释 | 速度块膨胀死代码，P(12:14,12:14) 永不膨胀 |
| #2 | DCReg 膨胀只改对角元素 | L497-L509 | 丢失协方差交叉项，膨胀后协方差结构不完整 |
| #3 | GNSS 姿态观测 H 矩阵硬编码单位阵 | L735 | 未正确建模 SO3 切空间到 roll/pitch 的雅可比 |
| #4 | 测量噪声 R 全部固定 | 全局 | 无法自适应传感器退化（如 GNSS 多路径效应） |

### 1.4 方案覆盖关系

| 优化方案 | 直击的缺陷 | 解决路径 |
|----------|-----------|---------|
| P0 马氏距离检测 | #4 的补充 | 拒绝异常 GNSS 测量，防止错误注入 |
| P0 自适应 R | #4 的核心 | 在线调整测量噪声，鲁棒应对传感器退化 |
| P1 GNSS H 矩阵修正 | #3 | 正确建模观测模型，提高 roll/pitch 约束效率 |
| P1 DCReg 膨胀修复 | #1, #2 | 修复膨胀死代码，恢复协方差结构 |
| P1 地面约束增强 | 间接 | 增加 VZ/AZ=0 约束，辅助 Z 轴稳定性 |
| P2 IMU 重力观测 | 间接 | 利用加速度计约束 roll/pitch，频率更高 |
| P2 编码器偏置 | 间接 | 提高轮速观测精度，间接改善姿态 |

---

## 2. 优化优先级矩阵

| 优先级 | 优化项 | 当前状态 | 预期收益 | 工作量 | 依赖 | 风险 |
|--------|--------|---------|---------|--------|------|------|
| 🔴 **P0** | **马氏距离异常值检测** | 简单阈值 | 高：防止 GNSS 错误测量注入 | 低（1 个函数） | 无 | 极低 |
| 🔴 **P0** | **自适应测量噪声 R** | 固定 R 矩阵 | 高：自动适应传感器退化 | 中（滑动窗口 + adapt_R） | 无 | 低 |
| 🟠 **P1** | **DCReg 膨胀修复+增强** | 膨胀死代码 + 物理轴膨胀 | 高：恢复协方差膨胀有效性 | 中（解注释 + 特征分解） | 无 | 中 |
| 🟠 **P1** | **GNSS 姿态 H 矩阵修正** | 硬编码单位阵 | 中：提高 roll/pitch 约束精度 | 低（修改 H 矩阵计算） | 无 | 低 |
| 🟠 **P1** | **地面约束 VZ/AZ=0** | 仅 NHC(vy=0,vz=0) | 中：改善 Z 轴稳定性 | 低（新增 2 行观测） | 无 | 低 |
| 🟡 **P2** | **IMU 重力方向观测激活** | 被注释 | 中：高频 roll/pitch 约束 | 低（取消注释 + 调参） | 无 | 低 |
| 🟡 **P2** | **编码器 WZ 偏置估计** | 缺失 | 中：长时航向保持 | 高（需新增状态或在线补偿） | 无 | 中 |
| 🟢 **P3** | **延迟测量补偿** | 缺失 | 低：LiDAR 延迟小 | 高（需 IMU 缓冲区） | 自适应 R | 高 |
| 🟢 **P3** | **GPS 航迹航向融合** | 缺失 | 高：航向可观测性 | 高（需航向状态管理） | P0+P1 完成 | ⏸️ 暂缓 |
| 🟢 **P3** | **跨传感器时钟偏斜检测** | 缺失 | 低：多传感器时间同步 | 中 | 无 | 低 |

---

## 3. P0 优化方案：马氏距离异常值检测

### 3.1 问题描述

当前代码使用简单阈值进行异常值检测：

```cpp
// ApplyWheelSpeedObservation 中
if (residual_vel.norm() > VEL_RESIDUAL_THRESHOLD ||
    residual_nhc.norm() > NHC_RESIDUAL_THRESHOLD) {
    return;  // 直接跳过
}
```

以及 GNSS 姿态观测中**完全没有异常值检测**。

### 3.2 方案设计

#### 3.2.1 马氏距离公式

对于创新向量 `ν = z - h(x)`，其马氏距离为：

```
d² = νᵀ · S⁻¹ · ν
```

其中 `S = H · P · Hᵀ + R` 是创新协方差矩阵。

#### 3.2.2 卡方阈值

| 观测维度 | 自由度 | 阈值 (99.9%) | 含义 |
|----------|--------|-------------|------|
| 轮速速度 (3D) | 3 | 16.27 | 前向速度 + NHC |
| 轮速速度+WZ (5D) | 5 | 20.52 | 含 WZ 约束 |
| GNSS 姿态 (3D) | 3 | 16.27 | roll/pitch/yaw |
| GNSS 姿态 roll-only (1D) | 1 | 10.83 | 单轴观测 |
| 地面约束 (1D) | 1 | 10.83 | VZ=0 或 AZ=0 |

#### 3.2.3 实现方案

**新增函数 `MahalanobisOutlierRejection`**：

```cpp
/// 马氏距离异常值检测
/// @param residual 创新向量 ν
/// @param H 观测雅可比 H
/// @param P 当前状态协方差 P
/// @param R 测量噪声协方差 R
/// @param threshold 卡方阈值
/// @return (是否异常, 马氏距离)
template <int dim>
std::pair<bool, double> MahalanobisOutlierRejection(
    const Eigen::Matrix<double, dim, 1>& residual,
    const Eigen::Matrix<double, dim, 23>& H,
    const Eigen::Matrix<double, 23, 23>& P,
    const Eigen::Matrix<double, dim, dim>& R,
    double threshold) {

    // S = H * P * H^T + R
    Eigen::Matrix<double, dim, dim> S =
        H * P * H.transpose() + R;

    // d^2 = ν^T * S^{-1} * ν
    // 使用 LLT 分解求解，避免显式求逆
    auto llt = S.llt();
    if (llt.info() != Eigen::Success) {
        return {true, std::numeric_limits<double>::max()};
    }
    double mahalanobis = residual.transpose() *
        llt.solve(residual);

    return {mahalanobis > threshold, mahalanobis};
}
```

#### 3.2.4 集成位置

在以下位置调用：

1. **`ApplyWheelSpeedObservation()` 末尾**：替代当前简单阈值
2. **`ApplyGnssAttitudeObservation()` 中**：新增检测，在 `kf_.update_single_observation()` 之前
3. **`ApplyImuGravityObservation()` 中**：新增检测

#### 3.2.5 日志输出

```cpp
auto [is_outlier, md] = MahalanobisOutlierRejection(...);
if (is_outlier) {
    LOG(WARNING) << "[Obs] GNSS attitude Mahalanobis outlier: "
                 << "md=" << md << " threshold=" << threshold
                 << " H_trace=" << H.trace()
                 << " residual_norm=" << residual.norm()
                 << " P_rot=" << P.diagonal().segment<3>(3).transpose();
    return;
}
```

---

## 4. P0 优化方案：自适应测量噪声估计

### 4.1 问题描述

当前所有测量噪声矩阵 R 均为固定值：

```cpp
// ApplyGnssAttitudeObservation
double variance = gnss_attitude_noise_rad2_;  // 固定值
Eigen::MatrixXd R = variance * Eigen::Matrix3d::Identity();

// ApplyWheelSpeedObservation
R.block<3, 3>(0, 0) = wheel_velocity_noise_ * Eigen::Matrix3d::Identity();
```

在 GNSS 信号质量变化（多路径、遮挡、卫星数量变化）时，固定 R 无法自适应调整，导致：
- R 设得太小 → GNSS 噪声大时错误注入
- R 设得太大 → GNSS 信号好时约束不足

### 4.2 方案设计

#### 4.2.1 核心算法：基于创新序列的在线协方差估计

从 FusionCore 的 `adapt_R()` 函数借鉴：

```cpp
/// 自适应测量噪声协方差估计
/// 使用滑动窗口的创新序列，估计实际测量噪声协方差
/// R_new = (1 - alpha) * R_old + alpha * C_hat
/// 其中 C_hat 是窗口内创新的协方差估计
/// @param R 当前测量噪声协方差（会被更新）
/// @param R_floor 噪声下限（保护，防止 R 过小）
/// @param window 创新序列滑动窗口
/// @param innovation 当前创新向量
/// @param enabled 是否启用自适应
template <int dim>
void AdaptR(Eigen::Matrix<double, dim, dim>& R,
            const Eigen::Matrix<double, dim, dim>& R_floor,
            InnovationWindow<dim>& window,
            const Eigen::Matrix<double, dim, 1>& innovation,
            bool enabled,
            double alpha = 0.01) {

    if (!enabled) return;

    window.push(innovation);
    if (!window.ready()) return;

    // 从滑动窗口估计实际协方差
    auto C_hat = window.estimate_covariance();

    // 指数移动平均
    R = (1.0 - alpha) * R + alpha * C_hat;

    // 保护下限：R 不能低于初始配置值
    for (int i = 0; i < dim; ++i) {
        R(i, i) = std::max(R(i, i), R_floor(i, i));
    }
}
```

#### 4.2.2 滑动窗口设计

```cpp
/// 创新序列滑动窗口
template <int dim>
struct InnovationWindow {
    static constexpr int WINDOW_SIZE = 200;  // 约 2 秒（100 Hz IMU 时）

    std::queue<Eigen::Matrix<double, dim, 1>> buffer_;
    Eigen::Matrix<double, dim, 1> sum_;
    int count_ = 0;

    void push(const Eigen::Matrix<double, dim, 1>& inno) {
        buffer_.push(inno);
        sum_ += inno;
        count_++;

        if (buffer_.size() > WINDOW_SIZE) {
            sum_ -= buffer_.front();
            buffer_.pop();
            count_--;
        }
    }

    bool ready() const { return count_ >= WINDOW_SIZE / 2; }

    Eigen::Matrix<double, dim, dim> estimate_covariance() const {
        if (count_ < 2) return Eigen::Matrix<double, dim, dim>::Zero();

        Eigen::Matrix<double, dim, 1> mean = sum_ / count_;
        Eigen::Matrix<double, dim, dim> cov;
        cov.setZero();

        // 单次遍历计算协方差
        auto buf = buffer_;  // 拷贝
        while (!buf.empty()) {
            auto diff = buf.front() - mean;
            cov += diff * diff.transpose();
            buf.pop();
        }
        cov /= (count_ - 1);

        return cov;
    }
};
```

#### 4.2.3 自适应 R 的保护机制

**保护下限（R_floor）**：R 不能低于初始配置值，防止滤波器过度自信导致发散。

**创新协方差饱和**：当 C_hat 的某通道值超过 `max_R_scale * R_initial` 时，截断到该上限，防止单次大创新破坏 R 的长期平滑。

```cpp
// 饱和上限保护
constexpr double MAX_R_SCALE = 100.0;
for (int i = 0; i < dim; ++i) {
    if (C_hat(i, i) > MAX_R_SCALE * R_initial(i, i)) {
        LOG(WARNING) << "[AdaptR] saturating C_hat[" << i << "]: "
                     << C_hat(i, i) << " > " << MAX_R_SCALE * R_initial(i, i);
        C_hat(i, i) = MAX_R_SCALE * R_initial(i, i);
    }
}
```

#### 4.2.4 各传感器的自适应配置

| 传感器 | 自适应 R 变量 | R_floor | alpha | 窗口大小 |
|--------|-------------|---------|-------|---------|
| 轮速速度 (3D) | `R_wheel_vel_` | `wheel_velocity_noise_ * I` | 0.01 | 200 |
| 轮速 NHC (2D) | `R_wheel_nhc_` | `wheel_nhc_noise_ * I` | 0.01 | 200 |
| GNSS 姿态 (3D) | `R_gnss_att_` | `gnss_attitude_noise_rad2_ * I` | 0.005 | 100 |
| IMU 重力 (3D) | `R_imu_grav_` | `imu_gravity_noise_ * I` | 0.005 | 100 |

**为什么 GNSS 的 alpha 更小**：GNSS 更新频率低（1-10 Hz），需要更长的平滑时间常数。

#### 4.2.5 集成位置

在 `ApplyWheelSpeedObservation()` 和 `ApplyGnssAttitudeObservation()` 的末尾，UKF 更新之后：

```cpp
// 在 ApplyWheelSpeedObservation 末尾
if (use_adaptive_wheel_) {
    AdaptR<3>(R_wheel_vel_, R_wheel_vel_floor_,
              wheel_vel_inno_window_, residual_vel, true);
    AdaptR<2>(R_wheel_nhc_, R_wheel_nhc_floor_,
              wheel_nhc_inno_window_, residual_nhc, true);
}

// 在 ApplyGnssAttitudeObservation 末尾
if (use_adaptive_gnss_) {
    AdaptR<3>(R_gnss_att_, R_gnss_att_floor_,
              gnss_att_inno_window_, residual, true);
}
```

#### 4.2.6 调试与监控

```cpp
// 每 100 帧打印一次自适应 R 状态
static int adapt_print_count = 0;
if (++adapt_print_count % 100 == 0) {
    LOG(INFO) << "[AdaptR] R_wheel_vel diag: "
              << R_wheel_vel_.diagonal().transpose()
              << " R_gnss_att diag: "
              << R_gnss_att_.diagonal().transpose();
}
```

---

## 5. P1 优化方案：GNSS 姿态观测 H 矩阵修正

### 5.1 问题描述

当前 `ApplyGnssAttitudeObservation()` 中：

```cpp
// 当前：H 矩阵硬编码为单位阵的前两行
H.block<2, 3>(0, 3) = Eigen::Matrix3d::Identity().topRows<2>();
```

但残差是在 SO3 切空间计算的：

```cpp
Eigen::AngleAxisd aa(R_diff);
Eigen::Vector3d residual = aa.angle() * aa.axis();
residual[2] = 0.0;  // 零化 yaw
```

**问题**：当当前姿态与 GNSS 姿态差异较大时，SO3 切空间残差的前两个分量不完全等于 `[roll_error, pitch_error]`，单位阵近似仅在 roll/pitch 接近零时成立。

### 5.2 方案设计

#### 5.2.1 正确的 H 矩阵

H 矩阵应该是 SO3 到 (roll, pitch) 的雅可比映射：

```cpp
// 正确的 H 矩阵设计
// 残差：r = (R_est^T * R_gnss) 的 roll/pitch 分量
// 状态：rot = SO3 (索引 3-5)
// 雅可比：∂r/∂δθ = J_rp

// 当前姿态旋转矩阵
Eigen::Matrix3d R_est = state.rot.toRotationMatrix();

// 提取当前欧拉角
double roll = std::atan2(R_est(2,1), R_est(2,2));
double pitch = -std::asin(R_est(2,0));

// 构建 (roll, pitch) → SO3 的雅可比
// 欧拉角 ZYX 顺序: R = Rz(yaw) * Ry(pitch) * Rx(roll)
// ∂(roll, pitch)/∂δθ = ...
Eigen::Matrix<double, 2, 3> J_rp;
J_rp.setZero();

// 近似：当 roll/pitch 较小时，J_rp ≈ [e1, e2]^T
// 精确推导见附录 A
if (std::abs(roll) < 10.0 * M_PI / 180.0 &&
    std::abs(pitch) < 10.0 * M_PI / 180.0) {
    // 小角度近似成立
    J_rp(0, 0) = 1.0;  // roll → rot_x
    J_rp(1, 1) = 1.0;  // pitch → rot_y
} else {
    // 大角度：使用精确雅可比
    // 见附录 A 的推导
    J_rp(0, 0) = 1.0;
    J_rp(0, 1) = std::sin(roll) * std::tan(pitch);
    J_rp(0, 2) = std::cos(roll) * std::tan(pitch);
    J_rp(1, 1) = std::cos(roll);
    J_rp(1, 2) = -std::sin(roll);
}

H.block<2, 3>(0, 3) = J_rp;
```

#### 5.2.2 简化方案（推荐初始实施）

由于实际应用中 roll/pitch 通常较小（地面车辆），采用小角度近似，但明确注释：

```cpp
// H 矩阵：SO3 → (roll, pitch) 映射
// 小角度近似：当 roll/pitch < 10° 时，J_rp ≈ [e1, e2]^T
// 对于地面车辆通常成立（roll/pitch < 5°）
// 如需精确映射，参考附录 A
H.block<2, 3>(0, 3) = Eigen::Matrix3d::Identity().topRows<2>();
H.row(2).setZero();  // yaw 始终不更新
```

#### 5.2.3 独立 roll/pitch 噪声配置

当前使用统一的 `gnss_attitude_noise_rad2_`，建议拆分为独立配置：

```cpp
// 独立的 roll/pitch/yaw 噪声
// 因为 GNSS 的 roll 和 pitch 精度通常不同
Eigen::Matrix3d R_gnss_att = Eigen::Matrix3d::Zero();
R_gnss_att(0, 0) = gnss_roll_noise_rad2_;   // roll 方差
R_gnss_att(1, 1) = gnss_pitch_noise_rad2_;  // pitch 方差
R_gnss_att(2, 2) = 1e6;  // yaw 方差极大 → 不更新 yaw
```

---

## 6. P1 优化方案：DCReg 协方差膨胀修复与增强

### 6.1 问题描述

当前 `InflateDegeneratePFromDCReg()` 存在两个严重问题：

**问题 #1**：`any_rot_degenerate` 永远为 false（L465-L470 被注释）

```cpp
// 当前代码（L465-L470）：
bool any_rot_degenerate = false;

// 旋转膨胀 (索引 3-5)
// if (dc_result.degenerate_mask[0] || dc_result.degenerate_mask[1] ||
//     dc_result.degenerate_mask[2]) {
//     any_rot_degenerate = true;
//     ...
// }
```

导致 `if (any_rot_degenerate)` 保护的速度块膨胀（L539-L550）永远是死代码。

**问题 #2**：物理轴膨胀丢失协方差结构

```cpp
// 当前：只膨胀对角元素
P(3, 3) = std::max(P(3, 3), target_sigma * target_sigma);  // roll
P(4, 4) = std::max(P(4, 4), target_sigma * target_sigma);  // pitch
P(5, 5) = std::max(P(5, 5), target_sigma * target_sigma);  // yaw
```

丢失了 P 矩阵的交叉项（如 `P(3,5)` roll-yaw 相关性），破坏了协方差结构。

### 6.2 方案设计

#### 6.2.1 修复：旋转块特征分解膨胀

替代当前物理轴膨胀，使用特征分解在退化方向上进行定向膨胀：

```cpp
void LaserOdometry::InflateDegeneratePFromDCReg(const DCRegResult& dc_result) {
    Eigen::Matrix<double, 23, 23> P = kf_.get_P();

    constexpr double ROT_TARGET_SIGMA = 2.0 * M_PI / 180.0;  // 2.0°
    constexpr double ROT_TARGET_VAR = ROT_TARGET_SIGMA * ROT_TARGET_SIGMA;
    constexpr double POS_TARGET_SIGMA = 0.5;                  // 0.5m
    constexpr double POS_TARGET_VAR = POS_TARGET_SIGMA * POS_TARGET_SIGMA;
    constexpr double VEL_TARGET_SIGMA = 1.0;                  // 1.0 m/s
    constexpr double VEL_TARGET_VAR = VEL_TARGET_SIGMA * VEL_TARGET_SIGMA;

    bool any_rot_degenerate = false;

    // ===== 旋转块膨胀 (索引 3-5) =====
    // 使用特征分解保留协方差结构
    if (dc_result.degenerate_mask[0] ||
        dc_result.degenerate_mask[1] ||
        dc_result.degenerate_mask[2]) {

        any_rot_degenerate = true;
        Eigen::Matrix3d P_rot = P.block<3,3>(3,3);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_rot);

        if (es.info() == Eigen::Success) {
            Eigen::Vector3d lambda = es.eigenvalues();
            Eigen::Matrix3d V = es.eigenvectors();

            // 对每个退化方向进行动态膨胀
            for (int i = 0; i < 3; i++) {
                double score = dc_result.degeneracy_scores[i];
                if (score > 0.01 || lambda(i) < ROT_TARGET_VAR) {
                    // 动态膨胀目标：退化越严重，膨胀越大
                    double dynamic_target = ROT_TARGET_VAR * (1.0 + score * 9.0);
                    // 平滑插值
                    lambda(i) = std::max(lambda(i), dynamic_target);
                }
            }
            P.block<3,3>(3,3) = V * lambda.asDiagonal() * V.transpose();
        } else {
            // 特征分解失败，回退到对角膨胀
            for (int i = 0; i < 3; i++) {
                if (dc_result.degenerate_mask[i]) {
                    double score = dc_result.degeneracy_scores[i];
                    double target = ROT_TARGET_VAR * (1.0 + score * 9.0);
                    P(3+i, 3+i) = std::max(P(3+i, 3+i), target);
                }
            }
        }
    }

    // ===== 位置块膨胀 (索引 0-2) =====
    if (dc_result.degenerate_mask[3] ||
        dc_result.degenerate_mask[4] ||
        dc_result.degenerate_mask[5]) {

        Eigen::Matrix3d P_pos = P.block<3,3>(0,0);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_pos);

        if (es.info() == Eigen::Success) {
            Eigen::Vector3d lambda = es.eigenvalues();
            Eigen::Matrix3d V = es.eigenvectors();

            for (int i = 0; i < 3; i++) {
                if (lambda(i) < POS_TARGET_VAR) {
                    lambda(i) = POS_TARGET_VAR;
                }
            }
            P.block<3,3>(0,0) = V * lambda.asDiagonal() * V.transpose();
        }
    }

    // ===== 速度块膨胀 (索引 12-14) =====
    // 旋转退化时连带膨胀（IMU 传播中速度与旋转强耦合）
    if (any_rot_degenerate) {
        Eigen::Matrix3d P_vel = P.block<3,3>(12,12);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_vel);

        if (es.info() == Eigen::Success) {
            Eigen::Vector3d lambda = es.eigenvalues();
            Eigen::Matrix3d V = es.eigenvectors();

            for (int i = 0; i < 3; i++) {
                if (lambda(i) < VEL_TARGET_VAR) {
                    lambda(i) = VEL_TARGET_VAR;
                }
            }
            P.block<3,3>(12,12) = V * lambda.asDiagonal() * V.transpose();
        }
    }

    // ===== Z 轴位置连带膨胀 =====
    // pitch 退化 → IMU 传播中重力补偿不准确 → Z 轴积分漂移
    if (dc_result.degenerate_mask[1]) {  // pitch degenerate
        P(2, 2) = std::max(P(2, 2), POS_TARGET_VAR);
        P(14, 14) = std::max(P(14, 14), VEL_TARGET_VAR);
    }

    kf_.change_P(P);

    // 日志输出
    LOG(WARNING) << "[DCReg] P inflated: mask=["
                 << dc_result.degenerate_mask[0] << ","
                 << dc_result.degenerate_mask[1] << ","
                 << dc_result.degenerate_mask[2] << ","
                 << dc_result.degenerate_mask[3] << ","
                 << dc_result.degenerate_mask[4] << ","
                 << dc_result.degenerate_mask[5] << "]"
                 << " cond_rot=" << dc_result.cond_schur_rot
                 << " cond_trans=" << dc_result.cond_schur_trans
                 << " scores=["
                 << dc_result.degeneracy_scores[0] << ","
                 << dc_result.degeneracy_scores[1] << ","
                 << dc_result.degeneracy_scores[2] << ","
                 << dc_result.degeneracy_scores[3] << ","
                 << dc_result.degeneracy_scores[4] << ","
                 << dc_result.degeneracy_scores[5] << "]"
                 << " P_rot_diag_post="
                 << P.block<3,3>(3,3).diagonal().transpose();
}
```

#### 6.2.2 膨胀目标动态调整

| 退化分数 | 旋转膨胀目标 | 物理含义 |
|----------|-------------|---------|
| 0.0（未退化） | 不膨胀 | 保持原 P |
| 0.5 | 2.0° + 1.5° = 3.5° | 中等退化，适度膨胀 |
| 1.0（完全退化） | 2.0° + 9 × 2.0° = 20.0° | 严重退化，大幅膨胀 |

#### 6.2.3 退化检测阈值可配置

```cpp
// 在类成员中增加
double dcreg_condition_threshold_ = 10.0;  // 默认值
double dcreg_rot_inflation_base_deg_ = 2.0;  // 基础膨胀 2°
double dcreg_rot_inflation_max_deg_ = 20.0;  // 最大膨胀 20°
```

---

## 7. P1 优化方案：地面约束伪测量增强

### 7.1 问题描述

当前非完整性约束（NHC）只约束了车体系下的 `vy=0` 和 `vz=0`，但缺少：
- **世界系垂直速度 VZ=0**：地面机器人在世界系下的 Z 轴速度应为零
- **加速度计垂直加速度 AZ=0**：防止 IMU 重力补偿不匹配导致的 Z 轴漂移

### 7.2 方案设计

#### 7.2.1 世界系 VZ=0 约束

在 `ApplyWheelSpeedObservation()` 末尾增加：

```cpp
// ===== 观测 3: 世界系垂直速度 VZ = 0 =====
// 地面机器人在世界系下的垂直速度应为零
// 虚拟测量: h(x) = state.vel(2) = 0
// residual = 0 - state.vel(2)
// H = [0, 0, 0, ..., 0, 1, 0, 0]

Eigen::MatrixXd H3 = Eigen::MatrixXd::Zero(1, 23);
H3(0, 14) = 1.0;  // vel.z (世界系)

Eigen::VectorXd residual_vz(1);
residual_vz(0) = -state.vel(2);  // 0 - vel_z

Eigen::MatrixXd R_vz(1, 1);
R_vz(0, 0) = ground_vz_noise_;  // 可配置，默认 0.1 m/s²

// 马氏距离检测
auto [is_outlier, md] = MahalanobisOutlierRejection<1>(
    residual_vz, H3, P, R_vz, CHI2_THRESHOLD_1DOF);
if (is_outlier) {
    LOG_FIRST_N(WARNING, 10)
        << "[Obs] VZ constraint Mahalanobis outlier: " << md;
} else {
    kf_.update_single_observation(H3, residual_vz, R_vz);
}
```

#### 7.2.2 加速度计 AZ=0 约束

在 `ApplyWheelSpeedObservation()` 之后或独立函数中增加：

```cpp
// ===== 观测 4: 加速度计垂直加速度 AZ = 0 =====
// 地面机器人在车体系下的垂直加速度应为零
// 状态中的 ax/ay/az 是 IMU 测量值减去偏置后的"真实加速度"
// 对于地面机器人，az ≈ 0（没有垂直运动）
// 注意：当车辆在斜坡上时，az 不一定为零，因此需要谨慎使用

if (use_ground_az_constraint_) {
    Eigen::MatrixXd H4 = Eigen::MatrixXd::Zero(1, 23);
    // 注意：当前状态没有 ax/ay/az，这里是概念设计
    // 实际实现需要根据状态定义调整
    // 在 IMU 重力观测中已有类似约束

    // 更实际的方案：使用加速度计读数直接约束
    // 当车辆在平坦路面低速行驶时:
    // z_acc ≈ -R^T * g, 所以 az_meas - ba ≈ -R^T * g 的 z 分量
    // 残差: (az_meas - ba) - (-R^T * g)_z
}
```

#### 7.2.3 融合 5D→6D 观测（可选）

将 VZ 约束合并到原 5D 轮速观测中，形成 6D 观测：

```cpp
// 合并 H 矩阵
Eigen::MatrixXd H_full(6, 23);
H_full << H1, H2, H3;
Eigen::VectorXd residual_full(6);
residual_full << residual_vel, residual_nhc, residual_vz(0);
```

---

## 8. P2 优化方案：IMU 重力方向观测激活

### 8.1 问题描述

`ApplyImuGravityObservation()` 目前在 `Run()` 中被注释。

### 8.2 方案设计

#### 8.2.1 取消注释并增加低加速检测的改进

```cpp
// 在 Run() 中取消注释
if (use_imu_gravity_) {
    ApplyImuGravityObservation();
}
```

#### 8.2.2 改进低加速检测

当前检测：

```cpp
double diff = std::abs(acc_norm - common::G_m_s2);
return diff < imu_gravity_acc_threshold_;
```

改进为滑动窗口方差检测：

```cpp
bool LaserOdometry::isLowAcceleration() const {
    if (!has_imu_data_) return false;

    // 方法 1: 加速度模值检测（当前）
    double acc_norm = last_acc_.norm();
    double diff = std::abs(acc_norm - common::G_m_s2);
    if (diff > imu_gravity_acc_threshold_) return false;

    // 方法 2: 加速度计滑动窗口方差检测（新增）
    // 即使模值接近 G，剧烈转弯时也会有向心加速度
    // 方差检测可以过滤掉这种工况
    static constexpr int WINDOW = 10;
    static Eigen::Vector3d acc_buffer[WINDOW];
    static int idx = 0;

    acc_buffer[idx++ % WINDOW] = last_acc_;
    if (idx < WINDOW) return false;  // 缓冲区未满

    // 计算滑动窗口方差
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (int i = 0; i < WINDOW; i++) mean += acc_buffer[i];
    mean /= WINDOW;

    double var = 0.0;
    for (int i = 0; i < WINDOW; i++) {
        var += (acc_buffer[i] - mean).squaredNorm();
    }
    var /= WINDOW;

    // 方差小 → 低加速状态
    return var < 0.5;  // 可配置
}
```

#### 8.2.3 自适应加速度计噪声

```cpp
// 在 ApplyImuGravityObservation 末尾
if (use_adaptive_imu_gravity_) {
    AdaptR<3>(R_imu_grav_, R_imu_grav_floor_,
              imu_grav_inno_window_, residual, true);
}
```

---

## 9. P2 优化方案：编码器 WZ 偏置在线估计

### 9.1 问题描述

差速驱动的左右轮半径不一致、轮胎气压差异等会导致编码器 WZ 读数存在系统性偏置。FusionCore 通过状态 `B_EWZ` 在线估计这个偏置，并在 GPS 丢失时自动减去。

当前 laser_odometry 没有这个偏置估计，在 LiDAR 退化时编码器 WZ 偏置会累积为航向漂移。

### 9.2 方案设计

#### 9.2.1 方案 A：简单低通滤波补偿（推荐，低工作量）

不修改状态向量，而是在轮速观测前对 WZ 做在线补偿：

```cpp
// 在 ApplyWheelSpeedObservation 中增加
// 编码器 WZ 偏置在线估计

// 1. 获取当前姿态的角速度（通过 IMU 陀螺仪）
state_ikfom state = kf_.get_x();
Eigen::Vector3d gyro = last_gyro_;  // 原始 IMU 陀螺仪读数
Eigen::Vector3d bg;
for (int i = 0; i < 3; i++) bg(i) = state.bg[i];
Eigen::Vector3d true_w = gyro - bg;  // 去除偏置后的真实角速度

// 2. 编码器 WZ 与 IMU WZ 的差异 = 编码器偏置
double encoder_wz = last_can_.angular_velocity_z;  // 编码器原始 WZ
double imu_wz = true_w(2);  // IMU 的 Z 轴角速度

// 注意：需要将 IMU 的 WZ 转换到车体系
// 如果 IMU 与车体对齐，直接使用 true_w(2)
// 否则需要 R_imu_to_body 旋转

// 3. 低通滤波估计偏置
static double ewz_bias = 0.0;
static bool ewz_initialized = false;
constexpr double EWZ_ALPHA = 0.001;  // 非常慢的时间常数（~1000 帧）

if (ewz_initialized) {
    // 只在轮速有效时更新
    if (std::abs(encoder_wz) > 0.01) {  // 有运动时
        double diff = encoder_wz - imu_wz - ewz_bias;
        // 使用 Huber 抑制异常值
        double huber_w = std::abs(diff) < 0.1 ? 1.0 : 0.1 / std::abs(diff);
        ewz_bias += EWZ_ALPHA * huber_w * diff;
    }
} else {
    ewz_bias = encoder_wz - imu_wz;
    ewz_initialized = true;
}

// 4. 修正编码器 WZ
double corrected_encoder_wz = encoder_wz - ewz_bias;

// 5. 使用修正后的 WZ 构建观测
```

#### 9.2.2 方案 B：增加状态 B_EWZ（高工作量，慎重）

如果需要精确估计，需要在状态向量中增加 `B_EWZ` 维度：
- 状态从 23 维 → 24 维
- 需要修改 `state_ikfom` 定义
- 需要修改 IEKF 框架的 `boxplus`/`boxminus`
- 需要修改 `get_f`、`df_dx`、`df_dw`
- 需要修改 `ObsModel` 的 H 矩阵维度

**不建议在当前阶段实施方案 B**，因为：
1. 需要修改 `esekfom.hpp` 框架
2. 回归测试工作量大
3. 方案 A 已经可以解决大部分问题

---

## 10. P3 优化方案：延迟测量补偿

### 10.1 问题描述

当前所有传感器更新都假设测量与当前状态时间对齐。但实际上：
- GNSS 姿态有 50-200ms 延迟
- 轮速数据可能有 10-50ms 延迟
- 延迟导致时间不对齐，引入额外误差

### 10.2 方案设计（概念设计，暂不实施）

#### 10.2.1 IMU 状态快照缓冲区

```cpp
/// 状态快照
struct StateSnapshot {
    double timestamp;
    state_ikfom state;
    Eigen::Matrix<double, 23, 23> P;
};

std::deque<StateSnapshot> snapshot_buffer_;
static constexpr int SNAPSHOT_BUFFER_SIZE = 1000;  // ~10 秒
```

#### 10.2.2 延迟测量处理流程

```cpp
void LaserOdometry::ApplyDelayedMeasurement(
    double measurement_timestamp,
    std::function<void()> apply_update) {

    // 1. 保存当前状态
    state_ikfom current_state = kf_.get_x();
    auto current_P = kf_.get_P();

    // 2. 找到最接近的快照
    auto it = std::lower_bound(
        snapshot_buffer_.begin(), snapshot_buffer_.end(),
        measurement_timestamp,
        [](const StateSnapshot& s, double t) {
            return s.timestamp < t;
        });

    if (it == snapshot_buffer_.end()) {
        LOG(WARNING) << "[Delay] Measurement too old: "
                     << measurement_timestamp;
        return;
    }

    // 3. 回滚到快照
    kf_.change_x(it->state);
    kf_.change_P(it->P);

    // 4. 应用延迟测量
    apply_update();

    // 5. 通过 IMU 数据重放前推到当前时间
    ReplayImuFrom(measurement_timestamp, current_time);

    // 6. 恢复当前状态（如果重放已更新到最新）
}
```

---

## 11. 实施路线图

### 阶段一：P0 快速修复（1-2 天）

```
Step 1: 马氏距离异常值检测
  ├── 新增 MahalanobisOutlierRejection 模板函数
  ├── 集成到轮速观测（替代简单阈值）
  ├── 集成到 GNSS 姿态观测（新增）
  └── 集成到 IMU 重力观测（新增）

Step 2: DCReg 膨胀死代码修复
  ├── 解注释旋转块特征分解膨胀
  └── 修复 any_rot_degenerate 逻辑

测试验证：
  ├── 回放数据包，对比 P 矩阵膨胀前后变化
  └── 验证 GNSS 更新后 roll 是否被修正
```

### 阶段二：P0 自适应 R（2-3 天）

```
Step 3: 自适应测量噪声
  ├── 新增 InnovationWindow 模板类
  ├── 新增 AdaptR 模板函数
  ├── 集成到轮速观测（3D + 2D 独立自适应）
  ├── 集成到 GNSS 姿态观测（3D 自适应）
  └── 配置参数（alpha, window_size, R_floor）

测试验证：
  ├── 模拟 GNSS 信号质量变化场景
  └── 验证 R 矩阵自适应变化
```

### 阶段三：P1 增强（3-5 天）

```
Step 4: GNSS 姿态 H 矩阵修正
  ├── 添加 roll/pitch 独立噪声配置
  └── 验证小角度近似条件

Step 5: DCReg 膨胀增强
  ├── 特征分解保留协方差结构
  ├── 动态膨胀目标
  ├── 速度块膨胀激活
  └── Z 轴连带膨胀

Step 6: 地面约束增强
  ├── VZ=0 世界系垂直速度约束
  └── 马氏距离检测

测试验证：
  ├── 隧道场景航向漂移测试
  └── 长时间运行 Z 轴稳定性测试
```

### 阶段四：P2 优化（2-3 天）

```
Step 7: IMU 重力观测激活
  ├── 取消注释
  ├── 改进低加速检测
  └── 自适应 R

Step 8: 编码器 WZ 偏置（方案 A）
  ├── 低通滤波估计
  └── 轮速 WZ 修正

测试验证：
  ├── LiDAR 退化场景航向保持
  └── 编码器偏置收敛性
```

---

## 12. 附录：关键数学推导

### 12.1 附录 A：SO3 → (roll, pitch) 的精确雅可比

对于 ZYX 欧拉角顺序 `R = Rz(yaw) * Ry(pitch) * Rx(roll)`：

```
R(2,1) = cos(roll) * sin(pitch)  →  roll = atan2(R(2,1), R(2,2))
R(2,2) = cos(roll) * cos(pitch)
R(2,0) = -sin(pitch)             →  pitch = -asin(R(2,0))
```

对 SO3 切空间扰动 δθ = [δθx, δθy, δθz]ᵀ：

```
∂roll/∂δθ = [1, sin(roll)*tan(pitch), cos(roll)*tan(pitch)]
∂pitch/∂δθ = [0, cos(roll), -sin(roll)]
```

当 roll/pitch 较小时，`tan(pitch) ≈ 0, sin(roll) ≈ 0, cos(roll) ≈ 1`，雅可比退化为单位阵前两行。

### 12.2 附录 B：P 矩阵特征分解膨胀的数学意义

原始 P 矩阵（3×3 旋转块）的特征分解：

```
P_rot = V · Λ · Vᵀ
```

其中 V 的列向量是主方向，Λ 是对应方差。

膨胀操作 `Λ_out = max(Λ, Λ_target)` 等价于：在 V 定义的每个主方向上，确保方差不小于目标值。这保留了原始协方差的方向结构，只在退化方向上增加不确定性。

**对比物理轴膨胀**（当前做法）：

```
P_new(3,3) = max(P(3,3), σ²_target)
```

只增加 roll 对角元素，但 `P(3,4)`、`P(3,5)` 等交叉项不变，破坏了协方差的正定性结构。

### 12.3 附录 C：自适应 R 的收敛性

自适应 R 的核心是 EMA（指数移动平均）：

```
R_k = (1-α) · R_{k-1} + α · C_hat_k
```

这个一阶滤波器的时间常数 τ = -1/ln(1-α) ≈ 1/α（当 α 很小时）。

| α | 时间常数（帧数） | 含义 |
|---|----------------|------|
| 0.01 | ~100 帧 | 约 1 秒（100 Hz IMU） |
| 0.005 | ~200 帧 | 约 2 秒 |
| 0.001 | ~1000 帧 | 约 10 秒 |

**保护下限 R_floor** 的作用：
- 防止 R 过小 → 滤波器过度信任传感器 → 发散
- 应设置为传感器在最佳条件下的噪声水平

### 12.4 附录 D：GNSS 更新后 P(3,3) 的稳态分析

设 LiDAR 更新后 P(3,3) = P₀，GNSS 观测噪声 R_gnss = R，则 GNSS 更新后的 P(3,3) 为：

```
P₁ = (1/P₀ + 1/R)⁻¹ = P₀ · R / (P₀ + R)
```

当 P₀ ≪ R 时：

```
P₁ ≈ P₀  （GNSS 几乎不降低协方差，也无法修正状态）
```

要使 GNSS 有效，需要 P₀ 与 R 可比：

```
P₀ ≈ R  →  P₁ ≈ R/2  （GNSS 有效降低一半协方差）
```

**因此，DCReg 膨胀的目标是使 P₀ 达到与 R_gnss 可比的量级**。

| 场景 | R_gnss (rad²) | 目标 P₀ (rad²) | 对应 σ |
|------|--------------|----------------|--------|
| 高精度 GNSS | 1e-4 (0.01 rad) | ≥ 1e-4 | ≥ 0.57° |
| 低精度 GNSS | 1e-3 (0.03 rad) | ≥ 1e-3 | ≥ 1.8° |
| 极差 GNSS | 1e-2 (0.1 rad) | ≥ 1e-2 | ≥ 5.7° |

当前 DCReg 膨胀目标 2.0°（σ² ≈ 1.22e-3 rad²）对于低精度 GNSS 是足够的，但对于高精度 GNSS 仍然偏大。**建议将膨胀目标与 GNSS 噪声匹配**：读取当前 GNSS 的噪声水平，动态设置膨胀目标。
