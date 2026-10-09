# ZUPT / NHC 设计文档（零速约束 + 非完整约束）

> 目标：车辆静止时用 ZUPT 约束速度/标定零偏/对齐重力，行驶时用 NHC 约束侧向/垂向速度，
> 全程速度与姿态都有物理约束，消除"停车→起步偏移"与速度方向损坏两类问题。
> 本文档为实现规格（代码为参考实现，落地时按仓库现状微调行号）。

---

## 1. 可观测性分析（决定该约束什么）

| 量 | 静止时 | 行驶时 | 依据 |
|---|---|---|---|
| 速度 v | **强可观**（真值=0） | 见 NHC | 车不动 |
| roll/pitch | **强可观**（加计=重力） | 弱（受加速度污染） | 静止时 acc = -Rᵀg |
| bg 陀螺零偏 | **强可观**（陀螺=-bg） | 不可观 | 静止时 ω=0 |
| **yaw** | **完全不可观** | 依赖 lidar/GNSS | 重力绕 z 旋转不变 |
| vy/vz（车体系） | 0（含于 ZUPT） | **强可观**（=0） | 轮式不侧滑/不腾空 |

**铁律：yaw 永不重置、永不硬对齐**（对重力不可观，硬动必错）。

---

## 2. 静止检测器（ZUPT 的成败在此）

三方联合判据 + 滞回。单靠轮速会被坡道蠕动、1km/h 量化骗过。

```
wheel_low  = |wheel_speed_obs_| < 0.15 m/s
imu_static = (单样本陀螺模长峰值 < 0.5°/s) 且 (|‖acc‖-9.8| 峰值 < 0.15 m/s²)
连续满足 stationary_frames=20 帧(2s) → 进入静止模式
速度 > 0.15×release_factor(2) → 解除（滞回防起步抖动反复开关）
```

用**本帧 `measures_.imu_`** 做统计（时间正好覆盖扫描区间，无需新缓冲）。

### 2.1 判定代码（`Run()`，SyncPackages/去畸变之后）

```cpp
    // ===== 静止检测：轮速 + IMU 联合（ZUPT/GNSS冻结/插点门控共用）=====
    const double speed_for_map = use_wheel_speed_ ? std::fabs(wheel_speed_obs_)
                                                 : state_point_.vel_.norm();
    const bool wheel_low = speed_for_map < zupt_wheel_thresh_;

    Vec3d gyr_mean = Vec3d::Zero(), acc_mean = Vec3d::Zero();
    double gyr_peak = 0.0, acc_dev_peak = 0.0;
    const int n_imu = static_cast<int>(measures_.imu_.size());
    for (const auto& imu : measures_.imu_) {
        gyr_mean += imu->angular_velocity;
        acc_mean += imu->linear_acceleration;
        gyr_peak = std::max(gyr_peak, imu->angular_velocity.norm());
        acc_dev_peak = std::max(acc_dev_peak,
                std::fabs(imu->linear_acceleration.norm() - G_m_s2));
    }
    if (n_imu > 0) { gyr_mean /= n_imu; acc_mean /= n_imu; }
    const bool imu_static = (gyr_peak < zupt_gyr_thresh_) && (acc_dev_peak < zupt_acc_thresh_);

    if (wheel_low && imu_static) {
        static_hold_cnt_++;
    } else if (speed_for_map > zupt_wheel_thresh_ * release_factor_) {
        static_hold_cnt_ = 0;
    }
    const bool stationary_now = static_hold_cnt_ >= stationary_frames_;
    if (stationary_now) stationary_cnt_++; else stationary_cnt_ = 0;
    const bool gnss_freeze = stationary_cnt_ >= gnss_freeze_frames_;
    zupt_active_ = enable_zupt_ && stationary_now;
```

---

## 3. ZUPT（零速更新）

### 3.1 观测模型

复用 ESKF 现有 `ObsType::WHEEL_SPEED` 分支（速度块标准 KF 更新，`eskf.cpp` 无需改动）：

```
观测 z = 0（车速真值），h(x) = v，residual = z - h = -v，H = I3（映射状态 12..14）
R = zupt_var·I3（1e-4）→ K = P_vel/(P_vel+1e-4) ≈ 1，速度钉死
```

### 3.2 观测函数（`ObsModelWheelspeed` 开头加 ZUPT 分支）

```cpp
void LaserOdometryCore::ObsModelWheelspeed(NavState& s, ESKF::CustomObservationModel& obs) {
    // ===== ZUPT：静止模式输出"速度=0"观测 =====
    if (zupt_active_) {
        obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
        obs.residual_.resize(3);
        obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        obs.R_ = zupt_var_ * Eigen::Matrix3d::Identity();
        obs.residual_ = -s.vel_;                 // z=0
        obs.valid_ = true;
        return;
    }
    // ……原有 1D/3D 轮速路径不动……
}
```

### 3.3 调用（`Run()` 更新序列，lidar 之后）

```cpp
    if (zupt_active_) {
        const NavState st0 = kf_.GetX();
        kf_.Update(ESKF::ObsType::WHEEL_SPEED, zupt_var_);
        SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[ZUPT] v " << st0.vel_.norm() << " -> " << kf_.GetX().vel_.norm() << " m/s";
    } else if (use_wheel_speed_ && have_new_wheel_speed_) {
        kf_.Update(ESKF::ObsType::WHEEL_SPEED, wheel_sigma_ * wheel_sigma_);
        have_new_wheel_speed_ = false;
    }
```

### 3.4 bg 陀螺零偏标定（ZUPT 的附带收益，对起步瞬态最大）

静止时陀螺读数 = -bg（地球自转 0.004°/s 可忽略）→ **可直接置值**（少数可硬设的量）。

```cpp
    // 累积（Run() 静止判定后）
    if (zupt_active_ && n_imu > 0) {
        gyr_acc_sum_ += gyr_mean * n_imu;
        acc_acc_sum_ += acc_mean * n_imu;
        bg_acc_n_   += n_imu;
    } else {
        gyr_acc_sum_.setZero(); acc_acc_sum_.setZero(); bg_acc_n_ = 0;
        bg_calibrated_this_stop_ = false;
    }

    // 应用（每次停车至多一次，静止 ≥bg_calib_frames=20 后）
    if (zupt_active_ && !bg_calibrated_this_stop_ && bg_acc_n_ >= bg_calib_frames_) {
        const Vec3d gyro_mean = gyr_acc_sum_ / static_cast<double>(bg_acc_n_);
        NavState s = kf_.GetX();
        s.bg_ = -gyro_mean;
        kf_.ChangeX(s);

        ESKF::CovType P_cur = kf_.GetP();
        const double old_v = std::max(P_cur(15, 15), 1e-12);
        const double scale = std::sqrt(bg_p_target_ / old_v);      // bg_p_target_=1e-6
        P_cur.block<3, 3>(15, 15) = bg_p_target_ * Eigen::Matrix3d::Identity();
        for (int i = 0; i < 23; i++) {                              // 交叉项同步缩，保 PSD
            if (i >= 15 && i < 18) continue;
            P_cur.block<1, 3>(i, 15) *= scale;
            P_cur.block<3, 1>(15, i) *= scale;
        }
        kf_.ChangeP(P_cur);
        bg_calibrated_this_stop_ = true;
        SYS_LOG(syslog::LogSeverity::INFO, true)
            << "[ZUPT] bg calibrated: " << (-gyro_mean).transpose() * 180.0 / M_PI << " deg/s";
    }
```

### 3.5 重力对齐（可选、软、仅长停车）

静止时加计给 roll/pitch 绝对参考。构造"重力对齐、yaw 保持不变"的目标姿态，
复用 GPS 分支的旋转块观测数学（yaw 分量置零）。

```cpp
void LaserOdometryCore::ObsModelGravity(NavState& s, ESKF::CustomObservationModel& obs) {
    obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);
    obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    obs.residual_.resize(3);

    const Vec3d a = acc_acc_sum_ / std::max(1, bg_acc_n_);   // 静止期加计均值(body)
    // 比力指向 -g：标准水平解算（符号按 acc 约定核对一次）
    const double roll_g  = std::atan2( a.y(),  a.z());
    const double pitch_g = std::atan2(-a.x(), std::sqrt(a.y() * a.y() + a.z() * a.z()));

    const Vec3d ypr = SO3ToEulerZYX_Robust(s.rot_);          // (yaw,pitch,roll)
    const SO3 R_target = EulerZYXToSO3(Vec3d(ypr[0], pitch_g, roll_g));  // yaw 不动

    obs.residual_ = (s.rot_.inverse() * R_target).log();
    obs.residual_(2) = 0.0;          // yaw 分量置零
    obs.h_x_(2, 2) = 0.0;
    obs.R_ = std::pow(gravity_noise_deg_ * M_PI / 180.0, 2) * Eigen::Matrix3d::Identity();
    obs.valid_ = true;
}
```

ESKF 接线（枚举/钩子 `ACC_AS_GRAVITY`/`acc_as_gravity_obs_func_` 已存在，只补两处）：

```cpp
// eskf.cpp Update() 分发链：
        } else if (obs == ObsType::ACC_AS_GRAVITY) {
            acc_as_gravity_obs_func_(x_, custom_obs_model_);
        } else if (obs == ObsType::GPS) {

// eskf.cpp 更新分支（与 GPS 分支同构）：
        } else if (obs == ObsType::ACC_AS_GRAVITY) {
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);
            h_x_cur.block(0, 3, 3, 3) = custom_obs_model_.h_x_;
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            if (!dx_current.allFinite() || !P_.allFinite()) return reject();
            x_ = x_.boxplus(dx_current);
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            return true;
        }
```

调用（仅 ≥gravity_align_frames=100 帧即 10s 的长停车，R 要松，软修正防与地图拉锯）：

```cpp
    if (zupt_active_ && bg_calibrated_this_stop_ && stationary_cnt_ >= gravity_align_frames_) {
        kf_.Update(ESKF::ObsType::ACC_AS_GRAVITY, 1.0);
    }
```

---

## 4. NHC（非完整约束，行驶时）

### 4.1 观测模型

轮式车辆车体系不侧滑、不腾空：

```
观测 z = [0, 0]ᵀ，h(x) = (R̂⁻¹·v).tail<2>()   // 车体系 vy, vz
residual = - (R̂⁻¹·v).tail<2>()
H(2×23)：对 vel 块 = (R̂⁻¹) 的后两行；对 rot 块 = ∂(R⁻¹v)/∂δθ ≈ (R̂⁻¹·hat(v)) 的后两行
R = nhc_var·I2（如 0.04，σ=0.2 m/s）
```

严格雅可比：h(x) = E·R̂ᵀv，E=[e_y;e_z]。右扰动 δθ 下 R→R·exp(δ)：
∂h/∂δθ = -E·R̂ᵀ·hat(v)（符号与本 ESKF 的 H 约定核对一次：residual=z-h、H=∂h/∂δ）。
∂h/∂v = E·R̂ᵀ。

### 4.2 观测函数与调用

```cpp
void LaserOdometryCore::ObsModelNHC(NavState& s, ESKF::CustomObservationModel& obs) {
    const Vec3d v_body = s.rot_.inverse() * s.vel_;
    obs.residual_.resize(2);
    obs.residual_(0) = -v_body(1);          // z_y=0
    obs.residual_(1) = -v_body(2);          // z_z=0

    const Mat3d Rt = s.rot_.inverse().matrix();
    // H(2×12) 布局与轮速分支一致：前 3 列=rot，3..5=外参（置零），6..8=vel
    obs.h_x_ = Eigen::MatrixXd::Zero(2, 12);
    // ∂h/∂δθ = -E·Rt·hat(v)  → 取后两行
    const Mat3d dhdtheta = -Rt * SO3::hat(s.vel_);
    obs.h_x_.block<2, 3>(0, 0) = dhdtheta.bottomRows<2>();
    // ∂h/∂v = Rt 后两行 → 映射到状态 12..14（分支代码把 h_x_ 的 6..8 列贴到 vel）
    obs.h_x_.block<2, 3>(0, 6) = Rt.bottomRows<2>();

    obs.R_ = nhc_var_ * Eigen::Matrix2d::Identity();
    obs.valid_ = true;
}
```

调用（行驶中、速度足够大时，轮速更新之后）：

```cpp
    if (!zupt_active_ && use_nhc_ && state_point_.vel_.norm() > nhc_min_speed_) {
        kf_.Update(ESKF::ObsType::NHC, nhc_var_);
    }
```

ESKF 接线：枚举加 `NHC`；分发加 `nhc_obs_func_`；更新分支**不能**直接套 WHEEL
分支（后者写死 3×3 且只贴 vel 块），需一个 2 行变体：

```cpp
        } else if (obs == ObsType::NHC) {
            // 2 观测：h_x_(2×12) 前 3 列→rot(3..5)，6..8 列→vel(12..14)
            Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(2, state_dim_);
            h_x_cur.block(0, 3, 2, 3)  = custom_obs_model_.h_x_.leftCols<3>();
            h_x_cur.block(0, 12, 2, 3) = custom_obs_model_.h_x_.block<2, 3>(0, 6);
            Eigen::MatrixXd K =
                P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + custom_obs_model_.R_).inverse();
            dx_current = K * custom_obs_model_.residual_;
            P_ = P_ - K * h_x_cur * P_;
            if (!dx_current.allFinite() || !P_.allFinite()) return reject();
            x_ = x_.boxplus(dx_current);
            P_ = (0.5 * (P_ + P_.transpose())).eval();
            return true;
        }
```

NHC 的价值：封死"速度方向损坏"模式（实测出现过 v 与前进方向夹角 >83°），
且不依赖轮速可用性（CAN 断流时仍有约束）。

---

## 5. 参数表

| 参数 | 建议值 | 说明 |
|---|---|---|
| wheel_thresh | 0.15 m/s | 静止轮速阈值（覆盖 1km/h 量化） |
| gyr_thresh_deg | 0.5 °/s | 静止陀螺峰值阈值 |
| acc_thresh | 0.15 m/s² | \|‖acc‖-g\| 峰值阈值 |
| stationary_frames | 20 (2s) | 进入静止模式的连续帧数 |
| release_factor | 2.0 | 速度>阈值×2 才解除（滞回） |
| zupt_var | 1e-4 (m/s)² | 零速观测方差 → K≈1 |
| bg_calib_frames | 20 | 标定等待帧数，每次停车一次 |
| bg_p_target | 1e-6 | 标定后 bg 的 P 对角 |
| gravity_align_frames | 100 (10s) | 仅长停车做重力对齐 |
| gravity_noise_deg | 1.0 ° | 重力对齐观测 σ（要松） |
| nhc_var | 0.04 (m/s)² | NHC 观测方差（σ=0.2 m/s） |
| nhc_min_speed | 1.0 m/s | 低速不启用（避免起步瞬态） |

## 6. 调用时序（Run()）

```
SyncPackages → IMU预测/去畸变
→ 静止检测(§2)
→ kf_.Update(LIDAR)
→ ZUPT 或 WHEEL(§3.1)
→ bg 标定(§3.4)              仅长静止、一次
→ 重力对齐(§3.5)             仅 ≥10s 静止、软
→ ConsumeGnssForFrame → GNSS 更新（静止期冻结/基准重置，见姊妹篇）
→ NHC(§4)                    行驶中、v>1m/s
→ MapIncremental（静止/起步跳帧不插）
```

## 7. 风险与验证

1. **atan2 符号约定**：roll_g/pitch_g 取决于 acc 方向约定（get_f 用 `R(acc-ba)+grav`）；
   先打日志对拍（静止平地 roll_g 与当前 roll 应差一恒定值=安装偏置）。
2. **P 一致性**：bg 置值必须同步收缩 P_bg 及交叉项（§3.4 已含）。
3. **坡道蠕动误触发**：三方联合判据是防线；重载车驻车沉降会晃 acc，必要时 acc_thresh 收紧到 0.1。
4. **重力对齐 vs 地图 roll 偏置拉锯**：R 要松（1°²）；若每次停车后 roll 跳 0.4°，
   说明在硬纠恒定偏置——应改做偏置标定补偿，不是加大力度。
5. **验证顺序**：①静止检测+ZUPT → 停车期 vel 精确 0、起步初速不超前轮速；
   ②bg 标定 → 起步 pitch 下探消失；③重力对齐 → 长停车后 roll/pitch 收敛且地图无台阶；
   ④NHC → 速度方向角稳定（v_body 的 y/z 分量≈0）。
