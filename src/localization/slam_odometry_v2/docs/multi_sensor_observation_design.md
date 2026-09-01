# slam_odometry_v2 多传感器观测融合设计文档

## 1. 概述

### 1.1 设计目标

在 slam_odometry_v2 中添加轮速和 GNSS 观测约束，实现：
- 在 LiDAR 观测更新后，分别进行轮速和 GNSS 观测更新
- 使用同一个滤波器（IKFoM ESKF）进行多传感器融合
- 当传感器失效时，自动跳过观测更新

### 1.2 核心方案

**方案3a**：在 IKFoM 库中添加 `update_single_observation` 函数，支持完整 23 维 H 矩阵

```cpp
// LiDAR 观测更新（现有代码，迭代更新）
kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);

// 轮速观测更新（新增，单次更新）
kf_.update_single_observation(H_wheel, residual_wheel, R_wheel);

// GNSS 姿态观测更新（新增，单次更新）
kf_.update_single_observation(H_gnss, residual_gnss, R_gnss);
```

---

## 2. 状态维度布局

### 2.1 state_ikfom 状态定义（23维）

```cpp
MTK_BUILD_MANIFOLD(state_ikfom,
    ((vect3, pos))           // 位置 (3维)      索引 0-2
    ((SO3, rot))             // 姿态 (3维)      索引 3-5
    ((SO3, offset_R_L_I))   // 外参旋转 (3维)  索引 6-8
    ((vect3, offset_T_L_I)) // 外参平移 (3维)  索引 9-11
    ((vect3, vel))           // 速度 (3维)      索引 12-14
    ((vect3, bg))            // 陀螺仪偏置 (3维) 索引 15-17
    ((vect3, ba))            // 加速度计偏置 (3维) 索引 18-20
    ((S2, grav))             // 重力 (2维)      索引 21-22
);
```

### 2.2 各观测对应的 H 矩阵位置

| 观测类型 | H 矩阵维度 | 有效列位置 | 观测的状态分量 |
|----------|-----------|-----------|---------------|
| LiDAR | Nx12 | 前 12 列 | pos, rot, offset_R, offset_T |
| 轮速 | 3x23 | 第 12-14 列 | vel |
| GNSS | 3x23 | 第 3-5 列 | rot |

### 2.3 正确性说明

**为什么不同维度的 H 矩阵可以正确融合？**

1. **状态是统一的 23 维**
2. **协方差 P 是 23x23 矩阵**，包含状态之间的相关性
3. **观测会通过协方差传播**：即使 LiDAR 只观测前 12 维，由于 P 中的相关性，也会间接影响 vel, bg, ba, grav

**参考 lm_slam 的实现**（eskf.cc 第 918-920 行）：
```cpp
kf_.Update(ESKF::ObsType::LIDAR, 1e-3);           // LiDAR 观测
kf_.Update(ESKF::ObsType::WHEEL_SPEED, 0.33*0.33); // 轮速观测
kf_.Update(ESKF::ObsType::GPS, 1e-3);              // GNSS 观测
```

---

## 3. 完整流程

### 3.1 流程图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        完整观测融合流程                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  1. 传感器回调（实时更新健康状态）                                     │
│     ├── onCan(): 更新轮速数据 + 标记健康状态                          │
│     ├── onGnss(): 更新 GNSS 数据 + 标记健康状态                       │
│     └── onLidar(): 更新 LiDAR 数据                                   │
│                                                                      │
│  2. Run() 函数（主循环）                                              │
│     ├── IMU 预测                                                     │
│     ├── LiDAR 观测更新（迭代，12 维 H）                               │
│     ├── 检查轮速健康状态                                              │
│     │   └── 如果健康：轮速观测更新（单次，23 维 H）                    │
│     ├── 检查 GNSS 健康状态                                           │
│     │   └── 如果健康：GNSS 观测更新（单次，23 维 H）                  │
│     └── 输出 odom                                                    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2 传感器回调（实时更新健康状态）

**设计原则**：在传感器回调中立即更新健康状态，Run() 中只检查超时。

#### 3.2.1 轮速回调

```cpp
void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    if (!odom_ || !config_.slam.use_vel) {
        return;
    }

    // 更新数据
    faster_lio::LaserOdometry::CanVelocityData can;
    can.timestamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
                        ? msg->header.stamp.toSec()
                        : ros::Time::now().toSec();
    can.speed_kmh = (msg->gear == 2.0) ? -msg->speed : msg->speed;
    can.valid = msg->valid;
    odom_->FeedCanVelocity(can);

    // 立即更新健康状态（线程安全）
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.has_can_data = true;
        sensor_health_.can_last_update = can.timestamp;
        sensor_health_.can_healthy = true;  // 收到数据即认为健康
    }
}
```

#### 3.2.2 GNSS 回调

```cpp
void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    if (!odom_ || !config_.slam.use_vel || !config_.slam.use_gnss_attitude) {
        return;
    }

    // 更新数据
    faster_lio::LaserOdometry::GnssAttitudeData gnss;
    gnss.timestamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
                         ? msg->header.stamp.toSec()
                         : ros::Time::now().toSec();
    gnss.heading_deg = msg->heading;
    gnss.pitch_deg = msg->pitch;
    gnss.roll_deg = msg->roll;
    gnss.valid = true;
    odom_->FeedGnssAttitude(gnss);

    // 立即更新健康状态（线程安全）
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.has_gnss_data = true;
        sensor_health_.gnss_last_update = gnss.timestamp;
        sensor_health_.gnss_status = msg->status;
        sensor_health_.gnss_nsv = msg->nsv1 + msg->nsv2;
        sensor_health_.gnss_healthy = (msg->status >= min_gnss_status_);  // 根据 status 判断
    }
}
```

### 3.3 Run() 函数（主循环）

```cpp
bool LaserOdometry::Run() {
    if (!SyncPackages()) {
        return false;
    }

    /// IMU 预测
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    /// 更新传感器健康状态（检查超时）
    updateSensorHealth();

    /// LiDAR 观测更新（现有代码，迭代更新）
    kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);

    /// 轮速观测更新（新增，单次更新）
    if (use_vel_ && sensor_health_.can_healthy) {
        applyWheelSpeedObservation();
    }

    /// GNSS 姿态观测更新（新增，单次更新）
    if (use_gnss_attitude_ && sensor_health_.gnss_healthy) {
        applyGnssAttitudeObservation();
    }

    // ... 其余代码 ...
    return true;
}
```

### 3.4 健康状态更新（Run() 中检查超时）

```cpp
void LaserOdometry::updateSensorHealth() {
    std::lock_guard<std::mutex> lock(health_mutex_);
    double current_time = measures_.lidar_bag_time_;

    // 轮速超时检查
    if (sensor_health_.has_can_data) {
        sensor_health_.can_age = current_time - sensor_health_.can_last_update;
        if (sensor_health_.can_age > can_timeout_sec_) {
            if (sensor_health_.can_healthy) {
                LOG(WARNING) << "[Health] wheel speed timeout: age="
                             << sensor_health_.can_age;
            }
            sensor_health_.can_healthy = false;
        }
    }

    // GNSS 超时检查
    if (sensor_health_.has_gnss_data) {
        sensor_health_.gnss_age = current_time - sensor_health_.gnss_last_update;
        if (sensor_health_.gnss_age > gnss_attitude_timeout_sec_) {
            if (sensor_health_.gnss_healthy) {
                LOG(WARNING) << "[Health] GNSS timeout: age="
                             << sensor_health_.gnss_age;
            }
            sensor_health_.gnss_healthy = false;
        }
    }

    // LiDAR 健康检查
    sensor_health_.lidar_points = scan_undistort_ ? scan_undistort_->size() : 0;
    sensor_health_.lidar_healthy = sensor_health_.lidar_points >= min_lidar_points_;
}
```

### 3.5 观测更新实现

#### 3.5.1 轮速观测更新

```cpp
void LaserOdometry::applyWheelSpeedObservation() {
    // 获取当前状态
    state_ikfom state = kf_.get_x();

    // 1. 计算观测残差
    Eigen::Vector3d can_body_vel(last_can_.speed_ms, 0.0, 0.0);
    Eigen::Vector3d can_world_vel = state.rot.toRotationMatrix() * can_body_vel;
    Eigen::Vector3d residual = state.vel - can_world_vel;

    // 2. 构建 H 矩阵（3x23，速度在第 12-14 维）
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);
    H.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();

    // 3. 噪声矩阵
    Eigen::MatrixXd R = wheel_velocity_noise_ * Eigen::Matrix3d::Identity();

    // 4. 单次更新
    kf_.update_single_observation(H, residual, R);

    LOG_EVERY_N(100, INFO) << "[Obs] wheel speed update: residual_norm=" << residual.norm();
}
```

#### 3.5.2 GNSS 姿态观测更新

```cpp
void LaserOdometry::applyGnssAttitudeObservation() {
    // 获取当前状态
    state_ikfom state = kf_.get_x();

    // 1. 计算观测残差（李代数空间）
    Eigen::Quaterniond q_obs = EulerZYXToQuaternion(
        last_gnss_.heading_deg, last_gnss_.pitch_deg, last_gnss_.roll_deg);
    SO3 rot_obs(q_obs);
    Eigen::Vector3d residual = (state.rot.inverse() * rot_obs).log();

    // 2. 构建 H 矩阵（3x23，姿态在第 3-5 维）
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    // 3. 噪声矩阵
    Eigen::MatrixXd R = gnss_attitude_noise_ * Eigen::Matrix3d::Identity();

    // 4. 单次更新
    kf_.update_single_observation(H, residual, R);

    LOG_EVERY_N(100, INFO) << "[Obs] GNSS attitude update: residual_norm=" << residual.norm()
                           << " status=" << last_gnss_.status << " nsv=" << last_gnss_.nsv;
}
```

### 3.6 IKFoM 库修改（添加 update_single_observation 函数）

```cpp
// esekfom.hpp 中添加新函数
void update_single_observation(Eigen::MatrixXd H, Eigen::VectorXd residual, Eigen::MatrixXd R) {
    // 构建完整的 H 矩阵
    Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(H.rows(), n);  // n = 23
    h_x_cur = H;

    // 计算卡尔曼增益
    Eigen::MatrixXd K = P_ * h_x_cur.transpose() * 
                        (h_x_cur * P_ * h_x_cur.transpose() + R).inverse();

    // 更新状态
    Eigen::VectorXd dx = K * residual;
    x_ = x_.boxplus(dx);

    // 更新协方差
    P_ = P_ - K * h_x_cur * P_;
}
```

---

## 4. 传感器健康判断

### 4.1 轮速数据判断

**不使用 valid 字段，直接通过有无数据判断**：

```cpp
bool isCanVelocityValid() const {
    std::lock_guard<std::mutex> lock(health_mutex_);
    if (!sensor_health_.has_can_data) return false;
    return sensor_health_.can_healthy;
}
```

### 4.2 GNSS 数据判断

**参考 slam_mapping_v2，使用 status 和 nsv 判断质量**：

```cpp
// 回调中立即判断
sensor_health_.gnss_healthy = (msg->status >= min_gnss_status_);

// min_gnss_status 默认值：33（RTK 固定解）
```

### 4.3 LiDAR 数据判断

```cpp
bool isLidarObservationValid() const {
    return scan_undistort_ && scan_undistort_->size() >= min_lidar_points_;
}
```

---

## 5. 配置参数

```yaml
# slam_odometry_v2.yaml

vel_gnss:
  # 开关
  use_vel: true
  use_gnss_attitude: true

  # 轮速观测参数
  wheel_velocity_noise: 0.25        # 轮速观测噪声 (m/s)
  can_timeout_sec: 0.5              # 轮速超时阈值 (秒)

  # GNSS 观测参数
  gnss_attitude_noise_deg: 2.0      # GNSS 姿态观测噪声 (度)
  gnss_attitude_timeout_sec: 1.0    # GNSS 超时阈值 (秒)
  min_gnss_status: 33               # 最小 GNSS 状态（RTK 固定解）

  # LiDAR 观测参数
  min_lidar_points: 100             # 最小点云数量
```

---

## 6. 健康状态结构

```cpp
struct SensorHealth {
    // 轮速
    bool can_healthy = false;
    double can_age = 1e9;
    bool has_can_data = false;
    double can_last_update = 0.0;

    // GNSS
    bool gnss_healthy = false;
    double gnss_age = 1e9;
    bool has_gnss_data = false;
    int gnss_status = 0;
    int gnss_nsv = 0;
    double gnss_last_update = 0.0;

    // LiDAR
    bool lidar_healthy = false;
    size_t lidar_points = 0;

    // 转为字符串（用于日志）
    std::string toString() const;
};
```

---

## 7. 日志设计

### 7.1 传感器状态日志

```cpp
// 启动时打印配置
ROS_INFO("[slam_odometry_v2] observation params: wheel_noise=%.2f m/s, gnss_noise=%.1f deg, "
         "min_gnss_status=%d, min_lidar_points=%d",
         config_.slam.wheel_velocity_noise,
         config_.slam.gnss_attitude_noise_deg,
         config_.slam.min_gnss_status,
         config_.slam.min_lidar_points);

// 定期打印传感器健康状态
LOG_EVERY_N(100, INFO) << "[Health] " << sensor_health_.toString();
```

### 7.2 观测更新日志

```cpp
// 轮速观测更新
LOG_EVERY_N(100, INFO) << "[Obs] wheel speed: residual_norm=" << residual.norm()
                       << " age=" << sensor_health_.can_age;

// GNSS 观测更新
LOG_EVERY_N(100, INFO) << "[Obs] GNSS attitude: residual_norm=" << residual.norm()
                       << " status=" << sensor_health_.gnss_status
                       << " nsv=" << sensor_health_.gnss_nsv;

// 传感器失效警告
LOG_IF(WARNING, !sensor_health_.can_healthy && sensor_health_.has_can_data)
    << "[Health] wheel speed unhealthy: age=" << sensor_health_.can_age;

LOG_IF(WARNING, !sensor_health_.gnss_healthy && sensor_health_.has_gnss_data)
    << "[Health] GNSS unhealthy: age=" << sensor_health_.gnss_age
    << " status=" << sensor_health_.gnss_status;
```

---

## 8. 线程安全

### 8.1 问题

- 传感器回调在 ROS 回调线程中执行
- Run() 在主线程中执行
- 需要保护共享数据（SensorHealth）

### 8.2 解决方案

```cpp
class LaserOdometry {
private:
    mutable std::mutex health_mutex_;  // 健康状态互斥锁
    SensorHealth sensor_health_;

public:
    // 回调中更新（加锁）
    void onCan(...) {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.can_healthy = true;
    }

    // Run() 中检查超时（加锁）
    void updateSensorHealth() {
        std::lock_guard<std::mutex> lock(health_mutex_);
        // 检查超时...
    }

    // 获取健康状态（加锁）
    SensorHealth getSensorHealth() const {
        std::lock_guard<std::mutex> lock(health_mutex_);
        return sensor_health_;
    }
};
```

---

## 9. 传感器失效处理

### 9.1 失效场景

| 失效传感器 | 可用传感器 | 处理方式 |
|-----------|-----------|---------|
| 轮速 | LiDAR + IMU + GNSS | 跳过轮速观测更新 |
| GNSS | LiDAR + IMU + 轮速 | 跳过 GNSS 观测更新 |
| LiDAR | IMU + 轮速 + GNSS | 跳过 LiDAR 观测更新 |

### 9.2 自动降权

**不需要显式切换**，滤波器自动处理：
- 传感器失效时，跳过观测更新
- 协方差矩阵 P 会自然增大（不确定性增加）
- 其他传感器的观测会继续更新状态

---

## 10. 实现步骤

### 10.1 第一步：修改 IKFoM 库

在 `esekfom.hpp` 中添加 `update_single_observation` 函数。

### 10.2 第二步：添加健康状态结构

在 `laser_odometry.h` 中添加 `SensorHealth` 结构和成员变量。

### 10.3 第三步：修改传感器回调

在 `slam_odometry_v2_node.cc` 的回调函数中更新健康状态。

### 10.4 第四步：添加观测更新函数

在 `laser_odometry.cc` 中添加 `applyWheelSpeedObservation` 和 `applyGnssAttitudeObservation` 函数。

### 10.5 第五步：修改 Run() 函数

在 LiDAR 观测更新后，调用轮速和 GNSS 观测更新。

### 10.6 第六步：添加配置参数

在配置文件和加载函数中添加新参数。

---

## 附录 A：参考实现

### lm_slam 的观测更新（eskf.cc）

```cpp
// LiDAR 观测更新（迭代）
kf_.Update(ESKF::ObsType::LIDAR, 1e-3);

// 轮速观测更新（单次）
kf_.Update(ESKF::ObsType::WHEEL_SPEED, 0.33*0.33);

// GNSS 观测更新（单次）
kf_.Update(ESKF::ObsType::GPS, 1e-3);
```

### lm_slam 的轮速观测模型

```cpp
// laser_mapping.cc 第 1374-1388 行
Vec3d vel_odom(wheel_speed_obs, 0.0, 0.0);
auto R_wl = s.rot_;
Vec3d vel_world = R_wl * vel_odom;
obs.residual_ = s.vel_ - vel_world;
```

### lm_slam 的 GNSS 观测模型

```cpp
// laser_mapping.cc 第 1339-1342 行
auto RPR = Vec3d(hc_heading, hc_pitch, hc_roll);
SO3 gps_R = EulerZYXToSO3(RPR);
obs.residual_ = (s.rot_.inverse() * gps_R).log();
```
