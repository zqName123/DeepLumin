# slam_odometry_v2 多传感器观测融合设计文档

## 1. 概述

### 1.1 文档目的

本文档详细描述在 slam_odometry_v2 中添加轮速和 GNSS 观测约束的设计方案，实现：
- 将轮速和 GNSS 从"硬赋值"改为标准的观测更新
- 当传感器失效时，自动降低权重或跳过观测
- 保持代码结构清晰，尽量解耦

### 1.2 当前问题

**当前实现方式**（硬赋值）：
```cpp
// 轮速校正：直接重置速度
state.vel(0) = can_world_vel_kmh(0) / 3.6;
kf_.change_x(state);  // 直接修改状态

// GNSS 校正：直接重置姿态
state.rot = SO3(q_modified.w(), q_modified.x(), q_modified.y(), q_modified.z());
kf_.change_x(state);  // 直接修改状态
```

**问题**：
- 没有考虑观测噪声，权重无法控制
- 没有传感器失效检测，可能使用旧数据
- 硬赋值可能导致状态跳变

### 1.3 设计目标

1. **标准观测更新**：使用卡尔曼滤波的标准观测模型
2. **自动权重调整**：根据传感器健康状态自动调整观测噪声
3. **传感器失效处理**：当数据无效或超时时，自动跳过观测
4. **代码解耦**：观测模型独立封装，易于测试和维护

---

## 2. 传感器数据判断策略

### 2.1 轮速数据判断

**设计原则**：不使用 `valid` 字段，直接通过有无数据判断。

**判断条件**：
```cpp
bool isCanVelocityValid(double current_time) const {
    // 1. 是否有数据
    if (!has_can_) return false;

    // 2. 时间新鲜度
    double age = std::abs(current_time - last_can_.timestamp);
    if (age > can_timeout_sec_) return false;

    return true;
}
```

**说明**：
- `has_can_`：是否收到过轮速数据
- `can_timeout_sec_`：轮速数据超时阈值（默认 0.5 秒）
- 不依赖 `valid` 字段，因为该字段可能无效

### 2.2 GNSS 数据判断

**参考 slam_mapping_v2 的实现**，使用 `status` 和 `nsv` 判断质量。

**判断条件**：
```cpp
bool isGnssAttitudeValid(double current_time) const {
    // 1. 是否有数据
    if (!has_gnss_attitude_) return false;

    // 2. 时间新鲜度
    double age = std::abs(current_time - last_gnss_attitude_.timestamp);
    if (age > gnss_attitude_timeout_sec_) return false;

    // 3. GNSS 质量判断（参考 slam_mapping_v2）
    if (last_gnss_attitude_.status < min_gnss_status_) return false;
    if (min_gnss_nsv_ > 0 && last_gnss_attitude_.nsv < min_gnss_nsv_) return false;

    return true;
}
```

**GNSS 质量指标**：
| 字段 | 含义 | 建议阈值 |
|------|------|----------|
| `status` | GNSS 状态（如 RTK 固定解=33） | >= 33 |
| `nsv` | 可见卫星数量 | >= 8 |
| `timestamp` | 数据时间 | 超时 < 1.0 秒 |

### 2.3 LiDAR 数据判断

**判断条件**：
```cpp
bool isLidarObservationValid() const {
    // 1. 点云数量
    if (!scan_undistort_ || scan_undistort_->size() < min_lidar_points_) return false;

    // 2. 配准质量（在观测模型中判断）
    return true;
}
```

---

## 3. 观测模型设计

### 3.1 状态维度布局（重要）

**state_ikfom 状态定义**（23维）：

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

**状态索引常量**：
```cpp
constexpr int POS_IDX = 0;      // 位置
constexpr int ROT_IDX = 3;      // 姿态
constexpr int OFFSET_R_IDX = 6; // 外参旋转
constexpr int OFFSET_T_IDX = 9; // 外参平移
constexpr int VEL_IDX = 12;     // 速度
constexpr int BG_IDX = 15;      // 陀螺仪偏置
constexpr int BA_IDX = 18;      // 加速度计偏置
constexpr int GRAV_IDX = 21;    // 重力
```

**验证来源**：lm_slam 的 ESKF Update 函数中：
- 轮速观测：`h_x_cur.block(0, 12, 3, 3) = custom_obs_model_.h_x_` → 速度在第 12-14 维
- GNSS 姿态观测：`h_x_cur.block(0, 3, 3, 3) = custom_obs_model_.h_x_` → 姿态在第 3-5 维

### 3.2 轮速观测模型

**观测维度**：3（vx, vy, vz）

**观测方程**：
```
z = H * x + v
其中：
z = [vx_wheel, vy_wheel, vz_wheel]  (轮速观测)
H = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, ...]  (速度在第 12-14 维)
x = [pos, rot, offset_R, offset_T, vel, bg, ba, grav]  (23维状态)
v ~ N(0, R)  (观测噪声)
```

**H 矩阵构建**：
```cpp
// H 矩阵：3 x 23
Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);

// 速度在状态的第 12-14 维
H.block<3, 3>(0, VEL_IDX) = Eigen::Matrix3d::Identity();  // 索引 12-14
```

**残差计算**：
```cpp
// 轮速观测：车体系 x 方向速度
Eigen::Vector3d can_body_vel(last_can_.speed_ms, 0.0, 0.0);

// 转换到世界系（考虑外参）
// 注意：lm_slam 中没有使用外参，直接用 rot
Eigen::Vector3d can_world_vel = state.rot.toRotationMatrix() * can_body_vel;

// 残差 = 状态速度 - 观测速度
residual = state.vel - can_world_vel;
```

**观测噪声**：
```cpp
// 基础噪声
double R_base = wheel_velocity_noise_;  // 默认 0.25 m/s

// 根据数据新鲜度调整
double age_factor = 1.0 + (age / can_timeout_sec_);
R = R_base * age_factor * Eigen::Matrix3d::Identity();
```

### 3.3 GNSS 姿态观测模型

**观测维度**：3（roll, pitch, yaw）

**观测方程**：
```
z = H * x + v
其中：
z = [roll_gnss, pitch_gnss, yaw_gnss]  (GNSS 姿态观测)
H = [0, 0, 0, 1, 0, 0, 0, 0, 0, ...]  (姿态在第 3-5 维)
x = [pos, rot, offset_R, offset_T, vel, bg, ba, grav]  (23维状态)
v ~ N(0, R)  (观测噪声)
```

**H 矩阵构建**：
```cpp
// H 矩阵：3 x 23
Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 23);

// 姿态在状态的第 3-5 维
H.block<3, 3>(0, ROT_IDX) = Eigen::Matrix3d::Identity();  // 索引 3-5
```

**残差计算**（李代数空间）：
```cpp
// GNSS 观测姿态
Eigen::Quaterniond q_obs = EulerZYXToQuaternion(
    gnss_attitude_.heading_deg,
    gnss_attitude_.pitch_deg,
    gnss_attitude_.roll_deg);
SO3 rot_obs(q_obs);

// 残差 = log(R_obs * R_state^{-1})
// 注意：lm_slam 中使用 s.rot_.inverse() * gps_R
residual = (state.rot.inverse() * rot_obs).log();
```

**观测噪声**：
```cpp
// 基础噪声
double R_base = gnss_attitude_noise_deg_ * M_PI / 180.0;  // 转为弧度

// 根据 GNSS 质量调整
double quality_factor = 1.0;
if (gnss_attitude_.status < min_gnss_status_) {
    quality_factor = 10.0;  // 低质量 GNSS，增大噪声
}
R = R_base * quality_factor * Eigen::Matrix3d::Identity();
```

### 3.3 LiDAR 观测模型

**保持现有实现**（点云配准），不修改。

---

## 4. 传感器健康状态

### 4.1 健康状态结构

```cpp
struct SensorHealth {
    // 轮速
    bool can_healthy = false;
    double can_age = 1e9;
    bool has_can_data = false;

    // GNSS
    bool gnss_healthy = false;
    double gnss_age = 1e9;
    bool has_gnss_data = false;
    int gnss_status = 0;
    int gnss_nsv = 0;

    // LiDAR
    bool lidar_healthy = false;
    size_t lidar_points = 0;
};
```

### 4.2 健康状态更新

**设计原则**：在传感器回调中立即更新健康状态，而不是在 Run() 中延迟检测。

**优势**：
- 实时性好：传感器失效时立即发现
- 准确性高：在数据到达时立即判断
- 响应快：恢复时也能立即发现

#### 4.2.1 回调中更新（推荐）

```cpp
// 在传感器回调中立即更新健康状态
void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg) {
    if (!odom_ || !config_.slam.use_vel) {
        return;
    }

    // 更新数据
    last_can_ = convert(msg);
    has_can_ = true;

    // 立即更新健康状态（线程安全）
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.has_can_data = true;
        sensor_health_.can_last_update = ros::Time::now().toSec();
        sensor_health_.can_healthy = true;  // 收到数据即认为健康
    }
}

void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    if (!odom_ || !config_.slam.use_vel || !config_.slam.use_gnss_attitude) {
        return;
    }

    // 更新数据
    last_gnss_ = convert(msg);
    has_gnss_ = true;

    // 立即更新健康状态（线程安全）
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.has_gnss_data = true;
        sensor_health_.gnss_last_update = ros::Time::now().toSec();
        sensor_health_.gnss_status = msg->status;
        sensor_health_.gnss_nsv = msg->nsv1 + msg->nsv2;
        sensor_health_.gnss_healthy = (msg->status >= min_gnss_status_);
    }
}
```

#### 4.2.2 Run() 中超时检查

```cpp
void LaserOdometry::updateSensorHealth() {
    std::lock_guard<std::mutex> lock(health_mutex_);
    double current_time = measures_.lidar_bag_time_;

    // 轮速超时检查
    if (sensor_health_.has_can_data) {
        sensor_health_.can_age = current_time - sensor_health_.can_last_update;
        if (sensor_health_.can_age > can_timeout_sec_) {
            sensor_health_.can_healthy = false;
            LOG_FIRST_N(WARNING, 1) << "[Health] wheel speed timeout: age="
                                    << sensor_health_.can_age;
        }
    }

    // GNSS 超时检查
    if (sensor_health_.has_gnss_data) {
        sensor_health_.gnss_age = current_time - sensor_health_.gnss_last_update;
        if (sensor_health_.gnss_age > gnss_attitude_timeout_sec_) {
            sensor_health_.gnss_healthy = false;
            LOG_FIRST_N(WARNING, 1) << "[Health] GNSS timeout: age="
                                    << sensor_health_.gnss_age;
        }
    }

    // LiDAR 健康检查（在 Run() 中，因为需要扫描数据）
    sensor_health_.lidar_points = scan_undistort_ ? scan_undistort_->size() : 0;
    sensor_health_.lidar_healthy = sensor_health_.lidar_points >= min_lidar_points_;
}
```

#### 4.2.3 健康状态结构（更新）

```cpp
struct SensorHealth {
    // 轮速
    bool can_healthy = false;
    double can_age = 1e9;
    bool has_can_data = false;
    double can_last_update = 0.0;    // 新增：最后更新时间

    // GNSS
    bool gnss_healthy = false;
    double gnss_age = 1e9;
    bool has_gnss_data = false;
    int gnss_status = 0;
    int gnss_nsv = 0;
    double gnss_last_update = 0.0;   // 新增：最后更新时间

    // LiDAR
    bool lidar_healthy = false;
    size_t lidar_points = 0;

    // 转为字符串（用于日志）
    std::string toString() const;
};
```

#### 4.2.4 线程安全设计

```cpp
class LaserOdometry {
private:
    // ... 其他成员 ...

    // 健康状态互斥锁
    mutable std::mutex health_mutex_;

    // 传感器健康状态
    SensorHealth sensor_health_;

    // 获取健康状态（线程安全）
    SensorHealth getSensorHealth() const {
        std::lock_guard<std::mutex> lock(health_mutex_);
        return sensor_health_;
    }
};
```

---

## 5. 代码结构设计

### 5.1 新增文件

```
slam_odometry_v2/
├── include/faster_lio/
│   ├── observation_model.hpp       # 观测模型定义
│   └── sensor_health.hpp           # 传感器健康状态
├── src/faster_lio/
│   ├── observation_model.cc        # 观测模型实现
│   └── laser_odometry.cc           # 修改：集成观测模型
```

### 5.2 类设计

#### 5.2.1 观测模型基类

```cpp
// observation_model.hpp
namespace faster_lio {

// 观测模型基类
class ObservationModel {
public:
    virtual ~ObservationModel() = default;

    // 计算观测矩阵、残差、噪声
    virtual bool compute(const state_ikfom& state,
                        Eigen::MatrixXd& H,
                        Eigen::VectorXd& residual,
                        Eigen::MatrixXd& R) = 0;

    // 观测是否有效
    virtual bool isValid() const = 0;

    // 观测维度
    virtual int dimension() const = 0;

    // 观测名称（用于日志）
    virtual std::string name() const = 0;
};

}  // namespace faster_lio
```

#### 5.2.2 轮速观测模型

```cpp
// observation_model.hpp
namespace faster_lio {

class WheelSpeedObservation : public ObservationModel {
public:
    WheelSpeedObservation(double noise_ms, double timeout_sec);

    bool compute(const state_ikfom& state,
                Eigen::MatrixXd& H,
                Eigen::VectorXd& residual,
                Eigen::MatrixXd& R) override;

    bool isValid() const override;
    int dimension() const override { return 3; }
    std::string name() const override { return "wheel_speed"; }

    // 更新数据
    void update(const CanVelocityData& can, double current_time);

    // 获取健康状态
    bool isHealthy() const { return has_data_ && age() < timeout_; }
    double age() const { return has_data_ ? std::abs(current_time_ - last_can_.timestamp) : 1e9; }

private:
    CanVelocityData last_can_;
    double current_time_ = 0.0;
    bool has_data_ = false;
    double noise_ms_;       // 观测噪声 (m/s)
    double timeout_sec_;    // 超时阈值 (秒)
};

}  // namespace faster_lio
```

#### 5.2.3 GNSS 姿态观测模型

```cpp
// observation_model.hpp
namespace faster_lio {

class GnssAttitudeObservation : public ObservationModel {
public:
    GnssAttitudeObservation(double noise_deg, double timeout_sec,
                           int min_status, int min_nsv);

    bool compute(const state_ikfom& state,
                Eigen::MatrixXd& H,
                Eigen::VectorXd& residual,
                Eigen::MatrixXd& R) override;

    bool isValid() const override;
    int dimension() const override { return 3; }
    std::string name() const override { return "gnss_attitude"; }

    // 更新数据
    void update(const GnssAttitudeData& gnss, double current_time);

    // 获取健康状态
    bool isHealthy() const;
    double age() const { return has_data_ ? std::abs(current_time_ - last_gnss_.timestamp) : 1e9; }
    int status() const { return last_gnss_.status; }
    int nsv() const { return last_gnss_.nsv; }

private:
    GnssAttitudeData last_gnss_;
    double current_time_ = 0.0;
    bool has_data_ = false;
    double noise_deg_;      // 观测噪声 (度)
    double timeout_sec_;    // 超时阈值 (秒)
    int min_status_;        // 最小 GNSS 状态
    int min_nsv_;           // 最小卫星数量
};

}  // namespace faster_lio
```

#### 5.2.4 传感器健康状态

```cpp
// sensor_health.hpp
namespace faster_lio {

struct SensorHealth {
    // 轮速
    bool can_healthy = false;
    double can_age = 1e9;
    bool has_can_data = false;

    // GNSS
    bool gnss_healthy = false;
    double gnss_age = 1e9;
    bool has_gnss_data = false;
    int gnss_status = 0;
    int gnss_nsv = 0;

    // LiDAR
    bool lidar_healthy = false;
    size_t lidar_points = 0;

    // 转为字符串（用于日志）
    std::string toString() const;
};

}  // namespace faster_lio
```

### 5.3 LaserOdometry 修改

```cpp
// laser_odometry.h
class LaserOdometry {
public:
    // ... 现有接口 ...

    // 新增：获取传感器健康状态（线程安全）
    SensorHealth getSensorHealth() const {
        std::lock_guard<std::mutex> lock(health_mutex_);
        return sensor_health_;
    }

    // 新增：设置观测参数
    void SetObservationParams(double wheel_noise_ms, double gnss_noise_deg,
                              int min_gnss_status, int min_gnss_nsv,
                              int min_lidar_points);

private:
    // ... 现有成员 ...

    // 新增：观测模型
    std::unique_ptr<WheelSpeedObservation> wheel_obs_;
    std::unique_ptr<GnssAttitudeObservation> gnss_obs_;

    // 新增：传感器健康状态（线程安全）
    mutable std::mutex health_mutex_;
    SensorHealth sensor_health_;

    // 新增：应用观测更新
    void applyObservationUpdate();
    void updateSensorHealth();  // Run() 中调用，检查超时

    // 新增：最小点云数量
    int min_lidar_points_ = 100;
};
```

---

## 6. 实现流程

### 6.1 回调中更新健康状态（推荐）

**设计原则**：在传感器回调中立即更新健康状态，Run() 中只检查超时。

```cpp
// 轮速回调
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

    // 立即更新健康状态
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.has_can_data = true;
        sensor_health_.can_last_update = can.timestamp;
        sensor_health_.can_healthy = true;  // 收到数据即认为健康
    }
}

// GNSS 回调
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

    // 立即更新健康状态
    {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.has_gnss_data = true;
        sensor_health_.gnss_last_update = gnss.timestamp;
        sensor_health_.gnss_status = msg->status;
        sensor_health_.gnss_nsv = msg->nsv1 + msg->nsv2;
        sensor_health_.gnss_healthy = (msg->status >= min_gnss_status_);
    }
}
```

### 6.2 Run() 函数修改

```cpp
bool LaserOdometry::Run() {
    if (!SyncPackages()) {
        return false;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    /// 更新传感器健康状态（检查超时）
    updateSensorHealth();

    /// 轮速观测更新（如果数据有效）
    if (use_vel_ && wheel_obs_ && wheel_obs_->isValid()) {
        applyWheelSpeedObservation();
    }

    /// GNSS 姿态观测更新（如果数据有效）
    if (use_gnss_attitude_ && gnss_obs_ && gnss_obs_->isValid()) {
        applyGnssAttitudeObservation();
    }

    /// the first scan
    if (flg_first_scan_) {
        // ... 现有代码 ...
    }

    /// downsample
    // ... 现有代码 ...

    /// LiDAR 观测更新（标准流程）
    // ... 现有代码 ...

    return true;
}
```

### 6.3 Run() 中超时检查

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

    // LiDAR 健康检查（在 Run() 中，因为需要扫描数据）
    sensor_health_.lidar_points = scan_undistort_ ? scan_undistort_->size() : 0;
    sensor_health_.lidar_healthy = sensor_health_.lidar_points >= min_lidar_points_;
}
```

### 6.2 观测更新实现

```cpp
void LaserOdometry::applyWheelSpeedObservation() {
    state_ikfom state = kf_.get_x();
    Eigen::MatrixXd H;
    Eigen::VectorXd residual;
    Eigen::MatrixXd R;

    if (wheel_obs_->compute(state, H, residual, R)) {
        // 使用 ESKF 的观测更新
        kf_.update_H_residual_R(H, residual, R);

        LOG_EVERY_N(100, INFO) << "[Obs] wheel speed update: "
                               << "residual=" << residual.transpose()
                               << " age=" << wheel_obs_->age();
    }
}

void LaserOdometry::applyGnssAttitudeObservation() {
    state_ikfom state = kf_.get_x();
    Eigen::MatrixXd H;
    Eigen::VectorXd residual;
    Eigen::MatrixXd R;

    if (gnss_obs_->compute(state, H, residual, R)) {
        // 使用 ESKF 的观测更新
        kf_.update_H_residual_R(H, residual, R);

        LOG_EVERY_N(100, INFO) << "[Obs] GNSS attitude update: "
                               << "residual=" << residual.transpose()
                               << " status=" << gnss_obs_->status()
                               << " nsv=" << gnss_obs_->nsv();
    }
}
```

---

## 7. 配置参数设计

### 7.1 YAML 配置

```yaml
# slam_odometry_v2.yaml

# 轮速/GNSS 观测参数
vel_gnss:
  # 开关
  use_vel: true
  use_gnss_attitude: true

  # 轮速观测参数
  wheel_velocity_noise_ms: 0.25     # 轮速观测噪声 (m/s)
  can_timeout_sec: 0.5              # 轮速超时阈值 (秒)

  # GNSS 观测参数
  gnss_attitude_noise_deg: 2.0      # GNSS 姿态观测噪声 (度)
  gnss_attitude_timeout_sec: 1.0    # GNSS 超时阈值 (秒)
  min_gnss_status: 33               # 最小 GNSS 状态（RTK 固定解）
  min_gnss_nsv: 8                   # 最小卫星数量

  # LiDAR 观测参数
  min_lidar_points: 100             # 最小点云数量
```

### 7.2 配置加载

```cpp
// slam_odometry_v2_config.hpp
struct SlamConfig {
    // ... 现有配置 ...

    // 观测参数
    double wheel_velocity_noise_ms = 0.25;
    double gnss_attitude_noise_deg = 2.0;
    int min_gnss_status = 33;
    int min_gnss_nsv = 8;
    int min_lidar_points = 100;
};
```

---

## 8. 日志设计

### 8.1 传感器状态日志

```cpp
// 启动时打印配置
ROS_INFO("[slam_odometry_v2] observation params: wheel_noise=%.2f m/s, gnss_noise=%.1f deg, "
         "min_gnss_status=%d, min_gnss_nsv=%d, min_lidar_points=%d",
         config_.slam.wheel_velocity_noise_ms,
         config_.slam.gnss_attitude_noise_deg,
         config_.slam.min_gnss_status,
         config_.slam.min_gnss_nsv,
         config_.slam.min_lidar_points);

// 定期打印传感器健康状态
LOG_EVERY_N(100, INFO) << "[Health] " << sensor_health_.toString();
```

### 8.2 观测更新日志

```cpp
// 轮速观测更新
LOG_EVERY_N(100, INFO) << "[Obs] wheel speed: residual_norm=" << residual.norm()
                       << " age=" << wheel_obs_->age()
                       << " healthy=" << wheel_obs_->isHealthy();

// GNSS 观测更新
LOG_EVERY_N(100, INFO) << "[Obs] GNSS attitude: residual_norm=" << residual.norm()
                       << " status=" << gnss_obs_->status()
                       << " nsv=" << gnss_obs_->nsv()
                       << " healthy=" << gnss_obs_->isHealthy();

// 传感器失效警告
LOG_IF(WARNING, !sensor_health_.can_healthy && sensor_health_.has_can_data)
    << "[Health] wheel speed unhealthy: age=" << sensor_health_.can_age;

LOG_IF(WARNING, !sensor_health_.gnss_healthy && sensor_health_.has_gnss_data)
    << "[Health] GNSS unhealthy: age=" << sensor_health_.gnss_age
    << " status=" << sensor_health_.gnss_status;
```

---

## 9. 传感器失效处理流程

### 9.1 轮速失效

```
正常情况：
  轮速数据有效 → 应用观测更新 → 速度约束

轮速超时：
  轮速数据超时 → 跳过观测更新 → 仅使用 LiDAR + IMU

轮速恢复：
  轮速数据恢复 → 重新应用观测更新 → 速度约束
```

### 9.2 GNSS 失效

```
正常情况：
  GNSS 数据有效且质量好 → 应用观测更新 → 姿态约束

GNSS 超时或质量差：
  GNSS 数据超时或 status < 33 → 跳过观测更新 → 仅使用 LiDAR + IMU + 轮速

GNSS 恢复：
  GNSS 数据恢复且质量好 → 重新应用观测更新 → 姿态约束
```

### 9.3 LiDAR 失效

```
正常情况：
  LiDAR 点云数量足够 → 应用观测更新 → 位姿约束

LiDAR 点云过少：
  LiDAR 点云数量 < 阈值 → 跳过观测更新 → 仅使用 IMU + 轮速 + GNSS（短期可用）

LiDAR 恢复：
  LiDAR 点云数量恢复 → 重新应用观测更新 → 位姿约束
```

---

## 10. 预期效果

### 10.1 传感器失效时的定位质量

| 失效传感器 | 可用传感器 | 定位质量 | 持续时间 |
|-----------|-----------|---------|---------|
| 无 | LiDAR + IMU + 轮速 + GNSS | 高 | 正常 |
| 轮速 | LiDAR + IMU + GNSS | 中高 | 长期 |
| GNSS | LiDAR + IMU + 轮速 | 中高 | 长期 |
| LiDAR | IMU + 轮速 + GNSS | 中 | 短期（漂移累积） |
| 轮速 + GNSS | LiDAR + IMU | 中 | 长期 |
| LiDAR + 轮速 | IMU + GNSS | 低 | 短期 |
| LiDAR + GNSS | IMU + 轮速 | 低 | 短期 |
| 全部 | 无 | 无 | 立即失效 |

### 10.2 连续性保证

- **没有显式切换**：只有权重变化，没有状态跳变
- **自动降权**：传感器失效时，观测噪声增大，权重降低
- **自动恢复**：传感器恢复时，观测噪声恢复正常，权重恢复

### 10.3 下游感知

通过 `SensorHealth` 结构，下游可以了解：
- 当前哪些传感器可用
- 传感器数据的新鲜度
- GNSS 的质量状态

---

## 11. 实现优先级

### 11.1 第一阶段：基础观测模型

1. 实现轮速观测模型（替换硬赋值）
2. 实现 GNSS 姿态观测模型（替换硬赋值）
3. 添加数据有效性判断

### 11.2 第二阶段：传感器健康检测

1. 实现 SensorHealth 结构
2. 添加超时检测
3. 根据健康状态调整观测噪声

### 11.3 第三阶段：状态发布增强

1. 在 odom 消息中添加传感器健康状态
2. 添加调试话题

---

## 12. 注意事项

### 12.1 线程安全

**问题**：传感器回调在 ROS 回调线程中执行，Run() 在主线程中执行，需要保护共享数据。

**解决方案**：
- 使用 `std::mutex` 保护 `SensorHealth` 结构
- 在回调中立即更新健康状态（加锁）
- 在 Run() 中检查超时（加锁）
- 获取健康状态时加锁

**代码示例**：
```cpp
class LaserOdometry {
private:
    mutable std::mutex health_mutex_;  // 健康状态互斥锁
    SensorHealth sensor_health_;

public:
    // 回调中更新（加锁）
    void onCan(...) {
        std::lock_guard<std::mutex> lock(health_mutex_);
        sensor_health_.has_can_data = true;
        sensor_health_.can_last_update = timestamp;
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

### 12.2 数值稳定性

- 残差计算需要处理奇异情况
- 观测噪声不能太小，避免滤波器发散

### 12.3 调参建议

| 参数 | 建议值 | 说明 |
|------|--------|------|
| `wheel_velocity_noise_ms` | 0.25 | 轮速噪声，根据轮速精度调整 |
| `gnss_attitude_noise_deg` | 2.0 | GNSS 姿态噪声，根据 GNSS 精度调整 |
| `can_timeout_sec` | 0.5 | 轮速超时，根据轮速频率调整 |
| `gnss_attitude_timeout_sec` | 1.0 | GNSS 超时，根据 GNSS 频率调整 |
| `min_gnss_status` | 33 | RTK 固定解 |
| `min_gnss_nsv` | 8 | 最小卫星数量 |
| `min_lidar_points` | 100 | 最小点云数量 |

---

## 附录 A：参考实现

### slam_mapping_v2 的 GNSS 判断

```cpp
// gnss_optimizer.cpp
bool GnssOptimizer::isReliable(const Keyframe& kf) const {
    const auto& g = kf.gnss;
    if (g.status < cfg_.min_status) return false;  // 状态检查
    if (cfg_.min_nsv > 0 && g.nsv < cfg_.min_nsv) return false;  // 卫星数量检查
    return true;
}

// converters.cpp
GnssObservation fromGnssMsg(const deeplumin_msgs::Gpchc& msg) {
    GnssObservation g;
    g.timestamp = msg.header.stamp.toSec();
    g.latitude = msg.latitude;
    g.longitude = msg.longitude;
    g.altitude = msg.altitude;
    g.status = msg.status;
    g.nsv = msg.nsv1 + msg.nsv2;
    g.valid = (std::abs(g.latitude) > 1e-9 && std::abs(g.longitude) > 1e-9);
    return g;
}
```

### lm_slam 的状态定义

```cpp
// nav_state.h
struct NavState {
    // 23维状态：pos(3) + rot(3) + offset_R(3) + offset_t(3) + vel(3) + bg(3) + ba(3) + grav(2)
    Vec3d pos_ = Vec3d::Zero();             // 位置
    SO3 rot_;                               // 旋转
    SO3 offset_R_lidar_;                    // 外参R
    Vec3d offset_t_lidar_ = Vec3d::Zero();  // 外参t
    Vec3d vel_ = Vec3d::Zero();             // 速度
    Vec3d bg_ = Vec3d::Zero();              // 陀螺零偏
    Vec3d ba_ = Vec3d::Zero();              // 加计零偏
    S2 grav_;                               // 重力
};
```

### lm_slam 的 ESKF Update 实现

```cpp
// eskf.cc - 关键代码
void ESKF::Update(ESKF::ObsType obs, const double& R) {
    // ...
    case wheelspeed:
        Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);  // 3x23
        h_x_cur.block(0, 12, 3, 3) = custom_obs_model_.h_x_;  // 速度在第 12-14 维
        // 卡尔曼增益计算
        K = P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + R).inverse();
        // 状态更新
        x_ = x_ + K * residual;
        // 协方差更新
        P_ = P_ - K * h_x_cur * P_;
        break;

    case gps:
        Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);  // 3x23
        h_x_cur.block(0, 3, 3, 3) = custom_obs_model_.h_x_;  // 姿态在第 3-5 维
        // 卡尔曼增益计算
        K = P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose() + R).inverse();
        // 状态更新
        x_ = x_ + K * residual;
        // 协方差更新
        P_ = P_ - K * h_x_cur * P_;
        break;
}
```

### lm_slam 的观测模型实现

```cpp
// laser_mapping.cc

// GNSS 姿态观测模型
void LaserMapping::ObsModelGPS(NavState &s, ESKF::CustomObservationModel &obs) {
    obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);  // 3x3 矩阵（会被放到正确的列位置）
    obs.residual_.resize(3);
    obs.R_ = 0.03 * Eigen::MatrixXd::Identity(3, 3);  // 观测噪声
    obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // 单位矩阵

    // GNSS 观测姿态
    auto RPR = Vec3d(hc_heading, hc_pitch, hc_roll);
    SO3 gps_R = EulerZYXToSO3(RPR);

    // 残差 = log(R_state^{-1} * R_obs)
    obs.residual_ = (s.rot_.inverse() * gps_R).log();
}

// 轮速观测模型
void LaserMapping::ObsModelWelspeed(NavState &s, ESKF::CustomObservationModel &obs) {
    obs.h_x_ = Eigen::MatrixXd::Zero(3, 3);  // 3x3 矩阵（会被放到正确的列位置）
    obs.residual_.resize(3);
    obs.R_ = 0.33 * Eigen::MatrixXd::Identity(3, 3);  // 观测噪声
    obs.h_x_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // 单位矩阵

    // 轮速观测：车体系 x 方向速度
    Vec3d vel_odom(wheel_speed_obs, 0.0, 0.0);

    // 转换到世界系（只使用 rot，没有使用外参）
    auto R_wl = s.rot_;
    Vec3d vel_world = R_wl * vel_odom;

    // 残差 = 状态速度 - 观测速度
    obs.residual_ = s.vel_ - vel_world;
}
```

**关键点**：
1. lm_slam 的 CustomObservationModel 中的 `h_x_` 是 3x3 矩阵
2. 在 ESKF::Update 中，会将这个 3x3 矩阵放到正确的位置：
   - 轮速：`h_x_cur.block(0, 12, 3, 3)` → 速度在第 12-14 维
   - GNSS：`h_x_cur.block(0, 3, 3, 3)` → 姿态在第 3-5 维
3. 残差计算方式：
   - GNSS：`(s.rot_.inverse() * gps_R).log()`
   - 轮速：`s.vel_ - vel_world`
