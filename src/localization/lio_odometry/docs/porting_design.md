# LIO Odometry 移植设计方案

## 1. 概述

### 1.1 目标

将 `/home/hhy/2004ros/lm_slam_ros/src/lightning-lm-ros1/src/core/lio` 以及依赖的核心代码移植到 DeepLumin_zq，作为一个独立的里程计节点 `lio_odometry`。

### 1.2 移植原则

1. **ROS-C++ 解耦**：ROS 节点层只负责订阅/发布/参数读取，核心算法零 ROS 依赖
2. **输出对齐**：话题/坐标系/帧率与 `slam_odometry_v2` 一致，可直接接入 `slam_mapping_v2` 和 `localization_v2`
3. **最小侵入**：不修改已有模块的接口，新增模块独立编译
4. **可选扩展**：CorrectionHandler 校正回传模块作为可选插件，默认关闭

### 1.3 与现有 slam_odometry_v2 的关系

| 维度 | slam_odometry_v2 | lio_odometry (本方案) | 说明 |
|---|---|---|---|
| ESKF 实现 | MTK/esekfom 模板元 | 自定义 ESKF (NavState, 23维) | 二者结构等价，实现风格不同 |
| Anderson 加速 | 不支持 | 支持 | lm_slam_ros 特色 |
| 轮速观测 | ApplyWheelSpeedObservation (ESKF内联) | ObsModelWelspeed (独立观测方程) | 逻辑等价 |
| GNSS 姿态观测 | ApplyGnssAttitudeObservation (ESKF内联) | ObsModelGPS (独立观测方程) | 逻辑等价 |
| 关键帧管理 | 无 (由 slam_mapping_v2 负责) | 有 internal `all_keyframes_` | 移植后保留，仅用于 iVox 重建 |
| iVox 实现 | ivox3d/ (PHC 节点型) | ivox3d/ (DEFAULT/PHC 均有) | 本方案复用 slam_odometry_v2 的 ivox3d |
| 校正回传接口 | 无 | CorrectionHandler (可选) | 新增模块 |

## 2. 整体架构

### 2.1 节点架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    lio_odometry 节点                              │
│                                                                 │
│  ┌─────────────────┐    ┌────────────────────────────────────┐  │
│  │ ROS 节点层       │    │  C++ 核心库                         │  │
│  │ (lio_odometry_   │    │  (slam_odometry, 零ROS依赖)           │  │
│  │  node.cc)        │    │                                    │  │
│  │                  │    │  ┌──────────────────────────────┐  │  │
│  │  ros::Subscriber │───▶│  │ LaserOdometryCore            │  │  │
│  │  - /ouster/imu   │    │  │  ├── ESKF (custom, 23维)    │  │  │
│  │  - /ouster/points│    │  │  ├── ImuProcess             │  │  │
│  │  - /can_receive  │    │  │  ├── PointCloudPreprocess   │  │  │
│  │  - /gnss_chc     │    │  │  ├── AndersonAcceleration   │  │  │
│  │  - /correction   │    │  │  ├── ObsModel (LiDAR匹配)   │  │  │
│  │                  │    │  │  ├── ObsModelWelspeed       │  │  │
│  │  ros::Publisher  │◀───│  │  ├── ObsModelGPS            │  │  │
│  │  - /slam_odometry│    │  │  ├── iVox LocalMap (共享)   │  │  │
│  │    /odom         │    │  │  └── ScanHistory (校正用)   │  │  │
│  │  - /slam_odometry│    │  └──────────────────────────────┘  │  │
│  │    /local_map    │    └────────────────────────────────────┘  │
│  │  - /slam_odometry│        ▲                                   │
│  │    /cloud_       │        │                                   │
│  │    registered    │   ┌────────────────────────┐               │
│  │  - TF            │   │ CorrectionHandler      │               │
│  └─────────────────┘   │ (可选，默认关闭)         │               │
│                        └────────────────────────┘               │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 目录结构

```
src/localization/lio_odometry/
├── include/
│   └── lio_odometry/
│       ├── core/
│       │   ├── laser_odometry_core.h    // 主算法类（从 laser_mapping.h 移植）
│       │   ├── eskf.h                   // ESKF 头文件（从 eskf.hpp 移植）
│       │   ├── eskf_types.h             // NavState, SO3, S2 等类型定义
│       │   ├── imu_processing.h         // IMU 预积分（从 imu_processing.hpp 移植）
│       │   ├── pointcloud_preprocess.h  // 点云预处理
│       │   ├── anderson_acceleration.h  // Anderson 加速（从 lm_slam 直接拷贝）
│       │   ├── correction_handler.h     // 校正回传处理器（新增）
│       │   └── types.h                  // 模块内部类型
│       └── ros/
│           ├── ros_adapter.h            // ROS 适配层
│           └── config.h                 // 配置加载
├── src/
│   ├── core/
│   │   ├── laser_odometry_core.cpp
│   │   ├── eskf.cpp
│   │   ├── imu_processing.cpp
│   │   ├── pointcloud_preprocess.cpp
│   │   └── correction_handler.cpp
│   └── ros/
│       ├── ros_adapter.cpp
│       ├── config.cpp
│       └── lio_odometry_node.cpp        // ROS 节点入口
├── config/
│   └── lio_odometry.yaml
├── launch/
│   └── lio_odometry.launch
├── CMakeLists.txt
└── package.xml
```

## 3. 核心数据流

### 3.1 主循环

```
每帧处理流程 (LaserOdometryCore::Run()):

IMU Buffer ──► SyncPackages()
                    │
                    ▼
            IMU Process (Predict + Undistort)
                    │
                    ▼
            First Scan? ──Yes──► Init iVox
                    │
                    No
                    ▼
            IEKF Update (LiDAR + 轮速 + GNSS)
                    │
                    ▼
            state_point_ = kf_.get_x()
                    │
                    ▼
            MapIncremental() → 插入 iVox
                    │
                    ▼
            MakeKF() → 创建关键帧（可选）
                    │
                    ▼
            PushScanHistory() → 存储校正用历史
                    │
                    ▼
            返回结果 (odom, local_map, etc.)
```

### 3.2 ROS 节点层到 C++ 核心的数据流

```
ROS 消息                          C++ 数据类型
─────────                        ────────────
sensor_msgs::Imu        ──►      ImuData {timestamp, acc, gyr}
sensor_msgs::PointCloud2 ──►     PointCloudType::Ptr (pcl)
deeplumin_msgs::CanReceiveInfo ──► WheelData {timestamp, speed_kmh, gear}
deeplumin_msgs::Gpchc    ──►     GnssAttitude {heading, pitch, roll}
nav_msgs::Odometry       ──►     CorrectionInput {T_map_odom}
(correction 话题)                  │
                                  ▼
                           LaserOdometryCore
                           (纯 C++, 无 ROS 头文件)
                                  │
                                  ▼
C++ 数据类型                       ROS 消息
─────────                        ────────────
NavState (pos, rot, vel...) ──►  nav_msgs::Odometry
PointCloudType::Ptr        ──►  sensor_msgs::PointCloud2
```

## 4. 移植文件清单

### 4.1 lm_slam_ros → lio_odometry 映射

| lm_slam_ros 源文件 | 目标文件 | 修改幅度 | 说明 |
|---|---|---|---|
| `core/lio/laser_mapping.h` | `core/laser_odometry_core.h` | **大改** | 剥离 ROS/UI/Keyframe 管理，重命名为 LaserOdometryCore |
| `core/lio/laser_mapping.cc` | `core/laser_odometry_core.cpp` | **大改** | 同上；IMU处理逻辑保留，移除PangolinWindow/地图保存 |
| `core/lio/eskf.hpp` | `core/eskf.h` | **中改** | 从 .hpp 转为 .h；更新 namespace 为 `slam_odometry` |
| `core/lio/eskf.cc` | `core/eskf.cpp` | **小改** | Predict/Update 逻辑不变 |
| `core/lio/imu_processing.hpp` | `core/imu_processing.h` | **中改** | 从 .hpp 转为 .h；更新类型引用路径 |
| `core/lio/pointcloud_preprocess.h` | `core/pointcloud_preprocess.h` | **小改** | 更新 namespace 和类型路径 |
| `core/lio/pointcloud_preprocess.cc` | `core/pointcloud_preprocess.cpp` | **小改** | 同上 |
| `core/lio/anderson_acceleration.h` | `core/anderson_acceleration.h` | **不变** | 直接拷贝 |
| `core/lio/pose6d.h` | `core/types.h` | **合并** | 合并进自有类型定义 |
| `common/nav_state.h` | `core/eskf_types.h` | **中改** | 保留 NavState 结构，更新 namespace |
| `common/eigen_types.h` | `core/types.h` | **合并** | 取需要的类型定义 |
| `common/s2.hpp` | `core/eskf_types.h` | **合并** | 保留 S2 流形类型 |
| - | `core/correction_handler.h/cpp` | **新增** | 校正回传处理器 |
| - | `ros/ros_adapter.h/cpp` | **新增** | ROS 适配层 |
| - | `ros/config.h/cpp` | **新增** | 配置加载 |
| - | `ros/lio_odometry_node.cpp` | **新增** | 节点入口 |

### 4.2 依赖引用策略

| 依赖 | 策略 | 说明 |
|---|---|---|
| `ivox3d` | **复用** slam_odometry_v2 的 `faster_lio/ivox3d/` | CMake target_link_libraries |
| `IKFoM_toolkit` | **不需** | 本方案使用自定义 ESKF，不需要 MTK 模板 |
| `pcl` | 正常链接 | 系统依赖 |
| `Sophus` | **不需** | 本方案使用自定义 SO3/S2 实现 |
| `yaml-cpp` | 正常链接 | 配置加载用 |
| `glog` | 正常链接 | 日志用 |
| `deeplumin_msgs` | 正常链接 | CanReceiveInfo, Gpchc 消息 |

## 5. 核心类型设计

### 5.1 namespace 设计

```
lm_slam_ros              lio_odometry
───────────              ───────────
lightning::              slam_odometry::
lightning::LaserMapping  slam_odometry::LaserOdometryCore
lightning::ESKF          slam_odometry::ESKF
lightning::NavState      slam_odometry::NavState
lightning::SO3           slam_odometry::SO3
lightning::ImuProcess    slam_odometry::ImuProcess
```

ROS 适配层使用:
```
namespace slam_odometry_ros {}  // ROS 适配层
```

### 5.2 核心类型定义

```cpp
// core/eskf_types.h
namespace slam_odometry {

/// 基于 lm_slam_ros 的 NavState（23维状态向量）
/// 状态组成: pos(3) + rot(3) + offset_R_LI(3) + offset_T_LI(3) + vel(3) + bg(3) + ba(3) + grav(2) = 23
struct NavState {
    static constexpr int kDim = 23;
    static constexpr int kFullDim = 24;
    
    double timestamp_ = 0.0;
    Vec3d pos_ = Vec3d::Zero();
    SO3 rot_;                          // SO3 (Sophus 风格)
    SO3 offset_R_lidar_;               // LiDAR → IMU 旋转
    Vec3d offset_t_lidar_ = Vec3d::Zero();  // LiDAR → IMU 平移
    Vec3d vel_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();         // gyro bias
    Vec3d ba_ = Vec3d::Zero();         // acc bias
    S2 grav_{0, 0, -9.81};             // S2 流形上的重力向量
    
    // 李代数运算: boxplus, boxminus, get_f, df_dx, df_dw ...
    FullVectState ToState() const;
    void FromState(const FullVectState& state);
    VectState boxminus(const NavState& other);
    NavState boxplus(const VectState& dx) const;
};

/// 自定义 ESKF (23维状态, 24维误差)
class ESKF {
public:
    static constexpr int kProcessNoiseDim = 12;
    static constexpr int kStateDim = NavState::kDim;
    using CovType = Eigen::Matrix<double, kStateDim, kStateDim>;
    
    enum class ObsType {
        LIDAR,
        WHEEL_SPEED,
        GPS,
        BIAS,
    };
    
    struct CustomObservationModel {
        bool valid_ = true;
        Eigen::MatrixXd residual_;
        Eigen::MatrixXd h_x_;
        Eigen::MatrixXd R_;
        double lidar_residual_mean_ = 0;
    };
    
    using CustomObsFunc = std::function<void(NavState&, CustomObservationModel&)>;
    
    void Init(const Options& options);
    void Predict(double dt, const ProcessNoiseType& Q, const Vec3d& gyro, const Vec3d& acce);
    void Update(ObsType type, double R);
    void ChangeX(const NavState& state);
    
    const NavState& GetX() const { return x_; }
    const CovType& GetP() const { return P_; }
    
private:
    NavState x_;
    CovType P_;
    // ... 内部实现
};

}  // namespace slam_odometry
```

## 6. ROS 适配层设计

### 6.1 RosAdapter 类

```cpp
// ros/ros_adapter.h
namespace slam_odometry_ros {

class LioOdometryRosAdapter {
public:
    bool Init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void Spin();   // 主循环
    
private:
    // 传感器订阅回调（仅做数据格式转换）
    void OnImu(const sensor_msgs::Imu::ConstPtr& msg);
    void OnLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void OnWheelSpeed(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg);
    void OnGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg);
    void OnCorrection(const nav_msgs::Odometry::ConstPtr& msg);  // 可选校正
    
    // 发布函数 - 话题名从 config_.outputs 读取
    void PublishOdometry(const slam_odometry::NavState& state, double timestamp);
    void PublishScans();
    void PublishTF(const slam_odometry::NavState& state, double timestamp);
    
    // 核心实例
    std::unique_ptr<slam_odometry::LaserOdometryCore> core_;
    
    // 可选校正处理器
    std::unique_ptr<slam_odometry::CorrectionHandler> correction_handler_;
    
    // ROS 句柄
    ros::Subscriber imu_sub_, lidar_sub_, wheel_sub_, gnss_sub_, correction_sub_;
    ros::Publisher odom_pub_, reg_scan_pub_, effect_scan_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 配置
    slam_odometry_ros::NodeConfig config_;
};

}  // namespace slam_odometry_ros
```

### 6.2 节点主循环

```cpp
// ros/lio_odometry_node.cpp
int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_odometry");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    slam_odometry_ros::LioOdometryRosAdapter adapter;
    if (!adapter.Init(nh, pnh)) {
        ROS_ERROR("[lio_odometry] Init failed");
        return -1;
    }
    
    ros::spin();
    return 0;
}
```

### 6.3 话题接口

话题统一使用 `slam_odometry` 命名空间，与 slam_odometry_v2 对齐，slam_mapping_v2 和 localization_v2 按话题配置自动适配。

| 配置键 | 默认值 | 类型 | frame_id | 频率 | 说明 |
|---|---|---|---|---|---|
| `outputs.odom_topic` | `/slam_odometry/odom` | `nav_msgs/Odometry` | `odom` | LiDAR频率 | 主里程计输出 |
| `outputs.registered_scan_topic` | `/slam_odometry/cloud_registered` | `sensor_msgs/PointCloud2` | `odom` | 每帧 | 配准后点云 |
| `outputs.effect_scan_topic` | `/slam_odometry/cloud_effect` | `sensor_msgs/PointCloud2` | `odom` | 每帧 | 有效对应点 |
| `outputs.publish_odom` | `true` | `bool` | - | - | 是否发布里程计 |
| `outputs.publish_tf` | `true` | `bool` | - | LiDAR频率 | 是否发布 odom→base_link TF |

### 6.4 配置参数

```yaml
# config/lio_odometry.yaml

# 传感器话题
common:
  imu_topic: "/ouster/imu"
  lidar_topic: "/ouster/points"
  wheel_speed_topic: "/can_receive_info"
  gnss_topic: "/gnss_chc_data"

# 坐标系
frames:
  odom: "odom"
  base_link: "base_link"
  imu: "imu_link"

# 外参
extrinsics:
  base_to_imu:
    rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    translation: [0, 0, 0]
  base_to_lidar:
    rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    translation: [0, 0, 0]

# LIO 核心参数
fasterlio:
  lidar_type: 3                    # 1=AVIA, 2=VELO32, 3=OUST64
  scan_line: 64
  blind: 1.2
  point_filter_num: 2
  max_iteration: 6
  filter_size_scan: 0.1
  filter_size_map: 0.5
  ivox_grid_resolution: 0.2
  ivox_nearby_type: 6
  lidar_update_r: 0.001
  
  acc_cov: 0.2
  gyr_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  
  extrinsic_est_en: true
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
  
  use_aa: false                     # Anderson Acceleration 开关
  skip_lidar_num: 5

# 轮速和 GNSS 约束
vel_gnss:
  use_wheel_speed: false
  use_gnss: false
  wheel_velocity_noise: 0.33
  gnss_attitude_noise: 0.03
  gnss_pitch_window_size: 50

# 校正回传（可选，默认关闭）
correction:
  enabled: false
  topic: "/slam_mapping_v2/correction"
  max_translation_jump: 5.0
  max_rotation_jump_deg: 30.0
  max_frequency_hz: 0.2
  rebuild_window: 50

# 发布话题配置（与 slam_odometry_v2 格式一致）
outputs:
  publish_odom: true
  odom_topic: "/slam_odometry/odom"
  registered_scan_topic: "/slam_odometry/cloud_registered"
  effect_scan_topic: "/slam_odometry/cloud_effect"
  publish_tf: true
  local_map_publish_interval: 10
```

## 7. ESKF 移植说明

### 7.1 类型映射

| lm_slam_ros (lightning) | lio_odometry (slam_odometry) | 说明 |
|---|---|---|
| `lightning::Vec3d` | `Eigen::Vector3d` | 直接使用 Eigen 原生类型 |
| `lightning::SO3` | `slam_odometry::SO3` | 保持 Sophus 风格的自定义 SO3 |
| `lightning::S2` | `slam_odometry::S2` | 保持 S2 流形实现 |
| `lightning::NavState` | `slam_odometry::NavState` | 23维状态向量，完全保留 |
| `lightning::ESKF` | `slam_odometry::ESKF` | 保留自定义 ESKF 实现 |
| `lightning::MeasureGroup` | `slam_odometry::MeasureGroup` | IMU+LiDAR 同步组 |

### 7.2 ESKF 状态维度说明

NavState (23维) = pos(3) + rot(3) + offset_R_LI(3) + offset_T_LI(3) + vel(3) + bg(3) + ba(3) + grav(2)

与 slam_odometry_v2 的 MTK 实现（state_ikfom）维度完全一致：
```
MTK: pos(3) + rot(3) + offset_R_L_I(3) + offset_T_L_I(3) + vel(3) + bg(3) + ba(3) + grav(2) = 23
```

### 7.3 协方差初始化

```cpp
ESKF::CovType P = ESKF::CovType::Identity();
// 位置初值
P.block<3,3>(0,0) = 0.01 * Eigen::Matrix3d::Identity();
// 姿态初值
P.block<3,3>(3,3) = 0.01 * Eigen::Matrix3d::Identity();
// 速度初值
P.block<3,3>(12,12) = 1.0 * Eigen::Matrix3d::Identity();
// 陀螺零偏
P.block<3,3>(15,15) = 0.0001 * Eigen::Matrix3d::Identity();
// 加计零偏
P.block<3,3>(18,18) = 0.0001 * Eigen::Matrix3d::Identity();
// 重力
P.block<2,2>(21,21) = 0.0001 * Eigen::Matrix2d::Identity();
```

### 7.4 观测模型移植

#### LiDAR 观测 (ObsModel)

直接移植 `laser_mapping.cc` 中的 `ObsModel` 实现：
```
点到面残差(norm_vec) → H 矩阵(12列: pos, rot, offset_R, offset_T)
残差维度随有效匹配点数变化
Anderson Acceleration 可选
```

#### 轮速观测 (ObsModelWelspeed)

```cpp
// 3维观测: 世界系速度 = 轮速(车体系x向) 转换到世界系
h_x_ = [I_3x3, 0_3x9]  // 只观测 vel 维度
residual_ = -(v_est - R * v_wheel)
R_ = wheel_velocity_noise^2 * I_3x3
```

#### GNSS 姿态观测 (ObsModelGPS)

```cpp
// 3维观测: 相对首帧的旋转增量
h_x_ = [I_3x3, 0_3x9]  // 只观测 rot 维度
residual_ = (R_est^{-1} * R_gnss).log()
R_ = gnss_attitude_noise^2 * I_3x3
```

## 8. 校正回传模块（可选）

### 8.1 设计动机

在长程建图场景（2km+）中，LIO 的前端 iVox 局部地图会随漂移累积而产生"新旧点云分裂"，影响后续扫描配准质量。此模块通过接收 slam_mapping_v2 的优化结果来修正前端状态。

### 8.2 设计原则

- **可插拔**：CorrectionHandler 作为独立模块，由 ROS 节点可选地创建
- **零侵入**：仅通过 `LaserOdometryCore` 的 3 个新增接口工作
- **安全限幅**：平移跳变 >5m 或旋转 >30° 时自动丢弃
- **防抖**：同一校正源 5 秒内最多触发一次

### 8.3 LaserOdometryCore 新增接口

```cpp
class LaserOdometryCore {
public:
    /// 推送扫描历史（在 MapIncremental 尾部调用）
    void PushScanHistory();
    
    /// 应用位姿校正：修改 ESKF state + 协方差复位
    /// @return true 成功
    bool ApplyPoseCorrection(const Vec3d& new_pos, const SO3& new_rot);
    
    /// 重建 iVox 局部地图（从 scan_history_ 重建最近 N 帧）
    bool RebuildLocalMap(int window_size);
    
    /// 获取当前 ESKF 状态（用于校正前诊断）
    void GetState(Vec3d& pos, SO3& rot) const;
    
private:
    struct ScanEntry {
        double timestamp_;
        CloudPtr cloud_body_;          // scan_down_body_ 的拷贝
        Vec3d pos_odom_;               // 插入时的 ESKF pos
        SO3 rot_odom_;                 // 插入时的 ESKF rot
    };
    std::deque<ScanEntry> scan_history_;
    static constexpr int kMaxScanHistory = 200;
    bool correction_enabled_ = false;  // 由配置控制
};
```

### 8.4 CorrectionHandler 接口

```cpp
namespace slam_odometry {

struct CorrectionInput {
    double timestamp = 0.0;
    bool valid = false;
    
    // map → odom 变换
    Eigen::Quaterniond q_map_odom = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_map_odom = Eigen::Vector3d::Zero();
    
    int keyframe_id = -1;
    std::string source = "unknown";
};

class CorrectionHandler {
public:
    bool Init(const CorrectionConfig& config);
    
    /// 接收校正数据（线程安全）
    void PushCorrection(const CorrectionInput& input);
    
    /// 检查是否有未执行的校正
    bool HasPendingCorrection() const;
    
    /// 执行校正：ApplyPoseCorrection + RebuildLocalMap
    bool Apply(LaserOdometryCore* core);
    
    /// 对里程计输出做后处理：T_map_base = T_map_odom * T_odom_base
    void CorrectOdomOutput(const Vec3d& raw_pos, const SO3& raw_rot,
                           Vec3d& out_pos, SO3& out_rot) const;
    
    bool hasCorrection() const { return has_correction_; }
    int correctionCount() const { return correction_count_; }
};

}  // namespace slam_odometry
```

### 8.5 校正流程

```
slam_mapping_v2:
  → PoseGraphOptimizer 优化完成
  → 计算 T_map_odom = latest_kf.pose_map * latest_kf.pose_odom^{-1}
  → 发布 nav_msgs/Odometry (frame_id="map", child_frame_id="odom")

lio_odometry 节点:
  → OnCorrection() 回调
    → 填充 CorrectionInput
    → correction_handler_.PushCorrection(input)
    
  → tick() 中:
    → core_->Run()  (正常处理一帧)
    → if correction_handler_.HasPendingCorrection():
        → correction_handler_.Apply(core_.get())
            → 1. ValidateJump()          ← 跳变门限检查
            → 2. core_->GetState(pos, rot)
            → 3. 计算 corrected_pose = T_map_odom * raw_pose
            → 4. core_->ApplyPoseCorrection(corrected_pos, corrected_rot)
                → kf_.ChangeX(modified_state)
                → state_point_ = modified_state
            → 5. core_->RebuildLocalMap(config_.rebuild_window)
                → 用 corrected_pose 变换 scan_history_ 中最近 N 帧
                → 重新插入 iVox
    → 发布里程计（使用 CorrectOdomOutput 做后处理）
```

## 9. CMakeLists.txt 构建

```cmake
cmake_minimum_required(VERSION 3.0)
project(lio_odometry)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    tf
    pcl_conversions
    pcl_ros
    deeplumin_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lio_odometry_core
    CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf deeplumin_msgs pcl_conversions
    DEPENDS PCL Eigen3 yaml-cpp
)

# 核心库（纯 C++，无 ROS 依赖）
add_library(lio_odometry_core
    src/core/laser_odometry_core.cpp
    src/core/eskf.cpp
    src/core/imu_processing.cpp
    src/core/pointcloud_preprocess.cpp
    src/core/correction_handler.cpp
)

target_include_directories(lio_odometry_core PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    # 引用 slam_odometry_v2 的 ivox3d
    ../slam_odometry_v2/include
)

target_link_libraries(lio_odometry_core
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)

# ROS 节点（链接核心库 + ROS 依赖）
add_executable(lio_odometry_node
    src/ros/lio_odometry_node.cpp
    src/ros/ros_adapter.cpp
    src/ros/config.cpp
)

target_link_libraries(lio_odometry_node
    lio_odometry_core
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)
```

## 10. 与 slam_mapping_v2 的集成

### 10.1 输入接口（里程计 → 建图）

lio_odometry 发布以下话题供 slam_mapping_v2 订阅（话题通过 YAML 配置，默认值如下）：
- `/slam_odometry/odom` → 里程计位姿
- `/slam_odometry/cloud_registered` → 配准后点云

slam_mapping_v2 在其 YAML 配置中指定里程计话题来源，无需修改代码即可接入 lio_odometry。话题命名空间统一为 `slam_odometry`。

### 10.2 输出接口（建图 → 里程计，可选校正）

slam_mapping_v2 在每次优化后发布：
- `/slam_mapping_v2/correction` (nav_msgs/Odometry)
  - `frame_id: "map"`, `child_frame_id: "odom"`
  - `pose`: T_map_odom（从最新关键帧的 `pose_map * pose_odom^{-1}` 计算）

### 10.3 话题依赖总图

```
lio_odometry ──/slam_odometry/odom─────────────► slam_mapping_v2
lio_odometry ──/slam_odometry/cloud_registered──► slam_mapping_v2
lio_odometry ◀──/slam_mapping_v2/correction────── slam_mapping_v2 (可选)

slam_mapping_v2 ──/map_to_odom─────────────► localization_v2
localization_v2 ◀──/slam_odometry/odom─────── lio_odometry (同)
```

## 11. 与 localization_v2 的集成

lio_odometry 发布的话题与 slam_odometry_v2 完全一致，localization_v2 不需要任何修改。

localization_v2 中订阅 odom 的部分（可在其 YAML 中配置话题名）：
```yaml
# localization_v2.yaml
slam_odometry_topic: "/slam_odometry/odom"  # 与 slam_odometry_v2 使用同一话题名
```

## 12. 测试计划

### 12.1 单元测试

| 测试项 | 方法 | 预期 |
|---|---|---|
| ESKF Predict | 输入已知IMU数据，验证状态递推 | 与 lm_slam_ros 输出一致（< 1e-6） |
| ESKF Update (LiDAR) | 输入固定观测，验证状态更新 | 收敛到正确位姿 |
| ESKF Update (轮速) | 输入轮速观测，验证速度修正 | 速度向轮速收敛 |
| ESKF Update (GNSS) | 输入GNSS姿态，验证姿态修正 | 姿态向GNSS收敛 |
| ObsModel 矩阵 | 对比 H 矩阵各个元素 | 与原始 lm_slam_ros 一致 |
| iVox 接口 | 添加/查询/移除点 | 与 slam_odometry_v2 的 ivox 行为一致 |

### 12.2 集成测试

| 测试场景 | 方法 | 验证指标 |
|---|---|---|
| 同一 bag 回放 | lio_odometry vs lm_slam_ros 同时运行 | 里程计输出误差 < 0.1% |
| 短程场景 | 100m 直线，对比 odom 输出 | 终点位姿误差 < 0.5m |
| 长程场景 | 2km+ 园区 bag | 不出现明显发散 |
| 轮速辅助 | 开启 use_wheel_speed，对比开启/关闭效果 | 速度更平滑 |
| GNSS 辅助 | 开启 use_gnss，对比开启/关闭效果 | pitch 漂移减小 |
| 校正回传 | 模拟 slam_mapping_v2 发校正消息 | ESKF 状态正确跳变，iVox 重建成功 |

### 12.3 对比测试脚本

```bash
# 同时运行 lio_odometry 和 lm_slam_ros 做 A/B 对比
roslaunch lio_odometry lio_odometry.launch bag:=/path/to/test.bag &
roslaunch lm_slam_ros lm_slam_ros.launch bag:=/path/to/test.bag &
# 对比 /slam_odometry/odom 和 lm_slam_ros 的 /path 话题
```

## 13. 实施步骤

### 阶段一：基础设施

| 步骤 | 内容 | 预计时间 |
|---|---|---|
| 1.1 | 创建目录结构、CMakeLists.txt、package.xml | 0.5天 |
| 1.2 | 拷贝 ivox3d/ 引用（确认 slam_odometry_v2 的可用性） | 0.5天 |
| 1.3 | 实现 `types.h`、`eskf_types.h`（NavState, SO3, S2） | 1天 |
| 1.4 | 移植 `anderson_acceleration.h` | 0.5天 |

### 阶段二：核心算法

| 步骤 | 内容 | 预计时间 |
|---|---|---|
| 2.1 | 移植 `eskf.h/cpp`（Predict, Update, boxplus/boxminus） | 2天 |
| 2.2 | 移植 `imu_processing.h/cpp`（IMU 初始化+预积分+去畸变） | 1.5天 |
| 2.3 | 移植 `pointcloud_preprocess.h/cpp` | 0.5天 |
| 2.4 | 移植 `laser_odometry_core.h/cpp`（ObsModel, MapIncremental） | 3天 |
| 2.5 | 移植轮速/GNSS 观测模型（ObsModelWelspeed, ObsModelGPS） | 1天 |
| 2.6 | 单元测试（ESKF, ObsModel 矩阵一致性） | 2天 |

### 阶段三：ROS 层

| 步骤 | 内容 | 预计时间 |
|---|---|---|
| 3.1 | 实现 `config.h/cpp`（YAML 参数加载） | 0.5天 |
| 3.2 | 实现 `ros_adapter.h/cpp` | 1天 |
| 3.3 | 实现 `lio_odometry_node.cpp` | 0.5天 |
| 3.4 | Bag 回放集成测试 | 2天 |

### 阶段四：校正回传（可选）

| 步骤 | 内容 | 预计时间 |
|---|---|---|
| 4.1 | 在 `LaserOdometryCore` 中新增 `PushScanHistory` 等接口 | 1天 |
| 4.2 | 实现 `CorrectionHandler` | 1.5天 |
| 4.3 | 集成到 ROS 节点 | 0.5天 |
| 4.4 | 长程场景测试 | 2天 |

### 总预计时间：约 21 天

---

## 14. 视觉光度误差接入可行性分析

### 14.1 背景：什么是光度误差

光度误差（Photometric Error）是**直接法视觉里程计**（如 DSO, SVO, LSD-SLAM）使用的观测模型。它的核心思想是：

> 同一个空间点在两帧图像上应当具有相同的亮度（灰度不变假设）。
> 
> 残差 = 参考帧像素亮度 − 当前帧投影到参考帧的亮度

与传统**特征点法**（提取 ORB/SIFT 再匹配）的区别：

| 方法 | 观测 | 信息利用率 | 弱纹理表现 | 计算量 |
|---|---|---|---|---|
| 特征点法 | 特征重投影误差 | 低（只用几百个特征点） | 差 | 低 |
| 光度误差 | 像素亮度差 | 高（可用数千像素） | 好 | 高 |
| 混合 | 两者结合 | 最高 | 最好 | 最高 |

### 14.2 当前框架对光度误差的适配性分析

#### 14.2.1 有利因素

**（a）ESKF 的 CustomObservationModel 模式天然可扩展**

当前的 `ESKF::Update(ObsType type, double R)` 机制已经支持多观测模型通过回调函数注入：

```cpp
// 现有的管理模式
eskf_options.lidar_obs_func_ = [this](NavState& s, CustomObservationModel& obs) { ObsModel(s, obs); };
eskf_options.wheelspeed_obs_func_ = [this](NavState& s, CustomObservationModel& obs) { ObsModelWelspeed(s, obs); };
eskf_options.gps_obs_func_ = [this](NavState& s, CustomObservationModel& obs) { ObsModelGPS(s, obs); };

// 新增观测只需：
eskf_options.photometric_obs_func_ = [this](NavState& s, CustomObservationModel& obs) { ObsModelPhotometric(s, obs); };
```

这是**框架层面的天然适配**——任何关于状态的残差模型都能通过 `CustomObsFunc` 接入。

**（b）IEKF 可以处理高维观测**

LiDAR 观测的残差维度随有效特征点数变化（可达数千维），与光度误差的高维残差性质匹配。ESKF 的 Update 已支持动态维度的 `residual_` 和 `h_x_`。

**（c）Anderson Acceleration 加速收敛**

光度误差的残差-雅可比映射往往收敛较慢（光度函数的非线性比点到面更强），Anderson 加速可以提升迭代收敛速度。

#### 14.2.2 需要解决的问题

**（a）状态向量需要扩展**

当前 NavState 只有 23 维，不包含逆深度/三维点等路标状态：

```
NavState (23维) = pos(3) + rot(3) + offset_R_LI(3) + offset_T_LI(3) + vel(3) + bg(3) + ba(3) + grav(2)
```

视觉光度误差的 H 矩阵需要映射到 pose（6DOF），但更先进的联合优化通常需要路标状态。有两个方案：

| 方案 | 描述 | 状态改动 | 精度 | 复杂度 |
|---|---|---|---|---|
| A: Pose-only 光度 | 只对位姿求光度残差雅可比，不做路标优化 | 不修改 | 中 | 低 |
| B: 加路标状态 | 扩展状态向量增加逆深度 | 需改为 23+N 维 | 高 | 高 |

**方案 A** 适合"以 LiDAR 为主、视觉为辅助"的场景。每一帧先做 LiDAR IEKF 更新，再用光度误差做一次"精细位姿修正"。H 矩阵只对 `pos` 和 `rot` 有非零块（6列），残差维度由选取的像素数决定。

```cpp
// Pose-only 光度误差观测模型（示意图）
void ObsModelPhotometric(NavState& s, ESKF::CustomObservationModel& obs) {
    // 1. 从图像中选取梯度明显的像素点
    // 2. 对每个像素点：
    //    - 根据当前估计位姿，将参考帧像素投影到当前帧
    //    - 计算灰度误差
    //    - 计算灰度对位姿的雅可比（链式法则：灰度向导数 × 投影到雅可比）
    // 3. 组装 H 矩阵：维度 [N_pixels] x 6 (只对 pos, rot 的扰动)
    obs.h_x_.resize(num_pixels, 12);  // 只对前 12 列（pos + rot）非零
    obs.h_x_.block(0, 0, num_pixels, 12) = H_pose;
    obs.h_x_.block(0, 12, num_pixels, 11) = 0;
    obs.residual_ = photometric_residuals;  // N_pixels 维
    obs.R_ = photometric_noise * I_N;       // 光度噪声
}
```

**（b）需要图像数据输入**

当前框架没有图像数据通道。需要：

1. `LaserOdometryCore` 新增 `FeedImage(double timestamp, const ImageData& image)` 接口
2. ROS 节点层增加相机订阅 `image_sub_`
3. 同步机制需要同时对齐 IMU + LiDAR + Camera 三模态数据

**（c）参考帧管理**

光度误差需要一个"参考帧"（通常是最近的图像关键帧）来计算残差。这与 LiDAR 的 iVox 局部地图类似——需要有参考帧缓冲队列。

```cpp
// 新增加的结构
struct ReferenceFrame {
    double timestamp_;
    cv::Mat image_;               // 灰度图
    std::vector<PixelPoint> pixels_;  // 选中的像素点（梯度足够大）
    NavState pose_;                 // 选取时的位姿
};
std::deque<ReferenceFrame> ref_frames_;
```

**（d）计算量挑战**

光度误差的计算量远大于 LiDAR 点面匹配：

| 操作 | LiDAR 匹配 | Pose-only 光度 | 全光度 |
|---|---|---|---|
| 数据点数量 | ~500 特征点 | ~2000 像素点 | ~10000 像素点 |
| 雅可比计算 | 点面距离 → 链式法则 | 灰度梯度 → 投影 → 6DOF | 灰度梯度 → 投影 → 6DOF + 逆深度 |
| 每帧耗时 | ~10ms (CPU) | ~30ms (CPU) | ~100ms (CPU) / ~20ms (GPU) |

对于实时性要求（LiDAR 10Hz → 100ms 一帧），需要使用 GPU（CUDA/OpenCL）加速光度误差计算。

### 14.3 推荐的集成路径

```
阶段一：Pose-only 光度修正（CPU，200-500 像素稀疏采样）
  └── 在 LiDAR IEKF 更新之后，额外做一次"光度精修"
  └── 不改 ESKF 状态维度
  └── 验证：同一 bag + 图像数据，对比有无光度修正的精度差异

阶段二：半稠密光度约束（CPU 并行加速）
  └── 增加到 2000-5000 像素
  └── 参考帧管理 + 滑动窗口
  └── 图像梯度预计算（降低每帧计算量）

阶段三：全稠密光度 + GPU 加速（可选）
  └── 使用 CUDA/OpenCL 实现完整光度误差计算
  └── 考虑扩展状态向量加入逆深度
  └── tight-coupled 视觉惯性融合
```

### 14.4 框架改动评估

```
框架改动面积：

LaserOdometryCore 新增：
  ├── FeedImage()                     // 数据入口
  ├── ObsModelPhotometric()           // 光度观测模型
  ├── ref_frame_buffer_               // 参考帧缓冲
  ├── camera_params_                   // 内参 + 畸变
  └── image_sync_                       // 图像+LiDAR+IMU同步

ESKF 新增：
  └── ObsType::PHOTOMETRIC             // 新的观测类型

ROS 节点层新增：
  ├── image_sub_                       // 相机订阅
  └── camera_info_sub_                 // 相机内参订阅

新增依赖：
  ├── cv_bridge / OpenCV               // 图像处理
  └── CUDA (可选)                      // GPU 加速
```

### 14.5 结论

```
是否可以接入视觉光度误差？   ✅ 是
框架的 ESKF 模式是否支持？ ✅ 天然适配（CustomObservationModel 回调）
当前状态向量是否够用？     ⚠️ 够（Pose-only），不够（Full joint）
是否需要扩展状态维度？     ❌ Pose-only 不需要；Full joint 需要
是否需要 GPU？             ❌ 稀疏(200-500px)不需要；稠密需要
接入难度（Pose-only 版本）：⭐⭐（中等，主要工作是图像数据通道）
接入难度（Full 版本）：    ⭐⭐⭐⭐（高，需要状态扩展+GPU）
推荐优先级：              Pose-only → 半稠密 → 全稠密
```

**核心推荐**：当前框架的 ESKF `CustomObservationModel` 模式为视觉光度误差提供了**非常自然**的接入点。建议从 **Pose-only 光度修正** 开始（不改状态维度、不改核心架构），验证视觉-LiDAR 融合的有效性后再逐步推进。

## 附录 A：关键代码差异对照

### A.1 ESKF::Predict

| lm_slam_ros (custom) | slam_odometry_v2 (MTK) |
|---|---|
| `NavState::get_f()` | `get_f(s, in)` |
| `NavState::df_dx()` | `df_dx(s, in)` |
| `NavState::df_dw()` | `df_dw(s, in)` |
| `oplus(f_, dt)` → 离散化 | `kf_.predict(dt)` → 离散化 |
| 手写 `F_x1_` 指数映射 | MTK 自动生成 |

两者数学等价。移植时保留 lm_slam_ros 的手写实现。

### A.2 ObsModel (LiDAR 匹配)

| lm_slam_ros | slam_odometry_v2 |
|---|---|
| H 矩阵: 12 列 (pos, rot, offset_R, offset_T) | H 矩阵: 12 列 (pos, rot, offset_R_L_I, offset_T_L_I) |
| 残差: 点到面距离 | 残差: 点到面距离 |
| 使用 `esti_plane` 估计平面 | 使用 `esti_plane` 估计平面（算法一致） |
| 支持 Anderson Acceleration | 不支持 |

移植时直接拷贝 ObsModel 实现，不需要从 MTK 版本转换。

### A.3 iVox 接口

lm_slam_ros 和 slam_odometry_v2 的 ivox3d 实现几乎一致，接口兼容：
- `AddPoints()`
- `GetClosestPoint()`
- `RemoveFarPoints()`

本方案直接引用 slam_odometry_v2 的 ivox3d，不拷贝新副本。