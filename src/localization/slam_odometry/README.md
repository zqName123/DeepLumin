# slam_odometry 包

## 概述

`slam_odometry` 是 DeepLumin 自动驾驶系统中的激光雷达惯性里程计模块，基于 **Faster-LIO** (Fast LiDAR-Inertial Odometry) 算法实现。该模块融合激光雷达点云、IMU、轮速计和 GNSS 数据，提供高频、高精度的实时位姿估计。

### 核心特性

- **多传感器融合**: 支持 LiDAR + IMU + 轮速计 + GNSS 姿态
- **自适应时间戳处理**: 自动检测点云时间戳，支持无时戳点云的特殊处理模式
- **IMU 预热机制**: 启动时收集 IMU 数据进行初始化，提高初始精度
- **频率自适应**: 无点时间戳模式下自动调整处理频率，平衡精度与性能
- **全局地图可视化**: 支持加载 PCD 文件进行全局地图可视化

---

## 目录结构

```
slam_odometry/
├── CMakeLists.txt              # CMake 构建配置
├── package.xml                 # ROS 包配置
├── README.md                   # 本说明文档
│
├── config/                     # 配置文件目录
│   └── slam_odometry_ysw.yaml  # ysw/ouster64 数据集配置文件
│
├── include/localization/       # 头文件目录
│   ├── common/                 # 通用类型和工具
│   │   ├── config.hpp          # 配置结构体定义
│   │   ├── math_utils.hpp      # 数学工具函数
│   │   └── types.hpp           # 基础类型定义（点云、位姿等）
│   │
│   ├── factory/                # 工厂模式实现
│   │   └── factory_registry.hpp # SLAM 算法工厂注册
│   │
│   ├── interface/              # 抽象接口定义
│   │   └── i_slam_odometry.hpp # SLAM 里程计接口类
│   │
│   ├── ros/                    # ROS 相关工具
│   │   ├── global_map_viz.hpp  # 全局地图可视化
│   │   ├── ouster_point.hpp    # Ouster 点云类型定义
│   │   ├── pointcloud_preprocess.hpp # 点云预处理
│   │   ├── ros_converters.hpp  # ROS 消息转换工具
│   │   └── sensor_topics.hpp   # 传感器话题配置加载
│   │
│   └── slam/                   # SLAM 算法实现
│       ├── faster_lio_impl.hpp # Faster-LIO 包装实现
│       └── laser_mapping_core.hpp # 激光建图核心类
│
├── launch/                     # 启动文件目录
│   └── slam_odometry_ysw.launch # ysw 场景启动文件
│
├── src/                        # 源代码目录
│   ├── slam_odometry_node.cpp  # 主节点入口
│   │
│   ├── factory/                # 工厂实现
│   │   └── factory_registry.cpp
│   │
│   ├── ros/                    # ROS 工具实现
│   │   ├── pointcloud_preprocess.cpp
│   │   ├── ros_converters.cpp
│   │   └── sensor_topics.cpp
│   │
│   └── slam/                   # SLAM 算法实现
│       ├── faster_lio_impl.cpp # Faster-LIO 接口实现
│       └── laser_mapping_core.cpp # 建图核心实现
│
└── thirdparty/                 # 第三方库
    └── faster_lio/             # Faster-LIO 原始代码
        ├── include/            # 头文件
        │   ├── IKFoM_toolkit/  # 迭代卡尔曼滤波工具包
        │   ├── ivox3d/         # 增量体素地图 (iVox)
        │   ├── imu_processing.hpp
        │   ├── imu_sample.hpp
        │   └── ...
        └── src/                # 源文件
```

### 目录结构说明

| 目录 | 说明 |
|------|------|
| `config/` | 存放 YAML 格式的参数配置文件，可根据不同场景创建多个配置 |
| `include/` | 头文件目录，按功能模块组织，便于代码复用和维护 |
| `launch/` | ROS 启动文件，定义节点启动参数和配置加载 |
| `src/` | 源代码实现，与头文件目录结构对应 |
| `thirdparty/faster_lio/` | Faster-LIO 第三方库，包含 IKFoM 和 iVox 核心算法 |

---

## 运行方式

### 1. 编译

```bash
# 在 DeepLumin 工作空间根目录执行
cd /home/hhy/2004ros/DeepLumin
catkin_make --pkg slam_odometry
# 或编译整个工作空间
catkin_make
```

### 2. 启动节点

#### 方式一：使用 Launch 文件（推荐）

```bash
# 启动 ysw 场景
roslaunch slam_odometry slam_odometry_ysw.launch
```

#### 方式二：直接运行节点

```bash
# 加载参数文件后运行节点
rosparam load /home/hhy/2004ros/DeepLumin/src/localization/slam_odometry/config/slam_odometry_ysw.yaml
rosrun slam_odometry slam_odometry_node
```

### 3. 运行依赖

确保以下话题有数据发布：
- `/ouster/imu` - IMU 数据
- `/ouster/points` - 激光雷达点云
- `/can_receive_info` - 轮速计数据（可选，根据配置）
- `/gnss_chc_data` - GNSS 姿态数据（可选，根据配置）

---

## 输入数据

### 1. IMU 数据

**话题名称**: `/ouster/imu` (可配置)

**消息类型**: `sensor_msgs/Imu`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `orientation` | 姿态四元数（通常由 IMU 内部滤波输出） |
| `angular_velocity` | 角速度（rad/s），x-y-z 三轴 |
| `linear_acceleration` | 线加速度（m/s²），x-y-z 三轴 |
| `header.stamp` | 时间戳，用于数据同步 |

**数据要求**: 频率建议 ≥ 200Hz，用于高频姿态预测

---

### 2. 激光雷达点云

**话题名称**: `/ouster/points` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `x, y, z` | 点云三维坐标（米） |
| `intensity` | 反射强度 |
| `t/time/timestamp` | 点时间戳（可选，用于运动补偿） |
| `ring` | 扫描线编号（可选） |

**数据要求**: 
- 支持 Ouster OS1-64 等 64 线激光雷达
- 频率：10Hz 或 20Hz
- 点云时间戳：支持自动检测，无时戳时启用兼容模式

---

### 3. 轮速计数据

**话题名称**: `/can_receive_info` (可配置)

**消息类型**: `deeplumin_msgs/CanReceiveInfo`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `speed` | 车速（km/h） |
| `gear` | 档位（2.0 表示倒车） |
| `valid` | 数据有效性标志 |
| `header.stamp` | 时间戳 |

**使用条件**: 配置文件中 `slam/faster_lio/use_vel: true`

---

### 4. GNSS 姿态数据

**话题名称**: `/gnss_chc_data` (可配置)

**消息类型**: `deeplumin_msgs/Gpchc`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `heading` | 航向角（度） |
| `pitch` | 俯仰角（度） |
| `roll` | 横滚角（度） |
| `header.stamp` | 时间戳 |

**使用条件**: 配置文件中 `slam/faster_lio/use_gnss_attitude: true`

---

## 输出数据

### 1. SLAM 里程计

**话题名称**: `/localization/slam_odom` (可配置)

**消息类型**: `nav_msgs/Odometry`

**内容说明**:
| 字段 | 说明 |
|------|------|
| `pose.pose.position` | 位置（x, y, z），单位：米 |
| `pose.pose.orientation` | 姿态四元数 |
| `pose.covariance` | 位姿协方差矩阵（6x6） |
| `twist.twist` | 速度（线速度和角速度） |
| `header.frame_id` | 参考坐标系（通常为 "odom"） |
| `child_frame_id` | 本体坐标系（通常为 "base_link"） |

**发布频率**: 与激光雷达频率一致（10Hz 或 20Hz）

---

### 2. 局部地图

**话题名称**: `/localization/local_map` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
- 当前局部地图的点云数据
- 基于 iVox 数据结构维护的局部子地图
- 用于可视化和调试

**发布频率**: 与激光雷达频率一致

---

### 3. 配准扫描点云

**话题名称**: `/localization/cloud_registered` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
- 当前帧点云配准到世界坐标系（odom）的结果
- 可用于观察配准质量和点云匹配情况

**发布频率**: 与激光雷达频率一致

---

### 4. 全局地图（可视化）

**话题名称**: `/localization/global_map` (可配置)

**消息类型**: `sensor_msgs/PointCloud2`

**内容说明**:
- 从 PCD 文件加载的全局地图
- **仅用于可视化，不参与 SLAM 计算**
- 通过体素滤波降采样后发布

**发布频率**: 可配置（默认 5 秒一次）

---

### 5. TF 变换

**发布内容**:
- `odom` → `base_link`: 里程计位姿变换
- 可选 `map` → `odom`: 全局定位修正（如使用）

**发布条件**: 配置文件中 `slam/publish_tf: true`

---

## 数据处理方式

### 1. 点云时间戳处理

支持三种模式，通过 `slam/point_time_mode` 配置：

| 模式 | 说明 |
|------|------|
| `auto` | 自动检测第一帧点云是否包含时间戳，后续保持一致 |
| `none` | 强制无时戳模式，启用特殊处理流程 |
| `present`/`curvature` | 强制正常模式，要求点云包含时间戳 |

**无时戳模式处理**:
- 启动阶段降低处理频率（默认 2Hz），确保稳定性
- 收集 20 帧有效里程计后恢复正常频率（10Hz）
- 不使用基于时间的运动补偿

---

### 2. IMU 预热机制

**目的**: 在正式处理点云前，收集足够 IMU 数据用于初始化

**流程**:
1. 启动时开始收集 IMU 数据
2. 同时满足以下条件后完成预热：
   - 持续时间 ≥ `imu_warmup_duration`（默认 1.0 秒）
   - 样本数 ≥ `min_imu_samples`（默认 20 个）
3. 预热完成前丢弃所有激光雷达数据

**效果**: 提高初始姿态估计精度，减少启动漂移

---

### 3. 多传感器融合

**融合策略**:

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│    IMU      │    │   LiDAR     │    │    CAN      │    │    GNSS     │
│  (200Hz+)   │    │   (10Hz)    │    │   (100Hz)   │    │   (10Hz)    │
└──────┬──────┘    └──────┬──────┘    └──────┬──────┘    └──────┬──────┘
       │                  │                  │                  │
       │    高频预测       │    低频修正      │   速度约束       │   姿态校正
       │                  │                  │                  │
       ▼                  ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         Faster-LIO 算法                              │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  IKFoM (Iterated Kalman Filter on Manifold)                 │   │
│  │  - 状态: 位置、速度、姿态、IMU 零偏                          │   │
│  │  - 预测: IMU 积分                                           │   │
│  │  - 更新: 点云配准 + 轮速约束 + GNSS 姿态                     │   │
│  └─────────────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  iVox (Incremental Voxel) 地图                              │   │
│  │  - 增量式体素地图，支持快速近邻查询                          │   │
│  │  - 分辨率: 0.5m (可配置)                                    │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

**轮速计融合**:
- 提取车速信息，根据档位计算带符号速度
- 作为速度观测约束加入卡尔曼滤波
- 异常检测：速度跳变超过阈值时拒绝数据

**GNSS 姿态融合**:
- 提取航向角、俯仰角、横滚角
- 当 SLAM 姿态与 GNSS 差异超过阈值时进行校正
- 通过增益系数控制校正强度

---

### 4. 坐标变换链

```
map (全局地图坐标系)
  │
  │  (全局定位修正，可选)
  ▼
odom (里程计坐标系)
  │
  │  T_odom_base (SLAM 输出)
  ▼
base_link (车辆本体坐标系)
  │
  ├── T_base_imu ──► imu_link (IMU 坐标系)
  │
  └── T_base_lidar ──► lidar_link (激光雷达坐标系)
```

**外参配置**:
- `base_to_imu`: base_link 到 IMU 的变换
- `base_to_lidar`: base_link 到激光雷达的变换
- 支持在线标定（`extrinsic_est_en: true`）

---

## 算法执行流程

### 主循环流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                         主循环 (5000Hz)                              │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │ 1. ros::spinOnce() - 处理传感器回调                            │ │
│  │    - onImu(): IMU 数据缓存 + 预热状态更新                      │ │
│  │    - onLidar(): 点云缓存（预热完成后）                         │ │
│  │    - onCan(): 轮速数据缓存                                     │ │
│  │    - onGnss(): GNSS 姿态缓存                                   │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                              │                                      │
│                              ▼                                      │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │ 2. node.tick() - SLAM 处理                                     │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ a. 频率控制检查 (shouldRunProcessOnce)                │  │ │
│  │    │    - 无点时模式：检查是否满足处理间隔                   │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  │                              │                                 │ │
│  │                              ▼                                 │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ b. SLAM 处理 (slam_->processOnce)                     │  │ │
│  │    │    - 状态预测 (IMU 积分)                              │  │ │
│  │    │    - 点云配准 (iVox 近邻搜索 + ICP)                   │  │ │
│  │    │    - 状态更新 (迭代卡尔曼滤波)                        │  │ │
│  │    │    - 地图更新 (iVox 增量更新)                         │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  │                              │                                 │ │
│  │                              ▼                                 │ │
│  │    ┌───────────────────────────────────────────────────────┐  │ │
│  │    │ c. 结果发布                                          │  │ │
│  │    │    - 坐标变换 (sensor -> base_link)                   │  │ │
│  │    │    - 发布 Odometry 消息                              │  │ │
│  │    │    - 广播 TF 变换                                    │  │ │
│  │    │    - 发布局部地图                                    │  │ │
│  │    │    - 发布配准点云                                    │  │ │
│  │    └───────────────────────────────────────────────────────┘  │ │
│  └───────────────────────────────────────────────────────────────┘ │
│                              │                                      │
│                              ▼                                      │
│  ┌───────────────────────────────────────────────────────────────┐ │
│  │ 3. rate.sleep() - 维持循环频率                                 │ │
│  └───────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

### 单次处理详细流程 (processOnce)

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Faster-LIO 单次处理流程                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  输入: IMU 缓冲区 + 当前点云帧                                       │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 1: 状态预测 (State Prediction)                         │   │
│  │                                                             │   │
│  │  使用 IMU 数据进行状态预测：                                 │   │
│  │  - 位置: p = p + v*dt + 0.5*(R*a + g)*dt²                  │   │
│  │  - 速度: v = v + (R*a + g)*dt                              │   │
│  │  - 姿态: R = R * Exp(ω*dt)                                 │   │
│  │  - 零偏: 保持不变（随机游走模型）                           │   │
│  │                                                             │   │
│  │  输出: 预测状态 x_pred 和协方差 P_pred                       │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 2: 点云预处理 (Point Cloud Preprocessing)              │   │
│  │                                                             │   │
│  │  - 降采样 (point_filter_num)                               │   │
│  │  - 盲区过滤 (blind)                                        │   │
│  │  - 运动补偿（如果有点时间戳）                               │   │
│  │  - 特征提取（平面特征点）                                   │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 3: 迭代卡尔曼更新 (Iterated Kalman Update)             │   │
│  │                                                             │   │
│  │  迭代过程 (max_iteration 次):                              │   │
│  │  ┌───────────────────────────────────────────────────────┐ │   │
│  │  │ for i = 1 to max_iteration:                          │ │   │
│  │  │                                                      │ │   │
│  │  │   1. 计算残差 (Residual)                             │ │   │
│  │  │      - 将点云转换到世界坐标系                        │ │   │
│  │  │      - 在 iVox 地图中查找近邻点                      │ │   │
│  │  │      - 计算点到平面的距离（残差）                    │ │   │
│  │  │                                                      │ │   │
│  │  │   2. 计算雅可比矩阵 (Jacobian)                       │ │   │
│  │  │      - 残差对状态量的偏导数                          │ │   │
│  │  │                                                      │ │   │
│  │  │   3. 卡尔曼增益 (Kalman Gain)                        │ │   │
│  │  │      K = P * H^T * (H * P * H^T + R)^-1              │ │   │
│  │  │                                                      │ │   │
│  │  │   4. 状态更新 (State Update)                         │ │   │
│  │  │      x = x + K * residual                            │ │   │
│  │  │                                                      │ │   │
│  │  │   5. 收敛检查                                        │ │   │
│  │  │      if |delta_x| < threshold: break                 │ │   │
│  │  │                                                      │ │   │
│  │  └───────────────────────────────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ 步骤 4: 地图更新 (Map Update)                               │   │
│  │                                                             │   │
│  │  - 将配准后的点云加入 iVox 地图                            │   │
│  │  - 更新体素网格的统计信息（均值、协方差）                   │   │
│  │  - 移除超出局部范围的旧地图点                              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  输出: 优化后的位姿估计 + 更新后的地图                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 配置文件说明

主要配置文件: `config/slam_odometry_ysw.yaml`

### 关键参数说明

#### 1. 坐标系配置 (`localization/frames`)

```yaml
frames:
  map: "map"              # 全局地图坐标系
  odom: "odom"            # 里程计坐标系
  base_link: "base_link"  # 车辆本体坐标系
  imu: "imu_link"         # IMU 坐标系
  lidar: "lidar_link"     # 激光雷达坐标系
```

#### 2. 传感器外参 (`localization/extrinsics`)

```yaml
extrinsics:
  base_to_imu:
    translation: [0.0, 0.0, 0.0]    # 平移 [x, y, z] (米)
    rotation: [1.0, 0.0, 0.0, ...]  # 3x3 旋转矩阵
  base_to_lidar:
    translation: [0.0, 0.0, 0.0]
    rotation: [1.0, 0.0, 0.0, ...]
```

#### 3. 话题配置 (`sensors`, `outputs`)

```yaml
sensors:
  imu_topic: "/ouster/imu"
  lidar_topic: "/ouster/points"
  wheel_topic: "/can_receive_info"
  gnss_topic: "/gnss_chc_data"

outputs:
  slam_topic: "/localization/slam_odom"
  local_map_topic: "/localization/local_map"
  registered_scan_topic: "/localization/cloud_registered"
  global_map_topic: "/localization/global_map"
```

#### 4. SLAM 核心参数 (`slam/faster_lio`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `scan_rate` | 10 | 激光雷达扫描频率 (Hz) |
| `use_vel` | true | 是否使用轮速计 |
| `use_gnss_attitude` | false | 是否使用 GNSS 姿态 |
| `preprocess/lidar_type` | 3 | 激光雷达类型 (3=Ouster) |
| `preprocess/scan_line` | 64 | 扫描线数 |
| `preprocess/blind` | 3.0 | 盲区距离 (米) |
| `mapping/acc_cov` | 0.1 | 加速度计噪声协方差 |
| `mapping/gyr_cov` | 0.1 | 陀螺仪噪声协方差 |
| `ivox/grid_resolution` | 0.5 | iVox 体素分辨率 (米) |
| `max_iteration` | 5 | 最大迭代次数 |
| `filter_size_surf` | 0.5 | 表面特征滤波尺寸 (米) |

---

## 调试与可视化

### RViz 配置

建议添加以下显示项：

| 显示项 | 话题 | 说明 |
|--------|------|------|
| Odometry | `/localization/slam_odom` | 显示轨迹 |
| PointCloud2 | `/localization/local_map` | 显示局部地图 |
| PointCloud2 | `/localization/cloud_registered` | 显示配准点云 |
| PointCloud2 | `/localization/global_map` | 显示全局地图 |
| TF | - | 显示坐标系变换 |

### 日志输出

关键日志信息：

```
# 初始化完成
[slam_odometry] imu=/ouster/imu lidar=/ouster/points ...
[slam_odometry] use_vel=true use_gnss_attitude=false ...
[slam_odometry] startup_imu_warmup=true duration=1.000s ...

# IMU 预热完成
[slam_odometry] IMU warmup ready: samples=200 duration=1.002s dropped_lidar=10

# 点云时间戳检测
[slam_odometry] point time field detected, use normal Faster-LIO lidar feed
[slam_odometry] point time field is missing, enable no-point-time compatible feed

# 无点时间戳启动完成
[slam_odometry] no-point-time startup done: valid_odom_frames=20, restore process rate to 10.000Hz
```

---

## 常见问题

### Q1: 启动时丢点云数据

**现象**: 日志显示 "drop lidar before IMU warmup"

**原因**: IMU 预热机制在收集足够数据前会丢弃点云

**解决**: 正常现象，等待 IMU 预热完成即可。如需快速启动，可设置 `enable_imu_warmup: false`

---

### Q2: 定位漂移严重

**可能原因**:
1. 外参标定不准确
2. IMU 噪声参数设置不当
3. 场景特征不足（如长走廊、隧道）

**排查方法**:
1. 检查外参配置
2. 调整 `acc_cov` 和 `gyr_cov` 参数
3. 查看退化检测日志

---

### Q3: 无点时间戳模式性能问题

**现象**: CPU 占用高或处理延迟大

**解决**: 调整处理频率参数：
```yaml
slam:
  no_point_time:
    startup_process_rate: 2.0    # 降低启动频率
    normal_process_rate: 10.0    # 降低正常频率
```

---

## 参考资料

1. **Faster-LIO 论文**: 
   - "Faster-LIO: Lightweight Tightly Coupled LiDAR-Inertial Odometry using Parallel Kalman Filter"
   
2. **IKFoM 工具包**:
   - 流形上的迭代卡尔曼滤波实现

3. **iVox 数据结构**:
   - 增量式体素地图，支持 O(1) 近邻查询

---

## 维护信息

- **作者**: DeepLumin Team
- **版本**: 1.0.0
- **最后更新**: 2024

如有问题，请联系维护团队或提交 Issue。