# LIO Odometry 设计文档

## 1. 概述

### 1.1 系统定位

LIO Odometry（`lio_odometry`）是一个紧耦合 LiDAR-Inertial 里程计算节点，基于 Faster-LIO / lightning-lm-ros1 的算法核心移植而来。它提供纯前端里程计功能：以 IMU 和 LiDAR 原始数据为输入，输出 6-DOF 位姿估计（帧率 = LiDAR 频率），同时维护一个 iVox 局部地图用于帧间配准。

与 `slam_odometry_v2` 的关系：
- **算法等价**：ESKF 状态维度（23维）、观测模型（点到面）、IMU 处理流程均等价
- **实现差异**：`slam_odometry_v2` 使用 MTK/esekfom 模板元编程实现 ESKF；`lio_odometry` 使用手写 ESKF（`NavState` 结构体 + 显式雅可比）
- **特色功能**：`lio_odometry` 支持 Anderson Acceleration（AA）加速 IEKF 收敛，可选轮速/GNSS 辅助观测
- **输出对齐**：话题名、坐标系、帧率与 `slam_odometry_v2` 完全一致

### 1.2 设计原则

1. **ROS 解耦**：核心算法零 ROS 依赖，ROS 节点层仅负责订阅/发布/参数读取
2. **轻量高效**：不维护全局地图或关键帧数据库，仅保持 iVox 局部地图
3. **数值鲁棒**：面向退化场景（转弯、快速运动、几何结构不足）做了特殊设计
4. **可扩展**：通过 `CustomObservationModel` 回调机制支持多种观测源（LiDAR、轮速、GNSS）

---

## 2. 系统架构

### 2.1 整体架构

```
┌────────────────────────────────────────────────────────────────────┐
│                        lio_odometry 节点                             │
│                                                                     │
│  ┌─────────────────┐      ┌──────────────────────────────────────┐ │
│  │ ROS 节点层        │      │  C++ 核心库（零 ROS 依赖）             │ │
│  │ (RosAdapter)     │      │                                      │ │
│  │                  │      │  ┌────────────────────────────────┐  │ │
│  │ ros::Subscriber  │─────▶│  │ LaserOdometryCore              │  │ │
│  │  - /ouster/imu   │      │  │  ├── ESKF (23维, 手写)         │  │ │
│  │  - /ouster/points│      │  │  ├── ImuProcess               │  │ │
│  │  - /can_receive  │      │  │  ├── PointCloudPreprocess      │  │ │
│  │  - /gnss_chc     │      │  │  ├── AndersonAcceleration      │  │ │
│  │  - /correction   │      │  │  ├── ObsModel (LiDAR点面)      │  │ │
│  │                  │      │  │  ├── ObsModelWelspeed          │  │ │
│  │ ros::Publisher   │◀─────│  │  ├── ObsModelGPS               │  │ │
│  │  - /slam_odometry│      │  │  └── iVox LocalMap (复用)      │  │ │
│  │    /odom         │      │  └────────────────────────────────┘  │ │
│  │  - /slam_odometry│      └──────────────────────────────────────┘ │
│  │    /local_map    │                                                │
│  └─────────────────┘                                                │
└────────────────────────────────────────────────────────────────────┘
```

### 2.2 目录结构

```
src/localization/lio_odometry/
├── include/lio_odometry/
│   ├── core/
│   │   ├── laser_odometry_core.h       // 主算法类
│   │   ├── eskf.h                      // ESKF 头文件
│   │   ├── types.h                     // 核心类型定义
│   │   ├── imu_processing.h            // IMU 处理
│   │   ├── pointcloud_preprocess.h     // 点云预处理
│   │   ├── anderson_acceleration.h     // Anderson 加速
│   │   └── correction_handler.h        // 校正回传（可选）
│   └── ros/
│       ├── ros_adapter.h               // ROS 适配层
│       └── config.h                    // 配置加载
├── src/
│   ├── core/
│   │   ├── laser_odometry_core.cpp     // 主循环 + 观测模型 + 地图管理
│   │   ├── eskf.cpp                    // ESKF 预测/更新
│   │   ├── imu_processing.cpp          // IMU 初始化/去畸变
│   │   ├── pointcloud_preprocess.cpp   // 点云预处理
│   │   ├── pcl_instantiate.cpp         // PCL 模板实例化
│   │   └── correction_handler.cpp      // 校正处理器
│   └── ros/
│       ├── ros_adapter.cpp             // ROS 消息转换
│       ├── config.cpp                  // YAML 配置
│       └── lio_odometry_node.cpp       // 节点入口
├── config/lio_odometry.yaml           // 默认配置
├── launch/lio_odometry.launch         // 启动文件
├── docs/
│   ├── design.md                       // 本文档
│   ├── porting_design.md              // 移植设计方案
│   └── startup_drift_fix.md           // 启动漂移修复记录
├── CMakeLists.txt
└── package.xml
```

---

## 3. 核心算法原理

### 3.1 ESKF（Error State Kalman Filter）

#### 3.1.1 状态向量定义

23 维状态向量 `x ∈ R²³` 由以下部分组成：

| 索引 | 维度 | 符号 | 含义 | 流形类型 |
|------|------|------|------|----------|
| 0-2  | 3    | `pos` | 位置（IMU 系在世界系） | 向量空间 |
| 3-5  | 3    | `rot` | 姿态（SO3 旋转向量） | SO3 流形 |
| 6-8  | 3    | `offset_R_lidar_` | LiDAR-IMU 旋转外参（SO3） | SO3 流形 |
| 9-11 | 3    | `offset_t_lidar_` | LiDAR-IMU 平移外参 | 向量空间 |
| 12-14| 3    | `vel` | 速度（世界系） | 向量空间 |
| 15-17| 3    | `bg` | 陀螺零偏 | 向量空间 |
| 18-20| 3    | `ba` | 加速度计零偏 | 向量空间 |
| 21-22| 2    | `grav` | 重力向量（S2 流形） | S2 流形 |

#### 3.1.2 状态传播（Predict）

IMU 动力学模型：

```
ṗ   = v
v̇   = R * (a_m - ba) + g
Ṙ   = R * [ω_m - bg]×
bġ  = 0
bȧ  = 0
ġ   = 0
```

离散化预测步骤（`ESKF::Predict`）：

```cpp
// 连续时间状态方程
f(x) = [v, ω, 0, 0, a_inertial+g, 0, 0, 0]^T

// 离散化：四阶 Runge-Kutta / 欧拉积分
x_{k+1} = x_k ⊕ (f(x_k) * Δt)

// 协方差传播
F_x = df/dx  (24×23 雅可比)
P_{k+1} = F_x * P_k * F_x^T + F_w * Q * F_w^T
```

关键设计点：
- SO3 流形上的雅可比使用 `A_matrix`（MTK 约定）进行切空间变换
- S2 流形（重力）使用 2 维切空间参数化，`S2_Bx` / `S2_Mx` / `S2_Nx_yy` 提供雅可比
- 预测步骤使用分段积分（最大步长 0.01s），适应 IMU 数据间隙

#### 3.1.3 IEKF 更新（Update）

迭代扩展卡尔曼滤波（IEKF）用于处理 LiDAR 点到面残差的非线性：

对于每次迭代 `i`：

```
1. 在当前估计 x_i 处线性化观测模型：
   h(x_i + δx) ≈ h(x_i) + H_i * δx

2. 计算卡尔曼增益：
   K_i = P_i * H_i^T * (H_i * P_i * H_i^T + R)^{-1}

3. 更新误差状态：
   δx_i = K_i * (z - h(x_i)) + (K_i * H_i - I) * (x_i ⊖ x_0)

4. 更新名义状态：
   x_{i+1} = x_i ⊕ δx_i

5. 更新协方差：
   P_{i+1} = (I - K_i * H_i) * P
```

其中 `x_0` 是迭代起始（传播后）状态，`⊖` 是流形上的减法（`boxminus`），`⊕` 是流形上的加法（`boxplus`）。

#### 3.1.4 协方差重置

收敛后，需将协方差从迭代起始点的切空间变换到最终状态的切空间：

```cpp
// SO3 状态的雅可比变换
J_SO3 = A_matrix(dx_segment).transpose()
// S2 状态的雅可比变换
J_S2 = Nx * before_update.grav.S2_Mx(dg)
// 后验协方差
P_final = J * (I - K*H) * P * J^T
```

**重要**：协方差重置使用的增量 `dx_current` 必须是 **KF 步的增量**，而不是 AA 之后的实际落点增量。使用 AA 后的增量会导致协方差被过度放大，使后续帧的卡尔曼增益偏小，降低收敛速度。

### 3.2 Anderson Acceleration（AA）

Anderson Acceleration 是一种加速定点迭代收敛的方法，用于 IEKF 中加速外循环收敛。

#### 3.2.1 原理

对于定点迭代 `x_{k+1} = g(x_k)`，AA 维护一个长度为 `m` 的历史窗口，利用历史信息构造更好的下一个估计：

```
F_i = g(x_i) - x_i           // 残差向量
dF_i = F_i - F_{i-1}         // 残差变化
dG_i = g(x_i) - g(x_{i-1})  // 迭代变化

求解最小二乘：min_θ || F_k - Σθ_i * dF_i ||
得到 θ* 后计算加速步：
x_acc = g(x_k) - Σθ_i * dG_i
```

AA 在 IEKF 中的位置：

```
       KF step
x_i ──────────▶ x_kf(i+1) ──▶ AA compute ──▶ x_{i+1}
                    ↑                          │
                    └──────────────────────────┘
                  (历史窗口缓存)
```

#### 3.2.2 收敛判据

收敛判据使用 `dx_current`（KF 步增量，不是 AA 步增量）的每个分量是否小于阈值 `limit_`：

```cpp
for (int j = 0; j < 23; j++) {
    if (std::fabs(dx_current[j]) > limit_[j]) {
        converge = false;
        break;
    }
}
```

**关键设计决策**：必须使用 KF 步增量而非 AA 步增量判收敛。AA 步的实际落点增量通常比 KF 步大，用它判收敛更难满足，导致迭代次数跑满，增加数值抖动风险。

### 3.3 平面拟合（QR A·n = -1）

#### 3.3.1 算法原理

给定 `N` 个近邻点 `{p_i}`，拟合平面 `n·p = d`。使用 QR 分解求解超定方程：

```
[x_i, y_i, z_i] · n = -1    (i = 1, ..., N)

A(N×3) * n(3×1) = b(N×1)
其中：A_i = [x_i, y_i, z_i], b_i = -1
```

求解 `n` 后归一化，得到平面法向量和距离：
```
n_normalized = n / ||n||
d = 1 / ||n||
```

#### 3.3.2 内点判据

点到面距离超过 `threshold`（默认 0.1m）的匹配点被拒绝：

```cpp
for (const auto& p : point) {
    if (fabs(n_normalized · [p, 1]) > threshold) {
        return false;  // 点不在平面上
    }
}
```

**与原版 PCA 版的比较**：

| 特性 | QR A·n=-1（当前） | PCA 特征分解（原移植版） |
|------|-------------------|------------------------|
| 原理 | 解超定方程，最小化代数距离 | 协方差矩阵特征分解 |
| 额外检查 | 无 | λ₀ > 0.1·λ₁ 和 λ₁ < 1e-6 |
| 对运动涂抹的容忍度 | 高 | 低 |
| 退化场景鲁棒性 | 好 | 差 |

PCA 版的额外特征值检查在转弯时尤为致命：角速度大时预测位姿有偏差，iVox 取回的 5 个点是"墙面 + 轻微运动涂抹"的混合。QR 版容忍这种涂抹、照样给出约束把状态拉回地图；PCA 版判定特征值超标直接拒点，导致 IEKF 失去恢复力。

### 3.4 iVox 体素地图

#### 3.4.1 数据结构

iVox（Incremental Voxel）是一种高效的空间哈希体素地图，支持增量式点云插入和最近邻查询。

每个体素节点 `IVoxNode` 维护一个 **最多 10 个点** 的容器：

```cpp
void IVoxNode::InsertPoint(const PointT& pt) {
    points_.emplace_back(pt);
    if (points_.size() >= 10) {
        points_.erase(points_.begin());  // 移除最早点
    }
}
```

这个 10 点上限是**关键设计**：
- **地图语义**："近期点云"——KNN 仅在最近落入该体素的 10 个点里选最近 5 个
- **效率**：KNN 线性扫描始终有限，不会随运行时间增长
- **准确性**：避免老点累积形成跨期涂抹，减少因匹配点跨期导致的特征值超标

#### 3.4.2 邻近查询

```
Nearest neighbor search:
1. 计算目标点所属体素坐标 (grid_coord = floor(p / resolution))
2. 根据 nearby_type (6/18/26) 收集邻域体素
3. 在每个体素内线性扫描，用 max_heap 维护 K 个最近邻
4. 返回 K 个最近的点及其所属节点指针
```

### 3.5 IMU 处理

#### 3.5.1 IMU 初始化

初始化的目标是估计：
- **重力方向**：`grav = -mean_acc / |mean_acc| * G`
- **陀螺零偏**：`bg = mean_gyr`
- **协方差初值**：对角阵，位置/姿态 1e-5，速度 1.0，偏置 1e-4

初始化条件是累积 `kMaxInitCount`（20 帧）IMU 数据，**不要求静止**。这样设计的目的是允许在运动中启动（如机器人在车上被唤醒）。

#### 3.5.2 点云去畸变

```
流程：
1. 以 last_imu 为起点，将所有 IMU 数据传播到对应时间点
2. 每两个相邻 IMU 测量之间取平均值作为驱动
3. 保存每个 IMU 时刻的状态（pose, vel, acc, gyro）
4. 帧尾外推：使用最后一个 IMU 区间的 head/tail 平均值（非瞬时值）
5. 对每个激光点，根据其时间戳在 IMU 状态间插值补偿

关键细节：
- 首帧 offset_time = 0.0（不是 imu_state.timestamp - pcl_beg_time）
- 反向循环比较符使用 > 而非 >=（避免 t=0 点被错误补偿）
- 帧尾外推使用平均角速度而非瞬时角速度，减少急转时的拖影
```

**帧尾外推的设计选择**：

```cpp
// ✅ 正确：使用最后一个 IMU 区间的平均值（与原版一致）
acc = acc_avr;      // 0.5 * (head.acc + tail.acc) * G/|mean_acc|
gyro = angvel_avr;  // 0.5 * (head.gyro + tail.gyro)

// ❌ 错误：使用瞬时值（原移植版）
acc = v_imu.back()->linear_acceleration * G / |mean_acc|;
gyro = v_imu.back()->angular_velocity;
```

瞬时值在急转时引入的单帧噪声会传递到扫描末端点的补偿中，产生细微拖影。平均值提供了更平滑的外推。

#### 3.5.3 IMU Gap 处理

当 IMU 数据间隔超过 `1.0s` 时：
- 设置 `imu_gap = true` 标志（仅用于日志，不拒绝扫描）
- 预测步骤仍继续执行（使用分步积分，`max_step=0.01s`）
- 点云保持去畸变处理

这种方式避免了因短暂 IMU 断流而丢弃整帧点云。

---

## 4. 详细流程

### 4.1 主循环（`LaserOdometryCore::Run()`）

```
每帧处理流程：

SyncPackages()
    ├─ 从 lidar_buffer_ / imu_buffer_ 同步一组数据
    ├─ 计算扫描时长（从点时间或历史均值）
    └─ 返回 false（数据不足）→ 退出

ImuProcess::Process(meas, kf_, scan_undistort)
    ├─ IMU 初始化（如果需要）
    └─ UndistortPcl：IMU 传播 + 点云去畸变

第一帧？
    ├─ Yes → 初始化 iVox 地图，返回
    └─ No  → 继续

点云降采样（VoxelGrid，filter_size = 0.2m）
    └─ 点数 < 5 → 返回 false

ESKF::Update(LIDAR, R)
    │  ObsModel() 回调：
    │      ├─ 计算世界坐标
    │      ├─ iVox 最近邻查询（5点）
    │      ├─ esti_plane() 平面拟合（QR A·n=-1）
    │      ├─ 点到面距离计算
    │      ├─ 鲁棒核（pd2 < p_body^2/81）
    │      └─ H 矩阵构建（12列）
    │
    ├─ Update 拒绝？
    │   └─ Yes → 恢复传播状态，跳过地图插入，返回
    │
    └─ Update 接受 → 状态更新

可选：轮速观测 / GNSS 姿态观测

MapIncremental()
    ├─ 对每个有效点判断是否需要加入地图
    └─ ivox_->AddPoints()

可选：局部地图范围剔除（map_trim_radius）
```

### 4.2 观测模型（`ObsModel`）

#### 4.2.1 LiDAR 观测构建

```
输入：当前帧降采样点集 (scan_down_body_)
输出：点到面残差 + H 矩阵

for each point p_body:
    1. p_world = R_wl * p_body + t_wl
       (R_wl = R * R_LI, t_wl = R * t_LI + t)
    
    2. nearest_points = ivox->GetClosestPoint(p_world, K=5)
    
    3. if 近邻数 ≥ MIN_NUM_MATCH_POINTS (3):
         plane_coef = esti_plane(nearest_points)
         if 平面有效:
             pd2 = plane_coef · [p_world, 1]
             if pd2 < p_body.norm^2 / 81 (鲁棒核):
                 有效匹配 ← 保留
                 residual = -pd2
    
    4. H 矩阵构建（每行 12 列）:
       h_x[i] = [norm, A, B, C]
         norm = R^T * n              (3列: 位置)
         A    = (R_LI*p + t_LI) × C (3列: 姿态)
         B    = p_body × R_LI^T * C  (3列: 外参旋转, 可选)
         C    = R^T * n              (3列: 外参平移)

若有效匹配点数 < 1 → 观测无效
```

#### 4.2.2 鲁棒核函数

点面距离的鲁棒性检查：

```cpp
bool valid_corr = p_body.norm() > 81.0f * pd2 * pd2;
```

这个条件是点到面距离的 Huber-like 核：
- 对小距离点（面内点）给予全权重
- 对大距离点（离群点）自动剔除
- 阈值与点距离原点的距离（`p_body.norm()`）成反比，即对远处的点更宽松

#### 4.2.3 H 矩阵设计

```
H (N_eff × 12) 的 12 列对应：
列 0-2: ∂r/∂pos = n              (法向量在世界系)
列 3-5: ∂r/∂rot = p_body × C     (力矩/旋转)
列 6-8: ∂r/∂offset_R = ...       (外参旋转, 仅extrinsic_est_en时非零)
列 9-11: ∂r/∂offset_t = C        (外参平移)

残差 r = -pd2 (点到面的带符号距离)
```

### 4.3 地图管理（`MapIncremental`）

```
for each scan_down_body point:
    1. 转换到世界坐标 p_world
    
    2. 计算所属体素中心 center
    
    3. 该点在最近邻中被选中的概率决定是否插入：
       - 如果最近邻中已有 5 个点且都更靠近体素中心 → 跳过
       - 否则 → 加入 points_to_add
    
    4. ivox_->AddPoints(points_to_add)
    
    5. （可选）每隔 N 帧，移除距当前位置超过 map_trim_radius 的远点
```

地图插入的保护机制：
- **Update 拒绝时不插入地图**（差异 3 的重要修复）：当 ESKF Update 返回 false 时，跳过 `MapIncremental()`，避免纯预测位姿下的错配点污染地图
- **Update 接受后不重算 ObsModel**：收敛后不再重算对应关系，避免因短暂有效点不足而错误拒帧

---

## 5. 关键设计决策

### 5.1 平面拟合：QR A·n = -1 而非 PCA

**决策**：使用 QR 分解解超定方程 `A·n = -1`，而不是 PCA 特征分解 + 特征值检查。

**原因**：PCA 版的额外特征值检查（`λ₀ > 0.1·λ₁` 和 `λ₁ < 1e-6`）在转弯时过于严格。转弯角速度大时预测位姿有偏差，iVox 取回的 5 个点是"墙面 + 轻微运动涂抹"的混合。QR 版容忍这种涂抹、照样给出约束把状态拉回地图；PCA 版判定特征值超标直接拒点，形成"预测差 → 拒点 → 预测更差"的正反馈。

**影响**：这是纯 LiDAR 场景下转弯性能的根本决定因素。

### 5.2 iVox 体素 10 点上限

**决策**：`IVoxNode::InsertPoint` 中保持每体素最多 10 点，超过时删除最早点。

**原因**：
1. **地图语义**：原版 KNN 仅在"最近落入该体素的 10 个点"里选最近 5 个，地图本质上是"近期点云"
2. **效率**：KNN 线性扫描始终有限，不会随运行时间增长
3. **准确性**：避免老点累积形成跨期涂抹，减少特征值超标概率

### 5.3 ESKF 拒绝策略

**决策**：采用宽松的拒绝策略，与原版 lightning-lm 一致：

| 场景 | 原版 / 当前实现 | 原移植版（已修复） |
|------|----------------|-------------------|
| 迭代中观测无效 | `continue`（下一迭代重找最近邻） | `return reject()`（整帧回滚） |
| 收敛后 | 直接接受 | 重算 ObsModel + 检查有效点 |
| P 对角负值 | 不检查 | 检查并拒帧 |
| 拒绝后入图 | 跳过 MapIncremental | 照常入图 |

**影响**：宽松的拒绝策略避免转弯时整帧丢更新形成连锁。`continue` 允许下一迭代重新找最近邻，通常能自愈。

### 5.4 AA 收敛判据使用 KF 增量

**决策**：收敛判据和协方差重置都使用 AA **之前**的 KF 步增量 `dx_current`。

**原因**：AA 后的实际落点增量通常比 KF 步大。用它判 1e-3 收敛门槛更难满足 → `converged_times` 难累计 → 6 次迭代经常跑满 → 病态帧抖动 → 触发拒绝。

### 5.5 IMU 外推使用区间均值

**决策**：帧尾外推 IMU 驱动使用最后一个区间的 head/tail 平均值，而非最后一个 IMU 的瞬时值。

**原因**：瞬时值在急转时引入的单帧噪声传递到扫描末端点补偿中，产生细微拖影。平均值提供平滑外推，与帧间 IMU 传播的一致性更高。

### 5.6 首帧 offset 恒为 0

**决策**：首个 `imu_pose` 的 `offset_time` 恒为 0.0。

**原因**：使用 `imu_state.timestamp - pcl_beg_time` 会导致：
- 正偏移：扫描最早 1~2ms 的点完全不补偿
- 负偏移（首帧边界）：t=0 的点被施加错误补偿

---

## 6. 问题与解决方案

### 6.1 启动垂直漂移

**现象**：启动后前几秒垂直方向出现明显漂移，即使 IMU 初始化正确。

**根因链**：
1. IMU 初始化过于严格（要求静止），运动中启动时初始化失败
2. `keep_first_imu_estimation_` 保护帧数过少（5 帧），未初始化完成时就允许更新
3. 外参在线估计在初始姿态不准时收敛到错误值
4. 协方差重置用错增量导致后续帧增益偏小

**解决方案**：
1. 简化 IMU 初始化（移除静止检测），允许运动中启动
2. 增大保护帧数（5 → 30/100 帧）
3. 移除 IMU 零值过滤，避免有效数据被丢弃
4. 修复协方差重置使用 KF 增量

### 6.2 转弯飘飞

**现象**：纯 LiDAR 模式下，转弯角速度大时位姿突发漂移，甚至完全发散。

**根因链**（按严重程度）：
1. **(★★★)** 平面拟合使用 PCA + 特征值检查 → 转弯时拒掉太多有效点
2. **(★★★)** iVox 体素无限增长 → 老点跨期涂抹 → 特征值超标
3. **(★★☆)** ESKF 拒绝过于激进 → 整帧丢更新
4. **(★★☆)** AA 收敛判据用错增量 → 迭代跑满 → 病态抖动

**解决方案**：
1. PCA → QR A·n=-1（容忍运动涂抹，保留约束）
2. 恢复 iVox 10 点上限（恢复近期点云语义）
3. 宽松 ESKF 拒绝策略（`continue` 而非 `return reject()`）
4. AA 收敛判据使用 KF 增量

### 6.3 IMU 断流导致跳帧

**现象**：IMU 数据短暂中断（>0.1s）后，系统直接跳过整帧点云处理。

**解决方案**：
1. IMU gap 阈值放宽到 1.0s（与原版一致）
2. gap 处理改为"仅警告 + 继续处理"，保留原始点云进行去畸变
3. 预测步骤使用分步积分（`max_step=0.01s`）适应 gap

### 6.4 地图污染

**现象**：ESKF Update 被拒绝后，纯预测位姿下的错配点仍然被插入 iVox 地图，导致下一帧更难匹配。

**解决方案**：拒绝后检查返回值，跳过 `MapIncremental()`：
```cpp
if (!lidar_accepted) {
    kf_.ChangeX(old_state);  // 恢复传播状态
    state_point_ = old_state;
    return true;              // 不执行 MapIncremental()
}
```

---

## 7. 配置参数说明

### 7.1 核心参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lidar_type` | 3 | 1=AVIA, 2=VELO32, 3=OUST64 |
| `scan_line` | 64 | 激光雷达线数 |
| `blind` | 1.2 | 盲区距离（米），盲区内点不处理 |
| `point_filter_num` | 1 | 点云采样间隔（1=全采） |
| `max_iteration` | 6 | IEKF 最大迭代次数 |
| `filter_size_scan` | 0.2 | 帧降采样体素大小（米） |
| `filter_size_map` | 0.5 | 地图插入时去重体素大小（米） |
| `ivox_grid_resolution` | 0.2 | iVox 体素分辨率（米） |
| `ivox_nearby_type` | 18 | 邻近体素搜索模式（6/18/26） |
| `esti_plane_threshold` | 0.1 | 平面拟合点到面距离阈值（米） |
| `use_aa` | true | 是否启用 Anderson Acceleration |
| `extrinsic_est_en` | true | 是否在线估计 LiDAR-IMU 外参 |
| `lidar_update_r` | 0.001 | LiDAR 观测噪声协方差 |

### 7.2 IMU 噪声参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `acc_cov` | 0.1 | 加速度计噪声 |
| `gyr_cov` | 0.1 | 陀螺噪声 |
| `b_acc_cov` | 0.0001 | 加速度计偏置随机游走 |
| `b_gyr_cov` | 0.0001 | 陀螺偏置随机游走 |

### 7.3 地图管理参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_trim_radius` | 150.0 | 局部地图剔除半径（米），0=不剔除 |
| `map_trim_interval_frames` | 10 | 每 N 帧执行一次剔除 |
| `skip_lidar_num` | 0 | 点云跳帧（0=不跳） |

### 7.4 辅助观测参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_wheel_speed` | false | 是否启用轮速观测 |
| `use_gnss` | false | 是否启用 GNSS 姿态观测 |
| `wheel_velocity_noise` | 0.33 | 轮速噪声 |
| `gnss_attitude_noise` | 0.03 | GNSS 姿态噪声 |

---

## 8. 与 slam_odometry_v2 对比

### 8.1 算法等价性

| 维度 | slam_odometry_v2 (MTK/esekfom) | lio_odometry (手写 ESKF) |
|------|--------------------------------|--------------------------|
| 状态维度 | 23 (state_ikfom) | 23 (NavState) |
| 预测方程 | MTK 模板自动微分 | 手写 get_f/df_dx/df_dw |
| 更新方程 | esekfom::update_iterated_dyn | ESKF::Update (显式迭代) |
| 观测模型 | h_share_model (函数指针) | ObsModel (回调函数) |
| AA 支持 | 无 | 支持（可选） |
| 平面拟合 | QR A·n=-1 | QR A·n=-1（已修复对齐） |
| iVox 上限 | 10 点/体素 | 10 点/体素（已修复对齐） |

### 8.2 性能差异

- **计算效率**：手写 ESKF 更便于优化和调试（无模板元编程编译开销）
- **数值行为**：收敛路径一致，检验同一套观测模型和噪声参数
- **退化鲁棒性**：当前已完全对齐（6 个差异均修复）

---

## 9. 调试与诊断

### 9.1 关键日志

```
// 初始化完成
[ImuProcess] initialized: mean_acc=... gravity=... gyro_bias=...

// 每帧统计（每 2 秒输出一次）
[odometry]: In num: N down M Map grid num: K effect num: E

// IMU 断流警告
[ImuProcess] IMU gap > 0.1s; scan may be incompletely undistorted
```

### 9.2 性能指标

- `effect_feat_num`（有效匹配点数）：正常场景 > 100，退化场景可能 < 20
- `final_res_`（归一化残差）：< 0.5 表示收敛良好
- `iterations_`（实际迭代次数）：收敛时 2-3 次，病态时 6 次
- `ivox_->NumValidGrids()`（地图规模）：随运行时间增长，`map_trim_radius` 控制上限

### 9.3 常见问题诊断

| 现象 | 检查点 |
|------|--------|
| 启动时漂移 | IMU 初始化日志、`keep_first_imu_estimation_` 帧数 |
| 转弯飘飞 | `esti_plane` 实现（PCA vs QR）、iVox 上限、effect num 骤降 |
| 逐帧抖动 | AA 是否启用、收敛判据增量类型、P 对角负值 |
| 越跑越慢 | iVox 体素上限、map_trim_radius 配置 |
| 地图污染 | Update 拒绝后是否跳过 MapIncremental |

---

## 10. 文件引用

| 文件 | 说明 |
|------|------|
| [laser_odometry_core.cpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/src/core/laser_odometry_core.cpp) | 主算法：主循环、ObsModel、MapIncremental |
| [laser_odometry_core.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/include/lio_odometry/core/laser_odometry_core.h) | 主算法头文件：类型定义、参数声明 |
| [eskf.cpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/src/core/eskf.cpp) | ESKF 预测与更新实现 |
| [eskf.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/include/lio_odometry/core/eskf.h) | ESKF 类型定义 |
| [imu_processing.cpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/src/core/imu_processing.cpp) | IMU 初始化与点云去畸变 |
| [imu_processing.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/include/lio_odometry/core/imu_processing.h) | IMU 处理头文件 |
| [types.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/include/lio_odometry/core/types.h) | 核心类型：NavState, S2, SO3, math, Pose6D |
| [anderson_acceleration.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/include/lio_odometry/core/anderson_acceleration.h) | Anderson Acceleration 模板实现 |
| [ivox3d_node.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/slam_odometry_v2/include/faster_lio/ivox3d/ivox3d_node.hpp) | iVox 体素节点（10 点上限） |
| [lio_odometry.yaml](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry/config/lio_odometry.yaml) | 默认配置文件 |