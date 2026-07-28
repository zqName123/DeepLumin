# slam_odometry

基于 Faster-LIO 的 SLAM 里程计模块，为 DeepLumin 定位系统提供实时里程计估计。

## 功能特性

- **Faster-LIO 算法**: 使用增量式 Voxel Octree (iVox) 加速的 LiDAR-IMU 融合 SLAM
- **实时里程计**: 提供高频位姿估计 (10Hz+)
- **点云配准**: 支持 Ouster 64/128 线激光雷达
- **IMU 融合**: 紧耦合 IMU 预积分与 LiDAR 观测
- **TF 发布**: 自动发布 map->odom->base_link 变换
- **全局地图可视化**: 可选发布全局地图用于 RViz 显示

## 依赖

- ROS Noetic
- PCL (Point Cloud Library)
- Eigen3
- deeplumin_msgs

## 编译

```bash
cd /home/hhy/2004ros/DeepLumin
catkin build slam_odometry
```

## 运行

### 快速启动

```bash
roslaunch slam_odometry slam_odometry_ysw.launch
```

### 参数配置

主要配置文件: `config/slam_odometry_ysw.yaml`

#### 关键参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `sensors/imu_topic` | IMU 数据话题 | `/ouster/imu` |
| `sensors/lidar_topic` | 激光雷达话题 | `/ouster/points` |
| `outputs/slam_topic` | 里程计输出话题 | `/localization/slam_odom` |
| `slam/publish_tf` | 是否发布 TF | `true` |
| `slam/point_time_mode` | 点云时间戳模式 | `auto` |

#### Faster-LIO 参数

```yaml
slam:
  faster_lio:
    scan_rate: 10              # 扫描频率
    preprocess:
      lidar_type: 3            # 3=Ouster
      scan_line: 64            # 线数
      blind: 3.0               # 盲区距离
      point_filter_num: 2      # 点云降采样
    mapping:
      acc_cov: 0.1             # 加速度噪声
      gyr_cov: 0.1             # 陀螺仪噪声
      fov_degree: 180.0        # 视场角
      det_range: 150.0         # 检测范围
    ivox:
      grid_resolution: 0.5     # iVox 网格分辨率
```

## 输入输出

### 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ouster/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/ouster/points` | `sensor_msgs/PointCloud2` | 原始点云 |

### 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/localization/slam_odom` | `nav_msgs/Odometry` | SLAM 里程计 |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 配准后点云 |
| `/localization/local_map` | `sensor_msgs/PointCloud2` | 局部地图 |
| `/localization/global_map` | `sensor_msgs/PointCloud2` | 全局地图 (可选) |

### TF 变换

- `map` → `odom` → `base_link`

## 节点说明

### slam_odometry_node

主节点，订阅传感器数据，运行 Faster-LIO 算法，发布里程计和点云。

```bash
rosrun slam_odometry slam_odometry_node
```

## 架构

```
┌─────────────────────────────────────────────────────────────┐
│                    slam_odometry_node                        │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  SensorTopics │  │  ROSConverters│  │  FasterLIO   │      │
│  │   (输入接口)   │  │  (数据转换)   │  │  (SLAM核心)  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│         │                   │                   │           │
│         ▼                   ▼                   ▼           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              SlamOdometryNode (主节点)                │  │
│  │  - 外参管理 (base_to_imu, base_to_lidar)              │  │
│  │  - 坐标系转换                                          │  │
│  │  - TF 发布                                             │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## 调试

### RViz 配置

```bash
roslaunch slam_odometry slam_odometry_ysw.launch
# 在 RViz 中添加:
# - PointCloud2: /localization/cloud_registered
# - Odometry: /localization/slam_odom
```

### 常见问题

1. **IMU 预热失败**
   - 检查 IMU 数据是否正常发布
   - 调整 `imu_warmup_duration` 和 `min_imu_samples`

2. **点云配准发散**
   - 检查 LiDAR-IMU 外参是否正确
   - 调整 `degeneracy_threshold`

3. **TF 跳跃**
   - 检查 `publish_tf` 参数
   - 确保没有其他节点发布冲突的 TF

## 维护者

- DeepLumin Team

## 许可证

Apache-2.0