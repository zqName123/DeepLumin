# slam_odometry_v2

基于 **faster_lio_ws 原始源码** 的纯里程计节点。把 faster-lio 的建图（全局地图累积 / PCD 保存 /
轨迹保存）拆分出去，只保留激光惯性里程计部分；算法代码逐行移植自
`faster_lio_ws/src/faster-lio`，与原始版本在 **-O3 优化编译** 下的运行效果一致。

## 与 slam_odometry 的关系

- 旧版 `slam_odometry` 内嵌的 vendored faster-lio（DeepLumin 改造版）在优化编译下存在
  运行期数值问题，因此 `slam_odometry_v2` **不依赖 slam_odometry 的任何库**，
  全部算法源码取自 `faster_lio_ws`。
- 输入输出与 `slam_odometry` 当前外接形式（`type=fastlio2`）对齐：
  IMU / LiDAR 原始消息直通，CAN 轮速与 GNSS 姿态送入辅助校正，
  输出 odom / local_map / registered_scan / effect_scan / TF。
- 建图部分（全局地图、关键帧、GNSS 优化、回环后端）由 `slam_mapping_v2` 承担，
  通过订阅本节点的 odom / registered_scan 实现。

## 目录结构

```
slam_odometry_v2/
├── CMakeLists.txt                 # -O3 优化编译（与 faster_lio_ws 一致）
├── package.xml
├── config/
│   └── slam_odometry_v2.yaml      # 统一配置：节点层 + faster-lio 算法层（单一文件）
├── launch/
│   └── slam_odometry_v2.launch
├── include/
│   ├── faster_lio/                # faster_lio_ws 算法头文件（原样移植）
│   │   ├── IKFoM_toolkit/         # IKFoM / ESKF
│   │   ├── ivox3d/                # 增量体素地图
│   │   ├── imu_processing.hpp     # IMU 处理与点云去畸变
│   │   ├── pointcloud_preprocess.h
│   │   ├── common_lib.h           # 公共类型（Pose6D 引用已按工程名适配）
│   │   ├── laser_odometry.h       # 纯里程计核心类（新建，裁剪自 laser_mapping.h）
│   │   ├── options.h / so3_math.h / use-ikfom.hpp / utils.h
│   └── slam_odometry_v2/ros/      # 节点配置头
├── src/
│   ├── faster_lio/
│   │   ├── laser_odometry.cc      # 纯里程计实现（裁剪自 laser_mapping.cc）
│   │   ├── pointcloud_preprocess.cc / options.cc / utils.cc  # 原样移植
│   ├── ros/slam_odometry_v2_config.cpp
│   └── slam_odometry_v2_node.cc   # ROS 节点
├── msg/Pose6D.msg                 # faster-lio 消息定义（原样移植）
└── thirdparty/livox_ros_driver/   # Livox 消息依赖（原样移植）
```

## 优化编译（-O3）

- 算法目标 `slam_odometry_v2_core` 显式使用与 faster_lio_ws 相同的优化参数：
  `-O3 -pthread -fexceptions`，**不启用 fast-math、不修改浮点融合**，
  保证数值行为与 faster_lio_ws 的 Release 构建一致。
- 已验证：`catkin_make --pkg slam_odometry_v2` 在 -O3 下编译、链接通过。
- 唯一对移植代码的改动：`common_lib.h` 中 `faster_lio/Pose6D.h` 消息头按本工程名
  改为 `slam_odometry_v2/Pose6D.h`（消息内容相同，仅命名空间随工程名生成），
  算法逻辑零改动。
- 不链接旧 `slam_odometry` / `libfaster_lio.so`，同一进程内不存在同名符号抢占问题。

## 编译与运行

```bash
cd /home/hhy/2004ros/DeepLumin
catkin_make --pkg slam_odometry_v2

# 方式一：roslaunch（自动加载配置并回放 bag）
roslaunch slam_odometry_v2 slam_odometry_v2.launch

# 方式二：直接运行（需额外指定算法配置文件路径）
rosparam load $(rospack find slam_odometry_v2)/config/slam_odometry_v2.yaml
rosparam set /slam_odometry_v2/node/faster_lio_config_yaml $(rospack find slam_odometry_v2)/config/slam_odometry_v2.yaml
rosrun slam_odometry_v2 slam_odometry_v2_node
```

## 输入输出（与 slam_odometry 当前外接形式对齐）

| 方向 | 话题 | 类型 |
|------|------|------|
| 输入 | `/ouster/imu` | sensor_msgs/Imu |
| 输入 | `/ouster/points` | sensor_msgs/PointCloud2（原始直通，faster-lio 自带预处理） |
| 输入 | `/can_receive_info`（use_vel） | deeplumin_msgs/CanReceiveInfo |
| 输入 | `/gnss_chc_data`（use_gnss_attitude） | deeplumin_msgs/Gpchc |
| 输出 | `/localization/slam_odom` | nav_msgs/Odometry（odom → base_link） |
| 输出 | `/localization/local_map` | sensor_msgs/PointCloud2（iVox 局部地图，odom 系） |
| 输出 | `/localization/cloud_registered` | sensor_msgs/PointCloud2（配准扫描，odom 系） |
| 输出 | `/localization/cloud_registered_effect` | sensor_msgs/PointCloud2（有效对应点，odom 系） |
| 输出 | TF odom → base_link | tf |

## 里程计 / 建图拆分

本节点（里程计）：
- IMU/LiDAR 缓冲与时间同步、IMU 初始化与去畸变、IEKF 状态估计、
  iVox **局部地图** 配准与增量、轮速/GNSS 姿态辅助校正。

slam_mapping_v2（建图，另行实现）：
- 订阅 odom + registered_scan，维护**全局地图**（faster-lio 风格 MapIncremental）、
  关键帧管理、GNSS 后端优化、回环检测与图优化、地图/轨迹保存。

## 验证建议

```bash
# 短时回放验证 -O3 运行效果（8min.bag 含 /ouster/imu /ouster/points /can_receive_info）
roslaunch slam_odometry_v2 slam_odometry_v2.launch play_bag:=true
rostopic echo -n 5 /localization/slam_odom
```
