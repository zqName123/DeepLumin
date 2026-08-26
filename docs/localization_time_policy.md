# DeepLumin Localization Time Policy

本文档定义 DeepLumin localization 各模块的时间使用原则，目标是在 rosbag 测试和实车运行中保持一致行为，避免因为时间基准混用导致数据超时、TF_OLD_DATA、TF_REPEATED_DATA 或模块误判掉线。

## 1. 时间类型

定位系统内必须区分三类时间。

### 1.1 观测时间 measurement time

来源：ROS 消息的 `msg.header.stamp`。

用途：

- IMU 积分、轮速/GNSS/点云同步。
- DR、SLAM、pose_fusion 的相对运动计算。
- odometry、path、动态点云、TF 的对外输出时间戳。
- 多传感器按采样时刻排序。

原则：

- 只要消息 `header.stamp` 有效，状态估计必须使用该时间。
- 不允许用节点接收时间替代传感器采样时间做积分或同步。
- `header.stamp == 0` 只允许在 RViz `/initialpose` 等人工输入场景回退到当前 ROS 时间。

### 1.2 接收时间 receive time

来源：消息进入当前节点回调时的 `ros::Time::now()`。

用途：

- 判断某个输入话题是否还在持续到达。
- manager 判断 DR、SLAM、fusion、global_matcher、relocalization、GNSS 等模块是否在线。
- global_matcher 判断 source cloud 是否超时。
- pose_fusion 诊断状态中的 last_*_age。

原则：

- 模块活性和数据流新鲜度优先使用 receive time。
- 不应使用 `ros::Time::now() - msg.header.stamp` 判断模块是否在线，因为 bag 测试时 wall time 与 bag stamp 可能不同。
- 同一条消息应同时保存 `measurement_time` 和 `receive_time`。

### 1.3 墙钟时间 wall time

来源：`ros::WallTime` 或 `std::chrono::steady_clock`。

用途：

- CPU 处理限频。
- 性能耗时统计。
- 无点时间字段点云场景下的工程节流。

原则：

- wall time 不参与定位状态、传感器同步、TF 时间戳。
- wall time 只描述程序运行节奏，不描述车辆运动时刻。

## 2. rosbag 测试策略

推荐测试方式：

```bash
rosparam set /use_sim_time true
rosbag play --clock /path/to/data.bag
```

测试 launch 应默认提供 `clock:=true` 或明确暴露该参数。

如果 `/use_sim_time=false` 播放旧 bag：

- 传感器消息 `header.stamp` 是 bag 记录时间。
- `ros::Time::now()` 是当前系统 wall time。
- 若直接用 `now - header.stamp` 做超时判断，会产生几十天甚至更大的 age，导致误报 timeout。

因此模块在线判断必须使用 receive time，才能兼容忘记开启 `/use_sim_time` 的测试场景。

## 3. 实车运行策略

实车运行通常使用：

```bash
/use_sim_time=false
```

要求：

- LiDAR、IMU、GNSS、CAN 等输入消息的 `header.stamp` 来自统一时钟源。
- 推荐使用 PTP、GPS time 或统一系统时间。
- 如果不同传感器时间源不一致，需要在驱动层或时间同步层修正，localization 模块不应隐式猜测跨传感器时差。

实车中 receive time 与 measurement time 通常接近，但仍需分开保存：

- measurement time 用于状态估计。
- receive time 用于判断 topic 是否持续到达。

## 4. 各模块约定

### 4.1 dr_odometry

- IMU `dt` 使用 IMU `header.stamp`。
- 轮速/GNSS 与 IMU 的新鲜度判断使用传感器 measurement time 差值。
- 对外 odom/path/TF 使用估计状态的 measurement time。
- 节点健康状态可使用 receive time 评估数据流是否在线。

### 4.2 slam_odometry

- LiDAR/IMU 同步使用 measurement time。
- 点云无逐点时间字段时，仍以雷达帧 `header.stamp` 作为扫描时刻。
- 无点时间字段的启动限频可以使用 wall time。
- 对外 slam odom、registered scan、local map 使用 SLAM 状态对应的 measurement time。

### 4.3 global_matcher

- source cloud、source odom、initial pose 都保留 measurement time。
- source cloud 是否超时默认使用 receive time。
- 配准结果、aligned scan、target submap、initial guess 的输出 stamp 使用 source cloud measurement time。
- GICP/ICP 耗时统计可使用 ROS time 或 wall time，推荐后续改为 wall time。

### 4.4 pose_fusion

- 融合递推和观测排序使用 measurement time。
- last_dr_age、last_slam_age、last_global_matcher_age、last_relocalization_age 使用 receive time。
- fused odom/path/TF 使用融合输出对应的 measurement time 或当前融合 tick 的 ROS time，后续应统一为状态最后更新时间。

### 4.5 localization_manager

- 场景判断中 GNSS 可靠性应同时考虑 GNSS measurement time 和 receive time。
- 模块在线/超时判断应使用 receive time。
- `/initialpose` 若 stamp 为 0，可回退到 receive time。

## 5. 常见问题判断

### 5.1 source cloud timeout

如果已经能 echo 到 `/localization/cloud_registered`，但 global_matcher 报：

```text
source cloud timeout
```

优先检查：

- `/use_sim_time` 是否为 true。
- rosbag 是否使用 `--clock`。
- global_matcher 是否启用了 `runtime/use_receive_time_for_cloud_age`。

### 5.2 TF_OLD_DATA

典型原因：

- 使用 wall time 运行 RViz，但 TF 使用旧 bag stamp。
- 同一个 TF frame 被多个节点同时发布。
- bag 循环播放或回退时间后 TF buffer 中已有较新的同 frame 数据。

处理原则：

- bag 测试使用 `/use_sim_time=true` 和 `rosbag play --clock`。
- 每条 TF 链只保留一个发布者。
- 循环播放 bag 前重启相关 TF 发布节点或清空测试环境。

### 5.3 TF_REPEATED_DATA

典型原因：

- 同一 frame 在同一 stamp 被重复发布。
- 多个节点发布同名 parent-child TF。

处理原则：

- fusion 测试时由 pose_fusion 统一发布 `map->odom` 与 `odom->base_link`。
- DR/SLAM 的 TF 发布在融合联调 launch 中关闭。

## 6. 开发检查清单

新增 localization 模块或 topic 时，必须检查：

- 是否同时保存 measurement time 和 receive time。
- 状态估计是否只使用 measurement time。
- 在线/超时判断是否使用 receive time。
- 输出 odom/path/TF 的 stamp 是否与坐标状态对应。
- bag 测试 launch 是否设置 `/use_sim_time=true` 且 rosbag 使用 `--clock`。
- 日志中是否能打印 time mismatch，方便区分“数据没到”和“时间基准不一致”。
