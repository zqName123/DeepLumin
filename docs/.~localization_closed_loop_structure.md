# DeepLumin Localization Closed Loop Structure

本文档描述当前 DeepLumin localization 的闭环结构：各模块坐标系约定、数据流、时间对齐、全局匹配如何修正融合结果，以及联调时应检查的关键点。

## 1. 总体目标

DeepLumin 定位系统采用工业主定位常见的三层 TF 结构：

```text
map -> odom -> base_link
```

含义：

- `odom -> base_link`：局部连续里程计，短期连续、低延迟，由 DR 递推，SLAM 平滑修正。
- `map -> odom`：全局地图校正量，低频、慢变，由 global_matcher 平滑修正，由 relocalization/initialpose 硬重置。
- `map -> base_link`：最终定位输出，由 `map_odom * odom_base` 合成。

核心公式：

```text
T_map_base(t) = T_map_odom(t) * T_odom_base(t)
```


## 1.1 点云坐标统一入口（方案 A）

在线定位链路中，原始 LiDAR 点云不再被各个 localization 模块分别旋转或修正，而是在入口处统一转换一次：

```text
/ouster/points -> sensor_frame_adapter -> /localization/points
```

`sensor_frame_adapter` 负责把车辆/传感器相关的原始点云坐标约定转换到 DeepLumin localization 标准 `lidar_link`。当前测试数据默认使用 `raw xy -> y,-x`，等价于绕 z 轴 `-90deg`；其他车辆如果原始点云已经满足标准坐标系，可以在 `sensor_frame_adapter.yaml` 中关闭转换或修改 yaw/rpy/translation。

转换原则：

- 只在入口转换一次，避免 global_matcher、slam_odometry、relocalization 各自做重复旋转。
- 输出 frame 固定为 `lidar_link`，后续模块通过 `T_base_lidar` 外参接入闭环。
- `/localization/points` 保留原始 `PointCloud2` 的字段，只修改 xyz 与 header frame，避免丢失 intensity/ring/time 等字段。

## 2. 模块职责

### 2.1 dr_odometry

输入：

- `/ouster/imu`
- `/can_receive_info`
- `/gnss_chc_data`，按策略可用

输出：

- `/localization/dr_odom`
- frame: `odom`
- child_frame: `base_link`
- 语义：`T_odom_base_dr`

职责：

- 提供局部 predictor。
- GNSS 可用时可使用 heading / position / velocity 约束。
- GNSS 不可用时使用 IMU + wheel 维持短时局部运动。

### 2.2 slam_odometry

输入：

- `/localization/points`，由 sensor_frame_adapter 标准化后的 LiDAR 点云
- `/ouster/imu`

输出：

- `/localization/slam_odom`
- frame: `odom`
- child_frame: `base_link`
- 语义：`T_odom_base_slam`

职责：

- 提供 LiDAR-IMU 局部里程计。
- 输出可作为 pose_fusion 的局部观测，对 `odom_base` 做平滑修正。
- 在线全局匹配不再默认使用 `/localization/cloud_registered` 作为 source cloud，避免 SLAM 漂移污染 global_matcher。
- 点云方向和坐标约定由 sensor_frame_adapter 统一处理，SLAM 内部不再额外旋转点云。

### 2.3 global_matcher

输入：

- source cloud：默认 `/localization/points`，由 sensor_frame_adapter 标准化后的 LiDAR 点云
- initial pose：`/localization/fused_odom`，语义 `T_map_base_init`
- source odom：保留接口 `/localization/slam_odom`，仅在 source cloud 为 odom frame 时使用
- global map：PCD 全局地图

当前在线默认模式：

```text
source cloud frame = lidar
source_cloud_is_odom_frame = false
```

初值：

```text
T_map_lidar_init = T_map_base_init * T_base_lidar
```

配准：

```text
T_map_lidar_final = GICP(source_lidar, target_submap, T_map_lidar_init)
```

输出：

```text
T_map_base_gm = T_map_lidar_final * T_lidar_base
```

发布：

- `/localization/global_matcher/source_scan_initial`：用初值变到 map 下的 source cloud。
- `/localization/global_matcher/aligned_scan`：用最终配准结果变到 map 下的 source cloud。
- `/localization/global_matcher/target_submap`：用当前初值裁剪出的地图子图。
- `/localization/global_matcher/result`：当前仍发布 `T_map_base_gm`。

职责：

- 低频提供地图约束。
- 不直接发布最终定位 TF。
- 不直接修改 DR/SLAM 局部里程计。
- 输出由 pose_fusion 转换成 `map_odom` 观测后融合。

### 2.4 relocalization

输出：

- `/localization/relocalization/result`
- 语义：可信的 `T_map_base_reloc`

职责：

- 全局重定位。
- 经过 localization_manager 门控后发布到 `/localization/manager/accepted_relocalization`。
- pose_fusion 收到 accepted relocalization 后硬重置 `map_odom`。

### 2.5 localization_manager

输入：

- `/initialpose`
- relocalization result
- GNSS
- DR/SLAM/global_matcher/fusion 状态

输出：

- `/localization/manager/initial_pose`
- `/localization/manager/observer_policy`
- `/localization/manager/accepted_relocalization`

职责：

- 初始化编排。
- RViz 2D 初值补齐 z 后发布 3D 初值。
- 场景判定和策略下发。
- GNSS 有无和可靠性判定。
- 决定是否使用 global_matcher / relocalization / GNSS。

### 2.6 pose_fusion

输入：

- `/localization/dr_odom`
- `/localization/slam_odom`
- `/localization/global_matcher/result`
- `/localization/manager/initial_pose`
- `/localization/manager/accepted_relocalization`
- `/localization/manager/observer_policy`

内部状态：

```text
state.odom_base = T_odom_base
state.map_odom  = T_map_odom
```

输出：

- `/localization/fused_odom`：`T_map_base`
- `/localization/fused_pose`：`T_map_base`
- `/localization/fused_path`
- TF：`map -> odom`，可选 `odom -> base_link`

职责：

- 用 DR delta 递推 `odom_base`。
- 用 SLAM 观测平滑修正 `odom_base`。
- 用 global_matcher 观测平滑修正 `map_odom`。
- 用 initialpose / relocalization 硬重置 `map_odom`。
- 输出最终 `map_base`。

## 3. Global Matcher 闭环更新

当前 global_matcher 消息仍输出 `T_map_base_gm(t_cloud)`。

pose_fusion 不能直接用当前 `state.odom_base(t_now)` 反解，否则车辆运动时会产生时间错配：

```text
错误形式：T_map_odom = T_map_base_gm(t_cloud) * inverse(T_odom_base(t_now))
```

当前优化后，pose_fusion 保存 `odom_base` 历史，并按 global_matcher result 的 `header.stamp` 查找或插值同一时刻的局部位姿：

```text
T_map_odom_obs(t_cloud)
  = T_map_base_gm(t_cloud) * inverse(T_odom_base(t_cloud))
```

然后通过质量门控和平滑增益更新：

```text
T_map_odom <- interpolate(T_map_odom, T_map_odom_obs, alpha)
```

这样闭环满足：

```text
T_map_base(t_now) = T_map_odom_corrected * T_odom_base(t_now)
```

## 4. 时间策略

系统区分三类时间：

- measurement time：`msg.header.stamp`，用于积分、同步、TF/odom/path 输出。
- receive time：回调时 `ros::Time::now()`，用于模块在线和 timeout 判断。
- wall time：`ros::WallTime` / `steady_clock`，用于耗时统计和工程限频。

Global matcher 到 pose_fusion 的闭环必须使用 measurement time：

```text
GlobalMatchResult.header.stamp == source cloud header.stamp
pose_fusion lookup odom_base at that same stamp
```

模块健康和 timeout 使用 receive time，避免 rosbag 测试中 bag stamp 与系统时间不一致导致误判。

## 5. 坐标系闭环检查

联调时必须满足：

```text
/localization/dr_odom.header.frame_id == odom
/localization/dr_odom.child_frame_id == base_link

/localization/slam_odom.header.frame_id == odom
/localization/slam_odom.child_frame_id == base_link

/localization/global_matcher/result.header.frame_id == map
/localization/global_matcher/result.child_frame_id == base_link

/localization/fused_odom.header.frame_id == map
/localization/fused_odom.child_frame_id == base_link
```

TF 发布责任：

- 融合联调时，pose_fusion 负责 `map -> odom`。
- `odom -> base_link` 只能有一个发布者。
- 若 DR/SLAM 调试节点发布 `odom -> base_link`，pose_fusion 应关闭 `publish_odom_to_base_tf`。
- 若 pose_fusion 发布完整 TF 链，应关闭 DR/SLAM 的 TF。

## 6. 数据同步策略

当前采用异步融合，不要求 DR、SLAM、GM 严格 message_filters 同步。

规则：

- DR：按连续 timestamp 递推。
- SLAM：作为局部 pose 观测进入 pose_fusion，当前为平滑更新。
- GM：用 result timestamp 查 `odom_base` 历史做时间对齐。
- Relocalization / initialpose：作为硬重置事件，下一个或当前 DR 样本用于对齐 `map_odom`。

## 7. 当前实现边界

已完成：

- 新增 sensor_frame_adapter，在线链路统一 `/ouster/points -> /localization/points`。
- global_matcher 在线输入改为标准 LiDAR 点云 `/localization/points`。
- global_matcher source cloud timeout 使用 receive time。
- pose_fusion 保存 `odom_base` 历史。
- global_matcher result 进入 pose_fusion 时按 `header.stamp` 查找 `T_odom_base(t)`。
- initialpose / relocalization / DR / SLAM / GM 的坐标职责明确。

仍建议后续增强：

- 扩展 `GlobalMatchResult`，显式携带 `map_odom` 和 `source_odom_stamp`，减少下游反解歧义。
- pose_fusion 的 SLAM 更新也按时间历史与 DR 对齐，而不是只和当前状态比残差。
- 建立统一 ObservationDebug，发布每次接受/拒绝观测的时间差、残差、增益。
- localization_manager 根据状态动态调整 global_matcher 频率。
