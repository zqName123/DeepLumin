# slam_mapping_v2 可行性分析与设计

## 结论

**可行。** 该架构即 FAST-LIO-SAM / SC-LIO-SAM 的经典两节点方案：
里程计节点（slam_odometry_v2，已完成）输出高频 odom + 配准扫描，
建图节点（slam_mapping_v2）订阅消息后做全局建图、关键帧、GNSS 优化与回环后端。
`slam_mapping` 中关键帧管理、GNSS 优化、回环检测与图优化模块均已存在且可复用，
只需把“内部直接调用 ISlamOdometry”改为“订阅 slam_odometry_v2 的消息”即可。

## 为什么可行：已有基础

| 需求 | 现有资产 | 复用方式 |
|------|----------|----------|
| 关键帧管理（距离/角度/时间阈值） | `slam_mapping/core/keyframe_manager.hpp` | 直接复用 |
| GNSS 后端优化（ENU 对齐、Huber、z 覆盖、白名单） | `slam_mapping/core/gnss_optimizer.hpp` | 直接复用 |
| 回环检测 + GTSAM ISAM2 图优化 | `slam_mapping/core/loop_closure_fastlio2_backend.hpp` | 直接复用 |
| 地图保存（PCD/关键帧/轨迹/回环结果） | `slam_mapping/core/map_saver.hpp` | 直接复用 |
| 可视化发布（path/keyframe/loop/gnss marker） | `slam_mapping/ros/visualization_publisher.hpp` | 直接复用 |
| 全局地图数据结构（iVox / 增量建图） | `slam_odometry_v2/include/faster_lio/ivox3d` + `MapIncremental` 逻辑 | 从 faster_lio_ws 移植，保证 -O3 一致 |
| 里程计输入 | `slam_odometry_v2` 发布的 `/localization/slam_odom` + `/localization/cloud_registered` | 订阅 |

## 架构与数据流

```
IMU/LiDAR/CAN/GNSS
        │
        ▼
slam_odometry_v2（faster_lio_ws 纯里程计，-O3）
   ├─ /localization/slam_odom            nav_msgs/Odometry (odom→base_link)
   ├─ /localization/cloud_registered     PointCloud2（扫描已变换到 odom 系）
   ├─ /localization/local_map            PointCloud2（局部 iVox 地图，调试用）
   └─ TF odom→base_link
        │
        ▼
slam_mapping_v2（新节点，异步后端）
   ├─ 订阅：slam_odom / cloud_registered / gnss(/gnss_chc_data) / can
   ├─ 全局地图：faster-lio 风格 MapIncremental（用 odom 位姿把扫描插入全局 iVox）
   ├─ 关键帧管理：KeyframeManager（阈值触发，存 cloud_body + pose_odom）
   ├─ GNSS 优化：GnssOptimizer（ENU↔map 对齐 + 因子门控）
   ├─ 回环：ScanContext/近邻关键帧检索 + GICP 精配准 + GTSAM ISAM2 图优化
   └─ 输出：/slam_mapping/global_map、优化后 path、关键帧 marker、loop/gnss marker、
            服务（start/stop/save/status）
```

## 与 slam_mapping 的关键差异（改造点）

1. **输入源切换**：slam_mapping 的 `MappingRuntime` 内部创建 `ISlamOdometry` 并直接
   `feedImu/feedLidar`；slam_mapping_v2 改为订阅 odom 消息 + registered_scan，
   用 `odom.pose` 替换内部 `getOdometry()`，用 registered_scan 替换内部 `getCurrentScan()`。
2. **不再需要 IMU 预热/同步逻辑**：同步与去畸变已由里程计节点完成。
3. **建图部分**：以 faster-lio 的 `MapIncremental`（世界系去重增量插入）在全局 iVox 上
   累积点云；该逻辑直接从 `slam_odometry_v2` 移植（同样用 faster_lio_ws 源码，
   避免旧 vendored 版的 -O3 问题）。
4. **坐标系约定**：
   - registered_scan 是 odom 系扫描（节点已按 `T_odom_base * T_base_lidar` 变换）。
   - 全局地图 frame 用 map；首次关键帧建立 map↔odom 初始变换（或直接用 odom 当 map，
     由 GNSS/回环优化后发布 map→odom）。
5. **回环修正的反馈**：ISAM2 优化后得到关键帧 map 系位姿，再回推 map→odom 变换并发布，
   供下游（pose_fusion 等）使用；里程计节点本身不回环，保持高频稳定。

## 消息/线程设计

- 主线程：订阅回调只入队（odom 队列、scan 队列、GNSS 队列），持锁。
- 处理线程（异步，默认 10~20Hz）：取最新 odom+scan → 全局 iVox 增量 → 关键帧判定 →
  关键帧入列 → 后端（GNSS 因子 + 回环检测 + ISAM2）以较低频率（如每 N 个关键帧）运行。
- 发布线程：主循环发布地图预览/path/marker/状态。
- 时间对齐：GNSS 按关键帧时间戳最近邻匹配（复用 GnssOptimizer 的 max_age 门控）；
  odom 与 scan 用同一时间戳（两者同源，无需插值）。

## 关键风险与对策

| 风险 | 对策 |
|------|------|
| 局部地图（里程计内）与全局地图（建图内）重复维护 | 这是标准做法：局部地图仅用于里程计配准；全局地图用于建图/回环。无冲突。 |
| 回环修正后全局地图与里程计轨迹不一致 | 发布 map→odom（或优化后 map 系轨迹），地图点始终用优化后位姿重建/累积；不修改里程计。 |
| registered_scan 量大（10Hz 全分辨率）传输开销 | 里程计已降采样（filter_size_surf）；建图侧再体素滤波（如 0.3~0.5m）后入图。 |
| GNSS 与关键帧时间差、ENU 原点漂移 | 复用 GnssOptimizer：min_spacing、max_age、一致性门控、alignment_frames。 |
| 后端优化阻塞建图线程 | 回环/GNSS 优化放独立后端线程，与建图解耦。 |
| -O3 数值问题复发 | 建图/回环所用 ivox/esekf/GICP 全部取自 faster_lio_ws 源码（经 slam_odometry_v2 验证），不碰旧 vendored 版。 |

## 建议的包结构（slam_mapping_v2）

```
slam_mapping_v2/
├── CMakeLists.txt / package.xml
├── config/slam_mapping_v2.yaml
├── launch/slam_mapping_v2.launch
├── include/
│   ├── slam_mapping_v2/
│   │   ├── common/config.hpp types.hpp
│   │   ├── core/global_map.hpp keyframe_manager.hpp gnss_optimizer.hpp
│   │   │        loop_closure_backend.hpp mapping_runtime.hpp map_saver.hpp
│   │   └── ros/ros_adapter.hpp visualization_publisher.hpp
│   └── faster_lio/（ivox3d 等，取自 faster_lio_ws；或直接依赖 slam_odometry_v2 的 include）
└── src/（core 无 ROS + ros 层分离，结构同 slam_mapping）
```

复用策略建议：直接把 `slam_mapping` 的 core 模块拷入并改输入源（订阅回调 → 构造
`MappingFrame{odom, scan}`），而不是从零重写；global_map 新增，其余模块保持原逻辑。

## 实施步骤（建议顺序）

1. 脚手架：包结构 + 订阅（slam_odom / cloud_registered / gnss）+ 配置加载。
2. 全局地图：移植 faster-lio `MapIncremental` + iVox，按 odom 位姿插入，发布地图预览。
3. 关键帧：复用 KeyframeManager，输出 keyframe 列表与 marker。
4. GNSS 优化：复用 GnssOptimizer，接入关键帧。
5. 回环：复用 Fastlio2SamLcLoopClosureBackend（GTSAM ISAM2），验证 loop 修正后 map→odom。
6. 保存与状态服务：复用 MapSaver + 服务接口。
7. 用 8min.bag 端到端验证：里程计轨迹 vs 建图轨迹 vs GNSS 对比。

## 端到端验证指标

- odom 轨迹与 GNSS 轨迹对齐误差（ATE，米级）
- 回环后位姿漂移收敛（闭环前后轨迹闭合差）
- 全局地图目视质量（道路边界、立柱、标志牌清晰）
- 实时性：里程计 ≥ 10Hz 稳定，建图线程不丢帧
