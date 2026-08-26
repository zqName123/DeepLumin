# DeepLumin `slam_mapping_node` 详细设计

## 1. 文档目标

本文档定义 `DeepLumin/localization` 中 `slam_mapping_node` 的详细设计。  
该节点的目标是：

- 作为建图模式下的核心算法节点
- 尽量复用 `FASTLIO2_SAM_LC_ws` 中 `map_builder_node` 的成熟流程
- 对上与 `mapping_manager` 形成稳定交互接口
- 对下复用 `slam_odometry` / Faster-LIO 核心
- 对外发布 GUI 可直接消费的实时建图结果

这里的设计重点不是重新发明一套建图算法，而是：

1. 明确复用哪些现有流程
2. 明确 DeepLumin 版本的节点职责边界
3. 明确与 `mapping_manager` 的接口
4. 明确实时输出与保存结果的组织方式

## 2. 设计原则

`slam_mapping_node` 设计应遵循以下原则：

1. 尽量复用已有建图流程，不重复造轮子
2. 算法核心与任务控制解耦
3. 保存流程可由外部触发，也支持安全收尾
4. GUI 所需的可视化输出必须明确、稳定
5. 关键帧和回环能力按“可选增强”设计，不强行第一版全量承诺

## 3. 参考实现复用策略

## 3.1 直接参考对象

推荐主要参考：

```text
FASTLIO2_SAM_LC_ws/src/FASTLIO2_SAM_LC/src/map_builder_node.cpp
```

该节点已经具备：

- 主线程 LIO 建图
- 关键帧管理
- 回环线程
- 图优化更新关键帧全局位姿
- `/save_map` 服务保存地图
- shutdown 自动保存
- 路径和回环 Marker 发布

这是当前最适合直接迁移思路的参考实现。

## 3.2 建议复用的能力

建议 `slam_mapping_node` 复用以下流程思想：

### 主线程流程

- 传感器同步
- LIO 建图更新
- 发布局部建图结果
- 关键帧判定与插入

### 子线程流程

- 回环候选搜索
- 子图构建
- ICP/配准验证
- 图优化
- 更新关键帧全局位姿

### 保存流程

- 服务触发保存地图
- 根据关键帧和关键帧点云重建最终地图
- 可在节点关闭时自动保存

### 可视化流程

- 发布当前地图点云
- 发布当前扫描点云
- 发布局部轨迹和全局轨迹
- 发布回环可视化 Marker

## 3.3 不建议直接照搬的部分

以下内容不建议原样照搬：

1. 原工程的 topic 命名
2. 原工程的 frame 命名
3. 原工程的服务定义
4. 原工程中与 DeepLumin 无关的局部兼容逻辑
5. 原工程内的业务性参数命名

DeepLumin 版本应统一到 `/mapping/*` 命名空间和自己的消息/服务体系。

## 4. 在 DeepLumin 系统中的位置

建议系统结构如下：

```text
mapping_manager
    |
    +-- start mapping runtime
    |
    +-- interact with slam_mapping_node
            |
            +-- LIO mapping main thread
            +-- optional loop closure thread
            +-- result exporter
            +-- runtime status publisher
```

节点定位：

- `slam_mapping_node` 是建图 runtime 中最核心的算法节点
- 它不是 GUI 的直接控制对象
- 它由 `mapping_manager` 拉起、管理和发指令

## 5. 节点职责定义

`slam_mapping_node` 的职责建议限定为 5 类。

## 5.1 在线建图

- 订阅 IMU、LiDAR
- 执行 LIO 前端建图
- 维护局部地图和实时位姿

## 5.2 关键帧管理

- 按位移/旋转阈值插入关键帧
- 保存关键帧局部位姿
- 保存关键帧点云缓存

## 5.3 可选回环与图优化

- 回环候选搜索
- 回环验证
- 生成回环约束
- 优化关键帧全局位姿
- 输出回环 Marker

## 5.4 实时结果发布

- 地图点云
- 当前扫描
- 当前轨迹
- 当前位姿
- 关键帧轨迹
- 回环可视化
- runtime 状态

## 5.5 结果保存

- 保存最终全局地图 PCD
- 保存轨迹文件
- 可选保存关键帧点云
- 可选保存关键帧位姿
- 可选保存回环边或优化结果

## 6. 与 `mapping_manager` 的边界

## 6.1 `mapping_manager` 负责

- 环境检测
- 启停 runtime
- 启停录包
- 管理任务状态机
- 管理输出目录
- 汇总结果路径
- 向 GUI 发布统一状态

## 6.2 `slam_mapping_node` 负责

- 算法运行
- 关键帧维护
- 回环检测与优化
- 输出实时地图
- 保存建图结果
- 上报算法运行状态和能力信息

## 6.3 为什么这样划分

这样划分有三个好处：

1. `mapping_manager` 不侵入算法内部
2. `slam_mapping_node` 不关心 GUI 和业务状态机
3. 后续更换算法时 GUI 和 manager 基本不变

## 7. 推荐内部架构

建议 `slam_mapping_node` 内部按如下组织：

```text
slam_mapping_node
├── sensor adapter / sync
├── mapping core adapter
├── keyframe manager
├── loop closure worker
├── result publisher
├── save/export module
└── runtime state reporter
```

如果第一版追求速度，可以先做单文件节点；但从长期维护看，建议拆层。

## 8. 内部数据结构建议

参考 `map_builder_node`，建议保留类似共享结构：

```text
SharedMappingData
  - keyframes
  - keyframe_clouds
  - loop_pairs
  - loop_history
  - global_offset / map_local transform
```

建议核心数据结构：

### `MappingKeyFrame`

包含：

- `id`
- `timestamp`
- `local_pose`
- `global_pose`
- `cloud_body`
- 可选 `scan_context`

### `LoopConstraintData`

包含：

- `from_id`
- `to_id`
- `relative_pose`
- `score`
- `accepted`

### `RuntimeStateData`

包含：

- `state`
- `frame_count`
- `keyframe_count`
- `point_count`
- `loop_count`
- `save_in_progress`
- `last_error`

## 9. 线程模型设计

建议直接复用 `mapping_builder` 的线程思想。

## 9.1 主线程

频率建议：`10~20Hz` 或与传感器频率同步。

职责：

1. 读取并同步 IMU / LiDAR
2. 调用 LIO core 更新位姿和地图
3. 更新当前点云输出
4. 判断是否插入关键帧
5. 发布实时地图与轨迹
6. 更新 runtime 状态

## 9.2 回环线程

频率建议：`0.5~2Hz`。

职责：

1. 监听是否有新关键帧插入
2. 搜索历史关键帧候选
3. 构造子图
4. 进行 ICP/匹配验证
5. 若成功，生成回环约束
6. 执行图优化
7. 更新关键帧全局位姿
8. 发布回环可视化信息

## 9.3 保存流程线程

建议保存过程不要完全阻塞主线程。可以有两种方式：

### 方案 A：停止后同步保存

- 收到停止保存请求
- 主循环退出
- 在退出阶段保存

优点：简单。  
缺点：保存时间长时，状态反馈较差。

### 方案 B：单独保存线程

- 收到保存请求后切状态为 `SAVING`
- 冻结关键数据快照
- 子线程完成保存

优点：状态更清晰，更适合与 `mapping_manager` 联动。  
建议 DeepLumin 采用这一方案。

## 10. 运行状态机设计

建议 `slam_mapping_node` 内部维护自己的 runtime 状态，而不是只依赖进程是否活着。

推荐状态：

```text
UNINITIALIZED
INITIALIZING
READY
RUNNING
STOPPING
SAVING
FINISHED
ERROR
```

### 状态说明

- `UNINITIALIZED`
  节点刚启动，尚未完成参数和资源初始化

- `INITIALIZING`
  正在等待首帧有效数据或关键模块 ready

- `READY`
  已具备进入建图运行的条件

- `RUNNING`
  正在进行建图

- `STOPPING`
  已收到停止请求，准备结束建图

- `SAVING`
  正在保存地图结果

- `FINISHED`
  保存成功结束

- `ERROR`
  运行中出现不可恢复错误

## 11. 输入接口设计

## 11.1 传感器输入

建议输入 topic：

- `/localization/points` 或 `/ouster/points`
- `/ouster/imu`
- 可选 `/can_receive_info`

建议 DeepLumin 建图模式仍优先走：

```text
/ouster/points -> sensor_frame_adapter -> /localization/points
```

这样坐标系入口统一，与定位链路保持一致。

## 11.2 来自 `mapping_manager` 的控制输入

建议通过 service 控制，不建议纯 topic。

推荐服务：

- `/mapping/runtime/start_session`
- `/mapping/runtime/stop_and_save`
- `/mapping/runtime/get_status`

### `start_session` 作用

- 传入当前任务参数
- 传入输出目录
- 传入是否启用回环
- 传入是否启用关键帧导出

### `stop_and_save` 作用

- 请求结束当前建图任务
- 请求保存结果
- 指定保存项和保存路径

### `get_status` 作用

- 查询当前 runtime 状态
- 查询计数信息
- 查询能力标志

## 12. 输出接口设计

## 12.1 实时数据输出

建议统一到 `/mapping/*` 命名空间。

### Topic 建议

- `/mapping/odom`
- `/mapping/current_scan`
- `/mapping/map_points`
- `/mapping/path_local`
- `/mapping/path_global`
- `/mapping/keyframe_markers`
- `/mapping/loop_markers`
- `/mapping/runtime_status`
- `/mapping/runtime_capabilities`

### 对应 GUI 价值

- `map_points`：实时地图显示
- `current_scan`：当前扫描显示
- `path_local / path_global`：轨迹显示
- `loop_markers`：回环效果显示
- `runtime_status`：状态栏和日志面板

## 12.2 保存接口

建议提供显式保存服务：

- `/mapping/runtime/save_map`
- 或由 `stop_and_save` 内部统一完成

参考 `map_builder_node` 现有 `/save_map` 服务，建议 DeepLumin 版本扩展为不仅保存 PCD，还保存其他结果。

## 13. 保存结果设计

## 13.1 必需保存项

第一版建议必需支持：

1. `global_map.pcd`
2. `task_metadata.json`
3. `task_config.yaml`

## 13.2 推荐保存项

1. `trajectory.tum`
2. `runtime_summary.json`

## 13.3 可选保存项

1. `keyframes/<id>.pcd`
2. `keyframe_poses.txt`
3. `loop_edges.json`
4. `optimized_poses_tum.txt`

## 13.4 为什么关键帧导出应做成可选

虽然 `map_builder_node` 已经具备：

- `key_poses`
- `cloud_history`

因此 DeepLumin 实现关键帧导出并不难，但仍建议作为“可选启用”原因如下：

1. 导出数据量较大
2. 有些场景只关心最终地图 PCD
3. GUI 不应强制要求该结果一定存在
4. `mapping_manager` 应根据 runtime 能力决定是否要求导出

## 14. 关键帧管理设计

建议基本复用 `map_builder_node` 的关键帧插入策略：

- 与上一关键帧相比，若平移超过阈值
- 或姿态变化超过阈值
- 则插入关键帧

建议参数：

- `keyframe/dist_thresh`
- `keyframe/rad_thresh`
- `keyframe/time_thresh`

每个关键帧保存：

- 局部位姿 `local_pose`
- 当前全局优化位姿 `global_pose`
- body 系去畸变点云 `cloud_body`

保存地图时：

- 使用 `global_pose` 将 `cloud_body` 变换到 map 系
- 拼接成最终 `global_map.pcd`

## 15. 回环设计

## 15.1 第一版建议

如果希望快速落地，建议直接参考 `map_builder_node` 的回环流程：

1. 关键帧位姿 KD-Tree 搜索
2. 空间邻近 + 时间阈值筛候选
3. 构建当前子图和历史子图
4. ICP 验证
5. 满足阈值则加入回环约束
6. ISAM2 优化
7. 更新关键帧全局位姿

## 15.2 DeepLumin 适配建议

推荐参数命名统一到：

- `loop_closure/enabled`
- `loop_closure/search_radius`
- `loop_closure/time_thresh`
- `loop_closure/index_gap_thresh`
- `loop_closure/submap_resolution`
- `loop_closure/submap_search_num`
- `loop_closure/icp_fitness_thresh`

## 15.3 GUI 展示接口

建议直接输出：

- `visualization_msgs/MarkerArray` 回环边
- 回环统计状态 topic

这样 GUI 可直接嵌 RViz 或用 bridge 转发。

## 16. 与 `mapping_manager` 的交互时序

## 16.1 开始建图

```text
mapping_manager -> start runtime process
mapping_manager -> /mapping/runtime/start_session
slam_mapping_node -> INITIALIZING
slam_mapping_node -> READY / RUNNING
slam_mapping_node -> publish runtime_status
mapping_manager -> observe RUNNING
```

## 16.2 结束建图

```text
mapping_manager -> /mapping/runtime/stop_and_save
slam_mapping_node -> STOPPING
slam_mapping_node -> SAVING
slam_mapping_node -> save outputs
slam_mapping_node -> FINISHED
mapping_manager -> validate outputs
mapping_manager -> publish task finished
```

## 16.3 异常处理

```text
runtime crash / save fail / sensor timeout
slam_mapping_node -> ERROR or process exit
mapping_manager -> detect failure
mapping_manager -> publish FAILED
```

## 17. 能力标志设计

建议 `slam_mapping_node` 启动后发布能力标志：

- `loop_supported`
- `keyframe_export_supported`
- `trajectory_export_supported`
- `autosave_supported`
- `save_map_service_supported`

这样 `mapping_manager` 和 GUI 都能做能力自适应。

## 18. 配置设计建议

建议新增独立配置：

```text
DeepLumin/src/localization/slam_mapping/config/slam_mapping.yaml
```

建议包含：

### 传感器输入

- `sensors/imu_topic`
- `sensors/lidar_topic`

### 帧命名

- `frames/map`
- `frames/local`
- `frames/base_link`
- `frames/lidar`

### LIO 核心参数

- 参考 `mapping_builder` 的 `lio_builder/*`
- 或对齐 `slam_odometry` 的 Faster-LIO 参数

### 关键帧参数

- `keyframe/dist_thresh`
- `keyframe/rad_thresh`
- `keyframe/time_thresh`

### 回环参数

- `loop_closure/*`

### 保存参数

- `save/default_resolution`
- `save/export_keyframes`
- `save/export_keyframe_poses`
- `save/export_trajectory`
- `save/export_loop_edges`
- `save/autosave_on_shutdown`

## 19. 为什么推荐新增 `slam_mapping_node` 而不是复用 `slam_odometry_node`

虽然二者都可以复用同一套 LIO 核心，但职责不同：

### `slam_odometry_node`

- 面向定位链路
- 强调局部连续位姿
- 强调低侵入输出

### `slam_mapping_node`

- 面向建图任务
- 强调关键帧、回环、全局优化
- 强调结果保存与导出
- 强调与 `mapping_manager` 的任务交互

如果强行把所有功能堆进 `slam_odometry_node`，会导致：

- 定位与建图逻辑耦合
- 参数混乱
- 状态机混乱
- 后续维护困难

因此新增建图专用节点是更合理的。

## 20. 第一版实现建议

建议分两步。

## 20.1 第一步：快速复用版

目标：尽快跑通建图闭环。

建议做法：

1. 以 `map_builder_node` 为蓝本实现 `slam_mapping_node`
2. 优先保持以下能力：
   - 实时建图
   - 关键帧缓存
   - 地图保存
   - 轨迹发布
   - 回环 Marker 发布
3. 优先做与 `mapping_manager` 的最小服务接口

## 20.2 第二步：DeepLumin 规范化版

目标：与 DeepLumin 现有 localization 体系对齐。

建议做法：

1. 统一 topic 命名到 `/mapping/*`
2. 统一 frame 约定
3. 统一服务风格到 `deeplumin_msgs`
4. 拆出 `keyframe_manager` / `loop_closure_worker` / `save_exporter`
5. 与 `mapping_manager` 的状态和能力消息完全对齐

## 21. 结论

`slam_mapping_node` 的最佳设计路线不是从零重新设计，而是：

- 明确把 `FASTLIO2_SAM_LC_ws` 的 `map_builder_node` 当作直接参考原型
- 将其成熟的建图、关键帧、回环、保存流程迁移到 DeepLumin
- 在外层补齐 `mapping_manager` 所需的服务接口、状态消息和能力标志

这样做的最大好处是：

1. 建图算法流程成熟可参考
2. `mapping_manager` 易于对接
3. GUI 易于消费实时输出
4. 关键帧、回环、保存功能自然成体系

## 22. 下一步建议

在这份详细设计之后，最适合继续做的是：

1. `slam_mapping_node` 所需消息与服务定义
2. `slam_mapping_node` 类图与源码骨架
3. `mapping_runtime.launch` 设计
4. `mapping_manager` 与 `slam_mapping_node` 的接口协议细化

如果你要，我下一步可以直接继续给出：

- `slam_mapping_node` 的类结构和源码骨架设计
- 或者直接开始在 `DeepLumin/src/localization` 下搭建 `slam_mapping` 包的代码骨架
