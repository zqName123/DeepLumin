# `slam_mapping` 改造开发任务清单

## 1. 文档目标

本文档将当前 `DeepLumin/src/localization/slam_mapping` 的问题分析转化为可执行的开发任务清单。

输出目标：

- 给出明确的改造优先级
- 标注每项任务的重要程度
- 标注每项任务的容易实施性
- 标注每项任务的可行性
- 给出建议改动点
- 给出最小验收标准

本文档面向工程实施，不再重复算法背景说明。

---

## 2. 评级说明

### 2.1 重要程度

- `P0`：必须优先解决，否则结果不稳定、不可信或不可复现
- `P1`：强烈建议解决，直接影响建图质量、回环稳定性或在线体验
- `P2`：中期优化项，提升上限、扩展性和大场景能力

### 2.2 容易实施性

- `高`：局部修改即可完成，风险可控
- `中`：跨 2-4 个模块，需联调
- `低`：涉及架构重构、核心算法链路或大量验证

### 2.3 可行性

- `高`：当前代码基础已具备，直接迭代即可
- `中`：可做，但需要补接口、补状态机或补评测
- `低`：需要较大重构或引入新依赖

---

## 3. 总体建议

建议按照以下顺序实施：

1. 先修“配置与状态可解释性”
2. 再修“启动稳定性与时间同步观测”
3. 再修“保存链路正确性”
4. 再修“回环候选与图优化鲁棒性”
5. 最后再做“descriptor / submap / 长时任务优化”

原因很直接：

- 当前 `slam_mapping` 的主要短板不是“前端完全不能跑”
- 而是“很多问题发生后不好解释、不好复现、不好验证”

---

## 4. P0 任务

## 4.1 任务 A：补全 `slam_mapping` 参数加载

- 重要程度：`P0`
- 容易实施性：`高`
- 可行性：`高`

### 问题

当前 `slam_mapping` 有多项回环参数定义了但没有从 YAML 加载，导致：

- 配置文件写了不生效
- 跨数据集排查结果不可信
- 回环行为和预期不一致

现状代码见：

- [param_loader.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_mapping/src/ros/param_loader.cpp:41)

### 任务内容

补充加载以下参数：

- `loop_closure/type`
- `loop_closure/optimizer`
- `loop_closure/time_thresh_sec`
- `loop_closure/odom_rot_noise`
- `loop_closure/odom_pos_noise`
- `loop_closure/loop_noise_scale`

必要时同步补：

- `loop_closure` 当前生效配置的启动日志

### 影响文件

- `DeepLumin/src/localization/slam_mapping/src/ros/param_loader.cpp`
- `DeepLumin/src/localization/slam_mapping/include/slam_mapping/common/config.hpp`
- `DeepLumin/src/localization/slam_mapping/config/*.yaml`

### 验收标准

- 修改 YAML 后，运行日志能打印所有关键回环参数
- `time_thresh_sec` 修改后，回环触发行为发生可观测变化
- 单元测试或最小集成测试能覆盖至少 1 个新增参数

---

## 4.2 任务 B：为 `slam_mapping` 增加显式 IMU warmup / startup gating

- 重要程度：`P0`
- 容易实施性：`中`
- 可行性：`高`

### 问题

当前底层 `ImuProcess` 会进行 IMU 初始化，但 `slam_mapping` 外层没有显式 warmup 状态管理。

结果是：

- 节点已经启动，但未必已进入稳定计算
- 用户看到 `RUNNING`，不代表前端已准备完成
- 预热阶段 LiDAR 帧处理结果不透明

参考现有更完善实现：

- [slam_odometry_node.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_odometry/src/slam_odometry_node.cpp:67)
- [slam_odometry_node.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_odometry/src/slam_odometry_node.cpp:173)

### 任务内容

在 `slam_mapping` 外层引入：

- `enable_imu_warmup`
- `imu_warmup_duration`
- `min_imu_samples`

建议行为：

- warmup 未完成时不进入正式建图
- 状态接口显式输出 `warming_up`
- 记录 warmup 持续时间、累计 IMU 数、启动前丢弃或缓存的 LiDAR 数

### 影响文件

- `DeepLumin/src/localization/slam_mapping/include/slam_mapping/common/config.hpp`
- `DeepLumin/src/localization/slam_mapping/src/ros/param_loader.cpp`
- `DeepLumin/src/localization/slam_mapping/src/core/mapping_runtime.cpp`
- `DeepLumin/src/localization/slam_mapping/src/ros/ros_adapter.cpp`

### 验收标准

- 启动后在 IMU 不足时状态不是正常 `running`
- IMU 达到门限后自动切换到正式建图
- 状态消息可看见 warmup 统计信息

---

## 4.3 任务 C：补齐时间同步和去畸变状态观测

- 重要程度：`P0`
- 容易实施性：`中`
- 可行性：`高`

### 问题

当前底层同步机制可用，但外层缺少有效观测。

底层逻辑见：

- [laser_mapping_core.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_odometry/src/slam/laser_mapping_core.cpp:249)
- [imu_processing.hpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_odometry/thirdparty/faster_lio/include/imu_processing.hpp:169)

关键问题：

- 点时间字段不可靠时会退回 `lidar_mean_scantime`
- 外层无法知道当前帧到底用了哪种同步分支
- 跨数据集问题难以定位

### 任务内容

将以下信息纳入状态或调试输出：

- `sync_branch`
- `lidar_scan_dt`
- `lidar_mean_scantime`
- `imu_wait_gap`
- `imu_used_count`
- `lidar_drop_count`
- 是否使用真实点时间

推荐做法：

- 高频信息保留 throttle log
- 关键摘要进入 `MappingStatus`

### 影响文件

- `DeepLumin/src/localization/slam_odometry/src/slam/laser_mapping_core.cpp`
- `DeepLumin/src/localization/slam_mapping/include/slam_mapping/common/types.hpp`
- `DeepLumin/src/localization/slam_mapping/src/ros/ros_converters.cpp`
- `DeepLumin/src/localization/slam_mapping/src/ros/visualization_publisher.cpp`

### 验收标准

- 运行中可判断当前帧同步分支
- 数据集切换后可快速定位是否因点时间退化导致问题
- 至少能解释“为什么某数据集回环不触发”

---

## 4.4 任务 D：修复地图/关键帧/轨迹保存成功判定

- 重要程度：`P0`
- 容易实施性：`高`
- 可行性：`高`

### 问题

当前只要 `map.pcd` 保存成功，其它保存失败也可能返回整体成功。

现状代码见：

- [map_saver.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_mapping/src/core/map_saver.cpp:30)

### 任务内容

需要分别检查：

- `saveKeyframes`
- `saveKeyframePoses`
- `saveTrajectory`

建议：

- `SaveResult` 增加失败子项摘要
- 对部分失败返回 `false` 或返回“部分成功”状态

### 影响文件

- `DeepLumin/src/localization/slam_mapping/src/core/map_saver.cpp`
- `DeepLumin/src/localization/slam_mapping/include/slam_mapping/common/types.hpp`
- `DeepLumin/src/localization/slam_mapping/src/ros/ros_converters.cpp`

### 验收标准

- 任一子项落盘失败时，调用方能收到明确错误
- GUI 和服务日志能看到失败原因

---

## 5. P1 任务

## 5.1 任务 E：回环候选策略从“首个命中”升级为“排序后验证”

- 重要程度：`P1`
- 容易实施性：`中`
- 可行性：`高`

### 问题

当前在候选搜索后直接选第一个满足时间/索引条件的候选帧：

- [loop_closure_fastlio2_backend.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_mapping/src/core/loop_closure_fastlio2_backend.cpp:259)

这会带来：

- 候选质量不稳定
- 容易漏掉更优回环对象
- 重复结构场景鲁棒性差

### 任务内容

建议实现：

- 对半径搜索结果排序
- 选择 top-k 进入 ICP
- 增加次优候选抑制
- 增加姿态差约束或简单先验过滤

### 影响文件

- `DeepLumin/src/localization/slam_mapping/src/core/loop_closure_fastlio2_backend.cpp`

### 验收标准

- 至少支持 `top_k_candidates` 参数
- 相同数据集多次运行回环触发更稳定
- 误检率明显下降或可观测下降

---

## 5.2 任务 F：回环优化结果反馈到在线全局输出

- 重要程度：`P1`
- 容易实施性：`中`
- 可行性：`中高`

### 问题

当前 `pose_map` 优化后只影响关键帧和最终保存结果，不影响在线 `current_odom_` 输出。

现状链路见：

- [mapping_runtime.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_mapping/src/core/mapping_runtime.cpp:129)

### 任务内容

建议引入双路输出：

- `odom_local`：前端连续位姿
- `odom_global`：pose graph 修正后的全局位姿

同时明确：

- GUI 默认看哪一路
- 地图保存使用哪一路

### 影响文件

- `DeepLumin/src/localization/slam_mapping/src/core/mapping_runtime.cpp`
- `DeepLumin/src/localization/slam_mapping/include/slam_mapping/common/config.hpp`
- `DeepLumin/src/localization/slam_mapping/src/ros/visualization_publisher.cpp`

### 验收标准

- 回环触发后在线全局轨迹可见修正效果
- 保存地图与在线全局路径一致

---

## 5.3 任务 G：优化关键帧策略

- 重要程度：`P1`
- 容易实施性：`中`
- 可行性：`高`

### 问题

当前关键帧仅依赖：

- 距离阈值
- 旋转阈值
- 时间阈值
- 最小点数

现状见：

- [keyframe_manager.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_mapping/src/core/keyframe_manager.cpp:11)

这对不同运动模式和退化场景不够稳。

### 任务内容

建议新增：

- 最小时间间隔
- 最大时间间隔
- 原地转向模式
- 低速抖动抑制
- 可选固定频率关键帧

### 影响文件

- `DeepLumin/src/localization/slam_mapping/include/slam_mapping/common/config.hpp`
- `DeepLumin/src/localization/slam_mapping/src/core/keyframe_manager.cpp`
- `DeepLumin/src/localization/slam_mapping/config/*.yaml`

### 验收标准

- 不同数据集下关键帧增长曲线更稳定
- 不再出现明显“关键帧过密”或“长时间不产关键帧”现象

---

## 5.4 任务 H：保存产物增加 metadata / 配置快照

- 重要程度：`P1`
- 容易实施性：`高`
- 可行性：`高`

### 问题

当前保存目录缺少实验自描述信息。

### 任务内容

建议在输出目录中额外保存：

- `run_summary.json` 或 `run_summary.yaml`
- 当前配置快照
- 启停时间
- 关键帧数
- 回环数
- 传感器 topic
- 外参摘要

### 影响文件

- `DeepLumin/src/localization/slam_mapping/src/core/map_saver.cpp`
- `DeepLumin/src/localization/slam_mapping/src/core/mapping_runtime.cpp`

### 验收标准

- 单独查看输出目录即可知道这份地图如何生成
- 便于批量实验比对

---

## 6. P2 任务

## 6.1 任务 I：引入 descriptor 回环前端

- 重要程度：`P2`
- 容易实施性：`中低`
- 可行性：`高`

### 问题

当前回环前端基本等于：

- 半径搜索
- ICP 验证

缺少全局描述子召回。

### 任务内容

可选方案：

- Scan Context
- ring-sector descriptor
- 复用现有 relocalization 模块中的描述子能力

### 影响文件

- `DeepLumin/src/localization/slam_mapping/src/core/loop_closure_fastlio2_backend.cpp`
- 可能新增 `descriptor` 子模块

### 验收标准

- 在大场景和重复结构场景中召回能力增强
- 减少纯半径搜索带来的不稳定性

---

## 6.2 任务 J：地图管理从全量重建升级到增量 submap

- 重要程度：`P2`
- 容易实施性：`低`
- 可行性：`中高`

### 问题

当前关键帧点云全部驻留内存，保存时全量重建地图：

- [map_saver.cpp](/home/hhy/2004ros/DeepLumin/src/localization/slam_mapping/src/core/map_saver.cpp:47)

对长时建图和大场景不友好。

### 任务内容

建议：

- 维护 submap cache
- 支持分块地图
- 仅重建受 pose graph 影响的区域

### 影响文件

- `MapSaver`
- `KeyframeManager`
- 可能新增 `SubmapManager`

### 验收标准

- 长轨迹任务内存增长更可控
- 保存耗时随关键帧数增长更平滑

---

## 6.3 任务 K：外参与时间偏移自检工具

- 重要程度：`P2`
- 容易实施性：`中`
- 可行性：`高`

### 问题

许多建图异常最终不是算法逻辑错误，而是：

- 外参错误
- 点时间字段错误
- IMU / LiDAR 时间基不一致

### 任务内容

增加启动自检和离线脚本：

- 外参打印
- 首帧 IMU / LiDAR 时间差
- 平均扫描周期统计
- IMU 初始化输出摘要

### 影响文件

- `slam_odometry`
- `slam_mapping`
- `docs/` 和 `scripts/`

### 验收标准

- 新数据集接入时可快速定位传感器问题

---

## 7. 算法专项优化判断

## 7.1 是否需要“累计足够 IMU 再开始计算”

- 结论：`需要`
- 优先级：`P0`
- 原因：
  - 底层虽已做 IMU 初始化
  - 但外层无显式 warmup 状态
  - 工程体验和可解释性不足

建议：

- 外层显式 gate
- 状态机可见
- 启动期统计可见

---

## 7.2 是否需要引入独立 IMU 预积分后端

- 结论：`短期不建议优先做`
- 优先级：`P2`
- 原因：
  - 当前 Faster-LIO 已做 IMU propagation 与去畸变
  - 当前主要短板不是“完全没有 IMU 积分”
  - 真正的问题在于同步、回环、保存链和状态可观测性

只有在以下目标明确时再考虑：

- 引入更完整因子图后端
- 做关键帧级 IMU 因子融合
- 替换当前前端架构

---

## 7.3 当前时间同步是否需要优化

- 结论：`需要，但先优化观测与兜底，不建议立刻重写核心机制`
- 优先级：`P0`

建议先做：

- 统计同步分支
- 暴露扫描周期估计
- 暴露 IMU 等待 gap
- 在点时间退化时明确报警

后续再评估是否需要：

- 估计 LiDAR-IMU 固定时间偏移
- 对不同雷达类型引入更稳的 scan end 估计模型

---

## 8. 推荐实施顺序

### 第一阶段：一周内可完成

1. 任务 A：补全参数加载
2. 任务 D：修复保存成功判定
3. 任务 H：保存 metadata / 配置快照
4. 任务 C：补齐同步状态观测

### 第二阶段：需要联调验证

1. 任务 B：IMU warmup / startup gating
2. 任务 G：关键帧策略优化
3. 任务 E：回环候选排序

### 第三阶段：结构增强

1. 任务 F：全局位姿在线反馈
2. 任务 I：descriptor 回环前端
3. 任务 J：增量 submap
4. 任务 K：外参与时间偏移自检工具

---

## 9. 最小可执行版本建议

如果当前只能做一轮最小改造，建议只做以下 5 项：

1. `P0-A` 参数加载补全
2. `P0-B` IMU warmup 门控
3. `P0-C` 同步状态输出
4. `P0-D` 保存链路修复
5. `P1-E` 回环候选排序

这样可以在不大改架构的前提下，显著提升：

- 可调性
- 可解释性
- 保存可靠性
- 回环稳定性

---

## 10. 验收建议

建议为每轮改造固定使用 3 类数据集回归：

1. 不闭环短轨迹数据集
2. 小范围明确闭环数据集
3. 点时间不稳定或雷达适配数据集

每轮固定检查：

- 是否正常 warmup
- 是否正常产出关键帧
- 回环是否按预期触发或不触发
- 地图、关键帧、轨迹文件是否完整
- 输出目录 metadata 是否可复现运行条件

