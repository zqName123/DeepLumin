# slam_mapping 功能对齐修改清单

## 1. 目标

本文档用于指导 `slam_mapping` 在保留当前工程结构的前提下，向 `lightning-lm` 的关键帧、回环检测、位姿优化、地图构建行为对齐。

对齐原则：

- 保留当前 `MappingRuntime + KeyframeManager + LoopClosureBackend + MapSaver` 分层结构。
- 不要求代码组织与 `lightning-lm` 完全一致。
- 要求四类功能的实现行为尽量一致：
  - 关键帧生成与保存语义一致
  - 回环候选筛选与验证流程一致
  - 位姿图优化触发与结果回写一致
  - 地图构建与保存结果一致

不在本次范围内的内容：

- 不做大规模架构重写
- 不做性能优化
- 不做 UI/可视化风格统一
- 不做额外功能扩展

## 2. 当前结构与目标映射

当前 `slam_mapping` 结构：

- `MappingRuntime`：运行时驱动与状态管理
- `KeyframeManager`：关键帧筛选与保存
- `LoopClosureBackend`：回环检测与图优化入口
- `Fastlio2SamLcLoopClosureBackend`：当前回环实现
- `MapSaver`：全局地图与关键帧导出

目标映射关系：

- 用 `KeyframeManager` 对齐 `lightning-lm` 的关键帧语义
- 用 `Fastlio2SamLcLoopClosureBackend` 对齐 `lightning-lm` 的回环检测和图优化流程
- 用 `MapSaver` 对齐 `lightning-lm` 的地图输出结果
- 用 `MappingRuntime` 负责把前端、关键帧、回环、保存串联起来

## 3. 修改范围

必须修改的文件：

- `include/slam_mapping/core/keyframe_manager.hpp`
- `src/core/keyframe_manager.cpp`
- `include/slam_mapping/core/loop_closure_fastlio2_backend.hpp`
- `src/core/loop_closure_fastlio2_backend.cpp`
- `include/slam_mapping/core/loop_closure_backend.hpp`
- `src/core/loop_closure_backend.cpp`
- `include/slam_mapping/core/map_saver.hpp`
- `src/core/map_saver.cpp`
- `include/slam_mapping/core/mapping_runtime.hpp`
- `src/core/mapping_runtime.cpp`
- `config/slam_mapping.yaml`

可能需要联动修改的文件：

- `include/slam_mapping/ros/visualization_publisher.hpp`
- `src/ros/visualization_publisher.cpp`
- `src/slam_mapping_node.cpp`

建议新增但非必须的文件：

- `include/slam_mapping/common/mapping_keyframe.hpp`

如果不新增该文件，则继续在 `keyframe_manager.hpp` 中维护 `MappingKeyframe` 定义。

## 4. 模块级详细修改清单

### 4.1 关键帧模块

目标：

- 保持当前 `KeyframeManager` 结构不变
- 使关键帧语义与 `lightning-lm` 一致
- 明确前端位姿和优化位姿的双轨管理

#### 4.1.1 数据结构修改

修改对象：

- `MappingKeyframe`

必须具备的字段：

- `int id`
- `double timestamp`
- `localization::Pose pose_odom`
- `localization::Pose pose_map`
- `localization::PointCloudPtr cloud_body`
- `std::size_t point_count`

建议新增字段：

- `bool loop_closed = false`
- `bool optimized = false`
- `int loop_match_count = 0`

要求：

- 新增关键帧时 `pose_map = pose_odom`
- 后端优化后只更新 `pose_map`
- `cloud_body` 保持 body 系原始关键帧点云，不直接覆盖为 world 系

#### 4.1.2 接口行为修改

修改文件：

- `include/slam_mapping/core/keyframe_manager.hpp`
- `src/core/keyframe_manager.cpp`

修改内容：

- 保留 `shouldAdd()` 逻辑入口
- 保留 `add()` 入口
- `add()` 内创建完整关键帧对象
- 关键帧 ID 使用 `keyframes_.size()` 保证连续
- 返回值除了 `bool` 外，建议补一个可获取最新关键帧的接口

建议新增接口：

- `const MappingKeyframe* latest() const`
- `MappingKeyframe* latestMutable()`
- `int latestIndex() const`

#### 4.1.3 关键帧筛选策略

当前保留的规则：

- 距离阈值
- 旋转阈值
- 时间阈值
- 最小点数阈值

要求：

- 先不新增复杂策略
- 仅保证行为上与 `lightning-lm` 的关键帧驱动方式兼容

验收标准：

- 新关键帧能稳定连续生成
- `pose_odom` 和 `pose_map` 初始一致
- 后续优化后 `pose_map` 变化不会污染 `pose_odom`

### 4.2 回环检测模块

目标：

- 保留 `LoopClosureBackend` 外壳
- 在 `Fastlio2SamLcLoopClosureBackend` 内部把回环行为对齐到 `lightning-lm`

#### 4.2.1 保持外部接口稳定

文件：

- `include/slam_mapping/core/loop_closure_backend.hpp`
- `src/core/loop_closure_backend.cpp`

要求：

- 保留 `configure()`
- 保留 `reset()`
- 保留 `update(std::vector<MappingKeyframe>*)`
- 保留 `supported()` 和 `enabled()`

可新增接口：

- `localization::PointCloudPtr latestSubmapCloud() const`
- `localization::PointCloudPtr latestCurrentCloudWorld() const`
- `std::pair<int, int> latestLoopPair() const`
- `bool loopUpdatedLastFrame() const`

说明：

- 新增接口用于后续可视化和状态同步
- 不破坏当前 runtime 的模块边界

#### 4.2.2 内部状态补齐

文件：

- `include/slam_mapping/core/loop_closure_fastlio2_backend.hpp`

建议新增成员：

- `int last_loop_check_index_`
- `int latest_loop_from_`
- `int latest_loop_to_`
- `bool loop_updated_last_frame_`
- `localization::PointCloudPtr latest_submap_cloud_`
- `localization::PointCloudPtr latest_current_cloud_world_`

保留现有成员：

- `isam2_`
- `graph_`
- `initialized_estimate_`
- `loop_history_`
- `previous_index_`
- `kdtree_history_poses_`
- `cloud_history_poses_`

#### 4.2.3 候选触发策略修改

目标行为参考 `lightning-lm`：

- 不是每个关键帧都做回环搜索
- 每隔一定关键帧数再检测一次
- 排除与当前帧距离太近的历史帧
- 排除相邻密集候选

需要新增的逻辑：

- `loop_kf_gap`：每隔多少关键帧才允许检测
- `closest_id_th`：排除离当前太近的历史帧
- `min_id_interval`：候选间隔约束
- `max_range`：空间距离约束

建议新增函数：

- `bool shouldDetectLoopForLatestKeyframe(const std::vector<MappingKeyframe>& keyframes) const`
- `std::vector<int> findLoopCandidateIndices(const std::vector<MappingKeyframe>& keyframes, int current_idx) const`

要求：

- `update()` 先调用触发判断
- 不满足触发条件则只维护 odom factors，不做回环搜索

#### 4.2.4 子图构建逻辑修改

目标行为对齐 `lightning-lm`：

- 历史关键帧周围固定窗口构建子图
- 当前帧使用单帧或小窗口构建对齐输入
- 子图按 `pose_map` 变换到世界系
- 子图构建后统一做 voxel 滤波

文件：

- `src/core/loop_closure_fastlio2_backend.cpp`

现有函数：

- `buildSubmap()`

需要修改：

- 历史子图使用 `center_index ± search_num`
- 世界坐标变换统一使用 `pose_map`
- 当前帧也提供专用构建函数，避免把所有逻辑塞在 `detectCandidate()`

建议新增函数：

- `localization::PointCloudPtr buildCurrentSubmap(const std::vector<MappingKeyframe>& keyframes, int current_idx) const`
- `localization::PointCloudPtr buildHistorySubmap(const std::vector<MappingKeyframe>& keyframes, int history_idx) const`

附加要求：

- 更新 `latest_submap_cloud_`
- 更新 `latest_current_cloud_world_`

#### 4.2.5 候选验证流程拆分

当前问题：

- `detectCandidate()` 同时承担候选搜索、子图构建、ICP 验证、结果判定

目标：

- 拆成更接近 `lightning-lm` 的多阶段流程

建议拆分为：

- `findLoopCandidateIndices()`
- `buildCurrentSubmap()`
- `buildHistorySubmap()`
- `alignLoopCandidate()`
- `acceptLoopCandidate()`

建议新增函数：

- `bool alignLoopCandidate(const localization::PointCloudPtr& current_submap, const localization::PointCloudPtr& history_submap, double* score, Eigen::Matrix3d* diff_rot, Eigen::Vector3d* diff_pos) const`

要求：

- 先找到候选历史帧索引
- 再做对齐验证
- 验证通过才返回约束

#### 4.2.6 重复回环抑制

要求：

- 已经出现在 `loop_history_` 里的 `(history_idx, current_idx)` 不重复添加
- 同一当前帧若成功闭环，不对非常接近的其他候选重复加边

建议：

- 保留 `loop_history_`
- 新增当前帧回环成功后的短路策略

验收标准：

- 回环候选数量合理
- 不会对同一对关键帧重复加 loop edge
- 最新子图能从 backend 读取

### 4.3 位姿图优化模块

目标：

- 保留 ISAM2
- 将约束构造和优化触发逻辑对齐到 `lightning-lm`

#### 4.3.1 Odom factor 组织调整

文件：

- `src/core/loop_closure_fastlio2_backend.cpp`

现有函数：

- `addOdomFactors()`

要求：

- 保留相邻帧 odom factor
- 相对位姿统一由 `pose_odom` 计算
- 不要用 `pose_map` 计算 odom factor

可选增强：

- 增加最近两帧之间的补充运动边

本次最低要求：

- 保持当前相邻帧约束
- 确保语义明确

#### 4.3.2 Loop factor 组织调整

现有函数：

- `addLoopFactor()`

要求：

- loop factor 仅在候选验证通过后添加
- 回环噪声与匹配 score 挂钩
- 异常 score 不允许入图

建议：

- `score <= 0` 或非有限值直接丢弃
- 将 `score` 映射为平移和旋转噪声，而不仅是统一倍乘

若本次不细分噪声，也至少做到：

- 保守噪声设置
- 异常 score 过滤

#### 4.3.3 优化触发逻辑调整

现有函数：

- `optimize()`
- `update()`

要求：

- 每次新关键帧都增量加 odom factor
- 检测到回环则立即加入 loop factor 并优化
- 未检测到回环时，若 graph 中有未处理增量，也可以做一次轻量 update

目标行为：

- 与 `lightning-lm` 一致，回环时立即更新优化位姿

#### 4.3.4 优化结果回写

要求：

- `optimize()` 完成后，遍历所有关键帧
- 用优化结果回写 `pose_map`
- `pose_odom` 不允许被覆盖

附加要求：

- 若关键帧结构增加 `optimized` 字段，则在回写后标记
- 若关键帧结构增加 `loop_closed` 字段，则对参与闭环的关键帧标记

验收标准：

- 回环发生后，历史关键帧 `pose_map` 会被统一更新
- 保存地图时读到的是优化后的 `pose_map`

### 4.4 地图构建与保存模块

目标：

- 保持 `MapSaver` 作为保存器的角色
- 使其输出结果与 `lightning-lm` 的地图语义一致

#### 4.4.1 全局地图构建

文件：

- `src/core/map_saver.cpp`

现有函数：

- `buildMap()`
- `transformCloud()`

要求：

- 继续按 `pose_map` 将 `cloud_body` 变换到世界系
- 不允许按 `pose_odom` 构建最终地图
- 若关键帧为空或点云为空，安全跳过

这部分当前逻辑基本正确，主要要求保持语义稳定。

#### 4.4.2 关键帧保存逻辑

现有函数：

- `saveKeyframes()`
- `saveKeyframePoses()`
- `saveTrajectory()`

要求：

- `keyframes/` 继续保存 body 系点云
- `keyframes.csv` 保存优化后位姿 `pose_map`
- `trajectory.csv` 保存优化后轨迹 `pose_map`

必要时增加说明字段：

- `id,timestamp,x,y,z,qx,qy,qz,qw`

不要求保存 `pose_odom`，但如后续需要排障，可新增 `keyframes_odom.csv`

#### 4.4.3 保存前的最终一致性

文件：

- `src/core/mapping_runtime.cpp`
- `src/core/map_saver.cpp`

要求：

- 停止建图并保存前，确保 loop backend 已完成最后一轮优化
- 保存时使用最新 `keyframe_manager_.keyframes()`

如果当前 `save()` 直接走 `map_saver_.saveAll()`，则需要保证：

- `save()` 调用前关键帧中的 `pose_map` 已是最终值

验收标准：

- 全局地图、关键帧位姿、轨迹三者一致
- 回环后保存的地图不会回退到优化前轨迹

### 4.5 Runtime 串联与状态管理

目标：

- 保持 `MappingRuntime` 为总调度器
- 将关键帧、回环、优化、保存更紧密地串联起来

#### 4.5.1 handleFrame 串联逻辑

文件：

- `src/core/mapping_runtime.cpp`

现有流程：

- `keyframe_manager_.add()`
- `loop_backend_.update()`
- 更新状态

修改要求：

- 新增关键帧后再调用 `loop_backend_.update()`
- `loop_backend_` 返回的结果要覆盖：
  - `loop_pairs`
  - `loop_count`
  - `latest_score`
  - `updated`

可选增强：

- 从 `loop_backend_` 拉取 `latestSubmapCloud()` 和 `latestCurrentCloudWorld()`
- 交给可视化发布模块使用

#### 4.5.2 状态字段同步

要求：

- `status_.loop_updated_last_frame`
- `status_.last_loop_score`
- `status_.last_loop_from`
- `status_.last_loop_to`
- `status_.loop_count`

必须以 backend 最新结果为准。

消息更新要求：

- 发生回环时输出：
  - `loop_detected:from->to score=...`
- 未发生回环时保持：
  - `running`

#### 4.5.3 reset 逻辑

要求：

- `reset()` 时：
  - 清空关键帧
  - 清空 loop history
  - 清空 graph 和 initialized estimate
  - 清空最新子图缓存
  - 重置回环状态变量

验收标准：

- 重启任务后不会继承上一次的回环状态和关键帧数据

### 4.6 配置文件修改

目标：

- 保留当前配置结构
- 向 `lightning-lm` 的功能参数对齐

文件：

- `config/slam_mapping.yaml`

#### 4.6.1 keyframe 配置

保留：

- `distance_thresh`
- `rotation_thresh_rad`
- `time_thresh_sec`
- `min_points`

#### 4.6.2 loop_closure 配置

保留：

- `enable`
- `type`
- `submap_search_num`
- `submap_resolution`
- `icp_fitness_thresh`
- `pose_graph_enable`

新增：

- `loop_kf_gap`
- `min_id_interval`
- `closest_id_th`
- `max_range`
- `ndt_score_th`
- `motion_trans_noise`
- `motion_rot_noise`
- `loop_trans_noise`
- `loop_rot_noise`
- `rk_loop_th`

兼容建议：

- 旧参数继续可读
- 新参数优先

#### 4.6.3 参数默认值建议

可参考 `lightning-lm` 的当前语义，建议初始值如下：

- `loop_kf_gap: 20`
- `min_id_interval: 20`
- `closest_id_th: 50`
- `max_range: 30.0`
- `ndt_score_th: 1.0`
- `motion_trans_noise: 0.1`
- `motion_rot_noise: 0.05236`
- `loop_trans_noise: 0.2`
- `loop_rot_noise: 0.05236`
- `rk_loop_th: 1.04`

## 5. 逐文件修改清单

### 5.1 `include/slam_mapping/core/keyframe_manager.hpp`

修改项：

- 扩展 `MappingKeyframe` 定义
- 新增关键帧辅助访问接口
- 明确 `pose_odom` / `pose_map` 语义

完成判据：

- 头文件能表达完整关键帧结构

### 5.2 `src/core/keyframe_manager.cpp`

修改项：

- 调整 `add()`
- 确保 `pose_map = pose_odom`
- 新增 `latestMutable()` 等辅助接口实现

完成判据：

- 新关键帧创建路径稳定

### 5.3 `include/slam_mapping/core/loop_closure_fastlio2_backend.hpp`

修改项：

- 增加回环状态成员
- 增加局部子图缓存成员
- 增加必要辅助函数声明

完成判据：

- 头文件反映新的回环内部状态和功能边界

### 5.4 `src/core/loop_closure_fastlio2_backend.cpp`

修改项：

- 重构 `update()`
- 增加回环触发判定
- 重构候选检索
- 重构子图构建
- 拆分候选验证
- 调整 loop factor 添加入图策略
- 保持优化结果回写

完成判据：

- 回环流程清晰分层
- 能输出与 `lightning-lm` 接近的行为

### 5.5 `include/slam_mapping/core/loop_closure_backend.hpp`

修改项：

- 如需对外提供局部子图、最新闭环信息，则在此增加透传接口

完成判据：

- runtime 可以不依赖具体 backend 实现细节

### 5.6 `src/core/loop_closure_backend.cpp`

修改项：

- 新增透传逻辑
- 保持现有 backend 包装方式

完成判据：

- 外层调用方式不破坏

### 5.7 `include/slam_mapping/core/map_saver.hpp`

修改项：

- 如需增加导出类型或辅助函数声明，在此补齐

完成判据：

- 保存器接口语义清楚

### 5.8 `src/core/map_saver.cpp`

修改项：

- 确认 `buildMap()` 完全基于 `pose_map`
- 确认 `saveKeyframePoses()` 和 `saveTrajectory()` 输出优化位姿
- 必要时增加稳定性检查

完成判据：

- 最终保存产物一致

### 5.9 `include/slam_mapping/core/mapping_runtime.hpp`

修改项：

- 若需要缓存 backend 输出的局部子图或回环状态，则补充成员和接口

完成判据：

- runtime 具备对后端状态的统一表达能力

### 5.10 `src/core/mapping_runtime.cpp`

修改项：

- 强化 `handleFrame()` 串联逻辑
- 强化 `status_` 更新逻辑
- 确保 `save()` 前使用最新优化结果

完成判据：

- 运行时状态与后端行为一致

### 5.11 `config/slam_mapping.yaml`

修改项：

- 新增回环参数
- 补充优化噪声参数
- 保持兼容字段

完成判据：

- 新实现可通过配置调参与 `lightning-lm` 靠拢

## 6. 实施顺序

推荐按以下顺序执行：

1. 先改关键帧结构和 `KeyframeManager`
2. 再改 `Fastlio2SamLcLoopClosureBackend` 的状态、候选检索和子图构建
3. 再改 loop factor 和优化触发逻辑
4. 再改 `MappingRuntime` 状态串联
5. 最后改 `MapSaver` 和配置文件

原因：

- 关键帧语义是后面所有模块的基础
- 回环和优化依赖关键帧结构稳定
- 地图保存依赖优化后的关键帧结果稳定

## 7. 验收清单

### 7.1 关键帧

- 新任务开始后关键帧 ID 从 0 连续递增
- 每个关键帧包含 `pose_odom`、`pose_map`、`cloud_body`
- 回环优化后 `pose_map` 变化，`pose_odom` 不变

### 7.2 回环检测

- 不是每个关键帧都触发回环搜索
- 候选历史帧会按索引间隔和距离筛选
- 已闭环 pair 不重复添加
- 能拿到当前最新子图

### 7.3 位姿优化

- 每次新增关键帧都会维护 odom factor
- 成功回环后会立即做图优化
- 优化完成后历史关键帧 `pose_map` 会回写更新

### 7.4 地图保存

- `map.pcd` 基于优化后位姿构建
- `keyframes.csv` 保存优化后位姿
- `trajectory.csv` 保存优化后轨迹
- 三者结果一致

### 7.5 Runtime 状态

- `loop_count` 正常累计
- `last_loop_from/to` 正常更新
- `loop_updated_last_frame` 仅在新回环发生时为真
- 保存前状态与关键帧最终结果一致

## 8. 风险点

需要重点关注的风险：

- 关键帧结构修改后，现有可视化和保存逻辑可能同步失配
- 回环候选拆分后，`update()` 的返回语义可能需要微调
- 参数增多后，旧配置文件若不补值可能导致行为异常
- 若 `loop_backend_` 新增局部子图接口，ROS 发布模块可能需要轻微适配

规避建议：

- 每完成一个阶段后先做编译检查
- 先保证逻辑正确，再做参数细调
- 不要在同一阶段同时改可视化和回环主流程

## 9. 建议的提交拆分

建议拆成 4 个提交：

1. `slam_mapping: align keyframe semantics with loop backend`
2. `slam_mapping: align loop candidate and submap building flow`
3. `slam_mapping: align pose graph update and optimization behavior`
4. `slam_mapping: align map saving and runtime status flow`

如果不拆提交，至少按这四组顺序完成并分别自测。
