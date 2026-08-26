# slam_mapping_v2 多线程设计

## 目标

- 订阅 `slam_odometry_v2` 的 odom / registered_scan / GNSS 消息完成建图。
- **后端优化（回环 + GNSS + ISAM2）与可视化不阻塞建图主流程**。
- 与 mapping_manager 兼容：start / stop / save / reset / get_status 服务 + status 话题。

## 线程划分

```
输入回调(AsyncSpinner×2)          仅入队：odom/scan/gnss → input_mutex_
        │
主线程(mapping, 100Hz)            popFrame 时间配对 → KeyframeManager.add
        │                          → pending_（只写队列，不碰已提交关键帧）
        │                          发布 odom/scan/状态(10Hz)/TF(map→odom)
        ▼
后端线程(backend)                 等 pending_cv_ → 独占锁合并关键帧
        │                          → LoopClosureBackend::update
        │                            （里程计因子+GNSS因子+回环检测+ISAM2）
        │                          → 结果写 LoopData（loop_mutex_）
        ▼
可视化线程(viz, 0.5s)             共享锁取关键帧快照 + LoopData
                                   → path/关键帧marker/地图预览/回环/GNSS marker
```

## 同步设计

| 数据 | 写者 | 读者 | 同步 |
|------|------|------|------|
| 输入队列 odom/scan/gnss | 回调线程 | 主线程 | `input_mutex_` |
| pending 关键帧 | 主线程 | 后端线程 | `pending_mutex_` + `pending_cv_` |
| 已提交关键帧 keyframes_ | 后端线程（独占） | 可视化/保存（共享） | `std::shared_mutex kf_mutex_` |
| LoopData（回环对/子图/GNSS 因子/map→odom） | 后端线程 | 可视化/保存/主线程 | `loop_mutex_` |
| MappingRuntimeStatus | 主/后端线程 | 任意 | `status_mutex_` |

关键点：**主线程从不取 `kf_mutex_`**，因此 ISAM2/GICP 耗时再长也不阻塞帧消费；
可视化线程只读快照；后端线程独占写关键帧并批量执行优化（`update()` 内部按
`previous_index_` 增量添加里程计因子，按批调用与逐帧调用语义一致）。

## 关键帧流程

1. 主线程 `KeyframeManager::add(odom, scan)`（内部维护首帧/最小帧间隔/距离/旋转/时间阈值）。
2. 从 `latest()` 拷贝关键帧，若启用 GNSS 因子则把 `gnss_buffer_` 中
   `|dt| <= max_age_sec` 的最优观测挂到该关键帧（消费式弹出，规则与 LIO-SAM 一致）。
3. 关键帧入 pending 并通知后端线程。
4. 后端线程合并进 keyframes_（id 按向量下标重排），执行回环/GNSS 优化，
   用最新关键帧的 `pose_map * pose_odom^{-1}` 计算 map→odom 供 TF 发布。

## 与 mapping_manager 集成

`mapping_manager/config/mapping_manager.yaml` 的 backend 段改为：

```yaml
backend:
  enabled: true
  start_service: /slam_mapping_v2/start
  stop_service: /slam_mapping_v2/stop
  get_status_service: /slam_mapping_v2/get_status
  status_topic: /slam_mapping_v2/status
```

## 调参入口

`config/slam_mapping_v2.yaml`：
- `v2/threading/backend_interval_sec`：后端唤醒周期（默认 0.1s）。
- `v2/threading/viz_interval_sec`：可视化周期（默认 0.5s）。
- `v2/threading/max_input_queue`：输入队列上限（防止积压）。
- `keyframe/loop_closure/gnss`：关键帧与后端算法参数（与 slam_mapping 同义）。
