# 可视化排查（path / map 看不到）

## 数据在哪里发布

- `/slam_mapping_v2/path`、`/map`、`/keyframes`、`/loop_markers`、
  `/gnss_markers`、`/gnss_factor_edges`、回环子图
  → **可视化线程**（`SlamMappingV2Node::visualizationLoop`），每
  `v2/threading/viz_interval_sec`（默认 0.5s）发布一次，内容来自关键帧快照。
- `/slam_mapping_v2/odom`、`/current_scan` → 主线程 `spinOnce`，每消费一帧发布。
- `/slam_mapping_v2/status` → 主线程 10Hz 发布。

## 看不到的原因（按可能性排序）

1. **状态不是 RUNNING**：可视化线程仅在 `state == RUNNING` 时发布。
   未调用 `/slam_mapping_v2/start`（或 `mapping/auto_start: false`）时，
   只有 status 有输出。→ 调用
   `rosservice call /slam_mapping_v2/start "{task_id: t1, task_name: demo, output_dir: /tmp/deeplumin_mapping/demo, enable_loop_closure: true, reset_before_start: true}"`
   或设 `auto_start: true`。
2. **还没有关键帧**：path 由关键帧生成（map 话题由每帧扫描增量累积，但回环/保存依赖关键帧）。需要车辆移动超过
   `keyframe/distance_thresh`（默认 2.0m）且满足 `min_frame_gap`（默认 10 帧）。
   用 `rosservice call /slam_mapping_v2/get_status` 看 `keyframe_count` 是否增长。
3. **TF/坐标系（已统一）**：所有可视化输出（path / map / keyframes / 回环 marker）统一 **map 系**；RViz 固定 `map`，需要 `map→odom`（v2 默认发布）+ `odom→base_link`（slam_odometry_v2 发布）完整 TF 链。

## 快速自检

```bash
rostopic hz /slam_mapping_v2/status
rosservice call /slam_mapping_v2/get_status "{}"   # state 应为 RUNNING，keyframe_count 增长
rostopic hz /slam_mapping_v2/path /slam_mapping_v2/map
```
