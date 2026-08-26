# slam_mapping_v2

基于**消息订阅**的建图节点：订阅 `slam_odometry_v2` 发布的 odom 与配准扫描，
完成全局地图、关键帧管理、GNSS 后端优化、回环检测与 GTSAM 图优化。
关键帧/回环/GNSS/保存/可视化均已重构为 **v2 自有模块**（单命名空间 `slam_mapping_v2`，不依赖 slam_mapping / slam_odometry 的任何 include/库）。可视化排查见 [docs/VISUALIZATION.md](docs/VISUALIZATION.md)。

## 多线程设计

- 输入回调（AsyncSpinner×2）：odom/scan/gnss 仅入队，不做任何计算。
- 主线程：消费帧 → odom/scan 时间配对 → 关键帧判定（KeyframeManager）→
  仅写入 pending 队列 → 发布 odom/scan/状态（10Hz）/TF。**从不访问后端关键帧**，
  因此 ISAM2/GICP 再慢也不阻塞建图。
- 后端线程：pending 非空时，独占锁合并关键帧 → LoopClosureBackend::update
  （里程计因子 + GNSS 因子 + 回环检测 + ISAM2 优化）→ 结果写入 LoopData 快照。
- 可视化线程：定时取关键帧快照（共享锁）与 LoopData，发布
  path / 关键帧 marker / 地图预览 / 回环 marker / GNSS marker / 回环子图。

## 输入输出

| 方向 | 话题 | 类型 |
|------|------|------|
| 输入 | `/localization/slam_odom` | nav_msgs/Odometry |
| 输入 | `/localization/cloud_registered` | sensor_msgs/PointCloud2（odom 系） |
| 输入 | `/gnss_chc_data`（gnss.enable） | deeplumin_msgs/Gpchc |
| 输出 | `/slam_mapping_v2/odom`、`/path`、`/current_scan`、`/map_preview` | 可视化 |
| 输出 | `/slam_mapping_v2/keyframes`、`/loop_markers`、`/gnss_markers`、`/gnss_factor_edges` | 可视化 |
| 输出 | `/slam_mapping_v2/status` | deeplumin_msgs/MappingStatus |
| 服务 | `/slam_mapping_v2/start|stop|save|reset|get_status` | mapping_manager 兼容 |

## 与 mapping_manager 集成

`slam_mapping_v2` 提供与 `slam_mapping` 相同的服务接口。将
`mapping_manager/config/mapping_manager.yaml` 的 backend 段指向 v2：

```yaml
backend:
  enabled: true
  start_service: /slam_mapping_v2/start
  stop_service: /slam_mapping_v2/stop
  get_status_service: /slam_mapping_v2/get_status
  status_topic: /slam_mapping_v2/status
```

## 编译与运行

```bash
cd /home/hhy/2004ros/DeepLumin
catkin_make --pkg slam_mapping_v2

# 先运行 slam_odometry_v2，再启动建图
roslaunch slam_odometry_v2 slam_odometry_v2.launch play_bag:=false
roslaunch slam_mapping_v2 slam_mapping_v2.launch
```

## 保存

```bash
rosservice call /slam_mapping_v2/start "{task_id: t1, task_name: demo, output_dir: /tmp/deeplumin_mapping/demo, enable_loop_closure: true, reset_before_start: true}"
rosservice call /slam_mapping_v2/stop "{task_id: t1, save_map: true, save_keyframes: true, save_keyframe_poses: true, save_trajectory: true}"
```
