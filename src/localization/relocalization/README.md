# relocalization

基于 Scan Context / CSC 描述子和 GICP / VGICP 精配准的全局重定位模块，为 DeepLumin 定位系统提供全局位姿估计。

## 功能特性

- **多描述子支持**：Scan Context（单通道距离）与 CSC（4 通道：距离/密度/强度/高度）
- **多匹配器支持**：PCL GICP 与 small_gicp VGICP（两级粗精配准）
- **多种 target 来源**：全局地图朝向裁剪（crop）/ 关键帧子图拼接（submap）
- **在线重定位**：实时接收 LiDAR 点云，输出 `map -> base_link` 全局位姿
- **离线 bag 评估**：支持 rosbag 批量评估，可选 map_align / truth_compare 两种模式
- **RViz 可视化**：评估过程多色点云与位姿箭头可视化
- **启停控制**：通过 `/localization_v2/relocalization/command` 话题控制匹配流程（start/stop）

## 依赖

- ROS Noetic
- PCL (Point Cloud Library)
- Eigen3
- OpenMP（并行加速）
- small_gicp（VGICP 匹配器）
- deeplumin_msgs
- deeplumin_cmake

## 编译

```bash
cd /home/hhy/2004ros/DeepLumin
catkin build relocalization
```

## 运行

### 1. 在线重定位

```bash
roslaunch relocalization global_relocalization.launch
```

订阅 `/localization/points`（或 yaml 中配置的 `lidar_topic`），发布：
- `/localization/relocalization/result`：`deeplumin_msgs/RelocalizationResult`
- `reloc_pose`：`geometry_msgs/PoseStamped`
- `reloc_status`：`std_msgs/String`
- `reloc_aligned_scan` / `reloc_target_submap`：`sensor_msgs/PointCloud2`

### 2. Bag 离线评估 + RViz 可视化

```bash
roslaunch relocalization bag_eval_viz.launch \
  bag_file:=/path/to/your.bag \
  bag_start_offset:=180.0 \
  bag_cloud_stride:=10
```

### 3. 构建描述子数据库

数据库由外部工具（如 `relocalization_framework` 的建库节点）生成 `.bin` 文件，本模块仅负责加载。

## 参数配置

主要配置文件：`config/relocalization.yaml`

### 模块选择

| 参数 | 说明 | 可选值 |
|------|------|--------|
| `descriptor/type` | 描述子类型 | `scan_context` / `csc` |
| `matcher/type` | 匹配器类型 | `gicp` / `vgicp` |
| `matcher/target_source` | target 点云来源 | `crop`（朝向裁剪全局地图）/ `submap`（关键帧子图） |

### 路径配置

| 参数 | 说明 |
|------|------|
| `keyframe_dir` | 关键帧 PCD 目录 |
| `pose_file` | 位姿文件（TUM 格式：timestamp tx ty tz qx qy qz qw） |
| `global_map_pcd` | 全局地图 PCD（crop 模式使用） |
| `scan_context_db` | 描述子数据库 `.bin` 文件 |

### Scan Context 参数

```yaml
scan_context:
  num_rings: 20
  num_sectors: 60
  max_radius: 100.0
  min_range: 3.0
  ring_key_candidates: 100
```

### CSC 参数

```yaml
csc:
  num_sectors: 60
  num_height_bins: 16
  max_radius: 100.0
  min_range: 3.0
  num_channels: 4
  channel_weights: [0.6, 0.5, 0.2, 0.3]
  similarity: "l1"       # l1 / balanced
  ring_key_candidates: 300
```

### GICP 参数

```yaml
gicp:
  max_iterations: 64
  max_correspondence_distance: 2.0
  transformation_epsilon: 0.01
  euclidean_fitness_epsilon: 0.01
  accept_fitness: 0.5
  inlier_distance: 0.5
  min_inlier_ratio: 0.6
  max_inlier_rmse: 0.5
```

### VGICP 参数（small_gicp）

```yaml
vgicp:
  voxel_resolution: 0.5
  num_neighbors: 30
  num_threads: 8
  coarse_scale: 10.0
  fine_scale: 1.0
```

### Crop 模式朝向裁剪参数

```yaml
matcher:
  target_source: "crop"
  crop:
    forward: 70.0
    backward: 50.0
    left: 40.0
    right: 40.0
    down: 10.0
    up: 10.0
```

### 子图参数

```yaml
submap:
  mode: radius           # radius / index / hybrid
  radius: 50.0
  neighbor_keyframes: 10
  before_frames: 0
  after_frames: 0
```

### 预处理参数

```yaml
preprocess:
  voxel_leaf: 0.5
  min_range: 3.0
  max_range: 80.0
```

> ⚠️ **一致性约束**：`preprocess.min_range` 应等于描述子 `min_range`，`preprocess.max_range` 应等于描述子 `max_radius`。修改后须重建数据库。

### 在线重定位参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `lidar_topic` | 输入 LiDAR 话题 | `/localization/points` |
| `frame_id` | 输出坐标系 | `map` |
| `child_frame_id` | 输出子坐标系 | `base_link` |
| `lidar_frame_id` | LiDAR 坐标系名 | `lidar_link` |
| `query_cloud_frame` | query 点云坐标系 | `auto`（使用输入 header.frame_id） |
| `result_topic` | 结果话题 | `/localization/relocalization/result` |
| `command_topic` | 启停命令话题 | `/localization_v2/relocalization/command` |
| `top_k` | SC 候选数量 | 20 |
| `extrinsics/base_to_lidar` | base_link -> lidar 外参 | 单位矩阵 |
| `query_frame/yaw_deg` | query 点云额外 yaw 旋转 | 0.0 |
| `query_frame/rotate_xy_y_negx` | query 点云 XY 轴交换（+ -90°） | false |

### Bag 评估参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `bag_file` | rosbag 路径 | - |
| `bag_lidar_topic` | bag 内 LiDAR 话题 | `/ouster/points` |
| `bag_cloud_stride` | 每隔 N 帧评估一次 | 10 |
| `max_queries` | 最多评估帧数 | 50 |
| `bag_start_offset` | 跳过 bag 开头秒数 | 0.0 |
| `bag_duration` | 只评估多少秒（0=到结束） | 0.0 |
| `bag_use_truth_pose` | true=与 TUM 真值对比；false=地图贴合评估 | false |
| `success_translation` | 成功判定平移阈值（m） | 1.0 |
| `success_yaw_deg` | 成功判定 yaw 阈值（°） | 5.0 |
| `log_topk_candidates` | 打印每帧 Top-K 候选 | true |

## 坐标系与 TF

- **输入**：query 点云默认在 LiDAR 坐标系（如 `lidar_link`）
- **输出**：`map -> base_link`（`RelocalizationResult.pose`）
- **外参转换**：若 `query_cloud_frame == "lidar"`，自动应用 `extrinsics/base_to_lidar` 将点云转到 base_link 系后再匹配
- **query 旋转**：支持 `query_frame_yaw_deg` 和 `rotate_xy_y_negx` 对输入点云做预处理旋转，用于 LiDAR 轴向与建库时不一致的场景

## 核心架构

```
┌─────────────────────────────────────────────────────────────────┐
│  RelocalizationCore                                             │
│  ├─ IRelocalizationDescriptor (ScanContextDescriptor / CscDescriptor) │
│  │   └─ query() → Top-K DescriptorCandidate (pose + yaw_diff)   │
│  ├─ IGlobalMatcher (GicpGlobalMatcher / VgicpGlobalMatcher)     │
│  │   └─ alignCandidates() → MatchResult (success + transform)   │
│  └─ KeyframeDatabase                                            │
│      ├─ loadKeyframeCloud()                                     │
│      ├─ buildSubmapAround() / cropOriented() / cropGlobalMap()  │
│      └─ loadTumPoses()                                          │
└─────────────────────────────────────────────────────────────────┘
```

### 匹配流程

1. **描述子提取**：将 query 点云转为 Scan Context / CSC 描述子
2. **粗检索**：Ring Key / Z-Key 粗筛 + Sector Key 精排，返回 Top-K 候选
3. **候选门控**（可选）：只保留 Top-1 周围 `candidate_gate_radius` 内的候选
4. **Target 构建**：根据 `target_source` 裁剪全局地图或拼接关键帧子图
5. **精配准**：GICP / VGICP 配准，初值使用候选位姿 + `yaw_diff` 修正
6. **质量判定**：converged + fitness + inlier_ratio + inlier_rmse 多阈值过滤

## 节点说明

| 可执行文件 | 节点名 | 用途 |
|-----------|--------|------|
| `global_relocalization_node` | `global_relocalization` | 在线重定位 |
| `bag_eval_node` | `bag_eval` | 离线 bag 评估 |
| `relocalization_node` | - | 旧版节点（保留兼容） |

## 成功判定条件

```
success = converged
          && fitness <= accept_fitness
          && inlier_ratio >= min_inlier_ratio
          && inlier_rmse <= max_inlier_rmse
```

失败时 `reject_reason` 字段说明：
- `no_candidate`：无描述子候选
- `gicp_not_converged`：GICP/VGICP 未收敛
- `quality_gate_failed`：收敛但质量指标未通过

## 注意事项

1. **数据库一致性**：修改 `scan_context.num_rings/num_sectors/max_radius/min_range` 或 `preprocess.*` 后必须重建 `.bin` 数据库
2. **CSC 数据库格式**：使用 magic header `SCR_CSC_DB_V2`，与 `relocalization_framework` 产出的 `.bin` 完全兼容
3. **VGICP 无 PCL fitness**：VGICP 的 `fitness` 仅作报告参考，成功判定以内点率/RMSE 为主
4. **坐标系对齐**：若 query 点云坐标系与建库时 key_point_frame 不一致，需通过 `query_frame_yaw_deg` 或 `rotate_xy_y_negx` 调整