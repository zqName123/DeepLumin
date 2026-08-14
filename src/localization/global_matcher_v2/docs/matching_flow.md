# global_matcher_v2 匹配流程与模式说明

本文档基于 `global_matcher_v2` 当前代码（节点 `global_matcher_v2_node.cpp`、
匹配器 `matcher.cpp`、地图管理器 `map_manager.cpp`）整理，说明：

1. 整体匹配流程；
2. 初始位姿从哪来、按什么优先级选择；
3. 每种匹配算法（icp / gicp / vgicp / fast_gicp / fast_gicp_vgicp）在粗/细
   两阶段分别做什么、用哪些参数；
4. 质量门控与结果输出；
5. 参数速查表。

---

## 1. 整体匹配流程

```text
source_cloud（当前帧点云）
initial_pose / source_odom / bootstrap / manual / default
        │
        ▼
① 输入快照与时效检查（max_cloud_age, min_source_points）
        │
        ▼
② 初始位姿选择（selectInitialGuess）→ map_source / map_base
        │
        ▼
③ 地图就绪（ensureActive：single 整图 / chunked 按位置加载分片）
        │   target 结构：带协方差点云 / PCL / KDTree / VGICP 体素地图（缓存）
        ▼
④ 匹配（match：粗配准 → 精细配准）
        │
        ▼
⑤ 内点统计 + 质量门控（inlier / rmse / delta / fitness）
        │
        ▼
⑥ 发布 result / status / path / 调试话题，更新 last_map_source
```

匹配由固定频率定时器触发（`runtime.match_rate`，默认 10Hz），每帧完整走一遍
①~⑥；`runtime.auto_match=false` 时不执行匹配。

---

## 2. 输入与坐标系

### 2.1 输入话题

| 话题 | 类型 | 含义 |
|---|---|---|
| `topics/source_cloud` | sensor_msgs/PointCloud2 | 当前帧点云，坐标系由 `runtime.source_cloud_frame` 决定（odom/lidar/base） |
| `topics/initial_pose` | nav_msgs/Odometry | 跟踪初值：融合里程计，**map->base_link** |
| `topics/source_odom` | nav_msgs/Odometry | 源里程计，**odom->base_link**（source 在 odom 系时用于构造 map_source） |
| `topics/bootstrap_initial_pose` | PoseWithCovarianceStamped | 引导初值（如 localization_manager 发布的 `/localization/manager/initial_pose`），**map->base_link** |
| `topics/manual_initial_pose` | PoseWithCovarianceStamped | 人工初值（RViz /initialpose），**map->base_link** |

### 2.2 坐标系约定

```text
map（全局地图，匹配 target 所在系）
  └── odom（source 点云所在系，source_cloud_is_odom_frame=true 时）
       └── base_link（车体系）
            └── lidar_link（source_cloud_frame=lidar 时，经 extrinsics.base_to_lidar）
```

匹配求解的未知量是 **T_map_source**（source 点云系到 map 系的变换）：

```text
source 在 odom 系： T_map_source = T_map_base * T_odom_base⁻¹
source 在 lidar 系：T_map_source = T_map_base * T_base_lidar
source 在 base 系： T_map_source = T_map_base
```

匹配完成后：

```text
T_map_base = T_map_source * T_source_base
```

其中 `T_source_base`：odom 系时为最新 `source_odom`，lidar 系时为
`base_to_lidar⁻¹`，base 系时为单位阵。

---

## 3. 初始位姿来源（selectInitialGuess）

优先级从高到低，只有被 `runtime.use_*` 开关允许的来源才会进入候选：

### 3.1 TRACKING（跟踪）模式

触发条件：`runtime.use_tracking_initial_pose=true` 且（已有融合里程计初值
`have_tracking` **或** 已有上一次成功匹配 `have_last_map_source_`）。

| 子来源 | 条件 | 初值 | 说明 |
|---|---|---|---|
| 上次成功结果 | `have_last_map_source_` | `T_map_source = last_map_source_` | 串行、无消息回路延迟，与 ysw_loc 的 map->odom 延续一致；**优先于 fused odom** |
| 融合里程计 | 上者不可用 | `T_map_base = /localization/fused_odom` 位姿 | 来自 `topics/initial_pose`，通常为 localization_v2 输出 |

TRACKING 下即使匹配失败也继续使用 fused odom 作为初值（失败只拒绝观测，
不回落到默认/单位初值），避免跳回地图原点。

### 3.2 BOOTSTRAP（引导）模式

触发条件：TRACKING 不可用（`use_tracking_initial_pose=false`，或既无 fused odom
也无历史结果）。

| 子来源 | 条件 | 初值 |
|---|---|---|
| 引导初值 | `use_bootstrap_initial_pose=true` 且已收到 | `T_map_base = bootstrap`（manager/manual 话题） |
| 默认初值 | `use_default_initial_pose=true` | `T_map_base = runtime.default_initial_pose`（translation + rpy） |
| 单位初值 | `allow_identity_bootstrap=true` | `T_map_base = I` |

若以上都不可用 → `bootstrap:no_initial_available`，本帧拒绝。

### 3.3 人工初值的特殊处理

收到 `manual_initial_pose` 时：

```text
last_map_source_ = T_map_base(manual) * T_odom_base⁻¹   （若已有 source_odom）
manual_reset_seq_ += 1
```

`manual_reset_seq_` 用于“匹配期间新到人工初值则丢弃本帧旧结果”，防止旧结果覆盖
新锚点。

### 3.4 与初始位姿相关的参数

```yaml
runtime:
  use_tracking_initial_pose: true
  use_bootstrap_initial_pose: true
  use_manual_initial_pose: true
  use_default_initial_pose: true
  allow_identity_bootstrap: true
  default_initial_pose: {translation: [0,0,0], rpy_deg: [0,0,0]}
  source_cloud_is_odom_frame: true
  source_cloud_frame: odom        # auto | odom | base_link | lidar
```

> 注意：默认/单位初值只在**没有 fused odom、没有历史结果**时才参与。若地图原点
> 与 SLAM 起始原点不一致，identity bootstrap 会差几千米，导致全部对应被拒
> （详见 §7 常见问题）。

---

## 4. 地图管理与 target 结构

### 4.1 地图来源

```yaml
map:
  mode: single      # single：整图（pcd_path） | chunked：分片（map_dir）
  load_voxel_leaf: 0.4
  chunk_size: 100.0
  load_radius_chunks: 1
  max_loaded_chunks: 64
  cov_num_neighbors: 20
  cov_num_threads: 4
  vgicp_voxel_resolution: 0.5
  vgicp_coarse_scale: 10.0
  vgicp_fine_scale: 1.0
```

### 4.2 target 结构（按活动分片集合缓存，一次构建多次复用）

分片/整图加载时（`loadChunk`）一次性完成：

- 体素降采样（`load_voxel_leaf`）；
- 转 small_gicp 点云并**预计算协方差**（`cov_num_neighbors` / `cov_num_threads`）。

活动分片集合变化时（`rebuildActiveStructuresLocked`）构建：

| 结构 | 用途 | 构建参数 |
|---|---|---|
| `PointCloud`（含协方差） | small_gicp gicp / vgicp / icp target | — |
| PCL `PointCloud` | fast_gicp target、可视化 | — |
| `KdTree<PointCloud>` | gicp / icp 最近邻、内点统计 | `KdTreeBuilderOMP(cov_num_threads)` |
| 粗 `GaussianVoxelMap` | vgicp 粗配准 target | `vgicp_voxel_resolution * vgicp_coarse_scale`，search offsets 27 |
| 细 `GaussianVoxelMap` | vgicp 精细配准 target | `vgicp_voxel_resolution * vgicp_fine_scale`，search offsets 27 |

只有 `matcher.type` 为 `vgicp` 或 `fast_gicp_vgicp` 时才预构建体素地图。

> single 模式下 target 是整图（0.4m 降采样后约 150 万点）；`crop.*` 参数只用于
> 可视化子图，**不参与匹配**（与 v1 不同，详见 §7）。

---

## 5. 匹配算法模式（matcher.type）

所有模式都执行“粗配准 → 精细配准”两阶段（`coarse_scale > 1` 才跑粗配准），
通用处理：

```text
source 降采样：source_voxel_leaf * scale（粗 scale=coarse_scale，细 scale=fine_scale）
对应距离门限：coarse/fine_max_correspondence_distance（0 时 = max_correspondence_distance * scale）
迭代/收敛：max_iterations、transformation_epsilon（translation/rotation eps）
```

各类型差异如下：

### 5.1 `fast_gicp`（默认，v1 行为）

| 阶段 | target | 处理 |
|---|---|---|
| 粗 | PCL 整图/活动地图，降采样 `target_voxel_leaf*coarse_scale` | `fast_gicp::FastGICP`：**自行重建 target KDTree+协方差**，`fast_gicp_num_threads` 线程、`fast_gicp_correspondence_randomness` 随机对应 |
| 细 | 同上，`target_voxel_leaf*fine_scale` | 同上，初值 = 粗配准结果 |

参数：`fast_gicp_num_threads`（默认 4）、`fast_gicp_correspondence_randomness`
（默认 20）、`max_iterations`、`transformation_epsilon`、
`max_correspondence_distance`（+ scale）。

> 特点：对初值容错好（收敛域宽），但 target 规模大时每帧重建树/协方差的成本高。
> 鲁棒核（`matcher.robust_kernel`）对 fast_gicp **不生效**。

### 5.2 `gicp`（small_gicp）

| 阶段 | target | 处理 |
|---|---|---|
| 粗/细 | 缓存的整图 `PointCloud`（含协方差）+ 缓存 `KdTree` | `Registration<GICPFactor, ParallelReductionOMP>`；source 每阶段重新降采样并估计协方差（`cov_num_neighbors`） |

参数：`num_threads`、`cov_num_neighbors`、`cov_num_threads`、`robust_kernel`
（none/huber/cauchy）、`robust_kernel_scale`。

> 特点：复用缓存的 target 树/协方差，target 侧零预处理，适合整图大 target；
> 鲁棒核生效。

### 5.3 `vgicp`（small_gicp）

| 阶段 | target | 处理 |
|---|---|---|
| 粗 | 缓存粗 `GaussianVoxelMap`（`vgicp_voxel_resolution*coarse_scale`） | `Registration<GICPFactor, ...>`，target_tree=voxelmap 本身（搜索 27 邻体素，天然局部） |
| 细 | 缓存细 `GaussianVoxelMap`（`vgicp_voxel_resolution*fine_scale`） | 同上，初值 = 粗配准结果 |

参数：`vgicp_voxel_resolution`、`vgicp_coarse_scale`、`vgicp_fine_scale`、
`num_threads`、`cov_num_neighbors`、`robust_kernel`、`robust_kernel_scale`。

> 特点：体素地图按活动分片集合/整图**预构建一次**，帧内只做 source 协方差 +
> 对齐；对应关系只查初值附近体素，整图 target 也不会搜到远处。

### 5.4 `icp`（small_gicp）

| 阶段 | target | 处理 |
|---|---|---|
| 粗/细 | 缓存整图 `PointCloud` + `KdTree` | `Registration<ICPFactor, ...>`（点对点最近邻，无协方差） |

参数：同 gicp（`robust_kernel` 可选）。

> 特点：最便宜、最粗糙，依赖初值质量。

### 5.5 `fast_gicp_vgicp`（混合分支）

| 阶段 | target | 处理 |
|---|---|---|
| 粗 | PCL 整图/活动地图，`target_voxel_leaf*coarse_scale` | **fast_gicp**（宽收敛域，负责把初值拉回来） |
| 细 | 缓存细 `GaussianVoxelMap` | **small_gicp vgicp**（快速精确细化） |

参数：同时使用 fast_gicp 与 vgicp 两组参数。

> 设计意图：粗配准用 fast_gicp 的鲁棒收敛域，精细配准用预构建体素地图的低成本
> 精度；细阶段不再对整图建树。

### 5.6 鲁棒核

仅 small_gicp 类型（gicp / vgicp / icp）生效：

```yaml
matcher:
  robust_kernel: huber     # none | huber | cauchy
  robust_kernel_scale: 1.0 # 核宽度 c
```

实现：`RobustFactor<Huber/Cauchy, Factor>` 包装因子，对误差加权，抑制动态物体/
植被等离群对应点对优化的拉偏。

---

## 6. 质量门控与结果输出

### 6.1 门控（gate.*）

```yaml
gate:
  min_source_points: 50      # 源点数下限
  min_target_points: 100     # target 点数下限
  max_fitness_score: 100.0   # 仅 fast_gicp 生效（PCL fitness 语义）
  min_inlier_ratio: 0.1      # 内点率下限
  max_inlier_rmse: 0.4       # 内点 RMSE 上限
  inlier_distance: 0.5       # 内点判定距离（米）
  max_delta_translation: 20.0  # 相对初值的位移上限
  max_delta_yaw_deg: 20.0      # 相对初值的偏航上限
```

内点统计：source 按 `final_map_source` 变换后，对缓存的 target `KdTree` 做最近邻，
距离 ≤ `inlier_distance` 记为内点。

> 说明：small_gicp 的 error 量纲与 PCL fitness 不同，所以 `max_fitness_score`
> 只对 `fast_gicp` 生效；gicp/vgicp/icp 以内点率 / RMSE / delta 门控（与 v1 对
> vgicp 的处理一致）。

### 6.2 结果输出

```yaml
result:
  pose_frame: odom    # odom：结果位姿 = map_source，child_frame_id=odom
                      # base：结果位姿 = map_base，child_frame_id=base_link
```

`GlobalMatchResult` 字段：`success / converged / map_id / reject_reason /
fitness_score / inlier_ratio / inlier_rmse / inlier_count / source_points /
target_points / elapsed_ms / initial_translation_error / initial_yaw_error`，
协方差由 `inlier_rmse` 与 delta 构造。

成功时更新 `last_map_source_`（下一帧 TRACKING 初值）；失败仅递增连续失败计数。

---

## 7. 常见问题（与 v1 的差异）

1. **crop 不参与匹配**：v1 每帧把地图裁成 70m FOV 子图作为 target；v2 匹配使用
   活动分片/整图的缓存结构，`crop.*` 只影响 `target_submap` 可视化。整图
   0.4m 降采样约 150 万点，**fast_gicp 每帧对整图重建树+协方差会导致单帧
   秒级、匹配近乎失效**——这正是 single 模式下“全局匹配没作用”的主要原因之一。
2. **初值坐标系**：默认/单位 bootstrap 只在没有 fused odom 与历史结果时使用；
   若地图原点与 SLAM 起始原点不一致，初值偏差几千米会直接导致全部对应被拒。
   应通过 relocalization / 人工 `/initialpose` / 正确 `default_initial_pose` 初始化。
3. **整图匹配的正确姿势**：需要整图 target 时优先 gicp/vgicp/icp（复用缓存
   树/体素，帧内只有 source 侧成本）；fast_gicp 适合 FOV 裁剪后的中小 target。
4. **发布开关**：`publish.*` 控制 result 之外的调试话题（global_map / path /
   status 默认开；aligned_scan / target_submap / initial_guess / source_initial
   默认关）。

---

## 8. 参数速查表

| 分组 | 参数 | 默认 | 作用对象 |
|---|---|---|---|
| runtime | match_rate | 10.0 | 匹配频率 |
| runtime | max_cloud_age / use_receive_time_for_cloud_age | 1.0 / true | 点云时效 |
| runtime | source_cloud_is_odom_frame / source_cloud_frame | true / odom | source 坐标系 |
| runtime | use_tracking/bootstrap/manual/default/identity | true | 初值来源开关 |
| map | mode / pcd_path / map_dir | single / - / - | 地图来源 |
| map | load_voxel_leaf | 0.4 | target 降采样 |
| map | chunk_size / load_radius_chunks / max_loaded_chunks | 100 / 1 / 64 | 分片加载 |
| map | cov_num_neighbors / cov_num_threads | 20 / 4 | target 协方差预计算 |
| map | vgicp_voxel_resolution / coarse_scale / fine_scale | 0.5 / 10 / 1 | 体素地图分辨率 |
| matcher | type | fast_gicp | 算法模式 |
| matcher | source_voxel_leaf / target_voxel_leaf | 0.08 / 0.08 | 两阶段降采样 |
| matcher | coarse_scale / fine_scale | 6.0 / 3.0 | 两阶段尺度 |
| matcher | max_iterations / transformation_epsilon | 30 / 0.001 | 迭代与收敛 |
| matcher | max_correspondence_distance / coarse_/fine_override | 3.5 / 0 / 0 | 对应距离门限 |
| matcher | robust_kernel / robust_kernel_scale | huber / 1.0 | 鲁棒核（small_gicp） |
| matcher | num_threads / fast_gicp_num_threads / randomness | 4 / 4 / 20 | 线程与对应随机性 |
| gate | min_source_points / min_target_points | 50 / 100 | 点数门限 |
| gate | max_fitness_score | 100.0 | 仅 fast_gicp |
| gate | min_inlier_ratio / max_inlier_rmse / inlier_distance | 0.1 / 0.4 / 0.5 | 质量门限 |
| gate | max_delta_translation / max_delta_yaw_deg | 20.0 / 20.0 | 初值一致性 |
| result | pose_frame | odom | 结果位姿语义 |
| publish | global_map / path / status 等 | 部分 true | 话题开关 |
