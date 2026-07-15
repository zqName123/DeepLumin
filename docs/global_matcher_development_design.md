# DeepLumin global_matcher 开发设计文档

> 适用模块：`DeepLumin/src/localization/global_matcher`  
> 参考代码：`ysw_loc/src/ws_localizationv1.3/global_localization/global_localization_node.cpp`  
> 目标：将 ysw_loc 中“基于初值的全局地图/子图匹配定位”能力，改造成 DeepLumin 中独立、接口化、可替换、可被 `pose_fusion` 和 `localization_manager` 编排的 `global_matcher` 模块。

---

## 1. 模块定位

`global_matcher` 必须保留为独立模块，不能与 `relocalization` 合并。

| 模块 | 目标 | 触发方式 | 是否需要初值 | 输出语义 |
|---|---|---|---|---|
| `global_matcher` | 在已有初值附近做当前点云与地图子图匹配 | 正常定位周期触发、轻度退化触发、地图段切换触发 | 需要 | 连续定位链路中的地图校正观测 |
| `relocalization` | 无可靠初值时做全局位置恢复 | 启动、定位丢失、人工触发、严重退化 | 不强依赖 | 全局恢复/重定位事件 |

`global_matcher` 参考 ysw_loc 全局定位节点的工程思路：

```text
当前点云 + 当前 odom/fused 初值
  -> 根据初值裁剪全局地图 FOV 子图
  -> 多尺度 GICP/NDT/ICP 精匹配
  -> 计算 fitness / inlier_ratio / inlier_rmse
  -> 输出 T_map_base 或 T_map_odom 校正结果
  -> pose_fusion 接收该结果作为观测更新
```

DeepLumin 中不能照搬 ysw_loc 单文件大节点。需要将地图管理、子图裁剪、匹配器、质量评估、ROS 适配拆开。

---

## 2. ysw_loc 可参考能力拆解

ysw_loc 代码中值得迁移的能力：

| ysw_loc 能力 | 对应函数/逻辑 | DeepLumin 改造方式 |
|---|---|---|
| 当前扫描缓存 | `cb_save_cur_scan` | `ScanBuffer` 或 ROS adapter 缓存 `cloud_registered` |
| odom 初值缓存 | `cb_save_cur_odom` | 订阅 `/localization/fused_odom`，备选 `/localization/slam_odom`、`/localization/dr_odom` |
| FOV 子图裁剪 | `crop_global_map_in_FOV` | `ISubmapProvider::getSubmap(center_pose, crop_config)` |
| 多尺度 GICP | `registration_at_scale(scale*6)` + `registration_at_scale(scale*3)` | `IFineMatcher` 支持 coarse/fine 两阶段 |
| 内点率/RMSE | `computeInlierRatio` | `MatchQualityEvaluator` 统一计算质量 |
| 地图切换 | `change_map_judge`、`map_num`、`map_locate_vec` | `MapSegmentManager`，不要写入 odom twist 字段 |
| 手工初值 | `/initialpose` 回调 | 作为 `SetInitialPose` 服务或订阅 RViz initialpose |
| 可视化输出 | `/submap`、`/cur_scan_in_map` | `/localization/global_matcher/target_submap`、`aligned_scan` |

ysw_loc 中不应照搬的部分：

1. 全局变量过多，线程安全边界不清晰。
2. 地图路径、CSV 路径、map topic 名称硬编码。
3. `map_num` 塞进 `Odometry.twist.twist.linear.x`，语义错误。
4. frame_id 使用 `map1`、`camera_init1` 等临时名，不符合 DeepLumin 统一 TF。
5. 匹配成功后直接发布定位，不经过 manager/fusion 质量验收。
6. 地图切换直接 reset，缺少状态机和过渡策略。

---

## 3. 总体架构

```text
global_matcher_node
  |-- RosAdapter
  |   |-- 订阅当前点云、初值 odom、人工初值、manager 命令
  |   `-- 发布 result、status、aligned_scan、target_submap
  |
  |-- GlobalMatcherCore
  |   |-- SubmapProvider
  |   |   |-- PcdMapProvider
  |   |   |-- TopicMapProvider
  |   |   `-- SegmentMapProvider
  |   |-- FineMatcher
  |   |   |-- GicpMatcher
  |   |   |-- NdtMatcher
  |   |   `-- IcpMatcher
  |   |-- MatchQualityEvaluator
  |   `-- MapSegmentManager
  |
  `-- Config / State / Diagnostics
```

运行链路：

```text
/cloud_registered + /localization/fused_odom
  -> build GlobalMatchRequest
  -> get submap by initial pose
  -> voxel filter source/target
  -> coarse match
  -> fine match
  -> quality evaluation
  -> publish GlobalMatchResult
  -> pose_fusion consumes result if accepted
  -> localization_manager monitors status and may trigger relocalization on repeated failures
```

---

## 4. 包结构规划

```text
global_matcher/
├── CMakeLists.txt
├── package.xml
├── include/global_matcher/
│   ├── common/
│   │   ├── types.hpp
│   │   ├── config.hpp
│   │   └── math_utils.hpp
│   ├── interface/
│   │   ├── i_global_matcher.hpp
│   │   ├── i_submap_provider.hpp
│   │   ├── i_fine_matcher.hpp
│   │   └── i_match_quality_evaluator.hpp
│   ├── core/
│   │   ├── global_matcher_core.hpp
│   │   ├── map_segment_manager.hpp
│   │   └── match_quality_evaluator.hpp
│   ├── matcher/
│   │   ├── gicp_matcher.hpp
│   │   ├── ndt_matcher.hpp
│   │   └── icp_matcher.hpp
│   ├── map/
│   │   ├── pcd_submap_provider.hpp
│   │   └── topic_submap_provider.hpp
│   └── ros/
│       ├── ros_adapter.hpp
│       └── param_loader.hpp
├── src/
│   ├── core/
│   ├── matcher/
│   ├── map/
│   ├── ros/
│   └── global_matcher_node.cpp
├── config/global_matcher.yaml
└── launch/global_matcher.launch
```

---

## 5. 核心数据结构

```cpp
namespace global_matcher {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

struct Pose3d {
  double timestamp = 0.0;
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  std::string frame_id = "map";
};

struct CropBoxConfig {
  double forward = 70.0;
  double backward = 7.0;
  double left = 35.0;
  double right = 35.0;
  double down = 10.0;
  double up = 10.0;
};

struct MatcherConfig {
  std::string type = "gicp";
  int num_threads = 4;
  int max_iterations = 40;
  double transformation_epsilon = 0.01;
  double max_correspondence_distance = 2.5;
  int correspondence_randomness = 20;
  double source_voxel_leaf = 0.2;
  double target_voxel_leaf = 0.4;
  std::vector<double> coarse_to_fine_scales = {6.0, 3.0, 1.0};
};

struct QualityGateConfig {
  double accept_fitness = 0.8;
  double min_inlier_ratio = 0.55;
  double max_inlier_rmse = 0.6;
  double inlier_distance = 0.5;
  double max_delta_translation = 5.0;
  double max_delta_yaw_deg = 15.0;
  int min_source_points = 200;
  int min_target_points = 1000;
};

struct GlobalMatcherConfig {
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  double match_rate = 1.0;
  bool enable_periodic_match = true;
  CropBoxConfig crop_box;
  MatcherConfig matcher;
  QualityGateConfig quality_gate;
};

struct GlobalMatchRequest {
  double timestamp = 0.0;
  CloudConstPtr source;
  Pose3d initial_pose_map;
  std::string map_id;
  bool has_initial_pose = false;
};

struct GlobalMatchResult {
  bool success = false;
  bool converged = false;
  double timestamp = 0.0;
  Pose3d pose_map;
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
  double fitness = 1e9;
  double inlier_ratio = 0.0;
  double inlier_rmse = 1e9;
  int inlier_count = 0;
  int source_points = 0;
  int target_points = 0;
  double elapsed_ms = 0.0;
  std::string map_id;
  std::string reject_reason;
  CloudPtr aligned_source;
  CloudPtr target_submap;
};

}  // namespace global_matcher
```

---

## 6. 核心接口

### 6.1 `IGlobalMatcher`

```cpp
class IGlobalMatcher {
public:
  virtual ~IGlobalMatcher() = default;
  virtual bool initialize(const GlobalMatcherConfig& config) = 0;
  virtual GlobalMatchResult match(const GlobalMatchRequest& request) = 0;
  virtual void reset() = 0;
};
```

### 6.2 `ISubmapProvider`

```cpp
class ISubmapProvider {
public:
  virtual ~ISubmapProvider() = default;
  virtual bool initialize(const MapProviderConfig& config) = 0;
  virtual bool loadMap(const std::string& map_id, const std::string& path) = 0;
  virtual CloudPtr getSubmap(const std::string& map_id,
                             const Pose3d& center,
                             const CropBoxConfig& crop) const = 0;
  virtual std::vector<std::string> mapIds() const = 0;
};
```

实现建议：

| 实现 | 用途 |
|---|---|
| `PcdSubmapProvider` | 从 PCD 文件加载全局地图，按位姿裁剪 |
| `TopicSubmapProvider` | 兼容 ysw_loc 的 map topic 加载方式 |
| `SegmentSubmapProvider` | 按 map_id / 路段管理多个地图分段 |

### 6.3 `IFineMatcher`

```cpp
class IFineMatcher {
public:
  virtual ~IFineMatcher() = default;
  virtual bool initialize(const MatcherConfig& config) = 0;
  virtual GlobalMatchResult align(const CloudConstPtr& source,
                                  const CloudConstPtr& target,
                                  const Eigen::Matrix4d& initial_guess) = 0;
};
```

实现优先级：

1. `GicpMatcher`：优先实现，参考 ysw_loc 的 FastGICP 参数。
2. `NdtMatcher`：后续可用于井下结构化巷道。
3. `IcpMatcher`：作为简单 baseline 和调试实现。

### 6.4 `IMatchQualityEvaluator`

```cpp
class IMatchQualityEvaluator {
public:
  virtual ~IMatchQualityEvaluator() = default;
  virtual MatchQuality evaluate(const CloudConstPtr& source,
                                const CloudConstPtr& target,
                                const Eigen::Matrix4d& transform,
                                const QualityGateConfig& config) const = 0;
};
```

需要实现 ysw_loc `computeInlierRatio` 的能力：将 source 按结果变换到 map 系，对 target 建 KDTree，统计距离小于阈值的内点比例、RMSE 和均值距离。

---

## 7. 算法流程

### 7.1 周期匹配流程

```text
1. RosAdapter 缓存最新 source cloud
2. RosAdapter 缓存最新 initial pose：优先 fused_odom，其次 slam_odom，再其次 dr_odom/GNSS
3. manager 触发或周期触发 match
4. GlobalMatcherCore 检查 request 有效性
5. SubmapProvider 根据 initial_pose 裁剪 target_submap
6. FineMatcher 多尺度配准
7. QualityEvaluator 计算 inlier_ratio / inlier_rmse
8. QualityGate 判断是否接受
9. 发布 GlobalMatchResult、aligned_scan、target_submap、status
```

### 7.2 子图裁剪

参考 ysw_loc `crop_global_map_in_FOV`，但 DeepLumin 中应参数化：

```text
T_map_base = initial_pose_map
CropBox 在 base_link 局部坐标系定义：
  x: [-backward, forward]
  y: [-right, left]
  z: [-down, up]
CropBox transform = inverse(T_map_base)
从全局地图裁剪出 target_submap
```

默认配置建议：

```yaml
crop_box:
  forward: 70.0
  backward: 7.0
  left: 35.0
  right: 35.0
  down: 10.0
  up: 10.0
```

### 7.3 多尺度匹配

参考 ysw_loc：先粗分辨率，再细分辨率。

```text
initial_guess
  -> GICP(scale=6.0)
  -> GICP(scale=3.0)
  -> GICP(scale=1.0)
  -> final transform
```

伪代码：

```cpp
Eigen::Matrix4d guess = request.initial_pose_map.matrix();
for (double scale : config.matcher.coarse_to_fine_scales) {
  source_filtered = voxel(source, source_leaf * scale);
  target_filtered = voxel(target, target_leaf * scale);
  matcher.setInputSource(source_filtered);
  matcher.setInputTarget(target_filtered);
  matcher.align(guess);
  if (!matcher.converged()) return reject("not converged");
  guess = matcher.finalTransform();
}
```

### 7.4 质量门控

匹配结果必须经过质量门控，不能直接进入 fusion。

```text
accept = converged
      && source_points >= min_source_points
      && target_points >= min_target_points
      && fitness <= accept_fitness
      && inlier_ratio >= min_inlier_ratio
      && inlier_rmse <= max_inlier_rmse
      && delta_translation <= max_delta_translation
      && delta_yaw <= max_delta_yaw
```

质量门控失败时仍发布 result，但：

```text
success = false
reject_reason = 具体原因
pose_fusion 不使用该观测
localization_manager 记录失败计数
```

---

## 8. 地图管理与地图切换

ysw_loc 使用 `map_num`、`start_post_list`、`map_locate_vec` 管理多段地图。DeepLumin 中应抽象为 `MapSegmentManager`。

### 8.1 地图段配置

```yaml
maps:
  default_map_id: "mine_segment_0"
  segments:
    - id: "mine_segment_0"
      pcd: "/path/to/segment_0.pcd"
      start_pose: [0, 0, 0, 0, 0, 0]
      end_pose: [100, 0, 0, 0, 0, 0]
      switch_distance: 4.0
    - id: "mine_segment_1"
      pcd: "/path/to/segment_1.pcd"
      start_pose: [100, 0, 0, 0, 0, 0]
      end_pose: [200, 0, 0, 0, 0, 0]
      switch_distance: 4.0
```

### 8.2 切换策略

1. 根据当前 fused pose 到当前 segment end_pose 的距离判断是否接近切换点。
2. 切换前预加载下一段地图。
3. 切换时发布状态，不直接 reset fusion。
4. 切换后 global_matcher 用新 map_id 做匹配，结果通过 pose_fusion 平滑校正。
5. 切换失败时保持旧地图并上报 manager。

不要采用：

```text
odom.twist.twist.linear.x = map_num
```

应使用结构化字段：`map_id`、`segment_id`、`match_status`。

---

## 9. ROS 接口设计

### 9.1 订阅

| 话题 | 消息类型 | 必需 | 说明 |
|---|---|---|---|
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 是 | SLAM 当前注册点云，推荐 source |
| `/ouster/points` | `sensor_msgs/PointCloud2` | 否 | 原始点云 fallback，需要内部预处理 |
| `/localization/fused_odom` | `nav_msgs/Odometry` | 推荐 | 最优初值 |
| `/localization/slam_odom` | `nav_msgs/Odometry` | 否 | fused 不可用时初值 |
| `/localization/dr_odom` | `nav_msgs/Odometry` | 否 | 短时 fallback 初值 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 否 | RViz 人工初值 |
| `/localization/manager/trigger_global_match` | Service/Action | 推荐 | manager 触发匹配 |

### 9.2 发布

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` 建议新增 | 匹配结果 |
| `/localization/global_matcher/status` | `deeplumin_msgs/LocalizationStatus` 或扩展消息 | 健康状态 |
| `/localization/global_matcher/aligned_scan` | `sensor_msgs/PointCloud2` | 对齐后的 source |
| `/localization/global_matcher/target_submap` | `sensor_msgs/PointCloud2` | 裁剪目标子图 |
| `/localization/global_matcher/initial_guess` | `geometry_msgs/PoseStamped` | 调试初值 |

### 9.3 服务

| 服务 | 请求 | 响应 |
|---|---|---|
| `/localization/global_matcher/load_map` | map_id、path | success、reason |
| `/localization/global_matcher/set_map_segment` | map_id | success、current_map_id |
| `/localization/global_matcher/trigger` | optional initial pose | result |
| `/localization/global_matcher/reset` | empty | success |

---

## 10. 消息建议

建议在 `deeplumin_msgs/msg/localization/` 增加：

```text
GlobalMatchResult.msg
```

字段建议：

```text
std_msgs/Header header
bool success
bool converged
string map_id
string reject_reason
geometry_msgs/PoseWithCovariance pose
float32 fitness
float32 inlier_ratio
float32 inlier_rmse
uint32 inlier_count
uint32 source_points
uint32 target_points
float32 elapsed_ms
```

如果需要调试完整 transform，也可补充：

```text
float64[16] transform
float64 delta_translation
float64 delta_yaw_deg
```

---

## 11. 配置文件模板

```yaml
frames:
  map: "map"
  odom: "odom"
  base_link: "base_link"
  lidar: "lidar_link"

topics:
  source_cloud: "/localization/cloud_registered"
  initial_odom: "/localization/fused_odom"
  fallback_slam_odom: "/localization/slam_odom"
  fallback_dr_odom: "/localization/dr_odom"
  result: "/localization/global_matcher/result"
  status: "/localization/global_matcher/status"
  aligned_scan: "/localization/global_matcher/aligned_scan"
  target_submap: "/localization/global_matcher/target_submap"

runtime:
  enable_periodic_match: true
  match_rate: 1.0
  min_cloud_interval: 0.2
  max_initial_pose_age: 0.5

map:
  provider: "pcd"
  default_map_id: "default"
  pcd_path: "/home/hhy/2004ros/map/global_map.pcd"
  map_voxel_leaf: 0.4

crop_box:
  forward: 70.0
  backward: 7.0
  left: 35.0
  right: 35.0
  down: 10.0
  up: 10.0

matcher:
  type: "gicp"
  num_threads: 4
  max_iterations: 40
  transformation_epsilon: 0.01
  max_correspondence_distance: 2.5
  correspondence_randomness: 20
  source_voxel_leaf: 0.2
  target_voxel_leaf: 0.4
  coarse_to_fine_scales: [6.0, 3.0, 1.0]

quality_gate:
  accept_fitness: 0.8
  min_inlier_ratio: 0.55
  max_inlier_rmse: 0.6
  inlier_distance: 0.5
  max_delta_translation: 5.0
  max_delta_yaw_deg: 15.0
  min_source_points: 200
  min_target_points: 1000
```

---

## 12. 与其他模块协作

### 12.1 与 `pose_fusion`

`global_matcher` 不直接发布最终定位。它发布匹配观测，`pose_fusion` 按协方差和质量权重融合：

```text
global_matcher/result -> pose_fusion/feedGlobalMatch()
```

`pose_fusion` 应根据 result 的质量字段设置观测噪声：

```text
fitness 越小、inlier_ratio 越高、inlier_rmse 越小 -> 协方差越小
质量接近阈值 -> 协方差增大
success=false -> 不融合
```

### 12.2 与 `localization_manager`

manager 负责：

1. 触发周期匹配或按需匹配。
2. 监控连续失败次数。
3. 在 global_matcher 连续失败且 SLAM/DR 不稳定时触发 relocalization。
4. 管理地图段切换。
5. 控制 global_matcher 和 relocalization 不同时抢占 fusion reset。

### 12.3 与 `relocalization`

`global_matcher` 输出失败不等于立即重定位。建议策略：

```text
1-3 次 global_matcher 失败：继续依赖 DR+SLAM，降低 global_matcher 权重
连续 N 次失败且 fusion covariance 增大：manager 触发 relocalization
relocalization 成功：reset fusion 和 global_matcher 初值
```

---

## 13. 开发阶段规划

### P0：消息、配置、接口骨架

1. 新增 `GlobalMatchResult.msg`。
2. 新增 `global_matcher.yaml` 和 launch。
3. 建立 `types/config/interface` 头文件。
4. `global_matcher_node` 完成参数加载和空转状态发布。

### P1：地图加载与子图裁剪

1. 实现 `PcdSubmapProvider`。
2. 实现 ysw_loc 风格 FOV CropBox 裁剪。
3. 发布 `/target_submap` 验证裁剪结果。
4. 支持 `/initialpose` 手工初值调试。

### P2：GICP 匹配

1. 实现 `GicpMatcher`。
2. 支持 coarse-to-fine 多尺度匹配。
3. 发布 `/aligned_scan`。
4. 输出 transform、fitness、elapsed_ms。

### P3：质量评估和 fusion 接入

1. 实现 KDTree inlier ratio/RMSE。
2. 实现质量门控。
3. 发布结构化 `GlobalMatchResult`。
4. `pose_fusion` 接收该观测。

### P4：manager 编排和地图段切换

1. manager 触发 `global_matcher/trigger`。
2. manager 维护连续失败计数。
3. 实现 `MapSegmentManager`。
4. 地图切换和 relocalization 触发联动。

---

## 14. 测试与验收

### 14.1 离线 bag 测试

输入：

```text
/localization/cloud_registered 或原始点云
/localization/fused_odom 或 slam_odom
地图 PCD
```

检查：

1. `/target_submap` 是否随车辆移动正确裁剪。
2. `/aligned_scan` 是否贴合目标子图。
3. `fitness/inlier_ratio/inlier_rmse` 是否稳定。
4. 输出 pose 是否无大跳变。
5. 连续失败时 status 是否给出明确原因。

### 14.2 RViz 可视化

显示：

```text
Map: /localization/global_matcher/target_submap
PointCloud: /localization/global_matcher/aligned_scan
Pose: /localization/global_matcher/result
Path: /localization/fused_odom path
```

### 14.3 验收指标建议

| 指标 | 建议阈值 |
|---|---:|
| 单次匹配耗时 | < 200 ms，后续优化到 < 100 ms |
| target_submap 点数 | > 1000 |
| source 点数 | > 200 |
| inlier_ratio | > 0.55 |
| inlier_rmse | < 0.6 m |
| 连续成功率 | > 95%（正常地图覆盖区域） |

---

## 15. 实现注意事项

1. 所有输入输出必须使用统一 frame：`map`、`odom`、`base_link`、`lidar_link`。
2. 不允许硬编码绝对地图路径，必须从 YAML 或服务加载。
3. 不允许把 `map_id` 塞进 `Odometry.twist` 等非语义字段。
4. 匹配结果不能直接 reset TF，必须作为观测交给 `pose_fusion` 或由 `localization_manager` 验收。
5. 全局地图很大时，必须使用子图裁剪或分段加载，不能每帧全图匹配。
6. 点云坐标系必须明确：source 是 lidar/base/odom/map 哪个系，进入 matcher 前必须转换一致。
7. FastGICP 可作为优先实现，但接口要允许替换为 PCL GICP、NDT、ICP。
8. 质量门控必须在 core 层实现，不能只靠 RViz 观察。
