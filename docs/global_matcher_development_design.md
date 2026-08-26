# DeepLumin global_matcher 开发设计文档

> 适用模块：`DeepLumin/src/localization/global_matcher`  
> 参考代码：`ysw_loc/src/ws_localizationv1.3/global_localization/global_localization_node.cpp`  
> 目标：将 ysw_loc 中“基于初值的全局地图/子图匹配定位”能力，改造成 DeepLumin 中独立、接口化、可替换、可被 `pose_fusion` 和 `localization_manager` 编排的 `global_matcher` 模块。

---

## 0. 当前实现状态同步

截至当前实现，`global_matcher` 已经从占位节点推进到可独立运行和可视化验证的版本，本文档后续设计应以当前接口为基线继续演进。

已完成能力：

1. 已引入 `fast_gicp` 第三方 catkin 包：`DeepLumin/src/third_party/fast_gicp`，默认关闭示例程序，仅编译 `libfast_gicp.so`。
2. `global_matcher` 默认匹配后端已切换为 `fast_gicp::FastGICP`，同时保留 `gicp` 和 `icp` 作为配置回退。
3. 已实现全局 PCD 地图加载、按初值 FOV 裁剪子图、两阶段 coarse/fine 匹配、质量门控和结构化结果发布。
4. 已实现地图段运行时加载/切换接口，当前使用 `std_msgs/String` 命令话题，后续可升级为 service/action。
5. 已实现 keyframe 测试节点，可使用 `/home/hhy/2004ros/relocalization_ws/key_point_frame` 和 `optimized_poses_tum.txt` 验证完整匹配链路。
6. 已实现 RViz 对比可视化：全局地图、配准前源帧、目标子图、配准后源帧、初值和匹配轨迹。

当前验证结果：`global_matcher_keyframe_test.launch frame_index:=100` 在 `fast_gicp` 默认参数下可收敛，典型耗时约 47-50 ms，输出 `success=1`、`converged=1`、`inlier_ratio=1.0`、`inlier_rmse≈0.068`。该结果用于说明后端和接口可用，正式阈值仍需按目标地图、点云密度和车辆场景标定。

当前实现仍是单文件主节点为主，后续重构方向仍应按本文档拆分 `core/map/matcher/ros/interface`，但不能改变已发布的 topic/message 语义。

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
  -> FastGICP 默认后端两阶段匹配，PCL GICP/ICP 可配置回退
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
| 多尺度 FastGICP | `registration_at_scale(scale*6)` + `registration_at_scale(scale*3)` | 当前默认 `fast_gicp::FastGICP`，保留 PCL GICP/ICP 回退 |
| 内点率/RMSE | `computeInlierRatio` | `MatchQualityEvaluator` 统一计算质量 |
| 地图切换 | `change_map_judge`、`map_num`、`map_locate_vec` | `MapSegmentManager`，不要写入 odom twist 字段 |
| 手工初值 | `/initialpose` 回调 | 作为 `SetInitialPose` 服务或订阅 RViz initialpose |
| 可视化输出 | `/submap`、`/cur_scan_in_map` | 已实现 `source_scan_initial`、`target_submap`、`aligned_scan`、`global_map` |

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
  |   |-- 订阅当前点云、初值 odom、人工初值、地图命令
  |   `-- 发布 result、status、source_scan_initial、aligned_scan、target_submap、global_map
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
  std::string type = "fast_gicp";
  int num_threads = 4;
  int max_iterations = 20;
  double transformation_epsilon = 0.01;
  double max_correspondence_distance = 2.0;
  int correspondence_randomness = 20;
  double source_voxel_leaf = 0.1;
  double target_voxel_leaf = 0.1;
  std::vector<double> coarse_to_fine_scales = {6.0, 3.0};
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

1. `FastGicpMatcher`：当前默认实现，使用 `fast_gicp::FastGICP`，参数对齐 ysw_loc。
2. `PclGicpMatcher`：当前作为 `matcher/type: gicp` 回退路径。
3. `IcpMatcher`：当前作为 `matcher/type: icp` baseline 和调试实现。
4. `NdtMatcher`：后续可用于井下结构化巷道。

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
  -> FastGICP(scale=6.0)
  -> FastGICP(scale=3.0)
  -> final transform
```

伪代码：

```cpp
Eigen::Matrix4d guess = request.initial_pose_map.matrix();
// 当前实现固定执行 coarse_scale 与 fine_scale 两次，后续可泛化为列表。
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


### 8.3 当前已实现的地图段接口

当前 `global_matcher` 已支持单地图加载和运行时地图段加载/切换，接口使用 `std_msgs/String`，便于 manager、命令行和 rosbag 调试。

| Topic | Type | data 格式 | 说明 |
|---|---|---|---|
| `/localization/global_matcher/load_map_command` | `std_msgs/String` | `map_id|/abs/path/to/map.pcd[|activate]` | 加载地图段，`activate` 默认 true |
| `/localization/global_matcher/set_map_segment_command` | `std_msgs/String` | `map_id` | 激活已加载地图段 |

示例：

```bash
rostopic pub -1 /localization/global_matcher/load_map_command std_msgs/String \
"data: segment_outin|/home/hhy/2004ros/ysw_loc/src/map/0730map/outin_90.pcd|true"

rostopic pub -1 /localization/global_matcher/set_map_segment_command std_msgs/String \
"data: default"
```

该接口只负责加载和激活地图段，不负责判断何时切换。自动切换策略应由 `localization_manager` 根据路线、区域、方向、入口和匹配质量编排。

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
| `/localization/manager/trigger_global_match` | Service/Action | 后续扩展 | manager 触发一次匹配，当前实现以 `auto_match` 周期匹配为主 |

### 9.2 发布

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | 匹配结果，`header.frame_id=map`，`child_frame_id=base_link` |
| `/localization/global_matcher/status` | `deeplumin_msgs/LocalizationStatus` | 健康状态和当前模式 |
| `/localization/global_matcher/source_scan_initial` | `sensor_msgs/PointCloud2` | 配准前 source 按初值投到 `map` 下，红色可视化 |
| `/localization/global_matcher/aligned_scan` | `sensor_msgs/PointCloud2` | 配准后的 source，青色可视化 |
| `/localization/global_matcher/target_submap` | `sensor_msgs/PointCloud2` | 裁剪目标子图，黄色可视化 |
| `/localization/global_matcher/global_map` | `sensor_msgs/PointCloud2` | RViz 用下采样全局地图，灰色可视化 |
| `/localization/global_matcher/initial_guess` | `geometry_msgs/PoseStamped` | 调试初值 |
| `/localization/global_matcher/path` | `nav_msgs/Path` | 匹配结果轨迹 |

### 9.3 命令接口和后续服务化

当前已实现的运行时命令接口是 topic：

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/localization/global_matcher/load_map_command` | `std_msgs/String` | `map_id|pcd_path[|activate]` |
| `/localization/global_matcher/set_map_segment_command` | `std_msgs/String` | `map_id` |

后续若 manager 需要严格的请求/响应语义，可在不改变当前 topic 的前提下补充 service/action：

| 服务 | 请求 | 响应 |
|---|---|---|
| `/localization/global_matcher/load_map` | map_id、path、activate | success、reason |
| `/localization/global_matcher/set_map_segment` | map_id | success、current_map_id |
| `/localization/global_matcher/trigger` | optional initial pose | result |
| `/localization/global_matcher/reset` | empty | success |

---

## 10. 消息定义

`deeplumin_msgs/msg/localization/GlobalMatchResult.msg` 已新增并通过编译，当前字段为：

```text
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
bool success
bool converged
string map_id
string reject_reason
float64 fitness_score
float64 inlier_ratio
float64 inlier_rmse
uint32 inlier_count
uint32 source_points
uint32 target_points
float64 elapsed_ms
float64 initial_translation_error
float64 initial_yaw_error
```

语义要求：

1. `header.frame_id` 是全局地图坐标系，通常为 `map`。
2. `child_frame_id` 是匹配结果对应的车体坐标系，通常为 `base_link`。
3. `pose` 表示 `T_map_base_match`，不是 `map->odom`。
4. `initial_translation_error` 和 `initial_yaw_error` 表示匹配结果相对初值的差异，用于 `pose_fusion` 和 `localization_manager` 门控。
5. `success=false` 时也必须发布质量字段和 `reject_reason`，便于诊断和失败计数。

## 11. 当前配置文件基线

当前 `config/global_matcher.yaml` 的关键配置如下，后续开发应保持字段兼容：

```yaml
frames:
  map: map
  odom: odom
  base_link: base_link

topics:
  source_cloud: /localization/cloud_registered
  initial_pose: /localization/fused_odom
  source_odom: /localization/slam_odom
  manual_initial_pose: /initialpose
  result: /localization/global_matcher/result
  status: /localization/global_matcher/status
  source_initial_scan: /localization/global_matcher/source_scan_initial
  aligned_scan: /localization/global_matcher/aligned_scan
  target_submap: /localization/global_matcher/target_submap
  global_map: /localization/global_matcher/global_map
  initial_guess: /localization/global_matcher/initial_guess
  path: /localization/global_matcher/path
  load_map_command: /localization/global_matcher/load_map_command
  set_map_segment_command: /localization/global_matcher/set_map_segment_command

map:
  default_map_id: default
  pcd_path: /home/hhy/2004ros/ysw_loc/src/map/0730map/yuhsuwan2_loc_rot_72.pcd
  publish_voxel_leaf: 5.0
  global_publish_period: 5.0

runtime:
  auto_match: true
  viz_test_mode: false
  source_cloud_is_odom_frame: true
  match_rate: 1.0
  max_cloud_age: 1.0
  max_path_size: 3000

crop:
  forward: 70.0
  backward: 7.0
  left: 35.0
  right: 35.0
  down: 10.0
  up: 10.0

matcher:
  type: fast_gicp
  source_voxel_leaf: 0.1
  target_voxel_leaf: 0.1
  coarse_scale: 6.0
  fine_scale: 3.0
  max_iterations: 20
  transformation_epsilon: 0.01
  max_correspondence_distance: 2.0
  fast_gicp_num_threads: 2
  fast_gicp_correspondence_randomness: 20
```

`source_cloud_is_odom_frame` 的语义：

1. 在线 DeepLumin 模式为 `true`：`source_cloud` 在 `odom` 或局部里程计坐标下，节点用 `fused_odom` 与 `slam_odom` 计算 `T_map_source` 初值。
2. keyframe 测试模式为 `false`：单帧点云在 `base_link` 局部坐标下，TUM 位姿直接作为 `T_map_source` 初值。

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

## 13. 开发状态与后续计划

### 已完成

1. `GlobalMatchResult.msg`、`LocalizationStatus` 发布链路已接通。
2. PCD 地图加载、无全局匹配下采样、FOV 子图裁剪已实现。
3. `fast_gicp` 第三方包已引入，默认后端为 `fast_gicp`，保留 `gicp`/`icp` 回退。
4. 两阶段 coarse/fine 匹配、质量门控、结果轨迹、调试点云发布已实现。
5. 地图段加载/切换命令接口已实现。
6. keyframe 测试节点和 RViz 对比可视化已实现。

### 后续 P1：接口化重构

1. 将当前单文件逻辑拆分为 `core/map/matcher/ros`。
2. 固化 `IFineMatcher`、`ISubmapProvider`、`IMapSegmentManager` 接口。
3. 保持当前 topic/message/config 向后兼容。

### 后续 P2：manager 触发与服务化

1. 增加一次性 trigger service/action，保留当前 `auto_match` 周期模式。
2. 将 `load_map_command`、`set_map_segment_command` 补充为 service。
3. 增加 pending 状态，防止 manager 重复触发。

### 后续 P3：批量评估和阈值标定

1. 对多个 keyframe 和 rosbag 做成功率、fitness、inlier、RMSE 统计。
2. 按地图和车辆速度标定 `gate/*` 参数。
3. 输出用于 `pose_fusion` 的协方差映射建议。

### 后续 P4：多地图段策略联调

1. manager 根据路线和区域决定地图段切换。
2. global_matcher 只执行加载/激活/匹配，不承载路线业务逻辑。
3. pose_fusion 按 `map_id` 验收匹配结果，避免不同地图段结果混用。

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
Global map: /localization/global_matcher/global_map
Source before match: /localization/global_matcher/source_scan_initial
Target submap: /localization/global_matcher/target_submap
Aligned source: /localization/global_matcher/aligned_scan
Initial guess: /localization/global_matcher/initial_guess
Match path: /localization/global_matcher/path
Path: /localization/fused_odom path
```

### 14.3 验收指标建议

| 指标 | 建议阈值 |
|---|---:|
| 单次匹配耗时 | fast_gicp keyframe 测试约 47-50 ms；在线目标 < 100 ms |
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
7. FastGICP 已是当前默认实现，但接口要允许替换为 PCL GICP、NDT、ICP。
8. 质量门控必须在 core 层实现，不能只靠 RViz 观察。
