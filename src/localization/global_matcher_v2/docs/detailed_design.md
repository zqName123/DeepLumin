# global_matcher_v2 详细设计文档

## 1. 概述

### 1.1 项目背景
`global_matcher_v2` 是新一代全局点云匹配定位节点，用于在已知全局地图中通过当前帧点云匹配获取车辆在地图坐标系中的位姿。该节点是 localization_v2 定位链路中的关键模块，负责提供全局约束以修正里程计漂移。

### 1.2 设计目标
- **高性能**：通过预构建 target 结构（协方差点云 / KDTree / VGICP 体素地图），实现匹配帧内零 target 预处理
- **大场景支持**：支持分片地图实时加载，避免整图常驻内存
- **算法统一**：icp / gicp / vgicp 统一基于 small_gicp，支持 Huber / Cauchy 鲁棒核
- **灵活配置**：提供 fast_gicp 粗配准 + vgicp 精细配准的混合分支，兼顾收敛域与精度

### 1.3 术语定义
| 术语 | 定义 |
|------|------|
| source | 当前帧点云，待配准的局部点云 |
| target | 全局地图点云，配准的参考目标 |
| map 系 | 全局地图坐标系，通常为地图原点 |
| odom 系 | 里程计坐标系，source 点云默认所在坐标系 |
| base 系 | 车辆本体坐标系 |
| lidar 系 | 激光雷达传感器坐标系 |
| T_map_source | source 点云系到 map 系的变换矩阵（匹配求解的未知量） |
| T_map_base | base_link 到 map 系的变换矩阵（最终输出位姿） |

---

## 2. 系统架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    global_matcher_v2_node                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  ROS 订阅层  │  │  初始位姿   │  │      匹配定时器          │  │
│  │ (Source/    │→ │   选择器    │→ │   (match_rate Hz)       │  │
│  │  Odom/      │  │             │  │                         │  │
│  │  Initial)   │  │             │  │                         │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
│                                              │                   │
│                              ┌───────────────┴───────────────┐   │
│                              ▼                               ▼   │
│                    ┌─────────────────┐            ┌─────────────┐│
│                    │   MapManager    │            │  Matcher    ││
│                    │ (地图管理器)     │            │ (匹配器)    ││
│                    │                 │            │             ││
│                    │ • single/chunked│            │ • icp/gicp  ││
│                    │ • 异步加载       │            │ • vgicp     ││
│                    │ • target 缓存    │            │ • fast_gicp ││
│                    │                 │            │ • 混合分支   ││
│                    └─────────────────┘            └─────────────┘│
│                              │                       │          │
│                              └───────────┬───────────┘          │
│                                          ▼                      │
│                              ┌─────────────────────┐            │
│                              │    质量门控 (Gate)   │            │
│                              │  • 内点率 / RMSE     │            │
│                              │  • 位移/偏航阈值     │            │
│                              └─────────────────────┘            │
│                                          │                      │
│                              ┌───────────┴───────────┐          │
│                              ▼                       ▼          │
│                    ┌─────────────────┐    ┌─────────────────┐   │
│                    │   结果发布       │    │   状态发布       │   │
│                    │ (GlobalMatchResult)│   │ (LocalizationStatus)│
│                    └─────────────────┘    └─────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 模块划分

| 模块 | 代码文件 | 职责 |
|------|---------|------|
| ROS 节点层 | `global_matcher_v2_node.cpp` | 话题订阅/发布、定时器、流程编排、坐标变换 |
| 配置加载 | `global_matcher_v2_config.hpp/cpp` | ROS 参数解析、配置结构体定义 |
| 地图管理 | `map_manager.hpp/cpp` | 单图/分片地图加载、异步加载、target 结构缓存 |
| 匹配器 | `matcher.hpp/cpp`, `matcher_impl.hpp` | 粗/细两阶段配准、多种算法实现 |

---

## 3. 代码文件作用

### 3.1 头文件 (include/global_matcher_v2/)

#### 3.1.1 `core/map_manager.hpp`
- **作用**：地图管理器接口定义
- **核心类**：`MapManager`
- **数据结构**：
  - `ChunkInfo`：分片元数据（id, 网格坐标, 边界框, 路径）
  - `MapManagerConfig`：地图管理配置参数
  - `CropBoxConfig`：可视化裁剪盒配置
  - `LoadedChunk`：已加载分片（PCL点云 + small_gicp点云）

#### 3.1.2 `core/matcher.hpp`
- **作用**：匹配器接口定义
- **核心类**：`PointCloudMatcher`
- **数据结构**：
  - `MatcherParams`：匹配器配置参数（类型、体素尺度、迭代参数、鲁棒核等）
  - `MatchOutput`：匹配输出结果（粗/细变换、fitness、收敛标志）

#### 3.1.3 `core/matcher_impl.hpp`
- **作用**：匹配器模板实现（仅被 `matcher.cpp` 包含）
- **核心功能**：
  - `configureRegistration()`：统一配置 small_gicp 注册参数
  - `alignWithTree()`：基于 KDTree 的配准（icp/gicp）
  - `alignWithVoxelMap()`：基于体素地图的配准（vgicp）
  - `configureFactor()`：鲁棒核参数注入（Huber/Cauchy）

#### 3.1.4 `ros/global_matcher_v2_config.hpp`
- **作用**：节点级配置结构体定义
- **核心类**：`GlobalMatcherV2Config`
- **包含内容**：坐标系、话题名、地图参数、运行时参数、门控参数、发布开关等

### 3.2 源文件 (src/)

#### 3.2.1 `core/map_manager.cpp`
- **作用**：地图管理器实现
- **关键函数**：
  - `parseManifest()`：解析分片清单（chunks.txt / index.txt）
  - `loadChunk()`：加载单个分片（降采样 + 协方差估计）
  - `ensureActive()`：按车辆位置确保所需分片已加载
  - `rebuildActiveStructuresLocked()`：重建活动分片的 target 结构
  - `loaderThreadLoop()`：后台异步加载线程
  - `cropSubmap()`：按 FOV 裁剪可视化子图
  - `estimateGroundZ()`：人工初值 z 补全

#### 3.2.2 `core/matcher.cpp`
- **作用**：匹配器实现
- **关键函数**：
  - `match()`：粗/细两阶段配准主入口
  - `runCoarse()` / `runFine()`：粗/细阶段分发
  - `runFastGicp()`：fast_gicp 配准
  - `runGicpTree()` / `runIcpTree()`：small_gicp 基于 KDTree 的配准
  - `runVgicp()`：small_gicp 基于体素地图的配准
  - `downsample()`：质心体素降采样（哈希实现，避免 PCL int32 限制）
  - `toSmallGicpWithCov()`：PCL 点云转 small_gicp 点云并估计协方差

#### 3.2.3 `ros/global_matcher_v2_config.cpp`
- **作用**：ROS 参数加载实现
- **关键函数**：
  - `loadGlobalMatcherV2Config()`：从 ROS Parameter Server 加载全部配置
  - 辅助函数：`loadBool()`, `loadDouble()`, `loadInt()`, `loadString()`, `loadVec3()`, `loadExtrinsic()`

#### 3.2.4 `ros/global_matcher_v2_node.cpp`
- **作用**：ROS 节点主程序
- **核心类**：`GlobalMatcherV2Node`
- **关键函数**：
  - 订阅回调：`onSourceCloud()`, `onTrackingInitialOdom()`, `onSourceOdom()`, `onBootstrapInitialPose()`, `onManualInitialPose()`, `onLoadMapCommand()`, `onSetMapSegmentCommand()`
  - 定时器：`onMatchTimer()`, `onMapTimer()`
  - 主流程：`runMatch()`
  - 初值选择：`selectInitialGuess()`
  - 质量评估：`computeInliers()`
  - 发布函数：`publishResult()`, `publishStatus()`, `publishGlobalMap()`, `publishAligned()`, 等

### 3.3 工具文件

#### 3.3.1 `src/tools/bench_voxel_build.cpp`
- **作用**：VGICP 体素地图构建性能基准测试工具

### 3.4 配置文件

#### 3.4.1 `config/global_matcher_v2.yaml`
- **作用**：节点默认配置文件
- **内容**：坐标系、话题、地图、运行时、匹配器、门控、结果、发布开关等全部参数

### 3.5 启动文件

#### 3.5.1 `launch/global_matcher_v2.launch`
- **作用**：独立运行启动文件

#### 3.5.2 `launch/localization_v2_with_gmv2.launch`
- **作用**：替换 localization_v2 链路中的 global_matcher

---

## 4. 接口设计

### 4.1 ROS 输入接口（订阅话题）

| 话题名 | 消息类型 | 含义 | 坐标系 | 是否必需 |
|--------|---------|------|--------|---------|
| `/localization/cloud_registered_effect` | `sensor_msgs/PointCloud2` | 当前帧点云 | odom/lidar/base | 是 |
| `/localization/fused_odom` | `nav_msgs/Odometry` | 融合里程计位姿（跟踪初值） | map→base_link | 否（TRACKING模式） |
| `/localization/slam_odom` | `nav_msgs/Odometry` | 源里程计 | odom→base_link | 否（source在odom系时） |
| `/localization/manager/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 引导初值 | map→base_link | 否（BOOTSTRAP模式） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 人工初值（RViz） | map→base_link | 否 |
| `/localization/global_matcher/load_map_command` | `std_msgs/String` | 热加载地图命令 | - | 否 |
| `/localization/global_matcher/set_map_segment_command` | `std_msgs/String` | 设置地图分片命令 | - | 否（chunked模式为no-op） |

### 4.2 ROS 输出接口（发布话题）

| 话题名 | 消息类型 | 含义 | 发布条件 |
|--------|---------|------|---------|
| `/localization/global_matcher/result` | `deeplumin_msgs/GlobalMatchResult` | 匹配结果位姿 | 每帧匹配后 |
| `/localization/global_matcher/status` | `deeplumin_msgs/LocalizationStatus` | 定位状态 | `publish.status=true` |
| `/localization/global_matcher/global_map` | `sensor_msgs/PointCloud2` | 全局地图（降采样） | `publish.global_map=true`，latch |
| `/localization/global_matcher/path` | `nav_msgs/Path` | 匹配轨迹 | `publish.path=true` |
| `/localization/global_matcher/aligned_scan` | `sensor_msgs/PointCloud2` | 配准后点云 | `publish.aligned_scan=true` |
| `/localization/global_matcher/source_scan_initial` | `sensor_msgs/PointCloud2` | 初值变换后点云 | `publish.source_initial_scan=true` |
| `/localization/global_matcher/target_submap` | `sensor_msgs/PointCloud2` | 裁剪子图 | `publish.target_submap=true` |
| `/localization/global_matcher/initial_guess` | `geometry_msgs/PoseStamped` | 初值位姿 | `publish.initial_guess=true` |

### 4.3 内部接口

#### 4.3.1 MapManager 接口

```cpp
// 配置与初始化
bool configure(const MapManagerConfig& config);
bool loadInitial();
bool reloadSingleMap(const std::string& pcd_path);

// 运行时
bool ensureActive(const Eigen::Vector3d& position);  // 确保车辆位置周边分片已加载
bool mapReady() const;
bool mapChanged() const;
void consumeMapChanged();

// Target 结构访问（线程安全，mutex_保护）
small_gicp::PointCloud::ConstPtr targetCloud() const;
pcl::PointCloud<pcl::PointXYZ>::ConstPtr targetPcl() const;
std::shared_ptr<const small_gicp::KdTree<small_gicp::PointCloud>> targetKdTree() const;
small_gicp::GaussianVoxelMap::ConstPtr targetVoxelMapCoarse() const;
small_gicp::GaussianVoxelMap::ConstPtr targetVoxelMapFine() const;

// 工具
pcl::PointCloud<pcl::PointXYZ>::Ptr cropSubmap(const Eigen::Matrix4f& map_base, const CropBoxConfig& crop) const;
bool estimateGroundZ(double x, double y, double radius, double* z_out) const;
```

#### 4.3.2 PointCloudMatcher 接口

```cpp
void configure(const MatcherParams& params);

MatchOutput match(
    const pcl::PointCloud<pcl::PointXYZ>& source_pcl,      // 源点云
    const small_gicp::PointCloud& target,                  // target small_gicp点云
    const pcl::PointCloud<pcl::PointXYZ>& target_pcl,      // target PCL点云
    const small_gicp::KdTree<small_gicp::PointCloud>& target_kdtree,  // target KDTree
    const small_gicp::GaussianVoxelMap* coarse_voxelmap,   // 粗体素地图（可为空）
    const small_gicp::GaussianVoxelMap* fine_voxelmap,     // 细体素地图（可为空）
    const Eigen::Matrix4f& init_T                          // T_map_source 初始猜测
) const;
```

---

## 5. 数据结构设计

### 5.1 配置数据结构

#### 5.1.1 GlobalMatcherV2Config
```cpp
struct GlobalMatcherV2Config {
  // 坐标系
  std::string map_frame, odom_frame, base_frame, lidar_frame;
  Eigen::Matrix4f base_to_lidar;

  // 话题名（约15个话题）
  std::string source_cloud_topic, initial_pose_topic, ...;

  // 地图管理参数
  std::string map_mode;  // "single" | "chunked"
  std::string map_pcd_path, map_dir;
  double chunk_size, load_voxel_leaf;
  int load_radius_chunks, max_loaded_chunks;
  int cov_num_neighbors, cov_num_threads;
  double vgicp_voxel_resolution, vgicp_coarse_scale, vgicp_fine_scale;

  // 运行时参数
  bool auto_match, source_cloud_is_odom_frame;
  std::string source_cloud_frame;  // "auto" | "odom" | "base_link" | "lidar"
  double match_rate, max_cloud_age;
  bool use_receive_time_for_cloud_age;
  bool use_tracking_initial_pose, use_bootstrap_initial_pose, ...;
  Eigen::Vector3f default_initial_translation, default_initial_rpy_deg;

  // 裁剪盒（仅可视化）
  CropBoxConfig crop;

  // 匹配器参数
  MatcherParams matcher;

  // 质量门控
  int min_source_points, min_target_points;
  double max_fitness_score, min_inlier_ratio, max_inlier_rmse;
  double inlier_distance, max_delta_translation, max_delta_yaw_deg;

  // 结果与发布
  std::string result_pose_frame;  // "odom" | "base"
  bool publish_global_map, publish_target_submap, ...;
};
```

#### 5.1.2 MatcherParams
```cpp
struct MatcherParams {
  std::string type;           // "icp" | "gicp" | "vgicp" | "fast_gicp" | "fast_gicp_vgicp"
  bool use_crop;              // 是否按FOV裁剪target

  // 两阶段降采样
  double source_voxel_leaf, target_voxel_leaf;
  double coarse_scale, fine_scale;

  // 迭代参数
  int max_iterations;
  double transformation_epsilon;
  double max_correspondence_distance;
  double coarse_max_correspondence_distance, fine_max_correspondence_distance;

  // 鲁棒核
  std::string robust_kernel;  // "none" | "huber" | "cauchy"
  double robust_kernel_scale;

  // 线程与协方差
  int num_threads, cov_num_neighbors, cov_num_threads;
  int fast_gicp_num_threads, fast_gicp_correspondence_randomness;

  // VGICP 参数
  double vgicp_voxel_resolution, vgicp_coarse_scale, vgicp_fine_scale;
};
```

#### 5.1.3 MapManagerConfig
```cpp
struct MapManagerConfig {
  std::string mode;           // "single" | "chunked"
  std::string pcd_path, map_dir;
  std::string chunks_file, index_file;
  double chunk_size;
  int load_radius_chunks;
  double load_voxel_leaf;
  int max_loaded_chunks;
  bool async_load;
  int cov_num_neighbors, cov_num_threads;
  bool prebuild_voxel_maps;
  double vgicp_voxel_resolution, vgicp_coarse_scale, vgicp_fine_scale;
};
```

### 5.2 运行时数据结构

#### 5.2.1 MatchOutput（匹配结果）
```cpp
struct MatchOutput {
  Eigen::Matrix4f coarse;     // 粗配准结果 T_map_source
  Eigen::Matrix4f final;      // 精细配准结果 T_map_source
  double coarse_fitness;      // 粗配准误差
  double fine_fitness;        // 精细配准误差
  bool coarse_converged;      // 粗配准是否收敛
  bool fine_converged;        // 精细配准是否收敛
};
```

#### 5.2.2 MatchMetrics（质量指标）
```cpp
struct MatchMetrics {
  uint32_t total_points;      // 有效总点数
  uint32_t inlier_count;      // 内点数
  double inlier_ratio;        // 内点率 = inlier_count / total_points
  double inlier_rmse;         // 内点均方根误差
  double fitness;             // 整体 fitness（所有点平均距离平方）
};
```

#### 5.2.3 InitialGuess（初始猜测）
```cpp
struct InitialGuess {
  Eigen::Matrix4f map_source;   // T_map_source（匹配初值）
  Eigen::Matrix4f source_base;  // T_source_base（source到base的变换）
  Eigen::Matrix4f map_base;     // T_map_base（车辆在map中的位姿初值）
  MatcherRunMode mode;          // TRACKING | BOOTSTRAP
  std::string reason;           // 初值来源说明
  bool valid;                   // 是否有效
};
```

#### 5.2.4 ChunkInfo（分片元数据）
```cpp
struct ChunkInfo {
  int id;                       // 分片ID
  int gx, gy;                   // 网格坐标
  std::string path;             // 相对map_dir的路径
  Eigen::Vector3d min_pt;       // 边界最小值
  Eigen::Vector3d max_pt;       // 边界最大值
  size_t point_count;           // 点数
};
```

#### 5.2.5 LoadedChunk（已加载分片）
```cpp
struct LoadedChunk {
  ChunkInfo info;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl;  // PCL点云（降采样后）
  small_gicp::PointCloud::Ptr sg;           // small_gicp点云（含协方差）
};
```

### 5.3 坐标系变换关系

```
map（全局地图坐标系）
  └── odom（里程计坐标系，source_cloud_is_odom_frame=true时）
       └── base_link（车辆本体坐标系）
            └── lidar_link（激光雷达坐标系，source_cloud_frame=lidar时）

匹配求解未知量：T_map_source

source在odom系：  T_map_source = T_map_base × T_odom_base⁻¹
source在lidar系： T_map_source = T_map_base × T_base_lidar
source在base系：  T_map_source = T_map_base

匹配完成后：
T_map_base = T_map_source × T_source_base
```

---

## 6. 配置文件参数详解

### 6.1 坐标系与外参 (frames / extrinsics)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `frames/map` | string | "map" | 全局地图坐标系名 |
| `frames/odom` | string | "odom" | 里程计坐标系名 |
| `frames/base_link` | string | "base_link" | 车辆本体坐标系名 |
| `frames/lidar` | string | "lidar_link" | 激光雷达坐标系名 |
| `extrinsics/base_to_lidar/translation` | float[3] | [0,0,0] | base到lidar的平移 |
| `extrinsics/base_to_lidar/rotation` | float[9] | 单位阵 | base到lidar的旋转矩阵 |

### 6.2 话题配置 (topics)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `topics/source_cloud` | `/localization/cloud_registered_effect` | 输入点云话题 |
| `topics/initial_pose` | `/localization/fused_odom` | 跟踪初值（融合里程计） |
| `topics/source_odom` | `/localization/slam_odom` | 源里程计 |
| `topics/bootstrap_initial_pose` | `/localization/manager/initial_pose` | 引导初值 |
| `topics/manual_initial_pose` | `/initialpose` | 人工初值（RViz） |
| `topics/result` | `/localization/global_matcher/result` | 匹配结果 |
| `topics/status` | `/localization/global_matcher/status` | 定位状态 |
| `topics/global_map` | `/localization/global_matcher/global_map` | 全局地图可视化 |
| `topics/path` | `/localization/global_matcher/path` | 轨迹可视化 |
| `topics/load_map_command` | `/localization/global_matcher/load_map_command` | 热加载命令 |

### 6.3 地图管理 (map)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `map/mode` | string | "single" | 地图模式：single（整图）/ chunked（分片） |
| `map/pcd_path` | string | "" | single模式：整图PCD路径 |
| `map/map_dir` | string | "" | chunked模式：分片目录 |
| `map/chunk_size` | double | 100.0 | 分片边长（米） |
| `map/load_radius_chunks` | int | 1 | 加载半径（分片数），加载(2r+1)²个分片 |
| `map/load_voxel_leaf` | double | 0.4 | 分片加载时体素降采样分辨率 |
| `map/max_loaded_chunks` | int | 64 | 内存中最多保留分片数（LRU淘汰） |
| `map/cov_num_neighbors` | int | 20 | 协方差估计近邻数 |
| `map/cov_num_threads` | int | 4 | 协方差估计线程数 |
| `map/vgicp_voxel_resolution` | double | 0.5 | VGICP体素分辨率 |
| `map/vgicp_coarse_scale` | double | 10.0 | 粗体素尺度倍数 |
| `map/vgicp_fine_scale` | double | 1.0 | 细体素尺度倍数 |
| `map/global_map_publish_voxel_leaf` | double | 5.0 | 全局地图可视化降采样 |
| `map/global_map_publish_period` | double | 0.0 | 全局地图重发周期（秒，0表示不重发） |

### 6.4 运行时 (runtime)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `runtime/auto_match` | bool | true | 是否自动匹配 |
| `runtime/source_cloud_is_odom_frame` | bool | true | source点云是否在odom系 |
| `runtime/source_cloud_frame` | string | "odom" | source点云坐标系：auto/odom/base_link/lidar |
| `runtime/match_rate` | double | 10.0 | 匹配频率（Hz） |
| `runtime/max_cloud_age` | double | 1.0 | 点云最大允许年龄（秒） |
| `runtime/use_receive_time_for_cloud_age` | bool | true | 是否用接收时间判断点云时效 |
| `runtime/use_tracking_initial_pose` | bool | true | 是否使用跟踪初值 |
| `runtime/use_bootstrap_initial_pose` | bool | true | 是否使用引导初值 |
| `runtime/use_manual_initial_pose` | bool | true | 是否使用人工初值 |
| `runtime/use_default_initial_pose` | bool | true | 是否使用默认初值 |
| `runtime/allow_identity_bootstrap` | bool | true | 是否允许单位阵初值 |
| `runtime/default_initial_pose/translation` | float[3] | [0,0,0] | 默认初值平移 |
| `runtime/default_initial_pose/rpy_deg` | float[3] | [0,0,0] | 默认初值欧拉角（度） |

### 6.5 人工初值Z补全 (manual_initial_pose)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `manual_initial_pose/z_search_radius` | double | 10.0 | 人工2D初值用该xy附近地图点云z中值补全 |

### 6.6 可视化裁剪 (crop)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `crop/forward` | double | 70.0 | 前方裁剪距离（米） |
| `crop/backward` | double | 7.0 | 后方裁剪距离（米） |
| `crop/left` | double | 35.0 | 左侧裁剪距离（米） |
| `crop/right` | double | 35.0 | 右侧裁剪距离（米） |
| `crop/down` | double | 35.0 | 下方裁剪距离（米） |
| `crop/up` | double | 35.0 | 上方裁剪距离（米） |

**注意**：crop参数仅用于`publish.target_submap`可视化，不参与匹配（与v1不同）。

### 6.7 匹配器 (matcher)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `matcher/type` | string | "gicp" | 算法类型：icp/gicp/vgicp/fast_gicp/fast_gicp_vgicp |
| `matcher/use_crop` | bool | false | 是否按FOV裁剪target（v1行为，fast_gicp推荐） |
| `matcher/source_voxel_leaf` | double | 0.08 | source降采样体素大小 |
| `matcher/target_voxel_leaf` | double | 0.08 | target降采样体素大小 |
| `matcher/coarse_scale` | double | 8.0 | 粗配准体素尺度倍数 |
| `matcher/fine_scale` | double | 1.0 | 精细配准体素尺度倍数 |
| `matcher/max_iterations` | int | 50 | 最大迭代次数 |
| `matcher/transformation_epsilon` | double | 0.01 | 变换收敛阈值 |
| `matcher/max_correspondence_distance` | double | 2.0 | 最大对应距离（米） |
| `matcher/coarse_max_correspondence_distance` | double | 0.0 | 粗配准最大对应距离（0=auto） |
| `matcher/fine_max_correspondence_distance` | double | 0.0 | 精细配准最大对应距离（0=auto） |
| `matcher/robust_kernel` | string | "none" | 鲁棒核：none/huber/cauchy |
| `matcher/robust_kernel_scale` | double | 5.0 | 鲁棒核宽度c |
| `matcher/num_threads` | int | 4 | small_gicp线程数 |
| `matcher/cov_num_neighbors` | int | 20 | source协方差估计近邻数 |
| `matcher/cov_num_threads` | int | 4 | source协方差估计线程数 |
| `matcher/fast_gicp_num_threads` | int | 4 | fast_gicp线程数 |
| `matcher/fast_gicp_correspondence_randomness` | int | 20 | fast_gicp对应随机性 |
| `matcher/vgicp_voxel_resolution` | double | 0.5 | VGICP体素分辨率（记录/校验用） |
| `matcher/vgicp_coarse_scale` | double | 10.0 | VGICP粗体素尺度 |
| `matcher/vgicp_fine_scale` | double | 1.0 | VGICP细体素尺度 |

### 6.8 质量门控 (gate)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `gate/min_source_points` | int | 50 | source点数下限 |
| `gate/min_target_points` | int | 100 | target点数下限 |
| `gate/max_fitness_score` | double | 100.0 | 最大fitness分数（仅fast_gicp生效） |
| `gate/min_inlier_ratio` | double | 0.7 | 最小内点率 |
| `gate/max_inlier_rmse` | double | 0.4 | 最大内点RMSE（米） |
| `gate/inlier_distance` | double | 0.5 | 内点判定距离阈值（米） |
| `gate/max_delta_translation` | double | 20.0 | 相对初值最大位移（米） |
| `gate/max_delta_yaw_deg` | double | 20.0 | 相对初值最大偏航角（度） |

### 6.9 结果输出 (result)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `result/pose_frame` | string | "odom" | 结果位姿坐标系：odom（map→odom）/ base（map→base_link） |

### 6.10 发布开关 (publish)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `publish/global_map` | bool | true | 发布全局地图（latch） |
| `publish/target_submap` | bool | true | 发布裁剪子图 |
| `publish/aligned_scan` | bool | false | 发布配准后点云 |
| `publish/source_initial_scan` | bool | false | 发布初值变换后点云 |
| `publish/initial_guess` | bool | false | 发布初值位姿 |
| `publish/path` | bool | false | 发布轨迹 |
| `publish/status` | bool | true | 发布定位状态 |

---

## 7. 详细运行流程

### 7.1 节点初始化流程

```
1. 加载配置 (loadGlobalMatcherV2Config)
   └── 从 ROS Parameter Server 读取全部参数

2. 初始化 MapManager
   ├── configure(mm_cfg)
   │   └── 设置地图模式、路径、分片参数等
   └── loadInitial()
       ├── single模式：加载整图PCD，降采样+协方差估计，重建target结构
       └── chunked模式：解析分片清单(chunks.txt/index.txt)，不加载地图

3. 初始化 Matcher
   └── configure(matcher_params)

4. 创建发布器（按 publish.* 开关条件创建）
   ├── result_pub（必需）
   ├── status_pub（publish.status）
   ├── global_map_pub（publish.global_map）
   ├── aligned_pub（publish.aligned_scan）
   ├── source_initial_pub（publish.source_initial_scan）
   ├── submap_pub（publish.target_submap）
   ├── initial_guess_pub（publish.initial_guess）
   └── path_pub（publish.path）

5. 创建订阅器
   ├── source_sub（source_cloud）
   ├── tracking_initial_sub（initial_pose）
   ├── source_odom_sub（source_odom）
   ├── bootstrap_initial_pose_sub
   ├── manual_initial_pose_sub
   ├── load_map_command_sub
   └── set_map_segment_command_sub

6. 创建定时器
   ├── match_timer（match_rate Hz）
   └── map_timer（global_map_publish_period，可选）
```

### 7.2 匹配主流程 (runMatch)

```
① 输入快照
   └── 加锁获取：latest_source_, tracking_map_base_, bootstrap_map_base_,
       source_odom_, have_*, manual_reset_seq_

② 输入检查
   ├── source为空？→ 拒绝("waiting source cloud")
   ├── 点云超时？→ 拒绝("source cloud timeout")
   └── source点数不足？→ 拒绝("source points below threshold")

③ 初始位姿选择 (selectInitialGuess)
   └── 详见 §8 初值选择

④ 地图就绪检查 (ensureActive)
   ├── 按当前位置计算网格坐标
   ├── 确定需加载的分片集合（(2r+1)²个）
   ├── 后台异步加载缺失分片
   ├── 同步等待当前分片加载完成
   ├── 活动集合变化？→ 重建target结构
   └── 返回地图是否就绪

⑤ Target 结构准备
   ├── use_crop=true：按FOV裁剪活动地图，构建target结构
   └── use_crop=false：使用MapManager缓存的target结构

⑥ 执行匹配 (matcher_.match)
   ├── 粗配准（coarse_scale > 1时）
   │   ├── source降采样：source_voxel_leaf * coarse_scale
   │   └── 执行粗配准算法
   └── 精细配准
       ├── source降采样：source_voxel_leaf * fine_scale
       └── 执行精细配准算法（初值=粗配准结果）

⑦ 内点统计 (computeInliers)
   ├── 将source按final_map_source变换
   ├── 对target KDTree做最近邻搜索
   ├── 距离≤inlier_distance记为内点
   └── 计算：内点率、内点RMSE、fitness

⑧ 质量门控
   ├── fast_gicp类型：检查fitness_score
   ├── 检查内点率 ≥ min_inlier_ratio
   ├── 检查内点RMSE ≤ max_inlier_rmse
   ├── 检查位移偏差 ≤ max_delta_translation
   └── 检查偏航偏差 ≤ max_delta_yaw_deg

⑨ 结果处理
   ├── 检查manual_reset_seq是否变化（防止旧结果覆盖新锚点）
   ├── 成功：更新last_map_source_（下一帧TRACKING初值）
   ├── 失败：递增连续失败计数
   └── 更新运行模式（TRACKING/BOOTSTRAP）

⑩ 发布结果
   ├── publishResult（GlobalMatchResult）
   ├── publishStatus（LocalizationStatus）
   ├── publishAligned（配准后点云）
   ├── publishSubmap（裁剪子图）
   └── 更新path（轨迹）
```

### 7.3 定时器触发流程

```
onMatchTimer (match_rate Hz, 默认10Hz)
├── auto_match=false？→ 发布警告状态，返回
└── 调用 runMatch()

onMapTimer (global_map_publish_period, 默认不重发)
└── 调用 publishGlobalMap()
```

---

## 8. 状态切换

### 8.1 运行模式定义

```cpp
enum class MatcherRunMode { BOOTSTRAP, TRACKING };
```

| 模式 | 含义 | 初值来源 |
|------|------|---------|
| **BOOTSTRAP** | 引导/初始化模式 | bootstrap初值 / 默认初值 / 单位阵 |
| **TRACKING** | 跟踪模式 | 上次成功结果 / 融合里程计 |

### 8.2 模式切换状态机

```
                    ┌─────────────────────────────────────┐
                    │                                     │
                    ▼                                     │
┌──────────┐   有tracking/   ┌──────────┐   匹配成功    ┌──────────┐
│  BOOTSTRAP │ ─────────────→ │ TRACKING │ ───────────→ │ TRACKING │
│  (初始)    │   last_result   │  (跟踪)  │               │  (持续)  │
└──────────┘                 └──────────┘               └──────────┘
     ▲                            │                          │
     │                            │ 匹配连续失败N次           │
     │                            │（当前实现不自动回落）      │
     │                            ▼                          │
     └──────────────────────┐  BOOTSTRAP  ◄──────────────────┘
                            │  (回落)     │  （当前不自动回落）
                            └─────────────┘
```

### 8.3 模式切换规则

| 当前模式 | 条件 | 下一模式 | 说明 |
|---------|------|---------|------|
| BOOTSTRAP | `use_tracking=true` 且 (有fused_odom 或 有last_map_source_) | TRACKING | 获得跟踪能力后切换 |
| TRACKING | 匹配成功 | TRACKING | 保持跟踪，更新last_map_source_ |
| TRACKING | 匹配失败 | TRACKING | **不自动回落BOOTSTRAP**，继续使用fused_odom作为初值 |

**设计意图**：
- TRACKING模式下即使匹配失败也继续使用fused_odom作为初值，避免跳回地图原点
- 失败只拒绝观测，不影响下一帧的初值选择
- 连续失败计数用于监控，但不触发模式回落

### 8.4 状态监控

```cpp
int consecutive_failures_ = 0;  // 连续失败计数
std::string current_mode_ = "bootstrap";  // 当前模式字符串
```

---

## 9. 初值选择

### 9.1 初值来源优先级

```
TRACKING模式（use_tracking_initial_pose=true 且 (have_tracking || have_last_map_source_)）
├── 1. 上次成功结果 (last_map_source_)
│   └── 条件：have_last_map_source_ = true
│   └── 初值：T_map_source = last_map_source_
│   └── 说明：串行延续，无消息回路延迟，优先于fused_odom
│
└── 2. 融合里程计 (fused_odom)
    └── 条件：have_tracking = true
    └── 初值：T_map_base = /localization/fused_odom 位姿
    └── 说明：来自localization_v2输出

BOOTSTRAP模式（TRACKING不可用）
├── 3. 引导初值 (bootstrap_initial_pose)
│   └── 条件：use_bootstrap=true 且 have_bootstrap=true
│   └── 初值：T_map_base = bootstrap位姿
│
├── 4. 默认初值 (default_initial_pose)
│   └── 条件：use_default=true
│   └── 初值：T_map_base = 配置的translation + rpy_deg
│
└── 5. 单位初值 (identity)
    └── 条件：allow_identity=true
    └── 初值：T_map_base = I

不可用 → "bootstrap:no_initial_available"，本帧拒绝
```

### 9.2 坐标系转换

根据 `source_cloud_frame` 配置，将 `T_map_base` 转换为 `T_map_source`：

| source坐标系 | T_source_base | T_map_source |
|-------------|---------------|--------------|
| odom | T_odom_base | T_map_base × T_odom_base⁻¹ |
| lidar | T_base_lidar⁻¹ | T_map_base × T_base_lidar |
| base_link | I | T_map_base |

### 9.3 人工初值特殊处理

收到 `manual_initial_pose` 时：

```cpp
// 1. Z补全：用地图点云在xy附近的z中值补全人工2D初值
if (estimateGroundZ(x, y, z_search_radius, &init_z)) {
    manual_pose(2,3) = init_z;
}

// 2. 立即更新last_map_source_（使下一帧进入TRACKING）
if (have_source_odom_) {
    last_map_source_ = manual_pose × source_odom_base⁻¹;
    have_last_map_source_ = true;
}

// 3. 递增manual_reset_seq_，防止旧匹配结果覆盖新锚点
++manual_reset_seq_;
```

---

## 10. 质量控制

### 10.1 输入质量检查

| 检查项 | 阈值参数 | 失败处理 |
|--------|---------|---------|
| source点云存在 | - | 拒绝("waiting source cloud") |
| 点云时效 | `max_cloud_age` | 拒绝("source cloud timeout") |
| source点数 | `min_source_points` | 拒绝("source points below threshold") |
| target点数 | `min_target_points` | 拒绝("target map points below threshold") |
| 地图就绪 | - | 拒绝("map chunks not ready") |

### 10.2 匹配结果质量门控

| 检查项 | 阈值参数 | 适用算法 | 失败原因 |
|--------|---------|---------|---------|
| Fitness Score | `max_fitness_score` | 仅fast_gicp | "fitness score above threshold" |
| 内点率 | `min_inlier_ratio` | 全部 | "inlier ratio below threshold" |
| 内点RMSE | `max_inlier_rmse` | 全部 | "inlier rmse above threshold" |
| 位移偏差 | `max_delta_translation` | 全部 | "translation delta above threshold" |
| 偏航偏差 | `max_delta_yaw_deg` | 全部 | "yaw delta above threshold" |
| 收敛标志 | - | 全部 | 警告但不拒绝（converged=false但门控通过） |

### 10.3 内点统计方法

```cpp
// 1. 将source按final_map_source变换到map系
pcl::transformPointCloud(source, *transformed, map_source);

// 2. 对target KDTree做最近邻搜索
for (each point in transformed) {
    kdtree.nearest_neighbor_search(point, &k_index, &k_sq_dist);
    
    // 3. 距离≤inlier_distance_sq记为内点
    if (k_sq_dist <= threshold_sq) {
        ++inlier_count;
        sum_sq += k_sq_dist;  // 用于RMSE
    }
    ++total_points;
    sum_all_sq += k_sq_dist;  // 用于fitness
}

// 4. 计算指标
inlier_ratio = inlier_count / total_points;
inlier_rmse = sqrt(sum_sq / inlier_count);
fitness = sum_all_sq / total_points;
```

### 10.4 协方差构造

```cpp
// 位置协方差：基于inlier_rmse
cov_pos = max(0.01, inlier_rmse * inlier_rmse);
covariance[0] = cov_pos;   // x
covariance[7] = cov_pos;   // y
covariance[14] = max(0.04, cov_pos);  // z

// 旋转协方差：固定小值
covariance[21] = 0.05;  // roll
covariance[28] = 0.05;  // pitch

// 偏航协方差：基于delta_yaw
cov_yaw = pow(max(0.5, abs(delta_yaw_deg)) * deg_to_rad, 2);
covariance[35] = cov_yaw;  // yaw
```

### 10.5 连续失败处理

```cpp
void updateModeAfterMatch(bool success) {
    if (success) {
        consecutive_failures_ = 0;
    } else {
        ++consecutive_failures_;
    }
}
```

- 连续失败不触发模式自动回落
- 连续失败计数仅用于监控和日志
- TRACKING模式下失败仍继续使用fused_odom作为下一帧初值

---

## 11. 地图管理详细设计

### 11.1 两种地图模式对比

| 特性 | single模式 | chunked模式 |
|------|-----------|------------|
| 加载方式 | 启动时一次性加载整图PCD | 解析清单，按位置实时加载周边分片 |
| 启动耗时 | 高（大PCD读取+降采样+协方差） | 低（只解析清单） |
| 内存占用 | 整图常驻 | 仅活动分片（LRU上限） |
| 运行期I/O | 无 | 边界跨越时后台异步加载 |
| target结构 | 启动时预构建，运行期不变 | 活动集合变化时重建 |
| 适用场景 | 小场景、测试 | 大场景、量产 |

### 11.2 分片加载策略

```
1. 计算当前网格坐标
   gx = floor(x / chunk_size + 0.5)
   gy = floor(y / chunk_size + 0.5)

2. 确定需加载分片集合
   needed = {(gx', gy') : |gx'-gx| ≤ load_radius_chunks 且 |gy'-gy| ≤ load_radius_chunks}

3. 异步加载缺失分片
   - 将缺失分片ID加入加载队列
   - 后台线程从队列取ID并加载

4. 同步等待当前分片
   - 当前所在分片必须加载完成（最多等5秒）
   - 邻接分片预加载不阻塞匹配

5. 更新活动集合
   - 活动集合 = 已加载的needed分片
   - 集合变化时重建target结构

6. LRU卸载
   - 当loaded_.size() > max_loaded_chunks时
   - 卸载最久未使用的非活动分片
```

### 11.3 Target 结构缓存

活动分片集合变化时（`rebuildActiveStructuresLocked`）构建：

| 结构 | 类型 | 用途 | 构建成本 |
|------|------|------|---------|
| `active_sg_` | `small_gicp::PointCloud` | small_gicp gicp/vgicp/icp target | 合并分片点云 |
| `active_pcl_` | `pcl::PointCloud` | fast_gicp target、可视化 | 合并分片点云 |
| `active_kdtree_` | `KdTree<PointCloud>` | gicp/icp最近邻、内点统计 | 构建KDTree |
| `active_voxel_coarse_` | `GaussianVoxelMap` | vgicp粗配准 | 体素化插入 |
| `active_voxel_fine_` | `GaussianVoxelMap` | vgicp精细配准 | 体素化插入 |

**关键设计**：这些结构只在活动分片集合变化时重建一次，匹配帧内零target预处理。

### 11.4 分片清单格式

#### chunks.txt（推荐格式）
```
# global_matcher_v2 chunk manifest
chunk_size 100.000000
chunk <id> <gx> <gy> <min_x> <min_y> <min_z> <max_x> <max_y> <max_z> <points> <path>
```

#### index.txt（兼容旧格式，lightning-lm使用）
```
0 0 0  # 首行占位符
<id> <gx> <gy> <path>
```

---

## 12. 匹配算法详细设计

### 12.1 算法类型对比

| 类型 | 粗配准 | 精细配准 | 鲁棒核 | 适用场景 |
|------|--------|---------|--------|---------|
| `icp` | small_gicp ICP | small_gicp ICP | ✓ | 简单场景、初值好 |
| `gicp` | small_gicp GICP | small_gicp GICP | ✓ | 大target、精度要求高 |
| `vgicp` | small_gicp VGICP | small_gicp VGICP | ✓ | 大target、性能要求高 |
| `fast_gicp` | fast_gicp | fast_gicp | ✗ | 中小target、收敛域宽 |
| `fast_gicp_vgicp` | **fast_gicp** | **small_gicp VGICP** | 粗✗细✓ | 宽收敛+高精度 |

### 12.2 两阶段配准流程

```
输入：source_pcl, target, target_pcl, target_kdtree, coarse_voxelmap, fine_voxelmap, init_T

// 计算对应距离门限
coarse_dist = coarse_max_correspondence_distance > 0 
              ? coarse_max_correspondence_distance 
              : max_correspondence_distance * coarse_scale
fine_dist = fine_max_correspondence_distance > 0 
            ? fine_max_correspondence_distance 
            : max_correspondence_distance * fine_scale

// ---- 粗配准 ----
if (coarse_scale > 1.0) {
    src_coarse = downsample(source_pcl, source_voxel_leaf * coarse_scale);
    coarse_result = runCoarse(src_coarse, target, target_pcl, target_kdtree, 
                              coarse_voxelmap, init, params, coarse_dist, 
                              target_voxel_leaf * coarse_scale);
} else {
    coarse_result = init_T;  // 跳过粗配准
}

// ---- 精细配准 ----
src_fine = downsample(source_pcl, source_voxel_leaf * fine_scale);
fine_result = runFine(src_fine, target, target_pcl, target_kdtree,
                      fine_voxelmap, coarse_result, params, fine_dist,
                      target_voxel_leaf * fine_scale);

返回：coarse_result, fine_result
```

### 12.3 鲁棒核机制

仅 small_gicp 类型（gicp / vgicp / icp）生效：

```cpp
// Huber核：对大于c的误差线性惩罚
if (params.robust_kernel == "huber") {
    using Factor = RobustFactor<Huber, GICPFactor>;
    result = alignWithTree<Factor>(...);
}

// Cauchy核：对大于c的误差饱和惩罚
else if (params.robust_kernel == "cauchy") {
    using Factor = RobustFactor<Cauchy, GICPFactor>;
    result = alignWithTree<Factor>(...);
}

// 无核：标准GICPFactor
else {
    result = alignWithTree<GICPFactor>(...);
}
```

**作用**：抑制动态物体、植被等离群对应点对优化的拉偏。

---

## 13. 线程安全设计

### 13.1 互斥锁使用

```cpp
mutable std::mutex mutex_;  // GlobalMatcherV2Node主锁

// 保护数据：
// - latest_source_, latest_source_stamp_, latest_source_receive_stamp_, latest_source_frame_
// - latest_tracking_map_base_, latest_tracking_stamp_, have_tracking_map_base_
// - latest_bootstrap_map_base_, latest_bootstrap_stamp_, have_bootstrap_map_base_
// - latest_source_base_, latest_source_odom_stamp_, have_source_odom_
// - last_map_source_, have_last_map_source_
// - manual_reset_seq_
```

```cpp
std::mutex mutex_;  // MapManager主锁
std::condition_variable cv_;  // 用于异步加载同步

// 保护数据：
// - load_queue_, queued_ids_, failed_ids_
// - loaded_, active_ids_, last_used_
// - active_sg_, active_pcl_, active_kdtree_, active_voxel_coarse_, active_voxel_fine_
// - revision_, map_changed_
```

### 13.2 线程模型

```
主线程（ROS Spin）
├── 订阅回调（加mutex_锁，更新latest_*数据）
├── 匹配定时器（加mutex_锁，快照数据后释放，执行runMatch）
└── 地图定时器

后台线程（MapManager::loaderThreadLoop）
├── 从加载队列取分片ID
├── 加载分片（I/O + 降采样 + 协方差估计）
└── 加入loaded_（加mutex_锁）
```

### 13.3 关键时序

```
匹配线程                                          加载线程
─────────                                         ────────
ensureActive(position)
  计算needed分片
  将缺失分片加入load_queue_
  释放mutex_
                                                  从queue取ID
                                                  加载分片...
  同步等待当前分片
  (cv_.wait_for, 最多5秒)                         加载完成，加入loaded_
                                                  通知cv_
  检查活动集合
  变化则重建target结构
  返回
```

---

## 14. 性能优化

### 14.1 Target结构预构建

| 优化点 | 说明 |
|--------|------|
| 协方差预计算 | 分片加载时一次性估计，不复算 |
| KDTree预构建 | 活动集合变化时构建，匹配帧内复用 |
| 体素地图预构建 | VGICP粗/细体素地图预构建，匹配帧内不复算 |
| 零target预处理 | 匹配帧内只做source降采样与配准 |

### 14.2 降采样优化

使用哈希实现的质心体素降采样，避免PCL VoxelGrid的int32体素数量限制：

```cpp
// 64位哈希键：(ix & 0x1FFFFF) | (iy << 21) | (iz << 42)
// 支持坐标范围：±2^20 * leaf ≈ ±100万米（leaf=1m时）
```

### 14.3 异步加载

- 邻接分片后台异步加载，不阻塞匹配
- 仅当前分片同步等待（首次进入新分片时）
- LRU控制内存上限

---

## 15. 错误处理与日志

### 15.1 错误等级

| 等级 | 触发条件 | 处理方式 |
|------|---------|---------|
| FAILURE | 地图加载失败 | 发布失败状态，节点继续运行 |
| WARNING | 匹配拒绝、分片加载失败 | 发布警告状态，记录日志 |
| NORMAL | 匹配成功 | 发布正常状态 |

### 15.2 关键日志

```
[ERROR] failed to init map manager
[WARN]  chunked mode: chunks are auto-managed, ignore load_map_command
[WARN]  matcher converged=false but quality gates passed
[INFO]  mode=%s success=%d converged=%d source=%zu target=%zu fitness=%.4f ...
```

---

## 16. 附录

### 16.1 文件清单

```
global_matcher_v2/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # 包描述
├── config/
│   └── global_matcher_v2.yaml  # 默认配置
├── launch/
│   ├── global_matcher_v2.launch          # 独立运行
│   └── localization_v2_with_gmv2.launch  # 替换v1链路
├── include/global_matcher_v2/
│   ├── core/
│   │   ├── map_manager.hpp     # 地图管理器接口
│   │   ├── matcher.hpp         # 匹配器接口
│   │   └── matcher_impl.hpp    # 匹配器模板实现
│   └── ros/
│       └── global_matcher_v2_config.hpp  # 配置结构体
├── src/
│   ├── core/
│   │   ├── map_manager.cpp     # 地图管理器实现
│   │   └── matcher.cpp         # 匹配器实现
│   ├── ros/
│   │   ├── global_matcher_v2_node.cpp    # 节点主程序
│   │   └── global_matcher_v2_config.cpp  # 配置加载
│   └── tools/
│       └── bench_voxel_build.cpp         # 体素构建基准测试
└── docs/
    ├── matching_flow.md        # 匹配流程说明（已有）
    └── detailed_design.md      # 本详细设计文档
```

### 16.2 依赖项

| 依赖 | 用途 |
|------|------|
| roscpp | ROS C++客户端库 |
| pcl_ros / pcl_conversions | PCL点云与ROS消息转换 |
| small_gicp | small_gicp配准库（icp/gicp/vgicp） |
| fast_gicp | fast_gicp配准库 |
| deeplumin_msgs | DeepLumin自定义消息（GlobalMatchResult等） |
| Eigen3 | 矩阵运算 |
| OpenMP | 并行加速 |

### 16.3 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v2.0 | - | 初始版本，重构自global_matcher |