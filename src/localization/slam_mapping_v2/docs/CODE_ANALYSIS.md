# slam_mapping_v2 详细分析文档

## 1. 项目概述

`slam_mapping_v2` 是一个基于**消息订阅**的 SLAM 建图节点，订阅前端里程计（如 FAST-LIO）发布的 odom 与配准扫描，完成全局地图构建、关键帧管理、GNSS 后端优化、回环检测与 GTSAM 位姿图优化。

### 核心特性
- **多线程架构**：输入回调、主线程、后端线程、可视化线程分离，后端优化不阻塞建图
- **模块化设计**：关键帧/回环/GNSS/保存/可视化均为 v2 自有模块
- **单命名空间**：`slam_mapping_v2`，不依赖旧版 slam_mapping / slam_odometry
- **支持多种回环检测**：激光回环（KDTree+GICP）、视觉回环（ORB+DBoW2）、混合模式

---

## 2. 代码结构

```
slam_mapping_v2/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # ROS 包配置
├── README.md                   # 快速入门
├── config/
│   └── slam_mapping_v2.yaml    # 运行时参数配置
├── docs/                       # 设计文档
│   ├── DESIGN.md               # 多线程设计
│   ├── VISUALIZATION.md        # 可视化排查
│   ├── ORB视觉回环详细设计中文.md
│   └── ...
├── include/slam_mapping_v2/
│   ├── common/                 # 公共数据类型与配置
│   │   ├── config.hpp          # 所有配置结构体
│   │   ├── types.hpp           # 核心数据类型（Pose, Keyframe, LoopCandidate 等）
│   │   └── types_visual.hpp    # 视觉回环相关类型
│   ├── core/                   # 核心算法模块
│   │   ├── mapping_runtime.hpp # 运行时主类（线程管理）
│   │   ├── keyframe_manager.hpp# 关键帧判定
│   │   ├── global_map.hpp      # iVox 全局地图
│   │   ├── pose_graph_optimizer.hpp # GTSAM 位姿图优化
│   │   ├── gnss_optimizer.hpp  # GNSS 约束优化
│   │   ├── map_saver.hpp       # 地图保存
│   │   ├── i_loop_detector.hpp # 回环检测接口
│   │   ├── loop_detector_kdtree_gicp.hpp # 激光回环检测
│   │   ├── visual_loop_detector.hpp      # 视觉回环检测
│   │   ├── loop_detector_factory.hpp     # 回环检测工厂
│   │   ├── orb_feature_provider.hpp      # ORB 特征提取
│   │   ├── visual_feature_buffer.hpp     # 视觉特征缓冲区
│   │   └── i_visual_feature_provider.hpp # 特征提取接口
│   └── ros/                    # ROS 适配层
│       ├── config_loader.hpp   # YAML 配置加载
│       ├── converters.hpp      # ROS 消息转换
│       └── publishers.hpp      # 话题发布
├── src/
│   ├── slam_mapping_v2_node.cc # 节点入口（主循环）
│   ├── core/                   # 核心实现
│   │   ├── mapping_runtime.cpp
│   │   ├── keyframe_manager.cpp
│   │   ├── global_map.cpp
│   │   ├── pose_graph_optimizer.cpp
│   │   ├── gnss_optimizer.cpp
│   │   ├── map_saver.cpp
│   │   ├── loop_detector_kdtree_gicp.cpp
│   │   ├── visual_loop_detector.cpp
│   │   ├── loop_detector_factory.cpp
│   │   ├── orb_feature_provider.cpp
│   │   ├── visual_feature_buffer.cpp
│   │   └── types_impl.cpp
│   └── ros/                    # ROS 层实现
│       ├── config_loader.cpp
│       ├── converters.cpp
│       └── publishers.cpp
├── launch/
│   └── slam_mapping_v2.launch  # 启动文件
├── rviz/
│   └── mapping.rviz            # RViz 配置
└── thirdparty/                 # 第三方库
    ├── DBoW2/                  # 词袋模型
    ├── ORBextractor/           # ORB 特征提取（ORB_SLAM3）
    └── ivox3d/                 # iVox 空间哈希地图
```

---

## 3. 输入输出

### 3.1 输入话题

| 话题名 | 消息类型 | 说明 | 配置项 |
|--------|---------|------|--------|
| `/localization/slam_odom` | `nav_msgs/Odometry` | 里程计位姿 | `input/odom_topic` |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 配准点云（odom 系） | `input/scan_topic` |
| `/gnss_chc_data` | `deeplumin_msgs/Gpchc` | GNSS 观测（可选） | `input/gnss_topic` |
| `/camera/image_raw` | `sensor_msgs/Image` | 相机图像（视觉回环用） | `input/image_topic` |

### 3.2 输出话题

| 话题名 | 消息类型 | 说明 | 配置项 |
|--------|---------|------|--------|
| `/slam_mapping_v2/status` | `deeplumin_msgs/MappingStatus` | 运行状态 | `output/status_topic` |
| `/slam_mapping_v2/path` | `nav_msgs/Path` | 优化后轨迹 | `output/path_topic` |
| `/slam_mapping_v2/map` | `sensor_msgs/PointCloud2` | 全局地图预览 | `output/map_topic` |
| `/slam_mapping_v2/odom` | `nav_msgs/Odometry` | 转发 odom（可选） | `output/odom_topic` |
| `/slam_mapping_v2/current_scan` | `sensor_msgs/PointCloud2` | 转发 current_scan（可选） | `output/current_scan_topic` |
| `/slam_mapping_v2/keyframes` | `visualization_msgs/MarkerArray` | 关键帧标记 | `output/keyframe_markers_topic` |
| `/slam_mapping_v2/loop_markers` | `visualization_msgs/MarkerArray` | 回环标记 | `output/loop_markers_topic` |
| `TF: map → odom` | `geometry_msgs/TransformStamped` | 地图到里程计变换 | - |

### 3.3 服务接口

| 服务名 | 请求类型 | 说明 |
|--------|---------|------|
| `/slam_mapping_v2/start` | `deeplumin_msgs/StartMappingRuntime` | 启动建图 |
| `/slam_mapping_v2/stop` | `deeplumin_msgs/StopMappingRuntime` | 停止并保存 |
| `/slam_mapping_v2/save` | `deeplumin_msgs/SaveMappingRuntime` | 手动保存 |
| `/slam_mapping_v2/reset` | `std_srvs/Trigger` | 重置所有数据 |
| `/slam_mapping_v2/get_status` | `deeplumin_msgs/GetMappingRuntimeStatus` | 获取状态 |

---

## 4. 关键功能介绍

### 4.1 多线程架构

```
┌─────────────────────────────────────────────────────────────┐
│                     slam_mapping_v2_node                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ AsyncSpinner│  │ AsyncSpinner│  │    主线程 (100Hz)    │  │
│  │  (odom回调)  │  │  (scan回调)  │  │  • 时间同步配对      │  │
│  │  • feedOdom  │  │  • feedScan  │  │  • 关键帧判定        │  │
│  │  • feedGnss  │  │  • feedImage │  │  • 全局地图插入      │  │
│  └──────┬──────┘  └──────┬──────┘  │  • 发布 TF/状态      │  │
│         │                │         └─────────────────────┘  │
│         └────────────────┘                  │                │
│                              input_mutex_   │                │
│                                             ▼                │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              后端线程 (backendLoop)                   │   │
│  │  • 合并 pending 关键帧 → keyframes_                   │   │
│  │  • 添加里程计因子                                     │   │
│  │  • GNSS 因子更新                                      │   │
│  │  • 回环检测 (激光/视觉/混合)                          │   │
│  │  • ISAM2 优化 → 更新所有 pose_map                     │   │
│  │  • 计算 t_map_odom                                    │   │
│  └──────────────────────────────────────────────────────┘   │
│                         │                                    │
│                         ▼                                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            可视化线程 (visualizationLoop)             │   │
│  │  • 发布 path / keyframes / loop_markers               │   │
│  │  • 发布 map (通过 t_map_odom 变换 global_map_)        │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

**同步机制**：

| 数据 | 写者 | 读者 | 同步原语 |
|------|------|------|---------|
| 输入队列 | 回调线程 | 主线程 | `input_mutex_` |
| pending 关键帧 | 主线程 | 后端线程 | `pending_mutex_` + `pending_cv_` |
| keyframes_ | 后端线程 | 可视化/保存 | `std::shared_mutex kf_mutex_` |
| LoopData | 后端线程 | 可视化/保存 | `loop_mutex_` |
| 状态 | 主/后端线程 | 任意 | `status_mutex_` |

### 4.2 关键帧管理

**判定条件**（`KeyframeManager::shouldAdd`）：
1. 点云有效且点数 ≥ `min_points`
2. 首帧强制添加
3. 与上一关键帧间隔 ≥ `min_frame_gap` 帧
4. 满足以下任一几何条件：
   - 距离 > `distance_thresh`
   - 旋转 > `rotation_thresh_rad`
   - 时间 > `time_thresh_sec`

**关键帧结构**（`Keyframe`）：
```cpp
struct Keyframe {
    int id = -1;                    // 关键帧 ID（按 keyframes_ 下标）
    double timestamp;               // 时间戳
    Pose pose_odom;                 // odom 系位姿（原始）
    Pose pose_map;                  // map 系位姿（优化后）
    PointCloudPtr cloud;            // 点云（odom 系）
    std::uint64_t source_frame;     // 来源帧序号
    GnssObservation gnss;           // 关联的 GNSS 观测
    bool loop_closed = false;       // 是否参与回环
    // 视觉回环字段...
};
```

### 4.3 全局地图

**iVox 增量地图**（`GlobalMap`）：
- 使用 faster-lio 的 iVox3D 空间哈希结构
- 每帧直接插入 odom 系点云（不做坐标变换）
- 体素去重：插入前检查附近是否已有足够近的点
- 可视化时通过统一的 `t_map_odom` 变换到 map 系

**注意**：运行时地图不反映后端优化的非刚性修正，保存时通过关键帧独立重投影生成准确地图。

### 4.4 位姿图优化

**GTSAM ISAM2 增量优化**（`PoseGraphOptimizer`）：
- 里程计因子：连续关键帧之间的相对位姿约束
- 回环因子：检测到回环时的相对位姿约束
- GNSS 因子：GNSS 观测的绝对位置约束
- 每次后端循环增量添加新因子并优化

### 4.5 回环检测

支持三种模式（通过 `loop_closure/detector_type` 配置）：

#### 4.5.1 激光回环（`laser`）
- **检测器**：`LoopDetectorKdtreeGICP`
- **流程**：
  1. 构建当前关键帧的子地图（KDTree 搜索邻近点）
  2. 在历史关键帧中搜索候选（距离/时间筛选）
  3. GICP 配准计算变换
  4.  fitness 阈值筛选

#### 4.5.2 视觉回环（`visual`）
- **检测器**：`VisualLoopDetector`
- **流程**：
  1. ORB 特征提取（独立线程）
  2. DBoW2 词袋向量计算
  3. 倒排索引检索候选帧
  4. BoW 加速特征匹配
  5. 几何验证（Fundamental + RANSAC）
  6. 连续性确认（多帧一致）

#### 4.5.3 混合模式（`hybrid`）
- 同时使用激光和视觉检测器
- 任一检测到回环即触发优化

### 4.6 GNSS 优化

**GnssOptimizer**：
- 自动对齐 GNSS 坐标系与 map 系
- 根据 GNSS 状态（固定/浮点/未知）设置不同噪声
- 鲁棒核函数（Huber）抑制异常值
- 一致性检查防止跳变

### 4.7 地图保存

**保存内容**：

| 文件 | 内容 | 坐标系 |
|------|------|--------|
| `map.pcd` | 全局地图点云 | **map 系**（优化后） |
| `keyframes/*.pcd` | 各关键帧点云 | odom 系（原始） |
| `keyframes.csv` | 关键帧位姿 | map 系（优化后） |
| `trajectory.csv` | 轨迹 | map 系（优化后） |
| `loop_results/loop_pairs.csv` | 回环对 | - |
| `gnss_alignment.txt` | GNSS 对齐参数 | - |

**保存时地图构建**（`MapSaver::buildMap`）：
```cpp
for (const auto& kf : keyframes) {
    // 每个关键帧独立变换：T_map_cloud = T_pose_map * T_pose_odom^-1
    transformCloud(*kf.cloud, kf.pose_map, kf.pose_odom, out.get());
}
```

---

## 5. 坐标系说明

| 坐标系 | 说明 | 数据来源 |
|--------|------|---------|
| **odom** | 里程计坐标系（FAST-LIO 局部系） | 前端里程计 |
| **map** | 全局地图坐标系（优化后） | 后端 ISAM2 |
| **base_link** | 机器人本体坐标系 | - |

**关键变换**：
- `T_map_odom = T_pose_map * T_pose_odom^-1`：odom 系 → map 系
- 发布 TF：`map → odom`（供其他节点使用）

---

## 6. 配置参数详解

### 6.1 关键帧配置 (`keyframe`)

```yaml
keyframe:
  distance_thresh: 2.0        # 距离阈值（米），超过则添加关键帧
  rotation_thresh_rad: 0.0    # 旋转阈值（弧度），0 表示不启用
  time_thresh_sec: 0.0        # 时间阈值（秒），0 表示不启用
  min_points: 50              # 最小点云数
  min_frame_gap: 10           # 与上一关键帧最小帧间隔
```

### 6.2 回环检测配置 (`loop_closure`)

```yaml
loop_closure:
  enable: true
  detector_type: "laser"      # "laser" | "visual" | "hybrid"
  min_index_gap: 20           # 回环检测最小关键帧间隔
  time_thresh_sec: 20.0       # 回环检测最小时间间隔
  loop_kf_gap: 30             # 连续检测间隔
  min_id_interval: 20         # 候选帧最小 ID 间隔
  closest_id_th: 50           # 最近帧阈值
  max_range: 20.0             # 子地图搜索范围
  submap_search_num: 25       # 子地图搜索点数
  submap_resolution: 0.3      # 子地图分辨率
  icp_fitness_thresh: 0.3     # GICP fitness 阈值
  loop_noise_scale: 1.0       # 回环噪声尺度
  loop_rot_noise: 0.05        # 回环旋转噪声
  loop_pos_noise: 0.5         # 回环位置噪声
  odom_rot_noise: 1e-6        # 里程计旋转噪声
  odom_pos_noise: 1e-4        # 里程计位置噪声
```

### 6.3 视觉回环配置 (`loop_closure/visual`)

```yaml
loop_closure:
  visual:
    vocabulary_path: ""         # ORB 词库路径
    nfeatures: 2000             # ORB 特征点数
    scale_factor: 1.2           # 金字塔尺度因子
    nlevels: 8                  # 金字塔层数
    ini_th_fast: 20             # FAST 角点初始阈值
    min_th_fast: 7              # FAST 角点最小阈值
    min_bow_score: 0.03         # 最小 BoW 分数
    min_shared_words: 20        # 最小共享单词数
    max_candidates: 3           # 最大候选帧数
    min_bow_matches: 50         # 最小匹配点数
    nn_ratio: 0.6               # 最近邻比例
    ransac_min_inliers: 15      # RANSAC 最小内点数
    ransac_probability: 0.99    # RANSAC 置信度
    ransac_max_iterations: 300  # RANSAC 最大迭代
    min_continuity: 3           # 连续性验证次数
    max_continuity_miss: 2      # 最大允许中断
    feature_extract_rate_hz: 10 # 特征提取频率
    buffer_size: 100            # 特征缓冲区大小
    camera:                     # 相机内参
      fx: 500.0
      fy: 500.0
      cx: 320.0
      cy: 240.0
    T_L_C: [...]               # LiDAR 到相机外参
```

### 6.4 GNSS 配置 (`loop_closure/gnss`)

```yaml
loop_closure:
  gnss:
    enable: true
    min_status: 33              # 最小状态值（RTK 固定=33）
    min_nsv: 0                  # 最小卫星数
    max_age_sec: 0.02           # 最大时间差
    min_spacing_m: 6.0          # 最小空间间隔
    use_gps_elevation: false    # 是否使用 GPS 高程
    z_override_sigma: 0.01      # 高程约束噪声
    sigma_fixed: 0.3            # 固定解噪声
    sigma_float: 1.5            # 浮点解噪声
    sigma_unknown: 3.0          # 未知解噪声
    sigma_floor: 0.2            # 噪声下限
    robust_delta: 1.0           # Huber 核参数
    alignment_frames: 20        # 对齐帧数
    consistency_thresh_m: 10.0  # 一致性阈值
    lever_arm: [0, 0, 0]        # 杆臂偏移
```

---

## 7. 常见问题与排查

### 7.1 视觉回环特征不匹配

**现象**：`Features not ready for kf X`

**原因**：
- 图像时间戳与 LiDAR 时间戳不同步
- 视觉特征线程时间戳匹配逻辑问题

**解决**：
- 检查传感器时间同步
- 修改 `visualFeatLoop` 中的时间戳容差
- 改用 `detector_type: "hybrid"` 作为备选

### 7.2 回环误匹配导致地图漂移

**现象**：轨迹漂移但可视化地图看起来正常

**原因**：
- 可视化地图使用统一 `t_map_odom` 变换，整体偏移
- 保存地图使用各关键帧独立变换，结构性畸变更明显

**解决**：
- 调整回环检测阈值（`icp_fitness_thresh`、`min_bow_matches`）
- 增大 `min_index_gap` 避免检测太近的回环
- 使用 GNSS 约束抑制漂移

### 7.3 关键帧添加过于频繁/稀疏

**调整**：
- `distance_thresh`：控制距离触发
- `rotation_thresh_rad`：控制旋转触发
- `min_frame_gap`：控制最小帧间隔

---

## 8. 编译与运行

```bash
# 编译
cd /home/hhy/2004ros/DeepLumin
catkin_make --pkg slam_mapping_v2

# 运行（需先启动 slam_odometry_v2）
roslaunch slam_mapping_v2 slam_mapping_v2.launch

# 服务控制
rosservice call /slam_mapping_v2/start "{task_id: 't1', task_name: 'demo', output_dir: '/tmp/mapping/demo', enable_loop_closure: true, reset_before_start: true}"
rosservice call /slam_mapping_v2/stop "{task_id: 't1', save_map: true, save_keyframes: true, save_keyframe_poses: true, save_trajectory: true}"
```

---

## 9. 与 mapping_manager 集成

在 `mapping_manager/config/mapping_manager.yaml` 中配置：

```yaml
backend:
  enabled: true
  start_service: /slam_mapping_v2/start
  stop_service: /slam_mapping_v2/stop
  get_status_service: /slam_mapping_v2/get_status
  status_topic: /slam_mapping_v2/status
```