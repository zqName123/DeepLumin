# ArUco 离线建图 V2 设计文档（简化版）

## 1. 问题分析

### 1.1 当前方案的问题

| 问题 | 描述 | 影响 |
|------|------|------|
| **SLAM 轨迹频率低** | `slam_mapping_v2` 只发布关键帧轨迹 | 无法实时关联每一帧的位姿 |
| **优化后位姿变化** | 后端优化会修正历史帧的 `T_map_base` | 已建好的 marker 位置失效 |
| **时间同步困难** | 图像帧和关键帧时间戳不完全对应 | 关联不准确 |

### 1.2 简化方案思路

**核心思想：在 slam_mapping_v2 保存关键帧时，同时保存对应的图像**

```
┌─────────────────────────────────────────────────────────┐
│ slam_mapping_v2 保存关键帧                              │
│                                                         │
│ 输出目录：                                              │
│   keyframe_poses.csv    (id, timestamp, 位姿)          │
│   keyframes/            (关键帧点云)                    │
│   images/               (关键帧图像) ← 新增           │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ aruco_relocalization 离线建图                           │
│                                                         │
│ 输入：                                                  │
│   - keyframe_poses.csv                                  │
│   - images/ 文件夹                                      │
│                                                         │
│ 处理：                                                  │
│   - 遍历每个关键帧                                      │
│   - 读取图像和位姿                                      │
│   - 检测 marker                                         │
│   - 计算 marker 在 map 下的位姿                        │
│                                                         │
│ 输出：marker_map.yaml                                   │
└─────────────────────────────────────────────────────────┘
```

---

## 2. 简化方案优势

| 对比项 | 原方案 | 简化方案 |
|--------|--------|----------|
| 时间戳关联 | 需要 | **不需要** |
| 数据格式 | 复杂（JSON + 图像） | **简单（CSV + 图像文件夹）** |
| 实现复杂度 | 高 | **低** |
| 数据量 | 大（所有帧） | **小（仅关键帧）** |
| 可重复性 | 需要重新采集 | **直接重新运行** |

---

## 3. 第一步：修改 slam_mapping_v2

### 3.1 修改内容

在 `MapSaver::saveKeyframes()` 中添加图像保存逻辑：

```cpp
bool MapSaver::saveKeyframes(const std::string& dir,
                             const std::vector<Keyframe>& keyframes,
                             const std::unordered_map<double, cv::Mat>& image_buffer) const {
  std::filesystem::create_directories(dir);
  std::filesystem::create_directories(dir + "/images");  // 新增

  bool ok = true;
  for (const auto& kf : keyframes) {
    // 保存点云（已有逻辑）
    if (kf.cloud && !kf.cloud->empty()) {
      // ... 保存点云 ...
    }

    // 新增：保存图像
    if (kf.has_visual && kf.image_timestamp > 0) {
      auto it = image_buffer.find(kf.image_timestamp);
      if (it != image_buffer.end()) {
        std::string image_path = dir + "/images/" + std::to_string(kf.id) + ".png";
        cv::imwrite(image_path, it->second);
      }
    }
  }
  return ok;
}
```

### 3.2 修改 Keyframe 结构体

```cpp
struct Keyframe {
  int id = -1;
  double timestamp = 0.0;
  Pose pose_odom;
  Pose pose_map;
  PointCloudPtr cloud;
  std::size_t point_count = 0;
  std::uint64_t source_frame = 0;
  bool optimized = false;
  bool loop_closed = false;
  int loop_match_count = 0;
  GnssObservation gnss;

  // ===== 视觉字段 =====
  bool has_visual = false;
  double image_timestamp = 0.0;
  int visual_feature_count = 0;
  cv::Mat image;  // 新增：存储图像数据（可选）
};
```

### 3.3 配置参数

```yaml
# slam_mapping_v2.yaml
save:
  save_keyframes: true
  save_keyframe_poses: true
  save_trajectory: true
  save_keyframe_images: true  # 新增：是否保存关键帧图像
  image_format: "png"         # 新增：图像格式
```

### 3.4 输出目录结构

```
output/
├── map.pcd                    # 全局点云地图
├── keyframe_poses.csv         # 关键帧位姿
├── trajectory.txt             # 轨迹（TUM格式）
├── keyframes/                 # 关键帧点云
│   ├── 0.pcd
│   ├── 1.pcd
│   └── ...
└── images/                    # 关键帧图像（新增）
    ├── 0.png
    ├── 1.png
    └── ...
```

### 3.5 keyframe_poses.csv 格式

```
id,timestamp,x,y,z,qx,qy,qz,qw
0,1234567890.123,1.0,2.0,3.0,0.0,0.0,0.0,1.0
1,1234567891.456,1.1,2.1,3.1,0.0,0.0,0.0,1.0
```

---

## 4. 第二步：aruco_relocalization 离线建图节点

### 4.1 功能描述

- 读取 `keyframe_poses.csv`
- 遍历每个关键帧
- 读取对应的图像（从 `images/` 文件夹）
- 检测 ArUco marker
- 计算 marker 在 map 坐标系下的位姿
- 聚合观测，输出 marker_map.yaml

### 4.2 数据结构

**关键帧数据：**
```cpp
struct KeyframeData {
  int id;
  double timestamp;
  Eigen::Isometry3d T_map_base;  // 优化后的位姿
  cv::Mat image;                 // 关键帧图像
};
```

**Marker 观测：**
```cpp
struct MarkerObservation {
  int keyframe_id;
  Eigen::Isometry3d T_map_marker;  // marker 在 map 下的位姿
  double reprojection_error;
};
```

### 4.3 配置参数

```yaml
# aruco_relocalization.yaml
offline_mapping_v2:
  # 输入路径
  keyframe_poses_csv: "/path/to/keyframe_poses.csv"
  keyframe_images_dir: "/path/to/images/"

  # 输出路径
  output_marker_map: "$(find aruco_relocalization)/config/marker_map.yaml"

  # 外参（base_link -> camera）
  extrinsic:
    lidar_to_camera:
      translation: [0.0, 0.3, 0.0]
      rotation: [0.5, -0.5, -0.5, 0.0]

  # 图像处理
  image_rotation: 3  # 逆时针 90°

  # 校验参数
  max_reproj_error: 5.0  # pixels
  min_observations: 3    # 每个 marker 至少被观测到的次数
```

### 4.4 工作流程

```
初始化：
1. 加载配置
2. 读取 keyframe_poses.csv
3. 初始化 ArUco 检测器

处理（遍历每个关键帧）：
1. 读取关键帧图像（images/{id}.png）
2. 图像旋转处理（如果需要）
3. ArUco marker 检测
4. 对每个检测到的 marker：
   a. 估计 T_cam_marker
   b. 计算重投影误差
   c. 校验是否通过
   d. 如果通过：
      - 计算 T_map_marker = T_map_base * T_base_camera * T_camera_marker
      - 保存观测
5. 输出统计信息

后处理：
1. 对每个 marker：
   a. 收集所有观测
   b. 加权平均得到最终位置
   c. 计算平均重投影误差
2. 输出 marker_map.yaml
```

### 4.5 关键代码结构

```cpp
class OfflineMappingV2 {
 public:
  bool initialize(const OfflineMappingV2Config& config);
  bool process();

 private:
  // 数据加载
  bool loadKeyframePoses(const std::string& csv_path);
  cv::Mat loadKeyframeImage(int keyframe_id);

  // 处理
  bool processKeyframe(const KeyframeData& kf);
  bool detectMarkers(const cv::Mat& image, std::vector<MarkerDetection>& detections);

  // 聚合
  bool aggregateMarkerObservations();
  bool saveMarkerMap(const std::string& path);

  OfflineMappingV2Config config_;
  MarkerDetector detector_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  Eigen::Isometry3d T_base_camera_;

  // 关键帧数据
  std::vector<KeyframeData> keyframes_;

  // Marker 观测
  struct MarkerAggregate {
    int id;
    std::vector<MarkerObservation> observations;
    Eigen::Isometry3d T_map_marker;
    double mean_reproj_error;
    int observation_count;
  };
  std::unordered_map<int, MarkerAggregate> marker_aggregates_;
};
```

---

## 5. 文件结构

### 5.1 slam_mapping_v2 修改

```
slam_mapping_v2/
├── include/slam_mapping_v2/
│   └── common/types.hpp  # Keyframe 添加 image 字段
├── src/
│   ├── core/
│   │   └── map_saver.cpp  # 添加 saveKeyframeImages
│   └── slam_mapping_v2_node.cc  # 保存图像时调用
└── config/
    └── slam_mapping_v2.yaml  # 添加 save_keyframe_images 配置
```

### 5.2 aruco_relocalization 新增

```
aruco_relocalization/
├── include/aruco_relocalization/
│   └── offline_mapping_v2/
│       └── offline_mapping_v2.hpp
├── src/
│   └── offline_mapping_v2/
│       └── offline_mapping_v2.cpp
├── launch/
│   └── offline_mapping_v2.launch
└── config/
    └── aruco_relocalization.yaml  # 添加 offline_mapping_v2 配置
```

---

## 6. 使用流程

### 6.1 第一步：运行 SLAM 建图

```bash
# 启动 SLAM（会自动保存关键帧图像）
roslaunch slam_mapping_v2 slam_mapping_v2.launch save_keyframe_images:=true

# 播放 bag
rosbag play your_bag.bag

# 等待完成，输出：
# /path/to/output/
#   ├── keyframe_poses.csv
#   ├── keyframes/
#   └── images/
```

### 6.2 第二步：离线 marker map 构建

```bash
# 运行离线建图
roslaunch aruco_relocalization offline_mapping_v2.launch \
  keyframe_poses_csv:=/path/to/keyframe_poses.csv \
  keyframe_images_dir:=/path/to/images/

# 输出：config/marker_map.yaml
```

### 6.3 第三步：使用 marker map 重定位

```bash
# 启动在线重定位
roslaunch aruco_relocalization aruco_relocalization.launch
```

---

## 7. Marker 位姿计算

### 7.1 坐标变换链

```
T_map_marker = T_map_base * T_base_camera * T_camera_marker

其中：
- T_map_base：从 keyframe_poses.csv 读取（优化后）
- T_base_camera：从配置文件读取（外参）
- T_camera_marker：从 ArUco 检测获取
```

### 7.2 聚合策略

**加权平均：**
```cpp
// 位置加权平均（权重 = 1 / reproj_error）
Eigen::Vector3d pos_sum = Eigen::Vector3d::Zero();
double weight_sum = 0.0;
for (const auto& obs : observations) {
    double weight = 1.0 / std::max(0.1, obs.reprojection_error);
    pos_sum += obs.T_map_marker.translation() * weight;
    weight_sum += weight;
}
Eigen::Vector3d pos_avg = pos_sum / weight_sum;

// 旋转加权平均（四元数，处理双覆盖）
Eigen::Quaterniond q_sum(0, 0, 0, 0);
Eigen::Quaterniond q_first = observations[0].quaternion();
for (const auto& obs : observations) {
    Eigen::Quaterniond q = obs.quaternion();
    if (q.dot(q_first) < 0) {
        q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
    }
    double weight = 1.0 / std::max(0.1, obs.reprojection_error);
    q_sum.w() += q.w() * weight;
    q_sum.x() += q.x() * weight;
    q_sum.y() += q.y() * weight;
    q_sum.z() += q.z() * weight;
}
q_sum.normalize();
```

---

## 8. 输出格式

### 8.1 marker_map.yaml

```yaml
# marker_map.yaml
frame_id: "map"
source: "offline_mapping_v2"
generation_time: "2026-09-02 12:00:00"

config:
  camera_matrix: [960.0, 0.0, 510.0, 0.0, 960.0, 960.0, 0.0, 0.0, 1.0]
  marker_size: 0.2
  dictionary: "DICT_5X5_1000"

markers:
  - id: 1
    x: 1.234
    y: 2.345
    z: 0.567
    qw: 0.707
    qx: 0.0
    qy: 0.0
    qz: 0.707
    size: 0.2
    observation_count: 10
    mean_reproj_error: 2.34
    keyframe_ids: [0, 5, 12, 23, 35, 42, 56, 67, 78, 89]

  - id: 2
    x: 3.456
    y: 4.567
    z: 0.789
    qw: 0.707
    qx: 0.0
    qy: 0.0
    qz: 0.707
    size: 0.2
    observation_count: 8
    mean_reproj_error: 3.12
    keyframe_ids: [2, 8, 15, 28, 38, 45, 58, 70]
```

### 8.2 处理报告

```yaml
# processing_report.yaml
processing_time: "2026-09-02 12:05:00"
statistics:
  total_keyframes: 100
  keyframes_with_markers: 45
  total_markers_detected: 5
  markers_in_map: 4
  markers_filtered: 1  # 观测次数不足

reprojection_errors:
  mean: 2.45
  max: 4.89
  min: 0.56

marker_details:
  - id: 1
    observation_count: 10
    mean_reproj_error: 2.34
    position_std: 0.05  # 位置标准差

  - id: 2
    observation_count: 8
    mean_reproj_error: 3.12
    position_std: 0.08
```

---

## 9. 可视化

### 9.1 保留现有可视化模块

当前的 `MarkerVisualizer` 可以保留，用于：
- 离线建图时显示检测结果
- 验证 marker 检测是否正确

### 9.2 可能的扩展

如果需要更详细的可视化，可以添加：
- 显示关键帧图像和检测结果
- 显示 marker 在地图中的位置
- 对比优化前后的 marker 位置

---

## 10. 总结

### 10.1 改造要点

1. **slam_mapping_v2 修改**：
   - 在保存关键帧时同时保存图像
   - 添加 `save_keyframe_images` 配置

2. **aruco_relocalization 新增**：
   - 离线建图节点 `offline_mapping_v2_node`
   - 读取关键帧图像和位姿
   - 遍历检测 marker
   - 输出 marker_map.yaml

3. **保留可视化模块**：
   - 用于离线建图时显示检测结果

### 10.2 关键优势

- **简单**：无需时间戳关联
- **高效**：只处理关键帧，数据量小
- **准确**：使用优化后的位姿
- **灵活**：可多次运行，调整参数

### 10.3 注意事项

1. **图像保存**：确保 slam_mapping_v2 正确保存图像
2. **外参配置**：确保 T_base_camera 配置正确
3. **图像格式**：建议使用 PNG 格式（无损压缩）
4. **磁盘空间**：关键帧图像会占用一定空间，需要预估
