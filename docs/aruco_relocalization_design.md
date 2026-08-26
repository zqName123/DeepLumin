# ArUco 路标重定位节点设计文档

## 1. 项目背景与目标

### 1.1 背景

当前系统已有基于 LiDAR 的重定位方案（Scan Context + GICP），但在地下矿井环境中存在以下局限：
- LiDAR 特征退化（长走廊、空旷区域）
- 计算开销大，实时性受限
- 依赖点云地图，建图成本高

ArUco 路标重定位作为视觉方案的补充，具有以下优势：
- 部署成本低，路标印刷即可
- 计算轻量，适合实时性要求高的场景
- 可与 LiDAR 重定位形成冗余互补

### 1.2 目标

实现一个独立的 ArUco 路标重定位节点，具备：
1. 离线建图：从已有数据生成 marker 地图
2. 在线重定位：实时检测 ArUco 并求解相机位姿
3. 多重校验：确保重定位结果的可靠性
4. 无缝集成：与现有 localization_v2 / pose_fusion 体系对接

---

## 2. 可行性分析

### 2.1 技术可行性

| 项目 | 状态 | 说明 |
|------|------|------|
| OpenCV ArUco | ✅ 可用 | OpenCV 4.2.0 自带 aruco 模块，支持 detectMarkers / estimatePoseSingleMarkers |
| PnP 求解 | ✅ 成熟 | cv::solvePnP / solvePnPRansac 已验证 |
| 相机标定 | ⚠️ 需确认 | 需要 camera_info 或离线标定文件 |
| 消息接口 | ✅ 可用 | RelocalizationResult.msg 已定义，可直接复用 |
| 集成接口 | ✅ 可用 | IRelocalizationClient 接口已定义 |

### 2.2 关键风险与应对

| 风险 | 影响 | 应对策略 |
|------|------|----------|
| 单 marker PnP 歧义 | 位姿不稳定 | 多 marker 联合求解 + 几何一致性校验 |
| 误检测 | 错误重定位 | ID 白名单 + 置信度阈值 |
| 大角度倾斜 | PnP 坏解 | 重投影误差校验 + 运动一致性校验 |
| 光照变化 | 检测率下降 | 自适应阈值 + 多字典支持 |
| 路标遮挡 | 有效约束不足 | 最小 marker 数量阈值（≥2） |

### 2.3 性能预估

- ArUco 检测：~5ms / 帧（640x480）
- PnP 求解（单 marker）：~1ms
- PnP 求解（多 marker 联合）：~3-5ms
- 总延迟：< 15ms（满足 30Hz 实时性要求）

---

## 3. 系统架构设计

### 3.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        aruco_relocalization 包                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐               │
│  │   模块 1     │    │   模块 2     │    │   模块 3     │               │
│  │ Marker 检测  │    │ 离线建图     │    │ 地图加载     │               │
│  │  (检测层)    │    │ (离线工具)   │    │ (在线启动)   │               │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘               │
│         │                   │                   │                       │
│         ▼                   ▼                   ▼                       │
│  ┌─────────────────────────────────────────────────────────┐           │
│  │                    模块 4：重定位核心                    │           │
│  │         (ID 匹配 → 单/多 marker 求解 → 输出候选位姿)     │           │
│  └─────────────────────────────┬───────────────────────────┘           │
│                                │                                       │
│                                ▼                                       │
│  ┌─────────────────────────────────────────────────────────┐           │
│  │                    模块 5：多重校验层                    │           │
│  │    (重投影误差 → 几何一致性 → 运动一致性 → 数量阈值)     │           │
│  └─────────────────────────────┬───────────────────────────┘           │
│                                │                                       │
│                                ▼                                       │
│                    /reloc/candidate_pose                                │
│                    (输出到 pose_fusion)                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 与现有系统集成

```
                    ┌─────────────────┐
                    │  camera_driver  │
                    └────────┬────────┘
                             │ /camera/image_raw
                             │ /camera/camera_info
                             ▼
                    ┌─────────────────┐
                    │    aruco_reloc  │◄──── marker_map.yaml
                    └────────┬────────┘
                             │ /aruco_reloc/result (RelocalizationResult)
                             ▼
                    ┌─────────────────┐
                    │localization_mgr │ (门控 + 状态机)
                    └────────┬────────┘
                             │ accepted_relocalization
                             ▼
                    ┌─────────────────┐
                    │  pose_fusion    │ (融合输出)
                    └─────────────────┘
```

---

## 4. 模块详细设计

### 4.1 模块 1：Marker 检测器封装

#### 4.1.1 职责

- 接收图像 + camera_info
- 检测图像中的 ArUco marker
- 输出每条 marker 的检测结果

#### 4.1.2 数据结构

```cpp
namespace aruco_reloc {

/// 单个 marker 检测结果
struct MarkerObservation {
  int id;                                    // marker ID
  std::array<cv::Point2f, 4> corners;       // 四个角点像素坐标
  cv::Vec3d rvec, tvec;                     // 旋转向量、平移向量（相机坐标系）
  Eigen::Isometry3d T_cam_marker;           // 相机到 marker 的变换
  double reprojection_error;                // 角点重投影误差
};

/// 一帧检测结果
struct FrameDetection {
  ros::Time stamp;
  std::vector<MarkerObservation> markers;
  cv::Mat debug_image;                      // 可选：带标注的调试图像
};

}  // namespace aruco_reloc
```

#### 4.1.3 核心接口

```cpp
class MarkerDetector {
 public:
  struct Config {
    std::string dictionary = "DICT_4X4_50";  // ArUco 字典类型
    double marker_size = 0.2;                // marker 物理边长（米）
    int min_corners_per_marker = 4;          // 最少角点数
    double corner_refinement_win_size = 5;   // 角点精化窗口
    bool corner_subpix = true;               // 是否亚像素精化
  };

  bool initialize(const Config& config, const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs);

  /// 检测一帧图像中的所有 marker
  FrameDetection detect(const cv::Mat& image, const ros::Time& stamp) const;

  /// 获取相机内参
  const cv::Mat& cameraMatrix() const { return camera_matrix_; }
  const cv::Mat& distCoeffs() const { return dist_coeffs_; }

 private:
  Config config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
};

}  // namespace aruco_reloc
```

#### 4.1.4 实现要点

1. **字典选择**：默认 `DICT_4X4_50`（50 个 marker，足够矿井场景）
2. **角点精化**：启用 `CORNER_REFINE_SUBPIX` 提高精度
3. **相机模型**：支持鱼眼/普通针孔模型（通过 camera_info 自动识别）
4. **性能优化**：可选降采样检测（大分辨率图像）

---

### 4.2 模块 2：离线建图子模块

#### 4.2.1 职责

- 使用全局优化位姿（map 坐标系）和图像，计算每个 marker 在 map 坐标系下的位姿
- 收集多帧观测，聚合得到最优 marker 位姿
- 输出 marker_map.yaml

#### 4.2.2 数据结构

```cpp
/// 单个 marker 地图项
struct MarkerItem {
  int id;
  Eigen::Isometry3d T_map_marker;  // map 坐标系下 marker 位姿
  double size;                     // marker 物理尺寸
  int observation_count;           // 观测帧数
  double mean_reproj_error;        // 平均重投影误差
};

/// marker 地图
using MarkerMap = std::unordered_map<int, MarkerItem>;
```

#### 4.2.3 离线建图流程（使用全局优化位姿）

```
┌─────────────────────────────────────────────────────────────────┐
│                     离线建图流程（map 坐标系）                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  输入：                                                         │
│    1. 图像 bag：/camera/image_raw, /camera/camera_info          │
│    2. 位姿文件：slam_mapping_v2 优化后的 TUM 轨迹               │
│       或 /localization/slam_mapping/optimized_poses 话题        │
│                                                                 │
│  步骤：                                                         │
│  1. 解析位姿文件，按时间戳索引 T_map_base                       │
│                                                                 │
│  2. 回放图像 bag                                                │
│     └── MarkerDetector → 检测结果                               │
│                                                                 │
│  3. 时间同步                                                    │
│     └── 检测结果 与 T_map_base 按时间戳最近邻匹配               │
│                                                                 │
│  4. 坐标系转换                                                  │
│     ├── T_map_cam = T_map_base × T_base_cam (相机外参)         │
│     └── T_map_marker = T_map_cam × T_cam_marker                │
│                                                                 │
│  5. 多帧聚合（同一个 marker ID）                                │
│     ├── 过滤：重投影误差 > 阈值的观测丢弃                       │
│     ├── 聚合：四元数 + 位置加权平均                             │
│     └── 统计：观测帧数、均值误差                                │
│                                                                 │
│  6. 输出 marker_map.yaml                                        │
│     └── id + x y z qw qx qy qz + size + stats                 │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### 4.2.4 位姿来源说明

**方案 A（推荐）：使用 slam_mapping_v2 全局优化位姿**

```
slam_mapping_v2 输出：
  优化后的轨迹文件（TUM 格式）：timestamp tx ty tz qx qy qz qw
  坐标系：map（经过 loop closure 全局优化）

读取方式：
  - 直接读取 TUM 文件
  - 或订阅 /localization/slam_mapping/optimized_poses 话题

优点：
  - 位姿在 map 坐标系下，与系统一致
  - 经过全局优化，精度高
  - 无累积漂移
```

**方案 B：使用 slam_odometry_v2 + T_map_odom（备选）**

```
如果无法获取全局优化位姿，可用 LIO 输出 + 校正：
  T_map_base = T_map_odom × T_odom_base

需要：
  - 订阅 /localization/slam_odom（LIO 输出）
  - 订阅 T_map_odom（如 pose_fusion 输出的 map→odom 校正）

缺点：
  - 在线校正可能不精确
  - 建图和使用时的 T_map_odom 可能不一致
```

#### 4.2.5 YAML 输出格式

```yaml
# marker_map.yaml
# 坐标系：map（全局坐标系，经 loop closure 优化）
frame_id: "map"
source_bag: "eee_02.bag"
source_pose_file: "eee_02_optimized_poses.txt"  # TUM 格式
camera_extrinsic:
  base_to_camera:
    translation: [0.5, 0.0, 1.2]
    rotation: [0.0, 0.0, 0.0, 1.0]  # qw, qx, qy, qz
generation_time: "2026-08-26 10:30:00"

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
    observation_count: 42
    mean_reproj_error: 1.23  # pixels

  - id: 5
    x: 5.678
    y: 3.456
    z: 1.234
    qw: 0.5
    qx: 0.5
    qy: 0.5
    qz: 0.5
    size: 0.2
    observation_count: 38
    mean_reproj_error: 0.98
```

#### 4.2.6 实现要点

1. **位姿文件读取**：
   - 支持 TUM 格式：`timestamp tx ty tz qx qy qz qw`
   - 按时间戳排序，支持最近邻查询
   - 时间戳单位：秒（float64）

2. **时间同步**：
   - 图像时间戳与位姿时间戳最近邻匹配
   - 容差：50ms（可配置）

3. **外参获取**：
   - 从配置文件加载（推荐，精确）
   - 或从 TF 树查询 `base_link → camera_link`（需静态 TF）

4. **聚合算法**：
   - 位置：加权平均（权重 = 1 / reproj_error）
   - 旋转：四元数加权平均（需处理双覆盖问题）

5. **质量过滤**：
   - 单帧重投影误差 > 5 pixels 丢弃
   - 观测帧数 < 5 的 marker 不输出（可配置）

6. **去重策略**：同一 ID 只保留最终聚合结果

---

### 4.3 模块 3：Marker 地图加载模块

#### 4.3.1 职责

- 在线启动时加载 marker_map.yaml
- 提供 marker 查询接口

#### 4.3.2 核心接口

```cpp
class MarkerMapLoader {
 public:
  bool loadFromFile(const std::string& yaml_path);

  /// 查询单个 marker
  bool getMarker(int id, MarkerItem& item) const;

  /// 查询多个 marker
  std::vector<MarkerItem> getMarkers(const std::vector<int>& ids) const;

  /// 获取所有 marker ID
  std::vector<int> getAllIds() const;

  /// 地图大小
  size_t size() const { return map_.size(); }

  /// 坐标系
  const std::string& frameId() const { return frame_id_; }

 private:
  MarkerMap map_;
  std::string frame_id_;
};
```

#### 4.3.3 校验规则

1. YAML 文件不存在或格式错误 → 抛出异常，节点启动失败
2. marker 尺寸 ≤ 0 或 > 10m → 报错并跳过该 marker
3. 四元数不合法（非单位四归一化）→ 报错并跳过
4. 坐标系字段缺失 → 使用默认值 "map" 并打印警告

---

### 4.4 模块 4：重定位求解核心

#### 4.4.1 职责

- 接收当前帧检测结果和 marker 地图
- 执行 ID 匹配和位姿求解
- 输出候选位姿（未经校验）

#### 4.4.2 求解策略

```
┌─────────────────────────────────────────────────────────────────┐
│                     重定位求解流程                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  输入：当前帧检测结果 {markers}，marker 地图 {map}              │
│                                                                 │
│  1. ID 匹配过滤                                                 │
│     └── 只保留在地图中存在的 marker                             │
│                                                                 │
│  2. 分支求解                                                    │
│     ├── 情况 A：检测到 1 个 marker                              │
│     │   └── T_map_cam = T_map_marker × T_cam_marker^(-1)      │
│     │       T_map_base = T_map_cam × T_base_cam^(-1)          │
│     │                                                           │
│     └── 情况 B：检测到 ≥2 个 marker                             │
│         └── 联合 PnP 求解（cv::solvePnP）                       │
│             - 2D：所有 marker 角点像素坐标                       │
│             - 3D：对应 marker 在 map 下的 3D 角点坐标           │
│             → 得到 T_map_cam，再转换为 T_map_base               │
│                                                                 │
│  3. 输出候选位姿（未经校验）                                    │
│     └── T_map_base_candidate                                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### 4.4.3 联合 PnP 求解细节

当检测到 ≥2 个 marker 时：

1. **构建 2D-3D 点对**：
   - 2D 点：所有 marker 的 4 个角点像素坐标
   - 3D 点：对应 marker 在 map 坐标系下的 3D 角点坐标

2. **marker 3D 角点计算**：
   ```
   // marker 中心在 map 下的位置为 T_map_marker（从地图读取）
   // marker 的 4 个角点相对于 marker 中心的偏移
   half = marker_size / 2
   corners_marker = [(-half, -half, 0), (half, -half, 0),
                     (half, half, 0), (-half, half, 0)]
   corners_map = T_map_marker × corners_marker
   ```

3. **PnP 求解**：
   ```cpp
   cv::Mat rvec, tvec;
   cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs,
                rvec, tvec, false, cv::SOLVEPNP_IPPE);
   // 或者用 RANSAC 版本增强鲁棒性
   cv::solvePnPRansac(object_points, image_points, camera_matrix, dist_coeffs,
                      rvec, tvec, false, 100, 8.0, 0.99, inliers,
                      cv::SOLVEPNP_ITERATIVE);
   ```

4. **初始值**（可选）：
   - 如果有上一帧位姿（T_map_base_prev），可作为 PnP 初始值加速收敛
   - 转换为相机位姿：T_map_cam_init = T_map_base_prev × T_base_cam
   - 使用 `cv::SOLVEPNP_ITERATIVE` 并传入初始 rvec/tvec

5. **坐标系转换**：
   ```
   // PnP 求解得到 T_map_cam（相机在 map 下的位姿）
   // 转换为 T_map_base（base_link 在 map 下的位姿）
   T_map_base = T_map_cam × T_base_cam^(-1)

   // 输出：map → base_link
   ```

#### 4.4.4 核心接口

```cpp
class RelocalizationSolver {
 public:
  struct Config {
    int min_markers = 1;               // 最小有效 marker 数
    bool use_ransac = true;            // 是否使用 RANSAC
    int ransac_iterations = 100;       // RANSAC 迭代次数
    double ransac_reproj_error = 8.0;  // RANSAC 重投影误差阈值（pixels）
    double ransac_confidence = 0.99;   // RANSAC 置信度
  };

  struct SolveResult {
    bool success = false;
    Eigen::Isometry3d T_map_cam;       // 求解得到的相机位姿
    int num_markers_used = 0;          // 使用的 marker 数量
    std::vector<int> marker_ids;       // 使用的 marker ID 列表
    std::vector<int> inlier_indices;   // RANSAC 内点索引
    double solve_time_ms = 0.0;        // 求解耗时
    std::string failure_reason;        // 失败原因
  };

  bool initialize(const Config& config, const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs);

  /// 执行重定位求解
  SolveResult solve(const FrameDetection& detection,
                    const MarkerMapLoader& map) const;

 private:
  /// 单 marker 求解
  SolveResult solveSingleMarker(const MarkerObservation& obs,
                                const MarkerItem& map_item) const;

  /// 多 marker 联合 PnP 求解
  SolveResult solveMultiMarker(const std::vector<MarkerObservation>& observations,
                               const std::vector<MarkerItem>& map_items) const;

  /// 计算 marker 在 map 下的 3D 角点
  std::vector<cv::Point3d> computeMarkerCorners(const MarkerItem& item) const;

  Config config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

}  // namespace aruco_reloc
```

---

### 4.5 模块 5：多重校验层

#### 4.5.1 职责

- 对候选位姿进行多重校验
- 过滤错误的重定位结果
- 输出最终可靠的位姿

#### 4.5.2 校验流程

```
┌─────────────────────────────────────────────────────────────────┐
│                     多重校验流程                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  输入：候选位姿 T_map_cam_candidate，检测结果，marker 地图      │
│                                                                 │
│  校验 1：重投影误差                                              │
│  ├── 将 map 下 marker 3D 角点投影回图像                         │
│  ├── 计算与检测角点的像素误差                                   │
│  └── 阈值：mean_error < 10 pixels（可配置）                     │
│                                                                 │
│  校验 2：几何一致性（多 marker 时）                              │
│  ├── 计算检测到的 marker 两两之间的距离                         │
│  ├── 对比 marker_map 中存储的距离                               │
│  └── 阈值：distance_error < 0.1m（可配置）                      │
│                                                                 │
│  校验 3：运动一致性（有历史位姿时）                              │
│  ├── 对比上一帧位姿与当前候选位姿                               │
│  ├── 平移跳变 < 2m，旋转跳变 < 30°（可配置）                    │
│  └── 绑架初始化阶段可放宽阈值                                   │
│                                                                 │
│  校验 4：数量阈值                                                │
│  └── 有效 marker 数量 ≥ min_markers（推荐 ≥2）                  │
│                                                                 │
│  全部通过 → 输出最终位姿                                        │
│  任一失败 → 输出失败，标记 reject_reason                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

#### 4.5.3 核心接口

```cpp
class RelocalizationValidator {
 public:
  struct Config {
    // 重投影误差校验
    double max_reproj_error = 10.0;    // pixels

    // 几何一致性校验
    double max_marker_distance_error = 0.1;  // meters

    // 运动一致性校验
    double max_translation_jump = 2.0;       // meters
    double max_rotation_jump = 30.0;         // degrees
    bool use_motion_check = true;

    // 数量阈值
    int min_markers = 1;
  };

  struct ValidationResult {
    bool passed = false;
    double mean_reproj_error = 0.0;
    double max_marker_distance_error = 0.0;
    double translation_jump = 0.0;
    double rotation_jump = 0.0;
    int num_valid_markers = 0;
    std::string reject_reason;           // 失败原因
  };

  bool initialize(const Config& config, const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs);

  /// 执行校验
  ValidationResult validate(
      const RelocalizationSolver::SolveResult& candidate,
      const FrameDetection& detection,
      const MarkerMapLoader& map,
      const Eigen::Isometry3d* last_pose = nullptr) const;

 private:
  /// 校验 1：重投影误差
  double computeReprojectionError(
      const Eigen::Isometry3d& T_map_cam,
      const std::vector<MarkerObservation>& observations,
      const MarkerMapLoader& map) const;

  /// 校验 2：几何一致性
  double computeMarkerDistanceError(
      const std::vector<MarkerObservation>& observations,
      const MarkerMapLoader& map) const;

  /// 校验 3：运动一致性
  std::pair<double, double> computeMotionJump(
      const Eigen::Isometry3d& current_pose,
      const Eigen::Isometry3d& last_pose) const;

  Config config_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

}  // namespace aruco_reloc
```

#### 4.5.4 重投影误差计算

```cpp
double computeReprojectionError(
    const Eigen::Isometry3d& T_map_cam,
    const std::vector<MarkerObservation>& observations,
    const MarkerMapLoader& map) {

  double total_error = 0.0;
  int total_points = 0;

  for (const auto& obs : observations) {
    MarkerItem map_item;
    if (!map.getMarker(obs.id, map_item)) continue;

    // 计算 marker 在 map 下的 3D 角点
    auto corners_3d = computeMarkerCorners(map_item);

    // 转换到相机坐标系
    Eigen::Isometry3d T_cam_map = T_map_cam.inverse();
    std::vector<cv::Point3d> corners_cam;
    for (const auto& p : corners_3d) {
      Eigen::Vector3d p_cam = T_cam_map * Eigen::Vector3d(p.x, p.y, p.z);
      corners_cam.emplace_back(p_cam.x(), p_cam.y(), p_cam.z());
    }

    // 投影到图像
    std::vector<cv::Point2d> projected;
    cv::projectPoints(corners_cam, cv::Vec3d::zeros(), cv::Vec3d::zeros(),
                      camera_matrix_, dist_coeffs_, projected);

    // 计算误差
    for (int i = 0; i < 4; ++i) {
      double dx = projected[i].x - obs.corners[i].x;
      double dy = projected[i].y - obs.corners[i].y;
      total_error += std::sqrt(dx * dx + dy * dy);
      total_points++;
    }
  }

  return total_points > 0 ? total_error / total_points : 999.0;
}
```

#### 4.5.5 几何一致性计算

```cpp
double computeMarkerDistanceError(
    const std::vector<MarkerObservation>& observations,
    const MarkerMapLoader& map) {

  double max_error = 0.0;

  for (size_t i = 0; i < observations.size(); ++i) {
    for (size_t j = i + 1; j < observations.size(); ++j) {
      MarkerItem item_i, item_j;
      if (!map.getMarker(observations[i].id, item_i) ||
          !map.getMarker(observations[j].id, item_j)) continue;

      // 地图中的距离
      double dist_map = (item_i.T_map_marker.translation() -
                         item_j.T_map_marker.translation()).norm();

      // 通过当前帧估计的距离（使用求解得到的 T_map_cam）
      // 这里需要将观测转换到 map 坐标系
      // 简化：直接用 marker 在相机坐标系下的距离（刚体变换保持距离）
      double dist_cam = (observations[i].T_cam_marker.translation() -
                         observations[j].T_cam_marker.translation()).norm();

      double error = std::abs(dist_map - dist_cam);
      max_error = std::max(max_error, error);
    }
  }

  return max_error;
}
```

---

## 5. 节点接口设计

### 5.1 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/image_raw` | sensor_msgs/Image | 相机图像 |
| `/camera/camera_info` | sensor_msgs/CameraInfo | 相机内参 |
| `/slam_pose` 或 VIO 输出 | geometry_msgs/PoseStamped | 离线建图时的 VIO 位姿 |

### 5.2 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/aruco_reloc/result` | RelocalizationResult | 重定位结果（成功/失败 + 位姿） |
| `/aruco_reloc/pose` | PoseStamped | 重定位位姿（仅成功时） |
| `/aruco_reloc/status` | String | 状态字符串（调试用） |
| `/aruco_reloc/debug_image` | Image | 带标注的调试图像（可选） |

### 5.3 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/aruco_reloc/start` | std_srvs/Trigger | 启动在线重定位 |
| `/aruco_reloc/stop` | std_srvs/Trigger | 停止在线重定位 |

### 5.4 参数

```yaml
# config/aruco_relocalization.yaml

# 运行模式
mode: "online"  # "offline_mapping" 或 "online"

# 路径
marker_map_path: "$(find aruco_relocalization)/config/marker_map.yaml"

# ArUco 配置
aruco:
  dictionary: "DICT_4X4_50"
  marker_size: 0.2  # meters
  corner_subpix: true

# 相机外参（base_link → camera_link）
extrinsic:
  base_to_camera:
    translation: [0.5, 0.0, 1.2]  # x, y, z (meters)
    rotation: [0.0, 0.0, 0.0, 1.0]  # qw, qx, qy, qz
  use_tf: true  # 是否从 TF 树获取外参（优先级高于配置文件）

# 离线建图配置（使用全局优化位姿）
offline_mapping:
  image_bag_path: ""  # 图像 bag 文件
  pose_file: ""  # 位姿文件（TUM 格式：timestamp tx ty tz qx qy qz qw）
  # 或者从话题获取位姿（如 slam_mapping_v2 优化后的轨迹）
  pose_topic: ""  # 可选，如 /localization/slam_mapping/optimized_poses
  output_path: "$(find aruco_relocalization)/config/marker_map.yaml"
  time_sync_tolerance: 0.05  # 秒
  min_observations: 5
  max_reproj_error: 5.0  # pixels

# 求解配置
solver:
  min_markers: 1
  use_ransac: true
  ransac_iterations: 100
  ransac_reproj_error: 8.0

# 校验配置
validator:
  max_reproj_error: 10.0  # pixels
  max_marker_distance_error: 0.1  # meters
  max_translation_jump: 2.0  # meters
  max_rotation_jump: 30.0  # degrees
  use_motion_check: true
  min_markers: 2  # 推荐 ≥2

# 坐标系（与 relocalization/global_matcher 保持一致）
frame_id: "map"  # 输出 map 坐标系下的位姿
child_frame_id: "base_link"  # 输出 base_link 位姿

# 话题
topics:
  image: "/camera/image_raw"
  camera_info: "/camera/camera_info"
  slam_odom: "/localization/slam_odom"  # 可选，用于运动一致性校验和 PnP 初始值
  result: "/aruco_reloc/result"  # 发布重定位结果（RelocalizationResult）
  pose: "/aruco_reloc/pose"
  status: "/aruco_reloc/status"
  command: "/aruco_reloc/command"  # 接收启停命令
  debug_image: "/aruco_reloc/debug_image"

# 调试
debug:
  publish_debug_image: false
  log_level: "INFO"
```

### 5.5 localization_v2 集成配置

```yaml
# localization_v2.yaml（只需修改这两项）
relocalization_result_topic: "/aruco_reloc/result"  # 指向 ArUco 节点
relocalization_command_topic: "/aruco_reloc/command"  # 启停命令
```

**集成方式**：
1. `localization_v2::RelocalizationRosAdapter` 自动订阅 `/aruco_reloc/result`
2. 接收到 `RelocalizationResult` 后通过 callback 传递给初始化状态机
3. 门控校验复用现有逻辑（fitness_score, inlier_ratio 等）
4. 成功后作为初始化/重置事件传递给 pose_fusion

**输出位姿语义**：
- `header.frame_id = "map"`（与 relocalization/GM 一致）
- `child_frame_id = "base_link"`
- `pose` = T_map_base（map → base_link）

**与现有模块的一致性**：

| 模块 | frame_id | 输出 | 方法 |
|------|----------|------|------|
| relocalization | "map" | T_map_base | Scan Context + GICP |
| global_matcher | "map" | T_map_base | 描述子 + 匹配 |
| **aruco_reloc** | **"map"** | **T_map_base** | **ArUco + PnP** |

---

## 6. 代码结构设计

### 6.1 目录结构

```
aruco_relocalization/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── aruco_relocalization.yaml    # 节点配置
│   └── marker_map.yaml             # 离线生成的 marker 地图
├── launch/
│   ├── aruco_relocalization.launch  # 在线重定位启动
│   └── offline_mapping.launch       # 离线建图启动
├── include/
│   └── aruco_relocalization/
│       ├── common/
│       │   └── types.hpp            # 数据结构定义
│       ├── detector/
│       │   └── marker_detector.hpp  # Marker 检测器
│       ├── mapping/
│       │   ├── marker_map_item.hpp  # 地图项定义
│       │   └── offline_mapper.hpp   # 离线建图器
│       ├── map/
│       │   └── marker_map_loader.hpp# 地图加载器
│       ├── solver/
│       │   └── relocalization_solver.hpp # 求解器
│       ├── validator/
│       │   └── relocalization_validator.hpp # 校验器
│       └── ros/
│           ├── param_loader.hpp     # 参数加载
│           └── node.hpp             # 节点类
├── src/
│   ├── detector/
│   │   └── marker_detector.cpp
│   ├── mapping/
│   │   └── offline_mapper.cpp
│   ├── map/
│   │   └── marker_map_loader.cpp
│   ├── solver/
│   │   └── relocalization_solver.cpp
│   ├── validator/
│   │   └── relocalization_validator.cpp
│   └── ros/
│       ├── param_loader.cpp
│       ├── node.cpp
│       ├── online_node.cpp          # 在线重定位节点 main
│       └── offline_mapping_node.cpp # 离线建图节点 main
└── test/
    ├── test_marker_detector.cpp
    ├── test_solver.cpp
    └── test_validator.cpp
```

### 6.2 类关系图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              ArucoRelocNode                             │
│  (ROS 节点：协调各模块，处理话题通信)                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  - detector_: MarkerDetector                                            │
│  - map_loader_: MarkerMapLoader                                         │
│  - solver_: RelocalizationSolver                                        │
│  - validator_: RelocalizationValidator                                  │
│  - last_pose_: Eigen::Isometry3d (上一帧位姿，用于运动一致性校验)       │
└─────────────────────────────────────────────────────────────────────────┘
        │               │               │               │
        ▼               ▼               ▼               ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐ ┌───────────────┐
│ MarkerDetector│ │MarkerMapLoader│ │RelocSolver    │ │RelocValidator │
├───────────────┤ ├───────────────┤ ├───────────────┤ ├───────────────┤
│ - detect()    │ │ - loadFile()  │ │ - solve()     │ │ - validate()  │
│ - camera_info │ │ - getMarker() │ │ - solveSingle │ │ - reprojErr   │
│ - dictionary  │ │ - getAllIds() │ │ - solveMulti  │ │ - geomConsist │
└───────────────┘ └───────────────┘ └───────────────┘ └───────────────┘
```

---

## 7. 坐标系设计与位姿来源

### 7.1 关键约束

**与现有系统保持一致**：
- `relocalization` 输出：**map → base_link**（frame_id = "map"）
- `global_matcher_v2` 输出：**map → base_link**（frame_id = "map"）
- `localization_v2` 期望输入：**map 坐标系下的位姿**

**结论**：ArUco 重定位也必须输出 **map → base_link**

### 7.2 坐标系关系

```
                     map (全局坐标系，经 loop closure 优化)
                       │
                       │ T_map_odom (可选，如果 odom≠map)
                       ▼
                     odom (里程计坐标系，有累积漂移)
                       │
                       │ T_odom_base (slam_odometry_v2 输出)
                       ▼
                   base_link (车体坐标系)
                       │
                       │ T_base_cam (相机外参，静态)
                       ▼
                   camera_link (相机坐标系)
                       │
                       │ T_cam_marker (ArUco 检测得到)
                       ▼
                   marker_xxx (路标坐标系)
```

### 7.3 离线建图：位姿来源选择

| 方案 | 位姿来源 | 坐标系 | 优点 | 缺点 |
|------|----------|--------|------|------|
| **A（推荐）** | slam_mapping_v2 全局优化位姿 | **map** | 与系统一致，精度高 | 需要 loop closure 后的位姿 |
| B | slam_odometry_v2 + T_map_odom | map | 可用原始 LIO | 需额外维护 T_map_odom |
| C | slam_odometry_v2（假设 odom≈map） | odom | 简单 | 长期漂移，不精确 |

**推荐方案 A**：使用 `slam_mapping_v2` 输出的全局优化位姿（经过 loop closure）

```
离线建图（方案 A）：
  位姿来源：slam_mapping_v2 输出的全局位姿文件（TUM 格式）
  T_map_base ← 从位姿文件读取
  T_base_cam ← 相机外参（标定）
  T_map_cam  = T_map_base × T_base_cam
  T_map_marker = T_map_cam × T_cam_marker

  marker_map.yaml 存储：T_map_marker（map 坐标系下）
```

### 7.4 在线重定位：输出 map → base_link

```
在线重定位：
  T_map_marker ← marker_map.yaml（map 坐标系下）
  T_cam_marker ← 检测得到

  单 marker：
    T_map_cam  = T_map_marker × T_cam_marker^(-1)
    T_map_base = T_map_cam × T_base_cam^(-1)

  多 marker（PnP）：
    求解得到 T_map_cam（相机在 map 下的位姿）
    T_map_base = T_map_cam × T_base_cam^(-1)

  输出：T_map_base（map → base_link）
  frame_id = "map"，child_frame_id = "base_link"
```

### 7.5 与 slam_odometry_v2 的关系

**在线重定位阶段**：
- 订阅 `/localization/slam_odom`（可选）：
  - 用于运动一致性校验（防止位姿跳变）
  - 作为 PnP 初始值（加速收敛）
- 不依赖 LIO 输出进行位姿计算（避免坐标系混淆）

**初始化场景**：
- 重定位成功 → 输出 T_map_base
- localization_v2 接收后，可计算 T_map_odom = T_map_base × T_odom_base^(-1)
- 后续由 LIO 提供增量位姿，pose_fusion 维护 map→odom 校正

### 7.6 外参配置

```yaml
# 相机外参（base_link → camera_link）
extrinsic:
  base_to_camera:
    translation: [0.5, 0.0, 1.2]  # x, y, z (meters)
    rotation: [0.0, 0.0, 0.0, 1.0]  # qw, qx, qy, qz
  # 或者使用 TF 树自动获取
  use_tf: true
  tf_timeout: 0.1
```

### 7.7 离线建图位姿文件格式

建议使用 TUM 格式（与项目其他工具一致）：
```
# timestamp tx ty tz qx qy qz qw
1629847200.123 1.234 2.345 0.567 0.0 0.0 0.707 0.707
1629847200.223 1.256 2.367 0.568 0.0 0.0 0.708 0.706
...
```

**来源**：
- `slam_mapping_v2` 优化后的轨迹文件
- 或从 `/localization/slam_mapping/optimized_poses` 话题提取

---

## 8. 与现有系统集成方案

### 8.1 集成架构（解耦设计）

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│   aruco_relocalization (独立节点)                                   │
│   ├── 订阅：/camera/image_raw, /camera/camera_info                │
│   ├── 订阅：/localization/slam_odom (可选，用于运动一致性校验)     │
│   └── 发布：/aruco_reloc/result (RelocalizationResult)            │
│                                                                     │
└────────────────────────────────┬────────────────────────────────────┘
                                 │
                                 │ RelocalizationResult
                                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│   localization_v2 (已有架构)                                        │
│   ├── RelocalizationRosAdapter ← 自动订阅 /aruco_reloc/result     │
│   ├── 门控校验（fitness_score, inlier_ratio 等）                   │
│   └── 融合输出（pose_fusion / 初始化）                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.2 localization_v2 集成方式

**零侵入集成**：只需修改配置，无需改动 localization_v2 代码。

```yaml
# localization_v2.yaml 配置修改
relocalization_result_topic: "/aruco_reloc/result"  # 指向 ArUco 节点
relocalization_command_topic: "/aruco_reloc/command" # 启停命令
```

**existing 架构支持**：
1. `localization_v2::RelocalizationRosAdapter` 已实现 `IRelocalizationClient` 接口
2. 自动订阅配置的 result_topic，解析 `RelocalizationResult` 消息
3. 通过 callback 将结果传递给初始化状态机

### 8.3 消息复用

使用现有的 `RelocalizationResult.msg`，字段映射：

| 字段 | ArUco 填充说明 |
|------|----------------|
| `header.stamp` | 图像时间戳 |
| `header.frame_id` | "map" 或 "odom"（取决于建图坐标系） |
| `child_frame_id` | "base_link" 或 "camera_link" |
| `pose` | T_map_base（转换后）或 T_map_cam（原始） |
| `success` | 校验是否通过 |
| `method` | "aruco" |
| `map_id` | marker_map 标识 |
| `fitness_score` | 重投影误差（pixels → 归一化） |
| `inlier_ratio` | 有效 marker 比例 |
| `inlier_rmse` | 重投影 RMSE |
| `inlier_count` | 有效 marker 数量 |
| `reject_reason` | 失败原因（如 "reproj_error", "geom_inconsistency"） |

### 8.4 localization_manager 集成（可选）

如果需要经过 `localization_manager` 门控：

1. `localization_manager` 已有 `onRelocalizationResult` 回调
2. 可增加 ArUco 特定门控阈值（如最小 marker 数量）
3. 门控逻辑复用 `relocalizationTrusted`

### 8.5 pose_fusion 集成

1. ArUco 重定位结果通过 `accepted_relocalization` 传递给 pose_fusion
2. pose_fusion 将其作为重置事件，重置滤波器状态
3. 坐标系需与 pose_fusion 期望的一致（通常 map → base_link）

### 8.6 完整坐标系关系

```
                     map (全局坐标系，经 loop closure 优化)
                       │
                       │ 存储在 marker_map.yaml
                       ▼
                   marker_xxx (路标坐标系)
                       │
                       │ T_cam_marker (ArUco 检测得到)
                       ▼
                   camera_link (相机坐标系)
                       │
                       │ T_base_cam^(-1) (相机外参的逆)
                       ▼
                   base_link (车体坐标系)
                       │
                       │ slam_odometry_v2 输出（增量）
                       ▼
                     odom (里程计坐标系)

坐标系变换链：

离线建图（建立 marker 地图）：
  T_map_base ← 全局优化位姿（slam_mapping_v2）
  T_map_cam  = T_map_base × T_base_cam
  T_map_marker = T_map_cam × T_cam_marker
  → 存储 T_map_marker 到 marker_map.yaml

在线重定位（求解相机位姿）：
  T_map_marker ← marker_map.yaml（已知）
  T_cam_marker ← 检测得到
  T_map_cam    = T_map_marker × T_cam_marker^(-1)
  T_map_base   = T_map_cam × T_base_cam^(-1)
  → 输出 T_map_base（map → base_link）
```

---

## 8. 实施计划

### 8.1 阶段 1：基础功能（1 周）

1. 创建包骨架（CMakeLists.txt, package.xml）
2. 实现 MarkerDetector（模块 1）
3. 实现 MarkerMapLoader（模块 3）
4. 基本节点框架 + 参数加载

### 8.2 阶段 2：核心算法（1 周）

1. 实现 RelocalizationSolver（模块 4）
2. 实现 RelocalizationValidator（模块 5）
3. 在线重定位节点联调

### 8.3 阶段 3：离线工具（3 天）

1. 实现 OfflineMapper（模块 2）
2. 离线建图节点：
   - 读取全局优化位姿文件（TUM 格式）
   - 回放图像 bag，检测 marker
   - 计算 T_map_marker，聚合输出 marker_map.yaml
3. 验证 marker 位姿在 map 坐标系下正确

### 8.4 阶段 4：集成测试（3 天）

1. **与 localization_v2 零侵入集成**：
   - 修改 `localization_v2.yaml` 配置：
     ```yaml
     relocalization_result_topic: "/aruco_reloc/result"
     relocalization_command_topic: "/aruco_reloc/command"
     ```
   - 无需改动 localization_v2 代码
2. 验证 RelocalizationRosAdapter 正确接收结果
3. 验证输出位姿在 map 坐标系下（与 relocalization/GM 一致）
4. 与 pose_fusion 对接（初始化/重置测试）
5. 真实数据测试（eee_02）

### 8.5 阶段 5：优化迭代（持续）

1. 根据测试结果调整参数
2. 性能优化（GPU 加速检测等）
3. 多字典支持
4. 多相机支持

---

## 9. 优化建议

### 9.1 检测优化

1. **自适应阈值**：根据图像亮度动态调整检测参数
2. **ROI 检测**：利用上一帧 marker 位置预测 ROI，加速检测
3. **多线程**：检测和求解可以流水线化

### 9.2 求解优化

1. **初始值传播**：利用上一帧位姿作为 PnP 初始值
2. **多假设求解**：单 marker 时生成多个候选（考虑对称性），通过校验选择最优
3. **滑动窗口**：多帧 marker 观测联合优化（类似 mini BA）

### 9.3 鲁棒性优化

1. **误检测过滤**：
   - 检测置信度过低丢弃
   - 检测到的 marker 尺寸与预期偏差过大丢弃
2. **地图质量**：
   - 离线建图时剔除低质量观测
   - 标记 marker 的"可信度"（观测帧数、误差统计）
3. **故障恢复**：
   - 连续失败 N 次后重置状态
   - 支持手动触发重定位

### 9.4 扩展性

1. **CharUco 支持**：扩展支持 CharUco board，提高精度
2. **动态地图**：支持在线更新 marker 位置（如 marker 被移动）
3. **多相机支持**：扩展支持多相机系统

---

## 10. 测试方案

### 10.1 单元测试

1. MarkerDetector：检测率、误检率、角点精度
2. RelocalizationSolver：求解精度、鲁棒性（噪声、遮挡）
3. RelocalizationValidator：校验准确率

### 10.2 集成测试

1. 离线建图：回放 eee_02，生成 marker_map.yaml，验证地图质量
2. 在线重定位：实时检测 + 求解 + 校验，输出位姿
3. 系统集成：与 localization_manager / pose_fusion 对接

### 10.3 性能测试

1. 检测延迟：不同分辨率、不同 marker 数量
2. 求解延迟：单 marker vs 多 marker
3. 端到端延迟：从图像输入到重定位输出

### 10.4 场景测试

1. 正常场景：marker 清晰可见
2. 挑战场景：
   - 远距离（marker 占比小）
   - 大角度倾斜
   - 部分遮挡
   - 光照变化（阴影、反光）
3. 故障场景：
   - marker 误检测
   - marker 地图错误
   - 相机标定不准

---

## 11. 风险与缓解

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|----------|
| 单 marker PnP 歧义 | 高 | 高 | 多 marker 联合求解；几何一致性校验 |
| 误检测导致错误重定位 | 中 | 高 | ID 白名单；置信度阈值；多重校验 |
| 光照变化导致检测率下降 | 中 | 中 | 自适应阈值；多字典支持 |
| 相机标定不准 | 低 | 高 | 在线标定验证；重投影误差监控 |
| marker 地图与实际不符 | 低 | 高 | 离线建图质量控制；地图版本管理 |

---

## 12. 总结

本设计方案提出了一个完整的 ArUco 路标重定位系统，包括：

1. **5 大核心模块**：检测、离线建图、地图加载、求解、校验
2. **清晰的接口设计**：模块间低耦合，易于测试和扩展
3. **完善的校验机制**：多重校验确保结果可靠性
4. **无缝的系统集成**：复用现有消息和接口
5. **详细的实施计划**：分阶段交付，降低风险

方案技术可行，风险可控，建议按计划实施。

---

## 附录 A：关键设计决策

### A.1 坐标系选择：为什么是 map 而不是 odom？

| 对比项 | odom 坐标系 | map 坐标系（推荐） |
|--------|-------------|-------------------|
| 累积漂移 | 有，长期运行会漂移 | 无，经全局优化 |
| 与系统一致性 | 与 relocalization/GM 不一致 | 完全一致 |
| 初始化精度 | 初始时刻 ok，后续漂移 | 始终精确 |
| 实现复杂度 | 简单（直接用 LIO） | 需要全局优化位姿 |

**结论**：使用 map 坐标系，与 relocalization/global_matcher 保持一致。

**位姿来源**：离线建图时使用 slam_mapping_v2 输出的全局优化位姿（经过 loop closure）。

### A.2 为什么解耦集成到 localization_v2？

| 方案 | 优点 | 缺点 |
|------|------|------|
| 直接集成到 localization_v2 | 紧耦合，调试方便 | 侵入性强，维护困难 |
| 独立节点 + 话题通信（推荐） | 解耦、易测试、易替换 | 需要话题通信开销 |
| Service/Action 调用 | 同步调用，实时性好 | 实现复杂，不利于异步处理 |

**结论**：采用独立节点 + RelocalizationResult 话题的方式，与现有架构完全一致。

### A.3 集成方式总结

```
┌─────────────────────────────────────────────────────────────────────┐
│                        集成方式：零侵入                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  1. aruco_relocalization 作为独立节点运行                           │
│  2. 发布 /aruco_reloc/result（RelocalizationResult）               │
│  3. localization_v2 配置指向该话题                                  │
│  4. RelocalizationRosAdapter 自动订阅并处理                        │
│  5. 无需改动 localization_v2 任何代码                              │
│                                                                     │
│  优势：                                                             │
│  - 解耦：ArUco 节点可独立开发、测试、部署                          │
│  - 灵活：可随时替换为其他重定位方案                                │
│  - 可靠：复用已有门控逻辑，安全性有保障                            │
│  - 简单：配置驱动，无需代码修改                                    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### A.4 输出位姿语义

| 场景 | 输出 | 说明 |
|------|------|------|
| 建图阶段 | T_map_marker | marker 在 map 坐标系下的位姿（存储到 YAML） |
| 重定位阶段 | T_map_base | base_link 在 map 坐标系下的位姿（输出） |

**输出坐标系**：map（与 relocalization/global_matcher 一致）

**消息字段**：
```cpp
RelocalizationResult msg;
msg.header.frame_id = "map";       // map 坐标系
msg.child_frame_id = "base_link";  // base_link
msg.pose.pose = T_map_base;        // map → base_link
```

---

## 附录 B：参考代码结构

```
aruco_relocalization/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── aruco_relocalization.yaml
│   └── marker_map.yaml
├── launch/
│   ├── aruco_relocalization.launch      # 在线重定位
│   └── offline_mapping.launch           # 离线建图
├── include/aruco_relocalization/
│   ├── common/types.hpp
│   ├── detector/marker_detector.hpp
│   ├── mapping/offline_mapper.hpp
│   ├── map/marker_map_loader.hpp
│   ├── solver/relocalization_solver.hpp
│   ├── validator/relocalization_validator.hpp
│   └── ros/node.hpp
└── src/
    ├── detector/marker_detector.cpp
    ├── mapping/offline_mapper.cpp
    ├── map/marker_map_loader.cpp
    ├── solver/relocalization_solver.cpp
    ├── validator/relocalization_validator.cpp
    └── ros/
        ├── node.cpp
        ├── online_node.cpp              # main: 在线重定位
        └── offline_mapping_node.cpp     # main: 离线建图
```

---

## 附录 C：后续扩展方向

1. **多相机支持**：扩展支持多个相机同时检测，提高鲁棒性
2. **CharUco 支持**：支持 CharUco board，提高检测精度
3. **动态地图更新**：支持在线更新 marker 位置
4. **GPU 加速**：使用 CUDA 加速 ArUco 检测（OpenCV contrib）
5. **多字典支持**：支持多种 ArUco 字典，适应不同场景
6. **与 LiDAR 重定位融合**：与现有 Scan Context + GICP 方案形成互补
