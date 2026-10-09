# 回环检测改造：small_gicp 配准 + 内点门控 + 区域裁剪

> 适用包：`src/localization/slam_mapping_v2`
> 目标场景：室外、激光前视约 180°、车辆反向回到同一路段时回环漏检。
>
> 三项改造：
> 1. **配准器**：引入 small_gicp（`gicp` / `vgicp` 两级粗-细配准，可选鲁棒核），保留原 PCL 点到面 ICP 作对照；
> 2. **接受判据**：全局 `getFitnessScore()` 均值 → **内点率 + 内点 RMSE**；
> 3. **区域裁剪**：配准/打分前按候选历史关键帧坐标系下的盒式范围裁剪两侧子图，去掉远端无对应点。
>
> 文档给出全部代码与插入位置，逐文件手工修改即可。

---

## 0. 改动文件总览

| # | 文件 | 改动 |
|---|---|---|
| 1 | `package.xml` | 加 `small_gicp` 依赖 |
| 2 | `CMakeLists.txt` | `small_gicp` 进 catkin 组件 + 链接 OpenMP |
| 3 | `include/slam_mapping_v2/common/config.hpp` | `LoopClosureConfig` 加 3 组配置结构 |
| 4 | `src/ros/config_loader.cpp` | 加载新参数 |
| 5 | `src/core/loop_detector_kdtree_gicp.cpp` | 核心：small_gicp 两级配准 + 内点门控 + 裁剪 |
| 6 | `config/slam_mapping_v2.yaml` | 新增配置段 |

**`include/slam_mapping_v2/core/loop_detector_kdtree_gicp.hpp` 不需要改**——所有新逻辑放在 .cpp 匿名 namespace 中，`icp_` 成员保留给 `pcl_icp` 对照路径。

---

## 1. `package.xml`

在 `<depend>pcl_conversions</depend>` 后加一行：

```xml
  <depend>small_gicp</depend>
```

---

## 2. `CMakeLists.txt`

**2a.** `find_package(catkin REQUIRED COMPONENTS ...)` 列表加 `small_gicp`：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  std_srvs
  cv_bridge
  deeplumin_msgs
  deeplumin_health_checker
  syslog_ros_wrap
  small_gicp
)
```

**2b.** `find_package(OpenCV REQUIRED)` 附近加：

```cmake
find_package(OpenMP REQUIRED)
```

**2c.** `catkin_package(...)` 的 `CATKIN_DEPENDS` 加 `small_gicp`：

```cmake
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs tf2_ros pcl_ros pcl_conversions std_srvs cv_bridge deeplumin_msgs small_gicp
)
```

**2d.** `slam_mapping_v2_core` 的 `target_link_libraries` 加 `OpenMP::OpenMP_CXX`
（small_gicp 的 `ParallelReductionOMP` / `estimate_covariances_omp` 需要，参照 `relocalization/CMakeLists.txt`）：

```cmake
target_link_libraries(slam_mapping_v2_core
  gtsam
  glog
  DBoW2
  ORBextractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
```

`small_gicp` 本体经 `${catkin_LIBRARIES}` 带入（其 catkin_package 导出 `LIBRARIES small_gicp`）。

---

## 3. `include/slam_mapping_v2/common/config.hpp`

**3a.** 在 `struct LoopClosureConfig` **之前**插入三个配置结构：

```cpp
/// 回环配准器配置
struct LoopRegistrationConfig {
  // "vgicp"   : small_gicp 两级 VGICP（粗体素+细体素，推荐，抗部分重叠）
  // "gicp"    : small_gicp 两级 GICP（粗层降采样点云+KdTree，细层原始分辨率）
  // "pcl_icp" : 原 PCL 点到面 ICP（对照用）
  std::string type = "vgicp";
  double max_correspondence_distance = 1.5;  ///< 细层对应点最大距离（米）
  int max_iterations = 30;
  double transformation_epsilon = 1e-6;
  int num_threads = 4;
  int cov_num_neighbors = 20;    ///< GICP 协方差估计邻域点数
  double voxel_resolution = 0.5; ///< 基础体素/降采样边长（米）
  double coarse_scale = 4.0;     ///< 粗层放大倍数（体素边长 or 降采样 leaf、对应距离）
  double fine_scale = 1.0;       ///< 细层放大倍数（仅 VGICP 用）
  std::string robust_kernel = "huber";  ///< "none" | "huber" | "cauchy"
  double robust_kernel_scale = 1.0;     ///< 鲁棒核宽度 c
};

/// 回环内点质量门控（取代全局 fitness 均值判据）
struct LoopInlierGateConfig {
  double inlier_distance = 0.5;   ///< 内点判定距离（米）
  double min_inlier_ratio = 0.4;  ///< 内点率下限
  double max_inlier_rmse = 0.3;   ///< 内点 RMSE 上限（米）
};

/// 打分/配准前区域裁剪（在候选历史关键帧坐标系下的盒式范围，单位米）
struct LoopCropConfig {
  bool enable = true;
  double forward = 40.0;   ///< +x（历史关键帧朝向）
  double backward = 40.0;  ///< -x
  double left = 30.0;      ///< +y
  double right = 30.0;     ///< -y
  double up = 10.0;        ///< +z
  double down = 5.0;       ///< -z
};
```

**3b.** `LoopClosureConfig` 内、`odom_pos_noise` 之后加三个成员：

```cpp
  double odom_rot_noise = 1e-6;   ///< 里程计因子旋转噪声
  double odom_pos_noise = 1e-4;   ///< 里程计因子平移噪声
  LoopRegistrationConfig registration;   ///< 配准器（small_gicp / PCL）
  LoopInlierGateConfig inlier_gate;      ///< 内点门控
  LoopCropConfig crop;                   ///< 打分前区域裁剪
  GnssConfig gnss;
  VisualLoopConfig visual;         ///< 视觉回环配置
```

---

## 4. `src/ros/config_loader.cpp`

在 `loop_closure/odom_pos_noise` 段之后、`// visual loop config` 之前插入：

```cpp
  // loop_closure/registration（配准器）
  auto& lr = cfg.loop_closure.registration;
  pnh.param<std::string>("loop_closure/registration/type", lr.type, lr.type);
  pnh.param<double>("loop_closure/registration/max_correspondence_distance",
                    lr.max_correspondence_distance, lr.max_correspondence_distance);
  pnh.param<int>("loop_closure/registration/max_iterations", lr.max_iterations, lr.max_iterations);
  pnh.param<double>("loop_closure/registration/transformation_epsilon",
                    lr.transformation_epsilon, lr.transformation_epsilon);
  pnh.param<int>("loop_closure/registration/num_threads", lr.num_threads, lr.num_threads);
  pnh.param<int>("loop_closure/registration/cov_num_neighbors", lr.cov_num_neighbors,
                 lr.cov_num_neighbors);
  pnh.param<double>("loop_closure/registration/voxel_resolution", lr.voxel_resolution,
                    lr.voxel_resolution);
  pnh.param<double>("loop_closure/registration/coarse_scale", lr.coarse_scale, lr.coarse_scale);
  pnh.param<double>("loop_closure/registration/fine_scale", lr.fine_scale, lr.fine_scale);
  pnh.param<std::string>("loop_closure/registration/robust_kernel", lr.robust_kernel,
                         lr.robust_kernel);
  pnh.param<double>("loop_closure/registration/robust_kernel_scale", lr.robust_kernel_scale,
                    lr.robust_kernel_scale);

  // loop_closure/inlier_gate（内点门控）
  auto& lg = cfg.loop_closure.inlier_gate;
  pnh.param<double>("loop_closure/inlier_gate/inlier_distance", lg.inlier_distance,
                    lg.inlier_distance);
  pnh.param<double>("loop_closure/inlier_gate/min_inlier_ratio", lg.min_inlier_ratio,
                    lg.min_inlier_ratio);
  pnh.param<double>("loop_closure/inlier_gate/max_inlier_rmse", lg.max_inlier_rmse,
                    lg.max_inlier_rmse);

  // loop_closure/crop（打分前区域裁剪）
  auto& lc = cfg.loop_closure.crop;
  pnh.param<bool>("loop_closure/crop/enable", lc.enable, lc.enable);
  pnh.param<double>("loop_closure/crop/forward", lc.forward, lc.forward);
  pnh.param<double>("loop_closure/crop/backward", lc.backward, lc.backward);
  pnh.param<double>("loop_closure/crop/left", lc.left, lc.left);
  pnh.param<double>("loop_closure/crop/right", lc.right, lc.right);
  pnh.param<double>("loop_closure/crop/up", lc.up, lc.up);
  pnh.param<double>("loop_closure/crop/down", lc.down, lc.down);
```

---

## 5. `src/core/loop_detector_kdtree_gicp.cpp`（核心）

### 5a. include 区追加

在 `#include "syslog/logger.h"` 之前加：

```cpp
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/robust_kernel.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
```

（`pcl/filters/voxel_grid.h` 已由本文件 .hpp 引入，粗层降采样可直接用 `pcl::VoxelGrid`。）

### 5b. 匿名 namespace 追加辅助函数

在 `debugLoopLog` 之后、`}  // namespace` 之前插入：

```cpp
// ===== 区域裁剪：点变换到 local 系后按盒式范围过滤 =====
// T_map_local: local 系到 map 系的变换；裁剪框以 local 原点为中心
PointCloudPtr cropCloudInLocalFrame(const PointCloudPtr& cloud, const Eigen::Matrix4d& T_map_local,
                                    const LoopCropConfig& c) {
  PointCloudPtr out(new PointCloud());
  if (!cloud) return out;
  out->reserve(cloud->size());
  const Eigen::Matrix4d T_local_map = T_map_local.inverse();
  for (const auto& pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    const Eigen::Vector4d p = T_local_map * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
    if (p.x() < -c.backward || p.x() > c.forward) continue;
    if (p.y() < -c.right || p.y() > c.left) continue;
    if (p.z() < -c.down || p.z() > c.up) continue;
    out->push_back(pt);
  }
  out->width = static_cast<std::uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

// ===== 内点统计：配准后源点到目标最近邻距离 < inlier_distance 记为内点 =====
struct InlierStats {
  double ratio = 0.0;  ///< 内点率
  double rmse = 0.0;   ///< 内点 RMSE（米），仅统计内点
  int inliers = 0;
  int total = 0;
};

InlierStats computeInlierStats(const PointCloud& source_aligned, const PointCloudPtr& target,
                               double inlier_distance) {
  InlierStats s;
  if (source_aligned.empty() || !target || target->empty()) return s;
  pcl::KdTreeFLANN<PointType> tree;
  tree.setInputCloud(target);
  const float dist_sq_th = static_cast<float>(inlier_distance * inlier_distance);
  std::vector<int> ids(1);
  std::vector<float> sq(1);
  double sum_sq = 0.0;
  for (const auto& pt : source_aligned.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    if (tree.nearestKSearch(pt, 1, ids, sq) <= 0) continue;
    ++s.total;
    if (sq[0] <= dist_sq_th) {
      ++s.inliers;
      sum_sq += static_cast<double>(sq[0]);
    }
  }
  if (s.total > 0) s.ratio = static_cast<double>(s.inliers) / static_cast<double>(s.total);
  if (s.inliers > 0) s.rmse = std::sqrt(sum_sq / static_cast<double>(s.inliers));
  return s;
}

// ===== 体素降采样（GICP 粗层用） =====
PointCloudPtr voxelDownsample(const PointCloudPtr& cloud, double leaf) {
  if (!cloud || cloud->empty() || leaf <= 1e-4) return cloud;
  PointCloudPtr out(new PointCloud());
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  vg.setInputCloud(cloud);
  vg.filter(*out);
  return sanitizeCloud(out);
}

// ===== small_gicp：点云转换（仅 xyz，法向不参与 GICP） =====
small_gicp::PointCloud::Ptr toSmallGicpCloud(const PointCloudPtr& cloud) {
  std::vector<Eigen::Vector4f> pts;
  if (cloud) {
    pts.reserve(cloud->size());
    for (const auto& p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      pts.emplace_back(p.x, p.y, p.z, 1.0f);
    }
  }
  return std::make_shared<small_gicp::PointCloud>(pts);
}

// ===== small_gicp：因子参数注入（鲁棒核宽度），参照 global_matcher_v2/matcher_impl.hpp =====
inline void configureFactor(small_gicp::GICPFactor::Setting&, double) {}
inline void configureFactor(
    small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>::Setting& s,
    double c) {
  s.robust_kernel.c = c;
}
inline void configureFactor(
    small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>::Setting& s,
    double c) {
  s.robust_kernel.c = c;
}

template <typename Factor>
void configureRegistration(small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP>& reg,
                           const LoopRegistrationConfig& rc, double max_dist) {
  reg.reduction.num_threads = std::max(1, rc.num_threads);
  reg.rejector.max_dist_sq = max_dist * max_dist;
  reg.criteria.translation_eps = rc.transformation_epsilon;
  reg.criteria.rotation_eps = rc.transformation_epsilon;
  reg.optimizer.max_iterations = std::max(1, rc.max_iterations);
  configureFactor(reg.point_factor, rc.robust_kernel_scale);
}

// ===== VGICP 两级：粗体素大收敛域 + 细体素精修（对齐 relocalization/vgicp_global_matcher） =====
template <typename Factor>
small_gicp::RegistrationResult alignTwoStageVgicp(const small_gicp::PointCloud& source,
                                                  const small_gicp::PointCloud& target,
                                                  const Eigen::Isometry3d& init,
                                                  const LoopRegistrationConfig& rc) {
  const double base = std::max(0.05, rc.voxel_resolution);
  const double coarse_scale = std::max(1.0, rc.coarse_scale);
  const double fine_scale = std::max(0.1, rc.fine_scale);

  // ---------- 第一层：粗配准（粗体素 + 放大对应距离） ----------
  auto coarse_map = std::make_shared<small_gicp::GaussianVoxelMap>(base * coarse_scale);
  coarse_map->set_search_offsets(27);
  coarse_map->insert(target);
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> coarse_reg;
  configureRegistration(coarse_reg, rc, rc.max_correspondence_distance * coarse_scale);
  const auto coarse_res = coarse_reg.align(*coarse_map, source, *coarse_map, init);

  // ---------- 第二层：精配准（细体素 + 标准对应距离） ----------
  auto fine_map = std::make_shared<small_gicp::GaussianVoxelMap>(base * fine_scale);
  fine_map->set_search_offsets(27);
  fine_map->insert(target);
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> fine_reg;
  configureRegistration(fine_reg, rc, rc.max_correspondence_distance);
  return fine_reg.align(*fine_map, source, *fine_map, coarse_res.T_target_source);

  // 只保留一层（精配准）时：注释掉【第一层】整段，并把上面 align 的第 4 个参数
  // coarse_res.T_target_source 改回 init 即可。
}

// ===== GICP 两级（KdTree 目标）=====
// 第一层（粗）：体素降采样后的源/目标 + 放大对应距离 → 扩大收敛域、吸收漂移
// 第二层（细）：原始分辨率源/目标 + 标准对应距离 → 收敛精度
// 与 VGICP 的粗/细体素层作用对等，区别只在目标表示（KdTree 点 vs 高斯体素）。
template <typename Factor>
small_gicp::RegistrationResult alignTwoStageGicpTree(
    const small_gicp::PointCloud& source, const small_gicp::PointCloud& target,
    const small_gicp::KdTree<small_gicp::PointCloud>& tree,
    const small_gicp::PointCloud& coarse_source, const small_gicp::PointCloud& coarse_target,
    const small_gicp::KdTree<small_gicp::PointCloud>& coarse_tree,
    const Eigen::Isometry3d& init, const LoopRegistrationConfig& rc) {
  // ---------- 第一层：粗配准（降采样点云 + 放大对应距离） ----------
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> coarse_reg;
  configureRegistration(coarse_reg, rc,
                        rc.max_correspondence_distance * std::max(1.0, rc.coarse_scale));
  const auto coarse_res = coarse_reg.align(coarse_target, coarse_source, coarse_tree, init);

  // ---------- 第二层：精配准（原始分辨率） ----------
  small_gicp::Registration<Factor, small_gicp::ParallelReductionOMP> fine_reg;
  configureRegistration(fine_reg, rc, rc.max_correspondence_distance);
  return fine_reg.align(target, source, tree, coarse_res.T_target_source);

  // 只保留一层（精配准）时：注释掉【第一层】整段，并把上面 align 的第 4 个参数
  // coarse_res.T_target_source 改回 init 即可。
}

// ===== small_gicp 配准入口：source → target，输出 T_target_source =====
bool runSmallGicp(const PointCloudPtr& source_pcl, const PointCloudPtr& target_pcl,
                  const LoopRegistrationConfig& rc, Eigen::Matrix4d* T_out,
                  bool* converged_out) {
  if (!T_out || !converged_out) return false;
  auto source = toSmallGicpCloud(source_pcl);
  auto target = toSmallGicpCloud(target_pcl);
  if (!source || !target || source->size() < 10 || target->size() < 10) return false;

  const int k = std::max(3, rc.cov_num_neighbors);
  const int nthreads = std::max(1, rc.num_threads);
  small_gicp::estimate_covariances_omp(*target, k, nthreads);
  small_gicp::estimate_covariances_omp(*source, k, nthreads);

  const Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
  small_gicp::RegistrationResult res;

  if (rc.type != "gicp") {
    // ---------- VGICP 两级 ----------
    if (rc.robust_kernel == "huber") {
      res = alignTwoStageVgicp<
          small_gicp::RobustFactor<small_gicp::Huber, small_gicp::GICPFactor>>(
          *source, *target, init, rc);
    } else if (rc.robust_kernel == "cauchy") {
      res = alignTwoStageVgicp<
          small_gicp::RobustFactor<small_gicp::Cauchy, small_gicp::GICPFactor>>(
          *source, *target, init, rc);
    } else {
      res = alignTwoStageVgicp<small_gicp::GICPFactor>(*source, *target, init, rc);
    }
  } else {
    // ---------- GICP 两级（KdTree）----------
    // 细层：原始分辨率目标树
    small_gicp::KdTree<small_gicp::PointCloud> fine_tree(target,
                                                        small_gicp::KdTreeBuilderOMP(nthreads));

    // 粗层：体素降采样点云（leaf = voxel_resolution * coarse_scale）+ 其目标树
    const double coarse_leaf =
        std::max(0.05, rc.voxel_resolution * std::max(1.0, rc.coarse_scale));
    auto source_coarse = toSmallGicpCloud(voxelDownsample(source_pcl, coarse_leaf));
    auto target_coarse = toSmallGicpCloud(voxelDownsample(target_pcl, coarse_leaf));
    if (!source_coarse || !target_coarse || source_coarse->size() < 10 ||
        target_coarse->size() < 10) {
      return false;
    }
    small_gicp::estimate_covariances_omp(*target_coarse, k, nthreads);
    small_gicp::estimate_covariances_omp(*source_coarse, k, nthreads);
    small_gicp::KdTree<small_gicp::PointCloud> coarse_tree(
        target_coarse, small_gicp::KdTreeBuilderOMP(nthreads));

    if (rc.robust_kernel == "huber") {
      res = alignTwoStageGicpTree<small_gicp::RobustFactor<small_gicp::Huber,
                                                           small_gicp::GICPFactor>>(
          *source, *target, fine_tree, *source_coarse, *target_coarse, coarse_tree, init, rc);
    } else if (rc.robust_kernel == "cauchy") {
      res = alignTwoStageGicpTree<small_gicp::RobustFactor<small_gicp::Cauchy,
                                                           small_gicp::GICPFactor>>(
          *source, *target, fine_tree, *source_coarse, *target_coarse, coarse_tree, init, rc);
    } else {
      res = alignTwoStageGicpTree<small_gicp::GICPFactor>(
          *source, *target, fine_tree, *source_coarse, *target_coarse, coarse_tree, init, rc);
    }
  }

  *T_out = res.T_target_source.matrix();
  *converged_out = res.converged;
  return T_out->array().isFinite().all();
}
```

### 5c. `configure()` 去掉硬编码 20m

```cpp
void KdtreeGicpLoopDetector::configure(const LoopClosureConfig& cfg) {
  cfg_ = cfg;
  kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  cloud_history_poses_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  submap_voxel_.setLeafSize(cfg_.submap_resolution, cfg_.submap_resolution, cfg_.submap_resolution);
  icp_.reset(new pcl::IterativeClosestPointWithNormals<PointType, PointType>());
  icp_->setMaxCorrespondenceDistance(cfg_.registration.max_correspondence_distance);  // 原为 20.0
  icp_->setMaximumIterations(cfg_.registration.max_iterations);
  icp_->setTransformationEpsilon(cfg_.registration.transformation_epsilon);
  icp_->setEuclideanFitnessEpsilon(cfg_.registration.transformation_epsilon);
  icp_->setRANSACIterations(0);
  reset();
}
```

### 5d. 整体替换 `alignCandidateSubmaps`

用下面实现**整函数替换**原 `alignCandidateSubmaps`（原文件约 201-304 行）：

```cpp
bool KdtreeGicpLoopDetector::alignCandidateSubmaps(const std::vector<Keyframe>& kfs,
                                                   int current_idx, int history_idx,
                                                   double* score, Mat3d* diff_rot,
                                                   Vec3d* diff_pos) {
  if (!score || !diff_rot || !diff_pos) return false;
  auto cur_submap = buildSubmap(kfs, current_idx, 0);
  auto hist_submap = buildSubmap(kfs, history_idx, cfg_.submap_search_num);

  // ---- 区域裁剪：配准与打分前只保留候选历史关键帧附近的盒式区域 ----
  // 两侧都裁到同一个框 → 只比较双方覆盖的交集，去掉远端无对应点
  if (cfg_.crop.enable) {
    const Eigen::Matrix4d T_hist = poseToMatrix(kfs[history_idx].pose_map);
    cur_submap = cropCloudInLocalFrame(cur_submap, T_hist, cfg_.crop);
    hist_submap = cropCloudInLocalFrame(hist_submap, T_hist, cfg_.crop);
  }

  latest_current_submap_ = cur_submap;
  latest_history_submap_ = hist_submap;
  if (!cur_submap || !hist_submap || cur_submap->empty() || hist_submap->empty()) return false;

  const auto& rc = cfg_.registration;
  Eigen::Matrix4d T_hist_cur = Eigen::Matrix4d::Identity();
  bool converged = false;

  if (rc.type == "pcl_icp") {
    // ---------- 对照路径：原 PCL 点到面 ICP（需法向量） ----------
    auto addNormals = [](const PointCloudPtr& cloud) -> PointCloudPtr {
      auto filtered = sanitizeCloud(cloud);
      if (!filtered || filtered->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
        return PointCloudPtr(new PointCloud());
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::copyPointCloud(*filtered, *xyz);
      std::vector<int> idx;
      pcl::removeNaNFromPointCloud(*xyz, *xyz, idx);
      if (xyz->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
        return PointCloudPtr(new PointCloud());
      }
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
      tree->setInputCloud(xyz);
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
      pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
      ne.setInputCloud(xyz);
      ne.setSearchMethod(tree);
      ne.setKSearch(std::min<int>(kMinNormalEstimationPoints, static_cast<int>(xyz->size())));
      ne.compute(*normals);
      PointCloudPtr out(new PointCloud());
      pcl::concatenateFields(*xyz, *normals, *out);
      return sanitizeCloud(out);
    };

    auto cur_norm = sanitizeCloud(addNormals(cur_submap));
    auto hist_norm = sanitizeCloud(addNormals(hist_submap));
    if (!cur_norm || !hist_norm ||
        cur_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints) ||
        hist_norm->size() < static_cast<std::size_t>(kMinNormalEstimationPoints)) {
      return false;
    }
    PointCloud aligned;
    icp_->setInputSource(cur_norm);
    icp_->setInputTarget(hist_norm);
    icp_->align(aligned, Eigen::Matrix4f::Identity());
    converged = icp_->hasConverged();
    T_hist_cur = icp_->getFinalTransformation().cast<double>();
  } else {
    // ---------- small_gicp：VGICP 两级 / GICP 两级（可带鲁棒核） ----------
    // 两侧子图都在 map 系，漂移为零时 T=I，故用单位阵作初值；
    // 粗配准层负责扩大收敛域以吸收漂移。
    if (!runSmallGicp(cur_submap, hist_submap, rc, &T_hist_cur, &converged)) {
      return false;
    }
  }

  if (!converged || !T_hist_cur.array().isFinite().all()) {
    debugLoopLog("align rejected: converged=" + std::to_string(converged));
    return false;
  }

  // ---- 内点质量门控：内点率 + 内点 RMSE ----
  PointCloud source_aligned;
  pcl::transformPointCloud(*cur_submap, source_aligned, T_hist_cur.cast<float>());
  const auto& gate = cfg_.inlier_gate;
  const InlierStats st = computeInlierStats(source_aligned, hist_submap, gate.inlier_distance);

  debugLoopLog("align result: converged=" + std::to_string(converged) +
               " inlier_ratio=" + std::to_string(st.ratio) +
               " inlier_rmse=" + std::to_string(st.rmse) +
               " inliers=" + std::to_string(st.inliers) + "/" + std::to_string(st.total) +
               " src_pts=" + std::to_string(cur_submap->size()) +
               " tgt_pts=" + std::to_string(hist_submap->size()));

  if (st.total <= 0 || st.ratio < gate.min_inlier_ratio || st.rmse > gate.max_inlier_rmse) {
    debugLoopLog("loop rejected by inlier gate: ratio=" + std::to_string(st.ratio) +
                 " (min " + std::to_string(gate.min_inlier_ratio) +
                 ") rmse=" + std::to_string(st.rmse) +
                 " (max " + std::to_string(gate.max_inlier_rmse) + ")");
    return false;
  }

  // score 语义"越小越好"：用内点 RMSE（米）
  *score = std::max(st.rmse, 1e-6);

  const auto& pre = kfs[history_idx];
  const auto& cur = kfs[current_idx];
  const Mat3d r_pre = pre.pose_map.orientation.normalized().toRotationMatrix();
  const Mat3d r_cur = cur.pose_map.orientation.normalized().toRotationMatrix();
  *diff_rot = r_pre.transpose() * T_hist_cur.block<3, 3>(0, 0) * r_cur;
  *diff_pos =
      r_pre.transpose() *
      (T_hist_cur.block<3, 3>(0, 0) * cur.pose_map.position + T_hist_cur.block<3, 1>(0, 3) -
       pre.pose_map.position);
  return diff_rot->array().isFinite().all() && diff_pos->array().isFinite().all();
}
```

删除内容：原 `getFitnessScore()` 门控、原 20m 判内点的统计段（由 `computeInlierStats` 取代）、被注释的 `init_guess`。`diff_rot/diff_pos` 公式原样保留。

---

## 6. `config/slam_mapping_v2.yaml`

在 `loop_closure:` 段内、`odom_pos_noise` 之后插入三段，并调整 `icp_fitness_thresh` 语义：

```yaml
  # 里程计因子旋转噪声（弧度）
  odom_rot_noise: 1e-6
  # 里程计因子平移噪声（米）
  odom_pos_noise: 1e-4

  # ===== 配准器（回环验证） =====
  registration:
    # "vgicp"  : small_gicp 两级 VGICP（粗体素 + 细体素，推荐，抗部分重叠）
    # "gicp"   : small_gicp 两级 GICP（粗层降采样点云 + 细层原始分辨率，KdTree）
    # "pcl_icp": 原 PCL 点到面 ICP（对照）
    type: "vgicp"
    # 细层对应点最大距离（米）；原 PCL 路径硬编码 20m，过大易引入假对应
    max_correspondence_distance: 1.5
    max_iterations: 30
    transformation_epsilon: 1e-6
    num_threads: 4
    # GICP 协方差估计邻域点数
    cov_num_neighbors: 20
    # 基础体素/降采样边长（米）：
    #   vgicp 粗层体素 = voxel_resolution*coarse_scale，细层 = voxel_resolution*fine_scale
    #   gicp  粗层 leaf = voxel_resolution*coarse_scale（细层为原始分辨率）
    voxel_resolution: 0.5
    coarse_scale: 4.0
    fine_scale: 1.0
    # 鲁棒核抑制室外植被等假对应："none" | "huber" | "cauchy"
    robust_kernel: "huber"
    robust_kernel_scale: 1.0

  # ===== 内点门控（取代全局 fitness 均值判据） =====
  inlier_gate:
    # 内点判定距离（米）：配准后最近邻距离小于该值计为内点
    inlier_distance: 0.5
    # 内点率下限
    min_inlier_ratio: 0.4
    # 内点 RMSE 上限（米）
    max_inlier_rmse: 0.3

  # ===== 打分/配准前区域裁剪（候选历史关键帧坐标系下的盒式范围，米） =====
  crop:
    enable: true
    forward: 40.0
    backward: 40.0
    left: 30.0
    right: 30.0
    up: 10.0
    down: 5.0

  # ICP 配准 fitness score 阈值：现已不参与门控，仅作为回环因子噪声的上限夹取值
  # （pose_graph_optimizer.buildLoopNoiseModel）。score 现为内点 RMSE（米制），
  # 请将此值设为与 inlier_gate/max_inlier_rmse 一致。
  icp_fitness_thresh: 0.3
  # 噪声下限夹取（米制）
  loop_score_floor: 0.05
```

---

## 7. GICP / VGICP 两级结构说明

| | VGICP（`type: "vgicp"`） | GICP（`type: "gicp"`） |
|---|---|---|
| 目标表示 | 高斯体素图 `GaussianVoxelMap` | 原始点 + `KdTree` |
| 粗层 | 体素 `voxel_resolution*coarse_scale`（默认 2.0m） | 体素降采样 leaf 同值（`voxelDownsample`） |
| 粗层对应距离 | `max_correspondence_distance*coarse_scale`（默认 6m） | 同左 |
| 细层 | 体素 `voxel_resolution*fine_scale`（默认 0.5m） | 原始分辨率点云 |
| 细层对应距离 | `max_correspondence_distance`（默认 1.5m） | 同左 |
| 协方差 | 源/目标均 `estimate_covariances_omp` | 同左（粗层降采样后重新估计） |
| 单层模式 | 注释 `alignTwoStageVgicp` 中【第一层】，细层 init 改回 `init` | 注释 `alignTwoStageGicpTree` 中【第一层】，细层 init 改回 `init` |

两者残差同为 `GICPFactor`（分布对分布马氏距离），鲁棒核、门控、裁剪行为一致；差别只在目标表示与粗层组织方式。室外稀疏/植被区 VGICP 的体素聚合协方差更稳；GICP 调参更少（无 `fine_scale` 依赖）、实现更直观。

---

## 8. 注意事项

1. **`pose_graph_optimizer.cpp` 不用改，但参数量纲变了**。
   `buildLoopNoiseModel`（`pose_graph_optimizer.cpp:18`）把 `score` 夹到
   `[loop_score_floor, icp_fitness_thresh]` 再映射回环因子噪声。旧 score 是 PCL
   `getFitnessScore()`（平方距离 m²），新 score 是内点 RMSE（米）。因此 yaml 里
   `icp_fitness_thresh` 必须从 0.1 改为 0.3（= `max_inlier_rmse`），否则噪声恒饱和。
   `addLoopFactor` 的 `score > 0` 检查不受影响（`std::max(rmse, 1e-6)` 保证）。

2. **裁剪框坐标系**：定义在候选历史关键帧的 `pose_map` 系（+x = 历史帧朝向）。
   反向回访场景 `forward/backward` 需覆盖走廊重叠段（默认 40m 对称），
   `left/right` 30m 覆盖路宽两侧。裁剪同时作用于 current 与 history 两侧，
   保证比较的是覆盖交集。裁剪后子图为空时函数直接返回 false。

3. **初值**：两侧子图都在 map 系，漂移为零时 `T=I`，故用单位阵。
   **不要**用 `T_hist⁻¹*T_cur` 作初值——那适合 body 系点云，对 map 系子图是重复施加。
   漂移靠粗配准层吸收（VGICP 粗体素 / GICP 降采样 + 放大对应距离）。
   若实测漂移大，把 `coarse_scale` 提到 8~10。

4. **单层/双层切换**：两个 `alignTwoStage*` 函数末尾都有注释说明——
   注释掉【第一层】整段，并把第二层 `align(...)` 的第 4 个初值参数由
   `coarse_res.T_target_source` 改回 `init`，即退化为单层精配准。

5. **A/B 对照**：`registration/type` 可切 `vgicp` / `gicp` / `pcl_icp`，
   新门控与裁剪对三条路径同等生效，可单变量对比配准器效果。
   注意 relocalization 包里的 `gicp` 是 **PCL** GICP，与本文的 small_gicp `gicp`
   不是同一实现，参数不可直接互抄（`getFitnessScore` 与 small_gicp `error` 量纲也不同）。

6. **`latest_current_submap_` / `latest_history_submap_` 现保存裁剪后的子图**，
   `loop_results/*.pcd` 反映实际参与比较的区域，便于调试。

7. **调参起点（反向回访场景）**：
   - `min_inlier_ratio: 0.4` 起步偏宽松，先看日志中成功/失败案例的 ratio 分布再收紧到 0.5~0.6；
   - `inlier_distance: 0.5` 与 `max_inlier_rmse: 0.3` 配套；室外植被多可放宽到 0.7 / 0.4；
   - 日志关键字：`align result:`（含 converged / inlier_ratio / inlier_rmse / inliers / 点数）
     与 `loop rejected by inlier gate:`。

---

## 9. 验证

1. 编译：
   ```bash
   cd /home/hhy/2004ros/DeepLumin_zq
   catkin_make --pkg slam_mapping_v2
   ```
2. 开回环建图：`config/slam_mapping_v2.yaml` 中 `loop_closure/enable: true`，
   `loop_closure/detector_type: "laser"`，`runtime/auto_start: true`（或走服务启动）。
3. 回放含反向回访路段的 bag，观察 `loclog` 中 `loop_debug`：
   - `radiusSearch: ... candidates=N` → 候选是否检索到（N>0 说明 xyz 邻近没问题）；
   - `align result: inlier_ratio=... inlier_rmse=...` → 验证阶段质量；
   - `loop detected history->current score=...` → 成功样本。
4. 对照实验：`registration/type` 分别取 `vgicp` / `gicp` / `pcl_icp`，
   各跑同一 bag，统计成功回环数与 `inlier_ratio` 分布。
5. 回环效果检查：保存产物 `loop_results/loop_pairs.csv`、
   `latest_loop_current_submap.pcd` / `latest_loop_history_submap.pcd`，
   在 CloudCompare 中观察反向场景下两侧子图重叠形态与配准收敛区。
