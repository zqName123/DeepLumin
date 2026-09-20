# BIEVR 凹凸图融入 VoxelMap 的实现分析

> 分析日期：2026-09-18
> 参考项目：BIEVR-LIO ([bievr_map.h](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/bievr_map.h), [ls_optimizer.h](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/ls_optimizer.h), [preprocess.cpp](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/preprocess.cpp))
> 目标项目：FAST-LIO2 VoxelMap ([voxel_map_types.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/include/lio_odometry/core/voxelmap/voxel_map_types.h), [voxel_map_fastlivo.cpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp), [laser_odometry_core.cpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/laser_odometry_core.cpp))

---

## 一、BIEVR 机制的移植可行性总览

BIEVR-LIO 与你的 FAST-LIO2 VoxelMap 在底层架构上有本质差异，不能直接"复制粘贴"。但 BIEVR 的核心设计——**体素内局部凹凸图 + 点对图像残差 + 地图引导采样**——可以抽象为三个独立模块，分别融入你的框架。

| 模块 | BIEVR 实现方式 | 你的 VoxelMap 现有结构 | 移植可行性 |
|------|---------------|----------------------|-----------|
| 凹凸图存储 | `Voxel.bump_img_` (MatrixXf) | `VoxelPlane` (平面参数) | ✅ 可扩展 |
| 凹凸图更新 | PCA+重投影+加权滑动平均 | `check_and_update` 递推更新 | ✅ 可新增独立函数 |
| 点对图配准残差 | `r = p_z - I(u,v)` + 图像梯度雅可比 | `r = n·p + d` + 法向量雅可比 | ✅ 可新增方案 |
| 地图引导采样 | `mean_img_dist_` 评分 | PCL VoxelGrid 均匀采样 | ✅ 可新增策略 |
| 优化器 | LM | IEKF | ⚠️ 不动优化器，只改残差 |

**核心原则**：IKF 后端**不动**，只改造体素数据结构 + 残差计算 + 点采样。

---

## 二、数据结构扩展方案

### 2.1 现有 VoxelPlane 结构

参考 [voxel_map_types.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/include/lio_odometry/core/voxelmap/voxel_map_types.h)，现有结构：

```cpp
struct VoxelPlane {
  V3D center_;           // 体素质心
  V4D plane_;            // 平面参数 [nx, ny, nz, d]
  M3D cov_;              // 协方差矩阵
  V3D eigenvalues_;      // 特征值
  double plane_var_;     // 平面方差
  int points_size_;      // 点数
  double sum_ppt_;       // 累加点数的倒数
  bool is_plane_;        // 是否有效平面
  bool is_point_;        // 是否是点特征
  int last_update_time_; // 最后更新时间戳
};
```

### 2.2 建议的扩展方案

**向 `VoxelPlane` 添加凹凸图成员**（条件编译，通过配置开关）：

```cpp
struct VoxelPlane {
  // === 原有成员保持不变 ===
  V3D center_;
  V4D plane_;
  M3D cov_;
  V3D eigenvalues_;
  double plane_var_;
  int points_size_;
  double sum_ppt_;
  bool is_plane_;
  bool is_point_;
  int last_update_time_;

  // === 新增凹凸图成员 ===
  // 当 enable_bump_ = true 时使用，false 时完全退化为原版行为
  static constexpr int kDefaultBumpSize = 16;   // 16×16 固定分辨率
  
  // 体素局部坐标系（与 BIEVR 的 T_O_W_ 类似）
  Eigen::Matrix3d R_O_W_;           // 世界→体素局部正交基旋转
  V3D t_O_W_;                       // 平移（质心）
  // 注意：R_O_W_ 的行：
  //   row(0) = x_normal_ = 平面内最大方差方向
  //   row(1) = y_normal_ = 平面内次大方差方向
  //   row(2) = normal_   = 最小方差方向（法向量）
  
  // 凹凸图
  float bump_img_[kDefaultBumpSize][kDefaultBumpSize];        // 高度偏移值
  float bump_weights_[kDefaultBumpSize][kDefaultBumpSize];    // 累加权重
  uint8_t bump_valid_[kDefaultBumpSize][kDefaultBumpSize];    // 有效掩码 (0/1)
  
  // 预计算梯度缓存（可选，降低实时计算）
  float bump_grad_u_[kDefaultBumpSize][kDefaultBumpSize];     // ∂bump/∂u
  float bump_grad_v_[kDefaultBumpSize][kDefaultBumpSize];     // ∂bump/∂v
  
  // 凹凸图统计
  double bump_mean_dist_{0.0};     // 平均 |bump| 值（用于采样评分）
  int bump_observed_pixels_{0};     // 有效像素数
  bool bump_image_dirty_{true};     // 是否需要重新计算梯度缓存
  int bump_last_update_frame_{-1};  // 最后一帧更新时间
};
```

### 2.3 体素局部坐标系的维护

BIEVR 在更新法向量时重新计算局部坐标系。在你的 VoxelMap 中，`check_and_update()` 已经递推更新了平面参数。需要新增逻辑：

```cpp
// 在 check_and_update() 中，当平面参数更新后：
void updateVoxelFrame(VoxelPlane& plane) {
  // plane.plane_ 存储 [nx, ny, nz, d]
  V3D normal = plane.plane_.head<3>();
  V3D x_axis = normal.unitOrthogonal();   // 任意正交方向
  V3D y_axis = normal.cross(x_axis).normalized();
  
  plane.R_O_W_.row(0) = x_axis;
  plane.R_O_W_.row(1) = y_axis;
  plane.R_O_W_.row(2) = normal;
  plane.t_O_W_ = plane.center_;
  
  // 法向量变化 → 凹凸图需要重投影
  plane.bump_image_dirty_ = true;
}
```

### 2.4 凹凸图像素尺寸与范围

BIEVR 使用动态尺寸（根据体素角点投影范围确定），但你的场景中固定 16×16 更合适：

```cpp
// 配置参数（voxel_map_config.h）
struct BumpConfig {
  bool enable = false;       // 总开关
  int image_size = 16;       // 图像尺寸
  double pixel_size = 0.05;  // 像素对应物理尺寸 (m)
  double update_threshold = 5;   // 累积多少新点触发一次更新
  double smooth_sigma = 0.8;     // 高斯平滑 sigma
  double normal_change_tol_deg = 5.0;  // 法向量变化阈值
  bool use_grad_cache = true;   // 是否预计算梯度缓存
  bool lazy_update = true;      // 懒更新
};
```

16×16 凹凸图覆盖的物理范围 = `16 × 0.05m = 0.8m`，略大于体素尺寸（0.5m），能完整覆盖体素。

---

## 三、凹凸图更新流程

### 3.1 更新主函数

参考 BIEVR 的 [bievr_map.cpp:L242-L275](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L242-L275)，但在你的框架内实现：

```cpp
void updateVoxelBumpImage(VoxelPlane& plane, 
                          const std::vector<V3D>& new_points,
                          int current_frame_id) {
  if (!plane.bump_image_enabled) return;
  
  // 1. 检查是否需要更新（懒更新）
  if (plane.points_size_ - plane.last_update_size_ < kUpdateThreshold)
    return;
  plane.last_update_size_ = plane.points_size_;
  
  // 2. 如果法向量发生了变化，做重投影
  if (plane.bump_image_dirty_) {
    reprojectBumpImage(plane);
    plane.bump_image_dirty_ = false;
  }
  
  // 3. 将新点投影到凹凸图
  for (const auto& p : new_points) {
    // 变换到局部坐标系
    V3D p_local = plane.R_O_W_ * (p - plane.t_O_W_);
    
    // 计算像素坐标
    float u = p_local.x() / kPixelSize + kImageSize / 2.0f;
    float v = p_local.y() / kPixelSize + kImageSize / 2.0f;
    int ui = static_cast<int>(std::round(u));
    int vi = static_cast<int>(std::round(v));
    
    if (ui < 0 || ui >= kImageSize || vi < 0 || vi >= kImageSize)
      continue;
    
    // 加权滑动平均（参考 BIEVR 的加权更新）
    float weight_new = 1.0f / (1.0f + p_local.norm() * 0.1f); // 距离衰减
    float old_val = plane.bump_img_[vi][ui];
    float old_w = plane.bump_weights_[vi][ui];
    plane.bump_weights_[vi][ui] += weight_new;
    plane.bump_img_[vi][ui] = (old_val * old_w + p_local.z() * weight_new) 
                              / plane.bump_weights_[vi][ui];
    plane.bump_valid_[vi][ui] = 1;
  }
  
  // 4. 高斯平滑（可选，参考 BIEVR 的 maskedGaussianSmooth）
  if (kEnableSmoothing) {
    gaussianSmoothBump(plane);
  }
  
  // 5. 更新统计量
  computeBumpScore(plane);
  
  // 6. 预计算梯度缓存
  if (kUseGradCache) {
    computeBumpGradients(plane);
  }
}
```

### 3.2 重投影逻辑

当体素法向量变化超过阈值时，参考 BIEVR 的 [reprojectImage](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L316-L353)：

```cpp
void reprojectBumpImage(VoxelPlane& plane) {
  float old_img[16][16], old_weights[16][16];
  memcpy(old_img, plane.bump_img_, sizeof(old_img));
  memcpy(old_weights, plane.bump_weights_, sizeof(old_weights));
  
  // 清空当前图像
  memset(plane.bump_img_, 0, sizeof(plane.bump_img_));
  memset(plane.bump_weights_, 0, sizeof(plane.bump_weights_));
  memset(plane.bump_valid_, 0, sizeof(plane.bump_valid_));
  
  // 对旧图像的每个有效像素，提升为3D点，变换到新坐标系，重新投影
  for (int v = 0; v < 16; ++v) {
    for (int u = 0; u < 16; ++u) {
      if (old_weights[v][u] <= 0) continue;
      
      // 旧局部坐标系下的点（假设 bump 高度在 Z 方向）
      V3D p_old_center = plane.R_O_W_old_.transpose() * V3D(
        (u - 8) * kPixelSize, 
        (v - 8) * kPixelSize, 
        old_img[v][u]);
      V3D p_w = p_old_center + plane.t_O_W_;  // 恢复到世界坐标
      
      // 用新坐标系重新投影
      V3D p_new_local = plane.R_O_W_ * (p_w - plane.t_O_W_);
      int new_u = static_cast<int>(std::round(p_new_local.x() / kPixelSize + 8));
      int new_v = static_cast<int>(std::round(p_new_local.y() / kPixelSize + 8));
      
      if (new_u >= 0 && new_u < 16 && new_v >= 0 && new_v < 16) {
        plane.bump_img_[new_v][new_u] = p_new_local.z();
        plane.bump_weights_[new_v][new_u] = old_weights[v][u];
        plane.bump_valid_[new_v][new_u] = 1;
      }
    }
  }
}
```

**简化建议**：对于你的第一版实现，法向量变化时可以**直接清空凹凸图**，从累积点重新构建。这样代码简单很多，但会丢失历史信息。BIEVR 的重投影机制更精确，但实现更复杂。

---

## 四、复合残差设计与雅可比推导（完整版）

### 4.1 与 BIEVR 残差的对照

BIEVR 使用纯粹的**点对凹凸图残差**：

\[
r_{\text{BIEVR}} = p_o^z - I(u, v)
\]

其中 \(p_o = T_{C/W} \cdot p_W\)，完全依赖凹凸图。

你的方案使用**复合残差**（更保守，退化到平面时仍有约束）：

\[
r_{\text{复合}} = d_{\text{plane}} + \text{bump}(u, v)
\]

其中 \(d_{\text{plane}} = \mathbf{n}^T \mathbf{p}_W + d\)。

这个方案的好处已经在先前讨论过。下面给出完整的雅可比推导。

### 4.2 完整雅可比推导

**定义**：

| 符号 | 含义 |
|------|------|
| \(\mathbf{p}_W\) | 世界坐标系下的点 |
| \(\mathbf{p}_I\) | IMU 坐标系下的点（原始测量） |
| \(\mathbf{R}_{WL}, \mathbf{t}_{WL}\) | LiDAR → 世界变换（待优化状态的一部分） |
| \(\mathbf{R}_{LI}\) | IMU → LiDAR 外参（已知固定） |
| \(\mathbf{n}, d\) | 平面参数（法向量 + 距离） |
| \(\mathbf{x}_n, \mathbf{y}_n\) | 平面局部基（固定于体素） |
| \(\mathbf{c}\) | 体素质心（固定于体素） |

**首先定义 \((u,v)\) 的计算**：

\[
\mathbf{p}_W = \mathbf{R}_{WL}(\mathbf{R}_{LI} \mathbf{p}_I + \mathbf{t}_{LI}) + \mathbf{t}_{WL}
\]

\[
u = \mathbf{x}_n^T (\mathbf{p}_W - \mathbf{c}), \quad 
v = \mathbf{y}_n^T (\mathbf{p}_W - \mathbf{c})
\]

**复合残差**：

\[
r = \underbrace{\mathbf{n}^T \mathbf{p}_W + d}_{d_{\text{plane}}} + \underbrace{\text{bump}(u, v)}_{\text{修正项}}
\]

**对平移 \(\mathbf{t}_{WL}\) 的雅可比**：

\[
\frac{\partial r}{\partial \mathbf{t}_{WL}} = 
\mathbf{n}^T \cdot \frac{\partial \mathbf{p}_W}{\partial \mathbf{t}_{WL}} + 
\frac{\partial \text{bump}}{\partial u} \cdot \frac{\partial u}{\partial \mathbf{p}_W} \cdot \frac{\partial \mathbf{p}_W}{\partial \mathbf{t}_{WL}} + 
\frac{\partial \text{bump}}{\partial v} \cdot \frac{\partial v}{\partial \mathbf{p}_W} \cdot \frac{\partial \mathbf{p}_W}{\partial \mathbf{t}_{WL}}
\]

其中：

\[
\frac{\partial \mathbf{p}_W}{\partial \mathbf{t}_{WL}} = \mathbf{I}_{3\times 3}, \quad
\frac{\partial u}{\partial \mathbf{p}_W} = \mathbf{x}_n^T, \quad
\frac{\partial v}{\partial \mathbf{p}_W} = \mathbf{y}_n^T
\]

因此：

\[
\boxed{\frac{\partial r}{\partial \mathbf{t}_{WL}} = 
\mathbf{n}^T + \frac{\partial \text{bump}}{\partial u} \cdot \mathbf{x}_n^T + 
\frac{\partial \text{bump}}{\partial v} \cdot \mathbf{y}_n^T}
\]

**对旋转 \(\mathbf{R}_{WL}\)（扰动 \(\delta \boldsymbol{\theta}\)）的雅可比**：

使用左扰动模型，\(\mathbf{R}_{WL} \leftarrow \mathbf{R}_{WL} \exp([\delta\boldsymbol{\theta}]_\times)\)：

\[
\frac{\partial \mathbf{p}_W}{\partial \delta \boldsymbol{\theta}} = 
-\mathbf{R}_{WL} [\mathbf{p}_L]_\times
\]

其中 \(\mathbf{p}_L = \mathbf{R}_{LI} \mathbf{p}_I + \mathbf{t}_{LI}\) 是 LiDAR 坐标系下的点。

\[
\boxed{\frac{\partial r}{\partial \delta \boldsymbol{\theta}} = 
\left( \mathbf{n}^T + \frac{\partial \text{bump}}{\partial u} \cdot \mathbf{x}_n^T + 
\frac{\partial \text{bump}}{\partial v} \cdot \mathbf{y}_n^T \right) \cdot 
\left( -\mathbf{R}_{WL} [\mathbf{p}_L]_\times \right)}
\]

**对速度、偏差的雅可比**：与原生 FAST-LIO2 完全相同，bump 项不引入额外依赖。

### 4.3 与现有代码的融合

在你的 [laser_odometry_core.cpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/laser_odometry_core.cpp) 中 `ObsModel()` 函数中，现有雅可比计算：

```cpp
// 现有代码：平面残差雅可比
obs.h_x_.block<1, 3>(idx, 0) = norm_vec.transpose();  // 平移
obs.h_x_.block<1, 3>(idx, 3) = point_imu.cross(R_wl.transpose() * norm_vec).transpose();  // 旋转
```

扩展为包含 bump 项：

```cpp
// 新增：bump 项雅可比修正
if (plane.bump_enabled && plane.bump_observed_pixels_ > 0) {
  // 1. 计算 (u,v) 坐标
  V3D p_w = R_wl * point_body + t_wl;
  V3D p_local = plane.R_O_W_ * (p_w - plane.t_O_W_);
  float u = p_local.x() / kPixelSize + kImageSize / 2.0f;
  float v = p_local.y() / kPixelSize + kImageSize / 2.0f;
  
  // 2. 双线性插值获取 bump 值 + 梯度
  float bump_val, dbump_du, dbump_dv;
  if (sampleBumpWithGradient(plane, u, v, bump_val, dbump_du, dbump_dv)) {
    // 3. 复合残差 = 平面距离 + bump
    obs.h_(idx) = plane_dist + bump_val;  // 而不是原来的 plane_dist
    
    // 4. bump 项对平移的雅可比
    V3D dbump_dt = dbump_du * plane.R_O_W_.row(0) + dbump_dv * plane.R_O_W_.row(1);
    obs.h_x_.block<1, 3>(idx, 0) += dbump_dt.transpose();  // 原有 n^T 基础上增加
    
    // 5. bump 项对旋转的雅可比
    V3D dbump_dr = dbump_dt.transpose() * (-R_wl * skew(point_body));
    obs.h_x_.block<1, 3>(idx, 3) += dbump_dr.transpose();
  }
}
```

### 4.4 自适应 R 的扩展

在现有的自适应 R 计算中增加凹凸图像素置信度项：

```cpp
// 现有自适应 R
double adaptive_R = r_base_ + sigma_l + sigma_p;

// 扩展：增加 bump 置信度
if (plane.bump_enabled) {
  double bump_confidence = 1.0;
  if (plane.bump_observed_pixels_ > 0) {
    // 稀疏像素 → 低置信度 → 增大 R
    double fill_ratio = plane.bump_observed_pixels_ / 256.0;  // 16×16=256
    bump_confidence = 0.5 + 0.5 * fill_ratio;
  }
  adaptive_R /= bump_confidence;  // 低置信度 → 增大 R → 降低权重
}
```

---

## 五、采样策略扩展

### 5.1 VoxelMap 的现状

你的代码中使用 PCL 的 `pcl::VoxelGrid` 进行均匀降采样：

```cpp
pcl::VoxelGrid<PointType> voxel_scan_;
// 配置：leaf_size = 0.1m 或 0.2m
```

### 5.2 地图引导采样实现

参考 BIEVR 的 [sampleInformed](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/preprocess.cpp#L83-L168)，在 VoxelMap 框架下的实现：

```cpp
void sampleInformedByVoxelMap(
    const PointCloud::Ptr& input_cloud,
    PointCloud::Ptr& output_cloud,
    const VoxelMapManagerFASTLIVO& voxel_map,
    const SE3& T_wl,
    int total_samples = 3000) 
{
  struct VoxelScore {
    size_t hash;
    double score;
    PointCloud::Ptr points;  // 该体素内的所有点
  };
  
  // 1. 对每个点，查找其在 VoxelMap 中的归属体素
  std::unordered_map<size_t, VoxelScore> voxel_scores;
  for (const auto& pt : *input_cloud) {
    V3D p_w = T_wl * V3D(pt.x, pt.y, pt.z);
    Vec3i idx = floor(p_w / voxel_map_resolution_);
    size_t hash = hashIndex(idx);
    
    auto& vs = voxel_scores[hash];
    if (vs.points == nullptr) {
      vs.points = PointCloud::Ptr(new PointCloud);
      vs.hash = hash;
      
      // 查询体素的 bump 统计评分
      auto* plane = voxel_map.getVoxel(idx);
      vs.score = (plane && plane->bump_enabled) 
                 ? plane->bump_mean_dist_ 
                 : 0.0;
    }
    vs.points->push_back(pt);
  }
  
  // 2. 按评分降序排列体素
  std::vector<VoxelScore*> sorted;
  for (auto& [_, vs] : voxel_scores) sorted.push_back(&vs);
  std::sort(sorted.begin(), sorted.end(), 
            [](auto a, auto b) { return a->score > b->score; });
  
  // 3. 高评分体素 → 保留所有点（fine 点）
  //    低评分体素 → 每个体素只保留一个点（coarse 点）
  int n_high = std::min(static_cast<int>(sorted.size()) / 3, 500);
  output_cloud->clear();
  
  for (int i = 0; i < sorted.size(); ++i) {
    if (i < n_high && sorted[i]->score > 0.01) {
      // 高评分体素：保留所有点
      *output_cloud += *(sorted[i]->points);
    } else {
      // 低评分体素：只取质心最近的点
      *output_cloud += sorted[i]->points->at(0);
    }
  }
  
  // 4. 控制总点数 ≤ total_samples
  if (output_cloud->size() > total_samples) {
    // 随机降采样到 total_samples
    pcl::RandomSample<PointType> sampler;
    sampler.setSample(total_samples);
    sampler.setInputCloud(output_cloud);
    sampler.filter(*output_cloud);
  }
}
```

### 5.3 与现有降采样链的融合

推荐在现有降采样链中新增一个步骤：

```
原始点云
  ↓ VoxelGrid 降采样 (0.2m)  — 保持现有逻辑
降采样后的点云
  ↓ Map-Informed 采样 (新增)  — 按凹凸梯度重新筛选
最终配准点云 (约 3000 点)
  ↓
送入 IKF 配准
```

这样原有逻辑不变，新增模块是可插拔的。

---

## 六、内存与计算开销评估

### 6.1 内存开销

每个体素新增凹凸图成员的内存：

| 成员 | 类型 | 大小 |
|------|------|------|
| `R_O_W_` | `Matrix3d` | 24 字节 |
| `t_O_W_` | `V3D` | 24 字节 |
| `bump_img_[16][16]` | `float[256]` | 1,024 字节 |
| `bump_weights_[16][16]` | `float[256]` | 1,024 字节 |
| `bump_valid_[16][16]` | `uint8_t[256]` | 256 字节 |
| `bump_grad_u_[16][16]` | `float[256]` | 1,024 字节 |
| `bump_grad_v_[16][16]` | `float[256]` | 1,024 字节 |
| 其他标量 | | ~32 字节 |
| **单个体素新增总计** | | **~3.4KB（有梯度缓存）/ ~2.3KB（无）** |

对比现有 VoxelPlane 大小（约 200 字节），增加了约 10-15 倍。

**但 LRU 缓存限制了体素总数**（你的配置 `lru_capacity: 3000`）：
- 无梯度缓存：3000 × 2.3KB ≈ **6.9MB**
- 有梯度缓存：3000 × 3.4KB ≈ **10.2MB**

在 2026 年的硬件上完全可接受。

### 6.2 计算开销

| 操作 | 每帧调用次数 | 单次耗时 | 总耗时 |
|------|------------|---------|-------|
| 投影点到凹凸图 | ~3000 点 | ~0.05μs (FP) | 0.15ms |
| 双线性插值 + 梯度 | ~3000 点 | ~0.1μs | 0.3ms |
| 雅可比修正 | ~3000 点 | ~0.05μs | 0.15ms |
| 凹凸图更新 | ~500 新点 | ~0.1μs | 0.05ms |
| 高斯平滑 | 1次 | ~0.05ms | 0.05ms |
| 梯度缓存预计算 | 1次 | ~0.02ms | 0.02ms |
| **合计额外开销** | | | **~0.7ms** |

在你的 `ObsModel` 总耗时约 3-10ms 的基础上，增加约 **7-20%** 的计算开销，实时性良好。

---

## 七、与 VoxelMap 现有机制的关键兼容分析

### 7.1 八叉树细分

你的 VoxelMap 使用八叉树：当体素内点过多时，[reinit_octo_tree](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L488-L501) 将体素分为 8 个子体素，重新拟合平面。

**问题**：子体素的凹凸图如何继承？

**解决方案**：
- 父体素的凹凸图**不传递**给子体素
- 子体素从零开始构建自己的凹凸图
- 原因：子体素的平面法向量可能完全不同于父体素，凹凸图失效

```cpp
// 在 reinit 之后，子体素初始化时：
child_plane.bump_enabled = true;
child_plane.bump_image_dirty_ = true;
// 凹凸图 stay zero，等待新点到来更新
```

### 7.2 滑动窗口裁剪

你的代码中，体素距离机器人过远时会被标记为 `last_update_time_ < threshold` 并在 `MapSliding` 中删除。

**处理方式**：凹凸图随体素一起销毁，不做额外处理。这与 BIEVR 的 LRU 机制本质上相同。

### 7.3 平面特征的体素

你的代码区分 `is_plane_` 和 `is_point_` 两种体素。对于 `is_point_` 体素（无有效平面），凹凸图无法构建，应**自动禁用**：

```cpp
if (!plane.is_plane_) {
  plane.bump_enabled = false;  // 非平面体素无法构建凹凸图
}
```

### 7.4 初始化阶段

地图尚未初始化时，凹凸图为空，采样和配准应自动退化回原有逻辑：

```cpp
if (voxel_map.size() < kMinMapSizeForBump) {
  bump_enabled_global = false;  // 全局禁用
}
```

---

## 八、实现路线

### Phase 1：数据结构扩展（~1 天）

1. 在 [voxel_map_types.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/include/lio_odometry/core/voxelmap/voxel_map_types.h) 的 `VoxelPlane` 中增加凹凸图成员
2. 在 [voxel_map_config.h](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/include/lio_odometry/core/voxelmap/voxel_map_config.h) 增加凹凸图配置参数
3. 新增条件编译宏 `BUMP_MAP_ENABLED`，关闭时编译出原版 VoxelMap

### Phase 2：凹凸图更新（~2 天）

1. 实现 `updateVoxelBumpImage()` 函数（参考 BIEVR 的 `updateBumpImage + integratePoints`）
2. 实现 `reprojectBumpImage()` 函数（参考 BIEVR 的 `reprojectImage`）
3. 实现 `computeBumpScore()` 计算凹凸评分
4. 在 `check_and_update()` 中嵌入凹凸图更新调用
5. 处理八叉树 reinit 时的凹凸图重置

### Phase 3：复合残差与雅可比（~2 天）

1. 实现 `sampleBumpWithGradient()` 双线性插值函数
2. 在 `ObsModel()` 中实现复合残差和修正雅可比
3. 扩展自适应 R 的计算
4. Valley 测试：在空旷场地对比有无凹凸图的轨迹

### Phase 4：地图引导采样（~1 天）

1. 实现 `sampleInformedByVoxelMap()`
2. 集成到现有降采样链中
3. 测试采样分布是否合理

### Phase 5：优化与调试（~2 天）

1. 性能剖析，统计各模块耗时
2. 平坦体素自动禁用凹凸图的分支优化
3. 隧道退化场景实战测试

---

## 九、关键风险 mitigation

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| 凹凸图雅可比推导错误 | 中 | 高 | Phase 3 中在空旷场地对比原版轨迹，验证无偏差 |
| 法向量变化后凹凸图不一致 | 中 | 中 | 法向量变化 >5° 时清空凹凸图重建，不做重投影 |
| 八叉树 reinit 后凹凸图丢失 | 高 | 低 | 预期行为，子体素从头构建即可 |
| 凹凸图过度增加计算延迟 | 低 | 中 | 懒更新 + 平坦体素自动禁用 |
| `is_point_` 体素错误启用凹凸图 | 中 | 中 | 非平面体素自动禁用 |
| 内存超限 | 低 | 中 | LRU 限制体素总数；16×16 固定尺寸保证上界 |
| 稀疏像素插值噪声 | 中 | 中 | 自适应 R 自动压低权重；有效掩码保护 |

---

## 十、总结

BIEVR-LIO 的凹凸图机制可以**分模块、渐进式**地融入 FAST-LIO2 VoxelMap：

1. **数据结构**：VoxelPlane 增加 ~2.3KB/体素的凹凸图，总量约 7-10MB，可接受
2. **更新机制**：参考 BIEVR 的加权滑动平均 + 高斯平滑，嵌套在现有 `check_and_update` 中
3. **残差**：复合残差 \(r = d_{\text{plane}} + \text{bump}(u,v)\)，雅可比在现有基础上叠加图像梯度项
4. **采样**：地图引导采样作为降采样链的第二步，可插拔
5. **优化器**：不动 IEKF，只改残差和雅可比

BIEVR 的纯凹凸图残差不适合直接移植（与 IKF 框架冲突），但**复合残差**方案保留了原生平面约束的同时，通过凹凸项在退化场景提供额外约束，是最优的移植选择。