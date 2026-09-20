# R-VoxelMap 技术分析文档

## 一、项目概述

**R-VoxelMap**（Recursive VoxelMap）是南开大学 MobFly Robotics 实验室提出的改进体素地图方法，发表于 **IEEE RAL 2026**。

**核心创新**：
1. **RANSAC + 递归平面拟合**：用 RANSAC 检测离群点，递归地重新拟合平面
2. **平面有效性检查**：通过投影聚类防止不同物理平面的错误合并
3. **离群点检测与重用管道**：在每次递归迭代中检测并重用离群点

**论文**：[R-VoxelMap: Accurate Voxel Mapping with Recursive Plane Fitting for Online LiDAR Odometry](https://arxiv.org/abs/2601.12377)

**代码基础**：基于 [VoxelMap](https://github.com/hku-mars/VoxelMap) 修改

---

## 二、与原始 VoxelMap 的核心区别

### 2.1 平面拟合策略

| 特性 | 原始 VoxelMap | R-VoxelMap |
|------|-------------|------------|
| **拟合方法** | 直接 PCA（对所有点做特征值分解） | RANSAC + PCA（先去离群点再拟合） |
| **离群点处理** | 无（所有点参与拟合） | RANSAC 检测，离群点不参与拟合 |
| **平面合并检查** | 无（仅靠特征值阈值） | 投影聚类检查（防止错误合并） |
| **递归更新** | 无 | 有（定期重新拟合，修正累积误差） |
| **点管理** | 单一列表 temp_points_ | 三个列表：plane_points_ + not_plane_points_ + new_points_ |

### 2.2 数据结构差异

**原始 VoxelMap**：
```cpp
class VoxelOctoTree {
    vector<pointWithVar> temp_points_;  // 所有点混在一起
    VoxelPlane *plane_ptr_;
    VoxelOctoTree *leaves_[8];
};
```

**R-VoxelMap**：
```cpp
class OctoTree {
    list<pointWithCov> plane_points_;     // 平面内点（RANSAC 内点）
    list<pointWithCov> not_plane_points_; // 平面外点（RANSAC 外点）
    list<pointWithCov> new_points_;       // 新加入的点（待处理）
    Plane *plane_ptr_;
    OctoTree *leaves_[8];
};
```

---

## 三、R-VoxelMap 算法流程

### 3.1 整体流程

```
输入点云
    ↓
插入体素（hash map）
    ↓
对每个体素调用 init_octo_tree()
    ↓
┌─────────────────────────────────────┐
│  init_octo_tree()                    │
│  1. RANSAC 平面拟合                  │
│  2. 平面有效性检查（投影聚类）        │
│  3. 分离内点/外点                    │
│  4. 外点分裂到子体素                 │
└─────────────────────────────────────┘
    ↓
残差构建（build_residual_omp）
    ↓
EKF 更新
    ↓
地图更新（update）
    ↓
LRU 缓存管理
```

### 3.2 RANSAC 平面拟合（核心创新1）

```cpp
// voxel_map_util.hpp - OctoTree::ransac()

bool ransac() {
    // 1. 将链表转为 vector（支持随机访问）
    vector<list<pointWithCov>::iterator> iter_vec;
    for (auto it = new_points_.begin(); it != new_points_.end(); ++it) {
        iter_vec.push_back(it);
    }

    // 2. RANSAC 迭代
    for (int i = 0; i < ransac_max_iter; i++) {
        // 随机采样 sample_num 个点
        vector<Vector3d> sample_points(sample_num);
        for (int j = 0; j < sample_num; j++) {
            sample_points[j] = iter_vec[rand() % N]->point;
        }

        // 用采样点拟合平面（PCA）
        center = mean(sample_points);
        cov = E[pp^T] - center * center^T;
        normal = eig(cov).eigenvector(min_eigenvalue);

        // 计算内点数
        inliers = 0;
        for (每个点) {
            distance = |(point - center) · normal|;
            if (distance < inlier_distance_threshold) {
                inliers++;
            }
        }

        // 更新最佳模型
        if (inliers > max_inliers) {
            max_inliers = inliers;
            best_normal = normal;
            best_center = center;
        }
    }

    // 3. 用最佳模型分离内点和外点
    if (max_inliers / N > isplane_p_threshold) {
        for (每个点) {
            if (是内点) {
                plane_points_.splice(point);  // 移到 plane_points_
            } else {
                not_plane_points_.splice(point);  // 移到 not_plane_points_
            }
        }
        return true;  // 是平面
    } else {
        not_plane_points_.splice(所有点);  // 全部移到 not_plane_points_
        return false;  // 不是平面
    }
}
```

**优势**：
- RANSAC 能有效剔除离群点，避免离群点污染平面拟合
- 即使有动态物体或噪声点，也能找到正确的平面

### 3.3 平面有效性检查（核心创新2）

```cpp
// voxel_map_util.hpp - OctoTree::plane_valid_check()

bool plane_valid_check() {
    // 1. 将平面内点投影到平面坐标系
    //    x_axis = plane.x_normal (平面内的主方向)
    //    y_axis = plane.y_normal (平面内的次方向)
    //    origin = plane.center

    unordered_map<pair<int,int>, vector<iterator>> grid_map;

    for (每个平面内点) {
        // 投影到平面
        x_coord = (point - origin) · x_axis;
        y_coord = (point - origin) · y_axis;

        // 量化到网格
        grid_x = floor(x_coord / resolution);
        grid_y = floor(y_coord / resolution);

        grid_map[{grid_x, grid_y}].push_back(point);
    }

    // 2. 用 DFS 对网格进行聚类
    vector<vector<pair<int,int>>> clusters;
    for (每个网格) {
        if (!visited) {
            cluster = DFS(网格);  // 4-连通域聚类
            clusters.push_back(cluster);
        }
    }

    // 3. 找到最大的聚类
    max_cluster = argmax(clusters, size);

    // 4. 检查最大聚类是否满足阈值
    if (max_cluster.size() / N > isplane_p_threshold) {
        // 最大聚类是主平面，其他聚类的点移到 new_points_（待重新拟合）
        for (非最大聚类的点) {
            new_points_.splice(point);  // 从 plane_points_ 移回 new_points_
        }
        return true;  // 平面有效
    } else {
        // 没有明显的主平面，全部移到 not_plane_points_
        not_plane_points_.splice(所有平面内点);
        return false;  // 平面无效
    }
}
```

**解决的问题**：
- 原始 VoxelMap 中，如果两个不同物理平面恰好在同一个体素内，它们会被错误地合并为一个平面
- R-VoxelMap 通过投影聚类，检测是否存在多个分离的平面，防止错误合并

**示例**：

```
原始 VoxelMap（错误合并）：
  ┌─────────────┐
  │  ────┐      │
  │      │      │  ← 两个不同平面被合并为一个
  │  ────┘      │
  └─────────────┘

R-VoxelMap（正确分离）：
  ┌─────────────┐
  │  ────┐      │
  │      │      │  ← 检测到两个聚类，只保留最大的
  │  ────┘      │
  └─────────────┘
  聚类1: {──┐}  ← 保留
  聚类2: {──┘}  ← 移回 new_points_，等待重新拟合
```

### 3.4 离群点检测与重用管道（核心创新3）

```cpp
// voxel_map_util.hpp - OctoTree::init_plane_ransac()

void init_plane_ransac() {
    // 步骤1：RANSAC 拟合
    ransac();  // 分离 plane_points_ 和 not_plane_points_

    // 步骤2：平面有效性检查
    if (需要检查) {
        plane_valid_check();  // 进一步分离，无效点移到 new_points_
    }

    // 步骤3：重新计算平面参数（排除无效点）
    if (new_points_.size() > 0) {
        // 从 plane_points_ 中减去被 valid_check 移出的点
        center = center * N - sum(被移出的点);
        N -= 被移出的点数;
        covariance = E[pp^T] / N - center * center^T;
        // 重新特征值分解
    }

    // 步骤4：检查是否仍为平面
    if (min_eigenvalue < threshold) {
        // 是平面，计算平面参数和协方差
        is_plane = true;
    } else {
        // 不是平面，所有点移到 not_plane_points_
        not_plane_points_.splice(plane_points_);
        is_plane = false;
    }
}
```

### 3.5 递归更新机制

```cpp
// voxel_map_util.hpp - OctoTree::update_octo_tree()

void update_octo_tree() {
    // 检查是否需要重新初始化
    if (!stop_reinit_ && all_points_num_root_ > reinit_size_vec[reinit_state_]) {
        reinit_octo_tree();  // 收集所有点，删除子树，重新拟合
        reinit_state_++;
        return;
    }

    // 将新点分配到子体素
    for (每个新点) {
        voxel = push_to_subvoxel(pv);  // 找到最佳匹配的子体素
        update_cache.insert(voxel);
    }

    // 更新受影响的子体素
    for (每个受影响的子体素) {
        voxel->update_single_voxel();
    }
}

void reinit_octo_tree() {
    // 收集所有点（plane + not_plane + new）
    collect_points(new_points_, layer_);

    // 删除所有子树
    for (int i = 0; i < 8; i++) {
        delete leaves_[i];
        leaves_[i] = nullptr;
    }

    // 重新初始化
    init_octo_ = false;
    plane_ptr_->is_init = false;
    init_octo_tree();  // 重新 RANSAC + valid check
}
```

**作用**：
- 随着时间推移，平面拟合可能因为累积误差而偏离
- 定期重新拟合可以修正这些误差
- `reinit_size_vec` 控制重新拟合的时机（按累积点数触发）

### 3.6 新点插入策略（check_and_update）

```cpp
// voxel_map_util.hpp - OctoTree::check_and_update()

bool check_and_update(const pointWithCov &pv) {
    // 增量更新平面参数（不重新拟合）
    int N = plane->points_size;
    Vector3d new_mean = (mean * N + pv.point) / (N + 1);
    Matrix3d new_ppt = curr_ppt + pv.point * pv.point^T;
    Matrix3d new_cov = new_ppt / (N + 1) - new_mean * new_mean^T;

    // 检查新点是否破坏平面性
    double min_eigenvalue = eig(new_cov).min();
    if (min_eigenvalue >= threshold) {
        return false;  // 新点会破坏平面，拒绝
    }

    // 接受新点，更新平面参数
    plane->center = new_mean;
    plane->covariance = new_cov;
    plane->points_size = N + 1;
    plane->normal = eigenvector(min_eigenvalue);
    plane->cov_need_update = true;  // 标记需要更新协方差
    return true;
}
```

**优势**：
- 不需要每次都重新拟合整个平面
- 增量更新 O(1) 复杂度
- 如果新点会破坏平面性，自动拒绝

---

## 四、R-VoxelMap vs VoxelMap 详细对比

### 4.1 平面拟合

| 特性 | VoxelMap | R-VoxelMap |
|------|---------|------------|
| 拟合方法 | 直接 PCA | RANSAC + PCA |
| 离群点处理 | 无 | RANSAC 剔除 |
| 平面判断 | 最小特征值 < 阈值 | RANSAC 内点比例 > 阈值 + 特征值检查 |
| 平面合并检查 | 无 | 投影聚类 DFS |
| 递归更新 | 无 | 有（按累积点数触发） |
| 增量更新 | 有（init_plane） | 有（check_and_update） |

### 4.2 点管理

| 特性 | VoxelMap | R-VoxelMap |
|------|---------|------------|
| 点存储 | 单一列表 temp_points_ | 三个列表 plane/not_plane/new |
| 点分类 | 无 | RANSAC 内点/外点 |
| 点重用 | 无 | 外点分裂到子体素 |

### 4.3 内存管理

| 特性 | VoxelMap | R-VoxelMap |
|------|---------|------------|
| 地图滑动 | 按距离滑动 | LRU 缓存淘汰 |
| 点数限制 | max_points_num | max_points_size + max_cov_points_size |
| 更新控制 | update_enable_ | update_enable_ + root_update_enable_ |

### 4.4 配置参数

| 参数 | VoxelMap | R-VoxelMap |
|------|---------|------------|
| 体素大小 | voxel_size | voxel_size |
| 最大层数 | max_layer | max_layer |
| 平面阈值 | planner_threshold | planer_threshold |
| 每层点数 | layer_init_num | layer_point_size |
| RANSAC 最大迭代 | - | ransac_max_iter |
| RANSAC 采样数 | - | ransac_sample_num |
| RANSAC 内点距离 | - | ransac_inlier_distance_threshold |
| RANSAC 平面比例 | - | ransac_isplane_p_threshold |
| 有效检查分辨率 | - | valid_check_resolution |
| 重新拟合阈值 | - | reinit_size_vec |
| LRU 容量 | - | lru_capacity |

---

## 五、残差构建与 EKF 更新

### 5.1 残差构建（与 VoxelMap 基本一致）

```cpp
// voxel_map_util.hpp - VoxelMap::build_residual_omp()

#pragma omp parallel for
for (每个点) {
    // 1. 找到对应体素
    voxel = voxel_map_[voxel_location];

    // 2. 在八叉树中查找最近平面
    build_single_residual(pv, voxel, ...);

    // 3. 如果失败，尝试相邻体素
    if (!success) {
        near_position = 调整位置;
        build_single_residual(pv, near_voxel, ...);
    }
}
```

### 5.2 EKF 更新（与 VoxelMap 一致）

```cpp
// voxelMapping.cpp 主循环

for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {
    // 1. 变换点云到世界系
    transformLidar(state, feats_down_body, world_lidar);

    // 2. 计算点协方差
    cov = R * body_cov * R^T + R * [-p×] * rot_var * [-p×]^T * R^T + t_var;

    // 3. 构建残差
    voxelmap.build_residual_omp(pv_list, ...);

    // 4. 构建 H 矩阵和 R
    for (每个有效点) {
        A = [p×] * R^T * normal;
        Hsub.row(i) << A, normal;

        sigma_l = J_nq * plane_cov * J_nq^T;
        R_inv(i) = 1 / (sigma_l + normal^T * cov * normal);
    }

    // 5. 迭代卡尔曼更新
    K_1 = (H^T * R^{-1} * H + P^{-1})^{-1};
    solution = K_1 * H^T * R^{-1} * z + (x_prop - x_est) - G * (x_prop_sub - x_est_sub);
    state += solution;

    // 6. 收敛判断
    if (rot_change < 0.01deg && t_change < 0.015cm) converged;

    // 7. 协方差更新
    P = (I - G) * P;
}
```

---

## 六、地图更新流程

```cpp
// voxelMapping.cpp 主循环

// EKF 更新后
transformLidar(state, feats_down_body, world_lidar);

// 计算每个点的协方差
for (每个点) {
    cov = R * body_cov * R^T + R * [-p×] * rot_var * [-p×]^T * R^T + t_var;
    pv.cov = cov;
}

// 按协方差排序（不确定性小的点优先）
sort(pv_list.begin(), pv_list.end(), var_contrast);

// 更新地图
voxelmap.update(pv_list);

// LRU 缓存管理
voxelmap.lru_cache_update();
```

### 6.1 LRU 缓存管理

```cpp
// voxel_map_util.hpp - VoxelMap::lru_cache_update()

void lru_cache_update() {
    while (lru_cache.size() > lru_capacity) {
        // 淘汰最久未使用的体素
        auto last = lru_cache.back();
        lru_cache.pop_back();
        loc2lru.erase(last);
        delete feat_map[last];
        feat_map.erase(last);
    }
}
```

**与 VoxelMap 的区别**：
- VoxelMap：按距离滑动（删除远处体素）
- R-VoxelMap：LRU 缓存淘汰（删除最久未访问的体素）

---

## 七、状态定义

```cpp
// common_lib.h - StatesGroup (18维)

struct StatesGroup {
    M3D rot_end;   // [0-2]  SO3 旋转
    V3D pos_end;   // [3-5]  位置
    V3D vel_end;   // [6-8]  速度
    V3D bias_g;    // [9-11] 陀螺仪偏差
    V3D bias_a;    // [12-14] 加速度计偏差
    V3D gravity;   // [15-17] 重力向量
    Matrix<double, 18, 18> cov;
};
```

**与 FAST-LIVO2 的区别**：R-VoxelMap 是 18 维（没有曝光时间），FAST-LIVO2 是 19 维。

---

## 八、关键配置参数

```yaml
mapping:
  # 体素参数
  voxel_size: 0.5
  max_layer: 2
  layer_point_size: [5, 5, 5, 5, 5]
  planer_threshold: 0.0025

  # RANSAC 参数
  ransac_max_iter: 10
  ransac_sample_num: 3
  ransac_inlier_distance_threshold: 0.1
  ransac_isplane_p_threshold: 0.6

  # 平面有效性检查
  valid_check_max_layer: 1
  valid_check_min_points_size: 20
  valid_check_resolution: 10

  # 递归更新
  reinit_size_vec: [100, 300, 1000, 3000, 10000]
  stop_reinit_threshold: 10000

  # 更新控制
  update_size_threshold: 5
  update_inlier_distance_threshold: 0.1
  max_points_size: 50
  max_cov_points_size: 200
  max_root_points_size: 5000

  # LRU 缓存
  lru_cache_capacity: 1000000
```

---

## 九、对 lio_odometry 优化的启示

| R-VoxelMap 特性 | 可借鉴到 lio_odometry |
|----------------|---------------------|
| RANSAC 平面拟合 | 在 iVox 中增加 RANSAC，剔除离群点 |
| 平面有效性检查 | 投影聚类，防止错误合并 |
| 递归更新 | 定期重新拟合，修正累积误差 |
| 增量更新 | check_and_update，O(1) 复杂度 |
| LRU 缓存 | 替代当前的 map_trim_radius |

**优先级**：
1. **RANSAC 平面拟合**：最直接的改进，减少离群点影响
2. **平面有效性检查**：防止隧道/走廊场景的错误合并
3. **递归更新**：长时间运行的鲁棒性提升

---

## 十、总结

R-VoxelMap 的核心创新点：

1. **RANSAC + 递归拟合**：先用 RANSAC 剔除离群点，再用 PCA 拟合平面。如果后续发现拟合不好，可以递归重新拟合。

2. **平面有效性检查**：通过投影聚类检测是否存在多个分离的物理平面，防止错误合并。

3. **离群点重用**：RANSAC 的外点不是直接丢弃，而是分裂到子体素中继续使用。

4. **LRU 缓存管理**：用 LRU 策略管理地图内存，比固定距离滑动更灵活。

这些改进使得 R-VoxelMap 在长走廊、隧道等退化场景下表现更好，平面拟合更准确，地图更干净。
