# 瓶颈 4 & 5 专项优化方案

> 基于 R-VoxelMap 设计思想，针对当前 `voxel_map_fastlivo` 中两个明确可立即优化的瓶颈

---

## 瓶颈 4：`UpdateMap` 每帧遍历全部体素

### 4.1 问题描述

当前 `UpdateMap` 在插入新点后执行全表遍历：

```cpp
// voxel_map_fastlivo.cpp:L559
void VoxelMapManagerFASTLIVO::UpdateMap(const std::vector<pointWithVar> &input_points) {
    // ... 先将点插入体素 ...

    // 问题：遍历整个 voxel_map_ 的全部体素
    for (auto &pair : voxel_map_) {           // ← O(N_voxels)
        if (pair.second->new_points_.size() > 0) {
            pair.second->update_octo_tree();  // 只对有点的体素真正做事
        }
    }
    // LRU 淘汰
    LRUEvict();
}
```

**问题本质**：
- 即使当前帧只有 50~80 个点落入 30~50 个体素，依然遍历全部 LRU 体素（如 500 个）
- 每帧遍历 `voxel_map_` 的时间 ≈ O(500) 纯遍历开销，虽不重但累积
- 更严重的是：**当遍历体素以检查是否有 `new_points_` 时，该体素的点还没用完就被检查**
- 遍历全表与 `update_octo_tree()` 之间的逻辑混杂，不利于缓存局部性

### 4.2 R-VoxelMap 的解决方案：`update_cache`

R-VoxelMap 在 [VoxelMap::update()](file:///home/hhy/2004ros/R-VoxelMap/include/voxel_map_util.hpp#L1330-L1380) 中引入了 `update_cache`：

```cpp
void update(const std::vector<pointWithCov> &input_points) {
    unordered_map<VOXEL_LOC, OctoTree *> update_cache;  // ← 只存有更新的体素

    for (uint i = 0; i < plsize; i++) {
        const pointWithCov p_v = input_points[i];
        // ... 计算 position ...

        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            iter->second->new_points_.push_back(p_v);
            iter->second->all_points_num_root_ ++;
        } else {
            // 创建新体素 ...
        }
        update_cache[position] = feat_map[position];  // ← 累积到 cache
    }

    // 只遍历 update_cache（该帧点真正触达的体素）
    for (auto& pair: update_cache) {                  // ← O(N_touched) ≈ 30~80
        pair.second->update_octo_tree();
        // LRU 更新 ...
    }
}
```

**核心思想**：维护一个只包含当前帧有新增点的体素的 `update_cache`，只遍历它而不遍历 `feat_map` 全表。

### 4.3 实施细节

#### 当前代码中已有部分支持

实际上在当前 `voxel_map_fastlivo.cpp` 的 `update_octo_tree()` 内部（[L189-198](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L189-L198)），已经有一个 `update_cache`：

```cpp
void VoxelOctoTreeFASTLIVO::update_octo_tree() {
    // ...
    std::unordered_set<VoxelOctoTreeFASTLIVO *> update_cache;
    for (auto &pv : new_points_) {
        VoxelOctoTreeFASTLIVO *voxel = push_to_subvoxel(pv);
        if (voxel != nullptr) {
            update_cache.insert(voxel);
        }
    }
    new_points_.clear();

    for (auto voxel : update_cache) {
        voxel->update_single_voxel();
    }
}
```

这个 `update_cache` 是在**单个体素内部**，用于八叉树子体素的推送更新。但**在 `VoxelMapManagerFASTLIVO::UpdateMap` 层面，仍然遍历全表**。

#### 需要修改的部分

**修改位置 1**：`VoxelMapManagerFASTLIVO::UpdateMap`（[L559](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L559)）

```cpp
void VoxelMapManagerFASTLIVO::UpdateMap(const std::vector<pointWithVar> &input_points) {
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTreeFASTLIVO *> update_cache;
    uint plsize = input_points.size();

    for (uint i = 0; i < plsize; i++) {
        const pointWithVar &p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_w[j] / voxel_size_;
            if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                                 (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            iter->second->new_points_.push_back(p_v);
            iter->second->all_points_num_root_++;
        } else {
            VoxelOctoTreeFASTLIVO *octo_tree =
                new VoxelOctoTreeFASTLIVO(config_.max_layer_, 0, ...);
            voxel_map_[position] = octo_tree;
            // ... 初始化 ...
            octo_tree->new_points_.push_back(p_v);
            octo_tree->all_points_num_root_++;
        }
        // ← 关键：记录该体素到 update_cache
        update_cache[position] = voxel_map_[position];

        // 更新 LRU
        UpdateLRU(position);
    }
    total_points_num_ += input_points.size();

    // 优化：只遍历 update_cache，而非 voxel_map_ 全表
    for (auto &pair : update_cache) {    // ← O(N_touched) ≈ 30~80
        if (pair.second->new_points_.size() > 0) {
            pair.second->update_octo_tree();
        }
    }

    // LRU 淘汰
    LRUEvict();
}
```

**修改位置 2**：`VoxelMapManagerFASTLIVO::~VoxelMapManagerFASTLIVO`（[L505](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L505)）

这个不需要改，析构函数必须遍历全表释放内存，是合法操作。

**修改位置 3**：`clearMemOutOfMap`（[L702](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L702)）

这个滑动窗口函数也必须遍历全表以检查每个体素是否在窗口外，是合法操作。

### 4.4 性能收益估算

| 场景 | 当前（遍历全表） | 优化后（update_cache） | 节省 |
|------|:--------------:|:--------------------:|:----:|
| 每帧遍历体素数 | 500（LRU 上限） | 30~80（实际触达） | **84~94%** |
| 每帧无用遍历 | 420~470 个无新点的体素 | 0 | **完全消除** |
| 体素数量增大时 | 线性增长 | 不增长（仅取决于点云密度） | **可扩展** |

**估测收益**：`UpdateMap` 阶段耗时降低约 **60~80%**（因为去掉了 84% 以上的无效体素检查）

---

## 瓶颈 5：`Eigen::EigenSolver` → `Eigen::SelfAdjointEigenSolver`

### 5.1 问题描述

在 `init_plane` 函数中（[L30](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L30)），协方差矩阵的特征分解使用了通用的 `EigenSolver`：

```cpp
// 当前代码（慢）
Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
Eigen::Matrix3cd evecs = es.eigenvectors();   // ← 复数矩阵！
Eigen::Vector3cd evals = es.eigenvalues();     // ← 复数向量！
Eigen::Vector3d evalsReal;
evalsReal = evals.real();                       // ← 额外的 .real() 转换
```

**问题**：
1. 协方差矩阵是**对称半正定矩阵**（Symmetric Positive Semi-Definite），所有特征值都是实数
2. `EigenSolver` 是通用求解器，支持非对称矩阵，但计算量大约是 `SelfAdjointEigenSolver` 的 **3~5 倍**
3. 返回复数类型 `Eigen::Matrix3cd` 和 `Eigen::Vector3cd`，还需要 `.real()` 转换，浪费更多周期
4. 在 `max_points_num=1000` 时，`update_plane_cov` 中每点都会经过特征值分解

### 5.2 解决方案：替换为 `SelfAdjointEigenSolver`

#### 知识背景

| 求解器 | 适用矩阵 | 返回类型 | 相对速度 | 额外步骤 |
|--------|---------|---------|:--------:|---------|
| `EigenSolver` | 任意方阵 | `std::complex` | 3~5x | 需要 `.real()` |
| `SelfAdjointEigenSolver` | 对称/埃尔米特矩阵 | 实数 | **1x** | 无 |

对于协方差矩阵 `cov = Σ(ppᵀ)/N - μμᵀ`，它必定是对称半正定的，所以 `SelfAdjointEigenSolver` 是**数学上正确且更高效**的选择。

#### 代码修改

**修改位置**：[voxel_map_fastlivo.cpp L30](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L30)

替换为：

```cpp
// 优化后（快 3~5 倍）
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance_);
Eigen::Matrix3d evecs = es.eigenvectors();     // ← 直接实数矩阵！
Eigen::Vector3d evals = es.eigenvalues();       // ← 直接实数向量！

// 不需要 .real()，直接使用
int evalsMin, evalsMax;
evals.minCoeff(&evalsMin);
evals.maxCoeff(&evalsMax);
int evalsMid = 3 - evalsMin - evalsMax;

// 退化特征值保护
if (evalsMin == evalsMax || evalsMid < 0 || evalsMid > 2) {
    plane->is_update_ = true;
    plane->is_plane_ = false;
    return;
}

// 获取最小特征值对应的特征向量（法向量）
Eigen::Vector3d normal = evecs.col(evalsMin);
```

**关键差异**：

| 对比项 | EigenSolver（当前） | SelfAdjointEigenSolver（优化） |
|-------|:-----------------:|:----------------------------:|
| 特征向量类型 | `Eigen::Matrix3cd`（复数） | `Eigen::Matrix3d`（实数） |
| 特征值类型 | `Eigen::Vector3cd`（复数） | `Eigen::Vector3d`（实数） |
| 额外转换 | `.real()` 去除虚部 | **无需转换** |
| 速度 | ~3μs/次 @ 3x3 | **~0.8μs/次** @ 3x3 |

### 5.3 检查当前代码中所有特征分解调用点

当前代码中使用了 `SelfAdjointEigenSolver` 的已正确，只有一处未改：

| 位置 | 行号 | 当前类型 | 应改为 |
|------|:---:|:-------:|:------:|
| `init_plane` | [30](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L30) | ❌ `EigenSolver` | ✅ `SelfAdjointEigenSolver` |
| `update_single_voxel` 协方差计算 | [219](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L219) | ✅ 已用 | — |
| `check_and_update` 协方差检查 | [368](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/lio_odometry_Voxlmap/src/core/voxelmap/voxel_map_fastlivo.cpp#L368) | ✅ 已用 | — |

**只有 `init_plane` 中一处需要修改**，涉及的风险极低。

### 5.4 R-VoxelMap 验证

R-VoxelMap 在所有协方差特征分解位置都使用了 `SelfAdjointEigenSolver`：

[R-VoxelMap voxel_map_util.hpp L225](file:///home/hhy/2004ros/R-VoxelMap/include/voxel_map_util.hpp#L225)：

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance);
Eigen::Matrix3d evecs = es.eigenvectors();
Eigen::Vector3d evals = es.eigenvalues();
```

代码经过 KITTI、UrbanNav 等数据集测试验证，说明 `SelfAdjointEigenSolver` 在体素地图场景中是安全且正确的。

### 5.5 性能收益估算

| 场景 | 当前 EigenSolver | 优化 SelfAdjointEigenSolver | 节省 |
|------|:---------------:|:-------------------------:|:----:|
| 单次 3x3 特征分解 | ~3μs | ~0.8μs | **~73%** |
| `init_plane`（每体素触发一次） | ~6μs（含其他操作） | ~3μs | **~50%** |
| 体素平面初始化累积（500 体素） | ~3ms | ~1.5ms | **~1.5ms** |
| 每帧 `BuildResidualListOMP` 中的特征分解 | ~15μs | ~4μs | **~11μs** |

**虽然是微小优化，但累积效果显著**：在 `max_points_num=1000` 的场景下，`update_plane_cov` 会对每个点做特征值分解，总耗时从 O(1000×3μs) ≈ 3ms 降为 O(1000×0.8μs) ≈ 0.8ms，**节省约 2.2ms/帧**。

---

## 实施计划

### 优先级评估

| 瓶颈 | 影响面 | 改动量 | 风险 | 难度 | 优先级 |
|:----:|:-----:|:------:|:---:|:----:|:-----:|
| 4. `update_cache` | 中（每帧遍历） | 1 个函数内修改 | 低 | 低 | **P0** |
| 5. `SelfAdjointEigenSolver` | 低（微优化） | 1 行代码 | 极低 | 极低 | **P1** |

### 实施步骤

```
瓶颈 5（P1、改动极小、风险最低）：
  1. 在 init_plane() L30 中将 EigenSolver 替换为 SelfAdjointEigenSolver
  2. 移除 .real() 调用
  3. 用 minCoeff()/maxCoeff() 替代 rowwise().sum().minCoeff()
  4. 编译验证

瓶颈 4（P0、收益明确）：
  1. 声明 std::unordered_map<VOXEL_LOCATION, VoxelOctoTreeFASTLIVO *> update_cache
  2. 插入循环末尾添加 update_cache[position] = voxel_map_[position]
  3. 将全表 for 循环替换为 update_cache for 循环
  4. 编译验证，确认 LRU 和 sliding 功能不受影响
```

### 预期效果

| 优化 | 帧率提升 | MapUpdate 耗时 | 代码行数 |
|:----:|:-------:|:-------------:|:--------:|
| 4. update_cache | **+8~15%** | 60~80% ↓ | +5 行 |
| 5. SelfAdjointEigenSolver | **+2~5%** | 10~15% ↓ | -3 行（移除 .real()） |
| **合计** | **+10~20%** | **70~90% ↓** | **~2 行净增** |

---

## 附：对比参考

### R-VoxelMap 对应实现

| 优化项 | R-VoxelMap 位置 | R-VoxelMap 配置 |
|:-----:|:--------------:|:--------------:|
| update_cache | [voxel_map_util.hpp L1330-1380](file:///home/hhy/2004ros/R-VoxelMap/include/voxel_map_util.hpp#L1330-L1380) | 默认使用 |
| SelfAdjointEigenSolver | [voxel_map_util.hpp L225](file:///home/hhy/2004ros/R-VoxelMap/include/voxel_map_util.hpp#L225) | 所有特征分解位置 |
| LRU 缓存 | [voxel_map_util.hpp L1262-1390](file:///home/hhy/2004ros/R-VoxelMap/include/voxel_map_util.hpp#L1262-L1390) | `lru_capacity: 120000` |
| check_and_update O(1) | [voxel_map_util.hpp L430](file:///home/hhy/2004ros/R-VoxelMap/include/voxel_map_util.hpp#L430) | `update_size_threshold: 5` |

### 当前代码已应用的优化

| 优化项 | 当前状态 | 是否到位 |
|:-----:|:-------:|:--------:|
| LRU 淘汰 | ✅ 已实现 | 到位 |
| `push_to_subvoxel` 先匹配再插入 | ✅ 已实现 | 到位 |
| `check_and_update` O(1) 增量更新 | ✅ 已实现 | 到位 |
| `reinit_octo_tree` 重初始化 | ✅ 已实现 | 到位 |
| 4. `update_cache` 避免全表遍历 | ❌ 待实施 | **本次优化** |
| 5. `SelfAdjointEigenSolver` | ❌ 待实施 | **本次优化** |
| 6. `update_plane_cov` 协方差增量传播 | 🔄 部分实现 | 需进一步优化 |
| 7. `BuildResidualListOMP` 搜索后早期退出 | ❌ 待实施 | 下阶段目标 |

---

> **文档说明**：本文专注于风险最低、改动最小、收益明确的两个瓶颈优化。瓶颈 1（IEKF 每轮全量重搜）和瓶颈 2（八叉树递归无早期退出）涉及架构性改动，已有自己独立的优化方案文档。