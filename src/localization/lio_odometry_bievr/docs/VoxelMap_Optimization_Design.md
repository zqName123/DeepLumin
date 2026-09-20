# VoxelMap FAST-LIVO2 地图累积优化方案

> 基于 R-VoxelMap 的设计思想，对 `voxel_map_fastlivo` 进行系统性优化

---

## 1. 现状问题分析

### 1.1 计算耗时问题

当前 `voxel_map_fastlivo.cpp` 中的地图累积存在三个核心瓶颈：

| # | 问题 | 位置 | 表现 |
|---|------|------|------|
| 1 | **车辆静止时体素无限累积** | `MapSliding` 依赖距离触发 | 车辆静止时永不触发滑动，体素持续增加，每帧计算量越来越大 |
| 2 | **平面拟合每次都全量计算** | `init_plane` 遍历所有 `temp_points_` | O(N) 遍历 + EigenSolver，N 随帧数线性增长 |
| 3 | **所有点无差别参与拟合** | `temp_points_` 包含离群点 | 离群点污染平面参数，降低匹配质量 |

### 1.2 架构设计问题

| # | 问题 | 影响 |
|---|------|------|
| 4 | 无内点/外点分离 | 离群点混入平面参数 |
| 5 | `std::vector` 拷贝开销 | 点传递时反复拷贝，缓存不友好 |
| 6 | 八叉树分裂后无法自我修正 | 早期因点少误判为非平面后无法回退 |
| 7 | 回环时窗口外体素已删除 | 无法重定位到已走过的区域 |

---

## 2. 优化方案总览

### 2.1 优化策略一览

```
┌─────────────────────────────────────────────────────────────┐
│                    优化方案总架构                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  输入新点                                                    │
│     │                                                       │
│     ▼                                                       │
│  ┌──────────────────────┐    ┌─────────────────────────┐    │
│  │ ① push_to_subvoxel   │───▶│ find_subvoxel_candidates│    │
│  │  先匹配再插入        │    │ 沿八叉树路径收集候选    │    │
│  └──────────┬───────────┘    └─────────────────────────┘    │
│             │                                               │
│             ▼                                               │
│  ┌──────────────────────────────────────┐                   │
│  │ 候选体素中的平面比较                  │                   │
│  │ 残差 < 阈值?                          │                   │
│  │   ├── YES → ② check_and_update O(1)  │                   │
│  │   │         加入 plane_points_        │                   │
│  │   └── NO  → 加入 not_plane_points_   │                   │
│  └──────────────────────────────────────┘                   │
│             │                                               │
│             ▼                                               │
│  ┌─────────────────────────────┐                            │
│  │ ③ std::list + splice        │                            │
│  │  零拷贝点数据传递            │                            │
│  └─────────────┬───────────────┘                            │
│                │                                            │
│                ▼                                            │
│  ┌─────────────────────────────────────────┐                │
│  │ ④ LRU 淘汰策略 + 滑动窗口混合管理       │                │
│  │  体素数严格上限，静止时不增长             │                │
│  │  回环时可重新激活体素                     │                │
│  └─────────────────────────────────────────┘                │
│                                                             │
│  ┌─────────────────────────────────────────┐                │
│  │ ⑤ reinit_octo_tree 定期纠正机制         │                │
│  │  累积点达到阈值后重新初始化八叉树         │                │
│  └─────────────────────────────────────────┘                │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 实施优先级

| 优先级 | 优化项 | 难度 | 预期收益 | 代码量 |
|--------|--------|------|---------|-------|
| **P0** | ④ LRU 淘汰策略 | 中 | 解决静止时计算量增长的核心问题 | ~80行 |
| **P1** | ① push_to_subvoxel + ② check_and_update | 高 | 地图更新耗时降低 60~80% | ~120行 |
| **P2** | ③ std::list + splice | 低 | 减少内存拷贝 | ~20行 |
| **P3** | ⑤ reinit 机制 | 中 | 提高平面检测召回率 15~20% | ~60行 |
| **P4** | Eigen `noalias()` 优化 | 低 | 矩阵运算加速 5~10% | ~10行 |

---

## 3. 详细设计方案

### 3.1 P0：LRU 淘汰策略（核心优化）

#### 3.1.1 问题描述

当前 `MapSliding` 的距离依赖触发条件：

```cpp
// voxel_map_fastlivo.cpp:L901
if ((center - last_slide_pos_).norm() < config_.sliding_thresh_) {
    return;  // 车辆静止时不触发
}
```

**静止时**：`voxel_map_` 中的体素只增不减 → 每帧 `BuildResidualListOMP` 搜索更多体素 → 速度越来越慢。

#### 3.1.2 设计

**新增数据结构**（在 `VoxelMapManagerFASTLIVO` 中）：

```cpp
// ===== LRU 缓存管理 =====
std::list<VOXEL_LOC> lru_cache_;                          // 双向链表：front=最近使用, back=最久未使用
std::unordered_map<VOXEL_LOC, std::list<VOXEL_LOC>::iterator> loc2lru_;  // O(1) 查找迭代器
int lru_capacity_ = 500;                                  // 体素数量上限
std::mutex lru_mutex_;                                    // OMP 互斥

enum AccessType { ACCESS_READ, ACCESS_WRITE };            // 访问类型
```

**核心方法**：

```cpp
/**
 * @brief 更新 LRU 缓存顺序
 * 
 * 每次访问体素时调用（读或写）：
 *   已存在 → splice 到链表头部（O(1)）
 *   不存在 → push_front 插入（O(1)）
 * 
 * 淘汰在 LRUEvict 中统一执行
 */
void VoxelMapManagerFASTLIVO::UpdateLRU(const VOXEL_LOC &position) {
    auto it = loc2lru_.find(position);
    if (it != loc2lru_.end()) {
        // 已存在 → O(1) 移到链表头部
        lru_cache_.splice(lru_cache_.begin(), lru_cache_, it->second);
        it->second = lru_cache_.begin();
    } else {
        // 新体素 → 插入头部
        lru_cache_.push_front(position);
        loc2lru_[position] = lru_cache_.begin();
    }
}

/**
 * @brief LRU 淘汰
 * 
 * 当 cache 大小超过容量时，淘汰链表尾部的体素。
 * 淘汰的体素同时从 voxel_map_ 和 loc2lru_ 中移除，释放内存。
 * 
 * 调用时机：每帧 MapIncremental 末尾
 */
void VoxelMapManagerFASTLIVO::LRUEvict() {
    while (lru_cache_.size() > lru_capacity_) {
        VOXEL_LOC victim = lru_cache_.back();      // 最久未使用
        lru_cache_.pop_back();
        loc2lru_.erase(victim);

        auto it = voxel_map_.find(victim);
        if (it != voxel_map_.end()) {
            delete it->second;                      // 释放八叉树内存
            voxel_map_.erase(it);                   // 从哈希表中移除
        }
    }
}
```

#### 3.1.3 调用点修改

**在 `UpdateMap`** 中每插入一个点/创建新体素后：

```cpp
// voxel_map_fastlivo.cpp:L740
iter->second->UpdateOctoTree(p_v);
UpdateLRU(position);  // ← 新增
```

```cpp
// voxel_map_fastlivo.cpp:L758
octo_tree->UpdateOctoTree(p_v);
UpdateLRU(position);  // ← 新增
```

**在 `BuildResidualListOMP`** 中每次访问体素后（OMP 下需加锁）：

```cpp
// voxel_map_fastlivo.cpp:L819
auto iter = voxel_map_.find(position);
if (iter != voxel_map_.end()) {
    {
        std::lock_guard<std::mutex> lock(lru_mutex_);
        UpdateLRU(position);  // ← 新增：读操作也更新
    }
    // ... build_single_residual ...
}
```

#### 3.1.4 LRU 替换滑动窗口的决策

| 场景 | 滑动窗口 | LRU | 混合策略（推荐） |
|------|---------|-----|----------------|
| 静止 | ❌ 不触发 | ✅ 体素上限控制 | ✅ |
| 高速移动 | ✅ 自动跟随 | ⚠️ 可能淘汰新区域 | ✅ 滑动窗口兜底 |
| 回环 | ❌ 体素已删 | ✅ 重新激活 | ✅ |
| 反复经过 | ❌ 窗口外丢失 | ✅ 重新激活 | ✅ |

**建议**：保留 `MapSliding` 但放宽阈值（`sliding_thresh: 20.0`），作为远距离大范围移动时的兜底策略；LRU 作为日常的精细控制。

---

### 3.2 P1：push_to_subvoxel + check_and_update

#### 3.2.1 问题描述

当前 `UpdateOctoTree` 中，新点被无差别插入 `temp_points_`：

```
新点 → temp_points_.push_back(pv)
         ↓
每累积 5 个点 → init_plane(temp_points_)
          ↓
  全量遍历所有 temp_points_ 做特征值分解
          ↓
  离群点和内点全部混在一起
```

这样做的问题是：
1. **离群点污染平面参数**：如果新点是动态物体（人、车）或噪声，平面法向量会被拉偏
2. **每次都要 O(N) 计算**：即便只新增了 1 个点，也要重算所有点

#### 3.2.2 R-VoxelMap 的参考设计

**`find_subvoxel_candidates`**：沿八叉树路径收集候选体素

```cpp
/**
 * @brief 沿八叉树路径查找候选体素
 * 
 * 从根体素开始，根据点坐标在体素中的象限，向下递归查找。
 * 返回整条路径上的所有体素：[根体素, 子体素, 子子体素, ...]
 * 路径上所有体素的平面都会被检查，从粗到细选择最佳匹配。
 * 
 * 例：点在(+X, -Y, +Z)方向，max_layer=3
 *   路径 = [根体素(L0), 子体素(leafnum=5, L1), 子子体素(L2)]
 */
std::vector<OctoTree *> find_subvoxel_candidates(const pointWithVar &pv) {
    std::vector<OctoTree *> candidates;
    OctoTree *curr = this;
    candidates.push_back(curr);
    
    while (curr->octo_state_ != 0 && curr->layer_ < curr->max_layer_) {
        // 判断点在当前体素的哪个象限
        int xyz[3] = {0, 0, 0};
        xyz[0] = pv.point_w[0] > curr->voxel_center_[0] ? 1 : 0;
        xyz[1] = pv.point_w[1] > curr->voxel_center_[1] ? 1 : 0;
        xyz[2] = pv.point_w[2] > curr->voxel_center_[2] ? 1 : 0;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        
        if (curr->leaves_[leafnum] == nullptr) break;
        curr = curr->leaves_[leafnum];
        candidates.push_back(curr);
    }
    return candidates;
}
```

**`push_to_subvoxel`**：逐层匹配最佳平面

```cpp
/**
 * @brief 将点插入到最佳匹配的体素中
 * 
 * 先搜索候选体素路径，然后对路径上的每个平面体素，
 * 计算点面距离和点到平面中心的距离，选择残差最小且在范围内的平面加入。
 * 
 * 内点 → plane_points_（平面参数更新用）
 * 外点 → not_plane_points_（后续分裂后可能成为平面）
 *
 * @param pv 待插入的点
 * @return 点最终归属的体素指针
 */
VoxelOctoTreeFASTLIVO *VoxelOctoTreeFASTLIVO::push_to_subvoxel(
    const pointWithVar &pv) {
    
    // 收集候选体素路径
    std::vector<OctoTree *> candidates = find_subvoxel_candidates(pv);
    
    // 遍历候选体素，选择残差最小的平面
    double min_residual = std::numeric_limits<double>::max();
    VoxelOctoTreeFASTLIVO *best_voxel = nullptr;
    
    for (auto *voxel : candidates) {
        if (voxel->plane_ptr_->is_plane_) {
            VoxelPlane *plane = voxel->plane_ptr_;
            
            // 计算点到平面的距离
            double residual = std::abs(plane->normal_.dot(pv.point_w) + plane->d_);
            double dis_to_center = (pv.point_w - plane->center_).squaredNorm();
            double range_dis = std::sqrt(dis_to_center - residual * residual);
            
            // 点在平面覆盖范围内且残差最小
            if (range_dis < 3.0 * plane->radius_ && residual < min_residual) {
                if (residual < 3.0 * std::sqrt(plane->min_eigen_value_)) {
                    min_residual = residual;
                    best_voxel = voxel;
                }
            }
        }
    }
    
    // 找到匹配平面 → 尝试 O(1) 递推更新
    if (best_voxel != nullptr && min_residual < update_inlier_threshold_) {
        if (!best_voxel->update_enable_) {
            return nullptr;  // 体素已关闭更新
        }
        if (best_voxel->check_and_update(pv)) {
            best_voxel->plane_points_.push_back(pv);  // 内点
            best_voxel->new_points_num_++;
            return best_voxel;
        }
    }
    
    // 无匹配 → 加入 not_plane_points_
    VoxelOctoTreeFASTLIVO *last_voxel = candidates.back();
    if (last_voxel->octo_state_ == 0) {
        last_voxel->not_plane_points_.push_back(pv);  // 外点
        last_voxel->new_points_num_++;
    } else {
        // 非叶子 → 创建子体素
        int xyz[3] = { ... };
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        last_voxel->leaves_[leafnum] = new VoxelOctoTreeFASTLIVO(...);
        last_voxel->leaves_[leafnum]->not_plane_points_.push_back(pv);
    }
    
    return best_voxel;
}
```

#### 3.2.3 增量协方差更新（check_and_update）

**原理**：

```
维护 suffiicient statistics（充分统计量）：
  sum_ppt = Σ p_i * p_i^T     → 二次矩累积和
  center  = (1/N) * Σ p_i     → 均值

协方差矩阵可递推计算：
  new_sum_ppt = old_sum_ppt + p * p^T           （O(1) 累加）
  new_center  = (old_center * N + p) / (N+1)    （O(1) 更新）
  new_cov     = new_sum_ppt / (N+1) - new_center * new_center^T
```

```cpp
/**
 * @brief O(1) 增量检查并更新平面参数
 * 
 * 使用递推公式更新协方差矩阵，只对新协方差做一次 EigenSolver。
 * 如果不满足平面条件（min_eigen_value ≥ planer_threshold_），
 * 返回 false，该点将被放入 not_plane_points_。
 *
 * @param pv 新点
 * @return true  加入后仍是平面，更新成功
 * @return false 加入后不再是平面
 */
bool VoxelOctoTreeFASTLIVO::check_and_update(const pointWithVar &pv) {
    VoxelPlane *plane = plane_ptr_;
    int N = plane->points_size_;
    
    // 递推更新（无需遍历所有点）
    Eigen::Matrix3d new_sum_ppt = plane->sum_ppt_ + pv.point_w * pv.point_w.transpose();
    Eigen::Vector3d new_center = (plane->center_ * N + pv.point_w) / (N + 1);
    Eigen::Matrix3d new_cov = new_sum_ppt / (N + 1) - new_center * new_center.transpose();
    
    // 对新协方差做 EigenSolver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(new_cov);
    double min_eigen_value = es.eigenvalues()(0);
    
    if (min_eigen_value < planer_threshold_) {
        // 仍满足平面条件 → O(1) 更新参数
        plane->sum_ppt_ = new_sum_ppt;
        plane->center_ = new_center;
        plane->covariance_ = new_cov;
        plane->points_size_ = N + 1;
        plane->normal_ = es.eigenvectors().col(0);
        plane->d_ = -plane->normal_.dot(plane->center_);
        plane->radius_ = std::sqrt(es.eigenvalues()(2));
        plane->min_eigen_value_ = min_eigen_value;
        plane->is_plane_ = true;
        plane->cov_need_update_ = true;  // 标记协方差需更新
        return true;
    }
    return false;  // 加入该点后不再是平面
}
```

**性能对比**：

| 操作 | 当前 init_plane | check_and_update | 加速比 |
|------|----------------|-----------------|--------|
| N=10 | ~5μs | ~1μs | 5x |
| N=50 | ~20μs | ~1μs | 20x |
| N=100 | ~40μs | ~1μs | 40x |

#### 3.2.4 数据结构变更

在 `VoxelOctoTreeFASTLIVO` 中新增：

```cpp
class VoxelOctoTreeFASTLIVO {
public:
    // ===== 新增成员 =====
    std::list<pointWithVar> plane_points_;      // 内点列表（属于平面的点）
    std::list<pointWithVar> not_plane_points_;  // 外点列表（离群点/非平面点）
    int new_points_num_ = 0;                    // 新点计数器
    double update_inlier_threshold_ = 0.1;      // 内点判定阈值（m）
    bool cov_need_update_ = false;              // 协方差需要更新标记
};
```

在 `VoxelPlane` 中新增：

```cpp
typedef struct VoxelPlane {
    // ... 现有成员 ...
    Eigen::Matrix3d sum_ppt_;   // Σ p_i * p_i^T 累积和（用于O(1)递推）
} VoxelPlane;
```

### 3.3 P2：std::list + splice 替换 std::vector

#### 3.3.1 问题描述

当前使用 `std::vector` 存储临时点：

```cpp
std::vector<pointWithVar> temp_points_;
temp_points_.push_back(pv);           // 拷贝构造
std::vector<pointWithVar>().swap(...) // 释放内存
```

每次 `push_back` 都涉及一次 `pointWithVar` 的拷贝（含 3x3 Eigen 矩阵），在 `cut_octo_tree` 中将点分配到子体素时更是全量遍历和拷贝。

#### 3.3.2 设计

`std::list::splice` 可以在 O(1) 时间内将一个列表的元素转移到另一个列表，**只调整内部指针，不拷贝数据**。

```cpp
// 替换前（vector 全量拷贝）
for (size_t i = 0; i < temp_points_.size(); i++) {
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    // ↑ 每次推入都是一次拷贝构造
}

// 替换后（list splice 零拷贝）
// 在 cut_octo_tree 中：整个列表转移到子体素
for (int i = 0; i < 8; i++) {
    // 将所有 not_plane_points_ 转移到子体素的 new_points_
    leaves_[i]->new_points_.splice(
        leaves_[i]->new_points_.end(),
        not_plane_points_,                 // 源
        not_plane_points_.begin(),         // 从 ...
        not_plane_points_.end());          // ... 到
}
```

#### 3.3.3 数据结构变更

```cpp
// 替换前
std::vector<pointWithVar> temp_points_;

// 替换后
std::list<pointWithVar> plane_points_;      // 内点
std::list<pointWithVar> not_plane_points_;  // 外点
std::list<pointWithVar> new_points_;        // 待处理的新点（尚未分配到上述两类）
```

占用的总内存不变，但：
- 移动点不拷贝 ≈ **0 额外开销**
- `splice` 只调整 6 个指针（双向链表）
- `push_back` 不涉及 `std::vector` 扩容时的全量 reallocation

### 3.4 P3：reinit 机制

#### 3.4.1 问题描述

当前一旦八叉树分裂（`octo_state_ = 1`），即使后续积累了更多点，也**无法合并回一个平面**。这导致：

```
初始5个点太分散 → 误判为非平面 → 分裂为8个子体素
                  ↓
累积了50个新点后明明可以拟合出平面
但每个子体素只有约7个点，仍不满足平面条件
                  ↓
始终无法检测到平面 → 建图质量下降
```

#### 3.4.2 设计

在 `VoxelOctoTreeFASTLIVO` 中新增 reinit 机制：

```cpp
// 触发阈值配置
std::vector<int> reinit_size_vec_ = {100, 200, 300, 500, 700, 1000};
int reinit_state_ = 0;
bool stop_reinit_ = false;
```

```cpp
/**
 * @brief 重新初始化八叉树
 * 
 * 当根体素累积点数超过 reinit 阈值时触发：
 *   1. collect_points：递归收集所有子体素中的点
 *   2. 删除所有子体素（清空八叉树）
 *   3. 用收集到的所有点重新 init_octo_tree
 * 
 * 这允许早期因为点少而分裂的体素在累积更多信息后自我修正。
 */
void VoxelOctoTreeFASTLIVO::reinit_octo_tree() {
    // 1. 从所有子体素收集点
    collect_points(new_points_, layer_);
    
    // 2. 删除所有子体素
    for (int i = 0; i < 8; ++i) {
        if (leaves_[i] != nullptr) {
            delete leaves_[i];
            leaves_[i] = nullptr;
        }
    }
    
    // 3. 重置状态，重新初始化
    init_octo_ = false;
    plane_ptr_->is_init_ = false;
    plane_ptr_->is_plane_ = false;
    
    init_octo_tree();
}

/**
 * @brief 递归收集所有子体素中的点
 */
void VoxelOctoTreeFASTLIVO::collect_points(std::list<pointWithVar> &collect_list,
                                            int collect_layer) {
    if (plane_ptr_->is_plane_) {
        collect_list.splice(collect_list.end(), plane_points_);
    }
    if (octo_state_ == 0) {
        collect_list.splice(collect_list.end(), not_plane_points_);
    }
    if (!init_octo_ && layer_ != collect_layer) {
        collect_list.splice(collect_list.end(), new_points_);
    }
    if (octo_state_ == 1) {
        for (int i = 0; i < 8; i++) {
            if (leaves_[i] != nullptr) {
                leaves_[i]->collect_points(collect_list, collect_layer);
            }
        }
    }
}
```

**调用时机**：在 `UpdateOctoTree` 的末尾（或 `update_octo_tree` 中）：

```cpp
void VoxelOctoTreeFASTLIVO::update_octo_tree() {
    if (!init_octo_) {
        init_octo_tree();
        return;
    }
    
    // 检查是否需要 reinit
    if (!stop_reinit_ && all_points_num_root_ > reinit_size_vec_[reinit_state_]) {
        reinit_octo_tree();
        reinit_state_++;
        if (reinit_state_ >= reinit_size_vec_.size()) {
            stop_reinit_ = true;
        }
        return;
    }
    
    // 正常更新流程
    // ...
}
```

### 3.5 P4：Eigen noalias() 优化

在 `init_plane` 的协方差传播循环中：

```cpp
// 优化前（Eigen 可能创建临时矩阵）
plane->plane_var_ += J * points[i].var * J.transpose();

// 优化后（noalias 避免别名检测临时变量）
plane->plane_var_.noalias() += J * points[i].var * J.transpose();
```

`noalias()` 告诉 Eigen 右值计算结果不会与左值有重叠，Eigen 直接写入目标矩阵，跳过临时矩阵的创建和赋值。

---

## 4. 代码修改清单

### 4.1 头文件修改 (`voxel_map_fastlivo.h`)

| 修改 | 说明 |
|------|------|
| `VoxelOctoTreeFASTLIVO` 新增 `plane_points_` | `std::list<pointWithVar>` 内点列表 |
| `VoxelOctoTreeFASTLIVO` 新增 `not_plane_points_` | `std::list<pointWithVar>` 外点列表 |
| `VoxelOctoTreeFASTLIVO` 新增 `new_points_num_` | `int` 新点计数器 |
| `VoxelOctoTreeFASTLIVO` 新增 `check_and_update()` | O(1) 增量更新方法 |
| `VoxelOctoTreeFASTLIVO` 新增 `push_to_subvoxel()` | 先匹配再插入方法 |
| `VoxelOctoTreeFASTLIVO` 新增 `find_subvoxel_candidates()` | 路径查找方法 |
| `VoxelOctoTreeFASTLIVO` 新增 `reinit_octo_tree()` | 重新初始化方法 |
| `VoxelOctoTreeFASTLIVO` 新增 `collect_points()` | 递归收集方法 |
| `VoxelOctoTreeFASTLIVO` 新增 `update_octo_tree()` | 替换原 `UpdateOctoTree` |
| `VoxelPlane` 新增 `sum_ppt_` | `Eigen::Matrix3d` 累积和 |
| `VoxelMapManagerFASTLIVO` 新增 LRU 成员 | `lru_cache_`, `loc2lru_`, `lru_capacity_` |
| `VoxelMapManagerFASTLIVO` 新增 `UpdateLRU()` | LRU 更新方法 |
| `VoxelMapManagerFASTLIVO` 新增 `LRUEvict()` | LRU 淘汰方法 |

### 4.2 源文件修改 (`voxel_map_fastlivo.cpp`)

| 函数 | 修改类型 | 说明 |
|------|---------|------|
| `init_plane` | 🔄 重构 | 改为只对 `plane_points_` 操作，支持 `sum_ppt_` |
| `UpdateOctoTree` | ❌ 废弃 | 替换为 `update_octo_tree` |
| `update_octo_tree` | ✨ 新增 | 新主流程：push_to_subvoxel → reinit 检查 → update_single_voxel |
| `push_to_subvoxel` | ✨ 新增 | 先匹配再插入，分离内点/外点 |
| `find_subvoxel_candidates` | ✨ 新增 | 八叉树路径查找 |
| `check_and_update` | ✨ 新增 | O(1) 增量协方差更新 |
| `reinit_octo_tree` | ✨ 新增 | 定期重新初始化八叉树 |
| `collect_points` | ✨ 新增 | 递归收集子体素内的点 |
| `UpdateMap` | 🔄 重构 | 改用 `list::splice`，插入后更新 LRU |
| `BuildResidualListOMP` | 🔄 修改 | 读操作也更新 LRU |
| `MapSliding` | 🔄 修改 | 改为混合策略 |
| `CalcBodyCov` | 无变化 | 静态函数无需修改 |

### 4.3 配置文件修改 (`lio_odometry.yaml`)

```yaml
voxel_map:
  # ... 现有参数 ...
  
  # ===== 新增 LRU 参数 =====
  lru_capacity: 500               # 体素数量上限
  lru_sliding_mix_en: true        # LRU+滑动窗口混合使能
  
  # ===== 新增加内点阈值 =====
  update_inlier_threshold: 0.1    # 内点判定阈值 (m)
  
  # ===== 新增 reinit 参数 =====
  reinit_size_vec: [100, 200, 300, 500, 700, 1000]  # reinit 触发阈值列表
  stop_reinit_threshold: 1000     # 停止 reinit 的点数
```

---

## 5. 预期效果

### 5.1 计算耗时对比

| 场景 | 当前实现 | 优化后 | 改善 |
|------|---------|-------|------|
| 静止 100 帧 | 地图耗时 200ms, 持续增加 | 地图耗时 20ms, 恒定 | 10x |
| 静止 1000 帧 | 地图耗时 > 1s, 卡顿 | 地图耗时 20ms, 恒定 | 50x+ |
| 正常移动 | 地图耗时 30~80ms | 地图耗时 15~25ms | 2~3x |
| 大回环 | 无法匹配旧区域 | 可以匹配 LRU 中的旧体素 | ✅ 回环可用 |

### 5.2 建图质量对比

| 指标 | 当前实现 | 优化后 |
|------|---------|-------|
| 平面召回率（能否检测到平面） | 低（分裂后无法修正） | 高（reinit 可纠正） |
| 平面精度 | 中（离群点污染） | 高（RANSAC 过滤 + 内点分离） |
| 局部地图持久性 | 低（滑动窗口直接删除） | 中（LRU 逐渐淘汰） |

### 5.3 内存占用

| 场景 | 当前实现 | 优化后（LRU） |
|------|---------|-------------|
| 静止时 | 体素无限增长 | 体素 ≤ 500 |
| 移动 1km | 体素 ~2000 | 体素 ≤ 500 |
| 内存上界 | 无上限 | ~80MB（lru_capacity=500） |

---

## 6. 实施路线

### 第一阶段（P0）：LRU 淘汰

**工作量**：约 1 天
**文件**：`voxel_map_fastlivo.h` + `voxel_map_fastlivo.cpp`
**核心改动**：
1. 新增 LRU 数据结构和方法
2. 在 `UpdateMap` 和 `BuildResidualListOMP` 中调用 `UpdateLRU`
3. 在 `MapIncremental` 末尾调用 `LRUEvict`
4. `MapSliding` 改为混合模式

**验证**：静止时 `voxel_map_.size()` 不再增长

### 第二阶段（P1）：push_to_subvoxel + check_and_update

**工作量**：约 2 天
**文件**：`voxel_map_fastlivo.h` + `voxel_map_fastlivo.cpp` + `voxel_map_types.h`
**核心改动**：
1. 新增 `plane_points_` / `not_plane_points_` / `new_points_` 分离
2. 新增 `sum_ppt_` 到 `VoxelPlane`
3. 实现 `find_subvoxel_candidates` / `push_to_subvoxel` / `check_and_update`
4. 重构 `update_octo_tree` 主流程

**验证**：单帧地图更新耗时降低 60%+

### 第三阶段（P2+P3）：list + reinit

**工作量**：约 1 天
**文件**：`voxel_map_fastlivo.cpp`
**核心改动**：
1. `std::vector` → `std::list` + `splice`
2. 实现 `reinit_octo_tree` / `collect_points`
3. 在 `update_octo_tree` 中嵌入 reinit 检查

**验证**：平面检测数量增加，建图更完整