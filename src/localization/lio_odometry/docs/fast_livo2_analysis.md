# FAST-LIVO2 技术分析文档

## 一、项目概述

FAST-LIVO2 是香港大学 MARS 实验室开发的 **LiDAR-惯性-视觉紧耦合里程计系统**，发表于 T-RO 2024。

**核心特点**：
- 直接法视觉里程计（不需要特征提取匹配）
- VoxelMap 体素地图（高效点云地图管理 + 平面拟合 + 不确定性传播）
- 迭代误差状态卡尔曼滤波器（ESKF）做状态估计
- 曝光时间在线估计（补偿光照变化）

**论文**：[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)

---

## 二、系统架构

### 2.1 模块总览

```
┌─────────────────────────────────────────────────────────────────┐
│                        LIVMapper (主控)                          │
│  ┌──────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────┐ │
│  │ Preprocess│  │ ImuProcess   │  │VoxelMapManager│  │VIOManager│ │
│  │  点云预处理 │  │ IMU处理+传播  │  │  Voxelmap管理  │  │  视觉里程计│ │
│  └──────────┘  └──────────────┘  └──────────────┘  └─────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 数据流

```
IMU ──→ ImuProcess ──→ 状态传播 + 点云去畸变
                              ↓
LiDAR ──→ Preprocess ──→ 降采样 ──→ VoxelMapManager.StateEstimation()
                                              ↓
                                      EKF 更新（LIO）
                                              ↓
                                      VoxelMap 更新
                                              ↓
Image ──→ VIOManager.processFrame() ──→ EKF 更新（VIO）
```

### 2.3 源文件清单

| 文件 | 作用 |
|------|------|
| `LIVMapper.h/cpp` | 主控节点，调度所有模块 |
| `preprocess.h/cpp` | 点云预处理（去畸变、降采样） |
| `IMU_Processing.h/cpp` | IMU 初始化、前向传播、去畸变 |
| `voxel_map.h/cpp` | 体素地图管理、残差构建、EKF 更新 |
| `vio.h/cpp` | 视觉里程计（直接法） |
| `visual_point.h/cpp` | 视觉地图点管理 |
| `frame.h/cpp` | 图像帧管理 |
| `common_lib.h` | 通用数据结构、状态定义 |

---

## 三、状态向量定义

```cpp
// common_lib.h - StatesGroup (19维)
#define DIM_STATE (19)

struct StatesGroup {
    M3D rot_end;           // [0-2]  SO3 旋转
    V3D pos_end;           // [3-5]  位置
    double inv_expo_time;  // [6]    逆曝光时间（视觉特有）
    V3D vel_end;           // [7-9]  速度
    V3D bias_g;            // [10-12] 陀螺仪偏差
    V3D bias_a;            // [13-15] 加速度计偏差
    V3D gravity;           // [16-18] 重力加速度
    Matrix<double, 19, 19> cov;  // 协方差矩阵
};
```

**状态维度分解**：

| 索引 | 状态 | 维度 | 来源 |
|------|------|------|------|
| 0-2 | 旋转 (SO3) | 3 | LIO + VIO |
| 3-5 | 位置 | 3 | LIO + VIO |
| 6 | 逆曝光时间 | 1 | VIO |
| 7-9 | 速度 | 3 | LIO |
| 10-12 | 陀螺仪偏差 | 3 | LIO |
| 13-15 | 加速度计偏差 | 3 | LIO |
| 16-18 | 重力向量 | 3 | LIO |

---

## 四、VoxelMap 原理详解

### 4.1 数据结构

```
voxel_map_ (unordered_map<VOXEL_LOCATION, VoxelOctoTree*>)
  │
  ├── VOXEL_LOCATION (int64_t x, y, z)  ← 体素网格坐标（哈希键）
  │
  └── VoxelOctoTree*                     ← 八叉树节点
        ├── temp_points_                  ← 暂存的点云
        ├── plane_ptr_ (VoxelPlane*)      ← 拟合的平面参数
        ├── leaves_[8]                    ← 8个子节点
        ├── layer_                        ← 当前层（0=根）
        ├── voxel_center_[3]              ← 体素中心坐标
        ├── quater_length_                ← 体素边长/4
        ├── octo_state_                   ← 0=叶子, 1=非叶子
        └── init_octo_                    ← 是否已初始化
```

**关键结构体**：

```cpp
// 体素位置（哈希键）
class VOXEL_LOCATION {
    int64_t x, y, z;  // 体素网格坐标
    // 哈希函数：(z * P + y) * P + x
};

// 平面参数
struct VoxelPlane {
    Eigen::Vector3d center_;           // 平面中心
    Eigen::Vector3d normal_;           // 平面法向量
    Eigen::Vector3d y_normal_;         // 次法向量
    Eigen::Vector3d x_normal_;         // 主法向量
    Eigen::Matrix3d covariance_;       // 3x3 协方差矩阵
    Eigen::Matrix<double, 6, 6> plane_var_;  // 6x6 平面参数协方差
    float radius_;                     // 平面半径
    float min_eigen_value_;            // 最小特征值
    float mid_eigen_value_;            // 中间特征值
    float max_eigen_value_;            // 最大特征值
    float d_;                          // 平面方程 d 参数
    bool is_plane_;                    // 是否为平面
};

// 带协方差的点
struct pointWithVar {
    Eigen::Vector3d point_b;      // 机体系坐标
    Eigen::Vector3d point_i;      // IMU 系坐标
    Eigen::Vector3d point_w;      // 世界系坐标
    Eigen::Matrix3d var;          // 世界系协方差
    Eigen::Matrix3d body_var;     // 机体系协方差
    Eigen::Matrix3d point_crossmat;  // 反对称矩阵
};

// 点到平面对应关系
struct PointToPlane {
    Eigen::Vector3d point_b_;    // 机体系点
    Eigen::Vector3d point_w_;    // 世界系点
    Eigen::Vector3d normal_;     // 平面法向量
    Eigen::Vector3d center_;     // 平面中心
    Eigen::Matrix<double, 6, 6> plane_var_;  // 平面参数协方差
    M3D body_cov_;               // 点协方差
    double d_;                   // 平面方程 d
    double eigen_value_;         // 特征值
    float dis_to_plane_;         // 点到平面距离
};
```

### 4.2 建图流程（BuildVoxelMap）

```cpp
// voxel_map.cpp BuildVoxelMap()

// 步骤1：计算每个点的协方差
for (每个降采样后的点) {
    // 测量噪声（距离误差 + 角度误差）
    calcBodyCov(point, dept_err, beam_err, body_var);

    // 传播到世界系（考虑状态不确定性）
    var = R * body_var * R^T + [-p×] * rot_var * [-p×]^T + t_var;
}

// 步骤2：将点插入体素
for (每个点) {
    // 计算体素坐标
    voxel_xyz = floor(point / voxel_size);

    // 哈希查找或创建
    VOXEL_LOCATION position(voxel_xyz);
    if (voxel_map_.find(position)) {
        voxel_map_[position]->temp_points_.push_back(pv);
    } else {
        VoxelOctoTree *tree = new VoxelOctoTree(...);
        voxel_map_[position] = tree;
    }
}

// 步骤3：初始化八叉树（拟合平面）
for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
    iter->second->init_octo_tree();
}
```

### 4.3 平面拟合算法（init_plane）

```cpp
// voxel_map.cpp VoxelOctoTree::init_plane()

// 步骤1：计算协方差矩阵和中心
center = Σ(point) / N;
covariance = Σ(point * point^T) / N - center * center^T;

// 步骤2：特征值分解
EigenSolver<Matrix3d> es(covariance);
eigenvalues = [λ_min, λ_mid, λ_max];
eigenvectors = [v_min, v_mid, v_max];

// 步骤3：判断是否为平面
if (λ_min < planner_threshold) {
    // 是平面
    normal = v_min;                    // 最小特征值对应的特征向量
    d = -normal · center;              // 平面方程 d 参数
    radius = sqrt(λ_max);              // 平面半径
    is_plane = true;

    // 步骤4：计算平面参数协方差（6x6）
    // 这是 FAST-LIVO2 的核心创新之一
    for (每个点 i) {
        // 雅可比矩阵：对特征值/特征向量的偏导
        for (m = 0; m < 3; m++) {
            if (m != min_index) {
                F_m = (p_i - center)^T / (N * (λ_min - λ_m))
                      * (v_m * v_min^T + v_min * v_m^T);
            } else {
                F_m = [0, 0, 0];
            }
        }

        // J = [d(特征向量)/d(点); d(中心)/d(点)]
        J.block<3,3>(0,0) = eigenvectors * F;
        J.block<3,3>(3,0) = I / N;

        // 累积平面参数协方差
        plane_var += J * point_var * J^T;
    }
} else {
    // 不是平面，需要分裂
    is_plane = false;
}
```

**核心思想**：
- 平面内点云的最小特征值接近 0
- 如果最小特征值大于阈值，说明点云不共面，需要分裂
- 平面参数协方差 `plane_var`（6x6）记录了平面拟合的不确定性，用于后续自适应 R

### 4.4 八叉树分裂（cut_octo_tree）

```cpp
// voxel_map.cpp VoxelOctoTree::cut_octo_tree()

if (layer >= max_layer) return;  // 达到最大层数

// 按点相对于体素中心的位置分配到8个子节点
for (每个点) {
    xyz[0] = point.x > center.x ? 1 : 0;
    xyz[1] = point.y > center.y ? 1 : 0;
    xyz[2] = point.z > center.z ? 1 : 0;
    leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    leaves_[leafnum]->temp_points_.push_back(point);
}

// 对每个子节点尝试拟合平面
for (i = 0; i < 8; i++) {
    if (leaves_[i] != nullptr && leaves_[i]->temp_points_.size() >= threshold) {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);

        if (!leaves_[i]->plane_ptr_->is_plane_ && layer < max_layer) {
            leaves_[i]->cut_octo_tree();  // 递归分裂
        }
    }
}
```

**分裂策略**：
- 当体素内点不构成平面时，分裂为 8 个子体素
- 每个子体素的边长是父体素的一半
- 递归直到达到最大层数或点构成平面

### 4.5 残差构建（BuildResidualListOMP）

```cpp
// voxel_map.cpp VoxelMapManager::BuildResidualListOMP()

#pragma omp parallel for  // 多线程并行
for (每个点 pv) {
    // 步骤1：找到对应体素（O(1) 哈希查找）
    VOXEL_LOCATION position = floor(pv.point_w / voxel_size);
    VoxelOctoTree *octo = voxel_map_[position];

    // 步骤2：在八叉树中查找最近平面
    build_single_residual(pv, octo, 0, is_success, prob, single_ptpl);

    // 步骤3：如果失败，尝试相邻体素
    if (!is_success) {
        // 检查是否在体素边界附近
        VOXEL_LOCATION near_position = position;
        if (靠近 +x 边界) near_position.x += 1;
        if (靠近 -x 边界) near_position.x -= 1;
        // ... y, z 方向同理

        auto iter_near = voxel_map_.find(near_position);
        if (iter_near != voxel_map_.end()) {
            build_single_residual(pv, iter_near->second, 0, ...);
        }
    }
}
```

### 4.6 单点残差计算（build_single_residual）

```cpp
// voxel_map.cpp VoxelMapManager::build_single_residual()

if (octo->plane_ptr_->is_plane_) {
    VoxelPlane &plane = *octo->plane_ptr_;

    // 步骤1：计算点到平面距离
    dis_to_plane = |normal · point_w + d|;

    // 步骤2：检查点是否在平面范围内
    dis_to_center = |point_w - center|;
    range_dis = sqrt(dis_to_center^2 - dis_to_plane^2);

    if (range_dis > 3 * radius) return;  // 超出平面范围

    // 步骤3：计算不确定性
    // 平面参数不确定性
    J_nq = [point_w - center, -normal];
    sigma_l = J_nq * plane_var * J_nq^T;

    // 加上点的不确定性
    sigma_l += normal^T * point_var * normal;

    // 步骤4：门限判断
    if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
        // 成功匹配
        is_success = true;

        // 记录对应关系
        single_ptpl.point_b_ = pv.point_b;
        single_ptpl.point_w_ = pv.point_w;
        single_ptpl.normal_ = plane.normal_;
        single_ptpl.center_ = plane.center_;
        single_ptpl.d_ = plane.d_;
        single_ptpl.plane_var_ = plane.plane_var_;
        single_ptpl.dis_to_plane_ = dis_to_plane;
    }
} else {
    // 不是平面，递归查找子节点
    if (current_layer < max_layer) {
        for (每个子节点) {
            build_single_residual(pv, child, layer+1, ...);
        }
    }
}
```

**关键创新**：
- **不确定性传播**：`sigma_l` 包含平面参数不确定性和点不确定性
- **自适应门限**：`sigma_num * sqrt(sigma_l)` 根据不确定性动态调整
- **概率选择**：当一个点匹配到多个平面时，选择概率最大的

### 4.7 地图滑动（mapSliding）

```cpp
// voxel_map.cpp VoxelMapManager::mapSliding()

if ((当前位置 - 上次滑动位置).norm() < sliding_thresh) return;

// 删除超出范围的体素
for (auto it = voxel_map_.begin(); it != voxel_map_.end(); ) {
    VOXEL_LOCATION loc = it->first;

    if (loc.x > x_max || loc.x < x_min ||
        loc.y > y_max || loc.y < y_min ||
        loc.z > z_max || loc.z < z_min) {
        delete it->second;
        it = voxel_map_.erase(it);
    } else {
        ++it;
    }
}
```

---

## 五、LIO 状态估计（EKF 更新）

### 5.1 迭代 EKF 更新流程

```cpp
// voxel_map.cpp VoxelMapManager::StateEstimation()

for (iterCount = 0; iterCount < max_iterations; iterCount++) {

    // ====== 步骤1：变换点云到世界系 ======
    TransformLidar(state.rot_end, state.pos_end, feats_down_body, world_lidar);

    // ====== 步骤2：计算每个点的协方差 ======
    for (每个点 i) {
        // 测量噪声
        calcBodyCov(point, dept_err, beam_err, body_var);

        // 传播到世界系
        cov = state.rot_end * body_var * state.rot_end^T
            + [-p×] * state.cov.rot * [-p×]^T
            + state.cov.pos;
    }

    // ====== 步骤3：构建残差列表（OMP 并行） ======
    BuildResidualListOMP(pv_list, ptpl_list);

    // ====== 步骤4：构建观测矩阵 H 和残差向量 ======
    Hsub.resize(effect_feat_num, 6);  // 6列：旋转3 + 平移3
    R_inv.resize(effect_feat_num);
    meas_vec.resize(effect_feat_num);

    for (每个有效点 i) {
        V3D point_this = extR * point_b + extT;  // 转到 IMU 系
        M3D point_crossmat = [point_this]×;

        // H 矩阵：对旋转和平移的偏导
        V3D A = point_crossmat * R^T * normal;
        Hsub.row(i) << A[0], A[1], A[2], normal[0], normal[1], normal[2];

        // 自适应 R（考虑平面和点的不确定性）
        // R = 0.001 + 平面参数不确定性 + 点不确定性
        double sigma_l = J_nq * plane_var * J_nq^T;
        R_inv(i) = 1.0 / (0.001 + sigma_l + normal^T * var * normal);

        // 残差
        meas_vec(i) = -dis_to_plane;
    }

    // ====== 步骤5：迭代卡尔曼更新 ======
    // K_1 = (H^T * R^{-1} * H + P^{-1})^{-1}
    H_T_H = Hsub^T * R^{-1} * Hsub;
    K_1 = (H_T_H + P^{-1})^{-1};

    // G = K_1 * H^T * R^{-1} * H
    G = K_1 * H_T_H;

    // 状态更新
    // solution = K_1 * H^T * R^{-1} * z + (x_prop - x_est) - G * (x_prop_sub - x_est_sub)
    auto vec = state_propagat - state_;
    solution = K_1 * HTz + vec - G * vec_sub;
    state_ += solution;

    // ====== 步骤6：收敛判断 ======
    rot_add = solution.block<3,1>(0,0);
    t_add = solution.block<3,1>(3,0);
    if (rot_add.norm() * 57.3 < 0.01 && t_add.norm() * 100 < 0.015) {
        converged = true;
    }

    // ====== 步骤7：重匹配判断 ======
    if (converged || (rematch_num == 0 && iterCount == max_iterations - 2)) {
        rematch_num++;
    }

    // ====== 步骤8：协方差更新 ======
    if (rematch_num >= 2 || iterCount == max_iterations - 1) {
        P = (I - G) * P;
        break;
    }
}
```

### 5.2 H 矩阵结构

```
Hsub = [A1, n1]     // 第1个点
       [A2, n2]     // 第2个点
       ...          // ...
       [An, nn]     // 第n个点

其中：
  Ai = [pi]× * R^T * ni   // 对旋转的偏导（3维）
  ni = 平面法向量          // 对平移的偏导（3维）
```

**注意**：LIO 的 H 矩阵只有 6 列（旋转 + 平移），不像 lio_odometry 有 12 列（包含外参）。

### 5.3 自适应 R 的计算

```
R_i = 0.001 + sigma_l + normal^T * var * normal

其中：
  sigma_l = J_nq * plane_var * J_nq^T   // 平面参数不确定性
  normal^T * var * normal                // 点不确定性在法向量方向的投影
```

**物理含义**：
- `sigma_l`：平面拟合的不确定性（特征值分解的误差传播）
- `normal^T * var * normal`：点坐标不确定性在平面法向量方向的分量
- 两者之和表示"点到平面距离"这个观测的总不确定性

---

## 六、VIO（直接法视觉里程计）

### 6.1 核心思想

FAST-LIVO2 使用**直接法**视觉里程计：
- 不提取特征点（如 ORB、FAST）
- 直接比较图像块（patch）的光度误差
- 使用 VoxelMap 中的 3D 点作为视觉地图点

### 6.2 视觉地图点

```cpp
// visual_point.h
class VisualPoint {
    Vector3d pos_;           // 3D 位置（世界系）
    Vector3d normal_;        // 表面法向量
    list<Feature*> obs_;     // 观测列表（图像块）
    Feature *ref_patch;      // 参考图像块
    bool is_normal_initialized_;
};
```

**视觉地图点的来源**：
- VoxelMap 中的平面点
- 通过 LiDAR-Camera 外参投影到图像平面
- 提取图像块作为参考 patch

### 6.3 处理流程

```cpp
// vio.cpp VIOManager::processFrame()

// 步骤1：从视觉稀疏地图中检索可见点
retrieveFromVisualSparseMap(img, pg, plane_map);

// 步骤2：对每个地图点
for (每个可见地图点) {
    // 投影到当前图像
    px = cam->world2cam(Rcw * p_world + Pcw);

    // 计算仿射变换矩阵（考虑深度和法向量）
    getWarpMatrixAffineHomography(cam, px_ref, xyz_ref, normal_ref, T_cur_ref, ...);

    // 获取最佳搜索层级
    getBestSearchLevel(A_cur_ref, max_level);
}

// 步骤3：光度误差 EKF 更新
computeJacobianAndUpdateEKF(img);

// 步骤4：更新视觉地图点
updateVisualMapPoints(img);
```

### 6.4 仿射变换（Affine Warp）

```cpp
// vio.cpp getWarpMatrixAffineHomography()

// 目的：补偿视角变化导致的图像块变形
// 原理：用平面的法向量计算单应性矩阵

xyz_ref = f_ref * depth_ref;  // 参考帧中的 3D 点

// 计算参考帧中 patch 四个角的 3D 坐标
xyz_du_ref = cam->cam2world(px_ref + [halfpatch, 0]) * depth_ref;
xyz_dv_ref = cam->cam2world(px_ref + [0, halfpatch]) * depth_ref;

// 通过平面单应性变换到当前帧
H = R_cur_ref + t_cur_ref * normal_ref^T / depth_ref;  // 单应性矩阵

px_cur = H * xyz_ref;
px_du = H * xyz_du_ref;
px_dv = H * xyz_dv_ref;

// 仿射变换矩阵
A_cur_ref.col(0) = (px_du - px_cur) / halfpatch;
A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch;
```

### 6.5 光度残差与 EKF 更新

```cpp
// vio.cpp computeJacobianAndUpdateEKF()

for (每个可见地图点) {
    // 1. 获取参考 patch
    float *ref_patch = map_point->ref_patch;

    // 2. 获取当前 patch（通过仿射变换）
    warpAffine(A_cur_ref, img_ref, px_ref, ...);
    float *cur_patch = warped_patch;

    // 3. 计算光度误差
    for (每个像素) {
        error = cur_patch[j] * exp(inv_expo_time) - ref_patch[j];
    }

    // 4. 计算雅可比矩阵
    // dI/dp：图像对像素的梯度（通过 Sobel 算子）
    // dp/dP：像素对 3D 点的投影雅可比
    // dP/dx：3D 点对状态的雅可比

    // 投影雅可比
    dp_dP = [fx/Z, 0, -fx*X/Z^2;
             0, fy/Z, -fy*Y/Z^2];

    // 状态雅可比
    dP_dx = [R^T * [p]×, -R^T, 0, 0, 0, 0, 0];  // 对旋转
    dP_dx = [0, 0, 0, I, 0, 0, 0];                // 对位置

    // 完整雅可比
    J = dI_dp * dp_dP * dP_dx;

    // 5. EKF 更新
    H_vio.row(i) = J;
    R_vio(i) = img_point_cov;
}

// EKF 更新
K = P * H^T * (H * P * H^T + R)^{-1};
dx = K * error;
state += dx;
P = (I - K * H) * P;
```

### 6.6 曝光时间估计

```cpp
// 状态中的 inv_expo_time 用于补偿光照变化
// 光度误差模型：
// I_cur(t) * exp(inv_expo_time) - I_ref = 0

// 在 EKF 更新中，inv_expo_time 作为状态的一部分被估计
// 雅可比中需要加上对 inv_expo_time 的偏导：
// d(error)/d(inv_expo_time) = I_cur(t) * exp(inv_expo_time)
```

**创新点**：在线估计曝光时间，使得直接法在光照变化环境下更加鲁棒。

---

## 七、IMU 处理

### 7.1 IMU 初始化

```cpp
// IMU_Processing.cpp ImuProcess::IMU_init()

// 前 MAX_INI_COUNT 帧用于初始化
for (每帧 IMU 数据) {
    // 增量均值计算
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    // 方差估计
    cov_acc = cov_acc * (N-1)/N + (cur_acc - mean_acc)^2 * (N-1)/(N*N);
    cov_gyr = cov_gyr * (N-1)/N + (cur_gyr - mean_gyr)^2 * (N-1)/(N*N);
}

// 初始化状态
state.gravity = -mean_acc / |mean_acc| * 9.81;  // 重力方向
state.rot_end = I;                                // 初始旋转
state.bias_g = mean_gyr;                          // 陀螺仪偏差
```

### 7.2 前向传播 + 点云去畸变

```cpp
// IMU_Processing.cpp ImuProcess::UndistortPcl()

// ====== 步骤1：前向传播（IMU 积分） ======
for (每对相邻 IMU 数据) {
    // 减去偏差
    angvel_avr -= state.bias_g;
    acc_avr = acc_avr * 9.81 / mean_acc.norm() - state.bias_a;

    // 状态传播
    R_imu = R_imu * Exp(angvel_avr, dt);
    acc_imu = R_imu * acc_avr + gravity;
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt^2;
    vel_imu = vel_imu + acc_imu * dt;

    // 记录每个时刻的位姿
    IMUpose.push_back(Pose6D(dt, acc, gyr, vel, pos, R));
}

// ====== 步骤2：后向传播（点云去畸变） ======
for (每个点，从后往前) {
    // 找到该点对应的 IMU 时刻
    dt = point.time - imu_pose.time;

    // 插值得到该时刻的位姿
    R_i = R_imu * Exp(angvel, dt);
    T_ei = pos + vel * dt + 0.5 * acc * dt^2 - 最终位置;

    // 将点变换到帧末尾时刻
    point_compensated = R_end^{-1} * (R_i * point + T_ei);
}
```

---

## 八、LVIO 紧耦合分析

### 8.1 核心问题：LIO 和 VIO 是先后执行还是同时执行？

**答案：是先后执行，但共享同一个状态向量和协方差矩阵。**

```
时间线：
  LiDAR 扫描期间：IMU 持续积分
        ↓
  LiDAR 扫描结束：LIO EKF 更新（状态 + 协方差更新）
        ↓
  图像到来：VIO EKF 更新（基于 LIO 更新后的状态 + 协方差）
        ↓
  下一帧 LiDAR：基于 VIO 更新后的状态继续
```

### 8.2 紧耦合的三个层次

#### 层次1：状态共享

LIO 和 VIO 更新**同一个 19 维状态向量**：

```
StatesGroup {
    rot_end,        // 旋转 ← LIO 更新 + VIO 更新
    pos_end,        // 位置 ← LIO 更新 + VIO 更新
    inv_expo_time,  // 曝光 ← 仅 VIO 更新
    vel_end,        // 速度 ← 仅 LIO 更新（VIO 不直接观测速度）
    bias_g,         // 陀螺仪偏差 ← 仅 LIO 更新
    bias_a,         // 加速度计偏差 ← 仅 LIO 更新
    gravity,        // 重力 ← 仅 LIO 更新
    cov             // 协方差 ← LIO 更新 + VIO 更新
}
```

#### 层次2：协方差传播

LIO 更新后的协方差矩阵 P 会传递给 VIO：

```
LIO EKF 更新后：
  P_lio = (I - G_lio) * P_pred

VIO EKF 更新时：
  K_vio = P_lio * H_vio^T * (H_vio * P_lio * H_vio^T + R_vio)^{-1}
  P_vio = (I - K_vio * H_vio) * P_lio
```

**关键点**：VIO 使用的是 LIO 更新后的协方差，这意味着：
- 如果 LIO 约束强（协方差小），VIO 的修正量就小
- 如果 LIO 约束弱（协方差大），VIO 的修正量就大

#### 层次3：互补约束

| 状态 | LIO 约束 | VIO 约束 | 互补性 |
|------|---------|---------|--------|
| 旋转 | 点云匹配（弱） | 图像光度（强） | VIO 补充旋转约束 |
| 位置 | 点云匹配（强） | 图像光度（中） | LIO 补充位置约束 |
| 速度 | IMU 积分 + 轮速 | 无直接约束 | 仅 LIO 约束 |
| 偏差 | IMU 预测 | 无直接约束 | 仅 LIO 约束 |
| 重力 | IMU 初始化 | 无直接约束 | 仅 LIO 约束 |
| 曝光 | 无 | 光度误差 | 仅 VIO 约束 |

### 8.3 紧耦合 vs 松耦合

**松耦合**：LIO 和 VIO 各自独立估计位姿，然后融合

```
LIO → 位姿1
VIO → 位姿2
融合 → 最终位姿（加权平均）
```

**紧耦合（FAST-LIVO2）**：LIO 和 VIO 共享状态，交替更新

```
IMU 预测 → 状态 x, 协方差 P
    ↓
LIO 更新 → x', P'（使用点云匹配）
    ↓
VIO 更新 → x'', P''（使用图像光度，基于 LIO 更新后的 P'）
```

**紧耦合的优势**：
1. **信息不丢失**：LIO 的不确定性信息（协方差）传递给 VIO
2. **互补约束**：旋转弱约束（LIO）+ 强约束（VIO）= 更好的旋转估计
3. **实时性**：不需要等待两个系统各自收敛

### 8.4 数据同步机制

```cpp
// LIVMapper.cpp sync_packages()

// 以图像捕获时间为界，分割 LiDAR 点云
switch (last_lio_vio_flg) {
    case WAIT/VIO:
        // 等待 LiDAR 和图像时间对齐
        // 将 LiDAR 点云按图像时间分割为两部分：
        //   pcl_proc_cur：图像时间之前的点（用于 LIO）
        //   pcl_proc_next：图像时间之后的点（留给下一帧）

        meas.lio_vio_flg = LIO;
        break;

    case LIO:
        // LIO 完成后，处理 VIO
        meas.lio_vio_flg = VIO;
        meas.img = img_buffer.front();
        break;
}
```

**时序图**：

```
LiDAR:  |----scan1----|----scan2----|----scan3----|
Image:        |img1|        |img2|        |img3|
IMU:   ||||||||||||||||||||||||||||||||||||||||||||||||

处理顺序：
1. LiDAR 到达 → 按图像时间分割点云
2. LIO 更新（使用图像时间之前的点 + IMU）
3. VIO 更新（使用当前图像 + LIO 更新后的状态）
4. 下一帧 LiDAR → 重复
```

---

## 九、与 lio_odometry 的关键差异

| 特性 | FAST-LIVO2 | lio_odometry |
|------|-----------|--------------|
| **地图结构** | VoxelMap（八叉树+平面） | iVox（体素+近邻搜索） |
| **残差类型** | 点到平面（带不确定性） | 点到平面 |
| **R 自适应** | 是（平面+点不确定性） | 否（固定 R） |
| **视觉融合** | 直接法 VIO | 无 |
| **曝光估计** | 是（inv_expo_time） | 无 |
| **八叉树分裂** | 是（max_layer） | 否 |
| **平面协方差** | 是（6x6 plane_var） | 否 |
| **地图滑动** | 是（mapSliding） | 是（map_trim） |
| **状态维度** | 19 维 | 23 维（含外参） |
| **外参在线估计** | 否 | 可选 |

---

## 十、对 lio_odometry 优化的启示

| FAST-LIVO2 特性 | 可借鉴到 lio_odometry |
|----------------|---------------------|
| 平面参数协方差 | 计算每个体素的 plane_var，用于自适应 R |
| 自适应 R | R = f(plane_var, point_var) |
| 不确定性传播 | 将平面拟合的不确定性传播到残差 |
| 八叉树分裂 | 在 iVox 中增加多分辨率支持 |
| 地图滑动 | 类似 map_trim，但按距离而非帧数 |
| 退化检测 | 通过特征值分析检测退化方向 |

---

## 附录：关键配置参数

```yaml
# lio 参数
lio:
  voxel_size: 0.5          # 体素大小（米）
  max_layer: 2             # 八叉树最大层数
  max_points_num: 50       # 体素内最大点数
  layer_init_num: [5,5,5,5,5]  # 每层初始化所需点数
  min_eigen_value: 0.0025  # 平面判断阈值（特征值）
  dept_err: 0.02           # 深度测量噪声
  beam_err: 0.05           # 波束方向噪声
  sigma_num: 3.0           # 匹配门限（sigma 倍数）
  max_iterations: 5        # EKF 最大迭代次数

# vio 参数
vio:
  patch_size: 8            # 图像块大小
  patch_pyrimid_level: 4   # 金字塔层数
  normal_en: true          # 使用法向量约束
  exposure_estimate_en: true  # 曝光时间估计
  outlier_threshold: 1000  # 外点门限
  img_point_cov: 100       # 图像点协方差

# imu 参数
imu:
  acc_cov: 0.5             # 加速度计噪声
  gyr_cov: 0.3             # 陀螺仪噪声
  b_acc_cov: 0.0001        # 加速度计偏差过程噪声
  b_gyr_cov: 0.0001        # 陀螺仪偏差过程噪声

# 局部地图滑动
local_map:
  map_sliding_en: true     # 启用地图滑动
  half_map_size: 100       # 地图半径（体素数）
  sliding_thresh: 8        # 滑动触发阈值（米）
```
