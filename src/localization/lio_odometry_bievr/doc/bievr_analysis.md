# BIEVR-LIO 核心机制深度分析

> 分析日期：2026-09-18
> 分析目标：BIEVR-LIO 的凹凸高度图（Bump Image）构建、更新、配准残差与采样策略

---

## 一、整体架构概览

BIEVR-LIO 的核心创新是将传统 LIO 的**点对面**残差替换为**点对凹凸高度图**的残差。其系统流程图如下：

```
IMU + LiDAR 原始数据
    │
    ├─ IMU 预积分 → 状态预测
    │
    ├─ 点云去畸变 (IMU 补偿)
    │
    ├─ Map-Informed 采样 ──────┐
    │                          │
    ├─ LM 配准 ─────────────────┤
    │   ├─ 投影点到体素局部平面  │
    │   ├─ 双线性插值 bump 值   │
    │   ├─ 残差: r = p_z - I(u,v)
    │   └─ 雅可比: SE3 + 图像梯度
    │                          │
    └─ 地图更新 ────────────────┘
        └─ 凹凸图构建 / 更新
```

核心代码路径：
- [bievr_map.h](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/bievr_map.h) / [bievr_map.cpp](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp) — 凹凸图构建与更新
- [ls_optimizer.h](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/ls_optimizer.h) / [ls_optimizer.cpp](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/ls_optimizer.cpp) — 配准优化
- [preprocess.cpp](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/preprocess.cpp) — Map-Informed 采样

---

## 二、体素数据结构 (Voxel)

**定义位置**：[bievr_map.h:L17-L31](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/bievr_map.h#L17-L31)

```cpp
struct Voxel {
  bool observed_{false};               // 体素是否已拟合出平面法向量
  Transform T_C_W_ = Identity();       // 世界→局部相机坐标系（投影变换）
  Transform T_O_W_ = Identity();       // 世界→局部平面坐标系（正交基）
  Eigen::MatrixXf bump_img_;           // 凹凸高度图 (H×W, 存储高度偏移)
  Eigen::MatrixXf bump_smoothed_;      // 高斯平滑后的凹凸图
  Eigen::MatrixXf bump_weights_;       // 每个像素的观测权重
  M3 outer_sum_ = Eigen::Matrix3d::Zero();  // ∑(p·p^T)，PCA 用
  V3 sum_ = Eigen::Vector3d::Zero();        // ∑p，PCA 用
  size_t num_points_{0};               // 累积点数
  double mean_img_dist_{0.0};          // 平均凹凸高度（用于采样评分）
  std::vector<Eigen::Vector4d> pending_points_;  // 未观测时的缓冲点
};
```

### 2.1 坐标系定义

每个体素维护两个局部坐标系：

1. **`T_O_W_` — 正交基坐标系（Plane Frame）**：
   - Z 轴 = 拟合平面的法向量（PCA 最小特征值方向）
   - X/Y 轴 = 平面内的两个正交方向
   - 原点 = 点云质心

2. **`T_C_W_` — 相机坐标系（Camera Frame）**：
   - 与 `T_O_W_` 共原点
   - 在 `T_O_W_` 基础上做了平移，使得图像原点与体素角点对齐
   - 用于将世界点 → 像素坐标 (u, v)

变换关系：

```
p_O = T_O_W_ * p_W         // 世界点 → 局部平面坐标
p_C = T_C_W_ * p_W         // 世界点 → 相机坐标
(u, v) = (p_C.x, p_C.y)    // 像素坐标（需要乘以 inv_px_size）
bump_value = p_C.z          // 理论上应等于对应像素的 bump 值
```

---

## 三、凹凸高度图（Bump Image）构建

### 3.1 主流程

**入口函数**：[bievr_map.cpp:L97-L162](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L97-L162) `BIEVRMap::integratePoints()`

处理流程：

1. **并行哈希计算**：对每个点计算 `hashIndex(p)`，确定所属体素
2. **并行排序分组**：按 hash 排序，提取唯一体素
3. **逐体素更新**（并行）：
   - `updateNormal()` — PCA 法向量估计
   - `updateBumpImage()` — 凹凸图更新（核心）

### 3.2 法向量估计

**函数**：[bievr_map.cpp:L202-L240](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L202-L240) `BIEVRMap::updateNormal()`

```cpp
// PCA 协方差矩阵
M3 covariance = (outer_sum - sum * mean.transpose()) / (num_points - 1.0);
Eigen::SelfAdjointEigenSolver<M3> solver(covariance);
Eigen::Vector3d normal = solver.eigenvectors().col(0);  // 最小特征值 → 法向量
```

**法向量变化检测**：如果新法向量与旧法向量夹角 > `norm_tol_rad_`（默认 3°），触发 `update_normal = true`，导致凹凸图重投影。

局部坐标系构建：

```cpp
Eigen::Vector3d z_axis = normal;                          // 法向量 → Z
Eigen::Vector3d x_axis = z_axis.unitOrthogonal();          // 任意正交方向 → X
Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized(); // Y = Z × X
```

### 3.3 凹凸图更新 — 正常情况

**函数**：[bievr_map.cpp:L242-L275](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L242-L275) `BIEVRMap::updateBumpImage()`

当法向量**未变化**时，只需将新点投影到凹凸图：

```cpp
// 1. 计算图像尺寸
ImageBounds bounds = computeImageSize(voxel, reference_point);

// 2. 初始化/重设图像尺寸
voxel.bump_img_.resize(bounds.height, bounds.width);
voxel.bump_weights_.resize(bounds.height, bounds.width);
voxel.bump_img_.setZero();
voxel.bump_weights_.setZero();

// 3. 逐点投影
for (const auto& p : points) {
    Point p_O = voxel.T_C_W_ * p;          // → 相机坐标
    int x = round(p_O.x() * inv_px_size);   // → 像素坐标 x
    int y = round(p_O.y() * inv_px_size);   // → 像素坐标 y

    // 加权滑动平均更新像素值
    double weight_new = weighted ? min(0.5, 1/range) : 1.0;
    voxel.bump_weights_(y, x) += weight_new;
    voxel.bump_img_(y, x) = (old_mean * old_weight + p_O.z() * weight_new)
                            / new_weight;
}

// 4. 高斯平滑
maskedGaussianSmooth(...);  // 仅对 changed 像素做 3×3 高斯平滑

// 5. 计算评分
computeScore(voxel);  // 计算 mean_img_dist_ = 平均 |bump|
```

### 3.4 凹凸图重投影 — 法向量变化时

**函数**：[bievr_map.cpp:L316-L353](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L316-L353) `BIEVRMap::reprojectImage()`

当法向量变化 >3° 时，**不能简单地丢弃旧凹凸图**，因为旧图包含历史观测信息。重投影的逻辑：

```
旧图像 bump_original (在旧平面坐标系中)
    ↓
将每个有效像素 (i,j) 提升为 3D 点：
    p_old_O = (j*px_size, i*px_size, bump_original(i,j))
    ↓
变换到新相机坐标系：
    p_new_C = T_C1_C0 * p_old_O
    ↓
投影到新像素坐标：
    x_new = round(p_new_C.x * inv_px_size)
    y_new = round(p_new_C.y * inv_px_size)
    ↓
更新新图像相应像素
```

其中 `T_C1_C0 = T_C_W_new * T_W_C_old` 是两个相机坐标系的相对变换。

### 3.5 图像尺寸计算

**函数**：[bievr_map.cpp:L277-L295](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/bievr_map.cpp#L277-L295) `BIEVRMap::computeImageSize()`

图像尺寸根据体素**八个角点投影到平面后的范围**来确定：

```cpp
// 体素八个角点在局部平面上的 (u,v) 范围
Eigen::MatrixXd uv = T_O_W_.matrix().block<2,3>(0,0) * voxel_corners
                   + T_O_W_.matrix().block<2,1>(0,3).replicate(1,8);
u_min = uv.row(0).minCoeff(), u_max = uv.row(0).maxCoeff();
v_min = uv.row(1).minCoeff(), v_max = uv.row(1).maxCoeff();

// 图像宽高 = ceil((max - min) / pixel_size) + 1
width = ceil((u_max - u_min) * inv_px_size) + 1;
height = ceil((v_max - v_min) * inv_px_size) + 1;
```

**参数配置**（[params.yaml](file:///home/hhy/2004ros/BIEVR-LIO/config/params.yaml)）：

```yaml
map:
  pixel_size_m: 0.05    # 每个像素代表 5cm
  voxel_size_m: 0.5     # 体素大小 0.5m
```

因此一个体素的典型图像尺寸 ≈ `(0.5/0.05 + 1) × (0.5/0.05 + 1) = 11×11` 像素。

---

## 四、配准残差与雅可比

### 4.1 残差设计

**定义位置**：[ls_optimizer.cpp:L72-L122](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/ls_optimizer.cpp#L72-L122) `LsqRegistration::linearize()`

**残差公式**：

\[
r = p_o^z - I(u, v)
\]

其中：
- \(p_o = T_{C/W} \cdot p_W\) = 世界点变换到局部相机坐标系
- \(p_o^z\) = 点在局部坐标系下的高度
- \(I(u,v)\) = 凹凸图的**双线性插值**高度值
- \(u = p_o^x \cdot \text{inv\_px\_size}\)
- \(v = p_o^y \cdot \text{inv\_px\_size}\)

**核心思想**：将体素表面建模为 \(z = I(u,v)\) 的函数，残差衡量当前点偏离这个曲面的程度。

### 4.2 双线性插值

**函数**：[ls_optimizer.h:L49-L75](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/ls_optimizer.h#L49-L75) `getSubPixelValue()`

```cpp
// 像素坐标 (x,y) 的浮点值分解
x0 = floor(x), y0 = floor(y)
x1 = x0 + 1,   y1 = y0 + 1
dx = x - x0,   dy = y - y0
dx1 = 1 - dx,  dy1 = 1 - dy

// 4 个角点权重（考虑有效掩码 v00~v11）
w0 = dx1 * dy1 * v00,  w1 = dx * dy1 * v01
w2 = dx1 * dy * v10,   w3 = dx * dy * v11

value = (w0*M00 + w1*M01 + w2*M10 + w3*M11) / wsum
```

**关键细节**：BIEVR 使用**带有效掩码的加权双线性插值**。如果某个角点的权重为 0（无观测），该角点不参与插值，权重归一化到有效角点。

### 4.3 图像梯度计算

**函数**：[ls_optimizer.h:L81-L140](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/ls_optimizer.h#L81-L140) `sampleValueAndGradient()`

使用中心差分计算图像梯度 \( \frac{\partial I}{\partial u}, \frac{\partial I}{\partial v} \)：

```cpp
// 计算 dIdx：在 x 方向上取前一点 (x0-1, y) 和后一点 (x1+1, y)
// 对差值做双线性插值得到 vm 和 vp
dIdx = 0.5 * (vp - vm)    // 等价于中央差分
```

### 4.4 雅可比矩阵

**雅可比推导** [ls_optimizer.cpp:L103-L117](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/ls_optimizer.cpp#L103-L117)：

```cpp
// SE3 对局部坐标的雅可比（3×6）
SE3_Jac.block<3,3>(0, 3) = R_o_j;           // d p_o / d t
SE3_Jac.block<3,3>(0, 0) = -R_o_j * skew(p_j);  // d p_o / d θ

// 图像梯度雅可比（1×2）
I_Jac(0, 0) = dIdx;     // ∂I/∂u
I_Jac(0, 1) = dIdy;     // ∂I/∂v
I_Jac *= inv_px_size;   // 归一化到米单位

// 总雅可比（1×6）
// 残差 r = p_o.z - I(u,v)
// J = ∂r/∂ξ = ∂p_o.z/∂ξ - ∂I/∂u * ∂u/∂p_o * ∂p_o/∂ξ - ∂I/∂v * ∂v/∂p_o * ∂p_o/∂ξ
//   = SE3_Jac.row(2) - I_Jac * SE3_Jac.topRows<2>()
J = SE3_Jac.row(2) - (I_Jac * SE3_Jac.topRows<2>());
```

数学展开：

\[
\mathbf{J} = \frac{\partial r}{\partial \boldsymbol{\xi}} =
\frac{\partial p_o^z}{\partial \boldsymbol{\xi}} -
\frac{\partial I}{\partial u} \cdot \frac{\partial u}{\partial p_o} \cdot \frac{\partial p_o}{\partial \boldsymbol{\xi}} -
\frac{\partial I}{\partial v} \cdot \frac{\partial v}{\partial p_o} \cdot \frac{\partial p_o}{\partial \boldsymbol{\xi}}
\]

其中：
- \(\frac{\partial p_o}{\partial \boldsymbol{\xi}} = \mathbf{SE3\_Jac}\) （由 SE3 左扰动模型得出）
- \(\frac{\partial u}{\partial p_o} = [\text{inv\_px\_size}, 0, 0]\)
- \(\frac{\partial v}{\partial p_o} = [0, \text{inv\_px\_size}, 0]\)

### 4.5 优化器

BIEVR 使用 **Levenberg-Marquardt** 算法（而非 IEKF）：

```cpp
// LM 步长求解
Eigen::LDLT<Matrix66> solver(H + lambda * I);
Vector6 d = solver.solve(-b);

// 更新姿态
delta.linear() = so3_exp(d.head<3>());  // SO3 指数映射
delta.translation() = d.tail<3>();
x0 = x0 * delta;  // 左乘更新
```

Huber 核函数用于鲁棒处理：

```cpp
double w = (abs(r) <= huber_delta) ? 1.0 : huber_delta / abs(r);
// 加权后：H += w * J^T * J,  b += w * J * r
```

---

## 五、Map-Informed 采样策略

**函数**：[preprocess.cpp:L83-L168](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/src/preprocess.cpp#L83-L168) `sampleInformed()`

### 5.1 采样评分

每个体素的评分 = `mean_img_dist_` = 体素凹凸图所有有效像素的 `|bump|` 均值。

物理含义：体素表面的**粗糙程度/纹理丰富度**。

- 平坦地面 → bump ≈ 0 → score ≈ 0
- 粗糙墙面 → bump 有明显起伏 → score > 0

### 5.2 采样策略

```
输入：降采样后的点云 (voxel_size=0.1m)
输出：fine_cloud + coarse_cloud

1. 对每个点，用先验位姿投影到世界坐标系
2. 查找所属体素，获取体素评分
3. 按评分降序排序体素
4. 取 top-K 体素（K = n_samples）：
   - 这些体素内的所有点 → fine_cloud（保留细节）
5. 其余体素：
   - 每个体素只取一个点 → coarse_cloud（保持覆盖）
```

**参数**（[pipeline.h:L55](file:///home/hhy/2004ros/BIEVR-LIO/BIEVR/include/bievr_lio/pipeline.h#L55)）：

```cpp
informed_sample_count = 300;  // 默认 300 个体素被选为 "fine"
```

**实际效果**：退化场景（隧道）中，墙面纹理区域被密集采样，平坦地面被稀疏采样。既保证约束又控制计算量。

---

## 六、与 FAST-LIO2 VoxelMap 的核心差异

| 维度 | BIEVR-LIO | FAST-LIO2 VoxelMap (你的代码) |
|------|-----------|-------------------------------|
| **地图存储** | 每个体素存储凹凸图 + 权重 + 平面法向量 | 每个体素存储平面参数 + 协方差 |
| **残差类型** | 点对凹凸图高度差 | 点对平面距离 |
| **残差公式** | \(r = p_z - I(u,v)\) | \(r = \mathbf{n}^T \mathbf{p} + d\) |
| **雅可比** | SE3 + 图像梯度 | 仅 SE3 × 法向量 |
| **优化器** | Levenberg-Marquardt | IEKF（迭代扩展卡尔曼滤波） |
| **采样** | Map-Informed（凹凸梯度引导） | 均匀体素降采样 |
| **法向量更新** | PCA 重新拟合 | 递推更新协方差 |
| **平面变化处理** | >3° 触发凹凸图重投影 | 无此概念 |
| **滑动窗口** | LRU 缓存（最大体素数限制） | LRU + 距离滑动窗口 |
| **八叉树** | 无（仅体素哈希） | 有（八叉树细分） |

---

## 七、关键参数与配置

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `voxel_size_m` | 0.5m | 体素大小 |
| `pixel_size_m` | 0.05m | 凹凸图像素大小（5cm） |
| `normal_tolerance_deg` | 3° | 法向量变化阈值，超过则触发重投影 |
| `smooth` | true | 是否对凹凸图做高斯平滑 |
| `weighted` | true | 是否使用距离加权更新 |
| `huber_delta` | 0.1 | Huber 核函数阈值 |
| `informed_sampling` | true | 是否启用 Map-Informed 采样 |
| `downsample_resolution` | 0.1m | 预处理降采样分辨率 |
| `informed_sample_count` | 300 | 保留为 fine 的体素数 |

---

## 八、总结：BIEVR 成功的关键设计

1. **凹凸图作为显式曲面模型**：比平面模型多了一个维度（高度变化），在退化场景中提供额外约束，在平坦场景中自动退化回平面效果。

2. **重投影机制保证一致性**：当平面法向量变化时，旧观测信息通过重投影保留在新坐标系中，不丢失历史信息。

3. **双线性插值 + 有效掩码**：使得凹凸图在稀疏观测下仍然可用，无效像素通过掩码机制不参与配准。

4. **Map-Informed 采样**：自动将计算资源集中到信息丰富的区域，是 BIEVR 在保持实时性的同时提升精度的关键。

5. **LM 配准 + Huber 鲁棒核**：相比 IEKF，LM 在处理高度非线性残差（凹凸图梯度突变）时更稳定。