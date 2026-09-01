# DCReg 退化检测 + Huber 鲁棒估计 集成设计方案

## 1. 目标

在 `slam_odometry_v2` 中引入两个能力，提升 LiDAR 在稀疏/退化结构（隧道、长廊、开阔地）中的匹配鲁棒性：

| 模块 | 作用 | 来源 |
|------|------|------|
| **DCReg 退化检测** | 基于 Schur 补解耦旋转/平移 Hessian，诊断每个自由度 (roll/pitch/yaw/x/y/z) 的退化程度 | [DCReg](https://github.com/JokerJohn/DCReg) |
| **Huber M-估计** | 对点面残差做鲁棒加权，抑制离群点对优化器的影响 | Point-LIO / FAST-LIVO2 |

DCReg 的诊断结果用于两处：
1. **观测模型** (`ObsModel`)：退化方向上的激光残差自动降权 (`w_degen`)
2. **iESKF 层**：退化方向上的状态协方差 P 做定向膨胀，让 GNSS/轮速观测有足够话语权

---

## 2. 现有架构分析

### 2.1 核心流程

```
Run()
  ├── SyncPackages()           // IMU-LiDAR 同步
  ├── p_imu_->Process()        // IMU 预测 + 去畸变
  ├── ApplyVelocityGnssAid()   // 轮速/GNSS 辅助校正（预更新）
  ├── downsample               // 体素滤波
  ├── kf_.update_iterated_dyn_share_modified()  ← 核心 IEKF 迭代
  │     └── 内部循环调用 ObsModel()  ← 每轮 GN 迭代调用一次
  │           ├── 最近邻搜索 + 平面拟合
  │           ├── 构建 h_x (N×12 Jacobian)  ← 12 列: [pos, rot, off_R, off_T]
  │           └── 构建 h   (N×1 残差)
  ├── 退化检测 & P 膨胀 (已注释掉)
  ├── state_point_ = kf_.get_x()
  └── MapIncremental()
```

### 2.2 IEKF 框架内部（不可修改）

`update_iterated_dyn_share_modified` 在 `esekfom.hpp` 中实现，关键逻辑：

```cpp
for (int i = -1; i < maximum_iter; i++) {
    h_dyn_share(x_, dyn_share);  // → 调用注册的回调 (lambda)

    // 组装 HTH = h_x^T * h_x (12×12)
    Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_;

    // 更新状态: P^{-1} += HTH / R
    cov P_temp = (P_ / R).inverse();
    P_temp.template block<12, 12>(0, 0) += HTH;

    // 求解 dx = P_temp^{-1} * h_x^T * h
    // 更新状态 x_.boxplus(dx)
}
```

**关键约束**：`ObsModel` 是通过 `InitWithoutROS()` 中注册的 lambda 回调调用的，不是硬编码的。因此**可以替换回调函数，无需修改 IEKF 框架**。

### 2.3 IEKF 回调注册机制（关键）

在 `InitWithoutROS()` 中：

```cpp
kf_.init_dyn_share(
    get_f, df_dx, df_dw,
    [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { 
        ObsModel(s, ekfom_data);  // ← lambda，可替换
    },
    options::NUM_MAX_ITERATIONS, epsi.data());
```

`h_dyn_share` 就是这里注册的 lambda。**这天然支持替换为另一个观测模型函数，是实现"零侵入"的关键。**

### 2.4 现有的退化检测（已注释）

```cpp
// 当前在 Run() 中有被注释掉的退化检测：
// DegeneracyInfo degeneracy = DetectDegeneracy();
// InflateDegenerateP(degeneracy);
```

现有 `DetectDegeneracy()` 基于**法向量协方差分析**（简单启发式），`InflateDegenerateP()` 在退化方向上膨胀 P 矩阵。方案将用 DCReg 替代这套逻辑。

---

## 3. 设计方案：包装函数法（推荐）

### 3.1 核心思路：新建 `ObsModelWithDCReg`，通过 lambda 开关切换

**原 `ObsModel()` 一行不改**。新建 `ObsModelWithDCReg()` 包装原函数，在返回后做后处理（DCReg + 加权）。通过 lambda 中的 bool 开关选择调用哪个函数。

```
ObsModel()                                // ← 完全不改
  ├── [原有] 最近邻搜索 + 平面拟合
  ├── [原有] 收集 corr_pts_, corr_norm_
  └── [原有] 构建 ekfom_data.h_x, ekfom_data.h

ObsModelWithDCReg(s, ekfom_data)          // ← 全新函数
  ├── ObsModel(s, ekfom_data)             // ① 复用原逻辑
  └── ApplyDCRegAndWeights(ekfom_data)    // ② 后处理
        ├── 提取 J_6 = h_x[:, 0:6]
        ├── H_6 = J_6^T · J_6
        ├── DCReg(H_6) → dcreg_result_
        ├── 逐点: w = Huber(r) × DeGen(J_row)
        └── h_x *= √w, h *= √w

Run()                                     // ← 仅加 3 行
  ├── kf_.update_iterated_dyn_share_modified()  // 内部调用 ObsModelWithDCReg
  ├── [新增] if (dcreg_result_.is_degenerate):
  │           InflateDegeneratePFromDCReg(dcreg_result_)
  ├── state_point_ = kf_.get_x()
  └── MapIncremental()
```

### 3.2 为什么用包装函数而不是修改原函数

| 维度 | 修改原 `ObsModel` | 新建 `ObsModelWithDCReg` 包装 |
|------|-------------------|-------------------------------|
| **ObsModel 改动** | ~40 行插入 | **0 行** |
| **回归风险** | 需要验证原功能不受影响 | 原功能完全不动，零风险 |
| **A/B 测试** | 需要改代码注释 | 一个 bool 开关 |
| **可扩展性** | 后续变体需继续改 ObsModel | 可加 `ObsModelWithCauchy` 等 |
| **代码可读性** | 开关逻辑散落在函数内部 | 功能隔离，职责清晰 |
| **性能开销** | 无额外函数调用 | 多一次函数调用（ObsModel 内部两个 `Timer::Evaluate` 才是耗时大头，这点开销可忽略） |

### 3.3 Lambda 注册改动（仅 5 行）

```cpp
// 原代码：
kf_.init_dyn_share(
    get_f, df_dx, df_dw,
    [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { 
        ObsModel(s, ekfom_data); 
    },
    options::NUM_MAX_ITERATIONS, epsi.data());

// 改为：
kf_.init_dyn_share(
    get_f, df_dx, df_dw,
    [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { 
        if (use_dcreg_degeneracy_) {
            ObsModelWithDCReg(s, ekfom_data);
        } else {
            ObsModel(s, ekfom_data);
        }
    },
    options::NUM_MAX_ITERATIONS, epsi.data());
```

### 3.4 数据流验证

**关键问题**：`ObsModelWithDCReg` 调用 `ObsModel` 后，后处理修改 `ekfom_data.h_x` 和 `ekfom_data.h`，IEKF 框架能正确工作吗？

**验证**：框架在每次 `h_dyn_share(x_, dyn_share)` 返回后，直接从 `dyn_share.h_x` 和 `dyn_share.h` 读取数据，组装 `HTH = h_x^T · h_x` 和 `K_h = h_x^T · h`。加权等价于 IRLS（迭代重加权最小二乘）：

```
HTH_weighted = Σ_i (√w_i · J_i)^T (√w_i · J_i) = Σ_i w_i · J_i^T · J_i
```

**✓ 完全正确。**

**副作用**：`ObsModel` 内部设置的 `effect_feat_num_`、`corr_pts_`、`corr_norm_` 等成员变量正常写入，`MapIncremental()` 等后续使用不受影响。**✓ 无问题。**

### 3.5 为何权重用 `sqrt(w)` 乘到 h_x 和 h

IEKF 内部组装 `HTH = h_x^T * h_x`。如果对每行 `h_x.row(i)` 乘以 `sqrt(w_i)`，则：

```
HTH_weighted = Σ_i (√w_i · J_i)^T (√w_i · J_i) = Σ_i w_i · J_i^T · J_i
```

等价于对每个点施加权重 `w_i`，即 IRLS（迭代重加权最小二乘）。**不修改 IEKF 框架，只修改输入数据。**

### 3.6 权重计算公式

#### 3.6.1 Huber 权重

对点面残差 `r_i`（单位：米）：

```
w_huber(r_i) = 1,              if |r_i| ≤ k
               k / |r_i|,      if |r_i| > k
```

- `k`：Huber 阈值，建议 **0.5m**（室外）或 **0.2m**（室内）
- 效果：残差 > k 的点被降权，抑制离群点

#### 3.6.2 退化降权

对每个点，其 6 维 Jacobian 行向量 `J_i ∈ R^{1×6}` 表示该点对 6 个自由度的约束贡献。DCReg 给出每个自由度的退化分数 `s_d ∈ [0, 1]`（d = 0..5 对应 roll/pitch/yaw/x/y/z）。

点 i 在退化方向 d 上的归一化贡献：

```
contrib_i(d) = |J_i(d)| / ||J_i||
```

退化惩罚：

```
penalty_i = Σ_{d: s_d > 0.01} s_d · contrib_i(d)
```

退化降权：

```
w_degen_i = 1.0 / (1.0 + α · penalty_i)
```

- `α`：降权强度，建议 **0.5**（温和）~ **2.0**（激进）
- 下限截断：`max(0.1, w_degen_i)`，避免完全丢弃

**直觉**：如果一个点的 Jacobian 在退化方向上有大分量，说明该点"试图"约束退化方向但不可靠，应降权。

#### 3.6.3 组合权重

```
w_final_i = w_huber_i · w_degen_i
```

然后将 `sqrt(w_final_i)` 乘到 `h_x.row(i)` 和 `h(i)`。

### 3.7 DCReg 退化诊断

#### 3.7.1 输入

从 `ObsModel` 返回后的 `ekfom_data.h_x` 中提取 6 维 Hessian：

```
J_6 = h_x.leftCols<6>()   // N×6, 列: [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]
H_6 = J_6^T · J_6          // 6×6 对称半正定
```

#### 3.7.2 DCReg 处理流程

```
H_6 (6×6)
  ├── DetectDegeneracy(H_6, params)
  │     ├── 全 Hessian SVD → cond_full
  │     ├── 分块: H_rr(3×3), H_rt(3×3), H_tt(3×3)
  │     ├── Schur 补旋转: S_rot = H_rr - H_rt · H_tt^{-1} · H_rt^T
  │     ├── Schur 补平移: S_trans = H_tt - H_rt^T · H_rr^{-1} · H_rt
  │     ├── 特征分解: S_rot = V_rot · Λ_rot · V_rot^T
  │     └── 特征分解: S_trans = V_trans · Λ_trans · V_trans^T
  │
  └── CharacterizeDegeneracy(detection, params)
        ├── 物理轴映射: 将 Schur 特征向量映射到 roll/pitch/yaw/x/y/z
        ├── 退化判定: λ_max / λ_i > threshold → 该方向退化
        └── 输出:
              - degenerate_mask[6]: 每个 DOF 是否退化
              - aligned_lambda[6]: 每个 DOF 的特征值
              - degeneracy_score[6]: 每个 DOF 的退化分数
```

#### 3.7.3 退化分数计算

```
对旋转 DOF (d=0,1,2):
    cond = λ_max_rot / λ_d
    s_d = clamp((cond - threshold) / threshold, 0, 1)

对平移 DOF (d=3,4,5):
    cond = λ_max_trans / λ_d
    s_d = clamp((cond - threshold) / threshold, 0, 1)
```

- `threshold`：退化条件数阈值，建议 **10.0**（DCReg 默认值）

### 3.8 iESKF P 矩阵定向膨胀

在 `Run()` 中，`update_iterated_dyn_share_modified` 返回后，根据 DCReg 结果膨胀 P：

```
P = kf_.get_P()  // 23×23

// 旋转块 (索引 3-5) 和 位置块 (索引 0-2)
对每个退化 DOF d:
    if degenerate_mask[d]:
        if d < 3:  // 旋转
            P_rot = P.block<3,3>(3,3)
            P_rot = V_rot · max(Λ_rot, Λ_target) · V_rot^T
            // Λ_target 对应 σ=2.0° 的不确定性
        else:      // 平移
            P_pos = P.block<3,3>(0,0)
            P_pos = V_pos · max(Λ_pos, Λ_target) · V_pos^T
            // Λ_target 对应 σ=0.5m 的不确定性

// 速度块 (索引 12-14): 旋转退化时连带膨胀
// （因为 IMU 传播中速度与旋转强耦合）
if any rot degenerate:
    P_vel = P.block<3,3>(12,12)
    P_vel eigenvalues clamped to σ=1.0 m/s

kf_.change_P(P)
```

**膨胀目标**：

| 自由度 | 膨胀目标 (σ) | 膨胀目标 (方差) | 原因 |
|--------|-------------|----------------|------|
| 旋转 (roll/pitch/yaw) | 2.0° | ~1.22×10⁻³ | 让 GNSS 姿态观测有话语权 |
| 平移 (x/y/z) | 0.5m | 0.25 | 让后续观测能修正位置 |
| 速度 (x/y/z) | 1.0 m/s | 1.0 | 旋转退化→速度积分不可靠→让轮速观测可修正 |

---

## 4. 文件改动清单

### 4.1 新增文件

| 文件 | 说明 |
|------|------|
| `include/faster_lio/dcreg_wrapper.h` | DCReg 轻量封装（仅依赖 Eigen） |

### 4.2 修改文件

| 文件 | 改动内容 | 改动量 |
|------|----------|--------|
| `include/faster_lio/laser_odometry.h` | 新增 `#include "dcreg_wrapper.h"`；新增 3 个方法声明（`ObsModelWithDCReg`、`ApplyDCRegAndWeights`、`InflateDegeneratePFromDCReg`）；新增 DCReg 成员变量 | ~25 行 |
| `src/faster_lio/laser_odometry.cc` | `InitWithoutROS()` 中 lambda 加 if-else（~5 行）；新增 3 个函数实现（~80 行）；`Run()` 中新增 P 膨胀调用（~3 行） | ~90 行 |
| `CMakeLists.txt` | 添加 DCReg include 路径 | ~2 行 |

### 4.3 不改动的文件

- `src/faster_lio/laser_odometry.cc` 中的 `ObsModel()` — **0 行改动**
- `IKFoM_toolkit/esekfom/esekfom.hpp` — **不修改 IEKF 框架**
- 其他所有现有文件 — 保持原样

---

## 5. 详细实现伪代码

### 5.1 `dcreg_wrapper.h`（新增）

```cpp
#ifndef FASTER_LIO_DCREG_WRAPPER_H
#define FASTER_LIO_DCREG_WRAPPER_H

#include <Eigen/Core>
#include <array>
#include <cmath>
#include "dcreg.hpp"  // DCReg 原版头文件

namespace faster_lio {

struct DCRegResult {
    bool is_degenerate = false;
    bool factorization_ok = false;
    // [roll, pitch, yaw, x, y, z]
    std::array<bool, 6> degenerate_mask = {false, false, false, false, false, false};
    std::array<double, 6> degeneracy_scores = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double cond_schur_rot = 0.0;
    double cond_schur_trans = 0.0;
};

// 核心函数：从 6×6 Hessian 计算退化诊断
inline DCRegResult ComputeDCReg(
    const Eigen::Matrix<double, 6, 6>& hessian,
    double degeneracy_threshold = 10.0);

// Huber 权重
inline double HuberWeight(double residual, double k);

// 退化降权：基于 Jacobian 行向量和退化分数
inline double DegeneracyWeight(
    const Eigen::Matrix<double, 1, 6>& J_row,
    const std::array<double, 6>& degeneracy_scores,
    double alpha);

}  // namespace faster_lio
#endif
```

### 5.2 `laser_odometry.h` 新增声明（~25 行）

```cpp
// 在 #include 区域新增：
#include "dcreg_wrapper.h"

// 在类声明中新增方法（放在现有 DetectDegeneracy / InflateDegenerateP 附近）：
class LaserOdometry {
    // ... 现有方法 ...

    /// DCReg 包装观测模型：调用原 ObsModel + DCReg 诊断 + 鲁棒加权
    void ObsModelWithDCReg(state_ikfom &s,
                           esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// DCReg 后处理：对 ekfom_data 做退化诊断 + Huber + 退化降权
    void ApplyDCRegAndWeights(
        esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// DCReg P 矩阵膨胀：在 DCReg 检测到的退化方向上进行定向协方差膨胀
    void InflateDegeneratePFromDCReg(const DCRegResult& dc_result);

    // ... 现有成员变量 ...

    ///////////////////////// DCReg 退化检测 & 鲁棒估计 ///////////////////////////
    bool use_dcreg_degeneracy_ = true;            ///< 是否启用 DCReg 退化检测
    bool use_huber_weight_ = true;                ///< 是否启用 Huber 鲁棒损失
    bool use_degeneracy_weight_ = true;           ///< 是否启用退化方向降权
    double dcreg_condition_threshold_ = 10.0;     ///< DCReg 退化条件数阈值
    double huber_k_ = 0.5;                        ///< Huber 损失阈值 (m)
    double degeneracy_weight_alpha_ = 0.5;        ///< 退化方向降权强度
    DCRegResult dcreg_result_;                    ///< 最新 DCReg 退化检测结果
};
```

### 5.3 `laser_odometry.cc` 新增函数实现（~80 行）

#### 5.3.1 `ObsModelWithDCReg()`

```cpp
void LaserOdometry::ObsModelWithDCReg(
    state_ikfom &s,
    esekfom::dyn_share_datastruct<double> &ekfom_data) {

    // ① 调用原观测模型
    ObsModel(s, ekfom_data);

    // ② 后处理：DCReg 诊断 + 鲁棒加权
    if (ekfom_data.valid) {
        ApplyDCRegAndWeights(ekfom_data);
    }
}
```

#### 5.3.2 `ApplyDCRegAndWeights()`

```cpp
void LaserOdometry::ApplyDCRegAndWeights(
    esekfom::dyn_share_datastruct<double> &ekfom_data) {

    if (effect_feat_num_ < 1) return;

    // 1. 提取 6-DOF Jacobian 并计算 Hessian
    Eigen::MatrixXd J_6 = ekfom_data.h_x.leftCols(6);  // N×6
    Eigen::Matrix<double, 6, 6> H_6 = J_6.transpose() * J_6;

    // 2. DCReg 退化诊断
    if (use_dcreg_degeneracy_) {
        dcreg_result_ = ComputeDCReg(H_6, dcreg_condition_threshold_);
    }

    // 3. 逐点计算权重并应用到 h_x 和 h
    for (int i = 0; i < effect_feat_num_; i++) {
        double w = 1.0;

        // Huber 鲁棒权重
        if (use_huber_weight_) {
            w *= HuberWeight(ekfom_data.h(i), huber_k_);
        }

        // 退化方向降权
        if (use_degeneracy_weight_ && dcreg_result_.is_degenerate) {
            Eigen::Matrix<double, 1, 6> J_row = J_6.row(i);
            w *= DegeneracyWeight(J_row, dcreg_result_.degeneracy_scores,
                                  degeneracy_weight_alpha_);
        }

        // 应用权重 (IRLS: w → √w)
        double sqrt_w = std::sqrt(w);
        ekfom_data.h_x.row(i) *= sqrt_w;
        ekfom_data.h(i) *= sqrt_w;
    }
}
```

#### 5.3.3 `InflateDegeneratePFromDCReg()`

```cpp
void LaserOdometry::InflateDegeneratePFromDCReg(const DCRegResult& dc_result) {
    Eigen::Matrix<double, 23, 23> P = kf_.get_P();

    constexpr double ROT_TARGET_SIGMA = 2.0 * M_PI / 180.0;  // 2.0°
    constexpr double ROT_TARGET_VAR = ROT_TARGET_SIGMA * ROT_TARGET_SIGMA;
    constexpr double POS_TARGET_SIGMA = 0.5;                  // 0.5m
    constexpr double POS_TARGET_VAR = POS_TARGET_SIGMA * POS_TARGET_SIGMA;
    constexpr double VEL_TARGET_SIGMA = 1.0;                  // 1.0 m/s
    constexpr double VEL_TARGET_VAR = VEL_TARGET_SIGMA * VEL_TARGET_SIGMA;

    bool any_rot_degenerate = false;

    // 旋转膨胀 (索引 3-5)
    if (dc_result.degenerate_mask[0] || dc_result.degenerate_mask[1] ||
        dc_result.degenerate_mask[2]) {
        any_rot_degenerate = true;
        Eigen::Matrix3d P_rot = P.block<3,3>(3,3);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_rot);
        Eigen::Vector3d lambda = es.eigenvalues();
        Eigen::Matrix3d V = es.eigenvectors();

        for (int i = 0; i < 3; i++) {
            if (lambda(i) < ROT_TARGET_VAR) {
                lambda(i) = ROT_TARGET_VAR;
            }
        }
        P.block<3,3>(3,3) = V * lambda.asDiagonal() * V.transpose();
    }

    // 平移膨胀 (索引 0-2)
    if (dc_result.degenerate_mask[3] || dc_result.degenerate_mask[4] ||
        dc_result.degenerate_mask[5]) {
        Eigen::Matrix3d P_pos = P.block<3,3>(0,0);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_pos);
        Eigen::Vector3d lambda = es.eigenvalues();
        Eigen::Matrix3d V = es.eigenvectors();

        for (int i = 0; i < 3; i++) {
            if (lambda(i) < POS_TARGET_VAR) {
                lambda(i) = POS_TARGET_VAR;
            }
        }
        P.block<3,3>(0,0) = V * lambda.asDiagonal() * V.transpose();
    }

    // 速度膨胀 (索引 12-14)：旋转退化时连带膨胀
    if (any_rot_degenerate) {
        Eigen::Matrix3d P_vel = P.block<3,3>(12,12);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(P_vel);
        Eigen::Vector3d lambda = es.eigenvalues();
        Eigen::Matrix3d V = es.eigenvectors();

        for (int i = 0; i < 3; i++) {
            if (lambda(i) < VEL_TARGET_VAR) {
                lambda(i) = VEL_TARGET_VAR;
            }
        }
        P.block<3,3>(12,12) = V * lambda.asDiagonal() * V.transpose();
    }

    kf_.change_P(P);

    LOG(WARNING) << "[DCReg] P inflated: mask=["
                 << dc_result.degenerate_mask[0] << ","
                 << dc_result.degenerate_mask[1] << ","
                 << dc_result.degenerate_mask[2] << ","
                 << dc_result.degenerate_mask[3] << ","
                 << dc_result.degenerate_mask[4] << ","
                 << dc_result.degenerate_mask[5] << "]"
                 << " cond_rot=" << dc_result.cond_schur_rot
                 << " cond_trans=" << dc_result.cond_schur_trans
                 << " scores=["
                 << dc_result.degeneracy_scores[0] << ","
                 << dc_result.degeneracy_scores[1] << ","
                 << dc_result.degeneracy_scores[2] << ","
                 << dc_result.degeneracy_scores[3] << ","
                 << dc_result.degeneracy_scores[4] << ","
                 << dc_result.degeneracy_scores[5] << "]";
}
```

### 5.4 `InitWithoutROS()` 改动（~5 行）

```cpp
// 原代码：
kf_.init_dyn_share(
    get_f, df_dx, df_dw,
    [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
        ObsModel(s, ekfom_data);
    },
    options::NUM_MAX_ITERATIONS, epsi.data());

// 改为：
kf_.init_dyn_share(
    get_f, df_dx, df_dw,
    [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
        if (use_dcreg_degeneracy_) {
            ObsModelWithDCReg(s, ekfom_data);
        } else {
            ObsModel(s, ekfom_data);
        }
    },
    options::NUM_MAX_ITERATIONS, epsi.data());
```

### 5.5 `Run()` 改动（~3 行）

```cpp
// 在 kf_.update_iterated_dyn_share_modified(...) 之后、state_point_ = kf_.get_x() 之前：
kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);

// ← 新增以下 3 行
if (use_dcreg_degeneracy_ && dcreg_result_.is_degenerate) {
    InflateDegeneratePFromDCReg(dcreg_result_);
}

state_point_ = kf_.get_x();
```

---

## 6. 配置参数

新增参数建议通过 yaml 配置文件控制，同时在代码中提供默认值：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_dcreg_degeneracy` | `true` | 是否启用 DCReg 退化检测（控制整个 DCReg 功能，含权重和 P 膨胀） |
| `use_huber_weight` | `true` | 是否启用 Huber 鲁棒损失 |
| `use_degeneracy_weight` | `true` | 是否启用退化方向降权 |
| `dcreg_condition_threshold` | `10.0` | DCReg 退化条件数阈值 |
| `huber_k` | `0.5` | Huber 损失阈值 (m) |
| `degeneracy_weight_alpha` | `0.5` | 退化方向降权强度 |

---

## 7. CMakeLists.txt 改动

```cmake
# 在 include_directories 中添加 DCReg 路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  /home/hhy/2004ros/DCReg/DCReg/include   # ← 新增
)
```

DCReg 是纯头文件库（header-only），只需添加 include 路径即可，无需链接。

---

## 8. 耦合度分析

| 维度 | 评估 |
|------|------|
| **IEKF 框架** | **零耦合** — 不修改 `esekfom.hpp`，只替换 lambda 回调 |
| **ObsModel 函数** | **零耦合** — 0 行改动，完全不动 |
| **现有退化检测** | **低耦合** — 新增 DCReg 路径，原有 `DetectDegeneracy()`/`InflateDegenerateP()` 保留不动 |
| **核心算法** | **零修改** — 点面匹配、最近邻搜索、IMU 预测等核心逻辑完全不改 |
| **DCReg 依赖** | **隔离在 `dcreg_wrapper.h`** — 如果未来移除 DCReg，只需删除该文件和 lambda 中的 if-else |
| **功能开关** | **可独立关闭** — `use_dcreg_degeneracy_` 控制整个 DCReg 功能；`use_huber_weight_`、`use_degeneracy_weight_` 可独立控制子功能 |

---

## 9. 与现有代码的兼容性

- 原有 `DetectDegeneracy()` 和 `InflateDegenerateP()` 保留不动，但保持注释状态
- **`ObsModel()` 一行不改**，原功能 100% 保持
- 新增的 DCReg 逻辑通过 `use_dcreg_degeneracy_` 开关控制，默认开启
- 当 `use_dcreg_degeneracy_ = false` 时，lambda 直接调用原 `ObsModel`，行为与改动前**完全一致**
- 所有新增代码在独立函数中，不干扰现有的并行循环

---

## 10. 潜在风险与缓解

| 风险 | 缓解措施 |
|------|----------|
| DCReg 每轮 GN 迭代都计算 EVD，增加计算量 | 6×6 EVD 极其轻量（< 0.01ms），可忽略 |
| 权重过早过度压制导致欠约束 | `w_degen` 下限截断 0.1，保留最低权重；Huber k 不宜过小 |
| P 膨胀过度导致滤波器发散 | 膨胀目标值保守（2.0° 旋转 / 0.5m 平移），仅膨胀到"有话语权"而非"主导" |
| DCReg 只能检测几何退化，无法检测欺骗式漂移 | 用户已明确知晓，需配合 IMU-LIO 一致性、GNSS 新息监控做兜底 |
| 多一次函数调用开销 | `ObsModel` 内部两个并行 `Timer::Evaluate` 才是耗时大头，额外函数调用开销 < 0.001ms，可忽略 |

---

## 11. 测试建议

1. **单元测试**：用已知退化场景（长廊 PCD）的 Hessian 验证 DCReg 输出正确
2. **A/B 对比测试**：在隧道/长廊/开阔地场景跑同一 bag，`use_dcreg_degeneracy_=true/false` 对比轨迹
3. **回归测试**：`use_dcreg_degeneracy_=false` 时，确保与原代码输出完全一致（逐帧 diff）
4. **正常场景测试**：在丰富几何结构场景确保 DCReg 不误报退化（`is_degenerate = false`）
5. **性能测试**：确认每帧额外耗时 < 0.5ms

---

## 12. 后续扩展

本方案预留了扩展空间：
- DCReg 的 `preconditioner` 可用于 IEKF 求解器（但需修改框架，暂不实现）
- 可添加 `ObsModelWithCauchy`、`ObsModelWithTukey` 等变体，通过 lambda 开关切换
- 可接入 `aligned_rot_basis`/`aligned_trans_basis` 做更精细的定向膨胀
- 退化降权可改为基于 Schur 特征向量投影的更精确版本