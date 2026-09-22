#ifndef DCREG_DCREG_HPP_
#define DCREG_DCREG_HPP_

#include "dcreg_utils.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <array>
#include <stdexcept>

namespace dcreg {

struct DegeneracyDetection {
  bool factorization_ok = false;
  double cond_full = std::numeric_limits<double>::quiet_NaN();
  double cond_schur_rot = std::numeric_limits<double>::quiet_NaN();
  double cond_schur_trans = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d lambda_schur_rot = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d lambda_schur_trans = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d raw_rot_basis = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d raw_trans_basis = Eigen::Matrix3d::Identity();
};

struct DegeneracyCharacterization {
  bool factorization_ok = false;
  bool is_degenerate = false;
  double cond_full = std::numeric_limits<double>::quiet_NaN();
  double cond_schur_rot = std::numeric_limits<double>::quiet_NaN();
  double cond_schur_trans = std::numeric_limits<double>::quiet_NaN();
  std::array<bool, 6> degenerate_mask = {false, false, false, false, false,
                                         false};
  Eigen::Vector3d lambda_schur_rot = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d lambda_schur_trans = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d aligned_lambda_schur_rot = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d aligned_lambda_schur_trans = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d clamped_lambda_schur_rot = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d clamped_lambda_schur_trans = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d aligned_rot_basis = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d aligned_trans_basis = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rot_axis_contribution_ratio = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d trans_axis_contribution_ratio = Eigen::Matrix3d::Zero();
  std::array<int, 3> rot_indices = {0, 1, 2};
  std::array<int, 3> trans_indices = {0, 1, 2};
  Matrix6d preconditioner = Matrix6d::Identity();
};

inline constexpr double kAxisOrthogonalityWarningThreshold = 1e-8;

inline double MaxOrthogonalityError(const Eigen::Matrix3d& basis) {
  return (basis.transpose() * basis - Eigen::Matrix3d::Identity())
      .cwiseAbs()
      .maxCoeff();
}

// ====================================================================
// AlignEigenBasisToAxes: 特征向量 → 物理轴 对齐（核心模块 2 辅助函数）
//
// 功能:
//   将 3×3 特征向量矩阵的列（原始特征向量）与参考物理轴 [X, Y, Z]
//   做最大点积匹配，解决 SelfAdjointEigenSolver 输出特征向量排列
//   "任意"的问题。
//
// 输入:
//   raw_basis:     原始特征向量矩阵, 3×3, 列 i 对应 eigenvalues[i]
//   refs:          3 个参考轴, refs[0]=UnitX, refs[1]=UnitY, refs[2]=UnitZ
//   label:         诊断标签, 用于日志 ("rot" 或 "trans")
//
// 输出:
//   aligned_basis:     对齐后的特征向量矩阵
//                        col 0 ≈ refs[0] 方向 (X)
//                        col 1 ≈ refs[1] 方向 (Y)
//                        col 2 ≈ refs[2] 方向 (Z)
//   original_indices:  原始索引到物理轴的映射关系
//                        original_indices[axis] = 原始特征向量列号
//
// 返回值:
//   true:  对齐成功
//   false: 未找到一个到三个候选的匹配 (理论不应发生)
//
// 算法: 贪心指派 (Greedy Assignment)
//   对每个物理轴 axis ∈ [0,1,2] (即 X/Y/Z):
//     在未使用的原始特征向量中, 找到与 refs[axis] 点积绝对值最大的
//     标记该候选向量为"已使用", 避免重复匹配
//     如果点积 < 0: 特征向量取反 (正负号歧义消除)
//   这是一个对 3×3 指派问题的贪心求解:
//     - 最优解: Hungarian Algorithm (O(n³))
//     - 贪心解: 逐轴最优匹配 (O(n²))
//     对 3×3 矩阵, 贪心和最优解等价 (只需要检查 3! = 6 种排列,
//     贪心一定找到最优)
//
// 为什么需要对齐?
//   Eigen::SelfAdjointEigenSolver 输出的特征值按升序排列,
//   特征向量列与之对应: col 0 对应最小特征值, col 2 对应最大。
//   但我们不知道 col 0 指向 roll 还是 pitch。
//   对齐后, aligned_basis.col(0) 才真正代表 roll 方向。
//
// 点积几何含义:
//   refs[axis].dot(raw_basis.col(candidate)):
//     = |refs|·|raw_basis.col|·cos(θ) = cos(θ)
//     当特征向量与参考轴夹角 θ=0° 时, 点积 = 1 (完全对齐)
//     当特征向量与参考轴夹角 θ=90° 时, 点积 = 0 (正交, 无关)
//     取绝对值是因为特征向量方向在分解中不固定 (v 和 -v 等价)
// ====================================================================
inline bool AlignEigenBasisToAxes(const Eigen::Matrix3d& raw_basis,
                                  const std::array<Eigen::Vector3d, 3>& refs,
                                  const char* label,
                                  Eigen::Matrix3d* aligned_basis,
                                  std::array<int, 3>* original_indices) {
  (void)label;  // 消除 unused parameter 编译警告

  // 初始化输出
  aligned_basis->setZero();
  original_indices->fill(-1);

  // 记录每个候选特征向量是否已被分配给某个物理轴
  // 保证每个物理轴获得不同的特征向量 (双射)
  std::array<bool, 3> used = {false, false, false};

  // ── 贪心匹配：外层循环遍历物理轴 ──
  for (int axis = 0; axis < 3; ++axis) {
    double best_score = -1.0;
    int best_index = -1;

    // 在未使用的特征向量中, 找与当前物理轴最匹配的
    for (int candidate = 0; candidate < 3; ++candidate) {
      if (used[candidate]) {
        continue;  // 已被其他轴占用, 跳过
      }

      // 关键计算: 点积绝对值
      // score = |refs[axis] · raw_basis.col(candidate)|
      // 取绝对值的原因: 特征向量 v 和 -v 数学等价
      // (特征分解只定义方向, 不定义方向),
      // 物理意义上, roll 的正方向由系统约定,\
      // 后续通过符号修正
      const double score = std::abs(refs[axis].dot(raw_basis.col(candidate)));

      if (score > best_score) {
        best_score = score;
        best_index = candidate;
      }
    }

    // 安全检查: 所有候选都不可用 (used 全 true)
    // 对 3×3 矩阵, 有 3 个物理轴和 3 个候选, 
    // 只要算法正确就不会触发
    if (best_index < 0) {
      return false;
    }

    // 标记该特征向量为已使用
    used[best_index] = true;
    // 记录原始索引: axis 轴映射到原始的第 best_index 列
    (*original_indices)[axis] = best_index;

    // ── 符号修正 ──
    // 取绝对值后丢失了方向信息, 需要恢复
    // 如果点积 < 0, 说明特征向量与参考轴方向相反, 需要取反
    Eigen::Vector3d aligned_column = raw_basis.col(best_index);
    if (refs[axis].dot(aligned_column) < 0.0) {
      aligned_column = -aligned_column;
    }
    // 将对齐后的特征向量写入输出矩阵的对应列
    (*aligned_basis).col(axis) = aligned_column;
  }

  // ── 正交性验证 ──
  // 理论上, 对齐后基底仍然是正交的 (原始特征向量正交 + 列置换和符号取反不破坏正交性)
  // 但由于 Eigen 特征分解的数值误差, 可能轻微偏离正交
  // 计算 ||aligned_basis^T · aligned_basis - I||_max
  // 如果误差 > 1e-8, 发出警告
  const double orthogonality_error = MaxOrthogonalityError(*aligned_basis);

  if (orthogonality_error > kAxisOrthogonalityWarningThreshold) {
    std::cerr << "[DCReg][Warning][AxisAlignment][" << label
              << "] mapped basis is not sufficiently orthogonal after axis "
                 "permutation/sign selection (error=" << orthogonality_error
              << ").\n";
  }
  return true;
}

// ====================================================================
// DetectDegeneracy: 退化检测（核心模块 1）
//
// 功能:
//   对 6×6 Hessian 矩阵做 Schur 补分解，将旋转和平移解耦，
//   分别计算各自的特征值和特征向量，用于评估每个 DOF 的可观性。
//
// 输入:
//   hessian: 6×6 矩阵 H = J^T J
//            注意: 调用方需要保证 H 的排列为 [rot(0-2), pos(3-5)]
//           (因为 DCReg 期望 h_rr = H.block(0,0) 是旋转块)
//   params:  求解参数（含 degeneracy_condition_threshold, kappa_target）
//
// 输出:
//   DegeneracyDetection 结构，包含:
//   - factorization_ok:     分解是否成功
//   - cond_full:            完整 6×6 Hessian 的条件数
//   - lambda_schur_rot:     旋转 Schur 补的 3 个特征值 (无序)
//   - lambda_schur_trans:   平移 Schur 补的 3 个特征值 (无序)
//   - raw_rot_basis:        旋转 Schur 补的特征向量 (原始、未对齐)
//   - raw_trans_basis:      平移 Schur 补的特征向量 (原始、未对齐)
//   - cond_schur_rot:       旋转 Schur 补的条件数 (λ_max / λ_min)
//   - cond_schur_trans:     平移 Schur 补的条件数 (λ_max / λ_min)
//
// Schur 补解耦原理:
//
//   H = J^T J 是 6×6 的对称半正定矩阵，描述了误差曲面在位姿空间中的曲率。
//   直接对 H 做 SVD 只能得到全局条件数，不能区分旋转和平移的各自退化情况。
//
//   将 H 分块为:
//       H = [H_rr  H_rt]    H_rr (3×3): 旋转-旋转 (子块)
//           [H_tr  H_tt]    H_tt (3×3): 平移-平移 (子块)
//                           H_rt, H_tr (3×3): 旋转-平移 交叉耦合
//
//   旋转的 Schur 补:  S_rot    = H_rr - H_rt · H_tt^{-1} · H_tr   (式 1)
//   平移的 Schur 补:  S_trans = H_tt - H_tr · H_rr^{-1} · H_rt   (式 2)
//
//   式 1 的物理含义: 从 H_rr 中减去平移通过交叉耦合对旋转的"贡献"，
//   得到的是"排除了平移估计不确定性后的纯旋转可观性"。
//
//   数学上: Schur 补 = 对 H 做块高斯消元后对角块的剩余部分。
//   对 H 做分块高斯消元:
//       [I     0]·[H_rr  H_rt] = [H_rr     H_rt   ]
//       [-H_tr·H_rr^{-1}  I] [H_tr  H_tt]   [0    S_trans]
//   右下角块 S_trans = H_tt - H_tr·H_rr^{-1}·H_rt 就是平移的 Schur 补。
//   同理，消去平移块得到 S_rot。
//
//   然后对 S_rot (3×3) 做特征分解 ⟹ 3 个特征值 λ_rot_i
//   对 S_trans (3×3) 做特征分解 ⟹ 3 个特征值 λ_trans_i
//
//   由于 S_rot 和 S_trans 都是对称半正定矩阵(当 Hessian 是 SPD 时)，
//   特征值分解使用 SelfAdjointEigenSolver。
//
// 条件数计算:
//   cond_schur_rot = max(λ_rot) / max(min(λ_rot), 1e-12)
//   条件数 >> 阈值 → 该子块存在退化方向
//
// 完整 Hessian 条件数 (参考):
//   cond_full = σ_max / σ_min (SVD 奇异值比)
//   作为辅助参考，不直接用于退化判定。
//
// 引用: Zhang, J., Kaess, M., & Singh, S. (2016).
//   "On degeneracy of optimization-based state estimation problems."
//   ICRA 2016. 式 (18) 对应 Schur 补解耦公式。
// ====================================================================
inline DegeneracyDetection DetectDegeneracy(const Matrix6d& hessian,
                                            const SolverParameters& params) {
  DegeneracyDetection detection;

  // ---- 步骤 1: 完整 Hessian 的 SVD ----
  // 计算 6 个奇异值 (实际上因为 Hessian 是对称半正定的，奇异值 = 特征值)
  // 用于计算全局条件数作为辅助参考
  const Eigen::JacobiSVD<Matrix6d> full_svd(
      hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto singular_values = full_svd.singularValues();
  detection.cond_full =
      singular_values(5) > 1e-12
          ? singular_values(0) / singular_values(5)
          : std::numeric_limits<double>::infinity();

  // ---- 步骤 2: 提取四个 3×3 子块 ----
  //   H = [H_rr(rot)  H_rt]     ← 注意: 要求 Hessian 排列为 [rot, pos]
  //       [H_tr       H_tt(pos)]
  const Eigen::Matrix3d h_rr = hessian.block<3, 3>(0, 0);
  const Eigen::Matrix3d h_rt = hessian.block<3, 3>(0, 3);
  const Eigen::Matrix3d h_tr = hessian.block<3, 3>(3, 0);
  const Eigen::Matrix3d h_tt = hessian.block<3, 3>(3, 3);

  // ---- 步骤 3: 检查子块是否可逆 ----
  // FullPivLU 是带全主元的高斯消元，可以判断矩阵是否满秩
  // 如果 h_rr 或 h_tt 不可逆（奇异矩阵），Schur 补公式中的逆不存在
  // 这种情况出现在 Hessian 严重退化时（如完全没有旋转/平移约束）
  const Eigen::FullPivLU<Eigen::Matrix3d> lu_rr(h_rr);
  const Eigen::FullPivLU<Eigen::Matrix3d> lu_tt(h_tt);
  if (!lu_rr.isInvertible() || !lu_tt.isInvertible()) {
    // 不可逆 → 无法计算 Schur 补 → 标记分解失败
    // 调用方 (CharacterizeDegeneracy) 会将其视为完全退化
    return detection;
  }

  // ---- 步骤 4: Schur 补解耦 ----
  // 式 (18) from Zhang et al. ICRA 2016:
  //   旋转的 Schur 补: S_rot = H_rr - H_rt · H_tt^{-1} · H_tr
  //   平移的 Schur 补: S_trans = H_tt - H_tr · H_rr^{-1} · H_rt
  //
  // H_rt 和 H_tr 捕捉了旋转-平移的耦合（例如: 旋转相机时，平移方向也会变化）
  // 减去耦合项后，S_rot 的曲率只反映纯旋转的信息量
  const Eigen::Matrix3d schur_rot = h_rr - h_rt * lu_tt.inverse() * h_tr;
  const Eigen::Matrix3d schur_trans = h_tt - h_tr * lu_rr.inverse() * h_rt;

  // ---- 步骤 5: 特征分解 ----
  // 对 3×3 对称矩阵做特征分解，得到 3 个特征值和对应的特征向量
  // 特征值大小 → 该方向的信息量（曲率）
  // 特征向量 → 该方向在物理空间中的指向
  //
  // 注意: SelfAdjointEigenSolver 保证特征值是升序排列 (λ_0 ≤ λ_1 ≤ λ_2)
  // 但特征向量的方向是任意的（正负号不固定）
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rot_solver(schur_rot);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> trans_solver(schur_trans);
  if (rot_solver.info() != Eigen::Success ||
      trans_solver.info() != Eigen::Success) {
    // 特征分解失败（数值不稳定）→ 标记分解失败
    return detection;
  }

  // ---- 步骤 6: 输出结果 ----
  detection.factorization_ok = true;
  // 特征值: 升序排列 [λ_min, λ_mid, λ_max]
  detection.lambda_schur_rot = rot_solver.eigenvalues();
  detection.lambda_schur_trans = trans_solver.eigenvalues();
  // 特征向量: 3×3 矩阵，每一列是一个特征向量
  // 列 i 对应 eigenvalues[i]
  detection.raw_rot_basis = rot_solver.eigenvectors();
  detection.raw_trans_basis = trans_solver.eigenvectors();
  // 条件数: 最大特征值 / 最小特征值 (防止除零)
  // 条件数越大 → 该方向的信息量越不平衡 → 越可能存在退化
  detection.cond_schur_rot =
      detection.lambda_schur_rot.maxCoeff() /
      std::max(detection.lambda_schur_rot.minCoeff(), 1e-12);
  detection.cond_schur_trans =
      detection.lambda_schur_trans.maxCoeff() /
      std::max(detection.lambda_schur_trans.minCoeff(), 1e-12);
  return detection;
}

// ====================================================================
// CharacterizeDegeneracy: 退化判定与预条件子构建（核心模块 2）
//
// 功能:
//   基于 DetectDegeneracy 的输出，将特征向量与物理轴 (X/Y/Z) 对齐，
//   计算每个轴方向的退化分数 (条件数)，生成退化掩码，
//   并构造协方差膨胀用的预条件子矩阵。
//
// 输入:
//   detection: DetectDegeneracy 的输出 (Schur 特征值/特征向量)
//   params:    求解参数 (degeneracy_condition_threshold, kappa_target)
//
// 输出:
//   DegeneracyCharacterization 结构，主要包含:
//   - is_degenerate:         是否存在退化方向
//   - degenerate_mask[6]:    6 个 DOF 的退化真值
//                            顺序: [roll, pitch, yaw, x, y, z]
//   - aligned_lambda_*:      对齐到物理轴后的特征值
//   - clamped_lambda_*:      夹紧后的最小特征值 (用于膨胀)
//   - aligned_*_basis:       对齐后的特征向量 (单位方向)
//   - preconditioner:        6×6 预条件子矩阵 (协方差膨胀用)
//
// 核心流程:
//   步骤 1: 特征向量 → 物理轴 对齐 (AlignEigenBasisToAxes)
//   步骤 2: 特征值重排 (按对齐后的顺序)
//   步骤 3: 逐轴条件数计算 & 退化判定
//   步骤 4: 最小特征值夹紧 (clamping)
//   步骤 5: 预条件子矩阵构建
//
// ====================================================================
inline DegeneracyCharacterization CharacterizeDegeneracy(
    const DegeneracyDetection& detection, const SolverParameters& params) {
  DegeneracyCharacterization characterization;

  // ── 复制基础信息 ──
  characterization.factorization_ok = detection.factorization_ok;
  characterization.cond_full = detection.cond_full;
  characterization.cond_schur_rot = detection.cond_schur_rot;
  characterization.cond_schur_trans = detection.cond_schur_trans;
  characterization.lambda_schur_rot = detection.lambda_schur_rot;
  characterization.lambda_schur_trans = detection.lambda_schur_trans;

  // 如果 DetectDegeneracy 中 Schur 补分解失败 (如 h_rr/h_tt 不可逆),
  // 直接标记全部 6 个方向为退化, 调用方会做最大程度膨胀
  if (!detection.factorization_ok) {
    characterization.is_degenerate = true;
    characterization.degenerate_mask = {true, true, true, true, true, true};
    return characterization;
  }

  // =============================================================
  // 步骤 1: 特征向量 → 物理轴 对齐
  // =============================================================
  // 问题: SelfAdjointEigenSolver 输出的特征向量是 "任意" 的。
  //   例如 rot 的 3 个特征向量 v₀, v₁, v₂ 分别对应 λ₀≤λ₁≤λ₂,
  //   但我们不知道 v₀ 究竟指向 roll、pitch 还是 yaw。
  //
  // 解决方法: 将特征向量与物理轴 [X, Y, Z] 做最大点积匹配。
  //   参考轴 = [UnitX, UnitY, UnitZ] = roll/pitch/yaw 各轴
  //   原始基底 = raw_rot_basis = [v₀, v₁, v₂]
  //   对每个轴, 找出与之最接近（点积绝对值最大）的 v_i
  //
  // 对齐结果:
  //   aligned_rot_basis.col(0) ≈ roll 方向    (绕 X 轴)
  //   aligned_rot_basis.col(1) ≈ pitch 方向   (绕 Y 轴)
  //   aligned_rot_basis.col(2) ≈ yaw 方向     (绕 Z 轴)
  //
  // rot_indices: 记录原始索引到物理轴的映射
  //   例如 rot_indices = {2, 0, 1} 表示:
  //     roll  ← 原始 v₂ (index 2)
  //     pitch ← 原始 v₀ (index 0)
  //     yaw   ← 原始 v₁ (index 1)
  const std::array<Eigen::Vector3d, 3> refs = {
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(),
  };
  if (!AlignEigenBasisToAxes(detection.raw_rot_basis, refs, "rot",
                             &characterization.aligned_rot_basis,
                             &characterization.rot_indices)) {
    // 对齐失败 → 保持原始 (理论不应发生)
    characterization.aligned_rot_basis = detection.raw_rot_basis;
    characterization.rot_indices = {0, 1, 2};
  }
  if (!AlignEigenBasisToAxes(detection.raw_trans_basis, refs, "trans",
                             &characterization.aligned_trans_basis,
                             &characterization.trans_indices)) {
    characterization.aligned_trans_basis = detection.raw_trans_basis;
    characterization.trans_indices = {0, 1, 2};
  }

  // =============================================================
  // 步骤 1.5: 轴贡献比 (Axis Contribution Ratio)
  // =============================================================
  // 当特征方向与物理轴不完全一致时 (如特征向量 = 0.7·X + 0.7·Y),
  // 这个比率描述了各物理轴在特征方向上的投影能量。
  //
  // 计算: aligned_basis 的每个元素的平方 = 该方向与物理轴夹角的余弦平方
  //   如 rot_axis_contribution_ratio(0,1) = |aligned_rot_basis(0,1)|²
  //   表示 pitch 特征方向被投影到 roll 轴上的分量能量
  //
  // 目前这个变量没有在后续使用中直接参与计算, 保留用于诊断/可视化
  characterization.rot_axis_contribution_ratio =
      characterization.aligned_rot_basis.cwiseProduct(
          characterization.aligned_rot_basis);
  characterization.trans_axis_contribution_ratio =
      characterization.aligned_trans_basis.cwiseProduct(
          characterization.aligned_trans_basis);

  // =============================================================
  // 步骤 2: 特征值重排 (按物理轴顺序)
  // =============================================================
  // 原始特征值 lambda_schur_rot 是升序排列 [λ₀, λ₁, λ₂],
  // 分别对应原始特征向量 [v₀, v₁, v₂]。
  //
  // rot_indices[axis] 告诉我们对齐后 axis 轴用的是原始的第几个特征向量。
  //   例如 rot_indices = {2, 0, 1}:
  //     roll(axis=0)  ← 原始 index=2 → aligned_lambda(0) = λ₂
  //     pitch(axis=1) ← 原始 index=0 → aligned_lambda(1) = λ₀
  //     yaw(axis=2)   ← 原始 index=1 → aligned_lambda(2) = λ₁
  //
  // 重排后:
  //   aligned_lambda_schur_rot = [λ_roll, λ_pitch, λ_yaw]
  //   aligned_lambda_schur_trans = [λ_x, λ_y, λ_z]
  for (int axis = 0; axis < 3; ++axis) {
    characterization.aligned_lambda_schur_rot(axis) =
        detection.lambda_schur_rot(characterization.rot_indices[axis]);
    characterization.aligned_lambda_schur_trans(axis) =
        detection.lambda_schur_trans(characterization.trans_indices[axis]);
  }

  // =============================================================
  // 步骤 3: 退化判定 (逐轴条件数比较)
  // =============================================================
  // 初始化 clamped_lambda = aligned_lambda (暂不修改)
  characterization.clamped_lambda_schur_rot =
      characterization.aligned_lambda_schur_rot;
  characterization.clamped_lambda_schur_trans =
      characterization.aligned_lambda_schur_trans;

  // 找出当前子块中最大的特征值 (最"紧"的方向)
  const double rot_max = characterization.aligned_lambda_schur_rot.maxCoeff();
  const double trans_max =
      characterization.aligned_lambda_schur_trans.maxCoeff();

  // 计算"夹紧"后的最小允许特征值:
  //   min_rot = rot_max / kappa_target
  //   物理含义: 假如最大特征值为 100, kappa_target=100,
  //   则最小特征值不能低于 1。如果某方向特征值只有 0.01,
  //   说明该方向的条件数 = 100/0.01 = 10000, 远超阈值。
  //   膨胀时会把它提升到 1, 使条件数降至 100.
  const double min_rot = std::max(rot_max / params.kappa_target, 1e-9);
  const double min_trans = std::max(trans_max / params.kappa_target, 1e-9);

  // 逐轴判定退化:
  //   rot_condition = rot_max / λ_axis
  //   如果 rot_condition > degeneracy_condition_threshold → 退化
  //   (该方向约束强度远低于最强方向)
  for (int axis = 0; axis < 3; ++axis) {
    const double rot_condition =
        rot_max /
        std::max(characterization.aligned_lambda_schur_rot(axis), 1e-12);
    if (rot_condition > params.degeneracy_condition_threshold) {
      characterization.is_degenerate = true;
      characterization.degenerate_mask[axis] = true;          // roll/pitch/yaw
      // 夹紧: 将退化方向的特征值提升到 min_rot
      characterization.clamped_lambda_schur_rot(axis) = min_rot;
    }

    const double trans_condition =
        trans_max /
        std::max(characterization.aligned_lambda_schur_trans(axis), 1e-12);
    if (trans_condition > params.degeneracy_condition_threshold) {
      characterization.is_degenerate = true;
      characterization.degenerate_mask[axis + 3] = true;      // x/y/z
      characterization.clamped_lambda_schur_trans(axis) = min_trans;
    }
  }

  // =============================================================
  // 步骤 4: 预条件子矩阵构建 (协方差膨胀)
  // =============================================================
  // 预条件子 M = diag(M_rot, M_trans)
  //
  // M_rot = aligned_rot_basis · clamped_λ⁻¹ · aligned_rot_basis^T
  //
  // 物理含义:
  //   - 在非退化方向: clamped_λ⁻¹ = 1/λ (很小, 不膨胀)
  //   - 在退化方向: clamped_λ⁻¹ = 1/min_rot (很大, 大膨胀)
  //
  // 通过特征基变换实现: 先投影到特征空间 (aligned_basis^T),
  // 在特征空间做缩放 (clamped_λ⁻¹), 再投影回物理轴 (aligned_basis)。
  //
  // 最终 M 被调用方 (如 ApplyDCRegAndWeights) 用于协方差膨胀:
  //   P_inflated = M * P * M^T   (或 P = M .* P, 取决于具体实现)
  //
  // 注意: 这里构建的是"预条件子"而不是直接修改 P,
  // 因为 DCReg 框架中膨胀逻辑由调用方决定具体方式。
  characterization.preconditioner.setZero();
  characterization.preconditioner.block<3, 3>(0, 0) =
      characterization.aligned_rot_basis *
      characterization.clamped_lambda_schur_rot.cwiseMax(1e-9)
          .cwiseInverse()
          .asDiagonal() *
      characterization.aligned_rot_basis.transpose();
  characterization.preconditioner.block<3, 3>(3, 3) =
      characterization.aligned_trans_basis *
      characterization.clamped_lambda_schur_trans.cwiseMax(1e-9)
          .cwiseInverse()
          .asDiagonal() *
      characterization.aligned_trans_basis.transpose();
  return characterization;
}

}  // namespace dcreg

#endif  // DCREG_DCREG_HPP_