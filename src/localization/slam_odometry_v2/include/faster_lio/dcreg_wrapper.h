#ifndef FASTER_LIO_DCREG_WRAPPER_H
#define FASTER_LIO_DCREG_WRAPPER_H

#include <Eigen/Core>
#include <array>
#include <cmath>

#include "dcreg.hpp"

namespace faster_lio {

/// DCReg 退化诊断结果（轻量结构，仅含需要的字段）
struct DCRegResult {
    bool is_degenerate = false;
    bool factorization_ok = false;
    // [roll, pitch, yaw, x, y, z]
    std::array<bool, 6> degenerate_mask = {false, false, false, false, false, false};
    std::array<double, 6> degeneracy_scores = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    /// 对齐到物理轴的退化方向基底（来自 CharacterizeDegeneracy），用于 P 矩阵定向膨胀：
    ///   aligned_rot_basis.col(d)   ≈ roll/pitch/yaw 轴（body 旋转切空间，d=0..2）
    ///   aligned_trans_basis.col(d) ≈ x/y/z 轴（世界系，d=0..2）
    /// 注意：输入 Hessian 列序须为 [rot, pos]（调用方已重排）。
    /// 分解失败时保持 Identity（配合全退化 mask，退化为逐物理轴膨胀）。
    Eigen::Matrix3d aligned_rot_basis = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d aligned_trans_basis = Eigen::Matrix3d::Identity();
    double cond_schur_rot = 0.0;
    double cond_schur_trans = 0.0;
};

/// 从 6×6 Hessian 计算 DCReg 退化诊断
inline DCRegResult ComputeDCReg(const Eigen::Matrix<double, 6, 6>& hessian,
                                double degeneracy_threshold = 10.0) {
    DCRegResult result;

    dcreg::SolverParameters params;
    params.degeneracy_condition_threshold = degeneracy_threshold;
    params.kappa_target = degeneracy_threshold;

    // ① DetectDegeneracy：Schur 补分解，解耦旋转/平移
    auto detection = dcreg::DetectDegeneracy(hessian, params);
    result.factorization_ok = detection.factorization_ok;
    result.cond_schur_rot = detection.cond_schur_rot;
    result.cond_schur_trans = detection.cond_schur_trans;

    if (!detection.factorization_ok) {
        // 分解失败 → 全退化
        result.is_degenerate = true;
        result.degenerate_mask = {true, true, true, true, true, true};
        result.degeneracy_scores = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        return result;
    }

    // ② CharacterizeDegeneracy：物理轴映射 + 退化判定
    auto characterization = dcreg::CharacterizeDegeneracy(detection, params);
    result.is_degenerate = characterization.is_degenerate;
    result.degenerate_mask = characterization.degenerate_mask;
    result.aligned_rot_basis = characterization.aligned_rot_basis;
    result.aligned_trans_basis = characterization.aligned_trans_basis;

    // ③ 计算退化分数 [0, 1]
    const double rot_max = characterization.aligned_lambda_schur_rot.maxCoeff();
    const double trans_max = characterization.aligned_lambda_schur_trans.maxCoeff();

    for (int i = 0; i < 3; i++) {
        double cond = rot_max / std::max(characterization.aligned_lambda_schur_rot(i), 1e-12);
        if (cond > degeneracy_threshold) {
            result.degeneracy_scores[i] = std::min(1.0, (cond - degeneracy_threshold) / degeneracy_threshold);
        }
    }
    for (int i = 0; i < 3; i++) {
        double cond = trans_max / std::max(characterization.aligned_lambda_schur_trans(i), 1e-12);
        if (cond > degeneracy_threshold) {
            result.degeneracy_scores[i + 3] = std::min(1.0, (cond - degeneracy_threshold) / degeneracy_threshold);
        }
    }

    return result;
}

/// Huber 鲁棒权重
inline double HuberWeight(double residual, double k) {
    double abs_r = std::abs(residual);
    if (abs_r <= k) {
        return 1.0;
    }
    return k / abs_r;
}

/// 退化方向降权：基于 Jacobian 行向量在各退化方向上的分量
/// degeneracy_scores 顺序: [roll, pitch, yaw, x, y, z]
/// J_row 列顺序:           [rot_x, rot_y, rot_z, pos_x, pos_y, pos_z]
/// (调用者已重排，两者自然对齐)
inline double DegeneracyWeight(const Eigen::Matrix<double, 1, 6>& J_row,
                               const std::array<double, 6>& degeneracy_scores,
                               double alpha) {
    double J_norm = J_row.norm();
    if (J_norm < 1e-12) return 1.0;

    double penalty = 0.0;
    for (int d = 0; d < 6; d++) {
        if (degeneracy_scores[d] > 0.01) {
            double contribution = std::abs(J_row(d)) / J_norm;
            penalty += degeneracy_scores[d] * contribution;
        }
    }
    // penalty² 指数衰减：轻度退化几乎不受影响，严重退化被强力抑制
    // alpha=5 时: score=0.01→0.05%, score=0.1→5%, score=0.5→71%, score=1.0→99.3%
    return std::max(0.001, std::exp(-alpha * penalty * penalty));
}

}  // namespace faster_lio

#endif  // FASTER_LIO_DCREG_WRAPPER_H