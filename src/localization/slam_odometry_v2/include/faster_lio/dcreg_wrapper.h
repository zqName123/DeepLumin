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
    // 截断下限避免完全丢弃
    return std::max(0.1, 1.0 / (1.0 + alpha * penalty));
}

}  // namespace faster_lio

#endif  // FASTER_LIO_DCREG_WRAPPER_H