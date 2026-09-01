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

// Core module 2 helper: map the raw Schur eigenvectors into physical axes.
inline bool AlignEigenBasisToAxes(const Eigen::Matrix3d& raw_basis,
                                  const std::array<Eigen::Vector3d, 3>& refs,
                                  const char* label,
                                  Eigen::Matrix3d* aligned_basis,
                                  std::array<int, 3>* original_indices) {
  (void)label;
  aligned_basis->setZero();
  original_indices->fill(-1);

  std::array<bool, 3> used = {false, false, false};
  for (int axis = 0; axis < 3; ++axis) {
    double best_score = -1.0;
    int best_index = -1;
    for (int candidate = 0; candidate < 3; ++candidate) {
      if (used[candidate]) {
        continue;
      }
      const double score = std::abs(refs[axis].dot(raw_basis.col(candidate)));
      if (score > best_score) {
        best_score = score;
        best_index = candidate;
      }
    }
    if (best_index < 0) {
      return false;
    }
    used[best_index] = true;
    (*original_indices)[axis] = best_index;
    Eigen::Vector3d aligned_column = raw_basis.col(best_index);
    if (refs[axis].dot(aligned_column) < 0.0) {
      aligned_column = -aligned_column;
    }
    (*aligned_basis).col(axis) = aligned_column;
  }
  const double orthogonality_error = MaxOrthogonalityError(*aligned_basis);

  if (orthogonality_error > kAxisOrthogonalityWarningThreshold) {
    std::cerr << "[DCReg][Warning][AxisAlignment][" << label
              << "] mapped basis is not sufficiently orthogonal after axis "
                 "permutation/sign selection (error=" << orthogonality_error
              << ").\n";
  }
  return true;
}

// Core module 1: detect degeneracy from the Schur complements.
inline DegeneracyDetection DetectDegeneracy(const Matrix6d& hessian,
                                            const SolverParameters& params) {
  DegeneracyDetection detection;
  const Eigen::JacobiSVD<Matrix6d> full_svd(
      hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto singular_values = full_svd.singularValues();
  detection.cond_full =
      singular_values(5) > 1e-12
          ? singular_values(0) / singular_values(5)
          : std::numeric_limits<double>::infinity();

  const Eigen::Matrix3d h_rr = hessian.block<3, 3>(0, 0);
  const Eigen::Matrix3d h_rt = hessian.block<3, 3>(0, 3);
  const Eigen::Matrix3d h_tr = hessian.block<3, 3>(3, 0);
  const Eigen::Matrix3d h_tt = hessian.block<3, 3>(3, 3);

  const Eigen::FullPivLU<Eigen::Matrix3d> lu_rr(h_rr);
  const Eigen::FullPivLU<Eigen::Matrix3d> lu_tt(h_tt);
  if (!lu_rr.isInvertible() || !lu_tt.isInvertible()) {
    return detection;
  }

  // Eq. (18): decouple rotation and translation observability.
  const Eigen::Matrix3d schur_rot = h_rr - h_rt * lu_tt.inverse() * h_tr;
  const Eigen::Matrix3d schur_trans = h_tt - h_tr * lu_rr.inverse() * h_rt;

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rot_solver(schur_rot);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> trans_solver(schur_trans);
  if (rot_solver.info() != Eigen::Success ||
      trans_solver.info() != Eigen::Success) {
    return detection;
  }

  detection.factorization_ok = true;
  detection.lambda_schur_rot = rot_solver.eigenvalues();
  detection.lambda_schur_trans = trans_solver.eigenvalues();
  detection.raw_rot_basis = rot_solver.eigenvectors();
  detection.raw_trans_basis = trans_solver.eigenvectors();
  detection.cond_schur_rot =
      detection.lambda_schur_rot.maxCoeff() /
      std::max(detection.lambda_schur_rot.minCoeff(), 1e-12);
  detection.cond_schur_trans =
      detection.lambda_schur_trans.maxCoeff() /
      std::max(detection.lambda_schur_trans.minCoeff(), 1e-12);
  return detection;
}

// Core module 2: characterize degeneracy in physical coordinates.
inline DegeneracyCharacterization CharacterizeDegeneracy(
    const DegeneracyDetection& detection, const SolverParameters& params) {
  DegeneracyCharacterization characterization;
  characterization.factorization_ok = detection.factorization_ok;
  characterization.cond_full = detection.cond_full;
  characterization.cond_schur_rot = detection.cond_schur_rot;
  characterization.cond_schur_trans = detection.cond_schur_trans;
  characterization.lambda_schur_rot = detection.lambda_schur_rot;
  characterization.lambda_schur_trans = detection.lambda_schur_trans;

  if (!detection.factorization_ok) {
    characterization.is_degenerate = true;
    characterization.degenerate_mask = {true, true, true, true, true, true};
    return characterization;
  }

  const std::array<Eigen::Vector3d, 3> refs = {
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(),
  };
  if (!AlignEigenBasisToAxes(detection.raw_rot_basis, refs, "rot",
                             &characterization.aligned_rot_basis,
                             &characterization.rot_indices)) {
    characterization.aligned_rot_basis = detection.raw_rot_basis;
    characterization.rot_indices = {0, 1, 2};
  }
  if (!AlignEigenBasisToAxes(detection.raw_trans_basis, refs, "trans",
                             &characterization.aligned_trans_basis,
                             &characterization.trans_indices)) {
    characterization.aligned_trans_basis = detection.raw_trans_basis;
    characterization.trans_indices = {0, 1, 2};
  }

  characterization.rot_axis_contribution_ratio =
      characterization.aligned_rot_basis.cwiseProduct(
          characterization.aligned_rot_basis);
  characterization.trans_axis_contribution_ratio =
      characterization.aligned_trans_basis.cwiseProduct(
          characterization.aligned_trans_basis);

  for (int axis = 0; axis < 3; ++axis) {
    characterization.aligned_lambda_schur_rot(axis) =
        detection.lambda_schur_rot(characterization.rot_indices[axis]);
    characterization.aligned_lambda_schur_trans(axis) =
        detection.lambda_schur_trans(characterization.trans_indices[axis]);
  }

  characterization.clamped_lambda_schur_rot =
      characterization.aligned_lambda_schur_rot;
  characterization.clamped_lambda_schur_trans =
      characterization.aligned_lambda_schur_trans;

  const double rot_max = characterization.aligned_lambda_schur_rot.maxCoeff();
  const double trans_max =
      characterization.aligned_lambda_schur_trans.maxCoeff();
  const double min_rot = std::max(rot_max / params.kappa_target, 1e-9);
  const double min_trans = std::max(trans_max / params.kappa_target, 1e-9);
  for (int axis = 0; axis < 3; ++axis) {
    const double rot_condition =
        rot_max /
        std::max(characterization.aligned_lambda_schur_rot(axis), 1e-12);
    if (rot_condition > params.degeneracy_condition_threshold) {
      characterization.is_degenerate = true;
      characterization.degenerate_mask[axis] = true;
      characterization.clamped_lambda_schur_rot(axis) = min_rot;
    }

    const double trans_condition =
        trans_max /
        std::max(characterization.aligned_lambda_schur_trans(axis), 1e-12);
    if (trans_condition > params.degeneracy_condition_threshold) {
      characterization.is_degenerate = true;
      characterization.degenerate_mask[axis + 3] = true;
      characterization.clamped_lambda_schur_trans(axis) = min_trans;
    }
  }

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