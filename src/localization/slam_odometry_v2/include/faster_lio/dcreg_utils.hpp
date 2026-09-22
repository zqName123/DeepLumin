#ifndef DCREG_UTILS_HPP_
#define DCREG_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace dcreg {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr int kMinCorrespondences = 10;

enum class Algorithm {
  kNone,
  kDCReg,
};

enum class Parameterization {
  kEuler,
  kSE3,
  kSO3,
  kQuaternion,
};

enum class ParallelMode {
  kAuto,
  kSerial,
  kOpenMP,
  kTbb,
};

struct SolverParameters {
  double search_radius = 1.0;
  int max_iterations = 30;
  double convergence_rot = 1e-5;
  double convergence_trans = 1e-3;

  // DCReg keeps only the Schur-based condition threshold and PCG settings.
  double degeneracy_condition_threshold = 10.0;
  double kappa_target = 10.0;
  double pcg_tolerance = 1e-6;
  int pcg_max_iterations = 10;

  // Point-to-plane construction constants kept consistent with the original code.
  int plane_fit_neighbors = 5;
  double max_plane_thickness = 0.2;
  double weight_slope = 0.9;
  double min_weight = 0.1;
  bool use_weight_derivative = true;
};

class Stopwatch {
 public:
  Stopwatch() { Reset(); }

  void Reset() { start_ = std::chrono::steady_clock::now(); }

  double ElapsedMilliseconds() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(now - start_).count();
  }

 private:
  std::chrono::steady_clock::time_point start_;
};

}  // namespace dcreg

#endif  // DCREG_UTILS_HPP_