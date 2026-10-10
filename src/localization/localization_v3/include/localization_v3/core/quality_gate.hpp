#pragma once

/**
 * @file quality_gate.hpp
 * @brief 匹配质量三级分级（header-only）
 *
 * 仅使用 inlier_ratio 与 inlier_rmse（不使用 fitness_score，语义与 v2 一致）：
 *   good     : inlier_ratio >= good_min_inlier_ratio 且 inlier_rmse <= max_inlier_rmse
 *   degraded : inlier_ratio >= degraded_min_inlier_ratio 且 inlier_rmse <= max_inlier_rmse
 *   bad      : 其余（RMSE 超标一律 bad）
 *
 * 质量分级只做一次：fusion_core（决定是否写入位姿）与
 * failure_detector / state_machine（决定失效计数）共用同一结果。
 */

namespace localization_v3 {

enum class QualityLevel {
  kGood = 0,
  kDegraded = 1,
  kBad = 2,
};

struct QualityThresholds {
  double good_min_inlier_ratio = 0.69;      ///< 好/退化分界
  double degraded_min_inlier_ratio = 0.4;   ///< 退化/差分界
  double max_inlier_rmse = 0.4;             ///< RMSE 上限（超标一律 bad）
};

inline QualityLevel classifyQuality(double inlier_ratio, double inlier_rmse,
                                    const QualityThresholds& th) {
  if (inlier_rmse > th.max_inlier_rmse) {
    return QualityLevel::kBad;
  }
  if (inlier_ratio >= th.good_min_inlier_ratio) {
    return QualityLevel::kGood;
  }
  if (inlier_ratio >= th.degraded_min_inlier_ratio) {
    return QualityLevel::kDegraded;
  }
  return QualityLevel::kBad;
}

inline const char* qualityName(QualityLevel level) {
  switch (level) {
    case QualityLevel::kGood:
      return "good";
    case QualityLevel::kDegraded:
      return "degraded";
    case QualityLevel::kBad:
      return "bad";
  }
  return "unknown";
}

}  // namespace localization_v3
