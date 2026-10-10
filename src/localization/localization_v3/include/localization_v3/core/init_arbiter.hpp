#pragma once

/**
 * @file init_arbiter.hpp
 * @brief 初始化仲裁（纯逻辑，header-only）
 *
 * 双源初始化（重定位 + GM 候选）的择优与超时兜底，评分语义与 v2 一致：
 *   score = inlier_ratio - 0.3 * inlier_rmse（重定位是初始化主源，同分优先重定位）
 *
 * 逻辑：
 *   非双源模式：重定位新鲜优先，否则 GM 候选
 *   双源模式：  两源齐备 → 评分择优；未到 fallback 超时则继续等；
 *               超时后取已就绪的一源，防止初始化被阻塞
 */

namespace localization_v3 {

enum class InitSource {
  kNone = 0,
  kRelocalization,
  kGlobalMatcher,
};

struct InitArbiterConfig {
  bool use_relocalization_init = true;
  bool use_global_matcher_init = true;
  double fallback_timeout_sec = 8.0;   ///< 双源等待超时（s）
  double reloc_result_max_age_sec = 3.0;  ///< 重定位结果时效窗（s）
};

/// v2 同款综合评分：内点率权重 1.0，RMSE 权重 -0.3
inline double initScore(double inlier_ratio, double inlier_rmse) {
  return inlier_ratio - 0.3 * inlier_rmse;
}

/**
 * @brief 选择初始化来源
 * @param reloc_fresh     重定位结果新鲜（时效窗内）
 * @param gm_ready        GM 候选已就绪
 * @param reloc_score     重定位综合评分（用 initScore 由 ratio/rmse 计算）
 * @param gm_score        GM 综合评分（同上）
 * @param elapsed_sec     距进入初始化的时长（用于 fallback 超时）
 */
inline InitSource selectInitSource(const InitArbiterConfig& cfg, bool reloc_fresh, bool gm_ready,
                                   double reloc_score, double gm_score, double elapsed_sec) {
  const bool both_mode = cfg.use_global_matcher_init && cfg.use_relocalization_init;

  if (!both_mode) {
    if (reloc_fresh && cfg.use_relocalization_init) {
      return InitSource::kRelocalization;
    }
    if (gm_ready && cfg.use_global_matcher_init) {
      return InitSource::kGlobalMatcher;
    }
    return InitSource::kNone;
  }

  if (reloc_fresh && gm_ready) {
    // 同分优先重定位（初始化主源，v2 语义）
    return (reloc_score >= gm_score) ? InitSource::kRelocalization : InitSource::kGlobalMatcher;
  }
  if (elapsed_sec < cfg.fallback_timeout_sec) {
    return InitSource::kNone;  // 继续等另一源
  }
  if (reloc_fresh) {
    return InitSource::kRelocalization;
  }
  if (gm_ready) {
    return InitSource::kGlobalMatcher;
  }
  return InitSource::kNone;
}

}  // namespace localization_v3
