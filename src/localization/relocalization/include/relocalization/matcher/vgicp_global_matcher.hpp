#pragma once

/**
 * @file vgicp_global_matcher.hpp
 * @brief VGICP 精配准（从 relocalization_framework 移植，与 global_matcher 两级配准一致）
 *
 * 与 relocalization_framework 的 VgicpMatcher 行为一致：
 *   - target 获取：全局地图朝向裁剪（crop/*）→ 关键帧子图 → 全局地图兜底
 *   - 候选空间门控：只保留 top1 周围 candidate_gate_radius 内的候选
 *   - 两级配准：粗层（体素×coarse_scale、对应距离×coarse_scale）+ 细层（×fine_scale）
 *   - VGICP 无 PCL fitness：成功以内点率 / RMSE 为主（fitness 仅报告）
 */

#include <relocalization/interface/i_global_matcher.hpp>

namespace relocalization {

class VgicpGlobalMatcher : public IGlobalMatcher {
 public:
  explicit VgicpGlobalMatcher(const GlobalMatcherConfig& config);

  MatchResult alignCandidates(const CloudConstPtr& source,
                              const std::vector<DescriptorCandidate>& candidates,
                              const KeyframeDatabase& database) const override;
  std::string summary() const override;

 private:
  MatchResult alignOne(const CloudConstPtr& source, const CloudConstPtr& target,
                       const DescriptorCandidate& candidate,
                       const Eigen::Matrix4d& initial_guess) const;

  GlobalMatcherConfig config_;
};

}  // namespace relocalization
