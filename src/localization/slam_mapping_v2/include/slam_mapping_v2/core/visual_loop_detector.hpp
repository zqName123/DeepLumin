#pragma once

/**
 * @file visual_loop_detector.hpp
 * @brief 基于视觉 ORB 特征的回环检测器
 *
 * 实现 ILoopDetector 接口，与现有后端循环完全兼容。
 * 检测流程：BoW检索 → 特征匹配 → 几何验证 → 连续性确认 → 返回 LoopCandidate
 */

#include "slam_mapping_v2/core/i_loop_detector.hpp"
#include "slam_mapping_v2/core/visual_feature_buffer.hpp"

#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#include <DBoW2/FORB.h>
#include <DBoW2/ScoringObject.h>
#include <DBoW2/TemplatedVocabulary.h>

#include <memory>
#include <mutex>
#include <unordered_map>

namespace slam_mapping_v2 {

/// 基于视觉 ORB 特征的回环检测器
class VisualLoopDetector : public ILoopDetector {
 public:
  VisualLoopDetector() = default;
  ~VisualLoopDetector() override = default;

  // ===== ILoopDetector 接口 =====
  void configure(const LoopClosureConfig& cfg) override;
  void reset() override;
  LoopCandidate detect(const std::vector<Keyframe>& kfs) override;
  const std::vector<std::pair<int, int>>& loopPairs() const override { return loop_history_; }
  PointCloudPtr latestCurrentSubmap() const override { return nullptr; }
  PointCloudPtr latestHistorySubmap() const override { return nullptr; }

  // ===== 视觉特有接口 =====
  void setFeatureBuffer(std::shared_ptr<VisualFeatureBuffer> buffer);

 private:
  /// 判断是否需要执行回环检测
  bool shouldCheck(const std::vector<Keyframe>& kfs, int current_idx) const;

  /// 计算 BoW 向量
  void computeBowVector(const VisualFrame& frame,
                        DBoW2::BowVector* bow_vec,
                        DBoW2::FeatureVector* feat_vec);

  /// 检索候选帧
  std::vector<BowQueryResult> retrieveCandidates(
      const DBoW2::BowVector& query_bow,
      const std::vector<Keyframe>& kfs,
      int current_idx, double min_score);

  /// BoW 加速特征匹配
  BoWMatchResult matchFeatures(const VisualFrame& query,
                               const VisualFrame& candidate,
                               const DBoW2::FeatureVector& query_fvec,
                               const DBoW2::FeatureVector& cand_fvec);

  /// 几何验证（Fundamental 矩阵 + RANSAC）
  bool geometricVerification(const VisualFrame& query,
                             const VisualFrame& candidate,
                             const BoWMatchResult& matches,
                             Mat3d* out_rot, Vec3d* out_pos, int* out_inliers);

  /// 连续性验证
  bool continuityCheck(int current_idx, int history_idx);

  LoopClosureConfig cfg_;
  VisualLoopConfig vcfg_;
  std::shared_ptr<VisualFeatureBuffer> feature_buffer_;

  // DBoW2 词库
  using OrbVocabulary = DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;
  std::unique_ptr<OrbVocabulary> voc_;

  // 倒排索引：word_id → [keyframe_id, ...]
  std::vector<std::vector<int>> inverted_file_;
  // 已入库的关键帧 BoW 向量（用于精确分数计算）
  std::unordered_map<int, DBoW2::BowVector> bow_vectors_;

  // 状态
  std::vector<std::pair<int, int>> loop_history_;
  int last_check_index_ = -1;
  int pending_history_idx_ = -1;
  int continuity_count_ = 0;
  std::mutex mutex_;
};

}  // namespace slam_mapping_v2
