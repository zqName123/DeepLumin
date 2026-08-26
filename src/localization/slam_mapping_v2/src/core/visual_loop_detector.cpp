/**
 * @file visual_loop_detector.cpp
 * @brief 视觉回环检测器实现
 *
 * 检测流程：BoW检索 → 特征匹配 → 几何验证 → 连续性确认
 */

#include "slam_mapping_v2/core/visual_loop_detector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <ros/ros.h>

#include <algorithm>

namespace slam_mapping_v2 {

// ============================================================================
// ILoopDetector 接口实现
// ============================================================================

void VisualLoopDetector::configure(const LoopClosureConfig& cfg) {
  std::lock_guard<std::mutex> lock(mutex_);
  cfg_ = cfg;
  vcfg_ = cfg.visual;

  // 加载词库
  if (vcfg_.vocabulary_path.empty()) {
    ROS_ERROR("[VisualLoopDetector] vocabulary_path is empty!");
    throw std::runtime_error("VisualLoopDetector: vocabulary_path not configured");
  }

  voc_ = std::make_unique<OrbVocabulary>();

  try {
    voc_->loadFromTextFile(vcfg_.vocabulary_path);
    ROS_INFO("[VisualLoopDetector] Loaded vocabulary from %s (%d words, %d levels)",
             vcfg_.vocabulary_path.c_str(), voc_->size(), voc_->getDepthLevels());
  } catch (const std::exception& e) {
    ROS_ERROR("[VisualLoopDetector] Failed to load vocabulary: %s", e.what());
    throw;
  }

  // 初始化倒排索引
  inverted_file_.resize(voc_->size());

  ROS_INFO("[VisualLoopDetector] Configured: min_bow_score=%.3f, min_continuity=%d",
           vcfg_.min_bow_score, vcfg_.min_continuity);
}

void VisualLoopDetector::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  loop_history_.clear();
  last_check_index_ = -1;
  pending_history_idx_ = -1;
  continuity_count_ = 0;

  // 清空倒排索引
  for (auto& list : inverted_file_) {
    list.clear();
  }
  bow_vectors_.clear();
}

LoopCandidate VisualLoopDetector::detect(const std::vector<Keyframe>& kfs) {
  std::lock_guard<std::mutex> lock(mutex_);

  LoopCandidate result;
  if (!feature_buffer_ || kfs.empty()) {
    return result;
  }

  // 获取缓冲区中所有可用的关键帧 ID
  auto available_ids = feature_buffer_->availableIds();
  if (available_ids.empty()) {
    return result;
  }

  // 只检查最新的、有特征的关键帧
  int target_idx = -1;
  int target_kf_id = -1;
  for (int i = static_cast<int>(kfs.size()) - 1; i >= 0; --i) {
    if (std::find(available_ids.begin(), available_ids.end(), kfs[i].id) != available_ids.end()) {
      target_idx = i;
      target_kf_id = kfs[i].id;
      break;
    }
  }

  if (target_idx < 0) {
    return result;
  }

  // 前置检查
  if (!shouldCheck(kfs, target_idx)) {
    return result;
  }

  // 从缓冲区获取当前帧的视觉特征
  VisualFrame current_frame;
  if (!feature_buffer_->get(target_kf_id, &current_frame) || !current_frame.features_ready) {
    return result;
  }

  int current_idx = target_idx;
  const auto& current_kf = kfs[current_idx];

  ROS_INFO_THROTTLE(2.0, "[VisualLoopDetector] Checking kf %d (id=%d), keypoints=%zu, buffer_size=%zu",
                    current_idx, current_kf.id, current_frame.keypoints.size(), feature_buffer_->size());

  // 计算 BoW 向量
  DBoW2::BowVector bow_vec;
  DBoW2::FeatureVector feat_vec;
  computeBowVector(current_frame, &bow_vec, &feat_vec);

  // 将当前帧加入倒排索引
  int current_kf_id = current_kf.id;
  bow_vectors_[current_kf_id] = bow_vec;
  for (const auto& [word_id, value] : bow_vec) {
    if (word_id < static_cast<int>(inverted_file_.size())) {
      inverted_file_[word_id].push_back(current_kf_id);
    }
  }

  // 检索候选帧
  auto candidates = retrieveCandidates(bow_vec, kfs, current_idx, vcfg_.min_bow_score);

  ROS_INFO_THROTTLE(2.0, "[VisualLoopDetector] kf=%d/%d keypoints=%zu bow_words=%zu candidates=%zu",
                    current_idx, current_kf.id, current_frame.keypoints.size(), bow_vec.size(), candidates.size());

  if (candidates.empty()) {
    return result;
  }

  // 对每个候选帧进行验证
  for (const auto& cand : candidates) {
    VisualFrame cand_frame;
    if (!feature_buffer_->get(cand.keyframe_id, &cand_frame) || !cand_frame.features_ready) {
      continue;
    }

    auto bow_it = bow_vectors_.find(cand.keyframe_id);
    if (bow_it == bow_vectors_.end()) {
      continue;
    }
    DBoW2::FeatureVector cand_fvec;
    if (cand_frame.descriptors.empty()) continue;
    std::vector<cv::Mat> desc_vec;
    desc_vec.reserve(cand_frame.descriptors.rows);
    for (int i = 0; i < cand_frame.descriptors.rows; ++i) {
      desc_vec.push_back(cand_frame.descriptors.row(i));
    }
    DBoW2::BowVector temp_bow;
    voc_->transform(desc_vec, temp_bow, cand_fvec, 4);

    // BoW 特征匹配
    auto match_result = matchFeatures(current_frame, cand_frame, feat_vec, cand_fvec);

    ROS_INFO_THROTTLE(2.0, "[VisualLoopDetector] cand=%d bow_score=%.3f matches=%d (min=%d)",
                      cand.keyframe_id, cand.bow_score, match_result.num_matches, vcfg_.min_bow_matches);

    if (match_result.num_matches < vcfg_.min_bow_matches) {
      continue;
    }

    // 几何验证
    Mat3d diff_rot;
    Vec3d diff_pos;
    int inliers = 0;

    if (!geometricVerification(current_frame, cand_frame, match_result,
                               &diff_rot, &diff_pos, &inliers)) {
      ROS_INFO_THROTTLE(2.0, "[VisualLoopDetector] GeoVerify FAIL: %d->%d inliers=%d (min=%d)",
                        current_idx, cand.keyframe_id, inliers, vcfg_.ransac_min_inliers);
      continue;
    }

    ROS_INFO("[VisualLoopDetector] GeoVerify OK: %d->%d inliers=%d", current_idx, cand.keyframe_id, inliers);

    // 连续性验证
    if (!continuityCheck(current_idx, cand.keyframe_id)) {
      ROS_INFO("[VisualLoopDetector] Continuity pending: %d->%d count=%d/%d",
               current_idx, cand.keyframe_id, continuity_count_, vcfg_.min_continuity);
      continue;
    }

    // 回环确认！
    result.valid = true;
    result.current_idx = current_idx;
    result.history_idx = cand.keyframe_id;
    result.score = 1.0 - cand.bow_score;

    // 平移尺度恢复
    const auto& pre_kf = kfs[cand.keyframe_id];
    const auto& cur_kf = kfs[current_idx];
    double scale = (cur_kf.pose_odom.position - pre_kf.pose_odom.position).norm();
    if (scale < 0.1) {
      ROS_WARN("[VisualLoopDetector] Odom distance too small (%.3fm), skipping loop %d->%d",
               scale, current_idx, cand.keyframe_id);
      result.valid = false;
      continue;
    }

    // 坐标系转换
    const Mat3d& R_L_C = vcfg_.T_L_C.block<3, 3>(0, 0);
    const Vec3d& t_L_C = vcfg_.T_L_C.block<3, 1>(0, 3);
    result.diff_rot = R_L_C * diff_rot * R_L_C.transpose();
    const Vec3d diff_pos_scaled = scale * diff_pos.normalized();
    result.diff_pos = R_L_C * diff_pos_scaled + t_L_C - R_L_C * diff_rot * R_L_C.transpose() * t_L_C;

    loop_history_.push_back({current_idx, cand.keyframe_id});
    last_check_index_ = current_idx;

    ROS_INFO("[VisualLoopDetector] Loop detected: %d -> %d (bow_score=%.3f, fitness=%.3f, inliers=%d, scale=%.3fm)",
             current_idx, cand.keyframe_id, cand.bow_score, result.score, inliers, scale);
    return result;
  }

  return result;
}

// ============================================================================
// 内部方法
// ============================================================================

bool VisualLoopDetector::shouldCheck(const std::vector<Keyframe>& kfs,
                                     int current_idx) const {
  // 关键帧数量检查
  if (static_cast<int>(kfs.size()) < cfg_.min_index_gap) {
    return false;
  }

  // 间隔检查
  if (current_idx - last_check_index_ < cfg_.loop_kf_gap) {
    return false;
  }

  return true;
}

void VisualLoopDetector::computeBowVector(const VisualFrame& frame,
                                          DBoW2::BowVector* bow_vec,
                                          DBoW2::FeatureVector* feat_vec) {
  if (!voc_ || frame.descriptors.empty()) {
    return;
  }

  // 将描述子转换为 vector<cv::Mat> 格式
  std::vector<cv::Mat> descriptor_vec;
  descriptor_vec.reserve(frame.descriptors.rows);

  for (int i = 0; i < frame.descriptors.rows; ++i) {
    descriptor_vec.push_back(frame.descriptors.row(i));
  }

  voc_->transform(descriptor_vec, *bow_vec, *feat_vec, 4);
}

std::vector<BowQueryResult> VisualLoopDetector::retrieveCandidates(
    const DBoW2::BowVector& query_bow,
    const std::vector<Keyframe>& kfs,
    int current_idx, double min_score) {

  // 使用倒排索引统计每个关键帧与查询帧共享的单词数
  std::unordered_map<int, int> shared_words_count;

  for (const auto& [word_id, value] : query_bow) {
    if (word_id < static_cast<int>(inverted_file_.size())) {
      for (int kf_id : inverted_file_[word_id]) {
        shared_words_count[kf_id]++;
      }
    }
  }

  // 过滤和排序候选
  std::vector<BowQueryResult> candidates;

  for (const auto& [kf_id, shared_count] : shared_words_count) {
    // 过滤条件
    // 1. 排除最近的帧
    if (current_idx - kf_id < cfg_.min_index_gap) {
      continue;
    }

    // 2. 共享单词数检查
    if (shared_count < vcfg_.min_shared_words) {
      continue;
    }

    // 3. 计算精确 BoW 分数（L1_NORM）
    auto bow_it = bow_vectors_.find(kf_id);
    if (bow_it == bow_vectors_.end()) {
      continue;
    }

    double score = voc_->score(query_bow, bow_it->second);
    if (score < min_score) {
      continue;
    }

    BowQueryResult cand;
    cand.keyframe_id = kf_id;
    cand.bow_score = score;
    cand.shared_words = shared_count;
    candidates.push_back(cand);
  }

  // 按分数降序排序
  std::sort(candidates.begin(), candidates.end(),
            [](const BowQueryResult& a, const BowQueryResult& b) {
              return a.bow_score > b.bow_score;
            });

  // 取 Top-N
  if (static_cast<int>(candidates.size()) > vcfg_.max_candidates) {
    candidates.resize(vcfg_.max_candidates);
  }

  return candidates;
}

BoWMatchResult VisualLoopDetector::matchFeatures(
    const VisualFrame& query,
    const VisualFrame& candidate,
    const DBoW2::FeatureVector& query_fvec,
    const DBoW2::FeatureVector& cand_fvec) {

  BoWMatchResult result;

  if (query.descriptors.empty() || candidate.descriptors.empty()) {
    return result;
  }

  // 使用 BoW 加速匹配（同节点内比较）
  auto qit = query_fvec.begin();
  auto cit = cand_fvec.begin();

  while (qit != query_fvec.end() && cit != cand_fvec.end()) {
    if (qit->first == cit->first) {
      // 同一个 BoW 节点，比较其中的描述子
      const auto& q_indices = qit->second;
      const auto& c_indices = cit->second;

      for (size_t i = 0; i < q_indices.size(); ++i) {
        int q_idx = q_indices[i];
        const cv::Mat& q_desc = query.descriptors.row(q_idx);

        int best_idx = -1;
        int best_dist = std::numeric_limits<int>::max();
        int second_dist = std::numeric_limits<int>::max();

        for (size_t j = 0; j < c_indices.size(); ++j) {
          int c_idx = c_indices[j];
          const cv::Mat& c_desc = candidate.descriptors.row(c_idx);

          // Hamming 距离
          int dist = cv::norm(q_desc, c_desc, cv::NORM_HAMMING);

          if (dist < best_dist) {
            second_dist = best_dist;
            best_dist = dist;
            best_idx = c_idx;
          } else if (dist < second_dist) {
            second_dist = dist;
          }
        }

        // Lowe's ratio test
        if (best_idx >= 0 && best_dist < vcfg_.nn_ratio * second_dist) {
          result.matches.push_back({q_idx, best_idx});
        }
      }

      ++qit;
      ++cit;
    } else if (qit->first < cit->first) {
      ++qit;
    } else {
      ++cit;
    }
  }

  result.num_matches = static_cast<int>(result.matches.size());
  return result;
}

bool VisualLoopDetector::geometricVerification(
    const VisualFrame& query,
    const VisualFrame& candidate,
    const BoWMatchResult& matches,
    Mat3d* out_rot, Vec3d* out_pos, int* out_inliers) {

  if (matches.num_matches < vcfg_.ransac_min_inliers) {
    return false;
  }

  // 提取匹配点对
  std::vector<cv::Point2f> pts1, pts2;
  pts1.reserve(matches.matches.size());
  pts2.reserve(matches.matches.size());

  for (const auto& [qi, ci] : matches.matches) {
    pts1.push_back(query.keypoints[qi].pt);
    pts2.push_back(candidate.keypoints[ci].pt);
  }

  // RANSAC 求 Fundamental 矩阵
  cv::Mat mask;
  cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC,
                                      3.0, vcfg_.ransac_probability, mask);

  if (F.empty()) {
    return false;
  }

  // 统计内点数
  int inliers = cv::countNonZero(mask);
  if (inliers < vcfg_.ransac_min_inliers) {
    return false;
  }

  // 从 Fundamental 恢复 R, t（需要相机内参）
  cv::Mat K = (cv::Mat_<double>(3, 3) <<
    vcfg_.camera.fx, 0, vcfg_.camera.cx,
    0, vcfg_.camera.fy, vcfg_.camera.cy,
    0, 0, 1);

  cv::Mat E = K.t() * F * K;  // Essential 矩阵

  cv::Mat R, t;
  cv::recoverPose(E, pts1, pts2, K, R, t, mask);

  // 输出结果
  *out_rot = Eigen::Map<Eigen::Matrix3d>(R.ptr<double>()).transpose();
  *out_pos = Eigen::Map<Eigen::Vector3d>(t.ptr<double>());
  *out_inliers = inliers;

  return true;
}

bool VisualLoopDetector::continuityCheck(int current_idx, int history_idx) {
  // 如果指向同一个历史帧，增加连续性计数
  if (history_idx == pending_history_idx_) {
    continuity_count_++;
  } else {
    // 新的候选帧，重置计数
    pending_history_idx_ = history_idx;
    continuity_count_ = 1;
  }

  // 检查是否满足连续性要求
  if (continuity_count_ >= vcfg_.min_continuity) {
    // 确认回环，重置状态
    continuity_count_ = 0;
    pending_history_idx_ = -1;
    last_check_index_ = current_idx;
    return true;
  }

  return false;
}

void VisualLoopDetector::setFeatureBuffer(
    std::shared_ptr<VisualFeatureBuffer> buffer) {
  feature_buffer_ = std::move(buffer);
}

}  // namespace slam_mapping_v2
