#pragma once

/**
 * @file csc_descriptor.hpp
 * @brief Cylindrical Scan Context (CSC) 描述子（从 relocalization_framework 移植）
 *
 * 与 relocalization_framework 的 CscDescriptor/CscDatabase 行为一致：
 *   - 4 通道：max_range / point_density(对数归一化) / mean_intensity / max_z
 *   - 预处理：距离裁剪 [min_range, max_radius]，可选地面点过滤（ground_threshold）
 *   - Z-Key 粗筛仅几何通道（排除强度）
 *   - 精匹配：similarity=l1（论文式）| balanced（逐格归一化 + 距离带加权）
 *   - DB 格式：magic "SCR_CSC_DB_V2"（与框架 .bin 完全一致，可复用 csc_database_v7.bin）
 */

#include <relocalization/interface/i_descriptor.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace relocalization {

struct CscConfig {
  int num_sectors = 60;
  int num_height_bins = 16;
  double min_height = -1000.0;
  double max_height = 15.0;
  int num_channels = 4;
  double max_radius = 100.0;
  double min_range = 3.0;
  double ground_threshold = -1000.0;
  std::vector<double> channel_weights = {0.6, 0.5, 0.2, 0.3};
  std::string similarity = "l1";
  int ring_key_candidates = 300;
};

struct CscEntry {
  int keyframe_id = -1;
  double timestamp = 0.0;
  Pose pose;
  Eigen::MatrixXd descriptor;
  Eigen::VectorXd ring_key;
  Eigen::VectorXd sector_key;
};

class CscDescriptor : public IRelocalizationDescriptor {
 public:
  explicit CscDescriptor(const CscConfig& config = CscConfig());

  const CscConfig& config() const { return config_; }
  const std::vector<CscEntry>& entries() const { return entries_; }

  void setRingKeyCandidates(int n) override { config_.ring_key_candidates = n; }
  Eigen::MatrixXd makeDescriptor(const CloudConstPtr& cloud) const override;
  std::vector<DescriptorCandidate> query(const Eigen::MatrixXd& descriptor, int top_k) const override;

  bool load(const std::string& path) override;
  std::size_t size() const override { return entries_.size(); }
  std::string summary() const override;

 private:
  Eigen::VectorXd makeRingKey(const Eigen::MatrixXd& descriptor) const;
  Eigen::VectorXd makeSectorKey(const Eigen::MatrixXd& descriptor) const;
  double bestDescriptorScore(const Eigen::MatrixXd& query, const Eigen::MatrixXd& candidate,
                             int* best_shift) const;

  CscConfig config_;
  std::vector<CscEntry> entries_;
};

}  // namespace relocalization
