#pragma once

#include <relocalization/interface/i_descriptor.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace relocalization {

struct ScanContextConfig {
  int num_rings = 20;
  int num_sectors = 60;
  double max_radius = 80.0;
  double min_range = 3.0;
  int ring_key_candidates = 50;
};

struct ScanContextEntry {
  int keyframe_id = -1;
  double timestamp = 0.0;
  Pose pose;
  Eigen::MatrixXd descriptor;
  Eigen::VectorXd ring_key;
  Eigen::VectorXd sector_key;
};

class ScanContextDescriptor : public IRelocalizationDescriptor {
 public:
  explicit ScanContextDescriptor(const ScanContextConfig& config = ScanContextConfig());

  const ScanContextConfig& config() const { return config_; }
  const std::vector<ScanContextEntry>& entries() const { return entries_; }

  void setRingKeyCandidates(int n) override { config_.ring_key_candidates = n; }
  Eigen::MatrixXd makeDescriptor(const CloudConstPtr& cloud) const override;
  Eigen::VectorXd makeRingKey(const Eigen::MatrixXd& descriptor) const;
  Eigen::VectorXd makeSectorKey(const Eigen::MatrixXd& descriptor) const;

  void clear();
  void add(int keyframe_id, double timestamp, const Pose& pose, const Eigen::MatrixXd& descriptor);
  std::vector<DescriptorCandidate> query(const Eigen::MatrixXd& descriptor, int top_k) const override;

  bool save(const std::string& path) const;
  bool load(const std::string& path) override;
  std::size_t size() const override { return entries_.size(); }
  std::string summary() const override;

 private:
  double descriptorDistance(const Eigen::MatrixXd& query,
                            const Eigen::MatrixXd& candidate,
                            int shift) const;
  int fastSectorAlignment(const Eigen::VectorXd& query_sector_key,
                          const Eigen::VectorXd& candidate_sector_key) const;

  ScanContextConfig config_;
  std::vector<ScanContextEntry> entries_;
};

}  // namespace relocalization
