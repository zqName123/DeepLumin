#pragma once

#include <relocalization/core/common.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace relocalization {

struct DescriptorCandidate {
  int keyframe_id = -1;
  double score = 1e9;
  double ring_dist = 1e9;
  int sector_shift = 0;
  double yaw_diff = 0.0;
  Pose pose;
};

class IRelocalizationDescriptor {
 public:
  virtual ~IRelocalizationDescriptor() = default;

  virtual bool load(const std::string& path) = 0;
  virtual void setRingKeyCandidates(int n) = 0;
  virtual Eigen::MatrixXd makeDescriptor(const CloudConstPtr& cloud) const = 0;
  virtual std::vector<DescriptorCandidate> query(const Eigen::MatrixXd& descriptor, int top_k) const = 0;
  virtual std::size_t size() const = 0;
  virtual std::string summary() const = 0;
};

}  // namespace relocalization
