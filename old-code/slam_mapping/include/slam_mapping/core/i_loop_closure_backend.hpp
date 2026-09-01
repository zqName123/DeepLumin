#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

#include <string>
#include <vector>

namespace slam_mapping {

class ILoopClosureBackend {
 public:
  virtual ~ILoopClosureBackend() = default;

  virtual std::string name() const = 0;
  virtual bool supported() const = 0;
  virtual void configure(const LoopClosureConfig& cfg) = 0;
  virtual void reset() = 0;
  virtual LoopClosureResult update(std::vector<MappingKeyframe>* keyframes) = 0;
  virtual localization::PointCloudPtr latestCurrentSubmap() const = 0;
  virtual localization::PointCloudPtr latestHistorySubmap() const = 0;
  virtual std::pair<int, int> latestLoopPair() const = 0;
  /// 已添加 GNSS 因子的位置（map 系），默认空（未实现 GNSS 的后端无需覆盖）
  virtual std::vector<Eigen::Vector3d> gnssFactorPositions() const { return {}; }
  /// 被门控拒绝的 GNSS 位置（map 系），默认空
  virtual std::vector<Eigen::Vector3d> gnssRejectedPositions() const { return {}; }
  /// 已添加 GNSS 因子：(关键帧 id, map 系位置)，默认空
  virtual std::vector<std::pair<int, Eigen::Vector3d>> gnssAcceptedFactors() const {
    return {};
  }
  /// 被门控拒绝的 GNSS 因子：(关键帧 id, map 系位置)，默认空
  virtual std::vector<std::pair<int, Eigen::Vector3d>> gnssRejectedFactors() const {
    return {};
  }
  /// GNSS ENU->map 变换；未实现/未对齐返回 false
  virtual bool gnssMapEnuTransform(Eigen::Matrix3d* rot, Eigen::Vector3d* trans) const {
    (void)rot; (void)trans; return false;
  }
  /// GNSS ENU 原点经纬高；未设置返回 false
  virtual bool gnssEnuOrigin(double* lat_deg, double* lon_deg, double* alt) const {
    (void)lat_deg; (void)lon_deg; (void)alt; return false;
  }
};

}  // namespace slam_mapping
