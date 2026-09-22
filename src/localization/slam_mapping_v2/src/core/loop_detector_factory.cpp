/**
 * @file loop_detector_factory.cpp
 * @brief 回环检测器工厂实现
 */

#include "slam_mapping_v2/core/loop_detector_factory.hpp"
#include "slam_mapping_v2/core/loop_detector_kdtree_gicp.hpp"
#include "slam_mapping_v2/core/visual_loop_detector.hpp"

#include "syslog/logger.h"

namespace slam_mapping_v2 {

std::unique_ptr<ILoopDetector> LoopDetectorFactory::create(
    LoopDetectorType type, const LoopClosureConfig& cfg) {

  switch (type) {
    case LoopDetectorType::LASER_KDTREE_GICP: {
      auto detector = std::make_unique<KdtreeGicpLoopDetector>();
      detector->configure(cfg);
      return detector;
    }

    case LoopDetectorType::VISUAL_ORB_BOW: {
      auto detector = std::make_unique<VisualLoopDetector>();
      detector->configure(cfg);
      return detector;
    }

    case LoopDetectorType::HYBRID: {
      // TODO: 实现 HybridLoopDetector
      // ROS_WARN("[LoopDetectorFactory] Hybrid detector not implemented yet, falling back to laser");
      SYS_LOG(syslog::LogSeverity::WARNING, true) << "Hybrid detector not implemented yet, falling back to laser";

      auto detector = std::make_unique<KdtreeGicpLoopDetector>();
      detector->configure(cfg);
      return detector;
    }

    default:
      // ROS_ERROR("[LoopDetectorFactory] Unknown detector type: %d", static_cast<int>(type));
      SYS_LOG(syslog::LogSeverity::ERROR, true) << "Unknown detector type: " << static_cast<int>(type);
      return nullptr;
  }
}

std::unique_ptr<ILoopDetector> LoopDetectorFactory::create(
    const std::string& type_str, const LoopClosureConfig& cfg) {

  if (type_str == "laser") {
    return create(LoopDetectorType::LASER_KDTREE_GICP, cfg);
  }

  if (type_str == "visual") {
    return create(LoopDetectorType::VISUAL_ORB_BOW, cfg);
  }

  if (type_str == "hybrid") {
    return create(LoopDetectorType::HYBRID, cfg);
  }

  // ROS_ERROR("[LoopDetectorFactory] Unknown detector type string: %s", type_str.c_str());
  SYS_LOG(syslog::LogSeverity::ERROR, true) << "Unknown detector type string: " << type_str;
  return nullptr;
}

}  // namespace slam_mapping_v2
