#pragma once

/**
 * @file loop_detector_factory.hpp
 * @brief 回环检测器工厂（配置驱动创建）
 */

#include "slam_mapping_v2/core/i_loop_detector.hpp"

#include <memory>
#include <string>

namespace slam_mapping_v2 {

/// 回环检测器类型枚举
enum class LoopDetectorType {
  LASER_KDTREE_GICP,   // 现有激光方案
  VISUAL_ORB_BOW,      // 视觉 ORB + DBoW2
  HYBRID,              // 混合（激光 + 视觉）
};

/// 回环检测器工厂
class LoopDetectorFactory {
 public:
  /// 根据枚举类型创建检测器
  static std::unique_ptr<ILoopDetector> create(
      LoopDetectorType type, const LoopClosureConfig& cfg);

  /// 根据字符串类型创建检测器（"laser" | "visual" | "hybrid"）
  static std::unique_ptr<ILoopDetector> create(
      const std::string& type_str, const LoopClosureConfig& cfg);
};

}  // namespace slam_mapping_v2
