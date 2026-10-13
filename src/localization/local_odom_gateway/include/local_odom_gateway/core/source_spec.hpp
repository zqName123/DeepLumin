#pragma once

/**
 * @file source_spec.hpp
 * @brief 前端里程计源的规格与统一输出类型（表驱动，新增源只加一条配置）
 *
 * 范围：本模块只管理**前端里程计**（SLAM / DR / 后续其它里程计）及其点云。
 * 传感器（IMU/CAN 等）由各自节点直订，不经本模块。
 */

#include <string>

#include <localization_common/types.hpp>

namespace local_odom_gateway {

using localization_common::Pose3d;
using localization_common::Twist;
using localization_common::TimedPose;
using localization_common::Vec3d;
using localization_common::Quatd;
using localization_common::identity;
using localization_common::isIdentity;
using localization_common::isFinite;
using localization_common::compose;
using localization_common::inverse;
using localization_common::between;
using localization_common::interpolate;
using localization_common::yawDiffDeg;
using localization_common::translationDiff;

/// 源标识（配置顺序即 id，便于表驱动仲裁）
using SourceId = int;
constexpr SourceId kInvalidSource = -1;

/// 源的点云能力
enum class CloudMode {
  kNone,          ///< 无云（GM 无法配）
  kRegistered,    ///< 源自带注册云（在源系，需 ⊕ glue 进虚拟系）
  kRawRegister,   ///< 无注册云：原始点云按统一源位姿注册
};

/// 源在运动链中的角色（两链隔离的基础）
enum class SourceRole {
  kAbsolute,  ///< 提供**绝对位姿**（锚）：SLAM / VIO 等，可作为主源
  kRelative,  ///< 提供**相对运动**（增量）：DR 等，从不提供锚
};

/// 运动链相位（两链隔离的形式化，见设计文档 §4.5）
enum class Phase {
  kNone = 0,   ///< 尚无输出
  kRunning,    ///< 主源样本正常节拍：输出 = 纯锚，观测链开，DR 增量不应用
  kGap,        ///< 主源样本超龄：输出 = 锚 ⊕ Δ，观测链关
  kOnBackup,   ///< 主源判死、备源接管：输出 = 锚 ⊕ Δ（锚冻结），观测链关
};

inline const char* toString(Phase p) {
  switch (p) {
    case Phase::kRunning:  return "RUNNING";
    case Phase::kGap:      return "GAP";
    case Phase::kOnBackup: return "ON_BACKUP";
    case Phase::kNone:
    default:               return "NONE";
  }
}

/// 源规格（配置驱动）
struct OdomSourceSpec {
  std::string name;
  SourceRole role = SourceRole::kAbsolute;  ///< 绝对源(锚) / 相对源(增量)
  int priority = 0;              ///< 高者优先（slam=10, dr=5, 其它按需）
  double timeout_sec = 0.5;      ///< 样本判死阈值（s）；0 = 用周期自适应
  double overdue_sec = 0.0;      ///< 样本超龄阈值（s）；0 = 用周期自适应
  int switch_back_confirm = 5;   ///< 降级源回切滞回帧数
  CloudMode cloud_mode = CloudMode::kNone;
};

/// 统一输出（同一时刻的两个视角，同处虚拟系）
struct Sample {
  Pose3d motion;   ///< 运动输出 → 下游
  Pose3d anchor;   ///< 纯锚（不含增量）→ 观测链
};

}  // namespace local_odom_gateway
