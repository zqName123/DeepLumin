#pragma once

/**
 * @file cloud_unifier.hpp
 * @brief 源点云统一：把源点云送进虚拟系（纯数学，PCL 变换在 ROS 层执行）
 *
 * 硬契约：**gateway 输出的里程计与源点云必须同处虚拟系。**
 * 凡与 map_odom（= map → 云端系）组合的量（fusion 可视化点云、TF、path）必须同系，
 * 否则 map_odom 与统一源混乘 → path/TF 偏移而点云看似正常。
 *
 * | CloudMode      | 变换 |
 * |----------------|------|
 * | kRegistered    | T = glue（源自带注册云，源系 → 虚拟系） |
 * | kRawRegister   | T = unified_pose ⊕ base_to_lidar（原始云按统一位姿注册） |
 * | kNone          | 无云 |
 *
 * 注：glue = I 时 kRegistered 路径可零成本原样透传（ROS 层判断，避免 PCL 往返）。
 */

#include "local_odom_gateway/core/source_spec.hpp"

namespace local_odom_gateway {

class CloudUnifier {
 public:
  void setBaseToLidar(const Pose3d& p) { base_to_lidar_ = p; }
  const Pose3d& baseToLidar() const { return base_to_lidar_; }

  /// 该模式下点云需要施加的变换（源点云所在系 → 虚拟系）
  Pose3d transform(CloudMode mode, const Pose3d& glue,
                   const Pose3d& unified_pose) const {
    switch (mode) {
      case CloudMode::kRegistered:
        return glue;  // 源注册云 ⊕ glue → 虚拟系
      case CloudMode::kRawRegister:
        // T_unified_lidar = T_unified_base ⊕ T_base_lidar
        return compose(unified_pose, base_to_lidar_);
      case CloudMode::kNone:
      default:
        return identity();
    }
  }

  /// kRegistered 且 glue=I → 可原样透传（无需 PCL 往返）
  static bool canPassThrough(CloudMode mode, const Pose3d& glue) {
    return mode == CloudMode::kRegistered && isIdentity(glue);
  }

 private:
  Pose3d base_to_lidar_;  ///< T_base_lidar：必须与真实安装一致，且与 LIO extrinsic_R 同源
};

}  // namespace local_odom_gateway
