#pragma once

#include "lio_odometry/core/types.h"

namespace slam_odometry {

struct CorrectionConfig {
    double max_jump_rot_deg = 30.0;
    double max_jump_trans = 2.0;
    int rebuild_window = 50;
    bool enable = false;
};

struct CorrectionInput {
    double timestamp = 0.0;
    bool valid = false;
    Eigen::Quaterniond q_map_odom = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_map_odom = Eigen::Vector3d::Zero();
    int keyframe_id = -1;
    std::string source = "unknown";
};

class LaserOdometryCore;

class CorrectionHandler {
   public:
    bool Init(const CorrectionConfig& config) { return true; }
    void PushCorrection(const CorrectionInput& input) {}
    bool HasPendingCorrection() const { return false; }
    bool Apply(LaserOdometryCore* core) { return false; }

   private:
    CorrectionConfig config_;
};

}  // namespace slam_odometry