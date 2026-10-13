#include "localization_v3/ros/status_publisher.hpp"

namespace localization_v3 {

namespace {

double ageOr(double age, double fallback) {
  return age >= 0.0 ? age : fallback;
}

}  // namespace

deeplumin_msgs::FusionStatus makeFusionStatus(const StatusInput& in, const ros::Time& stamp,
                                              const std::string& map_frame) {
  deeplumin_msgs::FusionStatus msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = map_frame;
  msg.current_map_id = in.map_id;

  // ========== 真实数据新鲜度 ==========
  msg.last_slam_age = ageOr(in.slam_age, 1e9);
  msg.last_global_matcher_age = ageOr(in.gm_age, 1e9);
  msg.last_relocalization_age = ageOr(in.reloc_age, 1e9);
  msg.last_dr_age = 1e9;   // 未使用 DR
  msg.last_gnss_age = 1e9; // 未使用 GNSS
  msg.last_can_age = 0.0;
  msg.last_imu_age = 0.0;
  msg.last_lidar_age = ageOr(in.slam_age, 1e9);

  // ========== 状态 → level / output_valid / failure_reason ==========
  switch (in.state) {
    case LocalizationStateMachine::State::kUninitialized:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
      msg.state = "UNINITIALIZED";
      msg.initialized = false;
      msg.output_valid = false;
      msg.failure_reason = "waiting for initial pose";
      msg.position_std = 100.0;
      msg.velocity_std = 10.0;
      msg.roll_std = 90.0;
      msg.pitch_std = 90.0;
      msg.yaw_std = 180.0;
      break;

    case LocalizationStateMachine::State::kRelocalizing:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
      msg.state = "RELOCALIZING";
      msg.initialized = false;
      msg.output_valid = false;
      msg.failure_reason = "relocalization in progress for initial pose";
      msg.position_std = 50.0;
      msg.velocity_std = 5.0;
      msg.roll_std = 45.0;
      msg.pitch_std = 45.0;
      msg.yaw_std = 90.0;
      break;

    case LocalizationStateMachine::State::kTracking: {
      msg.state = "TRACKING";
      msg.initialized = true;
      msg.output_valid = true;
      if (in.gm_failures == 0 && in.gm_degraded == 0) {
        msg.level = deeplumin_msgs::FusionStatus::LEVEL_NORMAL;
        msg.failure_reason = in.live_failure_reason;
      } else {
        msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
        msg.failure_reason = "GM quality: consecutive_failures=" + std::to_string(in.gm_failures) +
                             ", degraded_frames=" + std::to_string(in.gm_degraded);
      }
      msg.position_std = 0.1;
      msg.velocity_std = 0.5;
      msg.roll_std = 0.5;
      msg.pitch_std = 0.5;
      msg.yaw_std = 1.0;
      break;
    }

    case LocalizationStateMachine::State::kTrackingLost:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_DEGRADED;
      msg.state = "TRACKING_LOST";
      msg.initialized = true;
      msg.output_valid = in.allow_degraded_output;
      msg.failure_reason = in.live_failure_reason.empty()
                               ? ("tracking lost, GM consecutive failures=" +
                                  std::to_string(in.gm_failures) +
                                  ", relocalization attempting recovery")
                               : in.live_failure_reason;
      msg.position_std = 5.0;
      msg.velocity_std = 2.0;
      msg.roll_std = 5.0;
      msg.pitch_std = 5.0;
      msg.yaw_std = 10.0;
      break;

    case LocalizationStateMachine::State::kLocalizationFailure:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_FAILURE;
      msg.state = "LOCALIZATION_FAILURE";
      msg.initialized = true;
      msg.output_valid = false;
      msg.failure_reason = in.live_failure_reason.empty()
                               ? ("localization failure, reloc failures=" +
                                  std::to_string(in.reloc_failures))
                               : in.live_failure_reason;
      msg.position_std = 100.0;
      msg.velocity_std = 10.0;
      msg.roll_std = 90.0;
      msg.pitch_std = 90.0;
      msg.yaw_std = 180.0;
      break;
  }

  // ========== 数据源 ==========
  msg.used_sources.clear();
  msg.rejected_sources.clear();
  msg.reject_reasons.clear();

  if (in.have_slam) {
    msg.used_sources.push_back("slam_odometry_v2");
  }

  switch (in.state) {
    case LocalizationStateMachine::State::kTracking:
      if (in.have_map_odom) {
        msg.used_sources.push_back("global_matcher");
      }
      break;

    case LocalizationStateMachine::State::kTrackingLost:
      msg.rejected_sources.push_back("global_matcher");
      msg.reject_reasons.push_back("tracking lost");
      if (in.reloc_running) {
        msg.used_sources.push_back("relocalization_attempting");
      }
      break;

    case LocalizationStateMachine::State::kLocalizationFailure:
      msg.rejected_sources.push_back("global_matcher");
      msg.rejected_sources.push_back("relocalization");
      msg.reject_reasons.push_back("GM tracking lost");
      msg.reject_reasons.push_back("relocalization recovery failed");
      break;

    default:
      if (in.reloc_running) {
        msg.used_sources.push_back("relocalization");
      }
      break;
  }

  // ========== 可用性（由真实年龄导出） ==========
  msg.is_slam_available = in.slam_age >= 0.0 && in.slam_age < 1.0;
  msg.is_global_matcher_available = in.gm_age >= 0.0 && in.gm_age < 10.0;
  msg.is_relocalization_available = in.reloc_running || (in.reloc_age >= 0.0 && in.reloc_age < 10.0);
  msg.is_dr_available = false;
  msg.is_gnss_available = false;
  msg.is_can_available = true;
  msg.is_imu_available = true;
  msg.is_lidar_available = msg.is_slam_available;

  return msg;
}

}  // namespace localization_v3
