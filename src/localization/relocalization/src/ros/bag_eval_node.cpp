#include <relocalization/core/relocalization_core.hpp>
#include <relocalization/ros/eval_visualizer.hpp>
#include <relocalization/ros/param_loader.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace {

double translationError(const Eigen::Matrix4d& result, const relocalization::Pose& truth) {
  return (result.block<3, 1>(0, 3) - truth.t).norm();
}

double yawErrorDeg(const Eigen::Matrix4d& result, const relocalization::Pose& truth) {
  const Eigen::Quaterniond q_result(result.block<3, 3>(0, 0));
  const double yaw_result = relocalization::yawFromQuaternion(q_result);
  const double yaw_truth = relocalization::yawFromQuaternion(truth.q);
  return std::abs(relocalization::normalizeAngle(yaw_result - yaw_truth)) * 180.0 / M_PI;
}

double xyDistance(const relocalization::Pose& lhs, const relocalization::Pose& rhs) {
  return (lhs.t.head<2>() - rhs.t.head<2>()).norm();
}

relocalization::Pose poseFromTransform(const Eigen::Matrix4d& transform) {
  relocalization::Pose pose;
  pose.t = transform.block<3, 1>(0, 3);
  pose.q = Eigen::Quaterniond(transform.block<3, 3>(0, 0)).normalized();
  return pose;
}

double radToDeg(double rad) {
  return rad * 180.0 / M_PI;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_eval_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  const auto cfg = relocalization_ros::loadBagEvalConfig(pnh);
  if (cfg.bag_file.empty()) {
    ROS_ERROR("bag_file is empty. Example: roslaunch relocalization bag_eval_viz.launch bag_file:=/path/to/data.bag");
    return 1;
  }

  relocalization::RelocalizationCore core;
  if (!core.initialize(cfg.core)) {
    return 1;
  }

  std::vector<relocalization::Pose> truth_poses;
  if (cfg.use_truth_pose) {
    truth_poses = relocalization::loadTumPoses(cfg.core.database.pose_file);
    if (truth_poses.empty()) {
      ROS_ERROR_STREAM("bag_use_truth_pose=true but no poses in: " << cfg.core.database.pose_file);
      return 1;
    }
  }

  ros::NodeHandle viz_nh(nh, "offline_eval");
  relocalization::EvalVisualizer visualizer(viz_nh, cfg.viz);

  std::ofstream report(cfg.report_csv);
  if (!report.is_open()) {
    ROS_ERROR_STREAM("Failed to open report: " << cfg.report_csv);
    return 1;
  }
  if (cfg.use_truth_pose) {
    report << "mode,query_index,bag_time,truth_pose_id,top1_id,top1_ring_dist,top1_sc_score,topk_hit,success,fitness,inlier_ratio,inlier_rmse,inlier_count,correspondence_count,translation_error,yaw_error_deg,pose_time_diff,sc_sector_shift,sc_yaw_diff_deg,candidate_yaw_deg,initial_yaw_deg,final_yaw_deg,gicp_delta_trans,gicp_delta_yaw_deg,gicp_source_points,gicp_target_points,elapsed_ms\n";
  } else {
    report << "mode,query_index,bag_time,top1_id,top1_ring_dist,top1_sc_score,success,converged,fitness,inlier_ratio,inlier_rmse,inlier_count,correspondence_count,candidate_id,sc_sector_shift,sc_yaw_diff_deg,candidate_yaw_deg,initial_yaw_deg,final_yaw_deg,gicp_delta_trans,gicp_delta_yaw_deg,gicp_source_points,gicp_target_points,elapsed_ms\n";
  }

  rosbag::Bag bag;
  try {
    bag.open(cfg.bag_file, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to open bag: " << cfg.bag_file << " error: " << e.what());
    return 1;
  }

  rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{cfg.lidar_topic}));
  if (view.size() == 0) {
    ROS_ERROR_STREAM("No messages on topic " << cfg.lidar_topic << " in bag " << cfg.bag_file);
    return 1;
  }

  const ros::Time bag_begin = view.getBeginTime();
  const ros::Time bag_end = view.getEndTime();
  const ros::Time eval_begin = bag_begin + ros::Duration(cfg.bag_start_offset);
  const ros::Time eval_end = cfg.bag_duration > 0.0 ? eval_begin + ros::Duration(cfg.bag_duration) : bag_end;

  ROS_INFO_STREAM("Bag eval: " << cfg.bag_file);
  ROS_INFO_STREAM("  mode=" << (cfg.use_truth_pose ? "truth_compare" : "map_align"));
  ROS_INFO_STREAM("  topic=" << cfg.lidar_topic << " messages=" << view.size());
  ROS_INFO_STREAM("  eval window [" << eval_begin << ", " << eval_end << "] stride=" << cfg.cloud_stride);

  if (cfg.use_truth_pose) {
    const double bag_t0 = eval_begin.toSec();
    const double bag_t1 = eval_end.toSec();
    const double pose_t0 = truth_poses.front().timestamp;
    const double pose_t1 = truth_poses.back().timestamp;
    const bool time_overlap = !(bag_t1 < pose_t0 - cfg.max_pose_time_diff ||
                                bag_t0 > pose_t1 + cfg.max_pose_time_diff);
    if (!time_overlap) {
      ROS_ERROR("Bag timestamp range does NOT overlap pose_file. Use bag_use_truth_pose:=false for map-align eval.");
      return 1;
    }
  }

  int query_count = 0;
  int cloud_count = 0;
  int top1_hit = 0;
  int topk_hit = 0;
  int reloc_success = 0;

  for (const rosbag::MessageInstance& instance : view) {
    if (instance.getTime() < eval_begin || instance.getTime() > eval_end) {
      continue;
    }
    if (cfg.max_queries > 0 && query_count >= cfg.max_queries) {
      break;
    }

    sensor_msgs::PointCloud2::ConstPtr msg = instance.instantiate<sensor_msgs::PointCloud2>();
    if (!msg) {
      continue;
    }
    if (cloud_count++ % std::max(1, cfg.cloud_stride) != 0) {
      continue;
    }

    const double bag_time = msg->header.stamp.toSec();
    try {
      relocalization::CloudPtr raw = relocalization::cloudFromRosMsg(*msg);
      double query_frame_yaw_rad = cfg.query_frame_yaw_deg * M_PI / 180.0;
      if (cfg.query_rotate_xy_y_negx) {
        query_frame_yaw_rad += -M_PI / 2.0;
      }
      relocalization::CloudPtr query_local = std::abs(query_frame_yaw_rad) > 1e-9
          ? relocalization::rotateCloudYaw(raw, query_frame_yaw_rad)
          : raw;
      relocalization::CloudPtr source = relocalization::preprocessCloud(query_local, cfg.core.database.preprocess);
      if (!source || source->empty()) {
        ROS_WARN_STREAM("Skip empty cloud at bag_time=" << bag_time);
        continue;
      }

      relocalization::Pose truth;
      relocalization::PoseLookupResult truth_lookup;
      if (cfg.use_truth_pose) {
        truth_lookup = relocalization::lookupPoseByTimestamp(truth_poses, bag_time, cfg.max_pose_time_diff);
        if (!truth_lookup.ok) {
          ROS_WARN_STREAM_THROTTLE(2.0, "Skip cloud at bag_time=" << std::fixed << bag_time
                                  << " (no pose within " << cfg.max_pose_time_diff << "s)");
          continue;
        }
        truth = truth_lookup.pose;
      }

      const auto candidates = core.queryCandidates(source);
      if (candidates.empty()) {
        continue;
      }

      if (cfg.log_topk_candidates) {
        std::ostringstream topk_ss;
        topk_ss << "bag_query=" << (query_count + 1)
                 << " bag_time=" << std::fixed << std::setprecision(3) << bag_time
                 << " topk_candidates=" << candidates.size();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
          topk_ss << " | #" << i << " id=" << candidates[i].keyframe_id
                  << " score=" << std::setprecision(4) << candidates[i].score;
        }
        ROS_INFO_STREAM(topk_ss.str());
      }

      bool hit = false;
      if (cfg.use_truth_pose) {
        for (const auto& c : candidates) {
          if (xyDistance(c.pose, truth) <= cfg.place_success_radius) {
            hit = true;
            break;
          }
        }
        if (xyDistance(candidates.front().pose, truth) <= cfg.place_success_radius) {
          ++top1_hit;
        }
        if (hit) {
          ++topk_hit;
        }
      }

      relocalization::MatchResult result;
      bool ok = false;
      double t_err = 0.0;
      double yaw_err = 0.0;
      if (cfg.run_gicp) {
        result = core.matchCandidates(source, candidates);
        if (cfg.use_truth_pose) {
          t_err = result.success ? translationError(result.transform, truth) : 1e9;
          yaw_err = result.success ? yawErrorDeg(result.transform, truth) : 1e9;
          ok = result.success && t_err <= cfg.success_translation && yaw_err <= cfg.success_yaw_deg;
        } else {
          ok = result.success;
        }
      } else if (cfg.use_truth_pose) {
        ok = hit;
      }
      if (ok) {
        ++reloc_success;
      }
      ++query_count;

      const auto& top1 = candidates.front();
      if (cfg.use_truth_pose) {
        report << "truth_compare," << query_count << "," << std::setprecision(9) << bag_time << ","
               << truth.id << "," << top1.keyframe_id << "," << std::setprecision(6)
               << top1.ring_dist << "," << top1.score << "," << hit << "," << ok << ","
               << result.fitness << "," << result.inlier_ratio << "," << result.inlier_rmse << ","
               << result.inlier_count << "," << result.correspondence_count << "," << t_err << ","
               << yaw_err << "," << truth_lookup.time_diff << "," << result.scan_context_sector_shift << ","
               << radToDeg(result.scan_context_yaw_diff) << "," << radToDeg(result.candidate_yaw) << ","
               << radToDeg(result.initial_yaw) << "," << radToDeg(result.final_yaw) << ","
               << result.delta_translation << "," << radToDeg(result.delta_yaw) << ","
               << result.gicp_source_points << "," << result.gicp_target_points << ","
               << result.elapsed_ms << "\n";
      } else {
        report << "map_align," << query_count << "," << std::setprecision(9) << bag_time << ","
               << top1.keyframe_id << "," << std::setprecision(6) << top1.ring_dist << ","
               << top1.score << "," << ok << "," << result.converged << "," << result.fitness << ","
               << result.inlier_ratio << "," << result.inlier_rmse << "," << result.inlier_count << ","
               << result.correspondence_count << "," << result.candidate_id << ","
               << result.scan_context_sector_shift << "," << radToDeg(result.scan_context_yaw_diff) << ","
               << radToDeg(result.candidate_yaw) << "," << radToDeg(result.initial_yaw) << ","
               << radToDeg(result.final_yaw) << "," << result.delta_translation << ","
               << radToDeg(result.delta_yaw) << "," << result.gicp_source_points << ","
               << result.gicp_target_points << "," << result.elapsed_ms << "\n";
      }

      ROS_INFO_STREAM("bag_query=" << query_count
                      << " bag_time=" << std::fixed << std::setprecision(3) << bag_time
                      << " top1=" << top1.keyframe_id
                      << " sc_score=" << std::setprecision(4) << top1.score
                      << " success=" << ok
                      << " candidate=" << result.candidate_id
                      << " fitness=" << result.fitness
                      << " inlier_ratio=" << result.inlier_ratio
                      << " inlier_rmse=" << result.inlier_rmse
                      << " gicp_pts=" << result.gicp_source_points << "/" << result.gicp_target_points);

      if (cfg.viz.enable && cfg.run_gicp) {
        const relocalization::Pose viz_center = cfg.use_truth_pose ? truth :
            ((result.success || result.converged) ? poseFromTransform(result.transform) : result.candidate_pose);
        visualizer.publishGlobalMap(core.database(), viz_center, cfg.viz.crop_radius);
        if (cfg.use_truth_pose) {
          visualizer.publishComparison(query_count, source, truth, result, hit, ok, t_err, yaw_err);
        } else {
          visualizer.publishRelocResult(query_count, source, result, ok, result.candidate_pose);
        }
        visualizer.waitForNext();
      }
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("Skip bag_time=" << bag_time << ": " << e.what());
    }
  }

  bag.close();
  report.close();

  ROS_INFO_STREAM("Bag evaluation finished: queries=" << query_count
                  << " success_rate=" << (query_count ? static_cast<double>(reloc_success) / query_count : 0.0)
                  << " report=" << cfg.report_csv);
  if (cfg.use_truth_pose) {
    ROS_INFO_STREAM("  top1_rate=" << (query_count ? static_cast<double>(top1_hit) / query_count : 0.0)
                    << " topk_rate=" << (query_count ? static_cast<double>(topk_hit) / query_count : 0.0));
  }
  if (query_count == 0) {
    ROS_WARN("No valid bag queries processed. Check bag_file and bag_lidar_topic.");
  }
  if (cfg.viz.enable) {
    ROS_INFO("Visualization topics remain latched under /offline_eval/. Press Ctrl+C to exit.");
    ros::spin();
  }
  return 0;
}
