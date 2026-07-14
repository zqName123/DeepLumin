#include <relocalization/core/relocalization_core.hpp>
#include <relocalization/ros/param_loader.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace {

geometry_msgs::PoseStamped toPoseMsg(const Eigen::Matrix4d& tform, const std::string& frame_id) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.pose.position.x = tform(0, 3);
  msg.pose.position.y = tform(1, 3);
  msg.pose.position.z = tform(2, 3);
  Eigen::Quaterniond q(tform.block<3, 3>(0, 0));
  q.normalize();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

}  // namespace

class GlobalRelocalizationNode {
 public:
  GlobalRelocalizationNode() : pnh_("~") {
    config_ = relocalization_ros::loadOnlineRelocalizationConfig(pnh_);
    if (!core_.initialize(config_.core)) {
      throw std::runtime_error("failed to initialize relocalization core");
    }

    lidar_sub_ = nh_.subscribe(config_.lidar_topic, 1, &GlobalRelocalizationNode::onCloud, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reloc_pose", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("reloc_status", 1, true);
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reloc_aligned_scan", 1, true);
    target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("reloc_target_submap", 1, true);

    ROS_INFO_STREAM("Loaded " << core_.descriptor().size() << " descriptors");
    ROS_INFO_STREAM("query_frame.rotate_xy_y_negx=" << (config_.query_rotate_xy_y_negx ? "true" : "false")
                    << " yaw_deg=" << config_.query_frame_yaw_deg);
  }

 private:
  void onCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
    relocalization::CloudPtr raw = relocalization::cloudFromRosMsg(*msg);
    double query_frame_yaw_rad = config_.query_frame_yaw_deg * M_PI / 180.0;
    if (config_.query_rotate_xy_y_negx) {
      query_frame_yaw_rad += -M_PI / 2.0;
    }
    relocalization::CloudPtr query_local = std::abs(query_frame_yaw_rad) > 1e-9
        ? relocalization::rotateCloudYaw(raw, query_frame_yaw_rad)
        : raw;
    relocalization::CloudPtr source = relocalization::preprocessCloud(query_local,
                                                                      config_.core.database.preprocess);
    const auto query = core_.relocalize(source);
    const auto& result = query.match;

    std_msgs::String status;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4)
       << "success=" << result.success
       << " candidate=" << result.candidate_id
       << " sc_score=" << result.scan_context_score
       << " fitness=" << result.fitness
       << " inlier_ratio=" << result.inlier_ratio
       << " inlier_rmse=" << result.inlier_rmse
       << " inliers=" << result.inlier_count << "/" << result.correspondence_count
       << " gicp_pts=" << result.gicp_source_points << "/" << result.gicp_target_points
       << " elapsed_ms=" << result.elapsed_ms;
    status.data = ss.str();
    status_pub_.publish(status);

    if (!result.success) {
      ROS_WARN_STREAM(status.data);
      return;
    }

    pose_pub_.publish(toPoseMsg(result.transform, config_.frame_id));
    if (result.aligned && aligned_pub_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*result.aligned, out);
      out.header.stamp = ros::Time::now();
      out.header.frame_id = config_.frame_id;
      aligned_pub_.publish(out);
    }
    if (result.target && target_pub_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*result.target, out);
      out.header.stamp = ros::Time::now();
      out.header.frame_id = config_.frame_id;
      target_pub_.publish(out);
    }
    ROS_INFO_STREAM(status.data);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber lidar_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher target_pub_;
  relocalization_ros::OnlineRelocalizationConfig config_;
  relocalization::RelocalizationCore core_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_relocalization_node");
  try {
    GlobalRelocalizationNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    return 1;
  }
  return 0;
}
