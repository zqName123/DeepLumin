#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

constexpr double kDegToRad = M_PI / 180.0;

std::vector<double> loadVector(ros::NodeHandle& nh, const std::string& key,
                               const std::vector<double>& fallback) {
  std::vector<double> out;
  if (!nh.getParam(key, out) || out.size() != fallback.size()) {
    return fallback;
  }
  return out;
}

Eigen::Matrix4f makeTransform(const std::vector<double>& translation,
                              const std::vector<double>& rpy_deg,
                              double yaw_deg,
                              bool rotate_xy_y_negx) {
  const double roll = rpy_deg[0] * kDegToRad;
  const double pitch = rpy_deg[1] * kDegToRad;
  double yaw = (rpy_deg[2] + yaw_deg) * kDegToRad;
  if (rotate_xy_y_negx) {
    yaw += -M_PI / 2.0;
  }

  const Eigen::AngleAxisf rx(static_cast<float>(roll), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf ry(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rz(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
  transform(0, 3) = static_cast<float>(translation[0]);
  transform(1, 3) = static_cast<float>(translation[1]);
  transform(2, 3) = static_cast<float>(translation[2]);
  return transform;
}

bool getFieldOffset(const sensor_msgs::PointCloud2& msg, const std::string& name, uint32_t* offset) {
  for (const auto& field : msg.fields) {
    if (field.name == name && field.datatype == sensor_msgs::PointField::FLOAT32 && field.count == 1) {
      *offset = field.offset;
      return true;
    }
  }
  return false;
}

float readFloat(const uint8_t* ptr) {
  float value = 0.0F;
  std::memcpy(&value, ptr, sizeof(float));
  return value;
}

void writeFloat(uint8_t* ptr, float value) {
  std::memcpy(ptr, &value, sizeof(float));
}

}  // namespace

class SensorFrameAdapterNode {
 public:
  SensorFrameAdapterNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("topics/input_cloud", input_topic_, "/ouster/points");
    pnh_.param<std::string>("topics/output_cloud", output_topic_, "/localization/points");
    pnh_.param<std::string>("frames/output_frame", output_frame_, "lidar_link");
    pnh_.param<bool>("transform/enabled", transform_enabled_, true);
    pnh_.param<bool>("transform/rotate_xy_y_negx", rotate_xy_y_negx_, true);
    pnh_.param<double>("transform/yaw_deg", yaw_deg_, 0.0);
    pnh_.param<bool>("runtime/remove_nan", remove_nan_, true);
    pnh_.param<bool>("runtime/keep_input_stamp", keep_input_stamp_, true);
    pnh_.param<int>("runtime/queue_size", queue_size_, 5);

    const auto translation = loadVector(pnh_, "transform/translation", {0.0, 0.0, 0.0});
    const auto rpy_deg = loadVector(pnh_, "transform/rotation_rpy_deg", {0.0, 0.0, 0.0});
    transform_ = makeTransform(translation, rpy_deg, yaw_deg_, rotate_xy_y_negx_);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
    sub_ = nh_.subscribe(input_topic_, std::max(1, queue_size_), &SensorFrameAdapterNode::onCloud, this);

    ROS_INFO("[sensor_frame_adapter] input=%s output=%s frame=%s enabled=%s rotate_xy_y_negx=%s yaw_deg=%.3f",
             input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str(),
             transform_enabled_ ? "true" : "false", rotate_xy_y_negx_ ? "true" : "false", yaw_deg_);
    ROS_INFO_STREAM("[sensor_frame_adapter] transform raw->standard:\n" << transform_);
  }

 private:
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    if (!getFieldOffset(*msg, "x", &x_offset) || !getFieldOffset(*msg, "y", &y_offset) ||
        !getFieldOffset(*msg, "z", &z_offset)) {
      ROS_WARN_THROTTLE(2.0, "[sensor_frame_adapter] skip cloud: x/y/z FLOAT32 fields are required");
      return;
    }

    sensor_msgs::PointCloud2 out_msg = *msg;
    out_msg.header.stamp = keep_input_stamp_ ? msg->header.stamp : ros::Time::now();
    if (out_msg.header.stamp.isZero()) {
      out_msg.header.stamp = ros::Time::now();
    }
    out_msg.header.frame_id = output_frame_.empty() ? msg->header.frame_id : output_frame_;

    const std::size_t raw_points = static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    std::size_t valid_points = 0;

    if (remove_nan_) {
      out_msg.data.clear();
      out_msg.data.reserve(msg->data.size());
      out_msg.height = 1;
      out_msg.width = 0;
      out_msg.row_step = 0;
    }

    for (std::size_t i = 0; i < raw_points; ++i) {
      const uint8_t* src = msg->data.data() + i * msg->point_step;
      const float x = readFloat(src + x_offset);
      const float y = readFloat(src + y_offset);
      const float z = readFloat(src + z_offset);
      if (remove_nan_ && (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))) {
        continue;
      }

      uint8_t* dst = nullptr;
      if (remove_nan_) {
        const std::size_t old_size = out_msg.data.size();
        out_msg.data.resize(old_size + msg->point_step);
        dst = out_msg.data.data() + old_size;
        std::memcpy(dst, src, msg->point_step);
      } else {
        dst = out_msg.data.data() + i * msg->point_step;
      }

      if (transform_enabled_) {
        const Eigen::Vector4f p(x, y, z, 1.0F);
        const Eigen::Vector4f q = transform_ * p;
        writeFloat(dst + x_offset, q.x());
        writeFloat(dst + y_offset, q.y());
        writeFloat(dst + z_offset, q.z());
      }
      ++valid_points;
    }

    if (remove_nan_) {
      out_msg.width = static_cast<uint32_t>(valid_points);
      out_msg.row_step = out_msg.point_step * out_msg.width;
      out_msg.is_dense = true;
    }

    pub_.publish(out_msg);

    ROS_INFO_THROTTLE(2.0,
                      "[sensor_frame_adapter] cloud stamp=%.9f frame %s->%s points %zu->%zu",
                      out_msg.header.stamp.toSec(), msg->header.frame_id.c_str(),
                      out_msg.header.frame_id.c_str(), raw_points, valid_points);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  bool transform_enabled_ = true;
  bool rotate_xy_y_negx_ = true;
  bool remove_nan_ = true;
  bool keep_input_stamp_ = true;
  int queue_size_ = 5;
  double yaw_deg_ = 0.0;
  Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_frame_adapter");
  SensorFrameAdapterNode node;
  ros::spin();
  return 0;
}
