/**
 * @file slam_mapping_v2_node.cc
 * @brief slam_mapping_v2 节点（v2 自有结构：单命名空间，无外部包依赖）
 *
 * 线程：输入回调（AsyncSpinner×2）→ 主线程（消费帧/关键帧/全局地图/状态/TF）
 * → 后端线程（GNSS+回环+ISAM2）→ 可视化线程（path/map/调试 marker）。
 */

#include "slam_mapping_v2/core/mapping_runtime.hpp"
#include "slam_mapping_v2/ros/config_loader.hpp"
#include "slam_mapping_v2/ros/converters.hpp"
#include "slam_mapping_v2/ros/publishers.hpp"

#include <deeplumin_msgs/GetMappingRuntimeStatus.h>
#include <deeplumin_msgs/Gpchc.h>
#include <deeplumin_msgs/SaveMappingRuntime.h>
#include <deeplumin_msgs/StartMappingRuntime.h>
#include <deeplumin_msgs/StopMappingRuntime.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

#include "deeplumin_health_checker/health_checker/health_checker.h"
#include "syslog/logger.h"


#include <pcl/common/transforms.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace slam_mapping_v2 {

class SlamMappingV2Node {
 public:
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    cfg_ = slam_mapping_v2_ros::loadConfig(pnh);
    if (!runtime_.initialize(cfg_)) {
      // ROS_ERROR("[slam_mapping_v2] runtime initialize failed");
      SYS_LOG(syslog::LogSeverity::ERROR, true) << "runtime initialize failed";
      return false;
    }
    pubs_.initialize(nh, cfg_);
    health_checker_ = std::make_shared<deeplumin_health_checker::HealthChecker>(nh, pnh);
    health_checker_->ENABLE();
    health_checker_->NODE_ACTIVATE();

    std::cout << "SlamMappingV2Node: HealthChecker enabled and activated." << std::endl;

    odom_sub_ = nh.subscribe(cfg_.input.odom_topic, 200, &SlamMappingV2Node::onOdom, this);
    scan_sub_ = nh.subscribe(cfg_.input.scan_topic, 200, &SlamMappingV2Node::onScan, this);
    gnss_sub_ = nh.subscribe(cfg_.input.gnss_topic, 1000, &SlamMappingV2Node::onGnss, this);

    // 订阅图像话题（用于视觉回环和关键帧图像保存）
    if (!cfg_.input.image_topic.empty()) {
      // 判断是否为压缩图像话题
      if (cfg_.input.image_topic.find("compressed") != std::string::npos) {
        image_compressed_sub_ = nh.subscribe(cfg_.input.image_topic, 10,
                                             &SlamMappingV2Node::onCompressedImage, this);
        // ROS_INFO("[slam_mapping_v2] Subscribing to compressed image topic: %s",
        //          cfg_.input.image_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true) << "Subscribing to compressed image topic: " << cfg_.input.image_topic;
      } else {
        image_sub_ = nh.subscribe(cfg_.input.image_topic, 10,
                                  &SlamMappingV2Node::onImage, this);
        // ROS_INFO("[slam_mapping_v2] Subscribing to image topic: %s",
        //          cfg_.input.image_topic.c_str());
        SYS_LOG(syslog::LogSeverity::INFO, true) << "Subscribing to image topic: " << cfg_.input.image_topic;
      }
    }

    start_srv_ = nh.advertiseService(cfg_.services.start, &SlamMappingV2Node::handleStart, this);
    stop_srv_ = nh.advertiseService(cfg_.services.stop, &SlamMappingV2Node::handleStop, this);
    save_srv_ = nh.advertiseService(cfg_.services.save, &SlamMappingV2Node::handleSave, this);
    reset_srv_ = nh.advertiseService(cfg_.services.reset, &SlamMappingV2Node::handleReset, this);
    get_status_srv_ =
        nh.advertiseService(cfg_.services.get_status, &SlamMappingV2Node::handleGetStatus, this);

    // 调试图像发布（仅视觉或混合模式时启用）
    if (cfg_.loop_closure.detector_type == "visual" ||
        cfg_.loop_closure.detector_type == "hybrid") {
      debug_image_pub_ = nh.advertise<sensor_msgs::Image>(
          "/slam_mapping_v2/debug_image", 1);
      // ROS_INFO("[slam_mapping_v2] Debug image topic: /slam_mapping_v2/debug_image");
      SYS_LOG(syslog::LogSeverity::INFO, true) << "Debug image topic: /slam_mapping_v2/debug_image";
    }

    if (cfg_.runtime.auto_start) {
      StartRequest req;
      req.task_id = cfg_.runtime.task_name;
      req.task_name = cfg_.runtime.task_name;
      req.output_dir = cfg_.runtime.output_root + "/" + cfg_.runtime.task_name;
      req.enable_loop_closure = cfg_.loop_closure.enable;
      runtime_.start(req, ros::Time::now().toSec());
      // ROS_INFO("[slam_mapping_v2] auto_start task=%s output=%s", req.task_name.c_str(),
      //          req.output_dir.c_str());
      SYS_LOG(syslog::LogSeverity::INFO, true) << "auto_start task=" << req.task_name << " output=" << req.output_dir;
    }
    return true;
  }

  void startThreads() {
    runtime_.startThreads();
    stop_viz_ = false;
    viz_thread_ = std::thread(&SlamMappingV2Node::visualizationLoop, this);
  }

  void spinOnce() {
    if (runtime_.processOnce()) {
      const auto f = runtime_.latestFrame();
      if (cfg_.output.publish_odom) pubs_.publishOdom(f);
      if (cfg_.output.publish_current_scan) {
        FrameInput scan = f;
        Eigen::Matrix4d t;
        if (runtime_.mapOdomTransform(&t) && scan.cloud) {
          PointCloud tmp;
          pcl::transformPointCloud(*scan.cloud, tmp, t.cast<float>());
          scan.cloud = PointCloudPtr(new PointCloud(tmp));
        }
        pubs_.publishCurrentScan(scan);
      }
      if (cfg_.runtime.publish_tf) publishMapOdomTf(f.timestamp);
    }
    if ((ros::WallTime::now() - last_status_pub_time_).toSec() >= 0.1) {
      last_status_pub_time_ = ros::WallTime::now();
      pubs_.publishStatus(runtime_.status(), ros::Time::now());
    }
  }

  double processRateHz() const { return cfg_.runtime.process_rate_hz; }

  void shutdown() {
    stop_viz_ = true;
    if (viz_thread_.joinable()) viz_thread_.join();
    runtime_.shutdown();
  }

 private:
  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) { runtime_.feedOdom(slam_mapping_v2_ros::fromOdomMsg(*msg)); }

  void onScan(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    runtime_.feedScan(slam_mapping_v2_ros::fromScanMsg(*msg));
  }

  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg) {
    if (!cfg_.loop_closure.gnss.enable) return;
    runtime_.feedGnss(slam_mapping_v2_ros::fromGnssMsg(*msg));
  }

  void onImage(const sensor_msgs::Image::ConstPtr& msg) {
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (const cv_bridge::Exception& e) {
      // ROS_ERROR("[slam_mapping_v2] cv_bridge exception: %s", e.what());
      SYS_LOG(syslog::LogSeverity::ERROR, true) << "cv_bridge exception: " << e.what();
      return;
    }

    // ROS_INFO_THROTTLE(5.0, "[slam_mapping_v2] Image received: %dx%d ch=%d stamp=%.3f",
    //                   image.cols, image.rows, image.channels(), msg->header.stamp.toSec());
    SYS_LOG(syslog::LogSeverity::INFO, false) << "Image received: " << image.cols << "x" << image.rows << "x" << image.channels() << " " << msg->header.stamp.toSec();

    publishDebugImage(image, msg->header.stamp);
    cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
    runtime_.feedImage(image, msg->header.stamp.toSec());
  }

  void onCompressedImage(const sensor_msgs::CompressedImage::ConstPtr& msg) {
    cv::Mat image;
    try {
      image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    } catch (const std::exception& e) {
      // ROS_ERROR("[slam_mapping_v2] Compressed image decode exception: %s", e.what());
      SYS_LOG(syslog::LogSeverity::ERROR, true) << "Compressed image decode exception: " << e.what();
      return;
    }
    if (image.empty()) {
      // ROS_ERROR("[slam_mapping_v2] Failed to decode compressed image");
      SYS_LOG(syslog::LogSeverity::ERROR, true) << "Failed to decode compressed image";
      return;
    }

    // ROS_INFO_THROTTLE(5.0, "[slam_mapping_v2] Compressed image received: %dx%d ch=%d stamp=%.3f",
    //                   image.cols, image.rows, image.channels(), msg->header.stamp.toSec());
    // SYS_LOG(syslog::LogSeverity::INFO, false) << "Compressed image received: " << image.cols << "x" << image.rows << "x" << image.channels() << " " << msg->header.stamp.toSec();

    publishDebugImage(image, ros::Time(msg->header.stamp));
    cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
    runtime_.feedImage(image, msg->header.stamp.toSec());
  }

  void visualizationLoop() {
    const auto interval = std::chrono::milliseconds(
        static_cast<int>(cfg_.threading.viz_interval_sec * 1000.0));
    while (!stop_viz_.load()) {
      std::this_thread::sleep_for(interval);
      const auto st = runtime_.status();
      if (st.state != RuntimeState::RUNNING) {
        // ROS_WARN_THROTTLE(5.0, "[slam_mapping_v2] viz idle: state=%s (call %s or set runtime/auto_start)",
        //                   toString(st.state).c_str(), cfg_.services.start.c_str());
        SYS_LOG(syslog::LogSeverity::WARNING, true) << "viz idle: state=" << toString(st.state) << " " << cfg_.services.start;
        continue;
      }
      const auto kfs = runtime_.keyframes();
      pubs_.publishPath(kfs);
      if (cfg_.map.publish_global_map) {
        pubs_.publishMap(runtime_.mapCloud(), cfg_.frames.map, st.last_frame_time);
      }
      if (cfg_.output.publish_debug_markers) {
        const auto ld = runtime_.loopData();
        pubs_.publishLoopMarkers(ld.loop_pairs, kfs);
        pubs_.publishKeyframeMarkers(kfs);
      }
    }
  }

  /**
   * @brief 发布局部调试图像：逆时针旋转90°恢复正立视角 + 绘制状态信息
   *
   * 相机顺时针绕x轴旋转90°安装，图像原始方向是侧躺的。
   * 此函数将图像逆时针旋转90°恢复正立，并在左上角叠加状态文字，
   * 发布到 /slam_mapping_v2/debug_image，方便在 rviz 中查看。
   */
  void publishDebugImage(const cv::Mat& gray, const ros::Time& stamp) {
    if (debug_image_pub_.getNumSubscribers() == 0) return;

    // 逆时针旋转90°恢复正立视角
    cv::Mat upright;
    cv::rotate(gray, upright, cv::ROTATE_90_COUNTERCLOCKWISE);

    // 转 BGR 以便叠加彩色文字
    cv::Mat vis;
    cv::cvtColor(upright, vis, cv::COLOR_GRAY2BGR);

    // 绘制状态信息
    const auto st = runtime_.status();
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Size: %dx%d -> %dx%d (CCW90)",
                  gray.cols, gray.rows, upright.cols, upright.rows);
    cv::putText(vis, buf, {10, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 255, 0}, 3);

    std::snprintf(buf, sizeof(buf), "KFs: %zu  Loops: %lu",
                  runtime_.keyframes().size(), st.loop_count);
    cv::putText(vis, buf, {10, 90}, cv::FONT_HERSHEY_SIMPLEX, 1.2, {0, 255, 0}, 3);

    std::snprintf(buf, sizeof(buf), "K: fx=%.0f fy=%.0f cx=%.0f cy=%.0f",
                  cfg_.loop_closure.visual.camera.fx,
                  cfg_.loop_closure.visual.camera.fy,
                  cfg_.loop_closure.visual.camera.cx,
                  cfg_.loop_closure.visual.camera.cy);
    cv::putText(vis, buf, {10, 140}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {255, 200, 0}, 2);

    // 发布
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", vis).toImageMsg();
    msg->header.stamp = stamp;
    msg->header.frame_id = "camera_link";
    debug_image_pub_.publish(msg);
  }

  void publishMapOdomTf(double stamp) {
    Eigen::Matrix4d t;
    if (!runtime_.mapOdomTransform(&t)) return;
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time(stamp);
    tf.header.frame_id = cfg_.frames.map;
    tf.child_frame_id = cfg_.frames.odom;
    tf.transform.translation.x = t(0, 3);
    tf.transform.translation.y = t(1, 3);
    tf.transform.translation.z = t(2, 3);
    const Eigen::Quaterniond q(t.block<3, 3>(0, 0));
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf_broadcaster_.sendTransform(tf);
  }

  bool handleStart(deeplumin_msgs::StartMappingRuntime::Request& req,
                   deeplumin_msgs::StartMappingRuntime::Response& res) {
    const bool ok = runtime_.start(slam_mapping_v2_ros::fromStartReq(req), ros::Time::now().toSec());
    const auto st = runtime_.status();
    res.accepted = ok;
    res.state = toString(st.state);
    res.reason = st.message.empty() ? (ok ? "accepted" : "start failed") : st.message;
    return true;
  }

  bool handleStop(deeplumin_msgs::StopMappingRuntime::Request& req,
                  deeplumin_msgs::StopMappingRuntime::Response& res) {
    const bool ok = runtime_.stop(slam_mapping_v2_ros::fromStopReq(req));
    const auto st = runtime_.status();
    res.accepted = ok;
    res.state = toString(st.state);
    res.reason = st.message.empty() ? (ok ? "accepted" : "stop failed") : st.message;
    res.map_path = st.map_path;
    res.output_dir = st.output_dir;
    return true;
  }

  bool handleSave(deeplumin_msgs::SaveMappingRuntime::Request& req,
                  deeplumin_msgs::SaveMappingRuntime::Response& res) {
    const bool ok = runtime_.save(slam_mapping_v2_ros::fromSaveReq(req));
    SaveResult result;
    const auto st = runtime_.status();
    result.success = ok;
    result.message = st.message;
    result.map_path = st.map_path;
    result.keyframe_dir = st.keyframe_dir;
    result.keyframe_pose_path = st.keyframe_pose_path;
    result.trajectory_path = st.trajectory_path;
    slam_mapping_v2_ros::fillSaveRes(result, &res);
    return true;
  }

  bool handleReset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    runtime_.reset();
    res.success = true;
    res.message = "reset";
    return true;
  }

  bool handleGetStatus(deeplumin_msgs::GetMappingRuntimeStatus::Request&,
                       deeplumin_msgs::GetMappingRuntimeStatus::Response& res) {
    slam_mapping_v2_ros::fillStatusRes(runtime_.status(), &res, ros::Time::now());
    return true;
  }

  ros::NodeHandle nh_;
  MappingConfig cfg_;
  MappingRuntime runtime_;
  slam_mapping_v2_ros::Publishers pubs_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber gnss_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber image_compressed_sub_;
  ros::Publisher debug_image_pub_;
  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
  ros::ServiceServer save_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer get_status_srv_;

  std::atomic<bool> stop_viz_{false};
  std::thread viz_thread_;
  ros::WallTime last_status_pub_time_;

  std::shared_ptr<deeplumin_health_checker::HealthChecker> health_checker_; // 健康检查器
};

}  // namespace slam_mapping_v2

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_mapping_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // 初始化日志
  std::string log_dir;
  pnh.param<std::string>("log_dir", log_dir, "");
  SYS_LOG_INIT_FULL(log_dir,
                    syslog::LogSeverity::INFO,
                    50,
                    "slam_mapping_v2");


  slam_mapping_v2::SlamMappingV2Node node;
  if (!node.initialize(nh, pnh)) {
    // ROS_ERROR("[slam_mapping_v2] initialize failed");
    SYS_LOG(syslog::LogSeverity::ERROR, true) << "initialize failed";
    return 1;
  }
  node.startThreads();
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(node.processRateHz());
  while (ros::ok()) {
    node.spinOnce();
    rate.sleep();
  }
  spinner.stop();
  node.shutdown();
  return 0;
}
