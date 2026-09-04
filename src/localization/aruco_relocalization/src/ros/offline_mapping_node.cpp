/**
 * @file offline_mapping_node.cpp
 * @brief ArUco 离线建图节点（订阅 slam_mapping_v2 位姿，一次回放完成建图）
 */

#include <aruco_relocalization/mapping/offline_mapper.hpp>
#include <aruco_relocalization/ros/param_loader.hpp>
#include <aruco_relocalization/visualizer/marker_visualizer.hpp>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <atomic>
#include <csignal>

namespace {
aruco_reloc::OfflineMapper* g_mapper = nullptr;
std::atomic<bool> g_shutdown_requested{false};

void sigintHandler(int sig) {
  ROS_INFO("[offline_mapping] SIGINT received, saving marker map...");
  g_shutdown_requested.store(true);
}
}  // namespace

class OfflineMappingNode {
 public:
  OfflineMappingNode() : pnh_("~") {
    config_ = aruco_reloc::loadOfflineMappingConfig(pnh_);
    mapper_.initialize(config_);
    g_mapper = &mapper_;

    // 尝试从配置文件加载相机内参
    cv::Mat camera_matrix, dist_coeffs;
    if (aruco_reloc::loadCameraIntrinsics(pnh_, camera_matrix, dist_coeffs)) {
      // 检查是否是有效的内参（非单位矩阵）
      if (camera_matrix.at<double>(0, 0) != 1.0 || camera_matrix.at<double>(1, 1) != 1.0) {
        mapper_.setCameraIntrinsics(camera_matrix, dist_coeffs);
        ROS_INFO("[offline_mapping] using camera intrinsics from config file");
        use_camera_info_topic_ = false;

        // 初始化可视化器
        initVisualizer(camera_matrix, dist_coeffs);
      } else {
        ROS_INFO("[offline_mapping] config file has identity camera matrix, will use camera_info topic");
        use_camera_info_topic_ = true;
      }
    } else {
      use_camera_info_topic_ = true;
    }

    // 订阅
    path_sub_ = nh_.subscribe(config_.path_topic, 10, &OfflineMappingNode::onPath, this);
    image_sub_ = nh_.subscribe(config_.image_topic, 10, &OfflineMappingNode::onImage, this);

    // 只有在需要时才订阅 camera_info topic
    if (use_camera_info_topic_) {
      camera_info_sub_ = nh_.subscribe(config_.camera_info_topic, 1,
                                       &OfflineMappingNode::onCameraInfo, this);
      ROS_INFO("[offline_mapping] subscribing to camera_info topic: %s", config_.camera_info_topic.c_str());
    }

    ROS_INFO("[offline_mapping] node started");
    ROS_INFO("[offline_mapping] path_topic=%s image_topic=%s output=%s",
             config_.path_topic.c_str(), config_.image_topic.c_str(),
             config_.output_path.c_str());
  }

  ~OfflineMappingNode() {
    saveMap();
  }

  void saveMap() {
    if (!map_saved_) {
      mapper_.saveMarkerMap();
      map_saved_ = true;
    }
  }

 private:
  void initVisualizer(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    // 从参数服务器加载可视化配置
    aruco_reloc::VisualizerConfig viz_cfg;
    pnh_.param<bool>("visualizer/enabled", viz_cfg.enabled, true);

    if (!viz_cfg.enabled) {
      ROS_INFO("[offline_mapping] visualizer disabled");
      return;
    }

    pnh_.param<std::string>("visualizer/input_image_topic", viz_cfg.input_image_topic, "/camera/li_image");
    pnh_.param<std::string>("visualizer/output_image_topic", viz_cfg.output_image_topic, "/aruco_reloc/debug_image");
    pnh_.param<bool>("visualizer/draw_corners", viz_cfg.draw_corners, true);
    pnh_.param<bool>("visualizer/draw_ids", viz_cfg.draw_ids, true);
    pnh_.param<bool>("visualizer/draw_axes", viz_cfg.draw_axes, true);
    pnh_.param<bool>("visualizer/draw_pose_text", viz_cfg.draw_pose_text, true);
    pnh_.param<bool>("visualizer/draw_cube", viz_cfg.draw_cube, false);
    pnh_.param<int>("visualizer/corner_radius", viz_cfg.corner_radius, 5);
    pnh_.param<int>("visualizer/line_thickness", viz_cfg.line_thickness, 2);
    pnh_.param<double>("visualizer/axis_length", viz_cfg.axis_length, 0.1);

    // 获取 ArUco 配置
    std::string dictionary = "DICT_4X4_50";
    pnh_.param<std::string>("aruco/dictionary", dictionary, "DICT_4X4_50");
    pnh_.param<double>("aruco/marker_size", viz_cfg.marker_size, 0.2);

    if (visualizer_.initialize(viz_cfg, camera_matrix, dist_coeffs, dictionary)) {
      // 发布可视化图像
      debug_image_pub_ = nh_.advertise<sensor_msgs::Image>(viz_cfg.output_image_topic, 1);
      visualizer_initialized_ = true;
      ROS_INFO("[offline_mapping] visualizer initialized, publishing to %s",
               viz_cfg.output_image_topic.c_str());
    }
  }

  void onPath(const nav_msgs::Path::ConstPtr& msg) {
    mapper_.onPath(msg);
  }

  void onImage(const sensor_msgs::Image::ConstPtr& msg) {
    if (g_shutdown_requested.load()) return;

    // 处理图像
    mapper_.onImage(msg);

    // 可视化处理
    if (visualizer_initialized_) {
      publishVisualization(msg);
    }
  }

  void onCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    mapper_.onCameraInfo(msg);

    // 首次收到 camera_info 时初始化可视化器
    if (use_camera_info_topic_ && !visualizer_initialized_) {
      cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          camera_matrix.at<double>(i, j) = msg->K[i * 3 + j];
        }
      }

      cv::Mat dist_coeffs;
      if (msg->D.size() > 0) {
        dist_coeffs = cv::Mat(msg->D).reshape(1, 1);
      } else {
        dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
      }

      initVisualizer(camera_matrix, dist_coeffs);
    }
  }

  void publishVisualization(const sensor_msgs::Image::ConstPtr& msg) {
    cv::Mat image;
    try {
      image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("[offline_mapping] cv_bridge error: %s", e.what());
      return;
    }

    // 图像旋转处理
    if (config_.image_rotation != 0) {
      cv::RotateFlags flag;
      switch (config_.image_rotation) {
        case 1: flag = cv::ROTATE_90_CLOCKWISE; break;
        case 2: flag = cv::ROTATE_180; break;
        case 3: flag = cv::ROTATE_90_COUNTERCLOCKWISE; break;
        default: flag = cv::ROTATE_90_COUNTERCLOCKWISE;
      }
      cv::rotate(image, image, flag);
    }

    // 使用可视化器独立检测并绘制
    cv::Mat vis_image = visualizer_.processImage(image);

    // 发布可视化图像
    sensor_msgs::ImagePtr debug_msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", vis_image).toImageMsg();
    debug_msg->header.stamp = msg->header.stamp;
    debug_msg->header.frame_id = "camera_link";
    debug_image_pub_.publish(debug_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber path_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher debug_image_pub_;

  aruco_reloc::OfflineMappingConfig config_;
  aruco_reloc::OfflineMapper mapper_;
  aruco_reloc::MarkerVisualizer visualizer_;
  bool map_saved_ = false;
  bool use_camera_info_topic_ = true;
  bool visualizer_initialized_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_offline_mapping", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // 注册 SIGINT 处理
  std::signal(SIGINT, sigintHandler);

  try {
    OfflineMappingNode node;

    ros::Rate rate(100);
    while (ros::ok() && !g_shutdown_requested.load()) {
      ros::spinOnce();
      rate.sleep();
    }

    // 退出前保存
    node.saveMap();

    ROS_INFO("[offline_mapping] node shutdown");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[offline_mapping] exception: " << e.what());
    return 1;
  }

  return 0;
}
