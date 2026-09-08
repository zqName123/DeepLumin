#include <aruco_relocalization/visualizer/marker_visualizer.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <iomanip>
#include <sstream>

namespace aruco_reloc {

bool MarkerVisualizer::initialize(const VisualizerConfig& config,
                                   const cv::Mat& camera_matrix,
                                   const cv::Mat& dist_coeffs,
                                   const std::string& dictionary_name) {
  config_ = config;
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();

  // 初始化 ArUco 检测器
  auto dict_id = getDictionaryId(dictionary_name);
  detector_ = cv::aruco::getPredefinedDictionary(dict_id);
  detector_params_ = cv::aruco::DetectorParameters::create();

  initialized_ = true;

  ROS_INFO("[aruco_visualizer] initialized with dict=%s", dictionary_name.c_str());
  return true;
}

cv::aruco::PREDEFINED_DICTIONARY_NAME MarkerVisualizer::getDictionaryId(const std::string& name) {
  static const std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dict_map = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };

  auto it = dict_map.find(name);
  if (it != dict_map.end()) {
    return it->second;
  }
  ROS_WARN("[aruco_visualizer] unknown dictionary '%s', using DICT_4X4_50", name.c_str());
  return cv::aruco::DICT_4X4_50;
}

cv::Mat MarkerVisualizer::processImage(const cv::Mat& image) const {
  if (!initialized_) {
    ROS_WARN_THROTTLE(5.0, "[aruco_visualizer] not initialized");
    return image.clone();
  }

  // 检测 marker
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(image, detector_, marker_corners, marker_ids, detector_params_);

  // 调试日志
  if (!marker_ids.empty()) {
    ROS_INFO_THROTTLE(2.0, "[aruco_visualizer] detected %zu markers, ids: %s",
                      marker_ids.size(),
                      [&]() -> std::string {
                        std::string s;
                        for (auto id : marker_ids) s += std::to_string(id) + " ";
                        return s;
                      }().c_str());
  }

  // 构建检测结果
  VisualDetection detection;
  detection.timestamp = ros::Time::now().toSec();
  detection.has_pose = false;

  if (!marker_ids.empty()) {
    // 估计位姿
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, static_cast<float>(config_.marker_size),
                                         camera_matrix_, dist_coeffs_, rvecs, tvecs);

    ROS_INFO_THROTTLE(2.0, "[aruco_visualizer] marker_size=%.3f, camera_matrix: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                      config_.marker_size,
                      camera_matrix_.at<double>(0, 0),
                      camera_matrix_.at<double>(1, 1),
                      camera_matrix_.at<double>(0, 2),
                      camera_matrix_.at<double>(1, 2));

    for (size_t i = 0; i < marker_ids.size(); ++i) {
      MarkerObservation obs;
      obs.id = marker_ids[i];
      for (int j = 0; j < 4; ++j) {
        obs.corners[j] = marker_corners[i][j];
      }

      // 转换为 Isometry3d
      cv::Mat rmat;
      cv::Rodrigues(rvecs[i], rmat);
      obs.T_cam_marker = Eigen::Isometry3d::Identity();
      obs.T_cam_marker.translation() = Eigen::Vector3d(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          obs.T_cam_marker(r, c) = rmat.at<double>(r, c);
        }
      }

      // 打印位姿信息
      auto t = obs.T_cam_marker.translation();
      ROS_INFO("[aruco_visualizer] marker %d: T=[%.3f, %.3f, %.3f] rvec=[%.3f, %.3f, %.3f]",
               marker_ids[i], t.x(), t.y(), t.z(),
               rvecs[i][0], rvecs[i][1], rvecs[i][2]);

      // 计算重投影误差
      std::vector<cv::Point3d> obj_points = {
        {-config_.marker_size / 2, -config_.marker_size / 2, 0},
        { config_.marker_size / 2, -config_.marker_size / 2, 0},
        { config_.marker_size / 2,  config_.marker_size / 2, 0},
        {-config_.marker_size / 2,  config_.marker_size / 2, 0}
      };
      std::vector<cv::Point2d> projected;
      cv::projectPoints(obj_points, rvecs[i], tvecs[i], camera_matrix_, dist_coeffs_, projected);

      double total_error = 0.0;
      for (int j = 0; j < 4; ++j) {
        double dx = projected[j].x - marker_corners[i][j].x;
        double dy = projected[j].y - marker_corners[i][j].y;
        total_error += std::sqrt(dx * dx + dy * dy);
      }
      obs.reprojection_error = total_error / 4.0;

      ROS_INFO("[aruco_visualizer] marker %d: reproj_error=%.2f", marker_ids[i], obs.reprojection_error);

      detection.markers.push_back(obs);
    }
  }

  // 绘制可视化
  return drawVisualization(image, detection);
}

cv::Mat MarkerVisualizer::drawVisualization(const cv::Mat& image,
                                             const VisualDetection& detection) const {
  cv::Mat vis_image = image.clone();

  // 如果没有检测到 marker，直接返回
  if (detection.markers.empty()) {
    return vis_image;
  }

  // 绘制每个 marker
  for (const auto& obs : detection.markers) {
    // 1. 绘制角点
    if (config_.draw_corners) {
      drawCorners(vis_image, obs.corners);
    }

    // 2. 绘制 marker ID
    if (config_.draw_ids) {
      drawMarkerId(vis_image, obs.corners, obs.id);
    }

    // 3. 绘制坐标系
    if (config_.draw_axes) {
      drawAxes(vis_image, obs.T_cam_marker);
    }

    // 4. 绘制位姿文本
    if (config_.draw_pose_text) {
      cv::Point text_pos(obs.corners[0].x, obs.corners[0].y - 10);
      drawPoseText(vis_image, obs.T_cam_marker, obs.id, text_pos);
    }

    // 5. 绘制立方体
    if (config_.draw_cube) {
      drawCube(vis_image, obs.T_cam_marker, 0.2);  // marker_size = 0.2m
    }
  }

  // 绘制总体信息
  std::ostringstream info;
  info << "Markers: " << detection.markers.size();
  if (detection.has_pose) {
    auto t = detection.T_map_base.translation();
    info << " | Pose: (" << std::fixed << std::setprecision(2)
         << t.x() << ", " << t.y() << ", " << t.z() << ")";
  }

  cv::putText(vis_image, info.str(), cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, config_.text_scale,
              config_.text_color, config_.text_thickness);

  return vis_image;
}

void MarkerVisualizer::drawCorners(cv::Mat& image,
                                    const std::array<cv::Point2f, 4>& corners) const {
  // 绘制角点（圆形标记）和数字
  for (int i = 0; i < 4; ++i) {
    // 绘制角点圆形
    cv::circle(image, corners[i], config_.corner_radius,
               config_.corner_color, -1);  // 填充

    // 绘制角点数字（0, 1, 2, 3）
    cv::Point text_pos(corners[i].x + 8, corners[i].y - 5);
    cv::putText(image, std::to_string(i), text_pos,
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                config_.id_color, 1);
  }

  // 绘制边框（连接角点）
  for (int i = 0; i < 4; ++i) {
    cv::line(image, corners[i], corners[(i + 1) % 4],
             config_.corner_color, config_.line_thickness);
  }
}

void MarkerVisualizer::drawMarkerId(cv::Mat& image,
                                     const std::array<cv::Point2f, 4>& corners,
                                     int id) const {
  // 计算中心点
  cv::Point2f center(0, 0);
  for (const auto& corner : corners) {
    center += corner;
  }
  center *= 0.25f;

  // 绘制 ID
  std::ostringstream oss;
  oss << "ID:" << id;
  cv::putText(image, oss.str(), cv::Point(center.x - 20, center.y),
              cv::FONT_HERSHEY_SIMPLEX, config_.text_scale,
              config_.id_color, config_.text_thickness);
}

void MarkerVisualizer::drawAxes(cv::Mat& image,
                                 const Eigen::Isometry3d& T_cam_marker) const {
  // 坐标轴端点（在 marker 坐标系下）
  double len = config_.axis_length;
  std::vector<Eigen::Vector3d> axes_pts = {
      Eigen::Vector3d(len, 0, 0),   // X 轴
      Eigen::Vector3d(0, len, 0),   // Y 轴
      Eigen::Vector3d(0, 0, len)    // Z 轴
  };

  // 原点
  Eigen::Vector3d origin_3d(0, 0, 0);
  cv::Point2f origin_2d = projectPoint(T_cam_marker * origin_3d);

  // 检查原点是否有效
  if (origin_2d.x < 0 || origin_2d.y < 0) {
    ROS_WARN_THROTTLE(1.0, "[aruco_visualizer] invalid origin for axes drawing");
    return;
  }

  // 绘制三个坐标轴
  cv::Scalar colors[3] = {config_.x_axis_color, config_.y_axis_color, config_.z_axis_color};

  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d pt_3d = T_cam_marker * axes_pts[i];
    cv::Point2f pt_2d = projectPoint(pt_3d);

    // 检查端点是否有效
    if (pt_2d.x < 0 || pt_2d.y < 0) {
      continue;  // 跳过无效的轴
    }

    cv::arrowedLine(image, origin_2d, pt_2d, colors[i],
                    config_.line_thickness + 1, cv::LINE_AA, 0, 0.1);
  }
}

void MarkerVisualizer::drawPoseText(cv::Mat& image,
                                     const Eigen::Isometry3d& T_cam_marker,
                                     int id,
                                     const cv::Point& position) const {
  auto t = T_cam_marker.translation();

  // 检查位姿值是否有效
  if (std::isnan(t.x()) || std::isnan(t.y()) || std::isnan(t.z())) {
    ROS_WARN_THROTTLE(1.0, "[aruco_visualizer] marker %d has nan pose: [%.3f, %.3f, %.3f]",
                      id, t.x(), t.y(), t.z());
    return;  // 不绘制无效位姿
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "T: [" << t.x() << ", " << t.y() << ", " << t.z() << "]";

  // 绘制文本（带背景）
  int baseline = 0;
  cv::Size text_size = cv::getTextSize(oss.str(), cv::FONT_HERSHEY_SIMPLEX,
                                        config_.text_scale * 0.8,
                                        config_.text_thickness, &baseline);

  cv::Point bg_tl(position.x, position.y - text_size.height - 5);
  cv::Point bg_br(position.x + text_size.width, position.y + 5);
  cv::rectangle(image, bg_tl, bg_br, config_.bg_color, -1);
  cv::putText(image, oss.str(), position,
              cv::FONT_HERSHEY_SIMPLEX, config_.text_scale * 0.8,
              config_.text_color, config_.text_thickness);
}

void MarkerVisualizer::drawCube(cv::Mat& image,
                                 const Eigen::Isometry3d& T_cam_marker,
                                 double marker_size) const {
  double half = marker_size / 2.0;
  double height = marker_size;  // 立方体高度

  // 立方体顶点（在 marker 坐标系下）
  std::vector<Eigen::Vector3d> cube_pts = {
      // 底面
      Eigen::Vector3d(-half, -half, 0),
      Eigen::Vector3d(half, -half, 0),
      Eigen::Vector3d(half, half, 0),
      Eigen::Vector3d(-half, half, 0),
      // 顶面
      Eigen::Vector3d(-half, -half, -height),
      Eigen::Vector3d(half, -half, -height),
      Eigen::Vector3d(half, half, -height),
      Eigen::Vector3d(-half, half, -height)
  };

  // 转换到相机坐标系并投影
  std::vector<cv::Point2f> pts_2d;
  for (const auto& pt : cube_pts) {
    Eigen::Vector3d pt_cam = T_cam_marker * pt;
    pts_2d.push_back(projectPoint(pt_cam));
  }

  // 绘制底面
  for (int i = 0; i < 4; ++i) {
    cv::line(image, pts_2d[i], pts_2d[(i + 1) % 4],
             cv::Scalar(0, 255, 255), config_.line_thickness);
  }

  // 绘制顶面
  for (int i = 4; i < 8; ++i) {
    cv::line(image, pts_2d[i], pts_2d[4 + (i + 1) % 4],
             cv::Scalar(0, 255, 255), config_.line_thickness);
  }

  // 绘制侧面
  for (int i = 0; i < 4; ++i) {
    cv::line(image, pts_2d[i], pts_2d[i + 4],
             cv::Scalar(0, 255, 255), config_.line_thickness);
  }
}

cv::Point2f MarkerVisualizer::projectPoint(const Eigen::Vector3d& point_3d) const {
  // 使用相机内参投影
  double fx = camera_matrix_.at<double>(0, 0);
  double fy = camera_matrix_.at<double>(1, 1);
  double cx = camera_matrix_.at<double>(0, 2);
  double cy = camera_matrix_.at<double>(1, 2);

  // 检查 z 值是否有效
  if (point_3d.z() <= 0.001) {
    ROS_WARN_THROTTLE(1.0, "[aruco_visualizer] invalid z value: %.3f, point: [%.3f, %.3f, %.3f]",
                      point_3d.z(), point_3d.x(), point_3d.y(), point_3d.z());
    return cv::Point2f(-1, -1);  // 返回无效坐标
  }

  double x = point_3d.x() / point_3d.z();
  double y = point_3d.y() / point_3d.z();

  cv::Point2f pt;
  pt.x = fx * x + cx;
  pt.y = fy * y + cy;

  // 检查投影坐标是否有效
  if (std::isnan(pt.x) || std::isnan(pt.y)) {
    ROS_WARN_THROTTLE(1.0, "[aruco_visualizer] nan projection: point=[%.3f, %.3f, %.3f] -> [%.3f, %.3f]",
                      point_3d.x(), point_3d.y(), point_3d.z(), pt.x, pt.y);
    return cv::Point2f(-1, -1);
  }

  return pt;
}

}  // namespace aruco_reloc
