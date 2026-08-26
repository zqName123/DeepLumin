/**
 * @file global_matcher_node.cpp
 * @brief 全局匹配定位节点 - 基于点云配准的全局定位
 *
 * 该节点接收实时点云数据，通过与预先加载的全局地图进行配准（ICP/GICP/FastGICP），
 * 计算车辆在地图坐标系中的精确位姿。支持多种初始位姿来源（跟踪模式、引导模式、
 * 手动设置、默认值），并采用粗细两级配准策略提高匹配精度和鲁棒性。
 *
 * 主要功能：
 * 1. 加载和发布全局点云地图
 * 2. 接收实时点云和里程计数据
 * 3. 选择初始位姿猜测（跟踪/引导/默认）
 * 4. 从全局地图中裁剪局部子地图
 * 5. 执行粗细两级点云配准
 * 6. 计算内点指标并进行质量门控
 * 7. 发布匹配结果和可视化数据
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fast_gicp/gicp/fast_gicp.hpp>

#include <small_gicp/points/point_cloud.hpp>               // small_gicp::PointCloud
#include <small_gicp/util/normal_estimation_omp.hpp>       // estimate_covariances_omp
#include <small_gicp/ann/gaussian_voxelmap.hpp>            // GaussianVoxelMap（VGICP 的 target）
#include <small_gicp/factors/gicp_factor.hpp>              // GICPFactor
#include <small_gicp/registration/reduction_omp.hpp>       // ParallelReductionOMP
#include <small_gicp/registration/registration.hpp>        // Registration 模板

#include <small_gicp/ros/ros1.hpp>   // small_gicp::from_ros_msg / to_ros_msg
#include <small_gicp/registration/registration_helper.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <XmlRpcValue.h>

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <deeplumin_msgs/LocalizationStatus.h>

namespace {

// 点云类型定义：使用pcl::PointXYZ作为基本点类型
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = CloudT::Ptr;
using CloudConstPtr = CloudT::ConstPtr;

small_gicp::GaussianVoxelMap::Ptr vgicp_coarse_map = nullptr; // 测试
small_gicp::GaussianVoxelMap::Ptr vgicp_fine_map = nullptr;
// 数学常量：圆周率
constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 匹配质量指标结构体
 * @details 存储点云配准后的各项质量评估指标，用于判断匹配结果是否可靠
 */
struct MatchMetrics {
  uint32_t total_points = 0;      ///< 参与评估的总点数
  uint32_t inlier_count = 0;      ///< 内点数量（距离小于阈值的匹配点）
  double inlier_ratio = 0.0;      ///< 内点比例 = inlier_count / total_points
  double inlier_rmse = std::numeric_limits<double>::infinity();  ///< 内点均方根误差
  double fitness = std::numeric_limits<double>::infinity();  ///< 平均最近邻平方距离（PCL 语义 fitness，日志/消息用）
};

/**
 * @brief 地图分段结构体
 * @details 支持多地图分段管理，每个分段包含完整地图和可视化用降采样地图
 */
struct MapSegment {
  std::string id;      ///< 地图分段唯一标识
  std::string path;    ///< PCD文件路径
  CloudPtr full_map;   ///< 完整分辨率地图（用于配准）
  CloudPtr viz_map;    ///< 降采样地图（用于可视化发布）
  small_gicp::GaussianVoxelMap::Ptr vgicp_coarse;  ///< VGICP 粗体素地图（整张地图预构建）
  small_gicp::GaussianVoxelMap::Ptr vgicp_fine;    ///< VGICP 细体素地图（整张地图预构建）
};

/**
 * @brief 匹配器运行模式枚举
 * @details BOOTSTRAP: 引导模式，使用外部提供的初始位姿
 *          TRACKING: 跟踪模式，使用融合里程计作为初始猜测
 */
enum class MatcherRunMode {
  BOOTSTRAP,   ///< 引导模式：依赖外部初始位姿输入
  TRACKING     ///< 跟踪模式：使用融合里程计持续跟踪
};

/**
 * @brief 初始位姿猜测结构体
 * @details 包含从map到source的变换、source到base的变换、以及运行模式等信息
 */
struct InitialGuess {
  Eigen::Matrix4f map_source = Eigen::Matrix4f::Identity();  ///< map -> source（点云坐标系）变换
  Eigen::Matrix4f source_base = Eigen::Matrix4f::Identity(); ///< source -> base_link变换
  Eigen::Matrix4f map_base = Eigen::Matrix4f::Identity();    ///< map -> base_link变换
  MatcherRunMode mode = MatcherRunMode::BOOTSTRAP;           ///< 当前运行模式
  std::string reason;    ///< 选择该初始猜测的原因说明
  bool valid = true;     ///< 该猜测是否有效
};

/**
 * @brief 将运行模式枚举转换为字符串
 * @param mode 运行模式枚举值
 * @return 对应的字符串表示（"bootstrap"或"tracking"）
 */
std::string modeToString(MatcherRunMode mode) {
  switch (mode) {
    case MatcherRunMode::BOOTSTRAP: return "bootstrap";
    case MatcherRunMode::TRACKING: return "tracking";
  }
  return "unknown";
}

/**
 * @brief 将ROS Pose消息转换为4x4变换矩阵
 * @param pose geometry_msgs::Pose类型输入
 * @return Eigen::Matrix4f 4x4齐次变换矩阵（map->base或odom->base等）
 * @details 提取位置（translation）和四元数（rotation），构建齐次变换矩阵
 */
Eigen::Matrix4f poseToMatrix(const geometry_msgs::Pose& pose) {
  Eigen::Quaternionf q(static_cast<float>(pose.orientation.w),
                       static_cast<float>(pose.orientation.x),
                       static_cast<float>(pose.orientation.y),
                       static_cast<float>(pose.orientation.z));
  if (q.norm() < 1e-6f) {
    q = Eigen::Quaternionf::Identity();
  } else {
    q.normalize();
  }

  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = q.toRotationMatrix();
  mat(0, 3) = static_cast<float>(pose.position.x);
  mat(1, 3) = static_cast<float>(pose.position.y);
  mat(2, 3) = static_cast<float>(pose.position.z);
  return mat;
}

/**
 * @brief 将4x4变换矩阵转换为ROS Pose消息
 * @param mat Eigen::Matrix4f 4x4齐次变换矩阵
 * @return geometry_msgs::Pose ROS位姿消息
 * @details 从矩阵中提取平移部分作为position，旋转部分转换为四元数
 */
geometry_msgs::Pose matrixToPose(const Eigen::Matrix4f& mat) {
  geometry_msgs::Pose pose;
  Eigen::Matrix3f rot = mat.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rot);
  q.normalize();
  pose.position.x = mat(0, 3);
  pose.position.y = mat(1, 3);
  pose.position.z = mat(2, 3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

/**
 * @brief 从变换矩阵中提取偏航角（yaw）
 * @param mat 4x4齐次变换矩阵
 * @return double 偏航角（弧度）
 * @details 使用atan2从旋转矩阵的第一列提取yaw角，假设使用ZYX欧拉角顺序
 */
double yawFromMatrix(const Eigen::Matrix4f& mat) {
  return std::atan2(mat(1, 0), mat(0, 0));
}

/**
 * @brief 角度归一化到[-π, π]范围
 * @param angle 输入角度（弧度）
 * @return double 归一化后的角度
 */
double normalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

/**
 * @brief 计算两个变换矩阵之间的偏航角差
 * @param a 第一个变换矩阵
 * @param b 第二个变换矩阵
 * @return double 归一化后的yaw角差（弧度）
 */
double yawDiff(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return normalizeAngle(yawFromMatrix(a) - yawFromMatrix(b));
}

/**
 * @brief 计算两个变换矩阵之间的平移距离
 * @param a 第一个变换矩阵
 * @param b 第二个变换矩阵
 * @return double 平移向量差的欧氏距离
 */
double translationDiff(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Eigen::Vector3f da = a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3);
  return da.norm();
}

/**
 * @brief 对点云进行体素网格降采样
 * @param input 输入点云（常量指针）
 * @param leaf 体素叶子大小（米）
 * @return CloudPtr 降采样后的点云指针
 * @details 使用PCL的VoxelGrid滤波器，如果leaf<=1e-4则直接返回原点云副本
 */
CloudPtr downsample(const CloudConstPtr& input, double leaf) {
  CloudPtr output(new CloudT());
  if (!input || input->empty()) {
    return output;
  }
  if (leaf <= 1e-4) {
    *output = *input;
    return output;
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.setInputCloud(input);
  voxel.filter(*output);
  return output;
}

/**
 * @brief 从ROS参数服务器加载3D向量
 * @param nh ROS节点句柄
 * @param key 参数键名
 * @param value 输出Eigen::Vector3f指针
 * @return bool 加载成功返回true，失败返回false
 * @details 期望参数为包含3个double值的数组 [x, y, z]
 */
bool loadVector3(ros::NodeHandle& nh, const std::string& key, Eigen::Vector3f* value) {
  std::vector<double> flat;
  if (!nh.getParam(key, flat) || flat.size() != 3) {
    return false;
  }
  *value = Eigen::Vector3f(static_cast<float>(flat[0]),
                          static_cast<float>(flat[1]),
                          static_cast<float>(flat[2]));
  return true;
}

/**
 * @brief 从ROS参数服务器加载3x3旋转矩阵
 * @param nh ROS节点句柄
 * @param key 参数键名
 * @param value 输出Eigen::Matrix3f指针
 * @return bool 加载成功返回true，失败返回false
 * @details 期望参数为包含9个double值的数组，按行优先顺序排列
 */
bool loadMatrix3(ros::NodeHandle& nh, const std::string& key, Eigen::Matrix3f* value) {
  std::vector<double> flat;
  if (!nh.getParam(key, flat) || flat.size() != 9) {
    return false;
  }
  *value << static_cast<float>(flat[0]), static_cast<float>(flat[1]), static_cast<float>(flat[2]),
            static_cast<float>(flat[3]), static_cast<float>(flat[4]), static_cast<float>(flat[5]),
            static_cast<float>(flat[6]), static_cast<float>(flat[7]), static_cast<float>(flat[8]);
  return true;
}

/**
 * @brief 将角度制RPY转换为旋转矩阵
 * @param rpy_deg 角度制的roll, pitch, yaw向量（度）
 * @return Eigen::Matrix3f 3x3旋转矩阵
 * @details 使用ZYX欧拉角顺序：先roll绕X，再pitch绕Y，最后yaw绕Z
 */
Eigen::Matrix3f rpyDegToMatrix(const Eigen::Vector3f& rpy_deg) {
  const float deg_to_rad = static_cast<float>(kPi / 180.0);
  const Eigen::AngleAxisf roll(rpy_deg.x() * deg_to_rad, Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf pitch(rpy_deg.y() * deg_to_rad, Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf yaw(rpy_deg.z() * deg_to_rad, Eigen::Vector3f::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

/**
 * @brief 从ROS参数加载外参变换矩阵
 * @param nh ROS节点句柄
 * @param key 参数键名前缀
 * @return Eigen::Matrix4f 4x4齐次变换矩阵
 * @details 加载translation（3维向量）和rotation（3x3矩阵），组合成4x4变换矩阵
 */
Eigen::Matrix4f loadExtrinsic(ros::NodeHandle& nh, const std::string& key) {
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  loadVector3(nh, key + "/translation", &translation);
  loadMatrix3(nh, key + "/rotation", &rotation);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = translation;
  return transform;
}

/**
 * @brief 移除点云中的无效点（NaN）
 * @param input 输入点云
 * @return CloudPtr 过滤后的点云指针
 * @details 使用PCL的removeNaNFromPointCloud函数清理包含NaN坐标值的点
 */
CloudPtr removeInvalid(const CloudConstPtr& input) {
  CloudPtr output(new CloudT());
  if (!input) {
    return output;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *output, indices);
  return output;
}

}  // namespace

/**
 * @class GlobalMatcherNode
 * @brief 全局匹配定位节点主类
 *
 * 该类实现了完整的全局匹配定位流程：
 * 1. 参数加载与ROS话题初始化
 * 2. 全局地图加载与管理（支持多地图分段）
 * 3. 接收实时点云、里程计、初始位姿等输入
 * 4. 定时执行点云配准（粗细两级策略）
 * 5. 质量评估与门控判断
 * 6. 发布匹配结果、可视化数据和状态信息
 */
class GlobalMatcherNode {
 public:
  /**
   * @brief 构造函数
   * @param nh 公共节点句柄（用于创建全局话题）
   * @param pnh 私有节点句柄（用于加载参数）
   *
   * 初始化流程：
   * 1. 从参数服务器加载所有配置参数
   * 2. 创建发布器和订阅器
   * 3. 加载默认地图并发布
   * 4. 创建匹配定时器和地图发布定时器
   */
  GlobalMatcherNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    loadParams();

    // 创建结果发布器：发布全局匹配结果（位姿、质量指标等）
    result_pub_ = nh_.advertise<deeplumin_msgs::GlobalMatchResult>(result_topic_, 10);
    // 创建状态发布器：发布定位状态（正常/警告/失败）
    status_pub_ = nh_.advertise<deeplumin_msgs::LocalizationStatus>(status_topic_, 10);
    // 创建初始点云发布器（用于可视化初始猜测变换后的点云）
    source_initial_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(source_initial_scan_topic_, 2);
    // 创建对齐点云发布器（用于可视化配准后的点云）
    aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(aligned_scan_topic_, 2);
    // 创建子地图发布器（用于可视化当前使用的局部地图）
    submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(target_submap_topic_, 2);
    // 创建初始猜测位姿发布器
    initial_guess_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(initial_guess_topic_, 2);
    // 创建轨迹发布器
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 2);
    // 创建全局地图发布器（latch=true，新订阅者能收到最后一条消息）
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_, 1, true);

    // 订阅实时点云（source cloud）
    source_sub_ = nh_.subscribe(source_cloud_topic_, 2, &GlobalMatcherNode::onSourceCloud, this);
    // 订阅跟踪初始位姿（融合里程计）
    tracking_initial_sub_ = nh_.subscribe(initial_pose_topic_, 5, &GlobalMatcherNode::onTrackingInitialOdom, this);
    // 订阅源里程计（用于确定点云坐标系）
    source_odom_sub_ = nh_.subscribe(source_odom_topic_, 1, &GlobalMatcherNode::onSourceOdom, this);
    // 订阅引导初始位姿（来自mapping_manager等）
    bootstrap_initial_pose_sub_ = nh_.subscribe(bootstrap_initial_pose_topic_, 2, &GlobalMatcherNode::onBootstrapInitialPose, this);
    // 订阅手动初始位姿（RViz中的2D Pose Estimate）
    manual_initial_pose_sub_ = nh_.subscribe(manual_initial_pose_topic_, 2, &GlobalMatcherNode::onManualInitialPose, this);
    // 订阅加载地图命令
    load_map_command_sub_ = nh_.subscribe(load_map_command_topic_, 2, &GlobalMatcherNode::onLoadMapCommand, this);
    // 订阅切换地图分段命令
    set_map_segment_command_sub_ = nh_.subscribe(set_map_segment_command_topic_, 2, &GlobalMatcherNode::onSetMapSegmentCommand, this);

    // 加载默认地图，失败则发布失败状态
    if (!loadMap()) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE, 0.0, "failed to load map");
    } else {
      publishGlobalMap();
    }

    // 创建匹配定时器：按match_rate频率执行点云配准
    match_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, match_rate_)),
                                  &GlobalMatcherNode::onMatchTimer, this);
    // 创建地图发布定时器：按global_map_publish_period周期发布全局地图
    map_timer_ = nh_.createTimer(ros::Duration(std::max(0.5, global_map_publish_period_)),
                                &GlobalMatcherNode::onMapTimer, this);
  }

 private:
  /**
   * @brief 从ROS参数服务器加载所有配置参数
   *
   * 参数分组：
   * - frames/*: 坐标系名称配置
   * - topics/*: ROS话题名称配置
   * - map/*: 地图加载和发布参数
   * - runtime/*: 运行时行为控制参数
   * - crop/*: 局部地图裁剪范围参数
   * - matcher/*: 点云配准算法参数
   * - gate/*: 匹配质量门控阈值参数
   * - result/*: 结果输出格式参数
   */
  void loadParams() {
    pnh_.param<std::string>("frames/map", map_frame_, "map");
    pnh_.param<std::string>("frames/odom", odom_frame_, "odom");
    pnh_.param<std::string>("frames/base_link", base_frame_, "base_link");
    pnh_.param<std::string>("frames/lidar", lidar_frame_, "lidar_link");
    base_to_lidar_ = loadExtrinsic(pnh_, "extrinsics/base_to_lidar");

    pnh_.param<std::string>("topics/source_cloud", source_cloud_topic_, "/localization/cloud_registered");
    pnh_.param<std::string>("topics/initial_pose", initial_pose_topic_, "/localization/fused_odom");
    pnh_.param<std::string>("topics/source_odom", source_odom_topic_, "/localization/slam_odom");
    pnh_.param<std::string>("topics/bootstrap_initial_pose", bootstrap_initial_pose_topic_, "/localization/manager/initial_pose");
    pnh_.param<std::string>("topics/manual_initial_pose", manual_initial_pose_topic_, "/initialpose");
    pnh_.param<std::string>("topics/result", result_topic_, "/localization/global_matcher/result");
    pnh_.param<std::string>("topics/status", status_topic_, "/localization/global_matcher/status");
    pnh_.param<std::string>("topics/source_initial_scan", source_initial_scan_topic_, "/localization/global_matcher/source_scan_initial");
    pnh_.param<std::string>("topics/aligned_scan", aligned_scan_topic_, "/localization/global_matcher/aligned_scan");
    pnh_.param<std::string>("topics/target_submap", target_submap_topic_, "/localization/global_matcher/target_submap");
    pnh_.param<std::string>("topics/global_map", global_map_topic_, "/localization/global_matcher/global_map");
    pnh_.param<std::string>("topics/initial_guess", initial_guess_topic_, "/localization/global_matcher/initial_guess");
    pnh_.param<std::string>("topics/path", path_topic_, "/localization/global_matcher/path");
    pnh_.param<std::string>("topics/load_map_command", load_map_command_topic_, "/localization/global_matcher/load_map_command");
    pnh_.param<std::string>("topics/set_map_segment_command", set_map_segment_command_topic_, "/localization/global_matcher/set_map_segment_command");

    pnh_.param<std::string>("map/default_map_id", default_map_id_, "default");
    pnh_.param<std::string>("map/pcd_path", pcd_path_, "/home/hhy/2004ros/ysw_loc/src/map/0730map/yuhsuwan2_loc_rot_72.pcd");
    // Applied once at PCD load (ysw-style). 0 disables load-time downsampling and keeps full resolution.
    pnh_.param<double>("map/load_voxel_leaf", map_load_voxel_leaf_, 0.4);
    pnh_.param<double>("map/publish_voxel_leaf", global_map_publish_voxel_leaf_, 5.0);
    pnh_.param<double>("map/global_publish_period", global_map_publish_period_, 5.0);

    pnh_.param<bool>("runtime/auto_match", auto_match_, true);
    pnh_.param<bool>("runtime/viz_test_mode", viz_test_mode_, false);
    pnh_.param<bool>("runtime/source_cloud_is_odom_frame", source_cloud_is_odom_frame_, true);
    pnh_.param<std::string>("runtime/source_cloud_frame", source_cloud_frame_, "auto");
    pnh_.param<double>("runtime/match_rate", match_rate_, 1.0);
    pnh_.param<double>("runtime/max_cloud_age", max_cloud_age_, 1.0);
    pnh_.param<bool>("runtime/use_receive_time_for_cloud_age", use_receive_time_for_cloud_age_, true);
    pnh_.param<int>("runtime/max_path_size", max_path_size_, 3000);
    pnh_.param<bool>("runtime/use_tracking_initial_pose", use_tracking_initial_pose_, true);
    pnh_.param<bool>("runtime/use_bootstrap_initial_pose", use_bootstrap_initial_pose_, true);
    pnh_.param<bool>("runtime/use_manual_initial_pose", use_manual_initial_pose_, true);
    pnh_.param<bool>("runtime/use_default_initial_pose", use_default_initial_pose_, true);
    pnh_.param<bool>("runtime/allow_identity_bootstrap", allow_identity_bootstrap_, true);

    Eigen::Vector3f default_translation = Eigen::Vector3f::Zero();
    Eigen::Vector3f default_rpy_deg = Eigen::Vector3f::Zero();
    loadVector3(pnh_, "runtime/default_initial_pose/translation", &default_translation);
    loadVector3(pnh_, "runtime/default_initial_pose/rpy_deg", &default_rpy_deg);
    default_initial_map_base_ = Eigen::Matrix4f::Identity();
    default_initial_map_base_.block<3, 3>(0, 0) = rpyDegToMatrix(default_rpy_deg);
    default_initial_map_base_.block<3, 1>(0, 3) = default_translation;

    pnh_.param<double>("crop/forward", crop_forward_, 70.0);
    pnh_.param<double>("crop/backward", crop_backward_, 7.0);
    pnh_.param<double>("crop/left", crop_left_, 35.0);
    pnh_.param<double>("crop/right", crop_right_, 35.0);
    pnh_.param<double>("crop/down", crop_down_, 10.0);
    pnh_.param<double>("crop/up", crop_up_, 10.0);

    pnh_.param<std::string>("matcher/type", matcher_type_, "fast_gicp");
    pnh_.param<double>("matcher/source_voxel_leaf", source_voxel_leaf_, 0.1);
    pnh_.param<double>("matcher/target_voxel_leaf", target_voxel_leaf_, 0.1);
    pnh_.param<double>("matcher/coarse_scale", coarse_scale_, 6.0);
    pnh_.param<double>("matcher/fine_scale", fine_scale_, 3.0);
    pnh_.param<int>("matcher/max_iterations", max_iterations_, 20);
    pnh_.param<double>("matcher/transformation_epsilon", transformation_epsilon_, 0.01);
    pnh_.param<double>("matcher/max_correspondence_distance", max_correspondence_distance_, 2.0);
    pnh_.param<int>("matcher/fast_gicp_num_threads", fast_gicp_num_threads_, 2);
    pnh_.param<int>("matcher/fast_gicp_correspondence_randomness", fast_gicp_correspondence_randomness_, 20);

    // [vgicp]
    pnh_.param<double>("matcher/vgicp_voxel_resolution", vgicp_voxel_resolution_, 1.0);
    pnh_.param<int>("matcher/vgicp_num_neighbors", vgicp_num_neighbors_, 20);
    pnh_.param<int>("matcher/vgicp_num_threads", vgicp_num_threads_, 4);

    pnh_.param<int>("gate/min_source_points", min_source_points_, 200);
    pnh_.param<int>("gate/min_target_points", min_target_points_, 1000);
    pnh_.param<double>("gate/max_fitness_score", max_fitness_score_, 1.0);
    pnh_.param<double>("gate/min_inlier_ratio", min_inlier_ratio_, 0.5);
    pnh_.param<double>("gate/max_inlier_rmse", max_inlier_rmse_, 0.8);
    pnh_.param<double>("gate/inlier_distance", inlier_distance_, 0.5);
    pnh_.param<double>("gate/max_delta_translation", max_delta_translation_, 5.0);
    pnh_.param<double>("gate/max_delta_yaw_deg", max_delta_yaw_deg_, 20.0);
    pnh_.param<std::string>("result/pose_frame", result_pose_frame_, "base");

#ifdef GLOBAL_MATCHER_VIZ_TEST
    viz_test_mode_ = true;
#endif
    if (viz_test_mode_) {
      auto_match_ = true;
    }
  }

  /**
   * @brief 加载默认地图和配置的所有地图分段
   * @return bool 加载成功返回true
   *
   * 加载流程：
   * 1. 加载默认地图分段并激活
   * 2. 加载配置文件中指定的其他地图分段（不激活）
   */
  bool loadMap() {
    if (!loadMapSegment(default_map_id_, pcd_path_, true)) {
      return false;
    }
    loadConfiguredMapSegments();
    return true;
  }

  /**
   * @brief 加载单个地图分段
   * @param map_id 地图分段标识
   * @param pcd_path PCD文件路径
   * @param activate 是否立即激活该地图
   * @return bool 加载成功返回true
   *
   * 加载流程：
   * 1. 从PCD文件读取原始点云
   * 2. 移除无效点（NaN）
   * 3. 根据map_load_voxel_leaf_进行降采样（0表示不降采样）
   * 4. 生成可视化用降采样地图
   * 5. 存入map_segments_映射表
   * 6. 如需要则激活该地图
   */
  bool loadMapSegment(const std::string& map_id, const std::string& pcd_path, bool activate) {
    if (map_id.empty() || pcd_path.empty()) {
      ROS_ERROR("[global_matcher] map_id/path is empty, map_id='%s' path='%s'", map_id.c_str(), pcd_path.c_str());
      return false;
    }

    // 从PCD文件加载原始点云
    CloudPtr raw(new CloudT());
    const int ret = pcl::io::loadPCDFile<PointT>(pcd_path, *raw);
    if (ret != 0 || raw->empty()) {
      ROS_ERROR("[global_matcher] failed to load pcd map_id=%s path=%s", map_id.c_str(), pcd_path.c_str());
      return false;
    }
    // 移除包含NaN坐标值的无效点
    raw = removeInvalid(raw);
    if (!raw || raw->empty()) {
      ROS_ERROR("[global_matcher] pcd map is empty after invalid-point filtering, map_id=%s path=%s",
                map_id.c_str(), pcd_path.c_str());
      return false;
    }

    const std::size_t raw_points = raw->size();
    // 根据配置进行加载时降采样（减少内存占用和后续处理时间）
    CloudPtr loaded = raw;
    if (map_load_voxel_leaf_ > 1e-4) {
      loaded = downsample(raw, map_load_voxel_leaf_);  // 0.4
      if (!loaded || loaded->empty()) {
        ROS_ERROR("[global_matcher] pcd map is empty after load voxel leaf=%.3f, map_id=%s path=%s",
                  map_load_voxel_leaf_, map_id.c_str(), pcd_path.c_str());
        return false;
      }
    }

    // 构建地图分段结构体
    MapSegment segment;
    segment.id = map_id;
    segment.path = pcd_path;
    segment.full_map = loaded;  // 用于配准的完整（或降采样后）地图
    // 生成可视化用地图（进一步降采样以减少网络传输）
    segment.viz_map = downsample(segment.full_map, global_map_publish_voxel_leaf_);
    if (!segment.viz_map || segment.viz_map->empty()) {
      segment.viz_map = segment.full_map;
    }

    // VGICP 模式：对整张地图预构建体素化协方差地图（一次性成本），
    // 配准时直接复用，避免每帧对 target 重复降采样/协方差估计/体素化。
    if (matcher_type_ == "vgicp") {
      const ros::Time vg_start = ros::Time::now();
      // segment.vgicp_coarse = buildVgicpVoxelMap(segment.full_map, vgicp_voxel_resolution_ * coarse_scale_);
      // segment.vgicp_fine = buildVgicpVoxelMap(segment.full_map, vgicp_voxel_resolution_ * fine_scale_);
      segment.vgicp_coarse = nullptr;
      segment.vgicp_fine = nullptr;
      ROS_INFO("[global_matcher] built VGICP voxel maps id=%s coarse=%.2fm/%zu voxels fine=%.2fm/%zu voxels cost=%.2fs",
               map_id.c_str(), vgicp_voxel_resolution_ * coarse_scale_,
               segment.vgicp_coarse ? segment.vgicp_coarse->size() : 0,
               vgicp_voxel_resolution_ * fine_scale_,
               segment.vgicp_fine ? segment.vgicp_fine->size() : 0,
               (ros::Time::now() - vg_start).toSec());
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_segments_[map_id] = segment;
      if (activate) {
        activateSegmentLocked(map_id);
      }
    }

    ROS_INFO("[global_matcher] loaded map segment id=%s path=%s raw_points=%zu load_leaf=%.3f points=%zu viz_points=%zu activate=%d",
             map_id.c_str(), pcd_path.c_str(), raw_points, map_load_voxel_leaf_,
             segment.full_map->size(), segment.viz_map->size(), activate);
    if (activate) {
      publishGlobalMap();
    }
    return true;
  }

  /**
   * @brief 从参数服务器加载配置的额外地图分段
   *
   * 期望参数格式为数组，每个元素为包含id和path字段的结构体：
   * map/segments:
   *   - {id: "segment1", path: "/path/to/map1.pcd"}
   *   - {id: "segment2", path: "/path/to/map2.pcd"}
   */
  void loadConfiguredMapSegments() {
    XmlRpc::XmlRpcValue segments;
    if (!pnh_.getParam("map/segments", segments)) {
      return;
    }
    if (segments.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("[global_matcher] map/segments must be a list of {id, path}; ignore it");
      return;
    }

    for (int i = 0; i < segments.size(); ++i) {
      if (segments[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !segments[i].hasMember("id") || !segments[i].hasMember("path")) {
        ROS_WARN("[global_matcher] invalid map/segments[%d], require id and path", i);
        continue;
      }
      const std::string id = static_cast<std::string>(segments[i]["id"]);
      const std::string path = static_cast<std::string>(segments[i]["path"]);
      // 跳过已加载的默认地图
      if (id == default_map_id_ && path == pcd_path_) {
        continue;
      }
      loadMapSegment(id, path, false);
    }
  }

  /**
   * @brief 激活指定地图分段（必须在已持有mutex锁的情况下调用）
   * @param map_id 要激活的地图分段标识
   * @return bool 激活成功返回true
   *
   * 激活操作：
   * 1. 设置active_map_id_
   * 2. 更新global_map_和global_map_viz_指针
   * 3. 清空历史轨迹（path_）
   */
  bool activateSegmentLocked(const std::string& map_id) {
    auto it = map_segments_.find(map_id);
    if (it == map_segments_.end() || !it->second.full_map || it->second.full_map->empty()) {
      return false;
    }
    active_map_id_ = map_id;
    global_map_ = it->second.full_map;      // 用于配准的全局地图
    global_map_viz_ = it->second.viz_map;   // 用于可视化的降采样地图
    vgicp_coarse_voxelmap_ = it->second.vgicp_coarse;  // VGICP 预构建体素地图随地图分段切换
    vgicp_fine_voxelmap_ = it->second.vgicp_fine;
    path_.poses.clear();  // 切换地图后清空历史轨迹
    return true;
  }

  /**
   * @brief 实时点云数据回调函数
   * @param msg PointCloud2消息指针
   *
   * 处理流程：
   * 1. 将ROS消息转换为PCL点云格式
   * 2. 移除无效点（NaN）
   * 3. 记录接收时间戳
   * 4. 在mutex保护下更新latest_source_等成员变量
   */
  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    CloudPtr cloud(new CloudT());
    pcl::fromROSMsg(*msg, *cloud);
    cloud = removeInvalid(cloud);
    const ros::Time receive_stamp = ros::Time::now();
    std::lock_guard<std::mutex> lock(mutex_);
    latest_source_ = cloud;
    // 如果消息时间戳为0，使用接收时间作为备用
    latest_source_stamp_ = msg->header.stamp.isZero() ? receive_stamp : msg->header.stamp;
    latest_source_receive_stamp_ = receive_stamp;
    latest_source_frame_ = msg->header.frame_id;
  }

  /**
   * @brief 跟踪初始位姿回调（融合里程计）
   * @param msg Odometry消息指针，包含map->base_link的位姿
   *
   * 该回调接收来自simple_odom_fusion的融合位姿，
   * 在TRACKING模式下作为配准的初始猜测。
   */
  void onTrackingInitialOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_tracking_map_base_ = poseToMatrix(msg->pose.pose);
    latest_tracking_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_tracking_map_base_ = true;
  }

  /**
   * @brief 源里程计回调
   * @param msg Odometry消息指针，通常包含odom->base_link的位姿
   *
   * 用于确定source cloud的坐标系，以及构建map->source的初始猜测。
   * 当source_cloud_is_odom_frame_=true时，source cloud处于odom坐标系。
   */
  void onSourceOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_source_base_ = poseToMatrix(msg->pose.pose);
    latest_source_odom_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_source_odom_ = true;
  }

  /**
   * @brief 引导初始位姿回调
   * @param msg PoseWithCovarianceStamped消息指针
   *
   * 接收来自mapping_manager或其他节点的初始位姿，
   * 在BOOTSTRAP模式下使用。通常用于系统启动时的初始定位。
   */
  void onBootstrapInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_bootstrap_map_base_ = poseToMatrix(msg->pose.pose);
    latest_bootstrap_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_bootstrap_map_base_ = true;
    ROS_INFO("[global_matcher] received bootstrap initial pose from %s", bootstrap_initial_pose_topic_.c_str());
  }

  /**
   * @brief 手动初始位姿回调（RViz 2D Pose Estimate）
   * @param msg PoseWithCovarianceStamped消息指针
   *
   * 接收RViz中手动设置的初始位姿，需要use_manual_initial_pose_参数为true才生效。
   * 通常用于调试或在丢失定位时手动重新初始化。
   */
  void onManualInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!use_manual_initial_pose_) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_bootstrap_map_base_ = poseToMatrix(msg->pose.pose);
    latest_bootstrap_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    have_bootstrap_map_base_ = true;
    // 与 ysw_loc 的 need_init 一致：人工初值直接成为下一次匹配的初值（map->source = 人工map位姿 × odom⁻¹）
    if (have_source_odom_) {
      last_map_source_ = latest_bootstrap_map_base_ * latest_source_base_.inverse();
      have_last_map_source_ = true;
    }
    ++manual_reset_seq_;
    ROS_INFO("[global_matcher] received manual initial pose from %s", manual_initial_pose_topic_.c_str());
  }

  /**
   * @brief 加载地图命令回调
   * @param msg String消息，格式为 "map_id|pcd_path|activate_flag"
   *
   * 通过ROS话题动态加载新的地图分段。
   * 例如："segment1|/path/to/map.pcd|true"
   */
  void onLoadMapCommand(const std_msgs::String::ConstPtr& msg) {
    std::vector<std::string> fields;
    std::stringstream ss(msg->data);
    std::string item;
    while (std::getline(ss, item, '|')) {
      fields.push_back(item);
    }
    if (fields.size() < 2) {
      fields.clear();
      std::stringstream csv(msg->data);
      while (std::getline(csv, item, ',')) {
        fields.push_back(item);
      }
    }
    if (fields.size() < 2) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0,
                    "load_map_command format: map_id|pcd_path[|activate]");
      return;
    }

    bool activate = true;
    if (fields.size() >= 3) {
      const std::string flag = fields[2];
      activate = !(flag == "0" || flag == "false" || flag == "False" || flag == "no");
    }
    if (!loadMapSegment(fields[0], fields[1], activate)) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_FAILURE, 0.0, "failed to load map segment");
      return;
    }
    publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL, 1.0,
                  activate ? "loaded and activated map segment" : "loaded map segment");
  }

  /**
   * @brief 切换地图分段命令回调
   * @param msg String消息，内容为要激活的地图分段id
   *
   * 切换到已加载的地图分段，切换后会清空历史轨迹并发布新地图。
   */
  void onSetMapSegmentCommand(const std_msgs::String::ConstPtr& msg) {
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ok = activateSegmentLocked(msg->data);
    }
    if (!ok) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0,
                    "requested map segment is not loaded: " + msg->data);
      return;
    }
    ROS_INFO("[global_matcher] activated map segment id=%s", msg->data.c_str());
    publishGlobalMap();
    publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL, 1.0, "activated map segment");
  }

  /**
   * @brief 匹配定时器回调
   * @param event 定时器事件（未使用）
   *
   * 按配置的match_rate频率触发，检查auto_match_标志后执行runMatch()。
   * 如果auto_match_为false，发布警告状态。
   */
  void onMatchTimer(const ros::TimerEvent&) {
    if (!auto_match_) {
      publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0, "auto_match disabled");
      return;
    }
    runMatch();
  }

  /**
   * @brief 地图发布定时器回调
   * @param event 定时器事件（未使用）
   *
   * 在viz_test_mode_模式下定期发布全局地图，用于可视化调试。
   */
  void onMapTimer(const ros::TimerEvent&) {
    if (viz_test_mode_) {
      publishGlobalMap();
    }
  }

  /**
   * @brief 执行完整的全局匹配流程（核心函数）
   *
   * 匹配流程：
   * 1. 从mutex保护的数据成员中复制最新数据（避免长时间持有锁）
   * 2. 检查地图和点云数据的有效性
   * 3. 检查点云时效性（cloud_age < max_cloud_age_）
   * 4. 检查点云点数是否满足最小要求
   * 5. 选择初始位姿猜测（跟踪/引导/默认）
   * 6. 发布初始猜测位姿和变换后的初始点云（用于可视化）
   * 7. 从全局地图裁剪局部子地图
   * 8. 执行粗粒度配准（coarse_scale_）
   * 9. 执行细粒度配准（fine_scale_），以粗配准结果为初始值
   * 10. 计算内点指标（inlier ratio, RMSE）
   * 11. 计算与初始猜测的偏差（translation, yaw）
   * 12. 质量门控判断（fitness score, inlier ratio, RMSE, delta translation/yaw）
   * 13. 更新失败计数和运行模式
   * 14. 发布对齐点云、匹配结果、轨迹和状态
   *
   * 坐标系说明：
   * - map: 地图坐标系（全局固定坐标系）
   * - odom: 里程计坐标系（局部固定坐标系）
   * - base: 车体坐标系（base_link）
   * - source: 点云数据的坐标系（可能是odom或lidar）
   */
  void runMatch() {
    CloudPtr source;
    ros::Time source_stamp;
    ros::Time source_receive_stamp;
    Eigen::Matrix4f tracking_map_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f bootstrap_map_base = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f source_odom_base = Eigen::Matrix4f::Identity();
    bool have_tracking = false;
    bool have_bootstrap = false;
    bool have_source_odom = false;
    std::string source_frame;
    uint64_t match_start_seq = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      source = latest_source_;
      source_stamp = latest_source_stamp_;
      source_receive_stamp = latest_source_receive_stamp_;
      tracking_map_base = latest_tracking_map_base_;
      bootstrap_map_base = latest_bootstrap_map_base_;
      source_odom_base = latest_source_base_;
      have_tracking = have_tracking_map_base_;
      have_bootstrap = have_bootstrap_map_base_;
      have_source_odom = have_source_odom_;
      source_frame = latest_source_frame_;
      match_start_seq = manual_reset_seq_;
    }

    if (!global_map_ || global_map_->empty()) {
      publishRejected(source_stamp, "map not loaded");
      return;
    }
    if (!source || source->empty()) {
      publishRejected(ros::Time::now(), "waiting source cloud");
      return;
    }
    const ros::Time now = ros::Time::now();
    const ros::Time cloud_age_stamp = use_receive_time_for_cloud_age_ ? source_receive_stamp : source_stamp;
    const double cloud_age = cloud_age_stamp.isZero() ? std::numeric_limits<double>::infinity()
                                                       : (now - cloud_age_stamp).toSec();
    if (cloud_age > max_cloud_age_) {
      const double message_age = source_stamp.isZero() ? std::numeric_limits<double>::infinity()
                                                       : (now - source_stamp).toSec();
      const double receive_age = source_receive_stamp.isZero() ? std::numeric_limits<double>::infinity()
                                                               : (now - source_receive_stamp).toSec();
      ROS_WARN_THROTTLE(1.0,
                        "[global_matcher] source cloud timeout: age=%.3fs limit=%.3fs mode=%s msg_age=%.3fs receive_age=%.3fs msg_stamp=%.9f receive_stamp=%.9f now=%.9f",
                        cloud_age, max_cloud_age_,
                        use_receive_time_for_cloud_age_ ? "receive_time" : "measurement_time",
                        message_age, receive_age, source_stamp.toSec(), source_receive_stamp.toSec(), now.toSec());
      publishRejected(source_stamp, "source cloud timeout");
      return;
    }
    if (static_cast<int>(source->size()) < min_source_points_) {
      publishRejected(source_stamp, "source points below threshold");
      return;
    }

    const InitialGuess guess = selectInitialGuess(source_frame, have_tracking, tracking_map_base,
                                                  have_bootstrap, bootstrap_map_base,
                                                  have_source_odom, source_odom_base);
    current_mode_ = modeToString(guess.mode);
    if (!guess.valid) {
      publishRejected(source_stamp, guess.reason);
      return;
    }
    const Eigen::Matrix4f initial_map_source = guess.map_source;
    const Eigen::Matrix4f source_base = guess.source_base;
    const Eigen::Matrix4f initial_map_base_for_crop = guess.map_base;

    publishInitialGuess(source_stamp, initial_map_base_for_crop);
    publishSourceInitial(source_stamp, source, initial_map_source);
    const ros::Time start = ros::Time::now();
    CloudPtr target_submap = cropMap(initial_map_base_for_crop);

    
    // 把target_submap构建体素地图
    vgicp_coarse_map = buildVgicpVoxelMap(target_submap, vgicp_voxel_resolution_ * coarse_scale_);
    vgicp_fine_map = buildVgicpVoxelMap(target_submap, vgicp_voxel_resolution_ * fine_scale_);

    if (!target_submap || static_cast<int>(target_submap->size()) < min_target_points_) {
      publishSubmap(source_stamp, target_submap);
      publishRejected(source_stamp, "target submap points below threshold");
      return;
    }
    publishSubmap(source_stamp, target_submap);

    // const ros::Time start = ros::Time::now();
    double coarse_fitness = std::numeric_limits<double>::infinity();
    bool coarse_converged = false;
    Eigen::Matrix4f coarse = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f final_map_source = Eigen::Matrix4f::Identity();
    double fine_fitness = std::numeric_limits<double>::infinity();
    bool fine_converged = false;

    if (matcher_type_ == "vgicp") {
      coarse = registerAtScale(source, target_submap, initial_map_source,
                                                   coarse_scale_, &coarse_fitness, &coarse_converged);
      final_map_source = registerAtScale(source, target_submap, coarse,
                                                   fine_scale_, &fine_fitness, &fine_converged);
    } else {
      coarse = registerAtScale(source, target_submap, initial_map_source,
                                                   coarse_scale_, &coarse_fitness, &coarse_converged);
      // fine_fitness = std::numeric_limits<double>::infinity();
      // bool fine_converged = false;
      final_map_source = registerAtScale(source, target_submap, coarse,
                                                              fine_scale_, &fine_fitness, &fine_converged);
    }

    // 如果使用fast_gicp得到初始位姿， small_gicp得到精确位姿






    // const Eigen::Matrix4f coarse = registerAtScale(source, target_submap, initial_map_source,
    //                                                coarse_scale_, &coarse_fitness, &coarse_converged);
    // double fine_fitness = std::numeric_limits<double>::infinity();
    // bool fine_converged = false;
    // const Eigen::Matrix4f final_map_source = registerAtScale(source, target_submap, coarse,
    //                                                         fine_scale_, &fine_fitness, &fine_converged);
    // const double elapsed_ms = (ros::Time::now() - start).toSec() * 1000.0;
    const Eigen::Matrix4f final_map_base = final_map_source * source_base;

    MatchMetrics metrics = computeInliers(source, target_submap, final_map_source, inlier_distance_);
    const double delta_translation = translationDiff(final_map_base, initial_map_base_for_crop);
    const double delta_yaw = std::abs(yawDiff(final_map_base, initial_map_base_for_crop));

    const double elapsed_ms = (ros::Time::now() - start).toSec() * 1000.0;

    // VGICP 不提供 PCL 式 fitness：用 computeInliers 统计的平均最近邻距离填充日志/消息字段，
    // 成功判定以内点率 / RMSE 为主（fitness 门限仅对 icp/gicp/fast_gicp 生效）。
    if (matcher_type_ == "vgicp" && std::isfinite(metrics.fitness)) {
      fine_fitness = metrics.fitness;
    }

    // Gate by match quality only. hasConverged() means the last ICP step was small enough,
    // not that the alignment is bad — treat it as a soft warning.
    std::string reject_reason;
    bool success = true;
    if (matcher_type_ != "vgicp" && fine_fitness > max_fitness_score_) {
      reject_reason = "fitness score above threshold";
      success = false;
    } else if (metrics.inlier_ratio < min_inlier_ratio_) {
      reject_reason = "inlier ratio below threshold";
      success = false;
    } else if (metrics.inlier_rmse > max_inlier_rmse_) {
      reject_reason = "inlier rmse above threshold";
      success = false;
    } else if (delta_translation > max_delta_translation_) {
      reject_reason = "translation delta above threshold";
      success = false;
    } else if (delta_yaw > max_delta_yaw_deg_ * kPi / 180.0) {
      reject_reason = "yaw delta above threshold";
      success = false;
    }

    if (success && !fine_converged) {
      ROS_WARN_THROTTLE(2.0,
                        "[global_matcher] matcher hasConverged=false but quality gates passed "
                        "(fitness=%.4f inlier=%.3f rmse=%.3f); accepting result",
                        fine_fitness, metrics.inlier_ratio, metrics.inlier_rmse);
    }

    // 匹配期间收到了新的人工初值：本帧结果是基于旧初值算的，丢弃，避免旧结果覆盖新锚点
    // （等价于 ysw_loc 的 need_init 在匹配线程内同步生效，不存在“旧结果补刀”窗口）。
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (manual_reset_seq_ != match_start_seq) {
        ROS_WARN_THROTTLE(1.0,
                          "[global_matcher] manual initial pose arrived during match, discard stale result");
        publishRejected(source_stamp, "manual pose reset during match");
        return;
      }
    }

    // 与 ysw_loc 一致：保存上一次成功匹配的 map->source（source 在 odom 系时即 map->odom），
    // 作为下一帧初值，串行、无消息回路延迟。
    if (success) {
      last_map_source_ = final_map_source;
      have_last_map_source_ = true;
    }

    updateModeAfterMatch(success);
    publishAligned(source_stamp, source, final_map_source);
    const bool publish_map_source = result_pose_frame_ == "source" || result_pose_frame_ == "odom";
    const Eigen::Matrix4f result_pose = publish_map_source ? final_map_source : final_map_base;
    const std::string result_child_frame = publish_map_source ? odom_frame_ : base_frame_;
    publishResult(source_stamp, success, fine_converged, result_pose, result_child_frame, final_map_base,
                  fine_fitness, metrics, elapsed_ms, static_cast<uint32_t>(target_submap->size()),
                  delta_translation, delta_yaw, reject_reason);
    publishStatus(success ? deeplumin_msgs::LocalizationStatus::LEVEL_NORMAL
                          : deeplumin_msgs::LocalizationStatus::LEVEL_WARNING,
                  success ? 1.0 : 0.2,
                  success ? "" : reject_reason);

    ROS_INFO("[global_matcher] mode=%s success=%d converged=%d source=%zu target=%zu fitness=%.4f inlier=%.3f rmse=%.3f delta=%.3fm yaw=%.2fdeg cost=%.1fms reason=%s",
             current_mode_.c_str(), success, fine_converged, source->size(), target_submap->size(), fine_fitness,
             metrics.inlier_ratio, metrics.inlier_rmse, delta_translation,
             delta_yaw * 180.0 / kPi, elapsed_ms, reject_reason.c_str());
  }

  /**
   * @brief 选择初始位姿猜测
   * @param source_frame 点云消息的坐标系id
   * @param have_tracking 是否有跟踪位姿可用
   * @param tracking_map_base 跟踪模式下的map->base变换
   * @param have_bootstrap 是否有引导位姿可用
   * @param bootstrap_map_base 引导模式下的map->base变换
   * @param have_source_odom 是否有源里程计可用
   * @param source_odom_base 源里程计的odom->base变换
   * @return InitialGuess 初始猜测结构体
   *
   * 选择优先级（从高到低）：
   * 1. TRACKING模式：使用融合里程计（fused_odom）作为初始猜测
   *    - 即使之前匹配失败也继续使用，避免跳回地图原点
   * 2. BOOTSTRAP模式：
   *    a. 使用引导初始位姿（来自mapping_manager）
   *    b. 使用默认初始位姿（参数配置）
   *    c. 使用单位矩阵（identity，原点）
   *
   * 坐标系转换：
   * - 如果source cloud在odom坐标系：map_source = map_base * odom_base^{-1}
   * - 如果source cloud在lidar坐标系：map_source = map_base * base_to_lidar
   * - 否则假设source cloud在base坐标系：map_source = map_base
   */
  InitialGuess selectInitialGuess(const std::string& source_frame,
                                  bool have_tracking,
                                  const Eigen::Matrix4f& tracking_map_base,
                                  bool have_bootstrap,
                                  const Eigen::Matrix4f& bootstrap_map_base,
                                  bool have_source_odom,
                                  const Eigen::Matrix4f& source_odom_base) const {
    // 确定source cloud的坐标系模式
    const std::string source_mode = (source_cloud_frame_ == "auto") ? source_frame : source_cloud_frame_;
    const bool source_is_odom = source_cloud_is_odom_frame_ || source_mode == "odom" || source_mode == odom_frame_;
    const bool source_is_lidar = source_mode == "lidar" || source_mode == lidar_frame_;

    InitialGuess guess;
    bool have_map_base = false;
    Eigen::Matrix4f map_base = Eigen::Matrix4f::Identity();

    // 优先级1：跟踪模式。默认用上一次成功匹配的 map->source 作为初值（与 ysw_loc 的
    // T_map_to_odom 串行结构一致），避免 fused odom 话题异步回读带来的滞后；
    // 没有历史结果时才使用 fused odom。
    if (use_tracking_initial_pose_ && (have_tracking || have_last_map_source_)) {
      guess.mode = MatcherRunMode::TRACKING;
      if (have_last_map_source_) {
        guess.reason = "tracking:last_result";
        guess.map_source = last_map_source_;
        guess.source_base = have_source_odom ? source_odom_base : Eigen::Matrix4f::Identity();
        guess.map_base = guess.map_source * guess.source_base;
        ROS_INFO_THROTTLE(2.0, "[global_matcher] mode=%s initial=%s source_frame=%s failures=%d",
                          modeToString(guess.mode).c_str(), guess.reason.c_str(), source_mode.c_str(),
                          consecutive_failures_);
        return guess;
      }
      // 在线跟踪即使在匹配失败后也要继续使用fused_odom。
      // 失败的匹配帧只是被拒绝的观测，不是跳回地图原点/默认位置的理由。
      guess.reason = "tracking:fused_odom";
      map_base = tracking_map_base;
      have_map_base = true;
    } else {
      // 优先级2：引导模式（BOOTSTRAP）
      guess.mode = MatcherRunMode::BOOTSTRAP;
      if (use_bootstrap_initial_pose_ && have_bootstrap) {
        guess.reason = "bootstrap:initial_pose";
        map_base = bootstrap_map_base;
        have_map_base = true;
      } else if (use_default_initial_pose_) {
        guess.reason = "bootstrap:default_initial_pose";
        map_base = default_initial_map_base_;
        have_map_base = true;
      } else if (allow_identity_bootstrap_) {
        guess.reason = "bootstrap:identity";
        map_base = Eigen::Matrix4f::Identity();
        have_map_base = true;
      }
    }

    // 没有任何初始位姿可用
    if (!have_map_base) {
      guess.mode = MatcherRunMode::BOOTSTRAP;
      guess.reason = "bootstrap:no_initial_available";
      guess.valid = false;
      return guess;
    }

    // 根据source cloud的坐标系计算map->source变换
    if (source_is_odom) {
      // source cloud在odom坐标系：需要source_odom来构建map->source
      guess.source_base = have_source_odom ? source_odom_base : Eigen::Matrix4f::Identity();
      guess.map_source = have_source_odom ? map_base * source_odom_base.inverse() : map_base;
      if (!have_source_odom) {
        ROS_WARN_THROTTLE(2.0, "[global_matcher] source_odom missing, using map->base as map->source guess in %s",
                          guess.reason.c_str());
      }
    } else if (source_is_lidar) {
      // source cloud在lidar坐标系：使用base_to_lidar外参
      guess.source_base = base_to_lidar_.inverse();
      guess.map_source = map_base * base_to_lidar_;
    } else {
      // 假设source cloud在base坐标系
      guess.source_base = Eigen::Matrix4f::Identity();
      guess.map_source = map_base;
    }
    // 验证：map_base应该等于map_source * source_base
    guess.map_base = guess.map_source * guess.source_base;
    ROS_INFO_THROTTLE(2.0, "[global_matcher] mode=%s initial=%s source_frame=%s failures=%d",
                      modeToString(guess.mode).c_str(), guess.reason.c_str(), source_mode.c_str(), consecutive_failures_);
    return guess;
  }

  /**
   * @brief 根据匹配结果更新运行模式
   * @param success 本次匹配是否成功通过质量门控
   *
   * 成功时重置连续失败计数，失败时增加计数。
   * 连续失败次数会影响初始猜测的选择策略。
   */
  void updateModeAfterMatch(bool success) {
    if (success) {
      consecutive_failures_ = 0;
      return;
    }
    ++consecutive_failures_;
  }

  /**
   * @brief 从全局地图中裁剪局部子地图
   * @param map_base 当前估计的map->base变换（用于确定裁剪中心）
   * @return CloudPtr 裁剪后的局部点云指针
   *
   * 使用PCL的CropBox滤波器，以base_link位置为中心，
   * 按配置的crop_forward/backward/left/right/up/down范围裁剪。
   * 注意：CropBox的坐标系是在map_base处，所以需要传入map_base的逆变换。
   */
  CloudPtr cropMap(const Eigen::Matrix4f& map_base) const {
    CloudPtr cropped(new CloudT());
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(global_map_);
    // 设置裁剪框范围（以base_link为中心的车辆坐标系）
    crop.setMin(Eigen::Vector4f(static_cast<float>(-crop_backward_),   // 后方
                                static_cast<float>(-crop_right_),      // 右侧
                                static_cast<float>(-crop_down_),       // 下方
                                1.0f));
    crop.setMax(Eigen::Vector4f(static_cast<float>(crop_forward_),     // 前方
                                static_cast<float>(crop_left_),        // 左侧
                                static_cast<float>(crop_up_),          // 上方
                                1.0f));
    // 将裁剪框变换到地图坐标系（使用map_base的逆）
    crop.setTransform(Eigen::Affine3f(map_base.inverse()));
    crop.filter(*cropped);
    return cropped;
  }

  /**
   * @brief 将 PCL 点云转换为 small_gicp::PointCloud
   */
  small_gicp::PointCloud::Ptr toSmallGicpCloud(const CloudConstPtr& cloud) const {
    std::vector<Eigen::Vector4f> pts;
    if (cloud) {
      pts.reserve(cloud->size());
      for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        pts.emplace_back(p.x, p.y, p.z, 1.0f);
      }
    }
    return std::make_shared<small_gicp::PointCloud>(pts);
  }

  /**
   * @brief 对整张地图构建 VGICP 体素化协方差地图（一次性预处理）
   * @param cloud            地图点云（已按 load_voxel_leaf 降采样）
   * @param voxel_resolution 体素分辨率
   */
  small_gicp::GaussianVoxelMap::Ptr buildVgicpVoxelMap(const CloudConstPtr& cloud, double voxel_resolution) const {
    if (!cloud || cloud->empty() || voxel_resolution <= 1e-4) {
      return nullptr;
    }
    auto pc = toSmallGicpCloud(cloud);
    if (!pc || pc->size() < 4) {
      return nullptr;
    }
    small_gicp::estimate_covariances_omp(*pc, vgicp_num_neighbors_, vgicp_num_threads_);
    auto voxelmap = std::make_shared<small_gicp::GaussianVoxelMap>(voxel_resolution);
    voxelmap->set_search_offsets(27);
    voxelmap->insert(*pc);
    return voxelmap;
  }

  Eigen::Matrix4f registerAtScale(const CloudConstPtr& source,
                                  const CloudConstPtr& target,
                                  const Eigen::Matrix4f& initial_guess,
                                  double scale,
                                  double* fitness,
                                  bool* converged) const {
    // 与 ysw_loc 一致：粗/细两阶段按 scale 缩放体素（0.08 -> 粗 0.48 / 细 0.24），
    // 对应距离固定不缩放（ysw_loc 两阶段都用同一个 max_correspondence_distance）。
    const double source_leaf = (matcher_type_ != "vgicp")
                                   ? std::max(0.01, source_voxel_leaf_ * scale)
                                   : std::max(0.01, source_voxel_leaf_);
    const double target_leaf = (matcher_type_ != "vgicp")
                                   ? std::max(0.01, target_voxel_leaf_ * scale)
                                   : std::max(0.01, target_voxel_leaf_);
    CloudPtr source_filtered = downsample(source, source_leaf);
    // VGICP：target 使用整张地图的预构建体素地图，无需每帧降采样/体素化 target
    CloudPtr target_filtered;
    if (matcher_type_ != "vgicp") {
      target_filtered = downsample(target, target_leaf);
    }
    // target_filtered = downsample(target, target_leaf);

    if (source_filtered->size() < 4 || (matcher_type_ != "vgicp" && (!target_filtered || target_filtered->size() < 4))) {
      if (fitness) *fitness = std::numeric_limits<double>::infinity();
      if (converged) *converged = false;
      return initial_guess;
    }

    pcl::PointCloud<PointT> aligned;
    bool has_converged = false;
    double score = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f transform = initial_guess;

    if (matcher_type_ == "icp") {
      pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setInputSource(source_filtered);
      icp.setInputTarget(target_filtered);
      icp.setMaximumIterations(max_iterations_);
      icp.setTransformationEpsilon(transformation_epsilon_);
      icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
      icp.align(aligned, initial_guess);
      has_converged = icp.hasConverged();
      score = icp.getFitnessScore();
      // 与 ysw_loc 一致：未收敛时保持初始猜测（忽略本帧），避免坏迭代结果污染修正量
      transform = has_converged ? icp.getFinalTransformation() : initial_guess;
    } else if (matcher_type_ == "gicp") {
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setInputSource(source_filtered);
      gicp.setInputTarget(target_filtered);
      gicp.setMaximumIterations(max_iterations_);
      gicp.setTransformationEpsilon(transformation_epsilon_);
      gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);
      gicp.align(aligned, initial_guess);
      has_converged = gicp.hasConverged();
      score = gicp.getFitnessScore();
      transform = has_converged ? gicp.getFinalTransformation() : initial_guess;
    } else if (matcher_type_ == "vgicp") {
      // target 为整张地图的预构建体素地图（coarse/fine 两个分辨率）。
      // IncrementalVoxelMap 的最近邻搜索只查询变换后点所在 voxel 的邻近体素，
      // 因此以初始位姿作为初值后，对应点搜索被天然限制在初始位姿附近区域，
      // 无需遍历全图，也无需每帧重新体素化 target。
      // small_gicp::GaussianVoxelMap::Ptr voxelmap =
      //     (std::abs(scale - coarse_scale_) < 1e-6) ? vgicp_coarse_voxelmap_ : vgicp_fine_voxelmap_;
      small_gicp::GaussianVoxelMap::Ptr voxelmap =
          (std::abs(scale - coarse_scale_) < 1e-6) ? vgicp_coarse_map : vgicp_fine_map;
      // small_gicp::GaussianVoxelMap::Ptr voxelmap = vgicp_fine_map;

      auto source_pc = toSmallGicpCloud(source_filtered);
      if (!voxelmap || voxelmap->size() < 4 || !source_pc || source_pc->size() < 4) {
        if (fitness) *fitness = std::numeric_limits<double>::infinity();
        if (converged) *converged = false;
        return initial_guess;
      }

      // source 协方差（VGICP 因子依赖 source 点协方差）
      small_gicp::estimate_covariances_omp(*source_pc, vgicp_num_neighbors_, vgicp_num_threads_);

      // 配准：target=预构建体素地图，target_tree=voxelmap 本身
      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> reg;
      reg.reduction.num_threads = vgicp_num_threads_;
      reg.rejector.max_dist_sq = (max_correspondence_distance_ * scale) * (max_correspondence_distance_ * scale);
      reg.criteria.translation_eps = transformation_epsilon_;
      reg.criteria.rotation_eps = transformation_epsilon_;
      reg.optimizer.max_iterations = max_iterations_;

      Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
      init.matrix() = initial_guess.cast<double>();
      auto result = reg.align(*voxelmap, *source_pc, *voxelmap, init);

      has_converged = result.converged;
      transform = has_converged ? result.T_target_source.matrix().cast<float>() : initial_guess;
      // VGICP 无 PCL 式 fitness：score 保持 inf，日志/消息用 computeInliers 的平均最近邻距离（见 runMatch）
    } else {
      fast_gicp::FastGICP<PointT, PointT> gicp;
      gicp.setNumThreads(std::max(1, fast_gicp_num_threads_));
      gicp.setCorrespondenceRandomness(std::max(1, fast_gicp_correspondence_randomness_));
      gicp.setInputSource(source_filtered);
      gicp.setInputTarget(target_filtered);
      gicp.setMaximumIterations(max_iterations_);
      gicp.setTransformationEpsilon(transformation_epsilon_);
      gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);
      gicp.align(aligned, initial_guess);
      has_converged = gicp.hasConverged();
      score = gicp.getFitnessScore();
      transform = has_converged ? gicp.getFinalTransformation() : initial_guess;
    }

    if (fitness) *fitness = score;
    if (converged) *converged = has_converged;
    return transform;
  }

  MatchMetrics computeInliers(const CloudConstPtr& source,
                              const CloudConstPtr& target,
                              const Eigen::Matrix4f& map_source,
                              double threshold) const {
    MatchMetrics metrics;
    if (!source || !target || source->empty() || target->empty()) {
      return metrics;
    }

    CloudPtr transformed(new CloudT());
    pcl::transformPointCloud(*source, *transformed, map_source);

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target);
    std::vector<int> indices(1);
    std::vector<float> sq_distances(1);
    const double threshold_sq = threshold * threshold;
    double sum_sq = 0.0;       // 内点距离平方和（RMSE 用）
    double sum_all_sq = 0.0;   // 全部对应点距离平方和（fitness 用，PCL 语义）

    for (const auto& pt : transformed->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      if (kdtree.nearestKSearch(pt, 1, indices, sq_distances) <= 0) {
        continue;
      }
      ++metrics.total_points;
      sum_all_sq += sq_distances[0];
      if (sq_distances[0] <= threshold_sq) {
        ++metrics.inlier_count;
        sum_sq += sq_distances[0];
      }
    }
    if (metrics.total_points > 0) {
      metrics.inlier_ratio = static_cast<double>(metrics.inlier_count) / metrics.total_points;
      metrics.fitness = sum_all_sq / metrics.total_points;
    }
    if (metrics.inlier_count > 0) {
      metrics.inlier_rmse = std::sqrt(sum_sq / metrics.inlier_count);
    }
    return metrics;
  }

  void publishGlobalMap() const {
    const CloudPtr cloud = (global_map_viz_ && !global_map_viz_->empty()) ? global_map_viz_ : global_map_;
    if (!cloud || cloud->empty() || global_map_pub_.getNumSubscribers() == 0) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    global_map_pub_.publish(msg);
  }

  void publishSubmap(const ros::Time& stamp, const CloudConstPtr& cloud) const {
    if (!cloud || submap_pub_.getNumSubscribers() == 0) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    submap_pub_.publish(msg);
  }

  void publishSourceInitial(const ros::Time& stamp,
                            const CloudConstPtr& source,
                            const Eigen::Matrix4f& map_source) const {
    if (!source || source_initial_pub_.getNumSubscribers() == 0) {
      return;
    }
    CloudPtr transformed(new CloudT());
    pcl::transformPointCloud(*source, *transformed, map_source);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*transformed, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    source_initial_pub_.publish(msg);
  }

  void publishAligned(const ros::Time& stamp, const CloudConstPtr& source, const Eigen::Matrix4f& map_source) const {
    if (!source || aligned_pub_.getNumSubscribers() == 0) {
      return;
    }
    CloudPtr aligned(new CloudT());
    pcl::transformPointCloud(*source, *aligned, map_source);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*aligned, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    aligned_pub_.publish(msg);
  }

  void publishInitialGuess(const ros::Time& stamp, const Eigen::Matrix4f& map_base) const {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose = matrixToPose(map_base);
    initial_guess_pub_.publish(msg);
  }

  void publishResult(const ros::Time& stamp,
                     bool success,
                     bool converged,
                     const Eigen::Matrix4f& result_pose,
                     const std::string& result_child_frame,
                     const Eigen::Matrix4f& path_map_base,
                     double fitness,
                     const MatchMetrics& metrics,
                     double elapsed_ms,
                     uint32_t target_points,
                     double delta_translation,
                     double delta_yaw,
                     const std::string& reject_reason) {
    deeplumin_msgs::GlobalMatchResult msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.child_frame_id = result_child_frame;
    msg.pose.pose = matrixToPose(result_pose);
    const double cov_pos = std::max(0.01, metrics.inlier_rmse * metrics.inlier_rmse);
    const double cov_yaw = std::pow(std::max(0.5, std::abs(delta_yaw) * 180.0 / kPi) * kPi / 180.0, 2.0);
    for (int i = 0; i < 36; ++i) {
      msg.pose.covariance[i] = 0.0;
    }
    msg.pose.covariance[0] = cov_pos;
    msg.pose.covariance[7] = cov_pos;
    msg.pose.covariance[14] = std::max(0.04, cov_pos);
    msg.pose.covariance[21] = 0.05;
    msg.pose.covariance[28] = 0.05;
    msg.pose.covariance[35] = cov_yaw;
    msg.success = success;
    msg.converged = converged;
    msg.map_id = active_map_id_;
    msg.reject_reason = reject_reason;
    msg.fitness_score = fitness;
    msg.inlier_ratio = metrics.inlier_ratio;
    msg.inlier_rmse = metrics.inlier_rmse;
    msg.inlier_count = metrics.inlier_count;
    msg.source_points = metrics.total_points;
    msg.target_points = target_points;
    msg.elapsed_ms = elapsed_ms;
    msg.initial_translation_error = delta_translation;
    msg.initial_yaw_error = delta_yaw;
    result_pub_.publish(msg);

    geometry_msgs::PoseStamped path_pose;
    path_pose.header = msg.header;
    path_pose.pose = matrixToPose(path_map_base);
    path_.header.stamp = stamp;
    path_.header.frame_id = map_frame_;
    path_.poses.push_back(path_pose);
    if (static_cast<int>(path_.poses.size()) > max_path_size_) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + (path_.poses.size() - max_path_size_));
    }
    path_pub_.publish(path_);
  }

  void publishRejected(const ros::Time& stamp, const std::string& reason) {
    MatchMetrics empty;
    publishResult(stamp.isZero() ? ros::Time::now() : stamp, false, false,
                  Eigen::Matrix4f::Identity(), base_frame_, Eigen::Matrix4f::Identity(),
                  std::numeric_limits<double>::infinity(), empty, 0.0, 0, 0.0, 0.0, reason);
    publishStatus(deeplumin_msgs::LocalizationStatus::LEVEL_WARNING, 0.0, reason);
  }

  void publishStatus(uint8_t level, double quality, const std::string& reason) const {
    deeplumin_msgs::LocalizationStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    msg.level = level;
    msg.failure_reason = reason;
    msg.quality_score = quality;
    msg.is_gnss_available = false;
    msg.is_slam_available = true;
    msg.is_dr_available = false;
    msg.current_mode = (viz_test_mode_ ? "GLOBAL_MATCHER_VIZ_TEST" : "GLOBAL_MATCHER") +
                       std::string(":") + active_map_id_ + ":" + current_mode_;
    status_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber source_sub_;
  ros::Subscriber tracking_initial_sub_;
  ros::Subscriber source_odom_sub_;
  ros::Subscriber bootstrap_initial_pose_sub_;
  ros::Subscriber manual_initial_pose_sub_;
  ros::Subscriber load_map_command_sub_;
  ros::Subscriber set_map_segment_command_sub_;
  ros::Publisher result_pub_;
  ros::Publisher status_pub_;
  ros::Publisher source_initial_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher submap_pub_;
  ros::Publisher global_map_pub_;
  ros::Publisher initial_guess_pub_;
  ros::Publisher path_pub_;
  ros::Timer match_timer_;
  ros::Timer map_timer_;

  mutable std::mutex mutex_;
  std::map<std::string, MapSegment> map_segments_;
  CloudPtr global_map_;
  CloudPtr global_map_viz_;
  CloudPtr latest_source_;
  ros::Time latest_source_stamp_;
  ros::Time latest_source_receive_stamp_;
  std::string latest_source_frame_;
  Eigen::Matrix4f latest_tracking_map_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f latest_bootstrap_map_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f latest_source_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f default_initial_map_base_ = Eigen::Matrix4f::Identity();
  ros::Time latest_tracking_stamp_;
  ros::Time latest_bootstrap_stamp_;
  ros::Time latest_source_odom_stamp_;
  bool have_tracking_map_base_ = false;
  bool have_bootstrap_map_base_ = false;
  bool have_source_odom_ = false;
  // 上一次成功匹配的 map->source（source 在 odom 系时即 map->odom，对应 ysw_loc 的 T_map_to_odom），
  // 作为下一帧初值：串行、无消息回路延迟。
  bool have_last_map_source_ = false;
  Eigen::Matrix4f last_map_source_ = Eigen::Matrix4f::Identity();
  uint64_t manual_reset_seq_ = 0;  // 人工初值序号：匹配期间有新的人工初值则丢弃本帧旧结果
  nav_msgs::Path path_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string source_cloud_topic_;
  std::string initial_pose_topic_;
  std::string source_odom_topic_;
  std::string bootstrap_initial_pose_topic_;
  std::string manual_initial_pose_topic_;
  std::string result_topic_;
  std::string status_topic_;
  std::string source_initial_scan_topic_;
  std::string aligned_scan_topic_;
  std::string target_submap_topic_;
  std::string global_map_topic_;
  std::string initial_guess_topic_;
  std::string path_topic_;
  std::string pcd_path_;
  std::string default_map_id_;
  std::string active_map_id_;
  std::string load_map_command_topic_;
  std::string set_map_segment_command_topic_;
  std::string matcher_type_;
  std::string source_cloud_frame_ = "auto";

  bool auto_match_ = true;
  bool viz_test_mode_ = false;
  bool source_cloud_is_odom_frame_ = true;
  bool use_receive_time_for_cloud_age_ = true;
  bool use_tracking_initial_pose_ = true;
  bool use_bootstrap_initial_pose_ = true;
  bool use_manual_initial_pose_ = true;
  bool use_default_initial_pose_ = true;
  bool allow_identity_bootstrap_ = true;
  Eigen::Matrix4f base_to_lidar_ = Eigen::Matrix4f::Identity();
  double match_rate_ = 1.0;
  double max_cloud_age_ = 1.0;
  int max_path_size_ = 3000;
  int consecutive_failures_ = 0;
  std::string current_mode_ = "bootstrap";
  double map_load_voxel_leaf_ = 0.4;
  double global_map_publish_voxel_leaf_ = 5.0;
  double global_map_publish_period_ = 5.0;
  double crop_forward_ = 70.0;
  double crop_backward_ = 7.0;
  double crop_left_ = 35.0;
  double crop_right_ = 35.0;
  double crop_down_ = 10.0;
  double crop_up_ = 10.0;
  double source_voxel_leaf_ = 0.1;
  double target_voxel_leaf_ = 0.1;
  double coarse_scale_ = 6.0;
  double fine_scale_ = 3.0;
  int max_iterations_ = 20;
  double transformation_epsilon_ = 0.01;
  double max_correspondence_distance_ = 2.0;
  int fast_gicp_num_threads_ = 2;
  int fast_gicp_correspondence_randomness_ = 20;

  // [vgicp]
  double vgicp_voxel_resolution_ = 1.0;
  int vgicp_num_neighbors_ = 20;
  int vgicp_num_threads_ = 4;
  small_gicp::GaussianVoxelMap::Ptr vgicp_coarse_voxelmap_;  ///< 当前激活地图的 VGICP 粗体素地图
  small_gicp::GaussianVoxelMap::Ptr vgicp_fine_voxelmap_;    ///< 当前激活地图的 VGICP 细体素地图

  int min_source_points_ = 200;
  int min_target_points_ = 1000;
  double max_fitness_score_ = 1.0;
  double min_inlier_ratio_ = 0.5;
  double max_inlier_rmse_ = 0.8;
  double inlier_distance_ = 0.5;
  double max_delta_translation_ = 5.0;
  double max_delta_yaw_deg_ = 20.0;
  std::string result_pose_frame_ = "base";
};

int main(int argc, char** argv) {
#ifdef GLOBAL_MATCHER_VIZ_TEST
  ros::init(argc, argv, "global_matcher_viz_test");
#else
  ros::init(argc, argv, "global_matcher");
#endif
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GlobalMatcherNode node(nh, pnh);
  ros::spin();
  return 0;
}
