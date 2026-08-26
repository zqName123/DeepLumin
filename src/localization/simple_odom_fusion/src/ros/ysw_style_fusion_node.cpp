/**
 * @file ysw_style_fusion_node.cpp
 * @brief YSW风格里程计融合节点 —— 将SLAM局部里程计与全局定位结果融合，输出map->base_link位姿
 *
 * @details
 * 本节点实现了一种典型的"YSW风格"（类似cartographer风格）的坐标系融合策略：
 * - 接收SLAM算法输出的 odom->base_link 局部里程计（高频、平滑但会漂移）
 * - 接收全局定位模块（global matcher）输出的 map->base_link 绝对位姿（低频、有噪声但不漂移）
 * - 通过计算 map->odom = map->base_link ⊕ (odom->base_link)⁻¹ 得到坐标系修正量
 * - 最终输出 map->base_link 融合位姿，兼顾平滑性和全局一致性
 *
 * 坐标系关系：
 *   map (全局地图坐标系)
 *    └── odom (SLAM局部坐标系，通过map->odom修正)
 *         └── base_link (机器人本体坐标系)
 */

#include "simple_odom_fusion/core/math_utils.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

namespace sof = simple_odom_fusion;

namespace {

constexpr double kPi = 3.14159265358979323846;

/**
 * @brief 将ROS的geometry_msgs::Pose转换为内部Pose3d结构
 * @param msg ROS Pose消息
 * @return 内部Pose3d表示，包含位置和四元数朝向
 *
 * @details
 * - 提取位置xyz
 * - 提取朝向四元数（w,x,y,z顺序）
 * - 对四元数进行有效性检查：若包含NaN/Inf，则重置为单位四元数
 * - 最后对四元数归一化，确保旋转矩阵的正交性
 */
sof::Pose3d poseFromRos(const geometry_msgs::Pose& msg) {
  sof::Pose3d pose;
  pose.position = sof::Vec3d(msg.position.x, msg.position.y, msg.position.z);
  pose.orientation = sof::Quatd(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if (!sof::isFinite(pose.orientation)) {
    pose.orientation = sof::Quatd::Identity();
  } else {
    pose.orientation.normalize();
  }
  return pose;
}

/**
 * @brief 计算两个位姿之间绕 z 轴的偏航角差（度，取最小角差）
 */
double yawDiffDeg(const sof::Pose3d& a, const sof::Pose3d& b) {
  const sof::Quatd rel = a.orientation.normalized().conjugate() * b.orientation.normalized();
  const Eigen::Matrix3d R = rel.toRotationMatrix();
  double yaw = std::atan2(R(1, 0), R(0, 0));
  yaw = std::fmod(yaw + kPi, 2.0 * kPi);
  if (yaw < 0.0) {
    yaw += 2.0 * kPi;
  }
  yaw -= kPi;
  return yaw * 180.0 / kPi;
}

/**
 * @brief 将内部Pose3d结构填充到ROS的geometry_msgs::Pose中
 * @param pose 内部Pose3d表示
 * @param msg 输出ROS Pose消息的指针
 *
 * @details
 * - 将位置xyz写入msg
 * - 将归一化后的四元数写入msg（x,y,z,w顺序）
 */
void fillPose(const sof::Pose3d& pose, geometry_msgs::Pose* msg) {
  msg->position.x = pose.position.x();
  msg->position.y = pose.position.y();
  msg->position.z = pose.position.z();
  const auto q = pose.orientation.normalized();
  msg->orientation.x = q.x();
  msg->orientation.y = q.y();
  msg->orientation.z = q.z();
  msg->orientation.w = q.w();
}

/**
 * @brief 将Pose3d转换为4x4齐次变换矩阵（float精度）
 * @param pose 输入位姿
 * @return 4x4齐次变换矩阵，用于PCL点云变换等操作
 *
 * @details
 * 矩阵结构：
 *   [ R(3x3)  t(3x1) ]
 *   [ 0 0 0      1    ]
 * 其中R为旋转矩阵，t为平移向量
 */
Eigen::Matrix4f toMatrix4f(const sof::Pose3d& pose) {
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = pose.orientation.normalized().toRotationMatrix().cast<float>();
  mat.block<3, 1>(0, 3) = pose.position.cast<float>();
  return mat;
}

/**
 * @brief 将Pose3d转换为ROS的TransformStamped消息
 * @param pose 输入位姿
 * @param parent 父坐标系名称（如"map"）
 * @param child 子坐标系名称（如"odom"）
 * @param stamp 时间戳
 * @return ROS TransformStamped消息，可用于TF广播
 */
geometry_msgs::TransformStamped toTf(const sof::Pose3d& pose,
                                     const std::string& parent,
                                     const std::string& child,
                                     const ros::Time& stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.transform.translation.x = pose.position.x();
  msg.transform.translation.y = pose.position.y();
  msg.transform.translation.z = pose.position.z();
  const auto q = pose.orientation.normalized();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

/**
 * @brief 从ROS参数服务器加载布尔值参数
 * @param nh NodeHandle
 * @param key 参数键名
 * @param fallback 默认值（当参数不存在时使用）
 * @return 加载的布尔值
 */
bool loadBool(ros::NodeHandle& nh, const std::string& key, bool fallback) {
  bool value = fallback;
  nh.param<bool>(key, value, fallback);
  return value;
}

}  // namespace

/**
 * @class YswStyleFusionNode
 * @brief YSW风格里程计融合节点核心类
 *
 * @details
 * 核心融合逻辑：
 * 1. 持续接收SLAM的odom->base_link位姿（odom_base_）
 * 2. 间歇性接收全局定位的map->base_link位姿
 * 3. 计算map->odom修正量：map_odom_ = map_base ⊕ (odom_base)⁻¹
 * 4. 输出融合位姿：map_base = map_odom_ ⊕ odom_base_
 *
 * 这种设计的优势：
 * - SLAM里程计高频平滑，保证局部运动连续性
 * - 全局定位修正map->odom，消除长期漂移
 * - 即使全局定位暂时失效，SLAM仍可继续输出相对位姿
 */
class YswStyleFusionNode {
 public:
  /**
   * @brief 构造函数：初始化ROS通信、加载参数、创建定时器
   * @param nh 公共命名空间NodeHandle
   * @param pnh 私有命名空间NodeHandle（~）
   */
  YswStyleFusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadConfig();

    // ========== 订阅器初始化 ==========
    // 订阅SLAM局部里程计（高频输入，核心数据源）
    slam_sub_ = nh_.subscribe(slam_odom_topic_, 100, &YswStyleFusionNode::onSlamOdom, this);
    // 订阅全局定位结果（低频输入，用于修正漂移）
    gm_sub_ = nh_.subscribe(global_matcher_result_topic_, 1, &YswStyleFusionNode::onGlobalMatch, this);
    // 订阅初始位姿（用于人工重置机器人位置）
    initial_pose_sub_ = nh_.subscribe(initial_pose_topic_, 5, &YswStyleFusionNode::onInitialPose, this);
    // 订阅源点云（用于转发给全局定位模块）
    cloud_sub_ = nh_.subscribe(source_cloud_topic_, 5, &YswStyleFusionNode::onSourceCloud, this);

    // ========== 发布器初始化 ==========
    // 发布融合后的map->base_link里程计（主输出）
    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(fused_odom_topic_, 50);
    // 发布map->odom修正量（用于观察坐标系偏移）
    map_to_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(map_to_odom_topic_, 10);
    // 发布融合后的位姿（带协方差，用于其他模块消费）
    fused_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(fused_pose_topic_, 20);
    // 发布历史轨迹（用于RViz可视化）
    path_pub_ = nh_.advertise<nav_msgs::Path>(fused_path_topic_, 5);
    // 发布当前帧点云（经融合后的map->odom变换到map系，供可视化核对匹配精度）
    current_scan_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(current_scan_in_map_topic_, 2);

    // 初始化轨迹消息的坐标系
    path_.header.frame_id = map_frame_;

    // 全局地图可视化发布（由本节点负责，frame_id=map）
    setupGlobalMapPublisher();

    // ========== 定时器初始化 ==========
    // 创建固定频率的输出定时器，确保即使SLAM数据有波动也能稳定输出
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, output_rate_hz_)),
                             &YswStyleFusionNode::onTimer, this);

    // 打印启动信息，便于调试确认参数加载正确
    ROS_INFO("[ysw_style_fusion] slam=%s gm=%s fused=%s map_to_odom=%s scan_in_map=%s",
             slam_odom_topic_.c_str(), global_matcher_result_topic_.c_str(), fused_odom_topic_.c_str(),
             map_to_odom_topic_.c_str(), current_scan_in_map_topic_.c_str());
  }

 private:
  /**
   * @brief 从ROS参数服务器加载所有配置参数
   *
   * @details
   * 参数结构分为以下几类：
   * - frames/*: TF坐标系名称配置
   * - topics/*: ROS话题名称配置
   * - runtime/*: 运行时行为配置（输出频率、TF发布开关等）
   * - global_matcher/*: 全局定位相关配置
   * - initialization/*: 初始化相关配置
   */
  void loadConfig() {
    // --- 坐标系名称配置 ---
    pnh_.param<std::string>("frames/map", map_frame_, map_frame_);
    pnh_.param<std::string>("frames/odom", odom_frame_, odom_frame_);
    pnh_.param<std::string>("frames/base_link", base_frame_, base_frame_);

    // --- 话题名称配置 ---
    pnh_.param<std::string>("topics/slam_odom", slam_odom_topic_, slam_odom_topic_);
    pnh_.param<std::string>("topics/global_matcher_result", global_matcher_result_topic_,
                            global_matcher_result_topic_);
    pnh_.param<std::string>("topics/initial_pose", initial_pose_topic_, initial_pose_topic_);
    pnh_.param<std::string>("topics/source_cloud", source_cloud_topic_, source_cloud_topic_);
    pnh_.param<std::string>("topics/fused_odom", fused_odom_topic_, fused_odom_topic_);
    pnh_.param<std::string>("topics/map_to_odom", map_to_odom_topic_, map_to_odom_topic_);
    pnh_.param<std::string>("topics/fused_pose", fused_pose_topic_, fused_pose_topic_);
    pnh_.param<std::string>("topics/fused_path", fused_path_topic_, fused_path_topic_);
    pnh_.param<std::string>("topics/current_scan_in_map", current_scan_in_map_topic_,
                            current_scan_in_map_topic_);

    // --- 运行时参数配置 ---
    pnh_.param<double>("runtime/output_rate_hz", output_rate_hz_, output_rate_hz_);
    pnh_.param<int>("runtime/max_path_size", max_path_size_, max_path_size_);
    publish_tf_ = loadBool(pnh_, "runtime/publish_tf", publish_tf_);
    publish_map_to_odom_tf_ = loadBool(pnh_, "runtime/publish_map_to_odom_tf", publish_map_to_odom_tf_);
    publish_odom_to_base_tf_ = loadBool(pnh_, "runtime/publish_odom_to_base_tf", publish_odom_to_base_tf_);
    publish_path_ = loadBool(pnh_, "runtime/publish_path", publish_path_);

    // --- 全局定位模块配置 ---
    publish_current_scan_in_map_ =
        loadBool(pnh_, "global_matcher/publish_current_scan_in_map", publish_current_scan_in_map_);
    enable_jump_guard_ = loadBool(pnh_, "global_matcher/enable_jump_guard", enable_jump_guard_);
    pnh_.param<double>("global_matcher/max_jump_translation", max_jump_translation_,
                       max_jump_translation_);
    pnh_.param<double>("global_matcher/max_jump_yaw_deg", max_jump_yaw_deg_, max_jump_yaw_deg_);

    // --- 初始化配置 ---
    allow_initialpose_ = loadBool(pnh_, "initialization/allow_initialpose", allow_initialpose_);
    pnh_.param<double>("initialization/init_pose_z_search_radius", init_pose_z_search_radius_,
                       init_pose_z_search_radius_);
  }

  /**
   * @brief SLAM里程计回调函数（高频触发）
   * @param msg SLAM输出的odom->base_link里程计消息
   *
   * @details
   * 处理流程：
   * 1. 提取并保存odom->base_link位姿到odom_base_
   * 3. 保存速度信息（twist），用于输出中保留速度数据
   * 4. 标记have_slam_为true，表示已收到SLAM数据
   * 5. 触发当前点云在map系的发布（若配置了）
   * 6. 触发融合输出发布
   */
  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_base_ = poseFromRos(msg->pose.pose);
    latest_twist_ = msg->twist;
    have_slam_ = true;
    publishCurrentScanInMap();
    publishOutput();
  }

  /**
   * @brief 全局定位结果回调函数（低频触发）
   * @param msg 全局定位模块输出的map->odom位姿
   *
   * @details
   * 核心修正逻辑：
   * 1. 检查全局定位是否成功，失败则忽略
   * 2. 确保已收到SLAM数据（否则无法计算map->odom）
   * 3. 提取map->base_link位姿
   * 4. 计算map->odom修正量：map_odom_ = map_base ⊕ (odom_base)⁻¹
   * 5. 发布map->odom变换
   *
   * 这个修正量的更新频率通常远低于SLAM里程计（如1Hz vs 10-100Hz），
   * 但足以消除长期累积的漂移。
   */
  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    // 全局定位失败时忽略该结果（可能由于特征不足或匹配分数过低）
    if (!msg->success) {
      ROS_WARN_THROTTLE(2.0, "[ysw_style_fusion] ignore rejected global matcher result: %s",
                        msg->reject_reason.c_str());
      return;
    }
    // 若SLAM数据尚未到达，无法计算修正量，等待下次全局定位结果
    if (!have_slam_) {
      ROS_WARN_THROTTLE(2.0, "[ysw_style_fusion] global matcher result arrived before slam odom");
      return;
    }

    // 提取全局定位位姿
    const sof::Pose3d candidate = poseFromRos(msg->pose.pose);

    // 跳变门限（可选，默认关闭）：仅当 global_matcher/enable_jump_guard=true 时生效。
    // 默认关闭即完全 ysw_loc 行为：GM 报 success 就生效，未收敛保持上一帧是唯一防线。
    // 开启时，候选结果相对上一帧 map->odom 跳变过大则拒绝并保留上一帧（人工 initial pose 不受限）。
    if (enable_jump_guard_) {
      const double delta_trans = (candidate.position - map_odom_.position).norm();
      const double delta_yaw_deg = yawDiffDeg(map_odom_, candidate);
      if (delta_trans > max_jump_translation_ || delta_yaw_deg > max_jump_yaw_deg_) {
        ROS_WARN_THROTTLE(1.0,
                          "[ysw_style_fusion] reject global matcher result: jump too large "
                          "trans=%.3fm yaw=%.2fdeg (limit %.3fm/%.2fdeg), keep previous map->odom",
                          delta_trans, delta_yaw_deg, max_jump_translation_, max_jump_yaw_deg_);
        return;
      }
    }

    // 通过门限，更新 map->odom 修正量
    map_odom_ = candidate;
    have_map_odom_ = true;
    publishMapToOdom();
    ROS_INFO("[ysw_style_fusion] updated map->odom from global matcher map_id=%s child=%s fitness=%.4f inlier=%.3f",
             msg->map_id.c_str(), msg->child_frame_id.c_str(), msg->fitness_score, msg->inlier_ratio);
  }

  /**
   * @brief 初始位姿回调函数（人工重置机器人位置）
   * @param msg RViz 2D Pose Estimate工具发布的初始位姿
   *
   * @details
   * 处理流程：
   * 1. 检查是否允许通过initial_pose重置（安全开关）
   * 2. 提取map->base_link目标位姿
   * 3. 计算新的map->odom修正量：
   *    - 若已有SLAM数据：map_odom_ = map_base ⊕ (odom_base)⁻¹
   *    - 若尚无SLAM数据：直接将map_odom_设为map_base（此时odom_base_为单位变换）
   * 4. 发布更新后的map->odom变换
   *
   * 典型使用场景：
   * - 机器人在地图中的初始位置未知时
   * - SLAM发生严重漂移需要人工纠正时
   * - 重定位（relocalization）后确认正确位置
   */
  /**
   * @brief 计算人工初始位姿的z高度（与ysw_loc一致）
   * @param position 初始位姿在map系的x/y位置
   * @param z_out 输出z高度（取位姿附近地图点云的z范围中值）
   * @return 是否成功（需要已加载全局地图且搜索半径内有地图点）
   */
  bool computeInitPoseZ(const sof::Vec3d& position, double* z_out) const {
    if (!global_map_cloud_ || global_map_cloud_->empty()) {
      return false;
    }
    const double r2 = init_pose_z_search_radius_ * init_pose_z_search_radius_;
    const double x = position.x();
    const double y = position.y();
    bool found = false;
    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();
    for (const auto& p : global_map_cloud_->points) {
      const double dx = p.x - x;
      const double dy = p.y - y;
      if (dx * dx + dy * dy <= r2) {
        found = true;
        z_min = std::min(z_min, static_cast<double>(p.z));
        z_max = std::max(z_max, static_cast<double>(p.z));
      }
    }
    if (!found) {
      return false;
    }
    *z_out = 0.5 * (z_min + z_max);
    return true;
  }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!allow_initialpose_) {
      return;
    }
    const sof::Pose3d pose = poseFromRos(msg->pose.pose);
    // 与ysw_loc对齐：RViz 2D Pose Estimate 的位姿位于RViz固定坐标系（map）下。
    // 为兼容固定坐标系被设为odom/base_link的情况，按消息header.frame_id统一转换到map系，
    // 避免把odom系坐标误当map系使用导致map->odom跳变（原点漂移）。
    sof::Pose3d map_base = pose;
    const std::string pose_frame = msg->header.frame_id;
    if (!pose_frame.empty() && pose_frame != map_frame_) {
      if (pose_frame == odom_frame_) {
        // odom系位姿：map_base = map_odom ⊕ pose(odom->base)
        map_base = sof::compose(map_odom_, pose);
      } else if (pose_frame == base_frame_) {
        // base系位姿：位置即当前机器人位置
        map_base = sof::compose(map_odom_, odom_base_);
      } else {
        ROS_WARN_THROTTLE(2.0, "[ysw_style_fusion] unknown initial pose frame '%s', assume %s",
                          pose_frame.c_str(), map_frame_.c_str());
      }
    }
    // 与ysw_loc一致：初始位姿的z取人工位姿附近地图点云高度的中值 (max_z+min_z)/2
    double init_z = 0.0;
    if (computeInitPoseZ(map_base.position, &init_z)) {
      map_base.position.z() = init_z;
    }
    map_odom_ = have_slam_ ? sof::compose(map_base, sof::inverse(odom_base_)) : map_base;
    // 人工初值不受跳变门限限制，并作为后续 GM 结果的参考基准
    have_map_odom_ = true;
    publishMapToOdom();
    ROS_INFO("[ysw_style_fusion] reset map->odom from initial pose frame=%s z=%.3f", pose_frame.c_str(),
             map_base.position.z());
  }

  /**
   * @brief 源点云回调函数（缓存最新点云用于转发）
   * @param msg SLAM模块输出的源点云（通常在odom坐标系下）
   *
   * @details
   * 仅做缓存，不做实时处理。实际的坐标系变换和发布在publishCurrentScanInMap中完成。
   * 这种设计避免了在点云回调中进行耗时操作，保证SLAM数据处理的实时性。
   */
  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    latest_cloud_ = msg;
  }

  /**
   * @brief 定时器回调函数（固定频率触发）
   * @param event ROS定时器事件（未使用）
   *
   * @details
   * 定时器的作用：
   * 1. 作为SLAM回调的备份输出机制，确保输出频率稳定
   * 2. 当SLAM数据暂时中断时，持续发布最后一次已知位姿
   * 3. 在未收到SLAM数据时发布单位map->odom变换（便于下游模块初始化）
   *
   * 处理流程：
   * - 若尚未收到SLAM数据：发布单位map->odom，并警告等待SLAM
   * - 若已有SLAM数据：以当前 ROS 时间发布输出（避免TF被判定为过期数据）
   */
  void onTimer(const ros::TimerEvent&) {
    if (!have_slam_) {
      publishIdentityMapOdom();
      ROS_WARN_THROTTLE(2.0, "[ysw_style_fusion] waiting for slam odom: %s", slam_odom_topic_.c_str());
      return;
    }
    publishCurrentScanInMap();
    publishOutput();
  }

  /**
   * @brief 发布单位map->odom变换（初始化/等待状态使用）
   *
   * @details
   * 当SLAM数据尚未到达时，发布map->odom为单位变换。
   * 这保证了TF树的完整性，使下游模块可以正常启动而不因缺少TF而报错。
   * 一旦收到SLAM数据和全局定位结果，map->odom将被正确更新。
   */
  void publishIdentityMapOdom() {
    if (!publish_tf_ || !publish_map_to_odom_tf_) {
      return;
    }
    tf_broadcaster_.sendTransform(toTf(sof::Pose3d(), map_frame_, odom_frame_, ros::Time::now()));
  }

  /**
   * @brief 加载并发布全局地图（由本节点统一发布，frame_id=map）
   *
   * @details
   * 从PCD加载全局地图，做体素降采样后以latch方式发布到global_map_topic_，
   * 并定时重发，供RViz等订阅者显示。坐标系固定为map_frame_（map）。
   */
  void setupGlobalMapPublisher() {
    pnh_.param<bool>("map/publish_global_map", publish_global_map_, publish_global_map_);
    pnh_.param<std::string>("map/pcd_path", global_map_pcd_path_, global_map_pcd_path_);
    pnh_.param<std::string>("map/global_map_topic", global_map_topic_, global_map_topic_);
    pnh_.param<double>("map/publish_voxel_leaf", global_map_voxel_leaf_, global_map_voxel_leaf_);
    pnh_.param<double>("map/publish_period", global_map_publish_period_, global_map_publish_period_);

    if (!publish_global_map_) {
      return;
    }
    if (global_map_pcd_path_.empty()) {
      ROS_WARN("[ysw_style_fusion] map/publish_global_map is true but map/pcd_path is empty");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(global_map_pcd_path_, *raw) != 0 || raw->empty()) {
      ROS_ERROR("[ysw_style_fusion] failed to load global map: %s", global_map_pcd_path_.c_str());
      return;
    }

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*raw, *raw, valid_indices);

    global_map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    if (global_map_voxel_leaf_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setInputCloud(raw);
      const float leaf = static_cast<float>(global_map_voxel_leaf_);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*global_map_cloud_);
    } else {
      *global_map_cloud_ = *raw;
    }
    if (global_map_cloud_->empty()) {
      ROS_ERROR("[ysw_style_fusion] global map is empty after downsample: %s", global_map_pcd_path_.c_str());
      return;
    }

    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_, 1, true);
    publishGlobalMap();
    if (global_map_publish_period_ >= 0.5) {
      global_map_timer_ = nh_.createTimer(ros::Duration(global_map_publish_period_),
                                          &YswStyleFusionNode::onGlobalMapTimer, this);
    }
    ROS_INFO("[ysw_style_fusion] global map loaded path=%s raw_points=%zu viz_points=%zu topic=%s frame=%s voxel=%.3f period=%.3fs",
             global_map_pcd_path_.c_str(), raw->size(), global_map_cloud_->size(),
             global_map_topic_.c_str(), map_frame_.c_str(), global_map_voxel_leaf_,
             global_map_publish_period_);
  }

  /**
   * @brief 定时重发全局地图
   */
  void onGlobalMapTimer(const ros::TimerEvent&) { publishGlobalMap(); }

  /**
   * @brief 发布全局地图（frame_id=map，latch）
   */
  void publishGlobalMap() {
    if (!publish_global_map_ || !global_map_cloud_ || global_map_cloud_->empty() || !global_map_pub_) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*global_map_cloud_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    global_map_pub_.publish(msg);
  }

  /**
   * @brief 发布map->odom修正量作为Odometry消息
   *
   * @details
   * 将内部存储的map_odom_位姿发布为nav_msgs/Odometry格式。
   * 这主要用于：
   * - 其他节点订阅观察map->odom的当前值
   * - 记录bag时保存修正量历史
   * - 调试时分析修正量的变化趋势
   */
  void publishMapToOdom() {
    nav_msgs::Odometry odom;
    // 输出统一使用当前 ROS 时间：模拟时间下等于 bag 时间，
    // 墙钟模式下等于当前墙钟，避免把上游 measurement 时间戳转发出去
    // 导致 TF 消费者判定为过期数据（TF_OLD_DATA）。
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = odom_frame_;
    fillPose(map_odom_, &odom.pose.pose);
    map_to_odom_pub_.publish(odom);
  }

  /**
   * @brief 核心输出函数：发布融合后的位姿、轨迹、TF等
   *
   * @details
   * 融合公式：map_base = map_odom_ ⊕ odom_base_
   * 即：先应用map->odom修正，再应用odom->base_link局部里程计
   *
   * 发布内容：
   * 1. fused_odom: map->base_link里程计（含速度）
   * 2. fused_pose: map->base_link位姿（带协方差框架）
   * 3. map_to_odom: map->odom修正量
   * 4. path: 历史轨迹（可选，用于RViz可视化）
   * 5. TF变换: map->odom 和/或 odom->base_link（可选）
   */
  void publishOutput() {
    // 核心融合计算：map->base_link = map->odom ⊕ odom->base_link
    const sof::Pose3d map_base = sof::compose(map_odom_, odom_base_);

    // --- 发布融合里程计 ---
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = map_frame_;      // 父坐标系：map
    odom.child_frame_id = base_frame_;      // 子坐标系：base_link
    fillPose(map_base, &odom.pose.pose);
    odom.twist = latest_twist_;             // 保留SLAM的速度信息
    fused_odom_pub_.publish(odom);

    // --- 发布融合位姿（PoseWithCovarianceStamped格式） ---
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = odom.header;
    pose.pose.pose = odom.pose.pose;
    fused_pose_pub_.publish(pose);

    // --- 发布map->odom修正量 ---
    publishMapToOdom();

    // --- 发布历史轨迹（可选） ---
    if (publish_path_) {
      geometry_msgs::PoseStamped path_pose;
      path_pose.header = odom.header;
      path_pose.pose = odom.pose.pose;
      path_.header.stamp = odom.header.stamp;
      path_.poses.push_back(path_pose);
      // 轨迹长度限制：超过max_path_size_时移除最旧的点（滑动窗口）
      while (static_cast<int>(path_.poses.size()) > max_path_size_) {
        path_.poses.erase(path_.poses.begin());
      }
      path_pub_.publish(path_);
    }

    // --- 发布TF变换（可选） ---
    if (publish_tf_) {
      std::vector<geometry_msgs::TransformStamped> transforms;
      // map->odom变换（全局修正量）
      if (publish_map_to_odom_tf_) {
        transforms.push_back(toTf(map_odom_, map_frame_, odom_frame_, odom.header.stamp));
      }
      // odom->base_link变换（局部里程计）
      if (publish_odom_to_base_tf_) {
        transforms.push_back(toTf(odom_base_, odom_frame_, base_frame_, odom.header.stamp));
      }
      if (!transforms.empty()) {
        tf_broadcaster_.sendTransform(transforms);
      }
    }
  }

  /**
   * @brief 发布当前帧点云在全局地图（map系）中的位置（可视化/精度核对）
   *
   * @details
   * 功能说明：
   * - 接收SLAM输出的当前帧点云（odom系）
   * - 用融合后的map->odom（来自全局定位修正）变换到map系后发布
   * - 对应ysw_loc的/cur_scan_in_map，可在RViz中与全局地图叠加查看匹配精度
   *
   * 使用条件：
   * - publish_current_scan_in_map_为true
   * - 已缓存点云数据
   * - 已收到SLAM数据
   */
  void publishCurrentScanInMap() {
    if (!publish_current_scan_in_map_ || !latest_cloud_ || !have_slam_) {
      return;
    }

    // 将ROS点云消息转换为PCL格式（便于后续处理）
    pcl::PointCloud<pcl::PointXYZI> in;
    pcl::fromROSMsg(*latest_cloud_, in);
    if (in.empty()) {
      return;
    }

    // 用融合后的map->odom把当前帧点云变换到map系
    pcl::PointCloud<pcl::PointXYZI> out;
    pcl::transformPointCloud(in, out, toMatrix4f(map_odom_));

    // 转换回ROS消息并设置坐标系为map
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    // 输出统一使用当前 ROS 时间：模拟时间下等于 bag 时间，
    // 墙钟模式下等于当前墙钟，避免RViz消息过滤按旧测量时间戳丢弃。
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    current_scan_map_pub_.publish(msg);
  }

  // ========== ROS通信成员 ==========
  ros::NodeHandle nh_;           ///< 公共命名空间NodeHandle
  ros::NodeHandle pnh_;          ///< 私有命名空间NodeHandle（~）
  tf2_ros::TransformBroadcaster tf_broadcaster_;  ///< TF广播器

  // ========== 数据缓存 ==========
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;  ///< 最新接收的源点云
  nav_msgs::Path path_;                              ///< 历史轨迹
  geometry_msgs::TwistWithCovariance latest_twist_;  ///< 最新速度信息
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_cloud_;  ///< 降采样后的全局地图点云

  // ========== 坐标系与话题名称（带默认值） ==========
  std::string map_frame_ = "map";                                    ///< 全局地图坐标系
  std::string odom_frame_ = "odom";                                  ///< SLAM局部坐标系
  std::string base_frame_ = "base_link";                             ///< 机器人本体坐标系
  std::string slam_odom_topic_ = "/localization/slam_odom";          ///< SLAM里程计话题
  std::string global_matcher_result_topic_ = "/localization/global_matcher/result";  ///< 全局定位结果话题
  std::string initial_pose_topic_ = "/initialpose";                  ///< 初始位姿话题
  std::string source_cloud_topic_ = "/localization/cloud_registered"; ///< 源点云话题
  std::string fused_odom_topic_ = "/localization/ysw_style_odom";    ///< 融合里程计输出话题
  std::string map_to_odom_topic_ = "/localization/ysw_style_map_to_odom";  ///< map->odom输出话题
  std::string fused_pose_topic_ = "/localization/ysw_style_pose";    ///< 融合位姿输出话题
  std::string fused_path_topic_ = "/localization/ysw_style_path";    ///< 融合轨迹输出话题
  std::string current_scan_in_map_topic_ = "/localization/ysw_style/current_scan_in_map";  ///< 当前点云在map系输出话题
  std::string global_map_topic_ = "/localization/ysw_style/global_map";                    ///< 全局地图可视化话题
  std::string global_map_pcd_path_ = "/home/hhy/2004ros/relocalization_ws/global_map_manual_imu.pcd";  ///< 全局地图PCD路径

  // ========== 运行时参数 ==========
  double output_rate_hz_ = 100.0;     ///< 定时器输出频率（Hz）
  int max_path_size_ = 50000;          ///< 轨迹最大保存点数（滑动窗口）
  bool publish_tf_ = true;            ///< 是否发布TF变换
  bool publish_map_to_odom_tf_ = true;   ///< 是否发布map->odom的TF
  bool publish_odom_to_base_tf_ = true;  ///< 是否发布odom->base_link的TF
  bool publish_path_ = true;          ///< 是否发布历史轨迹
  bool publish_current_scan_in_map_ = true;  ///< 是否发布当前点云在map系的可视化
  bool publish_global_map_ = true;           ///< 是否发布全局地图可视化
  double init_pose_z_search_radius_ = 5.0;   ///< 初始位姿z高度搜索半径（米）
  double global_map_voxel_leaf_ = 0.0;       ///< 全局地图体素降采样大小（米），0=全分辨率（与ysw_loc可视化一致）
  double global_map_publish_period_ = 0.0;   ///< 全局地图重发周期（秒），0=仅latch发布一次
  bool allow_initialpose_ = true;     ///< 是否允许通过initial_pose重置位姿
  bool enable_jump_guard_ = false;    ///< 是否启用跳变门限（默认关=完全 ysw_loc 行为）
  double max_jump_translation_ = 2.0; ///< GM结果相对上一帧 map->odom 的最大平移跳变（米）
  double max_jump_yaw_deg_ = 10.0;    ///< GM结果相对上一帧 map->odom 的最大偏航跳变（度）

  // ========== 状态变量 ==========
  bool have_slam_ = false;            ///< 是否已收到SLAM数据
  bool have_map_odom_ = false;        ///< 是否已有可用的 map->odom（人工初值/已接受的 GM 结果）
  sof::Pose3d odom_base_;             ///< 当前odom->base_link位姿（来自SLAM）
  sof::Pose3d map_odom_;              ///< 当前map->odom修正量（来自全局定位或初始位姿）

  // ========== ROS通信句柄 ==========
  ros::Subscriber slam_sub_;          ///< SLAM里程计订阅器
  ros::Subscriber gm_sub_;            ///< 全局定位结果订阅器
  ros::Subscriber initial_pose_sub_;  ///< 初始位姿订阅器
  ros::Subscriber cloud_sub_;         ///< 源点云订阅器
  ros::Publisher fused_odom_pub_;     ///< 融合里程计发布器
  ros::Publisher map_to_odom_pub_;    ///< map->odom发布器
  ros::Publisher fused_pose_pub_;     ///< 融合位姿发布器
  ros::Publisher path_pub_;           ///< 轨迹发布器
  ros::Publisher current_scan_map_pub_; ///< 当前点云在map系发布器
  ros::Publisher global_map_pub_;        ///< 全局地图发布器（latch）
  ros::Timer global_map_timer_;          ///< 全局地图重发定时器
  ros::Timer timer_;                  ///< 固定频率输出定时器
};

/**
 * @brief 主函数：初始化ROS节点并进入spin循环
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "ysw_style_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  YswStyleFusionNode node(nh, pnh);
  ros::spin();
  return 0;
}
