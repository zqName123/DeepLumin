/**
 * @file simple_odom_fusion_node.cpp
 * @brief 简单里程计融合节点 —— 融合DR（航位推算）里程计与SLAM里程计，结合全局定位修正
 *
 * @details
 * 本节点是一个多源里程计融合系统，相比ysw_style_fusion_node更加复杂，核心特点：
 * - 接收DR（Dead Reckoning，航位推算）里程计：高频、短时精度高但长期漂移大
 * - 接收SLAM里程计：中频、提供局部一致性位姿
 * - 接收全局定位结果：低频、消除长期漂移
 * - 通过SimpleFusionCore进行时间同步、增量计算、融合滤波等核心算法
 * - 输出融合后的map->base_link位姿
 *
 * 坐标系关系：
 *   map (全局地图坐标系)
 *    └── odom (融合后的局部坐标系)
 *         └── base_link (机器人本体坐标系)
 *
 * 与YswStyleFusionNode的区别：
 * - 支持DR+SLAM双里程计输入（而非仅SLAM）
 * - 使用SimpleFusionCore处理复杂的同步和融合逻辑
 * - 支持增量模式（use_slam_increment）
 * - 支持多种初始化方式（全局定位、局部自动、初始位姿）
 */

#include "simple_odom_fusion/core/simple_fusion_core.hpp"

#include <deque>
#include <string>
#include <vector>

#include <deeplumin_msgs/GlobalMatchResult.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

namespace sof = simple_odom_fusion;

namespace {

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
 * @class SimpleOdomFusionNode
 * @brief 简单里程计融合节点核心类
 *
 * @details
 * 本节点是SimpleFusionCore的ROS封装层，负责：
 * 1. ROS通信管理（订阅/发布/TF/定时器）
 * 2. 参数加载与配置
 * 3. ROS消息与内部数据结构的转换
 * 4. 调用SimpleFusionCore进行核心融合计算
 * 5. 发布融合结果
 *
 * 核心融合逻辑在SimpleFusionCore中实现，本节点主要做数据桥接。
 * SimpleFusionCore的处理流程：
 * - 接收DR和SLAM里程计，进行时间同步配对
 * - 计算DR增量和SLAM增量
 * - 根据配置选择融合策略（直接融合或增量融合）
 * - 应用全局定位修正
 * - 输出融合后的map->base_link位姿
 */
class SimpleOdomFusionNode {
 public:
  /**
   * @brief 构造函数：初始化ROS通信、加载参数、创建融合核心
   * @param nh 公共命名空间NodeHandle
   * @param pnh 私有命名空间NodeHandle（~）
   */
  SimpleOdomFusionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadConfig();
    core_.initialize(config_);

    // ========== 订阅器初始化 ==========
    // DR（航位推算）里程计订阅：高频输入，通常来自轮速计或IMU积分
    dr_sub_ = nh_.subscribe(config_.topics.dr_odom, 200, &SimpleOdomFusionNode::onDrOdom, this);
    // SLAM里程计订阅：中频输入，提供局部一致的位姿估计
    slam_sub_ = nh_.subscribe(config_.topics.slam_odom, 100, &SimpleOdomFusionNode::onSlamOdom, this);
    // 全局定位结果订阅：低频输入，用于消除长期漂移
    gm_sub_ = nh_.subscribe(config_.topics.global_matcher_result, 20, &SimpleOdomFusionNode::onGlobalMatch, this);
    // 初始位姿订阅：用于人工重置机器人位置
    initial_pose_sub_ = nh_.subscribe(config_.topics.initial_pose, 5, &SimpleOdomFusionNode::onInitialPose, this);
    // 源点云订阅：用于转发给全局定位模块
    cloud_sub_ = nh_.subscribe(config_.topics.source_cloud, 5, &SimpleOdomFusionNode::onSourceCloud, this);

    // ========== 发布器初始化 ==========
    // 融合后的map->base_link里程计（主输出）
    fused_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.topics.fused_odom, 50);
    // 融合后的位姿（带协方差框架）
    fused_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(config_.topics.fused_pose, 20);
    // 历史轨迹（用于RViz可视化）
    path_pub_ = nh_.advertise<nav_msgs::Path>(config_.topics.fused_path, 5);
    // 转换后的源点云（供全局定位模块使用）
    gm_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(config_.topics.gm_source_cloud_odom, 2);

    // 初始化轨迹消息的坐标系
    path_.header.frame_id = config_.frames.map;

    // ========== 定时器初始化 ==========
    // 固定频率输出定时器，驱动SimpleFusionCore的update循环
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.output_rate_hz)),
                             &SimpleOdomFusionNode::onTimer, this);

    // 打印启动信息，便于调试确认参数加载正确
    ROS_INFO("[simple_odom_fusion] dr=%s slam=%s gm=%s initial=%s out=%s gm_cloud=%s",
             config_.topics.dr_odom.c_str(), config_.topics.slam_odom.c_str(),
             config_.topics.global_matcher_result.c_str(), config_.topics.initial_pose.c_str(),
             config_.topics.fused_odom.c_str(), config_.topics.gm_source_cloud_odom.c_str());
  }

 private:
  /**
   * @brief 从ROS参数服务器加载所有配置参数到FusionConfig结构体
   *
   * @details
   * 参数结构分为以下几类：
   * - frames/*: TF坐标系名称配置
   * - topics/*: ROS话题名称配置
   * - runtime/*: 运行时行为配置（输出频率、轨迹长度、TF发布开关等）
   * - sync/*: 时间同步配置（队列大小、容差、最大配对时间差）
   * - fusion/*: 融合算法参数（最大位移/旋转阈值、预测权重、修正增益等）
   * - initialization/*: 初始化相关配置（多种初始化方式的开关）
   * - global_matcher/*: 全局定位相关配置（修正开关、增益、间隔、点云转发等）
   */
  void loadConfig() {
    // --- 坐标系名称配置 ---
    pnh_.param<std::string>("frames/map", config_.frames.map, config_.frames.map);
    pnh_.param<std::string>("frames/odom", config_.frames.odom, config_.frames.odom);
    pnh_.param<std::string>("frames/base_link", config_.frames.base_link, config_.frames.base_link);
    pnh_.param<std::string>("frames/lidar", config_.frames.lidar, config_.frames.lidar);

    // --- 话题名称配置 ---
    pnh_.param<std::string>("topics/dr_odom", config_.topics.dr_odom, config_.topics.dr_odom);
    pnh_.param<std::string>("topics/slam_odom", config_.topics.slam_odom, config_.topics.slam_odom);
    pnh_.param<std::string>("topics/global_matcher_result", config_.topics.global_matcher_result,
                            config_.topics.global_matcher_result);
    pnh_.param<std::string>("topics/initial_pose", config_.topics.initial_pose, config_.topics.initial_pose);
    pnh_.param<std::string>("topics/source_cloud", config_.topics.source_cloud, config_.topics.source_cloud);
    pnh_.param<std::string>("topics/fused_odom", config_.topics.fused_odom, config_.topics.fused_odom);
    pnh_.param<std::string>("topics/fused_pose", config_.topics.fused_pose, config_.topics.fused_pose);
    pnh_.param<std::string>("topics/fused_path", config_.topics.fused_path, config_.topics.fused_path);
    pnh_.param<std::string>("topics/gm_source_cloud_odom", config_.topics.gm_source_cloud_odom,
                            config_.topics.gm_source_cloud_odom);

    // --- 运行时参数配置 ---
    pnh_.param<double>("runtime/output_rate_hz", config_.output_rate_hz, config_.output_rate_hz);
    pnh_.param<int>("runtime/max_path_size", config_.max_path_size, config_.max_path_size);
    config_.publish_tf = loadBool(pnh_, "runtime/publish_tf", config_.publish_tf);
    config_.publish_map_to_odom_tf = loadBool(pnh_, "runtime/publish_map_to_odom_tf", config_.publish_map_to_odom_tf);
    config_.publish_odom_to_base_tf = loadBool(pnh_, "runtime/publish_odom_to_base_tf", config_.publish_odom_to_base_tf);
    config_.publish_path = loadBool(pnh_, "runtime/publish_path", config_.publish_path);

    // --- 时间同步参数配置 ---
    // max_queue_size: DR和SLAM里程计缓存队列的最大长度，用于时间同步配对
    int max_queue_size = static_cast<int>(config_.max_queue_size);
    pnh_.param<int>("sync/max_queue_size", max_queue_size, max_queue_size);
    config_.max_queue_size = static_cast<std::size_t>(std::max(10, max_queue_size));
    // sync_tolerance: 时间同步容差，在此时间窗口内寻找配对
    pnh_.param<double>("sync/tolerance", config_.sync_tolerance, config_.sync_tolerance);
    // max_pair_dt: DR和SLAM配对的最大允许时间差，超过则丢弃
    pnh_.param<double>("sync/max_pair_dt", config_.max_pair_dt, config_.max_pair_dt);

    // --- 融合算法参数配置 ---
    // max_delta_translation: 两次更新间最大允许位移，超过则视为异常
    pnh_.param<double>("fusion/max_delta_translation", config_.max_delta_translation, config_.max_delta_translation);
    // max_delta_rotation_rad: 两次更新间最大允许旋转（弧度），超过则视为异常
    pnh_.param<double>("fusion/max_delta_rotation_rad", config_.max_delta_rotation_rad, config_.max_delta_rotation_rad);
    // dr_prediction_weight: DR预测在融合中的权重（0-1）
    pnh_.param<double>("fusion/dr_prediction_weight", config_.dr_prediction_weight, config_.dr_prediction_weight);
    // slam_correction_gain: SLAM修正的增益系数
    pnh_.param<double>("fusion/slam_correction_gain", config_.slam_correction_gain, config_.slam_correction_gain);
    // use_slam_increment: 是否使用SLAM增量模式（而非绝对位姿）
    config_.use_slam_increment = loadBool(pnh_, "fusion/use_slam_increment", config_.use_slam_increment);

    // --- 初始化参数配置 ---
    config_.allow_initialpose = loadBool(pnh_, "initialization/allow_initialpose", config_.allow_initialpose);
    // allow_global_matcher_initialization: 允许通过全局定位结果初始化
    config_.allow_global_matcher_initialization =
        loadBool(pnh_, "initialization/allow_global_matcher", config_.allow_global_matcher_initialization);
    // allow_local_auto_initialization: 允许局部自动初始化（如静止检测）
    config_.allow_local_auto_initialization =
        loadBool(pnh_, "initialization/allow_local_auto", config_.allow_local_auto_initialization);

    // --- 全局定位模块参数配置 ---
    // enable_global_matcher_correction: 是否启用全局定位修正
    config_.enable_global_matcher_correction =
        loadBool(pnh_, "global_matcher/enable_correction", config_.enable_global_matcher_correction);
    // global_matcher_correction_gain: 全局定位修正的增益系数（0-1，越小越平滑）
    pnh_.param<double>("global_matcher/correction_gain", config_.global_matcher_correction_gain,
                       config_.global_matcher_correction_gain);
    // global_matcher_min_interval: 两次全局定位修正的最小时间间隔（防止过于频繁修正）
    pnh_.param<double>("global_matcher/min_interval", config_.global_matcher_min_interval,
                       config_.global_matcher_min_interval);
    // publish_gm_source_cloud: 是否转发点云给全局定位模块
    config_.publish_gm_source_cloud =
        loadBool(pnh_, "global_matcher/publish_source_cloud_odom", config_.publish_gm_source_cloud);
    // gm_cloud_use_fused_pose: 转发点云时是否使用融合后的位姿进行变换
    config_.gm_cloud_use_fused_pose =
        loadBool(pnh_, "global_matcher/cloud_use_fused_pose", config_.gm_cloud_use_fused_pose);
  }

  /**
   * @brief 将ROS Odometry消息转换为内部OdomSample结构
   * @param msg ROS里程计消息
   * @return 内部OdomSample结构，包含时间戳、位姿、速度、坐标系信息
   *
   * @details
   * 转换内容：
   * - 时间戳：msg.header.stamp（若为0则使用当前时间）
   * - 位姿：pose.pose → Pose3d
   * - 线速度：twist.twist.linear → Vec3d
   * - 坐标系：frame_id和child_frame_id
   */
  sof::OdomSample sampleFromOdom(const nav_msgs::Odometry& msg) const {
    sof::OdomSample sample;
    sample.stamp = msg.header.stamp.isZero() ? ros::Time::now().toSec() : msg.header.stamp.toSec();
    sample.pose = poseFromRos(msg.pose.pose);
    sample.linear_velocity = sof::Vec3d(msg.twist.twist.linear.x,
                                        msg.twist.twist.linear.y,
                                        msg.twist.twist.linear.z);
    sample.frame_id = msg.header.frame_id;
    sample.child_frame_id = msg.child_frame_id;
    return sample;
  }

  /**
   * @brief DR（航位推算）里程计回调函数（高频触发）
   * @param msg DR里程计消息（通常来自轮速计或IMU积分）
   *
   * @details
   * DR里程计特点：
   * - 频率高（通常100-500Hz）
   * - 短期精度高，但长期累积漂移严重
   * - 作为融合预测的主要输入
   *
   * 处理：将ROS消息转换为OdomSample，喂入SimpleFusionCore
   */
  void onDrOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedDr(sampleFromOdom(*msg));
  }

  /**
   * @brief SLAM里程计回调函数（中频触发）
   * @param msg SLAM里程计消息
   *
   * @details
   * SLAM里程计特点：
   * - 频率中等（通常10-50Hz）
   * - 提供局部一致的位姿（在odom坐标系下）
   * - 用于修正DR的累积误差
   *
   * 处理：将ROS消息转换为OdomSample，喂入SimpleFusionCore
   */
  void onSlamOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    core_.feedSlam(sampleFromOdom(*msg));
  }

  /**
   * @brief 全局定位结果回调函数（低频触发）
   * @param msg 全局定位模块输出的map->base_link位姿
   *
   * @details
   * 处理流程：
   * 1. 提取时间戳和位姿，构建GlobalObservation
   * 2. 若系统尚未初始化且允许全局定位初始化：
   *    - 使用全局定位结果直接初始化系统位姿
   *    - 这是冷启动时的主要初始化方式
   * 3. 若系统已初始化：
   *    - 将全局定位结果作为观测，调用correctWithGlobalMatcher进行修正
   *
   * 全局定位修正特点：
   * - 频率低（通常0.1-1Hz）
   * - 绝对精度高，但可能有跳变
   * - 通过增益系数控制修正平滑度
   */
  void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
    sof::GlobalObservation obs;
    obs.stamp = msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec();
    obs.map_base = poseFromRos(msg->pose.pose);
    obs.success = msg->success;
    if (!core_.initialized() && config_.allow_global_matcher_initialization) {
      core_.resetWithMapBase(obs);
      return;
    }
    core_.correctWithGlobalMatcher(obs);
  }

  /**
   * @brief 初始位姿回调函数（人工重置机器人位置）
   * @param msg RViz 2D Pose Estimate工具发布的初始位姿
   *
   * @details
   * 处理流程：
   * 1. 检查是否允许通过initial_pose重置（安全开关）
   * 2. 提取map->base_link目标位姿，构建GlobalObservation
   * 3. 调用resetWithMapBase重置系统位姿
   *
   * 典型使用场景：
   * - 机器人在地图中的初始位置未知时
   * - 系统发生严重漂移需要人工纠正时
   * - 重定位后确认正确位置
   */
  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!config_.allow_initialpose) {
      return;
    }
    sof::GlobalObservation obs;
    obs.stamp = msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec();
    obs.map_base = poseFromRos(msg->pose.pose);
    obs.success = true;
    core_.resetWithMapBase(obs);
  }

  /**
   * @brief 源点云回调函数（缓存最新点云用于转发）
   * @param msg SLAM模块输出的源点云（通常在lidar或odom坐标系下）
   *
   * @details
   * 仅做缓存，不做实时处理。实际的坐标系转换和发布在publishMatcherCloud中完成。
   * 这种设计避免了在点云回调中进行耗时操作，保证DR/SLAM数据处理的实时性。
   */
  void onSourceCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    latest_cloud_ = msg;
  }

  /**
   * @brief 定时器回调函数（固定频率触发，驱动融合核心）
   * @param event ROS定时器事件（未使用）
   *
   * @details
   * 定时器是融合系统的核心驱动，每周期执行：
   * 1. 调用core_.update()进行融合计算
   *    - 内部处理DR和SLAM的时间同步
   *    - 计算增量、执行融合滤波
   *    - 返回有效的FusionOutput（若同步成功）
   * 2. 若update未返回有效输出（如同步失败）：
    *    - 使用core_.output()获取最新状态（可能基于预测）
   * 3. 检查输出有效性：
   *    - 无效：发布单位map->odom，等待数据同步
   *    - 有效：发布融合输出和点云
   */
  void onTimer(const ros::TimerEvent&) {
    auto maybe_output = core_.update();
    const sof::FusionOutput output = maybe_output ? *maybe_output : core_.output(ros::Time::now().toSec());
    if (!output.valid) {
      publishIdentityMapOdom();
      publishBootstrapMatcherCloud();
      ROS_WARN_THROTTLE(2.0, "[simple_odom_fusion] waiting for synchronized dr/slam odometry; publishing identity map->odom for RViz");
      return;
    }
    publishOutput(output);
    publishMatcherCloud(output);
  }

  /**
   * @brief 发布单位map->odom变换（初始化/等待状态使用）
   *
   * @details
   * 当DR和SLAM数据尚未同步成功时，发布map->odom为单位变换。
   * 这保证了TF树的完整性，使下游模块可以正常启动而不因缺少TF而报错。
   * 一旦融合核心产生有效输出，map->odom将被正确更新。
   */
  void publishIdentityMapOdom() {
    if (!config_.publish_tf || !config_.publish_map_to_odom_tf) {
      return;
    }
    sof::Pose3d identity;
    tf_broadcaster_.sendTransform(toTf(identity, config_.frames.map, config_.frames.odom, ros::Time::now()));
  }

  /**
   * @brief 在启动阶段转发源点云给全局定位模块（未做坐标变换）
   *
   * @details
   * 在系统尚未初始化时，直接将接收到的源点云转发到全局定位模块。
   * 此时不做任何坐标变换，仅修改header.frame_id为odom坐标系。
   * 全局定位模块可以使用这些点云进行初始匹配和定位。
   *
   * 与publishMatcherCloud的区别：
   * - publishBootstrapMatcherCloud：启动期使用，不做位姿变换
   * - publishMatcherCloud：正常运行期使用，根据融合位姿变换点云
   */
  void publishBootstrapMatcherCloud() {
    if (!config_.publish_gm_source_cloud || !latest_cloud_) {
      ROS_WARN_THROTTLE(2.0, "[simple_odom_fusion] waiting for source cloud topic: %s", config_.topics.source_cloud.c_str());
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*latest_cloud_, cloud);
    if (cloud.empty()) {
      ROS_WARN_THROTTLE(2.0, "[simple_odom_fusion] latest source cloud is empty");
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = latest_cloud_->header.stamp.isZero() ? ros::Time::now() : latest_cloud_->header.stamp;
    msg.header.frame_id = config_.frames.odom;
    gm_cloud_pub_.publish(msg);
  }

  /**
   * @brief 核心输出函数：发布融合后的位姿、轨迹、TF等
   * @param output SimpleFusionCore计算的融合输出
   *
   * @details
   * 发布内容：
   * 1. fused_odom: map->base_link里程计（含融合后的速度）
   * 2. fused_pose: map->base_link位姿（带协方差框架）
   * 3. path: 历史轨迹（可选，用于RViz可视化，滑动窗口限制）
   * 4. TF变换: map->odom 和/或 odom->base_link（可选）
   *
   * 速度说明：
   * - 输出速度是在map坐标系下的线速度（由SimpleFusionCore计算）
   * - 不同于原始DR/SLAM的twist，这是融合后的最优估计
   */
  void publishOutput(const sof::FusionOutput& output) {
    const ros::Time stamp(output.stamp);

    // --- 发布融合里程计 ---
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = config_.frames.map;      // 父坐标系：map
    odom.child_frame_id = config_.frames.base_link; // 子坐标系：base_link
    fillPose(output.map_base, &odom.pose.pose);
    // 填充融合后的速度（map坐标系下线速度）
    odom.twist.twist.linear.x = output.velocity_map.x();
    odom.twist.twist.linear.y = output.velocity_map.y();
    odom.twist.twist.linear.z = output.velocity_map.z();
    fused_odom_pub_.publish(odom);

    // --- 发布融合位姿（PoseWithCovarianceStamped格式） ---
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = odom.header;
    pose.pose.pose = odom.pose.pose;
    fused_pose_pub_.publish(pose);

    // --- 发布历史轨迹（可选） ---
    if (config_.publish_path) {
      geometry_msgs::PoseStamped path_pose;
      path_pose.header = odom.header;
      path_pose.pose = odom.pose.pose;
      path_.header.stamp = stamp;
      path_.poses.push_back(path_pose);
      // 轨迹长度限制：超过max_path_size时移除最旧的点（滑动窗口）
      while (static_cast<int>(path_.poses.size()) > config_.max_path_size) {
        path_.poses.erase(path_.poses.begin());
      }
      path_pub_.publish(path_);
    }

    // --- 发布TF变换（可选） ---
    if (config_.publish_tf) {
      std::vector<geometry_msgs::TransformStamped> transforms;
      // map->odom变换（全局修正量，来自融合核心）
      if (config_.publish_map_to_odom_tf) {
        transforms.push_back(toTf(output.map_odom, config_.frames.map, config_.frames.odom, stamp));
      }
      // odom->base_link变换（局部里程计，来自融合核心）
      if (config_.publish_odom_to_base_tf) {
        transforms.push_back(toTf(output.odom_base, config_.frames.odom, config_.frames.base_link, stamp));
      }
      if (!transforms.empty()) {
        tf_broadcaster_.sendTransform(transforms);
      }
    }
  }

  /**
   * @brief 将源点云变换到odom坐标系后转发给全局定位模块
   * @param output 融合输出，包含odom->base_link位姿
   *
   * @details
   * 功能说明：
   * - 接收SLAM输出的点云（通常在lidar坐标系下）
   * - 使用融合后的odom->base_link位姿将点云变换到odom坐标系
   * - 转发给全局定位模块，供其进行地图匹配
   *
   * 坐标变换链：
   * - 输入点云坐标系：通常是lidar或某个局部坐标系
   * - 变换矩阵：odom->base_link（即output.odom_base）
   * - 输出点云坐标系：odom
   *
   * 注意：当前代码中gm_cloud_use_fused_pose为true和false时都使用output.odom_base，
   * 这可能是简化实现或待完善的功能。
   */
  void publishMatcherCloud(const sof::FusionOutput& output) {
    if (!config_.publish_gm_source_cloud || !latest_cloud_) {
      return;
    }
    // 将ROS点云转换为PCL格式
    pcl::PointCloud<pcl::PointXYZI> in;
    pcl::fromROSMsg(*latest_cloud_, in);
    if (in.empty()) {
      return;
    }

    // 选择变换位姿（当前实现中两种情况都使用odom_base）
    const sof::Pose3d transform_pose = config_.gm_cloud_use_fused_pose ? output.odom_base : output.odom_base;
    // 执行点云坐标变换
    pcl::PointCloud<pcl::PointXYZI> out;
    pcl::transformPointCloud(in, out, toMatrix4f(transform_pose));

    // 转换回ROS消息并发布
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(out, msg);
    msg.header.stamp = latest_cloud_->header.stamp.isZero() ? ros::Time(output.stamp) : latest_cloud_->header.stamp;
    msg.header.frame_id = config_.frames.odom;
    gm_cloud_pub_.publish(msg);
  }

  // ========== ROS通信成员 ==========
  ros::NodeHandle nh_;           ///< 公共命名空间NodeHandle
  ros::NodeHandle pnh_;          ///< 私有命名空间NodeHandle（~）

  // ========== 融合核心与配置 ==========
  sof::FusionConfig config_;     ///< 融合配置参数结构体
  sof::SimpleFusionCore core_;   ///< 简单融合核心（处理同步、融合、修正等核心算法）

  // ========== TF与数据缓存 ==========
  tf2_ros::TransformBroadcaster tf_broadcaster_;     ///< TF广播器
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;  ///< 最新接收的源点云
  nav_msgs::Path path_;                              ///< 历史轨迹

  // ========== ROS通信句柄 ==========
  ros::Subscriber dr_sub_;           ///< DR（航位推算）里程计订阅器
  ros::Subscriber slam_sub_;         ///< SLAM里程计订阅器
  ros::Subscriber gm_sub_;           ///< 全局定位结果订阅器
  ros::Subscriber initial_pose_sub_; ///< 初始位姿订阅器
  ros::Subscriber cloud_sub_;        ///< 源点云订阅器
  ros::Publisher fused_odom_pub_;    ///< 融合里程计发布器
  ros::Publisher fused_pose_pub_;    ///< 融合位姿发布器
  ros::Publisher path_pub_;          ///< 轨迹发布器
  ros::Publisher gm_cloud_pub_;      ///< 全局定位点云发布器
  ros::Timer timer_;                 ///< 固定频率输出定时器
};

/**
 * @brief 主函数：初始化ROS节点并进入spin循环
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_odom_fusion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SimpleOdomFusionNode node(nh, pnh);
  ros::spin();
  return 0;
}