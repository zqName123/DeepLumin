/**
 * @file types.hpp
 * @brief SLAM odometry 公共数据类型（无 ROS 依赖）
 *
 * 核心类型：
 *   ImuData / WheelData / GnssData     - 传感器输入
 *   TimestampedPointCloud              - 带时间戳点云（PointXYZINormal，curvature=点时间 ms）
 *   DrState / OdomResult / Pose        - 状态与输出
 *   KeyFrame / LoopConstraint          - 建图与回环
 *
 * PointType 使用 PointXYZINormal 与 faster-lio 一致，支持 per-point 时间去畸变
 */
#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>

/**
内部数据类型定义：
ImuData：IMU数据
WheelData：轮速数据
GnssData：GNSS数据
TimestampedPointCloud：带时间戳点云
Pose：位姿
OdomResult：里程计结果
DrState：DR状态
NominalState：名义状态
KeyFrame：关键帧
RelocCandidate：重定位候选
LoopConstraint：回环约束
RelocSearchConfig：重定位搜索配置
SceneState：场景状态
ObserverWeights：观测权重
RelocState：重定位状态
RunMode：运行模式
RelocContext：重定位上下文

Vec3d：三维向量
Quatd：四元数
Mat3d：3x3矩阵
Mat6d：6x6矩阵
Mat15d：15x15矩阵
PointType：点类型
PointCloud：点云
PointCloudPtr：点云指针
*/
namespace localization {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;
using Mat3d = Eigen::Matrix3d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat15d = Eigen::Matrix<double, 15, 15>;
using PointType = pcl::PointXYZINormal;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloud::Ptr;

/**
 * IMU 采样（body 系）
 * 来源：ros_converters::fromRos(sensor_msgs::Imu)
 * 消费：DrOdometryEstimator::feedImu → DrEskfCore::predictImu
 */
struct ImuData {
    double timestamp = 0.0;   ///< 秒（ROS header.stamp.toSec）
    Vec3d gyro = Vec3d::Zero();   ///< 角速度 ω (rad/s)
    Vec3d accel = Vec3d::Zero();  ///< 线加速度 a (m/s²)，含重力比力
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
};

/**
 * 轮速采样
 * 消费：feedWheel 缓存 → feedImu 内插值到 IMU 时刻 → correctWheel
 */
struct WheelData {
    double timestamp = 0.0;
    double velocity = 0.0;       ///< 纵向速度 (m/s)，倒车为负
    double steer_angle = 0.0;    ///< 转向角（当前 DR 观测未用）
    double velocity_std = 0.05;  ///< 观测噪声 σ (m/s)
};

enum class GnssFixType : uint8_t {
    NO_FIX = 0,
    FLOAT = 1,
    FIXED = 2
};

struct GnssData {
    double timestamp = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    GnssFixType fix_type = GnssFixType::NO_FIX;
    double position_std = 1.0;
    Vec3d position_enu = Vec3d::Zero();
};

struct TimestampedPointCloud {
    PointCloudPtr cloud;
    double timestamp = 0.0;
    std::string frame_id;
};

struct Pose {
    double timestamp = 0.0;
    Vec3d position = Vec3d::Zero();
    Quatd orientation = Quatd::Identity();
    Mat6d covariance = Mat6d::Identity();
    std::string frame_id;
};

/**
 * 统一里程计输出（节点发布层使用）
 * dr_debug::tick 把 DrState 填进本结构，再交给 toRosOdom / toRosTransform。
 */
struct OdomResult {
    double timestamp = 0.0;
    Vec3d position = Vec3d::Zero();
    Quatd orientation = Quatd::Identity();
    Vec3d velocity = Vec3d::Zero();
    Mat6d covariance = Mat6d::Identity();
    bool valid = false;
    bool is_degenerate = false;
    double degeneracy_factor = 0.0;
    std::string frame_id = "odom";
};

/**
 * DR/ESKF 名义状态快照
 * 由 DrEskfCore::getState 填充；wheel_scale 由 DrOdometryEstimator 单独覆盖写入。
 * 误差状态 15 维对应：δp(3) δv(3) δθ(3) δbg(3) δba(3)。
 */
struct DrState {
    double timestamp = 0.0;
    Vec3d position = Vec3d::Zero();      ///< 世界/odom 系位置
    Vec3d velocity = Vec3d::Zero();      ///< 世界系速度
    Quatd orientation = Quatd::Identity(); ///< body → world
    Vec3d gyro_bias = Vec3d::Zero();
    Vec3d accel_bias = Vec3d::Zero();
    double wheel_scale = 1.0;            ///< 轮速比例（上层低通维护，非 ESKF 状态）
    Mat15d covariance = Mat15d::Identity();
};

struct NominalState {
    Vec3d position = Vec3d::Zero();
    Vec3d velocity = Vec3d::Zero();
    Quatd orientation = Quatd::Identity();
    Vec3d gyro_bias = Vec3d::Zero();
    Vec3d accel_bias = Vec3d::Zero();
    double wheel_scale = 1.0;
};

struct KeyFrame {
    int id = -1;
    double timestamp = 0.0;
    Pose pose_map;
    Pose pose_odom;
    PointCloudPtr cloud_body;
    Eigen::MatrixXd scan_context;
};

struct RelocCandidate {
    int keyframe_id = -1;
    double score = 0.0;
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
};

struct LoopConstraint {
    int from_id = -1;
    int to_id = -1;
    Pose relative_pose;
    Mat6d noise = Mat6d::Identity();
    double fitness_score = 0.0;
};

struct RelocSearchConfig {
    double xy_offset = 2.0;
    int yaw_offset_steps = 1;
    double yaw_resolution = 0.5;
};

enum class SceneState : uint8_t {
    OPEN_PIT,
    TRANSITION,
    UNDERGROUND
};

struct ObserverWeights {
    double slam_weight = 1.0;
    double gnss_weight = 0.0;
    double dr_prediction_weight = 1.0;
};

enum class RelocState : uint8_t {
    IDLE,
    SEARCHING,
    MATCHING,
    SUCCESS,
    FAILED
};

enum class RunMode : uint8_t {
    LOCALIZATION = 0,
    MAPPING = 1,
    RELOCALIZATION = 2,
    PURE_SLAM = 3
};

struct RelocContext {
    OdomResult last_slam_odom;
    Mat15d fusion_cov = Mat15d::Identity();
    int consecutive_degenerate_frames = 0;
    bool manual_trigger = false;
};

}  // namespace localization
