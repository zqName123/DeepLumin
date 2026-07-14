#include <localization/ros/ouster_point.hpp>
#include <localization/ros/pointcloud_preprocess.hpp>
#include <localization/ros/ros_converters.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace localization_ros {
namespace {

bool slamPreprocessConfigEqual(const localization::SlamConfig& a,
                               const localization::SlamConfig& b) {
    return a.lidar_type == b.lidar_type && a.scan_line == b.scan_line &&
           a.blind == b.blind && a.point_filter_num == b.point_filter_num &&
           std::abs(a.time_scale - b.time_scale) < 1e-9;
}

PointCloudPreprocess& getPointCloudPreprocess(const localization::SlamConfig& cfg) {
    static PointCloudPreprocess preprocessor;
    static localization::SlamConfig cached_cfg;
    static bool initialized = false;
    if (!initialized || !slamPreprocessConfigEqual(cfg, cached_cfg)) {
        preprocessor.set(cfg);
        cached_cfg = cfg;
        initialized = true;
    }
    return preprocessor;
}

void loadExtrinsicR(ros::NodeHandle& nh, localization::Mat3d& R) {
    std::vector<double> flat;
    if (!nh.getParam("slam/faster_lio/mapping/extrinsic_R", flat) || flat.size() != 9) {
        nh.getParam("slam/faster_lio/extrinsic_R", flat);
    }
    if (flat.size() == 9) {
        R << flat[0], flat[1], flat[2], flat[3], flat[4], flat[5], flat[6], flat[7], flat[8];
    }
}

void loadExtrinsicT(ros::NodeHandle& nh, localization::Vec3d& T) {
    std::vector<double> flat;
    if (!nh.getParam("slam/faster_lio/mapping/extrinsic_T", flat) || flat.size() != 3) {
        nh.getParam("slam/faster_lio/extrinsic_T", flat);
    }
    if (flat.size() == 3) {
        T << flat[0], flat[1], flat[2];
    }
}

}  // namespace

localization::FrameConfig loadFrameConfig(ros::NodeHandle& nh) {
    localization::FrameConfig cfg;
    nh.param<std::string>("localization/frames/map", cfg.map, "map");
    nh.param<std::string>("localization/frames/odom", cfg.odom, "odom");
    nh.param<std::string>("localization/frames/base_link", cfg.base_link, "base_link");
    nh.param<std::string>("localization/frames/imu", cfg.imu, "imu_link");
    nh.param<std::string>("localization/frames/lidar", cfg.lidar, "lidar_link");
    return cfg;
}

localization::SlamConfig loadSlamConfig(ros::NodeHandle& nh) {
    localization::SlamConfig cfg;
    nh.param<std::string>("slam/type", cfg.type, "faster_lio");
    nh.param<double>("slam/faster_lio/scan_rate", cfg.scan_rate, 10.0);

    nh.param<int>("slam/faster_lio/preprocess/lidar_type", cfg.lidar_type, 3);
    nh.param<int>("slam/faster_lio/preprocess/scan_line", cfg.scan_line, 64);
    nh.param<double>("slam/faster_lio/preprocess/blind", cfg.blind, 3.0);
    nh.param<double>("slam/faster_lio/preprocess/time_scale", cfg.time_scale, 1e-3);
    nh.param<int>("slam/faster_lio/preprocess/point_filter_num", cfg.point_filter_num, 2);

    nh.param<double>("slam/faster_lio/mapping/acc_cov", cfg.acc_cov, 0.1);
    nh.param<double>("slam/faster_lio/mapping/gyr_cov", cfg.gyr_cov, 0.1);
    nh.param<double>("slam/faster_lio/mapping/b_acc_cov", cfg.b_acc_cov, 0.0001);
    nh.param<double>("slam/faster_lio/mapping/b_gyr_cov", cfg.b_gyr_cov, 0.0001);
    nh.param<double>("slam/faster_lio/mapping/fov_degree", cfg.fov_degree, 180.0);
    nh.param<double>("slam/faster_lio/mapping/det_range", cfg.det_range, 150.0);
    nh.param<bool>("slam/faster_lio/mapping/extrinsic_est_en", cfg.estimate_extrinsic, false);
    loadExtrinsicT(nh, cfg.extrinsic_T);
    loadExtrinsicR(nh, cfg.extrinsic_R);

    nh.param<double>("slam/faster_lio/ivox/grid_resolution", cfg.ivox_grid_resolution, 0.5);
    nh.param<int>("slam/faster_lio/ivox/nearby_type", cfg.ivox_nearby_type, 18);
    nh.param<double>("slam/faster_lio/ivox/cube_side_length", cfg.cube_side_length, 2000.0);

    nh.param<double>("slam/faster_lio/filter_size_surf", cfg.filter_size_surf, 0.2);
    nh.param<double>("slam/faster_lio/filter_size_map", cfg.filter_size_map, 0.2);
    nh.param<int>("slam/faster_lio/max_iteration", cfg.max_iteration, 3);
    nh.param<double>("slam/faster_lio/esti_plane_threshold", cfg.esti_plane_threshold, 0.1);
    nh.param<double>("slam/faster_lio/degeneracy_threshold", cfg.degeneracy_threshold, 100.0);
    return cfg;
}

localization::ImuData fromRos(const sensor_msgs::Imu& msg) {
    localization::ImuData imu;
    imu.timestamp = msg.header.stamp.toSec();
    imu.gyro = localization::Vec3d(msg.angular_velocity.x, msg.angular_velocity.y,
                                   msg.angular_velocity.z);
    imu.accel = localization::Vec3d(msg.linear_acceleration.x, msg.linear_acceleration.y,
                                    msg.linear_acceleration.z);
    return imu;
}

localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg) {
    localization::TimestampedPointCloud pc;
    pc.timestamp = msg.header.stamp.toSec();
    pc.frame_id = msg.header.frame_id;
    pc.cloud.reset(new localization::PointCloud());
    pcl::fromROSMsg(msg, *pc.cloud);
    return pc;
}

localization::TimestampedPointCloud fromRos(const sensor_msgs::PointCloud2& msg,
                                            const localization::SlamConfig& cfg) {
    return fromRosLidar(msg, cfg);
}

localization::TimestampedPointCloud fromRosOuster(const sensor_msgs::PointCloud2& msg,
                                                  const localization::SlamConfig& cfg) {
    localization::TimestampedPointCloud pc;
    pc.timestamp = msg.header.stamp.toSec();
    pc.frame_id = msg.header.frame_id;
    pc.cloud.reset(new localization::PointCloud());

    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(msg, pl_orig);
    const int plsize = static_cast<int>(pl_orig.size());
    pc.cloud->reserve(plsize);
    for (int i = 0; i < plsize; ++i) {
        if (cfg.point_filter_num > 0 && i % cfg.point_filter_num != 0) {
            continue;
        }
        const auto& src = pl_orig.points[i];
        const double range = src.x * src.x + src.y * src.y + src.z * src.z;
        if (range < cfg.blind * cfg.blind) {
            continue;
        }

        localization::PointType added_pt;
        added_pt.x = src.y;
        added_pt.y = -src.x;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = src.t / 1e6f;
        pc.cloud->points.push_back(added_pt);
    }
    pc.cloud->width = pc.cloud->points.size();
    pc.cloud->height = 1;
    pc.cloud->is_dense = false;
    return pc;
}

localization::TimestampedPointCloud fromRosLidar(const sensor_msgs::PointCloud2& msg,
                                                 const localization::SlamConfig& cfg) {
    if (cfg.lidar_type == 3) {
        return fromRosOuster(msg, cfg);
    }

    localization::TimestampedPointCloud pc;
    pc.timestamp = msg.header.stamp.toSec();
    pc.frame_id = msg.header.frame_id;
    pc.cloud.reset(new localization::PointCloud());
    getPointCloudPreprocess(cfg).process(msg, pc.cloud);
    return pc;
}

nav_msgs::Odometry toRosOdom(const localization::OdomResult& odom,
                             const std::string& parent_frame,
                             const std::string& child_frame) {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time(odom.timestamp);
    msg.header.frame_id = parent_frame;
    msg.child_frame_id = child_frame;
    msg.pose.pose.position.x = odom.position.x();
    msg.pose.pose.position.y = odom.position.y();
    msg.pose.pose.position.z = odom.position.z();
    msg.pose.pose.orientation.x = odom.orientation.x();
    msg.pose.pose.orientation.y = odom.orientation.y();
    msg.pose.pose.orientation.z = odom.orientation.z();
    msg.pose.pose.orientation.w = odom.orientation.w();
    for (int i = 0; i < 36; ++i) {
        msg.pose.covariance[i] = odom.covariance(i / 6, i % 6);
    }
    msg.twist.twist.linear.x = odom.velocity.x();
    msg.twist.twist.linear.y = odom.velocity.y();
    msg.twist.twist.linear.z = odom.velocity.z();
    return msg;
}

sensor_msgs::PointCloud2 toRosPointCloud(const localization::TimestampedPointCloud& pc) {
    sensor_msgs::PointCloud2 msg;
    if (!pc.cloud) {
        return msg;
    }
    pcl::toROSMsg(*pc.cloud, msg);
    msg.header.stamp = ros::Time(pc.timestamp);
    msg.header.frame_id = pc.frame_id;
    return msg;
}

geometry_msgs::TransformStamped toRosTransform(const localization::OdomResult& odom,
                                               const std::string& parent_frame,
                                               const std::string& child_frame) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time(odom.timestamp);
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = odom.position.x();
    tf.transform.translation.y = odom.position.y();
    tf.transform.translation.z = odom.position.z();
    tf.transform.rotation.x = odom.orientation.x();
    tf.transform.rotation.y = odom.orientation.y();
    tf.transform.rotation.z = odom.orientation.z();
    tf.transform.rotation.w = odom.orientation.w();
    return tf;
}

}  // namespace localization_ros
