#include <localization/factory/factory_registry.hpp>
#include <localization/ros/ros_converters.hpp>
#include <localization/ros/global_map_viz.hpp>
#include <localization/ros/sensor_topics.hpp>
#include <localization/slam/faster_lio_impl.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <cstdint>
#include <string>


namespace {

Eigen::Matrix4d sensorExtrinsicToMatrix(const localization::SensorExtrinsic& ext) {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = ext.rotation;
    t.block<3, 1>(0, 3) = ext.translation;
    return t;
}

Eigen::Matrix4d odomToMatrix(const localization::OdomResult& odom) {
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
    t.block<3, 3>(0, 0) = odom.orientation.normalized().toRotationMatrix();
    t.block<3, 1>(0, 3) = odom.position;
    return t;
}

localization::OdomResult matrixToOdom(const localization::OdomResult& src,
                                      const Eigen::Matrix4d& transform) {
    localization::OdomResult out = src;
    out.position = transform.block<3, 1>(0, 3);
    out.orientation = localization::Quatd(transform.block<3, 3>(0, 0)).normalized();
    return out;
}

localization::OdomResult transformSensorOdomToBase(
    const localization::OdomResult& sensor_odom,
    const localization::SensorExtrinsic& base_to_sensor) {
    if (!sensor_odom.valid) {
        return sensor_odom;
    }
    const Eigen::Matrix4d t_odom_sensor = odomToMatrix(sensor_odom);
    const Eigen::Matrix4d t_base_sensor = sensorExtrinsicToMatrix(base_to_sensor);
    return matrixToOdom(sensor_odom, t_odom_sensor * t_base_sensor.inverse());
}

}  // namespace

class SlamOdometryNode {
public:
    explicit SlamOdometryNode(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
        localization::registerDefaultPlugins();
        frames_ = localization_ros::loadFrameConfig(pnh_);
        extrinsics_ = localization_ros::loadExtrinsicConfig(pnh_);
        sensor_topics_ = localization_ros::loadSensorTopicsConfig(pnh_);
        output_topics_ = localization_ros::loadOutputTopicsConfig(pnh_);
        slam_config_ = localization_ros::loadSlamConfig(pnh_);
        pnh_.param<bool>("slam/publish_tf", publish_tf_, true);
        pnh_.param<bool>("slam/startup/enable_imu_warmup", enable_imu_warmup_, true);
        pnh_.param<double>("slam/startup/imu_warmup_duration", imu_warmup_duration_, 1.0);
        pnh_.param<int>("slam/startup/min_imu_samples", min_imu_samples_, 20);
        pnh_.param<std::string>("slam/point_time_mode", point_time_mode_, "auto");
        pnh_.param<double>("slam/no_point_time/startup_process_rate", no_point_time_startup_process_rate_, 2.0);
        pnh_.param<double>("slam/no_point_time/normal_process_rate", no_point_time_normal_process_rate_, 10.0);
        pnh_.param<int>("slam/no_point_time/startup_valid_frames", no_point_time_startup_valid_frames_, 20);

        if (slam_config_.type != "faster_lio") {
            ROS_WARN("[slam_odometry] unsupported slam/type '%s', using faster_lio",
                     slam_config_.type.c_str());
        }

        slam_ = localization::SlamFactory::instance().create(slam_config_.type);
        if (!slam_) {
            ROS_ERROR("[slam_odometry] failed to create SLAM implementation for type: %s",
                      slam_config_.type.c_str());
            ros::shutdown();
            return;
        }
        if (!slam_->initialize(slam_config_)) {
            ROS_ERROR("[slam_odometry] failed to initialize Faster-LIO");
            ros::shutdown();
            return;
        }

        imu_sub_ = nh_.subscribe(sensor_topics_.imu, 200000, &SlamOdometryNode::onImu, this);
        lidar_sub_ = nh_.subscribe(sensor_topics_.lidar, 200000, &SlamOdometryNode::onLidar, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topics_.slam, 100);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topics_.local_map, 10);
        registered_scan_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>(output_topics_.registered_scan, 10);
        global_map_viz_.setup(nh_, pnh_, output_topics_.global_map, frames_.map, "slam_odometry");

        ROS_INFO("[slam_odometry] imu=%s lidar=%s odom=%s local_map=%s registered_scan=%s",
                 sensor_topics_.imu.c_str(), sensor_topics_.lidar.c_str(),
                 output_topics_.slam.c_str(), output_topics_.local_map.c_str(),
                 output_topics_.registered_scan.c_str());
        ROS_INFO("[slam_odometry] startup_imu_warmup=%s duration=%.3fs min_samples=%d publish_tf=%s",
                 enable_imu_warmup_ ? "true" : "false", imu_warmup_duration_, min_imu_samples_,
                 publish_tf_ ? "true" : "false");
        ROS_INFO("[slam_odometry] point_time_mode=%s no_point_time_startup_rate=%.3fHz normal_rate=%.3fHz startup_valid_frames=%d",
                 point_time_mode_.c_str(), no_point_time_startup_process_rate_,
                 no_point_time_normal_process_rate_, no_point_time_startup_valid_frames_);
    }

    void tick() {
        if (!slam_ || !shouldRunProcessOnce()) {
            return;
        }
        if (!slam_->processOnce()) {
            return;
        }

        const auto odom_sensor = slam_->getOdometry();
        const auto odom = transformSensorOdomToBase(odom_sensor, extrinsics_.base_to_imu);
        updateNoPointTimeStartupState(odom);
        const auto local_map = slam_->getLocalMap();
        const auto current_scan = slam_->getCurrentScan();

        if (odom.valid) {
            odom_pub_.publish(localization_ros::toRosOdom(odom, frames_.odom, frames_.base_link));
            if (publish_tf_) {
                tf_broadcaster_.sendTransform(
                    localization_ros::toRosTransform(odom, frames_.odom, frames_.base_link));
            }
        }
        if (local_map.cloud && !local_map.cloud->empty()) {
            auto map_msg = local_map;
            map_msg.frame_id = frames_.odom;
            local_map_pub_.publish(localization_ros::toRosPointCloud(map_msg));
        }
        if (current_scan.cloud && !current_scan.cloud->empty() && odom.valid) {
            localization::TimestampedPointCloud scan_world;
            scan_world.timestamp = current_scan.timestamp;
            scan_world.frame_id = frames_.odom;
            scan_world.cloud.reset(new localization::PointCloud());
            scan_world.cloud->reserve(current_scan.cloud->size());
            const Eigen::Matrix4d t_odom_base = odomToMatrix(odom);
            const Eigen::Matrix4d t_base_lidar = sensorExtrinsicToMatrix(extrinsics_.base_to_lidar);
            const Eigen::Matrix4d t_odom_lidar = t_odom_base * t_base_lidar;
            const Eigen::Matrix3d R = t_odom_lidar.block<3, 3>(0, 0);
            const Eigen::Vector3d t = t_odom_lidar.block<3, 1>(0, 3);
            for (const auto& pt : current_scan.cloud->points) {
                const Eigen::Vector3d pw = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + t;
                localization::PointType out = pt;
                out.x = static_cast<float>(pw.x());
                out.y = static_cast<float>(pw.y());
                out.z = static_cast<float>(pw.z());
                scan_world.cloud->push_back(out);
            }
            registered_scan_pub_.publish(localization_ros::toRosPointCloud(scan_world));
        }
    }

private:
    void updateImuWarmup(double stamp) {
        if (first_imu_stamp_ <= 0.0) {
            first_imu_stamp_ = stamp;
        }
        last_imu_stamp_ = stamp;
        ++imu_warmup_samples_;

        if (!enable_imu_warmup_ || imu_warmup_ready_) {
            return;
        }

        const double duration = last_imu_stamp_ - first_imu_stamp_;
        if (duration >= imu_warmup_duration_ &&
            imu_warmup_samples_ >= static_cast<std::uint64_t>(std::max(1, min_imu_samples_))) {
            imu_warmup_ready_ = true;
            ROS_INFO("[slam_odometry] IMU warmup ready: samples=%lu duration=%.3fs dropped_lidar=%lu",
                     static_cast<unsigned long>(imu_warmup_samples_), duration,
                     static_cast<unsigned long>(dropped_lidar_before_warmup_));
        }
    }

    bool canFeedLidar() const {
        return !enable_imu_warmup_ || imu_warmup_ready_;
    }

    static bool hasPointTimeField(const sensor_msgs::PointCloud2& msg) {
        return std::any_of(msg.fields.begin(), msg.fields.end(), [](const auto& field) {
            return field.name == "t" || field.name == "time" || field.name == "timestamp";
        });
    }

    bool noPointTimeMode(const sensor_msgs::PointCloud2& msg) {
        if (point_time_mode_ == "none") {
            point_time_known_ = true;
            point_time_available_ = false;
            return true;
        }
        if (point_time_mode_ == "present" || point_time_mode_ == "curvature") {
            point_time_known_ = true;
            point_time_available_ = true;
            return false;
        }
        if (!point_time_known_) {
            point_time_available_ = hasPointTimeField(msg);
            point_time_known_ = true;
            if (point_time_available_) {
                ROS_INFO("[slam_odometry] point time field detected, use normal Faster-LIO lidar feed");
            } else {
                ROS_WARN("[slam_odometry] point time field is missing, enable no-point-time compatible feed");
            }
        }
        return !point_time_available_;
    }

    bool noPointTimeActive() const {
        return point_time_known_ && !point_time_available_;
    }

    bool shouldRunProcessOnce() {
        if (!noPointTimeActive()) {
            return true;
        }

        const double rate = no_point_time_startup_done_ ? no_point_time_normal_process_rate_
                                                        : no_point_time_startup_process_rate_;
        if (rate <= 0.0) {
            return true;
        }

        const ros::WallTime now = ros::WallTime::now();
        if (!last_no_point_time_process_wall_.isZero() &&
            (now - last_no_point_time_process_wall_).toSec() < 1.0 / rate) {
            return false;
        }
        last_no_point_time_process_wall_ = now;
        return true;
    }

    void updateNoPointTimeStartupState(const localization::OdomResult& odom) {
        if (!noPointTimeActive() || no_point_time_startup_done_ || !odom.valid) {
            return;
        }
        ++no_point_time_valid_odom_count_;
        if (no_point_time_valid_odom_count_ >=
            static_cast<std::uint64_t>(std::max(1, no_point_time_startup_valid_frames_))) {
            no_point_time_startup_done_ = true;
            ROS_INFO("[slam_odometry] no-point-time startup done: valid_odom_frames=%lu, restore process rate to %.3fHz",
                     static_cast<unsigned long>(no_point_time_valid_odom_count_),
                     no_point_time_normal_process_rate_);
        }
    }

    void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
        if (slam_) {
            slam_->feedImu(localization_ros::fromRos(*msg));
            updateImuWarmup(msg->header.stamp.toSec());
        }
    }

    void onLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!slam_) {
            return;
        }
        if (!canFeedLidar()) {
            ++dropped_lidar_before_warmup_;
            if (dropped_lidar_before_warmup_ == 1 || dropped_lidar_before_warmup_ % 20 == 0) {
                const double duration = first_imu_stamp_ > 0.0 ? last_imu_stamp_ - first_imu_stamp_ : 0.0;
                ROS_INFO("[slam_odometry] drop lidar before IMU warmup: dropped=%lu imu_samples=%lu duration=%.3fs lidar_t=%.9f",
                         static_cast<unsigned long>(dropped_lidar_before_warmup_),
                         static_cast<unsigned long>(imu_warmup_samples_), duration,
                         msg->header.stamp.toSec());
            }
            return;
        }
        noPointTimeMode(*msg);
        slam_->feedLidar(localization_ros::fromRos(*msg, slam_config_));
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    localization::FrameConfig frames_;
    localization::ExtrinsicConfig extrinsics_;
    localization::SlamConfig slam_config_;
    bool publish_tf_ = true;
    bool enable_imu_warmup_ = true;
    double imu_warmup_duration_ = 1.0;
    int min_imu_samples_ = 20;
    std::string point_time_mode_ = "auto";
    double no_point_time_startup_process_rate_ = 2.0;
    double no_point_time_normal_process_rate_ = 10.0;
    int no_point_time_startup_valid_frames_ = 20;
    double first_imu_stamp_ = 0.0;
    double last_imu_stamp_ = 0.0;
    std::uint64_t imu_warmup_samples_ = 0;
    std::uint64_t dropped_lidar_before_warmup_ = 0;
    std::uint64_t no_point_time_valid_odom_count_ = 0;
    bool imu_warmup_ready_ = false;
    bool point_time_known_ = false;
    bool point_time_available_ = false;
    bool no_point_time_startup_done_ = false;
    ros::WallTime last_no_point_time_process_wall_;
    localization_ros::SensorTopicsConfig sensor_topics_;
    localization_ros::OutputTopicsConfig output_topics_;
    std::shared_ptr<localization::ISlamOdometry> slam_;
    localization_ros::GlobalMapVizPublisher global_map_viz_;
    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher registered_scan_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_odometry");
    ros::NodeHandle nh;
    SlamOdometryNode node(nh);
    ros::Rate rate(5000);
    while (ros::ok()) {
        ros::spinOnce();
        node.tick();
        rate.sleep();
    }
    return 0;
}
