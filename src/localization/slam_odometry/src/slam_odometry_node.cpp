#include <localization/ros/ros_converters.hpp>
#include <localization/ros/sensor_topics.hpp>
#include <localization/slam/faster_lio_impl.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

class SlamOdometryNode {
public:
    explicit SlamOdometryNode(ros::NodeHandle& nh) : nh_(nh), pnh_("~") {
        frames_ = localization_ros::loadFrameConfig(pnh_);
        sensor_topics_ = localization_ros::loadSensorTopicsConfig(pnh_);
        output_topics_ = localization_ros::loadOutputTopicsConfig(pnh_);
        slam_config_ = localization_ros::loadSlamConfig(pnh_);

        if (slam_config_.type != "faster_lio") {
            ROS_WARN("[slam_odometry] unsupported slam/type '%s', using faster_lio",
                     slam_config_.type.c_str());
        }

        slam_.reset(new localization::FasterLIOImpl());
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

        ROS_INFO("[slam_odometry] imu=%s lidar=%s odom=%s local_map=%s registered_scan=%s",
                 sensor_topics_.imu.c_str(), sensor_topics_.lidar.c_str(),
                 output_topics_.slam.c_str(), output_topics_.local_map.c_str(),
                 output_topics_.registered_scan.c_str());
    }

    void tick() {
        if (!slam_ || !slam_->processOnce()) {
            return;
        }

        const auto odom = slam_->getOdometry();
        const auto local_map = slam_->getLocalMap();
        const auto current_scan = slam_->getCurrentScan();

        if (odom.valid) {
            odom_pub_.publish(localization_ros::toRosOdom(odom, frames_.odom, frames_.base_link));
            tf_broadcaster_.sendTransform(
                localization_ros::toRosTransform(odom, frames_.odom, frames_.base_link));
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
            const Eigen::Matrix3d R = odom.orientation.toRotationMatrix();
            const Eigen::Vector3d t = odom.position;
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
    void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
        if (slam_) {
            slam_->feedImu(localization_ros::fromRos(*msg));
        }
    }

    void onLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (slam_) {
            slam_->feedLidar(localization_ros::fromRos(*msg, slam_config_));
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    localization::FrameConfig frames_;
    localization::SlamConfig slam_config_;
    localization_ros::SensorTopicsConfig sensor_topics_;
    localization_ros::OutputTopicsConfig output_topics_;
    std::shared_ptr<localization::ISlamOdometry> slam_;
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
