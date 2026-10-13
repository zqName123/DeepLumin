#pragma once

/**
 * @file map_viz.hpp
 * @brief 全局地图可视化发布（仅 RViz 观察用，算法不消费）
 *
 * 用途：把建图 PCD 降采样后发到 DR 输出同系（frame_id=odom），
 * 配合 /dr_odometry/path 观察 DR 轨迹形状是否与巷道一致。
 *
 * 注意：地图原生在 map 系、DR 在自有局部系，二者无自动对齐；
 * 可用 offset_* 把地图平移/旋转到 DR 轨迹附近（粗对齐、目测用）。
 */

#include <ros/ros.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace dr_odometry_v2_ros {

class MapVizPublisher {
 public:
  struct Config {
    bool enabled = false;
    std::string pcd_path;
    std::string topic = "/dr_odometry/global_map";
    std::string frame_id = "odom";
    double voxel_leaf = 5.0;       ///< 降采样体素（m）；0=不降采样
    double publish_period = 5.0;   ///< 周期重发（latched + 定时，方便后开 RViz）
    // 手动对齐：地图在输出系中的摆放（粗对齐目测用）
    double offset_x = 0.0;
    double offset_y = 0.0;
    double offset_z = 0.0;
    double offset_yaw_deg = 0.0;
  };

  ~MapVizPublisher();

  /**
   * @brief 立即建立发布并启动后台加载（不阻塞节点启动/订阅）。
   * 199MB 级 PCD 加载/降采样在独立线程完成，就绪后 latched + 周期发布。
   * 失败只记日志，不抛异常，节点照常运行。
   */
  bool setup(ros::NodeHandle& nh, const Config& config);

  bool ready() const { return ready_.load(); }

 private:
  void loadMap();
  void publishOnce();

  Config config_;
  ros::Publisher pub_;
  ros::Timer timer_;
  std::thread loader_;
  std::mutex mutex_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  std::atomic<bool> ready_{false};
};

}  // namespace dr_odometry_v2_ros
