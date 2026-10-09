#pragma once

#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "lio_odometry/core/eskf.h"
#include "lio_odometry/core/imu_processing.h"
#include "lio_odometry/core/pointcloud_preprocess.h"
#include "lio_odometry/core/types.h"
#include "lio_odometry/core/voxelmap/voxel_map_fastlivo.h"
#include "lio_odometry/core/voxelmap/voxel_map_config.h"

inline Eigen::Matrix3f SKEW_SYM_MATRIX(const Eigen::Vector3f& v) {
    Eigen::Matrix3f m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

namespace slam_odometry {

namespace fasterlio {
constexpr double INIT_TIME = 0.1;
constexpr int NUM_MATCH_POINTS = 5;
constexpr int MIN_NUM_MATCH_POINTS = 3;
extern int NUM_MAX_ITERATIONS;
extern float ESTI_PLANE_THRESHOLD;
extern float lidar_time_interval;
}  // namespace fasterlio

inline float calc_dist(const Vec3f& a, const Vec3f& b) {
    return (a - b).squaredNorm();
}

bool esti_plane(Vec4f& pca_result, const PointVector& point, const float& threshold = 0.1f);

struct LaserOdometryOptions {
    bool is_in_slam_mode = true;
    double kf_dis_th = 2.0;
    double kf_angle_th = 15.0 * M_PI / 180.0;
};

class LaserOdometryCore {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LaserOdometryCore(const LaserOdometryOptions& options = LaserOdometryOptions());
    ~LaserOdometryCore();

    bool Init(const std::string& config_yaml);
    bool Run();

    void ProcessPointCloud(const CloudPtr& cloud, double timestamp);
    void ProcessPointCloud(const pcl::PointCloud<OusterPoint>::Ptr& cloud, double timestamp);
    void ProcessPointCloud(const pcl::PointCloud<VelodynePoint>::Ptr& cloud, double timestamp);
    void ProcessIMU(const IMUPtr& msg_in);

    void FeedWheelSpeed(double timestamp, double speed_kmh, int gear);
    void FeedGnssAttitude(double heading_deg, double pitch_deg, double roll_deg, int status, int nsv);

    const NavState& GetState() const { return state_point_; }
    CloudPtr GetScanUndist() const { return scan_undistort_; }
    CloudPtr GetScanDownBody() const { return scan_down_body_; }
    CloudPtr GetScanDownWorld() const { return scan_down_world_; }
    CloudPtr GetEffectScan() const;

    const ESKF& GetESKF() const { return kf_; }
    LidarType GetLidarType() const { return preprocess_->GetLidarType(); }

   private:
    bool SyncPackages();
    // 传感器时间回退时的轻量对齐：只清输入缓冲与跨帧积分衔接，保留滤波器状态、
    // IMU 初始化结果和地图，避免行驶中被强制冷启动（静止初始化假设不成立会导致飘飞）。
    void ResyncStreams();
    // 完整重启（当前仅保留接口，无调用方）：清地图、重置滤波器并重新做 IMU 初始化。
    void ResetTracking();
    void ObsModel(NavState& s, ESKF::CustomObservationModel& obs);
    void ObsModelWheelspeed(NavState& s, ESKF::CustomObservationModel& obs);
    void ObsModelGPS(NavState& s, ESKF::CustomObservationModel& obs);
    // 按激光帧区间消费轮速队列，产出 wheel_v_mean_, wheel_valid_this_frame_ [testwheel]
    void ConsumeWheelSpeedForFrame(double t_beg, double t_end);

    inline void PointBodyToWorld(const PointType& pi, PointType& po) {
        Vec3d p_global(state_point_.rot_ *
                           (state_point_.offset_R_lidar_ * pi.getVector3fMap().cast<double>() +
                            state_point_.offset_t_lidar_) +
                       state_point_.pos_);
        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();
    bool LoadParamsFromYAML(const std::string& yaml);

    double filter(double new_value);

   private:
    LaserOdometryOptions options_;

    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;

    // FAST-LIVO 风格 VoxelMap 地图结构（替换 iVox 和 R-VoxelMap）
    slam_odometry::VoxelMapConfig vm_config_;
    slam_odometry::VoxelMapManagerFASTLIVO voxel_map_mgr_;
    std::vector<slam_odometry::pointWithVar> pv_list_;
    std::vector<slam_odometry::PointToPlane> ptpl_list_;

    double filter_size_map_min_ = 0;

    std::vector<double> extrinT_{3, 0.0};
    std::vector<double> extrinR_{9, 0.0};

    CloudPtr scan_undistort_{new PointCloudType()};
    CloudPtr scan_down_body_{new PointCloudType()};
    CloudPtr scan_down_world_{new PointCloudType()};
    pcl::VoxelGrid<PointType> voxel_scan_;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<CloudPtr> lidar_buffer_;
    std::deque<IMUPtr> imu_buffer_;

    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = true;
    int skip_lidar_num_ = 5;
    int skip_lidar_cnt_ = 0;

    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_num_ = 0;

    double last_lidar_time_ = 0;

    MeasureGroup measures_;

    ESKF kf_;

    NavState state_point_;

    Vec3d pos_lidar_;
    SO3 euler_cur_;
    bool extrinsic_est_en_ = true;
    bool use_aa_ = false;

    bool use_wheel_speed_ = false;
    bool use_gnss_ = false;

    double wheel_velocity_noise_ = 0.33;
    double gnss_attitude_noise_ = 0.03;
    double lidar_update_r_ = 0.001;
    bool use_adaptive_r_ = true;
    int min_effective_points_ = 20;
    double min_effective_ratio_ = 0.05;
    double r_base_ = 0.001;                // FAST-LIVO2 风格观测噪声基线

    int frame_num_ = 0;

    float map_trim_radius_ = 0.0f;            // iVox 局部地图剔除半径（米）；0=不剔除
    int map_trim_interval_frames_ = 10;        // 每 N 帧执行一次剔除（控制开销）

    bool keep_first_imu_estimation_ = false;
    // [testwheel]
    double wheel_speed_obs_ = 0.0;
    double can_speed_double_ = 0.0;
    bool have_new_wheel_speed_ = false;

    struct WheelSpeedSample {
        double timestamp = 0.0; // CAN 时间戳
        double wheel_speed = 0.0; // 轮速观测 m/s
    };
    std::deque<WheelSpeedSample> wheel_speed_buf_;
    double wheel_v_mean_ = 0.0;  // 当前帧内的平均轮速
    bool wheel_valid_this_frame_ = false; // 当前帧是否有有效轮速观测
    bool use_wheel_1d_ = true; // 是否使用 1D 轮速模型
    double wheel_sigma_ = 0.25; // 轮速模型噪声标准差 m/s
    double wheel_huber_delta_ = 0.8; // huber 阈值 m/s 残差超限自动降权
    double wheel_min_speed_ = 0.5; // 区间均速低于此值不更新
    double wheel_max_age_ = 0.15; // 轮速观测最大年龄（秒）；超过此值则认为过期



    double hc_heading_ = 0.0;
    double hc_pitch_ = 0.0;
    double hc_roll_ = 0.0;
    bool have_new_gnss_ = false;
    bool first_gnss_ = true;
    double hc_heading_b_ = 0.0;
    double hc_pitch_b_ = 0.0;
    double hc_roll_b_ = 0.0;

    std::deque<double> window_;
    double sum_ = 0.0;
    int window_size_ = 50;
};

}  // namespace slam_odometry