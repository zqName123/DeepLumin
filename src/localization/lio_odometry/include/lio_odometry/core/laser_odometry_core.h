#pragma once

#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ivox3d/ivox3d.h"
#include "lio_odometry/core/eskf.h"
#include "lio_odometry/core/imu_processing.h"
#include "lio_odometry/core/pointcloud_preprocess.h"
#include "lio_odometry/core/types.h"
#include "syslog/logger.h"
#include "lio_odometry/core/eskf.h"

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

/// 缓冲区治理策略（Stage 2）。
/// 问题：lidar_buffer_/time_buffer_/imu_buffer_ 无界，处理跟不上时单调增长，
///       且 SyncPackages 每 tick 只弹 1 帧，掉实时后永远播旧账。
/// 策略：时间裁剪为主 + 条数兜底 + drop-to-latest 追帧；LiDAR/IMU 联合裁剪。
/// enable=false 时 PruneBuffers() 直接返回，等于不存在（运行时回退）。
/// 详见 src/localization/docs/odom_buffer_memory_fix.md
struct BufferPolicyConfig {
    bool enable = true;               ///< false → 旧行为（无界缓冲）
    double lidar_keep_sec = 0.8;      ///< LiDAR 时间窗
    int lidar_max_frames = 10;        ///< LiDAR 条数硬上限（时间戳冻结时兜底）
    int lidar_catchup_frames = 3;     ///< 超过即 drop-to-latest，只留最新一帧
    double imu_keep_margin_sec = 0.2; ///< IMU 下界 = 最旧 LiDAR 时间 - margin
    int imu_max_samples = 3000;       ///< IMU 条数硬上限
    double imu_orphan_keep_sec = 2.0; ///< LiDAR 断流时 IMU 相对自身的保留窗
};

/// PruneBuffers() 的丢弃统计，供限频日志
struct BufferDropCount {
    int lidar = 0;
    int imu = 0;
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
    void FeedGnssAttitude(double timestamp, double heading_deg, double pitch_deg, double roll_deg, int status, int nsv);

    const NavState& GetState() const { return state_point_; }
    const NavState& GetIMUState() const;
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
    // 按激光帧区间消费 GNSS 角度队列，产出 heading_deg, pitch_deg, roll_deg
    bool ConsumeGnssForFrame(double t_end);
    void ApplyGnssRaw(double heading_deg, double pitch_deg, double roll_deg);

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

    /// 缓冲区治理（Stage 2）：时间裁剪 + 条数兜底 + drop-to-latest。
    /// 必须在持有 mtx_buffer_ 时调用（SyncPackages 开头）。
    /// 丢帧时会作废 lidar_pushed_ 半状态并断开 measures_.scan_ 引用。
    BufferDropCount PruneBuffers();

    double filter(double new_value);

   private:
    LaserOdometryOptions options_;

    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;

    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
    faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;

    double filter_size_map_min_ = 0;

    std::vector<double> extrinT_{3, 0.0};
    std::vector<double> extrinR_{9, 0.0};

    CloudPtr scan_undistort_{new PointCloudType()};
    CloudPtr scan_down_body_{new PointCloudType()};
    CloudPtr scan_down_world_{new PointCloudType()};
    std::vector<PointVector> nearest_points_;
    std::vector<Vec4f> corr_pts_;
    std::vector<Vec4f> corr_norm_;
    pcl::VoxelGrid<PointType> voxel_scan_;

    std::vector<float> residuals_;
    std::vector<uint8_t> point_selected_surf_;
    std::vector<Vec4f> plane_coef_;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    std::deque<CloudPtr> lidar_buffer_;
    std::deque<IMUPtr> imu_buffer_;

    /// 缓冲区治理策略与丢弃计数（Stage 2，见 docs/odom_buffer_memory_fix.md）
    BufferPolicyConfig buffer_policy_;
    std::uint64_t dropped_lidar_frames_ = 0;
    std::uint64_t dropped_imu_samples_ = 0;

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

    double map_insert_min_speed_ = 0.2; // 地图插入最小速度（m/s）；低于此值不插入地图
    int map_skip_frames_ = 10; // 地图插入跳帧帧数；每 N 帧插入一次地图
    int restart_cnt_ = 0; // 重启计数器；超过此值则重启地图
    int stationary_cnt_ = 0; //gnss静止计数器；超过此值则认为静止
    int gnss_freeze_frames_ = 30;
    // >>>>> [GNSS-REZERO] BEGIN —— 停车期 GNSS 基准重零开关（回退：置 false 或删除）<<<<<
    bool gnss_rezero_enable_ = true;
    // >>>>> [GNSS-REZERO] END <<<<<

    MeasureGroup measures_;

    ESKF kf_;
    ESKF kf_imu_;

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

    struct GnssAttitudeSample {
        double timestamp = 0.0; // GNSS 时间戳
        double heading_deg = 0.0; //  heading 角度（度）
        double pitch_deg = 0.0; // pitch 角度（度）
        double roll_deg = 0.0; // roll 角度（度）
        int status = 0; // 状态码
        int nsv = 0; // 有效卫星数
    };
    std::deque<GnssAttitudeSample> gnss_buf_;



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