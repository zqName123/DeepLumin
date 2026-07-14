#include "localization/slam/laser_mapping_core.hpp"

#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <execution>
#include <iomanip>
#include <memory>
#include <mutex>

#include "common_lib.h"
#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "use-ikfom.hpp"
#include "utils.h"

namespace localization {
namespace {

using namespace faster_lio;

#ifdef IVOX_NODE_TYPE_PHC
using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

struct LogEveryN {
    explicit LogEveryN(int n) : interval(n) {}
    bool hit() { return interval > 0 && (++count % interval) == 0; }

    int interval;
    int count = 0;
};

/*
    将Eigen::Vector3d类型的imu数据转换为fasterlio需要的数据类型ImuSample，定义在第三方库faster_lio中
*/
ImuSamplePtr toImuSample(const ImuData& imu) {
    auto sample = std::make_shared<ImuSample>();
    sample->timestamp = imu.timestamp;
    sample->angular_velocity.x = imu.gyro.x();
    sample->angular_velocity.y = imu.gyro.y();
    sample->angular_velocity.z = imu.gyro.z();
    sample->linear_acceleration.x = imu.accel.x();
    sample->linear_acceleration.y = imu.accel.y();
    sample->linear_acceleration.z = imu.accel.z();
    return sample;
}

void logIkfState(const char* stage, std::uint64_t sync_id, std::uint64_t scan_cb_id,
                   const state_ikfom& s) {
    LOG(INFO) << "[SLAM][Align][State][" << stage << "] sync=#" << sync_id << " feed_cb=#" << scan_cb_id
              << " pos=(" << s.pos.transpose() << ")"
              << " vel=(" << s.vel.transpose() << ")"
              << " grav=(" << s.grav[0] << " " << s.grav[1] << " " << s.grav[2] << ")"
              << " bg=(" << s.bg.transpose() << ")";
}

void logMapInitFirstPoints(const char* tag, const PointCloudType& undistort, const PointCloudType& world) {
    const int n = std::min(5, static_cast<int>(undistort.size()));
    LOG(INFO) << "[SLAM][" << tag << "][MapInitPts] count=" << n << " total=" << undistort.size();
    for (int i = 0; i < n; ++i) {
        const auto& u = undistort.points[i];
        const auto& w = world.points[i];
        LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][MapInitPts] i=" << i
                  << " undistort=(" << u.x << " " << u.y << " " << u.z << ") curv_ms=" << u.curvature
                  << " world=(" << w.x << " " << w.y << " " << w.z << ")";
    }
}

void logObsModelDiag(const char* tag, int frame_num, std::uint64_t sync_id, int max_frames, int cnt_pts,
                     int effect_feat_num, const CloudPtr& scan_down_body, const CloudPtr& scan_down_world,
                     const std::vector<PointVector>& nearest_points, const std::vector<float>& residuals,
                     const std::vector<char>& point_selected_surf, const common::VV4F& plane_coef,
                     const esekfom::dyn_share_datastruct<double>& ekfom_data, bool& logged_this_iekf) {
    if (frame_num >= max_frames || logged_this_iekf || effect_feat_num < 1) {
        return;
    }
    logged_this_iekf = true;

    double abs_sum = 0.0;
    double sq_sum = 0.0;
    double max_abs = 0.0;
    int valid_cnt = 0;
    for (int i = 0; i < cnt_pts; ++i) {
        if (!point_selected_surf[i]) {
            continue;
        }
        const double r = residuals[i];
        abs_sum += std::abs(r);
        sq_sum += r * r;
        max_abs = std::max(max_abs, static_cast<double>(std::abs(r)));
        ++valid_cnt;
    }
    const double abs_mean = valid_cnt > 0 ? abs_sum / valid_cnt : 0.0;
    const double rms = valid_cnt > 0 ? std::sqrt(sq_sum / valid_cnt) : 0.0;

    LOG(INFO) << "[SLAM][" << tag << "][ObsDiag] sync=#" << sync_id << " iekf_frame=" << frame_num
              << " effect_feat_num=" << effect_feat_num << " valid_cnt=" << valid_cnt
              << " residual abs_mean=" << abs_mean << " rms=" << rms << " max=" << max_abs;

    int logged_pts = 0;
    int effect_idx = 0;
    for (int i = 0; i < cnt_pts && logged_pts < 5; ++i) {
        if (!point_selected_surf[i]) {
            continue;
        }
        const auto& pb = scan_down_body->points[i];
        const auto& pw = scan_down_world->points[i];
        const auto& pc = plane_coef[i];
        LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][ObsDiag] pt#" << logged_pts
                  << " scan_idx=" << i << " effect_idx=" << effect_idx << " body=(" << pb.x << " " << pb.y << " "
                  << pb.z << ") world=(" << pw.x << " " << pw.y << " " << pw.z << ") residual=" << residuals[i]
                  << " plane_coef=(" << pc[0] << " " << pc[1] << " " << pc[2] << " " << pc[3] << ")";
        const auto& near = nearest_points[i];
        const int near_n = std::min(5, static_cast<int>(near.size()));
        for (int k = 0; k < near_n; ++k) {
            LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][ObsDiag] pt#" << logged_pts
                      << " near[" << k << "]=(" << near[k].x << " " << near[k].y << " " << near[k].z << ")";
        }
        if (effect_idx < ekfom_data.h_x.rows()) {
            const auto hx = ekfom_data.h_x.row(effect_idx);
            LOG(INFO) << std::fixed << std::setprecision(6) << "[SLAM][" << tag << "][ObsDiag] pt#" << logged_pts
                      << " h_x[0:5]=(" << hx(0) << " " << hx(1) << " " << hx(2) << " " << hx(3) << " " << hx(4)
                      << " " << hx(5) << ") h=" << ekfom_data.h(effect_idx);
        }
        ++logged_pts;
        ++effect_idx;
    }
}

void logPDiagonal(const char* tag, const char* stage, std::uint64_t sync_id, int iekf_frame,
                const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf) {
    const auto P = kf.get_P();
    const Eigen::Matrix<double, 23, 1> d = P.diagonal();
    LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][" << tag << "][P][" << stage << "] sync=#" << sync_id
              << " iekf_frame=" << iekf_frame << " pos=(" << d[0] << " " << d[1] << " " << d[2] << ") rot=(" << d[3]
              << " " << d[4] << " " << d[5] << ") off_R=(" << d[6] << " " << d[7] << " " << d[8] << ") off_T=("
              << d[9] << " " << d[10] << " " << d[11] << ") vel=(" << d[12] << " " << d[13] << " " << d[14] << ") bg=("
              << d[15] << " " << d[16] << " " << d[17] << ") ba=(" << d[18] << " " << d[19] << " " << d[20] << ") grav=("
              << d[21] << " " << d[22] << ")";
    LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][" << tag << "][P][" << stage << "] P_diag_all=("
              << d.transpose() << ")";
}

}  // namespace

struct LaserMappingCore::Impl {
    IVoxType::Options ivox_options;
    std::shared_ptr<IVoxType> ivox;
    std::shared_ptr<ImuProcess> imu_process;

    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;

    float det_range = 300.0f;
    double cube_len = 2000.0;
    double filter_size_map_min = 0.2;
    bool localmap_initialized = false;
    bool extrinsic_est_en = false;

    Vec3d extrinT = Vec3d::Zero();
    Mat3d extrinR = Mat3d::Identity();

    CloudPtr scan_undistort{new PointCloudType()};
    CloudPtr scan_down_body{new PointCloudType()};
    CloudPtr scan_down_world{new PointCloudType()};

    std::vector<PointVector> nearest_points;
    common::VV4F corr_pts;
    common::VV4F corr_norm;
    pcl::VoxelGrid<PointType> voxel_scan;
    std::vector<float> residuals;
    // 必须用 char 而非 bool：par_unseq 中对相邻元素并发写入时，
    // std::vector<bool> 按位压缩导致同字节并发写 → 数据竞争（UB）。
    std::vector<char> point_selected_surf;
    common::VV4F plane_coef;

    std::mutex buffer_mutex;
    std::deque<double> time_buffer;
    std::deque<PointCloudType::Ptr> lidar_buffer;
    std::deque<ImuSamplePtr> imu_buffer;

    common::MeasureGroup measures;

    double last_timestamp_lidar = 0;
    double lidar_end_time = 0;
    double last_timestamp_imu = -1.0;
    double first_lidar_time = 0.0;
    bool lidar_pushed = false;

    int scan_count = 0;
    bool flg_first_scan = true;
    bool flg_ekf_inited = false;
    double lidar_mean_scantime = 0.0;
    int scan_num = 0;
    int effect_feat_num = 0;
    int frame_num = 0;
    // 验证阶段关闭：obs_hessian 并行累加有 data race，且非 ysw 原版逻辑
    static constexpr bool kEnableDegeneracyDiag = false;
    static constexpr int kObsDiagMaxIekfFrames = 3;
    bool obs_diag_logged_this_iekf = false;
    // ysw_loc StandardPCLCallBack: push when lidar_buffer_.size() <= 2（最多缓存 3 帧）
    static constexpr std::size_t kMaxLidarBufferSize = 3;

    std::uint64_t imu_feed_count = 0;
    std::uint64_t lidar_feed_count = 0;
    std::uint64_t lidar_drop_count = 0;
    std::uint64_t process_tick_count = 0;
    std::uint64_t sync_ok_count = 0;

    bool syncPackages() {
        if (lidar_buffer.empty() || imu_buffer.empty()) {
            static LogEveryN throttle(3000);
            if (throttle.hit()) {
                LOG(INFO) << "[SLAM][Core][Sync] wait buffer: lidar_buf=" << lidar_buffer.size()
                          << " imu_buf=" << imu_buffer.size()
                          << " last_imu_ts=" << last_timestamp_imu;
            }
            return false;
        }

        if (!lidar_pushed) {
            measures.lidar_ = lidar_buffer.front();
            measures.lidar_bag_time_ = time_buffer.front();

            const double back_curv_s = measures.lidar_->points.empty()
                                           ? 0.0
                                           : measures.lidar_->points.back().curvature / double(1000);
            const char* sync_branch = "unknown";

            if (measures.lidar_->points.size() <= 1) {
                LOG(WARNING) << "[SLAM][Core][Sync] too few input points: "
                             << measures.lidar_->points.size();
                lidar_end_time = measures.lidar_bag_time_ + lidar_mean_scantime;
                sync_branch = "few_pts_use_mean";
            } else if (back_curv_s < 0.5 * lidar_mean_scantime) {
                lidar_end_time = measures.lidar_bag_time_ + lidar_mean_scantime;
                sync_branch = "use_mean_scantime";
            } else {
                scan_num++;
                lidar_end_time = measures.lidar_bag_time_ + back_curv_s;
                lidar_mean_scantime += (back_curv_s - lidar_mean_scantime) / scan_num;
                sync_branch = "use_back_curvature";
            }

            measures.lidar_end_time_ = lidar_end_time;
            lidar_pushed = true;

            static LogEveryN throttle(50);
            if (throttle.hit()) {
                LOG(INFO) << "[SLAM][Core][Sync] branch=" << sync_branch << " mean_scantime=" << lidar_mean_scantime
                          << " back_curv_ms=" << (back_curv_s * 1000.0)
                          << " scan_dt=" << (lidar_end_time - measures.lidar_bag_time_);
            }
        }

        if (last_timestamp_imu < lidar_end_time) {
            static LogEveryN throttle(3000);
            if (throttle.hit()) {
                LOG(INFO) << "[SLAM][Core][Sync] wait IMU: last_imu=" << last_timestamp_imu
                          << " lidar_end=" << lidar_end_time
                          << " gap=" << (lidar_end_time - last_timestamp_imu) << "s";
            }
            return false;
        }

        double imu_time = imu_buffer.front()->timestamp;
        measures.imu_.clear();
        while (!imu_buffer.empty() && imu_time < lidar_end_time) {
            imu_time = imu_buffer.front()->timestamp;
            if (imu_time > lidar_end_time) {
                break;
            }
            measures.imu_.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;

        ++sync_ok_count;
        const double imu_first_ts =
            measures.imu_.empty() ? -1.0 : imuTimestamp(measures.imu_.front());
        const double imu_last_ts = measures.imu_.empty() ? -1.0 : imuTimestamp(measures.imu_.back());
        LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][Align][Sync] OK #" << sync_ok_count
                  << " feed_cb=#" << scan_count << " lidar_t=" << measures.lidar_bag_time_
                  << " lidar_end=" << lidar_end_time << " scan_dt=" << (lidar_end_time - measures.lidar_bag_time_)
                  << " mean_scantime=" << lidar_mean_scantime << " lidar_pts=" << measures.lidar_->points.size()
                  << " imu_used=" << measures.imu_.size() << " imu_first_ts=" << imu_first_ts
                  << " imu_last_ts=" << imu_last_ts << " imu_buf_left=" << imu_buffer.size()
                  << " lidar_buf_left=" << lidar_buffer.size();
        return true;
    }

    common::V3D euler_cur = common::V3D::Zero();
    common::V3D pos_lidar = common::V3D::Zero();

    Eigen::Matrix<double, 6, 6> obs_hessian = Eigen::Matrix<double, 6, 6>::Zero();
    double degeneracy_threshold = 100.0;

    void pointBodyToWorld(const PointType* pi, PointType* po) const {
        common::V3D p_body(pi->x, pi->y, pi->z);
        common::V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) +
                             state_point.pos);

        po->x = static_cast<float>(p_global(0));
        po->y = static_cast<float>(p_global(1));
        po->z = static_cast<float>(p_global(2));
        po->intensity = pi->intensity;
    }

    void mapIncremental() {
        PointVector points_to_add;
        PointVector point_no_need_downsample;

        const int cur_pts = static_cast<int>(scan_down_body->size());
        points_to_add.reserve(cur_pts);
        point_no_need_downsample.reserve(cur_pts);

        std::vector<size_t> index(cur_pts);
        for (size_t i = 0; i < static_cast<size_t>(cur_pts); ++i) {
            index[i] = i;
        }

        std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t i) {
            pointBodyToWorld(&(scan_down_body->points[i]), &(scan_down_world->points[i]));

            PointType& point_world = scan_down_world->points[i];
            if (!nearest_points[i].empty() && flg_ekf_inited) {
                const PointVector& points_near = nearest_points[i];

                Eigen::Vector3f center =
                    ((point_world.getVector3fMap() / static_cast<float>(filter_size_map_min)).array().floor() + 0.5f) *
                    static_cast<float>(filter_size_map_min);

                Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

                if (fabs(dis_2_center.x()) > 0.5f * static_cast<float>(filter_size_map_min) &&
                    fabs(dis_2_center.y()) > 0.5f * static_cast<float>(filter_size_map_min) &&
                    fabs(dis_2_center.z()) > 0.5f * static_cast<float>(filter_size_map_min)) {
                    point_no_need_downsample.emplace_back(point_world);
                    return;
                }

                bool need_add = true;
                const float dist = common::calc_dist(point_world.getVector3fMap(), center);
                if (points_near.size() >= static_cast<size_t>(options::NUM_MATCH_POINTS)) {
                    for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                        if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6f) {
                            need_add = false;
                            break;
                        }
                    }
                }
                if (need_add) {
                    points_to_add.emplace_back(point_world);
                }
            } else {
                points_to_add.emplace_back(point_world);
            }
        });

        ivox->AddPoints(points_to_add);
        ivox->AddPoints(point_no_need_downsample);
    }

    void obsModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
        const int cnt_pts = static_cast<int>(scan_down_body->size());

        std::vector<size_t> index(cnt_pts);
        for (size_t i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        obs_hessian.setZero();

        Timer::Evaluate(
            [&, this]() {
                const auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
                const auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

                std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t i) {
                    PointType& point_body = scan_down_body->points[i];
                    PointType& point_world = scan_down_world->points[i];

                    common::V3F p_body = point_body.getVector3fMap();
                    point_world.getVector3fMap() = R_wl * p_body + t_wl;
                    point_world.intensity = point_body.intensity;

                    auto& points_near = nearest_points[i];
                    if (ekfom_data.converge) {
                        ivox->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                        point_selected_surf[i] =
                            (points_near.size() >= static_cast<size_t>(options::MIN_NUM_MATCH_POINTS)) ? 1 : 0;
                        if (point_selected_surf[i]) {
                            point_selected_surf[i] =
                                common::esti_plane(plane_coef[i], points_near, options::ESTI_PLANE_THRESHOLD) ? 1 : 0;
                        }
                    }

                    if (point_selected_surf[i]) {
                        auto temp = point_world.getVector4fMap();
                        temp[3] = 1.0f;
                        const float pd2 = plane_coef[i].dot(temp);

                        const bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                        if (valid_corr) {
                            point_selected_surf[i] = 1;
                            residuals[i] = pd2;
                        } else {
                            // 对应量不符合质量要求，必须清除标志；原始 Faster-LIO 此处有 else 分支，
                            // 遗漏会导致旧残差/标志被带入下一轮 IEKF，产生系统性漂移。
                            point_selected_surf[i] = 0;
                        }
                    }
                });
            },
            "    ObsModel (Lidar Match)");

        effect_feat_num = 0;

        corr_pts.resize(cnt_pts);
        corr_norm.resize(cnt_pts);
        for (int i = 0; i < cnt_pts; i++) {
            if (point_selected_surf[i]) {
                corr_norm[effect_feat_num] = plane_coef[i];
                corr_pts[effect_feat_num] = scan_down_body->points[i].getVector4fMap();
                corr_pts[effect_feat_num][3] = residuals[i];
                effect_feat_num++;
            }
        }
        corr_pts.resize(effect_feat_num);
        corr_norm.resize(effect_feat_num);

        if (effect_feat_num < 1) {
            ekfom_data.valid = false;
            LOG(WARNING) << "[SLAM][Core][ObsModel] No Effective Points!";
            return;
        }

        Timer::Evaluate(
            [&, this]() {
                ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num, 12);
                ekfom_data.h.resize(effect_feat_num);

                index.resize(effect_feat_num);
                const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
                const common::V3F off_t = s.offset_T_L_I.cast<float>();
                const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

                std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t i) {
                    common::V3F point_this_be = corr_pts[i].head<3>();
                    common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                    common::V3F point_this = off_R * point_this_be + off_t;
                    common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                    common::V3F norm_vec = corr_norm[i].head<3>();

                    common::V3F C(Rt * norm_vec);
                    common::V3F A(point_crossmat * C);

                    if (extrinsic_est_en) {
                        common::V3F B(point_be_crossmat * off_R.transpose() * C);
                        ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                            B[0], B[1], B[2], C[0], C[1], C[2];
                    } else {
                        ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2],
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
                    }

                    ekfom_data.h(i) = -corr_pts[i][3];

                    if constexpr (kEnableDegeneracyDiag) {
                        Eigen::Matrix<double, 6, 1> A_i;
                        A_i << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];
                        obs_hessian += A_i * A_i.transpose();
                    }
                });
            },
            "    ObsModel (IEKF Build Jacobian)");

        logObsModelDiag("Align", frame_num, sync_ok_count, kObsDiagMaxIekfFrames, cnt_pts, effect_feat_num,
                        scan_down_body, scan_down_world, nearest_points, residuals, point_selected_surf,
                        plane_coef, ekfom_data, obs_diag_logged_this_iekf);
    }

    OdomResult buildOdomResult() const {
        OdomResult odom;
        odom.timestamp = lidar_end_time;
        odom.position = state_point.pos;
        odom.orientation = Quatd(state_point.rot.coeffs()[3], state_point.rot.coeffs()[0],
                                 state_point.rot.coeffs()[1], state_point.rot.coeffs()[2]);
        odom.velocity = state_point.vel;
        odom.valid = flg_ekf_inited;
        odom.frame_id = "odom";

        // 状态布局：pos[0:3], rot[3:6], offset_R_L_I[6:9], offset_T_L_I[9:12], vel[12:15]...
        // 旋转协方差应取 P(3:6,3:6)，原实现误取 P(6:9,6:9)（外参旋转），导致融合层低估姿态不确定性。
        const auto P = kf.get_P();
        odom.covariance.block<3, 3>(0, 0) = P.block<3, 3>(0, 0);
        odom.covariance.block<3, 3>(3, 3) = P.block<3, 3>(3, 3);

        return odom;
    }

    std::pair<bool, double> computeDegeneracy() const {
        if (obs_hessian.isZero(1e-9)) {
            return {false, 0.0};
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(obs_hessian);
        const auto eigenvalues = solver.eigenvalues();
        const double min_ev = eigenvalues.minCoeff();
        const double max_ev = eigenvalues.maxCoeff();
        if (max_ev < 1e-9) {
            return {true, 1.0};
        }
        const double ratio = min_ev / max_ev;
        const bool degenerate = min_ev < degeneracy_threshold || ratio < 0.05;
        const double factor = degenerate ? std::min(1.0, std::max(0.0, 1.0 - ratio * 10.0)) : 0.0;
        return {degenerate, factor};
    }
};

LaserMappingCore::LaserMappingCore() : impl_(std::make_unique<Impl>()) {}

LaserMappingCore::~LaserMappingCore() = default;

bool LaserMappingCore::initialize(const SlamConfig& cfg) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = cfg;
    impl_ = std::make_unique<Impl>();

    options::NUM_MAX_ITERATIONS = cfg.max_iteration;
    options::ESTI_PLANE_THRESHOLD = static_cast<float>(cfg.esti_plane_threshold);
    impl_->degeneracy_threshold = cfg.degeneracy_threshold;
    impl_->det_range = static_cast<float>(cfg.det_range);
    impl_->cube_len = cfg.cube_side_length;
    impl_->filter_size_map_min = cfg.filter_size_map;
    impl_->extrinsic_est_en = cfg.estimate_extrinsic;
    impl_->extrinT = cfg.extrinsic_T;
    impl_->extrinR = cfg.extrinsic_R;

    impl_->ivox_options.resolution_ = static_cast<float>(cfg.ivox_grid_resolution);
    switch (cfg.ivox_nearby_type) {
        case 0:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::CENTER;
            break;
        case 6:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY6;
            break;
        case 26:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY26;
            break;
        case 18:
        default:
            impl_->ivox_options.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            break;
    }

    impl_->ivox = std::make_shared<IVoxType>(impl_->ivox_options);
    impl_->imu_process = std::make_shared<ImuProcess>();
    // 与 ysw_loc faster-lio LaserMapping::lidar_mean_scantime_ 初值一致（0.0，由 SyncPackages 在线估计）
    impl_->lidar_mean_scantime = 0.0;

    impl_->voxel_scan.setLeafSize(static_cast<float>(cfg.filter_size_surf),
                                  static_cast<float>(cfg.filter_size_surf),
                                  static_cast<float>(cfg.filter_size_surf));

    impl_->imu_process->SetExtrinsic(cfg.extrinsic_T, cfg.extrinsic_R);
    impl_->imu_process->SetGyrCov(common::V3D(cfg.gyr_cov, cfg.gyr_cov, cfg.gyr_cov));
    impl_->imu_process->SetAccCov(common::V3D(cfg.acc_cov, cfg.acc_cov, cfg.acc_cov));
    impl_->imu_process->SetGyrBiasCov(common::V3D(cfg.b_gyr_cov, cfg.b_gyr_cov, cfg.b_gyr_cov));
    impl_->imu_process->SetAccBiasCov(common::V3D(cfg.b_acc_cov, cfg.b_acc_cov, cfg.b_acc_cov));

    std::vector<double> epsi(23, 0.001);
    impl_->kf.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
            impl_->obsModel(s, ekfom_data);
        },
        options::NUM_MAX_ITERATIONS, epsi.data());

    odom_ = OdomResult();
    odom_.frame_id = "odom";
    local_map_.cloud.reset(new PointCloud());
    current_scan_.cloud.reset(new PointCloud());
    initialized_ = true;
    is_degenerate_ = false;
    degeneracy_factor_ = 0.0;

    LOG(INFO) << "[SLAM][Core][Init] OK type=" << cfg.type << " lidar_type=" << cfg.lidar_type
              << " scan_rate=" << cfg.scan_rate << " blind=" << cfg.blind
              << " point_filter=" << cfg.point_filter_num << " ivox_res=" << cfg.ivox_grid_resolution
              << " max_iter=" << cfg.max_iteration << " filter_size_map=" << cfg.filter_size_map
              << " extrinsic_est=" << cfg.estimate_extrinsic << " scan_period=" << impl_->lidar_mean_scantime
              << " LASER_POINT_COV=" << options::LASER_POINT_COV
              << " NUM_MAX_ITERATIONS=" << options::NUM_MAX_ITERATIONS;

    return true;
}

/**
    处理IMU数据
*/
void LaserMappingCore::feedImu(const ImuData& imu) {
    if (!initialized_) {
        LOG(WARNING) << "[SLAM][Core][FeedImu] skip: not initialized";
        return;
    }
    // 转化为fasterlio需要的类型
    const auto sample = toImuSample(imu);
    // 持锁，将imu数据加入到imu_buffer中
    std::lock_guard<std::mutex> lock(impl_->buffer_mutex);
    if (sample->timestamp < impl_->last_timestamp_imu) { // 数据产生回退，清除buffer
        LOG(WARNING) << "[SLAM][Core][FeedImu] loop back, clear buffer";
        impl_->imu_buffer.clear();
    }
    impl_->last_timestamp_imu = sample->timestamp;
    impl_->imu_buffer.emplace_back(sample); // 数据入队
    ++impl_->imu_feed_count;  // 队列中的imu数据量
    if (impl_->imu_feed_count == 1 || impl_->imu_feed_count % 1000 == 0) {
        LOG(INFO) << "[SLAM][Core][FeedImu] #" << impl_->imu_feed_count << " ts=" << sample->timestamp
                  << " buf=" << impl_->imu_buffer.size();
    }
}

void LaserMappingCore::feedLidar(const TimestampedPointCloud& pc) {
    if (!pc.cloud || pc.cloud->empty()) {
        LOG(WARNING) << "[SLAM][Align][FeedLidar] skip: empty cloud ts=" << pc.timestamp;
        return;
    }
    feedLidarCloud(pc.timestamp, pc.cloud);
}


void LaserMappingCore::feedLidarCloud(double timestamp, PointCloudPtr cloud) {
    if (!initialized_) {
        LOG(WARNING) << "[SLAM][Align][FeedLidar] skip: not initialized";
        return;
    }
    if (!cloud || cloud->empty()) {
        LOG(WARNING) << "[SLAM][Align][FeedLidar] skip: empty cloud ts=" << timestamp;
        return;
    }

    // 对齐 ysw StandardPCLCallBack：持锁 preprocess 后入队；scan_count 每回调 +1（含 buffer 满丢弃）
    std::lock_guard<std::mutex> lock(impl_->buffer_mutex);
    ++impl_->scan_count;

    if (timestamp < impl_->last_timestamp_lidar) {
        LOG(ERROR) << "[SLAM][Align][FeedLidar] loop back, clear buffer";
        impl_->lidar_buffer.clear();
        impl_->time_buffer.clear();
    }

    if (impl_->lidar_buffer.size() <= Impl::kMaxLidarBufferSize - 1) {
        impl_->lidar_buffer.push_back(cloud);
        impl_->time_buffer.push_back(timestamp);
        impl_->last_timestamp_lidar = timestamp;
        ++impl_->lidar_feed_count;
        const float curv_max = cloud->points.back().curvature;
        LOG(INFO) << "[SLAM][Align][FeedLidar] #" << impl_->scan_count << " ts=" << timestamp
                  << " pts=" << cloud->size() << " curv_max_ms=" << curv_max
                  << " buf=" << impl_->lidar_buffer.size();
    } else {
        ++impl_->lidar_drop_count;
        LOG(WARNING) << "[SLAM][Align][FeedLidar] buffer full, drop cb=#" << impl_->scan_count
                       << " buf=" << impl_->lidar_buffer.size();
    }
}

bool LaserMappingCore::processOnce() {
    if (!initialized_) {
        LOG(WARNING) << "[SLAM][Core][Process] skip: not initialized";
        return false;
    }

    // 对齐 ysw LaserMapping::Run() 每帧顺序：
    //   SyncPackages → ImuProcess → [首帧 ivox 建图 return] → downsample → IEKF → MapIncremental
    // IMU 初始化阶段：ImuProcess 不 undistort，scan 空则 return（lidar 已在 Sync 中 pop）
    ++impl_->process_tick_count;

    {
        std::lock_guard<std::mutex> buf_lock(impl_->buffer_mutex);
        if (!impl_->syncPackages()) {
            return false;
        }
    }

    LOG(INFO) << "[SLAM][Core][Process] tick=" << impl_->process_tick_count
              << " -> ImuProcess (imu_n=" << impl_->measures.imu_.size() << ")";

    logIkfState("before_ImuProcess", impl_->sync_ok_count, impl_->scan_count,
                impl_->kf.get_x());

    // IMU 为空或仍在初始化时，Process() 会提前返回而不修改 scan_undistort，
    // 若不先清空则会复用上一帧遗留的点云，导致错误匹配和漂移。
    impl_->scan_undistort->clear();
    impl_->imu_process->Process(impl_->measures, impl_->kf, impl_->scan_undistort);
    if (impl_->scan_undistort->empty()) {
        LOG(WARNING) << "[SLAM][Core][Process] ImuProcess done but scan empty (likely IMU init), imu_n="
                     << impl_->measures.imu_.size();
        return false;
    }

    logIkfState("after_ImuProcess", impl_->sync_ok_count, impl_->scan_count, impl_->kf.get_x());
    if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
        logPDiagonal("Align", "after_ImuProcess", impl_->sync_ok_count, impl_->frame_num, impl_->kf);
    }

    if (impl_->flg_first_scan) {
        impl_->state_point = impl_->kf.get_x();
        impl_->scan_down_world->resize(impl_->scan_undistort->size());
        for (size_t i = 0; i < impl_->scan_undistort->size(); i++) {
            impl_->pointBodyToWorld(&impl_->scan_undistort->points[i], &impl_->scan_down_world->points[i]);
        }
        impl_->ivox->AddPoints(impl_->scan_down_world->points);
        impl_->first_lidar_time = impl_->measures.lidar_bag_time_;
        impl_->flg_first_scan = false;
        LOG(INFO) << "[SLAM][Align][MapInit] first_scan sync=#" << impl_->sync_ok_count
                  << " feed_cb=#" << impl_->scan_count
                  << " lidar_t=" << impl_->measures.lidar_bag_time_
                  << " pts=" << impl_->scan_undistort->size() << " t0=" << impl_->first_lidar_time;
        logIkfState("first_scan_map", impl_->sync_ok_count, impl_->scan_count, impl_->state_point);
        logMapInitFirstPoints("Align", *impl_->scan_undistort, *impl_->scan_down_world);
        // 对齐 ysw Run()：首帧只建图，不 IEKF、不 publish、不更新 odom_
        return false;
    }

    impl_->obs_diag_logged_this_iekf = false;
    if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
        LOG(INFO) << "[SLAM][Align][ObsDiag] >>> IEKF start sync=#" << impl_->sync_ok_count
                  << " iekf_frame=" << impl_->frame_num << " feed_cb=#" << impl_->scan_count;
    }
    const double lidar_dt = impl_->measures.lidar_bag_time_ - impl_->first_lidar_time;
    impl_->flg_ekf_inited = lidar_dt >= options::INIT_TIME;

    Timer::Evaluate(
        [this]() {
            impl_->voxel_scan.setInputCloud(impl_->scan_undistort);
            impl_->voxel_scan.filter(*impl_->scan_down_body);
        },
        "Downsample PointCloud");

    const int cur_pts = static_cast<int>(impl_->scan_down_body->size());
    if (cur_pts < 5) {
        LOG(WARNING) << "[SLAM][Core][Process] Too few downsampled points: undistort="
                     << impl_->scan_undistort->size() << " down=" << cur_pts;
        return false;
    }

    LOG(INFO) << std::fixed << std::setprecision(9) << "[SLAM][Align][Process] downsample "
              << impl_->scan_undistort->size() << " -> " << cur_pts << " ekf_inited=" << impl_->flg_ekf_inited
              << " lidar_t=" << impl_->measures.lidar_bag_time_ << " first_lidar_t=" << impl_->first_lidar_time
              << " delta_t=" << lidar_dt << " INIT_TIME=" << options::INIT_TIME;

    impl_->scan_down_world->resize(cur_pts);
    impl_->nearest_points.resize(cur_pts);
    impl_->residuals.resize(cur_pts, 0);
    impl_->point_selected_surf.resize(cur_pts, 1);
    impl_->plane_coef.resize(cur_pts, common::V4F::Zero());

    Timer::Evaluate(
        [this]() {
            double solve_H_time = 0;
            if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
                logPDiagonal("Align", "before_IEKF", impl_->sync_ok_count, impl_->frame_num, impl_->kf);
                impl_->kf.set_iteration_diag(true);
            }
            impl_->kf.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            impl_->kf.set_iteration_diag(false);
            impl_->state_point = impl_->kf.get_x();
            impl_->euler_cur = SO3ToEuler(impl_->state_point.rot);
            impl_->pos_lidar =
                impl_->state_point.pos + impl_->state_point.rot * impl_->state_point.offset_T_L_I;
        },
        "IEKF Solve and Update");

    logIkfState("after_IEKF", impl_->sync_ok_count, impl_->scan_count, impl_->state_point);
    if (impl_->frame_num < Impl::kObsDiagMaxIekfFrames) {
        logPDiagonal("Align", "after_IEKF", impl_->sync_ok_count, impl_->frame_num, impl_->kf);
    }
    if (impl_->frame_num < 10) {
        LOG(INFO) << "[SLAM][Align][Process] early_pos frame #" << impl_->frame_num << " sync=#"
                  << impl_->sync_ok_count << " feed_cb=#" << impl_->scan_count
                  << " lidar_t=" << impl_->measures.lidar_bag_time_ << " pts="
                  << impl_->measures.lidar_->size() << " pos=(" << impl_->state_point.pos.transpose()
                  << ")";
    }

    Timer::Evaluate([this]() { impl_->mapIncremental(); }, "    Incremental Mapping");

    if constexpr (Impl::kEnableDegeneracyDiag) {
        const auto degeneracy = impl_->computeDegeneracy();
        is_degenerate_ = degeneracy.first;
        degeneracy_factor_ = degeneracy.second;
    } else {
        is_degenerate_ = false;
        degeneracy_factor_ = 0.0;
    }

    odom_ = impl_->buildOdomResult();
    odom_.is_degenerate = is_degenerate_;
    odom_.degeneracy_factor = degeneracy_factor_;

    // 返回 LiDAR body 系去畸变点云供关键帧存储（insertKeyFrame 期望 cloud_body）。
    // 注意：不转换到世界系，下游使用时以 odom_.pose 做外部变换；
    // 如需发布世界系，由调用方（如 slam_debug_node）自行用 odom 变换。
    current_scan_.timestamp = impl_->lidar_end_time;
    current_scan_.cloud.reset(new PointCloud(*impl_->scan_undistort));

    local_map_.timestamp = impl_->lidar_end_time;
    local_map_.cloud.reset(new PointCloud(*impl_->scan_down_world));

    impl_->frame_num++;
    LOG(INFO) << "[SLAM][Core][Process] frame #" << impl_->frame_num << " OK odom_valid=" << odom_.valid
              << " [mapping] In=" << impl_->scan_undistort->points.size() << " downsamp=" << cur_pts
              << " MapGrid=" << impl_->ivox->NumValidGrids() << " effect=" << impl_->effect_feat_num
              << " degenerate=" << (Impl::kEnableDegeneracyDiag ? (is_degenerate_ ? 1 : 0) : -1);

    return true;
}

void LaserMappingCore::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        initialize(config_);
    }
}

OdomResult LaserMappingCore::getOdometry() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return odom_;
}

TimestampedPointCloud LaserMappingCore::getLocalMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return local_map_;
}

TimestampedPointCloud LaserMappingCore::getCurrentScan() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_scan_;
}

bool LaserMappingCore::isDegenerate() const {
    return is_degenerate_;
}

double LaserMappingCore::getDegeneracyFactor() const {
    return degeneracy_factor_;
}

}  // namespace localization
