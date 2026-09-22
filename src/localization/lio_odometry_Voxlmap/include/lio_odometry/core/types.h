#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <vector>

#include <cfloat>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <deque>

#include "Sophus/se3.hpp"
#include "Sophus/se2.hpp"

namespace slam_odometry {

using IdType = size_t;

using SO2 = Sophus::SO2d;
using SE2 = Sophus::SE2d;
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Matrix<float, 4, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec7d = Eigen::Matrix<double, 7, 1>;
using Vec15d = Eigen::Matrix<double, 15, 1>;
using VecXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using Mat2f = Eigen::Matrix<float, 2, 2>;
using Mat3f = Eigen::Matrix<float, 3, 3>;
using Mat4f = Eigen::Matrix<float, 4, 4>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Mat4d = Eigen::Matrix<double, 4, 4>;
using Mat6d = Eigen::Matrix<double, 6, 6>;

using Mat12d = Eigen::Matrix<double, 1, 2>;
using Mat36f = Eigen::Matrix<float, 3, 6>;
using Mat23f = Eigen::Matrix<float, 2, 3>;
using Mat66f = Eigen::Matrix<float, 6, 6>;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

template <int N, typename T = double>
using VectorN = Eigen::Matrix<T, N, 1, Eigen::ColMajor>;
using Vector2 = VectorN<2>;
using Vector3 = VectorN<3>;
using Vector4 = VectorN<4>;
using Vector6 = VectorN<6>;
using Vector7 = VectorN<7>;
using VectorX = VectorN<Eigen::Dynamic>;

template <int N, typename T = double>
using MatrixN = Eigen::Matrix<T, N, N, Eigen::ColMajor>;
using Matrix2 = MatrixN<2>;
using Matrix3 = MatrixN<3>;
using Matrix4 = MatrixN<4>;
using MatrixX = MatrixN<Eigen::Dynamic>;

using H6d = Eigen::Matrix<double, 1, 6>;
using H1d = Eigen::Matrix<double, 1, 1>;

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;

using Aff3d = Eigen::Affine3d;
using Quatd = Eigen::Quaternion<double>;
using Trans3d = Eigen::Translation3d;
using AngAxisd = Eigen::AngleAxis<double>;
using Quat = Quatd;
using Aff3f = Eigen::Affine3f;
using Quatf = Eigen::Quaternion<float>;
using Trans3f = Eigen::Translation3f;
using AngAxisf = Eigen::AngleAxis<float>;
using SE3f = Sophus::SE3f;

// ============================================================================
// math utilities needed by S2
// ============================================================================
namespace math {

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T, typename Ts>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K = SKEW_SYM_MATRIX(r_axis);
        T r_ang = ang_vel_norm * dt;
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    } else {
        return Eye3;
    }
}

// Quaternion exponential: a rotation vector theta requires scale=0.5.
inline SO3 exp(const Vec3d& vec, double scale = 1.0) {
    const double angle = std::abs(scale) * vec.norm();
    const double sinc = angle < 1e-6 ? 1.0 - angle * angle / 6.0 : std::sin(angle) / angle;
    const Vec3d qv = (scale * sinc) * vec;
    return SO3(Quatd(std::cos(angle), qv.x(), qv.y(), qv.z()));
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> A_matrix(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> res;
    double squared_norm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squared_norm);
    if (norm < 1e-5) {
        res = Eigen::Matrix<T, 3, 3>::Identity();
    } else {
        res = Eigen::Matrix<T, 3, 3>::Identity() +
              (1 - std::cos(norm)) / squared_norm * SKEW_SYM_MATRIX(v) +
              (1 - std::sin(norm) / norm) / squared_norm * SKEW_SYM_MATRIX(v) * SKEW_SYM_MATRIX(v);
    }
    return res;
}

}  // namespace math

// ============================================================================
// S2: 球面流形，用于重力向量表示
// ============================================================================
struct S2 {
    static constexpr int den_ = 98090;
    static constexpr int num_ = 10000;
    static constexpr double length_ = double(den_) / double(num_);
    Vec3d vec_;

   public:
    S2() { vec_ = length_ * Vec3d(1, 0, 0); }
    S2(const Vec3d& vec) : vec_(vec) {
        vec_.normalize();
        vec_ = vec_ * length_;
    }

    double operator[](int idx) const { return vec_[idx]; }

    Eigen::Matrix<double, 3, 3> S2_hat() const {
        Eigen::Matrix<double, 3, 3> skew_vec;
        skew_vec << 0.0, -vec_[2], vec_[1], vec_[2], 0.0, -vec_[0], -vec_[1], vec_[0], 0.0;
        return skew_vec;
    }

    Eigen::Matrix<double, 3, 2> S2_Bx() const {
        Eigen::Matrix<double, 3, 2> res;
        if (vec_[0] + length_ > 1e-5) {
            res << -vec_[1], -vec_[2], length_ - vec_[1] * vec_[1] / (length_ + vec_[0]),
                -vec_[2] * vec_[1] / (length_ + vec_[0]), -vec_[2] * vec_[1] / (length_ + vec_[0]),
                length_ - vec_[2] * vec_[2] / (length_ + vec_[0]);
            res /= length_;
        } else {
            res = Eigen::Matrix<double, 3, 2>::Zero();
            res(1, 1) = -1;
            res(2, 0) = 1;
        }
        return res;
    }

    Eigen::Matrix<double, 3, 2> S2_Mx(const Eigen::Matrix<double, 2, 1>& delta) const {
        Eigen::Matrix<double, 3, 2> res;
        Eigen::Matrix<double, 3, 2> Bx = S2_Bx();
        if (delta.norm() < 1e-5) {
            res = -SO3::hat(vec_) * Bx;
        } else {
            Vec3d Bu = Bx * delta;
            SO3 exp_delta = math::exp(Bu, 0.5);
            res = -exp_delta.matrix() * SO3::hat(vec_) * math::A_matrix(Bu).transpose() * Bx;
        }
        return res;
    }

    Eigen::Matrix<double, 2, 3> S2_Nx_yy() const {
        Eigen::Matrix<double, 2, 3> res;
        Eigen::Matrix<double, 3, 2> Bx = S2_Bx();
        res = 1.0 / length_ / length_ * Bx.transpose() * SO3::hat(vec_);
        return res;
    }

    void oplus(const Vec3d& delta, double scale = 1.0) {
        vec_ = math::exp(delta, scale * 0.5) * vec_;
    }

    Vec2d boxminus(const S2& other) const {
        Vec2d res;
        double v_sin = (SO3::hat(vec_) * other.vec_).norm();
        double v_cos = vec_.transpose() * other.vec_;
        double theta = std::atan2(v_sin, v_cos);
        if (v_sin < 1e-5) {
            res[0] = (std::fabs(theta) > 1e-5) ? 3.1415926 : 0.0;
            res[1] = 0.0;
        } else {
            S2 other_copy = other;
            Eigen::Matrix<double, 3, 2> Bx = other_copy.S2_Bx();
            res = theta / v_sin * Bx.transpose() * SO3::hat(other.vec_) * vec_;
        }
        return res;
    }

    void boxplus(const Vec2d& delta, double scale = 1.0) {
        Eigen::Matrix<double, 3, 2> Bx = S2_Bx();
        Vec3d tmp = Bx * delta;
        SO3 res = math::exp(tmp, scale / 2.0);
        vec_ = res.matrix() * vec_;
    }
};

// ============================================================================
// NavState: 23维 ESKF 状态变量
// ============================================================================
struct NavState {
    constexpr static int dim = 23;
    constexpr static int full_dim = 24;

    using VectState = Eigen::Matrix<double, dim, 1>;
    using FullVectState = Eigen::Matrix<double, full_dim, 1>;

    NavState() = default;

    bool operator<(const NavState& other) { return timestamp_ < other.timestamp_; }

    FullVectState ToState() {
        FullVectState ret;
        ret.block<3, 1>(0, 0) = pos_;
        ret.block<3, 1>(3, 0) = rot_.log();
        ret.block<3, 1>(6, 0) = offset_R_lidar_.log();
        ret.block<3, 1>(9, 0) = offset_t_lidar_;
        ret.block<3, 1>(12, 0) = vel_;
        ret.block<3, 1>(15, 0) = bg_;
        ret.block<3, 1>(18, 0) = ba_;
        ret.block<3, 1>(21, 0) = grav_.vec_;
        return ret;
    }

    void FromVectState(const FullVectState& state) {
        pos_ = state.block<3, 1>(0, 0);
        rot_ = SO3::exp(state.block<3, 1>(3, 0));
        offset_R_lidar_ = SO3::exp(state.block<3, 1>(6, 0));
        offset_t_lidar_ = state.block<3, 1>(9, 0);
        vel_ = state.block<3, 1>(12, 0);
        bg_ = state.block<3, 1>(15, 0);
        ba_ = state.block<3, 1>(18, 0);
        grav_.vec_ = state.block<3, 1>(21, 0);
    }

    inline FullVectState get_f(const Vec3d& gyro, const Vec3d& acce) const {
        FullVectState res = FullVectState::Zero();
        Vec3d omega = gyro - bg_;
        Vec3d a_inertial = rot_ * (acce - ba_);
        for (int i = 0; i < 3; i++) {
            res(i) = vel_[i];
            res(i + 3) = omega[i];
            res(i + 12) = a_inertial[i] + grav_[i];
        }
        return res;
    }

    inline Eigen::Matrix<double, full_dim, dim> df_dx(const Vec3d& acce) const {
        Eigen::Matrix<double, full_dim, dim> cov = Eigen::Matrix<double, full_dim, dim>::Zero();
        cov.block<3, 3>(0, 12) = Mat3d::Identity();
        Vec3d acc = acce - ba_;
        cov.block<3, 3>(12, 3) = -rot_.matrix() * SO3::hat(acc);
        cov.block<3, 3>(12, 18) = -rot_.matrix();

        Vec2d vec = Vec2d::Zero();
        Eigen::Matrix<double, 3, 2> grav_matrix = grav_.S2_Mx(vec);
        cov.block<3, 2>(12, 21) = grav_matrix;
        cov.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
        return cov;
    }

    inline Eigen::Matrix<double, 24, 12> df_dw() const {
        Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
        cov.block<3, 3>(12, 3) = -rot_.matrix();
        cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
        cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
        return cov;
    }

    void oplus(const FullVectState& vec, double dt) {
        timestamp_ += dt;
        pos_ += vec.middleRows(0, 3) * dt;
        rot_ = rot_ * SO3::exp(vec.middleRows(3, 3) * dt);
        offset_R_lidar_ = offset_R_lidar_ * SO3::exp(vec.middleRows(6, 3) * dt);
        offset_t_lidar_ = offset_t_lidar_ + vec.middleRows(9, 3) * dt;
        vel_ += vec.middleRows(12, 3) * dt;
        bg_ += vec.middleRows(15, 3) * dt;
        ba_ += vec.middleRows(18, 3) * dt;
        grav_.oplus(vec.middleRows(21, 3) * dt);
    }

    VectState boxminus(const NavState& other) {
        VectState result;
        result.block<3, 1>(0, 0) = pos_ - other.pos_;
        result.block<3, 1>(3, 0) = (other.rot_.inverse() * rot_).log();
        result.block<3, 1>(6, 0) = (other.offset_R_lidar_.inverse() * offset_R_lidar_).log();
        result.block<3, 1>(9, 0) = offset_t_lidar_ - other.offset_t_lidar_;
        result.block<3, 1>(12, 0) = vel_ - other.vel_;
        result.block<3, 1>(15, 0) = bg_ - other.bg_;
        result.block<3, 1>(18, 0) = ba_ - other.ba_;
        Vec2d dg = grav_.boxminus(other.grav_);
        result.block<2, 1>(21, 0) = dg;
        return result;
    }

    NavState boxplus(const VectState& dx) {
        NavState ret;
        ret.timestamp_ = timestamp_;
        ret.pos_ = pos_ + dx.middleRows(0, 3);
        ret.rot_ = rot_ * SO3::exp(dx.middleRows(3, 3));
        ret.offset_R_lidar_ = offset_R_lidar_ * SO3::exp(dx.middleRows(6, 3));
        ret.offset_t_lidar_ = offset_t_lidar_ + dx.middleRows(9, 3);
        ret.vel_ = vel_ + dx.middleRows(12, 3);
        ret.bg_ = bg_ + dx.middleRows(15, 3);
        ret.ba_ = ba_ + dx.middleRows(18, 3);
        ret.grav_ = grav_;
        ret.grav_.boxplus(dx.middleRows(21, 2));
        return ret;
    }

    struct MetaInfo {
        MetaInfo(int idx, int vdim, int dof) : idx_(idx), dim_(vdim), dof_(dof) {}
        int idx_ = 0;
        int dim_ = 0;
        int dof_ = 0;
    };

    static const std::vector<MetaInfo> vect_states_;
    static const std::vector<MetaInfo> SO3_states_;
    static const std::vector<MetaInfo> S2_states_;

    friend inline std::ostream& operator<<(std::ostream& os, const NavState& s) {
        os << std::setprecision(18) << s.pos_.transpose() << " " << s.rot_.unit_quaternion().coeffs().transpose()
           << " " << s.offset_R_lidar_.unit_quaternion().coeffs().transpose() << " " << s.offset_t_lidar_.transpose()
           << " " << s.vel_.transpose() << " " << s.bg_.transpose() << " " << s.ba_.transpose() << " "
           << s.grav_.vec_.transpose();
        return os;
    }

    inline SE3 GetPose() const { return SE3(rot_, pos_); }
    inline SO3 GetRot() const { return rot_; }
    inline void SetPose(const SE3& pose) {
        rot_ = pose.so3();
        pos_ = pose.translation();
    }

    inline Vec3d Getba() const { return ba_; }
    inline Vec3d Getbg() const { return bg_; }
    inline Vec3d GetVel() const { return vel_; }
    void SetVel(const Vec3d& v) { vel_ = v; }

    double timestamp_ = 0.0;
    double confidence_ = 0.0;
    bool pose_is_ok_ = true;
    bool lidar_odom_reliable_ = true;
    bool is_parking_ = false;

    Vec3d pos_ = Vec3d::Zero();
    SO3 rot_;
    SO3 offset_R_lidar_;
    Vec3d offset_t_lidar_ = Vec3d::Zero();
    Vec3d vel_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    S2 grav_;
};

// ============================================================================
// Pose6D: 时间对齐后的6D姿态
// ============================================================================
struct Pose6D {
    Pose6D() = default;

    Pose6D(const double t, const Vec3d& a, const Vec3d& g, const Vec3d& v, const Vec3d& p, const Mat3d& R) {
        offset_time = t;
        acc = a;
        gyr = g;
        vel = v;
        pos = p;
        rot = R;
    }

    double offset_time = 0;
    Vec3d acc = Vec3d::Zero();
    Vec3d gyr = Vec3d::Zero();
    Vec3d vel = Vec3d::Zero();
    Vec3d pos = Vec3d::Zero();
    Mat3d rot = Mat3d::Identity();
};

// ============================================================================
// Point Types (PCL)
// ============================================================================
struct PointXYZIT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    double time;
    PointXYZIT() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct VelodynePoint {
    PCL_ADD_POINT4D
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct OusterPoint {
    PCL_ADD_POINT4D
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t ambient;
    std::uint32_t range;
};

}  // namespace slam_odometry

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(slam_odometry::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(slam_odometry::VelodynePoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (float, time, time)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(slam_odometry::OusterPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range))
// clang-format on

namespace slam_odometry {

using PointType = PointXYZIT;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

struct IMU {
    double timestamp = 0;
    Vec3d angular_velocity = Vec3d::Zero();
    Vec3d linear_acceleration = Vec3d::Zero();
};
using IMUPtr = std::shared_ptr<IMU>;

struct MeasureGroup {
    double timestamp_ = 0;
    double lidar_begin_time_ = 0;
    double lidar_end_time_ = 0;

    std::deque<IMUPtr> imu_;
    CloudPtr scan_ = nullptr;
};

}  // namespace slam_odometry