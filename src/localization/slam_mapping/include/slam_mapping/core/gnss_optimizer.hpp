#pragma once

#include "slam_mapping/common/config.hpp"
#include "slam_mapping/common/types.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Dense>

#include <vector>

namespace slam_mapping {

/**
 * @brief GNSS 后端因子管理器
 *
 * 独立封装后端 GNSS 优化所需的所有逻辑：
 * - WGS84 -> ENU 坐标转换（ENU 原点取首个通过门控且时间对齐的观测）
 * - map 系 <-> ENU 系对齐估计（2D yaw + 平移，z 偏移取中值），估计完成后冻结
 * - 可靠性门控：status / nsv / 时间差 / 间距 / 一致性
 * - 生成 gtsam::GPSFactor（仅位置 3DoF），可选 Huber 鲁棒核
 *
 * 与具体图结构解耦：只接收关键帧列表，并把因子追加到外部传入的因子图。
 */
class GnssOptimizer {
 public:
  void configure(const GnssOptimizationConfig& cfg);
  void reset();

  /**
   * @brief 为新增关键帧生成 GNSS 位置因子
   *
   * @param keyframes   关键帧列表（含时间对齐的 GNSS 观测）
   * @param graph       后端因子图（追加 GPSFactor）
   * @param latest_index 本次处理到的最新关键帧下标
   */
  void update(const std::vector<MappingKeyframe>& keyframes,
              gtsam::NonlinearFactorGraph* graph,
              int latest_index);

  int factorCount() const { return factor_count_; }
  int rejectedCount() const { return rejected_count_; }
  bool aligned() const { return alignment_complete_; }
  /// 获取估计的 ENU->map 变换（map = R * enu + t）；未完成对齐返回 false
  bool getMapEnuTransform(Eigen::Matrix3d* rot, Eigen::Vector3d* trans) const;
  /// 获取 ENU 原点经纬高（度/米）；未设置返回 false
  bool getEnuOrigin(double* lat_deg, double* lon_deg, double* alt) const;
  /// 已添加因子的 map 系位置（用于 rviz 可视化）
  const std::vector<Eigen::Vector3d>& acceptedPositions() const { return accepted_positions_; }
  /// 已添加因子：(关键帧 id, map 系测量位置)，用于绘制因子-关键帧残差连线
  const std::vector<std::pair<int, Eigen::Vector3d>>& acceptedFactors() const {
    return accepted_factors_;
  }
  /// 被空间门控拒绝的位置（map 系，限量保留，用于 rviz 可视化）
  const std::vector<Eigen::Vector3d>& rejectedPositions() const { return rejected_positions_; }
  /// 被门控拒绝的因子：(关键帧 id, map 系测量位置)，用于绘制因子-关键帧残差连线
  const std::vector<std::pair<int, Eigen::Vector3d>>& rejectedFactors() const {
    return rejected_factors_;
  }

 private:
  bool isReliable(const MappingKeyframe& kf) const;
  bool ensureOrigin(const MappingKeyframe& kf);
  bool toEnu(const GnssObservation& gnss, Eigen::Vector3d* enu) const;
  Eigen::Vector3d correctLeverArm(const MappingKeyframe& kf,
                                  const Eigen::Vector3d& enu) const;
  bool estimateAlignment();
  bool toMapFrame(const Eigen::Vector3d& enu, const MappingKeyframe& kf,
                  Eigen::Vector3d* map_pos) const;
  double positionSigma(const GnssObservation& gnss) const;
  void addGpsFactor(const MappingKeyframe& kf, const Eigen::Vector3d& map_pos,
                    gtsam::NonlinearFactorGraph* graph);
  void recordRejected(int keyframe_id, const Eigen::Vector3d& map_pos);

  GnssOptimizationConfig cfg_;

  // ENU 原点（首个通过门控且时间对齐的观测）
  bool origin_set_ = false;
  double origin_lat_ = 0.0;   ///< ENU 原点纬度（弧度）
  double origin_lon_ = 0.0;   ///< ENU 原点经度（弧度）
  double origin_alt_ = 0.0;   ///< ENU 原点海拔（米）
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();

  // map 系 <-> ENU 系对齐（yaw + xy 平移 + z 偏移；估计完成后冻结）
  bool alignment_complete_ = false;
  Eigen::Matrix3d rot_map_enu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d trans_map_enu_ = Eigen::Vector3d::Zero();

  // 对齐样本：(ENU 位置, map 位姿)，用于 2D Umeyama yaw 估计
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> alignment_pairs_;

  int last_checked_index_ = -1;  ///< 已处理到的最新关键帧下标
  Eigen::Vector2d last_added_xy_ = Eigen::Vector2d::Zero();  ///< 最近因子位置（map 系）
  int factor_count_ = 0;     ///< 已添加的 GNSS 因子数
  int rejected_count_ = 0;   ///< 被门控拒绝的观测数
  std::vector<Eigen::Vector3d> accepted_positions_;  ///< 已加因子位置（map 系）
  std::vector<std::pair<int, Eigen::Vector3d>> accepted_factors_;  ///< 已加因子：(关键帧 id, map 系位置)
  std::vector<Eigen::Vector3d> rejected_positions_;  ///< 被空间门控拒绝的位置（map 系，限量）
  std::vector<std::pair<int, Eigen::Vector3d>> rejected_factors_;  ///< 被拒因子：(关键帧 id, map 系位置)
};

}  // namespace slam_mapping
