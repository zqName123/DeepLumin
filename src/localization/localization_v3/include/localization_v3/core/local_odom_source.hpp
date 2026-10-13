#pragma once

/**
 * @file local_odom_source.hpp
 * @brief 统一局部里程计源（SLAM 主 / DR 备）—— 虚拟局部系管理
 *
 * 概念：本模块维护一个"虚拟局部里程计系"，unified = glue ⊕ T_source。
 *   SLAM / DR 位姿都被粘合到该系后输出，下游（fusion / GM / 状态机）看到的
 *   始终是一条连续的局部里程计，对源切换完全无感知——判断与状态转移因此零改动。
 *
 * 输出语义（单一纯源输出，DR 是否进输出链的唯一分界是"换源"这一事件）：
 *   ① 纯 SLAM（SLAM 源可用）
 *        unified = glue_slam ⊕ T_slam
 *        ← DR **结构性**不进输出链：没有任何"DR 增量补 gap"路径，
 *          正常节拍的周期抖动不可能触发 DR 参与（毛刺根除）。
 *          此期定位结果只作 DR 的标定观测（purePose 事件式，见下），DR 输出不被使用。
 *   ② DR 接管（SLAM 判死：age > slam_timeout_sec）
 *        unified = glue_dr ⊕ T_dr
 *        接管锚定（运动量不丢）：glue_dr = T0 ⊕ D(t0)⁻¹
 *        （t0/T0 = 最后 SLAM 样本的戳与位姿；D(t0) 按样本戳在 DR 历史插值）
 *        ⇒ 输出 = T0 ⊕ (D(t0)⁻¹ ⊕ D(t))——检测窗 t0→t_sw 内车辆的真实运动
 *          一次性补上（接管瞬间输出前进 v×(t_sw-t0)，之后 DR 递推续走）。
 *
 * 切换连续性：
 *   回切：glue_new = unified(t_sw⁻) ⊕ T_new(t_sw)⁻¹ ⇒ 切换瞬间 unified 严格相等；
 *   接管：不取恒等而取"运动量不变式"（上式）——检测窗运动补进输出，
 *         避免"轨迹短一截 + GM 事后回拉"的跳变。
 *   冷启动（虚拟系未经 LIO 确立）：glue_slam = I 直接定系、不换算
 *   ——此时 map_odom 尚未建立、无连续性可保，换算反而会把启动期 DR 的误差
 *   （roll/pitch 未重力对齐）永久烘进虚拟系。
 *
 * DR 观测隔离（防自回灌）：
 *   purePose() 是无仲裁副作用的纯源位姿取样，供桥在 SLAM 样本到达时事件式发观测锚。
 *   锚只在 SLAM 驱动输出时发布（桥侧过滤），接管期无 SLAM 样本 ⇒ 无观测事件
 *   ⇒ DR 输出（含自身增量）在结构上不可能成为自己的观测。
 */

#include <deque>
#include <vector>

#include "localization_v3/core/types.hpp"

namespace localization_v3 {

class LocalOdomSource {
 public:
  enum class Source { kNone = 0, kSlam, kDr };

  struct Config {
    /// SLAM 样本超龄判定（s）→ 切 DR
    double slam_timeout_sec = 0.5;
    /// DR 样本超龄判定（s）→ 双源皆失无输出
    double dr_timeout_sec = 0.5;
    /// 回切滞回：连续 N 次仲裁确认 SLAM 可用才切回（防断续流翻转）
    int switch_back_confirm_frames = 5;
    /// 翻转保护：flap_window_sec 内切换达 N 次 → 回切确认帧加倍（0=关闭）
    int flap_guard_count = 3;
    double flap_window_sec = 60.0;
    /// SLAM 源可用 = odom 新鲜 && 注册云新鲜
    /// （防"发 odom 不发云"的部分故障：LIO 里程计活着但配准云断了，GM 无云可配）
    bool slam_require_cloud = false;
    double slam_cloud_timeout_sec = 0.5;
    /// DR 位姿历史长度（s）：接管时按最后 SLAM 样本戳插值取 D(t0) 用
    double dr_history_sec = 1.0;
  };

  struct Snapshot {
    Source active = Source::kNone;
    double slam_age = -1.0;  ///< 秒；-1 = 从未收到
    double dr_age = -1.0;
    int switch_count = 0;
    bool flap_guard = false;  ///< 翻转保护已触发（回切确认帧加倍中）
  };

  void setConfig(const Config& config) { config_ = config; }
  const Config& config() const { return config_; }

  /// 喂样本（pose: 源系 -> base）
  void onSlam(const Pose3d& pose, double stamp);
  void onDr(const Pose3d& pose, double stamp);
  /// SLAM 注册云活性（云转发时喂入；slam_require_cloud 时参与源可用判定）
  void onSlamCloud(double stamp);

  /**
   * @brief 仲裁 + 取样。内部可发生源切换（粘合保证输出连续）。
   * @return false = 双源皆不可用（无输出，由 fusion 层看门狗判失效）
   */
  bool sample(Pose3d* out, double now_sec);

  Source active() const { return active_; }
  /// 当前源纯位姿 = glue ⊕ T_source。**无仲裁副作用**，
  /// 供事件式发布标定观测使用（sample() 会推进仲裁状态，不可在数据回调里调用）
  bool purePose(Pose3d* out) const;
  /// SLAM→虚拟系粘合。I = SLAM 系即虚拟系（云可原样透传）；
  /// 非 I 时源点云必须同步变换到虚拟系（桥契约：里程计与源点云同系）
  const Pose3d& slamGlue() const { return glue_slam_; }
  Snapshot snapshot(double now_sec) const;

 private:
  struct TimedPose {
    double stamp = -1.0;
    Pose3d pose;
  };

  void adoptSlam();
  void adoptDr();
  void recordSwitch(double now_sec);
  bool flapGuardTripped(double now_sec) const;
  /// SLAM 源可用性（odom 新鲜 [+ 注册云新鲜]）
  bool slamUsable(double now_sec) const;
  void pushDrHistory(const Pose3d& pose, double stamp);
  /// 按时间戳取 DR 位姿（线性插值位置 + slerp 姿态）；历史覆盖不到 → 最近样本
  bool drPoseAt(double stamp, Pose3d* out) const;

  Config config_;

  // 最近样本
  Pose3d last_slam_;
  Pose3d last_dr_;
  double last_slam_stamp_ = -1.0;
  double last_dr_stamp_ = -1.0;
  double last_slam_cloud_stamp_ = -1.0;

  // DR 位姿历史（仅接管锚定用：按最后 SLAM 样本戳插值取 D(t0)）
  std::deque<TimedPose> dr_history_;

  // 虚拟系粘合：unified = glue ⊕ T_source
  Source active_ = Source::kNone;
  Pose3d glue_slam_;
  Pose3d glue_dr_;
  /// 虚拟系是否已由 LIO 帧确立（= 是否出现过 SLAM 驱动输出）。
  /// 未确立时属于冷启动：LIO 一来直接 glue_slam = I（虚拟系定为 LIO 系），
  /// 不走连续性换算——此时 map_odom 尚未建立、无连续性可保，而换算会把
  /// 启动期 DR 的误差（roll/pitch 未重力对齐）永久烘进虚拟系。
  /// 确立之后的换源才用连续性不变式保持输出恒等。
  bool slam_frame_established_ = false;
  int slam_confirm_count_ = 0;
  int switch_count_ = 0;

  // 翻转保护（最近切换时刻）
  std::vector<double> switch_times_;
};

}  // namespace localization_v3
