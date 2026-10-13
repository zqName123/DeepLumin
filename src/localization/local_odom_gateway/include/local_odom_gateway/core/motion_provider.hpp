#pragma once

/**
 * @file motion_provider.hpp
 * @brief 运动链核心：锚 + 增量 + 相位（纯 C++，零 ROS）
 *
 * 公式：
 *   T(t) = A ⊕ Δ(t_a → t)
 *     A     : 锚 = 主源最近有效样本的位姿（虚拟系），带扫描戳 t_a
 *     Δ     : 运动增量 = D(t_a)⁻¹ ⊕ D(t)   （D = 增量源在其自有系的位姿）
 *
 * 关键性质：Δ 是位姿**差分**，与增量源的世界系无关（相似变换消掉）。
 * 所以不需要把 DR 系对齐到 odom 系，**只需要时间对齐**（按 t_a 插值 D(t_a)）。
 *
 * 相位与两链隔离（设计文档 §4.5 不变式）：
 *   kRunning  (age ≤ overdue)  : T = A          ← 增量不应用（DR 不进运动链）
 *   kGap      (overdue < age)  : T = A ⊕ Δ     ← 增量应用（纯传播，观测链已关）
 *   kOnBackup (age > dead)     : T = A ⊕ Δ     ← 同上，锚冻结（簿记换源）
 *
 * I1: kRunning ⇒ T 不含 Δ        I2: 非 kRunning ⇒ 观测链关闭（由 observation_gate 保证）
 * I3: 切换瞬间 T 恒等            I4: anchor 永不含 Δ
 *
 * 本类只负责数学与相位，**不知道任何源的存在**：
 * 何时落锚由 Arbiter 决定，增量源样本直接喂入。
 */

#include <deque>

#include "local_odom_gateway/core/source_spec.hpp"

namespace local_odom_gateway {

/// 锚：绝对位姿 + 时间戳 + 同刻的增量源位姿（配对）
struct Anchor {
  Pose3d pose;                 ///< A（虚拟系）
  double stamp = -1.0;         ///< t_a（主源样本的扫描戳）
  Pose3d increment_at;         ///< D(t_a)（增量源自有系，按戳插值）
  bool has_pairing = false;    ///< 是否配对成功（无配对则退化为冻结）
  bool valid = false;
};

class MotionProvider {
 public:
  struct Config {
    /// 样本超龄阈值（s）。>0 用固定值；=0 用周期自适应 `overdue_ratio × period_ema`
    double overdue_sec = 0.0;
    /// 样本判死阈值（s）。>0 用固定值；=0 用 `dead_ratio × period_ema`
    double dead_sec = 0.5;
    double overdue_ratio = 1.3;
    double dead_ratio = 2.0;
    double min_period_sec = 0.05;   ///< 周期 EMA 下限保护
    double max_period_sec = 0.30;   ///< 周期 EMA 上限保护
    double increment_history_sec = 1.0;  ///< 增量源轨迹历史长度（插值用）
  };

  void setConfig(const Config& config) { config_ = config; }
  const Config& config() const { return config_; }

  /**
   * @brief 主源新样本：落锚 + 按【扫描戳】配对增量源位姿 + 更新周期 EMA。
   * @param pose 虚拟系位姿（换源粘合由 Arbiter 施加后再传入）
   * @param stamp 主源样本的扫描戳（不是到达时刻）
   */
  void onAnchorSample(const Pose3d& pose, double stamp);

  /// 增量源（如 DR）轨迹样本：供按戳插值 D(t_a)
  void onIncrementSample(const Pose3d& pose, double stamp);

  /// 取运动输出。false = 尚无锚（无输出）
  bool sample(Sample* out, double now_sec) const;

  /// 相位（按锚的年龄判定，见类注释）
  Phase phase(double now_sec) const;

  bool hasAnchor() const { return anchor_.valid; }
  const Anchor& anchor() const { return anchor_; }
  /// 锚年龄（s）；无锚返回 -1
  double anchorAge(double now_sec) const;
  /// 实测主源周期 EMA（s）；未测得返回 -1
  double periodEma() const { return period_ema_; }
  /// 当前生效的超龄 / 判死阈值（s）
  double overdueLimit() const;
  double deadLimit() const;

 private:
  bool incrementAt(double stamp, Pose3d* out) const;
  void pushIncrement(const Pose3d& pose, double stamp);

  Config config_;
  Anchor anchor_;
  std::deque<TimedPose> increment_history_;
  TimedPose last_increment_;      ///< 最新增量源样本
  bool has_increment_ = false;
  double period_ema_ = -1.0;
  double last_anchor_stamp_ = -1.0;
};

}  // namespace local_odom_gateway
