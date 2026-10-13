#pragma once

/**
 * @file sensor_health.h
 * @brief 传感器数据健康判定（有无 + 新鲜度）与静止检测
 *
 * 解决的问题：
 *   此前轮速/GNSS 的使用完全靠配置开关（use_wheel_speed / use_gnss）判断，
 *   没有"数据到底有没有、新不新鲜"的统一判定。实测后果：配置开着但没有数据时，
 *   旧值/初值被静默当成有效数据使用（如 wheel_speed_obs_ 冻结在 0 导致把在动的车判成静止）。
 *
 * 设计约定（重要）：
 *   1. 新鲜度判定的参照时刻是**数据时间轴**上的 t_ref（当前帧的 lidar_end_time_），
 *      而不是 ros::Time::now() —— 后者在 bag 回放/倍速/use_sim_time 下会全错，
 *      且与估计器所处的时间轴不是一回事。
 *   2. 判定规则只在 Evaluate() 里写一处：usable = configured && ever_received && fresh，
 *      消费点只读快照，不再各自重复判断。
 *   3. 静止检测与 GNSS 开关**解耦**：是否静止只由速度决定，与"用不用 GNSS"无关。
 */

#include <cstdint>
#include <string>

namespace slam_odometry {

/// 单个传感器通道的数据可用性（有无 + 新鲜度）
struct SensorStatus {
    bool configured = false;      ///< 配置开关是否打开（use_xxx）；false 时一律不可用
    bool ever_received = false;   ///< 是否收到过至少一条数据（区分"没数据"与"数据值恰好是 0"）
    double last_stamp = -1.0;     ///< 最近一条数据的消息时间戳（主判据）
    double age = 1e9;             ///< t_ref - last_stamp，由 Evaluate() 刷新
    bool fresh = false;           ///< ever_received && age <= max_age
    std::uint64_t received = 0;   ///< 累计条数（诊断）
    std::uint64_t stale_hits = 0; ///< 判为不新鲜的次数（诊断）
    int quality_status = 0;       ///< GNSS status/nsv：仅透传供诊断，本次不参与判定
    int quality_nsv = 0;

    /// 可用 = 配置开启 ∧ 收到过 ∧ 新鲜。
    /// 三者缺一不可：只判有无会让断发后的旧值继续生效，只判新鲜度会让"从未收到"的初值蒙混过关。
    bool usable() const { return configured && ever_received && fresh; }
};

/**
 * @brief 统一的传感器数据健康判定
 *
 * 用法：各入口回调里调用 OnXxx(stamp) 登记；每帧在消费前调用一次 Evaluate(t_ref)
 * 拿到快照，之后所有消费点只读快照的 usable()。
 */
class SensorHealthMonitor {
   public:
    struct Options {
        double imu_max_age_sec = 0.2;    ///< IMU 5~10ms 周期，20~40 个周期
        double lidar_max_age_sec = 0.5;  ///< LiDAR 100ms 周期，5 帧
        double wheel_max_age_sec = 0.15; ///< 轮速 ~20ms 周期
        double gnss_max_age_sec = 1.0;   ///< GNSS 周期抖动大，放宽
        /// 允许的"时钟超前"上限（秒）。
        ///
        /// 注意 age = t_ref - last_stamp 在实时流水线里**通常是小负数**：
        /// t_ref 是当前处理帧的扫描末端时刻，而 last_stamp 是最新一条传感器数据，
        /// 后者天然领先前者一个流水线延迟（传输 + 组包 + 调度 + 处理），实测约 100ms
        /// 量级。数据比当前帧新恰恰说明数据在流动，**不能判为不新鲜**。
        /// 只有超前超过本阈值才认为是时基不同源（例如 GNSS 用接收机时钟、
        /// LiDAR 用 GPS 时钟）。
        double max_clock_skew_sec = 5.0;
    };

    /// @param use_wheel / use_gnss 对应配置开关；关闭的通道一律判定为不可用
    void Configure(bool use_wheel, bool use_gnss, const Options& opt);

    /// 入口登记（只记时间戳与计数，O(1)）。stamp 非法（<0 或非有限）会被忽略。
    void OnImu(double stamp);
    void OnLidar(double stamp);
    void OnWheel(double stamp);
    void OnGnss(double stamp, int status, int nsv);

    struct Snapshot {
        SensorStatus imu, lidar, wheel, gnss;
    };

    /// @param t_ref 当前帧的数据时间（通常是 lidar_end_time_）
    Snapshot Evaluate(double t_ref);

    /// 限频日志用的一行摘要
    std::string DebugString(const Snapshot& s) const;

   private:
    static void Touch(SensorStatus& st, double stamp);
    static void Refresh(SensorStatus& st, double t_ref, double max_age, double max_clock_skew);

    Options opt_;
    SensorStatus imu_, lidar_, wheel_, gnss_;
};

/**
 * @brief 静止检测（滞回）
 *
 * 把"当前帧速度"映射为"是否已确认静止"。与 GNSS 开关无关。
 *
 * 滞回语义（与原实现一致）：
 *   - speed < min_speed_mps          → 累积计数 ++
 *   - speed > resume_speed_mps       → 计数清零（解除）
 *   - 介于两者之间                    → 保持不变（既不累积也不解除）
 *   - 计数 >= confirm_frames         → confirmed() = true
 *   - 解除后 resume_hold() 尾帧内    → 调用方应继续跳过地图插点
 */
class StationaryDetector {
   public:
    struct Options {
        double min_speed_mps = 0.2;      ///< 低于此累积为静止候选
        double resume_speed_mps = 0.4;   ///< 高于此才解除（滞回）
        int confirm_frames = 30;         ///< 连续 N 帧才"确认静止"
        int resume_hold_frames = 10;     ///< 解除后继续跳过插点的尾帧数
    };

    void Configure(const Options& opt);
    void Reset();

    /// @return 是否已确认静止
    bool Update(double speed);

    bool confirmed() const { return confirmed_; }
    int hold_count() const { return hold_; }

    /// 解除静止后仍需跳过插点的尾帧计数
    int resume_hold() const { return resume_hold_; }
    /// 每帧递减尾帧计数（调用方在"解除后继续跳插"分支里调用）
    void TickResumeHold();

   private:
    Options opt_;
    int hold_ = 0;           ///< 静止候选累积帧数
    bool confirmed_ = false; ///< 已确认静止
    int resume_hold_ = 0;    ///< 解除后的尾帧计数
};

}  // namespace slam_odometry
