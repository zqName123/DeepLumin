#pragma once

/**
 * @file stationary_detector.h
 * @brief 静止检测（滞回）—— 移植自 lio_odometry_v2
 *
 * 把"当前帧速度"映射为"是否已确认静止"。用途：静止时不往 iVox 地图插点，
 * 避免同一批点被反复插入导致地图膨胀/拖影。
 *
 * 与 GNSS/轮速开关**解耦**：是否静止只由速度决定，与"用不用某个传感器"无关。
 * 调用方负责提供可靠的速度值（轮速需自带新鲜度判定，CAN 停发后不能用旧值）。
 *
 * 滞回语义（与 lio_odometry_v2 逐条一致，有回归测试锁定）：
 *   - speed < min_speed_mps          → 累积计数 ++
 *   - speed > resume_speed_mps       → 计数清零（解除）
 *   - 介于两者之间                    → 保持不变（既不累积也不解除）
 *   - 计数 >= confirm_frames         → confirmed() = true
 *   - 解除后 resume_hold() 尾帧内    → 调用方应继续跳过地图插点
 */

namespace faster_lio {

class StationaryDetector {
   public:
    struct Options {
        bool enable = true;              ///< false = 永不确认静止（调用方回到无条件插点）
        double min_speed_mps = 0.2;      ///< 低于此累积为静止候选
        double resume_speed_mps = 0.4;   ///< 高于此才解除（滞回）
        int confirm_frames = 30;         ///< 连续 N 帧才"确认静止"
        int resume_hold_frames = 10;     ///< 解除后继续跳过插点的尾帧数
    };

    void Configure(const Options& opt) { opt_ = opt; }

    void Reset() {
        hold_ = 0;
        confirmed_ = false;
        resume_hold_ = 0;
    }

    /// @return 是否已确认静止
    bool Update(double speed) {
        if (!opt_.enable) {
            return false;  // 门控关闭：永不静止，调用方无条件插点
        }
        // 滞回：低于 min 累积，高于 resume 清零，介于两者之间保持不变
        if (speed < opt_.min_speed_mps) {
            ++hold_;
        } else if (speed > opt_.resume_speed_mps) {
            hold_ = 0;
        }

        confirmed_ = (hold_ >= opt_.confirm_frames);
        if (confirmed_) {
            // 静止期间持续刷新尾帧计数；解除后的第一帧由它继承，再逐帧递减
            resume_hold_ = opt_.resume_hold_frames;
        }
        return confirmed_;
    }

    bool confirmed() const { return confirmed_; }
    int hold_count() const { return hold_; }

    /// 解除静止后仍需跳过插点的尾帧计数
    int resume_hold() const { return resume_hold_; }

    /// 每帧递减尾帧计数（调用方在"解除后继续跳插"分支里调用）
    void TickResumeHold() {
        if (resume_hold_ > 0) {
            --resume_hold_;
        }
    }

   private:
    Options opt_;
    int hold_ = 0;           ///< 静止候选累积帧数
    bool confirmed_ = false; ///< 已确认静止
    int resume_hold_ = 0;    ///< 解除后的尾帧计数
};

}  // namespace faster_lio
