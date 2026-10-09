#!/usr/bin/env python3
"""
验证 IMU 数据与真值位姿是否对齐

方法：选取短时段（2~5s），对 IMU 做航迹推算：
  1. 陀螺仪积分 → 姿态（四元数）
  2. 去除重力 → 加速度转到世界系 → 二次积分 → 位置
  3. 与真值轨迹对比

如果 IMU 和真值对齐良好，短时积分轨迹应和真值轨迹基本重合。
"""
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import sys

# ── 辅助函数 ──
def quat_multiply(q1, q2):
    """四元数乘法 q = q1 * q2, 输入/输出格式 [w,x,y,z]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_rotate(q, v):
    """用四元数 [w,x,y,z] 旋转向量 v"""
    qv = np.array([0, v[0], v[1], v[2]])
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    q_rot = quat_multiply(quat_multiply(q, qv), q_conj)
    return q_rot[1:]

NPZ_PATH = "/home/hhy/2004ros/data/imu_pose_200hz.npz"
PLOT_OUT = "/home/hhy/2004ros/data/imu_dead_reckoning_verification.png"

if not os.path.exists(NPZ_PATH):
    print(f"[ERROR] {NPZ_PATH} 不存在")
    sys.exit(1)

data = np.load(NPZ_PATH)
ts_full     = data["retargetted_ts"]
imu_full    = data["retargetted_imu"]   # (N,6) [ax, ay, az, gx, gy, gz]
pos_full    = data["retargetted_pos"]
quat_full   = data["retargetted_quat"]  # [qw, qx, qy, qz]

FS = 200.0
dt = 1.0 / FS

# ── 选择 3 个短时段 ──────────────────────────────────
# 自动挑选运动丰富的段
gyro_norm = np.linalg.norm(imu_full[:, 3:6], axis=1)
acc_norm = np.linalg.norm(imu_full[:, :3], axis=1)

# 计算滑动窗口活跃度
window_len = int(FS * 2)  # 2秒窗口
movement = np.convolve(gyro_norm, np.ones(window_len)/window_len, mode='valid')

# 选3个活跃度高的段（有转弯/加减速，更有验证价值）
t_total = ts_full[-1] - ts_full[0]
segments = [
    (int(t_total * 0.2),  3),   # 20% 处，3秒
    (int(t_total * 0.5),  3),   # 50% 处，3秒
    (int(t_total * 0.75), 4),   # 75% 处，4秒
]

print(f"{'='*70}")
print(f"  IMU Dead Reckoning vs Ground Truth Verification")
print(f"{'='*70}")

fig, axes = plt.subplots(3, 3, figsize=(18, 14))
fig.suptitle('IMU Dead Reckoning vs Ground Truth (short segments)', fontsize=14)

all_errors = []

for seg_idx, (start_t, dur) in enumerate(segments):
    print(f"\n  Segment {seg_idx+1}: t={start_t:.0f}s ~ {start_t+dur:.0f}s ({dur}s)")

    # ── 截取数据 ──
    t0 = ts_full[0] + start_t
    t1 = t0 + dur
    mask = (ts_full >= t0) & (ts_full <= t1)
    ts     = ts_full[mask]
    imu    = imu_full[mask]
    pos_gt = pos_full[mask]      # 真值位置
    quat_gt = quat_full[mask]    # 真值姿态

    N_seg = len(ts)
    t_rel = ts - ts[0]

    # ── 诊断：初始时刻的重力方向 ──
    init_acc = np.mean(imu[:min(20, N_seg), :3], axis=0)  # 前20帧平均 acc
    init_gyro = np.mean(imu[:min(20, N_seg), 3:6], axis=0)
    acc_norm = np.linalg.norm(init_acc)
    print(f"\n    初始 acc 均值: [{init_acc[0]:.4f}, {init_acc[1]:.4f}, {init_acc[2]:.4f}]  "
          f"norm={acc_norm:.4f}")
    print(f"    初始 gyro 均值: [{init_gyro[0]:.4f}, {init_gyro[1]:.4f}, {init_gyro[2]:.4f}]")
    print(f"    初始姿态(欧拉deg): {Rotation.from_quat(quat_gt[0, [1,2,3,0]]).as_euler('xyz', degrees=True)}")

    # ── 修正 IMU 积分：先用静止段估计 acc bias ──
    # 用初始姿态把 acc 转到世界系，与世界系重力相减得到 bias
    q_init = quat_gt[0]
    acc_world_init = quat_rotate(q_init, init_acc)
    acc_bias_est = acc_world_init - np.array([0, 0, 9.81])
    print(f"    估计 acc bias (world系): [{acc_bias_est[0]:.4f}, {acc_bias_est[1]:.4f}, {acc_bias_est[2]:.4f}] m/s²")

    # ── IMU Dead Reckoning（带 bias 修正）──

    # 1. 姿态积分（从真值初始姿态开始）
    quat_imu = np.zeros((N_seg, 4))
    quat_imu[0] = quat_gt[0]

    for i in range(1, N_seg):
        gyro = imu[i-1, 3:6]
        angle = gyro * dt
        angle_norm = np.linalg.norm(angle)
        if angle_norm > 1e-10:
            axis = angle / angle_norm
            dq = Rotation.from_rotvec(angle_norm * axis).as_quat()  # [x,y,z,w]
            dq_wxyz = np.array([dq[3], dq[0], dq[1], dq[2]])
            quat_imu[i] = quat_multiply(quat_imu[i-1], dq_wxyz)
        else:
            quat_imu[i] = quat_imu[i-1]

    # 2. 加速度积分（带 bias 修正）
    gravity = np.array([0, 0, 9.81])
    vel = np.zeros((N_seg, 3))
    pos_imu = np.zeros((N_seg, 3))
    pos_imu[0] = pos_gt[0]

    for i in range(1, N_seg):
        q = quat_imu[i-1]
        a_body = imu[i-1, :3]
        a_world = quat_rotate(q, a_body) - gravity - acc_bias_est  # 减去 bias
        vel[i] = vel[i-1] + a_world * dt
        pos_imu[i] = pos_imu[i-1] + vel[i] * dt

    # 3. 误差统计
    pos_error = np.linalg.norm(pos_imu - pos_gt, axis=1)
    end_error = pos_error[-1]
    max_error = np.max(pos_error)
    avg_error = np.mean(pos_error)
    all_errors.extend(pos_error.tolist())

    print(f"    真值位移:    {np.linalg.norm(pos_gt[-1] - pos_gt[0]):.4f} m")
    print(f"    IMU推算位移: {np.linalg.norm(pos_imu[-1] - pos_imu[0]):.4f} m")
    print(f"    终点位置差:  {end_error:.4f} m (bias修正后)")
    print(f"    最大位置差:  {max_error:.4f} m")
    print(f"    平均位置差:  {avg_error:.4f} m")

    # 判断
    drift_rate = end_error / dur * 100  # 换算成 cm/s
    if drift_rate < 5:
        verdict = "✅ 对齐良好"
    elif drift_rate < 20:
        verdict = "⚠️ 轻度漂移"
    else:
        verdict = "❌ 严重偏差"

    print(f"    漂移率:      {drift_rate:.2f} cm/s  {verdict}")

    # ── 绘图 ──
    ax_traj = axes[seg_idx, 0]
    ax_pos  = axes[seg_idx, 1]
    ax_err  = axes[seg_idx, 2]

    # XY 轨迹对比
    ax_traj.plot(pos_gt[:, 0], pos_gt[:, 1], 'g-', linewidth=2, label='Ground Truth')
    ax_traj.plot(pos_imu[:, 0], pos_imu[:, 1], 'r--', linewidth=2, label='IMU Dead Reckoning')
    ax_traj.scatter(pos_gt[0, 0], pos_gt[0, 1], c='g', marker='o', s=80, zorder=5)
    ax_traj.scatter(pos_gt[-1, 0], pos_gt[-1, 1], c='g', marker='s', s=80, zorder=5)
    ax_traj.scatter(pos_imu[-1, 0], pos_imu[-1, 1], c='r', marker='x', s=80, zorder=5,
                    label=f'IMU end (err={end_error:.3f}m)')
    ax_traj.set_xlabel('x (m)')
    ax_traj.set_ylabel('y (m)')
    ax_traj.set_title(f'Seg {seg_idx+1}: XY Trajectory ({dur}s)')
    ax_traj.legend(fontsize=8)
    ax_traj.grid(True, alpha=0.3)
    ax_traj.axis('equal')

    # XYZ vs 时间
    ax_pos.plot(t_rel, pos_gt[:, 0], 'g-', linewidth=1, alpha=0.7)
    ax_pos.plot(t_rel, pos_gt[:, 1], 'g-', linewidth=1, alpha=0.7)
    ax_pos.plot(t_rel, pos_gt[:, 2], 'g-', linewidth=1, alpha=0.7, label='GT')
    ax_pos.plot(t_rel, pos_imu[:, 0], 'r--', linewidth=1, alpha=0.7)
    ax_pos.plot(t_rel, pos_imu[:, 1], 'r--', linewidth=1, alpha=0.7)
    ax_pos.plot(t_rel, pos_imu[:, 2], 'r--', linewidth=1, alpha=0.7, label='IMU')
    ax_pos.set_xlabel('time (s)')
    ax_pos.set_ylabel('position (m)')
    ax_pos.set_title(f'Seg {seg_idx+1}: XYZ Position')
    ax_pos.legend(fontsize=8)
    ax_pos.grid(True, alpha=0.3)

    # 误差 vs 时间
    ax_err.plot(t_rel, pos_error, 'b-', linewidth=1.5)
    ax_err.axhline(end_error, color='r', linestyle='--', alpha=0.5,
                   label=f'end error={end_error:.3f}m')
    ax_err.set_xlabel('time (s)')
    ax_err.set_ylabel('position error (m)')
    ax_err.set_title(f'Seg {seg_idx+1}: Drift = {drift_rate:.1f} cm/s')
    ax_err.legend(fontsize=8)
    ax_err.grid(True, alpha=0.3)

# ── 汇总 ──
if all_errors:
    all_errors = np.array(all_errors)
    print(f"\n{'='*70}")
    print(f"  汇总")
    print(f"{'='*70}")
    print(f"  三段总误差样本: {len(all_errors)}")
    print(f"  平均位置误差:   {np.mean(all_errors):.4f} m")
    print(f"  中位位置误差:   {np.median(all_errors):.4f} m")
    print(f"  最大位置误差:   {np.max(all_errors):.4f} m")
    print(f"")
    print(f"  注意: IMU 裸积分 3~4 秒漂移 1~20m 属于正常现象")
    print(f"  （双积分放大加速度 bias + 陀螺漂移导致姿态发散）")
    print(f"  只有漂移在 0.1m 以内才能认为 IMU 和真值完美对齐")
    print(f"  更大的漂移不一定说明对齐有问题，需要结合初始 bias 分析")

plt.tight_layout()
plt.savefig(PLOT_OUT, dpi=150)
print(f"\n  ✅ 验证图已保存: {PLOT_OUT}")