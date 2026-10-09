#!/usr/bin/env python3
"""
验证 yaw offset (-π/2) 改正是否正确。

方法一：轨迹方向 vs GT yaw
  车辆前进方向（从位置差分得到）应与 GT yaw 指向一致。
  如果 yaw 改正正确： trajectory_yaw ≈ GT_yaw
  如果 yaw 改正有误： 两者相差 90° 或 -90°

方法二：陀螺仪积分 vs GT yaw 变化量
  如果对齐正确： d(GT_yaw)/dt ≈ gyro_z
"""
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os, sys

NPZ_PATH = "/home/hhy/2004ros/data/imu_pose_200hz.npz"
PLOT_OUT = "/home/hhy/2004ros/data/yaw_correction_verification.png"

if not os.path.exists(NPZ_PATH):
    print(f"[ERROR] {NPZ_PATH} 不存在")
    sys.exit(1)

data = np.load(NPZ_PATH)
ts     = data["retargetted_ts"]
imu    = data["retargetted_imu"]
pos    = data["retargetted_pos"]
quat   = data["retargetted_quat"]  # [w,x,y,z]

FS = 200.0
dt = 1.0 / FS
t = ts - ts[0]

# ── 1. 提取 GT yaw ─────────────────────────────────
# quat → scipy [x,y,z,w] → euler
gt_euler = Rotation.from_quat(quat[:, [1, 2, 3, 0]]).as_euler('xyz', degrees=True)
gt_yaw = gt_euler[:, 2]  # deg

# ── 2. 轨迹方向 yaw ────────────────────────────────
# 从位置差分得到运动方向
dx = np.gradient(pos[:, 0])
dy = np.gradient(pos[:, 1])
traj_yaw = np.degrees(np.arctan2(dy, dx))  # deg

# ── 3. 降采样（21 万点绘图太密） ────────────────────
step = max(1, len(ts) // 30000)
idx = slice(None, None, step)

# ── 4. 计算 yaw 差值 ────────────────────────────────
# yaw 差值（处理 ±180° 跳变）
yaw_diff = (gt_yaw - traj_yaw + 180) % 360 - 180
# 平滑一下减少噪声
from scipy.ndimage import uniform_filter1d
yaw_diff_smooth = uniform_filter1d(yaw_diff, size=int(FS * 0.5))

print(f"{'='*75}")
print(f"  Yaw Offset Correction Verification")
print(f"{'='*75}")
print(f"\n  方法一：GT yaw vs 轨迹方向")
print(f"  ─────────────────────────────────")

# 统计 yaw 差值的集中度
median_diff = np.median(yaw_diff)
std_diff = np.std(yaw_diff)
# 取差值的中位数附近（去除 ±180° 歧义影响）
valid_diff = yaw_diff[np.abs(yaw_diff - median_diff) < 90] if np.abs(median_diff) < 135 else \
             ((yaw_diff - 180) % 360 - 180)

if len(valid_diff) > 100:
    mean_diff = np.mean(valid_diff)
    std_diff_val = np.std(valid_diff)
    print(f"  大部分帧集中在: {mean_diff:.1f}° ± {std_diff_val:.1f}°")
    
    # 判断
    if abs(mean_diff) < 10:
        print(f"  ✅ yaw 改正正确（GT yaw ≈ 轨迹方向）")
    elif abs(mean_diff) < 30:
        print(f"  ⚠️  yaw 改正有微小偏差")
    else:
        print(f"  ❌ yaw 改正可能有问题（偏差 {mean_diff:.1f}°）")
else:
    print(f"  ⚠️  无法判断（数据方差过大）")

# ── 5. 方法二：gyro_z 积分 vs GT yaw ────────────────
print(f"\n  方法二：gyro_z 积分 vs GT yaw 变化")
print(f"  ─────────────────────────────────")

# 计算 gyro_z 积分和 GT yaw 变化
gyro_z = imu[:, 5]
gt_yaw_change = np.diff(gt_yaw, prepend=gt_yaw[0])
# 处理 ±180° 跳变
gt_yaw_change = (gt_yaw_change + 180) % 360 - 180

# 对陀螺仪积分做同样的累积变化
gyro_yaw_integrated = np.cumsum(gyro_z * dt) * 180.0 / np.pi  # rad→deg

# 计算 GT yaw 的累积变化（去跳变）
gt_yaw_unwrap = np.unwrap(gt_yaw, period=360)  # 连续的 yaw（无跳变）
gt_yaw_change_from_start = gt_yaw_unwrap - gt_yaw_unwrap[0]

# 取中间一段对比
mid = len(ts) // 2
half = min(int(FS * 30), mid)  # 取 30 秒
s = mid - half
e = mid + half
gyro_yaw_seg = gyro_yaw_integrated[s:e] - gyro_yaw_integrated[s]
gt_yaw_seg = gt_yaw_change_from_start[s:e] - gt_yaw_change_from_start[s]

# 相关性
corr = np.corrcoef(gyro_yaw_seg, gt_yaw_seg)[0, 1]
print(f"  30 秒段相关性: r = {corr:.4f}")
if corr > 0.8:
    print(f"  ✅ gyro_z 积分与 GT yaw 高度相关（yaw 改正正确）")
elif corr > 0.5:
    print(f"  ⚠️  中等相关，可能有少量偏差")
else:
    print(f"  ❌ 低相关，yaw 改正可能有问题")

# ── 6. 打印统计 ──
print(f"\n{'='*75}")
print(f"  统计摘要")
print(f"{'='*75}")
print(f"  样本数: {len(ts)}")
print(f"  GT yaw 范围: {gt_yaw.min():.1f}° ~ {gt_yaw.max():.1f}°")
print(f"  轨迹方向范围: {traj_yaw.min():.1f}° ~ {traj_yaw.max():.1f}°")
print(f"  中位 yaw 差(GT-轨迹): {np.median(yaw_diff):.1f}°")

# 检查各种偏移的占比
import collections

print(f"\n  ── 不同偏移量覆盖率 ──")
# 只在速度 > 0.5 m/s 时统计（去除静止噪声）
speed = np.sqrt(np.diff(pos[:, 0], prepend=pos[0,0])**2 + 
                np.diff(pos[:, 1], prepend=pos[0,1])**2) * FS
moving = speed > 0.5
yaw_diff_moving = yaw_diff[moving]

for offset in [0, 90, -90, 180, -180]:
    aligned = np.mean(np.abs((yaw_diff_moving - offset + 180) % 360 - 180) < 20)
    symbol = "✅" if offset == 0 and aligned > 0.5 else \
             "✅" if offset == -90 and aligned > 0.5 else ""
    flag = ""
    if offset == 0 and aligned > 0.5:
        flag = " ← 正确（无偏移）"
    elif offset == -90 and aligned > 0.5:
        flag = " ← 当前状态"
    elif offset == 90 and aligned > 0.5:
        flag = " ← 可能需反转 Q_OFFSET"
    print(f"  偏移 {offset:+4d}°: {aligned*100:.1f}% 帧对齐 {flag}")

# ── 可视化 ──
print(f"\n  绘制验证图 ...")
fig, axes = plt.subplots(2, 2, figsize=(16, 12))
fig.suptitle('Yaw Offset Correction Verification', fontsize=14)

# A: XY 轨迹（颜色 = GT yaw）
ax0 = axes[0, 0]
sc = ax0.scatter(pos[::step, 0], pos[::step, 1], c=gt_yaw[::step],
                 cmap='hsv', s=2, alpha=0.7)
ax0.set_xlabel('x (m)')
ax0.set_ylabel('y (m)')
ax0.set_title('Trajectory colored by GT yaw')
ax0.axis('equal')
ax0.grid(True, alpha=0.3)
plt.colorbar(sc, ax=ax0, label='GT yaw (deg)')

# B: GT yaw vs 轨迹方向 yaw（3s 窗口）
ax1 = axes[0, 1]
ax1.plot(t[idx], gt_yaw[idx], 'b-', linewidth=0.8, alpha=0.7, label='GT yaw')
ax1.plot(t[idx], traj_yaw[idx], 'g-', linewidth=0.8, alpha=0.5, label='Trajectory yaw')
ax1.set_xlabel('time (s)')
ax1.set_ylabel('yaw (deg)')
ax1.set_title('GT Yaw vs Trajectory Direction')
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.3)

# C: yaw 差分布直方图
ax2 = axes[1, 0]
ax2.hist(yaw_diff_smooth[::step], bins=90, range=(-180, 180),
         alpha=0.7, color='steelblue', edgecolor='none')
ax2.axvline(0, color='r', linestyle='--', linewidth=2, label='0° (correct)')
ax2.axvline(90, color='orange', linestyle=':', linewidth=1.5, label='+90° error')
ax2.axvline(-90, color='orange', linestyle=':', linewidth=1.5, label='-90° error')
ax2.set_xlabel('yaw diff (GT - trajectory) [deg]')
ax2.set_ylabel('count')
ax2.set_title('Yaw Difference Distribution')
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

# D: gyro_z 积分 vs GT yaw 变化
ax3 = axes[1, 1]
ax3.plot(t[s:e] - t[s], gyro_yaw_seg, 'r-', linewidth=1, alpha=0.7, label='gyro_z integrated')
ax3.plot(t[s:e] - t[s], gt_yaw_seg, 'b-', linewidth=1, alpha=0.7, label='GT yaw change')
ax3.set_xlabel('time (s)')
ax3.set_ylabel('yaw change (deg)')
ax3.set_title(f'Gyro Integration vs GT Yaw (30s segment)\ncorrelation r = {corr:.3f}')
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(PLOT_OUT, dpi=150)
print(f"  ✅ 验证图: {PLOT_OUT}")