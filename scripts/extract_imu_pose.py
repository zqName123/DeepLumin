#!/usr/bin/env python3
"""
从 bag 中提取 IMU 数据 + TF 真值位姿

用法：
  python3 extract_imu_pose.py [bag路径] [输出npz] [输出png]
  不带参数 → 使用下方 BAG_PATH / OUTPUT_NPZ / PLOT_PATH；
  只给 bag 不给输出路径 → 按 bag 文件名派生输出名（避免覆盖已有结果）。

流程：
  1. 读取 bag 中的 IMU 和 /tf
  2. TF 的 map→base_link 含 base_yaw_offset（-π/2），需转回真值
  3. 真值滤波：去尖峰（消除瞬时极大跳变）+ 平滑
  4. 插值到 IMU 时间戳，重采样到 200Hz
  5. 绘制轨迹验证
  6. 保存为 npz

真值滤波（仅作用于真值位姿，不影响 IMU）：
  去尖峰 —— 孤立的瞬时极大值（脉冲）判据为"往返量大 + 尖峰高度远超局部步长"，
            坏点用两端插值替换，只改该点、不影响其后轨迹，正常抖动（低于绝对下限）不处理。
  平滑   —— 高斯低通（无过冲、零相位）：位姿按关键帧/零阶保持下发时，插值后会带
            300mm/10ms 量级的假跳变；高斯平滑把这类瞬时跳变摊开，得到平滑真值。
            姿态在符号连续化后对四元数分量平滑再归一化（窗口内转角很小，等价于测地平滑）。
  注：不要对"帧间增量"做替换重积分 —— 本数据 300~500mm 的帧间位移是关键帧量化的
      真实运动（全轨迹 3125m），按跳变删掉增量会把真实位移整段消掉。

支持有 TF / 无 TF 两种 bag：
  无 map→base_link TF 时只提取 IMU，retargetted_pos / retargetted_quat 仍按
  (N,3) / (N,4) float64 写入 NaN 空值 —— key、形状、dtype、逐点对齐均与有真值时
  完全一致，下游加载代码无需区分，判空用 np.isnan(pos).all() 即可。

输出 npz 格式（4 个 key）：
  retargetted_ts    : (N,) float64  时间戳（秒）
  retargetted_imu   : (N, 6) float64 [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
  retargetted_pos   : (N, 3) float64 [x, y, z]                    （无 TF 时全 NaN）
  retargetted_quat  : (N, 4) float64 [qx, qy, qz, qw]             （xyzw，符号连续；无 TF 时全 NaN）
"""
import rosbag
import numpy as np
from scipy.spatial.transform import Slerp, Rotation
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 注册 3D projection
from tqdm import tqdm
import sys
import os

# ── 配置 ──────────────────────────────────────────
BAG_PATH = "/home/hhy/2004ros/data/2026-09-24-01-08-04.bag"
OUTPUT_NPZ = "/home/hhy/2004ros/data/imu_pose_200hz.npz"
PLOT_PATH = "/home/hhy/2004ros/data/imu_pose_trajectory.png"

IMU_TOPIC = "/ouster/imu"     # 根据实际话题名调整
TF_TOPIC = "/tf"
OUTPUT_HZ = 200              # 重采样目标频率

# base_yaw_offset = -π/2 → 逆旋转恢复真值
YAW_OFFSET = -1.5707963
Q_OFFSET = Rotation.from_euler('z', YAW_OFFSET)
Q_OFFSET_INV = Q_OFFSET.inv()  # 用于恢复真值

# ── 真值滤波配置（去尖峰 + 平滑）──────────────────
FILTER_TRUTH = True          # 总开关：对真值位姿滤波（不影响 IMU）
# 去尖峰（脉冲）
REL_FACTOR = 4.0             # 尖峰高度阈值 = REL_FACTOR × 局部步长
BACKTRACK = 0.5              # 脉冲判别：往返量占比 > 该值（"跳出去又跳回来"）
MIN_SPIKE_POS = 0.02         # 位置脉冲绝对下限 (m)，低于视为正常抖动不处理
MIN_SPIKE_ANG = 0.01         # 姿态脉冲绝对下限 (rad) ≈ 0.6°
LOCAL_WIN = 31               # 局部步长尺度的中值窗口（奇数，样本数）
# 平滑（在 OUTPUT_HZ 网格上做高斯低通，无过冲、零相位）
SMOOTH_SIGMA_MS = 50.0       # 高斯 σ (ms)；越大越平滑，≥ 关键帧下发间隔效果明显

# ── 命令行参数（可选）：[bag路径] [输出npz] [输出png] ────
if len(sys.argv) > 1:
    BAG_PATH = sys.argv[1]
    _stem = os.path.splitext(os.path.basename(BAG_PATH))[0]
    _dir = os.path.dirname(BAG_PATH) or "."
    OUTPUT_NPZ = sys.argv[2] if len(sys.argv) > 2 else \
        os.path.join(_dir, _stem + "_imu_pose_200hz.npz")
    PLOT_PATH = sys.argv[3] if len(sys.argv) > 3 else \
        os.path.splitext(OUTPUT_NPZ)[0] + ".png"


# ── 真值滤波工具 ──────────────────────────────────
def align_quat_continuity(quat):
    """消除 q / -q 歧义：使相邻四元数点积非负，分量连续不翻转"""
    q = np.array(quat, dtype=np.float64, copy=True)
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0.0:
            q[i] = -q[i]
    return q


def _slerp(q0, q1, t):
    """两点球面插值，t ∈ [0,1]"""
    q1 = np.array(q1, dtype=np.float64, copy=True)
    d = float(np.dot(q0, q1))
    if d < 0.0:                       # 走短弧
        q1, d = -q1, -d
    if d > 0.9995:                    # 近共线 → 线性插值
        q = q0 * (1.0 - t) + q1 * t
    else:
        th = np.arccos(np.clip(d, -1.0, 1.0))
        q = (np.sin((1.0 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


def local_step_scale(step_norm, win=LOCAL_WIN):
    """局部步长尺度（滑动中值），用于相对阈值"""
    if len(step_norm) == 0:
        return step_norm
    win = max(1, int(win) | 1)
    if win > len(step_norm):
        win = max(1, len(step_norm) | 1)
    half = win // 2
    out = np.empty_like(step_norm)
    for i in range(len(step_norm)):
        lo, hi = max(0, i - half), min(len(step_norm), i + half + 1)
        out[i] = np.median(step_norm[lo:hi])
    return np.maximum(out, 1e-12)


def despike_position(ts, pos, rel=REL_FACTOR, backtrack=BACKTRACK,
                     min_spike=MIN_SPIKE_POS, win=LOCAL_WIN):
    """
    位置去脉冲尖峰，返回 (pos_clean, n_pulse)：
      判据（同时满足）：相邻三点往返量大（跳出去又跳回来）且尖峰高度
      超过 max(rel × 局部步长, min_spike)。坏点用两端插值替换，不影响其后轨迹。
    """
    p = np.asarray(pos, dtype=np.float64)
    n = len(p)
    if n < 3:
        return p.copy(), 0
    p = p.copy()
    n_pulse = 0
    for _ in range(2):                       # 迭代以处理相邻坏点
        sloc = local_step_scale(np.linalg.norm(np.diff(p, axis=0), axis=1), win)
        flags = np.zeros(n, dtype=bool)
        pred = p.copy()
        for i in range(1, n - 1):
            a, b, c = p[i - 1], p[i], p[i + 1]
            d1 = np.linalg.norm(b - a)
            d2 = np.linalg.norm(c - b)
            d_span = np.linalg.norm(c - a)
            # 往返量：脉冲 ≈ d1+d2；正常运动/台阶 ≈ |d1-d2|
            if (d1 + d2 - d_span) <= backtrack * (d1 + d2 + 1e-15):
                continue
            t = d1 / (d1 + d2 + 1e-15)
            pr = a + t * (c - a)             # 两端插值预测
            if np.linalg.norm(b - pr) > max(rel * sloc[i - 1], min_spike):
                flags[i] = True
                pred[i] = pr
        if not flags.any():
            break
        p[flags] = pred[flags]
        n_pulse += int(flags.sum())
    return p, n_pulse


def despike_orientation(ts, quat_xyzw, rel=REL_FACTOR, backtrack=BACKTRACK,
                        min_spike=MIN_SPIKE_ANG, win=LOCAL_WIN):
    """
    姿态去脉冲尖峰，返回 (quat_clean, n_pulse)：
      先做半球连续化消除 q/-q 造成的假跳变，其余同位置（测地线代替直线距离）。
    """
    q = align_quat_continuity(quat_xyzw)
    n = len(q)
    if n < 3:
        return q, 0
    n_pulse = 0

    def ang(u, v):
        return 2.0 * np.arccos(np.clip(abs(float(np.dot(u, v))), 0.0, 1.0))

    for _ in range(2):
        rot = Rotation.from_quat(q)
        step_norm = np.linalg.norm((rot[:-1].inv() * rot[1:]).as_rotvec(), axis=1)
        sloc = local_step_scale(step_norm, win)
        flags = np.zeros(n, dtype=bool)
        pred = q.copy()
        for i in range(1, n - 1):
            a, b, c = q[i - 1], q[i], q[i + 1]
            d1, d2, d_span = ang(a, b), ang(b, c), ang(a, c)
            if (d1 + d2 - d_span) <= backtrack * (d1 + d2 + 1e-15):
                continue
            t = d1 / (d1 + d2 + 1e-15)
            pr = _slerp(a, c, t)             # 测地插值预测
            if ang(b, pr) > max(rel * sloc[i - 1], min_spike):
                flags[i] = True
                pred[i] = pr
        if not flags.any():
            break
        q[flags] = pred[flags]
        q = align_quat_continuity(q)
        n_pulse += int(flags.sum())
    return q, n_pulse


def smooth_series(x, sigma_samples):
    """
    高斯低通平滑（无过冲、零相位）。
    边缘用最小二乘趋势线性外推填充，避免端点被拉向内部。
    """
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    if sigma_samples <= 0 or n < 3:
        return x.copy()
    pad = int(np.ceil(3.0 * sigma_samples)) + 1
    k = min(n, max(2, int(sigma_samples)))               # 趋势拟合窗口（≥2 点才能拟直线）
    tt = np.arange(k, dtype=np.float64)
    slope_l = np.polyfit(tt, x[:k], 1)[0]                # (k, D) → 一次项
    slope_r = np.polyfit(tt, x[-k:], 1)[0]
    left = x[0] - slope_l * np.arange(pad, 0, -1)[:, None]
    right = x[-1] + slope_r * np.arange(1, pad + 1)[:, None]
    xpad = np.concatenate([left, x, right], axis=0)
    y = gaussian_filter1d(xpad, sigma_samples, axis=0, mode='nearest')
    return y[pad:pad + n]


def smooth_position(pos, sigma_samples):
    """位置高斯平滑"""
    return smooth_series(np.asarray(pos, dtype=np.float64), sigma_samples)


def smooth_orientation(quat_xyzw, sigma_samples):
    """姿态平滑：符号连续化后对四元数分量做高斯平滑，再归一化"""
    q = align_quat_continuity(quat_xyzw)
    q = smooth_series(q, sigma_samples)
    q = align_quat_continuity(q)
    q /= np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
    return q


# ── 1. 读取 bag ──────────────────────────────────
if not os.path.exists(BAG_PATH):
    print(f"[ERROR] bag 不存在: {BAG_PATH}")
    sys.exit(1)

print(f"正在读取: {BAG_PATH}")
bag = rosbag.Bag(BAG_PATH)

# ── 2. 解析 TF（map→base_link） ───────────────────
print("  解析 TF ...")
tf_timestamps = []
tf_positions = []
tf_quaternions = []

for topic, msg, t in bag.read_messages(topics=[TF_TOPIC]):
    for transform in msg.transforms:
        # 只取 map→base_link（或 map→base_footprint，根据实际 frame_id）
        if transform.header.frame_id == "map" and transform.child_frame_id == "base_link":
            ts = transform.header.stamp.to_sec()
            t_ = transform.transform.translation
            q_ = transform.transform.rotation
            tf_timestamps.append(ts)
            tf_positions.append([t_.x, t_.y, t_.z])
            tf_quaternions.append([q_.x, q_.y, q_.z, q_.w])  # [x,y,z,w]，与 scipy 一致

tf_timestamps = np.array(tf_timestamps)
tf_positions = np.array(tf_positions, dtype=np.float64)
tf_quaternions = np.array(tf_quaternions, dtype=np.float64)

# 去重 + 排序（TF 可能有相同时间戳的重复消息）
_, unique_idx = np.unique(tf_timestamps, return_index=True)
tf_timestamps = tf_timestamps[unique_idx]
tf_positions = tf_positions[unique_idx]
tf_quaternions = tf_quaternions[unique_idx]
sort_idx = np.argsort(tf_timestamps)
tf_timestamps = tf_timestamps[sort_idx]
tf_positions = tf_positions[sort_idx]
tf_quaternions = tf_quaternions[sort_idx]

print(f"    TF 帧数: {len(tf_timestamps)}（去重后）")
has_truth = len(tf_timestamps) > 0
if has_truth:
    print(f"    时间范围: {tf_timestamps[0]:.3f} ~ {tf_timestamps[-1]:.3f}")
else:
    print("    [WARN] 未找到 map→base_link 的 TF → 无真值，retargetted_pos/quat 将置 NaN 空值")

# ── 3. 去除 base_yaw_offset ───────────────────────
# published_q = q_offset * true_q  →  true_q = q_offset_inv * published_q
# 位置不受影响
if has_truth:
    print("  去除 yaw offset ...")
    r_pub = Rotation.from_quat(tf_quaternions)            # [x,y,z,w]
    tf_quaternions = (Q_OFFSET_INV * r_pub).as_quat()     # [x,y,z,w]

# ── 3.5 真值去尖峰（消除瞬时极大跳变）──────────────
n_pulse_pos = n_pulse_rot = 0
if FILTER_TRUTH and has_truth:
    print("  真值去尖峰 ...")
    tf_positions, n_pulse_pos = despike_position(tf_timestamps, tf_positions)
    tf_quaternions, n_pulse_rot = despike_orientation(tf_timestamps, tf_quaternions)
    print(f"    脉冲尖峰: 位置 {n_pulse_pos} 处, 姿态 {n_pulse_rot} 处")

# ── 4. 解析 IMU ────────────────────────────────────
print("  解析 IMU ...")
imu_timestamps = []
imu_data = []  # [ax, ay, az, gx, gy, gz]

for topic, msg, t in bag.read_messages(topics=[IMU_TOPIC]):
    ts = msg.header.stamp.to_sec()
    acc = msg.linear_acceleration
    gyro = msg.angular_velocity
    imu_timestamps.append(ts)
    imu_data.append([acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z])

imu_timestamps = np.array(imu_timestamps)
imu_data = np.array(imu_data, dtype=np.float64)

# IMU 也去重
_, imu_unique_idx = np.unique(imu_timestamps, return_index=True)
imu_timestamps = imu_timestamps[imu_unique_idx]
imu_data = imu_data[imu_unique_idx]
sort_idx = np.argsort(imu_timestamps)
imu_timestamps = imu_timestamps[sort_idx]
imu_data = imu_data[sort_idx]

print(f"    IMU 帧数: {len(imu_timestamps)}（去重后）")
print(f"    时间范围: {imu_timestamps[0]:.3f} ~ {imu_timestamps[-1]:.3f}")

if len(imu_timestamps) == 0:
    print("[ERROR] 未找到 IMU 数据")
    bag.close()
    sys.exit(1)

bag.close()

# ── 5. 取交集时间范围 ──────────────────────────────
# 留 1ms 边距避免浮点精度导致 SLERP 越界
EPS = 0.001
if has_truth:
    t_start = max(tf_timestamps[0], imu_timestamps[0]) + EPS
    t_end = min(tf_timestamps[-1], imu_timestamps[-1]) - EPS
else:
    t_start = imu_timestamps[0] + EPS
    t_end = imu_timestamps[-1] - EPS
print(f"\n  交集时间范围: {t_start:.3f} ~ {t_end:.3f} (时长 {t_end-t_start:.1f}s)")

if t_start >= t_end:
    print("[ERROR] TF 和 IMU 无重叠时间")
    sys.exit(1)

# ── 6. 创建 200Hz 时间网格 ────────────────────────
dt = 1.0 / OUTPUT_HZ
ts_grid = np.arange(t_start, t_end, dt)  # 严格在 [t_start, t_end) 内
if has_truth:
    # 钳制到 SLERP 有效的范围内（留 1us 边距）
    EPS_SLERP = 1e-6
    tf_min, tf_max = tf_timestamps[0] + EPS_SLERP, tf_timestamps[-1] - EPS_SLERP
    ts_grid = ts_grid[(ts_grid >= tf_min) & (ts_grid <= tf_max)]
N_grid = len(ts_grid)
print(f"  重采样网格: {N_grid} 点 @ {OUTPUT_HZ}Hz (dt={dt*1000:.1f}ms)")
print(f"  网格范围: {ts_grid[0]:.6f} ~ {ts_grid[-1]:.6f}")

# ── 7. 插值：真值位姿 ──────────────────────────────
if has_truth:
    print("  插值真值位姿 ...")
    # 位置：线性插值
    pos_interp = interp1d(tf_timestamps, tf_positions.T, kind='linear',
                           bounds_error=False, fill_value='extrapolate')
    pos_grid = pos_interp(ts_grid).T  # (N, 3)

    # 姿态：SLERP
    rotations = Rotation.from_quat(tf_quaternions)  # [x,y,z,w]
    slerp = Slerp(tf_timestamps, rotations)
    rot_grid = slerp(ts_grid)
    quat_grid = align_quat_continuity(rot_grid.as_quat())  # [x,y,z,w]，符号连续

    # ── 7.5 真值平滑（消除插值后的瞬时跳变）──────────
    if FILTER_TRUTH:
        sigma_samples = SMOOTH_SIGMA_MS * 1e-3 / dt
        print(f"  真值平滑 ... (高斯 σ = {SMOOTH_SIGMA_MS:.0f}ms = {sigma_samples:.1f} 点)")

        def _peak_stats(p, q):
            v = np.linalg.norm(np.diff(p, axis=0), axis=1) / dt
            w = np.linalg.norm((Rotation.from_quat(q[:-1]).inv() *
                                Rotation.from_quat(q[1:])).as_rotvec(), axis=1) / dt
            return v.max(), np.degrees(w.max())

        v0, w0 = _peak_stats(pos_grid, quat_grid)
        pos_grid = smooth_position(pos_grid, sigma_samples)
        quat_grid = smooth_orientation(quat_grid, sigma_samples)
        v1, w1 = _peak_stats(pos_grid, quat_grid)
        print(f"    峰值线速度 {v0:.1f} → {v1:.1f} m/s | 峰值角速度 {w0:.1f} → {w1:.1f} deg/s")
else:
    # 无真值：key / 形状 / dtype 保持一致，内容为 NaN 空值
    pos_grid = np.full((N_grid, 3), np.nan)
    quat_grid = np.full((N_grid, 4), np.nan)

# ── 8. 插值：IMU ───────────────────────────────────
print("  插值 IMU ...")
imu_interp = interp1d(imu_timestamps, imu_data.T, kind='linear',
                      bounds_error=False, fill_value='extrapolate')
imu_grid = imu_interp(ts_grid).T  # (N, 6)

# ── 9. 组装输出 ────────────────────────────────────
# 去除首尾可能存在 extrapolate 的点（丢弃没有原始数据支撑的部分）
# 保留 ts_grid 中完全被原始数据包络的部分
if has_truth:
    valid_mask = (ts_grid >= tf_timestamps[0]) & (ts_grid <= tf_timestamps[-1]) & \
                 (ts_grid >= imu_timestamps[0]) & (ts_grid <= imu_timestamps[-1])
else:
    valid_mask = np.ones(N_grid, dtype=bool)

ts_out = ts_grid[valid_mask]
imu_out = imu_grid[valid_mask]
pos_out = pos_grid[valid_mask]
quat_out = align_quat_continuity(quat_grid[valid_mask]) if has_truth else quat_grid[valid_mask]

print(f"\n  有效输出: {len(ts_out)} 帧（丢弃首尾外推部分 {N_grid - len(ts_out)} 帧）")

# ── 10. 保存 npz ──────────────────────────────────
np.savez(OUTPUT_NPZ,
         retargetted_ts=ts_out,
         retargetted_imu=imu_out,
         retargetted_pos=pos_out,
         retargetted_quat=quat_out)
print(f"\n  ✅ 已保存: {OUTPUT_NPZ}")
file_size_mb = os.path.getsize(OUTPUT_NPZ) / 1024 / 1024
print(f"     文件大小: {file_size_mb:.1f} MB")

# ── 11. 验证并绘制轨迹 ────────────────────────────
print("\n  绘制轨迹图 ...")
fig, axes = plt.subplots(2, 3, figsize=(18, 10))
# 将 (0,1) 替换为 3D 子图
fig.delaxes(axes[0, 1])
ax1 = fig.add_subplot(2, 3, 2, projection='3d')
fig.suptitle(f'IMU + Pose Dataset Verification\n'
             f'{len(ts_out)} samples @ {OUTPUT_HZ}Hz | '
             f'{ts_out[-1]-ts_out[0]:.1f}s duration',
             fontsize=14)

# 11a. XY 轨迹俯视图 / 11b. 3D 轨迹 / 11c. 位置 vs 时间
ax0 = axes[0, 0]
ax2 = axes[0, 2]
if has_truth:
    sc = ax0.scatter(pos_out[:, 0], pos_out[:, 1], c=ts_out - ts_out[0],
                     cmap='jet', s=2, alpha=0.7)
    ax0.set_xlabel('x (m)')
    ax0.set_ylabel('y (m)')
    ax0.set_title('Trajectory (XY)')
    ax0.grid(True, alpha=0.3)
    ax0.axis('equal')
    plt.colorbar(sc, ax=ax0, label='time (s)')

    sc3d = ax1.scatter(pos_out[:, 0], pos_out[:, 1], pos_out[:, 2],
                       c=ts_out - ts_out[0], cmap='jet', s=2, alpha=0.7)
    ax1.set_xlabel('x (m)')
    ax1.set_ylabel('y (m)')
    ax1.set_zlabel('z (m)')
    ax1.set_title('Trajectory (3D)')
    plt.colorbar(sc3d, ax=ax1, label='time (s)')

    ax2.plot(ts_out - ts_out[0], pos_out[:, 0], label='x', linewidth=0.8)
    ax2.plot(ts_out - ts_out[0], pos_out[:, 1], label='y', linewidth=0.8)
    ax2.plot(ts_out - ts_out[0], pos_out[:, 2], label='z', linewidth=0.8)
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('position (m)')
    ax2.set_title('Position vs Time')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
else:
    def _no_truth_placeholder(ax, name):
        # 3D 轴的 text() 要 x,y,z，用 text2D 走轴坐标
        txt = getattr(ax, 'text2D', ax.text)
        txt(0.5, 0.5, 'No ground truth\n(no map->base_link TF)', ha='center', va='center',
            transform=ax.transAxes, fontsize=13, color='gray')
        ax.set_title(name)
        ax.grid(True, alpha=0.3)

    _no_truth_placeholder(ax0, 'Trajectory (XY)')
    _no_truth_placeholder(ax1, 'Trajectory (3D)')
    _no_truth_placeholder(ax2, 'Position vs Time')

# 11d. 加速度 vs 时间
ax3 = axes[1, 0]
ax3.plot(ts_out - ts_out[0], imu_out[:, 0], label='acc_x', linewidth=0.8)
ax3.plot(ts_out - ts_out[0], imu_out[:, 1], label='acc_y', linewidth=0.8)
ax3.plot(ts_out - ts_out[0], imu_out[:, 2], label='acc_z', linewidth=0.8)
ax3.set_xlabel('time (s)')
ax3.set_ylabel('acceleration (m/s²)')
ax3.set_title('Accelerometer')
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)

# 11e. 角速度 vs 时间
ax4 = axes[1, 1]
ax4.plot(ts_out - ts_out[0], imu_out[:, 3], label='gyro_x', linewidth=0.8)
ax4.plot(ts_out - ts_out[0], imu_out[:, 4], label='gyro_y', linewidth=0.8)
ax4.plot(ts_out - ts_out[0], imu_out[:, 5], label='gyro_z', linewidth=0.8)
ax4.set_xlabel('time (s)')
ax4.set_ylabel('angular vel (rad/s)')
ax4.set_title('Gyroscope')
ax4.legend(fontsize=8)
ax4.grid(True, alpha=0.3)

# 11f. 姿态（欧拉角）vs 时间
ax5 = axes[1, 2]
if has_truth:
    # 连续展开，避免 ±180° 缠绕在图上显示成假跳变
    euler = np.degrees(np.unwrap(Rotation.from_quat(quat_out).as_euler('xyz'), axis=0))  # quat_out: [x,y,z,w]
    ax5.plot(ts_out - ts_out[0], euler[:, 0], label='roll', linewidth=0.8)
    ax5.plot(ts_out - ts_out[0], euler[:, 1], label='pitch', linewidth=0.8)
    ax5.plot(ts_out - ts_out[0], euler[:, 2], label='yaw', linewidth=0.8)
    ax5.set_xlabel('time (s)')
    ax5.set_ylabel('angle (deg, unwrapped)')
    ax5.set_title('Orientation (Euler)')
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)
else:
    ax5.text(0.5, 0.5, 'No ground truth\n(no map->base_link TF)', ha='center', va='center',
             transform=ax5.transAxes, fontsize=13, color='gray')
    ax5.set_title('Orientation (Euler)')
    ax5.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(PLOT_PATH, dpi=150)
print(f"  ✅ 轨迹图已保存: {PLOT_PATH}")

# ── 12. 打印统计摘要 ──────────────────────────────
print(f"\n{'='*60}")
print(f"  Dataset Summary")
print(f"{'='*60}")
print(f"  Samples:          {len(ts_out)}")
print(f"  Duration:         {ts_out[-1] - ts_out[0]:.2f} s")
print(f"  Sample rate:      {OUTPUT_HZ} Hz")
print(f"  IMU dims:         acc(3) + gyro(3) = 6")
print(f"  Pose dims:        pos(3) + quat(4) = 7")
print(f"  Quat layout:      [qx, qy, qz, qw] (xyzw)")
if has_truth:
    step_pos_out = np.linalg.norm(np.diff(pos_out, axis=0), axis=1)
    q_dot = np.abs(np.sum(quat_out[:-1] * quat_out[1:], axis=1))
    step_ang_out = 2.0 * np.arccos(np.clip(q_dot, 0.0, 1.0))
    print(f"  Truth:            map→base_link TF, 已滤波")
    print(f"  Truth filter:     pulse pos={n_pulse_pos}, rot={n_pulse_rot} | "
          f"gauss σ={SMOOTH_SIGMA_MS:.0f}ms")
    print(f"  Path length:      {step_pos_out.sum():.1f} m")
    print(f"  Max frame step:   pos {step_pos_out.max()*1000:.2f} mm, "
          f"ang {np.degrees(step_ang_out.max()):.3f} deg")
    print(f"  Yaw offset:       {YAW_OFFSET:.4f} rad (已去除)")
else:
    print(f"  Truth:            无 (未找到 map→base_link TF)")
    print(f"                    retargetted_pos / retargetted_quat = NaN 空值 ({len(ts_out)}×3 / {len(ts_out)}×4)")
print(f"  Time range:       {ts_out[0]:.3f} ~ {ts_out[-1]:.3f}")
print(f"  Output file:      {OUTPUT_NPZ}")
print(f"  Output keys:      retargetted_ts, retargetted_imu,")
print(f"                    retargetted_pos, retargetted_quat")
print(f"{'='*60}")
