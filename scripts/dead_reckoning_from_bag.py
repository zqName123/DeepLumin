#!/usr/bin/env python3
"""
直接从原始 bag 读取 /ouster/imu 做 Dead Reckoning 积分，
与 TF 真值对比，排除 npz 重采样的影响。
"""
import rosbag
import numpy as np
from scipy.spatial.transform import Slerp, Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os, sys

BAG_PATH = "/home/hhy/2004ros/data/2026-09-24-01-08-04.bag"
PLOT_OUT = "/home/hhy/2004ros/data/dead_reckoning_from_bag.png"

# yaw offset
YAW_OFFSET = -1.5707963
Q_OFFSET_INV = Rotation.from_euler('z', -YAW_OFFSET)

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_rotate(q, v):
    qv = np.array([0, v[0], v[1], v[2]])
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    q_rot = quat_multiply(quat_multiply(q, qv), q_conj)
    return q_rot[1:]

# ── 1. 读 bag ──
print(f"读取 bag: {BAG_PATH}")
bag = rosbag.Bag(BAG_PATH, 'r')

imu_t = []
imu_data = []  # [ax, ay, az, gx, gy, gz]

tf_t = []
tf_pos = []
tf_quat = []  # [w,x,y,z]

for topic, msg, t in bag.read_messages(topics=['/ouster/imu', '/tf']):
    ts = msg.header.stamp.to_sec() if hasattr(msg, 'header') else t.to_sec()
    
    if topic == '/ouster/imu':
        imu_t.append(ts)
        imu_data.append([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z,
                         msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])
    elif topic == '/tf':
        for tf_msg in msg.transforms:
            if tf_msg.child_frame_id == 'base_link' and tf_msg.header.frame_id == 'map':
                tf_t.append(ts)
                tf_pos.append([tf_msg.transform.translation.x,
                               tf_msg.transform.translation.y,
                               tf_msg.transform.translation.z])
                q = tf_msg.transform.rotation
                # 原始 tf 四元数（含 yaw offset）
                q_pub = np.array([q.w, q.x, q.y, q.z])
                # 去除 yaw offset
                r_pub = Rotation.from_quat([q.x, q.y, q.z, q.w])  # [x,y,z,w]
                r_true = Q_OFFSET_INV * r_pub
                q_true_scipy = r_true.as_quat()  # [x,y,z,w]
                tf_quat.append([q_true_scipy[3], q_true_scipy[0], q_true_scipy[1], q_true_scipy[2]])
                break

bag.close()

print(f"  IMU 帧数: {len(imu_t)}")
print(f"  TF 帧数:  {len(tf_t)}")

if len(imu_t) < 100 or len(tf_t) < 100:
    print("[ERROR] 数据不足")
    sys.exit(1)

# ── 2. 转换成 numpy ──
imu_t = np.array(imu_t)
imu_data = np.array(imu_data)
tf_t = np.array(tf_t)
tf_pos = np.array(tf_pos)
tf_quat = np.array(tf_quat)

# 按时间排序（bag 可能乱序）
sort_imu = np.argsort(imu_t)
imu_t = imu_t[sort_imu]
imu_data = imu_data[sort_imu]

sort_tf = np.argsort(tf_t)
tf_t = tf_t[sort_tf]
tf_pos = tf_pos[sort_tf]
tf_quat = tf_quat[sort_tf]

# ── 3. 时间范围 ──
t0 = max(imu_t[0], tf_t[0])
t1 = min(imu_t[-1], tf_t[-1])
print(f"\n  共同时间范围: {t0:.3f} ~ {t1:.3f} ({t1-t0:.1f}s)")

# ── 4. TF 插值到 IMU 时间戳 ──
# 先去重
_, unique_idx = np.unique(tf_t, return_index=True)
tf_t_u = tf_t[unique_idx]
tf_pos_u = tf_pos[unique_idx]
tf_quat_u = tf_quat[unique_idx]

print(f"  TF 去重后: {len(tf_t_u)} 帧")

# 只取共同时间范围内的 IMU
mask_imu = (imu_t >= t0) & (imu_t <= t1)
imu_t = imu_t[mask_imu]
imu_data = imu_data[mask_imu]
N = len(imu_t)
print(f"  有效 IMU 帧数: {N}")

# Slerp 插值
try:
    tf_slerp = Slerp(tf_t_u, Rotation.from_quat(tf_quat_u[:, [1,2,3,0]]))
    # 插值位置用线性
    tf_pos_interp = np.zeros((N, 3))
    for d in range(3):
        tf_pos_interp[:, d] = np.interp(imu_t, tf_t_u, tf_pos_u[:, d])
    tf_quat_interp = tf_slerp(imu_t).as_quat()  # [x,y,z,w]
    # 转成 [w,x,y,z]
    tf_quat_interp = np.column_stack([tf_quat_interp[:, 3], tf_quat_interp[:, 0],
                                       tf_quat_interp[:, 1], tf_quat_interp[:, 2]])
except Exception as e:
    print(f"  Slerp 失败: {e}")
    sys.exit(1)

# ── 5. 找静止段 ──
# 用 gyro 能量判断静止：连续 200 帧 gyro_norm < 0.05 rad/s
gyro_norm = np.linalg.norm(imu_data[:, 3:6], axis=1)
acc_norm = np.linalg.norm(imu_data[:, :3], axis=1)
stationary = (gyro_norm < 0.05) & (np.abs(acc_norm - 9.81) < 0.5)

from scipy.ndimage import label as label_nd
labeled, n_features = label_nd(stationary)

# 找最长的静止段
lengths = [(i, (labeled == i).sum()) for i in range(1, n_features + 1)]
lengths.sort(key=lambda x: x[1], reverse=True)

print(f"\n{'='*75}")
print(f"  找到 {n_features} 个静止段")
print(f"{'='*75}")

# 选一个长静止段做验证
best_static_idx = None
for seg_id, seg_len in lengths:
    if seg_len > 200:  # > 1s
        best_static_idx = seg_id
        break

if best_static_idx is not None:
    mask = labeled == best_static_idx
    idxs = np.where(mask)[0]
    seg_len = len(idxs)
    print(f"\n  最长静止段: {seg_len} 帧 ({seg_len/100:.1f}s) @ t={imu_t[idxs[0]]:.1f}s")
    
    # 取中间 0.5s 做 bias 估计
    mid = seg_len // 2
    half = min(50, seg_len // 2)
    bias_range = idxs[mid-half:mid+half]
    acc_bias_body = np.mean(imu_data[bias_range, :3], axis=0)
    gyro_bias = np.mean(imu_data[bias_range, 3:6], axis=0)
    q_gt_static = np.mean(tf_quat_interp[bias_range, :], axis=0)
    q_gt_static /= np.linalg.norm(q_gt_static)
    
    # body→world 的acc bias
    acc_bias_world = quat_rotate(q_gt_static, acc_bias_body) - np.array([0, 0, 9.81])
    
    print(f"  acc bias (body): [{acc_bias_body[0]:.4f}, {acc_bias_body[1]:.4f}, {acc_bias_body[2]:.4f}]")
    print(f"  gyro bias:       [{gyro_bias[0]:.5f}, {gyro_bias[1]:.5f}, {gyro_bias[2]:.5f}] rad/s")
    print(f"  acc bias (world): [{acc_bias_world[0]:.4f}, {acc_bias_world[1]:.4f}, {acc_bias_world[2]:.4f}]")
else:
    acc_bias_body = np.array([0.0, 0.0, 0.0])
    gyro_bias = np.array([0.0, 0.0, 0.0])
    acc_bias_world = np.array([0.0, 0.0, 0.0])
    bias_range = slice(0, min(50, N))
    print("  ⚠️  未找到静止段，用前 50 帧估计 bias")

# ── 6. 在静止段上做 Dead Reckoning ──
if best_static_idx is not None:
    print(f"\n{'='*75}")
    print(f"  静止段 Dead Reckoning 验证（使用全局 bias）")
    print(f"{'='*75}")
    
    mask = labeled == best_static_idx
    static_idxs = np.where(mask)[0]
    # 取 3 秒
    seg_len_3s = min(300, len(static_idxs))
    idxs = static_idxs[:seg_len_3s]
    
    ts_seg = imu_t[idxs]
    imu_seg = imu_data[idxs]
    pos_gt_seg = tf_pos_interp[idxs]
    quat_gt_seg = tf_quat_interp[idxs]
    
    N_seg = len(idxs)
    dt_seg = np.mean(np.diff(ts_seg))
    print(f"  帧数: {N_seg}, dt={dt_seg*1000:.1f}ms")
    print(f"  GT 位移: {np.linalg.norm(pos_gt_seg[-1] - pos_gt_seg[0]):.4f} m")
    
    # 姿态积分
    quat_imu = np.zeros((N_seg, 4))
    quat_imu[0] = quat_gt_seg[0]
    for i in range(1, N_seg):
        gyro = imu_seg[i-1, 3:6] - gyro_bias
        angle = gyro * dt_seg
        angle_norm = np.linalg.norm(angle)
        if angle_norm > 1e-10:
            axis = angle / angle_norm
            dq = Rotation.from_rotvec(angle_norm * axis).as_quat()
            dq_wxyz = np.array([dq[3], dq[0], dq[1], dq[2]])
            quat_imu[i] = quat_multiply(quat_imu[i-1], dq_wxyz)
        else:
            quat_imu[i] = quat_imu[i-1]
    
    # 位置积分（带 bias 修正，从静止估计的 bias）
    gravity = np.array([0, 0, 9.81])
    vel = np.zeros((N_seg, 3))
    pos_imu = np.zeros((N_seg, 3))
    pos_imu[0] = pos_gt_seg[0]
    
    for i in range(1, N_seg):
        q = quat_imu[i-1]
        a_body = imu_seg[i-1, :3]
        a_world = quat_rotate(q, a_body) - gravity - acc_bias_world
        vel[i] = vel[i-1] + a_world * dt_seg
        pos_imu[i] = pos_imu[i-1] + vel[i] * dt_seg
    
    pos_error = np.linalg.norm(pos_imu - pos_gt_seg, axis=1)
    print(f"  IMU 推算位移: {np.linalg.norm(pos_imu[-1] - pos_imu[0]):.4f} m")
    print(f"  终点位置差:   {pos_error[-1]:.4f} m")
    print(f"  最大位置差:   {np.max(pos_error):.4f} m")

# ── 7. 在 3 个不同时间段做 DR ──
print(f"\n{'='*75}")
print(f"  多段 Dead Reckoning 验证（从原始 bag）")
print(f"  [每段均从 GT 取初始速度，用局部 20 帧 bias 估计]")
print(f"{'='*75}")

# 选 3 个段
segments_info = []

# 段 1：最长静止段
if best_static_idx is not None:
    mask = labeled == best_static_idx
    static_idxs = np.where(mask)[0]
    seg_len_300 = min(300, len(static_idxs))
    segments_info.append(("静止段（最长）", static_idxs[:seg_len_300], "static"))
else:
    segments_info.append(("静止段", None, "static"))

# 段 2：前 1/3 处附近的运动段
for frac, name in [(0.3, "seg2"), (0.6, "seg3")]:
    search_idx = int(N * frac)
    found = False
    for offset in range(-500, 501):
        test_idx = search_idx + offset
        if 0 <= test_idx < N - 300 and not stationary[test_idx]:
            # 检查接下来 300 帧大部分是非静止
            end_idx = test_idx + 300
            non_static_ratio = 1.0 - (stationary[test_idx:end_idx].sum() / 300)
            if non_static_ratio > 0.7:  # 至少 70% 是非静止
                segments_info.append((f"运动段@{frac:.0%}", np.arange(test_idx, end_idx), name))
                found = True
                break
    if not found:
        # fallback：直接取连续 300 帧
        end_idx = min(search_idx + 300, N)
        segments_info.append((f"运动段@{frac:.0%}", np.arange(search_idx, end_idx), name))

fig, axes = plt.subplots(3, 3, figsize=(16, 14))
fig.suptitle('Dead Reckoning from Raw Bag (no resampling)', fontsize=14)

for seg_idx, (label, idxs, name) in enumerate(segments_info):
    if idxs is None or len(idxs) == 0:
        print(f"  {label}: 跳过（无数据）")
        continue
    
    N_seg = len(idxs)
    ts_seg = imu_t[idxs]
    imu_seg = imu_data[idxs]
    pos_gt_seg = tf_pos_interp[idxs]
    quat_gt_seg = tf_quat_interp[idxs]
    
    dt_avg = np.mean(np.diff(ts_seg))
    
    # 估计本段的 bias（前 20 帧）
    init_acc = np.mean(imu_seg[:min(20, N_seg), :3], axis=0)
    q_init = quat_gt_seg[0]
    acc_world_init = quat_rotate(q_init, init_acc)
    local_acc_bias = acc_world_init - np.array([0, 0, 9.81])
    
    # 姿态积分（带 gyro bias 修正）
    quat_imu = np.zeros((N_seg, 4))
    quat_imu[0] = quat_gt_seg[0]
    for i in range(1, N_seg):
        gyro = imu_seg[i-1, 3:6] - gyro_bias if np.linalg.norm(gyro_bias) > 0 else imu_seg[i-1, 3:6]
        angle = gyro * dt_avg
        angle_norm = np.linalg.norm(angle)
        if angle_norm > 1e-10:
            axis = angle / angle_norm
            dq = Rotation.from_rotvec(angle_norm * axis).as_quat()
            dq_wxyz = np.array([dq[3], dq[0], dq[1], dq[2]])
            quat_imu[i] = quat_multiply(quat_imu[i-1], dq_wxyz)
        else:
            quat_imu[i] = quat_imu[i-1]
    
    # 位置积分（初始速度从 GT 取）
    gravity = np.array([0, 0, 9.81])
    vel = np.zeros((N_seg, 3))
    pos_imu = np.zeros((N_seg, 3))
    pos_imu[0] = pos_gt_seg[0]
    
    # 初始速度从 GT 差分获得
    if N_seg > 1:
        vel[0] = (pos_gt_seg[1] - pos_gt_seg[0]) / dt_avg
    
    for i in range(1, N_seg):
        q = quat_imu[i-1]
        a_body = imu_seg[i-1, :3]
        a_world = quat_rotate(q, a_body) - gravity - local_acc_bias
        vel[i] = vel[i-1] + a_world * dt_avg
        pos_imu[i] = pos_imu[i-1] + vel[i] * dt_avg
    
    # 误差
    pos_error = np.linalg.norm(pos_imu - pos_gt_seg, axis=1)
    gt_disp = np.linalg.norm(pos_gt_seg[-1] - pos_gt_seg[0])
    imu_disp = np.linalg.norm(pos_imu[-1] - pos_imu[0])
    
    print(f"\n  {label} @ t={ts_seg[0]:.1f}s  ({N_seg}帧, {dt_avg*1000:.1f}ms/frame)")
    print(f"    GT 位移: {gt_disp:.4f} m | IMU 位移: {imu_disp:.4f} m | 终点差: {pos_error[-1]:.4f} m")
    print(f"    最大误差: {np.max(pos_error):.4f} m  初始 bias: [{local_acc_bias[0]:.4f}, {local_acc_bias[1]:.4f}, {local_acc_bias[2]:.4f}]")
    
    # 绘图
    ax_traj = axes[seg_idx, 0]
    ax_pos  = axes[seg_idx, 1]
    ax_err  = axes[seg_idx, 2]
    
    t_rel = ts_seg - ts_seg[0]
    
    ax_traj.plot(pos_gt_seg[:, 0], pos_gt_seg[:, 1], 'g-', lw=2, label='GT')
    ax_traj.plot(pos_imu[:, 0], pos_imu[:, 1], 'r--', lw=2, label='IMU DR')
    ax_traj.scatter(pos_gt_seg[0, 0], pos_gt_seg[0, 1], c='g', marker='o', s=60, zorder=5)
    ax_traj.scatter(pos_gt_seg[-1, 0], pos_gt_seg[-1, 1], c='g', marker='s', s=60, zorder=5)
    ax_traj.scatter(pos_imu[-1, 0], pos_imu[-1, 1], c='r', marker='x', s=60, zorder=5,
                    label=f'IMU end ({pos_error[-1]:.3f}m)')
    ax_traj.set_xlabel('x (m)')
    ax_traj.set_ylabel('y (m)')
    ax_traj.set_title(f'{label} ({name})')
    ax_traj.axis('equal')
    ax_traj.legend(fontsize=8)
    ax_traj.grid(alpha=0.3)
    
    ax_pos.plot(t_rel, pos_gt_seg[:, 0], 'g-', lw=1, label='GT x')
    ax_pos.plot(t_rel, pos_imu[:, 0], 'r--', lw=1, label='IMU x')
    ax_pos.plot(t_rel, pos_gt_seg[:, 1], 'g-', lw=1, alpha=0.5, label='GT y')
    ax_pos.plot(t_rel, pos_imu[:, 1], 'r--', lw=1, alpha=0.5, label='IMU y')
    ax_pos.set_xlabel('time (s)')
    ax_pos.set_ylabel('position (m)')
    ax_pos.set_title('X/Y Position')
    ax_pos.legend(fontsize=7)
    ax_pos.grid(alpha=0.3)
    
    ax_err.plot(t_rel, pos_error, 'b-', lw=1.5)
    ax_err.axhline(pos_error[-1], color='r', linestyle='--', alpha=0.5,
                   label=f'end error={pos_error[-1]:.3f}m')
    ax_err.set_xlabel('time (s)')
    ax_err.set_ylabel('error (m)')
    ax_err.set_title(f'Position Error')
    ax_err.legend(fontsize=8)
    ax_err.grid(alpha=0.3)

plt.tight_layout()
plt.savefig(PLOT_OUT, dpi=150)
print(f"\n  ✅ 验证图: {PLOT_OUT}")