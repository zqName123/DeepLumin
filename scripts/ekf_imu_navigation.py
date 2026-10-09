#!/usr/bin/env python3
"""
ESKF IMU 导航（15 维: p, v, θ, ba, bg）
- 从真值起点（第 0 帧 p/q/v）开始，用 IMU 数据做 ESKF 递推
- 关闭 ZUPT
- ba / bg 持续更新：初始值取自真值判定的真正静止段（只估一次），
  之后在真值静止段内做正常 ESKF 全状态更新（观测 acc/gyro，非 ZUPT）
- 默认只递推前 T_END=20s 便于查看轨迹（设 None 跑全段）
- 绘图时把 IMU 纯递推起点与真值起点对齐，便于逐维度(x/y/z)比较
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import median_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os, sys

NPZ_PATH = "/home/hhy/2004ros/data/imu_pose_200hz.npz"
PLOT_OUT = "/home/hhy/2004ros/data/ekf_imu_navigation.png"
SAVE_NPZ = "/home/hhy/2004ros/data/ekf_imu_result.npz"

T_END = 20.0        # 只递推前 N 秒；None = 全段
USE_ZUPT = False    # 关闭 ZUPT

if not os.path.exists(NPZ_PATH):
    print(f"[ERROR] {NPZ_PATH} 不存在")
    sys.exit(1)

data = np.load(NPZ_PATH)
ts      = data["retargetted_ts"]
imu     = data["retargetted_imu"]   # [ax, ay, az, gx, gy, gz]
pos_gt  = data["retargetted_pos"]
quat_gt = data["retargetted_quat"]  # [w,x,y,z]

if T_END is not None:
    n_end = int(np.searchsorted(ts - ts[0], T_END, side='right'))
    n_end = max(n_end, 2)
    ts      = ts[:n_end]
    imu     = imu[:n_end]
    pos_gt  = pos_gt[:n_end]
    quat_gt = quat_gt[:n_end]

FS = 200.0
dt = 1.0 / FS
N  = len(ts)
t  = ts - ts[0]
print(f"数据: {N} 帧, dt={dt*1000:.1f}ms, 时长={t[-1]:.1f}s"
      + (f"（只递推前 {T_END}s）" if T_END is not None else ""))

# ── 工具函数 ──────────────────────────────────────────
def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def qmul(q1, q2):
    w1,x1,y1,z1 = q1; w2,x2,y2,z2 = q2
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])

def qrot(q, v):
    qv = np.array([0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return qmul(qmul(q, qv), qc)[1:]

def qnorm(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-15 else np.array([1,0,0,0])

def q2R(q):
    w,x,y,z = q
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
                     [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
                     [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y]])

def rotvec_to_quat(rv):
    """旋转向量 → 四元数 [w,x,y,z]"""
    angle = np.linalg.norm(rv)
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    dq = Rotation.from_rotvec(rv).as_quat()  # [x,y,z,w]
    return np.array([dq[3], dq[0], dq[1], dq[2]])

def find_first_run(mask, min_len):
    """第一个连续 True 段 [start, end)"""
    run_start = None
    for i in range(len(mask)):
        if mask[i]:
            if run_start is None:
                run_start = i
        else:
            if run_start is not None and i - run_start >= min_len:
                return run_start, i
            run_start = None
    if run_start is not None and len(mask) - run_start >= min_len:
        return run_start, len(mask)
    return None, None

g = np.array([0.0, 0.0, 9.81])

# ══════════════════════════════════════════════════════
# 1. 真值检测真正静止段
# ══════════════════════════════════════════════════════
print("\n[1/4] 用真值检测真正静止段 ...")

WIN         = 200                 # 1s 窗口
DISP_THRESH = 0.05                # m，窗口内 GT 位移
ROT_THRESH  = np.radians(2.0)     # 窗口内 GT 转角
MIN_RUN     = 200                 # 静止段至少 1s

disp_1s  = np.full(N, np.nan)
rot_1s   = np.full(N, np.nan)
for i in range(WIN, N):
    disp_1s[i] = np.linalg.norm(pos_gt[i] - pos_gt[i-WIN])
    r0 = Rotation.from_quat([quat_gt[i-WIN,1], quat_gt[i-WIN,2],
                             quat_gt[i-WIN,3], quat_gt[i-WIN,0]])
    r1 = Rotation.from_quat([quat_gt[i,1], quat_gt[i,2],
                             quat_gt[i,3], quat_gt[i,0]])
    rot_1s[i] = np.linalg.norm((r0.inv() * r1).as_rotvec())

is_stationary = (disp_1s < DISP_THRESH) & (rot_1s < ROT_THRESH)

stat_start, stat_end = find_first_run(is_stationary, MIN_RUN)
if stat_start is None:
    print("  [WARN] 未找到静止段，放宽阈值重试 ...")
    DISP_THRESH, ROT_THRESH = 0.1, np.radians(3.0)
    is_stationary = (disp_1s < DISP_THRESH) & (rot_1s < ROT_THRESH)
    stat_start, stat_end = find_first_run(is_stationary, MIN_RUN)

if stat_start is None:
    print("  [ERROR] 真值未检测到真正静止段，无法估计初始 bias")
    sys.exit(1)

stat_count = int(np.sum(is_stationary))
stat_end_t = t[min(stat_end, N - 1)]
print(f"  真值静止帧: {stat_count}/{N} ({stat_count/N*100:.1f}%)")
print(f"  第一个静止段: frame {stat_start} ~ {stat_end} "
      f"({(stat_end-stat_start)/FS:.2f}s, t={t[stat_start]:.2f}~{stat_end_t:.2f}s)")

# ══════════════════════════════════════════════════════
# 2. 初始 bias（真值静止段，只估一次）
# ══════════════════════════════════════════════════════
print("\n[2/4] 估计初始 bias（真值静止段，只估一次）...")

seg_imu  = imu[stat_start:stat_end]
seg_quat = quat_gt[stat_start:stat_end]

# bg: 静止时 gyro 应为 0，残差即 bias
bg_init = np.mean(seg_imu[:, 3:6], axis=0)

# ba: 静止时 acc = R^T * g + ba → ba = acc - R^T * g
ba_sum = np.zeros(3)
for i in range(len(seg_imu)):
    q_ = seg_quat[i]
    R_ = Rotation.from_quat([q_[1], q_[2], q_[3], q_[0]]).as_matrix()
    ba_sum += seg_imu[i, :3] - R_.T @ g
ba_init = ba_sum / len(seg_imu)

print(f"  ba_init: [{ba_init[0]:.6f}, {ba_init[1]:.6f}, {ba_init[2]:.6f}] m/s²")
print(f"  bg_init: [{bg_init[0]:.6f}, {bg_init[1]:.6f}, {bg_init[2]:.6f}] rad/s")

# ══════════════════════════════════════════════════════
# 3. 初始状态（真值起点）
# ══════════════════════════════════════════════════════
print("\n[3/4] 初始状态（真值起点）...")

# 对 GT 位置做中值滤波平滑，再差分得初始速度
pos_smooth = np.copy(pos_gt)
for axis in range(3):
    pos_smooth[:, axis] = median_filter(pos_gt[:, axis], size=41)
vel_gt_smooth = np.gradient(pos_smooth, ts, axis=0)

n_v = min(100, N)
vel_init = np.mean(vel_gt_smooth[:n_v], axis=0)
vel_mag  = np.linalg.norm(vel_init)
if vel_mag < 0.5:
    print(f"  初始速度 ≈ {vel_mag:.3f} m/s (< 0.5)，设为 0")
    vel_init = np.zeros(3)
else:
    print(f"  初始速度: [{vel_init[0]:.3f}, {vel_init[1]:.3f}, {vel_init[2]:.3f}]  |v|={vel_mag:.3f} m/s")

# 起点 = 真值第 0 帧
p_init = np.copy(pos_gt[0])
q_init = np.copy(quat_gt[0])

print(f"  初始 p: [{p_init[0]:.2f}, {p_init[1]:.2f}, {p_init[2]:.2f}]")
print(f"  初始 q: [{q_init[0]:.6f}, {q_init[1]:.6f}, {q_init[2]:.6f}, {q_init[3]:.6f}]")
print(f"  初始 v: [{vel_init[0]:.3f}, {vel_init[1]:.3f}, {vel_init[2]:.3f}]")

# ══════════════════════════════════════════════════════
# 4. ESKF 递推（关闭 ZUPT，ba/bg 持续更新）
# ══════════════════════════════════════════════════════
print(f"\n[4/4] ESKF 递推（ZUPT={'开' if USE_ZUPT else '关'}，从真值起点开始）...")

# ── 状态初始化 ──
p   = p_init.copy()
v   = vel_init.copy()
q   = q_init.copy()
ba  = ba_init.copy()
bg  = bg_init.copy()

# ── 协方差初始化 ──
P = np.eye(15)
P[0:3, 0:3]   = np.eye(3) * 1e-4     # p:  信任真值起点 (0.01m std)
P[3:6, 3:6]   = np.eye(3) * 2.5e-3   # v:  GT 差分有噪声 (0.05m/s std)
P[6:9, 6:9]   = np.eye(3) * 1e-4     # θ:  信任真值起点 (0.01rad std)
P[9:12, 9:12]  = np.eye(3) * 1e-4    # ba: 信任初始估计 (0.01 m/s² std)
P[12:15, 12:15] = np.eye(3) * 1e-8   # bg: 信任初始估计 (0.0001 rad/s std)

# ── 过程噪声 ──
sigma_a  = 0.1     # m/s²/√Hz
sigma_g  = 0.01    # rad/s/√Hz
sigma_ba = 1e-2    # m/s²/√Hz（允许 ba 缓慢漂移 → 持续更新）
sigma_bg = 1e-4    # rad/s/√Hz（允许 bg 缓慢漂移 → 持续更新）
Q_c = np.diag([sigma_a**2]*3 + [sigma_g**2]*3 +
              [sigma_ba**2]*3 + [sigma_bg**2]*3)

# ── bias 限幅范围（物理合理范围）──
BA_LIM = np.array([1.0, 1.0, 1.0])     # m/s²
BG_LIM = np.array([0.1, 0.1, 0.1])     # rad/s

# ── ZUPT 观测噪声（USE_ZUPT=False 时不使用）──
sigma_v = 0.03     # m/s
R_zupt = np.eye(3) * sigma_v**2
H_zupt = np.zeros((3, 15))
H_zupt[0:3, 3:6] = np.eye(3)

# ── 静止段观测噪声（acc = R^T g + ba, gyro = bg）──
R_bias = np.diag([0.1**2]*3 + [0.005**2]*3)

I15 = np.eye(15)
I3  = np.eye(3)

# ── 存储（从真值起点开始全程有效）──
pos_ekf  = np.zeros((N, 3))
vel_ekf  = np.zeros((N, 3))
quat_ekf = np.zeros((N, 4))
ba_ekf   = np.zeros((N, 3))
bg_ekf   = np.zeros((N, 3))

pos_ekf[0]  = p
vel_ekf[0]  = v
quat_ekf[0] = q
ba_ekf[0]   = ba
bg_ekf[0]   = bg

zupt_count = 0
bias_update_count = 0

for i in range(1, N):
    acc  = imu[i-1, :3]
    gyro = imu[i-1, 3:6]
    dt_i = ts[i] - ts[i-1]

    # ── Nominal State 传播 ──
    # 姿态
    w = gyro - bg
    angle = w * dt_i
    if np.linalg.norm(angle) > 1e-12:
        dq_wxyz = rotvec_to_quat(angle)
        q = qnorm(qmul(q, dq_wxyz))
    # 速度
    a_body  = acc - ba
    R_mat   = q2R(q)
    a_world = R_mat @ a_body - g
    v = v + a_world * dt_i
    # 位置
    p = p + v * dt_i

    # ── 误差状态协方差传播 ──
    acc_skew = skew(a_body)
    Fd = np.eye(15)
    Fd[0:3, 3:6]   = I3 * dt_i
    Fd[3:6, 6:9]   = -R_mat @ acc_skew * dt_i
    Fd[3:6, 9:12]  = -R_mat * dt_i
    Fd[6:9, 6:9]   = I3 - skew(w) * dt_i
    Fd[6:9, 12:15] = -I3 * dt_i

    G = np.zeros((15, 12))
    G[3:6, 0:3]    = R_mat
    G[6:9, 3:6]    = I3
    G[9:12, 6:9]   = I3
    G[12:15, 9:12] = I3

    P = Fd @ P @ Fd.T + G @ Q_c @ G.T * dt_i
    P = (P + P.T) / 2

    # 数值保护：P 对角线钳制，防止发散
    P_diag_max = np.array([1e4, 1e4, 1e4,    # p:  100m
                           1e2, 1e2, 1e2,    # v:  10 m/s
                           1e0, 1e0, 1e0,    # θ:  1 rad
                           1e-1, 1e-1, 1e-1, # ba: 0.3 m/s²
                           1e-4, 1e-4, 1e-4]) # bg: 0.01 rad/s
    for j in range(15):
        if P[j, j] > P_diag_max[j]:
            scale = np.sqrt(P_diag_max[j] / P[j, j])
            P[j, :] *= scale
            P[:, j] *= scale
    # NaN / Inf 保护
    if not np.all(np.isfinite(P)):
        P = np.eye(15) * 1e-2
        P[12:15, 12:15] = 1e-6

    # ── ZUPT 更新（已关闭；标准 ESKF 全状态形式）──
    if USE_ZUPT and is_stationary[i]:
        y = -v                                   # 观测 v = 0
        S = H_zupt @ P @ H_zupt.T + R_zupt
        if np.linalg.cond(S) < 1e12:
            K = P @ H_zupt.T @ np.linalg.inv(S)
            dx = K @ y
            p = p + dx[0:3]
            v = v + dx[3:6]
            if np.linalg.norm(dx[6:9]) > 1e-12:
                q = qnorm(qmul(q, rotvec_to_quat(dx[6:9])))
            ba = np.clip(ba + dx[9:12], -BA_LIM, BA_LIM)
            bg = np.clip(bg + dx[12:15], -BG_LIM, BG_LIM)
            # Joseph 形式重置
            IKH = I15 - K @ H_zupt
            P = IKH @ P @ IKH.T + K @ R_zupt @ K.T
            P = (P + P.T) / 2
        zupt_count += 1

    # ── 静止段观测更新（正常 ESKF 全状态更新，非 ZUPT）──
    # 观测: acc = R^T g + ba , gyro = bg   （仅真值判定的静止段成立）
    if is_stationary[i]:
        g_body = R_mat.T @ g
        H_bias = np.zeros((6, 15))
        H_bias[0:3, 6:9]   = skew(g_body)   # ∂acc残差/∂θ
        H_bias[0:3, 9:12]  = I3             # ∂acc残差/∂ba
        H_bias[3:6, 12:15] = I3             # ∂gyro残差/∂bg
        y = np.empty(6)
        y[0:3] = acc - g_body - ba
        y[3:6] = gyro - bg
        S = H_bias @ P @ H_bias.T + R_bias
        if np.linalg.cond(S) < 1e12:
            K = P @ H_bias.T @ np.linalg.inv(S)
            dx = K @ y
            # 全状态注入
            p = p + dx[0:3]
            v = v + dx[3:6]
            if np.linalg.norm(dx[6:9]) > 1e-12:
                q = qnorm(qmul(q, rotvec_to_quat(dx[6:9])))
            ba = np.clip(ba + dx[9:12], -BA_LIM, BA_LIM)
            bg = np.clip(bg + dx[12:15], -BG_LIM, BG_LIM)
            # Joseph 形式重置
            IKH = I15 - K @ H_bias
            P = IKH @ P @ IKH.T + K @ R_bias @ K.T
            P = (P + P.T) / 2
        bias_update_count += 1

    # 速度限幅（数值保护）
    v_mag = np.linalg.norm(v)
    V_LIM = 15.0  # m/s
    if v_mag > V_LIM:
        v = v * (V_LIM / v_mag)

    # ── 存储 ──
    pos_ekf[i]  = p
    vel_ekf[i]  = v
    quat_ekf[i] = q
    ba_ekf[i]   = ba
    bg_ekf[i]   = bg

print(f"  ZUPT 次数: {zupt_count}（已关闭）")
print(f"  bias 更新次数: {bias_update_count}/{N} ({bias_update_count/N*100:.1f}%)")

# ══════════════════════════════════════════════════════
# 5. 评估
# ══════════════════════════════════════════════════════
print(f"\n{'='*60}")
print(f"  评估")
print(f"{'='*60}")

# 绘图/评估对齐：把 IMU 纯递推起点平移到真值起点，便于逐维度比较
align_shift = pos_gt[0] - pos_ekf[0]
pos_dr = pos_ekf + align_shift
print(f"  起点对齐偏移: [{align_shift[0]:.3f}, {align_shift[1]:.3f}, {align_shift[2]:.3f}] m")

pos_err = np.linalg.norm(pos_dr - pos_gt, axis=1)
print(f"  位置误差  平均={np.mean(pos_err):.2f}m  中位数={np.median(pos_err):.2f}m  终点={pos_err[-1]:.2f}m")

q_err_deg = np.full(N, np.nan)
for i in range(N):
    qd = qnorm(qmul(np.array([quat_gt[i,0], -quat_gt[i,1], -quat_gt[i,2], -quat_gt[i,3]]), quat_ekf[i]))
    if qd[0] < 0:
        qd = -qd
    q_err_deg[i] = 2 * np.degrees(np.arccos(np.clip(abs(qd[0]), -1, 1)))
print(f"  姿态误差  平均={np.nanmean(q_err_deg):.2f}°  中位数={np.nanmedian(q_err_deg):.2f}°  终点={q_err_deg[-1]:.2f}°")

print(f"\n  初始 ba: [{ba_init[0]:.6f}, {ba_init[1]:.6f}, {ba_init[2]:.6f}] m/s²")
print(f"  初始 bg: [{bg_init[0]:.6f}, {bg_init[1]:.6f}, {bg_init[2]:.6f}] rad/s")
print(f"  最终 ba: [{ba[0]:.6f}, {ba[1]:.6f}, {ba[2]:.6f}] m/s²")
print(f"  最终 bg: [{bg[0]:.6f}, {bg[1]:.6f}, {bg[2]:.6f}] rad/s")

# ══════════════════════════════════════════════════════
# 6. 绘图
# ══════════════════════════════════════════════════════
print("\n绘图 ...")

fig, axes = plt.subplots(3, 3, figsize=(20, 14))
fig.suptitle(f'ESKF IMU Navigation (from GT start, ZUPT off, bias update)\n'
             f'T<{T_END}s | final pos err: {pos_err[-1]:.2f}m | '
             f'att err: {np.nanmean(q_err_deg):.1f}° | '
             f'bias updates: {bias_update_count}',
             fontsize=13)

step = max(1, N // 15000)
AXIS_C = ['r', 'g', 'b']
AXIS_N = ['x', 'y', 'z']

# (0,0) 轨迹对比（起点已对齐）
ax = axes[0, 0]
ax.plot(pos_gt[::step, 0], pos_gt[::step, 1], 'g-', lw=1.5, alpha=0.6, label='GT')
ax.plot(pos_dr[::step, 0], pos_dr[::step, 1], 'r-', lw=1.5, alpha=0.8, label='IMU DR (start-aligned)')
ax.scatter(pos_gt[0, 0], pos_gt[0, 1], c='g', marker='o', s=80, zorder=5, label='GT start')
ax.scatter(pos_gt[-1, 0], pos_gt[-1, 1], c='g', marker='s', s=80, zorder=5, label='GT end')
ax.scatter(pos_dr[-1, 0], pos_dr[-1, 1], c='r', marker='x', s=80, zorder=5, label='DR end')
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.set_title('Trajectory (start-aligned)'); ax.axis('equal')
ax.legend(fontsize=8); ax.grid(alpha=0.3)

# (0,1) 位置误差
ax = axes[0, 1]
ax.plot(t, pos_err, 'b-', lw=1)
ax.set_xlabel('time (s)'); ax.set_ylabel('pos error (m)')
ax.set_title('Position Error'); ax.grid(alpha=0.3)

# (0,2) 姿态误差
ax = axes[0, 2]
ax.plot(t, q_err_deg, 'purple', lw=1)
ax.set_xlabel('time (s)'); ax.set_ylabel('att error (deg)')
ax.set_title('Attitude Error'); ax.grid(alpha=0.3)

# (1,0) 速度
ax = axes[1, 0]
ax.plot(t, vel_ekf[:, 0], 'r-', lw=0.8, label='vx')
ax.plot(t, vel_ekf[:, 1], 'g-', lw=0.8, label='vy')
ax.plot(t, vel_ekf[:, 2], 'b-', lw=0.8, label='vz')
ax.fill_between(t, -5, 5, where=is_stationary, alpha=0.15, color='gray', label='GT stationary')
ax.set_xlabel('time (s)'); ax.set_ylabel('velocity (m/s)')
ax.set_title('Estimated Velocity'); ax.legend(fontsize=7); ax.grid(alpha=0.3)
ax.set_ylim(-5, 5)

# (1,1) X/Y/Z 逐维度对比（同色同维度，实线 GT / 虚线 DR）
ax = axes[1, 1]
for k in range(3):
    ax.plot(t, pos_gt[:, k], '-', color=AXIS_C[k], lw=1, alpha=0.5, label=f'GT {AXIS_N[k]}')
    ax.plot(t, pos_dr[:, k], '--', color=AXIS_C[k], lw=1, alpha=0.9, label=f'DR {AXIS_N[k]}')
ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=is_stationary,
                alpha=0.08, color='gray')
ax.set_xlabel('time (s)'); ax.set_ylabel('position (m)')
ax.set_title('X/Y/Z vs GT (start-aligned)'); ax.legend(fontsize=7); ax.grid(alpha=0.3)

# (1,2) ba 演化（持续更新）
ax = axes[1, 2]
for k in range(3):
    ax.plot(t, ba_ekf[:, k], '-', color=AXIS_C[k], lw=1, label=f'ba_{AXIS_N[k]}')
    ax.axhline(y=ba_init[k], color=AXIS_C[k], ls=':', alpha=0.5)
ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=is_stationary,
                alpha=0.08, color='gray')
ax.set_xlabel('time (s)'); ax.set_ylabel('ba (m/s²)')
ax.set_title('Accelerometer Bias (updating; dotted=init)'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

# (2,0) bg 演化（持续更新）
ax = axes[2, 0]
for k in range(3):
    ax.plot(t, bg_ekf[:, k] * 1000, '-', color=AXIS_C[k], lw=1, label=f'bg_{AXIS_N[k]}')
    ax.axhline(y=bg_init[k] * 1000, color=AXIS_C[k], ls=':', alpha=0.5)
ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=is_stationary,
                alpha=0.08, color='gray')
ax.set_xlabel('time (s)'); ax.set_ylabel('bg (mrad/s)')
ax.set_title('Gyroscope Bias (updating; dotted=init)'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

# (2,1) 真值静止检测
ax = axes[2, 1]
ax.plot(t, disp_1s * 1000, 'b-', lw=0.8, alpha=0.7, label='GT disp 1s')
ax.axhline(y=DISP_THRESH * 1000, color='r', ls='--', alpha=0.5,
           label=f'disp thresh={DISP_THRESH*1000:.0f}mm')
ax.fill_between(t, 0, ax.get_ylim()[1], where=is_stationary, alpha=0.3, color='green',
                label='GT stationary')
ax.axvspan(t[stat_start], stat_end_t, alpha=0.2, color='orange', label='bias init seg')
ax.set_xlabel('time (s)'); ax.set_ylabel('GT 1s displacement (mm)')
ax.set_title('GT Stationary Detection'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

# (2,2) 逐维度位置误差
ax = axes[2, 2]
for k in range(3):
    ax.plot(t, pos_dr[:, k] - pos_gt[:, k], '-', color=AXIS_C[k], lw=1,
            label=f'{AXIS_N[k]} err')
ax.axhline(y=0, color='k', lw=0.5, alpha=0.5)
ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=is_stationary,
                alpha=0.08, color='gray')
ax.set_xlabel('time (s)'); ax.set_ylabel('error (m)')
ax.set_title('Position Error per Axis (start-aligned)'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig(PLOT_OUT, dpi=150)
print(f"  ✅ 图: {PLOT_OUT}")

# ── 保存结果 ──
np.savez(SAVE_NPZ,
         ts=ts, pos_ekf=pos_ekf, pos_dr=pos_dr, vel_ekf=vel_ekf, quat_ekf=quat_ekf,
         ba_ekf=ba_ekf, bg_ekf=bg_ekf,
         pos_gt=pos_gt, pos_error=pos_err, att_error=q_err_deg,
         is_stationary=is_stationary, align_shift=align_shift,
         ba_init=ba_init, bg_init=bg_init)
print(f"  ✅ 结果: {SAVE_NPZ}")

print(f"\n{'='*60}")
print(f"  ESKF 从真值起点递推（ZUPT 关闭，ba/bg 持续更新）")
print(f"  时长: {t[-1]:.0f}s | 终点位置差: {pos_err[-1]:.2f}m")
print(f"  平均姿态漂移: {np.nanmean(q_err_deg):.2f}°")
print(f"  bias 更新: {bias_update_count} 次 | ba 终值: [{ba[0]:.4f},{ba[1]:.4f},{ba[2]:.4f}]")
print(f"{'='*60}")
