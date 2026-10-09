#!/usr/bin/env python3
"""
分析 imu_pose_200hz.npz 数据集：
  1. 交互式 3D 轨迹（plotly，可鼠标拖动旋转）
  2. IMU 数据平滑度分析 + 频谱分析
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
import os
import sys

NPZ_PATH = "/home/hhy/2004ros/data/imu_pose_200hz.npz"
HTML_OUT = "/home/hhy/2004ros/data/trajectory_3d_interactive.html"
PLOT_OUT = "/home/hhy/2004ros/data/imu_smoothness_analysis.png"

if not os.path.exists(NPZ_PATH):
    print(f"[ERROR] {NPZ_PATH} 不存在，请先运行 extract_imu_pose.py")
    sys.exit(1)

data = np.load(NPZ_PATH)
ts       = data["retargetted_ts"]
imu      = data["retargetted_imu"]   # (N,6) [ax, ay, az, gx, gy, gz]
pos      = data["retargetted_pos"]   # (N,3)
quat     = data["retargetted_quat"]  # (N,4) [qw, qx, qy, qz]

FS = 200.0  # Hz
dt = 1.0 / FS
N  = len(ts)
t  = ts - ts[0]  # 相对时间（秒）

print(f"数据集大小: {N} 帧")
print(f"时长: {t[-1]:.1f}s")

# ════════════════════════════════════════════
# 1. 交互式 3D 轨迹（plotly）
# ════════════════════════════════════════════
print("\n[1/2] 生成交互式 3D 轨迹 ...")
import plotly.graph_objects as go
import plotly.io as pio

# 降采样到 1/10 以加快渲染（200Hz → 20Hz 仍然密集）
step = max(1, N // 50000)  # 最多 5 万点
idx = slice(0, N, step)

# 颜色映射：时间
colors = t[idx]

fig = go.Figure()

# 轨迹线
fig.add_trace(go.Scatter3d(
    x=pos[idx, 0], y=pos[idx, 1], z=pos[idx, 2],
    mode='lines',
    line=dict(color='blue', width=1),
    name='trajectory',
    showlegend=True
))

# 散点（颜色 = 时间）
fig.add_trace(go.Scatter3d(
    x=pos[idx, 0], y=pos[idx, 1], z=pos[idx, 2],
    mode='markers',
    marker=dict(
        size=1.5,
        color=t[idx],
        colorscale='Viridis',
        colorbar=dict(title="time (s)"),
        showscale=True
    ),
    name='samples',
    showlegend=False
))

# 起点/终点标记
fig.add_trace(go.Scatter3d(
    x=[pos[0, 0]], y=[pos[0, 1]], z=[pos[0, 2]],
    mode='markers+text',
    marker=dict(size=8, color='green'),
    text=['Start'],
    textposition='top center',
    name='start'
))

fig.add_trace(go.Scatter3d(
    x=[pos[-1, 0]], y=[pos[-1, 1]], z=[pos[-1, 2]],
    mode='markers+text',
    marker=dict(size=8, color='red'),
    text=['End'],
    textposition='top center',
    name='end'
))

fig.update_layout(
    title='3D Trajectory (mouse drag to rotate)',
    scene=dict(
        xaxis_title='X (m)',
        yaxis_title='Y (m)',
        zaxis_title='Z (m)',
        aspectmode='data'
    ),
    width=1200,
    height=900,
    hovermode='closest'
)

pio.write_html(fig, HTML_OUT, auto_open=False)
print(f"  ✅ 已保存: {HTML_OUT}")

# ════════════════════════════════════════════
# 2. IMU 平滑度分析
# ════════════════════════════════════════════
print("\n[2/2] 分析 IMU 平滑度 ...")

# 降采样到 1/10 用于绘图（否则 21 万点太密）
plot_step = max(1, N // 50000)
t_plot = t[::plot_step]
imu_plot = imu[::plot_step]

acc_names = ['acc_x', 'acc_y', 'acc_z']
gyro_names = ['gyro_x', 'gyro_y', 'gyro_z']
acc_unit = 'm/s²'
gyro_unit = 'rad/s'

# ── 2a. 时域统计 ──
print("\n  ── 时域统计 ──")
for i, name in enumerate(acc_names + gyro_names):
    col = imu[:, i]
    mean = np.mean(col)
    std  = np.std(col)
    rms  = np.sqrt(np.mean(col**2))
    p2p  = np.max(col) - np.min(col)
    unit = acc_unit if i < 3 else gyro_unit
    print(f"    {name:8s}: mean={mean:8.4f}  std={std:8.4f}  "
          f"rms={rms:8.4f}  peak2peak={p2p:8.4f}  [{unit}]")

# ── 2b. 差分分析（衡量平滑度：帧间跳变量） ──
print("\n  ── 帧间差分统计（衡量平滑度） ──")
for i, name in enumerate(acc_names + gyro_names):
    col = imu[:, i]
    diff = np.diff(col) / dt  # 导数（变化率）
    diff_std = np.std(diff)
    diff_max = np.max(np.abs(diff))
    unit = f"({acc_unit}/s)" if i < 3 else f"({gyro_unit}/s)"
    print(f"    {name:8s}: diff_std={diff_std:8.4f}  max|diff|={diff_max:8.4f}  [{unit}]")

# ── 2c. 滑动窗口方差（高频抖动检测） ──
window = int(FS * 0.05)  # 50ms 滑动窗口
print(f"\n  ── 高频抖动检测（{window*dt*1000:.0f}ms 滑动窗口） ──")
for i, name in enumerate(acc_names + gyro_names):
    col = imu[:, i]
    # 滑动方差
    means = np.convolve(col, np.ones(window)/window, mode='valid')
    vars_ = np.convolve((col - np.pad(means, (window//2, window-1-window//2), mode='edge'))**2,
                        np.ones(window)/window, mode='valid')
    # 取前几个百分位看抖动情况
    p50 = np.percentile(vars_, 50)
    p90 = np.percentile(vars_, 90)
    p99 = np.percentile(vars_, 99)
    unit = f"({acc_unit}²)" if i < 3 else f"({gyro_unit}²)"
    print(f"    {name:8s}: sliding_var_p50={p50:8.6f}  p90={p90:8.6f}  "
          f"p99={p99:8.6f}  [{unit}]")

# ── 2d. 频谱分析 ──
print("\n  ── 频谱分析 ──")
fig2, axes = plt.subplots(3, 2, figsize=(16, 12))
fig2.suptitle('IMU Smoothness & Spectrum Analysis', fontsize=14)

# 取中间 1/3 段做频谱（稳态段）
spec_start = N // 3
spec_end   = 2 * N // 3
spec_len   = spec_end - spec_start

freq = np.fft.rfftfreq(spec_len, d=dt)

for i in range(6):
    ax = axes[i // 2, i % 2]
    name = acc_names[i] if i < 3 else gyro_names[i - 3]
    unit = acc_unit if i < 3 else gyro_unit
    col = imu[spec_start:spec_end, i]
    
    # 去均值
    col_dm = col - np.mean(col)
    
    # 频谱
    fft_ = np.fft.rfft(col_dm * signal.windows.hann(spec_len), axis=0)
    psd = np.abs(fft_) ** 2 / (FS * spec_len)
    
    # 只显示 0.1~50Hz
    mask = (freq >= 0.1) & (freq <= 50)
    ax.loglog(freq[mask], psd[mask], linewidth=0.6)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel(f'PSD ({unit}²/Hz)')
    ax.set_title(f'{name} Spectrum')
    ax.grid(True, alpha=0.3, which='both')
    
    # 标注主要频率成分
    peak_mask = (freq >= 0.5) & (freq <= 50)
    if np.any(peak_mask):
        peak_idx = np.argmax(psd[peak_mask])
        peak_freq = freq[peak_mask][peak_idx]
        peak_val = psd[peak_mask][peak_idx]
        ax.axvline(peak_freq, color='r', linestyle='--', alpha=0.5, linewidth=0.8)
        ax.text(peak_freq, peak_val, f'  {peak_freq:.1f}Hz', fontsize=7, rotation=90)

plt.tight_layout()
plt.savefig(PLOT_OUT, dpi=150)
print(f"  ✅ 频谱图已保存: {PLOT_OUT}")

# ── 2e. 补充：加速度模值统计（总加速度变化） ──
acc_norm = np.linalg.norm(imu[:, :3], axis=1)
gyro_norm = np.linalg.norm(imu[:, 3:6], axis=1)

print("\n  ── 模值统计 ──")
print(f"    |acc|: mean={np.mean(acc_norm):.4f}  std={np.std(acc_norm):.4f}  "
      f"min={np.min(acc_norm):.4f}  max={np.max(acc_norm):.4f} [m/s²]")
print(f"    |gyro|: mean={np.mean(gyro_norm):.4f}  std={np.std(gyro_norm):.4f}  "
      f"min={np.min(gyro_norm):.4f}  max={np.max(gyro_norm):.4f} [rad/s]")

# ── 总结结论 ──
print(f"\n{'='*65}")
print(f"  分析结论")
print(f"{'='*65}")
print(f"  1. 数据量: {N} 帧 @ {FS}Hz, 时长 {t[-1]:.1f}s")
print(f"  2. 交互式 3D 轨迹: {HTML_OUT}")
print(f"  3. IMU 频谱图:     {PLOT_OUT}")
print(f"")
print(f"  IMU 平滑度判断:")
acc_std_mean = np.mean([np.std(imu[:, i]) for i in range(3)])
gyro_std_mean = np.mean([np.std(imu[:, i]) for i in range(3, 6)])
print(f"    - acc 三轴平均 std = {acc_std_mean:.4f} m/s²")
print(f"    - gyro 三轴平均 std = {gyro_std_mean:.4f} rad/s")
print(f"")

# 简单判断：Ouster IMU 的典型噪声水平
# acc noise ≈ 0.01~0.05 m/s², gyro noise ≈ 0.001~0.01 rad/s
if acc_std_mean < 0.1:
    print(f"    ✅ acc 噪声水平正常（< 0.1 m/s²）")
elif acc_std_mean < 0.5:
    print(f"    ⚠️  acc 噪声偏大（0.1~0.5 m/s²），存在一定抖动")
else:
    print(f"    ❌ acc 噪声过大（> 0.5 m/s²），数据质量较差")

if gyro_std_mean < 0.01:
    print(f"    ✅ gyro 噪声水平正常（< 0.01 rad/s）")
elif gyro_std_mean < 0.05:
    print(f"    ⚠️  gyro 噪声偏大（0.01~0.05 rad/s），存在一定抖动")
else:
    print(f"    ❌ gyro 噪声过大（> 0.05 rad/s），数据质量较差")

print(f"")

# 检测是否存在周期性抖动
freq = np.fft.rfftfreq(spec_len, d=dt)
psd_all = np.zeros_like(freq)
for i in range(6):
    col = imu[spec_start:spec_end, i] - np.mean(imu[spec_start:spec_end, i])
    fft_ = np.fft.rfft(col * signal.windows.hann(spec_len), axis=0)
    psd_all += np.abs(fft_) ** 2 / (FS * spec_len)

# 找最强峰值（>0.5Hz）
peak_mask = (freq >= 0.5) & (freq <= 50)
if np.any(peak_mask):
    peak_idx = np.argmax(psd_all[peak_mask])
    peak_freq = freq[peak_mask][peak_idx]
    peak_power = psd_all[peak_mask][peak_idx]
    mean_power = np.mean(psd_all[peak_mask])
    if peak_power / mean_power > 5:
        print(f"    ⚠️  检测到周期性成分: {peak_freq:.1f}Hz "
              f"（信噪比 {peak_power/mean_power:.1f}x）")
        if peak_freq < 2:
            print(f"       → 可能是车辆振动/颠簸")
        elif peak_freq < 10:
            print(f"       → 可能是发动机/电机振动")
        else:
            print(f"       → 可能是传感器高频噪声")
    else:
        print(f"    ✅ 未见明显周期性抖动")

print(f"{'='*65}")