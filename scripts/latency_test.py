#!/usr/bin/env python3
"""
时延分析脚本 v2
计算 /ouster/points 传感器时间 → /tf 发布时间的端到端时延

关键：使用消息 header.stamp 而非 bag 录制时间，因为 sim time 下录制时间会重合。

时延 = /tf 中 transform.header.stamp - /ouster/points 的 header.stamp
       包含 slam_odometry_v2 配准 + localization_v2 融合的全部处理耗时
"""
import rosbag
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys
import os

BAG_PATH = "/home/hhy/2004ros/data/test_mimo.bag"

if not os.path.exists(BAG_PATH):
    print(f"[ERROR] bag 不存在: {BAG_PATH}")
    sys.exit(1)

print(f"正在分析: {BAG_PATH}")
bag = rosbag.Bag(BAG_PATH)

# ── 1. 收集输入点云时间戳 ──
#    使用 msg.header.stamp（传感器时间，合并 bag 时已对齐到 sim time 基准）
input_times = []
for topic, msg, t in bag.read_messages(topics=['/ouster/points']):
    input_times.append(msg.header.stamp.to_sec())

# ── 2. 收集 TF 发布时间戳 ──
#    localization_v2::publishTf() 中 transform.header.stamp = ros::Time::now()
#    这就是 TF 的发布时间（在 sim time 基准上）
output_times = []
for topic, msg, t in bag.read_messages(topics=['/tf']):
    # tf2_msgs/TFMessage 包含多个 TransformStamped
    # 取第一个 transform 的 header.stamp 作为发布时间
    if msg.transforms:
        output_times.append(msg.transforms[0].header.stamp.to_sec())

bag.close()

print(f"  /ouster/points:  {len(input_times)} 帧")
print(f"  /tf:             {len(output_times)} 帧")
print(f"  输入时间范围:    {input_times[0]:.3f} ~ {input_times[-1]:.3f}")

# ── 3. 匹配并计算时延 ──
latencies = []
matched_times = []
skipped = 0

j = 0
for i, t_in in enumerate(input_times):
    while j < len(output_times) and output_times[j] < t_in:
        j += 1
    if j < len(output_times):
        delay = (output_times[j] - t_in) * 1000  # ms
        latencies.append(delay)
        matched_times.append(t_in)
    else:
        skipped += 1

# ── 4. 打印统计 ──
print(f"\n{'='*60}")
print(f"  端到端时延统计（点云传感器时间 → TF 发布时间）")
print(f"{'='*60}")
if latencies:
    print(f"  输入总帧数:    {len(input_times)}")
    print(f"  成功匹配:      {len(latencies)}")
    print(f"  未匹配:        {skipped}")
    print(f"  ─────────────────────────────")
    print(f"  平均时延:      {np.mean(latencies):8.1f} ms")
    print(f"  中位数:        {np.median(latencies):8.1f} ms")
    print(f"  P99:           {np.percentile(latencies, 99):8.1f} ms")
    print(f"  P95:           {np.percentile(latencies, 95):8.1f} ms")
    print(f"  P90:           {np.percentile(latencies, 90):8.1f} ms")
    print(f"  最大时延:      {np.max(latencies):8.1f} ms")
    print(f"  最小时延:      {np.min(latencies):8.1f} ms")
    print(f"  标准差:        {np.std(latencies):8.1f} ms")

    # ── 5. 画图 ──
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle('End-to-End Latency (ouster/points header -> TF publish)', fontsize=13)
    
    # 5a. 时延随时间变化
    ax1 = axes[0, 0]
    ax1.plot(matched_times, latencies, 'b.', alpha=0.6, markersize=2)
    ax1.axhline(np.mean(latencies), color='r', linestyle='--',
                label=f'mean={np.mean(latencies):.1f}ms')
    ax1.axhline(np.median(latencies), color='g', linestyle='--',
                label=f'median={np.median(latencies):.1f}ms')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Latency (ms)')
    ax1.set_title('Latency over Time')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    # 5b. 直方图
    ax2 = axes[0, 1]
    ax2.hist(latencies, bins=60, alpha=0.7, edgecolor='black', linewidth=0.5)
    ax2.axvline(np.mean(latencies), color='r', linestyle='--',
                label=f'mean={np.mean(latencies):.1f}ms')
    ax2.axvline(np.median(latencies), color='g', linestyle='--',
                label=f'median={np.median(latencies):.1f}ms')
    ax2.set_xlabel('Latency (ms)')
    ax2.set_ylabel('Count')
    ax2.set_title('Latency Distribution')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # 5c. 箱线图
    ax3 = axes[1, 0]
    bp = ax3.boxplot(latencies, vert=True, patch_artist=True)
    bp['boxes'][0].set_facecolor('lightblue')
    ax3.set_ylabel('Latency (ms)')
    ax3.set_title('Box Plot')
    ax3.grid(True, alpha=0.3)
    ax3.set_xticks([])
    
    # 5d. CDF
    ax4 = axes[1, 1]
    sorted_delays = np.sort(latencies)
    cdf = np.arange(1, len(sorted_delays) + 1) / len(sorted_delays)
    ax4.plot(sorted_delays, cdf, 'b-', linewidth=1.5)
    ax4.axhline(0.5, color='gray', linestyle=':', alpha=0.5)
    ax4.axhline(0.9, color='gray', linestyle=':', alpha=0.5)
    ax4.axhline(0.99, color='gray', linestyle=':', alpha=0.5)
    ax4.set_xlabel('Latency (ms)')
    ax4.set_ylabel('Cumulative Probability')
    ax4.set_title('CDF')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    output_path = '/home/hhy/2004ros/data/latency_analysis.png'
    plt.savefig(output_path, dpi=150)
    print(f"\n  Chart saved: {output_path}")
else:
    print("  [WARN] No input-output pairs matched")
    print(f"  Input range:  {input_times[0]:.3f} ~ {input_times[-1]:.3f}")
    if output_times:
        print(f"  Output range: {output_times[0]:.3f} ~ {output_times[-1]:.3f}")