#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘制 GNSS(Gpchc) 轨迹，检查其本身是否准确、是否有跳跃。

用途：
  - 单看 GNSS：轨迹形态、单步位移跳变、speed 合理性、status 分布；
  - 叠加对比：传入 slam_mapping 保存的 trajectory.csv / keyframes.csv，
    用时间匹配 + 2D Umeyama 对齐后叠加，定量看 GNSS 与建图轨迹的偏差。

用法：
  source /opt/ros/noetic/setup.bash
  source <DeepLumin>/devel/setup.bash
  python3 plot_gnss_trajectory.py --bag /path/to/xx.bag \
      --out gnss_trajectory.png \
      [--traj trajectory.csv] [--jump-thresh 10.0] [--status 33 42]
"""

import argparse
import bisect
import collections
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------- WGS84 坐标换算（与 gnss_optimizer.cpp 一致） ----------------

_EARTH_RADIUS = 6378137.0
_FLATTENING = 1.0 / 298.257223563
_E2 = _FLATTENING * (2.0 - _FLATTENING)
_DEG2RAD = math.pi / 180.0


def lla_to_ecef(lat_rad, lon_rad, alt):
    n = _EARTH_RADIUS / math.sqrt(1.0 - _E2 * math.sin(lat_rad) ** 2)
    return (
        (n + alt) * math.cos(lat_rad) * math.cos(lon_rad),
        (n + alt) * math.cos(lat_rad) * math.sin(lon_rad),
        (n * (1.0 - _E2) + alt) * math.sin(lat_rad),
    )


def ecef_to_enu(ecef, ref_lat_rad, ref_lon_rad, ref_ecef):
    slat, clat = math.sin(ref_lat_rad), math.cos(ref_lat_rad)
    slon, clon = math.sin(ref_lon_rad), math.cos(ref_lon_rad)
    d = (ecef[0] - ref_ecef[0], ecef[1] - ref_ecef[1], ecef[2] - ref_ecef[2])
    return (
        -slon * d[0] + clon * d[1],
        -clon * slat * d[0] - slon * slat * d[1] + clat * d[2],
        clon * clat * d[0] + slon * clat * d[1] + slat * d[2],
    )


def umeyama2d(src, dst):
    """2D Umeyama（旋转+平移），把 src 对齐到 dst，返回 (R, t)。"""
    n = len(src)
    ms = (sum(p[0] for p in src) / n, sum(p[1] for p in src) / n)
    md = (sum(p[0] for p in dst) / n, sum(p[1] for p in dst) / n)
    sxx = sxy = syx = syy = 0.0
    for p, q in zip(src, dst):
        pe = (p[0] - ms[0], p[1] - ms[1])
        qe = (q[0] - md[0], q[1] - md[1])
        sxx += pe[0] * qe[0]
        sxy += pe[0] * qe[1]
        syx += pe[1] * qe[0]
        syy += pe[1] * qe[1]
    theta = math.atan2(sxy - syx, sxx + syy)
    c, s = math.cos(theta), math.sin(theta)
    r = ((c, -s), (s, c))
    tx = md[0] - (r[0][0] * ms[0] + r[0][1] * ms[1])
    ty = md[1] - (r[1][0] * ms[0] + r[1][1] * ms[1])
    return r, (tx, ty)


def transform_xy(pts, r, t):
    return [(r[0][0] * p[0] + r[0][1] * p[1] + t[0],
             r[1][0] * p[0] + r[1][1] * p[1] + t[1]) for p in pts]


def rotz_deg(yaw_deg):
    th = math.radians(yaw_deg)
    c, s = math.cos(th), math.sin(th)
    return ((c, -s), (s, c))


def parse_align_file(path):
    """解析 gnss_alignment.txt（map = R*yaw * enu + t，ENU 以文件内 origin 为原点）。"""
    vals = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, value = line.partition(":")
            vals[key.strip()] = float(value.strip())
    return vals


def load_trajectory(path):
    """加载 slam_mapping 导出的轨迹，返回 (ts[], x[], y[], z[])。"""
    ts, xs, ys, zs = [], [], [], []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if "," in line:
                cols = [c.strip() for c in line.split(",")]
                if len(cols) < 5:
                    continue
                ts.append(float(cols[1]))
                xs.append(float(cols[2]))
                ys.append(float(cols[3]))
                zs.append(float(cols[4]))
            else:
                cols = line.split()
                if len(cols) < 4:
                    continue
                ts.append(float(cols[0]))
                xs.append(float(cols[1]))
                ys.append(float(cols[2]))
                zs.append(float(cols[3]))
    return ts, xs, ys, zs


def main():
    parser = argparse.ArgumentParser(description="GNSS 轨迹质量分析")
    parser.add_argument("--bag", required=True, help="rosbag 路径")
    parser.add_argument("--gnss-topic", default="/gnss_chc_data")
    parser.add_argument("--out", default="gnss_trajectory.png")
    parser.add_argument("--traj", default=None,
                        help="slam_mapping trajectory.csv/keyframes.csv，叠加对比")
    parser.add_argument("--align", default=None,
                        help="slam_mapping 导出的 gnss_alignment.txt；"
                             "用系统估计的 ENU->map 变换叠加（替代 Umeyama 拟合）")
    parser.add_argument("--jump-thresh", type=float, default=10.0,
                        help="单步位移视为跳变的阈值（米）")
    parser.add_argument("--status", type=int, nargs="*", default=None,
                        help="只统计指定 status（默认全部）")
    args = parser.parse_args()

    try:
        import rosbag
    except ImportError:
        sys.exit("缺少 rosbag 模块，请先 source /opt/ros/noetic/setup.bash")
    try:
        from deeplumin_msgs.msg import Gpchc  # noqa: F401  确保消息类型可反序列化
    except ImportError:
        sys.exit("缺少 deeplumin_msgs，请先 source <DeepLumin>/devel/setup.bash")

    # ---- 读取 GNSS ----
    rows = []  # (record_t, header_t, status, lat, lon, alt)
    bag = rosbag.Bag(args.bag, "r")
    for topic, m, t in bag.read_messages(topics=[args.gnss_topic]):
        rows.append((t.to_sec(), m.header.stamp.to_sec(),
                     m.status, m.latitude, m.longitude, m.altitude))
    bag.close()
    if not rows:
        sys.exit("topic %s 无数据" % args.gnss_topic)

    if args.status is not None:
        allowed = set(args.status)
        rows = [r for r in rows if r[2] in allowed]
        if not rows:
            sys.exit("过滤后无数据")

    # ---- ENU 转换 ----
    # 用 --align 时，ENU 原点必须与后端一致（后端保存的 origin），否则变换叠加会带常量偏移
    align_vals = parse_align_file(args.align) if args.align else {}
    if align_vals:
        ref_lat = align_vals["origin_lat"] * _DEG2RAD
        ref_lon = align_vals["origin_lon"] * _DEG2RAD
        ref_ecef = lla_to_ecef(ref_lat, ref_lon, align_vals["origin_alt"])
    else:
        origin = next((r for r in rows if abs(r[3]) > 1e-9 and abs(r[4]) > 1e-9), None)
        if origin is None:
            sys.exit("无有效经纬度")
        ref_lat = origin[3] * _DEG2RAD
        ref_lon = origin[4] * _DEG2RAD
        ref_ecef = lla_to_ecef(ref_lat, ref_lon, origin[5])

    times, header_times, statuses, enu = [], [], [], []
    for rt, ht, st, la, lo, al in rows:
        e = ecef_to_enu(lla_to_ecef(la * _DEG2RAD, lo * _DEG2RAD, al),
                        ref_lat, ref_lon, ref_ecef)
        times.append(rt)
        header_times.append(ht)
        statuses.append(st)
        enu.append(e)

    # ---- 逐点统计 ----
    jumps, speeds = [], []
    outlier_idx = []
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt <= 0 or dt > 5.0:
            continue
        d = math.hypot(enu[i][0] - enu[i - 1][0], enu[i][1] - enu[i - 1][1])
        jumps.append(d)
        speeds.append(d / dt)
        if d > args.jump_thresh:
            outlier_idx.append(i)

    outliers = [(times[i], statuses[i], jumps[i - 1], i)
                for i in outlier_idx]
    jumps_s = sorted(jumps)
    speeds_s = sorted(speeds)
    pct = lambda a, p: a[int(p * len(a))] if a else float("nan")

    print("== GNSS 轨迹统计 ==")
    print("观测数: %d  时长: %.1fs  平均频率: %.2fHz" %
          (len(times), times[-1] - times[0], len(times) / (times[-1] - times[0])))
    print("status 分布:", dict(collections.Counter(statuses)))
    print("单步位移(记录时间): p50=%.3fm p95=%.3fm p99=%.3fm max=%.1fm" %
          (pct(jumps_s, 0.5), pct(jumps_s, 0.95), pct(jumps_s, 0.99), jumps_s[-1]))
    print("speed: p50=%.2f p95=%.2f max=%.1f m/s" %
          (pct(speeds_s, 0.5), pct(speeds_s, 0.95), speeds_s[-1]))
    print("跳变(>%.1fm)次数: %d" % (args.jump_thresh, len(outliers)))
    for rt, st, j, _ in outliers[:20]:
        print("  t=%.1f status=%d jump=%.1fm" % (rt, st, j))
    if len(outliers) > 20:
        print("  ... 共 %d 个" % len(outliers))

    # ---- 绘图 ----
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    ax1, ax2 = axes
    xs = [e[0] for e in enu]
    ys = [e[1] for e in enu]

    # --align 模式下，主图直接绘制 map 系下的 GNSS（避免与建图轨迹错位）
    if align_vals:
        r_a = rotz_deg(align_vals["yaw_deg"])
        t_a = (align_vals["tx"], align_vals["ty"])
        xy_map = transform_xy(list(zip(xs, ys)), r_a, t_a)
        xs_plot = [p[0] for p in xy_map]
        ys_plot = [p[1] for p in xy_map]
        ax1_title = "GNSS trajectory (map frame, system transform)"
    else:
        xs_plot, ys_plot = xs, ys
        ax1_title = "GNSS trajectory (ENU, origin=first fix)"

    status_set = sorted(set(statuses))
    colors = plt.cm.tab10.colors
    for k, st in enumerate(status_set):
        idx = [i for i, s in enumerate(statuses) if s == st]
        ax1.plot([xs_plot[i] for i in idx], [ys_plot[i] for i in idx], ".",
                 color=colors[k % 10], markersize=1.5, label="status %d" % st)
    for _, _, _, i in outliers:
        ax1.plot(xs_plot[i], ys_plot[i], "x", color="red", markersize=6, zorder=5)
    ax1.set_aspect("equal")
    ax1.set_title(ax1_title)
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.legend(loc="best", markerscale=4)

    ax2.plot(times[1:], jumps, ".", markersize=1, label="step displacement")
    ax2.axhline(args.jump_thresh, color="red", linestyle="--",
                label="jump thresh %.1fm" % args.jump_thresh)
    ax2.set_yscale("log")
    ax2.set_title("Step displacement vs time")
    ax2.set_xlabel("record time (s)")
    ax2.set_ylabel("m")
    ax2.legend()

    # ---- 与建图轨迹叠加 ----
    if args.traj:
        ts, tx, ty, tz = load_trajectory(args.traj)
        if len(ts) < 10:
            print("trajectory 数据不足，跳过叠加")
        else:
            # 建图轨迹时间戳与 odom（雷达 header）同域，因此用 GNSS header 时间匹配
            g_ts = sorted(header_times)
            idx_by_time = {t: i for i, t in enumerate(header_times)}
            pairs = []
            for i, t0 in enumerate(ts):
                j = bisect.bisect_left(g_ts, t0)
                cand = []
                if j > 0:
                    cand.append(j - 1)
                if j < len(g_ts):
                    cand.append(j)
                for k in cand:
                    if abs(g_ts[k] - t0) <= 1.0:
                        gi = idx_by_time[g_ts[k]]
                        pairs.append(((tx[i], ty[i]), (enu[gi][0], enu[gi][1])))
                        break
            if len(pairs) >= 10:
                if align_vals:
                    # 用系统估计的 ENU->map 变换把 GNSS 转进 map 系（不重新拟合）
                    r = rotz_deg(align_vals["yaw_deg"])
                    t = (align_vals["tx"], align_vals["ty"])
                    gnss_in_map = transform_xy(list(zip(xs, ys)), r, t)
                    ax1.plot([p[0] for p in gnss_in_map], [p[1] for p in gnss_in_map],
                             "g-", linewidth=1.0, alpha=0.8, label="GNSS (system transform)")
                    ax1.plot(tx, ty, "k-", linewidth=1.0, alpha=0.7, label="mapping traj")
                    # 起点/终点标记，直接观察两端是否对齐
                    ax1.plot(gnss_in_map[0][0], gnss_in_map[0][1], "gs", markersize=8, label="gnss start")
                    ax1.plot(gnss_in_map[-1][0], gnss_in_map[-1][1], "g^", markersize=8, label="gnss end")
                    ax1.plot(tx[0], ty[0], "ks", markersize=8, label="map start")
                    ax1.plot(tx[-1], ty[-1], "k^", markersize=8, label="map end")
                    # 匹配点处：建图轨迹 vs 系统变换后的 GNSS
                    gnss_matched = [transform_xy([p[1]], r, t)[0] for p in pairs]
                    rmse = math.sqrt(sum(
                        (p[0][0] - g[0]) ** 2 + (p[0][1] - g[1]) ** 2
                        for p, g in zip(pairs, gnss_matched)) / len(pairs))
                    print("系统变换叠加: yaw=%.2fdeg t=(%.2f, %.2f) 匹配点=%d RMSE=%.2fm"
                          % (align_vals["yaw_deg"], align_vals["tx"], align_vals["ty"],
                             len(pairs), rmse))
                    ax1.legend(loc="best", markerscale=4)
                else:
                    r, t = umeyama2d([p[0] for p in pairs], [p[1] for p in pairs])
                    aligned = transform_xy(list(zip(tx, ty)), r, t)
                    rmse = math.sqrt(sum(
                        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
                        for a, b in zip(aligned, [p[1] for p in pairs])) / len(pairs))
                    ax1.plot([p[0] for p in aligned], [p[1] for p in aligned],
                             "k-", linewidth=1.0, alpha=0.7, label="mapping traj (aligned)")
                    print("叠加对齐(2D Umeyama, 匹配点 %d): RMSE=%.2fm" % (len(pairs), rmse))
                    ax1.legend(loc="best", markerscale=4)
            else:
                print("时间匹配点不足(%d)，跳过叠加" % len(pairs))

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print("图片已保存: %s" % args.out)


if __name__ == "__main__":
    main()
