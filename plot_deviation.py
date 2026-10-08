#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_deviation.py -- 正常数据集 vs 干扰数据集：位移偏差折线 + 建图点云对比（2x2 四个子图）

四个子图：
  (a) 位移偏差折线：固定像素贴图  vs 正常数据集
  (b) 位移偏差折线：世界平面投影贴图 vs 正常数据集
  (c) 建图点云对比：固定像素贴图  vs 正常数据集
  (d) 建图点云对比：世界平面投影贴图 vs 正常数据集

折线子图 (a)(b)：
  横轴 = 帧号，纵轴 = 相对正常数据集（clean）的位移偏差 (m)
  黑粗线 = 正常数据集基线（偏差恒为 0），彩线 = 干扰数据集逐帧偏差

建图子图 (c)(d)（X-Z 俯视，全部地图点 + 空白角密集区放大子图）：
  蓝点 = 正常数据集构建的地图点
  红点 = 干扰数据集构建的地图点
  主图不剔除任何点；空白角子图把半径 <= k x 中位半径 的密集区放大

⚠ 本数据集没有 ground truth：以 clean 组的估计位姿为基准，
  偏差 = 干扰组估计位姿 − clean 组估计位姿（按时间戳同帧对齐）。
  读数是“相对正常估计偏移了多少”，不是绝对定位误差。

输入
  <runs>/<组>/CameraTrajectory.txt   逐帧位姿（TUM）
  <runs>/<组>/map_points.csv         最终地图点
  <ds>/comparison_pairs.csv          帧号 <-> 时间戳

例
  python3 plot_deviation.py \
    --ds ~/dataset/slam_stereo_pairs_0_99_20260929 \
    --runs ~/ORB_SLAM3/runs/4seasons \
    --out ~/ORB_SLAM3/outputs/fig4_4seasons/figures/fig4_4seasons_deviation
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

CJK_CANDIDATES = [
    "Microsoft YaHei", "Microsoft YaHei UI", "SimHei", "Noto Sans CJK SC",
    "Source Han Sans SC", "WenQuanYi Zen Hei", "PingFang SC", "Heiti SC",
    "Arial Unicode MS",
]

CLEAN_C = "#111111"          # 基线
CLEAN_PT = "#1F4FD8"         # 正常数据集地图点（蓝）
ATTACK_PT = ["#D62728", "#FF7F0E", "#8C564B"]   # 异常数据集地图点（红 / 橙红）
ATTACK_LN = ["#D62728", "#1F77B4", "#2CA02C"]   # 偏差曲线颜色
GROUP_CN = {"clean": "正常数据集", "fixed_pixel": "固定像素贴图", "world_plane": "世界平面投影贴图"}


def setup_fonts():
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in CJK_CANDIDATES:
        if name in have:
            plt.rcParams["font.sans-serif"] = [name] + plt.rcParams["font.sans-serif"]
            plt.rcParams["axes.unicode_minus"] = False
            print("[i] 中文字体: %s" % name)
            return True
    print("[!] 没找到中文字体，中文可能显示成方块；可加 --no-cjk 或装 fonts-noto-cjk")
    return False


# -------------------------------------------------------------------- 读取
def load_tum(path):
    """TUM: ts x y z [qx qy qz qw] -> [(ts, (x,y,z)), ...]"""
    rows = []
    if not path or not os.path.isfile(path):
        return rows
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as fh:
        for line in fh:
            line = line.strip().lstrip("\ufeff")
            if not line or line.startswith("#"):
                continue
            v = line.replace(",", " ").replace(";", " ").split()
            if len(v) < 4:
                continue
            try:
                ts = float(v[0])
                xyz = (float(v[1]), float(v[2]), float(v[3]))
            except ValueError:
                continue
            rows.append((ts, xyz))
    rows.sort(key=lambda r: r[0])
    return rows


def load_xyz(path, cap=400000):
    pts = []
    if not path or not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as fh:
        for line in fh:
            line = line.strip().lstrip("\ufeff")
            if not line or line[0].isalpha():
                continue
            v = line.replace(",", " ").replace(";", " ").split()
            if len(v) < 3:
                continue
            try:
                pts.append((float(v[0]), float(v[1]), float(v[2])))
            except ValueError:
                continue
            if len(pts) >= cap:
                break
    return np.asarray(pts, float) if pts else None


def count_data_rows(path):
    n = 0
    if not path or not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as fh:
        for line in fh:
            s = line.strip().lstrip("\ufeff")
            if not s or s[0].isalpha():
                continue
            n += 1
    return n


def read_pairs(ds):
    """comparison_pairs.csv -> ({帧号: 时间戳}, [帧号...])"""
    path = os.path.join(ds, "comparison_pairs.csv")
    idx2ts, order = {}, []
    if not os.path.isfile(path):
        return idx2ts, order
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            try:
                i = int(r["frame_index"])
                ts = float(r["timestamp_seconds"])
            except (KeyError, ValueError):
                continue
            idx2ts[i] = ts
            order.append(i)
    order.sort()
    return idx2ts, order


# -------------------------------------------------------------------- 计算
def align_deviation(ref_rows, att_rows):
    """按时间戳对齐 -> [(ts, |d|, (dx,dy,dz)), ...] 和没对上的帧数"""
    cm = {round(ts, 6): xyz for ts, xyz in ref_rows}
    out, miss = [], 0
    for ts, xyz in att_rows:
        c = cm.get(round(ts, 6))
        if c is None:
            miss += 1
            continue
        d = (xyz[0] - c[0], xyz[1] - c[1], xyz[2] - c[2])
        out.append((ts, math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), d))
    return out, miss


def path_length(rows):
    s = 0.0
    for k in range(1, len(rows)):
        a, b = rows[k - 1][1], rows[k][1]
        s += math.sqrt(sum((a[j] - b[j]) ** 2 for j in range(3)))
    return s


def cloud_scale(pts):
    if pts is None or len(pts) < 2:
        return 0.0
    return float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))


ZOOM_MEDIAN_MULT = 2.6   # 放大子图视窗半宽 = k x 中位半径


def core_radius(kept, mult=ZOOM_MEDIAN_MULT):
    """返回近场核心半径（米）：mult x 中位半径；点太少或无效时返回 None。

    中位半径对远点尾巴免疫，所以这个阈值能自适应不同尺度的地图：
      - 点云连续展布（走廊、房间）-> 阈值远大于实际范围，一个点都不丢；
      - 存在成片稀疏远点（ORB-SLAM3 低视差 far point，贴在大致平行于像面的
        平面上）-> 尾巴整体被切掉，视图回到真正参与定位的近场地图。
    """
    pts = [P[:, [0, 2]] for P in kept.values() if P is not None and len(P)]
    if not pts or mult is None or mult <= 0:
        return None
    pool = np.vstack(pts)
    if len(pool) < 64:
        return None
    c = np.median(pool, axis=0)
    med = float(np.median(np.linalg.norm(pool - c, axis=1)))
    if not np.isfinite(med) or med <= 0:
        return None
    return float(mult) * med


# --------------------------------------------------------------------- 主
def build_parser():
    ap = argparse.ArgumentParser(description="正常 vs 攻击：位移偏差折线 + 建图点云对比")
    ap.add_argument("--ds", required=True, help="数据集根目录（读 comparison_pairs.csv）")
    ap.add_argument("--runs", required=True, help="~/ORB_SLAM3/runs/4seasons")
    ap.add_argument("--out", required=True, help="输出前缀（写 .png 和 .pdf）")
    ap.add_argument("--groups", default="fixed_pixel,world_plane")
    ap.add_argument("--reference", default="clean", help="基准组（默认 clean）")
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--no-pdf", action="store_true")
    ap.add_argument("--no-cjk", action="store_true")
    ap.add_argument("--zoom-k", type=float, default=ZOOM_MEDIAN_MULT,
                    help="(b) 放大子图视窗半宽 = k x 中位半径（0=不加子图，默认 %.1f）"
                         % ZOOM_MEDIAN_MULT)
    return ap


def main():
    args = build_parser().parse_args()
    args.ds = os.path.expanduser(args.ds)
    args.runs = os.path.expanduser(args.runs)
    args.out = os.path.expanduser(args.out)
    groups = [g for g in args.groups.split(",") if g]
    ref = args.reference

    cjk_ok = False if args.no_cjk else setup_fonts()

    idx2ts, frame_ids = read_pairs(args.ds)
    ts2idx = {round(ts, 6): i for i, ts in idx2ts.items()}

    print("[i] 运行目录: %s（下面所有数字都来自这里的真实产物）" % args.runs)
    # 双目有 CameraTrajectory.txt；单目没有，退回 KeyFrameTrajectory.txt（所有组统一用同一种）
    traj_name = "CameraTrajectory.txt"
    ref_rows = load_tum(os.path.join(args.runs, ref, traj_name))
    if len(ref_rows) < 2:
        traj_name = "KeyFrameTrajectory.txt"
        ref_rows = load_tum(os.path.join(args.runs, ref, traj_name))
    if len(ref_rows) < 2:
        raise SystemExit("[x] 读不到 %s/ 下的 CameraTrajectory.txt 或 KeyFrameTrajectory.txt"
                         " —— 先跑 bash run_compare.sh --slam" % os.path.join(args.runs, ref))
    if traj_name != "CameraTrajectory.txt":
        print("[i] 单目没有逐帧轨迹，基准和干扰组统一用 %s" % traj_name)
    L_ref = path_length(ref_rows)
    ref_cloud = load_xyz(os.path.join(args.runs, ref, "map_points.csv"))
    print("[i] 基准 %s: %d 帧, 轨迹长度 %.3f m, 地图点 %s"
          % (ref, len(ref_rows), L_ref,
             len(ref_cloud) if ref_cloud is not None else "无"))

    data = {}
    for g in groups:
        rows = load_tum(os.path.join(args.runs, g, traj_name))
        if len(rows) < 2:
            print("[!] 跳过 %s：没有/不足两帧的 %s" % (g, traj_name))
            continue
        dev, miss = align_deviation(ref_rows, rows)
        if not dev:
            print("[!] 跳过 %s：和基准没有对得上的时间戳" % g)
            continue
        d_abs = [d[1] for d in dev]
        data[g] = {
            "rows": rows,
            "dev": dev,
            "mean": float(np.mean(d_abs)),
            "max": float(np.max(d_abs)),
            "final": d_abs[-1],
            "len": path_length(rows),
            "miss": miss,
            "cloud": load_xyz(os.path.join(args.runs, g, "map_points.csv")),
        }
        print("[i] %-12s 帧=%3d 对齐=%3d 未对齐=%d | 长度=%.3f m | "
              "偏差 平均=%.3f 最大=%.3f 终点=%.3f m | 终点占轨迹=%.2f%% | 地图点=%s"
              % (g, len(rows), len(data[g]["dev"]), data[g]["miss"],
                 data[g]["len"], data[g]["mean"], data[g]["max"], data[g]["final"],
                 100.0 * data[g]["final"] / L_ref if L_ref > 0 else 0.0,
                 len(data[g]["cloud"]) if data[g]["cloud"] is not None else "无"))
        if miss:
            print("      [!] %d 帧没和基准对齐（攻击后可能丢跟踪）" % miss)

    if not data:
        raise SystemExit("[x] 一个攻击组都没读成功")

    # ------------------------------------------------------------------ 绘图
    # 两个干扰组 -> 2x2：上排 (a)(b) 位移偏差折线，下排 (c)(d) 建图点云对比
    # 只有一个干扰组 -> 自动降级成 1x2：(a) 偏差折线 + (b) 建图对比
    clouds = {ref: ref_cloud}
    for _g, _s in data.items():
        clouds[_g] = _s["cloud"]

    pairs = [(g, data[g]) for g in groups if g in data]

    def cn(name):
        return GROUP_CN.get(name, name)

    legend_fs, title_fs, label_fs, tick_fs = 8.0, 10.5, 9.5, 8.5

    if len(pairs) >= 2:
        fig, axes = plt.subplots(2, 2, figsize=(15.4, 10.6))
        slots = [(axes[0][0], axes[1][0], "a", "c"),
                 (axes[0][1], axes[1][1], "b", "d")]
    else:
        # 只有一组干扰数据：一排放两个子图（左=偏差折线，右=建图对比）
        fig, axes = plt.subplots(1, 2, figsize=(15.4, 5.9))
        slots = [(axes[0], axes[1], "a", "b")]

    # ---------------- 上排：位移偏差折线（基线 = 正常数据集）
    def draw_dev(ax, group, s_, letter):
        c = ATTACK_LN[0]
        xs, ys = [], []
        for j, d in enumerate(s_["dev"]):
            xs.append(ts2idx.get(round(d[0], 6), j))
            ys.append(d[1])
        x0 = min(xs) if xs else 0.0
        x1 = max(xs) if xs else 1.0
        span = (x1 - x0) or 1.0

        ax.axhline(0.0, color=CLEAN_C, lw=2.2,
                   label=("正常数据集（基线，偏差 = 0）" if cjk_ok
                          else "%s baseline (deviation = 0)" % ref))
        ax.plot(xs, ys, "-o", lw=1.8, ms=2.6, color=c,
                label=("%s 偏差（最大 %.3f m，终点 %.3f m）"
                       % (cn(group), s_["max"], s_["final"]) if cjk_ok else
                       "%s deviation (max %.3f, final %.3f)"
                       % (group, s_["max"], s_["final"])))
        ax.fill_between(xs, 0.0, ys, color=c, alpha=0.14)

        def put(x, y, text):
            if x >= x1 - 0.16 * span:
                ha, dx = "right", -8
            elif x <= x0 + 0.16 * span:
                ha, dx = "left", 8
            else:
                ha, dx = "center", 0
            ax.annotate(text, (x, y), textcoords="offset points", xytext=(dx, 8),
                        fontsize=8.0, color=c, ha=ha, va="bottom", zorder=6)

        kmax = int(np.argmax(ys))
        put(xs[kmax], ys[kmax], ("最大 %.3f m" if cjk_ok else "max %.3f m") % ys[kmax])
        if kmax != len(ys) - 1:
            put(xs[-1], ys[-1], ("终点 %.3f m" if cjk_ok else "final %.3f m") % ys[-1])

        ax.set_title(("(%s) 位移偏差：%s vs 正常数据集" % (letter, cn(group)) if cjk_ok
                      else "(%s) deviation: %s vs clean" % (letter, group)),
                     fontsize=title_fs, pad=7)
        ax.set_xlabel("帧号" if cjk_ok else "frame index", fontsize=label_fs)
        ax.set_ylabel("相对正常数据集的位移偏差 (m)" if cjk_ok
                      else "deviation vs clean (m)", fontsize=label_fs)
        ax.tick_params(labelsize=tick_fs)
        ax.grid(alpha=0.3)
        ytop = max([s_["max"]] + [0.0]) or 1.0
        ax.set_ylim(-0.06 * ytop, 1.22 * ytop)
        ax.set_xlim(x0 - 0.02 * span, x1 + 0.02 * span)
        ax.legend(fontsize=legend_fs, loc="upper left", framealpha=0.92)

    # ---------------- 下排：建图点云对比（全部地图点 + 空白角密集区放大子图）
    def draw_map(ax, group, letter):
        want = ((ref, CLEAN_PT, 0.75), (group, ATTACK_PT[0], 0.60))

        def scat(target, size):
            for name, col, alpha in want:
                P = clouds.get(name)
                if P is None or len(P) == 0:
                    continue
                lab = None
                if target is ax:
                    if cjk_ok:
                        lab = ("蓝点：正常数据集" if name == ref else "红点：%s" % cn(name))
                    else:
                        lab = ("blue: clean" if name == ref else "red: %s" % name)
                target.scatter(P[:, 0], P[:, 2], s=size, c=col, alpha=alpha,
                               edgecolors="none", label=lab)

        scat(ax, 2.2)

        have = [clouds[n][:, [0, 2]] for n, _c, _a in want
                if clouds.get(n) is not None and len(clouds[n])]
        order, zoom_corner = None, None
        if have:
            pool = np.vstack(have)
            lo = pool.min(axis=0)
            hi = pool.max(axis=0)
            ctr = (lo + hi) / 2.0
            half = max(hi[0] - lo[0], hi[1] - lo[1]) / 2.0
            half = half * 1.04 if half > 0 else 1.0
            if len(pool) >= 64:
                rel = (pool - ctr) / (2.0 * half)
                cnt = {}
                for tag, xr, yr in (("br", (0.10, 0.50), (-0.50, -0.10)),
                                    ("bl", (-0.50, -0.10), (-0.50, -0.10)),
                                    ("tr", (0.10, 0.50), (0.10, 0.50)),
                                    ("tl", (-0.50, -0.10), (0.10, 0.50))):
                    m = ((rel[:, 0] >= xr[0]) & (rel[:, 0] <= xr[1]) &
                         (rel[:, 1] >= yr[0]) & (rel[:, 1] <= yr[1]))
                    cnt[tag] = int(m.sum())
                tie = {"br": 0, "bl": 1, "tr": 2, "tl": 3}
                order = sorted(cnt, key=lambda t: (cnt[t], tie[t]))
                zoom_half = core_radius({n: clouds.get(n) for n, _c, _a in want},
                                        float(args.zoom_k))
                if zoom_half and zoom_half >= 0.75 * half:
                    zoom_half = None
                if zoom_half:
                    zc = np.median(pool, axis=0)
                    zoom_corner = order[0]
                    boxes = {"br": [0.575, 0.035, 0.39, 0.39],
                             "bl": [0.035, 0.035, 0.39, 0.39],
                             "tr": [0.575, 0.575, 0.39, 0.39],
                             "tl": [0.035, 0.575, 0.39, 0.39]}
                    axins = ax.inset_axes(boxes[zoom_corner])
                    scat(axins, 2.2)
                    axins.set_xlim(zc[0] - zoom_half, zc[0] + zoom_half)
                    axins.set_ylim(zc[1] - zoom_half, zc[1] + zoom_half)
                    axins.set_box_aspect(1)
                    axins.tick_params(labelbottom=False, labelleft=False,
                                      length=2.5, width=0.6)
                    axins.grid(alpha=0.25)
                    axins.set_title(("密集区放大（%.1f m 见方）" if cjk_ok
                                     else "dense region (zoom, %.1f m)")
                                    % (2.0 * zoom_half), fontsize=8.0, pad=3)
                    try:
                        ax.indicate_inset_zoom(axins, edgecolor="0.35", lw=1.0)
                    except Exception:
                        ax.add_patch(plt.Rectangle((zc[0] - zoom_half, zc[1] - zoom_half),
                                                   2 * zoom_half, 2 * zoom_half,
                                                   fill=False, ec="0.35", lw=1.0, ls="--"))
                    print("[i] 图(%s) 放大子图: 放在 %s 角，视窗 %.1f m 见方（主图仍是全部点）"
                          % (letter, zoom_corner, 2 * zoom_half))
            ax.set_xlim(ctr[0] - half, ctr[0] + half)
            ax.set_ylim(ctr[1] - half, ctr[1] + half)

        ax.set_box_aspect(1)
        ax.set_title(("(%s) 建图对比：%s vs 正常数据集" % (letter, cn(group)) if cjk_ok
                      else "(%s) map: %s vs clean" % (letter, group)),
                     fontsize=title_fs, pad=7)
        ax.set_xlabel("X (m)", fontsize=label_fs)
        ax.set_ylabel("Z (m)", fontsize=label_fs)
        ax.tick_params(labelsize=tick_fs)
        ax.grid(alpha=0.3)
        loc_map = {"br": "lower right", "bl": "lower left",
                   "tr": "upper right", "tl": "upper left"}
        if zoom_corner:
            leg_corner = next((t for t in order if t != zoom_corner), "tr")
        else:
            leg_corner = "tr"
        ax.legend(fontsize=legend_fs, loc=loc_map[leg_corner], framealpha=0.92,
                  borderaxespad=0.6)

    for (g, s_), (ax_d, ax_m, l_dev, l_map) in zip(pairs, slots):
        draw_dev(ax_d, g, s_, l_dev)
        draw_map(ax_m, g, l_map)
    for ax_d, ax_m, _l1, _l2 in slots[len(pairs):]:
        ax_d.axis("off")
        ax_m.axis("off")

    if cjk_ok:
        supt = ("正常数据集 vs 干扰数据集：位移偏差与建图对比（基准 = %s，无 ground truth）"
                % ref)
        if len(pairs) >= 2:
            note = ("注：(a)(b) 为逐帧位移偏差折线，基准 = %s 估计轨迹；"
                    "(c)(d) 为全部地图点 X-Z 俯视，蓝 = 正常数据集，红 = 干扰数据集，"
                    "角落子图为密集区放大。" % ref)
        else:
            note = ("注：(a) 为逐帧位移偏差折线，基准 = %s 估计轨迹；"
                    "(b) 为全部地图点 X-Z 俯视，蓝 = 正常数据集，红 = 干扰数据集，"
                    "角落子图为密集区放大。" % ref)
    else:
        supt = "clean vs attacked: deviation and mapping (baseline = %s)" % ref
        if len(pairs) >= 2:
            note = ("Note: (a)(b) per-frame deviation vs %s; (c)(d) all map points in X-Z, "
                    "blue = clean, red = attacked, corner inset = dense region." % ref)
        else:
            note = ("Note: (a) per-frame deviation vs %s; (b) all map points in X-Z, "
                    "blue = clean, red = attacked, corner inset = dense region." % ref)
    fig.suptitle(supt, fontsize=12.5, y=0.985)
    fig.text(0.5, 0.012, note, ha="center", fontsize=8.2, color="#555555")
    fig.tight_layout(rect=(0.0, 0.028, 1.0, 0.955), h_pad=2.0, w_pad=1.4)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    exts = ("png",) if args.no_pdf else ("png", "pdf")
    for ext in exts:
        path = "%s.%s" % (args.out, ext)
        fig.savefig(path, dpi=args.dpi, facecolor="white")
        print("[i] 写出 %s" % path)
    plt.close(fig)


if __name__ == "__main__":
    main()