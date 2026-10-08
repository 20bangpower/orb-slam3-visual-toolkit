#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pangolin_panel2.py -- 复刻论文 Fig.4 左侧那块"全局地图"面板（要素齐全版）

按 ORB-SLAM3 Viewer(Viewer.cc / MapDrawer.cc) 的真实画法逐项复刻：
    黑点 = 全部地图点                       Map::GetAllMapPoints() -> map_points.csv
    红点 = 当前帧的参考地图点(reference)      MapDrawer::mReferenceMapPoints -> ref_map_points.csv
           没有该文件时用 --ref-mode 近似（默认 frustum：当前关键帧视锥内的点）
    蓝色 = 关键帧相机视锥符号（小三角 '>'），串成 >>>>> 链   KeyFrameTrajectory.txt
    绿色 = 关键帧之间的连线(graph) + 当前相机
    白底 / 无网格 / 黑边框；默认取景把"地图点+关键帧链+相机"全包进来，不裁掉任何要素

例：
    python3 pangolin_panel2.py \
        --points map_points.csv --traj KeyFrameTrajectory.txt \
        --cam CameraTrajectory.txt --ref-mode frustum --menu none \
        --out pangolin_panel.png
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

MENU_PRESETS = {
    "paper": [
        ("chk", "Follow Camera", True), ("btn", "Camera View", None), ("btn", "Top View", None),
        ("chk", "Show Points", True), ("chk", "Show KeyFrames", True), ("chk", "Show Graph", True),
        ("chk", "Show Inertial Graph", True), ("chk", "Localisation Mode", False),
        ("btn", "Reset", None), ("btn", "Stop", None), ("chk", "Step By Step", False),
    ],
    "orb3": [
        ("chk", "Show Points", True), ("chk", "Show KeyFrames", True), ("chk", "Show Graph", True),
        ("chk", "Show Inertial Graph", True), ("chk", "Localisation Mode", False),
        ("btn", "Reset", None), ("btn", "Stop", None), ("chk", "Step By Step", False),
    ],
    "none": [],
}

C_PANEL, C_BTN, C_EDGE, C_SHADOW = "#EDEDED", "#F6F6F6", "#9C9C9C", "#C8C8C8"
PLANE = {"xz": (0, 2), "xy": (0, 1), "yz": (1, 2)}


# ------------------------------------------------------------------------ I/O
def load_tum(path):
    """TUM 格式：ts x y z qx qy qz qw（也兼容只有 xyz）。"""
    if not path or not os.path.isfile(path):
        return None, None
    xyz, quat = [], []
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as fh:
        for line in fh:
            line = line.strip().lstrip("\ufeff")
            if not line or line.startswith("#"):
                continue
            try:
                vals = [float(v) for v in line.replace(",", " ").replace(";", " ").split()[:8]]
            except ValueError:
                continue
            if len(vals) >= 8:
                xyz.append(vals[1:4]); quat.append(vals[4:8])
            elif len(vals) >= 3:
                xyz.append(vals[:3]); quat.append([0.0, 0.0, 0.0, 1.0])
    if not xyz:
        return None, None
    return np.asarray(xyz, float), np.asarray(quat, float)


def load_xyz(path, max_n=600000):
    if not path or not os.path.isfile(path):
        return None
    pts = []
    with open(path, "r", encoding="utf-8-sig", errors="ignore") as fh:
        for line in fh:
            s = line.strip().lstrip("\ufeff")
            if not s or s[0].isalpha():          # 跳过 x,y,z 表头
                continue
            p = s.replace(",", " ").replace(";", " ").split()
            if len(p) < 3:
                continue
            try:
                pts.append([float(v) for v in p[:3]])
            except ValueError:
                continue
    if not pts:
        return None
    P = np.asarray(pts, float)
    return P if len(P) <= max_n else P[np.random.default_rng(0).choice(len(P), max_n, False)]


def autofind(root, names):
    for r in [root] + sorted(d for d in glob.glob(os.path.join(root, "*")) if os.path.isdir(d)):
        for n in names:
            p = os.path.join(r, n)
            if os.path.isfile(p):
                return p
    return None


# -------------------------------------------------------------------- geometry
def to2d(P, plane, rot, flip_x, flip_y):
    ia, ib = PLANE[plane]
    a, b = np.asarray(P[:, ia], float), np.asarray(P[:, ib], float)
    if flip_x:
        a = -a
    if flip_y:
        b = -b
    if rot:
        t = np.radians(rot)
        a, b = a * np.cos(t) - b * np.sin(t), a * np.sin(t) + b * np.cos(t)
    return a, b


def quat_to_R(q):
    """TUM 四元数 (x,y,z,w) -> 旋转矩阵 R（相机系 -> 世界系）。"""
    x, y, z, w = [float(v) for v in q]
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], float)


def quat_fwd(q):
    """四元数 -> 世界系下的相机前向(+z)单位向量。"""
    q = np.asarray(q, float)
    n = np.linalg.norm(q, axis=1, keepdims=True)
    n[n < 1e-12] = 1.0
    x, y, z, w = (q / n).T
    return np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], 1)


def dir2d(D, plane, rot, flip_x, flip_y):
    ia, ib = PLANE[plane]
    a, b = np.asarray(D[:, ia], float), np.asarray(D[:, ib], float)
    if flip_x:
        a = -a
    if flip_y:
        b = -b
    if rot:
        t = np.radians(rot)
        a, b = a * np.cos(t) - b * np.sin(t), a * np.sin(t) + b * np.cos(t)
    n = np.hypot(a, b)
    n[n < 1e-12] = 1.0
    return a / n, b / n


def tangent2d(a, b, k=3):
    da, db = np.gradient(a), np.gradient(b)
    if k > 1 and len(a) >= k:
        ker = np.ones(k) / k
        da, db = np.convolve(da, ker, "same"), np.convolve(db, ker, "same")
    n = np.hypot(da, db)
    n[n < 1e-12] = 1.0
    return da / n, db / n


# ------------------------------------------------------------------ 红点(参考点)
def select_ref_frustum(P, xyz, quat, half_deg, depth, near):
    """近似 viewer 里的红点：落在"当前关键帧相机视锥"内的地图点。"""
    if P is None or len(P) == 0 or xyz is None or len(xyz) == 0:
        return None
    t = np.asarray(xyz[-1], float)
    R = quat_to_R(np.asarray(quat[-1], float)) if quat is not None else np.eye(3)
    c = (np.asarray(P, float) - t) @ R            # 世界 -> 相机
    z = c[:, 2]
    r = np.linalg.norm(c, axis=1)
    r[r < 1e-9] = 1e-9
    m = (z > near) & (z / r > math.cos(math.radians(half_deg)))
    if depth and depth > 0:
        m &= (z < depth)
    return P[m]


def select_ref_radius(P, xyz, radius, frames):
    """近似 viewer 里的红点：离最近若干帧相机 < radius 米的地图点。"""
    if P is None or len(P) == 0 or xyz is None or len(xyz) == 0:
        return None
    tail = np.asarray(xyz[-min(frames, len(xyz)):], float)
    d = np.min(np.linalg.norm(np.asarray(P, float)[:, None, :] - tail[None, :, :], axis=2), axis=1)
    return P[d < radius]


# --------------------------------------------------------------------- render
def draw_menu(ax, menu_w, H, items, dpi):
    ax.add_patch(Rectangle((0, 0), menu_w, H, facecolor=C_PANEL, edgecolor="none", zorder=20))
    pad, gap = menu_w * 0.055, menu_w * 0.030
    item_h = menu_w * 0.105
    box = item_h * 0.60
    fs = item_h * 0.42 * 72.0 / dpi
    y = pad
    for kind, label, state in items:
        if kind == "btn":
            ax.add_patch(Rectangle((pad, y), menu_w - 2 * pad, item_h, facecolor=C_BTN,
                                   edgecolor=C_EDGE, linewidth=0.7, zorder=21))
            ax.add_patch(Rectangle((pad + 0.6, y + item_h - 1.2), menu_w - 2 * pad - 0.6, 1.2,
                                   facecolor=C_SHADOW, edgecolor="none", zorder=22))
            ax.text(menu_w * 0.5, y + item_h * 0.52, label, ha="center", va="center",
                    fontsize=fs, color="black", zorder=23)
        else:
            ax.add_patch(Rectangle((pad, y + (item_h - box) * 0.5), box, box,
                                   facecolor="#E9A3A3" if state else "#FFFFFF",
                                   edgecolor=C_EDGE, linewidth=0.7, zorder=21))
            ax.text(pad + box * 1.45, y + item_h * 0.52, label, ha="left", va="center",
                    fontsize=fs, color="black", zorder=23)
        y += item_h + gap


def draw_frustum(ax, px, py, ux, uy, depth, halfw, color, lw, zorder=6, alpha=0.95):
    """ORB-SLAM3 的相机符号：顶点(相机中心) + 前方一个方底的投影 = 小三角 '>'。"""
    bx, by = px + ux * depth, py + uy * depth      # 底面中心（相机前方）
    ox, oy = -uy, ux                               # 垂直方向
    ax.plot([px, bx + ox * halfw, bx - ox * halfw, px],
            [py, by + oy * halfw, by - oy * halfw, py],
            lw=lw, color=color, alpha=alpha, zorder=zorder,
            solid_joinstyle="miter", solid_capstyle="round")


def render(args):
    dpi, W, H = args.dpi, args.width, args.height
    items = MENU_PRESETS[args.menu]
    menu_w = 0.0 if not items else W * args.menu_width_frac

    xyz, quat = load_tum(args.traj)
    if xyz is None:
        raise SystemExit("[x] 读不到关键帧轨迹（%s），请用 --traj 指定。" % args.traj)
    P = load_xyz(args.points) if args.points else None
    R = load_xyz(args.ref) if args.ref else None
    cam_xyz, cam_quat = load_tum(args.cam) if args.cam else (None, None)

    # ---- 红点：优先真实 ref_map_points.csv，否则按视锥/半径近似
    if R is None and P is not None and args.ref_mode != "none":
        if args.ref_mode == "frustum":
            R = select_ref_frustum(P, xyz, quat, args.ref_fov * 0.5, args.ref_depth, args.ref_near)
            how = "当前关键帧视锥 FOV %.0f deg, depth %s m" % (
                args.ref_fov, "inf" if not args.ref_depth else "%.1f" % args.ref_depth)
        else:
            src = cam_xyz if cam_xyz is not None and len(cam_xyz) else xyz
            R = select_ref_radius(P, src, args.ref_radius, args.ref_frames)
            how = "离最近 %d 帧相机 < %.2f m" % (args.ref_frames, args.ref_radius)
        if R is not None:
            print("[i] 红点近似[%s]：%d 个（占全部地图点 %.0f%%）"
                  % (how, len(R), 100.0 * len(R) / max(1, len(P))))

    a, b = to2d(xyz, args.plane, args.rot, args.flip_x, args.flip_y)
    pa, pb = (to2d(P, args.plane, args.rot, args.flip_x, args.flip_y)
              if P is not None and len(P) else (None, None))
    ra, rb = (to2d(R, args.plane, args.rot, args.flip_x, args.flip_y)
              if R is not None and len(R) else (None, None))
    if cam_xyz is not None and len(cam_xyz):
        ca, cb = to2d(cam_xyz, args.plane, args.rot, args.flip_x, args.flip_y)
    else:
        ca, cb = a, b

    # ---------------- 取景：默认把所有要素都包进来，不裁掉任何东西
    Xs, Ys = [a], [b]
    if args.fit == "all":
        for u, v in ((pa, pb), (ra, rb), (ca, cb)):
            if u is not None and len(u):
                Xs.append(u); Ys.append(v)
    elif args.fit == "points":
        if pa is not None and len(pa):
            Xs.append(pa); Ys.append(pb)
    else:                                         # traj：只看轨迹（点会被推出画面，慎用）
        Xs.append(np.asarray([a[-1]])); Ys.append(np.asarray([b[-1]]))
    X = np.concatenate([np.asarray(v, float).ravel() for v in Xs])
    Y = np.concatenate([np.asarray(v, float).ravel() for v in Ys])
    q = max(0.0, min(49.0, args.robust * 100.0))
    x0, x1 = np.percentile(X, [q, 100.0 - q])
    y0, y1 = np.percentile(Y, [q, 100.0 - q])
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    span = max(x1 - x0, y1 - y0) * (1.0 + 2 * args.pad)
    span = max(span, args.min_span, 1e-6)
    x0, x1, y0, y1 = cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2

    # ---------------- 画
    fig = plt.figure(figsize=(W / dpi, H / dpi), dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.set_aspect("equal"); ax.axis("off")
    mx0, mx1, my0, my1 = menu_w, W, 0, H
    ax.add_patch(Rectangle((mx0, my0), mx1 - mx0, my1 - my0, facecolor="white",
                           edgecolor="none", zorder=1))
    mw, mh = mx1 - mx0, my1 - my0
    sc = min(mw / span, mh / span) * (1.0 - 2 * args.pad)

    def T(da, db):
        return (mx0 + mw / 2 + (np.asarray(da) - cx) * sc,
                my0 + mh / 2 - (np.asarray(db) - cy) * sc)

    if args.grid_step > 0:
        step = args.grid_step
        for g in np.arange(np.floor(x0 / step) * step, x1 + step, step):
            px, _ = T(g, y0)
            ax.plot([px, px], [my0, my1], ls=(0, (4, 4)), lw=0.6, color="#CFCFCF", zorder=2)
        for g in np.arange(np.floor(y0 / step) * step, y1 + step, step):
            _, py = T(x0, g)
            ax.plot([mx0, mx1], [py, py], ls=(0, (4, 4)), lw=0.6, color="#CFCFCF", zorder=2)

    # 黑点：全部地图点
    if pa is not None and len(pa):
        px, py = T(pa, pb)
        ax.scatter(px, py, s=args.pt_size, marker="s", color=args.pt_color,
                   alpha=args.pts_alpha, linewidths=0, zorder=3)
        keep = (px >= mx0) & (px <= mx1) & (py >= my0) & (py <= my1)
        if keep.sum() < len(px):
            print("[!] 有 %d/%d 个地图点在画面外（--robust 0 可以一个都不裁）"
                  % (len(px) - keep.sum(), len(px)))
    # 红点：当前帧参考地图点
    if ra is not None and len(ra):
        px, py = T(ra, rb)
        ax.scatter(px, py, s=args.ref_size, marker="s", color=args.ref_color,
                   alpha=args.ref_alpha, linewidths=0, zorder=4)

    # 蓝色关键帧符号 + 绿色 graph 连线
    if args.glyph_source == "kf":
        sa, sb, sq = a, b, quat
    else:
        sa, sb, sq = ca, cb, (cam_quat if cam_quat is not None else quat)
    ta, tb = T(sa, sb)
    if args.glyph_link and len(ta) > 1:
        ax.plot(ta, tb, lw=args.link_lw, color=args.link_color, alpha=0.9,
                zorder=5, solid_capstyle="round")

    ia, ib = PLANE[args.plane]
    D = quat_fwd(sq) if sq is not None else None
    if D is not None and np.all(np.hypot(D[:, ia], D[:, ib]) < 1e-9):
        D = None
    if D is None:
        fa, fb = tangent2d(np.asarray(sa, float), np.asarray(sb, float))
    else:
        fa, fb = dir2d(D, args.plane, args.rot, args.flip_x, args.flip_y)

    n = len(sa)
    every = max(1, int(args.glyph_every))
    depth_px = args.glyph_depth * span * sc
    halfw_px = args.glyph_halfw * span * sc
    for i in range(0, n, every):
        ux, uy = fa[i], -fb[i]                    # 世界方向 -> 像素方向( y 轴翻转 )
        nn = math.hypot(ux, uy)
        if nn < 1e-9:
            continue
        draw_frustum(ax, ta[i], tb[i], ux / nn, uy / nn, depth_px, halfw_px,
                     args.glyph_color, args.glyph_lw, zorder=6)

    # 绿色当前相机（放大）
    ux, uy = fa[-1], -fb[-1]
    nn = math.hypot(ux, uy) or 1.0
    draw_frustum(ax, ta[-1], tb[-1], ux / nn, uy / nn,
                 depth_px * args.cam_scale, halfw_px * args.cam_scale,
                 args.cam_color, args.glyph_lw * 1.8, zorder=8)

    if args.anchor_out:
        cx_ = ta[-1] + ux / nn * depth_px * args.cam_scale
        cy_ = tb[-1] + uy / nn * depth_px * args.cam_scale
        ox_, oy_ = -uy / nn, ux / nn
        hw = halfw_px * args.cam_scale
        pts = [(ta[-1], tb[-1]),
               (cx_ + ox_ * hw, cy_ + oy_ * hw),
               (cx_ - ox_ * hw, cy_ - oy_ * hw)]
        xs_ = [float(q[0]) for q in pts]; ys_ = [float(q[1]) for q in pts]
        m_ = max(6.0, 0.006 * min(W, H))
        info = {
            "cx": sum(xs_) / 3.0 / W, "cy": 1.0 - sum(ys_) / 3.0 / H,
            "x0": (min(xs_) - m_) / W, "x1": (max(xs_) + m_) / W,
            "y0": 1.0 - (max(ys_) + m_) / H, "y1": 1.0 - (min(ys_) - m_) / H,
            "panel": os.path.abspath(args.out),
        }
        import json
        with open(args.anchor_out, "w", encoding="utf-8") as fh:
            json.dump(info, fh, ensure_ascii=False, indent=2)
        print("[i] 当前相机锚点 -> %s  (中心 %.3f, %.3f)" % (args.anchor_out, info["cx"], info["cy"]))

    if n > 1 and every == 1:
        seg = np.hypot(np.diff(ta), np.diff(tb))
        med = float(np.median(seg)) or 1.0
        if med < depth_px * 0.9:
            print("[!] 关键帧很密（中位间距 %.1f px < 符号长 %.1f px），会连成一条实心带；"
                  "建议加 --glyph-every %d" % (med, depth_px, max(2, int(depth_px / med))))

    if items:
        draw_menu(ax, menu_w, H, items, dpi)
        ax.plot([menu_w, menu_w], [0, H], lw=0.8, color=C_EDGE, zorder=24)
    ax.add_patch(Rectangle((0, 0), W, H, facecolor="none", edgecolor="black",
                           linewidth=args.border, zorder=25))
    fig.savefig(args.out, dpi=dpi, facecolor="white")
    plt.close(fig)
    print("[i] 面板 -> %s (%dx%d｜黑点 %d｜红点 %d｜符号 %d｜视窗 %.3f m)"
          % (args.out, W, H, 0 if P is None else len(P), 0 if R is None else len(R), n, span))


def main():
    ap = argparse.ArgumentParser(description="论文 Fig.4 左图：要素齐全的全局地图面板")
    ap.add_argument("--root", default=".")
    ap.add_argument("--points", default=None, help="map_points.csv（黑点=全部地图点）")
    ap.add_argument("--ref", default=None, help="ref_map_points.csv（红点=参考地图点，可选）")
    ap.add_argument("--traj", default=None, help="KeyFrameTrajectory.txt（关键帧=蓝色符号）")
    ap.add_argument("--cam", default=None, help="CameraTrajectory.txt（当前相机取最后一帧）")
    ap.add_argument("--plane", default="xz", choices=["xz", "xy", "yz"])
    ap.add_argument("--rot", type=float, default=0.0)
    ap.add_argument("--flip-x", action="store_true")
    ap.add_argument("--flip-y", action="store_true")
    ap.add_argument("--menu", default="none", choices=sorted(MENU_PRESETS),
                    help="要不要那条 UI 菜单（论文 (a)(b) 没有，(c) 有；默认 none）")
    ap.add_argument("--menu-width-frac", type=float, default=0.155)
    ap.add_argument("--width", type=int, default=1200)
    ap.add_argument("--height", type=int, default=900)
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--fit", default="all", choices=["all", "points", "traj"],
                    help="取景：all=所有要素都包进来（默认，不裁要素）")
    ap.add_argument("--robust", type=float, default=0.005, help="取景前裁掉的离群点比例（0=一个不裁）")
    ap.add_argument("--min-span", type=float, default=0.0)
    ap.add_argument("--pad", type=float, default=0.02)
    ap.add_argument("--grid-step", type=float, default=0.0, help=">0 才画网格（论文没有格子）")
    ap.add_argument("--pt-size", type=float, default=1.6)
    ap.add_argument("--pt-color", default="#000000")
    ap.add_argument("--pts-alpha", type=float, default=0.85)
    ap.add_argument("--ref-size", type=float, default=2.4)
    ap.add_argument("--ref-color", default="#FF0000")
    ap.add_argument("--ref-alpha", type=float, default=0.9)
    ap.add_argument("--ref-mode", default="frustum", choices=["frustum", "radius", "none"],
                    help="没有 ref_map_points.csv 时怎么近似红点")
    ap.add_argument("--ref-fov", type=float, default=90.0, help="视锥全角(度)，TUM fr1 约 63")
    ap.add_argument("--ref-depth", type=float, default=0.0, help="视锥最远距离(m)，0=不限")
    ap.add_argument("--ref-near", type=float, default=0.10, help="视锥最近距离(m)")
    ap.add_argument("--ref-radius", type=float, default=0.35, help="radius 模式：半径(m)")
    ap.add_argument("--ref-frames", type=int, default=20, help="radius 模式：最近多少帧")
    ap.add_argument("--glyph-source", default="kf", choices=["kf", "cam"],
                    help="蓝符号沿哪条轨迹画：kf=关键帧（论文画法，默认）")
    ap.add_argument("--glyph-link", action="store_true", default=True)
    ap.add_argument("--no-glyph-link", dest="glyph_link", action="store_false")
    ap.add_argument("--glyph-every", type=int, default=1, help="每 N 个画一个符号（太密就调大）")
    ap.add_argument("--glyph-depth", type=float, default=0.020, help="符号长（占面板比例）")
    ap.add_argument("--glyph-halfw", type=float, default=0.012, help="符号半宽（占面板比例）")
    ap.add_argument("--glyph-lw", type=float, default=0.75)
    ap.add_argument("--glyph-color", default="#1F1FFF")
    ap.add_argument("--link-color", default="#00CC00")
    ap.add_argument("--link-lw", type=float, default=1.1)
    ap.add_argument("--cam-color", default="#00C000")
    ap.add_argument("--cam-scale", type=float, default=2.6, help="当前相机符号放大倍数")
    ap.add_argument("--border", type=float, default=1.6)
    ap.add_argument("--anchor-out", default=None,
                    help="把当前相机在面板上的位置写成 json（给 compose_fig4.py --anchor-file 用）")
    ap.add_argument("--out", default="pangolin_panel.png")
    args = ap.parse_args()

    if args.traj is None:
        args.traj = autofind(args.root, ("KeyFrameTrajectory.txt", "CameraTrajectory.txt"))
    if args.points is None:
        args.points = autofind(args.root, ("map_points.csv", "MapPoints.txt"))
    if args.ref is None:
        args.ref = autofind(args.root, ("ref_map_points.csv", "reference_map_points.csv"))
    if args.cam is None:
        args.cam = autofind(args.root, ("CameraTrajectory.txt",))
    render(args)


if __name__ == "__main__":
    main()