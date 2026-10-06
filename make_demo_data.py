#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_demo_data.py -- 造一份"像 TUM fr1/xyz"的假数据，用来先跑通/预览版式，
                     免得每次都要重新跑一遍 ORB-SLAM3。

生成（默认写到 ./demo/）：
    KeyFrameTrajectory.txt    69 个关键帧位姿（TUM 8 列）
    CameraTrajectory.txt      792 帧全轨迹位姿
    map_points.csv            约 3000 个地图点 x,y,z
    current_features.csv      u,v,used,vo（模仿你 rgbd_tum.cc 导出的那份）
    frame.png                 占位数据帧（灰度 + ORB-SLAM3 风格特征框）

注意：frame.png 是**合成占位图**，只是为了让版式跑通；正式出图请换成你真实的
      current_frame.png（或 TUM 的 rgb/*.png）。
"""

from __future__ import annotations

import argparse
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

RNG = np.random.default_rng(7)


def fr1_xyz_like(n):
    """造一条像 fr1/xyz 的小房间回环轨迹（x-z 平面约 1.5 m，y 有轻微起伏）。"""
    t = np.linspace(0.0, 1.0, n)
    ang = 2.0 * np.pi * 1.15 * t
    r = 0.34 + 0.30 * np.sin(2.0 * np.pi * 1.0 * t + 0.6)
    x = r * np.cos(ang) + 0.06 * np.sin(2 * np.pi * 5 * t)
    z = r * np.sin(ang) + 0.05 * np.cos(2 * np.pi * 4 * t)
    y = 0.34 * np.sin(2.0 * np.pi * 0.9 * t + 1.1) + 0.35
    return np.stack([x, y, z], axis=1)


def quat_from_dir(d):
    """让相机光轴(相机 z 轴)对齐 d，其余自由度固定，返回 (N,4) qx,qy,qz,qw。"""
    d = d / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
    ref = np.tile([0.0, 0.0, 1.0], (len(d), 1))
    v = np.cross(ref, d)
    c = np.einsum("ij,ij->i", ref, d)
    s = np.linalg.norm(v, axis=1)
    # 罗德里格斯 -> 四元数
    q = np.zeros((len(d), 4))
    ok = s > 1e-9
    q[ok, 0] = v[ok, 0] / s[ok] * np.sqrt(np.maximum(0.0, (1 - c[ok]) / 2))
    q[ok, 1] = v[ok, 1] / s[ok] * np.sqrt(np.maximum(0.0, (1 - c[ok]) / 2))
    q[ok, 2] = v[ok, 2] / s[ok] * np.sqrt(np.maximum(0.0, (1 - c[ok]) / 2))
    q[ok, 3] = np.sqrt(np.maximum(0.0, (1 + c[ok]) / 2))
    q[~ok, 3] = 1.0
    return q


def write_tum(path, xyz, quat):
    with open(path, "w", encoding="utf-8") as fh:
        for i, (p, q) in enumerate(zip(xyz, quat)):
            fh.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n"
                     % (i * 0.0334, p[0], p[1], p[2], q[0], q[1], q[2], q[3]))


def synth_frame(w=640, h=480):
    """合成一张灰度"室内"图（占位用）：墙面 + 地板 + 几块家具 + 纹理 + 噪声。"""
    yy, xx = np.mgrid[0:h, 0:w].astype(float)
    horizon = 0.46 * h
    img = np.where(yy < horizon, 0.80 - 0.10 * (yy / max(horizon, 1.0)),
                   0.52 - 0.16 * ((yy - horizon) / max(h - horizon, 1.0)))
    img += 0.05 * np.sin(xx / 22.0) * np.cos(yy / 27.0)
    # 墙/地板交界 + 踢脚线
    img[np.abs(yy - horizon) < 2] -= 0.10
    img[(yy > horizon) & (yy < horizon + 10)] -= 0.05
    # 家具：显示器、桌子、椅子、门框
    objs = [(0.07, 0.20, 0.31, 0.52, -0.22), (0.06, 0.52, 0.33, 0.60, -0.12),
            (0.45, 0.26, 0.78, 0.56, -0.26), (0.44, 0.58, 0.80, 0.66, -0.14),
            (0.60, 0.62, 0.82, 0.96, -0.16), (0.84, 0.06, 0.97, 0.50, -0.20),
            (0.36, 0.62, 0.58, 0.98, -0.10)]
    for x0, y0, x1, y1, v in objs:
        X0, X1 = int(x0 * w), int(x1 * w)
        Y0, Y1 = int(y0 * h), int(y1 * h)
        grad = 0.85 + 0.3 * ((xx[Y0:Y1, X0:X1] - X0) / max(1, X1 - X0))
        img[Y0:Y1, X0:X1] += v * grad
        img[Y0:Y1, X0:X0 + 2] -= 0.10
        img[Y0:Y0 + 2, X0:X1] += 0.08
    # 纹理（模拟墙面/桌面细节）
    for _ in range(260):
        cx, cy = RNG.uniform(0, w), RNG.uniform(0, h)
        r = RNG.uniform(2, 8)
        m = ((xx - cx) ** 2 + (yy - cy) ** 2) < r * r
        img[m] += RNG.normal(0, 0.035)
    img += RNG.normal(0, 0.010, (h, w))
    return np.clip(img, 0.0, 1.0)


def main():
    ap = argparse.ArgumentParser(description="生成 TUM 风格假数据（跑通/预览版式用）")
    ap.add_argument("--outdir", default="demo")
    ap.add_argument("--kf", type=int, default=69)
    ap.add_argument("--cam", type=int, default=792)
    ap.add_argument("--points", type=int, default=3097)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    cam = fr1_xyz_like(args.cam)
    d = np.gradient(cam, axis=0)
    d += RNG.normal(0, 0.05, d.shape)
    d[:, 1] *= 0.15
    write_tum(os.path.join(args.outdir, "CameraTrajectory.txt"), cam, quat_from_dir(d))

    sel = np.linspace(0, len(cam) - 1, args.kf).astype(int)
    kf = cam[sel]
    write_tum(os.path.join(args.outdir, "KeyFrameTrajectory.txt"), kf, quat_from_dir(d[sel]))

    # 地图点：围绕轨迹撒，靠近轨迹更密（像真实建图结果）
    idx = RNG.choice(len(kf), args.points)
    base = kf[idx] + RNG.normal(0, 1.0, (args.points, 3)) * np.array([0.15, 0.11, 0.15])
    far = RNG.normal(0, 1.0, (args.points, 3)) * np.array([0.42, 0.26, 0.42])
    far[:, 1] = np.abs(far[:, 1]) * 0.6
    P = base * 0.78 + far * 0.22
    with open(os.path.join(args.outdir, "map_points.csv"), "w", encoding="utf-8") as fh:
        fh.write("x,y,z\n")
        for p in P:
            fh.write("%.6f,%.6f,%.6f\n" % (p[0], p[1], p[2]))

    # 特征点：沿图像边缘权重采样 + used 标记
    img = synth_frame()
    gy, gx = np.gradient(img)
    mag = np.hypot(gx, gy)
    p = (mag ** 2).ravel()
    p = p / p.sum()
    n = 330
    flat = RNG.choice(len(p), n, replace=False, p=p)
    us, vs = (flat % img.shape[1]).astype(float), (flat // img.shape[1]).astype(float)
    used = (RNG.random(n) < 0.42).astype(int)
    vo = (RNG.random(n) < 0.30).astype(int)
    with open(os.path.join(args.outdir, "current_features.csv"), "w", encoding="utf-8") as fh:
        fh.write("u,v,used,vo\n")
        for u, v, uu, vv in zip(us, vs, used, vo):
            fh.write("%.1f,%.1f,%d,%d\n" % (u, v, uu, vv))

    # 占位帧：黑框=匹配，绿框=未匹配，红点=本帧在用
    fig = plt.figure(figsize=(6.4, 4.8), dpi=200)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img, cmap="gray", vmin=0, vmax=1)
    matched = RNG.random(n) < 0.55
    for i in range(n):
        c = "black" if matched[i] else "#22C722"
        ax.add_patch(plt.Rectangle((us[i] - 5, vs[i] - 5), 10, 10, fill=False,
                                   edgecolor=c, linewidth=0.85))
    ax.scatter(us[used == 1], vs[used == 1], s=1.4, c="#FF2020", marker="o", linewidths=0)
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    fig.savefig(os.path.join(args.outdir, "frame.png"), facecolor="white")
    plt.close(fig)

    print("[i] 假数据写到 %s/  (kf=%d cam=%d points=%d feats=%d)"
          % (args.outdir, len(kf), len(cam), len(P), n))


if __name__ == "__main__":
    main()


