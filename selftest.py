#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
selftest.py -- 用自己生成的假数据把整条出图链路跑一遍，检查稳定性和降级行为。

不需要 SLAM、不需要真数据集、不需要 bash：

    python3 selftest.py              # 全部用例
    python3 selftest.py --quick      # 帧数和图都缩到最小，先看通不通
    python3 selftest.py --out 目录   # 指定沙箱目录（默认 ./selftest_out）

用例：
  1 地图面板 pangolin_panel2.py：三组都能画，都出锚点 json
  2 拼图 compose_compare.py：三组 -> 每组一张 + both；只有一个干扰组 -> 只出一张
  3 位移偏差 plot_deviation.py：两个干扰组 -> 2x2；一个 -> 1x2
  4 逐帧视频 make_fig4_video.py：fig4 模式，两组 / 一组
  5 逐帧对照导出 export_frame_pairs.py
  6 坏输入：缺 clean / map_points 乱码 / 空目录 -> 要报错清楚，不能抛 Traceback
  7 网页界面 gui_app.py：起服务，查组检测和产物列表
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
GROUPS = ("clean", "fixed_pixel", "world_plane")
CN = {"clean": "正常数据集", "fixed_pixel": "固定像素贴图", "world_plane": "世界平面投影贴图"}

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

RESULTS = []
ARGS = None


def tool(name):
    return os.path.join(HERE, name)


def add(name, ok, sec=0.0, note=""):
    RESULTS.append((name, bool(ok), sec, note))
    print("  %s %-46s %5.1fs  %s" % ("[ok]  " if ok else "[FAIL]", name, sec, note), flush=True)


def run(args, timeout=1200):
    t0 = time.time()
    p = subprocess.run([PY] + list(args), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, encoding="utf-8", errors="replace", timeout=timeout)
    return p.returncode, p.stdout or "", time.time() - t0


def size_ok(path, minimum=400):
    return os.path.isfile(path) and os.path.getsize(path) >= minimum


def tail(text, lines=4):
    rows = [r for r in text.strip().splitlines() if r.strip()]
    return " / ".join(rows[-lines:])[:220]


def no_traceback(text):
    return "Traceback (most recent call last)" not in text


def make_frames(ds, groups, n, modified_from, size=(180, 135)):
    ts0 = 1620666926.703571712
    rows = []
    for g in groups:
        for cam in ("cam0", "cam1"):
            os.makedirs(os.path.join(ds, g, cam), exist_ok=True)
    for i in range(n):
        ts = ts0 + i * 0.033333333
        paths = {}
        for g in groups:
            for ci, cam in enumerate(("cam0", "cam1")):
                img = Image.new("RGB", size, (32, 34, 38))
                d = ImageDraw.Draw(img)
                rr = random.Random(1000 * i + ci + (7 if g == "clean" else 0))
                for _ in range(70):
                    x = rr.randrange(size[0])
                    y = rr.randrange(size[1])
                    d.rectangle([x, y, x + rr.randrange(2, 9), y + rr.randrange(2, 9)],
                                fill=(80 + rr.randrange(140),) * 3)
                d.rectangle([6, 6, 44, 24], outline=(0, 200, 120), width=2)
                if g != "clean" and i >= modified_from:
                    px = 26 + (13 * i) % 90
                    py = 34 + (7 * i) % 58
                    d.rectangle([px, py, px + 40, py + 28], fill=(248, 248, 248))
                    d.rectangle([px, py, px + 40, py + 28], outline=(220, 40, 40), width=1)
                name = "%d.png" % int(round(ts * 1e9))
                img.save(os.path.join(ds, g, cam, name))
                paths["%s_%s" % (g, "left" if cam == "cam0" else "right")] = "%s/%s/%s" % (g, cam, name)
        row = {"frame_index": i, "timestamp_seconds": "%.9f" % ts,
               "modified_in_variants": 1 if i >= modified_from else 0}
        row.update(paths)
        rows.append(row)
    cols = ["frame_index", "timestamp_seconds", "modified_in_variants"]
    for g in GROUPS:
        for side in ("left", "right"):
            cols.append("%s_%s" % (g, side))
    lines = [",".join(cols)]
    for r in rows:
        lines.append(",".join(str(r.get(c, "")) for c in cols))
    with open(os.path.join(ds, "comparison_pairs.csv"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines) + "\n")
    return rows


def drift_traj(path, drift, seed):
    if not os.path.isfile(path):
        return
    rnd = random.Random(seed)
    out = []
    with open(path, encoding="utf-8", errors="ignore") as fh:
        for i, line in enumerate(fh):
            v = line.split()
            if len(v) < 8:
                continue
            k = i / 18.0
            p = [float(v[1]), float(v[2]), float(v[3])]
            p[0] += drift * k + 0.004 * math.sin(k * 3.1) + rnd.gauss(0, 0.0015)
            p[1] += 0.6 * drift * k + 0.003 * math.cos(k * 2.7) + rnd.gauss(0, 0.0015)
            p[2] += drift * k + rnd.gauss(0, 0.0015)
            out.append("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f" %
                       (float(v[0]), p[0], p[1], p[2], float(v[4]), float(v[5]), float(v[6]), float(v[7])))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(out) + "\n")


def drift_points(path, seed, spread=0.01, outliers=6):
    if not os.path.isfile(path):
        return
    rnd = random.Random(seed)
    with open(path, encoding="utf-8", errors="ignore") as fh:
        lines = [l for l in fh.read().splitlines() if l.strip()]
    body = []
    for l in lines:
        if l[0].isalpha():
            body.append(l)
            continue
        v = l.replace(",", " ").split()
        if len(v) < 3:
            continue
        body.append("%.6f,%.6f,%.6f" % (float(v[0]) + rnd.gauss(0, spread),
                                        float(v[1]) + rnd.gauss(0, spread),
                                        float(v[2]) + rnd.gauss(0, spread)))
    for _ in range(outliers):
        body.append("%.6f,%.6f,%.6f" % (rnd.uniform(-35, 35), rnd.uniform(-35, 35), rnd.uniform(-35, 35)))
    if not any(l[0].isalpha() for l in body):
        body.insert(0, "x,y,z")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(body) + "\n")


def make_runs(root, groups, kf, cam, pts, quick=False):
    os.makedirs(os.path.join(root, "runs", "4seasons"), exist_ok=True)
    for gi, g in enumerate(groups):
        rd = os.path.join(root, "runs", "4seasons", g)
        os.makedirs(rd, exist_ok=True)
        rc, out, _ = run([tool("make_demo_data.py"), "--outdir", rd,
                          "--kf", str(kf), "--cam", str(cam), "--points", str(pts)])
        if rc != 0:
            return rc, out
        frame = os.path.join(rd, "frame.png")
        if os.path.isfile(frame):
            if os.path.isfile(os.path.join(rd, "current_frame.png")):
                os.remove(os.path.join(rd, "current_frame.png"))
            os.rename(frame, os.path.join(rd, "current_frame.png"))
        if g != "clean":
            drift_traj(os.path.join(rd, "CameraTrajectory.txt"), 0.06 * (gi + 1), 200 + gi)
            drift_traj(os.path.join(rd, "KeyFrameTrajectory.txt"), 0.06 * (gi + 1), 300 + gi)
            drift_points(os.path.join(rd, "map_points.csv"), 400 + gi)
    return 0, ""


def runs_dir(root, g):
    return os.path.join(root, "runs", "4seasons", g)


def copy_lines(path):
    if not os.path.isfile(path):
        return []
    with open(path, encoding="utf-8", errors="ignore") as fh:
        return [l for l in fh.read().splitlines() if l.strip()]


def make_frame_snapshots(ds, root, groups, rows, cams=("cam0",)):
    """按 run_video.sh --slam 的产物格式造逐帧快照：runs/<组>/frames/F###_*。"""
    n = len(rows)
    for g in groups:
        rd = runs_dir(root, g)
        fd = os.path.join(rd, "frames")
        os.makedirs(fd, exist_ok=True)
        kf = copy_lines(os.path.join(rd, "KeyFrameTrajectory.txt"))
        cam = copy_lines(os.path.join(rd, "CameraTrajectory.txt"))
        pts = copy_lines(os.path.join(rd, "map_points.csv"))
        for i in range(n):
            tag = "F%03d" % i
            rel = rows[i].get("%s_left" % g)
            src = os.path.join(ds, rel) if rel else ""
            if src and os.path.isfile(src):
                shutil.copyfile(src, os.path.join(fd, tag + "_current_frame.png"))
            frac = (i + 1) / float(n)
            if kf:
                with open(os.path.join(fd, tag + "_kf_traj.txt"), "w", encoding="utf-8", newline="\n") as fh:
                    fh.write("\n".join(kf[:max(2, int(len(kf) * frac))]) + "\n")
            if cam:
                with open(os.path.join(fd, tag + "_cam_traj.txt"), "w", encoding="utf-8", newline="\n") as fh:
                    fh.write("\n".join(cam[:max(2, int(len(cam) * frac))]) + "\n")
            if pts:
                head = [l for l in pts if l[0].isalpha()]
                body = [l for l in pts if not l[0].isalpha()]
                keep = head + body[:max(2, int(len(body) * frac))]
                with open(os.path.join(fd, tag + "_map_points.csv"), "w", encoding="utf-8", newline="\n") as fh:
                    fh.write("\n".join(keep) + "\n")


def cell_label(g, letter):
    return "(%s) %s" % (letter, "Normal Scenario (no attack)" if g == "clean" else CN[g] + " Scenario")


def panel_args(root, g, width=760, height=560):
    rd = runs_dir(root, g)
    a = ["--traj", os.path.join(rd, "KeyFrameTrajectory.txt"), "--menu", "none",
         "--ref-mode", "frustum", "--ref-fov", "63", "--fit", "all",
         "--width", str(width), "--height", str(height),
         "--glyph-every", "2", "--glyph-depth", "0.012", "--glyph-halfw", "0.007",
         "--cam-scale", "1.4", "--out", os.path.join(rd, "pangolin_panel.png"),
         "--anchor-out", os.path.join(rd, "pangolin_panel.anchor.json")]
    if os.path.isfile(os.path.join(rd, "map_points.csv")):
        a += ["--points", os.path.join(rd, "map_points.csv")]
    if os.path.isfile(os.path.join(rd, "CameraTrajectory.txt")):
        a += ["--cam", os.path.join(rd, "CameraTrajectory.txt")]
    return a


def row_spec(root, g):
    return "%s,%s,%s|%s,%s,%s" % (
        cell_label("clean", "a"), os.path.join(runs_dir(root, "clean"), "pangolin_panel.png"),
        os.path.join(runs_dir(root, "clean"), "current_frame.png"),
        cell_label(g, "a'"), os.path.join(runs_dir(root, g), "pangolin_panel.png"),
        os.path.join(runs_dir(root, g), "current_frame.png"))


def compose_cmd(root, g, out_prefix):
    rd = runs_dir(root, g)
    return ["--row", row_spec(root, g),
            "--cell-label", cell_label("clean", "a"), "--cell-label", cell_label(g, "a'"),
            "--cell-ann", "Normal Scenario (no attack)",
            "--cell-ann", CN[g],
            "--cell-anchor-file", os.path.join(runs_dir(root, "clean"), "pangolin_panel.anchor.json"),
            "--cell-anchor-file", os.path.join(rd, "pangolin_panel.anchor.json"),
            "--ann-inset", "0.035", "--no-cell-title", "--height", "2.4", "--fontsize", "11",
            "--dpi", str(ARGS.dpi), "--out", out_prefix]


def case_panels(root):
    for g in GROUPS:
        rc, out, sec = run([tool("pangolin_panel2.py")] + panel_args(root, g))
        rd = runs_dir(root, g)
        add("1 面板 %-13s" % g,
            rc == 0 and size_ok(os.path.join(rd, "pangolin_panel.png"))
            and size_ok(os.path.join(rd, "pangolin_panel.anchor.json"), 20) and no_traceback(out),
            sec, tail(out, 1))


def case_compose(root, out):
    gs = ["fixed_pixel", "world_plane"]
    loners = 0
    for i, g in enumerate(gs):
        prefix = os.path.join(out, "fig4_4seasons_" + g)
        rc, txt, sec = run([tool("compose_compare.py")] + compose_cmd(root, g, prefix))
        ok = (rc == 0 and size_ok(prefix + ".png") and size_ok(prefix + ".pdf", 800) and no_traceback(txt))
        add("2 拼图 %-13s" % g, ok, sec, tail(txt, 2))
        loners += 0 if ok else 1
    both = os.path.join(out, "fig4_4seasons_both")
    rows = []
    for g in gs:
        rows += ["--row", row_spec(root, g)]
    rows += ["--cell-label", cell_label("clean", "a"), "--cell-label", cell_label(gs[0], "a'"),
             "--cell-label", cell_label("clean", "b"), "--cell-label", cell_label(gs[1], "b'"),
             "--ann-inset", "0.035", "--no-cell-title", "--height", "2.4", "--fontsize", "11",
             "--dpi", str(ARGS.dpi), "--out", both]
    rc, txt, sec = run([tool("compose_compare.py")] + rows)
    add("2 拼图 两组叠一张 _both", rc == 0 and size_ok(both + ".png") and no_traceback(txt), sec, tail(txt, 1))
    return loners


def case_compose_single(root, out):
    prefix = os.path.join(out, "single_fig4_4seasons_fixed_pixel")
    rc, txt, sec = run([tool("compose_compare.py")] + compose_cmd(root, "fixed_pixel", prefix))
    add("2 拼图 只有一个干扰组", rc == 0 and size_ok(prefix + ".png") and no_traceback(txt), sec, tail(txt, 1))


def case_deviation(root, out):
    runs = os.path.join(root, "runs", "4seasons")
    cases = [("两组 2x2", "fixed_pixel,world_plane", "fig4_4seasons_deviation"),
             ("一组 1x2", "fixed_pixel", "single_fig4_4seasons_deviation")]
    for name, gs, stem in cases:
        prefix = os.path.join(out, stem)
        rc, txt, sec = run([tool("plot_deviation.py"), "--ds", ARGS.ds, "--runs", runs,
                            "--groups", gs, "--out", prefix, "--dpi", "110"])
        add("3 偏差图 %-10s" % name,
            rc == 0 and size_ok(prefix + ".png") and size_ok(prefix + ".pdf", 800) and no_traceback(txt),
            sec, tail(txt, 2))


def case_video(root, out):
    runs = os.path.join(root, "runs", "4seasons")
    cases = [("两组", "fixed_pixel,world_plane", 1280), ("一组", "fixed_pixel", 960)]
    for name, gs, width in cases:
        d = os.path.join(out, "video_" + ("2" if "," in gs else "1"))
        rc, txt, sec = run([tool("make_fig4_video.py"), "--mode", "fig4", "--ds", ARGS.ds,
                            "--runs", runs, "--groups", gs, "--max", str(ARGS.vframes),
                            "--hold", "1.0", "--width", str(width), "--fig4-dpi", "80",
                            "--jobs", "2", "--out", d])
        made = [f for f in os.listdir(d) if os.path.isfile(os.path.join(d, f))] if os.path.isdir(d) else []
        media = [f for f in made if f.lower().endswith((".gif", ".mp4", ".webm"))]
        for m in media:
            shutil.copyfile(os.path.join(d, m), os.path.join(ARGS.video_root, m))
        add("4 视频 %-10s" % name, rc == 0 and bool(media) and no_traceback(txt), sec,
            ",".join(media) if media else tail(txt, 2))


def case_frame_pairs(root, out):
    d = os.path.join(out, "frame_pairs")
    rc, txt, sec = run([tool("export_frame_pairs.py"), "--ds", ARGS.ds, "--out", d,
                        "--groups", "fixed_pixel,world_plane", "--cams", "cam0",
                        "--max", str(ARGS.pframes), "--dpi", "70"])
    idx = os.path.join(d, "index.csv")
    pairs = os.path.isdir(os.path.join(d, "pairs", "fixed_pixel"))
    sheets = os.path.isdir(os.path.join(d, "sheets"))
    add("5 逐帧对照导出", rc == 0 and size_ok(idx, 40) and pairs and sheets and no_traceback(txt),
        sec, tail(txt, 2))


def case_bad_inputs(root, out):
    bad = os.path.join(out, "bad")
    if os.path.isdir(bad):
        shutil.rmtree(bad)
    os.makedirs(bad)
    nocs = os.path.join(bad, "no_clean")
    make_frames(nocs, ("fixed_pixel",), 3, 1)
    os.makedirs(os.path.join(bad, "runs", "4seasons", "fixed_pixel"), exist_ok=True)
    shutil.copytree(runs_dir(root, "fixed_pixel"), runs_dir(bad, "fixed_pixel"), dirs_exist_ok=True)
    rc, txt, sec = run([tool("plot_deviation.py"), "--ds", nocs,
                        "--runs", os.path.join(bad, "runs", "4seasons"),
                        "--groups", "fixed_pixel", "--out", os.path.join(out, "bad_noclean"),
                        "--dpi", "80"])
    add("6 缺 clean：要报错不要崩", rc != 0 and no_traceback(txt) and ("[x]" in txt),
        sec, tail(txt, 1))

    junk = os.path.join(bad, "junk")
    shutil.copytree(root, junk, ignore=shutil.ignore_patterns("bad", "outputs"))
    jd = runs_dir(junk, "fixed_pixel")
    with open(os.path.join(jd, "map_points.csv"), "w", encoding="utf-8") as fh:
        fh.write("x,y,z\n这不是数字\n1,2\n\0\0\n")
    rc, txt, sec = run([tool("pangolin_panel2.py")] + panel_args(junk, "fixed_pixel", 520, 380))
    add("6 map_points 乱码：要能跳过", rc == 0 and no_traceback(txt) and size_ok(os.path.join(jd, "pangolin_panel.png")),
        sec, tail(txt, 1))

    rc, txt, sec = run([tool("export_frame_pairs.py"), "--ds", ARGS.ds, "--out", os.path.join(out, "bad_pairs"),
                        "--groups", "no_such_group", "--cams", "cam0", "--max", "2", "--dpi", "60"])
    add("6 不存在的组：要明确提示", no_traceback(txt) and ("[x]" in txt or "[!]" in txt), sec, tail(txt, 1))


def case_dirty_data(root, out):
    """脏数据：BOM / 制表符 / 分号 / 空文件 / 单目风格 / 帧数不一致。"""
    ign = shutil.ignore_patterns("outputs")

    # 8a 带 BOM 的 csv + 制表符 + 多一列 + 半角分号 + 科学计数，
    #    写 6 行，最后一行只有两列是故意写坏的 -> 应该读到 5 个点
    d = os.path.join(out, "dirty_csv")
    if os.path.isdir(d):
        shutil.rmtree(d)
    shutil.copytree(root, d, ignore=ign)
    with open(os.path.join(runs_dir(d, "fixed_pixel"), "map_points.csv"),
              "w", encoding="utf-8-sig", newline="\n") as fh:
        fh.write("# comment\n\nx,y,z\n")
        fh.write("1.0,2.0,3.0\n")
        fh.write("2.0\t3.0\t4.0\n")
        fh.write("3.0,4.0,5.0,0.9\n")
        fh.write("1e-3,2E-3,3e-3\n")
        fh.write("5.0;6.0;7.0\n")
        fh.write("7.0,8.0\n")
    rc, txt, sec = run([tool("pangolin_panel2.py")] + panel_args(d, "fixed_pixel", 520, 380))
    add("8 脏 csv（BOM/制表符/分号/多列）", rc == 0 and "黑点 5" in txt and no_traceback(txt),
        sec, tail(txt, 1))

    # 8b 单目风格：三组都没有 CameraTrajectory.txt，偏差图要退回关键帧轨迹
    d = os.path.join(out, "dirty_mono")
    if os.path.isdir(d):
        shutil.rmtree(d)
    shutil.copytree(root, d, ignore=ign)
    for g in GROUPS:
        p = os.path.join(runs_dir(d, g), "CameraTrajectory.txt")
        if os.path.isfile(p):
            os.remove(p)
    rc, txt, sec = run([tool("plot_deviation.py"), "--ds", ARGS.ds,
                        "--runs", os.path.join(d, "runs", "4seasons"),
                        "--groups", "fixed_pixel,world_plane",
                        "--out", os.path.join(out, "dirty_mono_dev"), "--dpi", "80"])
    add("8b 单目风格（无 CameraTrajectory）",
        rc == 0 and "单目没有逐帧轨迹" in txt and no_traceback(txt), sec, tail(txt, 1))

    # 8c 轨迹整个空 / 地图点空文件 -> 跳过坏组，剩下的照常出图
    d = os.path.join(out, "dirty_empty")
    if os.path.isdir(d):
        shutil.rmtree(d)
    shutil.copytree(root, d, ignore=ign)
    for n in ("CameraTrajectory.txt", "KeyFrameTrajectory.txt"):
        open(os.path.join(runs_dir(d, "world_plane"), n), "w").close()
    open(os.path.join(runs_dir(d, "fixed_pixel"), "map_points.csv"), "w").close()
    rc, txt, sec = run([tool("plot_deviation.py"), "--ds", ARGS.ds,
                        "--runs", os.path.join(d, "runs", "4seasons"),
                        "--groups", "fixed_pixel,world_plane",
                        "--out", os.path.join(out, "dirty_empty_dev"), "--dpi", "80"])
    add("8c 空文件：跳过坏组继续出图",
        rc == 0 and "跳过 world_plane" in txt and "地图点=无" in txt and no_traceback(txt),
        sec, tail(txt, 1))

    # 8d 帧数不一致 + 时间戳错位，要如实报未对齐而不是崩
    d = os.path.join(out, "dirty_len")
    if os.path.isdir(d):
        shutil.rmtree(d)
    shutil.copytree(root, d, ignore=ign)
    for g in ("fixed_pixel", "world_plane"):
        for n in ("CameraTrajectory.txt", "KeyFrameTrajectory.txt"):
            fp = os.path.join(runs_dir(d, g), n)
            with open(fp, encoding="utf-8", errors="ignore") as fh:
                lines = [l for l in fh.read().splitlines() if l.strip()]
            keep = lines[:max(3, int(len(lines) * 0.5))]
            rows = []
            for k, l in enumerate(keep):
                v = l.split()
                if k % 7 == 0 and v:
                    v[0] = "%.6f" % (float(v[0]) + 9.0)
                rows.append(" ".join(v))
            with open(fp, "w", encoding="utf-8", newline="\n") as fh:
                fh.write("\n".join(rows) + "\n")
    rc, txt, sec = run([tool("plot_deviation.py"), "--ds", ARGS.ds,
                        "--runs", os.path.join(d, "runs", "4seasons"),
                        "--groups", "fixed_pixel,world_plane",
                        "--out", os.path.join(out, "dirty_len_dev"), "--dpi", "80"])
    add("8d 帧数不一致 / 时间戳错位",
        rc == 0 and "未对齐" in txt and no_traceback(txt), sec, tail(txt, 1))


def free_port(start):
    """找一个真能独占绑上的端口（免得被上一次没关干净的界面占着）。"""
    for port in range(start, start + 40):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                s.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            s.bind(("127.0.0.1", port))
            return port
        except OSError:
            continue
        finally:
            s.close()
    return start


def http_json(url, timeout=10):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        import json
        return json.loads(r.read().decode("utf-8"))


def case_gui(root, ds_two, root_two):
    port = free_port(ARGS.gui_port)
    args = [tool("gui_app.py"), "--no-browser", "--port", str(port), "--root", root, "--ds", ARGS.ds]
    t0 = time.time()
    proc = subprocess.Popen([PY] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace")
    base = "http://127.0.0.1:%d" % port
    state = None
    for _ in range(60):
        time.sleep(0.25)
        try:
            state = http_json(base + "/api/state")
            break
        except Exception:
            if proc.poll() is not None:
                break
    if state is None:
        add("7 网页界面 起服务", False, time.time() - t0, "服务没起来")
        return
    ok = (state["available"] == {"clean": True, "fixed_pixel": True, "world_plane": True}
          and set(state["groups"]) == {"fixed_pixel", "world_plane"} and state["has_ds_clean"])
    add("7 网页界面 组检测（三组）", ok, time.time() - t0,
        "groups=%s" % state["groups"])
    try:
        ls = http_json(base + "/api/ls?path=" + urllib.parse.quote(os.path.dirname(ARGS.ds)))
        entry = [d for d in ls["dirs"] if d["path"] == ARGS.ds]
        add("7 网页界面 数据集目录标注", bool(entry) and entry[0]["has_clean"] and entry[0]["has_dataset"],
            time.time() - t0, "has_clean")
    except Exception as exc:
        add("7 网页界面 数据集目录标注", False, 0, str(exc))
    try:
        pr = http_json(base + "/api/products")
        figs = [f["name"] for f in pr["figures"]]
        vids = [v["name"] for v in pr["videos"]]
        add("7 网页界面 产物列表", any(n.endswith(".png") for n in figs) and len(vids) >= 1,
            time.time() - t0, "figures=%d videos=%d" % (len(figs), len(vids)))
    except Exception as exc:
        add("7 网页界面 产物列表", False, 0, str(exc))
    try:
        body = json.dumps({"ds": ds_two, "root": root_two}).encode("utf-8")
        urllib.request.urlopen(urllib.request.Request(base + "/api/config", data=body,
                                                      headers={"Content-Type": "application/json"}), timeout=10).read()
        st2 = http_json(base + "/api/state")
        up = st2["available"]
        ok2 = (up["world_plane"] is False and up["fixed_pixel"] is True and st2["has_ds_clean"])
        add("7 网页界面 只有两组时降级", ok2, time.time() - t0, "available=%s" % up)
    except Exception as exc:
        add("7 网页界面 只有两组时降级", False, 0, str(exc))
    try:
        proc.terminate()
        proc.wait(timeout=10)
    except Exception:
        proc.kill()


def main():
    global ARGS
    ap = argparse.ArgumentParser(description="ORB-SLAM3 可视化工具链 · 自生成数据自检")
    ap.add_argument("--out", default="selftest_out", help="沙箱目录")
    ap.add_argument("--quick", action="store_true", help="最小规模快速过一遍")
    ap.add_argument("--dpi", type=int, default=0, help="拼图 dpi（0=快速模式默认）")
    ap.add_argument("--frames", type=int, default=0, help="生成多少帧（默认 8，quick 4）")
    ap.add_argument("--gui-port", type=int, default=8791)
    ap.add_argument("--keep", action="store_true", default=True, help="保留沙箱（默认保留）")
    a = ap.parse_args()

    a.ds = os.path.abspath(os.path.join(a.out, "ds", "slam_stereo_pairs_selftest"))
    a.ds_two = os.path.abspath(os.path.join(a.out, "ds", "slam_stereo_pairs_two_groups"))
    a.dpi = a.dpi or 110
    a.frames = a.frames or (4 if a.quick else 8)
    a.vframes = 2 if a.quick else 3
    a.pframes = 2 if a.quick else 4
    a.video_dirs = []
    ARGS = a

    out = os.path.abspath(a.out)
    root = os.path.join(out, "root")
    root_two = os.path.join(out, "root_two")
    figdir = os.path.join(root, "outputs", "fig4_4seasons", "figures")
    a.video_root = os.path.join(root, "outputs", "fig4_4seasons", "video")
    os.makedirs(figdir, exist_ok=True)
    os.makedirs(a.video_root, exist_ok=True)
    print("=" * 72)
    print("自检沙箱: %s" % out)
    print("Python  : %s" % PY)
    print("=" * 72)

    t0 = time.time()
    print("\n[造数据] 三组假数据集 + 三组 runs 导出")
    rows3 = make_frames(a.ds, GROUPS, a.frames, max(1, a.frames // 3))
    make_frames(a.ds_two, ("clean", "fixed_pixel"), a.frames, max(1, a.frames // 3))
    rc, msg = make_runs(root, GROUPS, 24, 80, 500, a.quick)
    if rc == 0:
        make_frame_snapshots(a.ds, root, GROUPS, rows3)
        copy_two = os.path.join(out, "root_two")
        for g in ("clean", "fixed_pixel"):
            shutil.copytree(runs_dir(root, g), runs_dir(copy_two, g), dirs_exist_ok=True)
        msg = msg or "三组 runs 数据 + 逐帧快照就绪"
    add("0 自生成数据", rc == 0 and os.path.isfile(os.path.join(runs_dir(root, "clean"), "CameraTrajectory.txt")),
        time.time() - t0, msg or "三组 runs 数据就绪")

    print("\n[1] 地图面板")
    case_panels(root)
    print("\n[2] 拼图（Fig.4 对照版式）")
    case_compose(root, figdir)
    case_compose_single(root, figdir)
    print("\n[3] 位移偏差图")
    case_deviation(root, figdir)
    print("\n[4] 逐帧视频")
    case_video(root, os.path.join(root, "outputs", "fig4_4seasons", "video"))
    print("\n[5] 逐帧对照导出")
    case_frame_pairs(root, os.path.join(root, "outputs", "fig4_4seasons", "frame_pairs"))
    print("\n[6] 坏输入")
    case_bad_inputs(root, figdir)
    print("\n[8] 脏数据（BOM/制表符/空文件/单目/帧数不一致）")
    case_dirty_data(root, out)

    print("\n[7] 网页界面")
    case_gui(root, a.ds_two, root_two)

    ok = sum(1 for r in RESULTS if r[1])
    bad = [r for r in RESULTS if not r[1]]
    print("\n" + "=" * 72)
    print("结果: %d/%d 通过，用时 %.1fs" % (ok, len(RESULTS), time.time() - t0))
    if bad:
        print("失败项：")
        for name, _, _, note in bad:
            print("  - %s   %s" % (name, note))
    print("沙箱产物：%s" % out)
    print("=" * 72)
    report = os.path.join(out, "selftest_report.txt")
    with open(report, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("自检结果 %d/%d 通过\n\n" % (ok, len(RESULTS)))
        for name, good, sec, note in RESULTS:
            fh.write("%s %-46s %6.1fs  %s\n" % ("ok  " if good else "FAIL", name, sec, note))
    print("报告：%s" % report)
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())