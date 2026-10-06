#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_fig4_video.py -- 逐帧 "(a)正常 vs (a')干扰" 对照 -> 视频（每帧停留 N 秒）

两种模式
  --mode pair   只读数据集图片：左=clean 原图，右=干扰图（红框=改动区），不用跑 SLAM
  --mode fig4   论文 Fig.4 版式：每个子图 = [地图面板 | 当前帧]
                需要先跑 SLAM 的逐帧快照（VIS_FRAMES_DIR，见 run_video.sh 的 --slam 一步）

产物（都在 --out 目录下）
  slides/<组>/F###.png    标准化后的单帧画面（默认 1920x1080，带页眉，可直接分发）
  <组>.mp4                视频（装了 ffmpeg 才有）
  <组>.gif                动图（没 ffmpeg 时的兜底；--gif 可强制生成）
  index_video.csv         清单：帧号 / 时间戳 / 停留秒数 / 画面文件

例
  python3 make_fig4_video.py --mode pair \
      --ds ~/dataset/slam_stereo_pairs_0_99_20260929 \
      --out ~/ORB_SLAM3/outputs/fig4_4seasons/video \
      --groups fixed_pixel,world_plane --cams cam0 --hold 1.5
"""

from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

try:
    import numpy as np
except Exception:
    np = None

from PIL import Image, ImageChops, ImageDraw, ImageFont

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

CLEAN_COLOR = (0, 150, 0)
DIFF_COLOR = (205, 0, 0)
INK = (20, 20, 20)
SOFT = (110, 110, 110)
RULE = (185, 185, 185)

ATTACK_LABEL = {
    "fixed_pixel": "Fixed-pixel overlay attack",
    "world_plane": "World-plane projection attack",
}

FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "C:/Windows/Fonts/msyhbd.ttc",
    "C:/Windows/Fonts/arialbd.ttf",
]


def load_font(size):
    for p in FONT_CANDIDATES:
        if os.path.isfile(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    return ImageFont.load_default()


def even(n):
    return n + (n % 2)


def fit_into(im, bw, bh):
    r = min(bw / float(im.width), bh / float(im.height))
    return im.resize((max(1, int(im.width * r)), max(1, int(im.height * r))), Image.LANCZOS)


def paste_center(canvas, im, box):
    x0, y0, x1, y1 = box
    canvas.paste(im, (int(x0 + (x1 - x0 - im.width) / 2.0),
                      int(y0 + (y1 - y0 - im.height) / 2.0)))


def diff_mask(a, b, thresh=12):
    """返回 (bbox, 差异灰度图)。b 相对 a 的改动区。"""
    d = ImageChops.difference(a.convert("RGB"), b.convert("RGB")).convert("L")
    if np is not None:
        arr = np.asarray(d, dtype=np.uint8)
        ys, xs = np.nonzero(arr > thresh)
        bbox = None if xs.size == 0 else (int(xs.min()), int(ys.min()),
                                         int(xs.max()) + 1, int(ys.max()) + 1)
        return bbox, d
    bw = d.point(lambda v: 255 if v > thresh else 0)
    return bw.getbbox(), d


def draw_rect(im, bbox, color=DIFF_COLOR, lw=3):
    if not bbox:
        return
    d = ImageDraw.Draw(im)
    x0, y0, x1, y1 = bbox
    x0 -= 2; y0 -= 2; x1 += 2; y1 += 2
    for k in range(lw):
        d.rectangle([x0 - k, y0 - k, x1 + k, y1 + k], outline=color)


def page_header(canvas, fonts, left, center, right, y=18, rule_y=64):
    d = ImageDraw.Draw(canvas)
    d.text((28, y), left, font=fonts["h1"], fill=INK)
    if center:
        w = d.textlength(center, font=fonts["h2"])
        d.text(((canvas.width - w) / 2.0, y + 8), center, font=fonts["h2"], fill=INK)
    if right:
        w = d.textlength(right, font=fonts["h2"])
        d.text((canvas.width - 28 - w, y + 8), right, font=fonts["h2"], fill=SOFT)
    d.line([(0, rule_y), (canvas.width, rule_y)], fill=RULE, width=2)


def column_titles(canvas, fonts, boxes, titles, y):
    d = ImageDraw.Draw(canvas)
    for (x0, _, x1, _), (txt, col) in zip(boxes, titles):
        if not txt:
            continue
        w = d.textlength(txt, font=fonts["cap"])
        d.text((x0 + (x1 - x0 - w) / 2.0, y), txt, font=fonts["cap"], fill=col)


def slide_pair(rec, ds, group, cam, args, fonts):
    """pair 模式的一帧：clean | 干扰（红框）[ | 差异 x6]"""
    side = "left" if cam == "cam0" else "right"
    p_clean = os.path.join(ds, rec["clean_" + side])
    p_mod = os.path.join(ds, rec["%s_%s" % (group, side)])
    if not (os.path.isfile(p_clean) and os.path.isfile(p_mod)):
        raise FileNotFoundError("%s | %s" % (p_clean, p_mod))

    a = Image.open(p_clean).convert("RGB")
    b = Image.open(p_mod).convert("RGB")
    bbox, dmap = diff_mask(a, b)

    ncol = 3 if args.diff_col else 2
    left_m, right_m, gap = 26, 26, 18
    title_h = 26
    cw = (args.width - left_m - right_m - gap * (ncol - 1)) / float(ncol)
    img_h = int(round(cw * a.height / float(a.width)))
    title_y, top = 74, 100
    bottom = top + img_h
    H = args.height if args.height > 0 else (bottom + 46 + 12)
    W, H = even(args.width), even(H)
    canvas = Image.new("RGB", (W, H), (255, 255, 255))
    page_header(canvas, fonts,
                "F%03d" % rec["idx"],
                "(a) Normal Scenario (no attack)     |     (a') %s"
                % ATTACK_LABEL.get(group, group),
                "%s / %s" % (group, cam))

    boxes = []
    for k in range(ncol):
        x0 = left_m + k * (cw + gap)
        boxes.append((x0, top, x0 + cw, bottom))

    titles = [("clean (original)", CLEAN_COLOR), ("%s (modified)" % group, DIFF_COLOR)]
    if ncol == 3:
        titles.append(("difference x6", SOFT))
    column_titles(canvas, fonts, boxes, titles, title_y)

    for k in range(ncol):
        paste_center(canvas, fit_into(a if k == 0 else b, boxes[k][2] - boxes[k][0],
                                      boxes[k][3] - boxes[k][1]), boxes[k])

    # 红框画在"干扰图"上（缩放到目标尺寸后按比例映射）
    box = boxes[1]
    sc = fit_into(b, box[2] - box[0], box[3] - box[1])
    off = (int(box[0] + (box[2] - box[0] - sc.width) / 2.0),
           int(box[1] + (box[3] - box[1] - sc.height) / 2.0))
    if bbox:
        r = sc.width / float(b.width)
        rb = (off[0] + bbox[0] * r, off[1] + bbox[1] * r,
              off[0] + bbox[2] * r, off[1] + bbox[3] * r)
        draw_rect(canvas, rb, DIFF_COLOR, 3)

    if ncol == 3:
        if np is not None:
            amp = np.asarray(dmap, dtype=np.float32) * 6.0
            amp = np.clip(amp, 0, 255).astype(np.uint8)
            dimg = Image.fromarray(amp, mode="L").convert("RGB")
        else:
            dimg = dmap.point(lambda v: min(255, v * 6)).convert("RGB")
        paste_center(canvas, fit_into(dimg, boxes[2][2] - boxes[2][0],
                                      boxes[2][3] - boxes[2][1]), boxes[2])

    d = ImageDraw.Draw(canvas)
    d.text((28, H - 28), "t = %.3f s   |   %s %s   |   hold %.2fs"
           % (rec["ts"], group, cam, args.hold), font=fonts["foot"], fill=SOFT)
    pct = ""
    if bbox:
        pct = "changed %d px (%.2f%%)  bbox %d,%d-%d,%d" % (
            (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]),
            100.0 * (bbox[2] - bbox[0]) * (bbox[3] - bbox[1]) / float(a.width * a.height),
            bbox[0], bbox[1], bbox[2], bbox[3])
    w = d.textlength(pct, font=fonts["foot"])
    d.text((args.width - 28 - w, H - 28), pct, font=fonts["foot"], fill=DIFF_COLOR)

    out = os.path.join(args.out, "slides", group, "F%03d.png" % rec["idx"])
    os.makedirs(os.path.dirname(out), exist_ok=True)
    canvas.save(out)
    return out

# --------------------------------------------------------------------- fig4
def render_panel(py, panel_py, frames_dir, tag, out_png, anchor, gly, log):
    kf = os.path.join(frames_dir, tag + "_kf_traj.txt")
    if not os.path.isfile(kf):
        return False
    pts = os.path.join(frames_dir, tag + "_map_points.csv")
    cam = os.path.join(frames_dir, tag + "_cam_traj.txt")
    cmd = [py, panel_py, "--traj", kf, "--menu", "none", "--ref-mode", "frustum",
           "--ref-fov", "63", "--fit", "all", "--width", "1200", "--height", "900",
           "--glyph-every", str(gly[0]), "--glyph-depth", str(gly[1]),
           "--glyph-halfw", str(gly[2]), "--cam-scale", str(gly[3]),
           "--out", out_png, "--anchor-out", anchor]
    if os.path.isfile(pts):
        cmd += ["--points", pts]
    if os.path.isfile(cam):
        cmd += ["--cam", cam]
    with open(log, "ab") as lg:
        r = subprocess.run(cmd, stdout=lg, stderr=lg)
    return r.returncode == 0 and os.path.isfile(out_png)


def placeholder_panel(path, w, h, text):
    im = Image.new("RGB", (w, h), (255, 255, 255))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, w - 1, h - 1], outline=(185, 185, 185), width=2)
    f = load_font(26)
    tw = d.textlength(text, font=f)
    d.text(((w - tw) / 2.0, h / 2.0 - 14), text, font=f, fill=(150, 150, 150))
    im.save(path)


def compose_row(py, compose_py, row, labels, anns, anchors, out_prefix, dpi, log):
    cmd = [py, compose_py, "--row", row]
    for v in labels:
        cmd += ["--cell-label", v]
    for v in anns:
        cmd += ["--cell-ann", v]
    for v in anchors:
        if v:
            cmd += ["--cell-anchor-file", v]
    cmd += ["--ann-inset", "0.035", "--no-cell-title", "--height", "2.4",
            "--fontsize", "11", "--dpi", str(dpi), "--no-pdf", "--out", out_prefix]
    with open(log, "ab") as lg:
        r = subprocess.run(cmd, stdout=lg, stderr=lg)
    return r.returncode == 0 and os.path.isfile(out_prefix + ".png")


def finalize(raw_png, out_png, args, fonts, left, center, right, foot_l, foot_r):
    """把一张原始拼版图放进 1920x1080 白底画布，加页眉页脚。"""
    content_w = args.width - 44
    raw, raw_h = None, 420
    try:
        raw = Image.open(raw_png).convert("RGB")
        raw_h = max(120, int(round(content_w * raw.height / float(raw.width))))
    except Exception as exc:
        print("      [!] 读不了 %s: %s" % (raw_png, exc))
    top = 100
    H = args.height if args.height > 0 else (top + raw_h + 46 + 12)
    W, H = even(args.width), even(H)
    canvas = Image.new("RGB", (W, H), (255, 255, 255))
    page_header(canvas, fonts, left, center, right)
    box = (22, top, W - 22, top + raw_h)
    if raw is None:
        d0 = ImageDraw.Draw(canvas)
        d0.text((30, top + 40), "render failed: %s" % raw_png, font=fonts["h2"], fill=DIFF_COLOR)
    else:
        paste_center(canvas, fit_into(raw, box[2] - box[0], box[3] - box[1]), box)
    d = ImageDraw.Draw(canvas)
    d.line([(0, H - 34), (canvas.width, H - 34)], fill=RULE, width=2)
    d.text((28, H - 27), foot_l, font=fonts["foot"], fill=SOFT)
    if foot_r:
        w = d.textlength(foot_r, font=fonts["foot"])
        d.text((canvas.width - 28 - w, H - 27), foot_r, font=fonts["foot"], fill=SOFT)
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    canvas.save(out_png)
    return out_png


def slide_fig4(rec, group, cam, args, fonts):
    tag = "F%03d" % rec["idx"]
    gframes = os.path.join(args.runs, group, "frames")
    cframes = os.path.join(args.runs, "clean", "frames")
    tmp = os.path.join(args.out, "_raw")
    os.makedirs(tmp, exist_ok=True)
    log = os.path.join(args.out, "render.log")

    p_clean = os.path.join(tmp, "clean_%s_panel.png" % tag)
    p_g = os.path.join(tmp, "%s_%s_panel.png" % (group, tag))
    a_clean = os.path.join(tmp, "clean_%s.anchor.json" % tag)
    a_g = os.path.join(tmp, "%s_%s.anchor.json" % (group, tag))

    okc = render_panel(args.py, args.panel_py, cframes, tag, p_clean, a_clean, args.glyph, log)
    okg = render_panel(args.py, args.panel_py, gframes, tag, p_g, a_g, args.glyph, log)
    if not okc:
        placeholder_panel(p_clean, 1200, 900, "clean: no map snapshot at %s" % tag)
        a_clean = ""
    if not okg:
        placeholder_panel(p_g, 1200, 900, "%s: no map snapshot at %s" % (group, tag))
        a_g = ""

    f_clean = os.path.join(cframes, tag + "_current_frame.png")
    f_g = os.path.join(gframes, tag + "_current_frame.png")

    if not (os.path.isfile(f_clean) and os.path.isfile(f_g)):
        raise FileNotFoundError("缺数据帧快照 %s | %s（先跑 run_video.sh --slam）"
                                % (f_clean, f_g))

    row = "%s,%s,%s|%s,%s,%s" % ("Normal Scenario", p_clean, f_clean,
                                 "Attack Scenario", p_g, f_g)
    labels = ["(a) Normal Scenario (no attack)",
              "(a') %s" % ATTACK_LABEL.get(group, group)]
    anns = ["Normal Scenario", "Attack Scenario"]
    raw_prefix = os.path.join(tmp, "%s_%s_row" % (group, tag))
    if not compose_row(args.py, args.compose_py, row, labels, anns, [a_clean, a_g],
                       raw_prefix, args.fig4_dpi, log):
        raise RuntimeError("compose_compare.py 失败，见 %s" % log)

    out = os.path.join(args.out, "slides", group, "F%03d.png" % rec["idx"])
    res = finalize(raw_prefix + ".png", out, args, fonts,
                    "F%03d" % rec["idx"],
                    "(a) Normal Scenario (no attack)     |     (a') %s"
                    % ATTACK_LABEL.get(group, group),
                    "%s / %s" % (group, cam),
                   "t = %.3f s   |   hold %.2fs" % (rec["ts"], args.hold),
                   "")
    if not args.keep_raw:
        for tmp_f in (p_clean, p_g, a_clean, a_g, raw_prefix + ".png"):
            try:
                if tmp_f and os.path.isfile(tmp_f):
                    os.remove(tmp_f)
            except OSError:
                pass
    return res



def slide_fig4_triple(rec, att_groups, cam, args, fonts):
    """fig4 版式的三板块版：一帧里放 (a) 正常 + (a1)/(a2) 各个干扰组。

    每个板块 = [地图面板 | 当前帧]，和单组版一样，只是横着排三段。
    """
    tag = "F%03d" % rec["idx"]
    tmp = os.path.join(args.out, "_raw")
    os.makedirs(tmp, exist_ok=True)
    log = os.path.join(args.out, "render.log")
    marks = ["(a')", "(a'')", "(a" + "'" * 3 + ")"]

    plan = [("clean", "(a) Normal Scenario (no attack)", "Normal Scenario")]
    for k, g in enumerate(att_groups):
        plan.append((g, "%s %s" % (marks[min(k, len(marks) - 1)],
                                   ATTACK_LABEL.get(g, g)), "Attack Scenario"))

    parts, labels, anns, anchors, tmp_files = [], [], [], [], []
    for name, lab, ann in plan:
        frames = os.path.join(args.runs, name, "frames")
        p_panel = os.path.join(tmp, "%s_%s_panel.png" % (name, tag))
        a_panel = os.path.join(tmp, "%s_%s.anchor.json" % (name, tag))
        if not render_panel(args.py, args.panel_py, frames, tag, p_panel, a_panel,
                            args.glyph, log):
            placeholder_panel(p_panel, 1200, 900,
                              "%s: no map snapshot at %s" % (name, tag))
            a_panel = ""
        f_cur = os.path.join(frames, tag + "_current_frame.png")
        if not os.path.isfile(f_cur):
            raise FileNotFoundError("缺当前帧图片 %s（先跑 run_video.sh --slam）" % f_cur)
        parts.append("%s,%s,%s" % (ann, p_panel, f_cur))
        labels.append(lab)
        anns.append(ann)
        anchors.append(a_panel)
        tmp_files += [p_panel, a_panel]

    raw_prefix = os.path.join(tmp, "triple_%s_row" % tag)
    if not compose_row(args.py, args.compose_py, "|".join(parts), labels, anns,
                       anchors, raw_prefix, args.fig4_dpi, log):
        raise RuntimeError("compose_compare.py 失败，见 %s" % log)

    head = ["(a) Normal"]
    for k, g in enumerate(att_groups):
        head.append("%s %s" % (marks[min(k, len(marks) - 1)],
                               ATTACK_LABEL.get(g, g).split()[0]))
    out = os.path.join(args.out, "slides", "triple", tag + ".png")
    res = finalize(raw_prefix + ".png", out, args, fonts,
                   tag,
                   "  |  ".join(head),
                   "triple / %s" % cam,
                   "t = %.3f s   |   hold %.2fs" % (rec["ts"], args.hold),
                   "")
    if not args.keep_raw:
        for tmp_f in tmp_files + [raw_prefix + ".png"]:
            try:
                if tmp_f and os.path.isfile(tmp_f):
                    os.remove(tmp_f)
            except OSError:
                pass
    return res

# ------------------------------------------------------------------- encode
def write_concat(slides, hold, path):
    with open(path, "w", encoding="utf-8") as fh:
        for i, s in enumerate(slides):
            fh.write("file '%s'\n" % s.replace("'", "'\\''"))
            if i < len(slides) - 1:
                fh.write("duration %.3f\n" % hold)
        fh.write("file '%s'\n" % slides[-1].replace("'", "'\\''"))


def encode_mp4(slides, out_mp4, hold, fps):
    """把每帧图片按 hold 秒拼成 mp4。试三种写法，都不行就返回 False（外层改出 GIF）。"""
    ff = shutil.which("ffmpeg")
    if not ff:
        return False
    lst = os.path.splitext(out_mp4)[0] + ".concat.txt"
    write_concat(slides, hold, lst)
    base = [ff, "-y", "-hide_banner", "-loglevel", "error",
            "-f", "concat", "-safe", "0", "-i", lst]
    tail = ["-c:v", "libx264", "-preset", "medium", "-crf", "18",
            "-movflags", "+faststart", out_mp4]
    evenf = "scale=trunc(iw/2)*2:trunc(ih/2)*2"
    tries = [
        ["-vf", "fps=%d,%s,format=yuv420p" % (fps, evenf)],
        ["-r", str(fps), "-vf", "%s,format=yuv420p" % evenf],
        ["-r", str(fps)],
    ]
    for i, extra in enumerate(tries):
        if os.path.isfile(out_mp4):
            try:
                os.remove(out_mp4)
            except OSError:
                pass
        r = subprocess.run(base + extra + tail)
        if r.returncode == 0 and os.path.isfile(out_mp4) and os.path.getsize(out_mp4) > 0:
            return True
        print("    [!] ffmpeg 第 %d 种写法没成功（rc=%s），换一种" % (i + 1, r.returncode))
    return False


def encode_gif(slides, out_gif, hold, width):
    frames = []
    for s in slides:
        im = Image.open(s).convert("RGB")
        if im.width > width:
            im = im.resize((width, int(im.height * width / float(im.width))), Image.LANCZOS)
        frames.append(im.convert("P", palette=Image.ADAPTIVE, colors=256))
    if not frames:
        return False
    frames[0].save(out_gif, save_all=True, append_images=frames[1:],
                   duration=int(round(hold * 1000)), loop=0, optimize=True)
    return os.path.isfile(out_gif)


def human(nbytes):
    for u in ("B", "KB", "MB", "GB"):
        if nbytes < 1024 or u == "GB":
            return "%.1f %s" % (nbytes, u)
        nbytes /= 1024.0

# --------------------------------------------------------------------- main
def read_pairs(ds):
    path = os.path.join(ds, "comparison_pairs.csv")
    if not os.path.isfile(path):
        raise SystemExit("[x] 缺 %s" % path)
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            r["idx"] = int(r["frame_index"])
            r["ts"] = float(r["timestamp_seconds"])
            r["mod"] = int(r.get("modified_in_variants") or 0)
            rows.append(r)
    return rows


def build_parser():
    ap = argparse.ArgumentParser(
        description="逐帧 (a)正常 vs (a')干扰 对照 -> 视频",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="pair", choices=["pair", "fig4"])
    ap.add_argument("--ds", required=True, help="数据集根目录")
    ap.add_argument("--out", required=True, help="输出目录")
    ap.add_argument("--runs", default="", help="fig4 模式：~/ORB_SLAM3/runs/4seasons")
    ap.add_argument("--groups", default="fixed_pixel,world_plane")
    ap.add_argument("--cams", default="cam0")
    ap.add_argument("--frames", default="", help="只做这些帧号，如 8,9,10")
    ap.add_argument("--all-frames", action="store_true", help="连没改动的帧也做")
    ap.add_argument("--max", type=int, default=0, help="只做前 N 帧")
    ap.add_argument("--hold", type=float, default=1.5, help="每帧停留秒数")
    ap.add_argument("--fps", type=int, default=30, help="视频帧率")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=0,
                    help="0=按内容自动（推荐，画面最大）；1080 强制 16:9")
    ap.add_argument("--diff-col", action="store_true", help="pair 模式加一列差异放大图")
    ap.add_argument("--jobs", type=int, default=4, help="并行渲染线程数")
    ap.add_argument("--fig4-dpi", type=int, default=150, help="fig4 模式 compose_compare.py 的 dpi")
    ap.add_argument("--panel-py", default="", help="pangolin_panel2.py 路径")
    ap.add_argument("--compose-py", default="", help="compose_compare.py 路径")
    ap.add_argument("--glyph-every", type=int, default=2)
    ap.add_argument("--glyph-depth", type=float, default=0.012)
    ap.add_argument("--glyph-halfw", type=float, default=0.007)
    ap.add_argument("--cam-scale", type=float, default=1.4)
    ap.add_argument("--triple", action="store_true",
                    help="fig4 模式：一帧里放 3 个板块（正常 + 两个干扰），输出单个视频 triple.mp4")
    ap.add_argument("--combined", action="store_true",
                    help="除了每组一个视频，再把所有组的帧合成一条 all.mp4")
    ap.add_argument("--keep-raw", action="store_true",
                    help="保留 _raw/ 里的中间面板与拼版图（默认出完就删，省几百 MB）")
    ap.add_argument("--no-ffmpeg", action="store_true", help="不调 ffmpeg（只出 GIF）")
    ap.add_argument("--gif", action="store_true", help="额外/强制生成 GIF")
    ap.add_argument("--gif-width", type=int, default=1280)
    ap.add_argument("--py", default=sys.executable or "python3")
    return ap


def main():
    args = build_parser().parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    if not args.panel_py:
        cand = [os.path.join(os.path.expanduser("~"), "ORB_SLAM3", "outputs",
                             "fig4_normal", "pangolin_panel2.py"),
                os.path.join(os.path.expanduser("~"), "ORB_SLAM3", "outputs",
                             "fig4_4seasons", "pangolin_panel2.py"),
                os.path.join(here, "pangolin_panel2.py")]
        args.panel_py = next((c for c in cand if os.path.isfile(c)), cand[0])
    if not args.compose_py:
        args.compose_py = os.path.join(here, "compose_compare.py")
    args.glyph = (args.glyph_every, args.glyph_depth, args.glyph_halfw, args.cam_scale)
    args.ds = os.path.expanduser(args.ds)
    args.out = os.path.expanduser(args.out)
    args.runs = os.path.expanduser(args.runs)

    needs = [(args.compose_py, "compose_compare.py")] if args.mode == "pair" else [
        (args.panel_py, "pangolin_panel2.py"), (args.compose_py, "compose_compare.py")]
    for need, what in needs:
        if not os.path.isfile(need):
            raise SystemExit("[x] 找不到 %s: %s" % (what, need))

    groups = [g for g in args.groups.split(",") if g]
    cams = [c for c in args.cams.split(",") if c]
    triple_groups = list(groups)
    if args.triple:
        if args.mode != "fig4":
            raise SystemExit("[x] --triple 只在 fig4 模式有效")
        if len(groups) < 2:
            raise SystemExit("[x] --triple 需要至少两个干扰组，如 --groups fixed_pixel,world_plane")
        groups = ["triple"]
        if args.width == 1920:
            args.width = 2560          # 一帧 3 板块，默认加宽才看得清
    if args.mode == "fig4" and "cam0" not in cams:
        raise SystemExit("[x] fig4 模式的逐帧快照只有左目(cam0)，请用 --cams cam0")

    rows = read_pairs(args.ds)
    total = len(rows)
    if not args.all_frames:
        rows = [r for r in rows if r["mod"] == 1]
    if args.frames:
        want = set(int(x) for x in args.frames.split(","))
        rows = [r for r in rows if r["idx"] in want]
    if args.max > 0:
        rows = rows[:args.max]
    if not rows:
        raise SystemExit("[x] 没有选到任何帧")

    os.makedirs(args.out, exist_ok=True)
    fonts = {"h1": load_font(34), "h2": load_font(20),
             "cap": load_font(19), "foot": load_font(17)}

    print("[i] 模式      : %s" % args.mode)
    print("[i] 数据集    : %s" % args.ds)
    print("[i] 输出      : %s" % args.out)
    print("[i] 帧        : %d 帧（总表 %d 帧，改动帧 %d）"
          % (len(rows), total, sum(1 for r in read_pairs(args.ds) if r["mod"] == 1)))
    print("[i] 停留      : %.2f s/帧   fps=%d   画布宽=%d 高=%s"
          % (args.hold, args.fps, args.width,
             "自动" if args.height <= 0 else str(args.height)))
    print("[i] ffmpeg    : %s" % (shutil.which("ffmpeg") or "没装（只能出 GIF，"
          "装一下：sudo apt-get install -y ffmpeg）"))

    index_rows = []
    all_slides = []
    for group in groups:
        print("\n──── %s ────" % group)
        slides = [None] * len(rows)

        def work(k):
            rec = rows[k]
            cam = cams[0]
            if args.mode == "pair":
                return slide_pair(rec, args.ds, group, cam, args, fonts)
            if args.triple:
                return slide_fig4_triple(rec, triple_groups, cam, args, fonts)
            return slide_fig4(rec, group, cam, args, fonts)

        errs = []
        with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as ex:
            futs = {ex.submit(work, k): k for k in range(len(rows))}
            done = 0
            for fut, k in futs.items():
                try:
                    slides[k] = fut.result()
                except Exception as exc:
                    errs.append((rows[k]["idx"], exc))
                    slides[k] = None
                done += 1
                if done % 10 == 0 or done == len(rows):
                    print("      %d/%d" % (done, len(rows)))

        if errs:
            for idx, exc in errs[:5]:
                print("      [!] F%03d 失败: %s" % (idx, exc))
            if len(errs) > 5:
                print("      [!] 还有 %d 帧失败，未逐条列出" % (len(errs) - 5))
        slides = [s for s in slides if s]
        print("      [i] 画面 %d/%d 张（%s 模式，%s）"
              % (len(slides), len(rows), args.mode,
                 "含未改动帧" if args.all_frames else "只做被改动的帧"))
        if len(slides) < len(rows):
            print("      [!] 少了 %d 张；对照 index_video.csv 和 render.log 查原因"
                  % (len(rows) - len(slides)))
        if not slides:
            print("      [x] %s 一张都没生成，跳过" % group)
            continue
        all_slides += slides

        stem = os.path.join(args.out, group)
        made = []
        if not args.no_ffmpeg:
            if encode_mp4(slides, stem + ".mp4", args.hold, args.fps):
                made.append(stem + ".mp4")
        if args.gif or not made:
            if encode_gif(slides, stem + ".gif", args.hold, args.gif_width):
                made.append(stem + ".gif")
        for m in made:
            print("      [i] 写出 %s  (%s)" % (m, human(os.path.getsize(m))))
        if not made:
            print("      [!] 没生成视频（ffmpeg 缺且 GIF 失败）；PNG 画面在 slides/%s/" % group)

        for s in slides:
            idx = int(os.path.basename(s)[1:4])
            rec = next(r for r in rows if r["idx"] == idx)
            index_rows.append((group, cams[0], idx, rec["ts"], args.hold, s))

    if args.combined and len(all_slides) > 1 and len(groups) > 1:
        stem = os.path.join(args.out, "all")
        print("\n──── 合并：%d 张画面 -> all ────" % len(all_slides))
        made = []
        if not args.no_ffmpeg and encode_mp4(all_slides, stem + ".mp4", args.hold, args.fps):
            made.append(stem + ".mp4")
        if args.gif or not made:
            if encode_gif(all_slides, stem + ".gif", args.hold, args.gif_width):
                made.append(stem + ".gif")
        for m in made:
            print("      [i] 写出 %s  (%s)" % (m, human(os.path.getsize(m))))

    csv_path = os.path.join(args.out, "index_video.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["group", "cam", "frame_index", "timestamp_seconds",
                    "hold_seconds", "slide"])
        w.writerows(index_rows)

    print("\n======== 完成 ========")
    print("单帧画面 : %s/slides/<组>/F###.png（%d 张）" % (args.out, len(index_rows)))
    print("视频     : %s/<组>.mp4  /  <组>.gif" % args.out)
    print("清单     : %s" % csv_path)


if __name__ == "__main__":
    main()