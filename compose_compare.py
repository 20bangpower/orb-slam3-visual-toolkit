#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compose_compare.py -- 一个图里并排放 N 个子图，每个子图 = [左=全局地图面板 | 右=当前数据帧]

论文 Fig.4 的版式：
    子图 (a)  = 正常输入        [地图面板 | 数据帧]
    子图 (a') = 同一段被干扰后   [地图面板 | 数据帧]
    两个子图并排放在同一张图里，各带一条居中标题（在面板下方）。

每个 cell 的写法：  "子图标题,地图面板.png,数据帧.png"
多个 cell 用 | 隔开；多行就重复 --row（配合 --cell-label 按行优先顺序给标题）。

例：
    python3 compose_compare.py \
      --row "(a) Normal Scenario (no attack),clean/panel.png,clean/frame.png|(a') Fixed-pixel overlay attack,att/panel.png,att/frame.png" \
      --cell-anchor-file clean/panel.anchor.json --cell-anchor-file att/panel.anchor.json \
      --out fig4_fixed_pixel
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import Rectangle

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass


# ------------------------------------------------------------------ 字体
CJK_CANDIDATES = [
    "Microsoft YaHei", "Microsoft YaHei UI", "SimHei", "Noto Sans CJK SC",
    "Source Han Sans SC", "WenQuanYi Zen Hei", "PingFang SC", "Heiti SC",
    "Arial Unicode MS",
]


def setup_fonts(want_cjk: bool) -> bool:
    if not want_cjk:
        return False
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in CJK_CANDIDATES:
        if name in have:
            plt.rcParams["font.sans-serif"] = [name] + plt.rcParams["font.sans-serif"]
            plt.rcParams["axes.unicode_minus"] = False
            print("[i] 中文字体: %s" % name)
            return True
    print("[!] 没找到中文字体，中文会显示成方块；用英文标题或先装 fonts-noto-cjk")
    return False


# -------------------------------------------------------------------- I/O
def load_rgb(path):
    img = plt.imread(path)
    if img.ndim == 2:
        img = np.dstack([img] * 3)
    if img.shape[2] == 4:
        a = img[..., 3:4]
        img = img[..., :3] * a + (1.0 - a)
    return np.clip(np.asarray(img, float), 0.0, 1.0)


def parse_cell(spec):
    parts = [p.strip() for p in spec.split(",")]
    if len(parts) < 2:
        raise SystemExit('[x] cell 写成 "标题,地图.png,帧.png": %r' % spec)
    name = parts[0]
    mp = parts[1]
    fp = parts[2] if len(parts) > 2 and parts[2] else None
    return name, mp, fp


def parse_row(spec):
    cells = [c for c in (c.strip() for c in spec.split("|")) if c]
    if not cells:
        raise SystemExit("[x] --row 是空的")
    return [parse_cell(c) for c in cells]


# --------------------------------------------------- 锚点（Current Position 框）
def blob_from_anchor_file(path):
    """读 pangolin_panel2.py --anchor-out 写的 json -> (cx,cy,x0,y0,x1,y1)。"""
    import json
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as fh:
            d = json.load(fh)
    except Exception as exc:
        print("[!] 读不了 anchor 文件 %s: %s" % (path, exc))
        return None
    try:
        return (float(d["cx"]), float(d["cy"]), float(d["x0"]), float(d["y0"]),
                float(d["x1"]), float(d["y1"]))
    except KeyError:
        return None


def find_color_blob(img, what, menu_frac=0.0):
    """在面板图上找某种颜色的连通像素 -> 轴内 0~1 坐标 (cx,cy,x0,y0,x1,y1)。"""
    if img is None:
        return None
    a = np.asarray(img, float)[..., :3]
    if a.max() > 1.5:
        a = a / 255.0
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    masks = {
        "green": (g > .35) & (g - r > .15) & (g - b > .15),
        "blue": (b > .35) & (b - r > .20) & (b - g > .20),
        "magenta": (r > .55) & (b > .55) & (g < .55),
        "red": (r > .45) & (r - g > .20) & (r - b > .20),
        "yellow": (r > .5) & (g > .5) & (b < .4),
        "black": (r < .25) & (g < .25) & (b < .25),
    }
    m = masks.get(what)
    if m is None:
        return None
    H, W = m.shape
    if menu_frac > 0:
        m = m.copy()
        m[:, :int(menu_frac * W)] = False
    ys, xs = np.nonzero(m)
    if len(xs) == 0:
        return None
    return (xs.mean() / W, 1.0 - ys.mean() / H,
            xs.min() / W, 1.0 - ys.max() / H, xs.max() / W, 1.0 - ys.min() / H)


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(description="并排拼 N 个子图，每个子图 = 地图面板 + 数据帧")
    ap.add_argument("--row", action="append", required=True,
                    help='"标题,map.png,frame.png|标题2,map2.png,frame2.png"')
    ap.add_argument("--row-label", action="append", default=[],
                    help="整行的居中大标题（可选，论文版式一般不需要）")
    ap.add_argument("--out", required=True, help="输出前缀（会写 .png 和 .pdf）")
    ap.add_argument("--no-pdf", action="store_true",
                    help="只写 .png 不写 .pdf（做视频帧时用，省时间省磁盘）")
    ap.add_argument("--height", type=float, default=2.4, help="每行内容高（英寸）")
    ap.add_argument("--gap", type=float, default=0.05, help="子图内 地图↔帧 间距（英寸）")
    ap.add_argument("--cell-gap", type=float, default=0.42, help="子图之间间距（英寸）")
    ap.add_argument("--map-aspect", default="auto", help="地图面板 宽/高；auto=按图算，或给数字")
    ap.add_argument("--frame-aspect", default="auto", help="数据帧 宽/高；auto=按图算，或给数字")
    ap.add_argument("--pad", type=float, default=0.10, help="整图四周留白（英寸）")
    ap.add_argument("--fontsize", type=float, default=11.0)
    ap.add_argument("--title-size", type=float, default=None, help="面板上方小标题字号")
    ap.add_argument("--celllabel-size", type=float, default=None, help="面板下方子图标题字号")
    ap.add_argument("--rowlabel-size", type=float, default=None)
    ap.add_argument("--no-cell-title", dest="cell_title", action="store_false", default=True,
                    help="不画面板上方的小标题（论文版式只有下方 (a) 标题）")
    ap.add_argument("--border", type=float, default=1.0, help="面板黑边框宽")
    ap.add_argument("--dpi", type=int, default=600)
    ap.add_argument("--no-cjk", action="store_true", help="不自动找中文字体")

    # ---- 每个子图的地图面板标注 ----
    ap.add_argument("--cell-label", action="append", default=[],
                    help='子图下方居中标题，按 cell 行优先顺序给，如 "(a) Normal Scenario (no attack)"')
    ap.add_argument("--cell-ann", action="append", default=[],
                    help='地图面板左上角蓝字，按 cell 顺序，如 "Normal Scenario"')
    ap.add_argument("--cell-ann2", action="append", default=[],
                    help='地图面板左下角品红字，按 cell 顺序（一般留空，用 --cell-anchor-file）')
    ap.add_argument("--ann-inset", type=float, default=0.035,
                    help="左上标注相对面板左边界的缩进（Pangolin 窗口截图带菜单栏时给 0.14）")
    ap.add_argument("--ann-top-y", type=float, default=0.955)
    ap.add_argument("--ann2-y", type=float, default=0.045)
    ap.add_argument("--ann-size", type=float, default=None)
    ap.add_argument("--ann-color", default="#2233CC")
    ap.add_argument("--ann2-color", default="#FF3FF3")

    # ---- 每个子图的 Current Position 框 + 标签（吃 pangolin_panel2.py 的 anchor.json）----
    ap.add_argument("--cell-anchor-file", action="append", default=[],
                    help="pangolin_panel2.py --anchor-out 写的 json（按 cell 顺序给）")
    ap.add_argument("--cell-anchor", action="append", default=[],
                    help='没有 json 时按颜色自动找：blue/green/magenta/red/yellow/black')
    ap.add_argument("--anchor-label", default="Current Position")
    ap.add_argument("--anchor-color", default="#FF3FF3")
    ap.add_argument("--anchor-box-color", default="#FF66E0")
    ap.add_argument("--anchor-lw", type=float, default=1.6)
    ap.add_argument("--anchor-size", type=float, default=None)
    ap.add_argument("--anchor-pad", type=float, default=0.012, help="方框相对锚点的外扩量")
    ap.add_argument("--anchor-side", default="auto",
                    choices=["auto", "top", "bottom", "left", "right"])
    ap.add_argument("--anchor-gap", type=float, default=0.035, help="文字离框的距离（轴内 0~1）")
    ap.add_argument("--anchor-dx", type=float, default=0.0)
    ap.add_argument("--anchor-dy", type=float, default=0.0)
    ap.add_argument("--anchor-ha", default="auto", choices=["auto", "center", "left", "right"])
    ap.add_argument("--anchor-menu-frac", type=float, default=0.0,
                    help="自动找颜色时忽略左侧菜单栏的宽度占比（窗口截图给 0.12）")
    args = ap.parse_args()

    setup_fonts(not args.no_cjk)

    rows = [parse_row(r) for r in args.row]
    labels = list(args.row_label)
    while len(labels) < len(rows):
        labels.append("")

    n_cells = max(len(r) for r in rows)
    H = args.height

    # 每个 cell 的宽度按第一行算，各行共用，保证列对齐
    map_ws, frame_ws = [], []
    for name, mp, fp in rows[0]:
        a = 1.0
        if args.map_aspect != "auto":
            a = float(args.map_aspect)
        elif os.path.isfile(mp):
            im = plt.imread(mp)
            a = im.shape[1] / float(im.shape[0])
        map_ws.append(a * H)
        if fp:
            b = 2.0
            if args.frame_aspect != "auto":
                b = float(args.frame_aspect)
            elif os.path.isfile(fp):
                im = plt.imread(fp)
                b = im.shape[1] / float(im.shape[0])
            frame_ws.append(b * H)
        else:
            frame_ws.append(0.0)

    cell_ws = [map_ws[i] + (args.gap + frame_ws[i] if frame_ws[i] > 0 else 0.0)
               for i in range(len(rows[0]))]
    cell_w = max(cell_ws) if cell_ws else 2 * H
    column_x = [args.pad + i * (cell_w + args.cell_gap) for i in range(n_cells)]

    fig_w = args.pad * 2 + n_cells * cell_w + (n_cells - 1) * args.cell_gap
    header_in = 0.34 if args.cell_title else 0.06
    label_in = 0.42
    row_h_in = header_in + H + label_in
    fig_h = args.pad * 2 + len(rows) * row_h_in

    fig = plt.figure(figsize=(fig_w, fig_h))
    ts = args.fontsize if args.title_size is None else args.title_size
    cls = args.fontsize + 1.5 if args.celllabel_size is None else args.celllabel_size
    ls = args.fontsize + 1.5 if args.rowlabel_size is None else args.rowlabel_size
    ann_size = args.fontsize + 2.0 if args.ann_size is None else args.ann_size
    a_size = args.fontsize + 1.0 if args.anchor_size is None else args.anchor_size

    n_anchor_done = 0
    for ri, cells in enumerate(rows):
        top_in = args.pad + ri * row_h_in
        y_content_top = 1.0 - (top_in + header_in) / fig_h
        y0 = y_content_top - H / fig_h

        for ci, (name, mp, fp) in enumerate(cells):
            k = ri * n_cells + ci
            x_in = column_x[ci]
            mw = map_ws[ci] if ci < len(map_ws) else H
            fw_pre = frame_ws[ci] if ci < len(frame_ws) else 2.0 * H

            if args.cell_title and name:
                fig.text((x_in + cell_w / 2.0) / fig_w,
                         1.0 - (top_in + header_in * 0.62) / fig_h,
                         name, ha="center", va="center", fontsize=ts,
                         fontweight="bold", color="black")

            # ---------------- 左：地图面板 ----------------
            aw = min(cell_w, mw) / fig_w
            axm = fig.add_axes([x_in / fig_w, y0, aw, H / fig_h])
            map_img = None
            if os.path.isfile(mp):
                map_img = load_rgb(mp)
                axm.imshow(map_img, aspect="auto")
            else:
                axm.text(0.5, 0.5, "missing:\n%s" % os.path.basename(mp),
                         ha="center", va="center", fontsize=7, color="red",
                         transform=axm.transAxes)
            axm.set_xticks([]); axm.set_yticks([])
            for sp in axm.spines.values():
                sp.set_linewidth(args.border); sp.set_color("black")

            # 左上角标注（论文里的 "Normal Scenario" / "Attack Scenario"）
            if k < len(args.cell_ann) and args.cell_ann[k]:
                axm.text(args.ann_inset, args.ann_top_y, args.cell_ann[k],
                         transform=axm.transAxes, ha="left", va="top",
                         fontsize=ann_size, fontweight="bold", color=args.ann_color,
                         path_effects=[pe.withStroke(linewidth=2.4, foreground="white")],
                         zorder=13)
            # 左下角标注（一般不用）
            if k < len(args.cell_ann2) and args.cell_ann2[k]:
                axm.text(args.ann_inset, args.ann2_y, args.cell_ann2[k],
                         transform=axm.transAxes, ha="left", va="bottom",
                         fontsize=ann_size, fontweight="bold", color=args.ann2_color,
                         path_effects=[pe.withStroke(linewidth=2.4, foreground="white")],
                         zorder=13)

            # Current Position：框 + 标签
            afile = args.cell_anchor_file[k] if k < len(args.cell_anchor_file) else None
            ablob = blob_from_anchor_file(afile)
            if ablob is None and k < len(args.cell_anchor) and args.cell_anchor[k]:
                ablob = find_color_blob(map_img, args.cell_anchor[k], args.anchor_menu_frac)
                if ablob is not None:
                    n_anchor_done += 1
            if ablob is not None:
                cx, cy, bx0, by0, bx1, by1 = ablob
                p = args.anchor_pad
                axm.add_patch(Rectangle((bx0 - p, by0 - p), (bx1 - bx0) + 2 * p, (by1 - by0) + 2 * p,
                                        transform=axm.transAxes, facecolor="none",
                                        edgecolor=args.anchor_box_color,
                                        linewidth=args.anchor_lw, zorder=12))
                side = args.anchor_side
                if side == "auto":
                    side = "left" if bx0 > 0.40 else "right"
                gap = args.anchor_gap
                if side == "top":
                    tx, ty, ha, va = cx + args.anchor_dx, min(0.985, by1 + p + gap), "center", "bottom"
                elif side == "bottom":
                    tx, ty, ha, va = cx + args.anchor_dx, max(0.015, by0 - p - gap), "center", "top"
                elif side == "left":
                    tx, ty, ha, va = max(0.015, bx0 - p - gap + args.anchor_dx), cy, "right", "center"
                else:
                    tx, ty, ha, va = min(0.985, bx1 + p + gap + args.anchor_dx), cy, "left", "center"
                ty += args.anchor_dy
                if args.anchor_ha != "auto":
                    ha = args.anchor_ha
                axm.text(tx, ty, args.anchor_label, transform=axm.transAxes,
                         color=args.anchor_color, fontsize=a_size, fontweight="bold",
                         ha=ha, va=va, zorder=13,
                         path_effects=[pe.withStroke(linewidth=2.4, foreground="white")])
                print("[i] cell(%d) 锚点 中心(%.3f,%.3f) 框(%.3f,%.3f)-(%.3f,%.3f)"
                      % (k, cx, cy, bx0, by0, bx1, by1))
            elif afile or (k < len(args.cell_anchor) and args.cell_anchor[k]):
                print("[!] cell(%d) 没找到锚点（%s / %s），跳过 Current Position 框"
                      % (k, afile, args.cell_anchor[k] if k < len(args.cell_anchor) else ""))

            # ---------------- 右：当前数据帧 ----------------
            if fp:
                fx = x_in + mw + args.gap
                axf = fig.add_axes([fx / fig_w, y0, fw_pre / fig_w, H / fig_h])
                if os.path.isfile(fp):
                    axf.imshow(load_rgb(fp), aspect="auto")
                else:
                    axf.text(0.5, 0.5, "missing:\n%s" % os.path.basename(fp),
                             ha="center", va="center", fontsize=7, color="red",
                             transform=axf.transAxes)
                axf.set_xticks([]); axf.set_yticks([])
                for sp in axf.spines.values():
                    sp.set_linewidth(args.border); sp.set_color("black")

            # ---------------- 子图标题（面板下方居中）----------------
            clab = args.cell_label[k] if k < len(args.cell_label) else ""
            if clab:
                width = mw + (args.gap + fw_pre if fp else 0.0)
                fig.text((x_in + width / 2.0) / fig_w, y0 - 0.10 / fig_h, clab,
                         ha="center", va="top", fontsize=cls, fontweight="bold")

        lab = labels[ri]
        if lab:
            fig.text((args.pad + (n_cells * cell_w + (n_cells - 1) * args.cell_gap) / 2.0) / fig_w,
                     y0 - 0.30 / fig_h, lab, ha="center", va="top",
                     fontsize=ls, fontweight="bold")

    fig.patch.set_facecolor("white")
    for ext in (("png",) if args.no_pdf else ("png", "pdf")):
        path = "%s.%s" % (args.out, ext)
        fig.savefig(path, dpi=args.dpi, facecolor="white")
        print("[i] 写出 %s" % path)
    print("[i] %d 行 x %d 列 / 画布 %.2f x %.2f in" % (len(rows), n_cells, fig_w, fig_h))


if __name__ == "__main__":
    main()