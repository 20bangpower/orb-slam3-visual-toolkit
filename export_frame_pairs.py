#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
export_frame_pairs.py -- 把数据集里【每一个被干扰的帧】和它的原图成对导出

不做 SLAM，只做数据整理，便于逐帧分析用：

  <out>/
    index.csv                        逐帧统计：被改了多少像素、改动区域的 bbox、平均差
    pairs/<group>/F008_cam0.png      单帧对照图（左=clean 原图，右=干扰后的图，红框=改动区域）
    pairs/<group>/F008_cam0_modified.png   只有被改后的那张图（原样拷出来）
    sheets/<group>_cam0_sheet01.png  一页 8 帧的对照表（滚动看/打印都方便）
    sheets/<group>_cam0_all.pdf      所有页合成一个 PDF
    frames/<group>/F008_cam0_modified.png  被改后的整图（和 pairs 里那份一样，按组放）
    README.txt

用法：
    python3 export_frame_pairs.py --ds <数据集根> --out <输出目录>
    python3 export_frame_pairs.py --ds <数据集根> --out <输出> --cams cam0     # 只看左目
    python3 export_frame_pairs.py --ds <数据集根> --out <输出> --diff-col      # 多一列"差异放大图"
    python3 export_frame_pairs.py --ds <数据集根> --out <输出> --all-frames    # 连没被改的帧也导
"""
from __future__ import annotations

import argparse
import csv
import os
import shutil
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass


CLEAN_COLOR = "#1A7F37"
DIFF_COLOR = "#D40000"

# cam0/cam1（目录名） <-> left/right（comparison_pairs.csv 里的列名）
CAM_FIELD = {"cam0": "left", "cam1": "right"}


def read_img(path):
    """读 PNG -> float32 (H,W,3) 0~1；读不了返回 None。"""
    if not path or not os.path.isfile(path):
        return None
    a = plt.imread(path)
    if a.ndim == 2:
        a = np.dstack([a] * 3)
    a = np.asarray(a, dtype=np.float32)
    if a.shape[2] == 4:
        a = a[..., :3]
    if a.max() > 1.5:
        a = a / 255.0
    return np.clip(a, 0.0, 1.0)


def diff_stats(a, b):
    """逐像素比较（PNG 是 8bit，量化到 0~255 再比）-> (改动数, 占比, bbox, 平均差)"""
    ai = np.rint(a * 255.0).astype(np.int16)
    bi = np.rint(b * 255.0).astype(np.int16)
    d = np.abs(ai - bi).max(axis=2)
    mask = d > 0
    n = int(mask.sum())
    H, W = mask.shape
    if n == 0:
        return 0, 0.0, None, 0.0
    ys, xs = np.nonzero(mask)
    bbox = (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()))
    return n, n / float(H * W), bbox, float(d.mean())


def load_rows(ds):
    """优先用根目录 comparison_pairs.csv（一帧一行，含三组路径）。"""
    cp = os.path.join(ds, "comparison_pairs.csv")
    if os.path.isfile(cp):
        with open(cp, encoding="utf-8", newline="") as fh:
            return list(csv.DictReader(fh)), "comparison_pairs.csv"
    return None, None


def load_rows_fallback(ds, groups):
    """没有 comparison_pairs.csv 时，用各组 pairs.csv 自己拼。"""
    per = {}
    for g in groups:
        p = os.path.join(ds, g, "pairs.csv")
        if not os.path.isfile(p):
            continue
        with open(p, encoding="utf-8", newline="") as fh:
            per[g] = list(csv.DictReader(fh))
    if not per:
        return None, None
    n = min(len(v) for v in per.values())
    rows = []
    keys = list(per)          # 只有真实存在的组才会进 per
    for i in range(n):
        r0 = per[keys[0]][i]
        row = {"frame_index": r0["frame_index"], "timestamp_seconds": r0["timestamp_seconds"],
               "modified_in_variants": r0.get("modified", "1")}
        for g, lst in per.items():
            row["%s_left" % g] = g + "/" + lst[i]["left"].replace("\\", "/")
            row["%s_right" % g] = g + "/" + lst[i]["right"].replace("\\", "/")
        rows.append(row)
    return rows, "各组的 pairs.csv"


def main():
    ap = argparse.ArgumentParser(description="逐帧导出 原图 vs 被干扰图 的对照")
    ap.add_argument("--ds", required=True, help="数据集根目录 (slam_stereo_pairs_0_99_20260929)")
    ap.add_argument("--out", required=True, help="输出目录")
    ap.add_argument("--groups", default="fixed_pixel,world_plane", help="要导哪些干扰组")
    ap.add_argument("--cams", default="cam0,cam1", help="要导哪些目：cam0(左) / cam1(右)")
    ap.add_argument("--rows", type=int, default=8, help="每页对照表几行（每行一帧）")
    ap.add_argument("--dpi", type=int, default=110, help="单帧对照图/对照表的 dpi")
    ap.add_argument("--max", type=int, default=0, help="只导前 N 帧（0=全部）")
    ap.add_argument("--all-frames", action="store_true", help="连没被改动的帧也导（默认只导被改的）")
    ap.add_argument("--diff-col", action="store_true", help="对照表里多一列：差异放大图")
    ap.add_argument("--no-pairs", action="store_true", help="不导单帧对照图")
    ap.add_argument("--no-sheets", action="store_true", help="不导对照表/PDF")
    ap.add_argument("--no-copy", action="store_true", help="不把被改后的整图拷出来")
    args = ap.parse_args()

    ds = os.path.abspath(os.path.expanduser(args.ds))
    out = os.path.abspath(os.path.expanduser(args.out))
    groups = [g.strip() for g in args.groups.split(",") if g.strip()]
    cams = [c.strip() for c in args.cams.split(",") if c.strip()]

    # 只保留数据集里真的有的干扰组：对面少一组也能正常出图
    keep = [g for g in groups if os.path.isdir(os.path.join(ds, g))]
    for g in groups:
        if g not in keep:
            print("[!] 数据集里没有 %s，跳过这一组" % g)
    groups = keep
    if not groups:
        print("[x] 一组干扰数据都没有（%s 下应有 <组> 目录）" % ds)
        return 1

    rows, src = load_rows(ds)
    if rows is None:
        rows, src = load_rows_fallback(ds, ["clean"] + groups)
    if not rows:
        print("[x] 在 %s 里找不到 comparison_pairs.csv，也找不到各组的 pairs.csv" % ds)
        return 1
    print("[i] 数据集: %s" % ds)
    print("[i] 帧表:   %s（%d 帧）" % (src, len(rows)))
    print("[i] 干扰组: %s   目: %s" % (",".join(groups), ",".join(cams)))

    os.makedirs(out, exist_ok=True)
    index_rows = []
    total_written = 0

    for g in groups:
        for cam in cams:
            field = CAM_FIELD.get(cam, cam)
            key_clean = "clean_%s" % field
            key_var = "%s_%s" % (g, field)
            if not rows[0].get(key_clean) or not rows[0].get(key_var):
                print("[!] %s 里没有 %s / %s 两列，跳过 %s/%s" % (src, key_clean, key_var, g, cam))
                continue

            sel = []
            for r in rows:
                mod = str(r.get("modified_in_variants", "1")).strip() in ("1", "true", "True")
                if mod or args.all_frames:
                    sel.append(r)
            if args.max and args.max > 0:
                sel = sel[:args.max]
            if not sel:
                print("[!] %s/%s 没有可导的帧" % (g, cam))
                continue

            d_pairs = os.path.join(out, "pairs", g)
            d_frames = os.path.join(out, "frames", g)
            d_sheets = os.path.join(out, "sheets")
            for d in (d_pairs, d_frames, d_sheets):
                os.makedirs(d, exist_ok=True)

            print("\n──── %s / %s：%d 帧 ────" % (g, cam, len(sel)))
            sheet_no = 0
            page = []          # 当前页收集 (idx, clean_img, var_img, stats)

            def flush_sheet(page, sheet_no):
                if not page or args.no_sheets:
                    return sheet_no
                sheet_no += 1
                n = len(page)
                ncol = 3 if args.diff_col else 2
                cell_w = 4.6 / ncol * 2
                fig_h = n * (cell_w * 0.5 + 0.30) + 0.5
                fig_w = cell_w * ncol + 0.6
                fig, axes = plt.subplots(n, ncol, figsize=(fig_w, fig_h), squeeze=False)
                for ri, (idx, ci, vi, (npix, frac, bbox, mdiff)) in enumerate(page):
                    cells = [("clean", ci, CLEAN_COLOR)]
                    cells.append(("modified", vi, DIFF_COLOR))
                    for k, (lab, img, col) in enumerate(cells):
                        ax = axes[ri][k]
                        ax.imshow(img, aspect="auto")
                        ax.set_xticks([]); ax.set_yticks([])
                        for sp in ax.spines.values():
                            sp.set_linewidth(1.0)
                            sp.set_color(col if k == 1 else "black")
                        if ri == 0:
                            ax.set_title("%s  |  %s" % (lab, g if k == 1 else "clean"),
                                         fontsize=7, color=col, fontweight="bold")
                        if k == 0:
                            ax.set_ylabel("F%03d" % idx, fontsize=7, rotation=0,
                                          ha="right", va="center")
                        if k == 1 and bbox is not None:
                            x0, y0, x1, y1 = bbox
                            ax.add_patch(Rectangle((x0, y0), x1 - x0 + 1, y1 - y0 + 1,
                                                   facecolor="none", edgecolor=DIFF_COLOR,
                                                   linewidth=1.2))
                            ax.text(0.995, 0.03, "%d px (%.2f%%)" % (npix, frac * 100),
                                    transform=ax.transAxes, ha="right", va="bottom",
                                    fontsize=6, color=DIFF_COLOR, fontweight="bold")
                    if args.diff_col:
                        ax = axes[ri][2]
                        dm = np.abs(np.rint(ci * 255) - np.rint(vi * 255)).max(axis=2) / 255.0
                        ax.imshow(np.clip(dm * 6.0, 0, 1), cmap="inferno", aspect="auto")
                        ax.set_xticks([]); ax.set_yticks([])
                        for sp in ax.spines.values():
                            sp.set_linewidth(1.0); sp.set_color("black")
                        if ri == 0:
                            ax.set_title("difference x6", fontsize=7, fontweight="bold")
                fig.suptitle("%s / %s   frames %d-%d   (left=clean, right=%s, red box=changed area)"
                             % (g, cam, page[0][0], page[-1][0], g),
                             fontsize=9, fontweight="bold")
                fig.tight_layout(rect=(0.01, 0.005, 0.995, 0.985))
                p_png = os.path.join(d_sheets, "%s_%s_sheet%02d.png" % (g, cam, sheet_no))
                p_pdf = os.path.join(d_sheets, "%s_%s_sheet%02d.pdf" % (g, cam, sheet_no))
                fig.savefig(p_png, dpi=args.dpi, facecolor="white")
                if not args.no_sheets:
                    page_pdf.savefig(fig)
                plt.close(fig)
                print("      页 %02d: 帧 %d-%d -> %s" % (sheet_no, page[0][0], page[-1][0], p_png))
                return sheet_no

            pdf_path = os.path.join(d_sheets, "%s_%s_all.pdf" % (g, cam))
            page_pdf = None if args.no_sheets else PdfPages(pdf_path)

            for r in sel:
                idx = int(r["frame_index"])
                pc = os.path.join(ds, r[key_clean].replace("\\", "/"))
                pv = os.path.join(ds, r[key_var].replace("\\", "/"))
                ci, vi = read_img(pc), read_img(pv)
                if ci is None or vi is None:
                    print("      [!] F%03d 读不到图，跳过（%s）" % (idx, pc if ci is None else pv))
                    continue
                npix, frac, bbox, mdiff = diff_stats(ci, vi)
                index_rows.append({
                    "group": g, "cam": cam, "frame_index": idx,
                    "timestamp_seconds": r["timestamp_seconds"],
                    "clean_path": r[key_clean].replace("\\", "/"),
                    "variant_path": r[key_var].replace("\\", "/"),
                    "changed_pixels": npix, "changed_frac": "%.6f" % frac,
                    "bbox_x0": "" if bbox is None else bbox[0],
                    "bbox_y0": "" if bbox is None else bbox[1],
                    "bbox_x1": "" if bbox is None else bbox[2],
                    "bbox_y1": "" if bbox is None else bbox[3],
                    "mean_abs_diff": "%.4f" % mdiff,
                })
                total_written += 1

                # 1) 被改后的整图单独拷一份
                if not args.no_copy:
                    shutil.copy2(pv, os.path.join(d_frames, "F%03d_%s_modified.png" % (idx, cam)))

                # 2) 单帧左右对照图
                if not args.no_pairs:
                    ncol = 3 if args.diff_col else 2
                    fig, axes = plt.subplots(1, ncol, figsize=(4.2 * ncol, 2.6))
                    axes = np.atleast_1d(axes)
                    axes[0].imshow(ci, aspect="auto")
                    axes[0].set_title("clean (original)", fontsize=8, color=CLEAN_COLOR,
                                      fontweight="bold")
                    axes[1].imshow(vi, aspect="auto")
                    axes[1].set_title("%s (modified)" % g, fontsize=8, color=DIFF_COLOR,
                                      fontweight="bold")
                    if bbox is not None:
                        x0, y0, x1, y1 = bbox
                        axes[1].add_patch(Rectangle((x0, y0), x1 - x0 + 1, y1 - y0 + 1,
                                                     facecolor="none", edgecolor=DIFF_COLOR,
                                                     linewidth=1.4))
                    for ax in axes[:2]:
                        ax.set_xticks([]); ax.set_yticks([])
                    if args.diff_col:
                        dm = np.abs(np.rint(ci * 255) - np.rint(vi * 255)).max(axis=2) / 255.0
                        axes[2].imshow(np.clip(dm * 6.0, 0, 1), cmap="inferno", aspect="auto")
                        axes[2].set_title("difference x6", fontsize=8, fontweight="bold")
                        axes[2].set_xticks([]); axes[2].set_yticks([])
                    fig.suptitle("F%03d  %s/%s   changed %d px (%.2f%%)   mean|d|=%.2f"
                                 % (idx, g, cam, npix, frac * 100, mdiff * 255),
                                 fontsize=9, fontweight="bold")
                    fig.tight_layout(rect=(0.005, 0.01, 0.995, 0.94))
                    fig.savefig(os.path.join(d_pairs, "F%03d_%s.png" % (idx, cam)),
                                dpi=args.dpi, facecolor="white")
                    plt.close(fig)

                page.append((idx, ci, vi, (npix, frac, bbox, mdiff)))
                if len(page) >= max(1, args.rows):
                    sheet_no = flush_sheet(page, sheet_no)
                    page = []

            sheet_no = flush_sheet(page, sheet_no)
            page = []
            if page_pdf is not None:
                page_pdf.close()
                print("      PDF: %s" % pdf_path)
            print("      %s/%s 完成：%d 帧" % (g, cam, len(sel)))

    # ---- index.csv ----
    if index_rows:
        csv_path = os.path.join(out, "index.csv")
        with open(csv_path, "w", encoding="utf-8", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(index_rows[0].keys()))
            w.writeheader()
            w.writerows(index_rows)
        print("\n[i] 逐帧统计 -> %s（%d 行）" % (csv_path, len(index_rows)))

    with open(os.path.join(out, "README.txt"), "w", encoding="utf-8") as fh:
        fh.write(
            "逐帧 原图 vs 被干扰图 导出\n"
            "数据源: %s\n\n"
            "pairs/<组>/F###_<cam>.png          单帧对照图：左=clean 原图，右=被干扰后的图\n"
            "                                   （红框 = 实际发生改动的像素范围）\n"
            "pairs/<组>/F###_<cam>_modified.png 只有被改后的那张整图\n"
            "frames/<组>/F###_<cam>_modified.png 同上，按组归档\n"
            "sheets/<组>_<cam>_sheetNN.png      一页 %d 帧的对照表\n"
            "sheets/<组>_<cam>_all.pdf          全部页 PDF\n"
            "index.csv                          逐帧：改动像素数/占比/bbox/平均差\n\n"
            "注意：改动只发生在索引 8-99；索引 0-7 三组逐字节相同（用于 SLAM 初始化）。\n"
            % (ds, args.rows))
    print("[i] 说明 -> %s/README.txt" % out)
    print("[i] 共导出 %d 帧对照" % total_written)
    return 0


if __name__ == "__main__":
    sys.exit(main())