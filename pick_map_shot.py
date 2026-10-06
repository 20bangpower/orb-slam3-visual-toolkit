#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pick_map_shot.py -- 从 capture_pangolin.sh / shot_fig4.sh 抓到的一堆窗口截图里，
挑出"地图最全"的那张 Pangolin Map Viewer 截图，作为 Fig.4 的**左图**。

为什么需要它：抓图脚本每 4~5 秒抓一次，地图是逐渐长出来的；最后几帧时地图最全，
但脚本也可能抓到"窗口刚开、图还是白的"或"被遮住、一片黑"的废图。

打分（越大越像"长满地图"）：
    score = 彩色像素数(通道极差>45) + 0.5 * 深色像素数(灰度<80)
    彩色=红色地图点/蓝绿关键帧；深色=黑色地图点/关键帧线
    菜单栏在每张图里都一样，是个常数，不影响排名。
    整张图 >50% 是深色 -> 判为废图（窗口被遮挡 / 抓图失败），直接 0 分。

用法：
    python3 pick_map_shot.py --shots <run>/shots --mapview <run>/map_view.png \
        --out <run>/pangolin_map.png
    python3 pick_map_shot.py --shots <run>/shots --out <run>/pangolin_map.png --list
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import sys

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass


# --------------------------------------------------------------- 读图后端
def load_rgb(path):
    """返回 (H,W,3) uint8 的 numpy 数组；先试 PIL，再试 matplotlib。"""
    import numpy as np
    try:
        from PIL import Image
        im = Image.open(path)
        im = im.convert("RGB")
        return np.asarray(im, dtype=np.uint8)
    except Exception:
        pass
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    a = mpimg.imread(path)
    if a.dtype != np.uint8:
        a = np.clip(a * 255.0, 0, 255).astype(np.uint8)
    if a.ndim == 2:
        a = np.dstack([a] * 3)
    return a[..., :3]


def score_of(arr):
    """地图信息量打分；返回 (score, colored, dark, frac_dark, size_ok)"""
    import numpy as np
    h, w = arr.shape[0], arr.shape[1]
    a = arr.astype(np.int16)
    mx = a.max(axis=2)
    mn = a.min(axis=2)
    spread = mx - mn
    gray = a.mean(axis=2)
    colored = int((spread > 45).sum())
    dark = int((gray < 80).sum())
    frac_dark = dark / float(h * w)
    size_ok = (h >= 200 and w >= 200)
    if not size_ok or frac_dark > 0.50:
        return 0.0, colored, dark, frac_dark, size_ok
    return float(colored) + 0.5 * float(dark), colored, dark, frac_dark, size_ok


# ------------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", default=None, help="抓图目录（含 w_*Map_Viewer*.png）")
    ap.add_argument("--mapview", default=None, help="备选：SLAM 进程自己存的 map_view.png")
    ap.add_argument("--out", required=True, help="输出左图路径（如 <run>/pangolin_map.png）")
    ap.add_argument("--menu-frac", type=float, default=0.0,
                    help=">0 时裁掉左侧菜单栏（占宽度比例），默认 0=保留（和论文一致）")
    ap.add_argument("--list", action="store_true", help="只打印候选打分，不写文件")
    args = ap.parse_args()

    cands = []
    if args.shots and os.path.isdir(args.shots):
        pats = ["w_*Map*Viewer*.png", "w_*Map_Viewer*.png", "w_*Map*.png"]
        seen = set()
        for p in pats:
            for f in sorted(glob.glob(os.path.join(args.shots, p))):
                if f not in seen:
                    seen.add(f)
                    cands.append(f)

    print("[i] 抓图目录: %s" % (args.shots or "(未给)"))
    print("[i] 候选窗口截图: %d 张" % len(cands))
    if args.mapview:
        print("[i] 备选 map_view.png: %s%s" % (
            args.mapview, "" if os.path.isfile(args.mapview) else "  (不存在)"))

    scored = []
    for f in cands:
        try:
            arr = load_rgb(f)
            s, col, dark, fd, ok = score_of(arr)
        except Exception as e:
            print("      [!] 读不了 %s: %s" % (os.path.basename(f), e))
            continue
        scored.append((s, f, col, dark, fd, ok, arr.shape))
    if args.mapview and os.path.isfile(args.mapview):
        try:
            arr = load_rgb(args.mapview)
            s, col, dark, fd, ok = score_of(arr)
            scored.append((s, args.mapview, col, dark, fd, ok, arr.shape))
        except Exception as e:
            print("      [!] 读不了 map_view.png: %s" % e)

    if not scored:
        print("[x] 一张候选都没有。抓图脚本是不是没跑起来？（看 <run>/capture.log）")
        return 1

    scored.sort(key=lambda t: t[0], reverse=True)
    print("      %-10s %-9s %-9s %-7s  %s" % ("score", "colored", "dark", "frac_dark", "文件"))
    for s, f, col, dark, fd, ok, shape in scored[:8]:
        tag = "" if ok else "  <- 废图(尺寸太小/过半深色)"
        print("      %-10.0f %-9d %-9d %-7.3f  %s%s"
              % (s, col, dark, fd, os.path.basename(f), tag))

    best = scored[0]
    if best[0] <= 0.0:
        print("[x] 所有候选都是废图（窗口没开出来 / 一直被遮挡）。")
        print("    先跑 bash show_viewer.sh 看看 Pangolin 窗口能不能开；")
        print("    抓图时建议装 xdotool，让脚本把两个窗口摆开：")
        print("      sudo apt-get install -y xdotool x11-utils imagemagick")
        return 2

    print("[i] 选中: %s  (score=%.0f)" % (best[1], best[0]))

    if args.list:
        return 0

    out = args.out
    if args.menu_frac and args.menu_frac > 0:
        import numpy as np
        try:
            from PIL import Image
        except Exception:
            print("[x] --menu-frac 需要 pillow: pip install pillow")
            return 1
        arr = load_rgb(best[1])
        x0 = int(arr.shape[1] * args.menu_frac)
        Image.fromarray(arr[:, x0:]).save(out)
        print("[i] 已裁掉左侧 %.0f%% 菜单栏" % (args.menu_frac * 100))
    else:
        os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
        shutil.copy2(best[1], out)
    print("[i] 左图 -> %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())