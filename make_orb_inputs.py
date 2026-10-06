#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_orb_inputs.py -- 把 4Seasons 双目包转成 ORB-SLAM3 能直接吃的输入

产出（默认写在 --out，缺省 <DS_ROOT>/orb_inputs）:
  <group>_mono.txt      ORB-SLAM3 单目关联文件: "<时间戳秒> <cam0/xxx.png>"
  <group>_mono_abs.txt  同上但用绝对路径（给不拼 argv[3] 的版本用）
  <group>_stereo.txt    3 列: "<时间戳秒> <cam0/xxx.png> <cam1/xxx.png>"
  4seasons_orb3_mono.yaml    单目配置（数值原样搬自 4seasons_rectified.yaml）
  4seasons_orb3_stereo.yaml  双目配置（额外补 STEREO.b / STEREO.ThDepth）

时间戳沿用包里的原始纳秒时间戳（秒为单位），相邻约 33 ms；800x400。
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

GROUPS = ["clean", "fixed_pixel", "world_plane"]

# 4seasons_rectified.yaml 里的原始数值（README 明确要求：勿改数值）
CALIB = {
    "fx": 501.4757919305817,
    "fy": 501.4757919305817,
    "cx": 421.7953735163109,
    "cy": 167.65799492501083,
    "bf": 150.69155075856955,
    "width": 800,
    "height": 400,
    "fps": 30.0,
}
BASELINE = CALIB["bf"] / CALIB["fx"]   # ≈ 0.30048 m


def mono_yaml() -> str:
    return """%YAML:1.0
# ORB-SLAM3 单目配置 -- 4Seasons recording_2021-05-10_19-15-19 (parking_garage_3_train)
# 数值原样取自包内 4seasons_rectified.yaml，勿改。输入已去畸变+校正，k1..k3/p1/p2 必须保持 0。
Camera.type: "PinHole"
Camera.fx: {fx!r}
Camera.fy: {fy!r}
Camera.cx: {cx!r}
Camera.cy: {cy!r}
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.k3: 0.0
Camera.width: {width}
Camera.height: {height}
Camera.bf: {bf!r}
Camera.fps: {fps!r}
Camera.RGB: 0
ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
""".format(**CALIB)


def stereo_yaml() -> str:
    # 注意：ORB-SLAM3 的 Settings.cc 读的键是 "ThDepth"（= 基线倍数），
    # 读 "STEREO.ThDepth" 的是 ORB-SLAM2。两个都写，省得踩坑。
    return mono_yaml() + """ThDepth: 40.0
STEREO.ThDepth: 40.0
STEREO.b: {b!r}
""".format(b=BASELINE)


def read_pairs(ds_root: str, group: str):
    path = os.path.join(ds_root, group, "pairs.csv")
    with open(path, encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ds-root", required=True, help="slam_stereo_pairs_0_99_20260929 的目录")
    ap.add_argument("--out", default=None, help="输出目录，缺省 <ds-root>/orb_inputs")
    ap.add_argument("--groups", default=",".join(GROUPS))
    args = ap.parse_args()

    ds_root = os.path.abspath(os.path.expanduser(args.ds_root))
    out = os.path.abspath(os.path.expanduser(args.out or os.path.join(ds_root, "orb_inputs")))
    os.makedirs(out, exist_ok=True)

    for group in [g.strip() for g in args.groups.split(",") if g.strip()]:
        rows = read_pairs(ds_root, group)
        gdir = os.path.join(ds_root, group)
        mono, mono_abs, stereo = [], [], []
        for r in rows:
            ts = r["timestamp_seconds"]
            left = r["left"].replace("\\", "/")
            right = r["right"].replace("\\", "/")
            mono.append("%s %s" % (ts, left))
            mono_abs.append("%s %s" % (ts, os.path.join(gdir, left)))
            stereo.append("%s %s %s" % (ts, left, right))

        for name, lines in ((group + "_mono.txt", mono),
                            (group + "_mono_abs.txt", mono_abs),
                            (group + "_stereo.txt", stereo)):
            p = os.path.join(out, name)
            with open(p, "w", encoding="utf-8") as fh:
                fh.write("\n".join(lines) + "\n")
            print("[i] %-46s %d 行" % (name, len(lines)))

        miss = [r["left"] for r in rows if not os.path.isfile(os.path.join(gdir, r["left"]))]
        if miss:
            print("[!] %s 缺 %d 张左图，例如 %s" % (group, len(miss), miss[0]))

    for name, text in (("4seasons_orb3_mono.yaml", mono_yaml()),
                       ("4seasons_orb3_stereo.yaml", stereo_yaml())):
        p = os.path.join(out, name)
        with open(p, "w", encoding="utf-8") as fh:
            fh.write(text)
        print("[i] %-46s 写出" % name)

    print()
    print("[i] baseline b = bf/fx = %.6f m" % BASELINE)
    print("[i] 输出目录: %s" % out)
    print()
    print("单目跑法（推荐，按数据集作者提示取左目）:")
    print("  ./Examples/Monocular/mono_tum_vis Vocabulary/ORBvoc.txt \\")
    print("      %s/4seasons_orb3_mono.yaml \\" % out)
    print("      <DS_ROOT>/clean  %s/clean_mono.txt" % out)


if __name__ == "__main__":
    main()