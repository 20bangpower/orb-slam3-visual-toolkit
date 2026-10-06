#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_cmake.py -- 给 ORB-SLAM3 加一个可执行目标，自动适配常见的几种布局

ORB-SLAM3 的 Examples 构建清单有两种主流写法：
  A) 每个例子一个清单：  Examples/Monocular/CMakeLists.txt
  B) 所有例子一个清单：  Examples/CMakeLists.txt        <- 你现在遇到的是这种
  C) 其它 fork：清单在别处，但一定有一条 add_executable(rgbd_tum ...) 或
     add_executable(mono_tum ...) 可以照抄

本脚本不猜：先扫出所有 CMakeLists.txt，找到定义参照目标（默认 mono_tum，
找不到就 rgbd_tum）的那一个，照抄它的写法追加新目标。

用法：
  python3 patch_cmake.py --root ~/ORB_SLAM3 \
      --source Examples/Monocular/mono_tum_vis.cc --target mono_tum_vis
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
import time

SKIP_DIRS = {"build", "Thirdparty", ".git", "CMakeFiles"}


def candidates(root: str):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames
                       if d not in SKIP_DIRS and not d.startswith(".")]
        if "CMakeLists.txt" in filenames:
            yield os.path.join(dirpath, "CMakeLists.txt")


def read(p: str) -> str:
    with open(p, encoding="utf-8", errors="ignore") as fh:
        return fh.read()


def find_def(dirs, target: str):
    """返回 [(cmake_path, add_executable 行)]"""
    hits = []
    pat = re.compile(r"add_executable\(\s*" + re.escape(target) + r"\b")
    for p in dirs:
        for line in read(p).splitlines():
            if pat.search(line):
                hits.append((p, line.strip()))
    return hits


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--source", required=True,
                    help="相对 root 的源文件路径，例如 Examples/Monocular/mono_tum_vis.cc")
    ap.add_argument("--target", default="mono_tum_vis")
    ap.add_argument("--refs", default="mono_tum,rgbd_tum,stereo_tum_vi",
                    help="参照目标，逗号分隔，按顺序找")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    root = os.path.abspath(os.path.expanduser(args.root))
    src_abs = os.path.join(root, *args.source.split("/"))
    if not os.path.isfile(src_abs):
        print("[x] 源文件不存在: %s" % src_abs)
        return 1

    all_cmake = list(candidates(root))
    print("[i] 扫到 %d 个 CMakeLists.txt" % len(all_cmake))

    # 已经有了？
    if find_def(all_cmake, args.target):
        p, line = find_def(all_cmake, args.target)[0]
        print("[i] %s 已存在于 %s，跳过" % (args.target, p))
        print("[i] CMAKE_FILE=%s" % p)
        return 0

    # 找参照目标
    cmake_file = None
    ref_name = None
    for ref in [r.strip() for r in args.refs.split(",") if r.strip()]:
        hits = find_def(all_cmake, ref)
        if not hits:
            continue
        # 优先挑目录和源文件同级的；否则挑路径最浅的
        same = [h for h in hits if os.path.dirname(h[0]) == os.path.dirname(src_abs)]
        pick = same[0] if same else sorted(hits, key=lambda h: len(h[0]))[0]
        cmake_file, ref_name = pick[0], ref
        print("[i] 参照目标: %s  ->  %s" % (ref_name, cmake_file))
        break

    if cmake_file is None:
        print("[x] 没在任何 CMakeLists.txt 里找到参照目标 %s" % args.refs)
        print("    ORB-SLAM3 的 Examples 构建清单可能被改过。请把下面这些记录下来再核对：")
        print("      ls -R %s/Examples | head -50" % root)
        print("      grep -rn 'add_executable' %s --include=CMakeLists.txt | head -40" % root)
        return 2

    # 库里链接的写法：照抄参照目标那一行
    token = "${PROJECT_NAME}"
    m = re.search(r"target_link_libraries\(\s*" + re.escape(ref_name) + r"\s+([^)\s]+)",
                  read(cmake_file))
    if m:
        token = m.group(1)
    print("[i] 链接写法照抄: target_link_libraries(%s %s)" % (ref_name, token))

    rel = os.path.relpath(src_abs, os.path.dirname(cmake_file)).replace(os.sep, "/")
    block = ("\n\n# ---- added by patch_mono.sh (%s) ----\n"
             "add_executable(%s %s)\n"
             "target_link_libraries(%s %s)\n"
             % (time.strftime("%Y-%m-%d %H:%M:%S"), args.target, rel,
                args.target, token))

    print("[i] 将追加到 %s:" % cmake_file)
    print("      add_executable(%s %s)" % (args.target, rel))
    print("      target_link_libraries(%s %s)" % (args.target, token))

    if args.dry_run:
        print("[i] --dry-run，不写文件")
        print("[i] CMAKE_FILE=%s" % cmake_file)
        return 0

    bak = "%s.bak-%s" % (cmake_file, time.strftime("%Y%m%d-%H%M%S"))
    shutil.copy2(cmake_file, bak)
    print("[i] 已备份: %s" % bak)
    with open(cmake_file, "a", encoding="utf-8") as fh:
        fh.write(block)
    print("[i] 已写入")
    print("[i] CMAKE_FILE=%s" % cmake_file)

    # 提醒：新目标要重新 cmake 一次才会被 make 看见
    root_cmake = os.path.join(root, "CMakeLists.txt")
    if os.path.isfile(root_cmake):
        sub = os.path.relpath(os.path.dirname(cmake_file), root).replace(os.sep, "/")
        txt = read(root_cmake)
        if sub != "." and ("add_subdirectory(%s)" % sub) not in txt and ("add_subdirectory(${PROJECT_SOURCE_DIR}/%s)" % sub) not in txt:
            print("[!] 根 CMakeLists 里没看到 add_subdirectory(%s) —— 这个清单可能不会被构建" % sub)
    return 0


if __name__ == "__main__":
    sys.exit(main())