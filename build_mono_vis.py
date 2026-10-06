#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_mono_vis.py -- 复用已有例子（rgbd_tum 等）的编译/链接参数，直接编出 mono_tum_vis

为什么不用 CMake：有些 ORB-SLAM3 副本里根本没有每个例子的 CMakeLists.txt，
但既然 rgbd_tum 编出来过，CMake 就一定留下了它的编译参数：
    <build>/Examples/RGB-D/CMakeFiles/rgbd_tum.dir/flags.make   编译参数
    <build>/Examples/RGB-D/CMakeFiles/rgbd_tum.dir/link.txt     链接命令

本脚本把这两份拿过来，只做两处替换：
    rgbd_tum.cc.o            -> mono_tum_vis.cc.o
    -o .../rgbd_tum          -> -o .../mono_tum_vis
然后编译 + 链接。
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys

SKIP_DIRS = {"build", "Thirdparty", ".git"}


def run(cmd, cwd, dry, log_lines):
    log_lines.append("$ (cd %s && %s)" % (cwd, " ".join(cmd)))
    if dry:
        return 0
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if p.stdout:
        log_lines.append(p.stdout.rstrip())
    if p.stderr:
        log_lines.append(p.stderr.rstrip())
    return p.returncode


def join_continuations(path):
    with open(path, encoding="utf-8", errors="ignore") as fh:
        raw = fh.read()
    raw = raw.replace("\\\r\n", " ").replace("\\\n", " ")
    return raw


def parse_flags(path):
    txt = join_continuations(path)
    out = {}
    for key in ("CXX_FLAGS", "CXX_DEFINES", "CXX_INCLUDES"):
        m = re.search(r"^%s\s*=\s*(.*)$" % key, txt, re.M)
        out[key] = m.group(1).strip() if m else ""
    m = re.search(r"^# compile CXX with\s+(\S+)", txt, re.M)
    out["CXX"] = m.group(1) if m else "/usr/bin/c++"
    return out


def find_dir_files_depth(root, fname, skip=SKIP_DIRS, maxdepth=3):
    root = os.path.abspath(root)
    base = root.rstrip(os.sep).count(os.sep)
    hits = []
    for dirpath, dirnames, filenames in os.walk(root):
        depth = dirpath.rstrip(os.sep).count(os.sep) - base
        if depth >= maxdepth:
            dirnames[:] = []
        else:
            dirnames[:] = [d for d in dirnames if d not in skip and not d.startswith(".")]
        if fname in filenames:
            hits.append(os.path.join(dirpath, fname))
    return hits


def find_dir_files(root, fname, skip=SKIP_DIRS):
    hits = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in skip]
        if fname in filenames:
            hits.append(os.path.join(dirpath, fname))
    return hits


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="ORB_SLAM3 根目录")
    ap.add_argument("--refs", default="rgbd_tum,mono_tum,stereo_tum_vi,mono_tum_vi",
                    help="参照哪个已编好的例子")
    ap.add_argument("--target", default="mono_tum_vis")
    ap.add_argument("--source", default="Examples/Monocular/mono_tum_vis.cc")
    ap.add_argument("--build", default=None, help="构建目录，缺省自动找")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    root = os.path.abspath(os.path.expanduser(args.root))
    src = os.path.join(root, *args.source.split("/"))
    if not os.path.isfile(src):
        print("[x] 源文件不存在: %s" % src)
        return 1

    build = args.build
    if not build:
        caches = find_dir_files(root, "CMakeCache.txt", skip={"Thirdparty", ".git"})
        if not caches:
            print("[i] 仓库里没有 CMakeCache.txt，往 $HOME 下再找一层（maxdepth 3）...")
            caches = find_dir_files_depth(os.path.expanduser("~"), "CMakeCache.txt",
                                          skip={"Thirdparty", ".git", ".cache", ".local"},
                                          maxdepth=3)
        if not caches:
            print("[x] 没找到 CMakeCache.txt，没法反推编译参数。")
            print("    请先按你原来的方式构建一次 ORB-SLAM3，再跑本脚本；")
            print("    或者用 --build /path/to/build 直接指定你的构建目录。")
            return 3
        build = os.path.dirname(sorted(caches, key=len)[0])
    print("[i] 构建目录: %s" % build)

    # 找参照目标的 flags.make / link.txt
    refs = [r.strip() for r in args.refs.split(",") if r.strip()]
    target_dir = None
    ref = None
    for r in refs:
        dirs = [os.path.dirname(p) for p in find_dir_files(build, "link.txt", skip={"Thirdparty", ".git"})
                if os.path.basename(os.path.dirname(p)) == "%s.dir" % r]
        if dirs:
            target_dir, ref = dirs[0], r
            break
    if target_dir is None:
        print("[x] 在 %s 里没找到任何 %s 的 CMakeFiles/*.dir/link.txt" % (build, "/".join(refs)))
        print("    已找到的 link.txt：")
        for p in find_dir_files(build, "link.txt", skip={"Thirdparty", ".git"})[:20]:
            print("      " + p)
        return 3

    flags_path = os.path.join(target_dir, "flags.make")
    link_path = os.path.join(target_dir, "link.txt")
    if not os.path.isfile(flags_path):
        print("[x] 缺少 %s" % flags_path)
        return 3
    wd = os.path.dirname(os.path.dirname(target_dir))   # cmake 的执行目录（CMakeFiles 的上一级）
    print("[i] 参照目标: %s" % ref)
    print("[i] 编译参数: %s" % flags_path)
    print("[i] 工作目录: %s" % wd)

    fl = parse_flags(flags_path)
    obj_rel = "CMakeFiles/%s.dir/%s.cc.o" % (ref, args.target)
    obj_abs = os.path.join(target_dir, "%s.cc.o" % args.target)

    compile_cmd = [fl["CXX"]]
    compile_cmd += fl["CXX_DEFINES"].split()
    compile_cmd += fl["CXX_INCLUDES"].split()
    compile_cmd += fl["CXX_FLAGS"].split()
    compile_cmd += ["-c", src, "-o", obj_rel]

    link_txt = join_continuations(link_path).strip()
    link_cmd = link_txt.split()

    # 1) 换目标文件
    obj_token = None
    for tok in link_cmd:
        if tok.endswith(".cc.o") and ref in tok:
            obj_token = tok
            break
    if obj_token:
        link_cmd = [t.replace(obj_token, obj_rel) for t in link_cmd]
    else:
        # 找不到就塞在 -o 前面
        i = link_cmd.index("-o") if "-o" in link_cmd else len(link_cmd)
        link_cmd = link_cmd[:i] + [obj_rel] + link_cmd[i:]

    # 2) 换输出文件名（保留目录）
    if "-o" in link_cmd:
        i = link_cmd.index("-o")
        old_out = link_cmd[i + 1]
        new_out = os.path.join(os.path.dirname(old_out), args.target)
        link_cmd[i + 1] = new_out
        out_abs = os.path.normpath(os.path.join(wd, new_out))
    else:
        out_abs = os.path.join(wd, args.target)

    print()
    print("[i] 编译命令:")
    print("      (cd %s)" % wd)
    print("      %s" % " ".join(compile_cmd))
    print("[i] 链接命令:")
    print("      %s" % " ".join(link_cmd))
    print()

    log = []
    rc = run(compile_cmd, wd, args.dry_run, log)
    if rc != 0:
        print("[x] 编译失败：")
        print("\n".join(log[-40:]))
        return 1
    print("[i] 编译通过")

    rc = run(link_cmd, wd, args.dry_run, log)
    if rc != 0:
        print("[x] 链接失败：")
        print("\n".join(log[-40:]))
        return 1
    print("[i] 链接通过")

    if args.dry_run:
        print("[i] --dry-run，没有真正执行")
        print("BIN=%s" % out_abs)
        return 0

    # 顺便把成品放到 Examples/<X>/ 下（和 rgbd_tum 的摆放习惯一致）
    final = src.replace(".cc", "")
    os.makedirs(os.path.dirname(final), exist_ok=True)
    if os.path.abspath(final) != os.path.abspath(out_abs):
        shutil.copy2(out_abs, final)
        print("[i] 已复制到 %s" % final)
    else:
        final = out_abs
    print("[i] 可执行文件: %s" % final)
    print("BIN=%s" % final)
    return 0


if __name__ == "__main__":
    sys.exit(main())